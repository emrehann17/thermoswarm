#!/usr/bin/env python3
"""Safely manage the verified three-X500 PX4/Gazebo SITL lifecycle."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / ".runtime" / "sim"
STATE = RUNTIME / "state.json"
LOCK = RUNTIME / "launcher.lock"
CONFIG_MODULE = ROOT / "ros2_ws/src/thermoswarm_control/thermoswarm_control/config.py"
EXPECTED = (
    ("x500_1", 1, 2, (0.0, 0.0)),
    ("x500_2", 2, 3, (5.0, 0.0)),
    ("x500_3", 3, 4, (10.0, 0.0)),
)
STOP_GRACE_S = 10
FORCE_GRACE_S = 5
GROUP_LEADERS = {}


class SimError(RuntimeError):
    pass


def config_module():
    spec = importlib.util.spec_from_file_location("thermoswarm_fleet_config", CONFIG_MODULE)
    if spec is None or spec.loader is None:
        raise SimError(f"Config loader bulunamadı: {CONFIG_MODULE}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_fleet(path=None):
    try:
        module = config_module()
    except ImportError as exc:
        raise SimError(f"Fleet config loader dependency missing: {exc}") from exc
    try:
        fleet = module.load_config(path or ROOT / "config/fleet.yaml")
    except module.ConfigError as exc:
        raise SimError(f"Fleet yapılandırması geçersiz: {exc}") from exc
    actual = tuple(
        (v.name, v.px4_instance, v.system_id, tuple(float(x) for x in v.spawn_xy_m))
        for v in fleet.vehicles
    )
    if actual != EXPECTED or any(v.model != "x500" for v in fleet.vehicles):
        raise SimError(
            "Bu launcher yalnızca doğrulanmış x500_1..3 / instance 1..3 / SYSID 2..4 / "
            "poz (0,0), (5,0), (10,0) düzenini destekler."
        )
    return fleet


def read_proc(pid):
    """Return stable process identity fields, or None if /proc entry is gone."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        # comm is parenthesized and can contain spaces or parentheses.
        tail = raw[raw.rfind(")") + 2 :].split()
        pgid = int(tail[2])
        start_ticks = int(tail[19])
        argv = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        argv = [part.decode(errors="replace") for part in argv if part]
        if not argv:
            return None
        return {
            "pid": int(pid), "pgid": pgid, "start_ticks": start_ticks,
            "cmd_hash": hashlib.sha256("\0".join(argv).encode()).hexdigest(),
            "argv": argv,
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
        return None


def matches(record):
    current = read_proc(record.get("pid", -1))
    return bool(
        current
        and current["start_ticks"] == record.get("start_ticks")
        and current["pgid"] == record.get("pgid") == record.get("pid")
        and current["cmd_hash"] == record.get("cmd_hash")
    )


def load_state():
    if not STATE.exists():
        return {"version": 1, "processes": []}
    try:
        data = json.loads(STATE.read_text())
        if data.get("version") != 1 or not isinstance(data.get("processes"), list):
            raise ValueError("unsupported state format")
        return data
    except (OSError, json.JSONDecodeError, ValueError, AttributeError) as exc:
        raise SimError(f"Runtime state okunamıyor; güvenlik için işlem yapılmadı: {STATE}: {exc}") from exc


def save_state(data):
    RUNTIME.mkdir(parents=True, exist_ok=True)
    temp = STATE.with_suffix(".tmp")
    temp.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(temp, STATE)


def proc_description(record):
    return f"{record.get('name', '?')} pid={record.get('pid')} pgid={record.get('pgid')}"


def status():
    state = load_state()
    if not state["processes"]:
        print("Yönetilen simülasyon yok.")
    for rec in state["processes"]:
        print(f"{'RUNNING' if matches(rec) else 'STALE/STOPPED'}  {proc_description(rec)}")
    print(f"Runtime/log dizini: {RUNTIME}")


def command_candidates():
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = [x.decode(errors="replace") for x in (entry / "cmdline").read_bytes().split(b"\0") if x]
            cwd = os.readlink(entry / "cwd")
        except (OSError, PermissionError):
            continue
        joined = " ".join(argv)
        basename = Path(argv[0]).name if argv else ""
        is_px4 = basename == "px4" and str(ROOT / "PX4-Autopilot") in cwd
        is_gz_server = basename in {"gz", "ruby"} and "gz sim" in joined
        if is_px4 or is_gz_server:
            found.append((int(entry.name), joined, cwd))
    return found


def require_dependencies():
    missing = []
    px4 = ROOT / "PX4-Autopilot/build/px4_sitl_default/bin/px4"
    if not px4.is_file() or not os.access(px4, os.X_OK):
        missing.append(f"PX4 executable: {px4}")
    for name in ("MicroXRCEAgent", "gz", "timeout", "bash"):
        if shutil.which(name) is None:
            missing.append(f"PATH komutu: {name}")
    for file in (Path("/opt/ros/jazzy/setup.bash"), ROOT / "ros2_ws/install/setup.bash"):
        if not file.is_file():
            missing.append(f"gerekli setup dosyası: {file}")
    if missing:
        raise SimError("Eksik bağımlılıklar:\n  - " + "\n  - ".join(missing))


def port_in_use(port):
    # UDP bind probe detects an existing agent without killing or querying it.
    probe = None
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("0.0.0.0", port))
        return False
    except OSError:
        # Some managed sandboxes prohibit even a local bind; /proc is a safe fallback.
        try:
            hex_port = f"{port:04X}"
            for table in (Path("/proc/net/udp"), Path("/proc/net/udp6")):
                if table.exists() and any(
                    len(fields := line.split()) > 1 and fields[1].rsplit(":", 1)[-1].upper() == hex_port
                    for line in table.read_text().splitlines()[1:]
                ):
                    return True
            return False
        except OSError as exc:
            raise SimError(f"UDP/{port} kullanımını doğrulayamadım: {exc}") from exc
    finally:
        if probe is not None:
            probe.close()


def process_record(name, popen, argv):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        info = read_proc(popen.pid)
        if info:
            return {
                "name": name, "pid": info["pid"], "pgid": info["pgid"],
                "start_ticks": info["start_ticks"], "cmd_hash": info["cmd_hash"],
                "argv": argv,
            }
        time.sleep(0.05)
    raise SimError(f"{name} process identity alınamadı (pid={popen.pid})")


def group_alive(record):
    try:
        os.killpg(record["pgid"], 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def signal_owned(record, sig):
    if not matches(record):
        return False
    try:
        os.killpg(record["pgid"], sig)
        return True
    except ProcessLookupError:
        return False


def cleanup_records(records):
    # Validate each isolated group leader once. A group ID cannot be reused while
    # any member remains, so bounded escalation can continue if its leader exits first.
    owned = [rec for rec in records if matches(rec)]

    def wait_for_groups(grace):
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and any(group_alive(r) for r in owned):
            for rec in owned:
                leader = GROUP_LEADERS.get(rec["pid"])
                if leader is not None:
                    leader.poll()  # reap children started in this invocation
            time.sleep(0.1)

    for rec in reversed(owned):
        signal_owned(rec, signal.SIGINT)
    wait_for_groups(STOP_GRACE_S)
    for rec in reversed(owned):
        if group_alive(rec):
            try:
                os.killpg(rec["pgid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
    wait_for_groups(FORCE_GRACE_S)
    for rec in reversed(owned):
        if group_alive(rec):
            try:
                os.killpg(rec["pgid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
    for rec in records:
        leader = GROUP_LEADERS.pop(rec["pid"], None)
        if leader is not None:
            try:
                leader.wait(timeout=0)
            except subprocess.TimeoutExpired:
                pass


def wait_until(label, timeout_s, predicate, records):
    deadline = time.monotonic() + timeout_s
    last = "henüz hazır değil"
    while time.monotonic() < deadline:
        for rec in records:
            if not matches(rec):
                raise SimError(f"{rec['name']} beklenmedik şekilde durdu; log: {rec['log']}")
        try:
            ready, last = predicate()
        except Exception as exc:  # subprocess/readiness errors are reported with the bounded wait context
            ready, last = False, str(exc)
        if ready:
            print(f"Hazır: {label}")
            return
        time.sleep(1)
    raise SimError(f"Hazırlık zaman aşımı ({label}, {timeout_s}s): {last}")


def run_query(argv, timeout=3, env=None):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, env=env, check=False)
    return result.returncode, result.stdout, result.stderr


def gz_has_model(name):
    code, out, err = run_query(["gz", "model", "--list"], timeout=3)
    if code:
        return False, err.strip() or "gz model --list hata verdi"
    names = [line.strip()[2:] for line in out.splitlines() if line.strip().startswith("- ")]
    return name in names, f"Gazebo modelleri: {', '.join(names) or '(boş)'}"


def check_ros_topics(timeout=5):
    setup = (
        f"source {str(Path('/opt/ros/jazzy/setup.bash'))!r}; "
        f"source {str(ROOT / 'ros2_ws/install/setup.bash')!r}; "
        "exec ros2 topic list"
    )
    code, out, err = run_query(["bash", "-c", setup], timeout=timeout)
    topics = set(out.splitlines())
    required = {f"/px4_{i}/fmu/out/vehicle_local_position_v1" for i in range(1, 4)}
    missing = sorted(required - topics)
    return code == 0 and not missing, ("ROS 2 telemetri topic'leri hazır" if not missing else
                                       f"eksik topic'ler: {', '.join(missing)}; {err.strip()}")


def verify_world():
    cmd = [sys.executable, str(ROOT / "scripts/thermoswarm_gz_world_check.py")]
    code, out, err = run_query(cmd, timeout=25)
    if code:
        raise SimError(f"Gazebo dünya doğrulaması başarısız:\n{out}{err}")
    print(out.rstrip())


def launch_one(name, argv, cwd, env, logfile, records, state):
    log_path = RUNTIME / logfile
    stream = log_path.open("ab", buffering=0)
    try:
        proc = subprocess.Popen(argv, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    finally:
        stream.close()
    try:
        rec = process_record(name, proc, argv)
    except Exception:
        # The child was created in a fresh session, so its just-created group is ours.
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=2)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=2)
        raise
    GROUP_LEADERS[rec["pid"]] = proc
    rec["log"] = str(log_path)
    records.append(rec)
    state["processes"].append(rec)
    save_state(state)
    print(f"Başlatıldı: {name} pid={rec['pid']} log={log_path}")
    return rec


def start(config_path=None, dry_run=False):
    fleet = load_fleet(config_path)
    previous = load_state()
    live = [r for r in previous["processes"] if matches(r)]
    if live:
        raise SimError("Yönetilen süreçler zaten çalışıyor; çift start reddedildi:\n  " +
                       "\n  ".join(proc_description(r) for r in live))
    require_dependencies()
    if dry_run:
        print(f"Fleet PASS: {fleet.source}")
        print("DRY RUN: MicroXRCEAgent udp4 -p 8888")
        px4 = ROOT / "PX4-Autopilot/build/px4_sitl_default/bin/px4"
        for vehicle in fleet.vehicles:
            name, instance, pose = vehicle.name, vehicle.px4_instance, vehicle.spawn_xy_m
            extra = " PX4_GZ_STANDALONE=1" if instance > 1 else " (Gazebo server başlatır)"
            print(f"DRY RUN:{extra} PX4_GZ_MODEL_POSE='{pose[0]:g},{pose[1]:g}' {px4} -i {instance} [{name}]")
        return
    conflicts = command_candidates()
    if conflicts:
        details = "\n".join(f"pid={pid}: {cmd} (cwd={cwd})" for pid, cmd, cwd in conflicts)
        raise SimError("Yönetilmeyen PX4/Gazebo süreci algılandı; çakışmayı elle çözün. Hiçbiri kapatılmadı:\n" + details)
    if port_in_use(8888):
        raise SimError("UDP 8888 kullanımda; mevcut DDS Agent kapatılmadı. Portu elle inceleyin.")

    px4 = ROOT / "PX4-Autopilot/build/px4_sitl_default/bin/px4"
    px4_cwd = ROOT / "PX4-Autopilot"
    env = os.environ.copy()
    env.update({"HEADLESS": "1", "GZ_IP": "127.0.0.1", "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "gz_x500"})
    RUNTIME.mkdir(parents=True, exist_ok=True)
    records = []
    state = {"version": 1, "created": time.time(), "processes": []}
    try:
        agent = shutil.which("MicroXRCEAgent")
        launch_one("dds-agent", [agent, "udp4", "-p", "8888"], px4_cwd, env,
                   "dds-agent.log", records, state)
        wait_until("Micro XRCE-DDS Agent UDP/8888", 15,
                   lambda: (port_in_use(8888), "UDP/8888 bind edilmedi"), records)

        for vehicle in fleet.vehicles:
            name, instance, pose = vehicle.name, vehicle.px4_instance, vehicle.spawn_xy_m
            proc_env = env.copy()
            proc_env["PX4_GZ_MODEL_POSE"] = f"{pose[0]:g},{pose[1]:g}"
            if instance > 1:
                proc_env["PX4_GZ_STANDALONE"] = "1"
            else:
                proc_env.pop("PX4_GZ_STANDALONE", None)
            log = f"px4-{instance}.log"
            rec = launch_one(name, [str(px4), "-i", str(instance)], px4_cwd, proc_env,
                             log, records, state)
            wait_until(f"Gazebo modeli {name}", fleet.safety.readiness_timeout_s,
                       lambda n=name: gz_has_model(n), records)

        verify_world()
        wait_until("üç ROS 2 telemetri topic'i", min(fleet.safety.readiness_timeout_s, 90),
                   check_ros_topics, records)
        print("SIM START PASS: 3 X500, DDS Agent ve ROS 2 topic readiness doğrulandı. ARM/uçuş komutu gönderilmedi.")
        print(f"Loglar: {RUNTIME}")
    except (Exception, KeyboardInterrupt) as exc:
        cleanup_records(records)
        state["processes"] = []
        save_state(state)
        for rec in records:
            print(f"Temizlendi: {rec['name']} (log: {rec['log']})")
        if isinstance(exc, KeyboardInterrupt):
            raise SimError(f"Başlatma operatör tarafından kesildi; başlatılan süreçler temizlendi. Loglar: {RUNTIME}") from exc
        if isinstance(exc, SimError):
            raise SimError(f"{exc}\nBaşlatılan süreçler temizlendi. Loglar: {RUNTIME}") from exc
        raise SimError(f"Başlatma başarısız: {exc}\nBaşlatılan süreçler temizlendi. Loglar: {RUNTIME}") from exc


def vehicle_safe_to_stop(instances, timeout=8):
    """Fail closed unless all three vehicles report disarmed and landed."""
    script = ROOT / "scripts/sim_vehicle_safety.py"
    setup = (
        f"source {str(Path('/opt/ros/jazzy/setup.bash'))!r}; "
        f"source {str(ROOT / 'ros2_ws/install/setup.bash')!r}; "
        f"exec python3 {str(script)!r} --timeout {timeout} --instances {','.join(map(str, instances))!r}"
    )
    try:
        code, out, err = run_query(["bash", "-c", setup], timeout=timeout + 5)
    except subprocess.TimeoutExpired as exc:
        raise SimError("Kapanış güvenlik kontrolü zaman aşımına uğradı; süreçler çalışır bırakıldı.") from exc
    print(out.rstrip())
    if code:
        raise SimError("Kapanış güvenlik kontrolü geçmedi; süreçler çalışır bırakıldı. "
                       "Tüm araçların disarmed + landed olduğunu doğrulayın. " + err.strip())


def stop():
    state = load_state()
    live = [rec for rec in state["processes"] if matches(rec)]
    if not live:
        state["processes"] = []
        save_state(state)
        print("Yönetilen çalışan süreç yok; stale state temizlendi.")
        return
    instances = sorted(int(rec["name"].removeprefix("x500_")) for rec in live
                       if rec.get("name", "").startswith("x500_"))
    if instances:
        vehicle_safe_to_stop(instances)
    unsafe = [rec for rec in live if not matches(rec)]
    if unsafe:
        raise SimError("Süreç sahipliği doğrulanamadı; hiçbir grup sonlandırılmadı.")
    cleanup_records(live)
    still_alive = [r for r in live if group_alive(r)]
    if still_alive:
        raise SimError("Kapatılamayan sahipli process group: " + ", ".join(proc_description(r) for r in still_alive))
    state["processes"] = []
    save_state(state)
    print("STOP PASS: yalnızca manifest'te kimliği doğrulanan process group'lar kapatıldı.")


def logs(tail=0):
    files = sorted(RUNTIME.glob("*.log")) if RUNTIME.exists() else []
    if not files:
        print(f"Henüz log yok. Beklenen konum: {RUNTIME}/*.log")
        return
    for path in files:
        print(f"== {path} ==")
        if tail:
            try:
                lines = path.read_text(errors="replace").splitlines()
                print("\n".join(lines[-tail:]))
            except OSError as exc:
                print(f"Log okunamadı: {exc}")


def check():
    script = ROOT / "scripts/check.sh"
    result = subprocess.run([str(script)], cwd=ROOT, check=False)
    if result.returncode:
        raise SimError(f"check.sh başarısız (exit={result.returncode})")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    start_parser = sub.add_parser("start", help="SITL altyapısını başlat")
    start_parser.add_argument("--config", help=argparse.SUPPRESS)
    start_parser.add_argument("--dry-run", action="store_true", help="Doğrula ve komut planını göster; süreç başlatma")
    sub.add_parser("status", help="Yönetilen süreçleri salt okunur göster")
    sub.add_parser("check", help="Mevcut Gazebo ve telemetry denetimlerini çalıştır")
    logs_parser = sub.add_parser("logs", help="Log konumlarını göster; isteğe bağlı son satırları yazdır")
    logs_parser.add_argument("--tail", type=int, default=0, metavar="N")
    sub.add_parser("stop", help="Yalnızca bu launcher'ın sahipli süreçlerini güvenli kapat")
    args = parser.parse_args(argv)
    def dispatch():
        if args.command == "start":
            start(args.config, args.dry_run)
        elif args.command == "status":
            status()
        elif args.command == "check":
            check()
        elif args.command == "logs":
            if args.tail < 0:
                raise SimError("--tail negatif olamaz")
            logs(args.tail)
        elif args.command == "stop":
            stop()

    try:
        # Queries and dry-run stay filesystem-read-only; only lifecycle mutations need a lock.
        locked = args.command == "stop" or (args.command == "start" and not args.dry_run)
        if locked:
            RUNTIME.mkdir(parents=True, exist_ok=True)
            with LOCK.open("a") as lock_stream:
                fcntl.flock(lock_stream, fcntl.LOCK_EX)
                dispatch()
        else:
            dispatch()
    except SimError as exc:
        print(f"sim.sh: HATA: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Offline ownership and configuration tests for sim_launcher.py."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).with_name("sim_launcher.py")
spec = importlib.util.spec_from_file_location("sim_launcher", MODULE_PATH)
sim = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = sim
spec.loader.exec_module(sim)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.old_runtime, self.old_state, self.old_lock = sim.RUNTIME, sim.STATE, sim.LOCK
        sim.RUNTIME = self.root / ".runtime" / "sim"
        sim.STATE = sim.RUNTIME / "state.json"
        sim.LOCK = sim.RUNTIME / "launcher.lock"

    def tearDown(self):
        sim.RUNTIME, sim.STATE, sim.LOCK = self.old_runtime, self.old_state, self.old_lock
        self.temp.cleanup()

    def test_validates_canonical_fleet(self):
        fleet = sim.load_fleet()
        self.assertEqual([v.name for v in fleet.vehicles], ["x500_1", "x500_2", "x500_3"])

    def test_invalid_yaml_fails_before_launch(self):
        config = self.root / "invalid.yaml"
        config.write_text("vehicles: [\n")
        with self.assertRaises(sim.SimError):
            sim.load_fleet(config)

    def test_unsupported_pose_fails_closed(self):
        source = (sim.ROOT / "config/fleet.yaml").read_text()
        config = self.root / "wrong-layout.yaml"
        config.write_text(source.replace("spawn_xy_m: [5, 0]", "spawn_xy_m: [6, 0]"))
        with self.assertRaises(sim.SimError):
            sim.load_fleet(config)

    def test_missing_dependencies_are_reported(self):
        with mock.patch.object(sim, "ROOT", self.root):
            with self.assertRaisesRegex(sim.SimError, "Eksik bağımlılıklar"):
                sim.require_dependencies()

    def test_stale_pid_identity_does_not_match_reused_or_missing_pid(self):
        info = sim.read_proc(os.getpid())
        rec = {**info, "name": "test", "start_ticks": info["start_ticks"] + 1}
        self.assertFalse(sim.matches(rec))
        rec["pid"] = 99999999
        self.assertFalse(sim.matches(rec))

    def test_duplicate_start_rejected_before_dependency_or_spawn(self):
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
        try:
            identity = sim.read_proc(child.pid)
            sim.save_state({"version": 1, "processes": [{**identity, "name": "x500_1"}]})
            with mock.patch.object(sim, "require_dependencies", side_effect=AssertionError("must not run")):
                with self.assertRaisesRegex(sim.SimError, "zaten çalışıyor"):
                    sim.start(dry_run=True)
        finally:
            child.terminate()
            child.wait(timeout=3)

    def test_failed_start_cleanup_terminates_only_new_owned_group(self):
        owned = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
        try:
            identity = sim.read_proc(owned.pid)
            record = {**identity, "name": "test-owned"}
            sim.GROUP_LEADERS[owned.pid] = owned
            try:
                raise sim.SimError("injected readiness failure")
            except sim.SimError:
                sim.cleanup_records([record])
            self.assertIsNotNone(owned.poll())
            self.assertIsNone(unrelated.poll())
        finally:
            if owned.poll() is None:
                owned.kill()
                owned.wait(timeout=3)
            unrelated.terminate()
            unrelated.wait(timeout=3)

    def test_stop_terminates_manifest_owner_and_leaves_foreign_group(self):
        owned = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
        foreign = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
        try:
            identity = sim.read_proc(owned.pid)
            record = {**identity, "name": "x500_1", "log": "test.log"}
            sim.GROUP_LEADERS[owned.pid] = owned
            sim.save_state({"version": 1, "processes": [record]})
            with mock.patch.object(sim, "vehicle_safe_to_stop"):
                sim.stop()
            self.assertIsNotNone(owned.poll())
            self.assertIsNone(foreign.poll())
        finally:
            if owned.poll() is None:
                owned.kill()
                owned.wait(timeout=3)
            foreign.terminate()
            foreign.wait(timeout=3)

    def test_stop_ignores_stale_manifest_pid(self):
        sim.save_state({"version": 1, "processes": [{
            "name": "x500_1", "pid": 99999999, "pgid": 99999999,
            "start_ticks": 1, "cmd_hash": "not-a-process",
        }]})
        sim.stop()
        self.assertEqual(json.loads(sim.STATE.read_text())["processes"], [])


if __name__ == "__main__":
    unittest.main(verbosity=2)

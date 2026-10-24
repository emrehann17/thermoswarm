#!/usr/bin/env python3
"""Read-only ROS thermal monitor; saves genuine hotspot evidence during user-run flight.

No publisher, vehicle command, Gazebo mutation or synthetic image. Exit 0 requires
confirmed hot frames; timeout/invalid stream exits 1; Ctrl-C exits 130. Use a new
output directory per run to prevent stale PNGs being mistaken for new evidence.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

from thermal_check import contrast, unpack_image, write_png
from wildfire_assets import TOPIC


class Monitor:
    """Bounded-memory acceptance state; transport callbacks supply real frames."""
    def __init__(self, hot_frames=3, min_hz=1, require_airborne=False, idle_timeout=10):
        self.hot_frames = hot_frames
        self.min_hz = min_hz
        self.require_airborne = require_airborne
        self.idle_timeout = idle_timeout
        self.stream_error = None
        self.samples = deque(maxlen=64)
        self.latest = None
        self.metrics = None
        self.telemetry = {}
        self.count = 0
        self.streak = 0
        self.confirmed = False
        self.last_stamp = None
        self.flight = {}

    def update_telemetry(self, kind, msg):
        self.telemetry[kind] = (msg, time.monotonic())

    def flight_state(self, now):
        fresh = all(k in self.telemetry and 0 <= now - self.telemetry[k][1] <= 2
                    for k in ('status', 'land', 'position'))
        result = {'telemetry_fresh': fresh, 'airborne_verified': False}
        if fresh:
            status = self.telemetry['status'][0]
            land = self.telemetry['land'][0]
            position = self.telemetry['position'][0]
            # VehicleStatus.ARMING_STATE_ARMED == 2 in pinned PX4 v1.17 messages.
            armed = status.arming_state == status.ARMING_STATE_ARMED
            altitude_valid = position.z_valid and math.isfinite(position.z)
            result.update({'system_id': int(status.system_id), 'armed': armed,
                           'landed': bool(land.landed),
                           'local_altitude_m': -float(position.z) if altitude_valid else None,
                           'telemetry_age_s': {k: now-v[1] for k,v in self.telemetry.items()}})
            result['airborne_verified'] = (status.system_id == 2 and armed and not land.landed
                                            and altitude_valid and -position.z >= .5)
        return result

    def accept(self, frame):
        if self.last_stamp is not None and frame['stamp'] <= self.last_stamp:
            raise ValueError('Non-increasing image timestamps; restart monitor after simulator reset')
        now = frame['received_monotonic']
        # A gap interrupts the required consecutive observation sequence.
        if self.samples and now - self.samples[-1][0] > 1:
            self.streak = 0
            self.samples.clear()
        self.last_stamp = frame['stamp']
        self.samples.append((now, frame['stamp']))
        self.count += 1
        self.latest, self.metrics = frame, contrast(frame)
        self.flight = self.flight_state(now)
        qualifies = self.metrics['hot_visible'] and (not self.require_airborne or self.flight['airborne_verified'])
        self.streak = self.streak + 1 if qualifies else 0
        self.confirmed = self.streak >= self.hot_frames and self.rates()[0] >= self.min_hz
        return self.confirmed

    def rates(self):
        if len(self.samples) < 2:
            return 0.0, 0.0
        a, b = self.samples[0], self.samples[-1]
        return ((len(self.samples)-1)/(b[0]-a[0]) if b[0] > a[0] else 0.0,
                (len(self.samples)-1)/(b[1]-a[1]) if b[1] > a[1] else 0.0)

    def report(self):
        arrival, simulation = self.rates()
        age = time.monotonic() - self.latest['received_monotonic'] if self.latest else None
        report = {'transport': 'ros', 'topic': TOPIC, 'frames_received': self.count,
                  'arrival_hz': arrival, 'simulation_hz': simulation,
                  'consecutive_hot_frames': self.streak, 'required_hot_frames': self.hot_frames,
                  'min_arrival_hz': self.min_hz, 'require_airborne': self.require_airborne,
                  'thermal_check_pass': (self.count >= 2 and arrival >= self.min_hz
                                         and age is not None and age <= self.idle_timeout
                                         and self.stream_error is None),
                  'hotspot_detected_pass': self.confirmed,
                  'source': 'live ROS sensor subscription; no synthetic image',
                  'recorded_utc': datetime.now(timezone.utc).isoformat(),
                  'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                  'flight_at_image_receipt': self.flight}
        if self.latest:
            report.update({k:v for k,v in self.latest.items() if k not in ('pixels','received_monotonic')})
            report.update(self.metrics)
            report['image_age_s'] = age
        if self.stream_error:
            report['stream_error'] = self.stream_error
        return report


def save_evidence(output, monitor, passed, reason):
    report = monitor.report()
    report.update({'pass': passed, 'reason': reason})
    if monitor.latest:
        name = 'hotspot.png' if passed else 'last-frame-not-accepted.png'
        path = output / name
        write_png(path, monitor.latest)
        report.update({'image_file': name, 'image_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                       'pixel_sha256': hashlib.sha256(monitor.latest['pixels']).hexdigest()})
        path.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def run(args):
    # The monitor has subscriptions only. Telemetry annotates image evidence;
    # it does not alter the mission, preflight policy or stop guard.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleLocalPosition

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    monitor = Monitor(args.hot_frames, args.min_hz, args.require_airborne, args.idle_timeout)
    rclpy.init(args=[])
    node = rclpy.create_node('thermoswarm_thermal_watch', enable_rosout=False,
                             start_parameter_services=False)
    errors = []
    def image_callback(msg):
        try:
            frame = unpack_image(msg.width, msg.height, msg.encoding, msg.step, msg.data,
                                 msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                 msg.header.frame_id)
            monitor.accept(frame)
        except ValueError as exc:
            errors.append(str(exc))
    node.create_subscription(Image, TOPIC, image_callback, qos_profile_sensor_data)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for kind, topic, message_type in (
        ('status', 'vehicle_status_v1', VehicleStatus),
        ('land', 'vehicle_land_detected', VehicleLandDetected),
        ('position', 'vehicle_local_position_v1', VehicleLocalPosition),
    ):
        node.create_subscription(message_type, '/px4_1/fmu/out/' + topic,
                                 lambda msg, k=kind: monitor.update_telemetry(k, msg), qos)
    started = time.monotonic()
    next_report = started
    code, reason = 1, 'timeout: required hotspot evidence not observed'
    try:
        print(f'WATCH READY: {TOPIC}; output={output}; no flight commands', flush=True)
        with (output / 'metrics.jsonl').open('w') as log:
            while time.monotonic() - started < args.timeout:
                rclpy.spin_once(node, timeout_sec=.1)
                now = time.monotonic()
                if errors:
                    raise ValueError(errors[0])
                last = monitor.latest['received_monotonic'] if monitor.latest else started
                if now - last > args.idle_timeout:
                    raise TimeoutError('No fresh thermal image within idle timeout')
                if now >= next_report or monitor.confirmed:
                    line = json.dumps(monitor.report())
                    print(line, flush=True)
                    log.write(line + '\n')
                    log.flush()
                    next_report = now + args.interval
                if monitor.confirmed:
                    code, reason = 0, 'consecutive real hot frames accepted'
                    break
    except KeyboardInterrupt:
        code, reason = 130, 'interrupted by user; hotspot acceptance incomplete'
    except (ValueError, TimeoutError) as exc:
        code, reason = 1, str(exc)
        monitor.stream_error = reason
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = save_evidence(output, monitor, code == 0, reason)
    print('THERMAL CHECK ' + ('PASS' if report['thermal_check_pass'] else 'FAIL') +
          ' (received stream only; see image_age_s for freshness)', flush=True)
    print('HOTSPOT DETECTED ' + ('PASS' if code == 0 else 'FAIL') + ': ' + reason, flush=True)
    print(f'Evidence: {output / "result.json"}', flush=True)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing paths are refused')
    parser.add_argument('--timeout', type=float, default=300, help='Total wall-time wait, seconds')
    parser.add_argument('--idle-timeout', type=float, default=10, help='Maximum wall-time without a new image')
    parser.add_argument('--interval', type=float, default=2, help='Periodic JSON report interval, seconds')
    parser.add_argument('--hot-frames', type=int, default=3, help='Required consecutive qualifying images (at least 3)')
    parser.add_argument('--min-hz', type=float, default=1, help='Minimum recent image arrival rate')
    parser.add_argument('--require-airborne', action='store_true',
                        help='Also require fresh SYSID 2, armed, not landed, valid local altitude >=0.5 m')
    args = parser.parse_args(argv)
    if (any(not math.isfinite(v) or v <= 0 for v in
            (args.timeout, args.idle_timeout, args.interval, args.min_hz)) or args.hot_frames < 3):
        parser.error('time/rate values must be finite and positive; hot-frames >= 3')
    try:
        return run(args)
    except (ImportError, OSError, RuntimeError) as exc:
        print(f'HOTSPOT DETECTED FAIL: {exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

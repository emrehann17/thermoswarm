#!/usr/bin/env python3
"""Read-only ROS 2 guard: permit simulator shutdown only when all X500 are landed/disarmed."""

import argparse
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8)
    parser.add_argument("--instances", default="1,2,3")
    args = parser.parse_args()
    try:
        instances = [int(value) for value in args.instances.split(",")]
        if not instances or len(set(instances)) != len(instances) or any(i not in (1, 2, 3) for i in instances):
            raise ValueError
    except ValueError:
        parser.error("--instances must be unique values from 1,2,3")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from px4_msgs.msg import VehicleLandDetected, VehicleStatus
    except ImportError as exc:
        print(f"Safety telemetry bağımlılığı bulunamadı: {exc}", file=sys.stderr)
        return 2

    class Guard(Node):
        def __init__(self):
            super().__init__("thermoswarm_sim_stop_guard")
            self.states = {i: {"status": None, "landed": None} for i in instances}
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            for i in instances:
                prefix = f"/px4_{i}/fmu/out"
                self.create_subscription(VehicleStatus, f"{prefix}/vehicle_status_v1",
                                         lambda msg, n=i: self.status(n, msg), qos)
                self.create_subscription(VehicleLandDetected, f"{prefix}/vehicle_land_detected",
                                         lambda msg, n=i: self.landed(n, msg), qos)

        def status(self, instance, msg):
            self.states[instance]["status"] = (msg, time.monotonic())

        def landed(self, instance, msg):
            self.states[instance]["landed"] = (msg, time.monotonic())

    try:
        rclpy.init(args=None)
        node = Guard()
    except Exception as exc:
        print(f"Kapanış reddedildi: ROS telemetry guard başlatılamadı: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    deadline = time.monotonic() + args.timeout
    safe_since = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            ready = all(v["status"] is not None and v["landed"] is not None
                        for v in node.states.values())
            if ready:
                unsafe = []
                for instance, data in node.states.items():
                    status, status_at = data["status"]
                    land, land_at = data["landed"]
                    armed = status.arming_state != VehicleStatus.ARMING_STATE_DISARMED
                    landed = bool(land.landed)
                    print(f"x500_{instance}: {'ARMED' if armed else 'DISARMED'}, {'LANDED' if landed else 'AIRBORNE'}")
                    if armed or not landed or now - status_at > 2.0 or now - land_at > 2.0:
                        unsafe.append(f"x500_{instance} (armed={armed}, landed={landed})")
                if unsafe:
                    print("Kapanış reddedildi: " + ", ".join(unsafe), file=sys.stderr)
                    return 1
                safe_since = safe_since if safe_since is not None else now
                if now - safe_since >= 1.0:
                    print(f"STOP GUARD PASS: {len(instances)}/{len(instances)} disarmed + landed with fresh telemetry")
                    return 0
            else:
                safe_since = None
        print("Kapanış reddedildi: timeout içinde tüm sahipli araçların status/land telemetry'si alınamadı.", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"Kapanış reddedildi: telemetry okunamadı: {exc}", file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

"""Regression tests for the installed thermoswarm_control implementation.

These tests use fake telemetry, time, and publishers. They never connect to PX4
or start a flight.
"""

import os
import types
import unittest
from unittest import mock

os.environ.setdefault("ROS_DOMAIN_ID", "87")
os.environ.setdefault("ROS_LOG_DIR", "/tmp")

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus

from thermoswarm_control import thermoswarm_three_x500_node as control


class FakeClock:
    def __init__(self, value=1000.0):
        self.value = value

    def monotonic(self):
        return self.value


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def position(z=0.0):
    return types.SimpleNamespace(
        x=0.0, y=0.0, z=z, vz=0.0, heading=0.0,
        xy_valid=True, z_valid=True, heading_good_for_control=True,
    )


def status(sysid, armed=False, nav=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
    return types.SimpleNamespace(
        system_id=sysid,
        pre_flight_checks_pass=True,
        failsafe=False,
        arming_state=(VehicleStatus.ARMING_STATE_ARMED if armed
                      else VehicleStatus.ARMING_STATE_DISARMED),
        nav_state=nav,
    )


class ThermoSwarmControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.clock = FakeClock()
        self.time_patch = mock.patch.object(control, "time", self.clock)
        self.time_patch.start()
        self.node = control.ThreeX500(execute=False)
        self.command_publishers = {}
        for aircraft in self.node.aircraft:
            publisher = FakePublisher()
            aircraft.pubs = {
                "mode": FakePublisher(),
                "setpoint": FakePublisher(),
                "command": publisher,
            }
            self.command_publishers[aircraft.index] = publisher

    def tearDown(self):
        if self.node is not None:
            self.node.destroy_node()
        self.time_patch.stop()

    def feed_telemetry(self, bad_sysid_index=None):
        for aircraft in self.node.aircraft:
            sysid = aircraft.sysid
            if aircraft.index == bad_sysid_index:
                sysid += 10
            aircraft.position = position()
            aircraft.status = status(sysid)
            aircraft.position_at = self.clock.value
            aircraft.status_at = self.clock.value

    def tick_with_fresh_telemetry(self):
        for aircraft in self.node.aircraft:
            aircraft.position_at = self.clock.value
            aircraft.status_at = self.clock.value
        self.node.tick()

    def test_audit_without_telemetry_fails(self):
        self.node.tick()
        self.clock.value += 25.1
        self.node.tick()

        self.assertEqual(self.node.phase, "FAILED")
        self.assertFalse(any(pub.messages for pub in self.command_publishers.values()))

    def test_audit_with_wrong_sysid_fails(self):
        self.feed_telemetry(bad_sysid_index=2)
        self.node.tick()
        self.clock.value += 1.1
        self.tick_with_fresh_telemetry()

        self.assertEqual(self.node.phase, "FAILED")
        self.assertFalse(any(pub.messages for pub in self.command_publishers.values()))

    def test_audit_with_valid_fake_telemetry_passes_without_commands(self):
        self.feed_telemetry()
        self.node.tick()
        self.clock.value += 1.1
        self.tick_with_fresh_telemetry()

        self.assertEqual(self.node.phase, "DONE")
        self.assertFalse(any(pub.messages for pub in self.command_publishers.values()))

    def test_main_returns_nonzero_when_audit_times_out(self):
        self.node.destroy_node()
        self.node = None
        self.clock.value = 2000.0

        def spin_once(node, timeout_sec):
            self.clock.value += 25.1
            node.tick()

        with (
            mock.patch.object(control.rclpy, "init"),
            mock.patch.object(control.rclpy, "ok", return_value=True),
            mock.patch.object(control.rclpy, "spin_once", side_effect=spin_once),
            mock.patch.object(control.rclpy, "shutdown"),
        ):
            self.assertEqual(control.main([]), 1)

    def test_commands_are_addressed_to_each_vehicle_sysid(self):
        for aircraft in self.node.aircraft:
            self.node.command(aircraft, VehicleCommand.VEHICLE_CMD_NAV_LAND)

        for aircraft in self.node.aircraft:
            sent = self.command_publishers[aircraft.index].messages
            self.assertEqual(len(sent), 1)
            self.assertEqual(sent[0].target_system, aircraft.sysid)

    def test_rejected_ack_is_recorded_and_unrelated_ack_is_ignored(self):
        aircraft = self.node.aircraft[0]
        ack = VehicleCommandAck()
        ack.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        ack.target_system = 1
        self.node.got_ack(aircraft, ack)
        self.assertEqual(aircraft.acks[ack.command][0], ack.result)

        ack.target_system = 42
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.node.got_ack(aircraft, ack)
        self.assertEqual(
            aircraft.acks[ack.command][0],
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
        )

    def test_prepare_preserves_five_waypoint_ned_route(self):
        self.feed_telemetry()
        self.node.prepare(self.clock.value)

        self.assertEqual([len(a.route) for a in self.node.aircraft], [5, 5, 5])
        self.assertEqual([a.route[0][0] for a in self.node.aircraft], ["KALKIS"] * 3)
        self.assertEqual([a.route[1][1][0] for a in self.node.aircraft], [2.0] * 3)
        self.assertEqual([a.route[0][1][2] for a in self.node.aircraft], [-5.0, -8.0, -11.0])

    def test_no_climb_triggers_failure_and_landing(self):
        self.node.execute = True
        self.node.phase = "FLIGHT"
        self.node.phase_since = self.clock.value
        for aircraft in self.node.aircraft:
            aircraft.position = position()
            aircraft.status = status(
                aircraft.sysid, armed=True,
                nav=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            )
            aircraft.position_at = aircraft.status_at = self.clock.value
            aircraft.route = [("KALKIS", (0.0, 0.0, -aircraft.altitude_m, 0.0))]

        self.node.tick()
        self.clock.value += control.CLIMB_CHECK_S + 0.1
        self.tick_with_fresh_telemetry()

        self.assertEqual(self.node.phase, "LAND")
        self.assertIn("YUKSELME YOK", self.node.aircraft[0].reason)
        for aircraft in self.node.aircraft:
            commands = self.command_publishers[aircraft.index].messages
            self.assertTrue(any(
                msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                for msg in commands
            ))

    def test_isolated_no_climb_lands_only_the_failing_vehicle(self):
        self.node.execute = True
        self.node.isolate = True
        self.node.phase = "FLIGHT"
        self.node.phase_since = self.clock.value
        for aircraft in self.node.aircraft:
            aircraft.position = position()
            aircraft.status = status(
                aircraft.sysid, armed=True,
                nav=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            )
            aircraft.position_at = aircraft.status_at = self.clock.value
            aircraft.route = [("KALKIS", (0.0, 0.0, -aircraft.altitude_m, 0.0))]

        self.node.tick()
        self.clock.value += control.CLIMB_CHECK_S + 0.1
        for aircraft in self.node.aircraft:
            aircraft.position_at = aircraft.status_at = self.clock.value
        for aircraft in self.node.aircraft[1:]:
            aircraft.position.z = -control.MIN_CLIMB_M - 0.1
        self.node.tick()

        self.assertEqual(self.node.phase, "FLIGHT")
        self.assertEqual([a.state for a in self.node.aircraft], ["FAILED", "ACTIVE", "ACTIVE"])
        land_indices = [
            index for index, publisher in self.command_publishers.items()
            if any(msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                   for msg in publisher.messages)
        ]
        self.assertEqual(land_indices, [1])

    def test_waypoint_arrival_requires_dwell_then_requests_landing(self):
        self.node.execute = True
        self.node.phase = "FLIGHT"
        self.node.phase_since = self.clock.value
        for aircraft in self.node.aircraft:
            aircraft.position = position()
            aircraft.status = status(
                aircraft.sysid, armed=True,
                nav=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            )
            aircraft.position_at = aircraft.status_at = self.clock.value
            aircraft.route = [("TEST", (0.0, 0.0, 0.0, 0.0))]

        self.node.tick()
        self.assertEqual([a.state for a in self.node.aircraft], ["ACTIVE"] * 3)
        self.clock.value += 1.6
        self.tick_with_fresh_telemetry()

        self.assertEqual([a.state for a in self.node.aircraft], ["COMPLETE"] * 3)
        self.assertEqual(self.node.phase, "LAND")


if __name__ == "__main__":
    unittest.main()

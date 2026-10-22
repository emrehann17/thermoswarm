#!/usr/bin/env python3
"""Offline unit tests for thermoswarm_three_x500_v3.py and thermoswarm_gz_world_check.py.

Safe to run while SITL is running: uses an isolated ROS_DOMAIN_ID and replaces
all publishers with fakes BEFORE any tick, so no message reaches PX4.

    ROS_DOMAIN_ID=87 python3 scripts/test_thermoswarm_three_x500_v3.py -v
"""
import os
import sys
import types
import unittest

os.environ['ROS_DOMAIN_ID'] = os.environ.get('TS_TEST_DOMAIN', '87')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus  # noqa: E402

import thermoswarm_gz_world_check as gzc  # noqa: E402
import thermoswarm_three_x500_v3 as v3  # noqa: E402

# Captured from the real failing session (gz model --list / -p, 2026-10-19).
LIST_OUT = """
Requesting state for world [default]...

Available models:
    - ground_plane
    - x500_0
    - x500_1
    - x500_2
    - x500_3
"""
POSE_OUT = """Requesting state for world [default]...
Model: [49]
  - Name: x500_1
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.007164 0.003842 -0.009097]
    [-0.000465 0.017196 -0.017920]
"""
REAL_POSES = {'x500_0': (-0.0208, -0.0213, -0.013), 'x500_1': (0.0072, 0.0038, -0.0091),
              'x500_2': (5.0179, -0.0007, -0.013), 'x500_3': (10.0061, 0.054, -0.013)}


class GzCheckTests(unittest.TestCase):
    def test_parse_list(self):
        self.assertEqual(gzc.parse_model_list(LIST_OUT),
                         ['ground_plane', 'x500_0', 'x500_1', 'x500_2', 'x500_3'])

    def test_parse_pose(self):
        self.assertAlmostEqual(gzc.parse_pose(POSE_OUT)[0], 0.007164)
        self.assertIsNone(gzc.parse_pose('garbage'))

    def test_real_failure_detected(self):
        models = gzc.parse_model_list(LIST_OUT)
        problems = gzc.evaluate(models, REAL_POSES, list(gzc.DEFAULT_EXPECTED), 1.5)
        self.assertTrue(any('x500_0' in p and 'yetim' in p for p in problems))
        self.assertTrue(any('x500_0 ile x500_1' in p for p in problems))

    def test_clean_world_passes(self):
        models = ['ground_plane', 'x500_1', 'x500_2', 'x500_3']
        poses = {k: v for k, v in REAL_POSES.items() if k != 'x500_0'}
        self.assertEqual(gzc.evaluate(models, poses, list(gzc.DEFAULT_EXPECTED), 1.5), [])

    def test_missing_model(self):
        p = gzc.evaluate(['ground_plane', 'x500_1'], {'x500_1': (0, 0, 0)}, ['x500_1', 'x500_2'], 1.5)
        self.assertTrue(any('yok' in s for s in p))


class ClimbTests(unittest.TestCase):
    def test_classify(self):
        self.assertEqual(v3.classify_climb(3.0, 0.0), 'WAIT')
        self.assertEqual(v3.classify_climb(11.0, 0.02), 'NO_CLIMB')
        self.assertEqual(v3.classify_climb(11.0, 0.6), 'OK')
        self.assertEqual(v3.classify_climb(1.0, 0.6), 'OK')


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class FakePub:
    def __init__(self, log, idx, kind):
        self.log, self.idx, self.kind = log, idx, kind

    def publish(self, msg):
        self.log.append((self.idx, self.kind, getattr(msg, 'command', None),
                         getattr(msg, 'target_system', None)))


def pos(z, x=0.0, y=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z, vz=0.0, heading=0.0, xy_valid=True, z_valid=True,
                                 heading_good_for_control=True)


def status(sysid, armed=False, nav=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
    return types.SimpleNamespace(
        system_id=sysid, pre_flight_checks_pass=True, failsafe=False,
        arming_state=VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED,
        nav_state=nav)


class StateMachineReplay(unittest.TestCase):
    """Replays the 2026-10-19 failure: px4_1 armed+Offboard, z stays 0."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def make(self, isolate):
        self.clock = FakeClock()
        v3.time = types.SimpleNamespace(monotonic=self.clock.monotonic)
        node = v3.ThreeX500(execute=True, isolate_failures=isolate)
        self.sent = []
        for a in node.aircraft:
            a.pubs = {k: FakePub(self.sent, a.index, k) for k in ('mode', 'setpoint', 'command')}
            a.position, a.status = pos(0.0), status(a.sysid)
        self.node = node
        return node

    def feed(self, climbing):
        for a in self.node.aircraft:
            a.position_at = a.status_at = self.clock.t
            cmds = [c for (i, k, c, _) in self.sent if i == a.index and k == 'command']
            if VehicleCommand.VEHICLE_CMD_NAV_LAND in cmds:
                a.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                a.position.z = min(a.position.z + 0.1, 0.0)
                if a.position.z >= 0.0:
                    a.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            elif VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM in cmds:
                a.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
                a.status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
                if a.index in climbing and a.route:
                    tx, ty, tz, _ = a.route[a.waypoint][1]
                    a.position.x, a.position.y, a.position.z = tx, ty, tz

    def run_until_end(self, climbing, max_s=300):
        for _ in range(int(max_s * 10)):
            self.clock.t += 0.1
            self.feed(climbing)
            self.node.tick()
            if self.node.phase in ('DONE', 'FAILED'):
                return
        self.fail('state machine did not finish')

    def test_commands_target_correct_sysid(self):
        node = self.make(isolate=False)
        self.run_until_end(climbing={1, 2, 3})
        self.assertEqual(node.phase, 'DONE')
        for idx, kind, _, tgt in self.sent:
            if kind == 'command':
                self.assertEqual(tgt, idx + 1)
        self.assertTrue(all(a.state == 'COMPLETE' for a in node.aircraft))
        node.destroy_node()

    def test_no_climb_detected_fast_group_land(self):
        node = self.make(isolate=False)
        self.run_until_end(climbing={2, 3})
        self.assertEqual(node.phase, 'FAILED')
        a1 = node.aircraft[0]
        self.assertEqual(a1.state, 'FAILED')
        self.assertIn('YUKSELME YOK', a1.reason)
        self.assertIn('YUKSELME YOK', node.reason)
        node.destroy_node()

    def test_no_climb_isolated_others_complete(self):
        node = self.make(isolate=True)
        self.run_until_end(climbing={2, 3})
        self.assertEqual(node.phase, 'FAILED')
        states = [a.state for a in node.aircraft]
        self.assertEqual(states, ['FAILED', 'COMPLETE', 'COMPLETE'])
        land1 = [s for s in self.sent if s[0] == 1 and s[2] == VehicleCommand.VEHICLE_CMD_NAV_LAND]
        self.assertTrue(land1)
        node.destroy_node()

    def test_ack_rejection_logged(self):
        node = self.make(isolate=False)
        a = node.aircraft[0]
        ack = VehicleCommandAck()
        ack.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        ack.target_system = 1
        node.got_ack(a, ack)
        self.assertEqual(a.acks[ack.command][0], VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED)
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()

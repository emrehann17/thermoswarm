"""CLI/preflight routing only: no ROS node, flight or publishers are created."""
import contextlib
import io
import unittest
from unittest import mock

from thermoswarm_control import mission_node
from thermoswarm_control import thermoswarm_gz_world_check as world_check


class PreflightTests(unittest.TestCase):
    def test_typo_and_abbreviated_execute_are_usage_errors_before_any_check(self):
        for arg in ('--executete', '--exec'):
            output = io.StringIO()
            with mock.patch.object(mission_node.rclpy, 'init') as init, \
                 mock.patch.object(world_check, 'check_world') as check, \
                 contextlib.redirect_stderr(output), self.assertRaises(SystemExit) as error:
                mission_node.main([arg])
            self.assertEqual(error.exception.code, 2)
            self.assertIn('unrecognized arguments: ' + arg, output.getvalue())
            self.assertIn('--execute', output.getvalue())
            init.assert_not_called()
            check.assert_not_called()

    def test_failed_world_prevents_ros_init_and_flight(self):
        with mock.patch.object(world_check, 'check_world', return_value=(False, ['obstacle'])) as check, \
             mock.patch.object(mission_node.rclpy, 'init') as init, \
             mock.patch.object(mission_node, 'MissionNode') as node:
            self.assertEqual(mission_node.main(['--execute']), 1)
        check.assert_called_once_with()  # auto uses authoritative live world discovery
        init.assert_not_called()
        node.assert_not_called()

    def test_valid_execute_reaches_same_world_check_before_lifecycle(self):
        # Deliberate exception at ROS init proves ordering without starting a node.
        with mock.patch.object(world_check, 'check_world', return_value=(True, [])) as check, \
             mock.patch.object(mission_node.rclpy, 'init', side_effect=RuntimeError('test stops before ROS')), \
             mock.patch.object(mission_node, 'MissionNode') as node, \
             self.assertRaisesRegex(RuntimeError, 'test stops before ROS'):
            mission_node.main(['--execute'])
        check.assert_called_once_with()
        node.assert_not_called()

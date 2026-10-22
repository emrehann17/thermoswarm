import unittest
from types import SimpleNamespace

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from thermoswarm_control.offboard import OffboardInterface
from thermoswarm_control.vehicle import create_fleet


class FakeLogger:
    def info(self, message):
        pass

    def error(self, message):
        pass


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.subscriptions = []
        self.logger = FakeLogger()

    def create_publisher(self, message_type, topic, qos):
        self.publishers.append((message_type, topic, qos))
        return object()

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((message_type, topic, callback, qos))
        return object()

    def get_logger(self):
        return self.logger


class OffboardInterfaceTests(unittest.TestCase):
    def test_topics_and_qos_match_the_existing_px4_interface(self):
        node = FakeNode()
        interface = OffboardInterface(node, create_fleet())

        self.assertEqual(len(node.publishers), 9)
        self.assertEqual(len(node.subscriptions), 12)
        self.assertEqual(set(interface.publishers), {1, 2, 3})
        self.assertEqual(
            {topic for _, topic, _ in node.publishers},
            {
                f"/px4_{index}/fmu/in/{name}"
                for index in (1, 2, 3)
                for name in (
                    "offboard_control_mode",
                    "trajectory_setpoint",
                    "vehicle_command",
                )
            },
        )
        self.assertEqual(
            {topic for _, topic, _, _ in node.subscriptions},
            {
                f"/px4_{index}/fmu/out/{name}"
                for index in (1, 2, 3)
                for name in (
                    "vehicle_local_position_v1",
                    "vehicle_status_v1",
                    "vehicle_command_ack",
                    "vehicle_land_detected",
                )
            },
        )
        for _, _, qos in node.publishers:
            self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)
            self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)
            self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
            self.assertEqual(qos.depth, 1)

    def test_setpoint_publishes_offboard_mode_and_current_waypoint(self):
        node = FakeNode()
        aircraft, *_ = create_fleet()

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        interface = OffboardInterface(node, [aircraft])
        mode_pub, setpoint_pub = Publisher(), Publisher()
        interface.publishers[aircraft.index]["mode"] = mode_pub
        interface.publishers[aircraft.index]["setpoint"] = setpoint_pub
        aircraft.route = [("A", (1.0, 2.0, -5.0, 0.4))]
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123_000_000,
        ))

        interface.publish_setpoint(aircraft)

        self.assertIsInstance(mode_pub.messages[0], OffboardControlMode)
        self.assertTrue(mode_pub.messages[0].position)
        self.assertIsInstance(setpoint_pub.messages[0], TrajectorySetpoint)
        self.assertEqual(list(setpoint_pub.messages[0].position), [1.0, 2.0, -5.0])
        self.assertAlmostEqual(setpoint_pub.messages[0].yaw, 0.4)


if __name__ == "__main__":
    unittest.main()

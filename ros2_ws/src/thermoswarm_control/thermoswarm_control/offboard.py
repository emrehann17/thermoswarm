"""ROS 2 and PX4 Offboard communication for the vehicle fleet."""

import time

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


CMD_NAMES = {
    VehicleCommand.VEHICLE_CMD_DO_SET_MODE: 'DO_SET_MODE',
    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
    VehicleCommand.VEHICLE_CMD_NAV_LAND: 'NAV_LAND',
}
ACK_OK = (
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
)


class OffboardInterface:
    """Own the PX4 ROS topics and translate mission actions into messages."""

    def __init__(self, node, aircraft):
        self.node = node
        self.logger = node.get_logger()
        self.publishers = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for vehicle in aircraft:
            prefix = f'{vehicle.namespace}/fmu'
            self.publishers[vehicle.index] = {
                'mode': node.create_publisher(
                    OffboardControlMode,
                    f'{prefix}/in/offboard_control_mode',
                    qos,
                ),
                'setpoint': node.create_publisher(
                    TrajectorySetpoint,
                    f'{prefix}/in/trajectory_setpoint',
                    qos,
                ),
                'command': node.create_publisher(
                    VehicleCommand,
                    f'{prefix}/in/vehicle_command',
                    qos,
                ),
            }
            node.create_subscription(
                VehicleLocalPosition,
                f'{prefix}/out/vehicle_local_position_v1',
                lambda msg, vehicle=vehicle: self.got_position(vehicle, msg),
                qos,
            )
            node.create_subscription(
                VehicleStatus,
                f'{prefix}/out/vehicle_status_v1',
                lambda msg, vehicle=vehicle: self.got_status(vehicle, msg),
                qos,
            )
            node.create_subscription(
                VehicleCommandAck,
                f'{prefix}/out/vehicle_command_ack',
                lambda msg, vehicle=vehicle: self.got_ack(vehicle, msg),
                qos,
            )
            node.create_subscription(
                VehicleLandDetected,
                f'{prefix}/out/vehicle_land_detected',
                lambda msg, vehicle=vehicle: setattr(vehicle, 'land', msg),
                qos,
            )

    def got_position(self, aircraft, msg):
        aircraft.position = msg
        aircraft.position_at = time.monotonic()

    def got_status(self, aircraft, msg):
        aircraft.status = msg
        aircraft.status_at = time.monotonic()

    def got_ack(self, aircraft, msg):
        # PX4 addresses ACKs to our source_system (1); ignore unrelated ACKs.
        if msg.target_system not in (0, 1) or msg.command not in CMD_NAMES:
            return
        aircraft.acks[msg.command] = (msg.result, time.monotonic())
        name = CMD_NAMES[msg.command]
        if msg.result in ACK_OK:
            self.logger.info(f'px4_{aircraft.index} ACK {name}: result={msg.result} (kabul)')
        else:
            self.logger.error(f'px4_{aircraft.index} ACK {name}: result={msg.result} (RED)')

    def stamp(self):
        return self.node.get_clock().now().nanoseconds // 1000

    def command(self, aircraft, code, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.stamp()
        msg.command = code
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = aircraft.sysid  # Target the vehicle, never broadcast.
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publishers[aircraft.index]['command'].publish(msg)
        aircraft.last_command = time.monotonic()

    def publish_setpoint(self, aircraft):
        if not aircraft.route:
            return
        _, (x, y, z, yaw) = aircraft.route[aircraft.waypoint]
        mode = OffboardControlMode()
        mode.timestamp = self.stamp()
        mode.position = True
        self.publishers[aircraft.index]['mode'].publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.stamp()
        setpoint.position = [float(x), float(y), float(z)]
        setpoint.yaw = float(yaw)
        self.publishers[aircraft.index]['setpoint'].publish(setpoint)

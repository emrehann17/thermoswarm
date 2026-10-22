#!/usr/bin/env python3
"""ThermoSwarm: one-X500 PX4 v1.17 ROS 2 SITL-only flight test."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)


class ThermoSwarmFlight(Node):
    def __init__(self):
        super().__init__('thermoswarm_single_x500_test')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.control_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.target_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status_v1', self.on_status, qos)

        self.position = None
        self.status = None
        self.phase = 'WAIT'
        self.phase_started = time.monotonic()
        self.ticks = 0
        self.target = None
        self.last_command = 0.0
        self.last_notice = 0.0
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.get_logger().info('PX4 telemetrisi ve preflight hazirligi bekleniyor.')

    def on_position(self, msg):
        self.position = msg

    def on_status(self, msg):
        self.status = msg

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def stage(self, name):
        self.phase = name
        self.phase_started = time.monotonic()
        self.get_logger().info(f'Asama: {name}')

    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = self.stamp()
        msg.position = True
        self.control_pub.publish(msg)

        sp = TrajectorySetpoint()
        sp.timestamp = self.stamp()
        sp.position = [self.target[0], self.target[1], self.target[2]]
        sp.yaw = self.target[3]
        self.target_pub.publish(sp)

    def command(self, code, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.stamp()
        msg.command = code
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.last_command = time.monotonic()

    def land(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('ROS 2 uzerinden LAND komutu gonderildi.')
        self.stage('LAND')

    def tick(self):
        now = time.monotonic()
        if self.position is None or self.status is None:
            return

        if self.phase == 'WAIT':
            ready = (self.position.xy_valid and self.position.z_valid
                     and self.status.pre_flight_checks_pass
                     and not self.status.failsafe
                     and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED)
            if not ready:
                if now - self.last_notice > 5:
                    self.get_logger().warning('Konum veya preflight henüz hazir degil.')
                    self.last_notice = now
                return
            p = self.position
            self.target = (float(p.x), float(p.y), float(p.z - 5.0), float(p.heading))
            self.get_logger().info(f'Hedef (NED): x={self.target[0]:.2f}, '
                                   f'y={self.target[1]:.2f}, z={self.target[2]:.2f}')
            self.stage('WARMUP')

        if self.phase in ('WARMUP', 'ASCEND', 'HOLD') or (
                self.phase == 'LAND'
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.publish_offboard()

        if self.phase == 'WARMUP':
            self.ticks += 1
            if self.ticks >= 20:  # 2 seconds of 10 Hz proof-of-life
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.stage('ASCEND')

        elif self.phase == 'ASCEND':
            armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if armed and offboard and self.position.z <= self.target[2] + 0.4:
                self.get_logger().info(f'5 m hedefine ulasildi: z={self.position.z:.2f}')
                self.stage('HOLD')
            elif now - self.phase_started > 30:
                if armed:
                    self.get_logger().warning('Yukselis zaman asimi; inis deneniyor.')
                    self.land()
                else:
                    self.get_logger().error('Arm/Offboard basarisiz. PX4 loglarini kontrol et.')
                    self.stage('DONE')
                    self.timer.cancel()
            elif now - self.last_command > 4 and not (armed and offboard):
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                if not armed:
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        elif self.phase == 'HOLD':
            if now - self.phase_started >= 5:
                self.land()

        elif self.phase == 'LAND':
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('BASARILI: Inis ve disarm ROS 2 ile dogrulandi.')
                self.stage('DONE')
                self.timer.cancel()
            elif (self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                  and now - self.last_command > 3):
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().warning('LAND modu henüz aktif degil; tekrar denendi.')


def main():
    rclpy.init()
    node = ThermoSwarmFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

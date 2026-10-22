#!/usr/bin/env python3
"""ThermoSwarm — SINGLE-VEHICLE Gazebo SITL-only ROS 2 waypoint test.

PX4 v1.17 / px4_msgs release/1.17 / ROS 2 Jazzy.
Uses PX4 NED coordinates (x North, y East, z Down).
Never run on real hardware. Launch only with --execute.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class ThermoSwarmWaypointTest(Node):
    def __init__(self):
        super().__init__('thermoswarm_x500_waypoint_test')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.on_status, qos)

        self.position = None
        self.status = None
        self.last_position_at = 0.0
        self.last_status_at = 0.0
        self.phase = 'WAIT'
        self.phase_since = time.monotonic()
        self.last_notice = 0.0
        self.last_command = 0.0
        self.proof_of_life_count = 0
        self.offboard_seen = False
        self.route = []
        self.leg_index = 0
        self.leg_since = 0.0
        self.arrival_since = None
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz heartbeat
        self.get_logger().info(
            'SITL-only waypoint testi basladi; konum ve preflight bekleniyor.')

    def on_position(self, msg):
        self.position = msg
        self.last_position_at = time.monotonic()

    def on_status(self, msg):
        self.status = msg
        self.last_status_at = time.monotonic()

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def set_phase(self, phase):
        self.phase = phase
        self.phase_since = time.monotonic()
        self.get_logger().info(f'ASAMA: {phase}')

    def command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.stamp()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1  # ONLY the single PX4 instance (system_id=1)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.last_command = time.monotonic()

    def publish_offboard(self):
        if not self.route:
            return
        target = self.route[self.leg_index][1]
        mode = OffboardControlMode()
        mode.timestamp = self.stamp()
        mode.position = True
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.stamp()
        sp.position = [float(target[0]), float(target[1]), float(target[2])]
        sp.yaw = float(target[3])
        self.setpoint_pub.publish(sp)

    def begin_landing(self, why):
        if self.phase == 'LAND':
            return
        self.get_logger().warning(f'INIS: {why}')
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.set_phase('LAND')

    def current_leg_name(self):
        return self.route[self.leg_index][0]

    def tick(self):
        now = time.monotonic()
        if self.phase in ('DONE', 'FAILED'):
            return

        if self.phase == 'WAIT' and now - self.phase_since > 60:
            self.get_logger().error('60 saniyede telemetri/preflight hazir olmadi.')
            self.set_phase('FAILED')
            return

        fresh = (self.position is not None and self.status is not None
                 and now - self.last_position_at < 2.0
                 and now - self.last_status_at < 2.0)
        if not fresh:
            if now - self.last_notice > 5:
                self.get_logger().warning('PX4 konum veya durum telemetrisi bekleniyor.')
                self.last_notice = now
            if self.phase not in ('WAIT', 'LAND') and now - min(
                    self.last_position_at, self.last_status_at) > 3.0:
                self.begin_landing('Telemetri guncel degil; LAND gonderiliyor.')
            return

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if offboard:
            self.offboard_seen = True

        if self.phase == 'WAIT':
            ready = (self.position.xy_valid and self.position.z_valid
                     and self.status.pre_flight_checks_pass
                     and not self.status.failsafe
                     and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED)
            if not ready:
                if now - self.last_notice > 5:
                    self.get_logger().warning('Preflight veya konum henuz hazir degil.')
                    self.last_notice = now
                return

            p = self.position
            x, y, z, yaw = float(p.x), float(p.y), float(p.z - 5), float(p.heading)
            # Four 4-m legs; all targets are 5 m above start, in local NED.
            self.route = [
                ('KALKIS_5M', (x, y, z, yaw)),
                ('WP1_KUZEY_4M', (x + 4, y, z, yaw)),
                ('WP2_DOGU_4M', (x + 4, y + 4, z, yaw)),
                ('WP3_GUNEY_4M', (x, y + 4, z, yaw)),
                ('BASLANGICA_DON', (x, y, z, yaw)),
            ]
            self.leg_index = 0
            self.leg_since = now
            self.get_logger().info(
                f'BASLANGIC NED x={x:.2f}, y={y:.2f}, hedef z={z:.2f}. '
                'Rota: yukselis, WP1, WP2, WP3, baslangica donus, inis.')
            self.set_phase('WARMUP')

        # In landing mode let PX4 control the descent. Continue the proof-of-life
        # only while PX4 still reports Offboard, as in the working prior script.
        if self.phase in ('WARMUP', 'FLIGHT') or (self.phase == 'LAND' and offboard):
            self.publish_offboard()

        if self.phase == 'WARMUP':
            self.proof_of_life_count += 1
            if self.proof_of_life_count >= 20:  # 2 seconds at 10 Hz
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.set_phase('FLIGHT')
                self.leg_since = now
                self.get_logger().info('Aktif hedef: KALKIS_5M')

        elif self.phase == 'FLIGHT':
            if self.status.failsafe:
                self.begin_landing('PX4 failsafe bildirdi.')
                return
            if self.offboard_seen and not offboard:
                self.begin_landing('Offboard modu beklenmedik bicimde kayboldu.')
                return
            if not (armed and offboard):
                if now - self.phase_since > 18:
                    if armed:
                        self.begin_landing('Arm/Offboard modu olusmadi.')
                    else:
                        self.get_logger().error('Arm/Offboard basarisiz; ucus baslamadi.')
                        self.set_phase('FAILED')
                    return
                if now - self.last_command > 4:
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    if not armed:
                        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                return

            label, (tx, ty, tz, _) = self.route[self.leg_index]
            distance_xy = math.hypot(self.position.x - tx, self.position.y - ty)
            distance_z = abs(self.position.z - tz)
            if distance_xy < 0.6 and distance_z < 0.5:
                if self.arrival_since is None:
                    self.arrival_since = now
                if now - self.arrival_since >= 1.5:
                    self.get_logger().info(
                        f'ULASILDI {label}: xy_hata={distance_xy:.2f}m, '
                        f'z_hata={distance_z:.2f}m')
                    if self.leg_index + 1 == len(self.route):
                        self.begin_landing('Tum waypointler tamamlandi.')
                    else:
                        self.leg_index += 1
                        self.arrival_since = None
                        self.leg_since = now
                        self.get_logger().info(
                            f'Yeni hedef: {self.current_leg_name()}')
                return
            self.arrival_since = None
            if now - self.leg_since > 40:
                self.begin_landing(f'Waypoint zaman asimi: {label}')

        elif self.phase == 'LAND':
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('BASARILI: Inis ve disarm dogrulandi.')
                self.set_phase('DONE')
                return
            if (self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                    and now - self.last_command > 4):
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().warning('LAND modu bekleniyor; komut tekrarlandi.')
            if now - self.phase_since > 60 and now - self.last_notice > 8:
                self.get_logger().error(
                    'Inis/durma henuz dogrulanamadi. PX4 terminalinden '
                    'durumu kontrol et ve gerekirse commander land uygula.')
                self.last_notice = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Allow automatic arming/takeoff in Gazebo SITL.')
    args = parser.parse_args()
    if not args.execute:
        parser.error('SITL ucusunu baslatmak icin --execute gerekli.')

    rclpy.init()
    node = ThermoSwarmWaypointTest()
    try:
        while rclpy.ok() and node.phase not in ('DONE', 'FAILED'):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        # If already flying, send a LAND request before quitting. This is only
        # a convenience for SITL, not a guarantee of a completed landing.
        if (node.status is not None and
                node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            node.get_logger().warning('Ctrl+C alindi; LAND komutu gonderiliyor.')
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

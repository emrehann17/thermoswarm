#!/usr/bin/env python3
"""ThermoSwarm three-X500 Gazebo SITL-only ROS 2 flight test.

PX4 v1.17.0 + px4_msgs release/1.17 + ROS 2 Jazzy.
Run first WITHOUT --execute for a read-only three-vehicle preflight audit.
Use --execute only in Gazebo SITL with the instances PX4 -i 1, -i 2, -i 3.

Coordinates are local PX4 NED (x North, y East, z Down) for EACH drone.
Routes use small 2 m squares, with 5/8/11 m AGL targets to separate aircraft.
This is an integration demo, NOT collision avoidance or a real-hardware controller.
"""

import argparse
import math
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleStatus, OffboardControlMode,
    TrajectorySetpoint, VehicleCommand,
)


@dataclass
class Aircraft:
    index: int
    sysid: int
    altitude_m: float
    position: object = None
    status: object = None
    position_at: float = 0.0
    status_at: float = 0.0
    route: list = field(default_factory=list)
    waypoint: int = 0
    dwell_since: object = None
    leg_since: float = 0.0
    last_command: float = 0.0
    offboard_ever: bool = False
    complete: bool = False
    pubs: dict = field(default_factory=dict)


class ThreeX500(Node):
    def __init__(self, execute):
        super().__init__('thermoswarm_three_x500_test')
        self.execute = execute
        self.aircraft = [Aircraft(1, 2, 5.0), Aircraft(2, 3, 8.0), Aircraft(3, 4, 11.0)]
        self.phase = 'WAIT'
        self.phase_since = time.monotonic()
        self.notice_at = 0.0
        self.heartbeat_count = 0
        self.last_tick = time.monotonic()
        self.aborted = False
        self.reason = ''
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for a in self.aircraft:
            prefix = f'/px4_{a.index}/fmu'
            a.pubs['mode'] = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos)
            a.pubs['setpoint'] = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos)
            a.pubs['command'] = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', qos)
            self.create_subscription(VehicleLocalPosition,
                f'{prefix}/out/vehicle_local_position_v1',
                lambda msg, a=a: self.got_position(a, msg), qos)
            self.create_subscription(VehicleStatus,
                f'{prefix}/out/vehicle_status_v1',
                lambda msg, a=a: self.got_status(a, msg), qos)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'Uc X500 telemetri/preflight kontrolu. ' +
            ('--execute AKTIF: hazir olduklarinda ucus baslar.' if execute
             else 'Salt-okunur mod: HICBIR ucus komutu gonderilmeyecek.'))

    def got_position(self, a, msg):
        a.position = msg
        a.position_at = time.monotonic()

    def got_status(self, a, msg):
        a.status = msg
        a.status_at = time.monotonic()

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def change(self, phase):
        self.phase = phase
        self.phase_since = time.monotonic()
        self.get_logger().info(f'ASAMA: {phase}')

    def fresh(self, a, now):
        return (a.position is not None and a.status is not None
                and now - a.position_at < 2.0 and now - a.status_at < 2.0)

    def correct_sysid(self, a):
        return a.status is not None and a.status.system_id == a.sysid

    def ready(self, a, now):
        return (self.fresh(a, now) and self.correct_sysid(a)
                and a.position.xy_valid and a.position.z_valid
                and a.status.pre_flight_checks_pass and not a.status.failsafe
                and a.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED)

    def report(self, now):
        for a in self.aircraft:
            if not self.fresh(a, now):
                self.get_logger().warning(f'px4_{a.index}: telemetri yok / guncel degil')
                continue
            p, s = a.position, a.status
            self.get_logger().info(
                f'px4_{a.index}: sysid={s.system_id} (beklenen {a.sysid}), '
                f'pos_valid={p.xy_valid and p.z_valid}, heading_ok={p.heading_good_for_control}, '
                f'preflight={s.pre_flight_checks_pass}, failsafe={s.failsafe}, '
                f'armed={s.arming_state == VehicleStatus.ARMING_STATE_ARMED}, '
                f'yerel_NED=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def command(self, a, code, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.stamp()
        msg.command = code
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = a.sysid  # CRITICAL: not broadcast and not always 1!
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        a.pubs['command'].publish(msg)
        a.last_command = time.monotonic()

    def publish_setpoint(self, a):
        if not a.route:
            return
        name, (x, y, z, yaw) = a.route[a.waypoint]
        mode = OffboardControlMode()
        mode.timestamp = self.stamp()
        mode.position = True
        a.pubs['mode'].publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = self.stamp()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        a.pubs['setpoint'].publish(sp)

    def prepare(self, now):
        for a in self.aircraft:
            p = a.position
            x, y, z, yaw = float(p.x), float(p.y), float(p.z-a.altitude_m), float(p.heading)
            a.route = [
                ('KALKIS', (x, y, z, yaw)),
                ('KUZEY_2M', (x+2, y, z, yaw)),
                ('DOGU_2M', (x+2, y+2, z, yaw)),
                ('GUNEY_2M', (x, y+2, z, yaw)),
                ('EVE_DON', (x, y, z, yaw)),
            ]
            a.waypoint, a.leg_since, a.dwell_since, a.complete = 0, now, None, False
            self.get_logger().info(f'px4_{a.index}: sysid={a.sysid} irtifa={a.altitude_m:.0f} m, 2m-kare rota')

    def request_landing(self, why, abort=False):
        if self.phase in ('LAND', 'DONE', 'FAILED'):
            return
        self.get_logger().warning(f'TOPLU INIS: {why}')
        if abort:
            self.aborted, self.reason = True, why
        for a in self.aircraft:
            if a.status is not None and a.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.change('LAND')

    def tick(self):
        now = time.monotonic()
        if self.phase in ('DONE', 'FAILED'):
            return
        if self.phase == 'WAIT':
            if now - self.notice_at > 4:
                self.report(now)
                self.notice_at = now
            if not self.execute:
                if all(self.fresh(a, now) for a in self.aircraft):
                    self.get_logger().info('Salt-okunur kontrol tamamlandi. Ucus komutu gonderilmedi.')
                    self.change('DONE')
                elif now - self.phase_since > 25:
                    self.change('FAILED')
                return
            if all(self.ready(a, now) for a in self.aircraft):
                self.prepare(now)
                self.change('WARMUP')
                return
            if now - self.phase_since > 90:
                self.get_logger().error('90 sn: uc arac ayni anda preflight READY degil. Ucus baslatilmadi.')
                self.change('FAILED')
            return

        stale = [a.index for a in self.aircraft if not self.fresh(a, now)]
        if stale and self.phase in ('WARMUP', 'FLIGHT'):
            self.request_landing(f'telemetri kaybi: {stale}', abort=True)
            return

        if self.phase in ('WARMUP', 'FLIGHT'):
            for a in self.aircraft:
                self.publish_setpoint(a)
        elif self.phase == 'LAND':
            # Keep heartbeat while PX4 still says Offboard, without overriding AUTO_LAND.
            for a in self.aircraft:
                if (self.fresh(a, now) and
                    a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                    self.publish_setpoint(a)

        if self.phase == 'WARMUP':
            self.heartbeat_count += 1
            if self.heartbeat_count >= 20:
                if not all(self.ready(a, now) for a in self.aircraft):
                    self.request_landing('WARMUP sonunda preflight bozuldu', abort=True)
                    return
                for a in self.aircraft:
                    self.command(a, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.command(a, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.change('FLIGHT')
            return

        if self.phase == 'FLIGHT':
            if any(a.status.failsafe for a in self.aircraft):
                self.request_landing('PX4 failsafe', abort=True)
                return
            for a in self.aircraft:
                armed = a.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                offboard = a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                if offboard:
                    a.offboard_ever = True
                if a.offboard_ever and not offboard:
                    self.request_landing(f'px4_{a.index} Offboard kaybetti', abort=True)
                    return
                if not (armed and offboard):
                    if now - self.phase_since > 20:
                        self.request_landing(f'px4_{a.index} arm/offboard basarisiz', abort=True)
                        return
                    if now - a.last_command > 4:
                        self.command(a, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        if not armed:
                            self.command(a, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    continue

                if a.complete:
                    continue
                label, (tx, ty, tz, _) = a.route[a.waypoint]
                xy_error = math.hypot(a.position.x-tx, a.position.y-ty)
                z_error = abs(a.position.z-tz)
                if xy_error < 0.6 and z_error < 0.5:
                    if a.dwell_since is None:
                        a.dwell_since = now
                    if now-a.dwell_since >= 1.5:
                        self.get_logger().info(
                            f'px4_{a.index} ULASILDI {label}: xy={xy_error:.2f} m, z={z_error:.2f} m')
                        if a.waypoint + 1 == len(a.route):
                            a.complete = True
                            self.get_logger().info(f'px4_{a.index}: tum waypointler tamamlandi')
                        else:
                            a.waypoint += 1
                            a.leg_since = now
                        a.dwell_since = None
                else:
                    a.dwell_since = None
                if not a.complete and now-a.leg_since > 40:
                    self.request_landing(f'px4_{a.index} waypoint zaman asimi: {label}', abort=True)
                    return
            if all(a.complete for a in self.aircraft):
                self.request_landing('3/3 rota tamamlandi')
            if now-self.phase_since > 180:
                self.request_landing('genel ucus zaman asimi', abort=True)
            return

        if self.phase == 'LAND':
            if all(a.status is not None and self.fresh(a, now)
                   and a.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                   for a in self.aircraft):
                if self.aborted:
                    self.get_logger().error(f'Ucus iptal edildi, tum araclar disarm: {self.reason}')
                    self.change('FAILED')
                else:
                    self.get_logger().info('BASARILI: 3/3 waypoint rotasi, inis ve disarm dogrulandi.')
                    self.change('DONE')
                return
            for a in self.aircraft:
                if (a.status is not None
                    and a.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and a.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                    and now-a.last_command > 3):
                    self.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if now-self.phase_since > 65 and now-self.notice_at > 8:
                self.get_logger().error(
                    'Inis teyit edilemiyor: pxh> commander land her aktif arac icin kontrol edilmeli.')
                self.notice_at = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Gazebo SITL ortaminda uc arac icin otomatik ucusu etkinlestir')
    args = parser.parse_args()
    rclpy.init()
    node = ThreeX500(execute=args.execute)
    try:
        while rclpy.ok() and node.phase not in ('DONE', 'FAILED'):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if args.execute and node.phase not in ('WAIT', 'DONE', 'FAILED'):
            node.request_landing('Ctrl+C', abort=True)
            deadline = time.monotonic()+5
            while rclpy.ok() and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

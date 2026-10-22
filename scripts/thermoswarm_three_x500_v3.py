#!/usr/bin/env python3
"""ThermoSwarm three-X500 Gazebo SITL-only ROS 2 flight test (v3).

PX4 v1.17.0 + px4_msgs release/1.17 + ROS 2 Jazzy.
Run first WITHOUT --execute for a read-only three-vehicle preflight audit.
Use --execute only in Gazebo SITL with the instances PX4 -i 1, -i 2, -i 3.

Coordinates are local PX4 NED (x North, y East, z Down) for EACH drone.
Routes use small 2 m squares, with 5/8/11 m AGL targets to separate aircraft.
This is an integration demo, NOT collision avoidance or a real-hardware controller.

v3 changes vs thermoswarm_three_x500_audit_fixed.py (root cause: orphan Gazebo
model x500_0 overlapping x500_1, so PX4-1 could not physically climb):
  * --execute first runs a read-only Gazebo world check (orphan/overlapping models).
  * VehicleCommandAck and VehicleLandDetected are tracked per vehicle.
  * Takeoff leg timer starts when arm+Offboard is CONFIRMED, not at route prepare.
  * NO-CLIMB detection: armed+Offboard but < 0.5 m climb after 10 s -> explicit failure.
  * Per-vehicle state (ACTIVE/COMPLETE/FAILED + reason) and per-vehicle final report.
  * Periodic per-vehicle log: target vs actual position, errors, arm/nav/land state.
  * Optional --isolate-failures: a failing vehicle lands alone, others continue.
    Default keeps the previous conservative behaviour (land all on any failure).
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleStatus, OffboardControlMode,
    TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLandDetected,
)

CLIMB_CHECK_S = 10.0      # time after confirmed arm+Offboard to show real climb
MIN_CLIMB_M = 0.5         # minimum climb expected within CLIMB_CHECK_S
LEG_TIMEOUT_S = 40.0
DIAG_PERIOD_S = 2.0

CMD_NAMES = {
    VehicleCommand.VEHICLE_CMD_DO_SET_MODE: 'DO_SET_MODE',
    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: 'ARM_DISARM',
    VehicleCommand.VEHICLE_CMD_NAV_LAND: 'NAV_LAND',
}
ACK_OK = (VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
          VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS)


@dataclass
class Aircraft:
    index: int
    sysid: int
    altitude_m: float
    position: object = None
    status: object = None
    land: object = None
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
    # v3 additions
    state: str = 'ACTIVE'           # ACTIVE / COMPLETE / FAILED
    reason: str = ''
    start_z: float = 0.0
    max_climb: float = 0.0
    flight_since: object = None     # monotonic time arm+Offboard was confirmed
    acks: dict = field(default_factory=dict)  # command -> (result, monotonic time)
    last_diag: float = 0.0


def classify_climb(flight_s, max_climb_m, check_s=CLIMB_CHECK_S, min_climb_m=MIN_CLIMB_M):
    """Pure helper (unit-testable): 'WAIT', 'OK' or 'NO_CLIMB'."""
    if max_climb_m >= min_climb_m:
        return 'OK'
    return 'NO_CLIMB' if flight_s > check_s else 'WAIT'


class ThreeX500(Node):
    def __init__(self, execute, isolate_failures=False):
        super().__init__('thermoswarm_three_x500_test')
        self.execute = execute
        self.isolate = isolate_failures
        self.aircraft = [Aircraft(1, 2, 5.0), Aircraft(2, 3, 8.0), Aircraft(3, 4, 11.0)]
        self.phase = 'WAIT'
        self.phase_since = time.monotonic()
        self.notice_at = 0.0
        self.heartbeat_count = 0
        self.last_tick = time.monotonic()
        self.audit_fresh_since = None
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
            # v3: command acknowledgements and land detector (both MESSAGE_VERSION 0)
            self.create_subscription(VehicleCommandAck,
                f'{prefix}/out/vehicle_command_ack',
                lambda msg, a=a: self.got_ack(a, msg), qos)
            self.create_subscription(VehicleLandDetected,
                f'{prefix}/out/vehicle_land_detected',
                lambda msg, a=a: setattr(a, 'land', msg), qos)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'Uc X500 telemetri/preflight kontrolu (v3). ' +
            ('--execute AKTIF: hazir olduklarinda ucus baslar.' if execute
             else 'Salt-okunur mod: HICBIR ucus komutu gonderilmeyecek.') +
            (' Ariza izolasyonu: AKTIF.' if (execute and isolate_failures) else ''))

    def got_position(self, a, msg):
        a.position = msg
        a.position_at = time.monotonic()

    def got_status(self, a, msg):
        a.status = msg
        a.status_at = time.monotonic()

    def got_ack(self, a, msg):
        # PX4 addresses the ack to our source_system (1); ignore unrelated acks.
        if msg.target_system not in (0, 1) or msg.command not in CMD_NAMES:
            return
        a.acks[msg.command] = (msg.result, time.monotonic())
        name = CMD_NAMES[msg.command]
        if msg.result in ACK_OK:
            self.get_logger().info(f'px4_{a.index} ACK {name}: result={msg.result} (kabul)')
        else:
            self.get_logger().error(f'px4_{a.index} ACK {name}: result={msg.result} (RED)')

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

    def diag(self, a, now, force=False):
        """v3: periodic target-vs-actual log for one vehicle."""
        if not force and now - a.last_diag < DIAG_PERIOD_S:
            return
        a.last_diag = now
        if a.position is None or a.status is None or not a.route:
            return
        label, (tx, ty, tz, _) = a.route[a.waypoint]
        p, s = a.position, a.status
        landed = None if a.land is None else bool(a.land.landed)
        self.get_logger().info(
            f'[DIAG px4_{a.index}] {a.state} wp={label} hedef=({tx:.2f},{ty:.2f},{tz:.2f}) '
            f'gercek=({p.x:.2f},{p.y:.2f},{p.z:.2f}) vz={p.vz:.2f} '
            f'xy_err={math.hypot(p.x - tx, p.y - ty):.2f} z_err={abs(p.z - tz):.2f} '
            f'tirmanis={a.start_z - p.z:.2f} m (max {a.max_climb:.2f}) '
            f'armed={s.arming_state == VehicleStatus.ARMING_STATE_ARMED} nav={s.nav_state} '
            f'landed={landed}')

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
            a.start_z, a.max_climb, a.flight_since = float(p.z), 0.0, None
            a.state, a.reason = 'ACTIVE', ''
            self.get_logger().info(f'px4_{a.index}: sysid={a.sysid} irtifa={a.altitude_m:.0f} m, 2m-kare rota')

    def armed(self, a):
        return a.status is not None and a.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def fail_vehicle(self, a, why, now):
        """v3: per-vehicle failure. Isolated -> land only this vehicle; else land all."""
        if a.state == 'FAILED':
            return
        a.state, a.reason = 'FAILED', why
        self.diag(a, now, force=True)
        self.get_logger().error(f'px4_{a.index} BASARISIZ: {why}')
        if self.isolate:
            self.aborted, self.reason = True, self.reason or f'px4_{a.index}: {why}'
            if self.armed(a):
                self.get_logger().warning(f'px4_{a.index}: yalnizca bu araca LAND gonderiliyor')
                self.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        else:
            self.request_landing(f'px4_{a.index} {why}', abort=True)

    def request_landing(self, why, abort=False):
        if self.phase in ('LAND', 'DONE', 'FAILED'):
            return
        self.get_logger().warning(f'TOPLU INIS: {why}')
        if abort:
            self.aborted, self.reason = True, self.reason or why
        for a in self.aircraft:
            if self.armed(a):
                self.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.change('LAND')

    def final_report(self):
        self.get_logger().info('--- ARAC BAZLI SONUC ---')
        for a in self.aircraft:
            done = a.waypoint + (1 if a.complete else 0)
            verdict = 'PASS' if a.state == 'COMPLETE' else 'FAIL'
            self.get_logger().info(
                f'px4_{a.index} (sysid {a.sysid}): {verdict} durum={a.state} '
                f'waypoint={done}/{len(a.route)} max_tirmanis={a.max_climb:.2f} m '
                f'hedef={a.altitude_m:.0f} m' + (f' neden="{a.reason}"' if a.reason else ''))

    def tick(self):
        now = time.monotonic()
        if self.phase in ('DONE', 'FAILED'):
            return
        if self.phase == 'WAIT':
            if now - self.notice_at > 4:
                self.report(now)
                self.notice_at = now
            if not self.execute:
                # DDS discovery can take a moment. Require all 3 position AND
                # status streams to remain fresh for at least one second.
                if all(self.fresh(a, now) for a in self.aircraft):
                    if self.audit_fresh_since is None:
                        self.audit_fresh_since = now
                    if now - self.audit_fresh_since >= 1.0:
                        self.get_logger().info('--- UC IHA TELEMETRI VE PREFLIGHT DENETIMI ---')
                        self.report(now)
                        if all(self.ready(a, now) for a in self.aircraft):
                            self.get_logger().info(
                                'AUDIT PASS: 3/3 sysid, konum, preflight ve failsafe kontrolleri uygun. '
                                'HICBIR ucus komutu gonderilmedi.')
                            self.change('DONE')
                            return
                        self.get_logger().error(
                            'AUDIT NOT READY: En az bir arac sistem kimligi, konum veya '
                            'preflight kontrolunu gecemedi. Ucus komutu gonderilmedi.')
                        self.change('FAILED')
                        return
                else:
                    self.audit_fresh_since = None
                if now - self.phase_since > 25:
                    self.get_logger().error(
                        'AUDIT FAILED: 25 sn icinde uc arac icin guncel konum ve '
                        'durum telemetrisi bir araya gelemedi. Ucus komutu gonderilmedi.')
                    self.report(now)
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
            for a in self.aircraft:
                if a.index in stale and a.state == 'ACTIVE':
                    a.state, a.reason = 'FAILED', 'telemetri kaybi'
            self.request_landing(f'telemetri kaybi: {stale}', abort=True)
            return

        if self.phase in ('WARMUP', 'FLIGHT'):
            for a in self.aircraft:
                if a.state != 'FAILED':
                    self.publish_setpoint(a)
                elif a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # Isolated failure: heartbeat only until PX4 leaves Offboard for AUTO_LAND.
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
            for a in self.aircraft:
                if a.state == 'FAILED':
                    # Isolated failure: make sure the vehicle really goes to AUTO_LAND.
                    if (self.armed(a)
                            and a.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                            and now - a.last_command > 3):
                        self.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    continue
                if a.status.failsafe:
                    self.fail_vehicle(a, 'PX4 failsafe', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
                armed = self.armed(a)
                offboard = a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                if offboard:
                    a.offboard_ever = True
                if a.offboard_ever and not offboard:
                    self.fail_vehicle(a, 'Offboard kaybetti', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
                if not (armed and offboard):
                    for code in (VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                 VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM):
                        ack = a.acks.get(code)
                        if ack is not None and ack[0] not in ACK_OK:
                            self.get_logger().warning(
                                f'px4_{a.index}: {CMD_NAMES[code]} reddedildi (result={ack[0]}); yeniden denenecek')
                    if now - self.phase_since > 20:
                        self.fail_vehicle(a, 'arm/offboard basarisiz', now)
                        if self.phase != 'FLIGHT':
                            return
                        continue
                    if now - a.last_command > 4:
                        self.command(a, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        if not armed:
                            self.command(a, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    continue

                if a.flight_since is None:
                    # v3: takeoff leg timer starts only after arm+Offboard is confirmed.
                    a.flight_since = now
                    a.leg_since = now
                    self.get_logger().info(f'px4_{a.index}: ARM + OFFBOARD dogrulandi (nav_state=OFFBOARD)')

                a.max_climb = max(a.max_climb, a.start_z - float(a.position.z))
                self.diag(a, now)
                if a.complete:
                    continue
                if a.waypoint == 0 and classify_climb(now - a.flight_since, a.max_climb) == 'NO_CLIMB':
                    self.fail_vehicle(
                        a, f'YUKSELME YOK: arm+Offboard {CLIMB_CHECK_S:.0f} sn dogrulandi fakat '
                        f'max tirmanis {a.max_climb:.2f} m < {MIN_CLIMB_M} m. Fiziksel engel / '
                        'Gazebo model cakismasi olasi (thermoswarm_gz_world_check.py)', now)
                    if self.phase != 'FLIGHT':
                        return
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
                            a.state = 'COMPLETE'
                            self.get_logger().info(f'px4_{a.index}: tum waypointler tamamlandi')
                        else:
                            a.waypoint += 1
                            a.leg_since = now
                        a.dwell_since = None
                else:
                    a.dwell_since = None
                if not a.complete and now-a.leg_since > LEG_TIMEOUT_S:
                    self.fail_vehicle(a, f'waypoint zaman asimi: {label}', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
            if all(a.state != 'ACTIVE' for a in self.aircraft):
                n_ok = sum(a.state == 'COMPLETE' for a in self.aircraft)
                self.request_landing(f'{n_ok}/3 rota tamamlandi', abort=n_ok < 3)
            elif now-self.phase_since > 180:
                for a in self.aircraft:
                    if a.state == 'ACTIVE':
                        a.state, a.reason = 'FAILED', 'genel ucus zaman asimi'
                self.request_landing('genel ucus zaman asimi', abort=True)
            return

        if self.phase == 'LAND':
            if all(a.status is not None and self.fresh(a, now)
                   and a.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                   for a in self.aircraft):
                self.final_report()
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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--execute', action='store_true', help='Gazebo SITL ortaminda uc arac icin otomatik ucusu etkinlestir')
    parser.add_argument('--isolate-failures', action='store_true',
                        help='Ariza yalnizca ilgili araci indirir; digerleri rotaya devam eder')
    parser.add_argument('--skip-gz-check', action='store_true',
                        help='Ucus oncesi Gazebo dunya denetimini atla (onerilmez)')
    args = parser.parse_args()
    if args.execute and not args.skip_gz_check:
        from thermoswarm_gz_world_check import check_world
        print('Ucus oncesi Gazebo dunya denetimi (salt-okunur)...')
        ok, problems = check_world()
        if not ok:
            for p in problems:
                print(f'GZ CHECK FAIL: {p}')
            print('Ucus BASLATILMADI. Gazebo/PX4 oturumunu temiz baslatip tekrar deneyin.')
            return 1
        print('GZ CHECK PASS.')
    rclpy.init()
    node = ThreeX500(execute=args.execute, isolate_failures=args.isolate_failures)
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
    return 0 if node.phase == 'DONE' else 1


if __name__ == '__main__':
    sys.exit(main())

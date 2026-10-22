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

import math
import time

from px4_msgs.msg import (
    VehicleStatus,
    VehicleCommand,
)
from .offboard import ACK_OK, CMD_NAMES
from .safety import CLIMB_CHECK_S, MIN_CLIMB_M, Safety, classify_climb
from .vehicle import create_fleet

LEG_TIMEOUT_S = 40.0
DIAG_PERIOD_S = 2.0

class Mission:
    """Own the mission phase machine and per-vehicle route progression."""

    def __init__(self, execute, isolate_failures=False, logger=None):
        self.execute = execute
        self.isolate = isolate_failures
        self.aircraft = create_fleet()
        self.offboard = None
        self.logger = logger
        self.phase = 'WAIT'
        self.phase_since = time.monotonic()
        self.notice_at = 0.0
        self.heartbeat_count = 0
        self.last_tick = time.monotonic()
        self.audit_fresh_since = None
        self.aborted = False
        self.reason = ''
        self.safety = Safety(self)

    def change(self, phase):
        self.phase = phase
        self.phase_since = time.monotonic()
        self.logger.info(f'ASAMA: {phase}')

    def report(self, now):
        for a in self.aircraft:
            if not self.safety.fresh(a, now):
                self.logger.warning(f'px4_{a.index}: telemetri yok / guncel degil')
                continue
            p, s = a.position, a.status
            self.logger.info(
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
        self.logger.info(
            f'[DIAG px4_{a.index}] {a.state} wp={label} hedef=({tx:.2f},{ty:.2f},{tz:.2f}) '
            f'gercek=({p.x:.2f},{p.y:.2f},{p.z:.2f}) vz={p.vz:.2f} '
            f'xy_err={math.hypot(p.x - tx, p.y - ty):.2f} z_err={abs(p.z - tz):.2f} '
            f'tirmanis={a.start_z - p.z:.2f} m (max {a.max_climb:.2f}) '
            f'armed={s.arming_state == VehicleStatus.ARMING_STATE_ARMED} nav={s.nav_state} '
            f'landed={landed}')

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
            self.logger.info(f'px4_{a.index}: sysid={a.sysid} irtifa={a.altitude_m:.0f} m, 2m-kare rota')

    def final_report(self):
        self.logger.info('--- ARAC BAZLI SONUC ---')
        for a in self.aircraft:
            done = a.waypoint + (1 if a.complete else 0)
            verdict = 'PASS' if a.state == 'COMPLETE' else 'FAIL'
            self.logger.info(
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
                if all(self.safety.fresh(a, now) for a in self.aircraft):
                    if self.audit_fresh_since is None:
                        self.audit_fresh_since = now
                    if now - self.audit_fresh_since >= 1.0:
                        self.logger.info('--- UC IHA TELEMETRI VE PREFLIGHT DENETIMI ---')
                        self.report(now)
                        if all(self.safety.ready(a, now) for a in self.aircraft):
                            self.logger.info(
                                'AUDIT PASS: 3/3 sysid, konum, preflight ve failsafe kontrolleri uygun. '
                                'HICBIR ucus komutu gonderilmedi.')
                            self.change('DONE')
                            return
                        self.logger.error(
                            'AUDIT NOT READY: En az bir arac sistem kimligi, konum veya '
                            'preflight kontrolunu gecemedi. Ucus komutu gonderilmedi.')
                        self.change('FAILED')
                        return
                else:
                    self.audit_fresh_since = None
                if now - self.phase_since > 25:
                    self.logger.error(
                        'AUDIT FAILED: 25 sn icinde uc arac icin guncel konum ve '
                        'durum telemetrisi bir araya gelemedi. Ucus komutu gonderilmedi.')
                    self.report(now)
                    self.change('FAILED')
                return
            if all(self.safety.ready(a, now) for a in self.aircraft):
                self.prepare(now)
                self.change('WARMUP')
                return
            if now - self.phase_since > 90:
                self.logger.error('90 sn: uc arac ayni anda preflight READY degil. Ucus baslatilmadi.')
                self.change('FAILED')
            return

        stale = [a.index for a in self.aircraft if not self.safety.fresh(a, now)]
        if stale and self.phase in ('WARMUP', 'FLIGHT'):
            for a in self.aircraft:
                if a.index in stale and a.state == 'ACTIVE':
                    a.state, a.reason = 'FAILED', 'telemetri kaybi'
            self.safety.request_landing(f'telemetri kaybi: {stale}', abort=True)
            return

        if self.phase in ('WARMUP', 'FLIGHT'):
            for a in self.aircraft:
                if a.state != 'FAILED':
                    self.offboard.publish_setpoint(a)
                elif a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # Isolated failure: heartbeat only until PX4 leaves Offboard for AUTO_LAND.
                    self.offboard.publish_setpoint(a)
        elif self.phase == 'LAND':
            # Keep heartbeat while PX4 still says Offboard, without overriding AUTO_LAND.
            for a in self.aircraft:
                if (self.safety.fresh(a, now) and
                    a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                    self.offboard.publish_setpoint(a)

        if self.phase == 'WARMUP':
            self.heartbeat_count += 1
            if self.heartbeat_count >= 20:
                if not all(self.safety.ready(a, now) for a in self.aircraft):
                    self.safety.request_landing('WARMUP sonunda preflight bozuldu', abort=True)
                    return
                for a in self.aircraft:
                    self.offboard.command(a, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.offboard.command(a, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.change('FLIGHT')
            return

        if self.phase == 'FLIGHT':
            for a in self.aircraft:
                if a.state == 'FAILED':
                    # Isolated failure: make sure the vehicle really goes to AUTO_LAND.
                    if (self.safety.armed(a)
                            and a.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                            and now - a.last_command > 3):
                        self.offboard.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    continue
                if a.status.failsafe:
                    self.safety.fail_vehicle(a, 'PX4 failsafe', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
                armed = self.safety.armed(a)
                offboard = a.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                if offboard:
                    a.offboard_ever = True
                if a.offboard_ever and not offboard:
                    self.safety.fail_vehicle(a, 'Offboard kaybetti', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
                if not (armed and offboard):
                    for code in (VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                 VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM):
                        ack = a.acks.get(code)
                        if ack is not None and ack[0] not in ACK_OK:
                            self.logger.warning(
                                f'px4_{a.index}: {CMD_NAMES[code]} reddedildi (result={ack[0]}); yeniden denenecek')
                    if now - self.phase_since > 20:
                        self.safety.fail_vehicle(a, 'arm/offboard basarisiz', now)
                        if self.phase != 'FLIGHT':
                            return
                        continue
                    if now - a.last_command > 4:
                        self.offboard.command(a, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        if not armed:
                            self.offboard.command(a, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    continue

                if a.flight_since is None:
                    # v3: takeoff leg timer starts only after arm+Offboard is confirmed.
                    a.flight_since = now
                    a.leg_since = now
                    self.logger.info(f'px4_{a.index}: ARM + OFFBOARD dogrulandi (nav_state=OFFBOARD)')

                a.max_climb = max(a.max_climb, a.start_z - float(a.position.z))
                self.diag(a, now)
                if a.complete:
                    continue
                if a.waypoint == 0 and classify_climb(now - a.flight_since, a.max_climb) == 'NO_CLIMB':
                    self.safety.fail_vehicle(
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
                        self.logger.info(
                            f'px4_{a.index} ULASILDI {label}: xy={xy_error:.2f} m, z={z_error:.2f} m')
                        if a.waypoint + 1 == len(a.route):
                            a.complete = True
                            a.state = 'COMPLETE'
                            self.logger.info(f'px4_{a.index}: tum waypointler tamamlandi')
                        else:
                            a.waypoint += 1
                            a.leg_since = now
                        a.dwell_since = None
                else:
                    a.dwell_since = None
                if not a.complete and now-a.leg_since > LEG_TIMEOUT_S:
                    self.safety.fail_vehicle(a, f'waypoint zaman asimi: {label}', now)
                    if self.phase != 'FLIGHT':
                        return
                    continue
            if all(a.state != 'ACTIVE' for a in self.aircraft):
                n_ok = sum(a.state == 'COMPLETE' for a in self.aircraft)
                self.safety.request_landing(f'{n_ok}/3 rota tamamlandi', abort=n_ok < 3)
            elif now-self.phase_since > 180:
                for a in self.aircraft:
                    if a.state == 'ACTIVE':
                        a.state, a.reason = 'FAILED', 'genel ucus zaman asimi'
                self.safety.request_landing('genel ucus zaman asimi', abort=True)
            return

        if self.phase == 'LAND':
            if all(a.status is not None and self.safety.fresh(a, now)
                   and a.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                   for a in self.aircraft):
                self.final_report()
                if self.aborted:
                    self.logger.error(f'Ucus iptal edildi, tum araclar disarm: {self.reason}')
                    self.change('FAILED')
                else:
                    self.logger.info('BASARILI: 3/3 waypoint rotasi, inis ve disarm dogrulandi.')
                    self.change('DONE')
                return
            for a in self.aircraft:
                if (a.status is not None
                    and a.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and a.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                    and now-a.last_command > 3):
                    self.offboard.command(a, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if now-self.phase_since > 65 and now-self.notice_at > 8:
                self.logger.error(
                    'Inis teyit edilemiyor: pxh> commander land her aktif arac icin kontrol edilmeli.')
                self.notice_at = now


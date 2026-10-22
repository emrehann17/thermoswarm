"""Preflight, telemetry freshness, and safe-failure policies."""

from px4_msgs.msg import VehicleCommand, VehicleStatus


CLIMB_CHECK_S = 10.0      # time after confirmed arm+Offboard to show real climb
MIN_CLIMB_M = 0.5         # minimum climb expected within CLIMB_CHECK_S


def classify_climb(flight_s, max_climb_m, check_s=CLIMB_CHECK_S, min_climb_m=MIN_CLIMB_M):
    """Return WAIT, OK, or NO_CLIMB for the current confirmed flight time."""
    if max_climb_m >= min_climb_m:
        return 'OK'
    return 'NO_CLIMB' if flight_s > check_s else 'WAIT'


class Safety:
    """Apply safety rules to a mission without owning its phase machine."""

    def __init__(self, mission):
        self.mission = mission

    def fresh(self, aircraft, now):
        return (
            aircraft.position is not None
            and aircraft.status is not None
            and now - aircraft.position_at < 2.0
            and now - aircraft.status_at < 2.0
        )

    def correct_sysid(self, aircraft):
        return aircraft.status is not None and aircraft.status.system_id == aircraft.sysid

    def ready(self, aircraft, now):
        return (
            self.fresh(aircraft, now)
            and self.correct_sysid(aircraft)
            and aircraft.position.xy_valid
            and aircraft.position.z_valid
            and aircraft.status.pre_flight_checks_pass
            and not aircraft.status.failsafe
            and aircraft.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        )

    def armed(self, aircraft):
        return (
            aircraft.status is not None
            and aircraft.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def fail_vehicle(self, aircraft, why, now):
        """Fail one aircraft, isolating it only when configured to do so."""
        mission = self.mission
        if aircraft.state == 'FAILED':
            return
        aircraft.state, aircraft.reason = 'FAILED', why
        mission.diag(aircraft, now, force=True)
        mission.logger.error(f'px4_{aircraft.index} BASARISIZ: {why}')
        if mission.isolate:
            mission.aborted = True
            mission.reason = mission.reason or f'px4_{aircraft.index}: {why}'
            if self.armed(aircraft):
                mission.logger.warning(f'px4_{aircraft.index}: yalnizca bu araca LAND gonderiliyor')
                mission.offboard.command(aircraft, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        else:
            self.request_landing(f'px4_{aircraft.index} {why}', abort=True)

    def request_landing(self, why, abort=False):
        """Request conservative fleet landing and transition the mission to LAND."""
        mission = self.mission
        if mission.phase in ('LAND', 'DONE', 'FAILED'):
            return
        mission.logger.warning(f'TOPLU INIS: {why}')
        if abort:
            mission.aborted = True
            mission.reason = mission.reason or why
        for aircraft in mission.aircraft:
            if self.armed(aircraft):
                mission.offboard.command(aircraft, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        mission.change('LAND')

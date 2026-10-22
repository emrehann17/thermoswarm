"""Per-aircraft telemetry and mission state."""

from dataclasses import dataclass, field


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
    state: str = 'ACTIVE'           # ACTIVE / COMPLETE / FAILED
    reason: str = ''
    start_z: float = 0.0
    max_climb: float = 0.0
    flight_since: object = None     # monotonic time arm+Offboard was confirmed
    acks: dict = field(default_factory=dict)  # command -> (result, monotonic time)
    last_diag: float = 0.0


def create_fleet():
    """Return the three configured vehicles in index order."""
    return [Aircraft(1, 2, 5.0), Aircraft(2, 3, 8.0), Aircraft(3, 4, 11.0)]

"""Per-aircraft telemetry and mission state."""

from dataclasses import dataclass, field

from .config import load_config


@dataclass
class Aircraft:
    index: int
    sysid: int
    altitude_m: float
    namespace: str = ''
    px4_instance: int = 0
    model: str = 'x500'
    spawn_xy_m: tuple = (0.0, 0.0)
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


def create_fleet(config=None):
    """Build vehicle state from validated YAML, in configured PX4 instance order."""
    config = config or load_config()
    return [
        Aircraft(
            index=vehicle.px4_instance,
            sysid=vehicle.system_id,
            altitude_m=vehicle.takeoff_altitude_m,
            namespace=vehicle.namespace,
            px4_instance=vehicle.px4_instance,
            model=vehicle.model,
            spawn_xy_m=vehicle.spawn_xy_m,
        )
        for vehicle in config.vehicles
    ]

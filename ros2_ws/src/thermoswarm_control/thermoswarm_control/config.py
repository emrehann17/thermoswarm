"""Load and validate the canonical ThermoSwarm fleet YAML."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


class ConfigError(ValueError):
    """Raised when a fleet file is missing or does not match schema v1."""


@dataclass(frozen=True)
class VehicleConfig:
    name: str
    px4_instance: int
    system_id: int
    namespace: str
    model: str
    spawn_xy_m: tuple
    takeoff_altitude_m: float


@dataclass(frozen=True)
class RoutePoint:
    name: str
    north_m: float
    east_m: float


@dataclass(frozen=True)
class SafetyConfig:
    telemetry_freshness_s: float
    audit_fresh_duration_s: float
    audit_timeout_s: float
    readiness_timeout_s: float
    offboard_warmup_heartbeats: int
    climb_check_s: float
    min_climb_m: float
    waypoint_xy_tolerance_m: float
    waypoint_z_tolerance_m: float
    waypoint_dwell_s: float
    leg_timeout_s: float
    mission_timeout_s: float


@dataclass(frozen=True)
class FleetConfig:
    schema_version: int
    vehicles: tuple
    route: tuple
    safety: SafetyConfig
    source: Path


def _mapping(value, where, required):
    if not isinstance(value, dict):
        raise ConfigError(f"{where} must be a YAML mapping")
    missing = required - value.keys()
    extra = value.keys() - required
    if missing:
        raise ConfigError(f"{where} missing required field(s): {', '.join(sorted(missing))}")
    if extra:
        raise ConfigError(f"{where} has unsupported field(s): {', '.join(sorted(extra))}")
    return value


def _integer(value, where, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ConfigError(f"{where} must be an integer >= {minimum}")
    return value


def _number(value, where, minimum=None, strictly_positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{where} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{where} must be finite")
    if minimum is not None and (result <= minimum if strictly_positive else result < minimum):
        op = ">" if strictly_positive else ">="
        raise ConfigError(f"{where} must be {op} {minimum}")
    return result


def default_config_path():
    """Find the installed package resource, with source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("thermoswarm_control")) / "config" / "fleet.yaml"
        if installed.is_file():
            return installed
    except (ImportError, LookupError):
        pass
    packaged = Path(__file__).resolve().parent / "config" / "fleet.yaml"
    if packaged.is_file():
        return packaged
    # Source checkout: config/fleet.yaml remains the one editable source.
    return Path(__file__).resolve().parents[4] / "config" / "fleet.yaml"


def load_config(path=None):
    """Load a validated schema-v1 fleet; only the verified three-vehicle size is supported."""
    source = Path(path).expanduser().resolve() if path else default_config_path().resolve()
    try:
        with source.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as exc:
        raise ConfigError(f"cannot read fleet config '{source}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in fleet config '{source}': {exc}") from exc

    root = _mapping(raw, "fleet config", {"schema_version", "vehicles", "mission"})
    version = _integer(root["schema_version"], "schema_version", 1)
    if version != 1:
        raise ConfigError(f"unsupported schema_version {version}; supported version is 1")
    if not isinstance(root["vehicles"], list):
        raise ConfigError("vehicles must be a YAML list")
    if len(root["vehicles"]) != 3:
        raise ConfigError(f"unsupported fleet size {len(root['vehicles'])}; exactly 3 vehicles are supported")

    vehicles = []
    for index, value in enumerate(root["vehicles"], 1):
        where = f"vehicles[{index - 1}]"
        item = _mapping(value, where, {
            "name", "px4_instance", "system_id", "namespace", "model",
            "spawn_xy_m", "takeoff_altitude_m",
        })
        for key in ("name", "namespace", "model"):
            if not isinstance(item[key], str) or not item[key].strip():
                raise ConfigError(f"{where}.{key} must be a non-empty string")
        if not item["namespace"].startswith("/") or item["namespace"] == "/" or item["namespace"].endswith("/"):
            raise ConfigError(f"{where}.namespace must be an absolute ROS namespace without a trailing slash")
        if item["model"] != "x500":
            raise ConfigError(f"{where}.model '{item['model']}' is unsupported; only 'x500' is supported")
        spawn = item["spawn_xy_m"]
        if not isinstance(spawn, list) or len(spawn) != 2:
            raise ConfigError(f"{where}.spawn_xy_m must contain exactly [x, y]")
        vehicles.append(VehicleConfig(
            name=item["name"],
            px4_instance=_integer(item["px4_instance"], f"{where}.px4_instance", 1),
            system_id=_integer(item["system_id"], f"{where}.system_id", 1),
            namespace=item["namespace"],
            model=item["model"],
            spawn_xy_m=tuple(_number(v, f"{where}.spawn_xy_m") for v in spawn),
            takeoff_altitude_m=_number(item["takeoff_altitude_m"], f"{where}.takeoff_altitude_m", 0),
        ))
    for field in ("name", "px4_instance", "system_id", "namespace"):
        values = [getattr(vehicle, field) for vehicle in vehicles]
        if len(set(values)) != len(values):
            raise ConfigError(f"vehicles contain repeated {field}")
    for vehicle in vehicles:
        if vehicle.system_id > 255:
            raise ConfigError(f"vehicle {vehicle.name}.system_id must be <= 255")

    mission = _mapping(root["mission"], "mission", {"route", "safety"})
    if not isinstance(mission["route"], list) or not mission["route"]:
        raise ConfigError("mission.route must be a non-empty YAML list")
    route = []
    for index, value in enumerate(mission["route"]):
        where = f"mission.route[{index}]"
        item = _mapping(value, where, {"name", "north_m", "east_m"})
        if not isinstance(item["name"], str) or not item["name"].strip():
            raise ConfigError(f"{where}.name must be a non-empty string")
        route.append(RoutePoint(
            item["name"], _number(item["north_m"], f"{where}.north_m"),
            _number(item["east_m"], f"{where}.east_m"),
        ))
    if len({point.name for point in route}) != len(route):
        raise ConfigError("mission.route contains repeated waypoint names")

    safety_values = _mapping(mission["safety"], "mission.safety", {
        "telemetry_freshness_s", "audit_fresh_duration_s", "audit_timeout_s",
        "readiness_timeout_s", "offboard_warmup_heartbeats", "climb_check_s",
        "min_climb_m", "waypoint_xy_tolerance_m", "waypoint_z_tolerance_m",
        "waypoint_dwell_s", "leg_timeout_s", "mission_timeout_s",
    })
    positive = {
        "telemetry_freshness_s", "audit_fresh_duration_s", "audit_timeout_s",
        "readiness_timeout_s", "climb_check_s", "waypoint_xy_tolerance_m",
        "waypoint_z_tolerance_m", "waypoint_dwell_s", "leg_timeout_s", "mission_timeout_s",
    }
    parsed_safety = {}
    for key, value in safety_values.items():
        parsed_safety[key] = (
            _integer(value, f"mission.safety.{key}", 1)
            if key == "offboard_warmup_heartbeats"
            else _number(value, f"mission.safety.{key}", 0,
                         strictly_positive=key in positive)
        )

    return FleetConfig(version, tuple(vehicles), tuple(route), SafetyConfig(**parsed_safety), source)

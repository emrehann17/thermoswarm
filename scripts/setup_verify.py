"""Read-only installed ROS package check. Never creates a ROS node."""

import sys
from pathlib import Path


def main():
    root = Path(sys.argv[1]).resolve()
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy  # noqa: F401
    import px4_msgs
    from px4_msgs.msg import VehicleStatus  # noqa: F401
    import thermoswarm_control
    from thermoswarm_control.config import load_config

    for name, module in (("px4_msgs", px4_msgs), ("thermoswarm_control", thermoswarm_control)):
        prefix = Path(get_package_prefix(name)).resolve()
        if not prefix.is_relative_to(root / "ros2_ws/install"):
            raise RuntimeError(f"{name}: wrong overlay: {prefix}")
        # ament_python --symlink-install imports through an installed .egg-link
        # into ros2_ws/build. Accept that standard layout, never a src fallback.
        origin = Path(module.__file__).absolute()
        if not any(origin.is_relative_to(base) for base in (
            root / "ros2_ws/install", root / "ros2_ws/build" / name,
        )):
            raise RuntimeError(f"{name}: imported outside installed workspace: {module.__file__}")
        print(f"PASS installed {name}: {prefix}")
    fleet_path = Path(get_package_share_directory("thermoswarm_control")) / "config/fleet.yaml"
    fleet = load_config(fleet_path)
    if fleet_path.read_bytes() != (root / "config/fleet.yaml").read_bytes():
        raise RuntimeError("installed fleet.yaml differs from config/fleet.yaml; rebuild workspace")
    print(f"PASS installed fleet.yaml: {fleet_path} ({len(fleet.vehicles)} vehicles)")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"FAIL installed ROS packages/fleet: {exc}", file=sys.stderr)
        sys.exit(1)

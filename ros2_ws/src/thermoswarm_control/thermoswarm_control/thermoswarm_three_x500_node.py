"""Backward-compatible import and console-script shim."""

import sys

import rclpy  # Re-exported for callers that historically patched this module.

from .mission import DIAG_PERIOD_S, LEG_TIMEOUT_S, Mission
from .mission_node import MissionNode, main
from .safety import CLIMB_CHECK_S, MIN_CLIMB_M, classify_climb
from .vehicle import Aircraft


# Preserve the old class name and CLI target while the implementation lives in
# the responsibility-specific modules.
ThreeX500 = MissionNode


if __name__ == '__main__':
    sys.exit(main())

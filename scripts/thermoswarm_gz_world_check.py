#!/usr/bin/env python3
"""CLI compatibility shim: the ROS package owns the single world safety policy."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/thermoswarm_control'))
from thermoswarm_control import thermoswarm_gz_world_check as implementation  # noqa: E402

if __name__ == '__main__':
    sys.exit(implementation.main())
else:
    # Legacy callers and mocks resolve to the same module used by ROS preflight.
    sys.modules[__name__] = implementation

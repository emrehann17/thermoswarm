#!/usr/bin/env python3
"""Stable process-group leader for Gazebo, which rewrites its own argv at startup.

The launcher signals the entire isolated group. Keep this supervisor alive until
its child exits, so PID/start-time/argv ownership checks remain valid throughout.
"""
import signal
import subprocess
import sys

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda *_: None)
    signal.signal(signal.SIGTERM, lambda *_: None)
    child = subprocess.Popen(sys.argv[1:])
    raise SystemExit(child.wait())

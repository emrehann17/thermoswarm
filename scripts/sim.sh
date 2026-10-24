#!/usr/bin/env bash
# Managed lifecycle entry point for the verified three-X500 PX4 SITL setup.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
exec python3 "${PROJECT_ROOT}/scripts/sim_launcher.py" "$@"

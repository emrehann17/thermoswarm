#!/usr/bin/env bash
# ThermoSwarm pre-flight diagnostics. Read-only: never arm or start an aircraft.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

log() { printf '[thermoswarm-check] %s\n' "$*"; }
fail() { printf '[thermoswarm-check] FAIL: %s\n' "$*" >&2; exit 1; }

[[ -f scripts/thermoswarm_gz_world_check.py ]] || fail 'Gazebo denetim betiği bulunamadı.'
[[ -f scripts/thermoswarm_three_x500_v3.py ]] || fail 'Uçuş denetim betiği bulunamadı.'
[[ -f /opt/ros/jazzy/setup.bash ]] || fail 'ROS 2 Jazzy setup bulunamadı.'
[[ -f ros2_ws/install/setup.bash ]] || fail 'ROS 2 workspace henüz build edilmemiş.'

# Keep ROS environment explicit; an IDE interpreter alone does not set DDS/ROS runtime paths.
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source ros2_ws/install/setup.bash
export GZ_IP="${GZ_IP:-127.0.0.1}"

PYTHON_BIN="${THERMOSWARM_PYTHON:-python3}"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || fail "Python bulunamadı: $PYTHON_BIN"
command -v timeout >/dev/null 2>&1 || fail 'timeout komutu bulunamadı.'

tmp_file="$(mktemp)"
trap 'rm -f "$tmp_file"' EXIT

log '1/2: Gazebo model ve çakışma denetimi (salt-okunur)'
if ! timeout 60s "$PYTHON_BIN" scripts/thermoswarm_gz_world_check.py 2>&1 | tee "$tmp_file"; then
  fail 'Gazebo denetim komutu hata verdi veya 60 saniyede bitmedi.'
fi
if ! grep -q 'GZ CHECK PASS' "$tmp_file" || grep -q 'GZ CHECK FAIL' "$tmp_file"; then
  fail 'Gazebo model kontrolü PASS vermedi; uçuşu BAŞLATMA.'
fi

: > "$tmp_file"
log '2/2: Üç X500 telemetri/preflight denetimi (salt-okunur)'
# No --execute flag: this is strictly the audit mode.
if ! timeout 90s "$PYTHON_BIN" scripts/thermoswarm_three_x500_v3.py 2>&1 | tee "$tmp_file"; then
  fail 'Telemetri denetim komutu hata verdi veya 90 saniyede bitmedi.'
fi
if ! grep -q 'AUDIT PASS' "$tmp_file" || grep -q 'AUDIT FAIL' "$tmp_file"; then
  fail '3/3 AUDIT PASS alınamadı; uçuşu BAŞLATMA.'
fi

log 'CHECK PASS: GZ CHECK ve AUDIT başarılı. Uçuş komutu GÖNDERİLMEDİ.'

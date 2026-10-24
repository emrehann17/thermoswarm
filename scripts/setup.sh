#!/usr/bin/env bash
# PATH changes in ros_check/build_workspace are intentionally isolated subshells.
# shellcheck disable=SC2030,SC2031
# Ubuntu 24.04 / WSL2 bootstrap. Default and --check never install or launch SITL.
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1 GIT_OPTIONAL_LOCKS=0 LC_ALL=C.UTF-8
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/setup-versions.env"
PX4="$ROOT/PX4-Autopilot"
MSGS="$ROOT/ros2_ws/src/px4_msgs"
AGENT="$ROOT/Micro-XRCE-DDS-Agent"
STATE="$ROOT/.runtime/setup"
PREFIX="$STATE/agent/install"
JOBS=2
MODE=plan
YES=0
ALLOW_SYSTEM=0

usage() {
    cat <<'HELP'
Usage: scripts/setup.sh [--plan | --check | --install] [--yes] [--allow-system] [--jobs N]
  no arguments / --plan  Read-only inventory and proposed changes; no network/sudo.
  --check               Read-only installed stack + source-pin checks; nonzero on failure.
  --install             Show plan, ask permission, then install missing components.
  --yes                 Consent to displayed LOCAL changes without a prompt.
  --allow-system        Permit sudo, apt metadata/repository additions and missing packages.
                        apt still asks before each package transaction. Existing package
                        upgrades/removals are rejected; resolve those manually, then rerun.
  --jobs N              Build parallelism (default 2, useful for WSL memory limits).
  -h, --help            Show this help.

Never run this script with sudo. Existing checkouts are never reset, pulled or switched.
Wrong revisions / local source edits block installation. Existing PX4 binaries, parameters,
ROS venv and system Agent are retained. No shell startup files are edited. No flight starts.
After installation: source scripts/env.sh
HELP
}
fail() { echo "ERROR: $*" >&2; exit 1; }
forbidden_mode=0
while (( $# )); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --plan|--check|--install)
            (( forbidden_mode == 0 )) || fail 'Choose only one mode.'
            MODE="${1#--}"; forbidden_mode=1 ;;
        --yes) YES=1 ;;
        --allow-system) ALLOW_SYSTEM=1 ;;
        --jobs) shift; [[ ${1:-} =~ ^[1-9][0-9]*$ ]] || fail '--jobs needs a positive integer.'; JOBS="$1" ;;
        *) fail "Unknown option: $1 (see --help)" ;;
    esac
    shift
done

platform_ok() {
    # shellcheck source=/dev/null
    source /etc/os-release
    [[ $ID == ubuntu && $VERSION_ID == 24.04 ]] &&
        [[ $(uname -m) == x86_64 || $(uname -m) == aarch64 ]]
}
installed() { [[ $(dpkg-query -W -f='${Status}' "$1" 2>/dev/null) == 'install ok installed' ]]; }
PACKAGES=(ca-certificates curl gnupg git procps util-linux build-essential cmake ninja-build ccache
    pkg-config python3-dev python3-venv python3-pip python3-setuptools python3-wheel
    python3-yaml python3-pytest python3-colcon-common-extensions python3-empy
    ros-jazzy-ros-base ros-jazzy-ament-cmake ros-jazzy-rosidl-default-generators
    gz-harmonic libgz-sim8-dev libeigen3-dev libssl-dev libtinyxml2-dev
    libxml2-dev libxml2-utils libunwind-dev cppzmq-dev protobuf-compiler
    libprotobuf-dev libopencv-dev libgstreamer-plugins-base1.0-dev
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good bc unzip zip rsync
    shellcheck)
MISSING=()
missing_packages() {
    MISSING=()
    local package
    for package in "${PACKAGES[@]}"; do installed "$package" || MISSING+=("$package"); done
}
repo_ok() {
    local dir="$1" ref="$2" top status subs
    [[ -d "$dir" ]] || { echo "MISSING source: $dir"; return 1; }
    top=$(git -C "$dir" rev-parse --show-toplevel 2>/dev/null) || return 1
    [[ $(realpath "$dir") == "$top" ]] || { echo "FAIL not an independent checkout: $dir"; return 1; }
    [[ $(git -C "$dir" rev-parse HEAD) == "$ref" ]] || { echo "FAIL revision: $dir (expected $ref); checkout preserved"; return 1; }
    status=$(git -C "$dir" status --porcelain --untracked-files=normal) || return 1
    [[ -z $status ]] || { echo "FAIL local edits/untracked source: $dir; preserved"; return 1; }
    subs=$(git -C "$dir" submodule status --recursive) || return 1
    if [[ -n $subs ]] && printf '%s\n' "$subs" | grep -q '^[+U-]'; then
        echo "FAIL missing/mismatched submodules: $dir; preserved"
        return 1
    fi
    echo "PASS source: $dir @ $ref"
}
agent_command() {
    if [[ -x "$STATE/bin/MicroXRCEAgent" ]]; then
        printf '%s\n' "$STATE/bin/MicroXRCEAgent"
    else
        command -v MicroXRCEAgent
    fi
}
agent_ok() {
    local exe lib output
    exe=$(agent_command) || return 1
    if [[ $exe == "$STATE/bin/MicroXRCEAgent" ]]; then
        exe="$PREFIX/bin/MicroXRCEAgent"
        output=$(LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ldd "$exe") || return 1
    else
        output=$(ldd "$exe") || return 1
    fi
    [[ $output != *'not found'* ]] || return 1
    lib=$(awk '/libmicroxrcedds_agent.so/ {print $3}' <<< "$output")
    [[ -n $lib && $(readlink -f "$lib") == *.so.2.4.3 ]] || return 1
    echo "PASS Agent v2.4.3 shared library: $(readlink -f "$lib")"
}
ros_check() (
    set +u
    # No daemon/node is started by --help, package prefix lookup or imports.
    unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO
    export PATH=/usr/local/bin:/usr/bin:/bin
    [[ -r /opt/ros/jazzy/setup.bash ]] || exit 1
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash || exit 1
    ros2 --help >/dev/null || exit 1
    echo 'PASS ROS 2 Jazzy CLI'
    [[ -r "$ROOT/ros2_ws/install/local_setup.bash" ]] || exit 1
    # shellcheck source=/dev/null
    source "$ROOT/ros2_ws/install/local_setup.bash" || exit 1
    cd /tmp
    /usr/bin/python3 -B "$ROOT/scripts/setup_verify.py" "$ROOT"
)
venv_ok() {
    [[ -x "$ROOT/.venv-ros/bin/python" ]] &&
        grep -q '^include-system-site-packages = true$' "$ROOT/.venv-ros/pyvenv.cfg" &&
        "$ROOT/.venv-ros/bin/python" -B -c 'import sys, yaml; assert sys.version_info[:2] == (3,12)'
}
check_all() {
    local bad=0
    if platform_ok; then echo 'PASS Ubuntu 24.04 platform'; else echo 'FAIL requires Ubuntu 24.04 amd64/arm64'; bad=1; fi
    if command -v git >/dev/null; then
        repo_ok "$PX4" "$PX4_REF" || bad=1
        repo_ok "$MSGS" "$MSGS_REF" || bad=1
        repo_ok "$AGENT" "$AGENT_REF" || bad=1
    else echo 'FAIL missing git'; bad=1; fi
    if command -v gz >/dev/null && gz sim --versions | grep -q '^8\.'; then
        echo 'PASS Gazebo Harmonic (gz-sim 8)'
    else echo 'FAIL Gazebo Harmonic'; bad=1; fi
    if [[ -x "$PX4/build/px4_sitl_default/bin/px4" ]]; then echo 'PASS PX4 SITL binary exists (not executed)'; else echo 'FAIL PX4 SITL binary'; bad=1; fi
    agent_ok || { echo 'FAIL Agent missing, unresolved libraries or version not provable as 2.4.3'; bad=1; }
    ros_check || { echo 'FAIL ROS workspace; see diagnostics above'; bad=1; }
    if venv_ok; then echo 'PASS .venv-ros Python 3.12 + system packages'; else echo 'FAIL .venv-ros'; bad=1; fi
    missing_packages
    if (( ${#MISSING[@]} )); then
        echo "FAIL missing build prerequisites: ${MISSING[*]}"; bad=1
    fi
    echo 'Read-only check finished; no simulator, ROS node or flight was started.'
    return "$bad"
}
plan() {
    echo "ThermoSwarm setup plan ($ROOT)"
    echo 'Target: Ubuntu 24.04 / ROS Jazzy / Gazebo Harmonic / PX4 1.17.0 / Agent 2.4.3'
    platform_ok || echo 'BLOCKED: requires Ubuntu 24.04 amd64/arm64.'
    missing_packages
    if (( ${#MISSING[@]} )); then
        echo "SYSTEM, explicit --allow-system required: missing apt packages: ${MISSING[*]}"
        echo 'May add Universe, ROS apt-source 1.3.0 and OSRF Harmonic repositories; sudo apt-get update.'
        echo 'Every apt transaction is simulated; installed package upgrades/removals cause a stop.'
    else echo 'KEEP all required apt packages (no apt update/install).'; fi
    local dir ref
    while read -r dir ref; do
        if [[ -e "$ROOT/$dir" ]]; then
            if command -v git >/dev/null; then repo_ok "$ROOT/$dir" "$ref" || true; fi
            echo "KEEP existing checkout: $dir; wrong revisions/edits block --install."
        else echo "CREATE source: $dir @ $ref (network, recursive pinned submodules)."; fi
    done <<PINS
PX4-Autopilot $PX4_REF
Micro-XRCE-DDS-Agent $AGENT_REF
ros2_ws/src/px4_msgs $MSGS_REF
PINS
    if [[ -x "$PX4/build/px4_sitl_default/bin/px4" ]]; then echo 'KEEP PX4 binary and local rootfs/parameters.'
    else
        echo 'BUILD PX4: isolated .venv-px4, Tools/setup/requirements.txt; make px4_sitl_default (no simulation).'
        echo "Python version constraints: $ROOT/scripts/requirements-px4-constraints.txt"
    fi
    if agent_ok; then echo 'KEEP existing Agent.'
    else echo "BUILD missing Agent and pinned Fast-CDR/Fast-DDS/memory/spdlog into $PREFIX; no sudo/ldconfig. Existing incompatible Agent blocks installation."; fi
    if [[ -d "$ROOT/.venv-ros" ]]; then echo 'KEEP .venv-ros (incompatible venv blocks installation).'
    else echo 'CREATE .venv-ros with /usr/bin/python3 --system-site-packages; no runtime pip installs.'; fi
    echo 'BUILD ros2_ws px4_msgs + thermoswarm_control if installed content/source fingerprint differs.'
    echo 'Existing unrecorded workspace is rebuilt once; later matching builds are skipped.'
    echo 'WRITE local build fingerprints and Agent wrapper under .runtime/setup; source scripts/env.sh manually.'
    echo 'FINISH with --check equivalent, without starting SITL/flight. No .bashrc edits, git resets or automatic upgrades.'
}
confirm() {
    local answer
    (( YES )) && return 0
    [[ -t 0 ]] || fail 'No interactive terminal; inspect --plan, then use --install --yes for local changes.'
    read -r -p 'Apply the displayed local setup plan? [y/N] ' answer
    [[ $answer == y || $answer == Y ]] || fail 'Cancelled; nothing changed.'
}
# This guard is also applied to dependency upgrades, not just requested packages.
apt_transaction() {
    local simulation
    simulation=$(apt-get -s --no-remove --no-upgrade install "$@") || fail 'apt simulation failed; no packages changed.'
    printf '%s\n' "$simulation"
    if grep -Eq '^(Remv |Inst [^ ]+ \[)' <<< "$simulation"; then
        fail 'apt would change/remove an installed version. Resolve manually; no packages changed.'
    fi
    # Deliberately no -y: apt shows the resolved transaction and requests approval.
    sudo apt-get --no-remove --no-upgrade install "$@"
}
system_setup() {
    (( ${#MISSING[@]} )) || return 0
    (( ALLOW_SYSTEM )) || fail 'Missing system dependencies: rerun with --allow-system after reviewing --plan.'
    command -v sudo >/dev/null || fail 'sudo is required; configure it as the Ubuntu administrator.'
    echo 'SYSTEM: sudo apt metadata refresh and missing prerequisite installation.'
    sudo apt-get update
    local bootstrap=() p
    for p in ca-certificates curl gnupg software-properties-common; do
        installed "$p" || bootstrap+=("$p")
    done
    (( ${#bootstrap[@]} == 0 )) || apt_transaction "${bootstrap[@]}"
    if ! apt-cache policy | grep -q 'noble/universe'; then
        echo 'SYSTEM: enable Ubuntu Universe repository.'
        sudo add-apt-repository -y universe
    fi
    if ! apt-cache policy ros-jazzy-ros-base | grep -q 'Candidate: [^ (]'; then
        installed ros2-apt-source && fail 'ROS apt source installed but no Jazzy candidate; repair repository manually.'
        local deb="$STATE/ros2-apt-source.deb" arch
        arch=$(dpkg --print-architecture)
        curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$ROS_APT_VERSION/ros2-apt-source_${ROS_APT_VERSION}.noble_all.deb" -o "$deb"
        echo "SYSTEM: install ROS apt source $ROS_APT_VERSION for $arch (keyring/repository)."
        apt_transaction "$deb"
        sudo apt-get update
    fi
    if ! apt-cache policy gz-harmonic | grep -q 'Candidate: [^ (]'; then
        if grep -Rqs 'packages.osrfoundation.org/gazebo' /etc/apt/sources.list /etc/apt/sources.list.d; then
            fail 'OSRF source exists but no Harmonic candidate; repair manually (existing sources preserved).'
        fi
        local key=/usr/share/keyrings/thermoswarm-osrf.gpg list=/etc/apt/sources.list.d/thermoswarm-osrf.list
        [[ ! -e $key && ! -e $list ]] || fail 'OSRF destination already exists; inspect it manually.'
        curl -fL https://packages.osrfoundation.org/gazebo.gpg -o "$STATE/osrf.gpg"
        gpg --show-keys "$STATE/osrf.gpg" >/dev/null
        sudo install -m 644 "$STATE/osrf.gpg" "$key"
        printf 'deb [arch=%s signed-by=%s] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main\n' "$(dpkg --print-architecture)" "$key" | sudo tee "$list" >/dev/null
        sudo apt-get update
    fi
    missing_packages
    (( ${#MISSING[@]} == 0 )) || apt_transaction "${MISSING[@]}"
}
clone_pinned() {
    local dir="$1" url="$2" ref="$3" tag="${4:-}"
    if [[ -e $dir ]]; then repo_ok "$dir" "$ref" || fail "Preserved incompatible checkout: $dir"; return; fi
    mkdir -p "$(dirname -- "$dir")"
    # Failed clones remain visible for manual inspection; never delete user data on retry.
    git init "$dir"
    git -C "$dir" remote add origin "$url"
    git -C "$dir" fetch --depth 1 origin "$ref"
    # PX4's build embeds git-describe output: retain its verified release tag
    # even with a shallow immutable-commit fetch. Never fetch into old checkouts.
    if [[ -n $tag ]]; then
        git -C "$dir" fetch --depth 1 origin "refs/tags/$tag:refs/tags/$tag"
        [[ $(git -C "$dir" rev-parse "$tag^{commit}") == "$ref" ]] || fail "Release tag $tag no longer matches pinned $ref."
    fi
    git -C "$dir" checkout --detach "$ref"
    git -C "$dir" submodule update --init --recursive
    repo_ok "$dir" "$ref" || fail "Source verification failed: $dir"
}
cmake_install() {
    local name="$1" src="$2" ref="$3"; shift 3
    local fingerprint marker="$STATE/agent/$name.done"
    fingerprint=$(printf '%s\n' "$ref" "$PREFIX" "$@" | sha256sum | cut -d' ' -f1)
    if [[ -f $marker && $(cat "$marker") == "$fingerprint" && -d $PREFIX/lib ]]; then
        echo "KEEP built Agent component: $name"; return
    fi
    cmake -S "$src" -B "$STATE/agent/build-$name" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_RPATH="$PREFIX/lib" \
        -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON "$@"
    cmake --build "$STATE/agent/build-$name" --parallel "$JOBS"
    cmake --install "$STATE/agent/build-$name"
    printf '%s\n' "$fingerprint" > "$marker"
}
build_agent() {
    agent_ok && return 0
    if agent_command >/dev/null; then fail 'Existing Agent version/libraries do not match v2.4.3; preserved. Resolve manually.'; fi
    local src="$STATE/agent/sources"
    clone_pinned "$src/fastcdr" https://github.com/eProsima/Fast-CDR.git "$FASTCDR_REF"
    clone_pinned "$src/memory" https://github.com/foonathan/memory.git "$MEMORY_REF"
    clone_pinned "$src/fastdds" https://github.com/eProsima/Fast-DDS.git "$FASTDDS_REF"
    clone_pinned "$src/spdlog" https://github.com/gabime/spdlog.git "$SPDLOG_REF"
    cmake_install fastcdr "$src/fastcdr" "$FASTCDR_REF" -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF
    cmake_install memory "$src/memory" "$MEMORY_REF" -DFOONATHAN_MEMORY_BUILD_TESTS=OFF -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    cmake_install fastdds "$src/fastdds" "$FASTDDS_REF" -DTHIRDPARTY=FORCE -DTHIRDPARTY_UPDATE=OFF -DCOMPILE_TOOLS=OFF -DBUILD_TESTING=OFF -DSECURITY=OFF -DSHM_TRANSPORT_DEFAULT=OFF
    cmake_install spdlog "$src/spdlog" "$SPDLOG_REF" -DSPDLOG_BUILD_SHARED=OFF -DSPDLOG_BUILD_EXAMPLE=OFF -DSPDLOG_BUILD_TESTS=OFF -DSPDLOG_INSTALL=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    cmake_install agent "$AGENT" "$AGENT_REF" -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON -DUAGENT_P2P_PROFILE=OFF -DUAGENT_BUILD_TESTS=OFF
    mkdir -p "$STATE/bin"
    # Scope Agent dependency libraries to its process; never shadow ROS DDS libraries.
    {
        echo '#!/usr/bin/env bash'
        # shellcheck disable=SC2016
        printf 'export LD_LIBRARY_PATH=%q${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n' "$PREFIX/lib"
        printf 'exec %q "$@"\n' "$PREFIX/bin/MicroXRCEAgent"
    } > "$STATE/bin/MicroXRCEAgent"
    chmod +x "$STATE/bin/MicroXRCEAgent"
    agent_ok || fail 'New local Agent failed library validation.'
}
build_px4() {
    [[ ! -x "$PX4/build/px4_sitl_default/bin/px4" ]] || return 0
    # Do not change an existing cache's compiler/interpreter or partial user build.
    [[ ! -e "$PX4/build/px4_sitl_default" ]] || fail 'Partial PX4 build cache/directory exists; inspect and build manually; directory preserved.'
    local venv="$ROOT/.venv-px4" fingerprint
    fingerprint=$(cat "$PX4/Tools/setup/requirements.txt" "$ROOT/scripts/requirements-px4-constraints.txt" | sha256sum | cut -d' ' -f1)
    if [[ -e $venv ]]; then
        [[ -x "$venv/bin/python" && -f "$STATE/px4-venv.sha256" && -f "$STATE/px4-python-resolved.txt" ]] || fail 'Unmanaged/partial .venv-px4; preserved. Inspect it manually.'
        [[ $(cat "$STATE/px4-venv.sha256") == "$fingerprint" ]] || fail 'PX4 Python requirements changed; existing venv preserved.'
        [[ $("$venv/bin/python" -m pip freeze) == "$(cat "$STATE/px4-python-resolved.txt")" ]] || fail 'PX4 Python versions changed; existing venv preserved.'
        "$venv/bin/python" -m pip check
    else
        /usr/bin/python3 -m venv "$venv"
        "$venv/bin/python" -m pip install -r "$PX4/Tools/setup/requirements.txt" -c "$ROOT/scripts/requirements-px4-constraints.txt"
        "$venv/bin/python" -m pip check
        "$venv/bin/python" -m pip freeze > "$STATE/px4-python-resolved.txt"
        printf '%s\n' "$fingerprint" > "$STATE/px4-venv.sha256"
    fi
    # Explicit target builds only. No gz_x500/run target; never launches PX4.
    (cd "$PX4"
        unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
        PATH="$venv/bin:$PATH" VIRTUAL_ENV="$venv" PYTHON_EXECUTABLE="$venv/bin/python" GZ_DISTRO=harmonic \
            make -j"$JOBS" j="$JOBS" px4_sitl_default
    )
}
workspace_fingerprint() {
    # Hash file contents, paths and symlink targets; mtime changes alone do not rebuild.
    /usr/bin/python3 -B - "$ROOT" "$MSGS_REF" <<'PY'
import hashlib
import sys
from pathlib import Path
root = Path(sys.argv[1])
h = hashlib.sha256(sys.argv[2].encode())
paths = [root / 'config/fleet.yaml', root / 'scripts/setup.sh']
paths += list((root / 'ros2_ws/src/thermoswarm_control').rglob('*'))
for path in sorted(paths):
    if any(p in {'__pycache__', '.pytest_cache', 'build', 'dist'} or p.endswith('.egg-info') for p in path.parts):
        continue
    if path.is_file() and path.suffix != '.pyc':
        h.update(str(path.relative_to(root)).encode())
        h.update(path.read_bytes())
print(h.hexdigest())
PY
}
colcon_build() (
    set +u
    unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO VIRTUAL_ENV
    export PATH=/usr/bin:/bin
    export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" MAKEFLAGS="-j$JOBS -l$JOBS"
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    cd "$ROOT/ros2_ws"
    /usr/bin/python3 -m colcon --log-base log build --base-paths src/px4_msgs src/thermoswarm_control \
        --packages-select px4_msgs thermoswarm_control --executor sequential --symlink-install \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
)
build_workspace() {
    local fingerprint
    fingerprint=$(workspace_fingerprint)
    if [[ -f "$STATE/workspace.sha256" && $(cat "$STATE/workspace.sha256") == "$fingerprint" ]] && ros_check; then
        echo 'KEEP ROS workspace: source fingerprint and installed package checks match.'; return
    fi
    colcon_build
    ros_check || fail 'Workspace verification failed; no success fingerprint written.'
    printf '%s\n' "$fingerprint" > "$STATE/workspace.sha256"
}
main() {
    case "$MODE" in
        check) check_all; return ;;
        plan) plan; return ;;
    esac
    plan
    platform_ok || fail 'Only Ubuntu 24.04 amd64/arm64 is supported.'
    (( EUID != 0 )) || fail 'Run as your normal Ubuntu user, not root/sudo.'
    if command -v pgrep >/dev/null && pgrep -x px4 >/dev/null; then
        fail 'PX4 is running; stop the simulation yourself before installing. No process was stopped.'
    fi
    # Validate existing resources before any sudo, network or local mutations.
    local dir ref
    for dir in "$PX4" "$MSGS" "$AGENT"; do
        [[ -e $dir ]] || continue
        command -v git >/dev/null || fail 'git missing; existing checkouts cannot be verified.'
        case "$dir" in "$PX4") ref="$PX4_REF" ;; "$MSGS") ref="$MSGS_REF" ;; *) ref="$AGENT_REF" ;; esac
        repo_ok "$dir" "$ref" || fail "Existing source preserved: $dir"
    done
    [[ ! -e "$ROOT/.venv-ros" ]] || venv_ok || fail 'Incompatible .venv-ros; preserved.'
    if agent_command >/dev/null; then agent_ok || fail 'Existing Agent incompatible/unverifiable; preserved.'; fi
    if (( ${#MISSING[@]} && ! ALLOW_SYSTEM )); then fail 'Missing dependencies; review --allow-system. Nothing changed.'; fi
    confirm
    mkdir -p "$STATE"
    # Prevent concurrent installers; plan/check intentionally create no locks.
    exec 9>"$STATE/install.lock"
    flock -n 9 || fail 'Another installer is running.'
    system_setup
    clone_pinned "$PX4" https://github.com/PX4/PX4-Autopilot.git "$PX4_REF" v1.17.0
    clone_pinned "$AGENT" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_REF" v2.4.3
    clone_pinned "$MSGS" https://github.com/PX4/px4_msgs.git "$MSGS_REF" v1.17.0
    if [[ ! -e "$ROOT/.venv-ros" ]]; then /usr/bin/python3 -m venv --system-site-packages "$ROOT/.venv-ros"; fi
    build_px4
    build_agent
    build_workspace
    check_all
    echo 'Setup complete. In your terminal: source scripts/env.sh'
}
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main; fi

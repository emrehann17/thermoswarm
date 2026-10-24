#!/usr/bin/env bash
# Source this file in Bash; do not execute it.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source scripts/env.sh' >&2
    exit 2
fi

_thermoswarm_env() {
    local root had_nounset=0 result=0
    root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    [[ $- == *u* ]] && had_nounset=1
    set +u
    if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$root/ros2_ws/install/local_setup.bash" || ! -f "$root/.venv-ros/bin/activate" ]]; then
        echo 'Missing environment. Run scripts/setup.sh --check first.' >&2
        result=1
    else
        # shellcheck source=/dev/null
        source /opt/ros/jazzy/setup.bash || result=1
        # Use local_setup to avoid replaying unrelated underlays from build time.
        # shellcheck source=/dev/null
        source "$root/ros2_ws/install/local_setup.bash" || result=1
        # shellcheck source=/dev/null
        source "$root/.venv-ros/bin/activate" || result=1
        if [[ -x "$root/.runtime/setup/bin/MicroXRCEAgent" ]]; then
            export PATH="$root/.runtime/setup/bin:$PATH"
        fi
        export GZ_DISTRO=harmonic
        export GZ_IP="${GZ_IP:-127.0.0.1}"
        export LANG="${LANG:-C.UTF-8}"
    fi
    (( had_nounset )) && set -u
    return "$result"
}
_thermoswarm_env

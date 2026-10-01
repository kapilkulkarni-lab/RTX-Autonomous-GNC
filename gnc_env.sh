# gnc_env.sh - GNC team shell environment. SOURCE this, don't run it:
#     source <repo>/gnc_env.sh      (after setup, just type: gnc)
#
# Order matters: ROS 2 first, then the venv, so the venv's python can still
# import rclpy / px4_msgs. Set GNC_NO_VENV=1 to skip the venv (used by the
# Gazebo/PX4/agent panes, since PX4's build expects the system python).
# Override any path below by exporting it before sourcing.

GNC_ROOT="${GNC_ROOT:-$HOME/dev}"
export PX4_DIR="${PX4_DIR:-$GNC_ROOT/PX4-Autopilot}"
export AVC_SIM_DIR="${AVC_SIM_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"  # this repo
export GNC_VENV="${GNC_VENV:-$GNC_ROOT/gnc-venv}"
export GNC_WS="${GNC_WS:-$GNC_ROOT/ros2_ws}"
GNC_ROS_DISTRO="${GNC_ROS_DISTRO:-jazzy}"

if command -v MicroXRCEAgent >/dev/null 2>&1; then
    export XRCE_AGENT="${XRCE_AGENT:-$(command -v MicroXRCEAgent)}"
else
    export XRCE_AGENT="${XRCE_AGENT:-$GNC_ROOT/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
fi

# Leave any active venv first so ROS is layered underneath cleanly.
if type deactivate >/dev/null 2>&1; then deactivate; fi

if [ -f "/opt/ros/$GNC_ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$GNC_ROS_DISTRO/setup.bash"
else
    echo "[gnc] warning: ROS 2 $GNC_ROS_DISTRO not found in /opt/ros" >&2
fi

# Your own ROS 2 workspace (px4_msgs, planner nodes) once it's built.
if [ -f "$GNC_WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$GNC_WS/install/setup.bash"
fi

# Let Gazebo find PX4's vehicle models (added once, even if sourced repeatedly).
_gnc_models="$PX4_DIR/Tools/simulation/gz/models"
case ":${GZ_SIM_RESOURCE_PATH:-}:" in
    *":$_gnc_models:"*) ;;
    *) export GZ_SIM_RESOURCE_PATH="$_gnc_models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" ;;
esac
unset _gnc_models

if [ -z "${GNC_NO_VENV:-}" ] && [ -f "$GNC_VENV/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$GNC_VENV/bin/activate"
fi

echo "[gnc] ROS 2 $GNC_ROS_DISTRO | venv: ${VIRTUAL_ENV:-off} | PX4: $PX4_DIR"

# WSL: DDS discovery over localhost UDP (no multicast / shared memory)
export FASTRTPS_DEFAULT_PROFILES_FILE="$AVC_SIM_DIR/fastdds_localhost.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

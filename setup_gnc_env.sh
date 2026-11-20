#!/usr/bin/env bash
# setup_gnc_env.sh - one-time GNC dev environment setup. Safe to re-run any time.
#
# Prerequisites (see README): Ubuntu 24.04 (native or WSL2), PX4-Autopilot cloned and
# its Tools/setup/ubuntu.sh run, ROS 2 Jazzy installed. This script does the rest:
#   XRCE-DDS agent, Python venv, comms avc_msgs (pinned in gnc.repos), px4_msgs +
#   avc_interfaces + gnc_offboard workspace build, `gnc` shortcut.
#
# Paths default to ~/dev/...; override with e.g.  PX4_DIR=/elsewhere ./setup_gnc_env.sh
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GNC_ROOT="${GNC_ROOT:-$HOME/dev}"
PX4_DIR="${PX4_DIR:-$GNC_ROOT/PX4-Autopilot}"
VENV="${GNC_VENV:-$GNC_ROOT/gnc-venv}"
WS="${GNC_WS:-$GNC_ROOT/ros2_ws}"
DISTRO="${GNC_ROS_DISTRO:-jazzy}"
AGENT_DIR="$GNC_ROOT/Micro-XRCE-DDS-Agent"

step() { printf '\n==> %s\n' "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

# Never build inside the venv, even if it was active when this was launched.
if [ -n "${VIRTUAL_ENV:-}" ]; then
    PATH="${PATH//$VIRTUAL_ENV\/bin:/}"
    unset VIRTUAL_ENV
fi

step "Checking prerequisites"
[ -f "/opt/ros/$DISTRO/setup.bash" ] || die "ROS 2 $DISTRO is not installed (README step 2)"
[ -d "$PX4_DIR" ] || die "PX4 not found at $PX4_DIR (README step 1), or set PX4_DIR=..."
command -v gz >/dev/null || die "Gazebo (gz) not found: run $PX4_DIR/Tools/setup/ubuntu.sh"
command -v vcs >/dev/null || die "vcstool (vcs) not found: sudo apt install ros-dev-tools (README step 2)"
echo "ROS 2 $DISTRO, PX4 at $PX4_DIR, repo at $REPO"

step "System packages"
sudo apt install -y tmux python3-venv python3-pip git cmake build-essential

step "Micro XRCE-DDS Agent (PX4 <-> ROS 2 bridge)"
if command -v MicroXRCEAgent >/dev/null || [ -x "$AGENT_DIR/build/MicroXRCEAgent" ]; then
    echo "already built"
else
    [ -d "$AGENT_DIR" ] || git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
    mkdir -p "$AGENT_DIR/build"
    (cd "$AGENT_DIR/build" && cmake .. && make -j"$(nproc)" && sudo make install)
    sudo ldconfig /usr/local/lib/
fi

step "Python venv ($VENV)"
if [ -d "$VENV" ]; then
    base="$("$VENV/bin/python" -c 'import sys; print(sys.base_prefix)')"
    if [ "$base" != "/usr" ]; then
        die "$VENV was made from $base, not the system python, so ROS 2 imports fail.
       Move it aside (mv $VENV ${VENV}.old) and re-run."
    fi
    sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' "$VENV/pyvenv.cfg"
else
    /usr/bin/python3 -m venv --system-site-packages "$VENV"
fi
touch "$VENV/COLCON_IGNORE"
"$VENV/bin/pip" install -q -r "$REPO/requirements-gnc.txt"

step "ROS 2 workspace ($WS)"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/px4_msgs" ]; then
    # px4_msgs must match the PX4 version, or topics silently carry the wrong layout.
    head="$(git -C "$PX4_DIR" rev-parse --abbrev-ref HEAD)"
    branch=main
    if [[ $head == release/* ]]; then
        branch="$head"
    elif [ "$head" = "HEAD" ]; then
        tag="$(git -C "$PX4_DIR" describe --tags --exact-match 2>/dev/null || true)"
        [[ $tag =~ ^v1\.([0-9]+) ]] && branch="release/1.${BASH_REMATCH[1]}"
    fi
    if ! git ls-remote --exit-code --heads https://github.com/PX4/px4_msgs.git "$branch" >/dev/null; then
        echo "px4_msgs has no branch $branch; using main"
        branch=main
    fi
    echo "PX4 on '$head' -> px4_msgs '$branch'"
    git clone -b "$branch" https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
fi

# Comms-owned avc_msgs, pinned in gnc.repos. Re-running checks out the pinned commit again,
# so a changed pin is picked up. vcs refuses to touch a clone with local changes; that is on
# purpose, because nobody on GNC edits avc_msgs.
step "Comms messages (avc_msgs, pinned in gnc.repos)"
vcs import "$GNC_ROOT" < "$REPO/gnc.repos"
COMMS_DIR="$GNC_ROOT/avc-comms-ws"
[ -d "$COMMS_DIR/src/avc_msgs/msg" ] || die "avc_msgs not found at $COMMS_DIR/src/avc_msgs after vcs import"
echo "avc-comms-ws at $(git -C "$COMMS_DIR" rev-parse --short HEAD)"

# Link packages into the workspace: the repo's own, plus avc_msgs only from the comms clone
# (its avc_comms package is not built here).
link_package() {  # link_package <name> <target dir>
    local package="$1" target="$2" link="$WS/src/$1" bak
    if [ -e "$link" ] && [ ! -L "$link" ]; then
        bak="$WS/$package.bak-$(date +%Y%m%d%H%M%S)"
        mv "$link" "$bak"
        touch "$bak/COLCON_IGNORE"   # keep colcon from seeing a duplicate package
        echo "Moved an older copy of $package to $bak"
    fi
    ln -sfn "$target" "$link"
}
link_package avc_interfaces "$REPO/avc_interfaces"
link_package gnc_offboard "$REPO/gnc_offboard"
link_package avc_msgs "$COMMS_DIR/src/avc_msgs"

step "Building workspace (px4_msgs takes 5-15 min the first time; later runs skip it)"
(
    set +e
    # shellcheck disable=SC1090
    source "/opt/ros/$DISTRO/setup.bash"
    set -e
    cd "$WS"
    colcon build --packages-skip-build-finished
    # avc_msgs first: gnc_offboard depends on it.
    colcon build --packages-select avc_msgs avc_interfaces gnc_offboard
)

step "Sanity check"
(
    set +e
    # shellcheck disable=SC1090
    source "/opt/ros/$DISTRO/setup.bash"
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    set -e
    "$VENV/bin/python" -c "import rclpy, px4_msgs.msg, avc_msgs.msg, avc_interfaces.msg, numpy; print('OK: rclpy + px4_msgs + avc_msgs + avc_interfaces import in the venv, numpy', numpy.__version__)"
)

step "Shell shortcut"
sed -i '/^alias gnc=/d' ~/.bashrc
echo "alias gnc='source $REPO/gnc_env.sh'" >> ~/.bashrc
chmod +x "$REPO"/*.sh

cat <<EOF

All set. Open a NEW terminal, then:
  gnc                          dev shell (ROS 2 + workspace + venv)
  $REPO/start_sim.sh           Gazebo + PX4 + agent + dev shell in tmux  (-H = no window)
  ros2 run gnc_offboard mission_manager   (in the dev pane, once PX4 says Ready; README: Flying mission_manager)
EOF

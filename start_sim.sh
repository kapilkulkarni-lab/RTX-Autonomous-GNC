#!/usr/bin/env bash
# start_sim.sh - launch Gazebo world + PX4 SITL + XRCE agent + a dev shell in one tmux window.
#
#   ./start_sim.sh                     reuse the current world
#   ./start_sim.sh -c 3 -s 12 -k ne    regenerate: challenge 3, seed 12, NE start corner
#   ./start_sim.sh -H                  headless: no Gazebo window (combine with the above)
#   ./start_sim.sh stop                shut everything down
#   gz sim -g                          open the Gazebo window on a running headless sim
#
# tmux keys: Ctrl+b then arrow = switch pane, Ctrl+b d = detach (sim keeps running),
#            `tmux attach -t avc` = reattach.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION=avc
WORLD=avc_field

stop_all() {
    tmux kill-session -t "$SESSION" 2>/dev/null
    pkill -f MicroXRCEAgent 2>/dev/null
    pkill -f 'bin/px4' 2>/dev/null
    pkill -f 'gz sim' 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    # Leftover Fast DDS shared-memory files from killed processes can block discovery.
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null
    return 0
}

if [ "${1:-}" = "stop" ]; then
    stop_all
    echo "Simulation stopped."
    exit 0
fi

CHALLENGE="" SEED="" CORNER="" HEADLESS=""
while getopts "c:s:k:Hh" opt; do
    case "$opt" in
        c) CHALLENGE="$OPTARG" ;;
        s) SEED="$OPTARG" ;;
        k) CORNER="$OPTARG" ;;
        H) HEADLESS=1 ;;
        *) sed -n '2,12p' "$0"; exit 0 ;;
    esac
done

export GNC_NO_VENV=1
# shellcheck disable=SC1091
source "$HERE/gnc_env.sh" >/dev/null

command -v tmux >/dev/null || { echo "tmux missing: sudo apt install tmux" >&2; exit 1; }
command -v gz >/dev/null || { echo "gz (Gazebo) not found on PATH" >&2; exit 1; }
[ -x "$XRCE_AGENT" ] || { echo "XRCE agent not found at $XRCE_AGENT" >&2; exit 1; }
[ -d "$PX4_DIR" ] || { echo "PX4 not found at $PX4_DIR" >&2; exit 1; }

SDF="$AVC_SIM_DIR/worlds/$WORLD.sdf"
MANIFEST="$AVC_SIM_DIR/worlds/${WORLD}_manifest.json"

if [ -n "$CHALLENGE$SEED$CORNER" ] || [ ! -f "$SDF" ]; then
    args=(--out-dir "$AVC_SIM_DIR/worlds" --world-name "$WORLD")
    [ -n "$CHALLENGE" ] && args+=(--challenge "$CHALLENGE")
    [ -n "$SEED" ] && args+=(--seed "$SEED")
    [ -n "$CORNER" ] && args+=(--start-corner "$CORNER")
    /usr/bin/python3 "$AVC_SIM_DIR/generate_avc_world.py" "${args[@]}" || exit 1
fi

POSE="$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["px4_spawn_pose"])' "$MANIFEST")" \
    || { echo "could not read spawn pose from $MANIFEST" >&2; exit 1; }

stop_all  # clean slate: no leftover PX4/Gazebo fighting over ports

ENV_CMD="export GNC_NO_VENV=1; source '$HERE/gnc_env.sh' >/dev/null"

# Pane 1: Gazebo (-s = physics server only, no GUI window)
GZ_FLAGS="-r"
[ -n "$HEADLESS" ] && GZ_FLAGS="-s -r"
tmux new-session -d -s "$SESSION" -n sim "$ENV_CMD; gz sim $GZ_FLAGS '$SDF'; exec bash"

echo -n "Waiting for Gazebo world '$WORLD'"
for _ in $(seq 60); do
    if gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; then break; fi
    echo -n "."; sleep 1
done
echo

# Pane 2: PX4 (attaches to the running world)
tmux split-window -t "$SESSION:sim" -h \
    "$ENV_CMD; cd '$PX4_DIR' && PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$WORLD PX4_GZ_MODEL_POSE='$POSE' make px4_sitl gz_x500; exec bash"

# Pane 3: XRCE-DDS agent (PX4 <-> ROS 2 bridge). Started with a clean environment so it
# uses the DDS libraries it was built with, not ROS 2's copies.
tmux split-window -t "$SESSION:sim" -v \
    "env -i FASTRTPS_DEFAULT_PROFILES_FILE='$HERE/fastdds_localhost.xml' HOME='$HOME' TERM=xterm-256color PATH=/usr/local/bin:/usr/bin:/bin '$XRCE_AGENT' udp4 -p 8888; exec bash"

# Pane 4: your dev shell, with the venv active
tmux split-window -t "$SESSION:sim" -v "source '$HERE/gnc_env.sh'; exec bash"

tmux select-layout -t "$SESSION:sim" tiled
tmux select-pane -t "$SESSION:sim.3"

echo "Spawn pose: $POSE"
if [ -n "${TMUX:-}" ]; then tmux switch-client -t "$SESSION"; else tmux attach -t "$SESSION"; fi

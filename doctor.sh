#!/usr/bin/env bash
# doctor.sh - health check of the sim chain. Run it while start_sim.sh is up.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
GNC_NO_VENV=1 source "$HERE/gnc_env.sh" >/dev/null 2>&1

# check "<ok message>" "<fail message>" command...
check() {
    local good="$1" badmsg="$2"; shift 2
    if "$@" >/dev/null 2>&1; then printf '  [ok]   %s\n' "$good"; else printf '  [FAIL] %s\n' "$badmsg"; fi
}

echo "avc-gnc doctor ($(cat "$HERE/VERSION" 2>/dev/null))"
check "ros2 found ($ROS_DISTRO)" "ros2 not found: is ROS 2 installed?" command -v ros2
check "px4_msgs built" "px4_msgs not built: run setup_gnc_env.sh" test -f "$GNC_WS/install/px4_msgs/share/px4_msgs/package.xml"
check "gnc_offboard built" "gnc_offboard not built: run setup_gnc_env.sh" test -f "$GNC_WS/install/gnc_offboard/share/gnc_offboard/package.xml"
check "workspace links to this repo" "$GNC_WS/src/gnc_offboard is not a link to this repo" test -L "$GNC_WS/src/gnc_offboard"
check "Gazebo running" "Gazebo not running" pgrep -f '[g]z sim'
check "PX4 running" "PX4 not running" pgrep -f '[b]in/px4'
agents=$(pgrep -fc '[M]icroXRCEAgent udp4')
check "one XRCE agent running" "$agents XRCE agents running (want exactly 1)" test "$agents" = 1
env | grep -E '^(ROS_DOMAIN_ID|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|FASTRTPS_|FASTDDS_)' | sed 's/^/  [env]  /'
count=$(timeout 15 ros2 topic list --no-daemon --spin-time 5 2>/dev/null | grep -c '^/fmu/')
check "${count:-0} PX4 topics visible to ROS 2" "no /fmu topics visible to ROS 2" test "${count:-0}" -gt 0

#!/bin/bash
# slam_lifecycle_up.sh — bring /slam_toolbox to 'active' (companion, ExecStartPost of q6a-slam-toolbox).
#
# slam_toolbox in Jazzy (2.8+) is a LIFECYCLE node: launched via `ros2 run` it sits in 'unconfigured'
# forever — no solver load, no /scan subscription, no logs, no errors (this cost us a whole debugging
# arc on 2026-07-08). This script walks it up state-by-state and is idempotent: safe to re-run, safe
# if the node is already active.
#
# Map save/resume is handled by q6a_map_persist.py (its own ROS node + systemd service), not this
# script -- a brief 2026-07-12 iteration had this script also calling deserialize_map directly, but
# that was reverted in favor of a dedicated node (see q6a_map_persist.py's docstring).
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

CS='/slam_toolbox/change_state lifecycle_msgs/srv/ChangeState'
for i in $(seq 1 30); do
    st=$(timeout 8 ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState {} 2>/dev/null \
         | grep -oE "label='[a-z]+'" | cut -d"'" -f2)
    case "$st" in
        active)        echo "slam_toolbox: active"; exit 0 ;;
        unconfigured)  timeout 10 ros2 service call $CS "{transition: {id: 1}}" >/dev/null 2>&1 ;;  # configure
        inactive)      timeout 10 ros2 service call $CS "{transition: {id: 3}}" >/dev/null 2>&1 ;;  # activate
        *)             ;;  # node not discovered yet — keep waiting
    esac
    sleep 2
done
echo "slam_toolbox: FAILED to reach active" >&2
exit 1

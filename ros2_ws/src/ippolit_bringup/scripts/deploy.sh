#!/bin/bash
# deploy.sh — run ON the Q6A. Idempotent: git pull -> rosdep -> colcon build -> restart the 4 launch
# groups. Replaces the old scp-file-by-file workflow (which caused real drift: a deployed mcu_node.py
# silently missing a month of decode work, a stray systemd drop-in overriding a committed unit edit).
# Device state is now fully derived from a git tag/branch + this script — see
# docs/navigation-architecture.md §2.6 (D4) and §7 (success criteria: repo = single source of truth).
set -euo pipefail

REPO=${IPPOLIT_REPO:-$HOME/ippolit}
WS="$REPO/ros2_ws"

echo "== deploy.sh: $(date) =="
cd "$REPO"
git fetch origin
git checkout "${1:-master}"
git pull --ff-only

echo "-- rosdep --"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "$WS/src" --ignore-src -y || echo "rosdep: non-fatal issues, continuing"

echo "-- colcon build --"
cd "$WS"
colcon build --symlink-install

echo "-- restart launch groups --"
sudo systemctl daemon-reload
for svc in ippolit-core ippolit-perception ippolit-nav ippolit-viz; do
    if systemctl list-unit-files | grep -q "^${svc}.service"; then
        sudo systemctl restart "$svc"
        echo "restarted $svc"
    else
        echo "skip $svc (unit not installed yet)"
    fi
done

echo "== deploy.sh: done =="

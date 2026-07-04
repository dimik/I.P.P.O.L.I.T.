#!/bin/bash
# View the Q6A IMX296 MJPEG stream on the Odyssey screen. Run ON the Odyssey.
# Starts the streamer on the Q6A over SSH (if not already running), then opens a viewer.
# Usage: ./view_q6a_cam.sh [CAM 2|3]
set -euo pipefail
CAM="${1:-2}"
Q6A="ippolit-lan"                 # wired link (192.168.20.2)
HOST_IP="192.168.20.2"; PORT=8092
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "== ensure streamer script is on the Q6A =="
scp -q "$REPO_DIR/q6a_camstream.py" "$Q6A:~/q6a_camstream.py"

echo "== start streamer on the Q6A (detached; harmless if already running) =="
ssh "$Q6A" "ss -ltn 2>/dev/null | grep -q ":$PORT " || \
  setsid python3 ~/q6a_camstream.py --cam $CAM --port $PORT </dev/null >~/camstream.log 2>&1 &" || true
sleep 3
echo "   stream: http://$HOST_IP:$PORT/   (log: ssh $Q6A 'tail -f ~/camstream.log')"

echo "== open viewer on the Odyssey =="
URL="http://$HOST_IP:$PORT/"
if command -v firefox >/dev/null; then firefox --new-window "$URL" >/dev/null 2>&1 &
elif command -v vlc >/dev/null; then vlc "${URL}stream" >/dev/null 2>&1 &
else xdg-open "$URL" >/dev/null 2>&1 & fi
echo "Viewer opened. Stop the stream: ssh $Q6A 'sudo pkill -f q6a_camstream.py'"

#!/bin/bash
# View the Q6A IMX296 MJPEG stream on the Odyssey screen. Run ON the Odyssey.
# Usage: ./view_q6a_cam.sh [CAM 2|3]        -> start streamer + open viewer
#        ./view_q6a_cam.sh calibrate [2|3]  -> one-time flat-field COLOR calibration (aim at white surface)
set -euo pipefail
Q6A="ippolit-lan"                 # wired link (192.168.20.2)
HOST_IP="192.168.20.2"; PORT=8092
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

stop_streamer() { ssh "$Q6A" "p=\$(ss -ltnp 2>/dev/null|grep ':$PORT '|grep -oP 'pid=\K[0-9]+'|head -1); [ -n \"\$p\" ] && kill -9 \$p; pkill -9 -f '[v]4l2-ctl'; sleep 2" || true; }

if [ "${1:-}" = calibrate ]; then
    CAM="${2:-2}"
    scp -q "$REPO_DIR/q6a_camstream.py" "$Q6A:~/q6a_camstream.py"
    echo ">>> Aim the camera at a UNIFORM white/gray surface (white wall/paper), filling the frame,"
    echo ">>> under the SAME lighting you'll use. This calibrates lens-shading + white balance."
    read -r -p "Press Enter when ready... "
    stop_streamer
    ssh "$Q6A" "python3 ~/q6a_camstream.py --calibrate --cam $CAM"
    echo "== color profile saved. copying it into the repo (shared) =="
    scp -q "$Q6A:~/imx296_flatfield.npz" "$REPO_DIR/imx296_flatfield.npz" 2>/dev/null || true
    echo "Done. Now run: ./view_q6a_cam.sh $CAM"
    exit 0
fi

CAM="${1:-2}"
echo "== ensure streamer script + color profile are on the Q6A =="
scp -q "$REPO_DIR/q6a_camstream.py" "$Q6A:~/q6a_camstream.py"
[ -f "$REPO_DIR/imx296_flatfield.npz" ] && scp -q "$REPO_DIR/imx296_flatfield.npz" "$Q6A:~/imx296_flatfield.npz" 2>/dev/null || true

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

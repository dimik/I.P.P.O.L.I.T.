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
    echo ">>> Aim at a UNIFORM, EVENLY-LIT white/grey surface filling the whole frame (a blank wall or"
    echo ">>> a sheet of paper). Avoid shadows, glare and light pools; slightly DEFOCUS to blur texture."
    echo ">>> This measures white balance + the radial colour-shading map (magenta-centre / green-edge)."
    read -r -p "Press Enter when ready... "
    stop_streamer
    ssh "$Q6A" "python3 ~/q6a_camstream.py --calibrate --cam $CAM"
    echo "== color profile saved. copying it into the repo (shared) =="
    scp -q "$Q6A:~/imx296_wb.npz" "$REPO_DIR/imx296_wb.npz" 2>/dev/null || true
    echo "Done. Now run: ./view_q6a_cam.sh $CAM"
    exit 0
fi

CAM="${1:-2}"
echo "== ensure streamer + detector + model + color profile are on the Q6A =="
scp -q "$REPO_DIR/q6a_camstream.py" "$Q6A:~/q6a_camstream.py"
scp -q "$REPO_DIR/q6a_yolo.py" "$Q6A:~/q6a_yolo.py"
scp -q "$REPO_DIR/q6a_gpu.py" "$Q6A:~/q6a_gpu.py"           # Adreno OpenCL ISP
scp -q "$REPO_DIR/q6a_v4l2.py" "$Q6A:~/q6a_v4l2.py"         # V4L2 mmap capture
scp -q "$REPO_DIR/q6a_detector.py" "$Q6A:~/q6a_detector.py" # NPU YOLO (separate process, no lock)
[ -f "$REPO_DIR/models/yolov8_det.bin" ] && scp -q "$REPO_DIR/models/yolov8_det.bin" "$Q6A:~/yolov8_det.bin" 2>/dev/null || true
[ -f "$REPO_DIR/models/coco_labels.txt" ] && scp -q "$REPO_DIR/models/coco_labels.txt" "$Q6A:~/coco_labels.txt" 2>/dev/null || true
[ -f "$REPO_DIR/imx296_wb.npz" ] && scp -q "$REPO_DIR/imx296_wb.npz" "$Q6A:~/imx296_wb.npz" 2>/dev/null || true

echo "== start streamer on the Q6A (detached; --gpu = full-res demosaic on the Adreno, ~23fps) =="
# ensure the Adreno OpenCL driver is registered as an ICD (pyopencl's loader needs it); idempotent
ssh "$Q6A" "[ -f /etc/OpenCL/vendors/adreno.icd ] || (sudo mkdir -p /etc/OpenCL/vendors && \
  echo /usr/lib/aarch64-linux-gnu/libOpenCL_adreno.so.1 | sudo tee /etc/OpenCL/vendors/adreno.icd >/dev/null)" 2>/dev/null || true
# --gpu falls back to --fast automatically if pyopencl/OpenCL is unavailable
ssh "$Q6A" "ss -ltn 2>/dev/null | grep -q ":$PORT " || \
  setsid python3 ~/q6a_camstream.py --cam $CAM --port $PORT --gpu </dev/null >~/camstream.log 2>&1 &" || true
sleep 3
echo "   stream: http://$HOST_IP:$PORT/stream   (log: ssh $Q6A 'tail -f ~/camstream.log')"

# DO NOT use Firefox: it leaks decoded MJPEG frames into shmem (~800MB) and the OOM killer
# stops it. VLC/mpv render multipart/x-mixed-replace with bounded memory.
echo "== open viewer on the Odyssey (VLC — Firefox OOMs on MJPEG) =="
SURL="http://$HOST_IP:$PORT/stream"
if command -v mpv >/dev/null; then mpv --profile=low-latency --no-cache "$SURL" >/dev/null 2>&1 &
elif command -v vlc >/dev/null; then vlc --network-caching=200 "$SURL" >/dev/null 2>&1 &
elif command -v ffplay >/dev/null; then ffplay -fflags nobuffer "$SURL" >/dev/null 2>&1 &
else echo "No mpv/vlc/ffplay — install one (avoid Firefox)."; fi
echo "Viewer opened (VLC). Stop the stream: ssh $Q6A 'pkill -f \"[q]6a_camstream.py\"'"

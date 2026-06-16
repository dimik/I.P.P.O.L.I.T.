#!/bin/sh
# cam_grab.sh — grab one camera frame via camsiphon (run ON the robot).
#
# Requires libcamsiphon.so preloaded into AVA (see _root.sh / build_ava_shims.sh). camsiphon
# watches /tmp/cam_grab and, on the next VIDIOC_DQBUF, writes the frame AVA just captured to
# /tmp/cam_frame.raw (+ /tmp/cam_fmt.txt with "WxH pf=XXXX sz=N"). Read-only; AVA is untouched.
#
# Pull + convert on your workstation, e.g.:
#   ssh dreame-home 'base64 /tmp/cam_frame.raw' | base64 -d > frame.nv21
#   python3 scripts/robot/nv21_to_png.py frame.nv21 672 504 frame.png

rm -f /tmp/cam_frame.raw
touch /tmp/cam_grab
i=0
while [ -e /tmp/cam_grab ] && [ $i -lt 30 ]; do i=$((i+1)); sleep 0.1 2>/dev/null || sleep 1; done
if [ -f /tmp/cam_frame.raw ]; then
    echo "grabbed: $(cat /tmp/cam_fmt.txt 2>/dev/null) -> /tmp/cam_frame.raw ($(wc -c < /tmp/cam_frame.raw) bytes)"
else
    echo "no frame — is libcamsiphon.so loaded into AVA? ($(grep -c libcamsiphon /proc/$(pidof ava)/maps 2>/dev/null) in maps)"
fi

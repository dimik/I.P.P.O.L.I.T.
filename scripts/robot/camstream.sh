#!/bin/sh
# camstream.sh — start/stop the cedar HW-JPEG MJPEG-over-HTTP camera stream (run ON the robot).
#
# Pipeline:  AVA -> camsiphon (RAM ring /tmp/cam_stream.buf) -> camstream (cedar JPEG) -> HTTP.
# View in any browser / VLC / ffplay:  http://<robot-ip>:8090/
#
# Requires: libcamsiphon.so preloaded into AVA (stream-capable build) and the vendor encoder
# libs staged at /data/chroot/opt/venc + /data/chroot/opt/camstream built (build_ava_shims.sh).
#
#   sh /data/camstream.sh start [port]   # default port 8090
#   sh /data/camstream.sh stop
#   sh /data/camstream.sh status

CH=/data/chroot
PORT="${2:-8090}"

case "${1:-start}" in
  start)
    [ -x "$CH/opt/camstream" ] || { echo "ERROR: $CH/opt/camstream missing (run build_ava_shims.sh)"; exit 1; }
    mountpoint -q "$CH/tmp" || mount --bind /tmp "$CH/tmp"  # share the ring into the chroot
    # NOTE: client-gated — camstream sets/clears /tmp/cam_stream itself per connection, so the
    # ring only fills (and AVA only does extra work) while a viewer is actually connected.
    pkill -9 -f /opt/camstream 2>/dev/null; sleep 1
    setsid chroot "$CH" sh -c "LD_LIBRARY_PATH=/opt/venc /opt/camstream $PORT" > /tmp/camstream.log 2>&1 </dev/null &
    sleep 3
    echo "camstream started on :$PORT (log: /tmp/camstream.log)"
    tail -3 /tmp/camstream.log 2>/dev/null
    ;;
  stop)
    pkill -9 -f /opt/camstream 2>/dev/null                       # matches the chroot'd /opt/camstream
    rm -f /tmp/cam_stream                                   # camsiphon stops filling the ring
    echo "camstream stopped, stream flag cleared"
    ;;
  status)
    pgrep -f /opt/camstream >/dev/null && echo "running (port: $(grep -o 'http://[^/]*' /tmp/camstream.log | tail -1))" || echo "not running"
    od -An -tu4 -N20 /tmp/cam_stream.buf 2>/dev/null | head -1 | awk '{print "ring: latest="$1" seq="$2" "$3"x"$4" size="$5}'
    ;;
  *) echo "usage: $0 {start [port]|stop|status}"; exit 1 ;;
esac

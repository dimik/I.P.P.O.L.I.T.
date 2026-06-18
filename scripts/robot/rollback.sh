#!/bin/sh
# rollback.sh — restore STOCK AVA operation by undoing our LD_PRELOAD shims + helper daemons.
# Run ON the robot:  sh /data/rollback.sh
#
# Use when a shim/daemon misbehaves and you want the vacuum back to factory behavior FAST, or to
# A/B test stock vs patched. It:
#   - kills our daemons (go2rtc, camstream, fanoff LiDAR gate) and clears their /tmp flags
#   - removes the patched-ava.sh bind-mount (so AVA relaunches with NO LD_PRELOAD shims)
#   - restarts AVA and verifies the shims are gone
#
# SCOPE: this is a "revert NOW, this boot" tool. The boot hooks (_root.sh + _root_postboot.sh)
# RE-APPLY everything on the next reboot — so rollback is for debugging / temporary stock mode,
# not a permanent uninstall. To make it permanent, also neutralise the boot hooks (see bottom).
#
# It deliberately does NOT touch the wifi / miio bind-mounts — removing those can drop the network
# (and your ssh session). Those are connectivity, not AVA behavior.

set -u
echo "=== rollback: stopping helper daemons ==="
pkill -9 -f /opt/go2rtc   2>/dev/null && echo "  go2rtc stopped"           || echo "  go2rtc not running"
pkill -9 -f /opt/camstream 2>/dev/null && echo "  camstream stopped"        || echo "  camstream not running"
pkill -f fanoff_flag      2>/dev/null && echo "  fanoff LiDAR gate stopped" || echo "  fanoff gate not running"
rm -f /tmp/cam_stream /tmp/cam_grab /tmp/cam_stream.buf /tmp/lidar_allow
echo "  flags cleared (/tmp/cam_stream, /tmp/cam_grab, /tmp/lidar_allow)"

echo "=== removing patched ava.sh bind-mount(s) ==="
n=0
while mountpoint -q /etc/rc.d/ava.sh 2>/dev/null; do
    umount /etc/rc.d/ava.sh 2>/dev/null || break
    n=$((n+1))
done
[ "$n" -gt 0 ] && echo "  removed $n bind-mount(s) — stock ava.sh restored" || echo "  ava.sh already stock (not bind-mounted)"

echo "=== restarting AVA (stock, no LD_PRELOAD) ==="
killall -9 ava 2>/dev/null; sleep 2
/etc/rc.d/ava.sh force >/dev/null 2>&1 &
sleep 10
A=$(pidof ava)
if [ -n "$A" ]; then
    echo "  ava pid=$A"
    for so in libfanoff_filter libfanoff_log libcamsiphon; do
        if grep -q "$so" "/proc/$A/maps" 2>/dev/null; then
            echo "  WARNING: $so STILL loaded — a bind-mount may have re-applied"
        fi
    done
    grep -qE "libfanoff|libcamsiphon" "/proc/$A/maps" 2>/dev/null || echo "  OK: no shims loaded — AVA is stock"
else
    echo "  WARNING: AVA not up yet — run: /etc/rc.d/ava.sh force"
fi
echo "=== rollback complete (stock behavior THIS boot; reboot re-applies the shims) ==="
echo
echo "To make stock PERMANENT across reboots, neutralise the boot hooks too, e.g.:"
echo "    mv /data/_root.sh /data/_root.sh.disabled"
echo "    mv /data/_root_postboot.sh /data/_root_postboot.sh.disabled   # NOTE: also stops the"
echo "    # wifi/Valetudo/chroot setup — only do this if you know how you'll keep network access."

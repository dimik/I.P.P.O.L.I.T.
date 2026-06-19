#!/bin/sh
# deploy_ava_shims.sh — install the AVA LD_PRELOAD shims live (without a reboot).
#
# MECHANISM (shared by every shim, feature-agnostic): AVA is launched by /etc/rc.d/ava.sh
# (squashfs, read-only) and restarted by the watchdog (exec_monitor/monitor.sh). To make EVERY
# launch preload our shims, we bind-mount a patched copy of ava.sh that exports LD_PRELOAD
# before exec'ing ava. Same squashfs-override trick as the wpa_supplicant.conf bind-mount.
#
# This is the same patch _root.sh applies at early boot (for reboot persistence). This script
# does it live AND restarts AVA so you can test immediately. Keep the LD_PRELOAD list here in
# sync with _root.sh.
#
# SHIMS (independent features; each loaded only if its .so is present):
#   libfanoff_filter.so  — fan/LiDAR quieting   (build: build_ava_shims.sh)
#   libcamsiphon.so      — read-only camera tap  (build: build_ava_shims.sh)
#   libserialtap.so         — read-only LiDAR ttyS3 tap -> /scan (build: build_ava_shims.sh)
#
# Run ON the robot:  sh /data/deploy_ava_shims.sh

set -e
LIBDIR=/data/lib
PATCHED=/data/ava.sh.preload
ORIG=/etc/rc.d/ava.sh

# Assemble the LD_PRELOAD list from whichever shims are built (order: filter, camsiphon, serialtap).
PRELOAD=""
for so in libfanoff_filter.so libcamsiphon.so libserialtap.so; do
    [ -f "$LIBDIR/$so" ] && PRELOAD="${PRELOAD:+$PRELOAD }$LIBDIR/$so"
done
[ -n "$PRELOAD" ] || { echo "ERROR: no shims in $LIBDIR — run build_ava_shims.sh first"; exit 1; }
echo "LD_PRELOAD list: $PRELOAD"

# Build a patched ava.sh: identical to the original but with LD_PRELOAD exported right after
# the shebang, so it applies to the `ava ... &` exec lines.
{
  head -1 "$ORIG"
  echo "export LD_PRELOAD=\"$PRELOAD\"   # injected by deploy_ava_shims.sh"
  tail -n +2 "$ORIG"
} > "$PATCHED"
chmod +x "$PATCHED"

echo "=== patched ava.sh preview (head) ==="
head -5 "$PATCHED"

# If a previous bind-mount is in place, drop it first (editing the file in place would change
# its inode and silently break an existing bind-mount).
mountpoint -q "$ORIG" 2>/dev/null && umount "$ORIG" 2>/dev/null || true
mount --bind "$PATCHED" "$ORIG"
echo "bind-mounted $PATCHED over $ORIG"

# Restart AVA so the new launch picks up LD_PRELOAD (watchdog or ava.sh will relaunch).
echo "restarting AVA..."
/etc/rc.d/ava.sh force >/dev/null 2>&1 &
sleep 8

# Verify the shims are mapped into the running AVA.
AVAPID=$(pidof ava)
echo "ava pid=$AVAPID"
for so in libfanoff_filter.so libcamsiphon.so libserialtap.so; do
    case " $PRELOAD " in *"$so"*)
        if grep -q "$so" "/proc/$AVAPID/maps" 2>/dev/null; then
            echo "OK: $so is loaded into AVA."
        else
            echo "WARNING: $so NOT found in AVA maps — check ava.sh patch / LD_PRELOAD path."
        fi ;;
    esac
done

echo
echo "TEST: enable manual nav from Valetudo and confirm the fan stays SILENT,"
echo "      and that driving + brush/pump still respond."
echo "Reboot persistence is handled by _root.sh (same patch at early boot)."

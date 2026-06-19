#!/bin/sh
# build_ava_shims.sh — compile the LD_PRELOAD shims injected into AVA, inside the robot's
# Ubuntu chroot. These are independent features that happen to share one mechanism (preload
# into AVA via the ava.sh bind-mount) and one build path (chroot gcc, freestanding):
#
#   libfanoff_log.so / libfanoff_filter.so  — fan/LiDAR quieting (src: fanoff_shim.c)
#       log    = Phase-1 capture (passthrough + dump MCU TX frames to /tmp/mcu_tx.log)
#       filter = production (rewrite MCU SetCleaning to idle; gate LiDAR motor)
#   libcamsiphon.so                          — read-only camera frame siphon (src: camsiphon.c)
#
# Must build in the chroot (native aarch64 gcc-13), but FREESTANDING so the result has no
# glibc symbol deps — AVA runs glibc 2.23, the chroot is glibc 2.39. See fanoff_shim.c.
#
# Run ON the robot:  sh /data/build_ava_shims.sh
# (deploy this script + fanoff_shim.c + camsiphon.c to /data/ first; see deploy_ava_shims.sh)

set -e

SRC=/data/fanoff_shim.c
OUTDIR=/data/lib
CHROOT=/data/chroot

mkdir -p "$OUTDIR"
cp "$SRC" "$CHROOT/tmp/fanoff_shim.c"

CFLAGS="-O2 -fPIC -shared -nostdlib -ffreestanding -fno-stack-protector -fno-builtin -Wall"

echo "Building libfanoff_log.so (capture mode)..."
chroot "$CHROOT" /usr/bin/gcc-13 $CFLAGS -DMODE_LOG    -o /tmp/libfanoff_log.so    /tmp/fanoff_shim.c

echo "Building libfanoff_filter.so (production mode)..."
chroot "$CHROOT" /usr/bin/gcc-13 $CFLAGS -DMODE_FILTER -o /tmp/libfanoff_filter.so /tmp/fanoff_shim.c

cp "$CHROOT/tmp/libfanoff_log.so"    "$OUTDIR/libfanoff_log.so"
cp "$CHROOT/tmp/libfanoff_filter.so" "$OUTDIR/libfanoff_filter.so"

echo "Built:"
ls -la "$OUTDIR/libfanoff_log.so" "$OUTDIR/libfanoff_filter.so"

# Sanity: confirm NO glibc version deps (must be loadable under glibc 2.23).
# readelf is in the chroot; .so should have no GLIBC_2.x verneed entries.
echo "--- version deps (should be EMPTY) ---"
chroot "$CHROOT" /usr/bin/readelf -V /tmp/libfanoff_filter.so 2>/dev/null | grep -i "GLIBC" || echo "  none — good (freestanding)"
echo "--- exported symbols (expect: write, writev) ---"
chroot "$CHROOT" /usr/bin/readelf -s /tmp/libfanoff_filter.so 2>/dev/null | grep -iE " write$| writev$" || true

# --- camsiphon: read-only camera frame siphon (separate LD_PRELOAD lib) ---
if [ -f /data/camsiphon.c ]; then
    echo "Building libcamsiphon.so (camera frame siphon)..."
    cp /data/camsiphon.c "$CHROOT/tmp/camsiphon.c"
    chroot "$CHROOT" /usr/bin/gcc-13 $CFLAGS -o /tmp/libcamsiphon.so /tmp/camsiphon.c
    cp "$CHROOT/tmp/libcamsiphon.so" "$OUTDIR/libcamsiphon.so"
    ls -la "$OUTDIR/libcamsiphon.so"
    echo "--- camsiphon version deps (should be EMPTY) ---"
    chroot "$CHROOT" /usr/bin/readelf -V /tmp/libcamsiphon.so 2>/dev/null | grep -i "GLIBC" || echo "  none — good (freestanding)"
    echo "--- camsiphon exports (expect: open, openat, mmap, ioctl) ---"
    chroot "$CHROOT" /usr/bin/readelf --dyn-syms /tmp/libcamsiphon.so 2>/dev/null | grep -iE " (open|openat|mmap|ioctl)$" || true
fi

# --- serialtap: read-only tap of AVA's sensor serial ports (ttyS3 LiDAR + ttyS4 MCU) -> shm rings ---
if [ -f /data/serialtap.c ]; then
    echo "Building libserialtap.so (ttyS3 LiDAR + ttyS4 MCU read-tap)..."
    cp /data/serialtap.c "$CHROOT/tmp/serialtap.c"
    chroot "$CHROOT" /usr/bin/gcc-13 $CFLAGS -o /tmp/libserialtap.so /tmp/serialtap.c
    cp "$CHROOT/tmp/libserialtap.so" "$OUTDIR/libserialtap.so"
    ls -la "$OUTDIR/libserialtap.so"
    echo "--- serialtap version deps (should be EMPTY) ---"
    chroot "$CHROOT" /usr/bin/readelf -V /tmp/libserialtap.so 2>/dev/null | grep -i "GLIBC" || echo "  none — good (freestanding)"
    echo "--- serialtap exports (expect: read, readv) ---"
    chroot "$CHROOT" /usr/bin/readelf --dyn-syms /tmp/libserialtap.so 2>/dev/null | grep -iE " (read|readv)$" || true
fi

# --- camstream: cedar HW-JPEG MJPEG-over-HTTP server (chroot-native, links vendor encoder) ---
# NOT a shim — a standalone server that runs inside the chroot (glibc 2.39 + vendor libs at
# /opt/venc). Stage the vendor CedarX encoder libs into the chroot (reproducible) then build.
if [ -f /data/camstream.c ]; then
    if [ ! -f "$CHROOT/opt/venc/libvencoder.so" ]; then
        echo "Staging vendor CedarX encoder libs -> $CHROOT/opt/venc ..."
        mkdir -p "$CHROOT/opt/venc"
        for l in libvencoder libvenc_codec libvenc_base libawh264 libVE libMemAdapter \
                 libcdc_base libcdx_base libcdx_common libOmxVenc; do
            cp -f /usr/lib/$l.so "$CHROOT/opt/venc/" 2>/dev/null
        done
    fi
    echo "Building camstream (cedar MJPEG server) -> /opt/camstream ..."
    cp /data/camstream.c "$CHROOT/tmp/camstream.c"
    chroot "$CHROOT" /usr/bin/gcc-13 -O2 -w /tmp/camstream.c -L/opt/venc \
        -lvencoder -lvenc_codec -lvenc_base -lMemAdapter -lVE -lcdc_base \
        -Wl,-rpath,/opt/venc -o /opt/camstream \
        && ls -la "$CHROOT/opt/camstream" || echo "  camstream build FAILED"
fi

#!/bin/bash
# Build the IMX296 kernel driver + a device-tree overlay for the Radxa Dragon Q6A (QCS6490, Hexagon v68).
# Usage:  ./build_imx296.sh [CAM]      CAM = 2 (default) or 3  -> the 15-pin RPi-CSI connector used.
# Run ON the Q6A. Needs: linux-headers-radxa-dragon-q6a, gcc, make, dtc, curl (all on Radxa OS).
#
# Why: the stock image ships CAMSS + imx219/imx412(577)/imx214 but NOT imx296, and its shipped camera
# overlays BOOT-LOOP the board (see docs/q6a-camera.md). This builds the missing driver + a fixed overlay.
# The per-cam overlay .dts files (committed next to this script) already contain all the fixes:
#   no linux,cma fragment | sony,imx296@0x1a | clock-names="inck" 37.125MHz | data-lanes=<1> | link 594MHz.
set -euo pipefail
CAM="${1:-2}"; [[ "$CAM" =~ ^[23]$ ]] || { echo "CAM must be 2 or 3"; exit 1; }
KVER="$(uname -r)"; BUILD="/lib/modules/$KVER/build"; HERE="$(cd "$(dirname "$0")" && pwd)"
OVL="qcs6490-radxa-dragon-q6a-cam${CAM}-imx296"

echo "== 1. fetch mainline imx296.c matching kernel $KVER =="
TAG="v$(echo "$KVER" | grep -oE '^[0-9]+\.[0-9]+')"   # v6.18 tag matches Radxa 6.18.x
[ -f imx296.c ] || curl -fsSL -o imx296.c \
  "https://raw.githubusercontent.com/torvalds/linux/$TAG/drivers/media/i2c/imx296.c"

echo "== 2. build + install imx296.ko =="
printf 'obj-m += imx296.o\n' > Makefile
make -C "$BUILD" M="$HERE" modules
sudo cp imx296.ko "/lib/modules/$KVER/kernel/drivers/media/i2c/"; sudo depmod -a

echo "== 3. compile the CAM${CAM} overlay =="
dtc -@ -I dts -O dtb -o "$OVL.dtbo" "$OVL.dts" 2>/dev/null
echo "Built $OVL.dtbo — deploy with:  ./deploy_imx296.sh $CAM"

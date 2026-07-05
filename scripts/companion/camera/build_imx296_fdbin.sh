#!/bin/bash
# Build + install the FD-binning-fixed imx296 kernel module ON the Q6A.
# Run from the Odyssey: ./build_imx296_fdbin.sh
#
# WHY: mainline imx296.c supports 2x2 FD binning (crop=full + half-size format ->
# CTRL0D HADD|FD_BINNING) but NEVER programs MIPIC_AREA3W (0x4182), the MIPI TX
# active-line count. It stays at the 1088 power-on default, so when the sensor
# only emits 544 binned lines qcom-camss waits forever for frame-end -> STREAMON
# hangs. The patch writes MIPIC_AREA3W = format->height in imx296_setup (correct
# for full-res=1088, crop, HADD, and FD binning=544). Verified: 2x2 728x544
# streams at full rate. Fetches the EXACT v6.18 source to match the running ABI.
set -euo pipefail
Q6A="${1:-ippolit-lan}"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== copy patch to the Q6A =="
scp -q "$HERE/imx296_fdbin.patch" "$Q6A:~/imx296_fdbin.patch"

ssh "$Q6A" 'set -e
KREL=$(uname -r)
[ -d /lib/modules/$KREL/build ] || { echo "no kernel headers for $KREL (apt install linux-headers-$KREL)"; exit 1; }
# derive the kernel version tag (e.g. 6.18.2 -> v6.18) for the mainline source fetch
KV=$(echo "$KREL" | grep -oP "^[0-9]+\.[0-9]+")
rm -rf ~/imx296-build && mkdir -p ~/imx296-build && cd ~/imx296-build
echo "== fetch mainline imx296.c (v$KV) =="
curl -fsSL "https://raw.githubusercontent.com/torvalds/linux/v$KV/drivers/media/i2c/imx296.c" -o imx296.c
echo "== apply FD-binning (MIPIC_AREA3W) patch =="
patch -p1 --fuzz=3 < ~/imx296_fdbin.patch || patch < ~/imx296_fdbin.patch
cat > Makefile <<EOF
obj-m += imx296.o
KDIR := /lib/modules/\$(shell uname -r)/build
all:
	\$(MAKE) -C \$(KDIR) M=\$(PWD) modules
EOF
echo "== build =="
make 2>&1 | tail -4
KO=/lib/modules/$KREL/kernel/drivers/media/i2c/imx296.ko
[ -f ~/imx296.ko.orig ] || sudo cp "$KO" ~/imx296.ko.orig   # one-time backup of the stock module
echo "== install + depmod + reload =="
sudo cp imx296.ko "$KO"
sudo depmod -a
echo 18-001a | sudo tee /sys/bus/i2c/drivers/imx296/unbind >/dev/null 2>&1 || true
sudo rmmod imx296 2>/dev/null || true
sudo modprobe imx296
sleep 2
echo "== done: $(modinfo imx296 | grep filename) =="
'
echo "FD-binning driver installed. Stream with: q6a_camstream.py --gpu --sensor-bin --destripe"

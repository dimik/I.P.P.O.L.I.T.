#!/bin/sh
# usb_ncm_gadget.sh — bring up a CDC-NCM USB-Ethernet gadget on the robot OTG port.
#
# Same correct Allwinner sun50iw10 BSP gadget ABI as the ECM build (dma_flag in
# struct usb_request, *f in usb_function_instance) so the modules match the
# robot's BUILT-IN composite framework. NCM aggregates frames per transfer ->
# higher throughput than ECM. vermagic = "4.9.191 SMP preempt mod_unload aarch64".
#
# RAM-only: modules in /tmp, configfs volatile, IP runtime. Reboot wipes it all;
# nothing touches eMMC. If bind crashes, watchdog reboots back to normal.
set -e

MODDIR=/tmp
G=/sys/kernel/config/usb_gadget/ncm

UDC=$(ls /sys/class/udc | head -1)
echo "[*] UDC = $UDC"
[ -n "$UDC" ] || { echo "no UDC found"; exit 1; }
[ "$(cat /sys/class/udc/$UDC/state)" = "not attached" ] || \
  echo "  WARN: UDC already in state $(cat /sys/class/udc/$UDC/state)"

# 0) ensure configfs is mounted
mount | grep -q 'configfs on /sys/kernel/config' || mount -t configfs none /sys/kernel/config

# 1) load function modules (u_ether first — usb_f_ncm depends on it)
lsmod | grep -q '^u_ether'   || insmod $MODDIR/u_ether.ko
lsmod | grep -q '^usb_f_ncm' || insmod $MODDIR/usb_f_ncm.ko
lsmod | grep -E 'u_ether|usb_f_ncm'

# 2) build the gadget
mkdir -p $G
echo 0x1d6b > $G/idVendor          # Linux Foundation
echo 0x0104 > $G/idProduct         # Multifunction Composite Gadget
echo 0x0100 > $G/bcdDevice
echo 0x0200 > $G/bcdUSB
mkdir -p $G/strings/0x409
echo "dreame-cortex" > $G/strings/0x409/manufacturer
echo "robot-ncm0"    > $G/strings/0x409/product
echo "0123456789"    > $G/strings/0x409/serialnumber

mkdir -p $G/functions/ncm.usb0
mkdir -p $G/configs/c.1/strings/0x409
echo "CDC NCM" > $G/configs/c.1/strings/0x409/configuration
echo 250 > $G/configs/c.1/MaxPower
ln -sf $G/functions/ncm.usb0 $G/configs/c.1/

# 3) bind to the UDC  <-- the moment of truth (this is where mainline modules crashed)
echo "[*] binding to UDC ..."
echo "$UDC" > $G/UDC
sleep 1
echo "[+] bound: state=$(cat /sys/class/udc/$UDC/state)"

# 4) bring up robot-side interface
IF=$(cat $G/functions/ncm.usb0/ifname)
echo "[*] gadget iface = $IF"
ip addr add 192.168.10.1/24 dev "$IF" 2>/dev/null || true
ip link set "$IF" up
echo "[+] $IF = 192.168.10.1/24 up"
ip -o addr show "$IF"

cat <<EOF

On the Q6A (USB host): cdc_ncm enumerates -> set 192.168.10.2/24 -> ping 192.168.10.1
Teardown (reboot-safe): echo '' > $G/UDC; rm -rf $G; rmmod usb_f_ncm u_ether
EOF

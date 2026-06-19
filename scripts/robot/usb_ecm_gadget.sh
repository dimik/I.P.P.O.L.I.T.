#!/bin/sh
# usb_ecm_gadget.sh — bring up a CDC-ECM USB-Ethernet gadget on the robot OTG port.
#
# Modules are built against the Allwinner sun50iw10 BSP gadget ABI (dma_flag in
# struct usb_request, *f in usb_function_instance) so they match the robot's
# BUILT-IN composite framework — unlike the earlier mainline build that crashed
# at bind. vermagic = "4.9.191 SMP preempt mod_unload aarch64" (byte-exact).
#
# EVERYTHING here is RAM-only: modules load from /tmp, configfs is volatile, the
# IP is runtime. A reboot wipes all of it — nothing is written to eMMC. If the
# bind crashes, the watchdog reboots and the robot returns to normal (as before).
#
# Run AFTER copying the 3 .ko to /tmp on the robot. Run as root on the robot.
set -e

MODDIR=/tmp
UDC=$(ls /sys/class/udc | head -1)          # expect 5100000.udc-controller
G=/sys/kernel/config/usb_gadget/ecm

echo "[*] UDC = $UDC"
[ -n "$UDC" ] || { echo "no UDC found"; exit 1; }

# 1) load function modules (u_ether first — usb_f_ecm depends on it)
insmod $MODDIR/u_ether.ko    2>/dev/null || echo "  u_ether already loaded?"
insmod $MODDIR/usb_f_ecm.ko  2>/dev/null || echo "  usb_f_ecm already loaded?"
lsmod | grep -E 'u_ether|usb_f_ecm' || true

# 2) build the gadget via configfs
mkdir -p /sys/kernel/config/usb_gadget
mkdir -p $G
echo 0x1d6b > $G/idVendor          # Linux Foundation
echo 0x0104 > $G/idProduct         # Multifunction Composite Gadget
echo 0x0100 > $G/bcdDevice
echo 0x0200 > $G/bcdUSB
mkdir -p $G/strings/0x409
echo "dreame-cortex"   > $G/strings/0x409/manufacturer
echo "robot-ecm0"      > $G/strings/0x409/product
echo "0123456789"      > $G/strings/0x409/serialnumber

mkdir -p $G/functions/ecm.usb0
mkdir -p $G/configs/c.1/strings/0x409
echo "CDC ECM" > $G/configs/c.1/strings/0x409/configuration
echo 250 > $G/configs/c.1/MaxPower
ln -sf $G/functions/ecm.usb0 $G/configs/c.1/

# 3) bind to the UDC  <-- this is the step that crashed with mainline modules
echo "[*] binding to UDC (the moment of truth)..."
echo "$UDC" > $G/UDC
echo "[+] bound OK"

# 4) bring up the robot-side interface
IF=$(cat $G/functions/ecm.usb0/ifname)
echo "[*] gadget iface = $IF"
ip addr add 192.168.10.1/24 dev "$IF" || true
ip link set "$IF" up
echo "[+] $IF = 192.168.10.1/24 up"

echo
echo "On the Q6A (USB host): cdc_ether should enumerate -> assign 192.168.10.2/24,"
echo "then: ping 192.168.10.1"
echo
echo "To tear down (also reboot-safe):"
echo "  echo '' > $G/UDC; rm -rf $G; rmmod usb_f_ecm u_ether"

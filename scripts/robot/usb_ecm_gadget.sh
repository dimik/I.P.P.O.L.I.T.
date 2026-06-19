#!/bin/sh
# usb_ecm_gadget.sh — bring up a CDC-ECM USB-Ethernet gadget on the robot OTG port.
#
# Modules are built against the Allwinner sun50iw10 BSP gadget ABI (dma_flag in
# struct usb_request, *f in usb_function_instance) so they match the robot's
# BUILT-IN composite framework. vermagic = "4.9.191 SMP preempt mod_unload aarch64".
#
# ECM is the PREFERRED default over NCM here: identical throughput (~11-12 MB/s,
# sw_udc-bound) but 5x lower latency (0.5 ms vs 2.7 ms) — NCM's aggregation buys
# nothing because the UDC caps below where it helps. See docs/usb-gadget.md.
#
# RAM-only: modules in /tmp, configfs volatile, IP runtime. Reboot wipes it all;
# nothing touches eMMC. If bind crashes, watchdog reboots back to normal.
#
# Usage:  usb_ecm_gadget.sh          # set up + bind
#         usb_ecm_gadget.sh down     # tear down (proper configfs order)
set -e

MODDIR=/tmp
G=/sys/kernel/config/usb_gadget/ecm
DEV_MAC=46:bb:2c:4c:0d:4b      # robot-side usb0 MAC
HOST_MAC=d6:7f:fa:3a:49:bd     # host-side iface MAC (-> enx<HOST_MAC> on the host)

teardown() {
  echo "" > $G/UDC 2>/dev/null || true
  sleep 1
  rm -f  $G/configs/c.1/ecm.usb0 2>/dev/null || true
  rmdir  $G/configs/c.1/strings/0x409 $G/configs/c.1 \
         $G/functions/ecm.usb0 $G/strings/0x409 $G 2>/dev/null || true
  rmmod usb_f_ecm 2>/dev/null || true
  echo "[+] torn down"
}

[ "$1" = "down" ] && { teardown; exit 0; }

UDC=$(ls /sys/class/udc | head -1)
echo "[*] UDC = $UDC"
[ -n "$UDC" ] || { echo "no UDC found"; exit 1; }

# 0) configfs mounted
mount | grep -q 'configfs on /sys/kernel/config' || mount -t configfs none /sys/kernel/config

# 1) load function modules (u_ether first — usb_f_ecm depends on it)
lsmod | grep -q '^u_ether'   || insmod $MODDIR/u_ether.ko
lsmod | grep -q '^usb_f_ecm' || insmod $MODDIR/usb_f_ecm.ko
lsmod | grep -E 'u_ether|usb_f_ecm'

# 2) build the gadget (pin MACs -> stable host iface name)
mkdir -p $G
echo 0x1d6b > $G/idVendor          # Linux Foundation
echo 0x0104 > $G/idProduct         # Multifunction Composite Gadget
echo 0x0100 > $G/bcdDevice
echo 0x0200 > $G/bcdUSB
mkdir -p $G/strings/0x409
echo "dreame-cortex" > $G/strings/0x409/manufacturer
echo "robot-ecm0"    > $G/strings/0x409/product
echo "0123456789"    > $G/strings/0x409/serialnumber

mkdir -p $G/functions/ecm.usb0
echo "$DEV_MAC"  > $G/functions/ecm.usb0/dev_addr
echo "$HOST_MAC" > $G/functions/ecm.usb0/host_addr
mkdir -p $G/configs/c.1/strings/0x409
echo "CDC ECM" > $G/configs/c.1/strings/0x409/configuration
echo 250 > $G/configs/c.1/MaxPower
ln -sf $G/functions/ecm.usb0 $G/configs/c.1/

# 2b) force OTG into peripheral mode (sunxi: READING this file triggers the role).
cat /sys/devices/platform/soc/usbc0/usb_device >/dev/null 2>&1 || true

# 3) bind to the UDC  <-- mainline-ABI modules crash here; BSP-ABI modules don't
echo "[*] binding to UDC ..."
echo "$UDC" > $G/UDC
sleep 1
echo "[+] bound: state=$(cat /sys/class/udc/$UDC/state)"

# 4) robot-side interface + static ARP (NCM/ECM unreliable at broadcast/ARP)
IF=$(cat $G/functions/ecm.usb0/ifname)
ip addr add 192.168.10.1/24 dev "$IF" 2>/dev/null || true
ip link set "$IF" up
arp -s 192.168.10.2 "$HOST_MAC" 2>/dev/null || true
echo "[+] $IF = 192.168.10.1/24 up (mac $(cat /sys/class/net/$IF/address))"

cat <<EOF

On the HOST (USB host side), with the micro-USB cable in (VBUS jumper not needed):
  IF=enx\$(echo $HOST_MAC | tr -d :)        # = enxd67ffa3a49bd
  sudo nmcli device set \$IF managed no      # stop NetworkManager flushing the IP
  sudo ip addr add 192.168.10.2/24 dev \$IF
  sudo ip link set \$IF up
  sudo ip neigh replace 192.168.10.1 lladdr $DEV_MAC dev \$IF   # static ARP
  ping 192.168.10.1
Teardown: $0 down
EOF

#!/bin/sh
# Late boot hook — runs after all Dreame services start
# Handles: WiFi DHCP, Ubuntu chroot mounts, Valetudo startup

LOG=/tmp/postboot.log
exec >> "$LOG" 2>&1
echo "=== _root_postboot.sh start $(date) ==="

sleep 30
echo "30s sleep done"

# --- AVA IoT connection ---
echo -n "miiot" > /data/config/ava/iot.flag
sleep 1
IOT_RESULT=$(avacmd iot '{"type":"iot", "notify":"open_server"}' 2>/dev/null || echo "failed")
echo "AVA IoT open_server: $IOT_RESULT"
logger -t postboot "AVA IoT server connection triggered"

# --- WiFi ---
echo "checking WiFi..."
STATUS=$(wpa_cli -iwlan0 status 2>/dev/null | grep "^wpa_state" | cut -d= -f2)
echo "wpa_state=$STATUS"
logger -t postboot "wpa_state=$STATUS"

if [ "$STATUS" = "COMPLETED" ]; then
    udhcpc -i wlan0 -b -p /var/run/udhcpc.wlan0.pid 2>/dev/null
    echo "DHCP started on wlan0"
    logger -t postboot "DHCP started"
else
    echo "Not connected ($STATUS) - killing hostapd and retrying..."
    logger -t postboot "Not connected ($STATUS) - killing hostapd and retrying"
    killall -9 hostapd 2>/dev/null; sleep 2
    ifconfig wlan0 down; sleep 1; ifconfig wlan0 up
    killall -9 wpa_supplicant 2>/dev/null; sleep 1
    mkdir -p /var/run/wpa_supplicant
    wpa_supplicant -Dnl80211 -B -s -iwlan0 -c/etc/wifi/wpa_supplicant.conf
    echo "wpa_supplicant restarted, waiting 20s..."
    sleep 20
    STATUS2=$(wpa_cli -iwlan0 status 2>/dev/null | grep "^wpa_state" | cut -d= -f2)
    echo "wpa_state after retry=$STATUS2"
    [ "$STATUS2" = "COMPLETED" ] && udhcpc -i wlan0 -b -p /var/run/udhcpc.wlan0.pid 2>/dev/null
fi
echo "IP after WiFi setup: $(ip addr show wlan0 2>/dev/null | grep 'inet ' | awk '{print $2}')"

# --- Ubuntu chroot mounts ---
echo "mounting chroot filesystems..."
logger -t postboot "mounting chroot filesystems"
CHROOT=/data/chroot
mount -t proc proc $CHROOT/proc 2>/dev/null || true
mount -t sysfs sysfs $CHROOT/sys 2>/dev/null || true
mount --bind /dev $CHROOT/dev 2>/dev/null || true
mount --bind /dev/pts $CHROOT/dev/pts 2>/dev/null || true
cp /etc/resolv.conf $CHROOT/etc/resolv.conf 2>/dev/null || true
echo "chroot mounts done"
logger -t postboot "chroot mounts done"

# --- work_mode check ---
WMODE=$(avacmd msg_cvt '{"type":"msgCvt","cmd":"get_prop","prop":"work_mode"}' 2>/dev/null)
echo "work_mode at postboot start: $WMODE"
logger -t postboot "work_mode=$WMODE"

# --- Valetudo ---
sleep 5
echo "starting Valetudo..."
logger -t postboot "starting Valetudo"
(while true; do
    VALETUDO_CONFIG_PATH=/data/valetudo_config/valetudo.json /data/valetudo >> /tmp/valetudo.log 2>&1
    echo "Valetudo exited at $(date), restarting..."
    logger -t postboot "Valetudo exited, restarting in 5s..."
    sleep 5
done) &

# Vacuum fan is suppressed at the MCU level by the LD_PRELOAD SetCleaning filter
# (preloaded onto AVA in _root.sh). No fan_speed/only_mop workarounds needed here.
echo "=== _root_postboot.sh complete $(date) ==="

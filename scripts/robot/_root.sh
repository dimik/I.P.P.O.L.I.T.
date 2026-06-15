#!/bin/sh
# Early boot hook — runs before wifi.sh and factory_ap.sh.
# See docs/wifi-hack.md for full WiFi bind-mount explanation.

LOG=/tmp/root_sh.log
exec >> "$LOG" 2>&1
echo "=== _root.sh start $(date) ==="

# Vacuum fan disable — preload the MCU SetCleaning filter into AVA so the vacuum fan
# never spins in manual nav / cleaning. The shim zeros the SetCleaning f3 byte on the
# wire to /dev/ttyS4 and recomputes the CRC. See CLAUDE.md "Vacuum fan disable".
# Must be established before app start runs /etc/rc.d/ava.sh (this is the early hook).
FANOFF=/data/lib/libfanoff_filter.so
if [ -f "$FANOFF" ]; then
    head -1 /etc/rc.d/ava.sh > /data/ava.sh.preload
    echo "export LD_PRELOAD=$FANOFF" >> /data/ava.sh.preload
    tail -n +2 /etc/rc.d/ava.sh >> /data/ava.sh.preload
    chmod +x /data/ava.sh.preload
    mount --bind /data/ava.sh.preload /etc/rc.d/ava.sh
    echo "fanoff: LD_PRELOAD shim bind-mounted onto ava.sh"
    logger -t root_sh "fanoff shim preload bind-mounted"
fi

# Prevent miio_client from entering WiFi AP (provisioning) mode on every boot.
# Without /etc/miio/wifi.conf, miio_client_helper_nomqtt.sh sends params:0
# (not configured) → miio_client → STATE_WIFI_AP_MODE → work_mode 17 (RemoteCtrlMode).
# /etc/miio/wifi.conf does not exist in squashfs; bind-mount the whole directory.
mkdir -p /data/config/miio
[ ! -f /data/config/miio/device.conf ] && cp /etc/miio/device.conf /data/config/miio/device.conf 2>/dev/null
[ ! -f /data/config/miio/device.token ] && cp /etc/miio/device.token /data/config/miio/device.token 2>/dev/null
[ ! -f /data/config/miio/wifi.conf ] && printf 'ssid="configured"\nkey_mgmt=WPA\n' > /data/config/miio/wifi.conf
mount --bind /data/config/miio /etc/miio
echo "miio dir bind-mounted: $(ls /etc/miio/)"
logger -t root_sh "miio dir bind mounted"

sleep 1
mkdir -p /data/config/wifi

# Write wpa_supplicant config only if it doesn't already exist with real credentials.
# To reset credentials: rm /data/config/wifi/wpa_supplicant.conf and reboot.
if ! grep -q 'ssid="4K"' /data/config/wifi/wpa_supplicant.conf 2>/dev/null; then
    echo "Writing wpa_supplicant.conf (home 2.4GHz network)"
    cat > /data/config/wifi/wpa_supplicant.conf << WPAEOF
ctrl_interface=/var/run/wpa_supplicant/
disable_scan_offload=1
update_config=0
wowlan_triggers=any

network={
    ssid="4K"
    psk=c6b3ae674bb363ca852a4174b7cae956a22ff7c63c96fcb794068fb9fa64b7e8
    key_mgmt=WPA-PSK
    proto=WPA WPA2
    scan_ssid=1
}
WPAEOF
fi

mount --bind /data/config/wifi/wpa_supplicant.conf /etc/wifi/wpa_supplicant.conf
echo "wpa_supplicant.conf bind-mounted: ssid=$(grep 'ssid=' /etc/wifi/wpa_supplicant.conf | head -1)"
logger -t root_sh "wpa_supplicant.conf bind mounted"
echo "=== _root.sh done ==="

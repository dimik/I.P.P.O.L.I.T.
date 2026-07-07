#!/bin/sh
# Late boot hook — runs after all Dreame services start
# Handles: WiFi DHCP, Ubuntu chroot mounts, Valetudo startup

LOG=/tmp/postboot.log
exec >> "$LOG" 2>&1
echo "=== _root_postboot.sh start $(date) ==="

sleep 30
echo "30s sleep done"

# --- CPU power-save: the stock firmware pins all 4 cores at 1.416GHz 24/7 (userspace governor),
# so even sitting idle the SoC burns ~full power (battery ~12h from 100%). This unit is a rover, not
# a vacuum (no cleaning), so AVA's pinned performance is unneeded. Switch to ondemand: the CPU idles
# at 408MHz when still and auto-ramps to 1.5GHz under load (ROS/nav/vision). Verified AVA does not
# re-pin it. Biggest single battery win. See docs/power.md.
for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo ondemand > "$c" 2>/dev/null; done
echo "CPU governor -> ondemand ($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null))"
logger -t postboot "cpu governor -> ondemand (idle downclock)"

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

# --- USB gadget-Ethernet (CDC-ECM) + DHCP for the companion (Q6A) on usb0 ---
# Brings up the BSP-ABI ethernet gadget on the OTG port (robot=192.168.10.1) and runs a dnsmasq
# bound ONLY to usb0 so the companion is plug-and-play (DHCP, gets 192.168.10.2). Modules + script
# live on /data (persistent). All defensive: failures here must not abort the rest of postboot.
# See docs/usb-gadget.md.  Caveat: WiFi AP mode does `killall -9 dnsmasq` (kills this too).
GDIR=/data/usb-gadget
if [ -f "$GDIR/usb_ecm_gadget.sh" ] && [ -f "$GDIR/usb_f_ecm.ko" ]; then
    echo "bringing up USB-ECM gadget..."
    MODDIR="$GDIR" sh "$GDIR/usb_ecm_gadget.sh" > /tmp/usb_gadget.log 2>&1 || echo "usb gadget setup returned nonzero (see /tmp/usb_gadget.log)"
    if [ -e /sys/class/net/usb0 ]; then
        [ -f /tmp/dnsmasq-usb0.pid ] && kill "$(cat /tmp/dnsmasq-usb0.pid)" 2>/dev/null
        dnsmasq --conf-file=/dev/null --user=root --port=0 --interface=usb0 --bind-interfaces \
                --except-interface=lo --dhcp-authoritative \
                --dhcp-range=192.168.10.2,192.168.10.2,255.255.255.0 \
                --dhcp-leasefile=/tmp/dnsmasq-usb0.leases --pid-file=/tmp/dnsmasq-usb0.pid \
            && echo "usb0 dnsmasq (DHCP for companion) started" \
            || echo "usb0 dnsmasq failed to start"
        logger -t postboot "usb0 gadget + dnsmasq up"
    else
        echo "usb0 not present after gadget setup — skipping dnsmasq"
    fi
else
    echo "USB gadget not staged in $GDIR — skipping"
fi

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

# --- camera MJPEG stream (cedar HW-JPEG, client-gated) ---
# Always-listening on :8090; camstream sets /tmp/cam_stream (camsiphon ring fill) only while a
# viewer is connected, so it's zero AVA overhead when idle. View: http://<robot-ip>:8090/
# Needs the stream-capable libcamsiphon preloaded (via _root.sh) + /opt/venc + /opt/camstream
# in the chroot (built by build_ava_shims.sh). See docs/sensors.md.
mount --bind /tmp $CHROOT/tmp 2>/dev/null || true        # share the camsiphon ring into the chroot
if [ -x $CHROOT/opt/camstream ]; then
    setsid chroot $CHROOT sh -c "LD_LIBRARY_PATH=/opt/venc /opt/camstream 8090" > /tmp/camstream.log 2>&1 </dev/null &
    echo "camstream MJPEG server started on :8090"
    logger -t postboot "camstream MJPEG server started on :8090"
fi

# --- H.264 / RTSP / WebRTC restream (go2rtc, on-robot, software libx264) ---
# Pulls camstream's MJPEG on demand and transcodes to H.264 (no HW H.264 on this device — cedar
# encoder is locked, no V4L2 M2M encoder). On-demand: no viewer -> no transcode -> camsiphon off.
#   RTSP rtsp://<ip>:8554/dreame   WebRTC http://<ip>:1984/
if [ -x $CHROOT/opt/go2rtc ] && [ -x $CHROOT/opt/ffmpeg ]; then
    setsid chroot $CHROOT sh -c "/opt/go2rtc -config /opt/go2rtc.yaml" > /tmp/go2rtc.log 2>&1 </dev/null &
    echo "go2rtc H.264/RTSP/WebRTC started (:8554 / :1984)"
    logger -t postboot "go2rtc started (:8554 / :1984)"
fi

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

# --- ROS RELOCATED TO THE COMPANION (Q6A), 2026-07-07 (phase 1.3) ---
# valetudo_bridge, lds_scan_node, mcu_node now run on the Q6A over the USB link (systemd units there).
# The robot keeps only the LD_PRELOAD serial taps + these ROS-free ring_forward.py pumps, which stream the
# tap rings' raw bytes over TCP to the companion ROS nodes. No ROS runs on the robot for map/scan/imu/odom.
# LiDAR ring (ttyS3) -> tcp/9901 -> Q6A lds-scan-node.service -> /scan
if [ -f $CHROOT/opt/ring_forward.py ]; then
    setsid chroot $CHROOT python3 /opt/ring_forward.py --path /tmp/lds_ring.buf --port 9901 --magic 0x0031534444530001 > /tmp/ringfwd_lds.log 2>&1 </dev/null &
    echo "ring_forward LDS (tcp/9901) started"
    logger -t postboot "ring_forward LDS started"
fi
# MCU ring (ttyS4) -> tcp/9902 -> Q6A mcu-node.service -> /imu/data + /odom/wheel
if [ -f $CHROOT/opt/ring_forward.py ]; then
    setsid chroot $CHROOT python3 /opt/ring_forward.py --path /tmp/mcu_ring.buf --port 9902 --magic 0x0031534444530001 > /tmp/ringfwd_mcu.log 2>&1 </dev/null &
    echo "ring_forward MCU (tcp/9902) started"
    logger -t postboot "ring_forward MCU started"
fi

# --- audio bridge: RELOCATED to the companion (Q6A audio-bridge.service), 2026-07-08 (phase 1.3c) ---
# The ROS /robot/speak subscription now runs on the Q6A; it pipes each utterance to the robot's ROS-free
# /opt/speak.py over ssh (piper/espeak + ffmpeg -> localhost mediad). No ROS on the robot for audio.

# --- charge_state poller: Valetudo's battery charging FLAG is stuck 'none' for the D10S Pro (mapping
# gap); AVA reports the truth via charge_state. Poll it (host avacmd) into /tmp/charge_state so the
# chroot valetudo_bridge can publish a correct /battery. See docs/sensors.md (Battery / charging).
(while true; do
    v=$(avacmd msg_cvt '{"type":"msgCvt","cmd":"get_prop","prop":"charge_state"}' 2>/dev/null | sed -n 's/.*"value":"\([^"]*\)".*/\1/p')
    [ -n "$v" ] && echo "$v" > /tmp/charge_state
    sleep 15
done) &
echo "charge_state poller started"

# LiDAR gate for the fanoff shim. The shim (preloaded onto AVA in _root.sh) always filters the
# vacuum fan; this daemon allows the LiDAR turret to run in active non-manual modes and blocks
# it during manual navigation (creates/removes /tmp/lidar_allow from Valetudo status). The fan
# is unconditional, so there is no race on the loud motor; the LiDAR is blocked-by-default so
# manual nav has no turret spin-up blip either.
echo "starting fanoff LiDAR gate..."
(setsid sh /data/fanoff_flag.sh </dev/null >/dev/null 2>&1 &)
logger -t postboot "fanoff LiDAR gate started"
echo "=== _root_postboot.sh complete $(date) ==="

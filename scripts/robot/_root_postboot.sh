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

# --- Valetudo -> ROS 2 bridge (map / odom / pose / status; no MQTT broker) ---
# Publishes /map (OccupancyGrid), /odom + map->base_link TF, /robot/status from Valetudo's HTTP
# API into the chroot's ROS 2 Jazzy. Retries until Valetudo's API is up. See valetudo_bridge.py.
if [ -f $CHROOT/opt/valetudo_bridge.py ]; then
    setsid chroot $CHROOT bash -lc 'source /opt/ros/jazzy/setup.bash; exec python3 /opt/valetudo_bridge.py' > /tmp/vbridge.log 2>&1 </dev/null &
    echo "valetudo_bridge (ROS) started"
    logger -t postboot "valetudo_bridge (ROS) started"
fi

# --- LiDAR -> ROS /scan (libserialtap shm ring -> sensor_msgs/LaserScan) ---
# libserialtap.so (preloaded onto AVA in _root.sh) tees AVA's ttyS3 reads to /tmp/lds_ring.buf;
# this node decodes LDS frames and publishes /scan. Zero cost when the turret is gated off
# (no ttyS3 reads -> empty ring). The /tmp bind-mount above shares the ring into the chroot.
# See scripts/robot/lds_scan_node.py + docs/ros.md / docs/sensors.md.
if [ -f $CHROOT/opt/lds_scan_node.py ]; then
    setsid chroot $CHROOT bash -lc 'source /opt/ros/jazzy/setup.bash; exec python3 /opt/lds_scan_node.py' > /tmp/lds_scan.log 2>&1 </dev/null &
    echo "lds_scan_node (ROS /scan) started"
    logger -t postboot "lds_scan_node (ROS /scan) started"
fi

# --- MCU -> ROS IMU + wheel odom (libserialtap shm ring -> sensor_msgs/Imu, nav_msgs/Odometry) ---
# libserialtap also tees AVA's ttyS4 (MCU) reads to /tmp/mcu_ring.buf; this node decodes the
# Status10ms/Status20ms frames and publishes /imu/data + /odom/wheel. The MCU streams continuously
# (even docked), so it self-calibrates gyro bias at startup assuming the robot is stationary.
# See scripts/robot/mcu_node.py + docs/sensors.md.
if [ -f $CHROOT/opt/mcu_node.py ]; then
    setsid chroot $CHROOT bash -lc 'source /opt/ros/jazzy/setup.bash; exec python3 /opt/mcu_node.py' > /tmp/mcu_node.log 2>&1 </dev/null &
    echo "mcu_node (ROS /imu/data,/odom/wheel) started"
    logger -t postboot "mcu_node (ROS imu/odom) started"
fi

# LiDAR gate for the fanoff shim. The shim (preloaded onto AVA in _root.sh) always filters the
# vacuum fan; this daemon allows the LiDAR turret to run in active non-manual modes and blocks
# it during manual navigation (creates/removes /tmp/lidar_allow from Valetudo status). The fan
# is unconditional, so there is no race on the loud motor; the LiDAR is blocked-by-default so
# manual nav has no turret spin-up blip either.
echo "starting fanoff LiDAR gate..."
(setsid sh /data/fanoff_flag.sh </dev/null >/dev/null 2>&1 &)
logger -t postboot "fanoff LiDAR gate started"
echo "=== _root_postboot.sh complete $(date) ==="

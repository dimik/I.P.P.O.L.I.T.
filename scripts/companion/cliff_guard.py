#!/usr/bin/env python3
"""cliff_guard.py — SAFETY: drop-off awareness near the 2nd-floor stairwell.

Two very different responses, on purpose:

  * WHEEL-DROP (`/cliff`, MCU Triggers byte[1]) = HARD e-stop. If a wheel has already left the ground the
    robot is going over NOW -> disable HighResolutionManualControl immediately (x3, WiFi can drop a PUT) +
    speak. This is the last-resort backstop.

  * MiDaS floor-drop ahead = ADVISORY, NOT a freeze. The robot must be able to travel *along* an edge at a
    safe distance, not get blocked in front of it, so this layer only PUBLISHES the hazard — it never cuts
    manual control. It republishes `/vision/floor`'s per-sector drop as `/cliff/ahead` (Bool = drop in the
    CENTER path ahead) so the drive controller can refuse to drive *forward* into a drop while still turning,
    reversing, or gliding parallel. Direction lives in `/vision/floor` (sectors left/center/right).

Calibrated at the real ladder edge 2026-07-09: floor `max_step` room <=0.205 vs edge 0.35-0.65 -> center
threshold 0.30 (fuse 0.24 when the forward LiDAR sector is anomalously open = stairwell signature).

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 cliff_guard.py
"""
import json
import math
import os
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.10.1')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'

MIDAS_STOP = float(os.environ.get('Q6A_CLIFF_MIDAS_STOP', '0.30'))   # center-sector drop = no-go-forward
MIDAS_FUSE = float(os.environ.get('Q6A_CLIFF_MIDAS_FUSE', '0.24'))   # weaker, needs LiDAR agreement
MIDAS_CLEAR = float(os.environ.get('Q6A_CLIFF_MIDAS_CLEAR', '0.22')) # re-arm hysteresis
CONSEC = int(os.environ.get('Q6A_CLIFF_CONSEC', '2'))
CLEAR_N = int(os.environ.get('Q6A_CLIFF_CLEAR_N', '6'))
LIDAR_FAR_M = float(os.environ.get('Q6A_CLIFF_LIDAR_FAR_M', '3.5'))
LIDAR_HALF_DEG = float(os.environ.get('Q6A_CLIFF_LIDAR_HALF_DEG', '20'))
LIDAR_STALE_S = 2.0
SPEAK_GAP = float(os.environ.get('Q6A_CLIFF_SPEAK_GAP', '8.0'))      # min seconds between spoken edge warnings


class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')
        self.pub_speak = self.create_publisher(String, '/robot/speak', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ahead = self.create_publisher(Bool, '/cliff/ahead', latched)   # drop in the forward path
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.create_subscription(Bool, '/bumper', self.on_bumper, latched)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.bumped = False
        self.last_bump = 0.0
        self.wheel_tripped = False
        self.ahead = False
        self.n_hit = self.n_clear = 0
        self.lidar_far = False
        self.lidar_at = 0.0
        self.last_warn = 0.0
        self.pub_ahead.publish(Bool(data=False))
        self.get_logger().info(
            f'cliff_guard up: wheel-drop -> HARD e-stop; MiDaS center-drop (>={MIDAS_STOP}, or >={MIDAS_FUSE}'
            f'+LiDAR-open) -> ADVISORY /cliff/ahead (never freezes; drive loop avoids forward). @ {ROBOT_ADDR}')

    # --- HARD e-stop: a wheel is already off the ground ---
    def on_cliff(self, m):
        if m.data and not self.wheel_tripped:
            self.wheel_tripped = True
            self.get_logger().error('WHEEL-DROP — hard e-stop (disable manual control)')
            threading.Thread(target=self.estop, daemon=True).start()
            self.pub_speak.publish(String(data='Cliff. Stopping.'))
        elif not m.data and self.wheel_tripped:
            self.wheel_tripped = False
            self.get_logger().warn('wheel-drop cleared')

    def on_bumper(self, m):
        # audible confirmation the bumper fired (front collision). Recovery/back-off is the drive
        # controller's job (q6a_drive); here we just say "Ouch!" on each fresh hit (throttled).
        if m.data and not self.bumped:
            self.bumped = True
            now = time.monotonic()
            if now - self.last_bump > 1.5:
                self.last_bump = now
                self.pub_speak.publish(String(data='Ouch!'))
                self.get_logger().info('BUMP -> Ouch!')
        elif not m.data:
            self.bumped = False

    def on_scan(self, m):
        half = math.radians(LIDAR_HALF_DEG)
        fin, n_fwd = [], 0
        for i, r in enumerate(m.ranges):
            a = math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                           math.cos(m.angle_min + i * m.angle_increment))
            if abs(a) <= half:
                n_fwd += 1
                if math.isfinite(r) and m.range_min <= r <= m.range_max:
                    fin.append(r)
        if n_fwd:
            fin.sort()
            med = fin[len(fin) // 2] if fin else float('inf')
            self.lidar_far = med > LIDAR_FAR_M or (len(fin) / n_fwd) < 0.3
            self.lidar_at = time.monotonic()

    # --- ADVISORY: drop ahead in the center path (never disables manual control) ---
    def on_floor(self, m):
        try:
            d = json.loads(m.data)
            center = float(d.get('sectors', {}).get('center', [d.get('max_step', 0.0)])[0])
        except Exception:
            return
        lidar_fresh = (time.monotonic() - self.lidar_at) < LIDAR_STALE_S
        hit = center >= MIDAS_STOP or (center >= MIDAS_FUSE and lidar_fresh and self.lidar_far)
        self.n_hit = self.n_hit + 1 if hit else 0
        self.n_clear = self.n_clear + 1 if center < MIDAS_CLEAR else 0
        if self.n_hit >= CONSEC and not self.ahead:
            self.ahead = True
            self.pub_ahead.publish(Bool(data=True))
            self.get_logger().warn(f'drop-off in the forward path (center step={center:.2f}) — /cliff/ahead=1')
            now = time.monotonic()
            if now - self.last_warn > SPEAK_GAP:
                self.last_warn = now
                self.pub_speak.publish(String(data='Edge ahead.'))
        elif self.ahead and self.n_clear >= CLEAR_N:
            self.ahead = False
            self.pub_ahead.publish(Bool(data=False))
            self.get_logger().info('forward path clear — /cliff/ahead=0')

    def estop(self):
        body = json.dumps({'action': 'disable'}).encode()
        for _ in range(3):
            try:
                req = urllib.request.Request(CAP, data=body, method='PUT',
                                             headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=1.0).read()
            except Exception as e:
                self.get_logger().warn(f'estop PUT failed: {e}')


def main():
    rclpy.init()
    node = CliffGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

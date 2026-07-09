#!/usr/bin/env python3
"""cliff_guard.py — SAFETY: stop the robot before (MiDaS+LiDAR) and at (wheel-drop) a stair edge.

The robot lives on the 2nd floor next to a ladder/stairwell. A horizontal 2D LiDAR cannot see a
down-staircase (the drop reads as "open"), and the MCU's Triggers byte only fires once the wheels have
LEFT the ground (wheel-drop — proven by the 2026-07-09 edge test). So before-the-edge protection comes
from fused perception, with the wheel-drop as last-resort backstop:

  1. MiDaS floor-drop (PRIMARY, calibrated 2026-07-09 at the real ladder edge):
     /vision/floor 'max_step' = largest relative fall between adjacent floor-band depth bins.
     Room max 0.205; edge square-on 0.581-0.649; edge re-approached (other angle/distance)
     0.345-0.48 -> STOP at 0.30 (angle-robust, still 1.5x above the room ceiling).
  2. MiDaS + LiDAR fusion: a weaker visual step (>=0.28) counts when the forward LiDAR sector is
     simultaneously anomalously open (median > 3.5 m or mostly no-return) — indoors a wall should
     terminate the beam; "open" toward a suspicious floor edge = stairwell signature.
  3. Wheel-drop backstop: /cliff (mcu_node, MCU Triggers byte[1]) — fires when wheels leave ground.

On any trip: DISABLE HighResolutionManualControl over REST (x3, WiFi can drop a PUT) + speak. Latches per
cause; re-arms with hysteresis when the signal clears. LiDAR stale (turret parked) -> fusion path inert,
MiDaS-primary + wheel-drop still protect.

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

MIDAS_STOP = float(os.environ.get('Q6A_CLIFF_MIDAS_STOP', '0.30'))   # calibrated: room<=0.205, edge>=0.345
MIDAS_FUSE = float(os.environ.get('Q6A_CLIFF_MIDAS_FUSE', '0.24'))   # weaker step, needs LiDAR agreement
MIDAS_CLEAR = float(os.environ.get('Q6A_CLIFF_MIDAS_CLEAR', '0.22')) # re-arm hysteresis
CONSEC = int(os.environ.get('Q6A_CLIFF_CONSEC', '2'))                # consecutive samples to trip
CLEAR_N = int(os.environ.get('Q6A_CLIFF_CLEAR_N', '10'))             # consecutive clear samples to re-arm
LIDAR_FAR_M = float(os.environ.get('Q6A_CLIFF_LIDAR_FAR_M', '3.5'))  # fwd median beyond this = "open"
LIDAR_HALF_DEG = float(os.environ.get('Q6A_CLIFF_LIDAR_HALF_DEG', '20'))
LIDAR_STALE_S = 2.0


class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')
        self.pub_speak = self.create_publisher(String, '/robot/speak', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_danger = self.create_publisher(Bool, '/cliff/ahead', latched)  # for drive loops
        self.wheel_tripped = False
        self.floor_tripped = False
        self.n_stop = self.n_fuse = self.n_clear = 0
        self.lidar_far = False
        self.lidar_at = 0.0
        self.get_logger().info(
            f'cliff_guard up: MiDaS floor-drop (stop>={MIDAS_STOP}, fuse>={MIDAS_FUSE}+LiDAR>{LIDAR_FAR_M}m) '
            f'+ wheel-drop backstop -> DISABLE manual control @ {ROBOT_ADDR}')

    # --- layer 3: wheel-drop backstop (fires when wheels already left the ground) ---
    def on_cliff(self, m):
        if m.data and not self.wheel_tripped:
            self.wheel_tripped = True
            self.get_logger().error('WHEEL-DROP — hard-stopping')
            self.trip('Cliff detected. Stopping.')
        elif not m.data and self.wheel_tripped:
            self.wheel_tripped = False
            self.get_logger().warn('wheel-drop cleared — re-armed')

    # --- layer 2 input: LiDAR forward-sector openness (corroboration only) ---
    def on_scan(self, m):
        half = math.radians(LIDAR_HALF_DEG)
        fin = []
        n_fwd = 0
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))    # wrap to [-pi, pi]; 0 = forward
            if abs(a) <= half:
                n_fwd += 1
                if math.isfinite(r) and m.range_min <= r <= m.range_max:
                    fin.append(r)
        if n_fwd == 0:
            return
        fin.sort()
        med = fin[len(fin) // 2] if fin else float('inf')
        finite_frac = len(fin) / n_fwd
        self.lidar_far = med > LIDAR_FAR_M or finite_frac < 0.3
        self.lidar_at = time.monotonic()

    # --- layers 1+2: MiDaS floor-drop, LiDAR-fused ---
    def on_floor(self, m):
        try:
            step = float(json.loads(m.data).get('max_step', 0.0))
        except Exception:
            return
        self.n_stop = self.n_stop + 1 if step >= MIDAS_STOP else 0
        self.n_fuse = self.n_fuse + 1 if step >= MIDAS_FUSE else 0
        self.n_clear = self.n_clear + 1 if step < MIDAS_CLEAR else 0
        lidar_fresh = (time.monotonic() - self.lidar_at) < LIDAR_STALE_S
        danger = (self.n_stop >= CONSEC or
                  (self.n_fuse >= CONSEC and lidar_fresh and self.lidar_far))
        if danger and not self.floor_tripped:
            self.floor_tripped = True
            self.n_clear = 0
            why = 'midas' if self.n_stop >= CONSEC else 'midas+lidar'
            self.get_logger().error(f'DROP-OFF AHEAD ({why}, step={step:.2f}) — stopping')
            self.trip('Drop off ahead. Stopping.')
            self.pub_danger.publish(Bool(data=True))
        elif self.floor_tripped and self.n_clear >= CLEAR_N:
            self.floor_tripped = False
            self.get_logger().warn('floor-drop cleared — re-armed')
            self.pub_danger.publish(Bool(data=False))

    # --- common action ---
    def trip(self, phrase):
        threading.Thread(target=self.estop, daemon=True).start()
        self.pub_speak.publish(String(data=phrase))

    def estop(self):
        body = json.dumps({'action': 'disable'}).encode()
        for _ in range(3):                 # repeat — a single PUT can drop over WiFi
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

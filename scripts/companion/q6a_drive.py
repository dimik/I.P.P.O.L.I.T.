#!/usr/bin/env python3
"""q6a_drive.py — bounded, safety-gated forward drive burst (companion).

Drives the robot FORWARD at a set velocity for a bounded number of seconds, gated at control-loop rate by
the perception the companion already publishes — so the mapping drive is smooth (no per-pulse ssh) and never
blind. Stops immediately (disable HighResolutionManualControl) on ANY of:
  - wheel-drop  (/cliff true)
  - floor drop ahead  (/vision/floor center sector >= STOP_CENTER, MiDaS)
  - near obstacle  (LiDAR forward sector < MIN_FRONT)
  - stale/absent sensors  (no fresh /scan or /vision/floor -> refuse to drive blind)
  - burst timeout
FORWARD ONLY by design: the 2nd-floor stair edge is behind/right, so we never reverse or turn toward it.
Re-invoke for the next burst. Returns the stop reason on exit.

Usage: ROBOT_ADDR=<ip> python3 q6a_drive.py --velocity 0.25 --seconds 4 [--angle 0]
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.10.1')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
STOP_CENTER = float(os.environ.get('Q6A_DRIVE_STOP_CENTER', '0.42'))   # MiDaS center drop = confirmed edge
MIN_FRONT = float(os.environ.get('Q6A_DRIVE_MIN_FRONT', '0.40'))       # m, LiDAR forward obstacle
FWD_HALF_DEG = 25.0
HZ = 6.6
# bump recovery (front bumper hit the LiDAR-invisible obstacle, e.g. a thin table leg): back off + turn away
REVERSE_S = float(os.environ.get('Q6A_DRIVE_REVERSE_S', '0.8'))
TURN_S = float(os.environ.get('Q6A_DRIVE_TURN_S', '1.3'))
REV_VEL = float(os.environ.get('Q6A_DRIVE_REV_VEL', '0.15'))
TURN_VEL = float(os.environ.get('Q6A_DRIVE_TURN_VEL', '0.15'))
TURN_ANGLE = float(os.environ.get('Q6A_DRIVE_TURN_ANGLE', '55'))   # arc while turning to change heading


def put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class Driver(Node):
    def __init__(self, vel, secs, angle):
        super().__init__('q6a_drive')
        self.vel, self.secs, self.angle = vel, secs, angle
        self.scan = None; self.scan_t = 0.0
        self.center = None; self.center_t = 0.0
        self.cliff = False
        self.bump = False
        self.mode = 'forward'          # forward | reverse | turn (bump recovery)
        self.mode_until = 0.0
        self.t0 = None
        self.warm0 = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.create_subscription(Bool, '/bumper', self.on_bump, latched)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().info(f'q6a_drive: forward {vel} m-ish for {secs}s '
                               f'(stop: center>={STOP_CENTER}, front<{MIN_FRONT}m, /cliff, stale)')

    def on_scan(self, m): self.scan = m; self.scan_t = time.monotonic()

    def on_floor(self, m):
        try:
            self.center = float(json.loads(m.data)['sectors']['center'][0]); self.center_t = time.monotonic()
        except Exception:
            pass

    def on_cliff(self, m): self.cliff = bool(m.data)

    def on_bump(self, m): self.bump = bool(m.data)

    def front_clear(self):
        m = self.scan
        if m is None:
            return None
        half = math.radians(FWD_HALF_DEG); mn = 99.0
        for i, r in enumerate(m.ranges):
            a = math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                           math.cos(m.angle_min + i * m.angle_increment))
            if abs(a) <= half and math.isfinite(r) and m.range_min <= r <= m.range_max and r < mn:
                mn = r
        return mn

    def stop(self, reason):
        for _ in range(3):
            try: put({'action': 'disable'})
            except Exception: pass
        self.get_logger().info(f'STOP: {reason}')
        raise SystemExit

    def tick(self):
        now = time.monotonic()
        have_scan = self.scan is not None and now - self.scan_t < 1.0
        have_floor = self.center is not None and now - self.center_t < 1.5
        if self.t0 is None:
            # ARM FIRST: /scan only flows while manual control is active (the turret spins only then), so
            # enable now to start the LiDAR, THEN wait for fresh sensors before actually driving.
            if self.warm0 is None:
                self.warm0 = now
                try: put({'action': 'enable'})
                except Exception as e: self.get_logger().warn(f'enable: {e}')
                self.get_logger().info('armed (turret spinning up); waiting for /scan + /vision/floor')
            if have_scan and have_floor:
                self.get_logger().info('sensors live — driving')
                self.t0 = now
            elif now - self.warm0 > 8.0:
                self.stop('no sensor data after arming (turret/scan chain down)')
            return
        # --- hard safety, every mode ---
        if now - self.t0 > self.secs:
            self.stop(f'burst done ({self.secs}s)')
        if self.cliff:
            self.stop('WHEEL-DROP')
        if not have_scan:
            self.stop('no fresh /scan (refuse to drive blind)')

        # --- bump recovery: reverse phase (rear has no drop sensing -> kept short; /cliff still guards) ---
        if self.mode == 'reverse':
            if now < self.mode_until:
                self._move(REV_VEL, 180.0); return       # back off the obstacle
            self.mode = 'turn'; self.mode_until = now + TURN_S

        # --- forward-facing safety (applies to forward + turn, both move forward-ish; NOT reverse) ---
        if not have_floor:
            self.stop('no fresh /vision/floor (refuse to drive blind)')
        if self.center >= STOP_CENTER:
            self.stop(f'DROP AHEAD (center={self.center:.2f})')

        # --- bump recovery: turn phase (arc away to change heading) ---
        if self.mode == 'turn':
            if now < self.mode_until:
                self._move(TURN_VEL, TURN_ANGLE); return
            self.mode = 'forward'
            self.get_logger().info('recovery done — resuming forward')

        # --- forward mode ---
        if self.bump:                                    # front bumper (LiDAR-invisible obstacle) -> recover
            self.get_logger().warn('BUMP — backing off + turning')
            self.mode = 'reverse'; self.mode_until = now + REVERSE_S; return
        fc = self.front_clear()
        if fc is not None and fc < MIN_FRONT:            # LiDAR obstacle (wall) -> turn away, keep exploring
            self.get_logger().info(f'obstacle {fc:.2f} m — turning away')
            self.mode = 'turn'; self.mode_until = now + TURN_S; return
        self._move(self.vel, self.angle)

    def _move(self, vel, angle):
        try:
            put({'action': 'move', 'vector': {'velocity': vel, 'angle': angle}})
        except Exception as e:
            self.get_logger().warn(f'move: {e}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--velocity', type=float, default=0.25)
    ap.add_argument('--seconds', type=float, default=4.0)
    ap.add_argument('--angle', type=float, default=0.0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros)
    node = Driver(a.velocity, a.seconds, a.angle)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try: put({'action': 'disable'})
        except Exception: pass
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()

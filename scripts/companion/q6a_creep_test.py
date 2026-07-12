#!/usr/bin/env python3
"""q6a_creep_test.py — SUPERVISED-ONLY diagnostic creep toward a MiDaS-detected drop.

⚠️ NOT FOR AUTONOMOUS/UNSUPERVISED USE. This intentionally does NOT hard-stop on the MiDaS floor-drop
signal — instead it reduces velocity proportionally as the signal strengthens, continuing to creep forward
at a reduced (but nonzero) speed, relying on wheel-drop (/cliff) as the ONLY remaining backstop. We have
empirically confirmed (2026-07-12) that wheel-drop only fires once a wheel has ALREADY left the ground —
a last-instant signal, not a safety margin. This script exists purely to gather finer-grained sensor data
during a slow supervised approach to a real edge (e.g. does anything else give an earlier signal if the
approach is much slower than q6a_drive.py's fixed-speed-then-hard-stop). A human MUST be present with a
hand ready to physically catch/stop the robot, or ready to hit disable, at all times.

This is a SEPARATE script from q6a_drive.py on purpose — q6a_drive.py's default hard-stop-on-MiDaS-drop
behavior is untouched and remains the production-safe behavior for any other use.

Usage: ROBOT_ADDR=<ip> python3 q6a_creep_test.py --seconds 15 --max-velocity 0.05 --min-velocity 0.015
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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.1.213')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
# ramp window: velocity scales from max (at RAMP_START) down to the floor (at RAMP_END and beyond).
# RAMP_START=0.42 matches q6a_drive.py's proven hard-stop threshold (STOP_CENTER) -- full speed right up
# until that point, THEN ease off, rather than easing off from ~1m out on ordinary floor-gradient noise
# (0.20 was too sensitive -- confirmed live 2026-07-12, it started slowing at ~1m from the edge).
RAMP_START = float(os.environ.get('Q6A_CREEP_RAMP_START', '0.42'))   # center reading where slowdown begins
RAMP_END = float(os.environ.get('Q6A_CREEP_RAMP_END', '0.58'))       # center reading where floor speed hits
HZ = 6.6


def put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class CreepTest(Node):
    def __init__(self, max_vel, min_vel, secs):
        super().__init__('q6a_creep_test')
        self.max_vel, self.min_vel, self.secs = max_vel, min_vel, secs
        self.scan_t = 0.0
        self.center = None; self.center_sharp = 0.0; self.center_t = 0.0
        self.cliff = False
        self.t0 = None; self.warm0 = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan_t', time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', lambda m: setattr(self, 'cliff', bool(m.data)), latched)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().warn(
            f'q6a_creep_test: SUPERVISED-ONLY, NOT for autonomous use. max_vel={max_vel} floor={min_vel} '
            f'ramp=[{RAMP_START},{RAMP_END}] for {secs}s. Wheel-drop /cliff is the ONLY hard backstop -- '
            f'human must be present with a hand ready at all times.')

    def on_floor(self, m):
        try:
            c = json.loads(m.data)['sectors']['center']
            self.center = float(c[0]); self.center_sharp = float(c[2]) if len(c) > 2 else 0.0
            self.center_t = time.monotonic()
        except Exception:
            pass

    def move(self, vel, angle=0.0):
        try:
            put({'action': 'move', 'vector': {'velocity': vel, 'angle': angle}})
        except Exception as e:
            self.get_logger().warn(f'move: {e}')

    def stop(self, reason):
        for _ in range(3):
            try: put({'action': 'disable'})
            except Exception: pass
        self.get_logger().warn(f'STOP: {reason}')
        raise SystemExit

    def tick(self):
        now = time.monotonic()
        have_scan = now - self.scan_t < 1.0
        have_floor = self.center is not None and now - self.center_t < 1.5
        if self.t0 is None:
            if self.warm0 is None:
                self.warm0 = now
                try: put({'action': 'enable'})
                except Exception as e: self.get_logger().warn(f'enable: {e}')
                self.get_logger().info('armed; waiting for /scan + /vision/floor')
            if have_scan and have_floor:
                self.get_logger().info('sensors live — creeping'); self.t0 = now
            elif now - self.warm0 > 8.0:
                self.stop('no sensor data after arming')
            return
        if now - self.t0 > self.secs:
            self.stop(f'done ({self.secs}s)')
        if self.cliff:                                          # the ONLY hard stop in this script
            self.stop('WHEEL-DROP (last-resort backstop fired)')
        if not have_scan or not have_floor:
            self.stop('stale sensors (refuse to drive blind even in creep mode)')
        # proportional speed reduction, NOT a stop -- floors at self.min_vel, never zero. Gated on sharpness
        # too (matches q6a_drive.py's MIN_SHARP=3.0): a smooth floor gradient can read a moderate center
        # value without being a real edge -- ignore the ramp entirely below that, same as the proven logic.
        if self.center_sharp < 3.0:
            frac = 0.0
        else:
            frac = (self.center - RAMP_START) / (RAMP_END - RAMP_START)
            frac = max(0.0, min(1.0, frac))
        vel = self.max_vel - frac * (self.max_vel - self.min_vel)
        self.move(vel, 0.0)
        self.get_logger().info(f'center={self.center:.3f} sharp={self.center_sharp:.1f} frac={frac:.2f} '
                               f'-> vel={vel:.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-velocity', type=float, default=0.05)
    ap.add_argument('--min-velocity', type=float, default=0.015)
    ap.add_argument('--seconds', type=float, default=15.0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros)
    node = CreepTest(a.max_velocity, a.min_velocity, a.seconds)
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

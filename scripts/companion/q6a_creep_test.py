#!/usr/bin/env python3
"""q6a_creep_test.py — SUPERVISED-ONLY constant-speed drive, wheel-drop is the ONLY stop condition.

⚠️ NOT FOR AUTONOMOUS/UNSUPERVISED USE, EVER. A human MUST be physically at the robot, hand ready to
catch/stop it, at all times.

**History (why MiDaS-based slowing was removed, 2026-07-12):** earlier versions of this script used the
MiDaS floor-drop signal (/vision/floor center+sharp) to proportionally reduce velocity when approaching an
edge. Live testing at the real edge confirmed this doesn't work as a safety aid: MiDaS goes BLIND right at
the boundary (center reads ~0, "no drop", the instant the robot is actually close enough to matter) --
so every cycle, once AVA's own wheel-drop detection recovered the robot, the ramp logic saw "clear floor"
and immediately commanded full speed again, straight back at the edge. Confirmed twice live, including
after fixing an actual bug (pausing must command vel=0, not just withhold commands) -- the behavior didn't
improve, because the underlying problem is a sensor blind spot, not tunable ramp parameters. User's call:
remove the MiDaS ramp entirely rather than keep chasing it ("it's useless because of the blind zone").

**What stops the robot now:** ONLY wheel-drop (/cliff, AVA's own signal we decode) -- and this time it is a
genuine hard stop that ENDS the run (raises SystemExit), not a pause. Also stale sensors (refuse to drive
blind) and the --seconds time bound. Nothing reduces speed on approach anymore -- it drives at a constant
commanded velocity until one of those three conditions fires.

This is a SEPARATE script from q6a_drive.py on purpose — q6a_drive.py's own hard-stop-on-MiDaS-drop AND
hard-stop-on-wheel-drop behavior are both untouched and remain the production-safe behavior for any other
use; this script exists only for this specific supervised experiment.

Usage: ROBOT_ADDR=<ip> python3 q6a_creep_test.py --velocity 0.3 --seconds 15
"""
import argparse
import json
import os
import sys
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.1.213')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
HZ = 6.6


def put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class CreepTest(Node):
    def __init__(self, vel, secs):
        super().__init__('q6a_creep_test')
        self.vel, self.secs = vel, secs
        self.scan_t = 0.0
        self.cliff = False
        self.t0 = None; self.warm0 = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan_t', time.monotonic()),
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', lambda m: setattr(self, 'cliff', bool(m.data)), latched)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().warn(
            f'q6a_creep_test: CONSTANT vel={vel} for {secs}s. NO MiDaS slowing (removed -- blind at the '
            f'boundary, confirmed useless live). Wheel-drop /cliff is the ONLY stop, and it IS a hard stop '
            f'now (ends the run). HUMAN PHYSICALLY CATCHING THE ROBOT IS STILL REQUIRED THE WHOLE TIME.')

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
        if self.t0 is None:
            if self.warm0 is None:
                self.warm0 = now
                try: put({'action': 'enable'})
                except Exception as e: self.get_logger().warn(f'enable: {e}')
                self.get_logger().info('armed; waiting for /scan')
            if have_scan:
                self.get_logger().info('scan live — driving'); self.t0 = now
            elif now - self.warm0 > 8.0:
                self.stop('no sensor data after arming')
            return
        if now - self.t0 > self.secs:
            self.stop(f'done ({self.secs}s)')
        if not have_scan:
            self.stop('stale scan (refuse to drive blind)')
        if self.cliff:                                    # the ONLY stop condition besides time/stale-scan
            self.stop('WHEEL-DROP (AVA /cliff) -- hard stop, run ends')
        self.move(self.vel, 0.0)
        self.get_logger().info(f'vel={self.vel:.3f} (constant, no MiDaS slowing)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--velocity', type=float, default=0.2)
    ap.add_argument('--seconds', type=float, default=15.0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros)
    node = CreepTest(a.velocity, a.seconds)
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

#!/usr/bin/env python3
"""q6a_creep_test.py — SUPERVISED-ONLY. MiDaS only slows the drive, never hard-stops. The human physically
catching the robot is still the primary backstop -- BUT this now RESPECTS AVA's own independent wheel-drop
detection + auto-recovery instead of fighting it.

⚠️⚠️ NOT FOR AUTONOMOUS/UNSUPERVISED USE, EVER.

**2026-07-12 incident (why the pause-on-cliff logic below exists):** with wheel-drop fully removed, a live
test confirmed something we could never confirm from logs -- AVA HAS ITS OWN independent wheel-drop
detection with automatic backward recovery, completely outside our software. But this script kept issuing
forward move commands every tick regardless of AVA's state, so every time AVA backed away to protect
itself, this script immediately pushed it toward the edge again -- an oscillating fight the user described
as the robot "trying to suicide" (repeatedly approaching, AVA saving it, us undoing that save). That is a
bug in THIS SCRIPT, not a safety feature -- fixed by pausing our own forward commands whenever /cliff is
active (plus a cooldown after it clears) so we stop fighting AVA's recovery.

This does NOT terminate the run and is NOT a substitute for supervision -- it only stops us from actively
undoing AVA's own protective action. The --seconds time bound and a stale-sensor abort are the only things
that end a run outright. A human MUST be physically at the robot, hand ready to catch/stop it, at all times.

This is a SEPARATE script from q6a_drive.py on purpose — q6a_drive.py's default hard-stop-on-MiDaS-drop
AND wheel-drop-stop behavior are both untouched and remain the production-safe behavior for any other use.

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
# how long /cliff must read clear before we resume pushing forward -- gives AVA's own backward recovery
# room to finish before we'd otherwise immediately re-approach the edge again (see docstring incident).
CLIFF_COOLDOWN_S = float(os.environ.get('Q6A_CREEP_CLIFF_COOLDOWN', '2.0'))
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
        self.t0 = None; self.warm0 = None
        self.cliff = False; self.cliff_clear_t = 0.0
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan_t', time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().warn(
            f'q6a_creep_test: no hard stop on MiDaS (slows only, floor={min_vel}); DOES pause forward '
            f'commands while AVA\'s own /cliff wheel-drop is active + {CLIFF_COOLDOWN_S}s after, to avoid '
            f'fighting its recovery. max_vel={max_vel} ramp=[{RAMP_START},{RAMP_END}] for {secs}s. '
            f'HUMAN PHYSICALLY CATCHING THE ROBOT IS STILL REQUIRED THE WHOLE TIME.')

    def on_cliff(self, m):
        was = self.cliff
        self.cliff = bool(m.data)
        if was and not self.cliff:
            self.cliff_clear_t = time.monotonic()
        if self.cliff != was:
            self.get_logger().warn(f'/cliff (AVA wheel-drop) -> {self.cliff}')

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
        if not have_scan or not have_floor:
            self.stop('stale sensors (refuse to drive blind even in creep mode)')
        # PAUSE (not a hard stop) while AVA's own wheel-drop is active, or within the cooldown after it
        # clears -- do NOT re-approach immediately and undo AVA's own recovery (see docstring incident).
        in_cooldown = (not self.cliff) and (time.monotonic() - self.cliff_clear_t < CLIFF_COOLDOWN_S) \
            and self.cliff_clear_t > 0
        if self.cliff or in_cooldown:
            self.get_logger().info(f'PAUSED (AVA /cliff={self.cliff}, cooldown={in_cooldown}) -- '
                                   f'not pushing forward, letting AVA finish its own recovery')
            return
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

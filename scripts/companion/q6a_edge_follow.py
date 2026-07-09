#!/usr/bin/env python3
"""q6a_edge_follow.py — LiDAR wall/edge following (companion), safety-gated.

Follows a wall / railing / furniture line on a chosen side at a set clearance, using the robot's own
360deg LDS. This is the standard, reliable approach (what LiDAR robots use); the earlier camera version
was the wrong tool — a forward MiDaS profile can't measure a lateral distance.

METHOD (researched: F1TENTH two-ray geometry vs sector line-fit; line-extraction/SLAM front-end literature).
We use a **least-squares (PCA) line fit over the whole follow-side sector**, NOT the two-ray trick, because
our LDS is a cheap unit: a live scan showed ~117/360 finite bins with big empty arcs, so any two specific
rays are often dropouts. Fitting a line to the 30-40 points that DO return in the side sector is robust to
single-bin dropouts and yields the wall's perpendicular distance `d` AND heading `psi` directly.

CONTROL (differential-drive, NOT Ackermann — the F1TENTH labs steer a car; we command Valetudo
{velocity, angle} where `angle` is a heading offset). PD on distance + heading:
    turn = KP_DIST*(d - setpoint) + KD_HEAD*psi        (in a "toward-wall positive" convention per side)
mapped to the Valetudo angle via STEER_SIGN. No integral term: the scan re-estimates every tick so drift is
corrected continuously; a *constant* offset from setpoint (rather than oscillation) would indicate a wrong
BODY_R/steer sign, not a missing I term — watch for that on the first run.

SETPOINT: the LDS sits at ~chassis center, so measured d = body_radius + desired gap. D10s Pro = 350 mm
diameter -> BODY_R = 0.175 m (spec). setpoint = BODY_R + gap (gap default 0.08 m => setpoint 0.255 m).
For a round chassis a fore/aft turret offset changes the lateral half-width by <5 mm, so this holds.

CORNERS (standard reactive wall-following): front sector below FRONT_STOP -> concave corner -> rotate away
in place. Too few inliers in the side sector -> wall lost / convex corner -> curve toward the follow side to
re-acquire.

SAFETY: wheel-drop /cliff -> disable manual control + exit (a *bare* drop-off with no vertical surface is
invisible to a horizontal LiDAR — the cliff-IR is the only backstop there). Stale scan -> stop. Bounded by
--seconds. Arm-first (enable manual control so the turret spins, then wait for /scan).

DRY-RUN (--dry-run): estimate + log d / psi / inliers / front / the command it WOULD send, but never move.
Use this first to confirm the estimator and the STEER_SIGN against the real wall before enabling motion.

Usage: ROBOT_ADDR=<ip> python3 q6a_edge_follow.py --side left --gap 0.08 --dry-run
       ROBOT_ADDR=<ip> python3 q6a_edge_follow.py --side left --gap 0.08 --seconds 25 --velocity 0.10
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.1.213')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
BODY_R = float(os.environ.get('Q6A_BODY_R', '0.175'))            # D10s Pro radius (350 mm dia, spec)
SIDE_HALF = math.radians(float(os.environ.get('Q6A_EF_SIDE_HALF_DEG', '40')))  # follow sector half-width
FRONT_HALF = math.radians(float(os.environ.get('Q6A_EF_FRONT_HALF_DEG', '22')))
FRONT_STOP = float(os.environ.get('Q6A_EF_FRONT_STOP', '0.35'))  # m; front closer than this = concave corner
MIN_INLIERS = int(os.environ.get('Q6A_EF_MIN_INLIERS', '12'))    # side points needed to trust the line
MAX_FIT_STD = float(os.environ.get('Q6A_EF_MAX_FIT_STD', '0.06'))  # m; reject a bad (non-linear) fit
KP_DIST = float(os.environ.get('Q6A_EF_KP_DIST', '120'))         # deg per m of distance error
KD_HEAD = float(os.environ.get('Q6A_EF_KD_HEAD', '0.45'))        # deg per deg of heading error
MAX_ANGLE = float(os.environ.get('Q6A_EF_MAX_ANGLE', '35'))      # clamp heading command (deg)
STEER_SIGN = float(os.environ.get('Q6A_EF_STEER_SIGN', '1'))     # flip if it steers the WRONG way
REACQUIRE_ANGLE = float(os.environ.get('Q6A_EF_REACQUIRE_ANGLE', '25'))  # curve toward side when wall lost
HZ = 6.6


def put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class EdgeFollow(Node):
    def __init__(self, side, gap, secs, vel, dry):
        super().__init__('q6a_edge_follow')
        self.side = side                                  # 'left' or 'right'
        self.phi0 = math.pi / 2 if side == 'left' else -math.pi / 2   # sector center bearing
        self.side_sign = 1.0 if side == 'left' else -1.0             # left wall on +y
        self.setpoint = BODY_R + gap
        self.secs, self.vel, self.dry = secs, vel, dry
        self.scan = None; self.scan_t = 0.0; self.cliff = False
        self.t0 = None; self.warm0 = None
        self.diag = (0, float('nan'), float('nan'), float('nan'))   # (npts, fit_std, raw_d, raw_psi)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', lambda m: setattr(self, 'cliff', bool(m.data)), latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().info(
            f'edge_follow[LiDAR]: follow {side} wall, gap={gap:.02f} -> setpoint d={self.setpoint:.3f}m, '
            f'{"DRY-RUN" if dry else f"{vel} for {secs}s"} '
            f'(KP={KP_DIST} KD={KD_HEAD} clamp={MAX_ANGLE}deg, steer_sign={STEER_SIGN})')

    def on_scan(self, m):
        self.scan = m; self.scan_t = time.monotonic()

    # ---- estimator: PCA line fit over the follow-side sector ----
    def estimate(self):
        m = self.scan
        n = len(m.ranges)
        pts = []
        # collect finite points within +/-SIDE_HALF of the side-center bearing
        span = int(round(math.degrees(SIDE_HALF)))
        c = int(round((self.phi0 - m.angle_min) / m.angle_increment))
        for d in range(-span, span + 1):
            i = (c + d) % n
            r = m.ranges[i]
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                b = m.angle_min + i * m.angle_increment
                pts.append((r * math.cos(b), r * math.sin(b)))
        self.diag = (len(pts), float('nan'), float('nan'), float('nan'))
        if len(pts) < MIN_INLIERS:
            return None
        P = np.asarray(pts)
        ctr = P.mean(axis=0)
        Q = P - ctr
        cov = Q.T @ Q / len(P)
        w, V = np.linalg.eigh(cov)                # ascending: w[0]=residual var, w[1]=along-wall var
        std = math.sqrt(max(w[0], 0.0))
        tangent = V[:, 1]; normal = V[:, 0]
        dist = abs(float(ctr @ normal))
        psi = math.degrees(math.atan2(tangent[1], tangent[0]))
        if psi > 90:   psi -= 180                 # line direction is 180-ambiguous -> [-90,90]
        elif psi < -90: psi += 180
        self.diag = (len(pts), std, dist, psi)    # logged in dry-run even if rejected below
        if std > MAX_FIT_STD:                      # points don't lie on a line (clutter, not a wall)
            return None
        return dist, psi, len(pts)

    def front_min(self):
        m = self.scan; n = len(m.ranges)
        span = int(round(math.degrees(FRONT_HALF)))
        c = int(round((0.0 - m.angle_min) / m.angle_increment))
        best = math.inf
        for d in range(-span, span + 1):
            r = m.ranges[(c + d) % n]
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                best = min(best, r)
        return best

    def move(self, vel, angle):
        if self.dry:
            return
        try:
            put({'action': 'move', 'vector': {'velocity': vel, 'angle': angle}})
        except Exception as e:
            self.get_logger().warn(f'move: {e}')

    def stop(self, reason):
        for _ in range(3):
            try: put({'action': 'disable'})
            except Exception: pass
        self.get_logger().info(f'STOP: {reason}')
        raise SystemExit

    def tick(self):
        now = time.monotonic()
        have_scan = self.scan is not None and now - self.scan_t < 1.0
        if self.t0 is None:                        # arm-first: manual control -> turret spins -> /scan
            if self.warm0 is None:
                self.warm0 = now
                try: put({'action': 'enable'})
                except Exception as e: self.get_logger().warn(f'enable: {e}')
                self.get_logger().info('armed; waiting for /scan')
            if have_scan:
                self.get_logger().info('scan live — following'); self.t0 = now
            elif now - self.warm0 > 8.0:
                self.stop('no /scan after arming (turret not spinning?)')
            return
        # --- hard safety ---
        if now - self.t0 > self.secs and not self.dry:
            self.stop(f'done ({self.secs}s)')
        if self.cliff:
            self.stop('WHEEL-DROP (bare edge / fell)')
        if not have_scan:
            self.stop('stale /scan')
        # --- estimate ---
        front = self.front_min()
        est = self.estimate()
        if front < FRONT_STOP:                     # concave corner: wall ahead -> rotate AWAY in place
            turn = STEER_SIGN * (-self.side_sign) * MAX_ANGLE
            self._act(self.vel * 0.25, turn, f'CORNER front={front:.2f} rotate-away', est, front)
        elif est is None:                          # wall lost / convex corner -> curve toward the side
            turn = STEER_SIGN * self.side_sign * REACQUIRE_ANGLE
            npts, std, rd, rpsi = self.diag        # why: too few points, or points not linear (std high)?
            self._act(self.vel * 0.6, turn,
                      f'WALL-LOST (n={npts} fit_std={std:.3f} raw_d={rd:.3f} raw_psi={rpsi:+.1f}) reacquire',
                      est, front)
        else:
            dist, psi, ninl = est
            e = dist - self.setpoint               # >0 too far (approach), <0 too close (retreat)
            turn_internal = self.side_sign * (KP_DIST * e) + KD_HEAD * psi   # toward-wall-positive
            turn = STEER_SIGN * max(-MAX_ANGLE, min(MAX_ANGLE, turn_internal))
            v = self.vel * max(0.35, 1.0 - abs(turn) / (MAX_ANGLE * 1.5))    # slow down in sharp turns
            self._act(v, turn, f'd={dist:.3f} e={e:+.3f} psi={psi:+.1f} n={ninl}', est, front)

    def _act(self, vel, angle, tag, est, front):
        self.move(vel, angle)
        self.get_logger().info(f'{"[dry] " if self.dry else ""}{tag} -> vel={vel:.2f} angle={angle:+.1f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', choices=['left', 'right'], default='left')
    ap.add_argument('--gap', type=float, default=0.08)         # desired clearance body-edge -> wall (m)
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--velocity', type=float, default=0.10)
    ap.add_argument('--dry-run', action='store_true')
    a, ros = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros)
    node = EdgeFollow(a.side, a.gap, a.seconds, a.velocity, a.dry_run)
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

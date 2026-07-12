#!/usr/bin/env python3
"""q6a_creep_test.py — SUPERVISED-ONLY drive with a speed-scaled MiDaS caution zone, wheel-drop is the
final hard stop.

⚠️ NOT FOR AUTONOMOUS/UNSUPERVISED USE, EVER. A human MUST be physically at the robot, hand ready to
catch/stop it, at all times.

**History (v1-v6, 2026-07-12):** earlier versions used MiDaS (/vision/floor center+sharp) to proportionally
ramp velocity DOWN TO ZERO near an edge. Live testing killed this design for two independent reasons:
(a) MiDaS goes BLIND right at the boundary (center reads ~0 the instant the robot is actually close enough
to matter), so once AVA's own wheel-drop recovery backed the robot up, the ramp saw "clear floor" and
immediately re-commanded full speed, fighting AVA in a repeating "approach -> recover -> re-approach" cycle
(confirmed twice, including after fixing a real bug where "pause" didn't send vel=0). (b) at max velocity
(1.0) the wheel-drop hard stop itself fired correctly and instantly, but the robot's PHYSICAL MOMENTUM
carried it past the point where AVA's own recovery could work at all -- it hung at the edge with wheels
off the ground, needing a physical rescue. v6 removed MiDaS entirely and just drove constant-speed to
isolate that finding; it reproduced the incident at velocity=1.0.

**v7 design (current) -- two independent fixes, not more threshold tuning:**
1. HARD VELOCITY CEILING (`MAX_SAFE_VEL`): every velocity ≤0.4 tested today let AVA's wheel-drop recovery
   work reliably; 1.0 didn't. This is clamped in code, unconditionally -- not just advised in a docstring,
   because the ceiling being advisory-only is exactly what caused the incident.
2. CAUTION ZONE, not a stop-ramp: MiDaS has no metric calibration (it's a relative disparity signal, not a
   real distance), so we can't compute a true physics stopping distance yet. As an approximation, the
   trigger point moves EARLIER (lower center threshold = farther out) as commanded speed rises toward the
   cap -- see `enter_thresh()`. Once triggered we do NOT ramp to zero; we hold a constant slow creep
   (`CAUTION_VEL`) so the robot keeps making real progress, exactly like AVA's own approach. Wheel-drop
   (/cliff) remains the only genuine hard stop (raises SystemExit, ends the run).
   ⚠️ **Caution has NO center-based exit** (fixed after a live repro, same day): a first version cleared
   caution once `center` dropped back below threshold, but a LOW reading up close IS the MiDaS blind spot,
   not evidence of a clear floor -- it exited right at the edge and re-commanded full speed for ~1.3s
   before wheel-drop, reproducing the exact "fighting AVA" failure mode v1-v5 were built to avoid. Once
   latched, caution only clears via the direction-change re-arm below, or the run ending.
3. DIRECTION-CHANGE RE-ARM: the MiDaS blind spot is specific to the current approach angle, not the
   location. If the commanded angle changes by more than `REARM_DEG`, the caution latch is cleared so the
   next tick's MiDaS reading is trusted fresh rather than staying latched on a stale heading's assessment.
4. WHAT ACTUALLY STOPS THE FIGHTING, determined by live elimination (all same day):
   - Continuous `CAUTION_VEL=0.15`: still fought AVA a couple of times right at the true edge.
   - `--pulse` (hard on/off: `CAUTION_VEL` then an explicit vel=0 pause): verified live, fully clean, but
     visibly jerky (stop-start-stop-start).
   - Continuous, but SLOWER (`CAUTION_VEL=0.08`, no pulsing): **still fought.** This falsifies the theory
     that a slower push avoids provoking the reflex -- it isn't about magnitude, it's about the pause.
     Some protective reflex (not necessarily the same signal as our decoded /cliff bit) needs an actual
     return-to-near-zero to release; any continuously-held nonzero command, however small, never gives it
     that.
   - Tried SOFT PULSE (`--soft`): smooth sinusoid between `PULSE_LOW=0.02` and `CAUTION_VEL`, never a
     discrete step. **Confirmed live: still fights.** Nails down that it's specifically ZERO that matters,
     not "low" -- a nonzero trough, however smooth or however small, never releases whatever reflex fires.
     Smoothness and fully avoiding the fight are in tension on this platform; kept behind `--soft` for
     future re-tuning only, not recommended.
   - **Current default: hard on/off pulse** (`CAUTION_VEL` then an explicit vel=0 pause) -- the only
     method verified clean live, twice. Yes, it's jerky (stop-start-stop-start); that appears to be the
     actual cost of not fighting AVA on this platform, not a tunable side effect.
5. ENTRY THRESHOLD TIGHTENED (`BASE_ENTER` 0.50->0.62->0.68, `ENTER_SPAN` 0.15->0.08->0.06): live feedback
   2026-07-12 -- 0.50 triggered caution ~0.5m before the true edge; 0.62 was still ~20-30cm out; user wants
   ~10cm. Raised again to 0.68. Still no metric MiDaS calibration behind this number -- an empirical retune
   from feel, may need further adjustment.
6. EVENT-DRIVEN caution (`--mode event`, new default, UNTESTED as of writing): all the pulsing/soft-pulse
   work above assumed we had to blindly guess when to pause, because our only decoded signal treated as an
   "AVA reaction" was `/cliff` -- which is actually just the downward IR sensors (Triggers byte[1]), and
   we'd already established those only co-fire AT THE INSTANT of full wheel-drop, not earlier. There is a
   SEPARATE, dedicated `/wheel_floating` topic (Triggers byte[0] bits 6-7, decoded 2026-07-11) that none of
   today's testing ever subscribed to or watched. New design: drive continuously at `CAUTION_VEL`, and
   PAUSE (vel=0) only when `/wheel_floating` is actually true, holding the pause for `WF_SETTLE_S` after it
   clears before resuming. **VERIFIED LIVE, twice, clean (2026-07-12)**: at both vel=0.3 and vel=0.4 (the
   MAX_SAFE_VEL cap), caution entered and drove CONTINUOUSLY at `CAUTION_VEL` the whole way -- `/wheel_
   floating` never even fired, so the pause logic wasn't exercised, and there was NO fighting either time.
   This also settles the earlier ambiguity (continuous 0.15 had fought in one test, was clean in another):
   most likely explanation is real run-to-run physical variability (hand-placement angle/position each
   time), not a deterministic software issue -- once entry timing was fixed (point 5) and warm-up made
   robust to DDS discovery latency (`WARM_TIMEOUT_S`, see below), continuous driving in caution has been
   clean on every subsequent test. `--mode pulse` (blind hard on/off) remains available as a fallback if
   fighting reappears, but is no longer the default recommendation.
7. WARM-UP TIMEOUT RAISED (8.0s -> 20.0s, see the literal in `tick()`): a fresh rclpy node's DDS discovery
   of `/scan` can legitimately take ~10-12s (a known Fast-DDS discovery-latency characteristic on this
   setup, not an outage) -- 8s was causing spurious "no sensor data after arming" aborts even though the
   LiDAR and `/scan` were fine the whole time (confirmed via the ring buffer's write-pointer advancing and
   `ros2 topic hz /scan` succeeding at a longer timeout).
8. ODOMETRY BLIND-CREEP (new, UNTESTED as of writing): the whole caution zone above still stops trusting
   MiDaS once it goes blind, but never gets the robot closer than "wherever it happened to be" at that
   point. Calibrated 2026-07-12 with a stationary tape-measure pass (robot placed by hand at known
   distances, /vision/floor logged at each -- see CHANGELOG for the full 8-point table): the strongest,
   most confident reading is ~15-20cm out (center 0.65-0.68, sharp 16-17); it starts declining by 10cm and
   is fully blind by 5cm (matches every wheel-drop-time reading seen all day). So: the moment `center` and
   `sharp` cross into that confirmed-strong band (`LATCH_CENTER`/`LATCH_SHARP`), we LATCH the current
   `/odom/wheel` position and stop trusting MiDaS's live reading entirely -- from then on we creep
   `BLIND_CREEP_M` (default 0.07m) tracked via odometry distance from the latch point, not vision. Straight
   -line wheel odometry is trusted here specifically because the earlier-established unreliability was for
   IN-PLACE PIVOTS (wheel slip during rotation), not forward travel. `/wheel_floating` reactive pausing
   stays active during blind creep too (it doesn't depend on MiDaS). Wheel-drop (/cliff) remains the final,
   unconditional safety net throughout -- the blind-creep distance is a deliberately conservative estimate
   from only 8 calibration points, not a guarantee.

This is still a SEPARATE script from q6a_drive.py -- its own hard-stop-on-MiDaS-drop and hard-stop-on-wheel
-drop behavior are untouched and remain the production-safe behavior for any other use; this script is the
supervised sandbox for iterating on close-approach behavior before anything migrates there.

Usage: ROBOT_ADDR=<ip> python3 q6a_creep_test.py --velocity 0.4 --seconds 20 --blind-creep 0.07
"""
import argparse
import json
import os
import sys
import math
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.1.213')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
HZ = 6.6

MAX_SAFE_VEL = 0.4        # unconditional ceiling -- validated recovery envelope, see docstring point 1
CAUTION_VEL = 0.15        # creep speed in caution (event/pulse modes) and the high point of soft
PULSE_LOW = 0.02          # (soft mode only) low point of the sinusoidal oscillation -- confirmed NOT
                          # enough to stop the fighting (still fought live 2026-07-12); kept for reference
PULSE_ON_S = 0.4          # pulse mode: how long each pulse drives forward; soft: half the osc. period
PULSE_OFF_S = 0.5         # pulse mode: explicit vel=0 pause between pulses; soft: other half of the period
WF_SETTLE_S = 0.6         # event mode: hold vel=0 for this long after /wheel_floating last cleared, before
                          # resuming -- gives the reflex room to fully release, not just its instant
BASE_ENTER = 0.56         # center threshold to enter caution at v->0. History: 0.50 triggered ~0.5m out
                          # (too early) -> raised to 0.62 (thresh=0.56 @ vel=0.3, still ~20-30cm out) ->
                          # raised to 0.68 (thresh=0.64) -> live-tested: center only PEAKED at 0.56 before
                          # the blind spot that run -- caution NEVER triggered, robot drove full speed to
                          # the true edge. `center`'s peak-before-blind-spot is noisy run-to-run (~0.5-0.76
                          # seen today), so a threshold tuned to trigger "closer" risks never triggering at
                          # all -- worse than triggering early, since a miss means full speed to the edge.
                          # Lowered to 0.52 (biased toward reliably triggering), then added the odometry
                          # blind-creep on top -- two clean runs at 0.52 (thresh=0.46 @ vel=0.3) both
                          # entered caution and landed safely, but user reported the SECOND run "started to
                          # slow down too early" + "stopped a bit further" vs the first, despite both
                          # latching at a near-identical center (~0.62-0.63) -- i.e. the caution-ENTRY point
                          # varies more run-to-run than the LATCH point does. Nudged up slightly (0.52->
                          # 0.56) to push entry a bit later/closer; this is inherently noisy sensor data, so
                          # expect this to reduce but not eliminate run-to-run variance.
ENTER_SPAN = 0.08         # how much earlier (lower threshold) entry moves at v==MAX_SAFE_VEL
REARM_DEG = 15.0          # commanded-angle delta that clears the caution latch (new approach direction)
MIN_SHARP = 3.0           # below this, treat center as noise (matches q6a_drive.py's gate)

# --- odometry blind-creep (calibrated 2026-07-12, stationary tape-measure pass, see CHANGELOG) ---
# Measured center/sharp at known distances from a real edge: 65cm~0.28/2.8, 50cm~0.45/5.6, 40cm~0.41/6.8,
# 30cm~0.41/7.9, 20cm~0.65/16.1, 15cm~0.68/17.2, 10cm~0.54/12.1 (already declining), 5cm~0.06/2.4 (blind).
# Strongest/most confident band is ~15-20cm out; decline starts between 15 and 10cm; fully blind by 5cm.
LATCH_CENTER = 0.60       # latch (stop trusting MiDaS, switch to odometry) once center reaches this --
                          # within the confirmed strong-signal band, not waiting for the decline/blind zone
                          # where the reading becomes ambiguous. Briefly lowered to 0.55 for reliability
                          # (two runs had peaked at 0.59-0.60 and never latched) -- but live-tested at 0.55:
                          # latch fired almost immediately after caution entry (center jumped 0.00->0.43->
                          # 0.54->0.57 in ~3 ticks) and the robot stopped 35cm from the true edge, far more
                          # than the ~5-13cm seen in the two earlier successful runs at the SAME location
                          # and setup. That's a real, unexplained gap between the stationary calibration and
                          # in-motion behavior (possibly approach-angle or processing-latency sensitivity),
                          # not just threshold noise -- rolled back to 0.60 (the value behind both actual
                          # successful landings) rather than keep tuning without understanding the cause.
LATCH_SHARP = 8.0         # ...secondary noise-reject gate. Was 14.0 -- live-tested: center and sharp don't
                          # reliably peak on the SAME tick (noise), so the AND-gate had repeated near-misses
                          # (center=0.60/sharp=13.5, then center=0.56/sharp=14.0, etc.) and latch never
                          # fired that run. Loosened sharp's role back toward a basic noise-reject gate
                          # (closer to MIN_SHARP's role for caution-entry) since `center` crossing its own
                          # threshold is already the primary confidence signal.
BLIND_CREEP_M = 0.05      # target distance to creep via odometry ONLY after latching, no MiDaS trust at
                          # all. Was 0.07m (2 clean runs, landed ~8-13cm both times but user reported the
                          # 2nd run "stopped a bit further" than the 1st) -- shortened to 0.05m to land a
                          # bit closer given the LATCH point itself has been fairly consistent (~0.62-0.63
                          # center) across both runs. Still deliberately conservative vs the full 5-10cm
                          # goal, given the calibration is only 8 stationary points, not a dense curve.
BLIND_VEL = 0.10          # creep speed once blind (driving with zero live distance feedback) -- slower
                          # than CAUTION_VEL on purpose.


def enter_thresh(v):
    frac = max(0.0, min(1.0, v / MAX_SAFE_VEL))
    return BASE_ENTER - ENTER_SPAN * frac


def put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class CreepTest(Node):
    def __init__(self, vel, secs, mode='event', blind_creep=BLIND_CREEP_M, force_unsafe_velocity=False):
        super().__init__('q6a_creep_test')
        if force_unsafe_velocity:
            self.vel = vel
            if vel > MAX_SAFE_VEL:
                self.get_logger().warn(
                    f'--force-unsafe-velocity: requested {vel} > MAX_SAFE_VEL {MAX_SAFE_VEL} -- NOT '
                    f'clamped, driving at the RAW requested velocity. This is the exact condition that '
                    f'caused a wheel-hang incident earlier today (2026-07-12) at vel=1.0 -- the robot\'s '
                    f'momentum carried it past the point where AVA\'s own wheel-drop recovery could work. '
                    f'The caution-zone/blind-creep logic below was designed and tested only at <=0.4.')
        else:
            self.vel = min(vel, MAX_SAFE_VEL)     # unconditional clamp -- see docstring point 1
            if vel > MAX_SAFE_VEL:
                self.get_logger().warn(f'requested velocity {vel} > MAX_SAFE_VEL {MAX_SAFE_VEL} -- clamped')
        self.secs = secs
        self.mode = mode           # 'event' (default) | 'pulse' | 'soft'
        self.blind_creep = blind_creep
        self.scan_t = 0.0
        self.cliff = False
        self.wheel_floating = False
        self.wf_last_true = None   # monotonic time /wheel_floating was last seen True
        self.center = 0.0
        self.center_sharp = 0.0
        self.caution = False
        self.blind = False        # latched past the point of trusting MiDaS -- odometry-only from here
        self.pos = None           # (x, y) from /odom/wheel, updated continuously
        self.latch_pos = None     # (x, y) recorded the instant we entered blind creep
        self.pulse_on = True      # phase within a caution pulse cycle: True=driving, False=paused/settling
        self.pulse_t0 = None      # monotonic time the current pulse phase started
        self.last_angle = 0.0
        self.t0 = None; self.warm0 = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan_t', time.monotonic()),
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', lambda m: setattr(self, 'cliff', bool(m.data)), latched)
        self.create_subscription(Bool, '/wheel_floating', self.on_wheel_floating, latched)
        self.create_subscription(String, '/vision/floor', self.on_floor, 10)
        self.create_subscription(Odometry, '/odom/wheel', self.on_odom, 20)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().warn(
            f'q6a_creep_test v7: vel={self.vel} for {secs}s, mode={mode}, blind_creep={blind_creep}m. '
            f'Caution zone via MiDaS (speed-scaled trigger) + direction-change re-arm + odometry blind-creep '
            f'past the MiDaS blind spot. Wheel-drop /cliff is the only genuine hard stop (ends the run). '
            f'HUMAN PHYSICALLY CATCHING THE ROBOT IS STILL REQUIRED.')

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pos = (p.x, p.y)

    def on_wheel_floating(self, m):
        wf = bool(m.data)
        self.wheel_floating = wf
        if wf:
            self.wf_last_true = time.monotonic()
            self.get_logger().warn('WHEEL_FLOATING TRUE -- an earlier reflex than /cliff (IR); pausing')

    def on_floor(self, m):
        try:
            c = json.loads(m.data)['sectors']['center']
            self.center = float(c[0])
            self.center_sharp = float(c[2]) if len(c) > 2 else 0.0
        except Exception:
            pass

    def move(self, vel, angle=0.0):
        if abs(angle - self.last_angle) > REARM_DEG:
            if self.caution:
                self.get_logger().info(f'direction change ({self.last_angle:.0f}->{angle:.0f}deg) '
                                       f'-- clearing caution+blind latch, re-assessing fresh')
            self.caution = False
            self.blind = False
            self.latch_pos = None
        self.last_angle = angle
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
            elif now - self.warm0 > 20.0:   # DDS discovery for a fresh node can take ~10-12s, not <8s
                self.stop('no sensor data after arming')
            return
        if now - self.t0 > self.secs:
            self.stop(f'done ({self.secs}s)')
        if not have_scan:
            self.stop('stale scan (refuse to drive blind)')
        if self.cliff:                                    # the ONLY hard stop besides time/stale-scan
            self.stop('WHEEL-DROP (AVA /cliff) -- hard stop, run ends')

        thresh = enter_thresh(self.vel)
        sharp_ok = self.center_sharp >= MIN_SHARP
        if not self.caution and sharp_ok and self.center >= thresh:
            self.caution = True
            self.pulse_on = True
            self.pulse_t0 = now
            desc = {'event': f'reactive: drive {CAUTION_VEL:.2f}, pause on /wheel_floating '
                             f'(+{WF_SETTLE_S}s settle)',
                   'pulse': f'hard pulse {CAUTION_VEL:.2f} on/{PULSE_ON_S}s off/{PULSE_OFF_S}s',
                   'soft': f'soft pulse {PULSE_LOW:.2f}<->{CAUTION_VEL:.2f}, period '
                           f'{PULSE_ON_S + PULSE_OFF_S}s (UNPROVEN, still fought live last test)'}[self.mode]
            self.get_logger().warn(f'CAUTION ENTER: center={self.center:.2f} sharp={self.center_sharp:.1f} '
                                   f'>= thresh={thresh:.2f} (vel={self.vel:.2f}) -- {desc}, latched until '
                                   f'direction change or wheel-drop')
        # NO center-based exit. Once triggered, a LOW reading up close is the known MiDaS blind spot, not
        # evidence of a clear floor -- exiting on it is exactly the false signal that broke every earlier
        # ramp version (confirmed live 2026-07-12: caution exited on center=0.06, resumed 0.3 for ~1.3s
        # straight into wheel-drop). The only way out of caution is move()'s direction-change re-arm, or
        # the run ending (wheel-drop / time bound).

        if self.caution and not self.blind and self.pos is not None \
                and self.center_sharp >= LATCH_SHARP and self.center >= LATCH_CENTER:
            self.blind = True
            self.latch_pos = self.pos
            self.get_logger().warn(f'BLIND-CREEP LATCH: center={self.center:.2f} sharp={self.center_sharp:.1f} '
                                   f'>= latch (center>={LATCH_CENTER}, sharp>={LATCH_SHARP}) -- MiDaS no '
                                   f'longer trusted, creeping {self.blind_creep*100:.0f}cm via odometry only')

        if self.blind:
            # Past the point of trusting MiDaS (calibrated 2026-07-12: this is where the reading starts
            # declining toward its blind-spot near-zero). Track distance via /odom/wheel instead -- straight
            # -line odometry is reliable here (the known unreliability was specifically for in-place pivots,
            # not forward travel). Wheel-drop stays the final safety net throughout.
            dx = self.pos[0] - self.latch_pos[0]
            dy = self.pos[1] - self.latch_pos[1]
            traveled = math.hypot(dx, dy)
            if traveled >= self.blind_creep:
                self.stop(f'BLIND-CREEP target reached ({traveled*100:.1f}cm since latch)')
            in_settle = self.wf_last_true is not None and (now - self.wf_last_true) < WF_SETTLE_S
            paused = self.wheel_floating or in_settle
            drive_vel = 0.0 if paused else BLIND_VEL
            tag = f' [BLIND traveled={traveled*100:.1f}cm/{self.blind_creep*100:.0f}cm' \
                  f'{" paused-wf" if paused else ""}]'
        elif self.caution and self.mode == 'event':
            # React to the actual earlier signal (/wheel_floating) instead of a blind timed pulse -- see
            # docstring point 6. Pause (vel=0) while it's active or within WF_SETTLE_S of last clearing.
            in_settle = self.wf_last_true is not None and (now - self.wf_last_true) < WF_SETTLE_S
            paused = self.wheel_floating or in_settle
            drive_vel = 0.0 if paused else CAUTION_VEL
            tag = ' [CAUTION paused-wf]' if paused else ' [CAUTION driving]'
        elif self.caution and self.mode == 'soft':
            # smooth continuous oscillation between PULSE_LOW and CAUTION_VEL -- CONFIRMED LIVE 2026-07-12
            # this still fights AVA (a nonzero trough isn't enough); kept only for future re-tuning.
            period = PULSE_ON_S + PULSE_OFF_S
            phase = ((now - self.pulse_t0) % period) / period      # 0..1
            frac = 0.5 - 0.5 * math.cos(2 * math.pi * phase)        # smooth 0..1..0
            drive_vel = PULSE_LOW + (CAUTION_VEL - PULSE_LOW) * frac
            tag = ' [CAUTION soft]'
        elif self.caution:   # mode == 'pulse'
            elapsed = now - self.pulse_t0
            limit = PULSE_ON_S if self.pulse_on else PULSE_OFF_S
            if elapsed >= limit:
                self.pulse_on = not self.pulse_on
                self.pulse_t0 = now
            drive_vel = CAUTION_VEL if self.pulse_on else 0.0
            tag = f' [CAUTION {"ON" if self.pulse_on else "OFF"}]'
        else:
            drive_vel = self.vel
            tag = ''
        self.move(drive_vel, 0.0)
        self.get_logger().info(f'vel={drive_vel:.3f}{tag} center={self.center:.2f} '
                               f'sharp={self.center_sharp:.1f} thresh={thresh:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--velocity', type=float, default=0.2)
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--mode', choices=['event', 'pulse', 'soft'], default='event',
                    help="'event' (default, verified clean live): drive continuously, pause only when "
                         "/wheel_floating actually fires (+settle). 'pulse': blind hard on/off timer -- "
                         "also verified clean, use if 'event' ever fights. 'soft': smooth oscillation -- "
                         "confirmed live to still fight, kept for reference only")
    ap.add_argument('--blind-creep', type=float, default=BLIND_CREEP_M,
                    help=f'meters to creep via odometry only, once MiDaS latches a confident reading '
                         f'(center>={LATCH_CENTER}, sharp>={LATCH_SHARP}). Default {BLIND_CREEP_M}m, '
                         f'calibrated 2026-07-12 -- see docstring/CHANGELOG for the distance curve.')
    ap.add_argument('--force-unsafe-velocity', action='store_true',
                    help=f'bypass the MAX_SAFE_VEL={MAX_SAFE_VEL} clamp and drive at the raw --velocity '
                         f'value. DANGEROUS: vel=1.0 caused a real wheel-hang incident on 2026-07-12 (AVA\'s '
                         f'wheel-drop recovery could not keep up with the momentum). Only use with a human '
                         f'physically ready to catch the robot, explicit awareness of that incident, and '
                         f'no expectation that the caution-zone/blind-creep logic (tested only <=0.4) will '
                         f'behave the same way above that speed.')
    a, ros = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros)
    node = CreepTest(a.velocity, a.seconds, mode=a.mode, blind_creep=a.blind_creep,
                     force_unsafe_velocity=a.force_unsafe_velocity)
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

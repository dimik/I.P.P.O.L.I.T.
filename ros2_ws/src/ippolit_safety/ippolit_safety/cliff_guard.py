#!/usr/bin/env python3
"""
cliff_guard.py — SAFETY: drop-off awareness near the 2nd-floor stairwell.

Two very different responses, on purpose:

  * WHEEL-DROP (`/cliff`, MCU Triggers byte[1]) = HARD e-stop. TWO independent layers (F1, D2):
    a zero-Twist hold published on `/cmd_vel_safety` (`twist_mux` priority 100 -- the primary
    stop, works even if `cmd_vel_bridge`'s REST calls are failing) PLUS the original direct REST
    `{"action":"disable"}` backstop (x3, WiFi can drop a PUT) + speak, kept as a last resort in
    case something downstream of `/cmd_vel_safety` is itself wedged. Also triggers the A5 rolling
    incident recorder's snapshot service so a wheel-drop always leaves an MCAP bag behind.

  * MiDaS floor-drop ahead = ADVISORY, NOT a freeze. The robot must be able to travel *along* an
    edge at a safe distance, not get blocked in front of it, so this layer only PUBLISHES the
    hazard — it never cuts manual control. It republishes `/vision/floor`'s per-sector drop
    (`ippolit_interfaces/FloorDrop`, typed per A3 -- was a JSON String) as `/cliff/ahead` (Bool =
    drop in the CENTER path ahead) so the drive controller can refuse to drive *forward* into a
    drop while still turning, reversing, or gliding parallel. Direction lives in `/vision/floor`
    (left/center/right fields).

Calibrated at the real ladder edge 2026-07-09: floor `max_step` room <=0.205 vs edge 0.35-0.65 ->
center threshold 0.30 (fuse 0.24 when the forward LiDAR sector is anomalously open = stairwell
signature).

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 cliff_guard.py

Parameters are declared below (see ippolit_bringup/config/cliff_guard.yaml for the deployed
values); this replaces the earlier Q6A_CLIFF_*/Q6A_OUCH_COOLDOWN environment-variable reads (A2).
ROBOT_ADDR stays a machine-local env var (sourced from /etc/default/ippolit-robot), per the A2
rule that only ROBOT_ADDR-class deployment values stay outside the ROS parameter system. The
MiDaS thresholds are SAFETY-CRITICAL, so their declared ranges are deliberately narrow band around
the calibrated values, not wide-open — see each ParameterDescriptor below.
"""
import json
import math
import os
import threading
import time
import urllib.request

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import FunctionDiagnosticTask, Updater
from geometry_msgs.msg import Twist
from ippolit_interfaces.msg import FloorDrop
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from rosbag2_interfaces.srv import Snapshot
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

CMD_VEL_SAFETY_HZ = 10.0   # faster than twist_mux's 0.5s per-topic timeout, so priority holds

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.10.1')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'

LIDAR_STALE_S = 2.0


class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')
        # SAFETY: center drop magnitude that alone means "cliff" (calibrated 2026-07-09: floor
        # max_step <=0.205, real edge 0.35-0.65). Range keeps tuning inside the proven band.
        self.declare_parameter(
            'midas_stop', 0.35,
            ParameterDescriptor(
                description='SAFETY: MiDaS center-sector drop magnitude that alone means cliff.',
                floating_point_range=[FloatingPointRange(from_value=0.20, to_value=0.60)]))
        self.declare_parameter(
            'midas_fuse', 0.28,
            ParameterDescriptor(
                description='SAFETY: weaker center drop that needs LiDAR-open agreement.',
                floating_point_range=[FloatingPointRange(from_value=0.15, to_value=0.50)]))
        self.declare_parameter(
            'midas_clear', 0.25,
            ParameterDescriptor(
                description='Re-arm hysteresis: center drop must fall below this to clear.',
                floating_point_range=[FloatingPointRange(from_value=0.10, to_value=0.45)]))
        self.declare_parameter(
            'min_sharp', 4.0,
            ParameterDescriptor(
                description=(
                    'SAFETY: minimum sharpness (real edge ~5-7, smooth floor gradient ~1.1-2.2) '
                    'required alongside midas_fuse/midas_stop to count as a real drop-off.'),
                floating_point_range=[FloatingPointRange(from_value=2.0, to_value=10.0)]))
        self.declare_parameter(
            'consec', 2,
            ParameterDescriptor(
                description='Consecutive hit frames required before latching /cliff/ahead=1.',
                integer_range=[IntegerRange(from_value=1, to_value=20)]))
        self.declare_parameter(
            'clear_n', 6,
            ParameterDescriptor(
                description='Consecutive clear frames required before latching /cliff/ahead=0.',
                integer_range=[IntegerRange(from_value=1, to_value=50)]))
        self.declare_parameter(
            'lidar_far_m', 3.5,
            ParameterDescriptor(
                description='Forward LiDAR median range (m) above which the sector is "open".',
                floating_point_range=[FloatingPointRange(from_value=0.5, to_value=8.0)]))
        self.declare_parameter(
            'lidar_half_deg', 20.0,
            ParameterDescriptor(
                description='Half-angle (deg) of the forward LiDAR sector checked for openness.',
                floating_point_range=[FloatingPointRange(from_value=5.0, to_value=90.0)]))
        self.declare_parameter(
            'speak_gap', 8.0,
            ParameterDescriptor(
                description='Minimum seconds between spoken "Edge ahead." warnings.',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=60.0)]))
        self.declare_parameter(
            'ouch_cooldown', 3.0,
            ParameterDescriptor(
                description='Minimum seconds between spoken "Ouch!" bump announcements.',
                floating_point_range=[FloatingPointRange(from_value=0.5, to_value=30.0)]))

        self.midas_stop = self.get_parameter('midas_stop').value
        self.midas_fuse = self.get_parameter('midas_fuse').value
        self.midas_clear = self.get_parameter('midas_clear').value
        self.min_sharp = self.get_parameter('min_sharp').value
        self.consec = self.get_parameter('consec').value
        self.clear_n = self.get_parameter('clear_n').value
        self.lidar_far_m = self.get_parameter('lidar_far_m').value
        self.lidar_half_deg = self.get_parameter('lidar_half_deg').value
        self.speak_gap = self.get_parameter('speak_gap').value
        self.ouch_cooldown = self.get_parameter('ouch_cooldown').value

        self.pub_speak = self.create_publisher(String, '/robot/speak', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ahead = self.create_publisher(Bool, '/cliff/ahead', latched)  # drop ahead
        self.pub_cmd_vel_safety = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self.cli_snapshot = self.create_client(
            Snapshot, '/rosbag_snapshot_recorder/snapshot')
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.create_subscription(Bool, '/bumper', self.on_bumper, latched)
        self.create_subscription(FloorDrop, '/vision/floor', self.on_floor, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / CMD_VEL_SAFETY_HZ, self.publish_cmd_vel_safety)
        self.bumped = False
        self.wheel_tripped = False
        self.ahead = False
        self.ouch_t = 0.0
        self.n_hit = self.n_clear = 0
        self.lidar_far = False
        self.lidar_at = 0.0
        self.last_warn = 0.0
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('cliff_guard')
        self.diag_updater.add(FunctionDiagnosticTask('wheel-drop e-stop state', self._diag))
        self.pub_ahead.publish(Bool(data=False))
        self.get_logger().info(
            f'cliff_guard up: wheel-drop -> HARD e-stop; MiDaS center-drop '
            f'(>={self.midas_stop}, or >={self.midas_fuse}+LiDAR-open) -> ADVISORY /cliff/ahead '
            f'(never freezes; drive loop avoids forward). @ {ROBOT_ADDR}')

    def _diag(self, stat):
        if self.wheel_tripped:
            # WARN, not ERROR: the safety system is doing exactly its job here, not malfunctioning
            stat.summary(DiagnosticStatus.WARN, 'WHEEL-DROP active -- manual control disabled')
        else:
            stat.summary(DiagnosticStatus.OK, 'clear')
        stat.add('wheel_tripped', str(self.wheel_tripped))
        stat.add('cliff_ahead_advisory', str(self.ahead))
        return stat

    # --- HARD e-stop: a wheel is already off the ground ---
    def on_cliff(self, m):
        if m.data and not self.wheel_tripped:
            self.wheel_tripped = True
            self.get_logger().error('WHEEL-DROP — hard e-stop (disable manual control)')
            threading.Thread(target=self.estop, daemon=True).start()
            self.pub_speak.publish(String(data='Cliff. Stopping.'))
            self.trigger_snapshot()
        elif not m.data and self.wheel_tripped:
            self.wheel_tripped = False
            self.get_logger().warn('wheel-drop cleared')

    def trigger_snapshot(self):
        """A5: dump the rolling incident recorder's RAM ring to MCAP on a real wheel-drop."""
        if not self.cli_snapshot.service_is_ready():
            self.get_logger().warn('rosbag snapshot service not ready -- incident not captured')
            return
        self.cli_snapshot.call_async(Snapshot.Request()).add_done_callback(self._on_snapshot_done)

    def _on_snapshot_done(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'snapshot call failed: {e}')
            return
        if res.success:
            self.get_logger().info('incident snapshot captured')
        else:
            self.get_logger().warn('snapshot service reported failure')

    def on_bumper(self, m):
        # audible confirmation the bumper fired (front collision). The rising edge = exactly one
        # "Ouch!" per distinct hit (clears when the bumper releases). Recovery/back-off is
        # q6a_drive's job.
        if m.data and not self.bumped:
            self.bumped = True
            now = time.monotonic()
            if now - self.ouch_t >= self.ouch_cooldown:   # collapse flickering bump -> one "Ouch!"
                self.ouch_t = now
                self.pub_speak.publish(String(data='Ouch!'))
                self.get_logger().info('BUMP -> Ouch!')
        elif not m.data:
            self.bumped = False

    def on_scan(self, m):
        half = math.radians(self.lidar_half_deg)
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
            self.lidar_far = med > self.lidar_far_m or (len(fin) / n_fwd) < 0.3
            self.lidar_at = time.monotonic()

    # --- ADVISORY: drop ahead in the center path (never disables manual control) ---
    def on_floor(self, m):
        center = m.center
        sharp = m.center_sharp
        lidar_fresh = (time.monotonic() - self.lidar_at) < LIDAR_STALE_S
        # a real drop-off is a SHARP discontinuity, not a smooth floor gradient's steepest step
        # require LiDAR corroboration: a real down-edge reads as an OPEN forward sector (beam
        # clears the drop); a false floor-discontinuity (rug/threshold/shadow/furniture base)
        # still returns LiDAR -> not open -> no warning. Also self-suppresses when idle (turret
        # parked -> stale LiDAR -> no false edge).
        hit = (sharp >= self.min_sharp and center >= self.midas_fuse
               and lidar_fresh and self.lidar_far)
        self.n_hit = self.n_hit + 1 if hit else 0
        self.n_clear = self.n_clear + 1 if center < self.midas_clear else 0
        if self.n_hit >= self.consec and not self.ahead:
            self.ahead = True
            self.pub_ahead.publish(Bool(data=True))
            self.get_logger().warn(
                f'drop-off in the forward path (center step={center:.2f}) — /cliff/ahead=1')
            now = time.monotonic()
            if now - self.last_warn > self.speak_gap:
                self.last_warn = now
                self.pub_speak.publish(String(data='Edge ahead.'))
        elif self.ahead and self.n_clear >= self.clear_n:
            self.ahead = False
            self.pub_ahead.publish(Bool(data=False))
            self.get_logger().info('forward path clear — /cliff/ahead=0')

    def publish_cmd_vel_safety(self):
        """
        Hold a zero Twist on /cmd_vel_safety while wheel-tripped.

        F1/D2: twist_mux gives this topic priority 100 (above teleop/nav), so this alone stops
        the robot regardless of what any other node is publishing -- independent of the estop()
        REST backstop below, which could itself be failing (network, AVA state) at the exact
        moment it's needed.
        """
        if self.wheel_tripped:
            self.pub_cmd_vel_safety.publish(Twist())

    def estop(self):
        body = json.dumps({'action': 'disable'}).encode()
        for _ in range(3):
            try:
                req = urllib.request.Request(
                    CAP, data=body, method='PUT',
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

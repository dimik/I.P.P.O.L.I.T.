#!/usr/bin/env python3
"""
cmd_vel_bridge.py — THE single actuation node: /cmd_vel (Twist) -> Valetudo REST manual control.

D2 (see docs/navigation-architecture.md): one node owns the whole AVA manual-control REST
interface, eliminating the class of bugs where multiple direct-REST callers race each other
(the pre-ROS `q6a_drive.py`/`q6a_creep_test.py` tools each called the REST API independently).
Upstream, `twist_mux` merges `/cmd_vel_safety` (priority 100, from `cliff_guard`),
`/cmd_vel_teleop` (50), and `/cmd_vel_nav` (10, Nav2 — not wired up until F4) onto `/cmd_vel`,
which is this node's only input.

SAFETY (G1): Valetudo HOLDS the last velocity it was sent — it does not time out on its own. So
"stopping" a drive means ACTIVELY sending a zero command, not just ceasing to publish. This node
runs a persistent ~6.6 Hz sender loop (the rate empirically established by the pre-ROS
`q6a_drive.py`) that resends the current (possibly zero) target every tick regardless of whether
a new `/cmd_vel` arrived, plus an explicit watchdog: if no `/cmd_vel` message has arrived within
`watchdog_timeout_s`, the target is forced to zero even if the last real message asked for motion.

Enable/disable ownership: this node is the only one that calls `{"action":"enable"}` on
HighResolutionManualControlCapability (lazily, on the first real non-idle command — never
proactively at startup, so an idle system with no `/cmd_vel` publishers never arms manual control
or spins up the turret). `cliff_guard` is the one other caller of this REST capability, and it
only ever calls `{"action":"disable"}` as a last-resort backstop — see its own module docstring.
After `idle_disable_s` of continuous zero/no-command, this node calls disable itself so the robot
doesn't sit in manual-control mode (turret spinning) forgotten; it re-enables lazily on the next
real command.

Twist -> Valetudo mapping is a PLACEHOLDER pending calibration (G8): `linear_scale` and
`angular_to_deg_scale` are not yet verified against `/odom_laser` on the real robot (deferred
alongside F0's physical map-resume test — see docs/navigation-architecture.md). Reverse is
unsupported until calibrated: negative `linear.x` clamps to 0 rather than driving backward with
an unverified mapping.

Parameters are declared in `CmdVelBridge.__init__` (see ippolit_bringup/config/cmd_vel_bridge.yaml
for the deployed values). `ROBOT_ADDR` stays a machine-local env var per the A2 convention.
"""
import json
import math
import os
import threading
import time
import urllib.request

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.node import Node

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.10.1')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'
SEND_HZ = 6.6   # matches the pre-ROS q6a_drive.py's empirically-established REST refresh rate


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def twist_to_valetudo(linear_x, angular_z, max_safe_vel, linear_scale,
                      angular_to_deg_scale, max_angle_deg):
    """
    Map a Twist's linear.x (m/s) + angular.z (rad/s) to a Valetudo (velocity, angle_deg) vector.

    Reverse is unsupported until calibrated: a negative linear_x clamps to 0 rather than driving
    backward with an unverified sign/scale. Returns (velocity, angle_deg).
    """
    vel = clamp(max(0.0, linear_x) * linear_scale, 0.0, max_safe_vel)
    angle = clamp(math.degrees(angular_z) * angular_to_deg_scale, -max_angle_deg, max_angle_deg)
    return vel, angle


def _put(body):
    req = urllib.request.Request(CAP, data=json.dumps(body).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    urllib.request.urlopen(req, timeout=1.0).read()


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter(
            'max_safe_vel', 0.4,
            ParameterDescriptor(
                description=(
                    'SAFETY: hard ceiling on the Valetudo velocity magnitude (0..1). 0.4 is the '
                    'validated envelope where AVA wheel-drop recovery still engages in time -- '
                    'see q6a_creep_test.py docstring point 1. Do not raise without re-verifying.'),
                floating_point_range=[FloatingPointRange(from_value=0.05, to_value=0.4)]))
        self.declare_parameter(
            'linear_scale', 1.0,
            ParameterDescriptor(
                description=(
                    'UNCALIBRATED PLACEHOLDER: m/s -> Valetudo velocity units (0..1) scale. '
                    'Needs verification against /odom_laser (G8) during a physical test '
                    'session before this bridge drives anything meaningfully.')))
        self.declare_parameter(
            'angular_to_deg_scale', 1.0,
            ParameterDescriptor(
                description=(
                    'UNCALIBRATED PLACEHOLDER: rad/s -> Valetudo "angle" degrees field scale. '
                    'Needs verification against /odom_laser (G8) during a physical test '
                    'session.')))
        self.declare_parameter(
            'max_angle_deg', 45.0,
            ParameterDescriptor(
                description='Clamp on the Valetudo angle field magnitude (deg).',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=180.0)]))
        self.declare_parameter(
            'watchdog_timeout_s', 0.4,
            ParameterDescriptor(
                description=(
                    'SAFETY (G1): if no /cmd_vel arrives within this long, actively force the '
                    'sent command to zero -- Valetudo holds the last velocity otherwise, so '
                    'merely stopping publishing is NOT a stop.'),
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0)]))
        self.declare_parameter(
            'idle_disable_s', 30.0,
            ParameterDescriptor(
                description=(
                    'Call REST disable after this many seconds of continuous zero/no-command, '
                    'so the robot does not sit in manual-control mode (turret spinning) '
                    'forgotten. Re-enables lazily on the next real command.'),
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=300.0)]))

        self.max_safe_vel = self.get_parameter('max_safe_vel').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_to_deg_scale = self.get_parameter('angular_to_deg_scale').value
        self.max_angle_deg = self.get_parameter('max_angle_deg').value
        self.watchdog_timeout_s = self.get_parameter('watchdog_timeout_s').value
        self.idle_disable_s = self.get_parameter('idle_disable_s').value

        self.lock = threading.Lock()
        self.target_vel = 0.0
        self.target_angle = 0.0
        self.last_cmd_t = 0.0
        self.zero_since_t = None
        self.enabled = False
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / SEND_HZ, self.tick)
        self.get_logger().info(
            f'cmd_vel_bridge up: /cmd_vel -> Valetudo REST @ {ROBOT_ADDR}, '
            f'max_safe_vel={self.max_safe_vel}, watchdog={self.watchdog_timeout_s}s '
            f'(linear/angular scale UNCALIBRATED -- see G8)')

    def on_cmd_vel(self, msg):
        vel, angle = twist_to_valetudo(
            msg.linear.x, msg.angular.z, self.max_safe_vel, self.linear_scale,
            self.angular_to_deg_scale, self.max_angle_deg)
        with self.lock:
            self.target_vel = vel
            self.target_angle = angle
            self.last_cmd_t = time.monotonic()

    def tick(self):
        now = time.monotonic()
        with self.lock:
            vel, angle, age = self.target_vel, self.target_angle, now - self.last_cmd_t
        stale = self.last_cmd_t == 0.0 or age > self.watchdog_timeout_s
        if stale:
            # G1: actively hold zero rather than just stop sending (Valetudo latches the last cmd)
            vel, angle = 0.0, 0.0

        if vel > 0.0:
            self.zero_since_t = None
            if not self.enabled:
                try:
                    _put({'action': 'enable'})
                    self.enabled = True
                    self.get_logger().info('manual control enabled')
                except Exception as e:
                    self.get_logger().warn(f'enable failed: {e}')
                    return
        elif self.zero_since_t is None:
            self.zero_since_t = now

        if self.enabled:
            try:
                _put({'action': 'move', 'vector': {'velocity': vel, 'angle': angle}})
            except Exception as e:
                self.get_logger().warn(f'move failed: {e}')
            if self.zero_since_t is not None and now - self.zero_since_t > self.idle_disable_s:
                try:
                    _put({'action': 'disable'})
                    self.enabled = False
                    self.get_logger().info(
                        f'manual control disabled after {self.idle_disable_s:.0f}s idle')
                except Exception as e:
                    self.get_logger().warn(f'disable failed: {e}')


def main():
    rclpy.init()
    node = CmdVelBridge()
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

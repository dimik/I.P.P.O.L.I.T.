"""Unit tests for cmd_vel_bridge's pure Twist -> Valetudo mapping (no rclpy dependency)."""
import math

from ippolit_control.cmd_vel_bridge import clamp, twist_to_valetudo


def test_clamp_within_range():
    assert clamp(0.5, 0.0, 1.0) == 0.5


def test_clamp_below_range():
    assert clamp(-1.0, 0.0, 1.0) == 0.0


def test_clamp_above_range():
    assert clamp(5.0, 0.0, 1.0) == 1.0


def test_forward_velocity_scaled_and_clamped():
    vel, angle = twist_to_valetudo(
        linear_x=1.0, angular_z=0.0, max_safe_vel=0.4,
        linear_scale=1.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert vel == 0.4   # clamped to max_safe_vel, not the raw scaled 1.0
    assert angle == 0.0


def test_negative_linear_x_clamped_to_zero_reverse_unsupported():
    vel, _ = twist_to_valetudo(
        linear_x=-0.5, angular_z=0.0, max_safe_vel=0.4,
        linear_scale=1.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert vel == 0.0


def test_zero_twist_yields_zero_velocity_and_angle():
    vel, angle = twist_to_valetudo(
        linear_x=0.0, angular_z=0.0, max_safe_vel=0.4,
        linear_scale=1.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert vel == 0.0
    assert angle == 0.0


def test_angular_z_converted_to_degrees_and_clamped():
    vel, angle = twist_to_valetudo(
        linear_x=0.0, angular_z=math.pi, max_safe_vel=0.4,   # pi rad/s -> 180 deg, clamped to 45
        linear_scale=1.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert vel == 0.0
    assert angle == 45.0


def test_angular_z_negative_clamped_symmetrically():
    _, angle = twist_to_valetudo(
        linear_x=0.0, angular_z=-math.pi, max_safe_vel=0.4,
        linear_scale=1.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert angle == -45.0


def test_linear_scale_applied_before_clamp():
    vel, _ = twist_to_valetudo(
        linear_x=0.1, angular_z=0.0, max_safe_vel=0.4,
        linear_scale=2.0, angular_to_deg_scale=1.0, max_angle_deg=45.0)
    assert vel == 0.2

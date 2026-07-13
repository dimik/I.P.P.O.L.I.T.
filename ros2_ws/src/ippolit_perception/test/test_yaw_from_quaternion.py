"""Unit tests for q6a_objmap's yaw_from_quaternion (the camera-yaw-from-TF helper, A4)."""
import math

from ippolit_perception.q6a_objmap import yaw_from_quaternion


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_identity_is_zero_yaw():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_recovers_the_calibrated_1p8_deg():
    # the F0(b)/A4 camera yaw offset encoded in the URDF; must round-trip through the helper
    x, y, z, w = _quat_from_yaw(math.radians(1.8))
    assert math.isclose(math.degrees(yaw_from_quaternion(x, y, z, w)), 1.8, abs_tol=1e-6)


def test_positive_and_negative_yaw():
    for deg in (-90.0, -30.0, 15.0, 45.0, 90.0):
        x, y, z, w = _quat_from_yaw(math.radians(deg))
        assert math.isclose(math.degrees(yaw_from_quaternion(x, y, z, w)), deg, abs_tol=1e-6)


def test_ignores_roll_pitch_returns_only_z_rotation():
    # a quaternion with roll+pitch but zero yaw must still report ~0 yaw
    roll, pitch, yaw = math.radians(20.0), math.radians(-15.0), 0.0
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    assert abs(math.degrees(yaw_from_quaternion(x, y, z, w))) < 1e-6

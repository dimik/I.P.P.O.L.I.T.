#!/usr/bin/env python3
"""Integrate raw gyro-z during a drive; compare to wheel + laser yaw change (G29 arbiter)."""
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def dd(a, b):
    if a is None or b is None:
        return None
    x = b - a
    while x > 180:
        x -= 360
    while x < -180:
        x += 360
    return x


def main():
    rclpy.init()
    n = Node('gyrochk')
    st = {'gz': 0.0, 'last': None, 'w0': None, 'w1': None, 'l0': None, 'l1': None}

    def imu(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        if st['last'] is not None:
            st['gz'] += m.angular_velocity.z * (t - st['last'])
        st['last'] = t

    def w(m):
        y = math.degrees(yaw(m.pose.pose.orientation))
        if st['w0'] is None:
            st['w0'] = y
        st['w1'] = y

    def la(m):
        y = math.degrees(yaw(m.pose.pose.orientation))
        if st['l0'] is None:
            st['l0'] = y
        st['l1'] = y

    n.create_subscription(Imu, '/imu/data', imu, 50)
    n.create_subscription(Odometry, '/odom/wheel', w, 20)
    n.create_subscription(Odometry, '/odom_laser', la, 20)
    end = time.time() + 11
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    gz = math.degrees(st['gz'])
    wc = dd(st['w0'], st['w1'])
    lc = dd(st['l0'], st['l1'])
    print(f'GYRO integrated yaw: {gz:+.1f} deg')
    print(f'WHEEL yaw change:    {wc:+.1f} deg' if wc is not None else 'WHEEL: no data')
    print(f'LASER yaw change:    {lc:+.1f} deg' if lc is not None else 'LASER: no data')
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


main()

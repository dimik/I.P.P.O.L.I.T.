#!/usr/bin/env python3
"""Log commanded /cmd_vel vs gyro-rate vs wheel/laser yaw at 5 Hz (G29/G30 command-path probe)."""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('cmdprobe')
        self.cmd_vx = self.cmd_wz = 0.0
        self.gyro_z = 0.0        # latest rate
        self.wy = self.ly = None
        self.t0 = None
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 20)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 50)
        self.create_subscription(Odometry, '/odom/wheel', self.cb_w, 20)
        self.create_subscription(Odometry, '/odom_laser', self.cb_l, 20)
        self.create_timer(0.2, self.tick)
        print('t,cmd_vx,cmd_wz_degs,gyro_z_degs,wheel_yaw,laser_yaw', flush=True)

    def cb_cmd(self, m):
        self.cmd_vx = m.linear.x
        self.cmd_wz = m.angular.z

    def cb_imu(self, m):
        self.gyro_z = m.angular_velocity.z

    def cb_w(self, m):
        self.wy = math.degrees(yaw(m.pose.pose.orientation))

    def cb_l(self, m):
        self.ly = math.degrees(yaw(m.pose.pose.orientation))

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        wy = f'{self.wy:.1f}' if self.wy is not None else ''
        ly = f'{self.ly:.1f}' if self.ly is not None else ''
        print(f'{t:.2f},{self.cmd_vx:.3f},{math.degrees(self.cmd_wz):.1f},'
              f'{math.degrees(self.gyro_z):.1f},{wy},{ly}', flush=True)


def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


main()

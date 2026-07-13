#!/usr/bin/env python3
"""Log wheel/laser/EKF/Valetudo pose synchronized at 5 Hz to compare odom sources (G29 probe)."""
import json
import math
import threading
import time
import urllib.request

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

HOST = 'http://192.168.1.213'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Logger(Node):
    def __init__(self):
        super().__init__('odom_compare')
        self.wheel = self.laser = self.ekf = self.val = None
        self.create_subscription(Odometry, '/odom/wheel', self.cb_wheel, 20)
        self.create_subscription(Odometry, '/odom_laser', self.cb_laser, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_ekf, 20)
        threading.Thread(target=self.poll_val, daemon=True).start()
        self.t0 = None
        self.create_timer(0.2, self.tick)
        print('t,w_x,w_y,w_yaw,l_x,l_y,l_yaw,e_x,e_y,e_yaw,v_x,v_y,v_yaw', flush=True)

    def cb_wheel(self, m):
        p = m.pose.pose
        self.wheel = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_laser(self, m):
        p = m.pose.pose
        self.laser = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_ekf(self, m):
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, yaw_of(p.orientation))

    def poll_val(self):
        while rclpy.ok():
            try:
                with urllib.request.urlopen(HOST + '/api/v2/robot/state/map', timeout=2) as r:
                    d = json.load(r)
                rp = next((e for e in d.get('entities', [])
                           if e.get('type') == 'robot_position'), None)
                if rp and rp.get('points'):
                    x = rp['points'][0] / 1000.0
                    y = -rp['points'][1] / 1000.0
                    yaw = math.radians(-((rp.get('metaData') or {}).get('angle', 0)))
                    self.val = (x, y, yaw)
            except Exception:
                pass
            time.sleep(0.3)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        def f(v):
            return f'{v[0]:.4f},{v[1]:.4f},{math.degrees(v[2]):.2f}' if v else ',,'
        print(f'{t:.2f},{f(self.wheel)},{f(self.laser)},{f(self.ekf)},{f(self.val)}', flush=True)


def main():
    rclpy.init()
    n = Logger()
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

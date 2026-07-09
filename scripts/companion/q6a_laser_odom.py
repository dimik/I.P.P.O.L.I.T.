#!/usr/bin/env python3
"""q6a_laser_odom.py — 2D laser scan-matching odometry (companion).

WHY THIS EXISTS: work_mode 17 blocks Valetudo autonomous nav, and during manual control
Valetudo reports NO pose (robot_position is frozen) and the D10s MCU emits no wheel odometry.
So there is *no* map-frame pose available while driving the robot manually. This node derives
a live pose from /scan alone — point-to-point ICP between consecutive LiDAR revolutions —
and publishes it as the `odom -> base_link` TF + nav_msgs/Odometry on /odom_laser. That is the
pose source the semantic object map needs during a manual mapping drive. slam_toolbox can layer
on top (consuming this odom) for a globally-consistent map + loop closure.

Frames: publishes odom->base_link (dynamic) and base_link->laser (static identity — the /scan
is already base_link-aligned by lds_scan_node's bearing convention, and the turret sits ~center).

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
     python3 q6a_laser_odom.py   (stop valetudo-bridge first so it doesn't also own /odom + TF)
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

ODOM_FRAME = os.environ.get('Q6A_ODOM_FRAME', 'odom')
BASE_FRAME = os.environ.get('Q6A_BASE_FRAME', 'base_link')
LASER_FRAME = os.environ.get('Q6A_LASER_FRAME', 'laser')
ODOM_TOPIC = os.environ.get('Q6A_LASER_ODOM_TOPIC', '/odom_laser')
ICP_ITERS = int(os.environ.get('Q6A_ICP_ITERS', '14'))
ICP_MAX_CORR = float(os.environ.get('Q6A_ICP_MAX_CORR', '0.35'))   # correspondence reject dist (m)
MIN_PTS = int(os.environ.get('Q6A_ICP_MIN_PTS', '18'))
MAX_STEP_XY = float(os.environ.get('Q6A_ICP_MAX_STEP_XY', '0.15'))  # reject bigger per-frame translation (m)
MAX_STEP_TH = float(os.environ.get('Q6A_ICP_MAX_STEP_TH', '0.15'))  # reject bigger per-frame rotation (rad ~8.6deg)
# sign flips (validated by driving forward and confirming +x): set to -1 if a delta is inverted
SX = float(os.environ.get('Q6A_ODOM_SX', '1'))
SY = float(os.environ.get('Q6A_ODOM_SY', '1'))
STH = float(os.environ.get('Q6A_ODOM_STH', '1'))


def scan_to_points(msg):
    n = len(msg.ranges)
    ang = msg.angle_min + np.arange(n) * msg.angle_increment
    r = np.asarray(msg.ranges, dtype=np.float64)
    m = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    r, ang = r[m], ang[m]
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)   # Nx2 in the laser frame


def icp(P, Q, init=(0.0, 0.0, 0.0), iters=ICP_ITERS, max_corr=ICP_MAX_CORR):
    """Transform (dx,dy,dth) that best aligns current points Q onto previous points P.

    Point-to-point ICP: re-associate each iteration under the running estimate, then solve the
    absolute Q->P rigid transform via SVD over inlier correspondences. init is a motion prior.
    """
    dx, dy, dth = init
    ok = False
    for _ in range(iters):
        c, s = math.cos(dth), math.sin(dth)
        R = np.array([[c, -s], [s, c]])
        Qt = Q @ R.T + np.array([dx, dy])
        # nearest neighbour in P for each transformed Q point (brute force; ~90x90)
        d2 = ((Qt[:, None, :] - P[None, :, :]) ** 2).sum(-1)
        idx = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(Qt.shape[0]), idx])
        keep = dist < max_corr
        if keep.sum() < MIN_PTS:
            break
        A = Q[keep]           # original current points
        B = P[idx][keep]      # matched previous points
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Rr = Vt.T @ U.T
        if np.linalg.det(Rr) < 0:
            Vt[1, :] *= -1
            Rr = Vt.T @ U.T
        dth = math.atan2(Rr[1, 0], Rr[0, 0])
        t = cb - Rr @ ca
        dx, dy = float(t[0]), float(t[1])
        ok = True
    return (dx, dy, dth, ok)


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class LaserOdom(Node):
    def __init__(self):
        super().__init__('q6a_laser_odom')
        self.prev = None                 # previous scan points (Nx2)
        self.x = self.y = self.th = 0.0  # pose in odom
        self.n = 0
        self.rej = 0
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, ODOM_TOPIC, 20)
        # static base_link -> laser (identity: scan already base_link-aligned, turret ~center)
        stf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = BASE_FRAME
        st.child_frame_id = LASER_FRAME
        st.transform.rotation.w = 1.0
        stf.sendTransform(st)
        self._stf = stf                  # keep alive
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # Broadcast odom->base_link at a HIGH rate with the current clock (not just per-scan at the scan
        # stamp). slam_toolbox's tf2 message-filter looks up the transform at each scan's timestamp; a
        # 5 Hz scan-stamped TF races the scans and its filter queue fills ("dropping ... queue is full").
        # A ~30 Hz always-fresh TF keeps the lookup satisfiable so slam actually processes scans.
        self.create_timer(1.0 / 30.0, self.broadcast_tf)
        self.get_logger().info(
            f'q6a_laser_odom up: /scan -> ICP -> {ODOM_TOPIC} + {ODOM_FRAME}->{BASE_FRAME} TF '
            f'(needs the turret spinning, i.e. manual_control active)')

    def on_scan(self, msg):
        pts = scan_to_points(msg)
        if pts.shape[0] < MIN_PTS:
            return
        if self.prev is not None:
            # seed from ZERO (small-motion assumption): a constant-velocity prior lets one bad
            # rotation estimate poison the next frame and run away to a 180-deg flip.
            dx, dy, dth, ok = icp(self.prev, pts, init=(0.0, 0.0, 0.0))
            if ok:
                # Reject implausible per-frame motion. At ~5 Hz and a slow indoor drive, one frame
                # is at most a few cm and a few degrees; anything larger is a bad scan match, not
                # real motion -> skip it (keep the last good pose) rather than integrate garbage.
                if abs(dx) < MAX_STEP_XY and abs(dy) < MAX_STEP_XY and abs(dth) < MAX_STEP_TH:
                    dx, dy, dth = SX * dx, SY * dy, STH * dth
                    # compose body-frame delta into the odom pose
                    self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
                    self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
                    self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
                    self.n += 1
                    if self.n % 25 == 0:
                        self.get_logger().info(
                            f'pose odom: x={self.x:+.2f} y={self.y:+.2f} yaw={math.degrees(self.th):+.0f}')
                else:
                    self.rej += 1
                    if self.rej % 10 == 1:
                        self.get_logger().warn(
                            f'rejected implausible ICP step dx={dx:+.2f} dy={dy:+.2f} '
                            f'dth={math.degrees(dth):+.0f} (total rejected {self.rej})')
        self.prev = pts
        self.publish(msg.header.stamp)

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()   # current time -> always fresh for tf2 lookups
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quat(self.th)
        self.tfb.sendTransform(t)

    def publish(self, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = ODOM_FRAME
        o.child_frame_id = BASE_FRAME
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = yaw_to_quat(self.th)
        self.pub.publish(o)


def main():
    rclpy.init()
    node = LaserOdom()
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

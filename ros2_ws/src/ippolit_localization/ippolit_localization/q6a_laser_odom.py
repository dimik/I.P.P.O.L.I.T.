#!/usr/bin/env python3
"""
q6a_laser_odom.py — 2D laser scan-matching odometry (companion).

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

Parameters are declared below (see ippolit_bringup/config/q6a_laser_odom.yaml for the deployed
values); this replaces the earlier Q6A_ICP_*/Q6A_ODOM_*/Q6A_*_FRAME environment-variable reads
(A2). `icp()`/`scan_to_points()` stay plain functions (no rclpy dependency, testable standalone);
their tunables are passed in explicitly by the node rather than read from module globals.
"""
import math

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def scan_to_points(msg):
    n = len(msg.ranges)
    ang = msg.angle_min + np.arange(n) * msg.angle_increment
    r = np.asarray(msg.ranges, dtype=np.float64)
    m = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    r, ang = r[m], ang[m]
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)   # Nx2 in the laser frame


def icp(P, Q, init=(0.0, 0.0, 0.0), iters=14, max_corr=0.35, min_pts=18):
    """
    Transform (dx,dy,dth) that best aligns current points Q onto previous points P.

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
        if keep.sum() < min_pts:
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
        self.declare_parameter(
            'odom_frame', 'odom', ParameterDescriptor(description='Odometry frame_id.'))
        self.declare_parameter(
            'base_frame', 'base_link', ParameterDescriptor(description='Robot base frame_id.'))
        self.declare_parameter(
            'laser_frame', 'laser', ParameterDescriptor(description='LiDAR frame_id.'))
        self.declare_parameter(
            'odom_topic', '/odom_laser',
            ParameterDescriptor(description='Topic to publish nav_msgs/Odometry on.'))
        self.declare_parameter(
            'icp_iters', 14,
            ParameterDescriptor(
                description='Max ICP re-association/solve iterations per scan pair.',
                integer_range=[IntegerRange(from_value=1, to_value=100)]))
        self.declare_parameter(
            'icp_max_corr', 0.35,
            ParameterDescriptor(
                description='Correspondence rejection distance (m) for ICP point matching.',
                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=5.0)]))
        self.declare_parameter(
            'min_pts', 18,
            ParameterDescriptor(
                description='Minimum inlier correspondences to keep refining an ICP estimate.',
                integer_range=[IntegerRange(from_value=3, to_value=1000)]))
        self.declare_parameter(
            'max_step_xy', 0.15,
            ParameterDescriptor(
                description='Reject a per-frame ICP translation (m) larger than this.',
                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=2.0)]))
        self.declare_parameter(
            'max_step_th', 0.15,
            ParameterDescriptor(
                description='Reject a per-frame ICP rotation (rad) larger than this.',
                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=3.2)]))
        self.declare_parameter(
            'sign_x', 1.0,
            ParameterDescriptor(
                description='Sign/scale flip for the ICP x delta (validated by driving forward).'))
        self.declare_parameter(
            'sign_y', 1.0,
            ParameterDescriptor(description='Sign/scale flip for the ICP y delta.'))
        self.declare_parameter(
            'sign_th', 1.0,
            ParameterDescriptor(description='Sign/scale flip for the ICP rotation delta.'))

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.icp_iters = self.get_parameter('icp_iters').value
        self.icp_max_corr = self.get_parameter('icp_max_corr').value
        self.min_pts = self.get_parameter('min_pts').value
        self.max_step_xy = self.get_parameter('max_step_xy').value
        self.max_step_th = self.get_parameter('max_step_th').value
        self.sx = self.get_parameter('sign_x').value
        self.sy = self.get_parameter('sign_y').value
        self.sth = self.get_parameter('sign_th').value

        self.prev = None                 # previous scan points (Nx2)
        self.x = self.y = self.th = 0.0  # pose in odom
        self.n = 0
        self.rej = 0
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)
        # static base_link -> laser (identity: scan already base_link-aligned, turret ~center)
        stf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.base_frame
        st.child_frame_id = self.laser_frame
        st.transform.rotation.w = 1.0
        stf.sendTransform(st)
        self._stf = stf                  # keep alive
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # Broadcast odom->base_link at a HIGH rate with the current clock (not just per-scan at
        # the scan stamp). slam_toolbox's tf2 message-filter looks up the transform at each
        # scan's timestamp; a 5 Hz scan-stamped TF races the scans and its filter queue fills
        # ("dropping ... queue is full"). A ~30 Hz always-fresh TF keeps the lookup satisfiable
        # so slam actually processes scans.
        self.create_timer(1.0 / 30.0, self.broadcast_tf)
        self.get_logger().info(
            f'q6a_laser_odom up: /scan -> ICP -> {self.odom_topic} + '
            f'{self.odom_frame}->{self.base_frame} TF '
            f'(needs the turret spinning, i.e. manual_control active)')

    def on_scan(self, msg):
        pts = scan_to_points(msg)
        if pts.shape[0] < self.min_pts:
            return
        if self.prev is not None:
            # seed from ZERO (small-motion assumption): a constant-velocity prior lets one bad
            # rotation estimate poison the next frame and run away to a 180-deg flip.
            dx, dy, dth, ok = icp(self.prev, pts, init=(0.0, 0.0, 0.0),
                                  iters=self.icp_iters, max_corr=self.icp_max_corr,
                                  min_pts=self.min_pts)
            if ok:
                # Reject implausible per-frame motion. At ~5 Hz and a slow indoor drive, one frame
                # is at most a few cm and a few degrees; anything larger is a bad scan match, not
                # real motion -> skip it (keep the last good pose) rather than integrate garbage.
                if abs(dx) < self.max_step_xy and abs(dy) < self.max_step_xy \
                        and abs(dth) < self.max_step_th:
                    dx, dy, dth = self.sx * dx, self.sy * dy, self.sth * dth
                    # compose body-frame delta into the odom pose
                    self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
                    self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
                    self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
                    self.n += 1
                    if self.n % 25 == 0:
                        yaw_deg = math.degrees(self.th)
                        self.get_logger().info(
                            f'pose odom: x={self.x:+.2f} y={self.y:+.2f} yaw={yaw_deg:+.0f}')
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
        t.header.stamp = self.get_clock().now().to_msg()   # current time -> always fresh for tf2
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quat(self.th)
        self.tfb.sendTransform(t)

    def publish(self, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
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

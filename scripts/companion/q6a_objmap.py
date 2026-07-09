#!/usr/bin/env python3
"""q6a_objmap.py — semantic object map (companion).

Fuses the perception + localization we already publish into a persistent map of objects:
  /vision/detections (YOLO label/bbox/conf/id + MiDaS disparity)
  /odom               (robot pose in the map frame, from valetudo-bridge)
  /scan               (LiDAR range — metric distance at a bearing; flows only while the turret spins)

Per confident detection: bearing = from the bbox x-center + the camera horizontal FOV; range = /scan at that
bearing (metric; MiDaS disparity is a relative fallback). Project to the map frame via the robot pose,
accumulate persistent objects (same class within MERGE_DIST are merged, position running-averaged), and
publish /object_map (JSON) + /object_markers (RViz MarkerArray).

CALIBRATION (do during the first drive): the camera H-FOV (Q6A_CAM_HFOV_DEG), any camera-yaw offset
(Q6A_CAM_YAW_DEG), and the bearing sign are estimates — tune against RViz (object markers vs the real room).
Room-tagging (which Valetudo segment each object is in) is a TODO refinement.
"""
import json, math, os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
try:
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_MARKERS = True
except Exception:
    HAVE_MARKERS = False

H_FOV = math.radians(float(os.environ.get('Q6A_CAM_HFOV_DEG', '110')))   # OV8856 horizontal FOV (estimate)
CAM_YAW = math.radians(float(os.environ.get('Q6A_CAM_YAW_DEG', '0')))    # camera yaw vs base_link forward
BEAR_SIGN = float(os.environ.get('Q6A_CAM_BEAR_SIGN', '-1'))             # image +x(right) -> which bearing sign
MERGE_DIST = float(os.environ.get('Q6A_OBJMAP_MERGE_M', '0.5'))          # merge same-class within this (m)
MIN_CONF = float(os.environ.get('Q6A_OBJMAP_MIN_CONF', '0.4'))           # map only confident detections
# dynamic/movable classes are NOT persistent scene furniture — never add them to the map (they'd smear
# across the odom track as they/the robot move: e.g. a supervising human reads as hundreds of "person" hits).
DYNAMIC = set(s.strip() for s in os.environ.get(
    'Q6A_OBJMAP_DYNAMIC', 'person,cat,dog,bird').split(',') if s.strip())
POSE_TOPIC = os.environ.get('Q6A_OBJMAP_POSE_TOPIC', '/pose')            # slam_toolbox map-frame pose ('' = off)


class ObjMap(Node):
    def __init__(self):
        super().__init__('q6a_objmap')
        self.pose = None            # (x, y, yaw) in map
        self.scan = None
        self.objects = []           # [{cls, x, y, n, conf}]
        self.slam_pose = False      # once slam's map-frame /pose flows, it wins over /odom (odom drifts)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        if POSE_TOPIC:
            self.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, self.on_posecov, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(String, '/vision/detections', self.on_dets, 10)
        # A bump = a real obstacle the LiDAR can't see (thin table leg, etc.) right in front of us.
        # Record it on the map at the robot's front so we remember + route around it later.
        self.bumped = False
        self.bump_front_m = float(os.environ.get('Q6A_OBJMAP_BUMP_FRONT_M', '0.20'))
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        _latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/bumper', self.on_bumper, _latched)
        self.pub_map = self.create_publisher(String, '/object_map', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/object_markers', 10) if HAVE_MARKERS else None
        self.create_timer(2.0, self.publish)
        self.get_logger().info(f'q6a_objmap up (HFOV={math.degrees(H_FOV):.0f}deg, merge={MERGE_DIST}m, '
                               f'min_conf={MIN_CONF}); needs /vision/detections + /odom (+ /scan for range)')

    @staticmethod
    def _to_pose(pose):
        p = pose.position; q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    def on_odom(self, m):
        if self.slam_pose:
            return                  # slam's map-frame /pose is authoritative; don't mix frames
        self.pose = self._to_pose(m.pose.pose)

    def on_posecov(self, m):
        if not self.slam_pose:
            self.slam_pose = True
            self.get_logger().info(f'switched pose source to {POSE_TOPIC} (slam map frame)')
        self.pose = self._to_pose(m.pose.pose)

    def on_scan(self, m):
        self.scan = m

    def scan_range(self, bearing):
        """LiDAR range (m) at a base_link bearing (rad), or None. /scan is robot-relative (laser frame)."""
        s = self.scan
        if s is None:
            return None
        i = int(round((bearing - s.angle_min) / s.angle_increment)) % len(s.ranges)
        # take the nearest finite reading in a small window (robust to a single-bin dropout)
        for d in (0, 1, -1, 2, -2):
            r = s.ranges[(i + d) % len(s.ranges)]
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                return r
        return None

    def on_dets(self, msg):
        if self.pose is None:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        w = data.get('w', 672)
        xr, yr, yaw = self.pose
        for det in data.get('dets', []):
            if det.get('conf', 0) < MIN_CONF or det.get('label') in DYNAMIC:
                continue                                   # skip low-conf + dynamic (person/pets) detections
            x1, y1, x2, y2 = det['bbox']
            xc = (x1 + x2) / 2.0
            bearing = BEAR_SIGN * ((xc / w) - 0.5) * H_FOV + CAM_YAW    # base_link bearing to the object
            rng = self.scan_range(bearing)                             # metric range (needs turret spinning)
            if rng is None:
                continue                                               # no LiDAR range yet -> can't place
            xm = xr + rng * math.cos(yaw + bearing)
            ym = yr + rng * math.sin(yaw + bearing)
            self.merge(det['label'], xm, ym, det.get('conf', 0.0))

    def on_bumper(self, m):
        if m.data and not self.bumped:      # rising edge = one obstacle mark per distinct hit
            self.bumped = True
            if self.pose is not None:
                xr, yr, yaw = self.pose
                bx = xr + self.bump_front_m * math.cos(yaw)     # just ahead of the robot = where it hit
                by = yr + self.bump_front_m * math.sin(yaw)
                self.merge('obstacle', bx, by, 1.0)
                self.get_logger().info(f'bump -> obstacle mapped at ({bx:.2f}, {by:.2f})')
        elif not m.data:
            self.bumped = False

    def merge(self, cls, x, y, conf):
        for o in self.objects:
            if o['cls'] == cls and math.hypot(o['x'] - x, o['y'] - y) < MERGE_DIST:
                o['x'] = (o['x'] * o['n'] + x) / (o['n'] + 1)
                o['y'] = (o['y'] * o['n'] + y) / (o['n'] + 1)
                o['n'] += 1
                o['conf'] = max(o['conf'], conf)
                return
        self.objects.append({'cls': cls, 'x': x, 'y': y, 'n': 1, 'conf': conf})

    def publish(self):
        self.pub_map.publish(String(data=json.dumps(
            {'objects': [{'cls': o['cls'], 'x': round(o['x'], 3), 'y': round(o['y'], 3),
                          'n': o['n'], 'conf': round(o['conf'], 3)} for o in self.objects]})))
        if self.pub_mk is not None:
            ma = MarkerArray()
            for i, o in enumerate(self.objects):
                mk = Marker()
                mk.header.frame_id = 'map'; mk.header.stamp = self.get_clock().now().to_msg()
                mk.ns = 'objects'; mk.id = i; mk.type = Marker.SPHERE; mk.action = Marker.ADD
                mk.pose.position.x = o['x']; mk.pose.position.y = o['y']; mk.pose.position.z = 0.1
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 0.2
                mk.color.a = 0.9; mk.color.r = 1.0; mk.color.g = 0.4; mk.color.b = 0.0
                ma.markers.append(mk)
                t = Marker()
                t.header = mk.header; t.ns = 'labels'; t.id = i; t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD; t.pose.position.x = o['x']; t.pose.position.y = o['y']; t.pose.position.z = 0.35
                t.pose.orientation.w = 1.0; t.scale.z = 0.18
                t.color.a = 1.0; t.color.r = t.color.g = t.color.b = 1.0
                t.text = f"{o['cls']} ({o['n']})"
                ma.markers.append(t)
            self.pub_mk.publish(ma)
        if self.objects:
            self.get_logger().info(f"{len(self.objects)} objects: " +
                                   ', '.join(f"{o['cls']}@({o['x']:.1f},{o['y']:.1f})x{o['n']}"
                                             for o in sorted(self.objects, key=lambda o: -o['n'])[:8]))


def main():
    rclpy.init()
    node = ObjMap()
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

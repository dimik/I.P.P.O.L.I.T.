#!/usr/bin/env python3
"""
q6a_objmap.py — semantic object map (companion).

Fuses the perception + localization we already publish into a persistent map of objects:
  /vision/detections (vision_msgs/Detection2DArray: label/score/bbox/track id -- typed per A3)
  /odom               (robot pose in the map frame, from valetudo-bridge)
  /scan               (LiDAR range — metric distance at a bearing; only while the turret spins)

Per confident detection: bearing = from the bbox x-center + the camera horizontal FOV; range =
/scan at that bearing (metric). Project to the map frame via the robot pose, accumulate
persistent objects (same class within merge_dist_m are merged, position running-averaged), and
publish /object_map (ippolit_interfaces/MappedObjectArray, typed per A3) + /object_markers
(RViz MarkerArray).

CALIBRATION (do during the first drive): the camera H-FOV (cam_hfov_deg), any camera-yaw offset
(cam_yaw_deg), and the bearing sign are estimates — tune against RViz (object markers vs the real
room). Room-tagging (which Valetudo segment each object is in) is a TODO refinement.

DISK PERSISTENCE (added 2026-07-12): until this, the object list lived only in memory -- a node
restart or reboot lost everything, forcing a full re-drive to rebuild it. Now loads the objmap_file
param (default /home/radxa/ros/maps/object_map.json) at startup if present, and saves it
periodically + on clean shutdown (atomic write: temp file + rename, so a mid-write crash/power-cut
can't corrupt the persisted file -- worst case you lose the last save_period_s of updates, not the
whole map). Same limitation as the slam_toolbox map: only a CLEAN stop triggers the shutdown save;
a hard power loss just loses anything since the last periodic save. This on-disk JSON format is
unaffected by A3 -- it's an implementation detail of this node, not a ROS interface.

Parameters are declared below (see ippolit_bringup/config/q6a_objmap.yaml for the deployed
values); this replaces the earlier Q6A_CAM_*/Q6A_OBJMAP_* environment-variable reads (A2).
"""
import json
import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from ippolit_interfaces.msg import MappedObject, MappedObjectArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
try:
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_MARKERS = True
except Exception:
    HAVE_MARKERS = False

_DEFAULT_ALLOW = [
    'chair', 'couch', 'bed', 'dining table', 'tv', 'refrigerator', 'oven', 'microwave',
    'sink', 'toilet', 'potted plant', 'bench', 'book', 'clock', 'vase', 'suitcase',
]


class ObjMap(Node):
    def __init__(self):
        super().__init__('q6a_objmap')
        self.declare_parameter(
            'cam_hfov_deg', 110.0,
            ParameterDescriptor(
                description='OV8856 horizontal FOV estimate (deg); tune against RViz.',
                floating_point_range=[FloatingPointRange(from_value=30.0, to_value=170.0)]))
        self.declare_parameter(
            'cam_yaw_deg', 0.0,
            ParameterDescriptor(
                description='Camera yaw offset vs base_link forward (deg).',
                floating_point_range=[FloatingPointRange(from_value=-180.0, to_value=180.0)]))
        self.declare_parameter(
            'bear_sign', -1.0,
            ParameterDescriptor(description='Sign flip for image +x(right) -> bearing.'))
        self.declare_parameter(
            'img_width', 672,
            ParameterDescriptor(
                description=(
                    'Camera frame width in pixels (fixed by the OV8856/camstream pipeline; '
                    'Detection2DArray carries bbox pixel coords but not frame dimensions, so '
                    'this is needed to normalize the bbox x-center for the bearing calc).'),
                integer_range=[IntegerRange(from_value=1, to_value=8192)]))
        self.declare_parameter(
            'merge_dist_m', 0.5,
            ParameterDescriptor(
                description='Merge same-class detections within this distance (m).',
                floating_point_range=[FloatingPointRange(from_value=0.05, to_value=5.0)]))
        self.declare_parameter(
            'min_conf', 0.45,
            ParameterDescriptor(
                description=(
                    'Minimum detection confidence to map. The allow_labels filter is what '
                    'blocks hallucinations now, so this can be lower to catch borderline REAL '
                    'furniture (e.g. fridge ~0.5).'),
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter(
            'min_n', 3,
            ParameterDescriptor(
                description='Publish only objects seen at least this many times.',
                integer_range=[IntegerRange(from_value=1, to_value=100)]))
        self.declare_parameter(
            'allow_labels', _DEFAULT_ALLOW,
            ParameterDescriptor(
                description=(
                    'Lower-cased YOLO labels eligible to be mapped as static room furniture. '
                    'The model hallucinates other classes (cat, laptop, pizza...) that a '
                    'confidence gate alone cannot suppress; an allowlist is robust.')))
        self.declare_parameter(
            'pose_topic', '/pose',
            ParameterDescriptor(
                description='slam_toolbox map-frame pose topic; empty falls back to /odom only.'))
        self.declare_parameter(
            'objmap_file', '/home/radxa/ros/maps/object_map.json',
            ParameterDescriptor(description="Disk persistence path ('' disables persistence)."))
        self.declare_parameter(
            'save_period_s', 30.0,
            ParameterDescriptor(
                description='Seconds between periodic disk saves.',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=600.0)]))
        self.declare_parameter(
            'bump_front_m', 0.20,
            ParameterDescriptor(
                description='Distance ahead of the robot to mark a bump-detected obstacle (m).',
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))

        self.h_fov = math.radians(self.get_parameter('cam_hfov_deg').value)
        self.cam_yaw = math.radians(self.get_parameter('cam_yaw_deg').value)
        self.bear_sign = self.get_parameter('bear_sign').value
        self.img_width = self.get_parameter('img_width').value
        self.merge_dist = self.get_parameter('merge_dist_m').value
        self.min_conf = self.get_parameter('min_conf').value
        self.min_n = self.get_parameter('min_n').value
        self.allow = {lab.strip().lower() for lab in self.get_parameter('allow_labels').value
                      if lab.strip()}
        self.pose_topic = self.get_parameter('pose_topic').value
        self.objmap_file = self.get_parameter('objmap_file').value
        self.save_period_s = self.get_parameter('save_period_s').value
        self.bump_front_m = self.get_parameter('bump_front_m').value

        self.pose = None            # (x, y, yaw) in map
        self.scan = None
        self.objects = []           # [{cls, x, y, n, conf}]
        self.slam_pose = False      # once slam's map-frame /pose flows, it wins over /odom
        n_loaded = self.load()
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        if self.pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped, self.pose_topic, self.on_posecov, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Detection2DArray, '/vision/detections', self.on_dets, 10)
        # A bump = a real obstacle the LiDAR can't see (thin table leg, etc.) right in front of
        # us. Record it on the map at the robot's front so we remember + route around it later.
        self.bumped = False
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/bumper', self.on_bumper, latched)
        self.pub_map = self.create_publisher(MappedObjectArray, '/object_map', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/object_markers', 10) \
            if HAVE_MARKERS else None
        self.create_timer(2.0, self.publish)
        if self.objmap_file:
            self.create_timer(self.save_period_s, self.save)
        self.get_logger().info(
            f'q6a_objmap up (HFOV={math.degrees(self.h_fov):.0f}deg, merge={self.merge_dist}m, '
            f'min_conf={self.min_conf}, loaded {n_loaded} objects from disk); needs '
            f'/vision/detections + /odom (+ /scan for range)')

    def load(self):
        if not self.objmap_file or not os.path.exists(self.objmap_file):
            return 0
        try:
            with open(self.objmap_file) as f:
                data = json.load(f)
            self.objects = data.get('objects', [])
            return len(self.objects)
        except Exception as e:
            self.get_logger().warn(
                f'load {self.objmap_file} failed: {e} -- starting from an empty map')
            self.objects = []
            return 0

    def save(self):
        if not self.objmap_file:
            return
        try:
            os.makedirs(os.path.dirname(self.objmap_file), exist_ok=True)
            tmp = self.objmap_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'objects': self.objects}, f)
            os.replace(tmp, self.objmap_file)  # atomic same-fs -- no half-written file on crash
        except Exception as e:
            self.get_logger().warn(f'save {self.objmap_file} failed: {e}')

    @staticmethod
    def _to_pose(pose):
        p = pose.position
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    def on_odom(self, m):
        if self.slam_pose:
            return                  # slam's map-frame /pose is authoritative; don't mix frames
        self.pose = self._to_pose(m.pose.pose)

    def on_posecov(self, m):
        if not self.slam_pose:
            self.slam_pose = True
            self.get_logger().info(f'switched pose source to {self.pose_topic} (slam map frame)')
        self.pose = self._to_pose(m.pose.pose)

    def on_scan(self, m):
        self.scan = m

    def scan_range(self, bearing):
        """Return LiDAR range (m) at a base_link bearing (rad); None if unavailable."""
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
        xr, yr, yaw = self.pose
        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf < self.min_conf or label.lower() not in self.allow:
                continue                                   # skip low-conf + non-furniture
            xc = det.bbox.center.position.x                # already center-of-bbox, in pixels
            bearing = (self.bear_sign * ((xc / self.img_width) - 0.5) * self.h_fov
                       + self.cam_yaw)                      # base_link bearing
            rng = self.scan_range(bearing)                  # metric range (needs turret spinning)
            if rng is None:
                continue                                    # no LiDAR range yet -> can't place
            xm = xr + rng * math.cos(yaw + bearing)
            ym = yr + rng * math.sin(yaw + bearing)
            self.merge(label, xm, ym, conf)

    def on_bumper(self, m):
        if m.data and not self.bumped:      # rising edge = one obstacle mark per distinct hit
            self.bumped = True
            if self.pose is not None:
                xr, yr, yaw = self.pose
                bx = xr + self.bump_front_m * math.cos(yaw)   # just ahead = where it hit
                by = yr + self.bump_front_m * math.sin(yaw)
                self.merge('obstacle', bx, by, 1.0)
                self.get_logger().info(f'bump -> obstacle mapped at ({bx:.2f}, {by:.2f})')
        elif not m.data:
            self.bumped = False

    def merge(self, cls, x, y, conf):
        for o in self.objects:
            if o['cls'] == cls and math.hypot(o['x'] - x, o['y'] - y) < self.merge_dist:
                o['x'] = (o['x'] * o['n'] + x) / (o['n'] + 1)
                o['y'] = (o['y'] * o['n'] + y) / (o['n'] + 1)
                o['n'] += 1
                o['conf'] = max(o['conf'], conf)
                return
        self.objects.append({'cls': cls, 'x': x, 'y': y, 'n': 1, 'conf': conf})

    def publish(self):
        # persistence gate: only surface objects seen >= min_n times (a transient YOLO false
        # positive stays at n=1-2). Bump 'obstacle' marks are deliberate ground truth -> kept.
        pub = [o for o in self.objects if o['n'] >= self.min_n or o['cls'] == 'obstacle']
        map_msg = MappedObjectArray()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        for o in pub:
            mo = MappedObject()
            mo.cls = o['cls']
            mo.position.x = o['x']
            mo.position.y = o['y']
            mo.n = o['n']
            mo.conf = o['conf']
            # room-tagging is a TODO refinement (see module docstring); left blank for now
            map_msg.objects.append(mo)
        self.pub_map.publish(map_msg)
        if self.pub_mk is not None:
            ma = MarkerArray()
            for i, o in enumerate(pub):
                mk = Marker()
                mk.header.frame_id = 'map'
                mk.header.stamp = self.get_clock().now().to_msg()
                mk.ns = 'objects'
                mk.id = i
                mk.type = Marker.SPHERE
                mk.action = Marker.ADD
                mk.pose.position.x = o['x']
                mk.pose.position.y = o['y']
                mk.pose.position.z = 0.1
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 0.2
                mk.color.a = 0.9
                mk.color.r = 1.0
                mk.color.g = 0.4
                mk.color.b = 0.0
                ma.markers.append(mk)
                t = Marker()
                t.header = mk.header
                t.ns = 'labels'
                t.id = i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = o['x']
                t.pose.position.y = o['y']
                t.pose.position.z = 0.35
                t.pose.orientation.w = 1.0
                t.scale.z = 0.18
                t.color.a = 1.0
                t.color.r = t.color.g = t.color.b = 1.0
                t.text = f"{o['cls']} ({o['n']})"
                ma.markers.append(t)
            self.pub_mk.publish(ma)
        if pub:
            top = sorted(pub, key=lambda o: -o['n'])[:8]
            summary = ', '.join(f"{o['cls']}@({o['x']:.1f},{o['y']:.1f})x{o['n']}" for o in top)
            self.get_logger().info(f'{len(pub)} mapped (of {len(self.objects)} raw): {summary}')


def main():
    rclpy.init()
    node = ObjMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            # best-effort final save on a CLEAN stop; a hard power-cut/SIGKILL skips this.
            node.save()
        except Exception:
            # pure file I/O so this should succeed regardless of ROS context state, but don't
            # let a failure here (e.g. the except branch's own get_logger call, if rclpy's
            # SIGINT handler already tore the context down -- see q6a_map_persist.py's docstring
            # for the same class of bug) skip destroy_node()/shutdown() below.
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

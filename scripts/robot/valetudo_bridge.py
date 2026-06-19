#!/usr/bin/env python3
"""valetudo_bridge.py — Valetudo REST/SSE -> ROS 2 bridge.

Runs in the robot's chroot (ROS 2 Jazzy). Publishes the robot's map + pose + status to ROS WITHOUT
an MQTT broker, and WITHOUT polling: one REST fetch seeds the initial map, then two SSE streams push
updates. Raw IMU/odometry over /dev/ttyS4 is deferred (the LD_PRELOAD read-tap destabilises AVA —
see docs/sensors.md / mcutap.c); this covers v1 nav needs from what Valetudo already exposes.

Why REST-seed + SSE (not pure SSE): Valetudo's map SSE is push-on-CHANGE only and sends no initial
snapshot, so a freshly-started/docked robot would have no map until it next moves. We GET the map
once for the seed, then ride the SSE — instant updates while cleaning, zero HTTP traffic when idle.

Publishes:
  /map           nav_msgs/OccupancyGrid   floor(0)/wall(100)/unknown(-1)   [latched]
  /odom          nav_msgs/Odometry        robot_position (pose only)
  TF map->base_link
  /robot/status  std_msgs/String          StatusStateAttribute

Coords: Valetudo is mm, +y DOWN; ROS (REP-103) is m, +y UP -> x=x_mm/1000, y=-y_mm/1000, yaw=-deg.

Run:  source /opt/ros/jazzy/setup.bash && python3 valetudo_bridge.py [--host http://127.0.0.1]
"""
import argparse, json, math, threading, time, urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from array import array


class ValetudoBridge(Node):
    def __init__(self, host):
        super().__init__('valetudo_bridge')
        self.host = host.rstrip('/')
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)  # latched for late subs
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_status = self.create_publisher(String, '/robot/status', 10)
        self.tf = TransformBroadcaster(self)
        self.pose = None   # (x, y, qz, qw), updated by SSE/seed, republished by the heartbeat
        # heartbeat: republish cached /odom + TF at 2 Hz so they stay fresh while idle (no HTTP —
        # the data comes from the SSE/seed; this just keeps TF from going stale for nav consumers)
        self.create_timer(0.5, self.pub_pose)

        # seed the initial map+pose once via REST (the map SSE sends no snapshot on connect)
        try:
            self.handle_map(self.fetch('/api/v2/robot/state/map'))
            self.get_logger().info('seeded initial map via REST')
        except Exception as e:
            self.get_logger().warn(f'initial map seed failed (will get it on first SSE update): {e}')

        # then push-driven: one SSE stream for the map, one for state attributes (no polling)
        threading.Thread(target=self.sse_loop,
                         args=('/api/v2/robot/state/map/sse', 'MapUpdated', self.handle_map),
                         daemon=True).start()
        threading.Thread(target=self.sse_loop,
                         args=('/api/v2/robot/state/attributes/sse', 'StateAttributesUpdated', self.handle_attrs),
                         daemon=True).start()
        self.get_logger().info(f'valetudo_bridge up (REST seed + SSE) on {self.host}')

    # ---- HTTP ----
    def fetch(self, path):
        with urllib.request.urlopen(self.host + path, timeout=4) as r:
            return json.load(r)

    def sse_loop(self, path, want_event, handler):
        """Long-lived SSE: accumulate event:/data: lines, dispatch on blank line; reconnect on drop."""
        while rclpy.ok():
            try:
                with urllib.request.urlopen(self.host + path, timeout=None) as resp:
                    event, data = None, []
                    for raw in resp:
                        if not rclpy.ok():
                            return
                        line = raw.decode('utf-8', 'replace').rstrip('\r\n')
                        if line.startswith('event:'):
                            event = line[6:].strip()
                        elif line.startswith('data:'):
                            data.append(line[5:].strip())
                        elif line == '':                     # end of one SSE event
                            if event == want_event and data:
                                try:
                                    handler(json.loads('\n'.join(data)))
                                except Exception as e:
                                    self.get_logger().warn(f'{path} parse: {e}')
                            event, data = None, []
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().warn(f'{path} dropped ({e}); reconnecting')
                    time.sleep(2.0)

    # ---- publishers ----
    def handle_map(self, m):
        now = self.get_clock().now().to_msg()
        ps = m['pixelSize']
        res = ps / 1000.0
        xs, ys = [], []
        for L in m.get('layers', []):
            d = L['dimensions']
            xs += [d['x']['min'], d['x']['max']]
            ys += [d['y']['min'], d['y']['max']]
        if xs:
            minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
            W, H = maxx - minx + 1, maxy - miny + 1
            grid = array('b', [-1]) * (W * H)
            for L in m.get('layers', []):
                val = 100 if L['type'] == 'wall' else 0
                cp = L.get('compressedPixels', []) or []
                for k in range(0, len(cp), 3):
                    x, y, cnt = cp[k], cp[k + 1], cp[k + 2]
                    r = maxy - y                              # ROS y-up: row 0 = max Valetudo-y
                    if 0 <= r < H:
                        for dx in range(cnt):
                            c = x - minx + dx
                            if 0 <= c < W:
                                i = r * W + c
                                if val == 100 or grid[i] != 100:
                                    grid[i] = val
            og = OccupancyGrid()
            og.header.stamp = now
            og.header.frame_id = 'map'
            og.info.resolution = res
            og.info.width = W
            og.info.height = H
            og.info.origin.position.x = minx * res
            og.info.origin.position.y = -maxy * res
            og.info.origin.orientation.w = 1.0
            og.data = grid
            self.pub_map.publish(og)

        rp = next((e for e in m.get('entities', []) if e.get('type') == 'robot_position'), None)
        if rp and rp.get('points'):
            x = rp['points'][0] / 1000.0
            y = -rp['points'][1] / 1000.0
            yaw = math.radians(-(rp.get('metaData') or {}).get('angle', 0))
            self.pose = (x, y, math.sin(yaw / 2.0), math.cos(yaw / 2.0))   # heartbeat republishes it

    def pub_pose(self):
        """heartbeat (timer): republish the cached pose as /odom + map->base_link TF."""
        if self.pose is None:
            return
        x, y, qz, qw = self.pose
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def handle_attrs(self, attrs):
        st = next((a for a in attrs if a.get('__class') == 'StatusStateAttribute'), None)
        if st:
            self.pub_status.publish(String(data=f"{st.get('value')}/{st.get('flag')}"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='http://127.0.0.1')
    a = ap.parse_args()
    rclpy.init()
    node = ValetudoBridge(a.host)
    try:
        rclpy.spin(node)               # no timers/subs; just keeps the node alive (threads publish)
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

#!/usr/bin/env python3
"""lds_scan_node.py — publish the robot's own LiDAR as sensor_msgs/LaserScan on /scan.

Reads the raw ttyS3 byte stream that libserialtap.so tees into the tmpfs shm ring
(/tmp/lds_ring.buf), decodes LDS frames (lds_decode), accumulates one revolution, and
publishes /scan. Runs in the chroot's ROS 2 (Jazzy). No MQTT, no polling of AVA.

Pipeline:  AVA read(ttyS3) --[libserialtap.so]--> /tmp/lds_ring.buf --[this node]--> /scan

Calibration (from cross-validation vs Valetudo's SLAM map): the LDS angle index runs OPPOSITE
the ROS CCW convention (handedness -1), and LDS angle 0 ~ robot forward. So
    base_link bearing = radians(-lds_deg + ANGLE_OFFSET_DEG).
ANGLE_OFFSET_DEG is a parameter — eyeball the scan against walls in RViz once and tune (~0-5deg).

Run:  source /opt/ros/jazzy/setup.bash && python3 lds_scan_node.py
"""
import math
import mmap
import os
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lds_decode  # noqa: E402

RING_PATH = '/tmp/lds_ring.buf'
HDR = 64
RING = 256 * 1024
MAGIC = 0x0031534444530001
BINS = 360                       # 1deg output resolution (LDS native ~1140/rev)


class LdsScanNode(Node):
    def __init__(self):
        super().__init__('lds_scan_node')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('range_min', 0.10)
        self.declare_parameter('range_max', 8.0)
        self.frame_id = self.get_parameter('frame_id').value
        self.offset = math.radians(self.get_parameter('angle_offset_deg').value)
        self.rmin = self.get_parameter('range_min').value
        self.rmax = self.get_parameter('range_max').value
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.mm = None
        self.read_pos = 0
        self.buf = bytearray()
        self.bins = [math.inf] * BINS
        self.prev_start = None
        self.have = False
        self.create_timer(0.01, self.poll)   # 100 Hz drain; lidar rev ~5 Hz
        self.get_logger().info('lds_scan_node up; waiting for ring data (turret must be spinning)')

    def open_ring(self):
        if self.mm is not None:
            return True
        if not os.path.exists(RING_PATH):
            return False
        try:
            fd = os.open(RING_PATH, os.O_RDONLY)
            self.mm = mmap.mmap(fd, HDR + RING, mmap.MAP_SHARED, mmap.PROT_READ)
            os.close(fd)
            magic = struct.unpack_from('<Q', self.mm, 8)[0]
            if magic != MAGIC:
                self.get_logger().warn(f'ring magic {magic:#x} != {MAGIC:#x}; waiting for tap')
                self.mm.close(); self.mm = None
                return False
            self.read_pos = struct.unpack_from('<Q', self.mm, 0)[0]   # skip backlog, start fresh
            self.get_logger().info('ring mapped')
            return True
        except Exception as e:
            self.get_logger().warn(f'ring open failed: {e}')
            return False

    def drain(self):
        """Return new bytes written since last drain (handles wrap + reader-fell-behind)."""
        wp = struct.unpack_from('<Q', self.mm, 0)[0]
        avail = wp - self.read_pos
        if avail <= 0:
            return b''
        if avail > RING:                          # we fell behind; skip to the freshest RING bytes
            self.read_pos = wp - RING
            avail = RING
        start = self.read_pos % RING
        end = wp % RING
        base = HDR
        if start < end:
            out = self.mm[base + start:base + end]
        else:                                     # wrapped
            out = self.mm[base + start:base + RING] + self.mm[base:base + end]
        self.read_pos = wp
        return bytes(out)

    def poll(self):
        if not self.open_ring():
            return
        chunk = self.drain()
        if not chunk:
            return
        self.buf += chunk
        if len(self.buf) > 4 * RING:              # safety clamp
            self.buf = self.buf[-RING:]
        # extract aligned 40-byte frames (55 aa .. a4)
        i, n, consumed = 0, len(self.buf), 0
        while i < n - 39:
            if self.buf[i] == 0x55 and self.buf[i+1] == 0xAA and self.buf[i+2] == 0x03 and self.buf[i+3] == 0x08:
                self.ingest(self.buf[i:i + 40]); i += 40; consumed = i
            else:
                i += 1
        # keep the unparsed tail (last <40 bytes or trailing partial)
        self.buf = self.buf[consumed:] if consumed else self.buf[-40:]

    def ingest(self, frame):
        d = lds_decode.decode_frame(frame)
        s = d['start_angle']
        if self.prev_start is not None and self.have and ((s - self.prev_start) & 0xFFFF) > 0x8000:
            self.publish()                        # start angle wrapped backwards -> revolution done
            self.bins = [math.inf] * BINS
            self.have = False
        self.prev_start = s
        for ang_deg, dist_mm, q in d['samples']:
            if dist_mm is None:
                continue
            bearing = -math.radians(ang_deg) + self.offset
            b = int((bearing % (2 * math.pi)) / (2 * math.pi) * BINS) % BINS
            r = dist_mm / 1000.0
            if self.rmin <= r <= self.rmax and r < self.bins[b]:
                self.bins[b] = r
                self.have = True

    def publish(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi * (BINS - 1) / BINS
        msg.angle_increment = 2 * math.pi / BINS
        msg.range_min = float(self.rmin)
        msg.range_max = float(self.rmax)
        msg.ranges = [float(r) for r in self.bins]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LdsScanNode()
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

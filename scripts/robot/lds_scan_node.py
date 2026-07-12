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
import socket
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
FAST_PERIOD = 0.02               # 50 Hz drain while data flows (lidar rev ~5 Hz; ample headroom)
IDLE_PERIOD = 0.5                # 2 Hz when the ring is dry (turret gated off) — avoids the old
                                 # fixed 100 Hz busy-poll that burned ~6% CPU 24/7 with no LiDAR
IDLE_AFTER = 25                  # consecutive empty polls (~0.5 s) before backing off to idle rate


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

        # source: '' = local tmpfs ring (on-robot); 'host:port' = raw bytes over TCP from ring_forward.py on
        # the robot (lets this node run on the COMPANION with ROS off the robot). Decode/publish is identical.
        self.declare_parameter('source', '')
        src = self.get_parameter('source').value
        self.src = None
        if src:
            h, _, p = src.partition(':')
            self.src = (h, int(p))
        self.sock = None
        self.mm = None
        self.read_pos = 0
        self.buf = bytearray()
        self.bins = [math.inf] * BINS
        self.prev_start = None
        self.have = False
        self.empty = 0
        self.cur_period = FAST_PERIOD
        self.timer = self.create_timer(FAST_PERIOD, self.poll)   # adaptive: 50 Hz active / 2 Hz idle
        self.get_logger().info('lds_scan_node up; waiting for ring data (turret must be spinning)')

    def _set_period(self, p):
        if p != self.cur_period:
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(p, self.poll)
            self.cur_period = p

    def open_tcp(self):
        """Connect to ring_forward on the robot; raw ring bytes stream in (non-blocking recv in drain)."""
        if self.sock is not None:
            return True
        try:
            s = socket.create_connection(self.src, timeout=2.0)
            s.setblocking(False)
            self.sock = s
            self.get_logger().info(f'connected to ring_forward {self.src[0]}:{self.src[1]}')
            return True
        except Exception as e:
            self.get_logger().warn(f'ring_forward connect failed: {e}')
            return False

    def drain_tcp(self):
        chunks = []
        try:
            while True:
                b = self.sock.recv(65536)
                if b == b'':
                    raise ConnectionError('ring_forward closed')
                chunks.append(b)
        except BlockingIOError:
            pass
        except (OSError, ConnectionError) as e:
            self.get_logger().warn(f'ring_forward recv: {e}; will reconnect')
            try: self.sock.close()
            except Exception: pass
            self.sock = None
        return b''.join(chunks)

    def open_ring(self):
        if self.src is not None:
            return self.open_tcp()
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
        """Return new bytes since last drain. TCP source (companion) or local ring (on-robot)."""
        if self.src is not None:
            return self.drain_tcp()
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
            self._set_period(IDLE_PERIOD)
            return
        chunk = self.drain()
        if not chunk:
            self.empty += 1
            if self.empty >= IDLE_AFTER:
                self._set_period(IDLE_PERIOD)   # ring dry (LiDAR off) -> stop busy-polling
            return
        self.empty = 0
        self._set_period(FAST_PERIOD)           # data flowing -> drain at full rate
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
            # SIGN FIXED 2026-07-12: was -ang_deg (assumed "LDS runs opposite ROS CCW", handedness -1, from a
            # single 2026-06-19 Valetudo-SLAM heading comparison). Cross-checked with an independent sensor
            # (MiDaS+camera) today: a wall confirmed on the robot's TRUE right (by camera view, wall occupies
            # the right/center of frame) was reading at LiDAR bearing +90 (our "left") even after fixing the
            # front offset -- a pure offset can't cause a left/right swap, only a wrong sign can. Flipped.
            bearing = math.radians(ang_deg) + self.offset
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

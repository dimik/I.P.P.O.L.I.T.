#!/usr/bin/env python3
"""mcu_node.py — publish the robot's MCU IMU + wheel odometry to ROS.

Reads the raw /dev/ttyS4 byte stream that libserialtap.so tees into the tmpfs shm ring
(/tmp/mcu_ring.buf), decodes the MCU frames, and publishes:
    /imu/data     sensor_msgs/Imu       (Status10ms type 0x02 — BMI055 gyro + accel)
    /odom/wheel   nav_msgs/Odometry     (Status20ms type 0x01 — raw wheel dead-reckoning)

Pipeline:  AVA read(ttyS4) --[libserialtap.so]--> /tmp/mcu_ring.buf --[this node]--> /imu/data,/odom/wheel

Frame:  3c | len(1) | type(1) | payload(len) | crc16(2, big-endian) | 3e
        CRC = Modbus-16 over [len,type,payload] (the MCU emits occasional corrupt frames — drop on
        CRC fail). Frame FORMATS from github.com/alufers/dreame_mcu_protocol (Z10), but SCALINGS
        differ — the Z10 pre-scaled, the D10s sends RAW sensor LSB (see params below):
          Status10ms `<Ihhhhhhbb`: ts, gyro_xyz, accel_xyz (raw LSB), leftDis,rightDis(mm)
          Status20ms `<Iiihhhhhhh`: ts, x,y(0.1mm), yaw(/100 °), yaw_int, L/R vel, edgeDis, 2x current

IMU notes: orientation is left UNKNOWN (no magnetometer; covariance[0]=-1 per REP-145) — we publish
angular_velocity (rad/s) + linear_acceleration (m/s²) for downstream EKF fusion. Gyro bias is
auto-calibrated from the first ~3 s assuming the robot is stationary at startup (it boots docked).
Axis/sign alignment vs base_link is a v1 passthrough — verify in RViz and adjust if needed.

Run:  source /opt/ros/jazzy/setup.bash && python3 mcu_node.py
"""
import math
import mmap
import socket
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8

RING_PATH = '/tmp/mcu_ring.buf'
HDR = 64
RING = 256 * 1024
MAGIC = 0x0031534444530001
DEG2RAD = math.pi / 180.0
G = 9.80665

# Modbus-16 (reflected, poly 0xA001) — matches dreame_mcu_protocol CRC_GetModbus16.
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class McuNode(Node):
    def __init__(self):
        super().__init__('mcu_node')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # ⚠️ The D10s emits the IMU stream (Status10ms / 0x02) ONLY WHEN ACTIVE (moving / cleaning) —
        # NOT when docked or idle (only wheel odom 0x01 flows then). So /imu/data only publishes during
        # activity, and a "hold still at startup" bias calibration can't work (no data when docked, and
        # data only arrives once moving). Instead: ADAPTIVE bias — update the gyro bias only while the
        # robot is detected still (|gyro-bias| below a threshold across a window), and publish always.
        # On the brief still moment when the IMU wakes (before the robot drives), bias self-calibrates.
        self.declare_parameter('still_window', 40)       # samples for the still detector
        self.declare_parameter('still_thresh_dps', 1.5)  # |gyro| below this (°/s) = "still"
        # The D10s does NOT match the Z10 scalings — the Z10 firmware pre-scaled (mg, centideg/s);
        # the D10s passes RAW sensor LSB. Both CONFIRMED empirically on the D10s (imu_type=2):
        #   accel: |accel| at rest = 16384 raw => 1g = 16384 LSB (±2g 16-bit) => accel_scale = 1/16384
        #   gyro:  spin cross-check vs wheel-odom yaw (2 runs, both directions: 0.01526/0.01527 °/s per
        #          LSB) => ±500°/s => gyro_scale = 1/65.536. (NOT the Z10's /100, NOT ±2000°/s.)
        # Still params so they can be re-tuned (e.g. re-run spin_calib.py) without recompiling.
        self.declare_parameter('gyro_scale', 0.0152587890625)   # raw -> °/s  (1/65.536, ±500°/s — CONFIRMED)
        self.declare_parameter('accel_scale', 6.103515625e-05)  # raw -> g    (1/16384,  ±2g    — CONFIRMED)
        self.imu_frame = self.get_parameter('imu_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.still_window = int(self.get_parameter('still_window').value)
        self.still_thresh = self.get_parameter('still_thresh_dps').value
        self.gscale = self.get_parameter('gyro_scale').value
        self.ascale = self.get_parameter('accel_scale').value

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)
        self.pub_odom = self.create_publisher(Odometry, '/odom/wheel', 50)
        # --- cliff / fall protection (SAFETY) ---
        # MCU Triggers frame (type 0x00) payload byte[1] = downward IR cliff/ground bits: 0x00 = safely
        # on the floor, non-zero = one or more sensors see "no floor" (a drop-off, or the robot lifted).
        # Calibrated 2026-07-09 by diffing on-floor (00) vs lifted (0x02..0x0f). We publish /cliff (Bool,
        # latched so late subscribers get the current state) — the cliff_guard node hard-stops driving on it.
        # Conservative by design: byte != 0 -> cliff (a false stop is safe; a missed drop is catastrophic).
        self.declare_parameter('cliff_byte', 1)          # payload index of the cliff/ground bits
        self.cliff_idx = int(self.get_parameter('cliff_byte').value)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cliff = self.create_publisher(Bool, '/cliff', latched)
        self.pub_cliff_raw = self.create_publisher(UInt8, '/cliff/raw', latched)
        self.cliff_state = None

        # source: '' = local tmpfs ring (on-robot); 'host:port' = raw bytes over TCP from ring_forward.py on
        # the robot (lets this node run on the COMPANION with ROS off the robot). Decode/publish is identical.
        self.declare_parameter('source', '')
        _src = self.get_parameter('source').value
        self.src = None
        if _src:
            _h, _, _p = _src.partition(':')
            self.src = (_h, int(_p))
        self.sock = None
        self.mm = None
        self.read_pos = 0
        self.buf = bytearray()
        self.bias = [0.0, 0.0, 0.0]
        self.bias_set = False
        self.recent = []         # rolling recent gyro samples for the still-detector
        # dedicated reader thread (an rclpy timer gets starved and drops frames) — drains the ring
        # in a tight loop and publishes; rclpy.spin() just keeps the node alive. Same pattern as
        # valetudo_bridge.py. Publishing from this thread is fine for these message rates.
        threading.Thread(target=self.reader_loop, daemon=True).start()
        self.get_logger().info('mcu_node up; /imu/data publishes when the robot is ACTIVE '
                               '(D10s sends no IMU when docked/idle); gyro bias auto-set when still')

    def reader_loop(self):
        while not self.open_ring() and rclpy.ok():
            time.sleep(0.2)
        idle = 0
        while rclpy.ok():
            got = self.poll()                       # bytes drained this pass
            if got:
                idle = 0
                time.sleep(0.003)                   # ~330 Hz while data flows (keeps up with 100 Hz IMU)
            else:
                idle = min(idle + 1, 10)
                time.sleep(0.005 * idle)            # ring dry -> back off to ~50 ms (was a flat 500 Hz poll)

    def open_tcp(self):
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
            if struct.unpack_from('<Q', self.mm, 8)[0] != MAGIC:
                self.mm.close(); self.mm = None
                return False
            self.read_pos = struct.unpack_from('<Q', self.mm, 0)[0]
            return True
        except Exception as e:
            self.get_logger().warn(f'ring open failed: {e}')
            return False

    def drain(self):
        if self.src is not None:
            return self.drain_tcp()
        wp = struct.unpack_from('<Q', self.mm, 0)[0]
        avail = wp - self.read_pos
        if avail <= 0:
            return b''
        if avail > RING:
            self.read_pos = wp - RING
        start, end, base = self.read_pos % RING, wp % RING, HDR
        out = self.mm[base + start:base + end] if start < end else \
            self.mm[base + start:base + RING] + self.mm[base:base + end]
        self.read_pos = wp
        return bytes(out)

    def poll(self):
        if not self.open_ring():
            return 0
        chunk = self.drain()
        self.buf += chunk
        if len(self.buf) > 4 * RING:
            self.buf = self.buf[-RING:]
        i, n = 0, len(self.buf)
        while i + 6 <= n:                         # smallest frame = 6 bytes (len=0)
            if self.buf[i] != 0x3C:
                i += 1; continue
            ln = self.buf[i + 1]
            total = ln + 6                        # 3c + len + type + payload + crc(2) + 3e
            if i + total > n:
                break                             # incomplete frame; keep from i for next tick
            if self.buf[i + total - 1] != 0x3E:    # not a real frame start (data 0x3c); resync
                i += 1; continue
            body = self.buf[i + 1:i + 3 + ln]      # [len, type, payload]
            stored = (self.buf[i + total - 3] << 8) | self.buf[i + total - 2]   # crc, big-endian
            if crc16(body) == stored:
                self.dispatch(self.buf[i + 2], self.buf[i + 3:i + 3 + ln])
                i += total
            else:
                i += 1                            # corrupt frame (MCU does this) — skip a byte
        self.buf = self.buf[i:]                   # keep ONLY the unparsed remainder (no data drop)
        return len(chunk)

    def dispatch(self, mtype, payload):
        now = self.get_clock().now().to_msg()
        if mtype == 0x00 and len(payload) > self.cliff_idx:   # Triggers — cliff/bump/dock (SAFETY)
            raw = payload[self.cliff_idx]
            cliff = raw != 0
            self.pub_cliff_raw.publish(UInt8(data=raw))
            self.pub_cliff.publish(Bool(data=cliff))
            if cliff != self.cliff_state:                     # log only on edge
                self.cliff_state = cliff
                self.get_logger().warn(f'CLIFF {"DETECTED" if cliff else "clear"} (Triggers byte=0x{raw:02x})')
            return
        if mtype == 0x02 and len(payload) == 18:          # Status10ms — IMU
            ts, gx, gy, gz, ax, ay, az, _ld, _rd = struct.unpack('<Ihhhhhhbb', payload)
            gyro = [gx * self.gscale, gy * self.gscale, gz * self.gscale]   # °/s
            acc = [ax * self.ascale, ay * self.ascale, az * self.ascale]    # g
            # adaptive gyro bias: update only while the robot is detected STILL (all recent samples
            # below the threshold vs the current bias). Self-calibrates on the still moment when the
            # IMU wakes, and stays frozen during motion.
            self.recent.append(gyro)
            if len(self.recent) > self.still_window:
                self.recent.pop(0)
            if len(self.recent) >= self.still_window:
                if max(max(abs(g[k] - self.bias[k]) for k in range(3)) for g in self.recent) < self.still_thresh:
                    nb = [sum(g[k] for g in self.recent) / len(self.recent) for k in range(3)]
                    if not self.bias_set:
                        self.get_logger().info(f'gyro bias set = {[round(b,3) for b in nb]} °/s')
                        self.bias_set = True
                    self.bias = nb
            m = Imu()
            m.header.stamp = now
            m.header.frame_id = self.imu_frame
            m.orientation_covariance[0] = -1.0            # orientation unknown (REP-145)
            m.angular_velocity.x = (gyro[0] - self.bias[0]) * DEG2RAD
            m.angular_velocity.y = (gyro[1] - self.bias[1]) * DEG2RAD
            m.angular_velocity.z = (gyro[2] - self.bias[2]) * DEG2RAD
            m.linear_acceleration.x = acc[0] * G
            m.linear_acceleration.y = acc[1] * G
            m.linear_acceleration.z = acc[2] * G
            self.pub_imu.publish(m)
        elif mtype == 0x01 and len(payload) == 26:        # Status20ms — wheel odom
            ts, x, y, yaw, yawi, lv, rv, edge, roll, side = struct.unpack('<Iiihhhhhhh', payload)
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = self.odom_frame
            o.child_frame_id = self.base_frame
            o.pose.pose.position.x = x / 10000.0          # 0.1mm -> m
            o.pose.pose.position.y = y / 10000.0
            th = math.radians(yaw / 100.0)
            o.pose.pose.orientation.z = math.sin(th / 2.0)
            o.pose.pose.orientation.w = math.cos(th / 2.0)
            self.pub_odom.publish(o)


def main():
    rclpy.init()
    node = McuNode()
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

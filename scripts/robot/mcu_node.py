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
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

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
        # MCU throttles Status10ms to ~22 Hz when idle/docked (ramps toward 100 Hz when active), so
        # keep the bias sample count modest or startup calibration drags. 64 ≈ 3 s at idle rate.
        self.declare_parameter('calib_samples', 64)
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
        self.calib_n = int(self.get_parameter('calib_samples').value)
        self.gscale = self.get_parameter('gyro_scale').value
        self.ascale = self.get_parameter('accel_scale').value

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)
        self.pub_odom = self.create_publisher(Odometry, '/odom/wheel', 50)

        self.mm = None
        self.read_pos = 0
        self.buf = bytearray()
        self.bias = [0.0, 0.0, 0.0]
        self.calib = []          # collected gyro samples until len == calib_n
        self.calibrated = False
        # dedicated reader thread (an rclpy timer gets starved and drops frames) — drains the ring
        # in a tight loop and publishes; rclpy.spin() just keeps the node alive. Same pattern as
        # valetudo_bridge.py. Publishing from this thread is fine for these message rates.
        threading.Thread(target=self.reader_loop, daemon=True).start()
        self.get_logger().info('mcu_node up; calibrating gyro bias (keep robot still)...')

    def reader_loop(self):
        while not self.open_ring() and rclpy.ok():
            time.sleep(0.2)
        while rclpy.ok():
            self.poll()
            time.sleep(0.002)

    def open_ring(self):
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
            return
        self.buf += self.drain()
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

    def dispatch(self, mtype, payload):
        now = self.get_clock().now().to_msg()
        if mtype == 0x02 and len(payload) == 18:          # Status10ms — IMU
            ts, gx, gy, gz, ax, ay, az, _ld, _rd = struct.unpack('<Ihhhhhhbb', payload)
            gyro = [gx * self.gscale, gy * self.gscale, gz * self.gscale]   # °/s
            acc = [ax * self.ascale, ay * self.ascale, az * self.ascale]    # g
            if not self.calibrated:
                self.calib.append(gyro)
                if len(self.calib) >= self.calib_n:
                    self.bias = [sum(c[k] for c in self.calib) / len(self.calib) for k in range(3)]
                    self.calibrated = True
                    self.get_logger().info(f'gyro bias = {[round(b,3) for b in self.bias]} °/s; publishing /imu/data')
                return
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

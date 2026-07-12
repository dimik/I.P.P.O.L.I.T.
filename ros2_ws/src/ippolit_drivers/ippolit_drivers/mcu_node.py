#!/usr/bin/env python3
"""
mcu_node.py — publish EVERYTHING the MCU protocol provides to ROS.

Reads the raw /dev/ttyS4 byte stream that libserialtap.so tees into the tmpfs shm ring
(/tmp/mcu_ring.buf), decodes every known MCU frame type, and publishes:
    /imu/data          sensor_msgs/Imu   (Status10ms 0x02 — BMI055 gyro+accel)
    /odom/wheel        nav_msgs/Odometry (Status20ms 0x01 — wheel dead-reckoning)
    /cliff /bumper /cliff/front /cliff/rear /wheel_floating /mcu/error (Triggers 0x00)
    /mcu/triggers                      (Triggers 0x00 full decode -- ippolit_interfaces/
        McuTriggers, typed per A3; was a JSON String)
    /cliff/edge_dist /mcu/status20     (Status20ms extras: edgeDis, roller/side-brush current)
    /mcu/status10                      (Status10ms extra: leftDis/rightDis wheel-distance deltas)
    /mcu/status100 /dustbin_missing    (Status100ms 0x03 — pitch/roll, wheel current,
        dust/water/hepa/carpet)
    /mcu/battery                       (BatteryStatus 0x2B — existence on THIS hardware
        unconfirmed)
    /mcu/hwinfo /mcu/fw_version        (HwInfo 0x29, McuFwVersionInfo 0x07 — static, rare,
        latched)
    /mcu/ping /mcu/status500           (PingMsg 0x0F, Status500ms 0x05 — heartbeats)
    /mcu/shutdown_event /mcu/factory_test /mcu/log_raw   (0x10, 0x04, 0x27 — rare/diagnostic)
    /mcu/unknown                       (catch-all: any type/length we don't recognize —
        nothing silently dropped, even if we can't interpret it. ~12 type bytes are
        undecoded even in the reference RE repo; they land here as raw hex.)
See docs/sensors.md for the full packet table + field map (Triggers bit-level in particular).

Pipeline:  AVA read(ttyS4) --[libserialtap.so]--> /tmp/mcu_ring.buf --[this node]--> the
topics above

Frame:  3c | len(1) | type(1) | payload(len) | crc16(2, big-endian) | 3e
        CRC = Modbus-16 over [len,type,payload] (the MCU emits occasional corrupt frames — drop
        on CRC fail). Frame FORMATS from github.com/alufers/dreame_mcu_protocol (Z10), but
        SCALINGS differ — the Z10 pre-scaled, the D10s sends RAW sensor LSB (see params below):
          Status10ms `<Ihhhhhhbb`: ts, gyro_xyz, accel_xyz (raw LSB), leftDis,rightDis(mm)
          Status20ms `<Iiihhhhhhh`: ts, x,y(0.1mm), yaw(/100 °), yaw_int, L/R vel, edgeDis,
          2x current

IMU notes: orientation is left UNKNOWN (no magnetometer; covariance[0]=-1 per REP-145) — we
publish angular_velocity (rad/s) + linear_acceleration (m/s²) for downstream EKF fusion. Gyro
bias is auto-calibrated from the first ~3 s assuming the robot is stationary at startup (it
boots docked). Axis/sign alignment vs base_link is a v1 passthrough — verify in RViz and adjust
if needed.

Run:  source /opt/ros/jazzy/setup.bash && python3 mcu_node.py

Publishes /diagnostics (A5): OK as long as ANY MCU frame (of any type) has been decoded within
the last few seconds. Status20ms (wheel odom) flows continuously in every mode including idle/
docked (unlike the IMU stream, which is active-only -- see the IMU notes above), so "no frames at
all" for more than a couple of seconds means the tap/ring itself is dead, not just idle.
"""
import json
import math
import mmap
import os
import socket
import struct
import threading
import time

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import FunctionDiagnosticTask, Updater
from ippolit_interfaces.msg import McuTriggers
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String, UInt8

MCU_STALE_S = 3.0   # no frames at all within this long -> the tap/ring itself is dead

# Full bit-level Triggers (type 0x00, 7 bytes) decode, ported from
# github.com/dimik/dreame_mcu_protocol (Z10 Pro RE) and cross-checked against our own captures
# (2026-07-11) — byte0 bit4/5 = bumpers (matches our calibrated 0x10=bumper), byte2/3 =
# ir_dock/ir_field (dock-homing IR beacon channels; explains the rapid ambient flicker we saw
# there, unrelated to cliff). byte1 bits 0-5 = d_view_* "drop-view" sensors — THIS is the real
# forward/rear cliff-IR, not wheel-drop as concluded on 2026-07-09; our existing cliff_idx=1
# check was already reading it, just as one undifferentiated OR'd byte.
_TRIGGERS_BOOL_BITS = {
    'key1': 0, 'key2': 1, 'key3': 2, 'key4': 3,
    'left_bumper': 4, 'right_bumper': 5, 'left_wheel_floating': 6, 'right_wheel_floating': 7,
    'd_view_lf': 8, 'd_view_lmf': 9, 'd_view_rmf': 10, 'd_view_rf': 11,
    'd_view_lb': 12, 'd_view_rb': 13, 'mag_signal_left': 14, 'mag_signal_right': 15,
    'ir_field_lf': 19, 'ir_field_lmf': 23, 'ir_field_rmf': 27, 'ir_field_rf': 31,
    'dock_sta': 32, 'lds_button1': 33, 'lds_button2': 34,
    'side_error': 37, 'roll_error': 38, 'pump_error': 39,
    'side_overcurrent': 40, 'roll_overcurrent': 41, 'fan_overcurrent': 42, 'pump_overcurrent': 43,
    'left_wheel_overcurrent': 44, 'right_wheel_overcurrent': 45,
    'lidar_error': 48, 'fan_error': 49, 'left_vel_error': 50, 'right_vel_error': 51,
    'left_mag_error': 52, 'right_mag_error': 53, 'imu_error': 54, 'charge_error': 55,
}
_TRIGGERS_INT3_FIELDS = {   # 3-bit dock-IR-beacon signal strength/channel codes (MSB-first)
    'ir_dock_lf': 16, 'ir_dock_lmf': 20, 'ir_dock_rmf': 24, 'ir_dock_rf': 28,
}
DVIEW_FRONT = ('d_view_lf', 'd_view_lmf', 'd_view_rmf', 'd_view_rf')
DVIEW_REAR = ('d_view_lb', 'd_view_rb')
ERROR_BITS = (
    'side_error', 'roll_error', 'pump_error', 'side_overcurrent', 'roll_overcurrent',
    'fan_overcurrent', 'pump_overcurrent', 'left_wheel_overcurrent', 'right_wheel_overcurrent',
    'lidar_error', 'fan_error', 'left_vel_error', 'right_vel_error',
    'left_mag_error', 'right_mag_error', 'imu_error', 'charge_error',
)


def decode_triggers(payload):
    """
    7-byte Triggers payload -> dict of every named field.

    bit i = payload[i//8] bit (i%8), LSB-first.
    """
    def bit(i):
        return (payload[i // 8] >> (i % 8)) & 1
    d = {name: bit(i) for name, i in _TRIGGERS_BOOL_BITS.items()}
    for name, lo in _TRIGGERS_INT3_FIELDS.items():
        v = 0
        for i in range(lo, lo + 3):
            v = (v << 1) | bit(i)      # MSB-first within the field
        d[name] = v
    return d


RING_PATH = '/tmp/mcu_ring.buf'
HDR = 64
RING = 256 * 1024
MAGIC = 0x0031534444530001
DEG2RAD = math.pi / 180.0
G = 9.80665


def crc16(data):
    """Modbus-16 (reflected, poly 0xA001) — matches dreame_mcu_protocol CRC_GetModbus16."""
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
        # ⚠️ The D10s emits the IMU stream (Status10ms / 0x02) ONLY WHEN ACTIVE (moving /
        # cleaning) — NOT when docked or idle (only wheel odom 0x01 flows then). So /imu/data
        # only publishes during activity, and a "hold still at startup" bias calibration can't
        # work (no data when docked, and data only arrives once moving). Instead: ADAPTIVE bias
        # — update the gyro bias only while the robot is detected still (|gyro-bias| below a
        # threshold across a window), and publish always. On the brief still moment when the IMU
        # wakes (before the robot drives), bias self-calibrates.
        self.declare_parameter('still_window', 40)       # samples for the still detector
        self.declare_parameter('still_thresh_dps', 1.5)  # |gyro| below this (°/s) = "still"
        # The D10s does NOT match the Z10 scalings — the Z10 firmware pre-scaled (mg,
        # centideg/s); the D10s passes RAW sensor LSB. Both CONFIRMED empirically on the D10s
        # (imu_type=2):
        #   accel: |accel| at rest = 16384 raw => 1g = 16384 LSB (±2g 16-bit) => accel_scale =
        #          1/16384
        #   gyro:  spin cross-check vs wheel-odom yaw (2 runs, both directions: 0.01526/0.01527
        #          °/s per LSB) => ±500°/s => gyro_scale = 1/65.536. (NOT the Z10's /100, NOT
        #          ±2000°/s.)
        # Kept as params so they can be re-tuned (e.g. re-run spin_calib.py) without recompiling.
        self.declare_parameter('gyro_scale', 0.0152587890625)   # raw->°/s (1/65.536, ±500°/s)
        self.declare_parameter('accel_scale', 6.103515625e-05)  # raw->g   (1/16384,  ±2g)
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
        # MCU Triggers frame (type 0x00) payload byte[1] = downward IR cliff/ground bits: 0x00 =
        # safely on the floor, non-zero = one or more sensors see "no floor" (a drop-off, or the
        # robot lifted). Calibrated 2026-07-09 by diffing on-floor (00) vs lifted (0x02..0x0f).
        # We publish /cliff (Bool, latched so late subscribers get the current state) — the
        # cliff_guard node hard-stops driving on it. Conservative by design: byte != 0 -> cliff
        # (a false stop is safe; a missed drop is catastrophic).
        self.declare_parameter('cliff_byte', 1)          # payload index of the cliff/ground bits
        self.cliff_idx = int(self.get_parameter('cliff_byte').value)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cliff = self.create_publisher(Bool, '/cliff', latched)
        self.pub_cliff_raw = self.create_publisher(UInt8, '/cliff/raw', latched)
        self.cliff_state = None
        # MCU Triggers byte[0] = bump/contact bits (front bumper microswitches + wheel-drop).
        # Calibrated 2026-07-09: 0x00 on clear floor (armed) vs non-zero (0xc0/0xd0/0x30..) on
        # contact. Conservative rule byte != 0 -> bump, so the drive controller can back off +
        # turn on collision.
        self.declare_parameter('bump_byte', 0)
        self.bump_idx = int(self.get_parameter('bump_byte').value)
        self.pub_bump = self.create_publisher(Bool, '/bumper', latched)
        self.pub_bump_raw = self.create_publisher(UInt8, '/bumper/raw', latched)
        self.bump_state = None
        # NEW 2026-07-11 (additive, does not change /cliff or /bumper semantics above): full
        # bit-level decode of the same Triggers byte[1] we already OR into /cliff, broken out
        # per physical sensor position (front d_view_lf/lmf/rmf/rf vs rear d_view_lb/rb), plus
        # wheel_floating separated out from the bumper byte. See docs/sensors.md for the field
        # map.
        self.pub_cliff_front = self.create_publisher(Bool, '/cliff/front', latched)
        self.pub_cliff_rear = self.create_publisher(Bool, '/cliff/rear', latched)
        self.pub_wheel_floating = self.create_publisher(Bool, '/wheel_floating', latched)
        self.pub_triggers_raw = self.create_publisher(McuTriggers, '/mcu/triggers', latched)
        self.front_state = self.rear_state = self.wf_state = None
        # Status20ms `edgeDis` (mm) — a CONTINUOUS distance-ish reading, decoded since day one
        # but never published (dead variable). Streamed every 20ms regardless of state change,
        # unlike the event-driven Triggers bits above — meaning matters TBD (untested at a real
        # edge yet).
        self.pub_edge_dist = self.create_publisher(Float32, '/cliff/edge_dist', 10)
        # NEW 2026-07-11: Status100ms (0x03, 10Hz) was never decoded at all despite being on the
        # wire (confirmed live). dust_container_missing is directly relevant to the
        # dustbin-interlock work earlier this project — read it straight from the MCU instead of
        # inferring via Valetudo.
        self.pub_status100 = self.create_publisher(String, '/mcu/status100', 10)
        self.pub_dustbin_missing = self.create_publisher(Bool, '/dustbin_missing', latched)
        self.dustbin_state = None
        # BatteryStatus (0x2B) was never decoded either — native voltage/current/temp/SoC,
        # potentially a much faster/richer source than the current 15s avacmd charge_state poll.
        # Existence/rate on THIS hardware is UNCONFIRMED (need a live capture) — publish if/when
        # one actually arrives.
        self.pub_battery_raw = self.create_publisher(String, '/mcu/battery', 10)
        # NEW 2026-07-11 (2nd pass, "expose everything"): the rest of the known TYPES_FROM_MCU
        # map, plus the previously-dead Status20/Status10 extra fields, plus a full (not
        # nonzero-only) Triggers dict + an error/overcurrent aggregate, plus a catch-all for
        # anything we don't recognize.
        self.pub_mcu_error = self.create_publisher(Bool, '/mcu/error', latched)
        self.error_state = None
        self.pub_status20_extra = self.create_publisher(
            String, '/mcu/status20', 10)   # roller/side current
        self.pub_status10_extra = self.create_publisher(
            String, '/mcu/status10', 10)   # left/rightDis (mm)
        self.pub_hwinfo = self.create_publisher(
            String, '/mcu/hwinfo', latched)        # mcu/imu/charge ids
        self.pub_fw_version = self.create_publisher(
            String, '/mcu/fw_version', latched)  # git hash + ver
        self.pub_ping = self.create_publisher(
            String, '/mcu/ping', 10)                 # MCU heartbeat probe
        self.pub_shutdown_event = self.create_publisher(String, '/mcu/shutdown_event', latched)
        self.pub_log_raw = self.create_publisher(
            String, '/mcu/log_raw', 10)           # format undocumented
        self.pub_status500 = self.create_publisher(
            String, '/mcu/status500', 10)       # RTC heartbeat
        self.pub_factory_test = self.create_publisher(UInt8, '/mcu/factory_test', latched)
        # 0x24 — undocumented even upstream ("something connected with the battery temperature",
        # 1 byte). Live-captured 2026-07-11 while charging: constant 0x00 for 30 frames over 15s
        # (no other candidate battery-analog packet type appeared — BatteryStatus 0x2B is
        # CONFIRMED ABSENT on this hardware). Best-effort: treat nonzero as a warning flag. Only
        # the "0 = OK" value has actually been observed.
        self.pub_battery_temp_flag = self.create_publisher(Bool, '/mcu/battery_temp_flag',
                                                           latched)
        self.battery_temp_flag_state = None
        self.pub_unknown = self.create_publisher(
            String, '/mcu/unknown', 10)           # nothing dropped silently
        self._unknown_seen = set()   # log each distinct (type, len) once, not every frame

        # source: '' = local tmpfs ring (on-robot); 'host:port' = raw bytes over TCP from
        # ring_forward.py on the robot (lets this node run on the COMPANION with ROS off the
        # robot). Decode/publish is identical.
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
        self.last_frame_t = 0.0
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('mcu_node')
        self.diag_updater.add(FunctionDiagnosticTask('MCU frame health', self._diag_mcu))
        # dedicated reader thread (an rclpy timer gets starved and drops frames) — drains the
        # ring in a tight loop and publishes; rclpy.spin() just keeps the node alive. Same
        # pattern as valetudo_bridge.py. Publishing from this thread is fine for these message
        # rates.
        threading.Thread(target=self.reader_loop, daemon=True).start()
        self.get_logger().info(
            'mcu_node up; /imu/data publishes when the robot is ACTIVE '
            '(D10s sends no IMU when docked/idle); gyro bias auto-set when still')

    def _diag_mcu(self, stat):
        age = time.monotonic() - self.last_frame_t if self.last_frame_t else None
        if age is not None and age < MCU_STALE_S:
            stat.summary(DiagnosticStatus.OK, 'MCU frames flowing')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'no MCU frames -- tap/ring likely dead')
        stat.add('last_frame_age_s', f'{age:.1f}' if age is not None else 'never')
        return stat

    def reader_loop(self):
        while not self.open_ring() and rclpy.ok():
            time.sleep(0.2)
        idle = 0
        while rclpy.ok():
            got = self.poll()                       # bytes drained this pass
            if got:
                idle = 0
                # ~330 Hz while data flows (keeps up with 100 Hz IMU)
                time.sleep(0.003)
            else:
                idle = min(idle + 1, 10)
                # ring dry -> back off to ~50 ms (was a flat 500 Hz poll)
                time.sleep(0.005 * idle)

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
            try:
                self.sock.close()
            except Exception:
                pass
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
                self.mm.close()
                self.mm = None
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
                i += 1
                continue
            ln = self.buf[i + 1]
            total = ln + 6                        # 3c + len + type + payload + crc(2) + 3e
            if i + total > n:
                break                             # incomplete frame; keep from i for next tick
            if self.buf[i + total - 1] != 0x3E:    # not a real frame start (data 0x3c); resync
                i += 1
                continue
            body = self.buf[i + 1:i + 3 + ln]      # [len, type, payload]
            stored = (self.buf[i + total - 3] << 8) | self.buf[i + total - 2]   # crc, big-endian
            if crc16(body) == stored:
                self.dispatch(self.buf[i + 2], self.buf[i + 3:i + 3 + ln])
                i += total
            else:
                i += 1                            # corrupt frame (MCU does this) — skip a byte
        self.buf = self.buf[i:]                   # keep ONLY the unparsed remainder (no drop)
        return len(chunk)

    def dispatch(self, mtype, payload):
        now = self.get_clock().now().to_msg()
        self.last_frame_t = time.monotonic()
        if mtype == 0x00 and len(payload) > max(self.cliff_idx, self.bump_idx):
            # Triggers — cliff/bump/dock
            craw = payload[self.cliff_idx]
            cliff = craw != 0
            self.pub_cliff_raw.publish(UInt8(data=craw))
            self.pub_cliff.publish(Bool(data=cliff))
            if cliff != self.cliff_state:                     # log only on edge
                self.cliff_state = cliff
                state = 'DETECTED' if cliff else 'clear'
                self.get_logger().warn(f'CLIFF {state} (Triggers byte=0x{craw:02x})')
            braw = payload[self.bump_idx]
            bump = braw != 0
            self.pub_bump_raw.publish(UInt8(data=braw))
            self.pub_bump.publish(Bool(data=bump))
            if bump != self.bump_state:
                self.bump_state = bump
                state = 'HIT' if bump else 'clear'
                self.get_logger().warn(f'BUMP {state} (Triggers byte=0x{braw:02x})')
            # full named-bit decode (additive, see module header)
            if len(payload) == 7:
                d = decode_triggers(payload)
                front = any(d[k] for k in DVIEW_FRONT)
                rear = any(d[k] for k in DVIEW_REAR)
                wf = bool(d['left_wheel_floating'] or d['right_wheel_floating'])
                err = any(d[k] for k in ERROR_BITS)
                self.pub_cliff_front.publish(Bool(data=front))
                self.pub_cliff_rear.publish(Bool(data=rear))
                self.pub_wheel_floating.publish(Bool(data=wf))
                self.pub_mcu_error.publish(Bool(data=err))
                trig_msg = McuTriggers()
                trig_msg.header.stamp = now
                for k in _TRIGGERS_BOOL_BITS:      # FULL state, not just nonzero
                    setattr(trig_msg, k, bool(d[k]))
                for k in _TRIGGERS_INT3_FIELDS:
                    setattr(trig_msg, k, d[k])
                self.pub_triggers_raw.publish(trig_msg)
                if err != self.error_state:
                    self.error_state = err
                    active = [k for k in ERROR_BITS if d[k]]
                    state = 'SET' if err else 'clear'
                    self.get_logger().warn(f'MCU/ERROR {state} ({", ".join(active)})')
                if front != self.front_state:
                    self.front_state = front
                    state = 'DETECTED' if front else 'clear'
                    self.get_logger().warn(
                        f'CLIFF/FRONT {state} '
                        f'(lf={d["d_view_lf"]} lmf={d["d_view_lmf"]} '
                        f'rmf={d["d_view_rmf"]} rf={d["d_view_rf"]})')
                if rear != self.rear_state:
                    self.rear_state = rear
                    state = 'DETECTED' if rear else 'clear'
                    self.get_logger().warn(
                        f'CLIFF/REAR {state} '
                        f'(lb={d["d_view_lb"]} rb={d["d_view_rb"]})')
                if wf != self.wf_state:
                    self.wf_state = wf
                    state = 'yes' if wf else 'clear'
                    self.get_logger().warn(
                        f'WHEEL-FLOATING {state} '
                        f'(l={d["left_wheel_floating"]} r={d["right_wheel_floating"]})')
            return
        if mtype == 0x02 and len(payload) == 18:          # Status10ms — IMU
            ts, gx, gy, gz, ax, ay, az, ld, rd = struct.unpack('<Ihhhhhhbb', payload)
            self.pub_status10_extra.publish(String(data=json.dumps(
                {'left_dist_mm': ld, 'right_dist_mm': rd})))
            gyro = [gx * self.gscale, gy * self.gscale, gz * self.gscale]   # °/s
            acc = [ax * self.ascale, ay * self.ascale, az * self.ascale]    # g
            # adaptive gyro bias: update only while the robot is detected STILL (all recent
            # samples below the threshold vs the current bias). Self-calibrates on the still
            # moment when the IMU wakes, and stays frozen during motion.
            self.recent.append(gyro)
            if len(self.recent) > self.still_window:
                self.recent.pop(0)
            if len(self.recent) >= self.still_window:
                spread = max(
                    max(abs(g[k] - self.bias[k]) for k in range(3)) for g in self.recent)
                if spread < self.still_thresh:
                    nb = [sum(g[k] for g in self.recent) / len(self.recent) for k in range(3)]
                    if not self.bias_set:
                        self.get_logger().info(f'gyro bias set = {[round(b, 3) for b in nb]} °/s')
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
            self.pub_edge_dist.publish(Float32(data=edge / 1000.0))  # edgeDis mm->m; meaning TBD
            self.pub_status20_extra.publish(String(data=json.dumps(
                {'roller_current_ma': roll, 'side_current_ma': side})))
        elif mtype == 0x03 and len(payload) == 9:
            # Status100ms — tilt + dust/water/hepa/carpet
            pitch, roll, lcur, rcur, flags = struct.unpack('<hhhhB', payload)
            dustbin_missing = bool(flags & 1)
            self.pub_status100.publish(String(data=json.dumps({
                'pitch_deg': pitch / 10.0, 'roll_deg': roll / 10.0,
                'left_current_ma': lcur, 'right_current_ma': rcur,
                'dust_container_missing': dustbin_missing,
                'water_tank_installed': bool((flags >> 1) & 1),
                'hepa_state': bool((flags >> 2) & 1),
                'carpet_state': bool((flags >> 3) & 1),
            })))
            self.pub_dustbin_missing.publish(Bool(data=dustbin_missing))
            if dustbin_missing != self.dustbin_state:
                self.dustbin_state = dustbin_missing
                state = 'MISSING' if dustbin_missing else 'present'
                self.get_logger().warn(f'DUSTBIN {state} (Status100 flags={flags:#04x})')
        elif mtype == 0x2B and len(payload) == 12:
            # BatteryStatus — native voltage/current/temp/SoC
            bv, bc, bt, cv, soc, unk = struct.unpack('<HHhHhH', payload)
            self.pub_battery_raw.publish(String(data=json.dumps({
                'battery_voltage_v': bv / 1000.0,
                'battery_current_ma': bc,   # unsigned, no direction bit
                'battery_temperature_c': bt / 10.0, 'charge_voltage_v': cv / 1000.0,
                'state_of_charge_pct': soc / 100.0, 'unknown': unk,
            })))
        elif mtype == 0x29 and len(payload) == 5:          # HwInfo — static hardware IDs, rare
            mcu_t, imu_t, imu2_t, charge_t, app_t = struct.unpack('<BBBBB', payload)
            self.pub_hwinfo.publish(String(data=json.dumps({
                'mcu_type': mcu_t, 'imu_type': imu_t, 'imu2_type': imu2_t,
                'charge_type': charge_t, 'app_type': app_t})))
            self.get_logger().info(
                f'HwInfo: mcu_type={mcu_t} imu_type={imu_t} imu2_type={imu2_t} '
                f'charge_type={charge_t} app_type={app_t}')
        elif mtype == 0x07 and len(payload) == 16:
            # McuFwVersionInfo — git hash + version string
            git_hash, version = struct.unpack('<10s6s', payload)
            self.pub_fw_version.publish(String(data=json.dumps({
                'git_hash': git_hash.decode('ascii', 'replace').rstrip('\x00'),
                'version': version.decode('ascii', 'replace').rstrip('\x00')})))
        elif mtype == 0x0F and len(payload) == 8:
            # PingMsg — MCU heartbeat/latency probe (we never reply Pong; read-only tap)
            ts, delta = struct.unpack('<II', payload)
            self.pub_ping.publish(String(data=json.dumps({'timestamp': ts, 'delta': delta})))
        elif mtype == 0x10 and len(payload) == 1:
            # ShutdownMsg — sent amid a poweroff sequence; occurrence itself is the signal
            self.pub_shutdown_event.publish(String(data=json.dumps(
                {'stamp_ns': self.get_clock().now().nanoseconds})))
            self.get_logger().warn('MCU ShutdownMsg seen (robot may be powering off)')
        elif mtype == 0x27 and len(payload) == 12:
            # McuLog — raw log/error bytes, format undocumented
            self.pub_log_raw.publish(String(data=payload.hex()))
        elif mtype == 0x05 and len(payload) == 6:          # Status500ms — RTC heartbeat
            unk1, seq, rtc_ts = struct.unpack('<BBI', payload)
            self.pub_status500.publish(String(data=json.dumps(
                {'unk1': unk1, 'sequence': seq, 'rtc_timestamp': rtc_ts})))
        elif mtype == 0x04 and len(payload) == 1:
            # FactoryTest — only meaningful in factory mode
            self.pub_factory_test.publish(UInt8(data=payload[0]))
        elif mtype == 0x24 and len(payload) == 1:
            # battery-temp-related (best-effort, see above)
            flag = bool(payload[0])
            self.pub_battery_temp_flag.publish(Bool(data=flag))
            if flag != self.battery_temp_flag_state:
                self.battery_temp_flag_state = flag
                state = 'SET' if flag else 'clear'
                self.get_logger().warn(
                    f'battery_temp_flag {state} '
                    f'(raw=0x{payload[0]:02x}, meaning of nonzero UNCONFIRMED)')
        else:
            # unrecognized type, or a known type at an unexpected length — surface it, don't
            # drop it.
            key = (mtype, len(payload))          # ~12 type bytes are undecoded even in the
            if key not in self._unknown_seen:    # reference RE repo; this is where they land.
                self._unknown_seen.add(key)
                self.get_logger().info(
                    f'MCU: unhandled type=0x{mtype:02x} len={len(payload)} '
                    f'payload={payload.hex()} (logged once per type/len)')
            self.pub_unknown.publish(String(data=json.dumps(
                {'type': mtype, 'len': len(payload), 'payload_hex': payload.hex()})))


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

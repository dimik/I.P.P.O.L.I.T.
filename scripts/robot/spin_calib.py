#!/usr/bin/env python3
"""spin_calib.py — derive the true gyro_z scale by cross-checking against wheel odometry.

Reads /tmp/mcu_ring.buf live for <secs>, decoding Status10ms (gyro_z, MCU timestamp) and Status20ms
(yaw). The robot must be STILL for the first ~1.5 s (for bias), then ROTATED. The MCU's own
microsecond timestamps drive the integration (no wall-clock jitter).

  scale[°/s per LSB] = (odom yaw delta, °) / ∫ (gyro_z_raw - bias) dt

This is the definitive gyro scale on THIS robot — independent of chip/datasheet/FS-range guesses.

Run (during a rotation):  python3 spin_calib.py <secs>
"""
import math, mmap, os, struct, sys, time

HDR, RING = 64, 256 * 1024
BIAS_SECS = 1.5

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
fd = os.open('/tmp/mcu_ring.buf', os.O_RDONLY)
mm = mmap.mmap(fd, HDR + RING, mmap.MAP_SHARED, mmap.PROT_READ)
read_pos = struct.unpack_from('<Q', mm, 0)[0]
buf = bytearray()
gyro = []   # (wall_t, ts_us, gz_raw)
yaws = []   # (wall_t, yaw_deg)
t0 = time.time()
while time.time() - t0 < secs:
    wp = struct.unpack_from('<Q', mm, 0)[0]
    if wp > read_pos:
        if wp - read_pos > RING:
            read_pos = wp - RING
        s, e = read_pos % RING, wp % RING
        buf += (mm[HDR + s:HDR + RING] + mm[HDR:HDR + e]) if s >= e else mm[HDR + s:HDR + e]
        read_pos = wp
        i, n = 0, len(buf)
        while i + 6 <= n:
            if buf[i] != 0x3C:
                i += 1; continue
            ln = buf[i + 1]; total = ln + 6
            if i + total > n:
                break
            if buf[i + total - 1] != 0x3E:
                i += 1; continue
            if crc16(buf[i + 1:i + 3 + ln]) == ((buf[i + total - 3] << 8) | buf[i + total - 2]):
                t, pl = buf[i + 2], buf[i + 3:i + 3 + ln]
                now = time.time()
                if t == 0x02 and len(pl) == 18:
                    v = struct.unpack('<Ihhhhhhbb', pl); gyro.append((now, v[0], v[3]))
                elif t == 0x01 and len(pl) == 26:
                    v = struct.unpack('<Iiihhhhhhh', pl); yaws.append((now, v[3] / 100.0))
                i += total
            else:
                i += 1
        buf = buf[i:]
    time.sleep(0.02)

if len(gyro) < 10 or len(yaws) < 3:
    print(f"insufficient data (gyro={len(gyro)} yaw={len(yaws)})"); sys.exit(1)

bias_samples = [g for g in gyro if g[0] - t0 < BIAS_SECS]
bias = sum(g[2] for g in bias_samples) / len(bias_samples) if bias_samples else 0.0

integral = 0.0; prev = None
for _, ts, gz in gyro:
    if prev is not None:
        dt = ((ts - prev) & 0xFFFFFFFF) / 1e6
        if 0 < dt < 0.5:
            integral += (gz - bias) * dt
    prev = ts

# unwrap odom yaw (deg)
yv = [y for _, y in yaws]
unw = [yv[0]]
for y in yv[1:]:
    d = y - (unw[-1] % 360 if abs(unw[-1]) > 180 else unw[-1])
    while d > 180: d -= 360
    while d < -180: d += 360
    unw.append(unw[-1] + d)
yaw_delta = unw[-1] - unw[0]

print(f"samples: gyro={len(gyro)} (bias from {len(bias_samples)}), yaw={len(yaws)}")
print(f"bias_z = {bias:.1f} raw")
print(f"odom yaw delta = {yaw_delta:+.1f}°   |   ∫(gz-bias)dt = {integral:+.1f} raw·s")
if abs(integral) > 50:
    scale = yaw_delta / integral
    print(f"=> gyro_scale = {scale:.5f} °/s per LSB")
    print(f"   (current default 0.061035 = ±2000°/s; Z10 /100 = 0.01)")
    print(f"   implied gyro full-scale ~ ±{abs(32768*scale):.0f} °/s")
else:
    print("rotation too small (|integral|<50) — spin the robot more during capture")

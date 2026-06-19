#!/usr/bin/env python3
"""confirm_mcu.py — sanity-check MCU decode + IMU scalings on THIS robot (vs the Z10 reference).

Reads /tmp/mcu_ring.buf (filled by libserialtap), CRC-validates frames, and reports:
  - packet type histogram
  - HwInfo (0x29): mcu_type, imu_type, imu2_type  -> identifies the actual IMU chip
  - Status10ms at rest: |accel| with /1000 scaling (should be ~1.0 g), gyro mean (should be ~0)
Use to CONFIRM the Z10-derived scalings before trusting them. Optional: pass 'spin' to also report
gyro_z integral vs Status20ms yaw delta over the capture (cross-checks the gyro scale on a rotation).
"""
import math, mmap, os, struct, sys, time
from collections import Counter

HDR, RING = 64, 256 * 1024

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

fd = os.open('/tmp/mcu_ring.buf', os.O_RDONLY)
mm = mmap.mmap(fd, HDR + RING, mmap.MAP_SHARED, mmap.PROT_READ)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
if secs > 0:
    start = struct.unpack_from('<Q', mm, 0)[0]
    time.sleep(secs)
    end = struct.unpack_from('<Q', mm, 0)[0]
else:                                   # secs=0: scan the WHOLE retained ring (catch boot HwInfo)
    end = struct.unpack_from('<Q', mm, 0)[0]
    start = max(0, end - RING)
n = end - start
if n > RING:
    start = end - RING; n = RING
s, e = start % RING, end % RING
raw = (mm[HDR + s:HDR + RING] + mm[HDR:HDR + e]) if s >= e else mm[HDR + s:HDR + e]
raw = bytes(raw)
print(f"scanned {n} bytes" + (f" over {secs}s ({n/secs:.0f} B/s)" if secs > 0 else " (whole retained ring)"))

types = Counter(); good = bad = 0
accs = []; gyros = []; yaws = []; hwinfo = None
i = 0
while i < len(raw) - 1:
    if raw[i] != 0x3C:
        i += 1; continue
    if i + 2 >= len(raw): break
    ln = raw[i + 1]; total = ln + 6
    if i + total > len(raw): break
    fr = raw[i:i + total]
    if fr[-1] != 0x3E:
        i += 1; continue
    body = fr[1:3 + ln]
    stored = (fr[-3] << 8) | fr[-2]
    if crc16(body) != stored:
        bad += 1; i += 1; continue
    good += 1; t = fr[2]; pl = fr[3:3 + ln]; types[t] += 1
    if t == 0x02 and len(pl) == 18:
        _, gx, gy, gz, ax, ay, az, _, _ = struct.unpack('<Ihhhhhhbb', pl)
        gyros.append((gx, gy, gz)); accs.append((ax, ay, az))
    elif t == 0x01 and len(pl) == 26:
        v = struct.unpack('<Iiihhhhhhh', pl); yaws.append(v[3] / 100.0)
    elif t == 0x29 and len(pl) == 5:
        hwinfo = struct.unpack('<BBBBB', pl)
    i += total

print(f"frames: {good} ok, {bad} crc-fail; types: {{ {', '.join(f'0x{k:02x}:{v}' for k,v in sorted(types.items()))} }}")
if hwinfo:
    print(f"HwInfo: mcu_type={hwinfo[0]} imu_type={hwinfo[1]} imu2_type={hwinfo[2]} charge={hwinfo[3]} app={hwinfo[4]}")
else:
    print("HwInfo (0x29): not seen in window (sent rarely; rerun or longer capture to ID the IMU chip)")

if accs:
    mags = [math.sqrt(ax*ax + ay*ay + az*az) for ax, ay, az in accs]
    am = sum(mags) / len(mags)
    gm = [sum(g[k] for g in gyros) / len(gyros) for k in range(3)]
    print(f"\nStatus10ms n={len(accs)} (at rest):")
    print(f"  raw |accel| mean = {am:.0f}  -> /1000 = {am/1000:.3f} g   (expect ~1.000 if /1000 is right)")
    print(f"  implied 1g LSB ~ {am:.0f}; /1000 {'CONFIRMS' if abs(am/1000-1.0)<0.05 else 'does NOT match -> rescale'} accel")
    print(f"  raw gyro mean = {[round(x,1) for x in gm]}  (should be ~0 at rest; this is the bias)")
    print(f"  /100 gyro mean = {[round(x/100,3) for x in gm]} °/s")

if len(sys.argv) > 1 and sys.argv[1] == 'spin' and gyros and len(yaws) > 2:
    # gyro_z integrated over the window vs wheel-odom yaw change — cross-checks gyro SCALE on a spin
    dt = secs / len(gyros)
    bias_z = sum(g[2] for g in gyros) / len(gyros)
    integ_100 = sum((g[2]) * dt for g in gyros) / 100.0   # if scale=/100, °
    yaw_delta = yaws[-1] - yaws[0]
    print(f"\nSPIN cross-check (rotate the robot during capture):")
    print(f"  wheel-odom yaw delta = {yaw_delta:.1f}°")
    print(f"  gyro_z integral @/100 = {integ_100:.1f}°  -> scale factor to match odom = {yaw_delta/integ_100 if integ_100 else 0:.3f} (×current /100)")

#!/usr/bin/env python3
"""
lds_decode.py — Dreame D10s Pro LDS (LiDAR) frame decoder + capture analyzer.

Reverse-engineered 2026-06-19 from a live capture of AVA reading /dev/ttyS3 @ 230400 baud
(see docs/sensors.md "Decoded LDS frame format"). This is the reference decoder the production
`/scan` publisher will be built on (the on-robot tap will feed raw bytes to `iter_points`).

40-byte fixed frame, little-endian, back-to-back:
    off  0  2   sync          55 aa
    off  2  2   type          03 08 (constant)
    off  4  2   speed         LE16, ~0x5E0A steady
    off  6  2   start angle   LE16, unit = 360deg/65536
    off  8  24  8 samples     each [dist:LE16 mm][quality:1]; dist==0x8000 => no return
    off 32  2   end angle     LE16, endAngle[n] ~= startAngle[n+1]
    off 34  6   timestamp     ~40-bit monotonic counter (no checksum, no end marker)

Sync is the 4-byte "55 aa 03 08" (sync + constant type). NOTE: an earlier capture made byte 39
look like a constant 0xa4 marker -- it is NOT; that was just the timestamp's high byte at the
time. Do not align on byte 39.

Usage:
    # decode a raw byte stream (file of concatenated frames):
    python3 lds_decode.py raw  capture.bin
    # parse + decode an `strace -f --seccomp-bpf -e trace=read -e read=26 ...` log:
    python3 lds_decode.py trace lds.txt
"""
import math
import re
import sys

FRAME_LEN = 40
DEG_PER_UNIT = 360.0 / 65536.0
INVALID = 0x8000


def le16(b, o):
    return b[o] | (b[o + 1] << 8)


def find_frames(raw):
    """Yield aligned 40-byte frames from a raw byte stream (resync on 55 aa 03 08)."""
    i, n = 0, len(raw)
    while i < n - (FRAME_LEN - 1):
        if raw[i] == 0x55 and raw[i + 1] == 0xAA and raw[i + 2] == 0x03 and raw[i + 3] == 0x08:
            yield bytes(raw[i:i + FRAME_LEN])
            i += FRAME_LEN
        else:
            i += 1


def decode_frame(fr):
    """-> dict(speed, start_angle, end_angle, samples=[(angle_deg, dist_mm, quality)])."""
    s_ang, e_ang = le16(fr, 6), le16(fr, 32)
    span = (e_ang - s_ang) & 0xFFFF
    samples = []
    for k in range(8):
        d = le16(fr, 8 + k * 3)
        q = fr[8 + k * 3 + 2]
        ang = (s_ang + span * k // 8) & 0xFFFF
        samples.append((ang * DEG_PER_UNIT, None if d == INVALID or d & INVALID else d, q))
    return {'speed': le16(fr, 4), 'start_angle': s_ang, 'end_angle': e_ang, 'samples': samples}


def iter_points(raw):
    """Yield (angle_deg, dist_mm, quality) for every valid point in a raw byte stream."""
    for fr in find_frames(raw):
        for ang, d, q in decode_frame(fr)['samples']:
            if d is not None:
                yield ang, d, q


def bytes_from_trace(path):
    """Reconstruct the fd-26 byte stream from an strace `-e read=26` hexdump log."""
    raw = bytearray()
    hexline = re.compile(r'^ \| [0-9a-f]{5}  (.{1,49})')
    for line in open(path):
        m = hexline.match(line)
        if m:
            for b in re.findall(r'[0-9a-f]{2}', m.group(1)[:48]):
                raw.append(int(b, 16))
    return raw


def summarize(raw):
    frames = list(find_frames(raw))
    pts = list(iter_points(raw))
    print(f'{len(raw)} bytes, {len(frames)} frames, {len(pts)} valid points')
    if not pts:
        return
    dists = [d for _, d, _ in pts]
    print(f'distance range: {min(dists)}..{max(dists)} mm')
    # top-down ASCII (robot at center)
    W = H = 41
    cx = cy = W // 2
    mx = max(dists)
    grid = [[' '] * W for _ in range(H)]
    for ang, d, _ in pts:
        r = d / mx * (cy - 1)
        x = int(round(cx + r * math.cos(math.radians(ang))))
        y = int(round(cy - r * math.sin(math.radians(ang))))
        if 0 <= x < W and 0 <= y < H:
            grid[y][x] = '#'
    grid[cy][cx] = '+'
    print(f'\ntop-down (#=obstacle, +=robot, extent={mx}mm):')
    print('\n'.join(''.join(r) for r in grid))


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ('raw', 'trace'):
        print(__doc__)
        sys.exit(1)
    mode, path = sys.argv[1], sys.argv[2]
    raw = bytearray(open(path, 'rb').read()) if mode == 'raw' else bytes_from_trace(path)
    summarize(raw)


if __name__ == '__main__':
    main()

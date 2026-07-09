#!/usr/bin/env python3
"""ring_forward.py — ROS-free tmpfs-ring -> TCP byte forwarder (runs on the ROBOT, in the chroot's python).

The LD_PRELOAD serial taps (libserialtap.so on ttyS3, mcutap on ttyS4) tee AVA's raw serial into tmpfs shm
rings (/tmp/lds_ring.buf, /tmp/mcu_ring.buf). To move ROS entirely onto the COMPANION, this streams a ring's
raw bytes over TCP so the (unchanged) ROS decode node can run on the Q6A instead of in the robot chroot.

Generic + stateless-per-client: each client gets bytes from the moment it connects (skips backlog). Handles
the ring's circular wrap + reader-fell-behind exactly like the old on-robot node's drain().

SELF-HEALING (fixed 2026-07-09): the serialtap creates/initializes the ring LAZILY — the file may not exist
when a client connects (turret parked at boot -> AVA hasn't read ttyS3 yet), the magic may be written after
the file appears, and the tap may recreate the file (new inode). So we don't mmap once and trust it forever:
we (re)open when the file appears, retry until the magic is valid, and re-mmap whenever the inode changes.
This removes the "restart ring_forward after every reboot to get /scan" dance.

Usage:  python3 ring_forward.py --path /tmp/lds_ring.buf --port 9901 [--magic 0x0031534444530001]
No ROS, no deps beyond stdlib — safe to keep after ROS is removed from the robot.
"""
import argparse, mmap, os, socket, struct, threading, time

HDR = 64
RING = 256 * 1024


def try_open(path, magic):
    """(re)open+mmap the ring if it exists, is full-size, and has a valid magic. Returns (mm, ino) or (None,None)."""
    try:
        st = os.stat(path)
    except OSError:
        return None, None
    if st.st_size < HDR + RING:            # tap still initializing (ftruncate not done) -> wait
        return None, None
    try:
        fd = os.open(path, os.O_RDONLY)
        mm = mmap.mmap(fd, HDR + RING, mmap.MAP_SHARED, mmap.PROT_READ)
        os.close(fd)
    except OSError:
        return None, None
    if magic and struct.unpack_from('<Q', mm, 8)[0] != magic:
        mm.close(); return None, None      # magic not written yet -> tap not fully up
    return mm, st.st_ino


def handle(conn, path, magic):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    mm = None
    ino = None
    read_pos = 0
    try:
        while True:
            # re-open on: first use, file gone/replaced (inode change), or a still-invalid mapping
            try:
                cur_ino = os.stat(path).st_ino
            except OSError:
                cur_ino = None
            if mm is None or cur_ino != ino:
                if mm is not None:
                    try: mm.close()
                    except Exception: pass
                    mm = None
                mm, ino = try_open(path, magic)
                if mm is None:
                    time.sleep(0.3); continue                 # not ready yet — keep trying (don't give up)
                read_pos = struct.unpack_from('<Q', mm, 0)[0]  # start fresh (skip backlog)
            wp = struct.unpack_from('<Q', mm, 0)[0]
            avail = wp - read_pos
            if avail <= 0:
                time.sleep(0.01); continue
            if avail > RING:                    # fell behind -> jump to freshest RING bytes
                read_pos = wp - RING; avail = RING
            start = read_pos % RING; end = wp % RING
            out = mm[HDR + start:HDR + end] if start < end else mm[HDR + start:HDR + RING] + mm[HDR:HDR + end]
            read_pos = wp
            conn.sendall(out)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        try: conn.close()
        except Exception: pass
        if mm is not None:
            try: mm.close()
            except Exception: pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--path', required=True, help='tmpfs ring file, e.g. /tmp/lds_ring.buf')
    ap.add_argument('--port', type=int, required=True)
    ap.add_argument('--magic', default='0', help='expected ring magic (hex); 0 = skip check')
    a = ap.parse_args()
    magic = int(a.magic, 0)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', a.port)); srv.listen(4)
    print(f'[ring_forward] {a.path} -> tcp/{a.port}', flush=True)
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle, args=(conn, a.path, magic), daemon=True).start()


if __name__ == '__main__':
    main()

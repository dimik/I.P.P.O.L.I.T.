#!/usr/bin/env python3
"""ring_forward.py — ROS-free tmpfs-ring -> TCP byte forwarder (runs on the ROBOT, in the chroot's python).

The LD_PRELOAD serial taps (libserialtap.so on ttyS3, mcutap on ttyS4) tee AVA's raw serial into tmpfs shm
rings (/tmp/lds_ring.buf, /tmp/mcu_ring.buf). To move ROS entirely onto the COMPANION, this streams a ring's
raw bytes over TCP so the (unchanged) ROS decode node can run on the Q6A instead of in the robot chroot.

Generic + stateless-per-client: each client gets bytes from the moment it connects (skips backlog). Handles
the ring's circular wrap + reader-fell-behind exactly like the old on-robot node's drain().

Usage:  python3 ring_forward.py --path /tmp/lds_ring.buf --port 9901 [--magic 0x0031534444530001]
No ROS, no deps beyond stdlib — safe to keep after ROS is removed from the robot.
"""
import argparse, mmap, os, socket, struct, threading, time

HDR = 64
RING = 256 * 1024


def handle(conn, path, magic):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    mm = None
    read_pos = 0
    try:
        while True:
            if mm is None:
                if not os.path.exists(path):
                    time.sleep(0.2); continue
                fd = os.open(path, os.O_RDONLY)
                mm = mmap.mmap(fd, HDR + RING, mmap.MAP_SHARED, mmap.PROT_READ); os.close(fd)
                if magic and struct.unpack_from('<Q', mm, 8)[0] != magic:
                    mm.close(); mm = None; time.sleep(0.5); continue   # tap not up yet
                read_pos = struct.unpack_from('<Q', mm, 0)[0]          # start fresh (skip backlog)
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

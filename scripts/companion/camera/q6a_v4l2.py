"""Direct V4L2 multiplanar mmap capture for the Q6A CAMSS RDI node.

Replaces the old `v4l2-ctl --stream-to=FILE` + tail-and-seek hack (piping to stdout HANGS this CAMSS
driver; the file tail loses ~4 fps to polling). This does a proper MMAP streaming loop and, each call,
drains all ready buffers and returns only the freshest frame (low latency, full sensor rate).

The node is Video-Capture-**Multiplanar**, so we drive the raw ioctls with linuxpy's ctypes structs
(linuxpy's high-level VideoCapture has no mplane/planes handling). pBAA == V4L2_PIX_FMT_SBGGR10P.
"""
import ctypes, fcntl, mmap, os, select
import linuxpy.video.raw as r

_MPLANE = r.BufType.VIDEO_CAPTURE_MPLANE
_MMAP = r.Memory.MMAP
_FIELD_NONE = 1


class V4l2Cam:
    def __init__(self, dev="/dev/video0", width=1456, height=1088, nbufs=4, pixelformat=None):
        self.fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        try:
            f = r.v4l2_format(); f.type = _MPLANE
            f.fmt.pix_mp.width = width; f.fmt.pix_mp.height = height
            f.fmt.pix_mp.pixelformat = int(pixelformat if pixelformat else r.PixelFormat.SBGGR10P)  # default 'pBAA' RAW10
            f.fmt.pix_mp.num_planes = 1
            f.fmt.pix_mp.field = _FIELD_NONE
            fcntl.ioctl(self.fd, r.IOC.S_FMT, f)
            self.frame_size = int(f.fmt.pix_mp.plane_fmt[0].sizeimage)

            req = r.v4l2_requestbuffers(); req.count = nbufs; req.type = _MPLANE; req.memory = _MMAP
            fcntl.ioctl(self.fd, r.IOC.REQBUFS, req)
            self.n = req.count

            self.maps = []
            for i in range(self.n):
                buf, planes = self._mkbuf(i)
                fcntl.ioctl(self.fd, r.IOC.QUERYBUF, buf)
                self.maps.append(mmap.mmap(self.fd, planes[0].length, mmap.MAP_SHARED,
                                           mmap.PROT_READ, offset=planes[0].m.mem_offset))
                self._qbuf(i)
            fcntl.ioctl(self.fd, r.IOC.STREAMON, ctypes.c_int(int(_MPLANE)))
        except Exception:
            os.close(self.fd); raise

    def _mkbuf(self, index=0):
        buf = r.v4l2_buffer(); buf.type = _MPLANE; buf.memory = _MMAP; buf.index = index
        planes = (r.v4l2_plane * 1)()
        buf.length = 1
        buf.m.planes = ctypes.cast(planes, ctypes.POINTER(r.v4l2_plane))
        return buf, planes

    def _qbuf(self, index):
        buf, _ = self._mkbuf(index)
        fcntl.ioctl(self.fd, r.IOC.QBUF, buf)

    def read_latest(self, timeout=1.0):
        """Block up to `timeout` for a frame, drain all ready buffers, return bytes of the freshest."""
        rr, _, _ = select.select([self.fd], [], [], timeout)
        if not rr:
            return None
        latest = None
        while True:
            buf, _ = self._mkbuf()
            try:
                fcntl.ioctl(self.fd, r.IOC.DQBUF, buf)
            except OSError:                       # EAGAIN — no more ready buffers
                break
            if latest is not None:
                self._qbuf(latest)                # requeue the older frame we're dropping
            latest = int(buf.index)
            if not select.select([self.fd], [], [], 0)[0]:
                break
        if latest is None:
            return None
        data = self.maps[latest][:self.frame_size]   # copy the freshest frame out of the mmap
        self._qbuf(latest)
        return data

    def close(self):
        try: fcntl.ioctl(self.fd, r.IOC.STREAMOFF, ctypes.c_int(int(_MPLANE)))
        except Exception: pass
        for mm in getattr(self, "maps", []):
            try: mm.close()
            except Exception: pass
        try: os.close(self.fd)
        except Exception: pass


if __name__ == "__main__":                        # standalone: measure capture rate
    import time, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) or ".")
    import q6a_camstream as C
    C.setup_pipeline(2, 3000, 300); time.sleep(1)
    import subprocess; subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False); time.sleep(1)
    cam = V4l2Cam("/dev/video0", C.W, C.H)
    print("frame_size:", cam.frame_size, "(expected", C.FRAME, ")")
    t0 = time.time(); n = 0
    while time.time() - t0 < 5:
        d = cam.read_latest()
        if d and len(d) == C.FRAME: n += 1
    print("V4L2 mmap read_latest fps: %.1f" % (n / 5))
    cam.close()

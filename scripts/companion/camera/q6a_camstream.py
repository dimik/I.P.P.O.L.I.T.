#!/usr/bin/env python3
"""IMX296 -> MJPEG-over-HTTP streamer for the Radxa Dragon Q6A (run ON the Q6A).

Sets up the CAMSS pipeline, captures raw MIPI-RAW10 (pBAA) frames from /dev/video0 via
`v4l2-ctl --stream-to=-`, unpacks + debayers them in numpy (no OpenCV needed), applies a
display auto-stretch, runs an overlay hook (YOLO boxes later), JPEG-encodes with PIL, and
serves multipart/x-mixed-replace MJPEG on :8092. View from the Odyssey with view_q6a_cam.sh.

Usage:  python3 q6a_camstream.py [--cam 2] [--port 8092] [--full]
  --full : full-res debayer (1456x1088, slower); default is fast half-res super-pixel (728x544).
"""
import argparse, subprocess, threading, sys
import numpy as np
from PIL import Image
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

W, H = 1456, 1088
STRIDE = 1824            # bytes/line for pBAA (1456*10/8=1820, padded to 1824)
FRAME = STRIDE * H       # 1,984,512 bytes/frame (v4l2 sizeimage)

# --- CAMSS media pipeline: sensor -> csiphyN -> csidX -> vfe0_rdi0 -> /dev/video0 ---
CAM_MAP = {2: ("msm_csiphy2", "msm_csid0"), 3: ("msm_csiphy3", "msm_csid1")}

def setup_pipeline(cam, exposure, gain):
    phy, csid = CAM_MAP[cam]
    rdi = "msm_vfe0_rdi0" if cam == 2 else "msm_vfe0_rdi1"
    m = ["media-ctl", "-d", "/dev/media0"]
    subprocess.run(m + ["-l", f'"{phy}":1 -> "{csid}":0 [1]'], check=False)
    subprocess.run(m + ["-l", f'"{csid}":1 -> "{rdi}":0 [1]'], check=False)
    fmt = "[fmt:SBGGR10_1X10/1456x1088]"
    # sensor entity name = "imx296 <cci-bus>-001a"; discover it from a link line in the topology
    top = subprocess.run(m + ["-p"], capture_output=True, text=True).stdout
    sensor = next((l.split('"')[1] for l in top.splitlines() if "imx296" in l and '"' in l), None)
    for ent in ([f'"{sensor}":0'] if sensor else []) + [f'"{phy}":0', f'"{phy}":1',
                f'"{csid}":0', f'"{csid}":1', f'"{rdi}":0']:
        subprocess.run(m + ["-V", f"{ent} {fmt}"], check=False)
    # exposure/gain: the sensor defaults to gain=0 + short exposure -> dark + noisy. Open it up.
    if sensor:
        sd = subprocess.run(m + ["-e", sensor], capture_output=True, text=True).stdout.strip()
        if sd:
            for c, v in [("vertical_blanking", exposure + 200), ("exposure", exposure),
                         ("analogue_gain", gain)]:
                subprocess.run(["v4l2-ctl", "-d", sd, "--set-ctrl", f"{c}={v}"], check=False)
    return rdi

def unpack_raw10(buf):
    """MIPI RAW10 packed -> uint16 Bayer (H, W). 4px in 5 bytes; b4 holds the 4x2 LSBs."""
    a = np.frombuffer(buf, np.uint8).reshape(H, STRIDE)[:, :1820].reshape(H, 364, 5)
    hi = a[:, :, 0:4].astype(np.uint16) << 2
    lo = a[:, :, 4].astype(np.uint16)
    px = hi | ((lo[:, :, None] >> (2 * np.arange(4))) & 3)
    return px.reshape(H, W)                      # 10-bit values 0..1023

def debayer(px, full):
    """BGGR Bayer -> RGB uint8 with a per-frame contrast stretch."""
    if full:
        img = (px >> 2).astype(np.uint8)         # quick 10->8; simple bilinear-ish via slicing
        rgb = np.empty((H, W, 3), np.uint8)
        rgb[..., 2] = img; rgb[..., 1] = img; rgb[..., 0] = img  # placeholder gray (full debayer TODO)
        out = rgb
    else:  # fast 2x2 super-pixel: BGGR -> half-res RGB
        b = px[0::2, 0::2]; g = (px[0::2, 1::2] + px[1::2, 0::2]) // 2; r = px[1::2, 1::2]
        out = np.dstack([r, g, b]).astype(np.float32)
    # destripe both axes: remove per-row (horizontal) AND per-column (vertical) fixed-pattern offsets.
    g = np.median(out, axis=(0, 1), keepdims=True)
    out -= np.median(out, axis=1, keepdims=True) - g   # horizontal-line FPN (per-row offset)
    out -= np.median(out, axis=0, keepdims=True) - g   # vertical-line FPN (per-column offset)
    out = np.clip(out, 0, None)
    # gray-world white balance: scale each channel to a common mean so a neutral surface reads gray.
    # (A per-channel *stretch* only equalizes endpoints, not the mean -> green midtones stayed dominant.)
    means = out.reshape(-1, 3).mean(axis=0)
    out *= means.mean() / np.maximum(means, 1e-3)
    # global contrast stretch for exposure/display
    lo, hi = np.percentile(out, 1), np.percentile(out, 99)
    out = np.clip((out - lo) * (255.0 / max(hi - lo, 1)), 0, 255)
    return out.astype(np.uint8)

def draw_overlay(rgb, dets=None):
    """Hook for YOLO overlay — dets: list of (x1,y1,x2,y2,label,conf) in rgb pixel coords.
    Drawn with PIL later once the NPU detector feeds boxes. For now a no-op passthrough."""
    return rgb

class State:
    jpeg = None
    lock = threading.Lock()
    clients = 0                 # active /stream viewers
    wake = threading.Event()    # signalled when a viewer connects

def process(buf, full):
    rgb = draw_overlay(debayer(unpack_raw10(buf), full))
    bio = BytesIO(); Image.fromarray(rgb).save(bio, "JPEG", quality=80)
    with State.lock:
        State.jpeg = bio.getvalue()

def capture_loop(rdi, full, batch=16):
    """v4l2-ctl --stream-count=0 (continuous-to-pipe) HANGS on this CAMSS driver, but bounded
    count=N-to-file works. So capture batches to tmpfs and process them, in a loop."""
    tmp = "/dev/shm/q6a_cap.raw"
    import time
    while True:
        if State.clients == 0:                 # no viewer -> stop capturing (camera + CPU idle)
            State.jpeg = None
            State.wake.wait(); State.wake.clear()
            continue
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", "/dev/video0",
                 "--set-fmt-video=width=1456,height=1088,pixelformat=pBAA",
                 "--stream-mmap", f"--stream-count={batch}", f"--stream-to={tmp}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            with open(tmp, "rb") as f:
                while State.clients > 0:        # abort the batch early if all viewers left
                    buf = f.read(FRAME)
                    if len(buf) < FRAME:
                        break
                    process(buf, full)
        except Exception as e:
            print("capture error:", e, flush=True)
            time.sleep(1)

PAGE = (b"<!doctype html><html><head><title>Q6A IMX296</title>"
        b"<style>body{margin:0;background:#111}img{width:100vw;height:100vh;object-fit:contain}</style>"
        b"</head><body><img src='/stream'></body></html>")

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE))); self.end_headers()
            self.wfile.write(PAGE); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        import time
        with State.lock:
            State.clients += 1
        State.wake.set()                       # wake the capture loop
        try:
            while True:
                with State.lock:
                    j = State.jpeg
                if j:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(j) + j + b"\r\n")
                time.sleep(0.04)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with State.lock:
                State.clients -= 1             # last viewer out -> capture loop idles

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=2, choices=(2, 3))
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--exposure", type=int, default=8000, help="sensor exposure (lines); higher=brighter")
    ap.add_argument("--gain", type=int, default=120, help="analogue gain 0..480 (0.1dB); higher=brighter+noisier")
    args = ap.parse_args()
    print("start: setting up pipeline...", flush=True)
    rdi = setup_pipeline(args.cam, args.exposure, args.gain)
    print(f"pipeline ready ({rdi}); starting capture + server", flush=True)
    threading.Thread(target=capture_loop, args=(rdi, args.full), daemon=True).start()
    print(f"MJPEG stream on http://0.0.0.0:{args.port}/  (cam{args.cam})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()

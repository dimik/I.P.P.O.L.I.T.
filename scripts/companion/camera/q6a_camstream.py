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

def _smooth(v, win):
    """Low-pass a (N,3) brightness profile along axis 0 with a box filter (for destriping)."""
    k = np.ones(win) / win
    return np.stack([np.convolve(v[:, c], k, mode="same") for c in range(3)], axis=1)

def _conv3(a, k):
    """3x3 convolution via edge-padded shifted adds (no scipy)."""
    p = np.pad(a, 1, mode="edge"); o = np.zeros_like(a)
    for dy in range(3):
        for dx in range(3):
            if k[dy, dx]:
                o += k[dy, dx] * p[dy:dy + a.shape[0], dx:dx + a.shape[1]]
    return o

_KG = np.array([[0, .25, 0], [.25, 0, .25], [0, .25, 0]])          # 4-neighbour G
_KRB = np.array([[.25, .5, .25], [.5, 1, .5], [.25, .5, .25]])     # bilinear R/B upsample

def demosaic_bggr(px):
    """Full-resolution bilinear demosaic of a BGGR Bayer plane -> (H,W,3) float."""
    px = px.astype(np.float32)
    R = np.zeros_like(px); G = np.zeros_like(px); B = np.zeros_like(px)
    B[0::2, 0::2] = px[0::2, 0::2]
    G[0::2, 1::2] = px[0::2, 1::2]; G[1::2, 0::2] = px[1::2, 0::2]
    R[1::2, 1::2] = px[1::2, 1::2]
    G = np.where(G > 0, G, _conv3(G, _KG))     # interpolate G only at R/B sites
    R = _conv3(R, _KRB); B = _conv3(B, _KRB)   # bilinear-upsample the quarter-density R/B
    return np.dstack([R, G, B])

def debayer(px, full=True):
    """BGGR Bayer -> full-res RGB uint8: demosaic -> destripe -> white balance -> contrast stretch."""
    if full:
        out = demosaic_bggr(px)                                  # sharp 1456x1088
    else:  # fast half-res super-pixel (fallback)
        b = px[0::2, 0::2]; g = (px[0::2, 1::2] + px[1::2, 0::2]) // 2; r = px[1::2, 1::2]
        out = np.dstack([r, g, b]).astype(np.float32)
    # destripe: subtract the HIGH-FREQUENCY part of the per-column and per-row brightness profiles
    col = out.mean(axis=0); out -= (col - _smooth(col, 41))[None, :, :]   # vertical stripes
    row = out.mean(axis=1); out -= (row - _smooth(row, 41))[:, None, :]   # horizontal stripes
    out = np.clip(out, 0, None)
    # gray-world white balance: scale each channel to a common mean so a neutral surface reads gray
    means = out.reshape(-1, 3).mean(axis=0)
    out *= means.mean() / np.maximum(means, 1e-3)
    # global contrast stretch for display
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

def capture_loop(rdi, full, batch=200):
    """Smooth continuous capture. v4l2-ctl --stream-count=0-to-pipe HANGS this CAMSS driver, and a
    capture-then-process batch stutters (freeze during capture). Instead: run a large finite batch
    writing to a tmpfs file while we *tail* it, always seeking to the LATEST complete frame (dropping
    any we can't keep up with -> low latency, no stutter). Brief hiccup only every ~batch frames."""
    import time, os
    tmp = "/dev/shm/q6a_cap.raw"
    while True:
        if State.clients == 0:                 # no viewer -> stop capturing (camera + CPU idle)
            State.jpeg = None
            State.wake.wait(); State.wake.clear()
            continue
        try:
            open(tmp, "wb").close()             # truncate before the new batch
            proc = subprocess.Popen(
                ["v4l2-ctl", "-d", "/dev/video0",
                 "--set-fmt-video=width=1456,height=1088,pixelformat=pBAA",
                 "--stream-mmap", f"--stream-count={batch}", f"--stream-to={tmp}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            f = open(tmp, "rb"); last = 0
            while State.clients > 0:
                n = os.path.getsize(tmp) // FRAME
                if n > last:                    # new frame available -> jump to the freshest one
                    f.seek((n - 1) * FRAME)
                    buf = f.read(FRAME)
                    if len(buf) == FRAME:
                        process(buf, full); last = n
                elif proc.poll() is not None:   # batch finished -> restart
                    break
                else:
                    time.sleep(0.01)
            f.close()
            if proc.poll() is None:
                proc.terminate()
                try: proc.wait(timeout=3)
                except Exception: proc.kill()
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
    ap.add_argument("--fast", action="store_true", help="half-res debayer (faster, softer); default is full-res")
    ap.add_argument("--exposure", type=int, default=3000, help="sensor exposure (lines); higher=brighter+more motion blur")
    ap.add_argument("--gain", type=int, default=100, help="analogue gain 0..480 (0.1dB); higher=brighter+noisier")
    args = ap.parse_args()
    print("start: setting up pipeline...", flush=True)
    rdi = setup_pipeline(args.cam, args.exposure, args.gain)
    print(f"pipeline ready ({rdi}); starting capture + server", flush=True)
    threading.Thread(target=capture_loop, args=(rdi, not args.fast), daemon=True).start()
    print(f"MJPEG stream on http://0.0.0.0:{args.port}/  (cam{args.cam})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()

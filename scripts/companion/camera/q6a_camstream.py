#!/usr/bin/env python3
"""IMX296 -> MJPEG-over-HTTP streamer for the Radxa Dragon Q6A (run ON the Q6A).

Sets up the CAMSS pipeline, captures raw MIPI-RAW10 (pBAA) frames from /dev/video0 via
`v4l2-ctl --stream-to=-`, unpacks + debayers them in numpy (no OpenCV needed), applies a
display auto-stretch, runs an overlay hook (YOLO boxes later), JPEG-encodes with PIL, and
serves multipart/x-mixed-replace MJPEG on :8092. View from the Odyssey with view_q6a_cam.sh.

Usage:  python3 q6a_camstream.py [--cam 2] [--port 8092] [--full]
  --full : full-res debayer (1456x1088, slower); default is fast half-res super-pixel (728x544).
"""
import argparse, subprocess, threading, sys, os
import numpy as np
from PIL import Image
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

W, H = 1456, 1088
STRIDE = 1824            # bytes/line for pBAA (1456*10/8=1820, padded to 1824)
FRAME = STRIDE * H       # 1,984,512 bytes/frame (v4l2 sizeimage)

# --- Color pipeline: deterministic, measured from raw Bayer statistics (NOT per-frame guessing) ---
# A raw Bayer sensor needs black-level subtraction + fixed white-balance gains. On this IMX296 the two
# green phases read ~1.6x red/blue (CFA + sensor QE peak in green) -> without WB everything looks green.
# Measured from raw: black~56, R/G=B/G~0.62 globally and SCENE-INDEPENDENT, so a single fixed gain per
# channel neutralises it everywhere (no fragile per-pixel flat-field). Re-derive on a grey card via
# --calibrate; it writes the 4 numbers below to PROFILE, which load_profile() then uses to override.
BLACK_LEVEL = 56.0
WB_R, WB_G, WB_B = 1.60, 1.00, 1.52          # raw per-channel gains -> neutral grey (measured)
SHADE = None                                  # optional (H,W,3) radial COLOR-shading gain (green-relative)
GPU = None                                    # optional Adreno OpenCL ISP (q6a_gpu.GpuDemosaic)
DESTRIPE = False                              # optional FPN band removal (CPU, ~32ms); off by default
TARGET_MEAN = 95.0                            # tone-map target brightness
ACCEL = threading.Lock()                       # serialize GPU (Adreno) and NPU (Hexagon) — concurrent use crashes
def init_gpu():
    global GPU
    try:
        from q6a_gpu import GpuDemosaic
        GPU = GpuDemosaic(W, H)
        GPU.set_shade(SHADE)                   # upload color-shading map once (if a profile is loaded)
        print(f"GPU ISP enabled: {GPU.dev_name} (full demosaic+WB+tonemap on GPU)", flush=True)
    except Exception as e:
        GPU = None
        print(f"GPU unavailable ({e}); using CPU numpy demosaic", flush=True)

def _auto_scale(px):
    """Brightness scale for the tone-map, from a cheap raw subsample (auto-exposure). Odd stride
    cycles the Bayer phases; CFA-weighted mean gain approximates the demosaiced brightness."""
    sub = px[::3, ::3].astype(np.float32) - BLACK_LEVEL
    np.clip(sub, 0, None, out=sub)
    avg_wb = (WB_R + 2.0 * WB_G + WB_B) / 4.0          # BGGR: 1 B, 2 G, 1 R per 2x2
    return TARGET_MEAN / max(sub.mean() * avg_wb, 1.0)
PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imx296_wb.npz")
def load_profile():
    global BLACK_LEVEL, WB_R, WB_G, WB_B, SHADE
    if os.path.exists(PROFILE):
        z = np.load(PROFILE); BLACK_LEVEL = float(z["black"]); WB_R, WB_G, WB_B = (float(x) for x in z["wb"])
        msg = f"loaded WB profile: black={BLACK_LEVEL:.0f} wb=({WB_R:.3f},{WB_G:.3f},{WB_B:.3f})"
        if "shade" in z.files:                 # small green-relative gain grid -> upscale to full res
            s = z["shade"]
            SHADE = np.stack([np.asarray(Image.fromarray(s[..., c]).resize((W, H), Image.BILINEAR))
                              for c in range(3)], axis=2).astype(np.float32)
            msg += f" + shading map {s.shape[:2]}"
        print(msg, flush=True)
    else:
        print(f"using default WB: black={BLACK_LEVEL:.0f} wb=({WB_R:.3f},{WB_G:.3f},{WB_B:.3f}) (no shading map)", flush=True)

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
    """BGGR Bayer -> full-res RGB uint8: black-level + raw WB -> demosaic -> destripe -> tone map."""
    if GPU is not None and full:
        scale = _auto_scale(px)              # cheap CPU auto-exposure metric from the raw subsample
        with ACCEL:                          # serialize GPU vs NPU — concurrent Adreno+Hexagon crashes
            out = GPU.isp(px, BLACK_LEVEL, WB_R, WB_G, WB_B, scale)  # full ISP on GPU -> uint8
        return _destripe_u8(out) if DESTRIPE else out
    # CPU fallback: black+WB+demosaic + shade + destripe + tone map
    px = px.astype(np.float32) - BLACK_LEVEL
    np.clip(px, 0, None, out=px)
    px[1::2, 1::2] *= WB_R                    # R sites   (raw white balance, per Bayer position)
    px[0::2, 0::2] *= WB_B                    # B sites   (G sites keep WB_G=1)
    out = _debayer_cpu(px, full)
    if SHADE is not None and SHADE.shape == out.shape:
        out *= SHADE                          # radial color-shading correction (magenta centre -> green edge)
    return _post(out)

def _destripe_u8(u8):
    """Optional FPN destripe on the GPU-produced uint8 frame (col/row high-pass)."""
    out = u8.astype(np.float32)
    col = out.mean(0); out -= (col - _smooth(col, 41))[None, :, :]
    row = out.mean(1); out -= (row - _smooth(row, 41))[:, None, :]
    return np.clip(out, 0, 255).astype(np.uint8)

def _debayer_cpu(px, full):
    if full:
        return demosaic_bggr(px)                                 # sharp 1456x1088
    b = px[0::2, 0::2]; g = (px[0::2, 1::2] + px[1::2, 0::2]) * 0.5; r = px[1::2, 1::2]
    return np.dstack([r, g, b]).astype(np.float32)              # fast half-res super-pixel

_GAMMA_LUT = (255.0 * (np.arange(256) / 255.0) ** 0.7).astype(np.uint8)   # gamma 0.7 as a 256-entry LUT

def _post(out):
    """destripe (remove FPN column/row banding) + tone map (target-mean lift + gamma) -> uint8."""
    col = out.mean(axis=0); out -= (col - _smooth(col, 41))[None, :, :]   # vertical stripes
    row = out.mean(axis=1); out -= (row - _smooth(row, 41))[:, None, :]   # horizontal stripes
    out = np.clip(out, 0, None)
    out -= np.percentile(out[::4, ::4], 1)                 # black lift (subsampled percentile: 44ms->4ms)
    np.clip(out, 0, None, out=out)
    out *= 95.0 / max(out.mean(), 1.0)                     # target mean brightness ~95 (robust to a lamp)
    return _GAMMA_LUT[np.clip(out, 0, 255).astype(np.uint8)]  # gamma via LUT (float pow 59ms->~19ms)

def calibrate(cam, exposure, gain, n=40):
    """White-balance calibration: aim at a UNIFORM grey/white surface; measure the raw black level and
    per-channel WB gains that render it neutral. Saves 4 numbers to PROFILE (load_profile uses them)."""
    import time
    setup_pipeline(cam, exposure, gain)
    print(f"CALIBRATION: fill the frame with a UNIFORM grey/white surface. Capturing {n} frames...", flush=True)
    subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False); time.sleep(1)
    tmp = "/dev/shm/q6a_cal.raw"
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0",
                    "--set-fmt-video=width=1456,height=1088,pixelformat=pBAA",
                    "--stream-mmap", f"--stream-count={n}", f"--stream-to={tmp}"], check=True)
    acc = np.zeros((H, W), np.float64); cnt = 0
    with open(tmp, "rb") as f:
        for _ in range(n):
            b = f.read(FRAME)
            if len(b) < FRAME: break
            acc += unpack_raw10(b); cnt += 1
    px = acc / max(cnt, 1)                                  # averaged raw Bayer (low noise)
    black = float(min(px[0::2, 0::2].min(), px[1::2, 1::2].min(), px[0::2, 1::2].min()))
    Rm = (px[1::2, 1::2] - black).mean(); Bm = (px[0::2, 0::2] - black).mean()
    Gm = ((px[0::2, 1::2] - black).mean() + (px[1::2, 0::2] - black).mean()) / 2
    wb = np.array([Gm / max(Rm, 1e-3), 1.0, Gm / max(Bm, 1e-3)], np.float32)
    # radial COLOR shading: WB-correct + demosaic the flat field, fit a smooth quadratic per channel
    # (robust to local non-uniformity), then a GREEN-RELATIVE gain that flattens R/G,B/G across the field
    # without touching luminance (green gain = 1 -> no vignette boost -> no corner-noise amplification).
    q = np.clip(px - black, 0, None); q[1::2, 1::2] *= wb[0]; q[0::2, 0::2] *= wb[2]
    flat = demosaic_bggr(q)                                 # (H,W,3) WB-corrected flat field
    GX, GY = 32, 24
    coarse = np.stack([np.asarray(Image.fromarray(flat[..., c]).resize((GX, GY), Image.BILINEAR))
                       for c in range(3)], axis=2).astype(np.float64)     # (GY,GX,3) mean-pooled
    xs = np.linspace(-1, 1, GX); ys = np.linspace(-1, 1, GY); X, Y = np.meshgrid(xs, ys)
    A = np.stack([np.ones_like(X), X, Y, X * X, X * Y, Y * Y], axis=-1).reshape(-1, 6)  # quadratic basis
    model = np.stack([(A @ np.linalg.lstsq(A, coarse[..., c].ravel(), rcond=None)[0]).reshape(GY, GX)
                      for c in range(3)], axis=2)            # smooth per-channel surface
    cy, cx = GY // 2, GX // 2
    ratioR = (model[..., 0] / np.maximum(model[..., 1], 1e-3))    # R/G across field
    ratioB = (model[..., 2] / np.maximum(model[..., 1], 1e-3))    # B/G across field
    gR = np.clip(ratioR[cy, cx] / np.maximum(ratioR, 1e-3), 0.5, 2.0)   # gain to flatten R/G to centre
    gB = np.clip(ratioB[cy, cx] / np.maximum(ratioB, 1e-3), 0.5, 2.0)
    shade = np.stack([gR, np.ones_like(gR), gB], axis=2).astype(np.float32)  # green untouched
    np.savez(PROFILE, black=np.float32(black), wb=wb, shade=shade)
    print(f"saved profile ({cnt} frames): black={black:.0f} wb=({wb[0]:.3f},1.000,{wb[2]:.3f}) "
          f"shade R[{gR.min():.2f}-{gR.max():.2f}] B[{gB.min():.2f}-{gB.max():.2f}] to {PROFILE}", flush=True)

from PIL import ImageDraw, ImageFont
_FONT = ImageFont.load_default()
# stable per-class-ish palette (indexed by hash of label) for box colors
_PALETTE = [(255, 64, 64), (64, 200, 64), (64, 160, 255), (255, 200, 0), (255, 64, 255),
            (0, 220, 220), (255, 128, 0), (160, 96, 255)]

def draw_overlay(rgb, dets=None):
    """Draw YOLO detections. dets: list of (x1,y1,x2,y2,label,conf) in rgb pixel coords."""
    if not dets:
        return rgb
    im = Image.fromarray(rgb); d = ImageDraw.Draw(im)
    for x1, y1, x2, y2, label, conf in dets:
        col = _PALETTE[hash(label) % len(_PALETTE)]
        d.rectangle([x1, y1, x2, y2], outline=col, width=3)
        tag = f"{label} {conf:.2f}"
        tw = d.textlength(tag, font=_FONT); th = 12
        d.rectangle([x1, y1 - th - 2, x1 + tw + 4, y1], fill=col)
        d.text((x1 + 2, y1 - th - 2), tag, fill=(0, 0, 0), font=_FONT)
    return np.asarray(im)

class State:
    jpeg = None
    lock = threading.Lock()
    clients = 0                 # active /stream viewers
    wake = threading.Event()    # signalled when a viewer connects
    rgb = None                  # latest full-res RGB frame (for the detector to consume)
    dets = []                   # latest YOLO detections (drawn onto every frame)

def process(buf, full):
    rgb = debayer(unpack_raw10(buf), full)
    with State.lock:
        State.rgb = rgb; dets = State.dets
    rgb = draw_overlay(rgb, dets)                       # draw_overlay returns a new array (PIL)
    bio = BytesIO(); Image.fromarray(rgb).save(bio, "JPEG", quality=80)
    with State.lock:
        State.jpeg = bio.getvalue()

def detector_loop(interval=0.25):
    """Run YOLO on the latest frame at ~1/interval fps when a viewer is connected. Optional: if the
    NPU stack or model binary is missing, the detector stays disabled and the stream runs plain."""
    import time
    try:
        from q6a_yolo import YoloDetector
        det = YoloDetector()
        print("YOLO detector loaded (NPU)", flush=True)
    except Exception as e:
        print(f"YOLO detector disabled: {e}", flush=True); return
    while True:
        if State.clients == 0 or State.rgb is None:
            State.dets = []
            State.wake.wait(0.5); continue
        with State.lock:
            rgb = State.rgb
        try:
            with ACCEL:                      # serialize NPU vs GPU demosaic (concurrent accel crashes)
                dets = det.infer(rgb)
            with State.lock:
                State.dets = dets
        except Exception as e:
            print("detect error:", e, flush=True); time.sleep(0.5)
        time.sleep(interval)

def capture_loop(rdi, full, batch=300):
    """Smooth continuous capture. v4l2-ctl --stream-count=0-to-pipe HANGS this CAMSS driver, and a
    capture-then-process batch stutters (freeze during capture). Instead: run a large finite batch
    writing to a tmpfs file while we *tail* it, always seeking to the LATEST complete frame (dropping
    any we can't keep up with -> low latency, no stutter). Brief hiccup only every ~batch frames."""
    import time, os
    tmp = "/dev/shm/q6a_cap.raw"
    subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False)  # clear orphans once at startup
    while True:
        if State.clients == 0:                 # no viewer -> stop capturing (camera + CPU idle)
            State.jpeg = None
            subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False)
            State.wake.wait(); State.wake.clear()
            continue
        proc = None
        try:
            open(tmp, "wb").close()             # truncate before the new batch
            proc = subprocess.Popen(
                ["v4l2-ctl", "-d", "/dev/video0",
                 "--set-fmt-video=width=1456,height=1088,pixelformat=pBAA",
                 "--stream-mmap", f"--stream-count={batch}", f"--stream-to={tmp}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            f = open(tmp, "rb"); last = 0; last_sz = -1; grew = time.time()
            while State.clients > 0:
                sz = os.path.getsize(tmp)
                if sz != last_sz:
                    last_sz = sz; grew = time.time()
                n = sz // FRAME
                if n > last:                    # new frame -> jump to the freshest one
                    f.seek((n - 1) * FRAME)
                    buf = f.read(FRAME)
                    if len(buf) == FRAME:
                        process(buf, full); last = n
                elif proc.poll() is not None:   # batch finished -> restart
                    break
                elif time.time() - grew > 2.5:  # capture stalled -> kill + restart
                    break
                else:
                    time.sleep(0.01)
            f.close()
        except Exception as e:
            print("capture error:", e, flush=True); time.sleep(1)
        finally:
            if proc and proc.poll() is None:
                proc.terminate()
                try: proc.wait(timeout=2)
                except Exception: proc.kill()

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
    ap.add_argument("--exposure", type=int, default=3000, help="sensor exposure (lines); LOWER=higher fps (frame length scales with exposure) but noisier in dim light. 3000~21fps, 6000~13fps")
    ap.add_argument("--gain", type=int, default=200, help="analogue gain 0..480 (0.1dB); higher=brighter+noisier")
    ap.add_argument("--calibrate", action="store_true", help="capture a flat-field color profile (aim at a white/gray surface)")
    ap.add_argument("--no-yolo", action="store_true", help="disable the NPU YOLO detection overlay")
    ap.add_argument("--gpu", action="store_true", help="full-res ISP on the Adreno GPU (OpenCL) instead of CPU")
    ap.add_argument("--destripe", action="store_true", help="also remove FPN column/row banding (CPU, ~32ms)")
    args = ap.parse_args()
    DESTRIPE = args.destripe
    if args.calibrate:
        calibrate(args.cam, args.exposure, args.gain)
        sys.exit(0)
    load_profile()
    if args.gpu:
        init_gpu()
        if GPU is None:
            args.fast = True         # GPU unavailable -> half-res CPU (fast) instead of slow full-res CPU
    print("start: setting up pipeline...", flush=True)
    rdi = setup_pipeline(args.cam, args.exposure, args.gain)
    print(f"pipeline ready ({rdi}); starting capture + server", flush=True)
    threading.Thread(target=capture_loop, args=(rdi, not args.fast), daemon=True).start()
    if not args.no_yolo:
        threading.Thread(target=detector_loop, daemon=True).start()
    print(f"MJPEG stream on http://0.0.0.0:{args.port}/  (cam{args.cam})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()

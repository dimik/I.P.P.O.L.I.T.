#!/usr/bin/env python3
"""IMX296 -> MJPEG-over-HTTP streamer for the Radxa Dragon Q6A (run ON the Q6A).

Sets up the CAMSS pipeline, captures raw MIPI-RAW10 (pBAA) frames from /dev/video0 via
`v4l2-ctl --stream-to=-`, unpacks + debayers them in numpy (no OpenCV needed), applies a
display auto-stretch, runs an overlay hook (YOLO boxes later), JPEG-encodes with PIL, and
serves multipart/x-mixed-replace MJPEG on :8092. View from the Odyssey with view_q6a_cam.sh.

Usage:  python3 q6a_camstream.py [--cam 2] [--port 8092] [--full]
  --full : full-res debayer (1456x1088, slower); default is fast half-res super-pixel (728x544).
"""
import argparse, subprocess, threading, sys, os, json
import numpy as np
from PIL import Image
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

W, H = 1456, 1088
STRIDE = 1824            # bytes/line for pBAA (1456*10/8=1820, padded to 1824)
FRAME = STRIDE * H       # 1,984,512 bytes/frame (v4l2 sizeimage)
OUT_W, OUT_H = W, H      # output (displayed/detected) resolution; halved by --bin

# --- Color pipeline: deterministic, measured from raw Bayer statistics (NOT per-frame guessing) ---
# A raw Bayer sensor needs black-level subtraction + fixed white-balance gains. On this IMX296 the two
# green phases read ~1.6x red/blue (CFA + sensor QE peak in green) -> without WB everything looks green.
# Measured from raw: black~56, R/G=B/G~0.62 globally and SCENE-INDEPENDENT, so a single fixed gain per
# channel neutralises it everywhere (no fragile per-pixel flat-field). Re-derive on a grey card via
# --calibrate; it writes the 4 numbers below to PROFILE, which load_profile() then uses to override.
BLACK_LEVEL = 56.0
BLACK_RGB = None                              # per-channel black pedestal (R,G,B) from a dark-frame calibration;
                                              # None -> use the single BLACK_LEVEL for all channels
WB_R, WB_G, WB_B = 1.60, 1.00, 1.52          # raw per-channel gains -> neutral grey (measured)
SHADE = None                                  # optional (H,W,3) radial COLOR-shading gain (green-relative)
GPU = None                                    # optional Adreno OpenCL ISP (q6a_gpu.GpuDemosaic)
CAMERA_MODEL = "imx296"                        # tuning/<model>.json -> ready-made CCM (add a camera = add a file)
CCM_CT = 3600                                  # colour temp (K) to interpolate the CCM to (indoor default)
CCM_ON = True                                  # apply the ready-made color correction matrix
DESTRIPE = False                              # optional FPN band removal (CPU, ~32ms); off by default
BIN = False                                   # GPU digital 2x2 binning -> half-res, ~2x less noise + faster
SENSOR_BIN = False                            # sensor 2x2 FD binning (charge-domain, cleaner): capture 728x544
TARGET_MEAN = 95.0                            # tone-map target brightness (DISPLAY, digital — see AE below)
# --- Sensor auto-exposure (real integration-time control, not the digital tone-map) ---
# The tone-map only rescales the DISPLAY; it can't recover raw pixels that clipped at 1023 in bright light.
# AE nudges the sensor exposure (lines) — and gain at the extremes — so the raw stays well-exposed with
# highlight headroom. vblank tracks exposure so frame length stays minimal (lower exposure -> higher fps).
SENSOR_SD = None                              # sensor subdev path (discovered in setup_pipeline)
EXPOSURE = 3000; GAIN = 240                   # current sensor exposure (lines) / gain (ctrl); AE updates these
AE_ON = True                                  # runtime sensor auto-exposure
AE_TARGET = 70.0                              # target raw high-byte MEDIAN (robust to dark corners + bright window)
AE_MIN_EXP = 30; AE_MAX_EXP = 4000            # exposure clamp (lines); cap keeps fps >=~24 + out of the deep-noise regime
# YOLO runs in a SEPARATE PROCESS (q6a_detector.py) sharing frames via shared memory. The Adreno (GPU
# ISP) and Hexagon (NPU) crash if driven concurrently in ONE process (shared userspace allocator
# corruption) but run fine across processes -> no lock, true concurrency. shm layout <-> q6a_detector.py.
MAX_DET = 32; CTRL_OFF = 32; CTRL_SIZE = CTRL_OFF + MAX_DET * 6 * 4
DET = None                                     # dict of shm views + the detector subprocess (None if disabled)
YOLO_FPS = 10                                   # cap NPU YOLO to this rate (0=unlimited ~26fps). A slow robot
                                               # doesn't need per-frame detection; boxes persist between updates,
                                               # so 10fps frees the NPU (~60% idle -> cooler, shares HTP w/ the LLM).
LABELS = [str(i) for i in range(80)]
TUNING_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning")
def load_ccm(model, ct):
    """Load the ready-made color correction matrix from tuning/<model>.json (Raspberry Pi libcamera tuning),
    linearly interpolated to the target colour temperature `ct`. Returns a (3,3) float32, or None if absent.
    This is the cross-channel colour science (fixes the residual spectral cast) — no per-unit guessing."""
    path = os.path.join(TUNING_DIR, f"{model}.json")
    if not os.path.exists(path):
        print(f"no tuning file {path} -> CCM disabled", flush=True); return None
    try:
        d = json.load(open(path))
        algos = {list(a)[0]: list(a.values())[0] for a in d["algorithms"]}
        ccms = sorted(algos["rpi.ccm"]["ccms"], key=lambda e: e["ct"])
        cts = [e["ct"] for e in ccms]
        if ct <= cts[0]: m = ccms[0]["ccm"]
        elif ct >= cts[-1]: m = ccms[-1]["ccm"]
        else:
            i = max(k for k in range(len(cts)) if cts[k] <= ct)
            f = (ct - cts[i]) / (cts[i + 1] - cts[i])
            m = [a + (b - a) * f for a, b in zip(ccms[i]["ccm"], ccms[i + 1]["ccm"])]
        print(f"loaded CCM from tuning/{model}.json @ {ct}K (of {len(ccms)} CTs {cts[0]}-{cts[-1]})", flush=True)
        return np.asarray(m, np.float32).reshape(3, 3)
    except Exception as e:
        print(f"CCM load failed ({e}); disabled", flush=True); return None

def init_gpu():
    global GPU
    try:
        from q6a_gpu import GpuDemosaic
        GPU = GpuDemosaic(W, H)
        GPU.set_shade(SHADE)                   # upload color-shading map once (if a profile is loaded)
        GPU.set_ccm(load_ccm(CAMERA_MODEL, CCM_CT) if CCM_ON else None)   # ready-made color matrix
        print(f"GPU ISP enabled: {GPU.dev_name} (full demosaic+WB+CCM+tonemap on GPU)", flush=True)
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

def _auto_scale_packed(buf):
    """Auto-exposure scale from the packed RAW10 high bytes (~value>>2; avoids the CPU unpack)."""
    a = np.frombuffer(buf, np.uint8)[:STRIDE * H].reshape(H, STRIDE)[:, :W * 10 // 8].reshape(H, W // 4, 5)
    m = a[::4, ::4, :4].astype(np.float32).mean() * 4.0 - BLACK_LEVEL   # high bytes -> ~10-bit level
    avg_wb = (WB_R + 2.0 * WB_G + WB_B) / 4.0
    return TARGET_MEAN / max(max(m, 0.0) * avg_wb, 1.0)
PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imx296_wb.npz")
def load_profile():
    global BLACK_LEVEL, BLACK_RGB, WB_R, WB_G, WB_B, SHADE
    if os.path.exists(PROFILE):
        z = np.load(PROFILE); BLACK_LEVEL = float(z["black"]); WB_R, WB_G, WB_B = (float(x) for x in z["wb"])
        if "blk" in z.files:                   # per-channel black pedestal (dark-frame calibrated)
            BLACK_RGB = tuple(float(x) for x in z["blk"])
        msg = f"loaded WB profile: black={BLACK_LEVEL:.0f}" + (f" blk={tuple(round(x,1) for x in BLACK_RGB)}" if BLACK_RGB else "") + f" wb=({WB_R:.3f},{WB_G:.3f},{WB_B:.3f})"
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
    global SENSOR_SD, EXPOSURE, GAIN
    EXPOSURE, GAIN = exposure, gain
    phy, csid = CAM_MAP[cam]
    rdi = "msm_vfe0_rdi0" if cam == 2 else "msm_vfe0_rdi1"
    m = ["media-ctl", "-d", "/dev/media0"]
    subprocess.run(m + ["-l", f'"{phy}":1 -> "{csid}":0 [1]'], check=False)
    subprocess.run(m + ["-l", f'"{csid}":1 -> "{rdi}":0 [1]'], check=False)
    fmt = f"[fmt:SBGGR10_1X10/{W}x{H}]"   # W,H = 728x544 in --sensor-bin (driver enables 2x2 FD binning)
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
            SENSOR_SD = sd
            # frame length = H + vblank must be >= exposure; use the MINIMUM -> max fps at this exposure
            # (no noise cost). The old "exposure+200" padded the frame ~1.4x longer than needed.
            vblank = max(30, exposure - H + 64)
            for c, v in [("exposure", 100), ("vertical_blanking", vblank), ("exposure", exposure),
                         ("analogue_gain", gain)]:
                subprocess.run(["v4l2-ctl", "-d", sd, "--set-ctrl", f"{c}={v}"], check=False)
    return rdi

def _set_exposure(exp):
    """Set sensor integration time (lines) + track vblank so the frame length stays minimal (max fps).
    Set vblank (VMAX) THEN exposure in one call (v4l2-ctl applies left-to-right) so exposure never exceeds
    VMAX. NB: do NOT stage a tiny exposure=100 first — that emits one near-black frame that the tone-map
    amplifies into a full-screen 'snow' flash on every AE adjustment."""
    if SENSOR_SD is None:
        return
    vblank = max(30, exp - H + 64)
    subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl",
                    f"vertical_blanking={vblank},exposure={exp}"], check=False)

_AE_N = 0
def auto_exposure(buf):
    """Real sensor AE from the packed RAW10 high bytes (cheap, no unpack). Every ~8 frames, nudge the
    sensor exposure (then gain at the floor/ceiling) so the raw is well-exposed with highlight headroom.
    Damped (per-step change clamped) to avoid hunting; a saturated-pixel check cuts exposure hard when
    highlights blow (mean-only AE gets fooled by a bright window)."""
    global EXPOSURE, GAIN, _AE_N
    if not AE_ON or SENSOR_SD is None:
        return
    _AE_N += 1
    if _AE_N % 8:                                  # ~4 Hz at 31 fps
        return
    a = np.frombuffer(buf, np.uint8)[:STRIDE * H].reshape(H, STRIDE)[:, :W * 10 // 8].reshape(H, W // 4, 5)
    hi = a[::4, ::4, :4].astype(np.float32)         # high-byte subsample (raw >> 2, 0..255)
    # Median brightness: robust to BOTH a dark surround (which pulls the mean down → overexpose to noise)
    # AND a bright window/lamp (which pulls the mean up → crush exposure → dark subject). The tone-map
    # normalizes DISPLAY brightness separately; AE just keeps the raw in a sane band (no clip, low noise).
    mid = float(np.median(hi))
    ratio = AE_TARGET / max(mid, 1.0)
    if 0.75 < ratio < 1.30:
        return                                      # within deadband -> hold steady (no hunting)
    ratio = min(max(ratio, 0.5), 2.0)
    target_exp = EXPOSURE * ratio                   # partial step toward target -> damped, no overshoot/chase
    new_exp = int(EXPOSURE + 0.4 * (target_exp - EXPOSURE))
    new_exp = int(min(max(new_exp, AE_MIN_EXP), AE_MAX_EXP))
    if new_exp <= AE_MIN_EXP and (mean > AE_TARGET or sat > 0.03) and GAIN > 0:
        GAIN = max(0, GAIN - 48)                    # exposure floored but still bright -> drop gain
        subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl", f"analogue_gain={GAIN}"], check=False)
    elif new_exp >= AE_MAX_EXP and mean < AE_TARGET * 0.6 and GAIN < 480:
        GAIN = min(480, GAIN + 48)                  # exposure maxed but dark -> add gain
        subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl", f"analogue_gain={GAIN}"], check=False)
    if new_exp != EXPOSURE:
        EXPOSURE = new_exp; _set_exposure(EXPOSURE)

def unpack_raw10(buf):
    """MIPI RAW10 packed -> uint16 Bayer (H, W). 4px in 5 bytes; b4 holds the 4x2 LSBs."""
    a = np.frombuffer(buf, np.uint8).reshape(H, STRIDE)[:, :W * 10 // 8].reshape(H, W // 4, 5)
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

def debayer(buf, full=True):
    """Packed RAW10 -> RGB uint8. GPU path unpacks on-device; CPU path unpacks first."""
    if GPU is not None and full:
        scale = _auto_scale_packed(buf)      # auto-exposure from the packed RAW10 (no CPU unpack)
        blk = BLACK_RGB if BLACK_RGB is not None else BLACK_LEVEL   # per-channel (R,G,B) or scalar
        if BIN:
            out = GPU.isp_bin(buf, STRIDE, blk, WB_R, WB_G, WB_B, scale, destripe=DESTRIPE)  # all on GPU
        else:
            out = GPU.isp(buf, STRIDE, blk, WB_R, WB_G, WB_B, scale, destripe=DESTRIPE)
        return out
    # CPU fallback: unpack + black+WB+demosaic + shade + destripe + tone map
    px = unpack_raw10(buf).astype(np.float32)
    if BLACK_RGB is not None:                 # per-channel black by Bayer position (BGGR)
        px[0::2, 0::2] -= BLACK_RGB[2]; px[1::2, 1::2] -= BLACK_RGB[0]
        px[0::2, 1::2] -= BLACK_RGB[1]; px[1::2, 0::2] -= BLACK_RGB[1]
    else:
        px -= BLACK_LEVEL
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
    # per-channel black: prefer a prior DARK-frame calibration (true pedestal) -> WB won't bake in a
    # brightness-dependent cast; else fall back to the single-min estimate.
    blk_prof = None
    if os.path.exists(PROFILE):
        _z = np.load(PROFILE)
        if "blk" in _z.files: blk_prof = [float(v) for v in _z["blk"]]
    if blk_prof is not None:
        bR, bG, bB = blk_prof
    else:
        b0 = float(min(px[0::2, 0::2].min(), px[1::2, 1::2].min(), px[0::2, 1::2].min())); bR = bG = bB = b0
    Rm = (px[1::2, 1::2] - bR).mean(); Bm = (px[0::2, 0::2] - bB).mean()
    Gm = ((px[0::2, 1::2] - bG).mean() + (px[1::2, 0::2] - bG).mean()) / 2
    wb = np.array([Gm / max(Rm, 1e-3), 1.0, Gm / max(Bm, 1e-3)], np.float32)
    # radial COLOR shading: WB-correct + demosaic the flat field, fit a smooth quadratic per channel
    # (robust to local non-uniformity), then a GREEN-RELATIVE gain that flattens R/G,B/G across the field
    # without touching luminance (green gain = 1 -> no vignette boost -> no corner-noise amplification).
    q = px.copy()                                          # per-channel black subtract (BGGR)
    q[1::2, 1::2] -= bR; q[0::2, 0::2] -= bB; q[0::2, 1::2] -= bG; q[1::2, 0::2] -= bG
    q = np.clip(q, 0, None); q[1::2, 1::2] *= wb[0]; q[0::2, 0::2] *= wb[2]
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
    _sv = dict(black=np.float32(min(bR, bG, bB)), wb=wb, shade=shade)
    if blk_prof is not None: _sv["blk"] = np.array(blk_prof, np.float32)   # preserve dark-frame black
    np.savez(PROFILE, **_sv)
    print(f"saved profile ({cnt} frames): blk=({bR:.0f},{bG:.0f},{bB:.0f}) wb=({wb[0]:.3f},1.000,{wb[2]:.3f}) "
          f"shade R[{gR.min():.2f}-{gR.max():.2f}] B[{gB.min():.2f}-{gB.max():.2f}] to {PROFILE}"
          + ("" if blk_prof else "  [no dark-frame black yet -> run calibrate-dark for the color-cast fix]"), flush=True)

def calibrate_dark(cam, exposure, gain, n=40):
    """Dark-frame black calibration: COVER THE LENS completely. Measures the true per-channel black
    pedestal (R,G,B) so the WB no longer amplifies a black-level error into a brightness-dependent color
    cast (green shadows / magenta highlights). Merges blk into the profile (keeps any wb + shade); then
    re-run the grey `--calibrate` so the WB is derived against these blacks."""
    import time
    setup_pipeline(cam, exposure, gain)
    print(f"DARK CALIBRATION: cover the lens COMPLETELY (no light). Capturing {n} frames...", flush=True)
    subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False); time.sleep(1)
    tmp = "/dev/shm/q6a_dark.raw"
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0",
                    "--set-fmt-video=width=1456,height=1088,pixelformat=pBAA",
                    "--stream-mmap", f"--stream-count={n}", f"--stream-to={tmp}"], check=True)
    acc = np.zeros((H, W), np.float64); cnt = 0
    with open(tmp, "rb") as f:
        for _ in range(n):
            b = f.read(FRAME)
            if len(b) < FRAME: break
            acc += unpack_raw10(b); cnt += 1
    px = acc / max(cnt, 1)
    blR = float(px[1::2, 1::2].mean()); blB = float(px[0::2, 0::2].mean())
    blG = float((px[0::2, 1::2].mean() + px[1::2, 0::2].mean()) / 2)
    blk = np.array([blR, blG, blB], np.float32)
    sv = dict(black=np.float32(min(blR, blG, blB)), blk=blk)
    if os.path.exists(PROFILE):                            # keep an existing WB / shading map
        z = np.load(PROFILE)
        if "wb" in z.files: sv["wb"] = z["wb"]
        if "shade" in z.files: sv["shade"] = z["shade"]
    np.savez(PROFILE, **sv)
    warn = "  WARNING: black>150 — was the lens actually covered?" if max(blR, blG, blB) > 150 else ""
    print(f"saved dark profile ({cnt} frames): blk R={blR:.1f} G={blG:.1f} B={blB:.1f} to {PROFILE}{warn}\n"
          f">>> Now re-run the grey calibration so WB is derived against these blacks: "
          f"./view_q6a_cam.sh calibrate {cam}", flush=True)

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
    jseq = 0                    # bumped per new frame (serve loop sends each unique frame once)
    lock = threading.Lock()
    clients = 0                 # active /stream viewers
    wake = threading.Event()    # signalled when a viewer connects
    rgb = None                  # latest full-res RGB frame (for the detector to consume)
    dets = []                   # latest YOLO detections (drawn onto every frame)

def process(buf, full):
    rgb = debayer(buf, full)                                # packed RAW10 in -> (H,W,3) uint8 (GPU unpacks)
    dets = None
    if DET is not None:
        DET["frame"][:] = rgb                              # publish latest frame to the detector (shm)
        DET["fseq"][0] += 1
        dets = _read_dets()                                # newest detections (lag ~1 inference)
    rgb = draw_overlay(rgb, dets)                          # returns a new array (PIL) if dets else same
    bio = BytesIO(); Image.fromarray(rgb).save(bio, "JPEG", quality=80)
    with State.lock:
        State.jpeg = bio.getvalue(); State.jseq += 1

def _read_dets():
    n = int(DET["dcnt"][0])
    if n <= 0:
        return None
    out = []
    for r in DET["dbuf"][:n]:
        ci = int(r[5]); lab = LABELS[ci] if 0 <= ci < len(LABELS) else str(ci)
        out.append((int(r[0]), int(r[1]), int(r[2]), int(r[3]), lab, float(r[4])))
    return out

def init_detector():
    """Create the shared-memory frame/dets buffers and spawn q6a_detector.py (NPU, separate process).
    The GPU ISP (this process) and the NPU YOLO (that process) then run concurrently with no lock."""
    global DET, LABELS
    import atexit
    from multiprocessing import shared_memory
    lp = os.path.expanduser("~/coco_labels.txt")
    if os.path.exists(lp):
        LABELS = [l.strip() for l in open(lp) if l.strip()]
    for nm in ("q6a_frame", "q6a_ctrl"):                    # clear any stale segments
        try: shared_memory.SharedMemory(name=nm).unlink()
        except FileNotFoundError: pass
    try:
        fshm = shared_memory.SharedMemory(name="q6a_frame", create=True, size=OUT_W * OUT_H * 3)
        cshm = shared_memory.SharedMemory(name="q6a_ctrl", create=True, size=CTRL_SIZE)
    except Exception as e:
        print(f"YOLO disabled (shm: {e})", flush=True); return
    DET = {"fshm": fshm, "cshm": cshm,
           "frame": np.ndarray((OUT_H, OUT_W, 3), np.uint8, buffer=fshm.buf),
           "fseq": np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=0),
           "dcnt": np.ndarray((1,), np.int32, buffer=cshm.buf, offset=16),
           "ow": np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=24),
           "oh": np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=26),
           "dbuf": np.ndarray((MAX_DET, 6), np.float32, buffer=cshm.buf, offset=CTRL_OFF)}
    DET["fseq"][0] = 0; DET["dcnt"][0] = 0
    DET["ow"][0] = OUT_W; DET["oh"][0] = OUT_H          # publish output dims for the detector
    proc = subprocess.Popen(["python3", os.path.expanduser("~/q6a_detector.py")],
                            env={**os.environ, "Q6A_DET_FPS": str(YOLO_FPS)})
    DET["proc"] = proc

    def _cleanup():
        try: proc.terminate()
        except Exception: pass
        for s in (fshm, cshm):
            try: s.close(); s.unlink()
            except Exception: pass
    atexit.register(_cleanup)
    print("YOLO detector spawned (separate process, no lock)", flush=True)

def capture_loop(rdi, full):
    """Prefer direct V4L2 mmap streaming (full sensor rate, ~23fps); fall back to the file-tail method."""
    import time
    subprocess.run(["pkill", "-9", "-f", "v4l2-ctl"], check=False)   # free the device
    try:
        from q6a_v4l2 import V4l2Cam
    except Exception as e:
        print(f"V4L2 mmap unavailable ({e}); using file-tail capture", flush=True)
        return _capture_loop_file(rdi, full)
    import time as _t
    cam = None; fails = 0; _hbn = 0; _hbt = _t.time()
    while True:
        if State.clients == 0:                 # no viewer -> release camera, idle
            State.jpeg = None
            if cam is not None: cam.close(); cam = None
            State.wake.wait(); State.wake.clear(); continue
        try:
            if cam is None:
                cam = V4l2Cam("/dev/video0", W, H); fails = 0
            data = cam.read_latest(timeout=1.0)  # drains to the freshest frame (low latency)
            if data is not None and len(data) == FRAME:
                auto_exposure(data)              # nudge sensor exposure/gain (real AE, no-op if --no-ae)
                process(data, full)
                _hbn += 1                        # heartbeat: server-side publish rate (server froze? -> 0)
                if _t.time() - _hbt >= 2.0:
                    print(f"[hb] publish {_hbn/(_t.time()-_hbt):.0f} fps jseq={State.jseq} clients={State.clients} jpeg={0 if State.jpeg is None else len(State.jpeg)//1024}KB", flush=True)
                    _hbn = 0; _hbt = _t.time()
        except Exception as e:
            print("capture error (v4l2):", e, flush=True)
            if cam is not None:
                try: cam.close()
                except Exception: pass
                cam = None
            fails += 1
            if fails >= 3:
                print("V4L2 capture failing; falling back to file-tail", flush=True)
                return _capture_loop_file(rdi, full)
            time.sleep(0.5)

def _capture_loop_file(rdi, full, batch=300):
    """Fallback: v4l2-ctl --stream-count=0-to-pipe HANGS this CAMSS driver, and a
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
        import time, socket
        # Low latency: a small send buffer + no Nagle means wfile.write() blocks after ~1 frame instead of
        # letting the kernel queue SECONDS of frames ahead of a slow client. The serve loop below then reads
        # the LATEST frame after each write (send-on-new), so it drops stale frames -> latency ~1 frame, not 3-5s.
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 96 * 1024)
        except Exception:
            pass
        with State.lock:
            State.clients += 1
        State.wake.set()                       # wake the capture loop
        last = -1
        try:
            while True:
                with State.lock:
                    j = State.jpeg; s = State.jseq
                if j is not None and s != last:        # send each unique frame once, promptly
                    last = s
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(j) + j + b"\r\n")
                else:
                    time.sleep(0.005)
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
    ap.add_argument("--gain", type=int, default=240, help="sensor gain, ctrl 0..480 = 0..48dB (0.1dB/step). Per the IMX296 datasheet ONLY 0..240 (0..24dB) is ANALOG; 240..480 adds DIGITAL gain. Analog is the only stage that lowers input-referred read noise, so 240 (max analog) is the cleanest value + keeps full highlight range; the ISP tone-map handles brightness. Raise toward 480 (digital) or bump --exposure only for genuinely dark scenes.")
    ap.add_argument("--calibrate", action="store_true", help="capture a flat-field color profile (aim at a white/gray surface)")
    ap.add_argument("--calibrate-dark", action="store_true", help="dark-frame black calibration (COVER THE LENS): measures per-channel black to fix the brightness-dependent color cast; then re-run --calibrate")
    ap.add_argument("--no-yolo", action="store_true", help="disable the NPU YOLO detection overlay")
    ap.add_argument("--no-ae", action="store_true", help="disable sensor auto-exposure (use the fixed --exposure/--gain). AE nudges real integration time so bright scenes don't clip; --exposure is just the starting point.")
    ap.add_argument("--camera-model", default="imx296", help="tuning/<model>.json to load the ready-made color-correction matrix (CCM) from (default imx296)")
    ap.add_argument("--ccm-ct", type=int, default=3600, help="colour temperature (K) to interpolate the CCM to (2500=warm .. 7400=daylight; 3600 indoor default)")
    ap.add_argument("--no-ccm", action="store_true", help="disable the ready-made color-correction matrix")
    ap.add_argument("--yolo-fps", type=float, default=10.0, help="cap YOLO to this detection rate (0=unlimited, ~26fps NPU max). Detections overlay persists between updates, so a low rate saves NPU heat/power with no visual loss on a slow robot.")
    ap.add_argument("--gpu", action="store_true", help="full-res ISP on the Adreno GPU (OpenCL) instead of CPU")
    ap.add_argument("--destripe", action="store_true", help="also remove FPN column/row banding (CPU, ~32ms)")
    ap.add_argument("--bin", action="store_true", help="GPU digital 2x2 binning: half-res (728x544), ~2x less noise + faster")
    ap.add_argument("--sensor-bin", action="store_true", help="[EXPERIMENTAL / NON-FUNCTIONAL] 2x2 binning on the IMX296 (charge-domain FD binning -> 728x544). The imx296 MIPIC_AREA3W patch stops the STREAMON hang but mainline qcom-camss still delivers EMPTY (0xFF) buffers - the binned pixel payload never lands (no kernel error). Use --bin (GPU digital) instead; it gives the same 728x544 and is faster.")
    args = ap.parse_args()
    DESTRIPE = args.destripe
    YOLO_FPS = args.yolo_fps
    AE_ON = not args.no_ae
    CAMERA_MODEL = args.camera_model; CCM_CT = args.ccm_ct; CCM_ON = not args.no_ccm
    BIN = args.bin
    if BIN:
        OUT_W, OUT_H = W // 2, H // 2
    if args.calibrate_dark:
        calibrate_dark(args.cam, args.exposure, args.gain)
        sys.exit(0)
    if args.calibrate:
        calibrate(args.cam, args.exposure, args.gain)   # always full-res 1456x1088 (WB gains are mode-independent)
        sys.exit(0)
    if args.sensor_bin:
        # The sensor emits a 2x2-binned 728x544 Bayer frame; capture at that size and let the GPU do a
        # plain demosaic (no GPU bin). Rebind the capture geometry BEFORE init_gpu/setup_pipeline read it.
        SENSOR_BIN = True; BIN = False
        W, H = 728, 544
        STRIDE = 912                              # pBAA: 728*10/8=910, padded to 912 (v4l2 sizeimage 496128)
        FRAME = STRIDE * H
        OUT_W, OUT_H = W, H
        print("sensor 2x2 FD binning: capture 728x544 (charge-domain, cleaner) -> GPU plain demosaic", flush=True)
    load_profile()
    if args.gpu:
        init_gpu()
        if GPU is None:
            args.fast = True         # GPU unavailable -> half-res CPU (fast) instead of slow full-res CPU
    print("start: setting up pipeline...", flush=True)
    rdi = setup_pipeline(args.cam, args.exposure, args.gain)
    print(f"pipeline ready ({rdi}); starting capture + server", flush=True)
    if not args.no_yolo:
        init_detector()                        # spawn q6a_detector.py (NPU) + set up shared memory
    threading.Thread(target=capture_loop, args=(rdi, not args.fast), daemon=True).start()
    print(f"MJPEG stream on http://0.0.0.0:{args.port}/  (cam{args.cam})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()

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

# --- Camera geometry/format: DEFAULTS ONLY. All sensor-specifics come from profiles/<model>.json at
# startup (load_camera_profile), so these scripts are camera-agnostic. Defaults keep the module importable. ---
W, H = 1456, 1088
STRIDE = 1824            # bytes/line for packed RAW10 (align8(W*bits/8)); recomputed from the profile
FRAME = STRIDE * H       # bytes/frame (v4l2 sizeimage)
OUT_W, OUT_H = W, H      # output (displayed/detected) resolution; halved by --bin
BAYER = "RGGB"           # CFA order (top-left 2x2, row-major); set by the profile. Our IMX296 delivers RGGB
                         # through this CAMSS pipeline (verified with a colour chart: BGGR gave a red<->blue swap)
RX, RY, BX, BY = 0, 0, 1, 1   # R and B pixel positions within the 2x2, derived from BAYER
MBUS_CODE = "SBGGR10_1X10"    # sensor media-bus code for the media-ctl link (CFA-agnostic for RAW passthrough)
PIXFMT = "pBAA"          # V4L2 capture pixelformat (packed RAW10)
ENTITY_MATCH = "imx296"  # substring to find the sensor subdev entity in the media graph
GAIN_MAX = 480           # analogue_gain control max (ctrl units)
GAIN_ANALOG_MAX = 240    # ctrl units that are still ANALOG gain (0..240 = 0..24dB); 240..480 is DIGITAL
                         # (adds noise, no SNR gain) and shifts the calibration operating point -> AE caps here
CCM_MATRICES = None      # [(ct, [9 floats]), ...] from the profile; interpolated to CCM_CT

# --- Color pipeline: deterministic, measured from raw Bayer statistics (NOT per-frame guessing) ---
# A raw Bayer sensor needs black-level subtraction + fixed white-balance gains. On this IMX296 the two
# green phases read ~1.6x red/blue (CFA + sensor QE peak in green) -> without WB everything looks green.
# Measured from raw: black~56, R/G=B/G~0.62 globally and SCENE-INDEPENDENT, so a single fixed gain per
# channel neutralises it everywhere (no fragile per-pixel flat-field). Re-derive on a grey card via
# --calibrate; it writes the 4 numbers below to PROFILE, which load_profile() then uses to override.
BLACK_LEVEL = 56.0
BLACK_RGB = None                              # per-channel black pedestal (R,G,B) from a dark-frame calibration;
                                              # None -> use the single BLACK_LEVEL for all channels
WB_R, WB_G, WB_B = 1.60, 1.00, 1.52          # raw per-channel gains -> neutral grey (AWB updates these at runtime)
SHADE = None                                  # optional (H,W,3) radial colour-shading gain from --calibrate
GPU = None                                    # optional Adreno OpenCL ISP (q6a_gpu.GpuDemosaic)
CAMERA_MODEL = "imx296"                        # profiles/<model>.json -> all sensor config (add a camera = add a file)
CCM_CT = 3600                                  # colour temp (K) to interpolate the CCM to (indoor default)
CCM_ON = False                                 # ready-made CCM (RPi IMX296 tuning), opt-in via --ccm. OFF by
                                               # default: its aggressive green row crushes G toward magenta when
                                               # WB gains are high, and it amplifies low-light chroma noise.
DESTRIPE = False                              # optional FPN band removal; off by default
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
AE_MIN_EXP = 30; AE_MAX_EXP = 3000            # exposure clamp (lines); VMAX fixed here -> ~32fps, no vblank churn; cap keeps fps >=~24 + out of the deep-noise regime
# --- Auto white balance (damped, constrained gray-world) --- tracks the illuminant (day<->evening) so
# adjusts the WB gains (software, GPU reads them next frame -> no glitch). ANCHORED gray-world: opt-in via
# --awb. Unbounded gray-world runs away to magenta on a non-gray room (observed 1.6->3.2/1.5->4.1); this
# version is CLAMPED to +-AWB_ANCHOR_MARGIN around the loaded calibration WB, so it can only make small
# corrections (tracks the sensor's thermal drift + modest light changes) and PHYSICALLY cannot reach magenta.
AWB_ON = False
AWB_ALPHA = 0.05                              # WB gain smoothing per update (slow, flicker-free)
AWB_ANCHOR_MARGIN = 0.10                      # AWB may move each gain only +-10% from the calibration anchor
                                              # (covers the ~8% thermal drift; tight enough to stay near neutral)
_AWB_ANCHOR_R = _AWB_ANCHOR_B = None          # captured from the loaded WB on the first AWB update
AWB_RMIN, AWB_RMAX = 1.2, 3.2                 # (legacy absolute range; the anchor clamp below is tighter)
AWB_BMIN, AWB_BMAX = 1.2, 4.2
# YOLO runs in a SEPARATE PROCESS (q6a_detector.py) sharing frames via shared memory. The Adreno (GPU
# ISP) and Hexagon (NPU) crash if driven concurrently in ONE process (shared userspace allocator
# corruption) but run fine across processes -> no lock, true concurrency. shm layout <-> q6a_detector.py.
MAX_DET = 32; CTRL_OFF = 32; CTRL_SIZE = CTRL_OFF + MAX_DET * 6 * 4
DET = None                                     # dict of shm views + the detector subprocess (None if disabled)
YOLO_FPS = 10                                   # cap NPU YOLO to this rate (0=unlimited ~26fps). A slow robot
                                               # doesn't need per-frame detection; boxes persist between updates,
                                               # so 10fps frees the NPU (~60% idle -> cooler, shares HTP w/ the LLM).
LABELS = [str(i) for i in range(80)]
PROFILES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")

def _bayer_pos(s):
    """CFA string (row-major 2x2, e.g. 'BGGR') -> (rx,ry,bx,by): R and B pixel offsets within the 2x2."""
    r, b = s.index("R"), s.index("B")
    return (r % 2, r // 2, b % 2, b // 2)

def _fourcc(s):
    """V4L2 fourcc string (e.g. 'pBAA') -> its 32-bit int code."""
    return sum(ord(c) << (8 * i) for i, c in enumerate(s[:4]))

def load_camera_profile(model):
    """Load ALL sensor-specific config from profiles/<model>.json into module globals -> the scripts stay
    camera-agnostic (adding a camera = adding a profile: geometry, CFA, MIPI format, colour defaults, AE
    bounds, and the ready-made CCM). Returns True if a profile was found."""
    global W, H, STRIDE, FRAME, OUT_W, OUT_H, BAYER, RX, RY, BX, BY, MBUS_CODE, PIXFMT, ENTITY_MATCH
    global BLACK_LEVEL, WB_R, WB_G, WB_B, AE_TARGET, AE_MIN_EXP, AE_MAX_EXP, GAIN_MAX, GAIN_ANALOG_MAX, CCM_MATRICES, CCM_CT
    global AWB_RMIN, AWB_RMAX, AWB_BMIN, AWB_BMAX
    path = os.path.join(PROFILES_DIR, f"{model}.json")
    if not os.path.exists(path):
        print(f"no camera profile {path}; using built-in defaults ({W}x{H} {BAYER})", flush=True); return False
    p = json.load(open(path)); s = p["sensor"]
    W, H = int(s["width"]), int(s["height"])
    BAYER = s.get("bayer", BAYER); RX, RY, BX, BY = _bayer_pos(BAYER)
    MBUS_CODE = s.get("mbus_code", MBUS_CODE); PIXFMT = s.get("v4l2_pixelformat", PIXFMT)
    ENTITY_MATCH = s.get("entity_match", ENTITY_MATCH)
    bits = int(s.get("bits", 10))
    STRIDE = ((W * bits // 8 + 7) // 8) * 8       # packed RAW10 stride, aligned up to 8 bytes (as v4l2 pads)
    FRAME = STRIDE * H; OUT_W, OUT_H = W, H
    cd = p.get("color_defaults", {})
    BLACK_LEVEL = float(cd.get("black_level", BLACK_LEVEL))
    if "wb_rgb" in cd: WB_R, WB_G, WB_B = (float(x) for x in cd["wb_rgb"])
    ae = p.get("ae", {})
    AE_TARGET = float(ae.get("target", AE_TARGET)); AE_MIN_EXP = int(ae.get("min_exposure", AE_MIN_EXP))
    AE_MAX_EXP = int(ae.get("max_exposure", AE_MAX_EXP)); GAIN_MAX = int(ae.get("gain_max", GAIN_MAX))
    GAIN_ANALOG_MAX = int(ae.get("gain_analog_max", GAIN_ANALOG_MAX))
    awb = p.get("awb", {})
    if "r_gain" in awb: AWB_RMIN, AWB_RMAX = (float(x) for x in awb["r_gain"])
    if "b_gain" in awb: AWB_BMIN, AWB_BMAX = (float(x) for x in awb["b_gain"])
    ccm = p.get("ccm", {})
    if ccm.get("matrices"):
        CCM_MATRICES = sorted(((int(m["ct"]), list(m["ccm"])) for m in ccm["matrices"]), key=lambda e: e[0])
        CCM_CT = int(ccm.get("ct", CCM_CT))
    print(f"camera profile {model}: {W}x{H} {BAYER} {MBUS_CODE}/{PIXFMT} stride={STRIDE} black={BLACK_LEVEL:.0f} "
          f"ae[{AE_MIN_EXP}-{AE_MAX_EXP}]@{AE_TARGET:.0f} ccm={'yes' if CCM_MATRICES else 'no'}", flush=True)
    return True

def load_ccm(ct):
    """Interpolate the profile's ready-made CCM matrices to colour temperature `ct` -> (3,3) float32 (or None).
    The cross-channel colour science (fixes the residual spectral cast); lab-derived, not per-unit guessing."""
    if not CCM_MATRICES:
        return None
    cts = [c for c, _ in CCM_MATRICES]
    if ct <= cts[0]: m = CCM_MATRICES[0][1]
    elif ct >= cts[-1]: m = CCM_MATRICES[-1][1]
    else:
        i = max(k for k in range(len(cts)) if cts[k] <= ct)
        f = (ct - cts[i]) / (cts[i + 1] - cts[i])
        m = [a + (b - a) * f for a, b in zip(CCM_MATRICES[i][1], CCM_MATRICES[i + 1][1])]
    print(f"CCM @ {ct}K (of {len(cts)} CTs {cts[0]}-{cts[-1]})", flush=True)
    return np.asarray(m, np.float32).reshape(3, 3)

def init_gpu():
    global GPU
    try:
        from q6a_gpu import GpuDemosaic
        GPU = GpuDemosaic(W, H, bayer=(RX, RY, BX, BY))
        GPU.set_shade(SHADE)                   # upload color-shading map once (if a profile is loaded)
        GPU.set_ccm(load_ccm(CCM_CT) if CCM_ON else None)   # ready-made color matrix (profile)
        stages = "demosaic+WB" + ("+AWB" if AWB_ON else "") + ("+CCM" if CCM_ON else "") \
            + ("+shade" if SHADE is not None else "") + "+tonemap"
        print(f"GPU ISP enabled: {GPU.dev_name} ({stages} on GPU)", flush=True)
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
        if "shade" in z.files:                 # radial colour-shading gain grid (from --calibrate) -> full res
            s = np.asarray(z["shade"], np.float32)
            SHADE = np.stack([np.asarray(Image.fromarray(s[..., c]).resize((W, H), Image.BILINEAR))
                              for c in range(3)], axis=2).astype(np.float32)
            msg += f" + shading map {z['shade'].shape[:2]}"
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
    fmt = f"[fmt:{MBUS_CODE}/{W}x{H}]"    # MBUS_CODE + W,H from the camera profile
    # sensor entity name = e.g. "imx296 <cci-bus>-001a"; discover it from a link line in the topology
    top = subprocess.run(m + ["-p"], capture_output=True, text=True).stdout
    sensor = next((l.split('"')[1] for l in top.splitlines() if ENTITY_MATCH in l and '"' in l), None)
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
            # Fix VMAX for the AE ceiling so AE only ever changes exposure (no mid-stream vblank churn ->
            # no per-frame timing glitch / motion 'snow'). fps is then fixed at this VMAX rate (~32).
            vb_exp = AE_MAX_EXP if AE_ON else exposure
            vblank = max(30, vb_exp - H + 64)
            for c, v in [("exposure", 100), ("vertical_blanking", vblank), ("exposure", exposure),
                         ("analogue_gain", gain)]:
                subprocess.run(["v4l2-ctl", "-d", sd, "--set-ctrl", f"{c}={v}"], check=False)
    return rdi

def _set_exposure(exp):
    """Set ONLY the sensor integration time (SHS1 via the exposure control). VMAX/vblank is fixed once at
    setup (for AE_MAX_EXP), so exposure changes are clean per-frame latches. Changing vblank (frame length)
    mid-stream reconfigures the sensor timing and glitches one frame -> full-screen 'snow' on movement
    (motion -> AE adjusts frequently). Exposure-only avoids that; fps is then fixed at the VMAX rate."""
    if SENSOR_SD is None:
        return
    subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl", f"exposure={exp}"], check=False)

_AE_N = 0
def auto_exposure(buf):
    """Real sensor AE from the packed RAW10 high bytes (cheap, no unpack). Every ~8 frames, nudge the
    sensor exposure toward a target MEDIAN brightness (robust to a dark surround or a bright window),
    then gain only at the exposure floor/ceiling. Deadband + partial-step damping avoid hunting."""
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
    if new_exp <= AE_MIN_EXP and mid > AE_TARGET and GAIN > 0:
        GAIN = max(0, GAIN - 48)                    # exposure floored but still bright -> drop gain
        subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl", f"analogue_gain={GAIN}"], check=False)
    elif new_exp >= AE_MAX_EXP and mid < AE_TARGET * 0.6 and GAIN < GAIN_ANALOG_MAX:
        GAIN = min(GAIN_ANALOG_MAX, GAIN + 48)      # exposure maxed but dark -> add gain, but only up to the
                                                    # ANALOG max (240): digital gain adds noise + row-lines and
                                                    # shifts the calibration operating point (green drift). The
                                                    # tone-map lifts dim scenes instead.
        subprocess.run(["v4l2-ctl", "-d", SENSOR_SD, "--set-ctrl", f"analogue_gain={GAIN}"], check=False)
    if new_exp != EXPOSURE:
        EXPOSURE = new_exp; _set_exposure(EXPOSURE)

_AWB_N = 0
def auto_wb(buf):
    """ANCHORED gray-world AWB from the packed RAW10 high bytes. Every ~8 frames, estimate the per-Bayer-channel
    MEDIANS (robust to bright/coloured spots), take the gray-world WB that equalises them, and move WB_R/WB_B
    a small fraction toward it -- but CLAMPED to +-AWB_ANCHOR_MARGIN around the loaded calibration WB. This
    tracks the sensor's thermal drift + modest light changes yet physically cannot run away to magenta (which
    unbounded gray-world does on a non-gray room). WB is a software gain the GPU reads each frame -> no glitch."""
    global WB_R, WB_B, _AWB_N, _AWB_ANCHOR_R, _AWB_ANCHOR_B
    if not AWB_ON:
        return
    if _AWB_ANCHOR_R is None:                       # anchor to the calibrated WB loaded at startup
        _AWB_ANCHOR_R, _AWB_ANCHOR_B = WB_R, WB_B
    _AWB_N += 1
    if _AWB_N % 8:
        return
    a = np.frombuffer(buf, np.uint8)[:STRIDE * H].reshape(H, STRIDE)[:, :W * 10 // 8].reshape(H, W // 4, 5)
    h = a[:, ::4, :4].astype(np.float32) * 4.0     # subsample groups -> ~10-bit level per pixel (high byte<<2)
    bR = BLACK_RGB[0] if BLACK_RGB else BLACK_LEVEL
    bG = BLACK_RGB[1] if BLACK_RGB else BLACK_LEVEL
    bB = BLACK_RGB[2] if BLACK_RGB else BLACK_LEVEL
    Rm = float(np.median(h[RY::2][:, :, RX::2])) - bR       # per-channel medians by CFA position
    Bm = float(np.median(h[BY::2][:, :, BX::2])) - bB
    Gm = (float(np.median(h[BY::2][:, :, RX::2])) + float(np.median(h[RY::2][:, :, BX::2]))) / 2 - bG
    if Gm < 8 or Rm < 4 or Bm < 4:                 # too dark to estimate WB reliably -> hold
        return
    m = AWB_ANCHOR_MARGIN                           # clamp targets to +-m around the calibration anchor
    tR = min(max(Gm / Rm, _AWB_ANCHOR_R * (1 - m)), _AWB_ANCHOR_R * (1 + m))
    tB = min(max(Gm / Bm, _AWB_ANCHOR_B * (1 - m)), _AWB_ANCHOR_B * (1 + m))
    WB_R += AWB_ALPHA * (tR - WB_R)                # slow, flicker-free (no fast-lock: starts at the calibration)
    WB_B += AWB_ALPHA * (tB - WB_B)

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
    """Full-resolution bilinear demosaic of a Bayer plane -> (H,W,3) float. Bayer-order-agnostic: the R/G/B
    samples are placed by the profile's CFA positions (RX/RY/BX/BY), then interpolated (name kept for compat)."""
    px = px.astype(np.float32)
    R = np.zeros_like(px); G = np.zeros_like(px); B = np.zeros_like(px)
    B[BY::2, BX::2] = px[BY::2, BX::2]
    G[BY::2, RX::2] = px[BY::2, RX::2]; G[RY::2, BX::2] = px[RY::2, BX::2]
    R[RY::2, RX::2] = px[RY::2, RX::2]
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
    if BLACK_RGB is not None:                 # per-channel black by Bayer position (profile CFA)
        px[BY::2, BX::2] -= BLACK_RGB[2]; px[RY::2, RX::2] -= BLACK_RGB[0]
        px[BY::2, RX::2] -= BLACK_RGB[1]; px[RY::2, BX::2] -= BLACK_RGB[1]
    else:
        px -= BLACK_LEVEL
    np.clip(px, 0, None, out=px)
    px[RY::2, RX::2] *= WB_R                  # R sites   (raw white balance, per Bayer position)
    px[BY::2, BX::2] *= WB_B                  # B sites   (G sites keep WB_G=1)
    out = _debayer_cpu(px, full)
    if SHADE is not None and SHADE.shape == out.shape:
        out *= SHADE                          # radial color-shading correction (from --calibrate)
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
    b = px[BY::2, BX::2]; g = (px[BY::2, RX::2] + px[RY::2, BX::2]) * 0.5; r = px[RY::2, RX::2]
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
                    f"--set-fmt-video=width={W},height={H},pixelformat={PIXFMT}",
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
        b0 = float(min(px[RY::2, RX::2].min(), px[BY::2, BX::2].min(), px[BY::2, RX::2].min())); bR = bG = bB = b0
    Rm = (px[RY::2, RX::2] - bR).mean(); Bm = (px[BY::2, BX::2] - bB).mean()
    Gm = ((px[BY::2, RX::2] - bG).mean() + (px[RY::2, BX::2] - bG).mean()) / 2
    wb = np.array([Gm / max(Rm, 1e-3), 1.0, Gm / max(Bm, 1e-3)], np.float32)
    # radial COLOR shading: WB-correct + demosaic the flat field, fit a smooth quadratic per channel
    # (robust to local non-uniformity), then a GREEN-RELATIVE gain that flattens R/G,B/G across the field
    # without touching luminance (green gain = 1 -> no vignette boost -> no corner-noise amplification).
    q = px.copy()                                          # per-channel black subtract (BGGR)
    q[RY::2, RX::2] -= bR; q[BY::2, BX::2] -= bB; q[BY::2, RX::2] -= bG; q[RY::2, BX::2] -= bG
    q = np.clip(q, 0, None); q[RY::2, RX::2] *= wb[0]; q[BY::2, BX::2] *= wb[2]
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
                    f"--set-fmt-video=width={W},height={H},pixelformat={PIXFMT}",
                    "--stream-mmap", f"--stream-count={n}", f"--stream-to={tmp}"], check=True)
    acc = np.zeros((H, W), np.float64); cnt = 0
    with open(tmp, "rb") as f:
        for _ in range(n):
            b = f.read(FRAME)
            if len(b) < FRAME: break
            acc += unpack_raw10(b); cnt += 1
    px = acc / max(cnt, 1)
    blR = float(px[RY::2, RX::2].mean()); blB = float(px[BY::2, BX::2].mean())
    blG = float((px[BY::2, RX::2].mean() + px[RY::2, BX::2].mean()) / 2)
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
                cam = V4l2Cam("/dev/video0", W, H, pixelformat=_fourcc(PIXFMT)); fails = 0
            data = cam.read_latest(timeout=1.0)  # drains to the freshest frame (low latency)
            if data is not None and len(data) == FRAME:
                try:
                    auto_exposure(data)          # nudge sensor exposure/gain (real AE, no-op if --no-ae)
                    auto_wb(data)                # track the illuminant (damped gray-world, no-op if --no-awb)
                except Exception as e:
                    print("AE/AWB error (non-fatal):", e, flush=True)  # never crash the capture -> reinit
                process(data, full)
                _hbn += 1                        # heartbeat: server-side publish rate (server froze? -> 0)
                if _t.time() - _hbt >= 2.0:
                    print(f"[hb] publish {_hbn/(_t.time()-_hbt):.0f} fps jseq={State.jseq} clients={State.clients} jpeg={0 if State.jpeg is None else len(State.jpeg)//1024}KB exp={EXPOSURE} wb=({WB_R:.2f},{WB_B:.2f})", flush=True)
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
                 f"--set-fmt-video=width={W},height={H},pixelformat={PIXFMT}",
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
    ap.add_argument("--awb", action="store_true", help="enable ANCHORED auto white balance: gray-world clamped +-15%% around the calibrated WB, so it tracks the sensor's thermal drift + modest light changes but cannot run away to magenta. OFF by default (bare runs use fixed WB); view_q6a_cam.sh enables it.")
    ap.add_argument("--camera-model", default="imx296", help="profiles/<model>.json to load all sensor config from (geometry, CFA, format, defaults, AE, CCM). Add a camera = add a profile.")
    ap.add_argument("--ccm-ct", type=int, default=None, help="override the profile's CCM colour temperature (K) (2500=warm .. 7400=daylight)")
    ap.add_argument("--ccm", action="store_true", help="enable the ready-made color-correction matrix (RPi IMX296 tuning) (OFF by default: crushes green toward magenta with high WB gains + amplifies low-light noise)")
    ap.add_argument("--yolo-fps", type=float, default=10.0, help="cap YOLO to this detection rate (0=unlimited, ~26fps NPU max). Detections overlay persists between updates, so a low rate saves NPU heat/power with no visual loss on a slow robot.")
    ap.add_argument("--gpu", action="store_true", help="full-res ISP on the Adreno GPU (OpenCL) instead of CPU")
    ap.add_argument("--destripe", action="store_true", help="also remove static FPN column/row banding (luminance-only, fused)")
    ap.add_argument("--bin", action="store_true", help="GPU digital 2x2 binning: half-res (728x544), ~2x less noise + faster")
    ap.add_argument("--sensor-bin", action="store_true", help="[EXPERIMENTAL / NON-FUNCTIONAL] 2x2 binning on the IMX296 (charge-domain FD binning -> 728x544). The imx296 MIPIC_AREA3W patch stops the STREAMON hang but mainline qcom-camss still delivers EMPTY (0xFF) buffers - the binned pixel payload never lands (no kernel error). Use --bin (GPU digital) instead; it gives the same 728x544 and is faster.")
    args = ap.parse_args()
    DESTRIPE = args.destripe
    YOLO_FPS = args.yolo_fps
    AE_ON = not args.no_ae; AWB_ON = args.awb
    CAMERA_MODEL = args.camera_model; CCM_ON = args.ccm
    load_camera_profile(CAMERA_MODEL)              # sets geometry/format/defaults/AE/CCM from profiles/<model>.json
    if args.ccm_ct is not None: CCM_CT = args.ccm_ct   # CLI colour-temp overrides the profile default
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

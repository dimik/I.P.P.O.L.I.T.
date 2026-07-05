# I.P.P.O.L.I.T. — Q6A Vision Stack & Video Pipeline (review brief)

A self-contained technical brief of the on-device camera + AI-vision stack, written for a reviewer to
evaluate the design decisions and their rationale. Primary source with full history: `q6a-camera-pipeline.md`
(§1–14); sensor/overlay bring-up: `q6a-camera.md`.

## 1. Goal & context
Add a camera + on-device AI vision to a robot-vacuum companion board. A Sony **IMX296** global-shutter
camera feeds live object detection (YOLOv8 / COCO) and a human-viewable MJPEG stream, all computed
**on-device** (no cloud). The board is passively cooled inside the robot's top compartment.

## 2. Hardware
- **Compute:** Radxa Dragon Q6A — Qualcomm **QCS6490**: 8-core Kryo CPU, **Adreno 643** GPU (OpenCL/Vulkan),
  **Hexagon v68** NPU (~12 TOPS). 16 GB LPDDR5, passively cooled.
- **Sensor:** Sony **IMX296** — 1.58 MP **global shutter**, colour Bayer (**RGGB**, see §6), 1456×1088,
  **1-lane MIPI CSI-2, RAW10 only** (no on-sensor ISP — it is a machine-vision sensor).
- **Host/dev:** Seeed Odyssey X86 — the viewing/dev machine, wired point-to-point to the Q6A
  (`192.168.20.0/24`, `ssh ippolit-lan`).
- **OS:** Ubuntu 24.04 (Radxa `rsdk-r2`), `qcom` kernel 6.18, headless (auto-suspend masked).

## 3. Why the whole ISP runs in userspace (central constraint)
- The QCS6490 has a Spectra ISP + Venus encoder, but on **mainline `qcom-camss` the camera path is RDI-only**
  — it delivers **packed raw Bayer** (`pBAA`, 10-bit) and nothing more. The Titan ISP pixel pipeline (HW
  demosaic) is driven by an undocumented embedded-CPU firmware stream, out of mainline scope. Verified: the
  `vfe_pix` pads enumerate but `STREAMON` hangs / yields 0 frames.
- **CAMX** (vendor ISP path) needs a proprietary CHI-CDK IMX296 sensor bring-up (gated) and pulls a
  conflicting fastrpc; the **Venus HW encoder hard-reboots this board** (TrustZone PIL gate). Both ruled out.
- So: **capture raw → demosaic + ISP + encode in userspace**, exactly as the kernel maintainers prescribe.
  GPU for the ISP, NPU for detection, CPU for glue.

## 4. Pipeline architecture (two processes, no lock)
```
IMX296 ─MIPI─► CAMSS RDI ─► /dev/video0 (mplane, pBAA packed RAW10)
      │
      ▼  PROCESS A  q6a_camstream.py  (Adreno GPU only)
  V4l2Cam.read_latest()  ── V4L2 MMAP, drain-to-freshest frame (low latency)
     → GpuDemosaic.isp()/isp_bin()  ── ONE OpenCL kernel: unpack RAW10 +
        black + WB + [AWB] + [CCM] + [shade] + demosaic + tonemap → uint8 RGB
     → write RGB to shm "q6a_frame" (seqlock)  ───────────────┐
     → [headless: stop here]                                  │
     → draw YOLO overlay (PIL) → JPEG (libjpeg-turbo)         │
     → ThreadingHTTPServer: multipart/x-mixed-replace MJPEG   │
                                                              │
      ▼  PROCESS B  q6a_detector.py  (Hexagon NPU only)       │
  read latest frame from shm (seqlock snapshot) ◄────────────┘
     → YoloDetector.infer()  (QNN v68 context binary, qai_appbuilder)
     → write detections to shm "q6a_ctrl"  ──► Process A reads for overlay
```

**Key decisions to weigh:**
- **Two processes, not two threads.** Driving Adreno (OpenCL) and Hexagon (QNN/fastrpc) **concurrently in one
  process segfaults** (shared userspace dma-heap/rpcmem allocator corruption; the board survives). Separate
  processes run fine → true GPU∥NPU concurrency. Frames cross via `multiprocessing.shared_memory` with a
  **seqlock** (write seq → copy → re-read seq; retry on mismatch): single writer, lock-free, no torn frames.
  This replaced an in-process accel lock that cost ~25%.
- **Drain-to-latest capture.** Custom V4L2 multiplanar MMAP loop (`q6a_v4l2.py`, linuxpy raw ioctls — the node
  is mplane and `v4l2-ctl` piping to stdout *hangs* this driver). Each read drains all queued buffers and
  returns only the freshest → low latency; drops frames it can't keep up with instead of building backlog.
- **Client-gating.** In server mode the capture loop idles when no HTTP viewer is connected (zero cost).
  `--headless` bypasses that and runs unconditionally (the detector is the consumer).
- **MJPEG, not H.264.** No usable HW encoder on mainline (Venus reboots the board); libjpeg-turbo (~3 ms) is
  faster end-to-end than SW x264 at this resolution, and the stream is on a LAN. The server caps `SO_SNDBUF`
  to ~1 frame + `TCP_NODELAY` so it drops stale frames instead of accumulating seconds of latency for a slow
  client.

## 5. GPU ISP (the throughput win)
The entire ISP is one Adreno OpenCL kernel operating **directly on the packed RAW10** (unpacks on-device — no
CPU unpack). Two variants: `isp` (full-res bilinear demosaic) and `isp_bin` (2×2 bin → 728×544, ~2× less
noise, lighter). It is **latency-bound** (~10 ms/frame; the Adreno power-gates between frames) not
compute-bound (actual pipelined compute ~3.4 ms). The CPU only JPEGs + serves. This took the pipeline from
**~3 fps / 79 °C (pure numpy) → ~16–19 fps** with YOLO.

**`PYOPENCL_NO_CACHE=1` is mandatory** — the pyopencl kernel cache silently served *stale* compiled binaries
across source edits (a real trap: kernel changes had no effect until the cache was bypassed).

## 6. Colour / ISP stages — all profile-driven (`profiles/<model>.json`)
The scripts (`q6a_camstream.py`, `q6a_gpu.py`, `q6a_v4l2.py`) are **camera-agnostic**; every sensor/lens/
tuning parameter lives in the profile. Sections: `sensor` (geometry, CFA, format), `color_defaults` + `ccm`
(spectral), `ae`, `awb`, `tonemap`, `shadow_tint` (sensor), `shading` (lens); plus a per-unit
`imx296_wb.npz` calibration (black / WB / shade) that overrides the profile defaults.

Stages, in order: **unpack → per-channel black → white balance → [white-patch AWB] → [CCM] → [radial shade]
→ tonemap (gamma) → [shadow green comp]**.

Durable findings (all worth review):
- **CFA is RGGB, not BGGR.** The profile initially declared BGGR; the sensor delivers RGGB through this
  pipeline → a **red↔blue channel swap** that caused ~2 days of "colour casts" no WB/CCM could fix.
  Diagnosed with a colour test chart (yellow→cyan, magenta-stable = R↔B). One-line fix.
- **AWB = white-patch, not gray-world.** Gray-world balances the *whole-scene average*, so a room with warm
  content (wood/furniture) tints the neutral walls **green** (the complement). White-patch references the
  **brightest non-clipped blocks** (walls/ceiling ≈ white), selected by **green (the WB reference) →
  WB-independent, stable** (no chicken-and-egg). Bounded to a plausible R/B gain range.
- **Shadow green comp.** The sensor's R/B response falls off faster than G at low signal → shadows go green
  even at correct WB. Tone-dependent: in dark pixels pull R/B *toward* G but **only when below it** (reduces
  green, never creates magenta), tapering out by a knee. Gap-proportional per channel.
- **AE:** median-metered, **exposure-only with fixed VMAX** (changing vblank mid-stream glitches a frame),
  **gain capped at the analog max (240)** — digital gain (240–480) adds noise + shifts the operating point
  for no SNR gain. Per-frame AE/AWB errors are non-fatal (an exception must never trigger a camera reinit).
- **Known residual: mixed room lighting** (bright warm doorway vs green-ambient walls) — no single global WB
  can neutralize two illuminants at once; that is physics, not a bug.

## 7. Performance & thermal (measured)
- Full pipeline + YOLO, indoor: **~16 fps** (VMAX-limited by `max_exposure=6000` for low light; drop to 3000
  → ~32 fps at more noise). YOLOv8 NPU ~**35–44 ms/infer**, **capped to 10 fps** (`--yolo-fps`; a slow robot
  does not need per-frame detection, and it frees NPU/heat).
- **Thermal:** idle ~61 °C; active GPU+NPU ~72–74 °C; hot-trip 90 °C, critical 110 °C. Passive cooling,
  enclosed compartment. `--headless` halves the streamer CPU (51% → 27%).
- **YOLO:** **YOLOv8, not v11** (v11's attention needs HTP arch ≥73; v68 rejects it). Model path: AI-Hub ONNX
  → 2.42 DLC (x86 converter) → v68 context binary (on-board generator). Input NCHW [1,3,640,640],
  bottom-right letterbox.

## 8. Headless / production mode (`--headless`)
For a no-viewer production robot: skips the MJPEG server, JPEG encode and overlay draw; runs
capture → GPU demosaic → shm → YOLO unconditionally.
- **Keep AE** (essential exposure control, not cosmetic — a fixed exposure goes black/blown as light changes).
- **Colour compensations (AWB/CCM/shadow_tint/shading) are for human eyes — drop them.** YOLO/COCO is
  colour-cast-robust (it detected fine even through the R↔B swap; grayscale would *hurt* it). Keep fixed
  WB + demosaic + tonemap (≈free).
- Launch: `python3 q6a_camstream.py --cam 2 --gpu --bin --headless` (AWB/CCM already opt-in → off).

## 9. Areas to challenge (for the reviewer)
1. **Two-process shm seqlock** vs a proper double-buffer/atomic — is the single-writer seqlock airtight under
   the detector's read cadence, and is a torn read truly impossible (vs merely skipped)?
2. **White-patch AWB robustness** — fails if the brightest surface is a coloured light (excluded via a clip
   threshold — sufficient?); degrades on scenes with no neutral/white surface.
3. **Shadow green comp** desaturates genuinely-coloured shadows (side effect of "pull toward G"). Acceptable?
4. **MJPEG transport** over the robot↔host USB-gadget-ethernet (~11 MB/s ceiling) — is per-frame JPEG right,
   or should detection results travel separately from (or instead of) video in production?
5. **VMAX-limited fps vs noise** — 16 fps low-noise vs 32 fps noisy: which serves downstream nav/detection
   better?
6. **Client-gating / headless minimal set** (AE-on, colour-comp-off) — is that the right production config for
   a YOLO-only robot, and should there be a thermal guard (auto-drop `--yolo-fps` above ~82 °C)?
7. **GPU ISP is latency-bound** (power-gating), so fp16/zero-copy were judged low-ROI — is that the right call,
   or is clock-pinning / a persistent GPU context worth it?

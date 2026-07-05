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

## 3. Why the whole ISP runs in userspace on the GPU (central decision)
This is the load-bearing architectural decision. The QCS6490 has a **Spectra 570L ISP** (HW demosaic, tone,
3A) and a **Venus** encoder — on paper you would let the ISP demosaic and Venus encode H.264, and the CPU/GPU
do almost nothing. **Neither is usable on this board's mainline software stack**, established empirically:

- **Mainline `qcom-camss` exposes only the RDI (Raw Dump Interface).** `/dev/video0` delivers **packed raw
  Bayer** (`pBAA`, MIPI RAW10) and nothing else. The Titan ISP's pixel pipeline (demosaic/tone/3A) is driven
  by an **undocumented embedded-CPU command stream** that Qualcomm deliberately keeps out of mainline. The
  `vfe3_pix`/`vfe4_pix` pads *do* enumerate and *advertise* UYVY output, which is a trap — wiring them and
  calling `STREAMON` **hangs the driver / yields 0 frames** (verified experiment). There is no mainline path
  to the hardware demosaic.
- **CAMX (the vendor camera stack, `qcom-camx` + `qtiqmmfsrc`) is the only route to the ISP, and it is
  triple-blocked:** (a) it needs a **proprietary CHI-CDK sensor bring-up for the IMX296** — the PPA ships only
  the *compiled* `com.qti.sensormodule.*.bin`, no XML / `buildbins` / source, so you cannot add a sensor;
  (b) `qcom-camx` pulls **`qcom-fastrpc`, which conflicts with Radxa's fastrpc fork** (the same clash that
  blocked a full QAIRT 2.46 apt migration — see §9); (c) it targets the **Venus encoder, which reboots the
  board** (below).
- **The Venus HW encoder (`/dev/video17`, H.264/HEVC) HARD-REBOOTS this board.** The driver enumerates NV12→
  H264 and looks healthy, but starting an encode **instantly power-cycles the board** (uptime went 17374 s →
  135 s; last log `qcom-venus: non legacy binding`). Root cause: on sc7280/QCS6490 non-ChromeOS boards Venus
  firmware loads via a **TrustZone secure PIL** and encode-start trips a hypervisor gate → reset. Fixing it
  needs **physical UEFI access** (SPI fw ≥ `6.0.260120` **and** UEFI → *Hypervisor Override = Enabled*), and
  even then it is a *bandwidth* win, not latency/fps. So: no HW H.264.
- **There is no HW JPEG anywhere on mainline** either (not in Venus, camss, GPU or DSP), so the encode is
  libjpeg-turbo on the CPU.

**Conclusion (matches what the kernel/linux-media maintainers prescribe):** capture RDI raw, then do
**unpack + demosaic + white-balance + tone-map + encode in userspace**. Then the choice was *where*:
- **CPU (numpy):** correct but **~3 fps at 79 °C** — the demosaic alone is ~290 ms/frame. Unusable.
- **GPU (Adreno OpenCL):** the whole ISP as one kernel → **~16–19 fps**, CPU nearly idle. **Chosen.** It also
  contradicts the common community claim that "the Adreno is unusable on Q6A Ubuntu, only the NPU works" —
  the proprietary `qcom-adreno-cl` driver works on the stock `msm` kernel via dma-heap, no KGSL swap.
- **NPU:** wrong tool for a demosaic (it is a fixed-function tensor engine); reserved for YOLO.

So the pipeline is a direct consequence of the hardware/software reality, not a preference: **RDI raw → GPU
ISP → NPU detection → CPU JPEG/serve.**

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

## 9. Dead ends hit & hardware/software limitations
Every rejected path below was tried and failed empirically — listed so the reviewer can judge whether any
deserves revisiting and understands the constraints the design works within.

### Sensor bring-up / kernel
- **Nothing shipped working.** No `imx296.ko` and no working overlay existed for this board; both were built
  from scratch. Overlay gotchas that each cost a boot-loop or 0-frames:
  - **Radxa's stock camera overlays BOOT-LOOP the board** — the overlay's `linux,cma` reserved-memory node
    collides with the `cdsp`/`video`/`zap` firmware reservations → kernel can't reserve them → loop. Fix:
    **strip the `linux,cma` fragment** (CAMSS uses system CMA). This was ahead of the community — nobody had a
    working setup.
  - **`data-lanes=<1>`** — IMX296 is 1-lane MIPI; 2 lanes → **0 frames**.
  - **`clock-names="inck"`** + 37.125 MHz mclk (driver requirement).
  - **en7581 DTB trap** — enabling an overlay can trigger a BLS-entry regen that picks the wrong DTB
    (`en7581-evb.dtb`, a MediaTek board — the kernel ships all vendors' DTBs) → won't boot. Must pin
    `/etc/kernel/devicetree`.
  - Boot backend is **embloader (systemd-boot/EDK2), not extlinux** — overlays enable via a
    `devicetree-overlay` line in the BLS entry, not the usual extlinux.conf.
- **Recovery:** a camera-overlay brick is a boot-config problem, **not** fs corruption — recover via microSD +
  mount NVMe + remove the overlay line. No data-wiping reflash needed. (One `dd`-clone left the SD & NVMe with
  duplicate FS/PART UUIDs — never boot both inserted.)
- **`BG10` (unpacked RAW10) → `STREAMON` EPIPE.** Must use **`pBAA`** (packed) at the video node.

### Sensor 2×2 FD binning — investigated deeply, abandoned
Goal was charge-domain 2×2 binning (cleaner SNR + ¼ MIPI). The mainline `imx296.c` *has* binning code
(crop=full + half-size subdev fmt → `CTRL0D` HADD|FD_BINNING) but **never programs `MIPIC_AREA3W` (0x4182)**,
the MIPI TX active-line count → camss hangs waiting for frame-end. A one-line driver patch stops the hang, but
then **frames come back EMPTY (all 0xFF)** — the binned pixel payload never lands. Confirmed via an
instrumented **`qcom-camss` debug build** that the CSID (`rx=0x0`, zero CSI-2 errors) and VFE IRQ status are
**byte-for-byte identical to a working full-res frame** — camss isn't dropping anything; the **sensor's
FD-binned MIPI payload itself is invalid** (railed at max-code 0x3FF, exposure-independent). It needs Sony's
NDA FD-readout registers (VCUTMODE/OB/sequence) that neither mainline nor RPi's driver program. **Verdict:
non-functional through mainline qcom-camss; GPU digital `--bin` gives the same 728×544, works, and is faster.**

### Accelerator / model limits
- **GPU + NPU concurrently in one PROCESS segfaults** — they corrupt the shared userspace dma-heap/rpcmem
  allocator (no traceback; the board survives). Forced the **two-process** architecture (§4). GPU-alone or
  NPU-alone are fine.
- **YOLOv11/YOLOv10 do NOT run on v68 — at ANY QNN version.** Their attention `MatMul` (C2PSA/PSA) requires
  HTP arch **≥73**; v68 is rejected (`incorrect Value 68, expected >= 73`), confirmed locally *and* by AI
  Hub's own cloud compile failing (`exit code 14`). → forced **YOLOv8** (no attention).
- **QAIRT version maze.** AI Hub only builds **2.45/2.46/2.47**; the board runs **2.42**; a 2.45 artifact
  won't load (`dlc handle code 1002`); the pip `qai_appbuilder` caps at 2.40. The working recipe: export a
  **w8a16 QDQ ONNX** from AI Hub → convert **ONNX→2.42 DLC** with the x86 `qairt-converter` on the Odyssey →
  `qnn-context-binary-generator` on the Q6A for the v68 binary. Note the **x86 `qairt-quantizer` SIGILLs on
  the Odyssey's Celeron J4125 (no AVX2)** — quantization can't run on the host; only the *converter* (no AVX2
  needed) does. A full **QAIRT 2.46 apt migration was abandoned** — blocked by the Radxa-vs-qcom fastrpc fork
  clash, and buys no capability.
- **`PYOPENCL_NO_CACHE` trap:** the pyopencl compiled-kernel cache **silently served stale binaries** across
  `q6a_gpu.py` edits — kernel changes had *no effect* until the cache was bypassed. Cost real debugging time;
  now forced off + cache dirs cleared on launch.

### Colour pipeline dead ends (the 2-day saga, condensed)
The root cause was a **red↔blue Bayer swap** (profile said BGGR, sensor delivers **RGGB**) — every "cast" we
chased was that swap × WB/CCM, and no colour correction can fix a channel swap. Diagnosed only when a
**colour test chart** made it unambiguous (yellow→cyan, magenta-stable). Along the way, each of these was
tried and reverted because it was treating the symptom, not the cause:
- **Gray-world AWB** → tints neutral walls green on a non-gray room (→ replaced by white-patch).
- **Neutral-weighted AWB** (first white-patch attempt) → chicken-and-egg: judging "neutral" via the *current*
  WB locked onto warm surfaces that looked grey at the wrong WB. Fixed by selecting on **brightness (G)**,
  which is WB-independent.
- **Highlight desaturation, chroma denoise, per-row/col chroma-flatten, CCM softening, shade chroma clamp/
  smooth/flip, saturation boost** — all added to fight the swap, all removed after the RGGB fix.
- **Grey-card calibration mismatch:** a paper held near the camera is lit differently than the room, so its
  WB doesn't transfer; and **WB is brightness-dependent** (fixed black over-subtracts weak R,B at low signal),
  so the paper (bright) and walls (dim) want different WB. Root-fixed with a **dark-frame black calibration**
  (true per-channel pedestal) + white-patch AWB. **Mixed room lighting** (warm doorway vs green-ambient walls)
  is an unsolvable-with-one-WB residual — physics.

### Hardware/software ceilings the design lives within
- **Passive cooling, enclosed compartment** → sustained GPU+NPU load sits ~72–74 °C (90 °C hot-trip). Heavy
  sustained compute (e.g. a 3B GPU LLM) has thermally shut the board down; the NPU is the coolest path.
- **Robot↔host link: USB-gadget ethernet ~11–12 MB/s** (a hard `sw_udc` DMA ceiling, not the bus) — fine for
  compressed MJPEG/H.264 + ROS topics, **not** raw frames.
- **Fixed VMAX** ties frame rate to `max_exposure` (6000 → ~16 fps; 3000 → ~32 fps) — the low-light/fps knob.
- **Small 1/2.9" sensor at high analog gain** → a real low-light luminance-noise floor; exposure is the only
  clean lever (no HW denoise; digital gain 240→480 adds noise for no SNR).
- **v68 NPU tops out at ~1B LLMs** and YOLOv8-class detectors — no 3B / attention path on this chip.

## 10. Areas to challenge (for the reviewer)
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

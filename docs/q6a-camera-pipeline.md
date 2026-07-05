# Q6A camera streaming pipeline — journey & final architecture

Definitive record of the IMX296 → live-detection streaming pipeline on the Radxa Dragon Q6A (QCS6490,
Hexagon v68 NPU, Adreno 635 GPU). Covers **what we built, every path we tried and rejected (and why),
and the final architecture**. Sensor/overlay bring-up details live in `q6a-camera.md`; the QNN/QAIRT
version deep-dive is in `q6a-qairt-2.46-migration.md`.

**End state:** full-resolution **1456×1088 MJPEG at ~19 fps with live YOLOv8 COCO detection**, board at
~62 °C. From a naive ~3 fps / 79 °C starting point — a ~6.4× throughput gain at full res.

---

## 1. Final architecture

Two processes on the Q6A, one per accelerator, sharing frames through shared memory — **no lock**:

```
 IMX296 (raw Bayer) ── MIPI CSI ──► CAMSS RDI ──► /dev/video0 (mplane, pBAA packed RAW10)
        │
        ▼   PROCESS A — q6a_camstream.py  (Adreno GPU only)
   ┌──────────────────────────────────────────────────────────────┐
   │ q6a_v4l2.V4l2Cam.read_latest()   V4L2 MMAP, drain-to-latest    │
   │        → unpack_raw10 (CPU)                                    │
   │        → q6a_gpu.GpuDemosaic.isp()   ◄── Adreno OpenCL kernel: │
   │             black-level + WB + shading + demosaic + gamma      │
   │             → uint8 RGB                                        │
   │        → write RGB to shm "q6a_frame", bump frame_seq  ────────┼──┐
   │        → read detections from shm "q6a_ctrl"          ◄────────┼┐ │
   │        → draw_overlay (boxes+labels, PIL)                      ││ │
   │        → JPEG encode → State.jpeg                              ││ │
   │ ThreadingHTTPServer: multipart/x-mixed-replace, client-gated   ││ │
   └───────────────────────────────────────────────────────────────┘│ │
                                                                      │ │
   ┌──────────────────────────────────────────────────────────────┐ │ │
   │ PROCESS B — q6a_detector.py  (Hexagon NPU only)               │ │ │
   │   read latest frame from shm (seqlock snapshot)   ◄───────────┼─┘ │
   │     → q6a_yolo.YoloDetector.infer()  (QNN context binary)     │   │
   │     → write detections to shm  ──────────────────────────────┼───┘
   └──────────────────────────────────────────────────────────────┘
         Adreno ∥ Hexagon run truly concurrently; kernel arbitrates the hardware.
```

**Why two processes:** driving the Adreno (OpenCL) and Hexagon (QNN/fastrpc) **concurrently in one
process segfaults** (they corrupt shared userspace dma-heap/rpcmem allocator state; the board survives).
They run **fine across separate processes**. So the NPU lives in its own process; an in-process lock
(the old fix) cost ~25% and is gone.

**Shared-memory layout** (`multiprocessing.shared_memory`, cheap on unified LPDDR5):
- `q6a_frame`: `H*W*3` uint8 RGB. Single writer (streamer). Reader (detector) takes a **seqlock
  snapshot** — read `frame_seq`, copy, re-read `frame_seq`; retry/skip on mismatch → no torn frames.
- `q6a_ctrl`: `[0]=frame_seq u64  [8]=det_seq u64  [16]=det_count i32  [32:]=32×(x1,y1,x2,y2,conf,cls) f32`.
  Detector writes dets; streamer reads them for overlay (lag ~1 inference, invisible).

**Per-frame cost (Process A):** unpack 12 ms + GPU ISP ~20 ms + JPEG ~10 ms ≈ 42 ms (≈23 fps ceiling).
Process B (NPU) runs independently at ~20 fps.

### Module map
| File | Role |
|---|---|
| `q6a_camstream.py` | Process A: capture + GPU ISP + overlay + MJPEG server; spawns Process B; shm IPC |
| `q6a_gpu.py` | `GpuDemosaic` — Adreno OpenCL ISP kernel (`isp()` → uint8; `demosaic()` → float for tests) |
| `q6a_v4l2.py` | `V4l2Cam` — V4L2 multiplanar MMAP capture, DQBUF-drain-to-latest |
| `q6a_detector.py` | Process B: standalone NPU YOLO; shm frame in → dets out |
| `q6a_yolo.py` | `YoloDetector` — QNN context binary via `qai_appbuilder`; letterbox + threshold + NMS |
| `view_q6a_cam.sh` | Odyssey launcher: deploy all + start streamer + open VLC |
| `build_yolo.sh` | Reproduce the YOLO model: AI-Hub ONNX → 2.42 DLC → v68 context binary |
| `models/yolov8_det.bin`, `coco_labels.txt` | Committed v68 context binary + labels |

### Run
On the Odyssey: `./view_q6a_cam.sh 2` (deploys everything, starts the streamer with `--gpu`, opens VLC).
View with **VLC/mpv, NOT Firefox** (Firefox leaks MJPEG frames into shmem → OOM). Stream URL:
`http://192.168.20.2:8092/stream`. One-time on the Q6A: `pip install --break-system-packages --user
pyopencl` + register the Adreno ICD (the launcher does the ICD automatically).

---

## 2. The journey (chronological)

### 2a. Raw capture → software ISP
The IMX296 is a **raw machine-vision sensor with no ISP** — the CAMSS RDI path gives packed 10-bit Bayer
(`pBAA`), which is not a viewable image. So the whole ISP (demosaic, white balance, tone-map, encode) had
to be done by us. First cut: pure numpy on the CPU → correct but **~3 fps, 79 °C**.

### 2b. Color pipeline (deterministic, not guessing)
It rendered green. Root cause: the sensor is a **color BGGR Bayer** where green reads ~1.6× red/blue
(CFA/QE) — it needs **white balance**, not a bad-camera diagnosis. Fixed with measured constants
(black level ~56, R×1.60 / B×1.52 at the raw level) + an optional smooth radial color-shading map from a
grey-card `--calibrate`. Auto/gray-world per-frame guessing was tried and rejected. (Full detail in
`q6a-camera.md`.)

### 2c. YOLO on the NPU — the QNN version odyssey
Getting object detection onto the v68 NPU took several dead-ends:
- **YOLOv11 (and YOLOv10) do NOT run on v68 — at any QNN version.** Their attention `MatMul` (C2PSA /
  PSA) requires HTP arch **≥73**; v68 is rejected (`incorrect Value 68, expected >= 73`). Confirmed
  locally *and* by AI Hub's own cloud compile failing (`exit code 14`). → **Use YOLOv8** (no attention).
- **Version mismatch:** AI Hub only builds QAIRT **2.45/2.46/2.47**; the board runs **2.42**; a 2.45
  artifact won't load (`dlc handle code 1002`). Fix: export a **w8a16 QDQ ONNX** from AI Hub, convert
  **ONNX→2.42 DLC** with the x86 `qairt-converter` in `~/qairt-x86` on the Odyssey (the converter, unlike
  the quantizer, does **not** need AVX2 → runs on the J4125), then `qnn-context-binary-generator` on the
  Q6A builds the **v68 context binary**. Reproduced in `build_yolo.sh`.
- Runs in-process via `qai_appbuilder` (float I/O; NCHW `[1,3,640,640]` input, **bottom-right letterbox**
  — centered padding halves the scores; outputs scores/class_idx/boxes).

### 2d. Hardware acceleration — investigated, then rejected (why we do it in userspace)
The QCS6490 *has* a Spectra 570L ISP + Venus encoder, but on **mainline they're unusable for us**:
- **Mainline `qcom-camss` is RDI-only.** The Titan ISP pixel pipeline (hardware demosaic) is driven by an
  **undocumented embedded-CPU command stream**, deliberately out of mainline scope. The `vfe3_pix` pads
  *enumerate* and advertise YUV, but `STREAMON` hangs / yields 0 frames (verified experiment).
- **CAMX is the only route to the ISP** (`qcom-camx` + `qtiqmmfsrc`) — ruled out: needs a proprietary
  CHI-CDK **sensor bring-up for the IMX296** (the PPA ships only compiled `com.qti.sensormodule.*.bin`,
  no XML/`buildbins`; the source is gated); the **Venus encoder reboots this board** on our firmware
  (needs a UEFI *Hypervisor Override* + fw ≥ `6.0.260120`, ≤720p); and `qcom-camx` pulls `qcom-fastrpc`,
  which **conflicts with Radxa's fastrpc**. (Same fastrpc clash blocked a full **QAIRT 2.46 apt
  migration**, which we also investigated and abandoned — no capability gain. See
  `q6a-qairt-2.46-migration.md`.)
- **The kernel maintainers prescribe exactly what we did:** RDI raw + demosaic in userspace on CPU/**GPU**.

### 2e. GPU ISP (the big win)
Moved the demosaic to the **Adreno 635 via OpenCL** (`q6a_gpu.py`, pyopencl). This exposed that the CPU
`_post` (destripe + tone-map) was the real bottleneck (156 ms), so we folded the **whole ISP into one
GPU kernel** (demosaic + WB + shading + gamma → **uint8**, 4× smaller readback), moved auto-exposure to a
cheap raw subsample, and made destripe optional. Per-frame **211 ms → 42 ms**, temp **79 → 52 °C**.
- Setup gotcha: pyopencl's bundled loader needs the Adreno registered as an **ICD**
  (`/etc/OpenCL/vendors/adreno.icd → libOpenCL_adreno.so.1`).

### 2f. V4L2 mmap capture
The `v4l2-ctl → tmpfs file → tail-and-seek` hack (piping to stdout **hangs** this CAMSS driver) capped
capture at ~16.8 fps. Replaced with a proper **V4L2 multiplanar MMAP** loop (`q6a_v4l2.py`,
DQBUF-drain-to-latest, linuxpy raw structs) → **23 fps** capture. Falls back to the file method on failure.

### 2g. Exposure = sensor frame rate
In this pipeline the sensor frame length ∝ exposure, so exposure directly sets fps: **6000≈13, 3000≈21,
2000≈27 fps** — lower is faster but noisier in dim light. Default set to **3000** (matches the ~23 fps GPU
ceiling; auto-tone-map handles brightness).

### 2h. Lock elimination → two processes
The GPU ISP + NPU YOLO in one process crashed, so they'd been serialized by an `ACCEL` lock (~25% cost).
Root-caused as in-process userspace corruption (segfault, board survives; GPU already coexisted with the
NPU LLM daemon across processes). Validated that separate processes coexist, then split YOLO into
`q6a_detector.py` sharing frames via shared memory. **Lock gone; live 16 → 19 fps.**

---

## 3. Performance evolution (full-res 1456×1088, with YOLO)

| Stage | live fps | temp | per-frame proc |
|---|---:|---:|---:|
| CPU numpy full-res ISP | ~3 | 79 °C | 211 ms |
| GPU demosaic only (CPU `_post`) | ~4.7 | 79 °C | (`_post` 156 ms) |
| + cheaper `_post` (subsample percentile + gamma LUT) | ~6.8 | — | 147 ms |
| **Lean full-GPU ISP → uint8** | ~8.6 | **52 °C** | **42 ms** |
| + exposure 6000→3000 (sensor 13→21 fps) | ~12.6 | 53 °C | |
| + direct V4L2 mmap capture (16.8→23 fps) | ~16.0 | 52 °C | |
| **+ two-process, no lock (GPU ∥ NPU)** | **~19.2** | 62 °C | |

Remaining gap (19 vs 23 capture ceiling): the 4.75 MB shm frame copy + memory-bandwidth contention.
A double-buffered zero-copy shm would close most of it (not yet done — diminishing returns).

---

## 4. Hard-won gotchas (don't re-learn these)
- **YOLOv11/v10 can't run on v68** (attention `MatMul` needs arch ≥73) — YOLOv8/v9 only.
- **AI Hub is 2.45+, board is 2.42** — convert ONNX→2.42 DLC on the Odyssey (`qairt-converter`, no AVX2).
- **Mainline camss = RDI only**; hardware ISP needs CAMX (proprietary IMX296 bring-up) — not viable.
- **GPU + NPU concurrent in one process = native crash** — use separate processes (no lock).
- **`v4l2-ctl` piping to stdout hangs** this driver; the node is **multiplanar**; format is **pBAA**.
- **pyopencl needs the Adreno ICD registered**; the driver ships as a direct `libOpenCL`, not an ICD.
- **Firefox OOMs on MJPEG** (shmem leak) — view with VLC/mpv.
- **Board is fragile:** NPU restart-storming wedges the cdsp (→ reboot); a hard power-cut while writing
  corrupts the ext4 root (→ initramfs; recover with `e2fsck -y /dev/nvme0n1p3`).

---

## 5. Update — entire ISP on the GPU, CPU near-idle (2026-07-05)
After the two-process split, three more efficiency moves put the *whole* ISP on the Adreno:
- **GPU RAW10 unpack** — the kernels (`isp`/`isp_bin`) take the **packed pBAA buffer + stride** and unpack
  on-device (`bget_packed`); the ~12 ms CPU `unpack_raw10` is gone. Auto-exposure reads the packed high
  bytes (`_auto_scale_packed`).
- **GPU destripe** — `col_sum`/`row_sum` reduction kernels + `destripe_sub` (CPU only smooths the tiny
  (W,3)/(H,3) correction vectors). Replaces the ~11 ms CPU `_destripe_u8`. `--destripe` now runs on GPU.
- **Analog gain 200→380** — cleaner low light (analog gain beats digital tone-map scaling).

**Net:** unpack + demosaic/bin + WB + shading + tone-map + destripe **all run on the Adreno**. The CPU does
only auto-exposure (~1 ms) + JPEG (~4 ms in bin) + the shm copy + HTTP serve. **CPU load ~1.3 → 0.42**,
**temp 57 → 55 °C**, fps **~21.5 (sensor-limited at exp 3000)** — the camera, not the SoC, is now the cap.
What the sensor exposes that is *not* usable: 8-bit/YUV video formats (advertised but hang — RDI is
packed-10-bit only; needs the ISP). Only lever left for more fps is lower exposure (noisier).

**Default run config:** `--gpu --bin --destripe --gain 380` (via `view_q6a_cam.sh`). Remaining known item:
the cyan/green cast is the room illuminant — fix with a grey-card `./view_q6a_cam.sh calibrate` (needs a
person to aim the camera at a uniform surface).

---

## 6. Efficiency deep-dive & final numbers (2026-07-05)

### Performance table (full arc, live fps with YOLO)
| Stage | fps | temp | notes |
|---|---:|---:|---|
| CPU numpy full-res ISP | ~3 | 79 °C | everything on CPU |
| Lean full-GPU ISP → uint8 | ~8.6 | 52 °C | demosaic+WB+tonemap on GPU |
| + V4L2 mmap capture | ~16 | 52 °C | drain-to-latest |
| + two-process (no lock) | ~19 | 62 °C | GPU ∥ NPU |
| + GPU unpack + gain 380 | ~21 | 57 °C | RAW10 unpack on GPU; CPU freed |
| + GPU destripe (round-trip) | ~22 | 55 °C | all ISP on GPU, but stalled on readback |
| **+ vblank fix + GPU-only destripe** | **~32** | 56 °C | **frame timing + no round-trip** |
| `--gpu --bin` (no destripe) | 32.2 | 47 °C | fastest/coolest |
| `--gpu` (full-res, no destripe) | 22.0 | 49 °C | full 1456×1088 |

**~3 → ~32 fps** live, full detection, CPU near-idle (load 0.15). Default: `--gpu --bin --destripe`
(gain 240, YOLO capped 10 fps). Further pipeline optimization + Venus/zero-copy investigation: **§8**.

### The vblank win (biggest free gain)
The IMX296 does **60 fps at full res** — our cap was never the sensor, it was `frame_length`. Frame length =
`H + vblank` must be ≥ exposure; the old `vblank = exposure+200` padded it ~1.4× longer than needed. Setting
`vblank = max(30, exposure − H + 64)` (minimum for the exposure) gives **+45% fps at the SAME exposure**
(same brightness, same noise): exp=3000 went 22 → 32 fps. To go faster still, only shorter exposure helps
(noisier in dim light) — the fundamental light/speed trade.

### Format investigation — settled
- **The sensor is 10-bit only** — confirmed by **Sony's IMX296LLR datasheet**: "10-bit A/D converter",
  "CSI-2 ... RAW10 output", ADC=10 for all drive modes. So **BA81/8-bit is impossible** at the silicon,
  and the mainline `imx296.c` (which exposes only `SBGGR10_1X10`) faithfully reflects that.
  - The datasheet lists a hardware "2x2 Vertical FD binning" mode (720x540 @ 120.8 fps, 10-bit, charge-domain).
    **We chased it end-to-end and it does NOT work through mainline qcom-camss — see §7.** GPU digital `--bin`
    stays the half-res path.
  - Analog gain is 0-24 dB only (ctrl 0-240); above that is digital gain. Best-value default is **gain=240**
    (max analog = the only stage that lowers input-referred read noise; digital gain is redundant with the ISP
    tone-map and costs highlight range). Raise toward 480 (digital) only for genuinely dark scenes.
- **UYVY/YUV** need the ISP (demosaic) → unavailable on mainline (RDI is raw passthrough). Both hang.
- **`pBAA` (packed RAW10) is the only capture format** — and we now **unpack it on the GPU** (`bget_packed`),
  so the packing costs nothing.

### Design questions
- **Do we need destripe? NO — it's OFF by default (2026-07-05).** The scene-based column high-pass
  subtracts a *per-channel* correction, so on real scenes with vertical structure it **injects magenta/green
  color stripes**, and the fused every-8-frame refresh made them *blink*. The sensor's real residual FPN is
  mild (~7% column deviation, barely visible). A proper fix needs a **dark-frame FPN calibration** (capture
  lens-covered → pure column offsets); the `--destripe` flag remains but is unhelpful until that's built.
- **More efficient binary ops for unpack/destripe?** No meaningful win. Unpack is per-pixel bit-shifts on
  the GPU (memory-bound; vectorised `uchar4` loads would be marginal). Destripe is GPU reductions + a
  box-corr kernel. Both are already off the CPU and fast.
- **Rewrite to C/Rust?** **Not worth it.** The heavy compute already runs native — the ISP on the Adreno
  (OpenCL) and detection on the Hexagon (QNN). The CPU is idle (load 0.15); its only jobs are auto-exposure
  (~1 ms), JPEG (~4 ms, native libjpeg under PIL), the shm copy, and HTTP I/O. Python is just orchestration
  at ~30 fps — a rewrite would gain ~nothing. The ceiling is the sensor/GPU, not the language.
- **JPEG:** 3.9 ms (bin) / 13.6 ms (full). Small; hardware JPEG needs CAMX (unavailable), GPU JPEG is
  impractical (serial Huffman). libjpeg-turbo would ~halve it if ever needed.

## 7. Sensor 2×2 FD binning — full investigation & why it's a dead end on qcom-camss (2026-07-05)

The IMX296 datasheet advertises a hardware **2×2 Vertical FD binning** mode (720×540 @ 120.8 fps, charge-domain
= cleaner SNR than our digital GPU bin, ¼ the MIPI data). We tried to enable it properly. Verdict: **the
sensor bins, but its FD-binned pixel payload is invalid through mainline qcom-camss, and the real fix needs
Sony's NDA register sequence.** GPU digital `--bin` remains the shipping half-res path (same 728×544, works,
and is actually *faster* in the pipeline — its `isp_bin` kernel is ~2× lighter than a full demosaic).

**The driver patch (necessary but not sufficient).** Mainline `imx296.c` *has* binning code (crop=full +
half-size subdev format → `CTRL0D` `HADD_ON_BINNING | WINMODE_FD_BINNING`) but **never programs `MIPIC_AREA3W`
(0x4182)** — the MIPI TX active-line count. It stays at the 1088 power-on default, so when FD binning emits
544 lines, qcom-camss waits forever for frame-end → **STREAMON hangs**. Fix: write `MIPIC_AREA3W =
format->height` in `imx296_setup` (correct for full-res=1088, crop, HADD, FD-bin=544). Reproducible via
`scripts/companion/camera/build_imx296_fdbin.sh` + `imx296_fdbin.patch` (out-of-tree, on-board; gcc-13 +
`linux-headers`, vermagic matches, unsigned insmod OK; stock `.ko` backed up to `~/imx296.ko.orig`). i2c
readback confirmed the write landed (`i2ctransfer -f -y 18 w2@0x1a 0x41 0x82 r2` → `0x20 0x02` = 544).

**After the patch: no hang, but empty frames.** Raw captures come back **uniformly 0xFF** (every byte, std 0.0,
even at exposure=4 → not saturation). Axis isolation was decisive:
| Mode | CTRL0D | Result |
|---|---|---|
| Full-res 1456×1088 | WINMODE_ALL | real data (std 55) ✓ |
| **H-only HADD** 728×1088 | HADD | **real data (std 55) ✓** |
| V-only FD 1456×544 | FD_BINNING | empty 0xFF ✗ |
| 2×2 both 728×544 | HADD\|FD_BINNING | empty 0xFF ✗ |

So **horizontal HADD works; only vertical FD binning fails.** The sensor *is* genuinely binning (frame timing
halves — raw capture hits **162 fps binned vs 85 full-res** at short exposure, even above the datasheet's 120).

**Debug-build proof it's not camss and not the patch.** Built the entire `qcom-camss.ko` (25 objs) out-of-tree
on-board and hot-reloaded it (unbind imx296 → `rmmod qcom_camss` → `insmod` → imx296 re-binds via async).
SoC = `qcom,sc7280-camss` → `vfe_ops_170` (`camss-vfe-17x.c`) + `csid_ops_gen2` (`camss-csid-gen2.c`). This
kernel has **no ftrace/dyndbg** (compiled out), so `pr_err` was added to `csid_isr` + `vfe_isr` to dump the
raw interrupt-status registers. **FD-binned and working full-res frames produce byte-for-byte identical IRQ
status**: same VFE SOF (`s0=0x01000200`), same write-master done (`bus1=0x1`), same reg-update, and **CSID
`rx=0x0` = zero CSI-2 RX errors** (no CRC/ECC/DT/line-length errors) in both. The VFE write-master is MIPI-RAW
passthrough (`WM_BUFFER_HEIGHT_CFG=0` — it writes whatever arrives on the bus). So camss processes the FD frame
exactly like a good one, with no complaint.

**Conclusion.** The CSID/VFE aren't dropping anything — the sensor's FD-binned MIPI payload is itself invalid
(all max-code 0x3FF, exposure-independent = a railed FD readout). The mainline/RPi drivers only flip the
`CTRL0D` mode bits, which enable the binning *framing and timing* (hence 2× fps, no hang) but **not a valid
vertical-FD readout** — that needs additional IMX296 readout-sequence registers (FD/VCUT/OB timing) that Sony
keeps **NDA** and neither driver programs (matching the mainline author's own "this should be double-checked"
comment). Web search for a public FD-binning register sequence turned up nothing beyond the `CTRL0D` bits.
Not fixable without the confidential datasheet. **Investigation closed.** Stock camss restored; the imx296
`MIPIC_AREA3W` patch is kept (inert for the non-binned modes we use). Debug artifacts remain on-board at
`~/camss-build` should datasheet access ever appear.

## 8. Pipeline optimization — demosaic + JPEG wall (2026-07-05)

Profiled the GPU-bin pipeline per stage (728×544 out) and optimized it. Net result: the **bright-light
pipeline ceiling rose ~54 → ~76 fps**, and — more valuable for a passively-cooled robot — **indoor draw
dropped from ~60 °C to ~52 °C at the same (exposure-limited) 31 fps**, because the GPU now does far less work.

| Optimization | Effect | Status |
|---|---|---|
| **Destripe fused into the demosaic kernel** | destripe 5.7 ms → **0.6 ms** (real loop 54 → 73 fps) | shipped |
| **`native_powr` tonemap + `-cl-fast-relaxed-math`** | `isp_bin` kernel 10.3 → 7.2 ms (`pow` is Qualcomm's costliest math class) | shipped |
| Non-blocking host↔device copies (single `finish`/frame) | tiny | shipped |
| Destripe col/row correction recompute every 8 frames (static FPN) | avoids the per-frame reduction | shipped (folded into fusion) |

**Destripe fusion (the big one).** FPN column/row banding is *static*, so instead of a separate full-image
`destripe_sub` pass every frame (a 2nd GPU kernel submission ≈ 5.7 ms in this latency-bound regime), the
cached col/row correction is now **subtracted inline in the `isp_bin` output write**. Every 8th frame the
kernel renders *without* destripe and the correction is recomputed from that raw frame for the next 8 (the
one un-destriped frame per period is imperceptible; verified per-frame mean std 0.18, no pumping). Destripe
went from a 24 fps tax to ~3.5 fps.

**The `isp_bin` per-frame time is submission latency, not compute.** Micro-profiling the sub-steps *pipelined*
(tight loop, one `finish`): upload 1.98 MB = 1.4 ms, kernel = 1.4 ms, readback 1.19 MB = 0.7 ms → **~3.4 ms
of real work**; the ~10 ms per-frame figure is Adreno **power-gating between frames** at 32 fps (≈21 ms idle
gap) paying a wakeup/submit latency on each `finish()`. This reframes the whole optimization space:
- **fp16 / zero-copy have low ROI** — they shave the 3.4 ms compute, not the ~6.6 ms wakeup latency. They
  only help when the GPU is kept busy (bright light). Zero-copy IS feasible here (`cl_qcom_dmabuf_host_ptr`
  is advertised → import a V4L2 `VIDIOC_EXPBUF` dmabuf as a `cl_mem` via a ctypes shim, `CL_MEM_DMABUF_HOST_PTR_QCOM`
  0x411D + page-align + `EXT_MEM_PADDING`), but not worth the effort given the latency-bound profile.
- **Pinning the GPU clock** (devfreq `userspace` @ 812 MHz vs `simple_ondemand`) gained only ~0.8 ms and
  raises idle heat — rejected for the passive-cooled robot.
- The driver even exposes `CL_QCOM_UNORM_MIPI10`/`CL_QCOM_BAYER` image formats (hardware RAW10 unpack +
  bilinear) and `cl_qcom_vector_image_ops` `qcom_read_imagef_2x2` — a future kernel rewrite could use them,
  but again the win is on the latency-hidden compute. (Ref: Qualcomm OpenCL guide 80-NB295-11.)

**JPEG is already optimal on CPU.** Pillow links **libjpeg-turbo** (NEON) → 2.7 ms for 728×544. There is
**no hardware JPEG on mainline** (not in Venus `venc_formats`, not in camss, no GPU/DSP path) — confirmed.
Keep libjpeg-turbo.

**Venus H.264 hardware encoder — CONFIRMED it hard-reboots this board.** `/dev/video17` (`qcom-venus`)
enumerates NV12→H264/HEVC and the driver is healthy, but calling `v4l2h264enc` **instantly reset the board**
(uptime 17374 s → 135 s; last log `qcom-venus: non legacy binding`). Root cause (confirmed on the Radxa
forum + linux-media): on sc7280/QCS6490 non-ChromeOS boards Venus firmware loads via TrustZone secure PIL,
and encode-start trips the hypervisor memory gate → reset. **Fix requires physical UEFI access we can't do
remotely:** update SPI boot firmware to ≥ `6.0.260120` **and** set UEFI → Hypervisor Settings → *Hypervisor
Override = Enabled* (the "auto" default reboots). Even fixed, Venus is a **bandwidth** win (inter-frame
H.264 ≈ 5–20× smaller than MJPEG) **not a latency/fps win** (HW encode ≈ 4–5 ms + 1 frame of M2M pipeline
delay — worse than our 3 ms MJPEG). So it's a future option for slashing stream bandwidth over the link,
gated on a firmware/UEFI change; it does nothing for the demosaic+JPEG throughput wall. NV12 input needs
128-wide / 32-high alignment (728→768 stride, 1456→1536). (Refs: Radxa forum "encoder reboot" thread;
kernel stateful-encoder API; FFmpeg `v4l2_m2m_enc.c`.)

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

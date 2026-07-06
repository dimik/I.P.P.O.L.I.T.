# I.P.P.O.L.I.T. — Q6A Vision Pipeline: Architecture Improvement Plan

**Date:** 2026-07-06. Derived from [`q6a-pipeline-review-findings.md`](q6a-pipeline-review-findings.md) (Fable-5
review of [`q6a-pipeline-review-brief.md`](q6a-pipeline-review-brief.md)). This is the *actionable* plan:
what to change, why, effort, dependencies, and the order — filtered by engineering judgment, not a restatement
of the review.

## 0. What the review changes about how we plan

The review **validated the core architecture** (RDI raw → single-kernel GPU ISP → shm → NPU YOLO, two
processes) and, more usefully, **re-identified the binding constraint**. Three conclusions drive everything
below:

1. **The ceiling is thermal + DDR bandwidth, NOT compute.** The whole pipeline + LLM is ~1.0 CPU core, GPU at
   its *lowest* OPP. There is only **~1.5 W (~12–17 °C) of passive headroom**, and the **NPU has no kernel
   throttle path** (hot-notify 90 °C → 110 °C PMIC power-off, already hit once). ⇒ Every improvement is judged
   by its **watts/°C**, not its ms. The two biggest levers *reduce* load: stop doing wasted work (ISP cadence)
   and make YOLO cheaper (w8a8).
2. **There are real correctness gaps that block the roadmap** — chiefly a **torn-frame seqlock** and **no
   process supervision / NPU-context release**. These are cheap but *load-bearing*: ByteTrack/depth need
   untorn frames, and an orphaned NPU client wedges the cDSP → reboot. They land first.
3. **Two planning constants were wrong** (12 GB not 16; ~15 GB/s practical DDR not 40–50) and a **5.6 GB
   pinned-memory gap** needs a clean-boot re-measure — these gate the memory-heavy items (LLM + Nav2 + 2nd
   cam).

**Framing:** P0 = correctness/safety prerequisites (must land first). P1 = compute/thermal reduction (highest
architectural leverage, cheap). P2 = capability additions. P3 = larger/conditional integrations. Plus a
"deferred/rejected" list and owner-gated questions.

---

## Implementation status (audited 2026-07-06)

Authoritative status of every item, from a critical code+device audit (do NOT infer "done" from a CHANGELOG
title — the "P0.7, P0.8" entry landed only P0.7). ✅ done · ⚠️ partial/done-differently · ❌ not done.

| Item | Status | Reality |
|---|---|---|
| P0.1 seqlock | ✅ | both channels fenced |
| P0.2 return-channel | ✅ | `dseq` guarded read + fence |
| P0.3 supervision/teardown | ✅ | respawn + clean NPU release + staleness cutoff |
| P0.4 resource_tracker | ✅ | detector `unregister()`s attached shm |
| P0.5 MJPEG timeout | ✅ | `settimeout(10)` + widened except |
| P0.6 errno re-raise | ✅ | EAGAIN-only break, else raise |
| P0.7 `--fast`+YOLO | ✅ | OUT dims halved before shm alloc + shape guard |
| P0.8 assert + drop file-tail | ✅ | open-time `frame_size==FRAME` assert added (`expect_size`); file-tail fallback + `q6a_cap.raw` deleted; fault recovery = reinit+backoff (fixed 2026-07-06) |
| P0.9 thermal governor | ✅ | throttle (82/87) **+ hard cutoffs added 2026-07-06**: 88 °C clean detector-park (respawn at 82, hysteresis) + 95 °C orderly SIGTERM before the 110 °C PMIC; unit-tested. (force-bin + LLM-refuse still deferred → see **P1.7** decode-contention scheduling) |
| P0.10 label rename | ✅ | `yolov8_det` |
| P1.1 ISP-at-cadence | ✅ | headless paces ISP to YOLO_FPS |
| P1.2 w8a8 + native input | ✅ | deployed default; ~3× vs w8a16; reproducible w/o any `.pt` |
| P1.3 held-fd AE | ✅ | `VIDIOC_S_CTRL`, no per-tick fork |
| P1.4 strip desktop | ✅ | +~480 MB RAM |
| P1.5 mode-dependent AE | ⚠️ | **done DIFFERENTLY: owner capped stationary at 2000 (not the plan's 6000) / moving 1200, for high fps. Legitimate override.** |
| P1.6 constants + rename | ✅ | 12 GB / ~15 GB/s in CLAUDE.md |
| P2.1 ByteTrack | ✅ | verified live; stable IDs, low-conf recovery to 0.23 |
| **P2.2 CAM3** | ❌ | **all four landmines still hardcoded (`/dev/video0`, shm names, `pkill v4l2-ctl`, `.npz` path); only the media-graph `--cam` is parameterized** |
| P2.3 mono-depth | ⚠️ | model gate ✅ (w8a8 v68, 5.28 ms); coexistence ✅ (depth+detector leak-free); **runtime ✅ — `q6a_depth.py` publishes inverse-depth at 5 fps, correct near/far structure, parks with the detector (2026-07-07)**; still NOT done: **LiDAR/floor-plane metric scale** (needs robot `/scan`), a depth consumer, and true 3-way-active (blocked on `q6a-llmd` health + pinned-mem re-measure) |
| P3.1 ROS/Nav2 · P3.2 power HW | ❌ | not started |

**Before P2.3 goes live:** land P0.8, harden P0.9 (add the 88/95 °C hard rungs), then run the 3-process
(detector+LLM+depth) coexistence + in-compartment thermal + pinned-memory re-measure (Investigations §1–2).
The rule below still holds: no sustained accelerator load without a live guard + an in-compartment re-measure.

---

## P0 — Correctness & safety (prerequisites; land before adding any load)

Mostly small, but several are architecturally load-bearing (enable tracking, prevent field failures / cDSP
wedge / thermal shutdown). Target: **~1 focused session.**

| # | Change | Why (impact) | Effort | Files |
|---|---|---|---|---|
| P0.1 | **Odd/even seqlock** on the frame shm (bump seq→odd before write, →even after; reader retries on odd/changed) | Torn inferences happen now (~2 ms window/60 ms). Harmless for boxes, **disqualifying for ByteTrack/depth**. Prerequisite for P2. | S (~20 lines) | `q6a_camstream.py`, `q6a_detector.py` |
| P0.2 | **Detection return-channel protocol** — `dseq` write+read, timestamp each detection set; streamer reads guarded | Return channel currently has *no* protocol (`dseq` written, never read; `dcnt`/`dbuf` read unguarded). Needed for ByteTrack + ROS. | S | both |
| P0.3 | **systemd unit(s): `Restart=on-failure` + clean SIGTERM that releases the QNN context** + detector-Popen watchdog + **staleness cutoff** (drop boxes if det seq stale > N ms) | Dead detector = frozen boxes forever; streamer SIGKILL → orphan detector holds NPU → **cDSP wedge → reboot**. Highest combined field risk. | M | new `systemd/`, `q6a_detector.py`, `q6a_camstream.py` |
| P0.4 | **SharedMemory `track=False` workaround** (Py 3.12 resource_tracker unlinks live shm on the detector's exit) | Detector exit can unlink the shm out from under the streamer. | S | both |
| P0.5 | **MJPEG socket `settimeout(10)`** | A suspended/NAT-half-open client blocks the handler thread forever → `State.clients` never decrements → **camera+GPU pinned on, on a battery robot**. | S (1 line) | `q6a_camstream.py` |
| P0.6 | **`read_latest`: check errno, re-raise real errors** (don't swallow EIO/ENODEV as EAGAIN) | Device error + select firing → 100% busy-spin, vision dead, no recovery. The existing reinit path then engages. | S | `q6a_v4l2.py` |
| P0.7 | **Fix-or-delete `--fast`+YOLO** (half-res CPU frames vs full-res shm dims → crash every frame; GPU-init failure falls into this path) | A driver regression currently becomes **total silent vision loss**. | S | `q6a_camstream.py` |
| P0.8 | **`frame_size == FRAME` assert at open** + delete the file-tail fallback | Silent 0 fps on any stride/padding mismatch; file-tail writes 580 MB/batch to `/dev/shm` and is itself the crash-loop path. | S | `q6a_v4l2.py`, `q6a_camstream.py` |
| P0.9 | **Thermal governor** (poll max(cpu*,nspss*) @0.5 Hz; ladder: 78→YOLO 5 fps, 84→2 fps+force bin+refuse LLM, 88→park detector *cleanly*, 95→orderly SIGTERM; 60 s hysteresis; drive fan as rung 0 if fitted) | **The NPU has no other throttle** — this is the only software mitigation. The 3B-GPU shutdown was this exact path. Mandatory before any added load. Bench-calibrate, shift −5 °C after in-compartment measurement. | M | new `q6a_thermal_guard.py` or in-streamer thread |
| P0.10 | Rename the stale `yolov11_det` QNNContext label → `yolov8_det` | Logs currently mislabel the deployed w8a8 v8 model. | S (1 line) | `q6a_yolo.py:48` |

---

## P1 — Compute/thermal reduction (highest architectural leverage, low cost)

These directly attack the binding constraint (thermal/bandwidth). Do them before any capability additions —
they *create* the headroom the rest of the roadmap spends.

- **P1.1 — ISP/publish at detector cadence in `--headless` (biggest free win).** Today the GPU demosaics +
  memcpys 16–32 fps to feed a 10 fps consumer. Gate the ISP+shm-publish to `YOLO_FPS` (or the detector's
  consumption rate) → **~40–60 % GPU + memcpy reduction** for zero capability loss. Keep full rate only when a
  viewer is attached. *Effort M; files `q6a_camstream.py`. This is the single cheapest thermal win in software.*
- **P1.2 — YOLO w8a8 + quantized-input path (best optimization target).** The deployed export is **w8a16 with
  float I/O**; appbuilder quantizes float→uint16 per call (`copyFromFloatToNative`, 5–15 ms ≈ 25 % of the
  inference budget), and ecosystem w8a8 numbers are ~12 ms vs our 38–44 ms. A w8a8 export + a quantized-input
  tensor path likely **halves-to-thirds per-inference cost → same 10 Hz at ~15 % HTP duty instead of ~40 %**,
  freeing thermal + LLM headroom. Worth more than any model upgrade. *Effort M–L (one AI Hub export job + the
  2.42-DLC pipeline in `build_yolo.sh` + input-tensor plumbing in `q6a_yolo.py`); gate: confirm w8a8 v8
  compiles for v68 and accuracy holds.*
- **P1.3 — Held-fd `VIDIOC_S_CTRL` for AE** (replace `subprocess.run(v4l2-ctl, check=False)`). Removes a
  **10–20 ms fork per exposure/gain change from the capture thread** and makes set-ctrl failures visible
  (module EXPOSURE/GAIN state currently diverges from hardware silently). *Effort S–M; `q6a_camstream.py` +
  `q6a_v4l2.py`. Control law itself is fine — keep it.*
- **P1.4 — Strip the desktop stack for production** (GNOME/gdm, docker, fwupd, cups ≈ 500 MB+ RSS + idle CPU +
  the Prime-core-pinned-at-2.7 GHz anomaly). Frees RAM (we only have 12 GB) and a few °C. *Effort S (systemd
  disable/mask); reversible.*
- **P1.5 — Mode-dependent AE ceiling.** Stationary: keep `max_exposure=6000` (low noise; YOLO discards the
  extra frames anyway). Moving/rotating (key off `cmd_vel`/status): clamp AE ceiling to ~2000–3000 so gain
  compensates — 60 ms integration at 1 rad/s ≈ 3.5°/frame **motion blur**, bad for detection-during-turns and
  poison for any future VO. *Effort S–M; needs a motion signal from the bridge.*
- **P1.6 — Fix planning constants in `CLAUDE.md`** (RAM **12 GB**, DDR **~22 GB/s theoretical / ~15 GB/s
  practical**), and rename `yolov11_det` everywhere. *Effort S; doc-only but prevents future mis-sizing.*

---

## P2 — Capability additions (after P0/P1 create headroom)

- **P2.1 — ByteTrack** (Kalman + Hungarian on <50 objects, <1 ms/frame on a Silver core). Adds stable track
  IDs + temporal smoothing → better downstream behavior and lower-confidence-frame recovery. **Depends on P0.1
  + P0.2** (untorn frames + timestamped detections). *Effort M; new module consuming the detection shm.*
- **P2.2 — Second camera (CAM3) enablement.** Hardware is *not* the constraint (disjoint
  csiphy3→csid1→vfe0_rdi1; +3.4 ms GPU, +1–2 °C). The blockers are 4 code landmines in `q6a_camstream.py`:
  hardcoded `/dev/video0`, hardcoded+unlinked-at-init shm names, `pkill -9 -f v4l2-ctl` (kills the sibling),
  fixed `.npz` path. **Parameterize all four**, then live-verify CAM3 → `/dev/video1` (the one unproven link).
  *Effort M; do only if a 2nd camera is actually wanted (coverage/redundancy/depth).*
- **P2.3 — NPU mono-depth (MiDaS-V2, official 4.117 ms w8a8 on QCS6490) with LiDAR/floor-plane scale** — the
  review's substitute for stereo. Gives off-plane obstacle sense + metric depth for ~0.2 W, without a second
  SLAM. **Depends on validating 3-process accelerator coexistence** (detector + LLM + depth) on a clean boot
  with dmabuf-growth monitoring (the one-process segfault + suspected leak make this untested). *Effort L; do
  only if depth/obstacles is a real requirement (see Q7).*

---

## P3 — Larger / conditional integrations

- **P3.1 — ROS2/Nav2 integration shape (frames stay OUT of DDS).** One thin node reads the (fixed) detection
  shm and publishes detections/pose/small crops (~KB/s); frames never enter DDS (a DDS image sub costs
  serialize+copy+~56 MB history and half a Silver core, and the GPU+NPU one-process segfault forbids a
  composed container holding both). Nav2 as a single composed rclcpp container on Silvers: Regulated Pure
  Pursuit, 0.1 m costmap, 5 Hz, `ROS_LOCALHOST_ONLY`, **skip AMCL/slam_toolbox** (Valetudo bridge supplies
  map→base_link). **Prereq: relocate `valetudo_bridge.py` to the Q6A.** Open: does Nav2 take the bridge's
  2 Hz TF directly or need an EKF (`robot_localization` fusing the tapped IMU) shim (Q5)? *Effort L.*
- **P3.2 — Power/thermal hardware & daemons** (these unlock everything thermal-bound):
  - **Fit a 25 mm fan / lid-heatsink off the 12 V rail (~0.3–0.5 W)** — the single cheapest enabler; community
    data on this SoC shows **+15–25 °C envelope**. Drive it as thermal-guard rung 0. **Do this early if any
    sustained multi-accelerator load is planned.**
  - **Brownout daemon** — watch battery SoC via the bridge, clean poweroff <~20 % (an unclean cut risks the
    ext4 root + a cDSP wedge). *Effort S–M.*
  - **Dock pass-through test** — if the Dreame charger sustains net-positive with ~8 W Q6A load, docked-sentry
    is indefinite. *Test, not code.*

---

## Restored from the review (dropped by plan v1; surfaced by the 2026-07-06 audit)

The review recommended these; the first plan neither carried nor rejected them. They are NOT in the
deferred/rejected table below — they are real, un-triaged suggestions.

- **P1.7 — NPU decode-contention scheduling** (review findings §1/§5). Throttle YOLO to ~5 Hz *while the LLM
  is decoding* and coalesce LLM queries. This is **contention-triggered, not temperature-triggered** — P0.9
  only reacts to heat, so nothing today prevents LLM decode from jittering the vision cadence (or vice-versa).
  **Directly relevant to P2.3**: a third accelerator (depth) makes NPU/DDR contention worse. *Effort M;
  cross-process signal (e.g. the LLM daemon sets a flag the detector reads). Do alongside/after P0.9.*
- **P2.4 — VO-lite (SMF-VO-class, <10 ms/frame, single Silver core)** (review findings §4). The review's
  stereo substitute was mono-depth **plus** a light visual-odometry for the one real localization gap — the
  LiDAR-parked manual-drive mode where the robot is currently blind. Plan v1 kept only the mono-depth half
  (P2.3) and silently dropped VO-lite. *Effort M–L; gate on whether the manual-drive blind spot matters.*
- **P3.3 — Gemma-3-1B on llama.cpp CPU with GBNF grammar-constrained decoding** (review findings §8). For
  constrained-JSON event-driven ROS handlers: Genie cannot grammar-constrain, and Gemma-3-1B (IFEval 80.2)
  beats Llama-1B (59.5) at instruction-following. Two-model split: NPU Llama for free-text, CPU Gemma+GBNF
  for tool/JSON. *Effort M; revisits the dropped offline-agent problem with the right tool.*

Minor (also dropped): a `qai_hub_models` w8a8 **LLM** export test on qcs6490 (settle if a 2nd NPU-LLM path
exists); **measure ISP/YOLO frame-time jitter during LLM decode** (quantifies P1.7's payoff); and 3 code nits
from findings §11 (the `hash(label)` random box colour — now moot, ByteTrack keys colour off track_id; the
detector dim-fallback crash on a `--bin`-sized shm; the GPU `d_in = W*H*2` ≤10-bit sizing caveat).

---

## Explicitly deferred / rejected (do NOT build)

| Item | Verdict | Reason |
|---|---|---|
| **Stereo VSLAM** | Drop | +3–5 W CPU (ORB-SLAM3 66–88 ms/frame) on ~1.5 W headroom, to *duplicate* LiDAR+odom+IMU+Valetudo localization. Use NPU mono-depth + LiDAR scale instead (P2.3). |
| **SAHI tiled inference** | Skip | ~7 NPU passes ≈ 250–310 ms/frame; starves the shared HTP. 640×640 @10 Hz is the indoor norm. If far-field recall matters: one on-demand floor-band crop on low-confidence frames. |
| **YOLOv8-seg** | Gate | Run one AI Hub v68 export test *and* have an actual mask consumer first; +20–45 % NPU + CPU mask post. |
| **Venus H.264** | Skip | Board-resetting codec needing physical UEFI access for a *bandwidth-only* win; MJPEG at 1.1 MB/s is already 10 % of the link. |
| **Continuous VLM/captioning** | Event-only | No v68 NPU path; CPU Moondream/SmolVLM ≈ 5–15 s/caption → 0.1–0.2 Hz event-triggered only. |
| **fp16 / zero-copy GPU ISP** | Not now | GPU is latency-bound (power-gate). Revisit + A/B mid-OPP pinning (450/550) only at dual-camera, where sustained cadence may amortize the wakeup. |
| **QAIRT 2.46 apt migration** | Skip | fastrpc fork clash; no capability gain. |

---

## Investigations that gate decisions (non-code, do before the load-heavy items)

1. **Reboot → re-measure the 5.6 GB pinned-memory gap** (MemAvailable was only 2.84 GB). Suspected
   fastrpc/dma-heap leakage from the day's crashed GPU+NPU experiments — could be a clean-boot non-issue or a
   real leak that gates LLM+Nav2+2nd-cam coexistence. Watch per-run dmabuf growth.
2. **In-compartment (lid-closed) thermal** — the 72–78 °C figures are bench-side; expect +5–10 °C enclosed,
   which eats half-to-all the 1.5 W budget. Measure before committing any added load.
3. **PD inline power meter** — profile idle / +ISP / +YOLO / +LLM / all. One afternoon replaces ±40 % model
   error and makes the power table real.
4. **Confirm QCS6490 variant** (Tj 95 °C standard vs 105 °C extended) — sets the thermal-guard thresholds.

---

## Open questions blocking parts of the plan (owner input)

1. In-compartment vs bench temps + ambient? (calibrates the thermal guard / whether headroom is real)
2. Battery: stock 5200 mAh (≈75 Wh, matches "12 h @ 6 W") or upgraded? (all runtime math hinges on it)
3. On the robot 12 V buck or bench? Which module — buck-boost (regulates 12 V-out at 12 V-in) or plain buck
   (dropout → PD collapse → hard cut)?
4. Does the dock charger sustain net-positive at ~8 W Q6A draw? (→ indefinite docked-sentry)
5. What consumes detections in headless — the ROS bridge node? Does Nav2 take the 2 Hz TF or need an EKF shim?
6. What actually drives any SAHI/seg want (small far objects? floor debris? pixel masks?) — default is "don't".
7. Is stereo wanted for **depth/obstacles** (→ mono-depth answers it) or **localization** (→ redundant)?

---

## Suggested sequencing (milestones)

- **M1 — Harden (P0):** one session. Ship the correctness fixes + thermal governor + systemd supervision.
  Outcome: the current pipeline is field-safe and won't wedge the cDSP or thermally die unattended.
- **M2 — Reclaim headroom (P1):** ISP-at-detector-cadence + w8a8 YOLO + held-fd AE + strip desktop + constant
  fixes. Outcome: roughly *halve* the vision compute/thermal load; measure the new envelope (needs the
  investigations + power meter). This is where "thermally thin" becomes "comfortable" — pair with the fan.
- **M3 — Add capability (P2), one at a time, each behind the thermal guard:** ByteTrack → CAM3 (if wanted) →
  mono-depth (after the 3-process coexistence test). Re-measure thermals after each.
- **M4 — Integrate (P3):** valetudo_bridge relocation → thin detection node → composed Nav2; power hardware
  (fan, brownout daemon, dock test) in parallel.

**Rule enforced throughout:** never add a sustained accelerator load without (a) the thermal guard live and
(b) an in-compartment thermal re-measurement. The headroom is ~1.5 W — it must be spent deliberately.

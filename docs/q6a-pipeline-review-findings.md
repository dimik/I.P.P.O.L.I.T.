# I.P.P.O.L.I.T. — Q6A Vision Stack & Video Pipeline: Review Findings

**Date:** 2026-07-06. Response to [`q6a-pipeline-review-brief.md`](q6a-pipeline-review-brief.md).

**Method.** Review of the brief + pipeline source (`scripts/companion/camera/`), a **live probe of the Q6A
over the wired link while the pipeline was running**, web research against primary sources (Qualcomm
datasheets, mainline sc7280 DT, published edge benchmarks), and four independent analysis passes
(perf/thermal, concurrency, power budget, adversarial design review) over the combined evidence
(13 agents, ~230 tool calls). Every claim below is grounded in repo code/docs, tonight's measurements,
or a cited external source (§13).

---

## 1. TL;DR verdict

The core architecture — RDI raw → single-kernel GPU ISP → shm → NPU YOLO, two processes because of the
dma-heap segfault — is **sound and well-reasoned**; every major judgment call in the brief survives
scrutiny. The live probe confirms it is cheap: the whole pipeline plus resident LLM costs **~1.0 CPU
core, GPU at its *lowest* OPP (315 MHz), 71–78 °C, zero throttle events**. The headroom problem is not
compute — it is **thermals (~12–17 °C to the 90 °C floor ≈ ~1.5 W of additional sustained dissipation,
passively) and memory bandwidth** — and two planning constants are wrong: the board is **12 GB, not
16 GB**, and the DDR budget is **~22 GB/s theoretical / ~15 GB/s practical, not 40–50 GB/s**.

Concurrency wishlist verdicts:

| Component | Verdict | Binding constraint |
|---|---|---|
| 2nd IMX296 (capture + GPU ISP) | **Fits, with conditions** | Code blockers, not hardware (§4) |
| Stereo VSLAM | **Does not fit** | CPU-thermal; strategically redundant vs LiDAR SLAM |
| ROS2 Jazzy + Nav2 | **Fits** | None at target rates (~1–1.5 cores, 0.3–1.5 GB) |
| Resident 1B NPU LLM | **Fits, with conditions** | DDR bandwidth during decode; HTP timesharing |
| SAHI tiled inference | **Skip** | 7 NPU passes ≈ 250–310 ms/frame; starves the HTP |
| YOLOv8-seg | **Gate** on an AI Hub v68 export test + an actual mask consumer |
| ByteTrack | **Add now** | <1 ms/frame; needs the seqlock fix first |
| All together | **Fits only if** stereo→mono-depth, leak resolved, NPU scheduling + thermal guard added |

The single cheapest enabler in the whole roadmap is a **25 mm fan or lid-as-heatsink off the 12 V rail
(~0.3–0.5 W)**: community data on this exact SoC shows it moves the envelope **15–25 °C** — the
difference between "thermally thin" and "comfortable" — and the NPU zone has **no kernel throttle path
on mainline** (hot-notify 90 °C → critical 110 °C PMIC power-off, already hit once), so software
duty-cycling + cooling are the *only* NPU mitigations that exist.

---

## 2. Live ground truth (2026-07-05/06, pipeline running: `--cam 2 --gpu --bin --awb`, 1 MJPEG client)

| What | Measured |
|---|---|
| camstream / detector CPU | 52.6% / 39.0% of one core → **≈1.0 core total**, mostly Silvers |
| Throughput | 16 fps publish, YOLO ~10 Hz; `model_inference` 38.4–44.5 ms + **5.2–15.3 ms `copyFromFloatToNative`** |
| Temps | CPU 71–78 °C (cpu6 hottest 78.3), GPU 67–68 °C, NPU 67–69 °C, DDR ~72 °C; **zero throttle events this boot** |
| GPU devfreq | **315 MHz — lowest OPP — throughout**; `simple_ondemand` never ramps under OpenCL load |
| LLM daemon | RSS 95.6 MB + 1.78 GB dmabuf weights; **1.34 s total CPU over 18 h** — adaptive-libGenie truly zero idle |
| Memory | **12,016 MB physical (12 GB board)**; MemAvailable only **2.84 GB** — ~5.6 GB kernel-pinned, unaccounted (suspected fastrpc/dma-heap leakage from the day's crashed GPU+NPU experiments) |
| Anomalies | Prime core pinned 2.707 GHz at ~9% util (13,835 s at max freq this boot); full GNOME/gdm + docker + fwupd + cups still running (~500 MB+ RSS) |
| ROS2 | **No nodes running** — "ROS2 running" is really "installed". Daemon alone: 69.8 MB RSS, ~2% CPU. Real Nav2 footprint unmeasured on this board |

**Planning corrections for CLAUDE.md / the brief:**
- **RAM: 12 GB** (11.5 GB usable), not 16 GB.
- **DDR: ~22 GB/s theoretical** (Radxa ships LPDDR5-5500 on a 2×16 bus; SoC-optimal is 6400 = 25.6 GB/s),
  **~15 GB/s practical multi-master** (measured single-core memcpy 7.6–8.4 GB/s). The 40–50 GB/s figure is
  ~2× reality. **1B LLM decode alone needs ~9–10 GB/s effective** — half the practical bus — which is why
  GPU llama.cpp ≈ NPU decode speed, and why ISP/YOLO frame-time jitter should be expected (and measured
  once) during LLM decode.
- Deployed detector is **yolov8_det.bin (w8a16)**; the `"yolov11_det"` string in logs is a stale
  `QNNContext` label in `q6a_yolo.py:48`. Rename it.

---

## 3. Performance & thermal

**Bottleneck hierarchy (confirmed):**
1. **Sensor exposure/VMAX is the fps ceiling** — everything else has slack.
2. **GPU ISP is genuinely latency-bound** (~3.4 ms real work vs ~10 ms incl. the ~6.6 ms Adreno
   power-gate wakeup — corroborated by devfreq never leaving 315 MHz). Skipping fp16/zero-copy was right
   *today*; the regime **expires under dual-camera load**, where interleaved frames keep the GPU awake and
   may amortize the wakeup for free. Re-A/B 315 vs 450/550 MHz pinning at CAM3 bring-up, not before.
3. **CPU is nearly idle** (~1.0 core total).
4. **The NPU is the least understood resource and hides the cheapest win.**

**The YOLO 3× gap is the best optimization target.** Measured 38–44 ms; ecosystem numbers for
YOLOv8-class w8a8 on this NPU are ~12 ms. The deployed export is **w8a16 with float I/O** — appbuilder
quantizes float→uint16 per call (`copyFromFloatToNative`, 5–15 ms, ~25% of budget). Test a w8a8 export +
a quantized-input tensor path: likely half-to-third per-inference cost → same 10 Hz at ~15% HTP duty
instead of ~40%, freeing thermal + LLM headroom. Worth more than any model upgrade.

**Thermal headroom, quantified.** Fitting measured points gives ~8–9 °C/W for this passive setup; from
78 °C (hottest zone) to the 90 °C floor is **~1.5 W of additional sustained dissipation**. Fits: second
camera + ISP (+0.3–0.5 W), ByteTrack (~0), mono-depth (~0.2 W), YOLO at higher duty (+0.5–1 W). Does not
fit: CPU stereo SLAM (+2.5–4 W), sustained SAHI+LLM (+2–3 W). Caveats: (i) the 72–74 °C figures appear
to be **bench-side, not in the closed compartment** (robot offline during Q6A sessions) — expect
+5–10 °C enclosed, which eats half-to-all of that budget; measure lid-closed before committing.
(ii) Official Tj is 95 °C (standard variant) — the 90–110 °C band is already marginal silicon territory.

**Thermal governor — build now; it is load-bearing.** Mainline sc7280 DT: CPU passive trips 90 °C, GPU
95 °C, but **nspss (NPU) zones have only hot-notify 90 °C and critical 110 °C** — no cooling device
bound. The 3B-GPU shutdown was this exact path. Ladder (bench-calibrated; shift −5 °C after
in-compartment measurement), keyed on max(cpu\*, nspss\*) polled at 0.5 Hz:
- **<78 °C** normal (YOLO 10 fps).
- **78–84 °C**: YOLO→5 fps, ISP at detector cadence, LLM queries queued.
- **84–88 °C**: YOLO→2 fps, force `--bin`, refuse LLM (busy reply), drop MJPEG clients.
- **≥88 °C**: park the detector *cleanly* (release the QNN context — an orphaned NPU client wedges the
  cDSP until reboot), capture keepalive 2 fps.
- **≥95 °C**: orderly SIGTERM of everything — a clean shutdown beats the PMIC yanking power.
- Hysteresis: re-escalate instantly; de-escalate one rung after 60 s below (threshold − 4 °C). Log
  transitions. If the fan is fitted, drive it as rung zero.

---

## 4. Second camera & the stereo question

**CAMSS hardware is not the constraint.** sc7280 CAMSS: 5× CSIPHY, 3× CSID (+2 lite), 3× VFE (3 RDI
each); CAM2 = csiphy2→csid0→vfe0_rdi0, CAM3 = disjoint csiphy3→csid1→vfe0_rdi1. Concurrent independent
streams are an explicit camss design feature (Qualcomm runs 5 cameras on RB3 Gen 2). Second RAW10 stream
≈ 60 MB/s DMA — trivial. A second GPU ISP instance ≈ +3.4 ms compute on a GPU at min clock (+1–2 °C).

**The blockers are in the code** (`q6a_camstream.py`): `/dev/video0` hardcoded in the capture path
(`--cam 3` reconfigures media-ctl but still opens video0); shm names `q6a_frame`/`q6a_ctrl` hardcoded on
both sides *and unlinked at init* (a second instance destroys the first's); `pkill -9 -f v4l2-ctl` kills
the sibling's helpers; `.npz` calibration path fixed. Parameterize all four, then live-verify CAM3 →
`/dev/video1` — the one unproven link.

**Stereo VSLAM — drop it.** Best published proxies (RPi5, 4×A76): ORB-SLAM3 stereo tracking 66–88
ms/frame ≈ ~3 sustained cores with mapping/loop threads; Basalt ~2 cores @30 fps; VINS-Fusion 2–3 cores.
That is +3–5 W on a board with ~1.5 W headroom, to duplicate localization the LDS LiDAR + odom + IMU +
Valetudo map already provide. Practitioner consensus for LiDAR-equipped indoor robots: add vision for
*semantics and off-plane obstacles*, not a second SLAM. **Substitute:** MiDaS-V2 on the NPU (official
**4.117 ms** on QCS6490, w8a8) with metric scale from LiDAR/floor plane; optionally a single-core VO-lite
(SMF-VO-class, <10 ms/frame) for the one real gap — LiDAR-parked manual-drive mode where the robot is
currently blind. Keep the second camera anyway (coverage/redundancy/future option).

**If real stereo is ever revived:** mainline `imx296.c` has no trigger mode, but the sensor's **XTRIG**
hardware sync is one shared GPIO/PWM line + a small patch to the self-built `imx296.ko` (InnoMaker
publishes the register pokes). An *unsynced* free-running pair has ~1° inter-view error at 1 rad/s
rotation — unusable under dynamics; ORB-SLAM3 assumes hardware-synced pairs.

---

## 5. NPU contention (YOLO + LLM) & a third client

Two HTP contexts coexist as processes (proven in production). **v68 has no preemption** — dispatched
work serializes FIFO. YOLO 10 Hz × 40 ms = 40% duty today; during LLM decode expect YOLO to stretch to
80+ ms and token cadence to jitter. Both tolerable at 0.3 m/s (10 Hz = 3 cm/cycle; 5 Hz still outruns
stopping distance) — but measure once, and add policy: optionally halve YOLO to 5 Hz while decoding,
coalesce LLM queries. A third HTP client (mono-depth) alongside detector + LLM daemon is plausible but
**untested** — verify on a clean boot with dmabuf-growth monitoring first, given the known one-process
GPU+NPU allocator corruption and the suspected crash-path leak.

---

## 6. ROS2 / Nav2 integration shape

**Keep frames out of DDS entirely.** A DDS image subscription costs serialize + copy + history
preallocation (~56 MB+ and ~half a Silver core per endpoint); rclpy pays per-message deserialization;
true zero-copy needs rclcpp + fixed-size types — the existing shm seqlock already beats all of it, and
the GPU+NPU one-process segfault forbids a composed container holding both anyway. **Right shape:** one
thin node reads the detection shm (after it gets a seq+timestamp protocol) and publishes
detections/pose/small crops (~KB/s); frames never enter DDS. Nav2 as a single composed rclcpp container
pinned to Silvers: Regulated Pure Pursuit, 0.1 m costmap, 5 Hz, `ROS_LOCALHOST_ONLY`, **skip
AMCL/slam_toolbox** (the Valetudo bridge supplies map→base_link). Budget ~1–1.5 cores, 300–500 MB.
Relocating `valetudo_bridge.py` to the Q6A comes first. Open: does Nav2 accept the bridge's 2 Hz TF
directly, or is an EKF (`robot_localization` fusing the tapped IMU) needed as a shim?

---

## 7. Planned detection upgrades

- **SAHI — skip.** At 1456×1088 with a 640 model: ~6 tiles + full frame ≈ 7 NPU passes ≈ 250–310
  ms/frame (3–4 fps), saturating the HTP the LLM shares. SAHI's proven gains (+5–7 AP) are on
  aerial/small-object benchmarks; for household objects at indoor ranges, 640×640 @10 Hz is the
  practitioner norm. If far-field recall ever matters: a single on-demand floor-band crop tile (~2×
  cost) on low-confidence frames.
- **YOLOv8-seg — gate.** Run one AI Hub export job to settle whether seg heads compile for qcs6490/v68
  (the IoT model pages contradict each other); expect +20–45% NPU time + CPU mask postprocessing; only
  bother if something consumes masks.
- **ByteTrack — add now.** Kalman + Hungarian on <50 objects is <1 ms/frame on a Silver core. Needs the
  seqlock + detection-channel fixes and detection timestamps first.

---

## 8. The 1B LLM, grounded

Measured numbers sit exactly at this chip's ceiling: Radxa's own path documents Llama-3.2-1B w4a16 at
**~100 tok/s prefill / 10–12 tok/s decode** (matches our ~12; ~9.8 post-adaptive-libGenie). Qualcomm AI
Hub lists this model **only for v73+ devices — QCS6490 absent** — corroborating "v68 = prebuilt ≤1B
only". Decode is DDR-bound everywhere (Rubik Pi 3 official llama.cpp: Qwen2-1.5B Q4_0 = 7.2 tok/s CPU /
9.6 GPU), so no backend buys speed; **NPU wins on perf/W and prefill (~5–8×)**.

**What a 1B is reliable for** (consistent across evals + practitioner reports): intent
classification/routing, constrained-choice answers, short summaries, structured extraction with a
*tight* schema. **Not** tool calling or multi-step agentics — a Llama-3B benchmark failed 9/9 tool-call
scenarios with confident wrong answers; the project's dropped-MCP-agent lesson is independently
validated. Notes:
- **Quantization tax is disproportionate at 1B scale**: w4a16 costs sub-2B models ~4–6 MMLU points, so
  the deployed model is meaningfully below its FP16 evals.
- **Gemma-3-1B is a much better instruction-follower** (IFEval 80.2 vs Llama-1B 59.5). No v68 Genie
  bundle exists, but llama.cpp CPU at ~10 tok/s with **GBNF grammar-constrained decoding** (which Genie
  cannot do) may be the better *constrained-JSON* workhorse for event-driven ROS handlers, keeping the
  NPU Llama for fast free-text. Quality ladder at this size: Qwen3-1.7B ≥ Qwen2.5-1.5B > Gemma-3-1B ≈
  Llama-3.2-1B > Qwen3-0.6B.
- **VLM/captioning:** no NPU path on v68. Moondream-0.5B or SmolVLM2-500M on CPU ≈ 5–15 s/caption —
  viable only event-triggered (0.1–0.2 Hz), not continuous.
- Loose thread: Quectel Pi H1 (QCS6490) docs claim `qai_hub_models` w8a8 LLM export works on this
  chipset — likely doc-rot from v73 boards, but one test export would settle whether a second NPU model
  path exists.

---

## 9. Power budget (184 Wh shared pack)

**Reconciliation flag:** docs record "~6 W idle, ~12 h to dead" = ~72 Wh — matching the **stock
5200 mAh pack** (74.9 Wh), not 184 Wh. If the pack was upgraded to 12800 mAh, runtime scales ~2.5×;
confirm, since all runtime math hinges on it.

Best current estimates (board-side; buck ~87% → battery-side ×1.15; every number here is a **model, not
a measurement** — the board has zero power telemetry):

| Subsystem | Est. W | Basis |
|---|---|---|
| Q6A idle (resident LLM, GNOME still on) | ~4 | Radxa Q6A wall measurement; 61–66 °C idle consistent |
| Q6A current pipeline (ISP+YOLO+MJPEG) | ~6.5–7 | +11–13 °C over idle ≈ +2.5–3 W at ~8–9 °C/W |
| LLM decode burst | +2.5–3.5 | sustained NPU 1B ≈ 80 °C ≈ 3–4 W NPU |
| Planned full stack (2 cams, seg, Nav2, episodic LLM) | ~8.5–10.5 | **at/above the ~5–6 W passive envelope** |
| Robot base idle (post-ondemand) | ~3–4 [guess] | pre-fix measured ~6 W at pinned governor |
| LiDAR turret spinning | ~1.5–2.5 [guess] | typical LDS |
| Drive motors (0.3 m/s, fan off) | ~10–15 [guess] | vacuum-class gearmotors |

**Scenarios (~150 Wh usable):** parked-sentry ~13 h (optimized headless/event-driven ~16–17 h); patrol
at 30% motion duty ~8–9 h.

**Actions:** (a) buy a USB-C PD inline meter; profile idle / +ISP / +YOLO / +LLM / all — one afternoon
replaces ±40% error bars. (b) **Test dock pass-through** — if the Dreame charger sustains net-positive
with ~8 W parasitic Q6A load, docked-sentry is indefinite and battery sizing stops mattering for the
primary use case; never tested. (c) **Brownout:** a 4S pack sags toward ~12 V empty; verify the
10–30 V-input PD module truly regulates 12 V out at 12 V in (a real buck-boost will; a plain buck hits
dropout → PD collapse → hard cut). Add a daemon watching battery SoC via the bridge that does a clean
poweroff below ~20% — an unclean cut risks the ext4 root and a cDSP wedge.

---

## 10. Verdicts on the brief's §10 challenge areas

1. **Seqlock — NOT airtight; protocol flaw, not memory-ordering.** Writer does `frame[:] = rgb` *then*
   `fseq += 1` (`q6a_camstream.py:582-583`); reader copies then re-checks seq
   (`q6a_detector.py:67-69`). A reader landing mid-write sees the same seq on both sides of a torn copy
   — ~2 ms window per ~60 ms period → **torn inferences already happen occasionally**. Harmless for
   boxes; disqualifying for ByteTrack/SLAM. Fix ≈ 20 lines: odd/even seqlock (bump to odd before write,
   even after; reader retries on odd/changed) or two-slot buffer + index flip. CPython/aarch64 fence
   pedantry is ns-scale — ignore. Also: the detection *return* channel has no protocol at all (`dseq`
   written but never read; streamer reads `dcnt`/`dbuf` unguarded).
2. **White-patch AWB** — insufficient as a universally robust algorithm (coloured LEDs, no-neutral
   scenes) but the right engineering call: cosmetic, opt-in, off in headless, YOLO cast-robust. Stop
   investing. Rule for stereo: never per-camera *auto* WB on a pair — lock both to identical fixed gains.
3. **Shadow green comp** — acceptable (videoconference-grade compromise; off in headless). If
   colour-based classification ever appears: turn it off, don't tune it.
4. **MJPEG — right; the bandwidth worry is unfounded**: 70 KB/frame × 16 fps ≈ 1.1 MB/s ≈ 10% of even
   the USB-gadget ceiling (and the Q6A stream rides GbE/WiFi anyway). Venus H.264 = board-resetting
   codec + physical UEFI access for a bandwidth-only win — skip. **In production, detections travel
   separately** (ROS topic, ~KB/s); video stays a client-gated debug tap.
5. **16 vs 32 fps — wrong dichotomy; make it mode-dependent.** Stationary: keep `max_exposure=6000`
   (YOLO is 10 Hz-capped; extra frames are discarded work; low noise buys recall). Moving/rotating:
   clamp AE ceiling to ~2000–3000, let gain compensate — 60 ms integration at 1 rad/s ≈ 3.5°/frame
   motion blur, poison for future VO, bad for detection during turns. Key off cmd_vel/status.
6. **Headless — right call, two gaps**: sync ISP/publish to `YOLO_FPS` (free 40–60% GPU+memcpy cut —
   today 16–32 fps demosaic feeds a 10 fps consumer), and the thermal guard is mandatory (§3). Also
   strip GNOME/gdm/docker/fwupd/cups.
7. **GPU latency-bound; fp16/zero-copy skipped — correct today**; revisit at dual-camera where sustained
   cadence may amortize the wakeup; A/B mid-OPP pinning (450/550, not 812) then, not now.

---

## 11. Code findings, ranked by likelihood × impact

1. **No supervision + shm lifecycle** — worst combined risk. Detector `Popen` never polled → dead
   detector = frozen boxes forever (no staleness cutoff; `dcnt` not zeroed on error). Streamer SIGKILL →
   orphan detector spinning 250 wakeups/s *holding the NPU context* (cdsp-wedge risk = reboot). Python
   3.12 (Ubuntu 24.04): `SharedMemory` attach registers with the resource_tracker even without
   `create=True` — detector exit **unlinks the live shm under the streamer** (3.13 fixes via
   `track=False`; 3.12 needs the workaround). → systemd `Restart=on-failure` + clean SIGTERM releasing
   the QNN context.
2. **MJPEG stalled-client hang** (`q6a_camstream.py:750-768`) — no socket timeout; a suspended
   laptop/NAT half-open blocks the handler thread forever, `State.clients` never decrements → **camera +
   GPU pinned on indefinitely on a battery robot**. Fix: `settimeout(10)`.
3. **`read_latest` swallows all OSError as EAGAIN** (`q6a_v4l2.py:64-67`) — device error (EIO/ENODEV)
   with select still firing → 100% CPU busy-spin, vision dead, no recovery. Check `errno`, re-raise (the
   existing reinit path then engages).
4. **`--fast` + YOLO is a guaranteed crash loop** — half-res CPU frames vs full-res shm dims →
   `ValueError` every frame; GPU-init failure *falls back into this same path*, so a driver regression
   becomes total silent vision loss.
5. **AE via `subprocess.run(v4l2-ctl, check=False)`** — a failing set-ctrl is invisible → module
   EXPOSURE/GAIN state diverges from hardware, AE acts on fiction; plus 10–20 ms fork per adjust in the
   capture thread. Replace with a held-fd `VIDIOC_S_CTRL` ioctl with checked returns. (Control law
   itself stable: deadband + damping converge, no oscillation.)
6. **Torn-frame seqlock + unguarded detection channel** (§10.1) — fix together, before ByteTrack.
7. **Second-camera landmines** (§4) + **frame-size assert**: capture accepts only
   `len(data)==FRAME` from self-computed stride, ignoring driver `sizeimage` — any padding mismatch on a
   new sensor/resolution = silent 0 fps. Assert `cam.frame_size == FRAME` at open.

Client-gating reviewed clean (increment-before-wake, predicate re-check — no lost wakeup). Nits:
`hash(label)` per-run-randomized box colors; detector dim fallback crashes on a `--bin`-sized shm if
dims unpublished; GPU `d_in` sized W*H*2 only safe for ≤10-bit packed profiles; delete the file-tail
fallback (580 MB `/dev/shm` writes per batch, and it is the crash-loop path).

---

## 12. Priority order

**Week 1 — correctness (mostly one-liners):** odd/even seqlock + detection-channel seq/timestamp;
systemd supervision + clean SIGTERM + staleness cutoff; MJPEG socket timeout; EAGAIN re-raise;
frame-size assert; fix-or-delete `--fast`+YOLO; rename the `yolov11_det` label.

**Before adding any load:** thermal governor (the NPU has no other throttle); reboot → re-measure the
5.6 GB pinned-memory gap (watch per-run growth); strip GNOME/docker/fwupd/cups; PD power meter →
per-workload watts; in-compartment (lid-closed) thermal measurement; fit the fan/heatsink; fix the
Prime-core pin + cpu6 hotspot.

**Roadmap:** profile the YOLO w8a8/quantized-input path (likely 2–3× NPU headroom for free);
ISP-at-detector-cadence in headless; parameterize + bring up CAM3; ByteTrack; NPU mono-depth (after
testing 3-process accelerator coexistence); relocate valetudo_bridge → composed Nav2; mode-dependent AE
ceiling; dock pass-through test; brownout daemon; one AI Hub export job for YOLO-seg-on-v68; update
CLAUDE.md constants (12 GB, ~22/15 GB/s).

**Open questions for the project owner:**
1. Were the 72–74 °C measurements bench or in-compartment, and what was ambient?
2. Was the battery upgraded from stock 5200 mAh? ("12 h at ~6 W" matches 75 Wh, not 184 Wh.)
3. Is the Q6A already on the robot's 12 V buck, or bench-powered? Which buck module exactly — does it
   regulate 12 V out at 12 V in?
4. Does the dock charger sustain net-positive charge with the Q6A drawing ~8 W?
5. What consumes detections in headless production — is the ROS bridge node the intended `q6a_ctrl`
   consumer, and does Nav2 accept the bridge's 2 Hz TF directly (or EKF shim)?
6. What requirement drives SAHI + seg? (Small far objects? Floor debris? Pixel masks?) Default answer
   is "don't".
7. Is stereo wanted for depth/obstacles or localization? If depth: mono-depth + LiDAR scale answers it.
8. Which QCS6490 variant (Tj 95 °C standard vs 105 °C extended 300-AA) is on the Q6A?

---

## 13. Key sources

**QCS6490 power/thermal:** Qualcomm QCS6490/QCS5430 datasheet 80-23889-1 Rev AW (6.9 W Dhrystone max,
Tj 95/105 °C) — dl.radxa.com/q6a/hw/datasheets; Rubik Pi 3 FAQ (~4 W CPU-only, ~8 W CPU+GPU+DSP) —
rubikpi.ai/faq + community.rubikpi.ai/t/277; Radxa Q6A measured idle ~4 W / stress ~10 W + fan data —
smarthomecircle.com/radxa-dragon-q6a-best-alternative-to-the-raspberry-pi-5; sc7280 DT trip points —
github.com/torvalds/linux v6.12 `sc7280.dtsi`; OnLogic Factor 101 fanless QCS6490 —
cnx-software.com 2026-02-23; Radxa Q6A product brief Rev 1.5 (LPDDR5 5500 MT/s).

**1B LLMs:** Radxa Q6A NPU docs (Llama-3.2-1B ~100 prefill / 10–12 decode) — docs.radxa.com;
Thundercomm Rubik Pi 3 llama.cpp benchmark (Qwen2-1.5B Q4_0: CPU 7.15, GPU 9.61 tok/s);
huggingface.co/qualcomm/Llama-v3.2-1B-Instruct (v73+ only — QCS6490 absent); quantization tax at small
scale — arxiv.org/html/2505.02214; NPU-vs-CPU/GPU decode bandwidth-bound — arxiv.org/html/2410.03613;
1B tool-calling failure — dev.to (Llama-3B 9/9 fail); Gemma-3-1B IFEval 80.2 — embedl.com, llm-stats.com;
edge VLMs — learnopencv.com/vlm-on-edge-devices, moondream.ai/blog/introducing-moondream-0-5b.

**Stereo/VSLAM:** SMF-VO RPi5 benchmarks (ORB-SLAM3 65.7–88.3 ms, Basalt 14.3–32 ms) —
arxiv.org/html/2511.09072; Jetson VIO CPU benchmark — arxiv.org/abs/2103.01655; qcom-camss concurrency
— docs.kernel.org/admin-guide/media/qcom_camss.html + patchwork linux-media camss sc7280 series +
docs.qualcomm.com 80-70023-17 (5-camera streaming); IMX296 XTRIG — docs.arducam.com global-shutter
external-trigger + github.com/INNO-MAKER/cam-imx296raw-trigger; MiDaS-V2 4.117 ms QCS6490 —
huggingface.co/qualcomm/Midas-V2; LiDAR+vision practitioner guidance — kudan.io blog,
arxiv.org/html/2501.09490.

**SAHI/seg/tracking:** SAHI paper — arxiv.org/abs/2202.06934; docs.ultralytics.com/guides/
sahi-tiled-inference; AI Hub yolov8_det/yolov8_seg/yolov11_seg IoT pages (contradictory qcs6490
listings — verify by export); ByteTrack cost — arxiv.org/pdf/2510.09653 et al.

**ROS2:** RSS/instrumented per-entity costs — discourse.openrobotics.org/t/21206; image transport CPU —
answers.ros.org/question/312964; FastDDS SHM — fast-dds.docs.eprosima.com; Nav2 tuning/composition —
docs.nav2.org/tuning; RMW reports — osrf.github.io/TSC-RMW-Reports.

*Full evidence bundle (13 agent reports incl. the live probe raw numbers) generated 2026-07-06; this
document is the durable synthesis.*

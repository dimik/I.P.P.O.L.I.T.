I'll review the video pipeline and Q6A architecture. Let me start by reading the review brief and scouting the relevant docs, then fan out research and review agents.

Still running — the gather agents (web research + live board probe) take a while. Waiting for completion.

30 minutes in — let me check which agents have finished rather than blocking blindly.

# IPPOLIT video pipeline & Q6A architecture — review

**Method:** I read the review brief and the pipeline source, ran a live probe of the Q6A over the wired link while your pipeline was running, fanned out web research (QCS6490 power/thermal primary sources, 1B-LLM benchmarks, embedded VSLAM, SAHI/ByteTrack, ROS2 footprints), and ran four independent analysis passes over the combined evidence (13 agents, ~230 tool calls). Everything below is grounded in either your code/docs, live measurements from tonight, or cited external data.

## TL;DR verdict

The core architecture — RDI raw → single-kernel GPU ISP → shm → NPU YOLO, two processes because of the dma-heap segfault — is **sound and well-reasoned**; every major judgment call in the brief survives scrutiny. The live probe confirms it's cheap: the whole pipeline plus resident LLM costs **~1.0 CPU core, GPU at its *lowest* OPP (315 MHz), 71–78 °C, zero throttle events**. Your headroom problem is not compute — it's **thermals (~12–17 °C to the 90 °C floor ≈ ~1.5 W of additional sustained dissipation, passively) and memory bandwidth**, and two of your planning constants are wrong: the board is **12 GB, not 16 GB**, and the DDR budget is **~22 GB/s theoretical / ~15 GB/s practical, not 40–50 GB/s**.

Of your concurrency wishlist: **second camera — yes** (hardware confirmed capable; the blockers are all in your code). **ROS2 + Nav2 — yes** (~1–1.5 cores, well under a gigabyte done right). **Resident 1B LLM — yes with scheduling** (it's already free at idle; decode contends for DDR + the HTP). **Full stereo VSLAM — no, and you shouldn't want it**: 2–3 sustained big cores (+3–5 W) on a board with ~1.5 W of passive headroom, duplicating localization your LiDAR already provides. NPU mono-depth (MiDaS-V2 is an official **4.1 ms** on QCS6490) + LiDAR scale gets most of the value at ~zero cost. **SAHI — skip it** (7 NPU passes ≈ 250–310 ms/frame, starves the HTP you share with the LLM, and it's designed for aerial small-object scenes, not a living room). **ByteTrack — add it now**, it's <1 ms/frame, but fix the seqlock first because it's not actually airtight (details below).

The single cheapest enabler in your whole roadmap is a **25 mm fan or lid-as-heatsink off the 12 V rail (~0.3–0.5 W)**: community data on this exact SoC shows it moves the envelope **15–25 °C**, which is the difference between "concurrency plan is thermally thin" and "comfortable" — and the NPU zone has **no kernel throttle path on mainline** (hot-notify at 90 °C, then straight to 110 °C PMIC power-off, which you have already hit once), so software duty-cycling + cooling are the *only* NPU mitigations that exist.

---

## 1. Live ground truth (tonight, pipeline running: `--cam 2 --gpu --bin --awb`, 1 MJPEG client)

| What | Measured |
|---|---|
| camstream / detector CPU | 52.6% / 39.0% of one core → **≈1.0 core total**, mostly Silvers |
| Throughput | 16 fps publish, YOLO ~10 Hz, `model_inference` 38.4–44.5 ms + **5.2–15.3 ms `copyFromFloatToNative`** |
| Temps | CPU 71–78 °C (cpu6 hottest at 78.3), GPU 67–68 °C, NPU 67–69 °C, DDR ~72 °C; **zero throttle events this boot** |
| GPU devfreq | **315 MHz — the lowest OPP — throughout**; `simple_ondemand` never ramps under OpenCL load |
| LLM daemon | RSS 95.6 MB + 1.78 GB dmabuf weights; **1.34 s total CPU over 18 h** — the adaptive-libGenie fix genuinely achieved zero idle cost |
| Memory | **12,016 MB physical (12 GB board)**; MemAvailable only **2.84 GB** — ~5.6 GB is kernel-pinned and unaccounted (suspected fastrpc/dma-heap leakage from the day's crashed GPU+NPU experiments) |
| Anomalies | Prime core pinned at 2.707 GHz at ~9% util (13,835 s at max freq this boot — wasted heat); full GNOME/gdm + docker + fwupd + cups still running (~500 MB+ RSS) |
| ROS2 | **No nodes running.** Daemon alone: 69.8 MB RSS, ~2% CPU. The "ROS2 running" premise in your request is really "ROS2 installed" — its real footprint with Nav2 is still unmeasured on this board |

Two planning corrections to make in CLAUDE.md and the brief: **12 GB RAM** (not 16), and **~22 GB/s theoretical / ~15 GB/s practical DDR** (Radxa ships 5500 MT/s on a 2×16 bus; measured single-core memcpy is 7.6–8.4 GB/s). The 40–50 GB/s figure is ~2× reality, and it changes conclusions: **1B LLM decode alone needs ~9–10 GB/s effective** — half the practical bus — which is exactly why your GPU llama.cpp measured the same ~12 tok/s as the NPU, and why you should expect visible ISP/YOLO frame-time jitter whenever the LLM is decoding.

## 2. Performance & thermal

**Bottleneck hierarchy (confirmed):** (1) sensor exposure/VMAX is the fps ceiling — everything else has slack; (2) the GPU ISP is genuinely latency-bound (~3.4 ms real work vs ~10 ms with the ~6.6 ms Adreno power-gate wakeup — corroborated by devfreq never leaving 315 MHz), so skipping fp16/zero-copy was right *today*, with the caveat that this regime **expires under dual-camera load**, where interleaved frames keep the GPU awake and may amortize the wakeup for free — re-A/B 315 vs 450/550 MHz pinning at CAM3 bring-up, not before; (3) CPU is nearly idle; (4) the NPU is the least understood resource and where the cheapest win hides.

**The YOLO 3× gap is your best optimization target.** You measure 38–44 ms; Qualcomm's ecosystem numbers for YOLOv8-class w8a8 on this NPU are ~12 ms. Your export is **w8a16 with float I/O** — the appbuilder quantizes float→uint16 per call (`copyFromFloatToNative`, 5–15 ms, ~25% of your budget). Testing a w8a8 export and a quantized-input tensor path could roughly halve-to-third per-inference cost — same 10 Hz at ~15% HTP duty instead of ~40%, freeing thermal and LLM headroom. That's worth more than any model upgrade you're considering.

**Thermal headroom, quantified:** fitting your measured points gives ~8–9 °C/W for this passive setup; from 78 °C (hottest zone) to the 90 °C floor is **~1.5 W of additional sustained dissipation**. What fits in that: a second camera + ISP (+0.3–0.5 W), ByteTrack (~0), mono-depth (~0.2 W), YOLO at higher duty (+0.5–1 W). What doesn't: CPU stereo SLAM (+2.5–4 W), sustained SAHI+LLM (+2–3 W). Two caveats: your 72–74 °C figures appear to be **bench-side, not in the closed compartment** (the robot was offline during the Q6A sessions) — expect +5–10 °C enclosed, which eats half to all of that budget, so measure lid-closed before committing; and the official Tj spec is 95 °C (standard variant), so the 90–110 °C band is already marginal silicon territory.

**Thermal governor — build it now, it's load-bearing, not polish.** Mainline sc7280 DT gives CPU zones passive trips at 90 °C and GPU at 95 °C, but **nspss (NPU) zones have only hot-notify at 90 °C and critical 110 °C** — no cooling device is bound. Your 3B-GPU shutdown was this exact path. Concrete ladder (bench-calibrated, shift −5 °C after in-compartment measurement): <78 °C normal → 78–84 °C: YOLO to 5 fps, ISP at detector cadence, queue LLM requests → 84–88 °C: YOLO 2 fps, force `--bin`, refuse LLM, drop MJPEG clients → ≥88 °C: park the detector *cleanly* (release the QNN context — an orphaned NPU client wedges the cDSP until reboot) → ≥95 °C: orderly SIGTERM of everything, because a clean shutdown beats the PMIC yanking power. Re-escalate instantly, de-escalate one rung after 60 s of hysteresis. If you fit the fan, drive it as rung zero.

## 3. Concurrency feasibility

| Component | Verdict | Binding constraint |
|---|---|---|
| 2nd IMX296 (capture + GPU ISP) | **Fits, with conditions** | Your code, not hardware — see below |
| Stereo VSLAM | **Does not fit** | CPU-thermal; also strategically redundant |
| ROS2 Jazzy + Nav2 | **Fits** | None at your rates (~1–1.5 cores, 0.3–1.5 GB) |
| Resident 1B NPU LLM | **Fits, with conditions** | DDR bandwidth during decode; HTP timesharing |
| All of the above together | **Fits only if** stereo→mono-depth, leak resolved, NPU scheduling + thermal guard added | |

**Second camera:** sc7280 CAMSS has 5× CSIPHY, 3× CSID (+2 lite), 3× VFE with 3 RDI each; CAM2 uses csiphy2→csid0→vfe0_rdi0 and CAM3 maps to disjoint csiphy3→csid1→vfe0_rdi1 — concurrent independent streams are an explicit design feature (Qualcomm runs 5 cameras on RB3 Gen 2). A second RAW10 stream is ~60 MB/s DMA — trivial. The real blockers are in `q6a_camstream.py`: **`/dev/video0` is hardcoded in the capture path** (`--cam 3` reconfigures media-ctl but then still opens video0), shm names `q6a_frame`/`q6a_ctrl` are hardcoded on both sides *and unlinked at init* (a second instance destroys the first's), `pkill -9 -f v4l2-ctl` kills the sibling's helpers, and the `.npz` calibration path is fixed. Parameterize all four, then live-verify CAM3 → `/dev/video1` — the one unproven link.

**Stereo VSLAM — drop it.** Best published proxies (RPi5, 4×A76): ORB-SLAM3 stereo tracking 66–88 ms/frame ≈ 3 sustained cores with mapping/loop threads; Basalt ~2 cores at 30 fps; that's +3–5 W on a board with ~1.5 W of headroom, to duplicate localization the LDS LiDAR + wheel odom + IMU + Valetudo map already give you. Practitioner consensus for LiDAR-equipped indoor robots is to add vision for *semantics and off-plane obstacles*, not a second SLAM. The substitute that captures ~90% of the value: **MiDaS-V2 on the NPU (official 4.117 ms on QCS6490, w8a8)** with metric scale from the LiDAR/floor plane, plus optionally a single-core VO-lite (SMF-VO-class, <10 ms/frame) for the one real gap — the LiDAR-parked manual-drive mode where the robot is currently blind. Keep the second camera anyway (coverage/redundancy/future option). If you ever do real stereo: mainline `imx296.c` has no trigger mode, but the sensor's **XTRIG** hardware sync is one shared GPIO/PWM line + a small patch to your self-built `imx296.ko` (InnoMaker publishes the register pokes) — an *unsynced* free-running pair is ~1° of inter-view error at 1 rad/s rotation, unusable under dynamics.

**NPU contention (YOLO + LLM):** two HTP contexts coexist as processes (proven by your current deployment), but v68 has no preemption — dispatched work serializes FIFO. YOLO at 10 Hz × 40 ms = 40% duty today; during LLM decode expect YOLO to stretch to 80+ ms and token cadence to jitter. Both are tolerable at 0.3 m/s (10 Hz = 3 cm/cycle; even 5 Hz outruns your stopping distance) — but measure it once, and add a simple policy: optionally halve YOLO to 5 Hz while decoding, coalesce LLM queries. A third HTP client (mono-depth) alongside the LLM daemon and detector is plausible but **untested** — verify on a clean boot with dmabuf-growth monitoring before architecting around it, given the known one-process GPU+NPU allocator corruption and the suspected crash-path leak.

**ROS2/Nav2 shape:** keep frames **out of DDS entirely** — your shm seqlock already beats anything image_transport can do (a DDS image subscription costs serialize+copy+history preallocation, ~56 MB+ and ~half a Silver core per endpoint; true zero-copy needs rclcpp with fixed-size types). One thin node bridges the detection shm to `detections/pose` topics (KB/s). Run Nav2 as a single composed rclcpp container pinned to Silvers: Regulated Pure Pursuit, 0.1 m costmap, 5 Hz, `ROS_LOCALHOST_ONLY`, **skip AMCL/slam_toolbox** — the Valetudo bridge already supplies `map→base_link`. Budget ~1–1.5 cores, 300–500 MB. Relocating `valetudo_bridge.py` to the Q6A is still pending and should come first. One open design question: whether Nav2 accepts the bridge's 2 Hz TF directly or wants an EKF (`robot_localization` fusing the tapped IMU) as a shim.

## 4. Planned detection upgrades

- **SAHI — skip.** At 1456×1088 with a 640 model you'd run ~6 tiles + full frame ≈ 7 NPU passes ≈ 250–310 ms/frame (3–4 fps), saturating the HTP the LLM shares. SAHI's proven gains (+5–7 AP) are on aerial/small-object benchmarks; for household objects at indoor ranges, 640×640 at 10 Hz is already the practitioner norm. If far-field recall ever matters, use a single on-demand floor-band crop tile (~2× cost) on low-confidence frames.
- **YOLOv8-seg — gate it.** First run one AI Hub export job to settle whether seg heads compile for qcs6490/v68 (the docs contradict each other); expect +20–45% NPU time plus CPU mask postprocessing, and only bother if something actually consumes masks.
- **ByteTrack — add now.** Kalman + Hungarian on <50 objects is <1 ms/frame on a Silver core; effectively free. But it needs the seqlock and detection-channel fixes first (below), plus timestamps on detections.

## 5. The 1B LLM, grounded

Your measured numbers are exactly at the ceiling of what this chip does: Radxa's own path documents Llama-3.2-1B w4a16 at **~100 tok/s prefill / 10–12 tok/s decode**, matching your ~12 (9.8 post-patch). Qualcomm's AI Hub lists this model *only* for v73+ devices — QCS6490 absent — corroborating your "v68 = prebuilt ≤1B only" finding. Decode is DDR-bound everywhere (llama.cpp on the Rubik Pi 3: Qwen2-1.5B Q4_0 at 7.2 tok/s CPU / 9.6 GPU), so no backend buys you speed; the NPU wins on perf/W and prefill.

**What a 1B is actually good for** (consistent across evals and practitioner reports): intent classification/routing, constrained-choice answers, short summaries, structured extraction with a *tight* schema. **Not** tool calling or multi-step agentics — a Llama-3B benchmark failed 9/9 tool-call scenarios with confident wrong answers; your dropped-MCP-agent lesson is independently validated. Two things worth knowing: the **quantization tax is disproportionate at this scale** (w4a16 costs sub-2B models ~4–6 MMLU points, so your deployed model is meaningfully below its FP16 evals), and **Gemma-3-1B is a much better instruction-follower** (IFEval 80.2 vs Llama-1B's 59.5) — no v68 Genie bundle exists, but llama.cpp CPU at ~10 tok/s with **GBNF grammar-constrained decoding** (which Genie can't do) may be the better *constrained-JSON* workhorse for event-driven ROS handlers, keeping the NPU Llama for fast free-text. For captioning: no VLM has an NPU path on v68; Moondream-0.5B or SmolVLM2-500M on CPU at ~5–15 s/caption is viable only as event-triggered (0.1–0.2 Hz), not continuous.

## 6. Power budget (184 Wh shared)

First, a reconciliation: your docs record "~6 W idle, ~12 h to dead" — that's ~72 Wh, which matches the **stock 5200 mAh pack** (74.9 Wh), not the 12800 mAh/184 Wh you quote now. If the pack was upgraded, runtime scales ~2.5×; please confirm, because all runtime math hinges on it.

Best current estimates (board-side; buck ~87% → battery-side ×1.15): Q6A ~4 W idle-with-resident-LLM, ~6.5–7 W current pipeline, +2.5–3.5 W during LLM decode bursts; planned full stack (2 cams, seg, Nav2, episodic LLM) ~8.5–10.5 W — **at or above the ~5–6 W passive envelope**, which is the strongest argument for the fan. Robot base: ~3–4 W idle post-ondemand [unmeasured], LiDAR ~2 W spinning, motors ~10–15 W moving. Scenarios on ~150 Wh usable: **parked sentry ~13 h** (optimized headless/event-driven ~16–17 h); **patrol at 30% motion duty ~8–9 h**. Every number marked here is a model, not a measurement — the board exposes **zero power telemetry** (no power_supply nodes), so a $15 USB-C PD inline meter measuring idle/+ISP/+YOLO/+LLM/all is one afternoon that replaces ±40% error bars with facts.

Two flags: (a) **dock pass-through is the highest-leverage unknown** — if the Dreame charger sustains net-positive with an ~8 W parasitic Q6A load, docked-sentry runtime is indefinite and battery sizing stops mattering for your primary use case; nobody has tested it. (b) **Brownout**: a 4S pack sags toward ~12 V empty; verify your 10–30 V-input PD module truly regulates 12 V out at 12 V in (a genuine buck-boost will; a plain buck hits dropout and hard-cuts the Q6A). Add a daemon that watches battery SoC via the bridge and does a clean poweroff below ~20% — an unclean cut risks the ext4 root and a cDSP wedge.

## 7. Answers to your seven challenge areas (§10 of the brief)

1. **Seqlock — not airtight, and it's a protocol flaw, not memory-ordering.** The writer does `frame[:] = rgb` *then* `fseq += 1` ([q6a_camstream.py:582–583](scripts/companion/camera/q6a_camstream.py:582)); the reader copies then re-checks seq ([q6a_detector.py:67–69](scripts/companion/camera/q6a_detector.py:67)). A reader landing mid-write sees the same seq on both sides of a torn copy — ~2 ms window per ~60 ms period, so **torn inferences already happen occasionally**. Harmless for boxes; disqualifying for ByteTrack/SLAM. Fix is ~20 lines: odd/even seqlock (bump to odd before write, even after; reader retries on odd/changed) or a two-slot buffer + index flip. The CPython/aarch64 fence pedantry is real but ns-scale — ignore it. Separately, the detection *return* channel has no protocol at all: `dseq` is written but never read; the streamer reads `dcnt`/`dbuf` unguarded.
2. **White-patch AWB** — insufficient as a universally robust algorithm (coloured LEDs, no-neutral scenes), but the right engineering call because you made it cosmetic and it's off in headless. Stop investing. One rule if stereo ever happens: never run *per-camera auto* WB on a pair — lock both to identical fixed gains; photometric consistency beats correctness.
3. **Shadow green comp** — acceptable; a normal videoconference-grade compromise, and YOLO never sees it. If you ever do colour-based classification, turn it off rather than tune it.
4. **MJPEG transport — right, and the bandwidth worry is unfounded**: 70 KB/frame × 16 fps ≈ 1.1 MB/s ≈ 10% of even the USB-gadget ceiling (and the Q6A stream rides the GbE/WiFi path anyway). Venus H.264 would buy bandwidth you don't need at the cost of a board-resetting codec and physical UEFI access. But yes — **in production, detections travel separately** as a ROS topic; video stays a client-gated debug tap.
5. **16 vs 32 fps — wrong dichotomy; make it mode-dependent.** Stationary: keep `max_exposure=6000` (YOLO is 10 Hz-capped; extra frames are discarded work; low noise buys recall). Moving/rotating: clamp the AE ceiling to ~2000–3000 and let gain compensate — 60 ms integration at 1 rad/s is ~3.5°/frame of motion blur, poison for any future VO and bad for detection during turns. You already have the cmd_vel/status signal to key on.
6. **Headless config — right call, two gaps**: sync ISP/publish to `YOLO_FPS` (free 40–60% GPU+memcpy cut — today you demosaic 16–32 fps into a 10 fps consumer), and the thermal guard is mandatory, not optional (see §2). Also strip GNOME/gdm/docker/fwupd/cups — you're running a desktop on a robot brain.
7. **GPU latency-bound, fp16/zero-copy skipped — correct today**, revisit at dual-camera where sustained cadence may amortize the wakeup; A/B mid-OPP pinning (450/550, not 812) for 30 minutes then, not now.

## 8. Code findings, ranked by likelihood × impact

1. **No supervision + shm lifecycle** — the worst combined risk. `Popen` for the detector is never polled; a dead detector = frozen boxes forever (no staleness cutoff, `dcnt` never zeroed on error). SIGKILL of the streamer orphans the detector spinning at 250 wakeups/s *while holding the NPU context* (cdsp-wedge risk = reboot). And on Python 3.12 (Ubuntu 24.04), `SharedMemory` attach registers with the resource_tracker even without `create=True` — a detector exit **unlinks the live shm under the streamer** (fixed in 3.13 via `track=False`; on 3.12 you need the workaround). Put both processes under systemd `Restart=on-failure` with clean SIGTERM handlers that release the QNN context.
2. **MJPEG stalled-client hang** ([q6a_camstream.py:750–768](scripts/companion/camera/q6a_camstream.py:750)) — no socket timeout; a suspended laptop/NAT half-open blocks the handler thread forever and `State.clients` never decrements → **camera+GPU pinned on indefinitely on a battery robot**. One line: `settimeout(10)`.
3. **`read_latest` swallows all OSError as EAGAIN** ([q6a_v4l2.py:64–67](scripts/companion/camera/q6a_v4l2.py:64)) — a device error (EIO/ENODEV) with select still firing → 100% CPU busy-spin, vision dead, no recovery. Check `errno`, re-raise; the existing reinit path then engages.
4. **`--fast` + YOLO is a guaranteed crash loop** — half-res CPU frames vs full-res shm dims → `ValueError` every frame; and GPU-init failure *falls back into this same path*, so a driver regression becomes total silent vision loss.
5. **AE via `subprocess.run(v4l2-ctl, check=False)`** — a failing set-ctrl is invisible, so the module's EXPOSURE/GAIN state can diverge from hardware and the AE law acts on fiction; plus a 10–20 ms fork per adjustment in the capture thread. Replace with a held-fd `VIDIOC_S_CTRL` ioctl with checked returns. (The control law itself is stable — deadband + damping converge, no oscillation expected.)
6. Torn-frame seqlock + unguarded detection channel (§7.1) — fix together, before ByteTrack.
7. Second-camera landmines (§3) and a frame-size assert (`cam.frame_size == FRAME` at open — today a padding mismatch on any new sensor/resolution silently drops every frame at 0 fps with no error).

Client-gating itself reviewed clean (increment-before-wake, predicate re-check — no lost wakeup); the `clients += 1` GIL nit I flagged mid-review is real but negligible next to #2.

## 9. Priority order

**This week (correctness, mostly one-liners):** odd/even seqlock + detection-channel seq/timestamp; systemd supervision + clean SIGTERM + staleness cutoff; MJPEG socket timeout; EAGAIN check; frame-size assert; fix-or-delete `--fast`+YOLO; rename the `yolov11_det` label.

**Before adding any load:** thermal governor (the NPU has no other throttle); reboot and re-measure the 5.6 GB pinned-memory gap (and watch per-run growth); strip GNOME/docker/fwupd/cups; buy a PD power meter and profile per-workload watts; measure in-compartment thermals lid-closed; fit the fan/heatsink; fix the Prime-core pin and cpu6 hotspot.

**Roadmap:** profile the YOLO w8a8/quantized-input path (likely 2–3× NPU headroom for free); ISP-at-detector-cadence in headless; parameterize + bring up CAM3; ByteTrack; mono-depth on NPU (after testing 3-process accelerator coexistence); relocate valetudo_bridge, then composed Nav2; mode-dependent AE ceiling; test dock pass-through; brownout daemon; one AI Hub export job to settle YOLO-seg-on-v68; update CLAUDE.md constants (12 GB, ~22/15 GB/s).

## 10. Open questions for you

1. **Were the 72–74 °C measurements bench-side or in the closed compartment, and what was ambient?** This is the single biggest unknown in the thermal budget.
2. **Was the battery upgraded from the stock 5200 mAh?** The documented "12 h at ~6 W" matches a 75 Wh pack, not 184 Wh.
3. **Is the Q6A already wired to the robot's 12 V buck, or still bench-powered?** (CLAUDE.md lists integration as TODO.) And which buck module exactly — does it regulate 12 V out at 12 V in?
4. **Does the dock charger sustain net-positive charge with the Q6A drawing ~8 W?** If yes, docked-sentry is indefinite and most battery math stops mattering.
5. **What consumes detections in headless production today?** Nothing reads `q6a_ctrl` in headless — is the planned ROS bridge node the intended consumer, and does Nav2 accept the bridge's 2 Hz `map→base_link` TF directly, or do you want an EKF with the tapped IMU?
6. **What's driving SAHI-seg on your requirements list?** Nothing in the docs states the use case (small far objects? floor-level debris? pixel-accurate masks?). The right call differs sharply by answer, and the default recommendation is "don't."
7. **Is stereo wanted for depth/obstacles or for localization?** If depth: mono-depth + LiDAR scale answers it. If localization: what failure of the existing LiDAR SLAM motivates it?
8. **Which QCS6490 variant is on the Q6A** — standard (Tj 95 °C) or extended 300-AA (105 °C)? Determines how much of the 90–110 °C band is in-spec.

The evidence bundle (13 agent reports with sources — Qualcomm datasheets, sc7280 DT trip points, RPi5 VSLAM benchmarks, Rubik Pi llama.cpp numbers, AI Hub model pages) is saved at `/tmp/claude-1000/-home-dp-ippolit/01d850bd-c881-444f-b121-708259eedeea/tasks/review/` if you want the raw material behind any claim. Happy to turn any recommendation here into a patch — the week-one correctness list is a small, well-bounded diff.

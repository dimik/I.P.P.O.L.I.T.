# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from).

---

## 2026-07-06 — Repo hygiene: untrack stock YOLO .pt checkpoints; archive the raw Fable review

**What:** (1) Added `scripts/companion/camera/*.pt` to `.gitignore` and `git rm --cached`'d the
already-tracked `yolo11n.pt`. Neither `.pt` is a build input — `build_yolo.sh` exports its model from the
Qualcomm AI-Hub module, not a local checkpoint; `yolo11n.pt` had been swept into an unrelated white-balance
commit by accident, and the untracked `yolov8n.pt` is a redundant stock ultralytics checkpoint. (2) Committed
`docs/fable5-review.md` — the raw Fable-5 review session (the conversational source; `q6a-pipeline-review-
findings.md` is its durable synthesis and stays the canonical one) — for provenance.

**Why:** P1.2 is fully reproducible without either `.pt` (AI-Hub holds the reference weights; the committed
`.bin`/`.dlc` artifacts are byte-identical to what's deployed). Tracking 6 MB stock checkpoints that nothing
reads only bloats the repo. Verified: repo-wide grep finds no `ultralytics`/`YOLO(`/`torch.load` reference to
either file.

Files: `.gitignore`, `docs/fable5-review.md` (new), untracked `yolo11n.pt`.

---

## 2026-07-06 — Commit the post-CFA-fix WB recalibration (imx296_wb.npz)

**What:** Updated the committed `imx296_wb.npz` to the recalibrated profile that is **currently live on the
device**. The white-balance gain vector drops from a mean **1.807** (roughly R≈2.4 / G≈1.0 / B≈2.0 — the
over-correction left over from the green-fighting "white-patch AWB" era, commit `b90cda2`) to a **near-neutral
`(1.104, 1.000, 1.144)`**, and the 24×32×3 radial shading map is refreshed (mean 1.017→1.165). Per-channel
black levels are unchanged (58.5).

**Why:** After the CFA red↔blue fix (RGGB, not BGGR), the sensor no longer needs the heavy chroma correction
the old profile baked in; a fresh flat-field calibration yields near-unity gains, which is what a correctly
demosaiced IMX296 should have. The repo profile had drifted from the deployed one — this realigns them.

**Verify:** the streamer loads exactly this profile on the Q6A (`loaded WB profile: … wb=(1.104,1.000,1.144)
+ shading map (24, 32)`), stream + detections healthy (person/tv detected with correct colour). Regenerate
with `./view_q6a_cam.sh calibrate 2` (grey card) / `calibrate-dark 2` (covered lens). File: `imx296_wb.npz`.

---

## 2026-07-06 — ByteTrack: stable per-object track IDs on the detector (plan P2.1)

**What:** New `q6a_bytetrack.py` — a numpy-only ByteTrack (no scipy): two-stage per-class IoU association
(match HIGH-confidence detections first, then recover tracks with the LEFTOVER low-confidence detections a
plain tracker discards) + a constant-velocity Kalman filter for predict/smooth. It runs in the **detector
process** right after `infer()` (<1 ms for tens of boxes on a Silver core). The detector now runs YOLO at
`conf=0.1` so ByteTrack gets the low-confidence pool it needs — new tracks still only spawn from high-conf
(≥0.4) boxes, so low-conf dets can only *extend* an existing track, never fabricate one. The detection shm
row grows **6→7 floats** (`x1,y1,x2,y2,conf,cls,track_id`); the streamer reads `track_id`, colours each box
by its **stable track id** (an object keeps its colour) and labels it `#<id> <label> <conf>`. Publishing
stays under the P0.1/P0.2 seqlock on both channels.

**Why:** Fable-5 P2.1 — per-frame boxes flicker identity; a stable track_id is the prerequisite for any
downstream temporal logic (counting, "did this object move", ROS tracking). ByteTrack's low-conf recovery
keeps an object's ID through brief confidence dips instead of dropping/re-numbering it.

**Verify (on-device, live):** with a client pulling the stream — capture ~93 fps, **detector published at
exactly the 10 fps `DET_FPS` cap** (`dseq` +60 over 3 s = 30 sets), 7-float shm protocol clean in both
directions under the seqlock. **Track IDs stable across ~30 frames** — `tid=1` (tvmonitor) and `tid=2`
(person) held the whole window (one transient re-acquire id, expected with greedy IoU). **Low-conf recovery
confirmed:** `tid=2` retained its ID down to **conf 0.23** (well below the 0.4 spawn threshold) — a box that
would flicker/vanish without ByteTrack. No crash; P0.4 shm-unlink safety honoured on both sides (the detector
unregisters attached shm from the Py3.12 resource_tracker). Files: `q6a_bytetrack.py` (new), `q6a_detector.py`,
`q6a_camstream.py`, `view_q6a_cam.sh` (deploys the new module).

---

## 2026-07-06 — Mode-dependent AE: lower exposure ceiling while the robot moves (plan P1.5)

**What:** AE now uses two exposure ceilings — the stationary default (`ae.max_exposure`, 2000) and a lower
moving ceiling (`ae.max_exposure_moving`, 1200) applied while the robot is in motion. A `motion_monitor()`
daemon reads the **MCU wheel-odom shm ring** (`/tmp/mcu_ring.buf`, type 0x01 frames: `lv,rv` mm/s, emitted
~50 Hz regardless of robot state) and sets `State.moving` when `max(|lv|,|rv|) > motion.wheel_mm_s` (40).
`_ae_ceiling()` returns the moving ceiling when moving. Startup VMAX is sized to `max(both ceilings)` so
switching never changes frame timing or fps. Feature is off when `max_exposure_moving = 0`.

**Why:** Fable-5 P1.5 — a moving/rotating robot needs short integration to avoid motion blur; standing still
it can integrate longer for a cleaner, lower-gain image. This keeps the owner's fast 2000/high-fps setup as
the stationary baseline and only tightens further during motion (both ≤ VMAX, so no fps loss). *(To instead
favor clean stationary images, raise `ae.max_exposure` and keep `max_exposure_moving` ~2000 — note fps then
drops to that higher ceiling's rate; documented in the profile.)*

**Robustness:** no ring / not advancing (bench, robot off, tap down) → reported stationary (default ceiling),
no motion tag in the heartbeat. Monitor **reopens** the ring after ~3 s stale (handles a tap restart / new
inode). Reads only the newest ~4 KB each 100 ms tick.

**Verify (on-device):** standalone parser test (incl. rotation = opposite wheels, threshold) all pass;
synthetic-ring integration — **MOVING → `moving` tag + exp clamped to exactly 1200**; **STILL → `still` tag +
exp released to 2000**; bench (no ring) → no tag, default 2000, no crash, ~31 fps. Live end-to-end against the
real MCU tap pends the robot being attached (bench has no ring). Files: `q6a_camstream.py`,
`profiles/imx296.json`.

---

## 2026-07-06 — Strip the desktop/daemon stack on the Q6A (plan P1.4)

**What:** Switched the Q6A to a headless runtime. `systemctl set-default multi-user.target` (no graphical
boot) and `systemctl disable --now` on: **gdm** (→ frees gnome-shell/Xwayland/mutter/gjs/gsd-*),
**docker + containerd** (no running containers; the 13.4 GB NPU image is left on disk, only the daemon
stopped), **cups, fwupd, colord, bluetooth, avahi-daemon, upower, accounts-daemon, udisks2, rtkit-daemon**.
Untouched (load-bearing): ssh, NetworkManager/wpa_supplicant (our `enp1s0` = 192.168.20.2 link), dbus,
polkit, systemd-*, **q6a-llmd**, property-vault, serial-getty.

**Why:** Fable-5 P1 — the board is an SSH-only robot companion; the GNOME session + docker/print/firmware
daemons were pure idle RAM/CPU on a RAM- and thermal-constrained board.

**Verify:** available RAM **1795 → 2272 MB (+~480 MB)** in the same running state (camera + detector + LLM);
all GNOME/Xwayland/docker processes gone; `q6a-llmd` still active; **GPU ISP re-initialised fine with no
display server** ("Adreno(TM) 635"), detector up, stream healthy, 65 °C. **Fully reversible:**
`sudo systemctl set-default graphical.target` + `sudo systemctl enable --now gdm docker …`.

*(System/deployment change — no repo files, but recorded for the revert steps.)*

---

## 2026-07-06 — Native-uint8 input path for w8a8 (skip copyFromFloatToNative) (plan P1.2 cont.)

**What:** `q6a_yolo.py` now constructs the w8a8 context with `input_data_type=DataType.NATIVE`
(output stays `FLOAT`) and feeds the raw uint8 letterbox directly instead of float `[0,1]`. Gated to the w8a8
model; the w8a16 fallback keeps the float path (its native input is 16-bit).

**Why:** the w8a8 graph's input quant is scale 1/255, so uint8 pixels *are* the native tensor — feeding them
directly is bit-identical to the float path but turns the ~5–8 ms `copyFromFloatToNative` quantize into a
~0.3 ms memcpy.

**Verify:** isolated A/B on the real frame — FLOAT 12.9 ms vs NATIVE 9.1 ms, **detections identical**
(`match=True`, tv 0.84). Live in the full pipeline: log shows `input_data_type: native`, input copy
`memscpy ~0.5 ms` (was `copyFromFloatToNative ~5–8 ms`), `model_inference ~9–10 ms`, stream healthy.
**Cumulative P1.2:** the NPU detector step went from ~30 ms (w8a16 float: ~22 ms infer + ~6 ms quantize) to
**~10 ms** (~9.5 ms infer + ~0.5 ms memcpy) — roughly **3×**, same detections on confident objects.
File: `q6a_yolo.py`.

---

## 2026-07-06 — Deploy w8a8 YOLO as the default detector (plan P1.2, owner-approved)

**What:** `q6a_yolo.py` now selects `~/yolov8_det_w8a8.bin` when present and **falls back** to the w8a16
`~/yolov8_det.bin` otherwise. `view_q6a_cam.sh` deploys the w8a8 binary alongside the w8a16 one. Owner
approved the accuracy trade-off (identical on confident detections, softer on <~0.5-conf marginals).

**Why:** ~45% faster core inference halves the NPU duty that drives the board's binding thermal constraint,
with no change to confident detections. w8a16 stays in the repo + on-device as an instant fallback (revert =
remove/rename the w8a8 bin).

**Verify (live, full pipeline running):** detector loaded w8a8; live `model_inference` **~13–16 ms** (vs
w8a16 ~20–24 ms — the occasional spike is concurrent GPU-ISP contention, absent in the isolated 12 ms A/B),
stream healthy at ~15 fps, detections overlaying. Files: `q6a_yolo.py`, `view_q6a_cam.sh`.

---

## 2026-07-06 — Build + benchmark w8a8 YOLOv8 (plan P1.2); fix build-script success gate

**What:** Built `yolov8_det_w8a8` end-to-end (AI-Hub w8a8 export → 2.42 DLC → v68 context binary) and
A/B-benchmarked it against the deployed w8a16 on a real captured frame. Committed the artifacts
(`models/yolov8_det_w8a8.bin`, `models/yolov8_det_w8a8_242.dlc`). **Not deployed** — awaiting owner sign-off on
the accuracy trade-off. Also fixed `build_yolo.sh`'s step-2 gate: it now checks the DLC file exists (with
`tr '\r' '\n'` to surface the outcome) instead of a `grep -vi WARNING` pipe that failed under `pipefail`
because the w8a8 converter's success token shares a line with thousands of warnings.

**Gate result:** w8a8 **composes on v68** (the plan's open risk) — 3.77 MB context binary, no errors.

**Benchmark (25 iters, real 728×544 frame, NPU-only):**
| | core HTP inference | end-to-end (infer+quant+NMS) | detections |
|---|---|---|---|
| w8a16 (current) | ~20–24 ms | **median 32.2 ms** | tv 0.859, backpack 0.479 |
| w8a8 (new) | ~11–13 ms | **median 21.7 ms** | tv 0.840 |

**Read:** ~**45 % faster core inference** (~22→~12 ms, matching ecosystem w8a8 numbers), **~33 % faster
end-to-end**. `copyFromFloatToNative` is ~5 ms for *both* (float input either way) — the native-uint8 path
(task #21) could shave that further but is a separate, uncertain change. **Accuracy:** identical on the
confident detection (tv 0.84 vs 0.86, ~same box); w8a8 dropped the *marginal* backpack (0.479, barely over the
0.30 threshold) — the expected int8-activation softening on low-confidence detections. Caveat: single frame,
not a full mAP sweep.

**Why:** Fable-5's best optimization target — halving inference frees the biggest chunk of NPU duty/heat, and
at YOLO_FPS=10 that directly lowers the thermal load that's the binding constraint.

Files: `build_yolo.sh`, `models/yolov8_det_w8a8.bin`, `models/yolov8_det_w8a8_242.dlc`.

---

## 2026-07-06 — Parameterize build_yolo.sh for precision (enables the P1.2 w8a8 build)

**What:** `build_yolo.sh` now takes a `PRECISION` env var (default `w8a16` = current behavior) passed to the
AI-Hub export, and derives the export module name from the `MODEL` alias (strips a trailing `_w8a8`/`_w8a16`)
so a w8a8 build can be produced under a *separate* output name (`PRECISION=w8a8 ./build_yolo.sh
yolov8_det_w8a8`) without clobbering the live `yolov8_det.bin`. **No build was run** — this is tooling prep.

**Why:** P1.2 (w8a8 YOLO) is the last high-leverage compute item, but it's qualitatively different from the
landed fixes: it runs a **cloud AI-Hub compile**, **swaps the working detector**, may **lose accuracy** (int8
activations), the plan itself gates it on "confirm w8a8 v8 composes on v68", and the real speedup also needs a
native-uint8 input path in `q6a_yolo.py` (to drop `copyFromFloatToNative`, ~10–15 ms/infer). That combination
warrants owner sign-off before executing, so the autonomous batch stops here with the toolchain made ready.
Toolchain confirmed available: qhm venv, qairt-x86, AI-Hub token, and `w8a8` is a supported export precision.

Files: `build_yolo.sh`.

---

## 2026-07-06 — Held-fd VIDIOC_S_CTRL for AE (drop the per-tick v4l2-ctl fork) (plan P1.3)

**What:** AE now sets `exposure`/`analogue_gain` through a **held-open sensor-subdev fd + `VIDIOC_S_CTRL`
ioctl** (`_set_ctrl()`), replacing the three `subprocess.run(["v4l2-ctl", …])` calls. The fd (on `SENSOR_SD`,
e.g. `/dev/v4l-subdev27`) and the linuxpy/ioctl handles are cached on first use; any failure permanently falls
back to `v4l2-ctl` (with a one-time log line).

**Why:** Fable-5 P1 — each AE adjustment was a `fork+exec+open` of `v4l2-ctl` (~10–30 ms), several times a
second while tracking light. The ioctl is a single syscall on an already-open fd — no process churn, lower
jitter in the capture thread.

**Verify (on-device):** confirmed the ioctl works on the subdev first (`G_CTRL`/`S_CTRL` round-trip), then in
the pipeline: AE moved exposure **3000→6000** (real control change via the held fd), **0** `held-fd S_CTRL
unavailable` fallbacks, **0** `v4l2-ctl` subprocesses spawned during the run, 94 frames/6 s healthy. Initial
sensor config at setup still uses `v4l2-ctl` (one-time, before `SENSOR_SD` is set). File: `q6a_camstream.py`.

---

## 2026-07-06 — Headless: run the GPU ISP at detector cadence, not capture rate (plan P1.1)

**What:** In `--headless` (production, no viewer) the capture loop now paces the expensive GPU debayer to
`YOLO_FPS`. Each iteration still *drains* the camera (cheap) to keep the processed frame fresh, but frames
arriving within `1/YOLO_FPS` of the last ISP run are dropped **before** the debayer instead of being processed
and then discarded by the detector's frame-drop. Non-headless (viewer watching) is unchanged — full rate for
smooth video.

**Why:** Fable-5 P1 — with no human viewer the only ISP consumer is the NPU at `YOLO_FPS` (default 10). Running
the ISP at the ~16-19 fps capture rate burns GPU cycles, heat and DDR on frames nobody reads. This is the
biggest steady-state compute/thermal saving available without touching the model.

**Verify (on-device, `--headless --yolo-fps 10`):** heartbeat publish rate dropped to **~8 fps** (paced to the
10 Hz detector cadence) from ~16 fps full-rate — roughly halving GPU ISP work; temp steady ~62 °C, detector
fed (`fseq` advancing). Restored the non-headless bench config (62 frames/4 s). File: `q6a_camstream.py`.

---

## 2026-07-06 — Software thermal governor: pace the pipeline under the 90°C trip (plan P0.9)

**What:** Added a `thermal_governor()` daemon thread that polls the hottest of the 34 SoC thermal zones every
2 s and sets `State.throttle` (an inter-frame sleep) with hysteresis:
- ≥ **87 °C** (CRIT) → 0.40 s sleep (~2.5 fps ceiling), shed heat fast
- ≥ **82 °C** (HI) → 0.12 s sleep (~8 fps ceiling)
- ≤ **76 °C** (LO) → release (full rate); the HI/LO gap prevents oscillation

Both capture loops (mmap `capture_loop` **and** the `_capture_loop_file` fallback) honour `State.throttle`
after each `process()`. The heartbeat now prints `temp=NN C` (+ `throttle=…s` when active).

**Why:** Fable-5's binding constraint — the board's real limit is heat (kernel trip at 90 °C) and the **NPU has
no kernel throttle**, so nothing stops the pipeline cooking the SoC. Throttling the *streamer's* frame cadence
cascades: the detector only infers on new frames (`fseq`), so slowing publish cools **both** the GPU ISP and
the NPU from one control point — no cross-process IPC needed.

**Verify (on-device, with thresholds temporarily lowered to force it):** governor engaged at real temps
(logged `[thermal] CRIT 56.7°C → hard throttle`), and delivered rate dropped to **17 frames / 8 s (~2 fps)**
vs **~155 / 8 s (~19 fps)** unthrottled — throttle demonstrably reduces load on both paths. Restored real
thresholds: production runs full-rate at 60–78 °C (78 frames/5 s), heartbeat shows `temp`. **Debugging notes
for future me:** running a copy from `/tmp` silently forces the file-tail path (can't import `~/q6a_v4l2`);
launch the streamer so ssh returns immediately then measure in a *separate* ssh (a trailing `sleep` in the
launching session SIGHUPs the child); QNN warnings are `\r`-terminated and interleave stdout — normalize with
`tr '\r' '\n'` before grep. File: `q6a_camstream.py`.

---

## 2026-07-06 — Detector supervision + clean NPU/shm teardown + staleness cutoff (plan P0.3, P0.4)

**What:** Made the two-process split survive detector death and shut down cleanly.
- **Supervision (streamer):** `init_detector()` now runs a daemon supervisor thread that `wait()`s on the
  detector and respawns it if it dies, with **exponential backoff 5→60 s** (reset after a >60 s stable run) so
  a crash-looping detector never restart-storms the HTP/fastrpc stack (which can wedge cdsp → reboot). A
  `DET["stop"]` Event lets teardown stop the supervisor before killing the child.
- **Clean teardown:** detector installs SIGTERM/SIGINT handlers that break its loop into a `finally` which
  calls `ctx.release()` (real QNN API) and `close()`s (not unlinks) the shm. Streamer gets a SIGTERM handler
  that `sys.exit()`s so `atexit._cleanup` actually runs (a bare SIGTERM otherwise skips atexit, orphaning the
  detector with the NPU held); `_cleanup` now SIGTERMs the child and `wait(3)`→`kill()`.
- **resource_tracker fix:** detector `unregister()`s the attached segments from Py3.12's `resource_tracker`, so
  the detector exiting no longer unlinks the streamer's live `q6a_frame`/`q6a_ctrl` (the exact failure I hit
  when an ad-hoc probe destroyed the running segment).
- **Staleness cutoff (streamer):** `_read_dets()` tracks when `dseq` last advanced; if it stalls for
  >`DET_STALE_SEC` (2 s) it clears the overlay instead of drawing stale boxes forever while the detector is
  dead/respawning.

**Why:** Fable-5 findings — the spawned detector was fire-and-forget (silent permanent loss of detections on
any crash), teardown SIGKILLed everything (no NPU release, risking fastrpc state), and the Py3.12
resource_tracker would unlink a segment the detector merely attached to.

**Verify (all on-device):** baseline streamer+detector up, 46 fps stream. `kill -9` the detector → supervisor
logged `exited rc=-9 … respawn in 10s` then `respawned` + `YOLO ready`, NPU did not wedge, stream continued.
`kill -TERM` the streamer → streamer gone cleanly, **no orphan detector**, **shm cleaned**, detector logged
`shutting down: releasing NPU context + shm`. Files: `q6a_camstream.py`, `q6a_detector.py`.

---

## 2026-07-06 — Fix --fast / GPU-fallback resolution mismatch + add shm shape guard (plan P0.7, P0.8)

**What:** (P0.7) When the half-res CPU debayer is active (`--fast`, or the automatic `--gpu`→CPU fallback when
the Adreno fails to init), `OUT_W/OUT_H` are now halved to `W//2 x H//2` — previously they were halved only
for `--bin`. (P0.8) `process()` now asserts `rgb.shape == (OUT_H, OUT_W, 3)` before publishing to shm and
raises a clear `RuntimeError` naming the mismatch.

**Why:** Fable-5 finding. The half-res debayer emits 728×544 but the shm frame + detector were sized
1456×1088, so `DET["frame"][:] = rgb` broadcast-crashed. Critically this was reachable *without* asking for
`--fast`: `--gpu` with a failed GPU init silently sets `args.fast=True`. The shape guard turns any future
mode/alloc mismatch into a named error instead of a cryptic NumPy broadcast failure.

**Verify:** `--fast --awb` (no --bin/--gpu, the formerly-broken path) → **61 frames, 0 mismatch/capture
errors**, detector correctly at 728×544 (would have crashed before). Restored production `--gpu --bin --awb`
→ 61 frames, 0 errors, 728×544. File: `q6a_camstream.py`.

---

## 2026-07-06 — V4L2 DQBUF: distinguish EAGAIN from real device errors (plan P0.6)

**What:** `read_latest()` in `q6a_v4l2.py` caught *all* `OSError` from `VIDIOC_DQBUF` and treated it as "no
more ready buffers" (`break`). Now it breaks only on `EAGAIN`/`EWOULDBLOCK` and **re-raises** everything else
(`ENODEV`, `EIO`, …). Added `import errno`.

**Why:** Fable-5 finding — a genuine device error (camera unplugged, CAMSS fault) was silently swallowed, so
`read_latest` kept returning `None` and the capture loop looped forever without ever reinitialising the
device. The caller already wraps `read_latest` in `try/except` (close cam, retry, fall back to file-tail after
3 fails), so a re-raised error now drives that recovery path instead of a silent stall.

**Verify:** Restarted; `curl /stream` → **62 JPEG frames in 4 s (~15.5 fps)**, `capture error` count = 0
(normal EAGAIN drain still breaks cleanly, no false positives). File: `q6a_v4l2.py`.

---

## 2026-07-06 — MJPEG send timeout: drop half-open clients (plan P0.5)

**What:** Added `self.connection.settimeout(10.0)` in the `/stream` handler and widened the write-loop
`except` to include `socket.timeout`/`TimeoutError`/`OSError`. A stalled client now raises out of
`wfile.write()`, hits the `finally`, and decrements `State.clients`.

**Why:** Fable-5 finding — with no timeout, a half-open client (network dropped, no RST) blocks `wfile.write()`
forever. `State.clients` never returns to 0, so the capture loop keeps running the full GPU ISP (heat, power,
DDR) for a viewer that will never read another byte. 10 s ≫ the time a healthy client needs to drain one
96 KB frame, so real viewers are unaffected.

**Verify:** Restarted; normal `curl /stream` → **62 JPEG frames in 4 s (~15.5 fps)**, log shows healthy
17 fps publish with `clients=1` while connected. No regression. File: `q6a_camstream.py`.

---

## 2026-07-06 — Seqlock hardening on both shm channels (plan P0.1, P0.2)

**What:** Made the lock-free shared-memory handoff a *correct* seqlock on both directions.
- **Frame channel (streamer→detector):** `process()` now bumps `fseq` to **odd before** copying the RGB frame
  into shm and **even after** (was a single post-increment). Reader in `q6a_detector.py` already rejected an
  odd seq / a seq that changed mid-copy, so it now provably never consumes a torn frame.
- **Detection channel (detector→streamer):** `q6a_detector.py` now wraps the `dbuf`+`dcnt` write in the same
  odd/even `dseq` fence. Added `dseq` (offset 8) to the streamer's `DET` dict and rewrote `_read_dets()` as a
  guarded read: reject odd `dseq`, snapshot rows, re-check `dseq`; bounded 4-retry then fall back to the last
  good detection set (cosmetic overlay must never block the display path). Previously `_read_dets` read
  `dcnt`/`dbuf` with no fence and could overlay a half-written box list.

**Why:** Fable-5 finding — the frame handoff was described as a seqlock but the writer lacked the odd/even
fence, leaving a window where the reader could copy mid-write and see an unchanged seq on both sides
(torn frame). The return channel had no fence at all.

**Verify:** Deployed both files; restarted via the blessed launcher path (`setsid … &`, ssh returns
immediately — a trailing `sleep`/check in the *same* ssh session SIGHUPs the child). `curl /stream` →
**98 JPEG frames in ~6 s (~16 fps)**, no crash, zero `infer error`, board 58–65 °C. Both seqs settle even at
rest. **Note:** attaching to the shm from an ad-hoc Python probe *unlinked* the segment on exit
(Py3.12 `resource_tracker` leaked-object cleanup) — this is exactly the P0.4 hazard; the upcoming detector
step will attach with tracking disabled. Files: `q6a_camstream.py`, `q6a_detector.py`.

---

## 2026-07-06 — Fix wrong planning constants + stale model label (plan P1.6, P0.10)

**What:** (1) CLAUDE.md RAM `up to 16GB` → **12GB on this board** (11.5GB usable, avail ~2.8GB) with the
correct DDR figures (~22 GB/s theoretical / ~15 GB/s practical, not 40-50); fixed the LLM-section bandwidth
claim too. (2) `q6a_yolo.py` `QNNContext("yolov11_det", …)` → `"yolov8_det"` — the deployed model is YOLOv8
(v11 doesn't run on v68), so logs no longer mislabel it.

**Why:** Fable-5 review verified against `free` (11558 MB) and single-core memcpy — the 16GB/40-50GB/s figures
were wrong and would mis-size the LLM+Nav2+2nd-cam budgets. The `yolov11_det` string was a copy-paste label,
not the model.

**Verify:** `free` confirms 12GB; label is cosmetic (QNN context name), takes effect on next detector restart.
Files: `CLAUDE.md`, `scripts/companion/camera/q6a_yolo.py`.

---

## 2026-07-06 — Start the review-driven improvement batch; add this changelog

**What:** Established `CHANGELOG.md` and kicked off the architecture-improvement batch derived from the
Fable-5 review (`docs/q6a-pipeline-review-findings.md`) and plan (`docs/q6a-pipeline-improvement-plan.md`).

**Why:** The review was verified against the live board + source (every spot-checked code claim held), so the
plan's verified items are worth executing. Binding constraint is thermal + DDR bandwidth (~1.5 W passive
headroom), not compute — so the batch prioritizes (a) correctness/safety prerequisites and (b) changes that
*reduce* load (ISP-at-detector-cadence, w8a8 YOLO).

**Baseline (measured, this session):** `--gpu --bin --awb`, 1 client → ~16 fps publish, YOLO ~10 Hz (38–44 ms
infer incl. 5–15 ms float-I/O quantize), ~1.0 CPU core total, GPU pinned at 315 MHz, board 61 °C idle /
72–78 °C active (19 h uptime, bench-side). RAM **12 GB** (not 16), avail ~2.8 GB.

**Planned order (each = one commit):** constants+label → seqlock → MJPEG timeout → EAGAIN re-raise →
--fast/frame-assert → detector supervision → thermal governor → ISP-at-detector-cadence → w8a8 YOLO.

Files: `CHANGELOG.md` (new). Commit: _this_.

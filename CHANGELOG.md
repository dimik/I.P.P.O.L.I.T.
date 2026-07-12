# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from).

---

## 2026-07-08 — Cloud voice brain live: Cloudflare Workers AI endpoint (STT + LLM) + robot speaks LLM replies

**Milestone:** the cloud half of voice control is **deployed and verified end-to-end** — the planned
replacement for the retired on-device 1B LLM. `scripts/ask.sh "hello robot"` made the robot answer
*"I am IPPOLIT, a converted Dreame robot vacuum now serving as an autonomous AI rover"* through its
own Piper voice, with the reply generated in the cloud. New doc: **`docs/voice-cloud.md`**.

**Worker (`cloud/voice-worker/`, live at `https://ippolit-voice.poklonskiydmitry.workers.dev`):** one
HTTPS round trip — 16 kHz WAV in → Whisper (`@cf/openai/whisper-large-v3-turbo`) → Llama
(`@cf/meta/llama-3.3-70b-instruct-fp8-fast`, JSON mode) → `{transcript, reply, voice, actions[]}`.
Routes: `POST /voice` (audio), `POST /text` (typed/testing), `GET /healthz`. Auth = shared bearer
secret (`AUTH_TOKEN` Worker secret ↔ gitignored `.dev.vars`). Live robot context (battery, status,
`/object_map` objects) rides in every request, so status questions are answered in the same single
LLM call — no tool loop by design (the hallucinated-tool failure mode killed the offline agent).
Free tier 10k neurons/day ≈ hundreds of commands/day.

**Verified:** intent suite via `/text` — goto-object with correct coords, battery Q&A from context,
refusal on unknown targets, **Polish in → Polish reply + `gosia` voice**, stop; `/voice` with real
spoken audio = perfect transcript + correct action in **3.8 s** round trip; 401 on bad token.

**Two Workers-AI gotchas burned in** (handled in `src/index.ts`, documented so they stay learned):
(1) **JSON mode is model-gated** — llama-4-scout (first pick) is not on the supported list and
*silently ignores* `response_format`, free-styling the JSON shape; (2) in JSON mode the AI binding
returns `response` as an **already-parsed object**, not a string — `JSON.parse` throws.

**Companion side (ready, awaiting a USB mic — the robot has NO mic, both paths dead per
`docs/sensors.md`):** `scripts/companion/q6a_voice.py` (arecord + energy-VAD or `/voice/trigger`
push-to-talk → Worker → `/robot/speak` + Valetudo actions: dock/stop/pause/locate/goto_point with
the meters→mm inverse of the bridge transform) + `q6a-voice.service` + `ippolit-voice.env.example`.
`goto_point` sign convention still needs one live calibration drive.

**Fixed along the way:** CLAUDE.md `LocateCapability` row was wrong — verified live: PUT **needs**
`{"action":"locate"}` (200); empty body = 400. Also marked the Mac's `dreame-*` ssh aliases as stale
(gone from `~/.ssh/config`; the Mac's `id_rsa_dreame` is rejected by the robot — access path is via
the Q6A's `robot-usb`/`robot-wifi` aliases, whose key works).

---

## 2026-07-12 — q6a_creep_test.py: wheel-drop hard stop removed too, per explicit repeated confirmation

Following the ambiguous "stopped at the edge" event (investigated but couldn't confirm AVA vs our software
via logs -- exhaustive search of trace_sync.log/log_0/log_err/ava_cmd.log/dmesg found no persisted evidence
either way), user asked to remove ALL stop gates, keeping only MiDaS-based speed reduction. Given this
contradicted the earlier stated design ("hard stop on wheel drop only"), asked a direct clarifying question
before touching safety code: does this include wheel-drop too? Confirmed explicitly and unambiguously:
"Remove wheel-drop too -- truly no stop gates."

**Implemented exactly that** in q6a_creep_test.py: removed the /cliff subscription and hard-stop check
entirely. The only things that end a run now are the --seconds time bound and a stale-sensor abort (kept
because it's needed for the MiDaS ramp to mean anything at all, not a cliff-specific safeguard). MiDaS
proportionally reduces velocity as before but NEVER stops the robot. Docstring rewritten with unambiguous
warnings: NOT for autonomous/unsupervised use, ever; a human physically catching the robot is the ONLY
backstop for the entire run. q6a_drive.py's production hard-stop-on-MiDaS AND hard-stop-on-wheel-drop
behavior are both untouched -- this change is scoped entirely to the separate supervised test script.

Files: `scripts/companion/q6a_creep_test.py`.

---

## 2026-07-12 — New SUPERVISED-ONLY creep-test script (does not hard-stop on MiDaS)

User wants the MiDaS floor-drop signal to reduce speed rather than hard-stop, to observe sensor behavior
during a slower approach to a real edge. Given we just confirmed wheel-drop only fires once a wheel has
ALREADY left the ground (a last-instant signal, not a safety margin), asked directly whether this is for
supervised testing or eventual autonomous use -- confirmed supervised-only (human present, hand ready, same
as all of today's real-edge testing).

Given that answer, implemented as a NEW, separate script (`q6a_creep_test.py`) rather than modifying
`q6a_drive.py`'s default hard-stop behavior at all -- production safety logic is untouched. The new script:
proportionally reduces velocity as the MiDaS center-drop signal strengthens (ramps from full speed at
center=0.20 down to a floor speed at center=0.55, never fully to zero), with wheel-drop `/cliff` as the
ONLY hard stop. Docstring and startup log both carry an explicit "NOT for autonomous/unsupervised use"
warning. Not yet run live -- pending explicit go-ahead each time given it intentionally removes the MiDaS
safety margin.

Files: `scripts/companion/q6a_creep_test.py` (new).

---

## 2026-07-12 — RESOLVED (negative): downward IR cliff sensors don't detect a real edge in practice

User asked to confirm whether we actually have working data from the robot's downward-facing IR cliff
sensors, given the 2026-07-09/07-11 investigation left this genuinely open (decode confirmed correct, but
never verified to trip during a real approach, only during a full lift).

**First found a real deployment gap:** the mcu_node.py decoder that added /cliff/front, /cliff/rear, and
/mcu/triggers (2026-07-11) had never actually been pushed to the Q6A's running mcu-node.service -- it only
existed in the repo. Confirmed via grep (zero matches for decode_triggers/d_view in the deployed file).
Deployed it, verified /cliff and /bumper (the existing safety-critical topics) still worked identically
before proceeding -- no regression.

**Live test at the REAL stairwell edge**, using q6a_drive.py (hard-stops on wheel-drop OR MiDaS sharp
floor-drop) while separately monitoring /cliff/front, /cliff/rear, and raw /mcu/triggers bits:
- First attempt: robot was already very close to the edge, wheel-drop fired almost immediately (1.7s) --
  didn't give us a window to observe the IR sensors before the last-resort backstop engaged.
- Repositioned ~10-15cm back for more runway, retested: **MiDaS correctly stopped the robot** (DROP AHEAD,
  center=0.66 sharp=662.6 -- an unambiguous, far-past-threshold cliff signature) **while /cliff/front stayed
  False the entire time and no d_view_* bit ever activated.**

**Conclusion: the front IR cliff sensors do not detect this real edge, even while actively driving toward
it** -- rules out the "only works while driving" hypothesis from 2026-07-09 (this test WAS while driving).
This is a confirmed negative, not an open question anymore. Real fall protection remains MiDaS+LiDAR
forward-drop detection (validated again here) as early warning, plus wheel-drop /cliff as the last-resort
backstop (validated in the first attempt) -- NOT the downward IR sensors, despite them being correctly
decoded and wired into ROS.

Files: `docs/sensors.md` (resolved the open item), mcu-node.service redeployed on the Q6A.

---

## 2026-07-12 — Corner-avoidance logic validated live (first time), edge-follow now fully calibrated

With both LiDAR bearing fixes deployed (offset + sign), re-ran the edge-follow controller with --side right
now correctly corresponding to the true physical right. The robot happened to be positioned right at the
90deg interior corner set up earlier for this exact test -- took the opportunity to validate corner-handling
directly rather than reposition.

**7s supervised run, clean result:** front=0.28m triggered CORNER (rotate-away, angle=-14deg clamp) ->
front clearance grew smoothly (0.28->0.35m) as it rotated -> transitioned cleanly back to normal
wall-following (d=0.244m tracking toward the 0.299m setpoint) -> oscillated a few more times between
corner-avoid/follow near the tight bend (expected) -> settled around d~=0.30-0.32m by the end -> stopped
cleanly on schedule, no collision, no stuck state.

This is the first live validation of the corner/reactive-avoidance logic (previously only the straight-
wall-following convergence had been tested). Combined with the two LiDAR bearing fixes and the BODY_R
clearance calibration from earlier today, task #14 (edge-following drive controller) now has: correct
sign, calibrated clearance, validated straight-line convergence, AND validated corner handling -- all under
the NOW-correct bearing convention (previous tests before today's fixes should not be trusted for which
physical side was followed, though the control math itself was always sound).

---

## 2026-07-12 — SAFETY FIX #2: LiDAR bearing had the WRONG SIGN (left/right swapped), not just offset

Follow-up to the angle_offset_deg fix. After correcting the offset, re-verified with the wall the user
confirmed was on the robot's TRUE right (as if driving forward) — LiDAR still read it at bearing +90
("left" in our convention). A pure offset error cannot swap left and right (it only shifts all bearings by
a constant); only a wrong SIGN can. This meant the original 2026-06-19 "handedness -1" decision (`bearing =
-ang_deg + offset`), based on a single Valetudo-SLAM heading comparison, was itself wrong — it apparently
validated the FRONT alignment but not the LEFT/RIGHT sense.

**Independently cross-checked with a second sensor before committing to the fix**, per the user's own
suggestion: pulled a frame from the already-running combined YOLO+MiDaS camera stream (:8093). The RGB view
clearly showed the flat wall occupying the center-to-right of frame (a window/chair alcove receding on the
left) — visually confirming wall-on-the-right independent of the LiDAR entirely.

**Fixed:** flipped the sign in `lds_scan_node.py` (`bearing = +ang_deg + offset`, was `-ang_deg + offset`)
and re-derived the offset for the new sign convention (`angle_offset_deg=43.0`, was `-43.0`). Re-verified
BOTH calibration points together after redeploying: wall (true right) now reads cleanly at bearing 270
(dense, 0.3m), nothing at bearing 90 (left) — matches physical reality and the camera cross-check.

**This means bearing was doubly wrong all session** (wrong offset AND wrong sign) until now — any earlier
`--side left`/`--side right` edge-follow results should be treated as testing "whichever side ended up
there," not verified to be the stated physical side. The control-loop math itself (STEER_SIGN, BODY_R,
convergence behavior) remains valid; only the side-label correctness was in question, and is now fixed.

Files: `scripts/robot/lds_scan_node.py`, `scripts/companion/systemd/lds-scan-node.service`.

---

## 2026-07-12 — SAFETY FIX: LiDAR front-bearing was miscalibrated by ~43deg (never actually tuned)

User asked "are you sure we have front lidar position calibrated properly?" during confusing corner-test
setup (turn commands producing inconsistent rotation, wall appearing at unexpected bearings after
repositioning). Investigated rather than assume:

**Found the deployed `lds-scan-node.service` runs `angle_offset_deg` at its CODE DEFAULT (0.0)** — the
"eyeball and tune ~0-5deg" calibration the node's own docstring assumed had happened was **never actually
applied**. Ran a clean, unambiguous test: placed a single isolated paper bag directly at the robot's true
front bumper (removes all ambiguity from nearby walls/corners) — `/scan` read it at **bearing=+43deg**, not
0deg. A ~43deg error, far larger than the assumed few-degree slop.

**Fixed:** added `-p angle_offset_deg:=-43.0` to `lds-scan-node.service`'s ExecStart, deployed, restarted.
Re-ran the identical paper-bag test: now reads at **exactly bearing=0deg**. Clean before/after confirmation.

**Implication for today's earlier edge-follow work:** the STEER_SIGN, BODY_R, and convergence-behavior
calibrations done earlier this session remain valid (they're properties of the control loop / geometry, not
tied to an absolute bearing label) — but every `--side left`/`--side right` test was run under this
uncalibrated bearing convention, so which TRUE physical side was actually being followed may not match what
was reported at the time. Going forward, `--side` should correctly correspond to the robot's real left/right.

Single-point calibration (one paper-bag placement) — re-verify with a second isolated-object test at a
different bearing if more precision is needed.

Files: `scripts/companion/systemd/lds-scan-node.service`.

---

## 2026-07-12 — Thermal enclosure installed: significant cooling improvement confirmed

User installed a thermal enclosure on the Q6A. Measured steady-state (9 min uptime, full active autonomy
stack confirmed running: q6a-vision YOLO+MiDaS ~55% CPU, slam_toolbox, laser-odom, objmap, cliff-guard,
announce, valetudo-bridge, mcu-node, lds-scan-node, audio-bridge -- all 11 companion services active)
against the documented pre-enclosure baseline (idle ~66C, active GPU+NPU ~72-80C, hot-trip 90C):

- CPU cores (cpu1-11, cpuss0/1): 55-58C
- GPU/NPU (gpuss, nspss): 50-52C
- Overall range: 47-58C

**Running the full active stack now sits cooler than the old baseline's IDLE temperature** -- ~8-15C
headroom gained vs the old idle point, 20C+ vs the old active-load point. Substantially more thermal
margin before the 90C hot-trip / 110C critical shutdown.

Files: `docs/q6a-companion.md` (thermal envelope section updated).

---

## 2026-07-12 — SAFETY FIX: edge-follow clearance model was off by ~4.4cm (calibrated against real measurement)

First successful closed-loop validation of the LiDAR edge-follow controller (STEER_SIGN=-1, KP=55, KD=0.30,
clamp=14deg from 2026-07-11): a 15s supervised run converged cleanly and smoothly, d: 0.367->0.307m, no
oscillation/overshoot/wall-loss, exactly as designed.

**But a tape measurement caught a real safety gap.** At the moment d=0.299m, a precise tape measure on the
robot read an 0.08m gap to the railing — not the 0.124m our BODY_R=0.175m (350mm-diameter spec) implied.
Implied correction: BODY_R_effective = 0.299 - 0.08 = 0.219m, ~4.4cm more than spec. Had the earlier run
been allowed to reach the OLD setpoint (d=0.255m), the real-world gap would have been only ~3.6cm, not the
intended 8cm — a real risk near a stairwell railing. Source of the 4.4cm gap is unresolved (under-spec'd
body radius, LiDAR not exactly centered, or a yaw-related corner effect — psi was nonzero throughout) —
documented as an empirical correction, not a decomposed root cause.

**Fixed:** `BODY_R` in `q6a_edge_follow.py` updated 0.175 -> 0.219m. Verified immediately: at the exact
spot ground-truthed at 8cm, the corrected model now reads `e~=0` (was `e=+0.048` before the fix) — the
setpoint math now means what it claims to mean. Single-point calibration — re-verify with a second
measurement at a different distance before trusting for unsupervised operation.

Files: `scripts/companion/q6a_edge_follow.py`.

---

## 2026-07-11 — MCU firmware RE attempt for battery telemetry: ATTEMPTED, INCONCLUSIVE

User asked to attempt finding richer battery telemetry inside the MCU firmware itself (~/dreame-re/mcu.bin,
GD32F303-class, 151KB). Real effort, proper tooling, honest inconclusive result:

**Confirmed the firmware is a debug/test build** with a full interactive console compiled in — 540 readable
strings, including named debug variables `batVolt(mV)`, `batCurrent`, `chgVolt(mV)`, `batTemp`,
`chargeCurrent`, and (valuable side-finding) `chargePWM`/`PWMcharge` — **this unit uses PWM-based charge
control, not BQ24725/SMBus** (the BQ24725 string found earlier in AVA's binary is most likely dead/
alternate-SKU code). Also a working diagnostic command: `-m/-i/-c/-b` = mcu/imu/charge/battery.

**Could not trace whether/how this reaches the SoC**, despite installing proper tooling (Ghidra 12.1.2 +
binutils-arm-none-eabi, JDK 21 — all via the user's sudo, correct ARM:LE:32:Cortex processor/base address)
and trying three independent approaches:
1. Raw byte scan for string addresses anywhere in the image — zero hits.
2. Ghidra's static reference analyzer (proper auto-analysis, not naive linear objdump) — zero references,
   INCLUDING for sanity-check strings we know are used (the console's own help text). This rules out a
   tooling-quality problem — the actual code pattern (very likely a runtime-indexed table access) defeats
   constant-propagation reference analysis generally.
3. Call-graph heuristic (find the biggest repeated-callee outlier as a proxy for a table-print loop) —
   found a real 78x outlier, decompiled it, turned out to be an LED blink-pattern state machine, unrelated.

**Decided to stop** (matches the risk-managed pattern in this project — same class of open-ended effort as
the abandoned H.264 SPS/PPS RE) rather than continue into manual disassembly reading or firmware emulation.
`avacmd battery`/`charge_state` remain the practical source of truth.

Files: `docs/sensors.md` (full writeup under "MCU firmware RE attempt").

---

## 2026-07-11 — Decided against an AVA-internal battery siphon

Follow-up to the battery-driver question: user asked whether we should siphon battery status out of AVA
the same way `camsiphon`/`serialtap` do for camera/LiDAR. Checked `/tmp/log/log_0`'s `WritePropInt`/
`WritePropString` internal-property-write log (the mechanism CLAUDE.md already documents AVA using for its
own state) for a battery entry — **not there either**: only 13 distinct property types logged, all static
boot-time settings, nothing tracking the live 47% battery value, no line mentioning battery/charge/power.

Key distinction from camera/LiDAR: those needed a siphon because Valetudo/`avacmd` expose NO interface for
raw frames/scans at all — a genuine gap. Battery has no such gap: `avacmd get_prop battery`/`charge_state`
already works cleanly and is already relayed to `/battery`. A camera-style hook into AVA's internals would
only get the SAME percentage, event-driven instead of ~15s-polled — not richer telemetry, since prior
investigation already showed AVA itself never receives voltage/current/temperature from the MCU at all.
**Decided: not worth building** — the latency gain doesn't justify reverse-engineering AVA's internal
symbols for redundant data. Closes out the battery investigation thread.

Files: `docs/sensors.md`.

---

## 2026-07-11 — Battery driver question resolved: no kernel driver applies, decisive elimination

User asked whether we should install a "proper" battery-power-supply driver instead of the generic one that
never binds. Investigated properly rather than assume, and the answer is a clean **no** — there is nothing
to install a driver FOR:

1. Found `BQ24725` (a real TI charge-controller chip) as a literal string in `/ava/bin/ava` — matches the
   MCU protocol's `HwInfo.charge_type` enum exactly, so it's genuinely the hardware. But it has **zero
   device-tree nodes** (`find .../devicetree/base -iname "*bq24*"` -> empty) — nothing for any kernel driver
   to bind to.
2. **Decisive test:** inspected AVA's actual open file descriptors directly (`/proc/<pid>/fd/`, not a
   syscall-trace snapshot that could miss a boot-time-only open) — `/dev/null`, `/dev/video2`, `/dev/ttyS4`
   (MCU), `/dev/ttyS3` (LiDAR), camera/GPU nodes. **Zero I2C fds, zero GPADC/input fd.** This rules out BOTH
   remaining hypotheses from the earlier investigation: AVA does not talk to BQ24725 directly over I2C, and
   it does not read the SoC's GPADC directly either (superseding that earlier guess).

**Conclusion: BQ24725 must be wired to the separate motor-control MCU, not the SoC.** AVA's only relevant
hardware fd is `/dev/ttyS4` — so whatever `avacmd battery`/`charge_state` report is necessarily relayed over
the SAME `3c..3e` serial protocol we already tap. There's no missing/wrong Linux driver anywhere in this
picture (not for AXP806 — wrong chip, no fuel-gauge silicon; not for BQ24725 — no devicetree node, not on
a bus the SoC can reach). If richer battery telemetry is ever wanted, the real next step is more MCU-
protocol reverse-engineering (same class of work as the Triggers bit-map: check for a request/response
exchange we haven't caught passively, or re-examine currently-unknown packet types for a mislabeled
battery field) — not kernel/driver work.

Files: `docs/sensors.md` (corrected/superseded the GPADC hypothesis with this decisive finding).

---

## 2026-07-11 — Root-cause investigation: why BatteryStatus never appears

User asked to dig into WHY (not just confirm) BatteryStatus (0x2B) never appears. Ruled out "just rare" with
a 90s raw capture (up from 15s) — zero frames, packet-type histogram scaled perfectly proportionally between
the two runs (same 9 types throughout both), no new/rare packet ever showed up. That's a much stronger
negative than the earlier 15s result.

Investigated where AVA's battery numbers actually come from, since `avacmd battery`/`charge_state` clearly
have live data (watched `battery` climb 35->39->40% live):
- PMIC is an **AXP806** (`dmesg`: "AXP20x variant AXP806 found") — regulator/LDO-only, no fuel-gauge. This
  is WHY the generic `axp803-battery-power-supply` kernel driver exists but has zero bound devices (wrong
  chip variant for that driver).
- No `/sys/class/power_supply/*` device, no IIO ADC device, no `bq24`/`bq27`/charger anywhere in dmesg —
  no discoverable dedicated battery-management IC via any standard Linux framework. (One red herring ruled
  out: a second i2c device sharing the same 0x36 address on a different bus turned out to be the OV8856
  camera, unrelated.)
- The SoC's own hardware GPADC (`5070000.gpadc`) IS initialized at boot, registered as an input device
  (suggests shared use with the physical button panel via a resistor ladder) — plausible but UNCONFIRMED
  additional use for direct battery-voltage sensing (resistor divider into an SoC ADC pin, no smart battery
  IC — a common cost-reduction pattern). Would need to open the device or dump the devicetree to confirm;
  out of scope for now.

**Conclusion:** this is very likely not a decoding gap at all — the battery is probably measured by a
different subsystem (SoC GPADC) entirely, not the ttyS4 MCU we tap for motors/sensors. `/mcu/battery` stays
in `mcu_node.py` for forward-compat but shouldn't be expected to ever populate on this unit; `avacmd
charge_state`/`battery` remain the real source (already what `valetudo_bridge.py` uses for `/battery`).

Files: `docs/sensors.md` (full investigation writeup).

---

## 2026-07-11 — Live battery investigation while charging: BatteryStatus confirmed ABSENT

User asked to check everything battery-related while the robot was actively charging (35%). Ran a direct
raw-MCU capture on the robot (no Q6A/ROS needed — chroot python3 reading /tmp/mcu_ring.buf over
dreame-wifi) for 15s during active charging:

- **`BatteryStatus` (0x2B): CONFIRMED ABSENT — zero frames**, and no other packet type in the capture
  carries a plausible voltage/current/SoC field. This is a definitive result, not "unconfirmed": the D10s
  Pro's MCU firmware does not emit this packet type. `/mcu/battery` (added earlier today) will likely never
  publish on this hardware. `avacmd charge_state` + Valetudo's battery-level poll remain the only real
  source — not a gap we can close by decoding more MCU traffic.
- Found `0x24` (1 byte, undocumented upstream but commented "something connected with the battery
  temperature") flowing at ~2Hz, constant `0x00` throughout. Decoded as `/mcu/battery_temp_flag` (best
  effort — only the "OK/0" value has actually been observed; nonzero meaning unconfirmed).
- Checked the other currently-undecoded types seen in the capture: `0x12` (7B) is a monotonic
  timestamp/counter, `0x23` (5B, "base-related") and `0x26` (2B, ~50Hz, "_CtrlMcuCMD/slowSensor") were
  constant the whole capture — none look battery-related, left undecoded (still visible on /mcu/unknown).
- **Bonus cross-validation**: `Triggers.dock_sta` read `1` for all 150 frames (genuinely docked),
  `Status100ms.dust_container_missing` read `False` for all 150 frames (dustbin genuinely installed) — both
  match physical reality, first live confirmation of these bits. Also: Triggers flows while just
  docked/charging, not only during active manual control as the 2026-07-09 note assumed.

Files: `scripts/robot/mcu_node.py` (`/mcu/battery_temp_flag`), `docs/sensors.md`.

---

## 2026-07-11 — Full MCU protocol coverage: every known packet type now decoded + published

Following the gap audit, closed every remaining gap so nothing from the MCU is silently dropped. Built and
ran a full stub test harness (fake rclpy/Node/publishers) to exercise `dispatch()` against every packet
type + two catch-all cases (a totally unknown type, and a known type at the wrong length) before deploying
anywhere near hardware — all 15 cases passed cleanly, 25 publishers constructed as expected.

**Added:**
- `Status20ms` roller/side-brush current (previously decoded then discarded) -> `/mcu/status20` (JSON)
- `Status10ms` leftDis/rightDis 100Hz wheel-distance deltas (previously discarded) -> `/mcu/status10` (JSON)
- `/mcu/triggers` now publishes the FULL field dict every time (was nonzero-only before — a field going
  false used to just vanish from the JSON instead of showing `false`)
- `/mcu/error` — aggregate of all 17 error/overcurrent bits in Triggers, edge-logged like /cliff and /bumper
- `HwInfo` (0x29) -> `/mcu/hwinfo` (mcu/imu/charge/app type IDs, latched)
- `McuFwVersionInfo` (0x07) -> `/mcu/fw_version` (git hash + version, latched)
- `PingMsg` (0x0F) -> `/mcu/ping`, `Status500ms` (0x05) -> `/mcu/status500` (RTC unix timestamp)
- `ShutdownMsg` (0x10) -> `/mcu/shutdown_event` (occurrence = the signal; sent amid a poweroff sequence)
- `McuLog` (0x27) -> `/mcu/log_raw` (raw hex, format itself is undocumented even upstream)
- `FactoryTest` (0x04) -> `/mcu/factory_test`
- **Catch-all**: any unrecognized type, or a known type at an unexpected length, now publishes to
  `/mcu/unknown` (raw type + hex payload) and logs once per distinct (type, length) — previously these were
  silently dropped with no visibility at all. ~12 type bytes remain genuinely undecoded (no known field
  layout even in the upstream RE repo) but are no longer invisible.

Purely additive throughout — no existing publisher's semantics changed, so `cliff_guard.py`/`q6a_drive.py`/
`q6a_objmap.py` are unaffected. Architectural note added to docs: this is a read-only MCU tap; we never
craft outbound frames ourselves (driving goes through Valetudo -> AVA), so there's no outbound-side gap.

Files: `scripts/robot/mcu_node.py`, `docs/sensors.md` (full packet table with ROS-exposure column).

---

## 2026-07-11 — MCU protocol gap audit: Status100ms + BatteryStatus were never decoded at all

User asked whether we have gaps in what we decode/expose from the MCU protocol. Audit against
`dreame_mcu_protocol`'s full `TYPES_FROM_MCU` map (13 known types) found `mcu_node.py` only ever handled 3:
Triggers (0x00), Status20ms (0x01), Status10ms (0x02). Two real gaps fixed (pure decode work, no robot
access needed):

- **`Status100ms` (0x03, 10Hz) — was not decoded AT ALL**, despite being confirmed live on the wire (our
  own `mcu_full_dump.py` capture earlier this session). Carries pitch/roll tilt, wheel currents, and a
  **`dust_container_missing` bit** — directly relevant to this project's earlier dustbin-interlock saga
  (vendorErrorCode 8). Sanity-decoded a real captured frame (`06 00 f9 ff 02 00 02 00 00`) -> pitch=0.6deg
  roll=-0.7deg, currents=2mA, flags=0 — sane idle values, struct transfers directly. Now publishes
  `/mcu/status100` (JSON) + `/dustbin_missing` (Bool, latched, edge-logged).
- **`BatteryStatus` (0x2B) — was not decoded at all.** Native voltage/current/temperature/charge_voltage/SoC,
  vs our current 15s `avacmd charge_state` poll workaround. Now publishes `/mcu/battery` (JSON) IF the robot
  actually emits this type — **existence/rate on THIS hardware is unconfirmed**, needs a live capture to
  verify. Note: `battery_current` is UNSIGNED in the reference struct (no direction bit) — calibrate
  charging-vs-discharging against the known avacmd `charge_state` value, don't assume sign.

**Remaining known gaps (not fixed, mostly low-value or genuinely unknown at the protocol level):**
`Status20ms`'s roller/side-brush current and `Status10ms`'s `leftDis`/`rightDis` are decoded into locals
then discarded (brush-jam/wheel-slip diagnostics, no current consumer). `HwInfo`/`McuFwVersionInfo`/
`PingMsg`/`ShutdownMsg`/`McuLog`/`Status500ms` are diagnostic-only, not exposed (low value). A dozen more
type bytes (0x04, 0x06, 0x0B, 0x0D, 0x11-0x13, 0x20-0x26, 0x28) are **undecoded in the reference repo
itself** — genuinely unknown, not just unexposed by us. Outbound (ToMcu_*) traffic is architecturally out
of scope — we're a read-only tap; driving goes through Valetudo -> AVA, we never craft MCU frames ourselves.

Files: `scripts/robot/mcu_node.py`, `docs/sensors.md` (full packet table with ROS-exposure column).

---

## 2026-07-11 — Full MCU Triggers bit-map found: front-bottom IR cliff sensors ARE decodable

User pointed out real IR sensors at the front bottom of the chassis and asked whether we have access.
Re-cloned `github.com/dimik/dreame_mcu_protocol` (not kept locally per policy) and found its `Triggers`
class — a complete 7-byte bit map that matches our own captured payload length exactly. This OVERTURNS the
2026-07-09 "byte[1] is wheel-drop, not forward cliff-IR" conclusion: byte[1] genuinely IS the forward/rear
drop-view IR cliff sensors (`d_view_lf/lmf/rmf/rf` front, `d_view_lb/rb` rear) — we just never broke the
single OR'd byte into its 6 individual bits. Validated the decode against our own real captures AND every
historical calibration value from that session (0x10->bumper, 0xc0->both wheels floating [explains the old
"hard push" mystery — a shove can rock the chassis], 0x0f-> all 4 front sensors when fully lifted, 0x08->
one corner) — all consistent, no surprises.

Also found: `byte[2]/[3]` are `ir_dock_*`/`ir_field_*` — dock-homing IR beacon channels, explaining the
rapid ambient flicker seen in raw captures (unrelated to cliff, just beacon/ambient IR noise). And
`Status20ms`'s `edgeDis` (int16, mm) field has been DECODED but never PUBLISHED since day one (a dead local
variable) — a continuous (50Hz, not event-gated) distance-ish reading, unlike the event-driven Triggers
bits.

**Implemented (additive, no change to existing safety semantics):** `scripts/robot/mcu_node.py` now has
`decode_triggers()` (pure bit math, no bitstring dependency) publishing `/cliff/front` (OR of the 4 front
d_view bits), `/cliff/rear`, `/wheel_floating` (separated from bumper), the full nonzero-field dict as JSON
on `/mcu/triggers`, and `/cliff/edge_dist` (m) from the previously-unused edgeDis. The original `/cliff`
(byte[1]!=0) and `/bumper` (byte[0]!=0) are UNCHANGED — `cliff_guard.py`/`q6a_drive.py` keep working exactly
as before; this is pure additive visibility.

**Open, needs a live re-test:** whether `/cliff/front` actually trips when the robot is stationary at a
real edge (the 2026-07-09 static hold test read 0 throughout — possibly an AVA active-driving-only sampling
gate, or the hold geometry not putting the sensor windows over the void) — vs while actively driving. Also
`edgeDis`'s meaning/threshold is completely uncalibrated. Both pending robot battery charge + Q6A power-on.

Files: `scripts/robot/mcu_node.py`, `docs/sensors.md` (full bit-map table).

---

## 2026-07-11 — Odyssey has direct WiFi access to the robot (no Q6A hop needed)

Discovered while diagnosing an unreachable Q6A: the Odyssey (this dev box) is ALSO on the home WiFi LAN
(`wlo2` 192.168.1.150/24) and has direct `dreame-wifi` SSH + Valetudo REST access to the robot
(192.168.1.213) — no need to route through the Q6A at all. Useful whenever the Q6A is down/rebooting/
brownout-off but you still need robot state (battery, Valetudo attributes, avacmd).

Also: confirmed the Q6A `q6a-brownout` service is doing its job — the Q6A went unreachable (wired link
`enp2s0` on the Odyssey went DOWN, matching a dead link-partner) exactly when the robot's battery hit ~12%
and auto-docked to charge. Not a fault; the brownout service power-off is designed to protect against a
dirty cut as the 14.8V rail sags under charging handoff. Recovery: let the robot charge, then power the
Q6A back on manually (no auto-poweron observed).

---

## 2026-07-10 — Edge-following rewritten for LiDAR (line-fit PD), camera version scrapped

User correction: a 360deg LiDAR is the right sensor for edge/wall following (as on their other camera-less
robot); the camera/MiDaS version solved the wrong problem (forward depth can't measure a lateral distance).
Researched the field (F1TENTH two-ray geometry vs sector line-fit; line-extraction/SLAM front-ends) and
rebuilt `scripts/companion/q6a_edge_follow.py` accordingly.

**Calibration pulled from source + a LIVE scan (not assumed):**
- `/scan` convention is deterministic in `lds_scan_node.py`: LDS is CW, the node negates it -> ROS CCW,
  bin 0 = forward, 360 bins @ 1deg, range_min 0.10 m. So the "mirrored world" risk is handled in code; only
  a ~0-5deg forward-zero offset remains (needs a known-wall / SLAM cross-check).
- Body radius = 0.175 m (D10s Pro = 350 mm dia, spec) -> setpoint = 0.175 + gap. Round chassis => fore/aft
  turret offset shifts lateral half-width <5 mm; base_link->laser TF is identity, so lateral math holds.
- Live scan: ~117/360 bins finite, stable across 14 frames (not spin-up), dense contiguous arc where a
  surface is (37-43/45) and empty arcs = open space >8 m. Resolves to 0.142 m. This SPARSITY is exactly why
  line-fit over two-ray: any two fixed rays are often dropouts; fitting 30-70 side points is robust.

**Controller:** PCA least-squares line fit over the follow-side sector (+/-40deg) -> perpendicular distance
`d` + wall heading `psi`; PD `turn = KP*(d-setpoint) + KD*psi` mapped to the Valetudo {velocity, angle}
heading command (diff-drive, not Ackermann). No I-term (continuous re-estimation). Corner handling: front
sector < FRONT_STOP -> rotate away (concave); too few inliers / fit_std too high -> curve toward side to
re-acquire (convex/lost). Fit-quality gate (MAX_FIT_STD 0.06 m) rejects clutter. Safety: wheel-drop /cliff
hard-stop (the only sensor for a BARE drop-off — horizontal LiDAR is blind to a railless void), stale-scan
stop, bounded by --seconds. `--dry-run` estimates + logs d/psi/inliers/fit_std with zero motion.

**Verified live (dry-run):** estimator collects side points, fits, and correctly reports WALL-LOST on the
current cluttered spot (n=73 but fit_std=0.355 m >> 0.06 -> not a wall). Pending: position the robot at a
real straight wall to confirm d~=setpoint & psi~=0, then a supervised low-speed run to fix STEER_SIGN + tune
KP/KD. Data path runs over WiFi (192.168.1.213) — the USB gadget link is down but unused.

---

## 2026-07-10 — CORRECTION: verified slam root cause — it was NOT FastDDS (controlled evidence)

Rigorously re-tested the "CycloneDDS fixed slam" claim from 2026-07-09. **It does not hold up.** Controlled
experiments on the Q6A (isolated ROS_DOMAIN_ID=43, identical pub/sub, both RMWs):

- **FastDDS re-match (item 1): REFUTED.** An established subscriber re-matched a *restarted* publisher in
  ~2-3 s on FastDDS — IDENTICAL to CycloneDDS (both drop to 0 during the kill gap, recover to full rate).
  `NODE_NAME_UNKNOWN` reproduced as a **stale ros2-daemon artifact**: after `ros2 daemon stop`, FastDDS
  resolves node names fine (== CycloneDDS). Not a FastDDS discovery defect.
- **`ros2 topic hz` (item 2): stated mechanism REFUTED.** Source: it computes rate from receipt-time
  intervals of RECEIVED msgs and prints nothing until >=1 s accumulates / nothing if none arrive. A constant
  10 Hz topic reads a correct ~10.0 immediately AND settled on BOTH RMWs. The real confound behind the many
  "SILENT" readings: the topics were **intermittent/sparse** (turret-gated `/scan`, movement-gated slam
  `/pose`) measured in **short 4-6 s windows** -> genuinely little/no data -> misread as "topic dead."
- **QoS mismatch (item 3): REFUTED.** All pairs compatible (/scan RELIABLE pub -> RELIABLE+BEST_EFFORT subs;
  /pose, /odom_laser RELIABLE<->RELIABLE; all VOLATILE). Not the cause.
- **CPU/latency (item 4): CONFIRMED comparable.** 50 KB @ 20 Hz on the Q6A: FastDDS and CycloneDDS both ~5%
  CPU (pub+sub each); latency mean 2.50 vs 2.44 ms, p95 5.55 vs 4.26 ms. CycloneDDS not heavier (slightly
  better tail). So the switch was middleware-behavior/cosmetic, not resource.

**Actual root causes of the slam saga (what really fixed it):** (1) the `/scan` chain was genuinely broken
after reboot — `ring_forward` stale-mmap + `lds-scan-node` connect-while-idle (REAL bugs, fixed); (2)
slam_toolbox only publishes `/pose` while the robot MOVES (minimum_travel gate) and needs continuous `/scan`
at sensor registration; (3) `ros2 topic hz` misled diagnosis on intermittent topics. **CycloneDDS coincided
with these fixes and gave a cleaner CLI (node names resolve without daemon fuss), which aided debugging, but
it was not the fix.** Keeping CycloneDDS is fine (comparable overhead, marginally better tail latency, clean
discovery) — just not for the reason originally stated.

Unverifiable post-hoc: the exact original re-match "failures" (no old logs kept) — but since FastDDS
re-matches fine in controlled tests, those were almost certainly the intermittent-topic + measurement
artifacts above, not a DDS defect.

---

## 2026-07-10 — TTS volume root cause (ALSA mixer), vision-seek drive, edge gate hardening

**Volume — real root cause found after several wrong knobs.** TTS was too loud and NOTHING software reduced
it: not Valetudo's `SpeakerVolumeControlCapability`, not mediad's per-play volume field (`single,file,VOL`),
not an ffmpeg `volume=` filter on the OGG. mediad plays at whatever the **robot ALSA hardware mixer** is set
to. Fix: `amixer -c 0 sset 'digital volume' 22` (~35% of 0-63) — verified quieter and that it **persists**
(mediad doesn't reset it per-play). Made persistent in `_root_postboot.sh` (resets on reboot otherwise).
`speak.py` also gained a real ffmpeg `volume` filter + `SPEAK_VOL`, but the ALSA mixer is the effective control.

**Vision-seek drive.** `q6a_drive` now steers toward the highest-confidence furniture in view
(`/vision/detections`, proportional to bbox x-offset; `Q6A_DRIVE_SEEK`/`STEER_SIGN`) so a mapping drive
actually points the camera at furniture instead of wandering blind. Keeps all safety gates.

**objmap conf lowered 0.55 -> 0.45.** With the class allowlist now blocking hallucinations, confidence no
longer needs to be high; lowered it to catch borderline REAL furniture (fridge ~0.5).

**Edge gate hardened (false "Edge ahead" again).** Raised sharpness bar 3.0 -> 4.0 AND now require LiDAR
corroboration for the edge advisory: a real down-edge = an OPEN forward LiDAR sector (beam clears the drop);
a false floor-discontinuity (rug/threshold/shadow/furniture base) still returns LiDAR -> not open -> no
warning. Also self-suppresses when idle (turret parked -> stale LiDAR). `Ouch!` gained a 3 s cooldown
(bumper flickers during back-off+turn recovery, over-announced).

Files: `scripts/robot/speak.py`, `scripts/robot/_root_postboot.sh`, `scripts/companion/cliff_guard.py`,
`scripts/companion/q6a_objmap.py`, `scripts/companion/q6a_drive.py`.

---

## 2026-07-09 — Stop hallucinated object recognition (confidence + persistence gating)

The robot announced/mapped phantom objects ("cat"/"laptop on the floor"). Evidence (live capture): the model
is fine — real furniture detects high (chair 0.72-0.81, tv/fridge 0.62+) — but COCO false positives on textured
floor sit at ~0.44-0.55, and the pipeline was far too permissive: detector at **conf 0.1**, announcer spoke
anything **>=0.5** for 3 frames. Clean gap between real (>=0.62) and phantom (~0.5), so raised the bars:
- `q6a_vision` detector CONF 0.1 -> **0.30** (0.1 leaked junk into ByteTrack).
- `q6a_announce` MIN_CONF 0.5 -> **0.6**, MIN_HITS 3 -> **5** (must be confident AND persist 5 frames).
- `q6a_objmap` MIN_CONF 0.4 -> **0.55** + **persistence gate**: only publish objects seen **>=3x** (drops
  one-off false positives; bump 'obstacle' marks exempt). (task 11)

**Follow-up — confidence wasn't enough, added a CLASS ALLOWLIST.** The "cat" hallucination kept peaking >0.6
for 5+ frames (announced again), and raising the bar further would cut real furniture (tv 0.62). So both
`q6a_announce` and `q6a_objmap` now accept only plausible indoor classes (chair, couch, bed, dining table, tv,
refrigerator, oven, microwave, sink, toilet, potted plant, bench, book, clock, vase, ...; announcer also
person) via `Q6A_ANNOUNCE_ALLOW` / `Q6A_OBJMAP_ALLOW`. Hallucination-prone classes (cat, laptop, pizza, ...)
are dropped regardless of confidence. Verified: YOLO still detects "cat" (~0.5-0.65) but it is no longer
announced or mapped; real furniture unaffected.

Files: `scripts/companion/q6a_vision.py`, `q6a_announce.py`, `q6a_objmap.py`.

---

## 2026-07-09 — slam_toolbox FIXED via CycloneDDS (root cause: FastDDS + a lying `ros2 topic hz`)

The "slam is broken" saga resolved. **slam_toolbox works** — the failure was two compounding things, neither a
slam bug:
1. **`ros2 topic hz` / `echo --once` were unreliable** — a fresh-subscriber discovery race reported topics
   SILENT even while they were flowing to *established* subscribers (objmap, the tf2 buffer). This drove a
   long series of wrong "slam produces no /pose" conclusions. Reliable indicators (node logs, `tf2_echo`,
   objmap's "switched pose source to /pose") told the true story.
2. **FastDDS discovery fragility** — `_NODE_NAME_UNKNOWN_` in the CLI, existing-subscriber-vs-restarted-
   publisher re-match gaps, and message-filter flakiness.

Fix: **switched RMW to CycloneDDS** (`ros-jazzy-rmw-cyclonedds-cpp`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
in the shared EnvironmentFile; all nodes restarted). Result: clean discovery (node names resolve), the
node-restart re-match fragility is gone, and — verified — **`map→base_link` resolves** (so slam's `map→odom`
is live) and **objmap consumes slam's `/pose`** to build a coherent map: `chair@(-1.6,-1.5)` with 1121 merged
observations + refrigerator + chairs across a consistent map frame (drift-corrected, far less fragmented than
raw laser-odom).

Also fixed this session (all committed): object announcer, MiDaS+LiDAR cliff (non-blocking, directional,
sharpness discriminator vs smooth floor), safety-gated drive controller, bumper "Ouch!" + back-off/turn
recovery + obstacle-on-map, ring_forward self-heal, KillSignal clean restarts.

Files: `scripts/companion/systemd/ippolit-robot.env` (RMW_IMPLEMENTATION); deployed to
`/etc/default/ippolit-robot` on the Q6A + `ros-jazzy-rmw-cyclonedds-cpp` installed.

---

## 2026-07-09 — Fix false "Edge ahead": sharpness discriminator (cliff vs smooth floor)

The MiDaS floor-drop detector false-alarmed "Edge ahead" on open floor / doorways. Root cause found from a
live capture: a **smooth floor's perspective decay** (`154->142->...->58->44->34->27`) has a steepest single
step of ~0.24-0.36 — overlapping the shallow end of real edges — so `max_step` alone can't separate them.
A **real** edge is a sharp discontinuity (`...111.5->42...` = 0.62 at one bin, neighbours smooth).

Fix: `/vision/floor` now reports **sharpness = max_step / median|step|** per sector. A real drop-off is sharp
(~5-7); a smooth gradient is ~1.1-2.2. Cliff detection (cliff_guard + q6a_drive) now requires BOTH a drop
magnitude AND sharpness >= 3.0. Verified: the false-positive floor (max_step 0.24, sharp 2.22) is now
classified "smooth floor, no edge" and cliff_guard stays quiet; real edges (sharp discontinuity) still fire.

Files: `scripts/companion/q6a_vision.py` (sharpness in floor profile), `scripts/companion/cliff_guard.py`
(require sharp), `scripts/companion/q6a_drive.py` (require sharp for DROP-AHEAD stop).

---

## 2026-07-09 — Bumper handling: "Ouch!" + back-off/turn recovery

The robot kept pushing helplessly into a thin table leg (LiDAR-invisible: below/around the scan plane).
Added physical-bumper handling end-to-end, calibrated live against the actual leg.

- **`mcu_node` bumper decode:** MCU Triggers frame byte[0] = bump/contact bits. Calibrated 2026-07-09:
  0x00 clear vs **0x10 = front bumper** pressed (0xc0/0xd0 under hard push + wheel-drop). Publishes latched
  `/bumper` (Bool) + `/bumper/raw` (UInt8); logs `BUMP HIT`/`clear` on the edge. Sampled only while active
  (manual control armed), same as the cliff byte.
- **`cliff_guard` says "Ouch!":** on `/bumper` rising edge -> `/robot/speak` "Ouch!" (throttled 1.5 s) so a
  bump is audibly confirmed. Verified: 3 hits -> 3 "Ouch!"s.
- **`q6a_drive` bump recovery:** forward | reverse | turn state machine. Front bumper -> reverse 0.8 s +
  turn-arc 1.3 s (55deg) -> resume forward. LiDAR obstacle (< 0.4 m) -> turn away (keep exploring). Drop-ahead
  (MiDaS) + wheel-drop (/cliff) still HARD-stop. Reverse is kept short (rear has no drop sensing).
  Verified live: drove into the leg -> Ouch -> back off + turn -> continued, repeatably.

Possible refinement: a single 55deg turn can re-approach the leg in a tight corner (it recovered each time,
never stuck) — turn-until-front-clear or alternating turn direction would clear faster.

Files: `scripts/robot/mcu_node.py` (bumper decode), `scripts/companion/cliff_guard.py` (Ouch),
`scripts/companion/q6a_drive.py` (recovery state machine).

---

## 2026-07-09 — Pipeline robustness: self-healing ring_forward + clean-restart DDS

Attacked the three fragilities from the mapping session. Two fixed, one narrowed.

**FIXED — `ring_forward` self-heal (the "/scan dead after reboot" dance).** Root cause: the serialtap
creates/inits `/tmp/lds_ring.buf` LAZILY (first ttyS3 read = turret spin), *after* ring_forward starts at
boot, so the old code mmap'd once (missing file / not-yet-valid magic / stale inode) and never recovered.
Rewrote `handle()` to (re)open the ring whenever it appears, wait for full size + valid magic, and re-mmap on
inode change. Proven: forwarder streams live bytes (8 KB pulled off :9901) the moment the turret spins, no
manual restart. Deployed to the robot chroot.

**FIXED — clean node restarts (DDS re-match).** Nodes died on systemd's default SIGTERM without destroying
their DDS participant -> stale endpoints -> peers never re-matched a restarted publisher. Added
`KillSignal=SIGINT` (+ `TimeoutStopSec=8`) to every unit (rclpy/rclcpp handle SIGINT -> clean shutdown).
Plus the operational rule: **restart publishers before consumers** — a fresh subscriber reliably discovers an
existing publisher, whereas an existing subscriber does NOT promptly re-match a *restarted* publisher (FastDDS
limitation). With that order, `lds-scan-node -> laser_odom -> /odom_laser` came up clean and objmap placed
furniture.

**NARROWED — two residual timing quirks (documented, not yet auto-healed):**
- `lds-scan-node` must (re)connect to the forwarder *while the turret is spinning*; if it connects idle it
  stays stuck until reconnected during a stream. (Its TCP reader should recover on first data — TODO.)
- **slam_toolbox `/pose` still flaky.** It activates (lifecycle OK) and the full TF chain is present
  (`odom->base_link` + static `base_link->laser`), but its tf2 message-filter drops every scan
  ("queue is full") -> no `/pose`. Worked yesterday; flaky today after the restart churn — a scan/TF timing
  or message-filter-queue issue needing dedicated work. **objmap runs fine on laser-odom meanwhile** (places
  chairs/tv), so mapping is unblocked; the cost is odom drift (fragmentation) without slam's correction.

Net: `/scan` no longer needs the manual forwarder-restart dance after a robot reboot, and node restarts are
clean if done publisher-first. slam `/pose` reliability is the remaining open item.

Files: `scripts/robot/ring_forward.py` (self-heal), `scripts/companion/systemd/*.service`
(`KillSignal=SIGINT`), Q6A drop-ins `/etc/systemd/system/*.service.d/killsignal.conf`.

---

## 2026-07-09 — Mapping drive works: safety-gated drive controller + person-filtered object map

**Milestone:** first clean furniture-mapping drive. `q6a_drive` drove the robot forward under full cliff
safety and objmap populated **chairs, zero persons** — the whole SLAM+vision+objmap+announcer+cliff stack
running together during motion.

**`q6a_drive.py` (new) — bounded, safety-gated forward drive burst.** Q6A-side control node (subscribes the
sensors ONCE, ~7 Hz loop — no per-pulse ssh, so it's smooth/fast). Drives forward at a given velocity for N
seconds, stopping immediately on: wheel-drop (`/cliff`), MiDaS floor-drop ahead (`/vision/floor` center >=
0.42), near obstacle (LiDAR fwd < 0.4 m), or **stale/absent sensors (refuses to drive blind)**. Forward-only
by design (2nd-floor edge is behind-right). Two gotchas solved: (1) **arm-first** — `/scan` only flows while
manual control is armed (turret spins only then), so enable BEFORE waiting for sensors, else deadlock;
(2) a startup warmup so a fresh node's DDS discovery has time (never false-abort at launch). Verified 0.28 and
0.35 bursts, clean stops.

**`q6a_objmap` dynamic-class filter.** `person`/`cat`/`dog`/`bird` are never added to the persistent map —
a supervising human otherwise smears into hundreds of "person" hits across the odom track (saw 584×). Now
furniture-only.

**Known issues surfaced:** (1) **odom drift fragments** one chair into ~3 nearby entries — the slam_toolbox
drift-correction job (its `/pose` was down this session; objmap fell back to laser-odom, which drifts).
(2) **DDS re-match fragility** — restarting a node (objmap, lds-scan-node) drops its topic matches; had to
restart objmap *mid-drive* (while `/scan`+`/odom_laser` were live) for it to re-discover. Worth hardening
(fixed discovery / restart order / a persistent drive service). (3) `/scan` chain still needs the
`ring_forward` re-mmap dance after a Q6A reboot.

Files: `scripts/companion/q6a_drive.py` (new), `scripts/companion/drive_sense.sh` (new helper),
`scripts/companion/q6a_objmap.py` (dynamic filter).

---

## 2026-07-09 — Object announcer + cliff/stair fall protection (3-layer, validated)

**Object announcer (`q6a_announce` / `q6a-announce.service`):** new node — watches `/vision/detections` and
speaks new objects ("I see a chair") via `/robot/speak` -> audio-bridge -> Piper. Debounced (min_conf 0.5,
min_hits 3, per-label cooldown 25 s, global 3 s spacing) so it narrates, not spams. Verified audible.
(Speaker was at vol 60, not the vol-0 mute from the manual-nav work; bumped to 70.)

**Cliff / stair fall protection (SAFETY — robot is on the 2nd floor near a ladder).** Key fact:
**a horizontal 2D LiDAR cannot see a down-staircase** — a drop reads as "wide open", so "drive toward the
open direction" is exactly the trap. Real protection = the robot's own downward IR cliff sensors. Built a
guard chain, tested two ways (lift, and holding the robot at an actual edge):
- **`mcu_node` cliff decode:** the MCU Triggers frame (type 0x00) payload **byte[1]** — publishes latched
  `/cliff` (Bool) + `/cliff/raw` (UInt8); conservative rule byte!=0 -> stop (a false stop is safe).
- **`cliff_guard` / `q6a-cliff-guard.service`:** on `/cliff` rising edge -> immediately DISABLE
  HighResolutionManualControl over REST (x3 for WiFi) + speak "Cliff detected. Stopping." Latches, re-arms.
- **AVA native (discovered):** lifting throws AVA's own "wheels not in contact with ground" error — an
  independent reflex under ours.
- **⚠️ CRITICAL FINDING — byte[1] is WHEEL-DROP, not forward cliff-IR.** Lift test passed (byte 0x08 ->
  `/cliff` true -> guard hard-stop + speak -> re-arm on set-down; robot recovered to idle). BUT holding the
  robot at a **real edge** (front over the drop, wheels still on the floor) read **byte=0 — no detection**
  (`/cliff/raw` @ 8 Hz steady 0). So `/cliff` only fires once wheels have LEFT the ground — a LATE backstop,
  NOT before-the-edge protection. (Open: whether AVA polls forward cliff-IR only while actively driving —
  untestable without driving at the edge.)
- **✅ FORWARD-DROP DETECTOR (MiDaS + LiDAR fusion) — built + verified live at the real ladder.**
  `q6a_vision` now publishes `/vision/floor` = row-median depth profile of the bottom 45% of the MiDaS map
  (floor ahead) + `max_step` (largest relative fall between adjacent bins; MiDaS is affine-invariant so we
  use RELATIVE steps). `cliff_guard` fuses three layers: (1) MiDaS `max_step >= 0.30` -> STOP (primary);
  (2) weaker step `>= 0.24` + forward LiDAR sector anomalously open (median > 3.5 m / mostly no-return =
  stairwell signature) -> STOP; (3) `/cliff` wheel-drop backstop. On trip: DISABLE manual control (REST x3)
  + speak "Drop off ahead" + publish `/cliff/ahead`; latches, re-arms with hysteresis.
  **Calibrated at the actual edge:** facing room `max_step` <=0.205; facing the drop 0.345-0.65 (square-on
  0.58-0.65). **Verified live:** facing the room = silent; the instant the robot faced the drop ->
  `DROP-OFF AHEAD (midas, step=0.43) — stopping` + manual control cut (and wheel-drop `/cliff` also fired).
   So there is now a **before-the-edge stop**, not just the late backstop.
- **Refined to non-blocking + directional (per user: "travel by the edge, don't freeze at it"):** the MiDaS
  layer is now **advisory** — it publishes `/cliff/ahead` (drop in the CENTER forward path) + speaks "Edge
  ahead", but **never disables manual control** (only the wheel-drop `/cliff` does the hard e-stop). And
  `/vision/floor` now carries **per-sector L/C/R** drop (`sectors:{left,center,right}:[max_step,step_at]`) so
  the drive controller can tell drop-dead-ahead (center -> no forward) from drop-to-the-side (edge alongside
  -> travel parallel) and glide by an edge instead of stopping at it. Wheel-drop stays the hard backstop.
  TODO: an edge-following drive behavior that holds a set lateral distance (~5-10 cm) using the L/R sectors.

Files: `scripts/companion/q6a_announce.py`, `scripts/companion/cliff_guard.py`,
`scripts/companion/systemd/{q6a-announce,q6a-cliff-guard}.service` (new), `scripts/robot/mcu_node.py`
(Triggers/cliff decode, additive).

---

## 2026-07-08 — Working semantic object map via companion laser-SLAM odometry (2.4 done)

**Milestone:** the companion now builds a semantic object map during a manual drive, localizing **itself**
from the LiDAR — no Valetudo pose, no wheel odom, no autonomous nav. A clearance-gated slow drive produced a
**12-object map** (`tv` 268×, `chair` 204×, `person`, …) with correct relative placement.

**LiDAR in manual control (the "unpatch"):** `/scan` is gated OFF during `manual_control` by the fanoff design
— the `fanoff_flag` gate daemon clears `/tmp/lidar_allow` in manual mode, so the shim rewrites AVA's turret
`spin=01` → `park=00`. Confirmed AVA *wants* it spinning (strace: `3c 02 14 04 01` on ttyS4). Fix for mapping:
**stop the gate daemon + hold `/tmp/lidar_allow` present** → the shim passes AVA's own spin → turret spins →
`/scan` live at 5.2 Hz. (Non-persistent; a reboot restores the daemon. The LDS `ring_forward` must also be
restarted once the ring first appears — it mmaps lazily on the first tapped read.)

**Hard blocker found — no pose while manually driving:**
- **work_mode 17 blocks autonomous nav** — `BasicControl start`/`home` + `GoToLocation` all HTTP 400. Manual
  control is the only drive mode.
- **Valetudo reports no pose in manual mode** — `robot_position` frozen (unchanged after a >1 m move);
  `/odom/wheel` silent (D10s MCU sends no telemetry unless active, and even when moving it stayed silent).
  So the object map had been pinning everything to one frozen pose.

**Fix — companion laser odometry (`q6a-laser-odom` / new `q6a_laser_odom.py` + service):** numpy point-to-point
**ICP scan-matcher** on `/scan` → live `odom→base_link` TF + `/odom_laser` + static `base_link→laser`. Killed a
catastrophic **180° yaw-flip** (a constant-velocity ICP prior ran away): now seeds each match from zero and
rejects implausible per-frame steps (>0.15 m / >8.6°). objmap re-pointed at this pose via a systemd drop-in
remap (`-r /odom:=/odom_laser`), no code change.

**Driving safety:** added **per-pulse LiDAR clearance gating** to the manual-drive loop (stop if FRONT < 0.6 m)
after the robot repeatedly rammed a wall when driven blind (no pose feedback). Confirmed driving works by
watching sector clearances change (`0.25 m` wall → `1.6 m` clear on reverse).

**Rough edges (next):** odom **drift** fragments one object into a few merged entries; low-conf YOLO false
positives (`bird`, `umbrella`). Supersedes the pose/drive plan in decisions **D2/D3** → see **D7** in
`docs/companion-autonomy.md`.

**slam_toolbox WORKING (root cause found after two wrong diagnoses):** Jazzy's slam_toolbox 2.8+ is a
**LIFECYCLE node** — launched bare via `ros2 run` it sits in `unconfigured` forever: the Ceres solver loads in
`on_configure()` and the `/scan` subscription only exists after `on_activate()`, so it logs nothing, subscribes
nothing, and errors nothing. (Earlier notes called it a "construction deadlock" then a "message-filter
swallowing scans" — both wrong; gdb showed the main thread healthily idle in `rclcpp::spin()`, and the
"2 subscribers on /scan" were our own laser-odom + objmap, slam had zero.) Fix:
`ros2 service call /slam_toolbox/change_state {configure, activate}` — instantly `Registering sensor`, live
`/map` occupancy grid, `map→odom` TF, `/pose`. Made persistent via `slam_lifecycle_up.sh` (state-driven,
idempotent) as `ExecStartPost` in the unit; **service enabled**, restart-tested end-to-end.

**TF-tree fix (REP-105):** valetudo-bridge published `map→base_link` while laser-odom publishes
`odom→base_link` — **two parents for `base_link`**, an invalid TF tree that only "worked" because the bridge's
pose was frozen. Fixed: bridge drop-in remaps `/tf:=/tf_valetudo /map:=/map_valetudo /odom:=/odom_valetudo`
(keeps `/robot/status` `/battery`), `valetudo_bridge.py` now `parse_known_args()` + forwards `--ros-args` to
`rclpy.init` (its argparse used to reject remaps). Canonical chain now:
`map ─slam_toolbox→ odom ─laser_odom→ base_link ─static→ laser`, verified single-parent + resolving.

**objmap on the SLAM pose:** `q6a_objmap` now subscribes slam's map-frame `/pose` (PoseWithCovarianceStamped,
`Q6A_OBJMAP_POSE_TOPIC`) and prefers it over `/odom` once it flows (graceful fallback to laser odom when SLAM
is down; never mixes frames). Verified: "switched pose source to /pose (slam map frame)", `/map` grew
123×147→158×164 over a clearance-gated drive, SLAM error-free. Full objects-in-map-frame populate run needs a
proper drive around the room (camera faced walls in the test corner) — next session.

Files: `scripts/companion/slam_lifecycle_up.sh` (new), `scripts/companion/systemd/q6a-slam-toolbox.service`
(ExecStartPost + enabled), `scripts/companion/q6a_objmap.py` (/pose source), `scripts/robot/valetudo_bridge.py`
(ros-args passthrough), bridge tf-remap drop-in on the Q6A. (gdb was `apt install`ed on the Q6A during diagnosis.)

Files: `scripts/companion/q6a_laser_odom.py` (new), `scripts/companion/systemd/q6a-laser-odom.service` (new),
`scripts/companion/slam_toolbox.yaml` (new), `scripts/companion/systemd/q6a-slam-toolbox.service` (new,
disabled), objmap remap drop-in on the Q6A, `docs/companion-autonomy.md` (D7 + D2/D3 superseded notes).

---

## 2026-07-08 — Object-map node (2.4) + side-by-side YOLO|MiDaS view

**Object map (`q6a_objmap` / `q6a-objmap.service`):** new node subscribing `/vision/detections` + `/odom` +
`/scan`. Per confident detection: bearing = bbox x-center + camera H-FOV (~110deg est); range = `/scan` at
that bearing (metric; needs the turret spinning); project to the map frame via the robot pose; accumulate
persistent objects (same class within 0.5 m merged, position running-averaged); publish `/object_map` (JSON) +
`/object_markers` (RViz MarkerArray). Verified plumbing: `active`, publishes `{"objects":[]}` while docked (no
`/scan`, camera at the wall). **Calibration** (H-FOV, bearing sign, camera-yaw) + **room-tagging** (Valetudo
segment per object) pending the first drive.

**Vision view:** `q6a_vision` `:8093` now serves a side-by-side composite **[YOLO-boxed RGB | MiDaS depth
colormap (red=near)]** so both nets are visible in one stream; reachable from the Odyssey at
`http://192.168.20.2:8093/` (opened in mpv).

Files: `scripts/companion/q6a_objmap.py` (new), `scripts/companion/systemd/q6a-objmap.service` (new),
`scripts/companion/q6a_vision.py`.

---

## 2026-07-08 — Companion runs over WiFi (robot free to drive) + two config-bug fixes

**What:** Flipped the companion to reach the robot over **home WiFi** (`192.168.1.213`) instead of the USB
cable, so the robot can drive free. Set `/etc/default/ippolit-robot` to `ROBOT_ADDR=192.168.1.213`
`ROBOT_SSH=robot-wifi` + added a `robot-wifi` ssh alias on the Q6A. Two bugs surfaced + fixed:
- **systemd inline-comment bug**: a trailing `# comment` on `EnvironmentFile=-/etc/default/ippolit-robot`
  was parsed as part of the file PATH -> env file never loaded -> services fell back to the USB default IP.
  Comments moved off the directive lines.
- **DDS discovery**: with `usb0` down, Jazzy's default `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` failed multicast
  discovery across wlan0/enp1s0 -> topics invisible. Set **LOCALHOST** (all ROS nodes are on the Q6A) ->
  robust loopback discovery. Also disabled robot `wlan0` power-save (variable latency was tripping the nodes'
  2 s connect; runtime-only — add to the robot boot hook if WiFi becomes the default).

**Verify:** over WiFi — `/odom/wheel` @67 Hz, `/battery` 66% (charging), `/vision/detections` publishing;
robot cam confirmed (black while docked = facing the wall). Robot free to drive.
Files: the 6 companion unit files + `ippolit-robot.env`.

---

## 2026-07-08 — Companion robot-link switchable USB<->WiFi (enables free-roam driving)

**What:** All 6 companion services (valetudo-bridge, mcu-node, lds-scan-node, audio-bridge, q6a-vision,
q6a-brownout) now read the robot address + ssh alias from a single optional `/etc/default/ippolit-robot`
(`ROBOT_ADDR`, `ROBOT_SSH`), defaulting to the USB gadget (192.168.10.1 / robot-usb) when absent (current
behavior unchanged). Set `ROBOT_ADDR=192.168.1.213 ROBOT_SSH=robot-wifi` + restart -> the companion talks to
the robot over **home WiFi** instead of the USB cable, so the robot can **drive free** (the USB tether would
yank when it moves). Robot-side services bind 0.0.0.0 - no robot change. (systemd `$$` escaping so bash does
the `${VAR:-default}` expansion.)

**Why:** the object-map / SLAM work needs the robot moving; WiFi frees it from the cable. WiFi is
higher-latency/jitterier than USB - fine for the slow Valetudo-GoTo object-map drive; USB stays preferred for
tight obstacle avoidance. **Deploy-tested when the devices are next powered on.** Caveat: needs the AP to
allow client-to-client traffic (no AP isolation).

Files: `scripts/companion/systemd/ippolit-robot.env` (new) + the 6 unit files.

---

## 2026-07-08 — Docs: sync CLAUDE.md to the robot-brain architecture

**What:** Added a prominent current-architecture banner at the top of `CLAUDE.md` pointing to
`docs/companion-autonomy.md`, and corrected the two most-drifted sections: the chroot **ROS is removed**
(runs on the companion; chroot keeps only ROS-free `ring_forward.py`/`speak.py` + video/TTS tools), and the
**on-device LLM is retired**. Marks the robot-side ROS/vision/LLM descriptions as superseded.
File: `CLAUDE.md` (`docs/companion-autonomy.md` already current).

---

## 2026-07-08 — Brownout guard (P3.2): clean poweroff before the robot battery dies

**What:** New `q6a_brownout.py` (non-ROS daemon, `q6a-brownout.service`) watches the robot battery over USB
and, while **discharging**, clean-`systemctl poweroff`s the Q6A at CRIT — the Q6A runs off the robot battery,
so an unclean cut risks the ext4 root + a cDSP wedge (which the Q6A handles badly). Charging state comes from
**AVA `charge_state`** (robot `/tmp/charge_state`) because **Valetudo's battery flag is broken on the D10S Pro
(stuck `none`)**; level from Valetudo. Thresholds: **WARN 25%** (log + TTS warning, optional send-home — off by
default since AVA auto-docks), **CRIT 12%** (clean poweroff). One-shot latches; resets while charging.
Camera-free, drive-free.

**Verify (on-device):** dry-run with forced thresholds → correctly detected `discharging` (charge_state
"not charge") and logged the CRIT clean-poweroff + WARN paths without executing; live service `active` at
91%, monitoring at 60 s poll, no action (above WARN). Files: `scripts/companion/q6a_brownout.py` (new),
`scripts/companion/systemd/q6a-brownout.service` (new), `docs/companion-autonomy.md`.

---

## 2026-07-08 — Phase 2.3: MiDaS depth fused into the vision node (per-detection relative range)

**What:** `q6a_vision` now loads **MiDaS-V2 w8a8 as a 2nd NPU context in the same process** (de-risked: YOLO +
MiDaS coexist in one process — ~13 ms + ~6 ms, no crash). Each frame runs YOLO+ByteTrack + a 256×256 MiDaS
inverse-depth map; each detection gets the **median disparity in its bbox** (higher = nearer), published in
`/vision/detections` (`disp`) and drawn in the annotated `:8093` view (`d<value>`). Full depth map kept in
`Shared.depth` for obstacle use later.

**Verify (live room):** physically correct — **`tv` d7** (far, across the room), **`person` d104** +
**`chair` d104** (near). One puller, one process, both nets on the NPU.

**Metric scaling — deferred/noted:** disparity→meters needs the camera FOV/extrinsics + LiDAR `/scan`, and
`/scan` only flows while the turret spins (parked when docked). Relative depth already orders objects by
distance (feeds the object map 2.4) and flags near obstacles; the metric fit vs `/scan` is a refinement for
when the robot navigates. Files: `scripts/companion/q6a_vision.py`, `docs/companion-autonomy.md`.

---

## 2026-07-08 — Phase 2 kickoff: robot-camera decision + companion vision node (YOLO) + decisions doc

**Camera decision (D6):** frame-grab comparison → **use the robot OV8856** (MJPEG `:8090` over USB), **not
the IMX296**. The OV8856 gives a clean forward-facing room view (TV/table/chair/bed/floor all visible at
robot height); the IMX296 returned only noise (post-reboot CAMSS bringup fault) and its compartment FOV is
unknown. Bonus: the robot hands us JPEG, so the Q6A skips the whole GPU-ISP/demosaic stack.

**`q6a_vision` node (2.2):** pulls the robot MJPEG → JPEG decode → **w8a8 YOLOv8 on the NPU + ByteTrack** →
publishes `/vision/detections` (JSON: label/conf/bbox/track_id) and serves an annotated MJPEG on `:8093`.
Runs as `q6a-vision.service` (systemd; QAIRT 2.42 + ROS env). Crops to the valid 504 rows (camstream pads
672×504→672×672).

**Verify (on-device, live room):** detects `tv` (~0.67) and `chair` with stable ByteTrack IDs; annotated
`:8093` shows correct boxes on the robot's view; `/vision/detections` publishing.

**Docs:** new `docs/companion-autonomy.md` — the robot-brain architecture + decisions **D1–D6** (semantic
object map, reuse-Valetudo-SLAM, drive-via-Valetudo-GoTo/LiDAR-gate, ROS-on-companion, LLM retirement,
robot-camera choice) with rationale.

**Next:** MiDaS metric depth on the robot frames (fuse with `/scan`) → semantic object map → "go to kitchen".
Files: `scripts/companion/q6a_vision.py` (new), `scripts/companion/systemd/q6a-vision.service` (new),
`docs/companion-autonomy.md` (new).

---

## 2026-07-08 — Phase 1.3 (c+d) — audio relocated + chroot ROS removed → ROS FULLY on the companion

**1.3c audio:** New robot `speak.py` (ROS-free: piper/espeak-ng + ffmpeg → localhost mediad) + companion
`audio_bridge.py` (ROS `/robot/speak` subscription that pipes each utterance to `speak.py` over ssh). This
reuses the robot's existing TTS stack (no ~270 MB copy to the Q6A) and works around **mediad binding
`127.0.0.1`** (so the play trigger must run on the robot). Companion `audio-bridge.service`. Verified
end-to-end: `ros2 topic pub /robot/speak` → companion node → robot speaker (owner confirmed hearing it).
Note: had to raise the Valetudo speaker volume **0→60** — it was muted by the fanoff work; safe now since we
drive via Valetudo GoTo (not manual-control, which is what triggered AVA's voice prompt).

**1.3d:** Removed `/data/chroot/opt/ros/jazzy` (**147 MB**) — ROS 2 is no longer installed on the robot
(`/data` 62%→58%). Robot ROS node procs swept; `/scan` publisher count = 1 (companion only).

**PHASE 1 COMPLETE — ROS lives entirely on the companion (Q6A):**
- Services (systemd, boot-enabled): `valetudo-bridge`, `mcu-node`, `lds-scan-node`, `audio-bridge`.
- Topics: `/map`, `/scan`, `/imu/data`, `/odom`, `/odom/wheel`, `/battery`, `/robot/status`, `/robot/speak`.
- Robot = LD_PRELOAD taps + ROS-free `ring_forward.py` (LDS/MCU) + `speak.py` + Valetudo. No ROS.

**Follow-up:** CLAUDE.md still documents the old chroot-ROS layout — update to ROS-on-companion. The old
`scripts/robot/audio_bridge.py` is superseded by `speak.py` + `scripts/companion/audio_bridge.py`.

Files: `scripts/robot/speak.py` (new), `scripts/companion/audio_bridge.py` (new),
`scripts/companion/systemd/audio-bridge.service` (new), `scripts/robot/_root_postboot.sh`.

---

## 2026-07-07 — Phase 1.3 (a+b): companion node services + robot boot hook launches forwarders

**What:**
- **Companion systemd units** for the two sensor nodes — `mcu-node.service` + `lds-scan-node.service`
  (`-p source:=192.168.10.1:{9902,9901}`) — alongside the existing `valetudo-bridge.service`. All three ROS
  nodes are now **supervised + boot-started on the Q6A**.
- **Robot `_root_postboot.sh`**: replaced the three chroot-ROS launches (`valetudo_bridge`, `lds_scan_node`,
  `mcu_node`) with the two ROS-free `ring_forward.py` launches (LDS `tcp/9901`, MCU `tcp/9902`). `audio_bridge`
  kept for now. Deployed to the robot's live `/data/_root_postboot.sh` (backup:
  `_root_postboot.sh.bak.pre-ros-relocate`); **robot `sh -n` passes**. The gadget/networking setup runs
  earlier in the hook, so the swap can't affect the USB link. Repo copy synced.

**Verify:** all 3 companion services `active`; `/odom/wheel` live; robot-side boot-hook syntax clean. On the
next robot reboot it will start the forwarders (not the ROS nodes) — dual-publish risk removed.

**Remaining in 1.3:** relocate `audio_bridge` to the companion (the last chroot-ROS node) → then remove ROS
from the chroot. Files: `scripts/companion/systemd/{mcu-node,lds-scan-node}.service` (new),
`scripts/robot/_root_postboot.sh`.

---

## 2026-07-07 — Phase 1.2: forward robot LiDAR + IMU/odom to companion ROS (taps stay, ROS moves)

**What:** New `ring_forward.py` (robot, **ROS-free**, stdlib-only): streams a serial-tap tmpfs ring's raw bytes
over TCP. `lds_scan_node.py` + `mcu_node.py` gained a `source` param — local tmpfs ring (on-robot, the
unchanged default) **or** `host:port` TCP from the forwarder (on-companion); the decode/publish is byte-for-byte
identical either way. So the robot keeps only the LD_PRELOAD serial taps + these thin forwarders (no ROS), and
the ROS nodes run on the Q6A. This is the mechanism that lets ROS leave the robot.

**Deployed/cutover:** robot chroot runs 2 forwarders — LDS `:9901` (`/tmp/lds_ring.buf`), MCU `:9902`
(`/tmp/mcu_ring.buf`); companion runs `mcu_node` + `lds_scan_node` with `-p source:=192.168.10.1:{9902,9901}`.
Robot's own `mcu_node` + `lds_scan_node` stopped.

**Verify (on-device):** both nodes logged "connected to ring_forward"; **`/odom/wheel` live at 49 Hz** on the
companion (wheel odom x/y streaming from the robot MCU over USB, matching the ~50 Hz Status rate); `/imu/data`
ready (publishes when the robot is active — docked D10s sends no IMU). **`/scan`** node connected + waiting —
verification pends the LiDAR turret spinning (fanoff-gated while docked); the forwarding mechanism is identical
to the verified MCU path, so no robot motion was forced to test it.

**Next (1.3):** boot-persistence — companion systemd units for the 3 nodes (bridge is already one) + edit the
robot `_root_postboot.sh` to launch the **forwarders** instead of the chroot ROS nodes, then remove ROS from
the chroot. (Forwarders + companion nodes are manually launched right now; a reboot needs 1.3.) Files:
`scripts/robot/ring_forward.py` (new), `lds_scan_node.py`, `mcu_node.py` (dual-source).

---

## 2026-07-07 — Robot-brain migration step 1: USB link + relocate valetudo_bridge to the companion

**Context:** repurposing the Q6A as the robot's full autonomy brain over the USB-gadget link (ROS + MiDaS
depth + obstacle-avoidance/SLAM, driving via Valetudo; a semantic object map is a planned add). Retired the
on-device LLM first to free RAM (see the retire entry / memory) — MemAvailable ~1.8 → ~11 GB.

**What:**
- **USB network robot↔Q6A confirmed plug-and-play:** robot ECM gadget `usb0`=192.168.10.1, Q6A auto-DHCPs
  `enxd67ffa3a49bd`=192.168.10.2, **0.68 ms**; Valetudo REST reachable over it; Q6A→robot SSH via a new
  `robot-usb` alias (dreame key copied to the Q6A).
- **Relocated `valetudo_bridge.py`** from the robot chroot ROS to the **companion** as a systemd service
  (`valetudo-bridge.service`, `--host http://192.168.10.1`) → publishes `/map`, `/odom`, `map→base_link` TF,
  `/robot/status`, `/battery` on the Q6A ROS. Pure Valetudo HTTP/SSE — **no ROS on the robot needed** for
  these. Stopped the robot's bridge (runtime) to avoid dual-publish.
- Gotcha: **`%h` in a *system* systemd unit = `/root`**, not the `User=`'s home — use the absolute path.

**Verify (on-device):** companion bridge `active`; `/battery` = 0.96 (96%, live from the robot over USB),
`/map` width 176, `/odom` + TF publishing; robot chroot `valetudo_bridge` stopped.

**Next:** the robot still runs `lds_scan_node` (`/scan`), `mcu_node` (IMU/`/odom/wheel`), `audio_bridge` in
its chroot ROS (DDS-visible on the Q6A over USB). **Phase 1.2** = forward the LiDAR + IMU serial-tap rings to
companion ROS nodes (robot keeps the LD_PRELOAD taps + a ROS-free forwarder); **1.3** = remove robot ROS +
edit `_root_postboot.sh` (the robot bridge restarts on reboot until then). File:
`scripts/companion/systemd/valetudo-bridge.service` (new).

---

## 2026-07-07 — Incident + root cause: NPU/dma memory leaks monotonically until reboot (leak fix B / record)

**Incident:** getting `q6a-llmd` healthy (reloading its 1.8 GB model) drove the board into OOM-thrash and a
wedge; recovery was a **reboot** (~clean boot → **11 GB free**, vs ~1.8 GB before).

**Root cause (evidence, persistent journal boot -1, Jul 5 05:27 → Jul 6 23:15):** on the QCS6490 every NPU
client opens a fastrpc **process-domain (PD)** + rpcmem/dma-heap buffers on the cDSP. An **unclean exit
(`kill -9`, or a client killed mid-op) or a cDSP SSR** (`Broken pipe` on the fastrpc session — ×18 over the
boot) **orphans** those mappings; the host kernel can't reclaim DSP-pinned memory, so it stays "used" but is
**invisible in `ps`/RSS/slab/cache** (peak process RSS was 16 MB while 11 GB was "used"). It **accumulates
monotonically until reboot** — `err 5005` / `remote_munmap failed` ×16-18 are the reclaim-failure signatures;
182 PDs were created over the uptime. Over ~1.5 days this built to ~9 GB. The 6 OOMs were clustered in one
4-minute window — the LLM reload consuming the last headroom the leak had left, not the cause of the bulk.
The sysfs cdsp SSR that *would* reset it **hangs on this kernel**, so reboot is the only reclaim.

**Operational takeaway:** treat **periodic reboot as maintenance**; watch `MemAvailable` (huge "used" + tiny
top-RSS = this leak; confirm via `/sys/kernel/debug/dma_buf/bufinfo`). The three fixes below attack it from
both ends — **A** (graceful stop, `ctx.release`) cuts the self-inflicted leak *rate*; **C** (pre-load mem
guard) prevents the *consequence* (OOM-wedge) once a leak has accumulated; and the daemon self-recovery +
`StartLimit` keep an SSR from silently killing the LLM. None *prevent* SSR-stranding — reboot remains the cure.
Recorded in the [[project_q6a_adaptive_genie]] memory note too.

---

## 2026-07-07 — q6a-llmd pre-load memory guard: fail fast instead of OOM-wedging the board (leak fix C)

**What:** `q6a_llmd.py` now checks `MemAvailable` before loading the ~1.8 GB model. If it's below
`Q6A_LLM_MIN_FREE_MB` (default 2500), it logs a clear message ("board likely low on RAM from accumulated
NPU/dma leak — REBOOT to reclaim") and exits **before** touching the model, socket, or NPU — instead of
loading into a depleted box and OOM-thrashing the whole board into a wedge.

**Why:** this is exactly what turned the q6a-llmd SSR recovery into a board-wide crisis today — a blind reload
onto ~1.8 GB free drove 6 OOMs → thrash → wedge → reboot. The guard converts that into a clean, diagnosable
failure with the board still usable; systemd's `StartLimitBurst` caps the (harmless, fast-failing) restart
loop, and the fix is a reboot. Complements leak-fix A (graceful stop, which reduces the *rate* of leak) — this
one prevents the *consequence* when a leak has already accumulated.

**Verify (on-device):** ran the daemon with `Q6A_LLM_MIN_FREE_MB=99999999` → it printed the refusal and
exited(1) **without loading the model or binding the socket** (no OOM, no NPU touch); the live service (default
threshold, 9.1 GB free) was untouched and still answered "OK". Takes effect on the daemon's next restart.
File: `q6a_llmd.py`.

---

## 2026-07-07 — Stop leaking NPU memory on restart: graceful SIGTERM stop + harness ctx.release (leak fix A)

**What:** `view_q6a_cam.sh`'s `stop_streamer` was `kill -9` on the streamer + `pkill -9` the detector — SIGKILL
**orphans** each NPU client's fastrpc PD + rpcmem on the cDSP, which is **unreclaimable until reboot** and
accumulates → OOM (root-caused in the q6a-llmd incident, same day). Now it **SIGTERMs** the streamer first
(its atexit cascade runs each NPU child's `ctx.release()` for a clean HTP teardown), waits up to ~10 s, and
only escalates to `-9` as a fallback; detector/depth get SIGTERM→(2 s)→SIGKILL too. Also added `ctx.release()`
to the two depth diagnostics (`depth_bench.py`, `depth_coexist.py`), which previously just exited and orphaned
their MiDaS context every run.

**Why:** every dev-cycle restart (there are many) was stranding an NPU context. This is the *self-inflicted*
half of the leak — it doesn't stop SSR-stranding (nothing does), but it stops the bleed we control.

**Verify (on-device, A/B on the mechanism):** started streamer+detector (dma_buf 23 objects / 1.862 GB,
MemAvail 9.20 GB) → ran the new graceful stop → detector logged **"releasing NPU context + shm"** (clean
`ctx.release`, not a kill), dma_buf **dropped to 15 objects / 1.855 GB** (contexts freed, no orphan), and
**MemAvailable rose to 9.37 GB** (~+170 MB reclaimed). A SIGKILL would have left those orphaned. Files:
`view_q6a_cam.sh`, `depth_bench.py`, `depth_coexist.py`.

---

## 2026-07-07 — Fix q6a-llmd: recover from a cDSP SSR + add self-healing (was dead ~22 h)

**Root cause:** at Jul 6 00:51:41 the **cDSP had a subsystem restart (SSR)** — the fastrpc session got
`errno 0x68 Broken pipe`, the listener thread exited, and the daemon's Genie/HTP remote handle died. The
daemon had **no recovery**: the Python process stayed alive (so systemd's `Restart=on-failure` never fired)
holding a dead handle. Every query then failed (empty reply / Broken pipe / hang) until a query at 22:31
finally closed the handle (`num of open handles: 0`). Net: the LLM was **silently dead for ~22 h**. The cDSP
itself had recovered — the detector + depth open fresh handles on it fine — so only the daemon was stuck.

**Fix:**
1. **Restored it** — a single `systemctl restart` reloaded the model onto the healed cDSP. Verified: "Blue"/
   "Green"/"Mars"/"Healthy" replies at ~0.8 s (normal warm latency).
2. **Self-healing (`q6a_llmd.py`)** — each request now tracks health (`ok = query SUCCESS AND tokens actually
   generated`). A healthy query resets the counter; **2 consecutive dead-context failures → `os._exit(1)`** so
   systemd reloads the daemon fresh (a new process re-does `fastrpc_apps_user_init` on the recovered cDSP —
   the only reliable in-field recovery from an SSR). Empty-input connections don't count.
3. **Anti-restart-storm (unit)** — `StartLimitIntervalSec=300` + `StartLimitBurst=4`: if the cDSP is *genuinely*
   wedged, systemd stops after 4 reloads/5 min and leaves it failed (a human/monitor intervenes) rather than
   restart-storming, which per hard-won experience wedges the cDSP → needs a reboot.

**Verify (on-device):** deployed the hardened daemon + unit, `daemon-reload`, restart → model loaded, socket
up, **3 consecutive queries correct with the pid stable** (happy path does not false-trigger the exit),
`StartLimitBurst=4` / interval 5 min confirmed active, holds the cDSP. Files: `q6a_llmd.py`,
`systemd/q6a-llmd.service`. *(Note: the true 3-way-active coexistence test — detector+LLM-decoding+depth,
gated in the P2.3 entries — can now be run since the LLM serves again; still pending the pinned-memory
re-measure.)*

---

## 2026-07-07 — P2.3 depth runtime: MiDaS process publishing inverse-depth, coexists with the detector

**What:** New `q6a_depth.py` — a 3rd accelerator process (mirrors the detector's hardened structure) that reads
the streamer's frame shm, resizes to 256×256, runs the w8a8 MiDaS-V2 on the NPU, and publishes a 256×256
inverse-depth map to a new **`q6a_depth` shm** (`[0]=depth_seq u64, [8]=dw, [10]=dh, [16]=scale f32,
[20]=shift f32, [64:]=256×256 u8`) under the same odd/even seqlock. Streamer gains `--depth` (+ `--depth-fps`,
default 5): `init_depth()` creates the shm and spawns/supervises the process with the same anti-restart-storm
backoff **and thermal-park awareness** as the detector — the P0.9 governor now parks **both** NPU consumers at
88 °C. Opt-in only (`DEPTH=1 ./view_q6a_cam.sh`); off by default pending in-compartment thermal validation.

Input is native uint8 NCHW (dequant ≈ pixel/255; the small approximation is absorbed by the affine metric
rescale since MiDaS output is affine-invariant). The `scale`/`shift` shm fields are the **metric affine fit**
(`metric = 1/(scale·disp + shift)`) — currently 0/unset: **LiDAR/floor-plane scaling is the remaining piece**
and needs the robot's `/scan` (no LiDAR on the bench).

**Why:** P2.3 — off-plane obstacle sense + metric depth for ~0.2 W without a 2nd SLAM. The coexistence test
cleared depth+detector (leak-free), so the runtime proceeds on that basis (owner-approved).

**Verify (on-device, `--depth` + active detector):** all three processes up (camstream + detector + depth),
`q6a_depth` shm 65600 B created. **Depth publishes at the 5 fps cap** (`depth_seq` +30/3 s = 15) while the
**detector coexists at ~10 fps** (dseq +58/3 s) — 2 NPU processes publishing concurrently, no wedge.
**Depth map is qualitatively correct**: std 66.5 (strong structure, not flat) and the vertical gradient is
physically right — row-band inverse-depth **top=15.9 (far) → mid=101 → bottom=134 (near)**, i.e. the floor by
the robot reads nearest and the far wall farthest. Baseline (no-depth) restored after. Files: `q6a_depth.py`
(new), `q6a_camstream.py`, `view_q6a_cam.sh`.

**Next:** LiDAR/floor-plane metric scaling (needs the robot), a depth consumer (overlay / ROS publish), and
the in-compartment thermal + pinned-memory re-measure before depth runs by default alongside a healthy LLM.

---

## 2026-07-07 — P2.3 coexistence + dmabuf-growth test: depth+detector clean, no leak; LLM gate found

**What:** Ran `depth_coexist.py` — MiDaS depth at 10 Hz as a 3rd NPU context alongside the live,
actively-inferring detector (stream pulled), sampling MemAvailable / dmabuf bytes / temp / latency, with an
84 °C auto-abort (safely under the new 88 °C park). Two runs (120 s + 90 s).

**Results — the good:**
- **Depth + active detector = 2 concurrent NPU contexts coexist cleanly:** ~1088 depth inferences, **0
  errors**, no cDSP wedge, no crash. Depth latency **7.6–7.9 ms median** (p95 ≤12 ms) under detector
  contention vs 5.28 ms isolation — a real slowdown but far inside a 10 Hz budget.
- **No dmabuf leak:** dmabuf **stable to the byte** (1,892,548,608 B, Δ0) over 100 s. No memory leak
  (MemAvailable Δ ±4 MB). Thermals 70 → **75 °C peak** bench-side under this load.

**Results — the gate (important):**
- **The LLM was NOT an active 3rd context** — `q6a-llmd` (pid 1640) has **released its NPU context**: RSS
  ~102 MB (not the ~1.7 GB model), **no `/dev/fastrpc-cdsp` fd held**, journal shows
  `closed module libQnnHtpV68Skel.so … num of open handles: 0`, and a fresh load fails
  (`Qnn getQnnSystemInterface FAILED`). Pre-existing (dormant since ~Jul 6 22:31), **not** caused by depth.
  So this validated 2-active + depth, **not** a true 3-way-active (detector+LLM-decoding+depth).
- **Memory is the likely binding gate for full 3-way:** the ~1.77 GB MemAvailable was measured with the LLM
  model **unloaded**. A healthy resident LLM (~1.7 GB) + detector + depth would push RAM to the edge → this
  is exactly Investigation §1 (pinned-memory re-measure), now clearly the constraint to settle.

**Verdict:** depth coexists with the vision pipeline with no leak/wedge — the depth *runtime* can proceed on
that basis. But the full 3-accelerator picture is gated on (a) restoring `q6a-llmd` health (a fresh QNN load
currently fails; restart is delicate — cdsp-wedge risk), (b) the pinned-memory re-measure (does LLM+detector+
depth fit in 12 GB?), and (c) in-compartment thermals (bench 75 °C → enclosed +5–10 °C approaches the 88 °C
park). Device left clean (streamer+detector up, 67 °C). Files: `depth_coexist.py`, `depth_bench.py` (new
diagnostics).

---

## 2026-07-06 — Harden P0.9: add the thermal hard-cutoff rungs (detector-park + orderly shutdown)

**What:** The thermal governor was a frame-cadence throttle only (82/87 °C + hysteresis) with **no hard
cutoff**. Added the two missing safety rungs above the throttle band:
- **PARK @ 88 °C** — sheds the NPU entirely: sets `State.park`, SIGTERMs the detector (clean HTP-context
  release), and the supervisor **holds it down** (no respawn, no backoff penalty) until temp falls back to
  `THERMAL_HI` (82 °C), then respawns. The 82↔88 hysteresis band stops respawn flapping. Overlay clears via
  the existing staleness cutoff while parked. This removes the NPU heat that frame-throttling alone can't
  reach (the detector is a separate process; throttling only slows how often it's *fed*).
- **EMERGENCY @ 95 °C** — orderly `SIGTERM` self (`os.kill(getpid())` → the `_term` handler → atexit
  `_cleanup`: release NPU + clean shm), the last software line before the **110 °C PMIC hard power-off**.
  Deliberately does **not** auto-restart (avoid re-entering a thermal runaway).

Refactored the per-tick decision into a pure `_thermal_step(t, hot)` so the ladder is unit-testable.

**Why:** Fable-5 P0.9 (audit-corrected — was ⚠️ partial). The NPU has no kernel throttle and the 110 °C PMIC
cut was already hit once; before P2.3 adds a **third** sustained accelerator (depth), the board needs an
actual cutoff, not just a slowdown.

**Verify:** (1) **Unit test** `_thermal_step` across the ladder — 11/11 pass: 60 °C idle, 83 °C→0.12 s, 87.5 °C
→0.40 s, **88.5 °C→park set + detector `terminate()` called**, 85 °C→park held (hysteresis, no re-terminate),
80 °C→unpark, **96 °C→shutdown flagged + `os.kill(SIGTERM)` invoked**. (2) **On-device:** redeployed, restarted
`--gpu --bin --awb` → streamer+detector up, `curl /stream` 44 fps, heartbeat `temp=59–63 °C`, **0 false
PARK/EMERGENCY** at real idle temps. Full park→respawn / emergency-shutdown at real 88/95 °C is validated in
the (still-pending) in-compartment thermal soak. File: `q6a_camstream.py`.

---

## 2026-07-06 — Finish P0.8: open-time FRAME assert + delete the /dev/shm file-tail fallback

**What:** Completed the half of P0.8 the earlier "P0.7, P0.8" entry left undone.
- **Open-time assert:** `V4l2Cam.__init__` takes `expect_size` and raises a named `RuntimeError` if the
  driver-negotiated `sizeimage` ≠ the caller's `FRAME` (stride/padding/format mismatch). `capture_loop` passes
  `expect_size=FRAME`. Previously a mismatch made `read_latest` return frames the caller silently dropped
  (`len != FRAME`) → **0 fps forever with no error**; now it fails loud at open.
- **Deleted the file-tail fallback:** removed `_capture_loop_file()` and both call sites. It ran
  `v4l2-ctl --stream-to=/dev/shm/q6a_cap.raw` in ~595 MB (300×FRAME) batches and was itself the crash-loop
  path. The mmap path is now the only capture path: import failure is fatal (loud), and a transient capture
  fault recovers by **device reinit + progressive backoff** (`min(0.5·fails, 5) s`) instead of dropping into
  the huge-write loop. A persistent config mismatch now surfaces as the open-time FRAME assert on each retry.

**Why:** Fable-5 P0.8 (audit-corrected). The fallback masked faults, wrote ~0.6 GB/batch to tmpfs, and could
crash-loop; the missing assert turned any format mismatch into a silent 0 fps. Both are prerequisites before
adding load (P2.3).

**Verify (on-device):** redeployed `q6a_camstream.py` + `q6a_v4l2.py`, restarted `--gpu --bin --awb`; streamer
+ detector up, shm allocated, `curl /stream` pulled **19.25 MB** with **no capture errors, no assert failure,
no `q6a_cap.raw`** — i.e. the open-time assert passed for the correct `pBAA` format and the huge-write path is
gone. Files: `q6a_v4l2.py`, `q6a_camstream.py`.

---

## 2026-07-06 — Correct the improvement plan against a critical implementation audit

**What:** Audited every plan item against the live code + device (not CHANGELOG titles) and corrected
`docs/q6a-pipeline-improvement-plan.md`: added an authoritative **Implementation status** table and a
**Restored from the review** section. Corrections of record:
- **P0.8 — NOT done** (was implied done by the "P0.7, P0.8" entry): no open-time `frame_size==FRAME` assert
  exists, and the ~595 MB `/dev/shm` file-tail fallback the plan said to *delete* is still present + reachable.
- **P0.9 — partial:** a 2-rung frame-cadence throttle + hysteresis only; the 88 °C detector-park, 95 °C
  SIGTERM, force-bin and LLM-refuse rungs are missing → **no hard thermal cutoff exists.**
- **P2.2 — not done:** all four CAM3 landmines still hardcoded.
- **P1.5 — done differently** (owner capped AE at 2000 not the plan's 6000; legitimate).
- **3 review suggestions plan v1 silently dropped**, now tracked: **P1.7** NPU decode-contention scheduling
  (contention- not temperature-triggered; matters more once depth is a 3rd NPU load), **P2.4** VO-lite for the
  LiDAR-parked manual-drive blind spot (the dropped half of the review's stereo substitute), **P3.3**
  Gemma-3-1B + GBNF CPU-LLM for constrained-JSON ROS handlers.

**Why:** the owner asked for a critical audit — is each suggestion done / not done / done differently — rather
than trusting the plan's forward-looking framing. Establishes a single source of truth before the P2.3 runtime.

**Sequence agreed:** land P0.8 → harden P0.9 (hard rungs) → *then* the depth runtime + coexistence test.
File: `docs/q6a-pipeline-improvement-plan.md`.

---

## 2026-07-06 — MiDaS-V2 mono-depth composes + runs on v68 (plan P2.3, model gate)

**What:** Stood up `build_depth.sh` (mirrors the YOLO 3-hop toolchain: AI-Hub ONNX → x86 2.42 DLC →
v68 context binary) and **proved the make-or-break gate: MiDaS-V2 w8a8 composes on v68 and runs on the NPU.**
The w8a8 context binary (27.66 MB) builds with **no compose errors**, loads on the HTP, and inferences a
valid 256×256 depth map at **5.28 ms/frame** in isolation (25 iters; matches the plan's cited official
~4.117 ms). Model I/O is native uint8: `image[1,3,256,256]` → `depth_estimates[1,1,256,256]` (affine-invariant
inverse-depth; needs LiDAR/floor-plane scale to become metric — a runtime step, not this build).

**Findings (hard-won, they shape the path):**
- **Float is impossible on this target** — AI-Hub rejects `--precision float` because the QCS6490 HTP has
  no fp16 support for compiled models. So depth *must* be int8 (w8a8); there is no float-first shortcut.
- **From-scratch w8a8 quantize is blocked** — MiDaS calibrates on the **NYUV2** dataset, which is private
  (manual Kaggle download) → `UnfetchableDatasetError`. YOLO didn't hit this (COCO auto-fetches).
- **Path that works: `--fetch-static-assets`** — downloads Qualcomm's official pre-quantized w8a8 ONNX
  (30 MB, from their public S3). MobileNetV2 backbone + conv decoder, **zero attention ops** (`MatMul`/
  `Softmax`/`LayerNorm` absent) — which is exactly why v68 composes it (unlike the ViT-based Depth-Anything).
  Env wrinkles: `QAIHM_CI=1` auto-accepts the isl-org repo clone; venv needs `geffnet==1.0.2`/`timm==1.0.15`.

**Verify (on-device, isolation):** context-binary-generator OK (27.66 MB, no errors); standalone HTP bench
loaded the context and ran 5.28 ms/frame, output size 65536 (=256²), uint8 depth 53–153. Ran with the
streamer/detector **stopped** — the 3-process (detector+LLM+depth) NPU coexistence is a separate, deliberately
thermally-gated test (P2.3 depends on it + the pinned-memory re-measure), NOT yet done. Streamer restored after.

**Not yet done (the rest of P2.3):** runtime depth process reading the frame shm, LiDAR/floor-plane metric
scaling, 3-accelerator coexistence + thermal validation, and P0.9 hardening (no 88/95 °C hard cutoff yet)
before any sustained depth load goes live. The 27.66 MB `.bin` is reproducible via `build_depth.sh` and is
**not committed** pending the runtime integration that consumes it. File: `build_depth.sh` (new).

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

> **Correction (2026-07-06 audit):** this landed **P0.7 only**. P0.8 was NOT completed — the shape guard
> here is a per-frame `process()` check, not the planned **open-time `frame_size==FRAME` assert**, and the
> **`/dev/shm` file-tail fallback was not deleted** (it's still reachable). Tracked as ❌ in the plan; fixed
> in a later entry.

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

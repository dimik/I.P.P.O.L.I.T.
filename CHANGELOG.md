# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from).

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

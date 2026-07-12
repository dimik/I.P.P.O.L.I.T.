# Navigation Architecture — room navigation, object mapping, stairwell safety, visualization

**Status: PLAN (2026-07-12).** This doc is the architecture + step-by-step implementation plan for taking
IPPOLIT from "validated building blocks" to "navigates a room, maps it with objects, never falls down the
stairwell, and you can watch it all live." It is written to be executed incrementally by a simpler model:
every phase has exact files, exact interfaces, a verification procedure, and the relevant landmines from
past sessions. **Read the Gotchas section (§8) before touching anything.**

Guiding constraints (non-negotiable, from project decisions):
- **Everything runs on the Q6A companion.** The robot keeps only AVA/Valetudo + the LD_PRELOAD taps +
  ROS-free forwarders. No ROS on the robot.
- **AVA is used as little as possible** — but it *owns the motors*, so the one unavoidable AVA-path is
  Valetudo's `HighResolutionManualControlCapability` REST endpoint (velocity+angle PUTs). The architecture
  therefore funnels ALL actuation through exactly one node (§3.1) so the AVA/REST surface stays minimal
  and swappable.
- **ROS best practices**: standard messages (`geometry_msgs/Twist`, `nav_msgs/OccupancyGrid`), REP-105
  frames, Nav2 for planning/control, `twist_mux` for command arbitration, lifecycle-managed nodes,
  per-node systemd units (this project deliberately does NOT use colcon packages or `ros2 launch` — each
  node is a plain script + a systemd unit; keep that convention).

---

## 1. Current state (verified, as of 2026-07-12)

### Working and validated

| Layer | Node / unit | Topics / TF | Notes |
|---|---|---|---|
| LiDAR | `lds-scan-node` | `/scan` (~5 Hz) | robot LDS tap → TCP ring → LaserScan. Bearing offset+sign calibrated 2026-07-12 (`angle_offset_deg:=43.0`, sign flipped in `lds_scan_node.py`) |
| Odometry | `q6a-laser-odom` | `odom→base_link` TF, `/odom_laser` | ICP scan matcher. Wheel odom (`/odom/wheel` from `mcu-node`) exists but slips during in-place pivots — laser odom is authoritative |
| SLAM | `q6a-slam-toolbox` | `/map`, `map→odom` TF, `/pose` | Jazzy lifecycle node — needs `slam_lifecycle_up.sh` (ExecStartPost) or it idles unconfigured forever |
| Map persistence | `q6a-map-persist` | (services) | periodic serialize every 30 s to `/home/radxa/ros/maps/apartment.posegraph`; resume on start. **Real multi-node resume UNVERIFIED** (only tested vs empty graphs). Has `MIN_RESUME_BYTES` guard — deserializing a zero-scan graph SEGFAULTS slam_toolbox (found live) |
| Vision | `q6a-vision` | `/vision/detections`, `/vision/floor` (JSON String) | YOLOv8+ByteTrack+MiDaS on the robot's OV8856 (siphoned frames). MiDaS is *relative* disparity, not metric |
| Object map | `q6a-objmap` | `/object_map` (JSON String), `/object_markers` (MarkerArray) | conf gate + class allowlist + ≥3-sighting persistence + 0.5 m merge. Persists to `maps/object_map.json`. Camera HFOV/yaw/bearing-sign are ESTIMATES (never calibrated) |
| MCU signals | `mcu-node` | `/cliff`, `/cliff/front`, `/cliff/rear`, `/wheel_floating`, `/bumper`, `/imu/data`, `/odom/wheel`, `/mcu/*` | full Triggers decode. **Key finding: downward IR (`d_view_*`) gives NO early warning — co-fires with wheel-drop at the instant of failure** |
| Cliff safety | `cliff-guard` | `/cliff/ahead` (advisory) + hard e-stop on `/cliff` | e-stop = REST `disable` ×3. Advisory layer never blocks motion (edge-following must stay possible) |
| Drive (scripts, not services) | `q6a_drive.py`, `q6a_edge_follow.py`, `q6a_creep_test.py` | — | each does its OWN raw REST PUTs — this is the main refactor target (§3.1) |
| Bridge | `valetudo-bridge` | `/battery`, `/robot/status`, `/map_valetudo` (relabeled aside) | Valetudo's own map/pose deliberately NOT in the TF tree |
| Audio | `audio-bridge`, `q6a-announce` | `/robot/speak` | |

TF tree (REP-105, correct, single-parent): `map ─(slam_toolbox)→ odom ─(laser_odom)→ base_link ─(static)→ laser`.

Environment (`/etc/default/ippolit-robot`): `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (⚠️ topics are NOT visible off-board — see §6),
`ROBOT_ADDR=192.168.1.213`.

Installed already: `ros-jazzy-nav2-bringup`, `ros-jazzy-nav2-costmap-2d`, `ros-jazzy-nav2-map-server`.
NOT installed yet: `twist-mux`, `foxglove-bridge` (both are Phase deps below).

### Validated safety behaviors (edge/stairwell) — the inputs to §5

1. **MiDaS floor-drop** (`/vision/floor` center+sharp): reliable mid-range (calibration 2026-07-12:
   strongest at 15–20 cm from edge, center≈0.65–0.68 / sharp≈16–17; declining by 10 cm; **fully blind ≤5 cm**
   and at any point the chassis occludes the view). Trigger thresholds are noisy run-to-run (center peak
   0.5–0.76 observed) — bias thresholds toward *reliably firing early* over precisely-timed.
2. **Caution zone** (creep-test v7): on MiDaS trigger, drop to 0.15 and **latch** (never un-latch on a low
   reading — a low reading up close IS the blind spot); re-arm only on >15° direction change.
3. **Odometry blind-creep**: latch pose at a confident MiDaS reading, creep a fixed odometry-tracked
   distance. Worked (5–13 cm landings) but variance up to 35 cm at identical setup — precision open item.
4. **`/wheel_floating`** is the earliest reflex signal; pause (true zero, not low velocity) while it's
   active +0.6 s settle. Continuously commanding ANY nonzero velocity fights AVA's recovery reflex.
5. **`MAX_SAFE_VEL = 0.4`**: AVA's wheel-drop self-recovery works at ≤0.4, **fails at 1.0** (momentum →
   wheels off ground, physical rescue needed). The caution zone independently prevented a repeat even at
   raw 1.0 — but 0.4 stays a hard clamp in the actuation layer.
6. **A 2-D LiDAR cannot see a hole in the floor.** The stairwell reads as *open space* in `/scan` and in
   the SLAM map. This is THE reason the stairwell needs map-level annotation (§5.1) + perception-level
   virtual obstacles (§5.2) — the occupancy grid alone will happily route a planner straight into the hole.

---

## 2. Gap analysis — what's missing for the stated goals

| Goal | Missing |
|---|---|
| **Navigate the room** (go to a pose/object) | No `cmd_vel` abstraction; no Nav2 bringup (planner/controller/BT/costmaps); no command arbitration (nav vs safety vs teleop currently race each other with raw REST) |
| **Map the room with objects** | No verified map resume with real data; no full-room coverage drive; camera extrinsics uncalibrated (object positions systematically skewed); no room/segment tagging |
| **Handle the stairwell** | Nothing marks the hole on the map (LiDAR sees it as free!); cliff logic lives in per-script code instead of the costmap where the *planner* can see it; user wants "caution/slow zone", not hard no-go → Nav2 SpeedFilter + small lethal KeepoutFilter core |
| **Visualize** | DDS is localhost-only → RViz on another machine sees nothing; no websocket bridge; no saved viz layout |

---

## 3. Target architecture

```
                 PERCEPTION (exists)                      NAVIGATION (new)
  /scan ─┬─→ q6a-laser-odom ─→ odom→base_link      ┌────────────────────────────────┐
         ├─→ q6a-slam-toolbox ─→ /map, map→odom,   │ nav2: map_server(+keepout,     │
         │      /pose            ↑ resume/save     │  +speed masks), planner_server,│
         │                 q6a-map-persist         │  controller_server, bt_navigator,│
         ├────────────────────────────────────────→│  behavior_server, lifecycle_mgr │
  camera → q6a-vision ─→ /vision/detections ──────→│  global+local costmaps          │
                     └─→ /vision/floor ──┐         └───────────────┬────────────────┘
                                         │                         │ /cmd_vel_nav
                                         ▼                         ▼
                              q6a-cliff-scan (new)          twist_mux (new)
                              MiDaS drop → virtual          priorities:
                              obstacle LaserScan ──→ local   1. /cmd_vel_safety (cliff_guard)
                              costmap obstacle layer         2. /cmd_vel_teleop (Foxglove)
                                                             3. /cmd_vel_nav (Nav2)
                                                                   │ /cmd_vel
                                                                   ▼
                                                        q6a-cmd-vel-bridge (new)
                                                        Twist → Valetudo REST
                                                        · MAX_SAFE_VEL clamp (0.4)
                                                        · enable/disable ownership
                                                        · explicit-zero on idle (Valetudo
                                                          HOLDS last velocity!)
                                                        · watchdog: no Twist 0.5s → zero
                                                                   │ one REST surface
                                                                   ▼
                                                     Valetudo HighResolutionManualControl
                                                              (the only AVA touchpoint)

  VISUALIZATION (new): foxglove_bridge :8765 (websocket) → Foxglove Studio on Mac/Odyssey
    panels: /map + masks, /object_markers, /pose, /scan, camera MJPEG (robot :8090), teleop→/cmd_vel_teleop
```

Design rationale:
- **One actuation node** (`q6a-cmd-vel-bridge`) = the entire AVA dependency behind one standard interface.
  If the REST path ever changes (or a future direct-MCU path appears), one file changes.
- **twist_mux** (`ros-jazzy-twist-mux`, config-only, no code) replaces today's implicit race where
  cliff_guard's REST `disable` fights a drive script's REST `move` — the exact "fighting" class of bug
  that burned a whole day on 2026-07-12, solved by construction.
- **Cliff hazards live in the costmap**, where the planner can route around them *proactively*, instead of
  only in reactive per-script checks. Reactive layers stay as the inner safety net (defense in depth, §5).
- **Foxglove over RViz**: works through the existing localhost-only DDS (single websocket out), browser/
  desktop app on any machine, has map/marker/teleop panels. RViz stays possible later via DDS peers config,
  but is not required.

---

## 4. Implementation plan — phases and steps

Conventions for every step: scripts live in `scripts/companion/`, units in `scripts/companion/systemd/`;
deploy = `scp` to `ippolit-lan:/tmp/` then `sudo cp` into `/home/radxa/ros/` (+ `/etc/systemd/system/` for
units, then `daemon-reload`); **every deployed file gets committed to git in the same session** (standing
rule); every change gets a CHANGELOG entry. All new units copy the pattern of `q6a-objmap.service`
(`KillSignal=SIGINT`, `EnvironmentFile=-/etc/default/ippolit-robot`, `User=radxa`, `Restart=on-failure`).

### Phase 0 — close the open verification items (no new code)

0.1 **Verify real map resume** (task #22). Drive the robot manually (or `q6a_drive.py`) for ≥1 min while
    SLAM maps; confirm `maps/apartment.posegraph` grows ≫50 KB; `sudo systemctl restart q6a-slam-toolbox
    q6a-map-persist`; confirm log `resumed saved map`, `/map` still shows the previously-mapped area, and
    slam_toolbox does NOT segfault (watch `journalctl -fu q6a-slam-toolbox` during the restart).
    *If it segfaults with a real graph too, STOP — the persistence design needs rework before anything
    downstream (masks are drawn against this map).*
0.2 **Calibrate camera bearing for objmap.** Place one high-confidence object (the tv) at a known bearing;
    compare `/object_markers` position against reality; tune `Q6A_CAM_HFOV_DEG` / `Q6A_CAM_YAW_DEG` /
    `Q6A_CAM_BEAR_SIGN` env vars (in `/etc/default/ippolit-robot`) until the marker lands right. The LiDAR
    bearing fix (2026-07-12) makes this meaningful now; before it, object bearings were doubly wrong.

### Phase 1 — actuation layer (`cmd_vel`) — everything else depends on this

1.1 **`q6a_cmd_vel_bridge.py` + `q6a-cmd-vel-bridge.service`.** Subscribes `/cmd_vel`
    (`geometry_msgs/Twist`), translates to Valetudo `{action:"move", vector:{velocity, angle}}`.
    Requirements (each encodes a live-learned lesson):
    - Clamp `velocity` to `MAX_SAFE_VEL=0.4` unconditionally. No override flag in this node — the
      `--force-unsafe-velocity` escape hatch stays only in the supervised `q6a_creep_test.py`.
    - Own `enable`/`disable`: enable on first nonzero Twist, disable on clean shutdown.
    - **Watchdog**: if no Twist for 0.5 s, send an explicit `{velocity:0}` — Valetudo HOLDS the last
      commanded velocity indefinitely (confirmed live; "pausing" by not sending is NOT stopping).
    - Send at a steady ~6.6 Hz from a persistent process (bash-loop/subprocess delivery gaps let the
      motion decay — confirmed live).
    - **Twist→(velocity,angle) mapping is UNKNOWN and must be calibrated** (step 1.2). Start with:
      `velocity = clamp(|linear.x|, 0, 0.4)`, `angle = clamp(degrees-ish * angular.z, -90, 90)`,
      reverse unsupported initially (Valetudo vector semantics for reverse unverified — treat
      `linear.x < 0` as stop until calibrated).
    - Params via env: `Q6A_CMDVEL_MAX_VEL`, `Q6A_CMDVEL_WATCHDOG_S`, `Q6A_CMDVEL_ANGLE_GAIN`.
1.2 **Calibrate the mapping** with a `turn_diag.py`-style script (exists in scratchpad history; rewrite:
    publish fixed Twists, watch `/odom_laser` yaw/position — NOT wheel odom, it slips in pivots). Produce:
    m/s per `velocity` unit, rad/s per `angle` degree, minimum effective values, pure-rotation recipe.
    Record results as constants + comments in the bridge.
1.3 **Install `twist_mux`** (`sudo apt install ros-jazzy-twist-mux`) + `twist_mux.yaml` + unit. Topics/
    priorities: `/cmd_vel_safety` (prio 100, timeout 0.5 s), `/cmd_vel_teleop` (50), `/cmd_vel_nav` (10)
    → out `/cmd_vel`.
1.4 **Port `cliff_guard` to the mux.** On wheel-drop e-stop: publish zero-Twist to `/cmd_vel_safety` at
    ~7 Hz for a hold period (this outranks and *silences* nav/teleop — no more REST races). KEEP the direct
    REST `disable ×3` as the second, independent action (belt-and-braces; it also covers non-cmd_vel
    scripts like creep-test). Add `/wheel_floating` → safety-zero while active +0.6 s settle (validated
    behavior, §1.4).
1.5 **Port one drive script as proof** (`q6a_drive.py` → publish `/cmd_vel_nav` instead of raw REST) and
    verify behavior unchanged. `q6a_edge_follow.py` / `q6a_creep_test.py` can migrate later — they are
    supervised tools, not services.
    ✅ Phase gate: teleop Twist moves robot; killing the publisher stops it in <0.5 s; wheel-drop test
    (lift robot) zeroes `/cmd_vel` regardless of publishers.

### Phase 2 — visualization

2.1 `sudo apt install ros-jazzy-foxglove-bridge`; new unit `q6a-foxglove-bridge.service`
    (`ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765`). Works with LOCALHOST-only DDS
    since it's an on-board node exporting over its own websocket.
2.2 Foxglove Studio (Mac or Odyssey) → `ws://radxa-dragon-q6a.local:8765`. Build + save a layout into the
    repo (`docs/foxglove-layout.json`): Map panel (`/map`), 3D panel (`/object_markers`, `/pose`, `/scan`,
    TF), Raw topic (`/object_map`), Image panel via the robot camera MJPEG (`http://192.168.1.213:8090/`),
    Teleop panel → `/cmd_vel_teleop`, plots for `/vision/floor` center/sharp (invaluable for edge work).
    ✅ Phase gate: watch the map grow + markers appear live while teleoping from the Foxglove panel.

### Phase 3 — map the room properly (needs 1+2)

3.1 **Coverage drive**: teleop (Foxglove) around the full room perimeter + interior at ≤0.3, LiDAR turret
    forced on (see gotcha G6), ending back near the start to give loop closure a chance. Watch `/map` live.
    Confirm at least one loop-closure log line from slam_toolbox (first-ever real loop-closure validation).
3.2 Save + verify resume again (Phase 0.1 procedure) — now with a full-room graph.
3.3 **Export the grid**: `apartment.yaml/.pgm` already saved by `q6a-map-persist` (`save_map` succeeds once
    a real map exists — `result=1` just meant "no map yet").
3.4 **Author the stairwell masks** (the "caution not no-go" answer, per explicit user direction):
    - `maps/keepout_mask.pgm/.yaml`: copy of the map with a LETHAL band only over the physical hole +
      ~15 cm rim (matches blind-creep landing variance) — the region where being there at all = falling.
    - `maps/speed_mask.pgm/.yaml`: broader zone (~0.8 m around the hole) encoding "max 40% speed here"
      (SpeedFilter percentage semantics) — the caution-driving zone.
    - Mask authoring = editing the PGM (GIMP/Python/PIL); document exact pixel↔world math
      (`resolution`, `origin` from the map yaml) in a comment/README next to the masks.
    - Locate the hole in map coords by teleoping near it (supervised!) and reading `/pose`, or from
      recorded `/cliff/ahead` events + `/pose` during the coverage drive.
    ✅ Phase gate: full-room map survives a Q6A reboot; masks exist and align with the map in Foxglove.

### Phase 4 — Nav2 bringup (needs 1+3)

4.1 `nav2_params.yaml` (in repo, deployed next to the other configs). Key choices for THIS robot:
    - Costmaps: global = static layer (`/map` from slam_toolbox) + obstacle layer (`/scan`) + inflation
      (radius ≥0.25 m; robot_radius 0.18 m per BODY_R work) + **KeepoutFilter + SpeedFilter** (masks via
      two extra `nav2_map_server` instances serving the Phase-3 masks); local = rolling, obstacle layer
      from `/scan` **and** `/virtual_cliff_scan` (Phase 5), inflation.
    - Controller: RPP (regulated pure pursuit) or DWB with `max_vel_x` mapped to real m/s from the 1.2
      calibration (≤ the m/s equivalent of Valetudo 0.4); in-place rotation allowed (diff-drive).
    - Planner: default NavFn is fine at apartment scale.
    - `cmd_vel` remap → `/cmd_vel_nav` (into the mux — Nav2 NEVER talks REST).
    - Lifecycle: one `nav2_lifecycle_manager` autostarting the nav2 nodes; all under ONE
      `q6a-nav2.service` unit (exception to one-node-per-unit — nav2 is a managed set; use
      `ros2 launch nav2_bringup navigation_launch.py params_file:=...` inside the unit, or a minimal
      python launch file in the repo. This is the one place `ros2 launch` earns its keep).
4.2 First goal test: send `NavigateToPose` from Foxglove (3D panel → pose goal) across open floor, away
    from the stairwell. Expect ~5 Hz `/scan` to make the local costmap laggy — keep speeds low
    (this is also why the reactive layers stay).
4.3 Tune until: reaches goals ±0.15 m, no oscillation, respects keepout/speed masks (watch costmap
    overlays in Foxglove).
    ✅ Phase gate: repeatable A→B navigation across the room with masks honored.

### Phase 5 — cliff-aware navigation (the stairwell, end-to-end)

Four independent layers, outermost first (all four must hold):
1. **Map masks** (Phase 3.4 / 4.1): planner never *plans* near the hole; SpeedFilter enforces caution
   speed if a path skirts the zone.
2. **`q6a_cliff_scan.py` (new) + unit**: consumes `/vision/floor` + `/pose`; when center-drop fires
   (cliff_guard thresholds: 0.30/sharp≥4, LiDAR-fused 0.24), publish a synthetic `LaserScan`
   (`/virtual_cliff_scan`, frame `base_link`) with a short-range return in the drop's direction → local
   costmap marks it lethal → controller steers off *dynamically*, even if the robot was placed somewhere
   the masks don't cover. **Latch each virtual obstacle for ≥10 s and clear only on >15° heading change**
   (MiDaS blind-spot rule — never clear because the reading went quiet up close).
3. **Reactive mux layer** (Phase 1.4): `/wheel_floating` pause + wheel-drop safety-zero.
4. **AVA's own recovery**: proven at ≤0.4 — guaranteed by the bridge clamp.
   Test protocol (supervised, human ready to catch, exactly like the creep-test sessions): goal on the far
   side of the speed zone → expect detour/slowdown; goal *inside* the keepout → Nav2 must refuse/fail
   gracefully; robot hand-placed pointing at the hole outside the speed zone, goal beyond it → virtual
   cliff scan must divert it. Watch `/virtual_cliff_scan` + local costmap in Foxglove throughout.
   ✅ Phase gate: all three scenarios pass twice each.

### Phase 6 — object map integration ("go to the fridge")

6.1 **Room tagging**: segment the finished map into named rooms — simplest: a `maps/rooms.yaml` of named
    rectangles in map coords (hand-authored while looking at Foxglove). `q6a_objmap.py` stamps each object
    with its room at publish time.
6.2 **`q6a_goto_object.py`**: resolve class/room query against `/object_map` → standoff pose (~0.6 m back
    along the robot→object bearing) → `NavigateToPose` action. This is the hook the cloud voice worker's
    `goto-object` action already emits (see `docs/voice-cloud.md`).
6.3 (Optional, later) migrate `/object_map` JSON-String to `vision_msgs/Detection3DArray` for
    ecosystem-standard tooling. Not blocking anything.

### Phase 7 — stretch (explicitly out of scope for now)
Frontier exploration (m-explore-ros2) for autonomous coverage; metric MiDaS scaling vs `/scan`;
multi-floor. Do not start these before Phases 0–6 are done.

---

## 5. Stairwell decision record (why this shape)

- User explicitly rejected a blanket permanent no-go: the robot must be able to *work near* the edge
  (edge-following at 5–10 cm is a project goal). Hence **SpeedFilter caution zone** (slow, not forbidden)
  + a **small lethal keepout only over the physical hole + rim**.
- The hole is invisible to every mapping sensor we have (2-D LiDAR at turret height sees free space;
  MiDaS goes blind at the boundary). So the map annotation is *authored*, not sensed — and the sensed
  layers (2–4) exist precisely because authored data can be wrong or the robot can start off-map.
- IR floor sensors were conclusively shown useless for early warning (co-fire with wheel-drop). Do not
  spend more time on them.

## 6. Visualization decision record

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (deliberate, part of the DDS-stability fix) means off-board
RViz sees nothing without config surgery. `foxglove_bridge` runs on-board and serves everything over one
websocket — no DDS changes, works from the Mac and the Odyssey, includes teleop + map + 3-D panels.
RViz remains an option later via CycloneDDS static peers, but nothing in Phases 0–6 needs it.

## 7. What we deliberately do NOT do

- No ROS on the robot (settled 2026-07-08). No new AVA shims for motion — REST manual control only.
- No colcon/workspace conversion — plain scripts + systemd units stay (the one launch-file exception is
  the Nav2 set, §4.1).
- No Valetudo GoTo/segment cleaning for autonomy (blocked in work_mode 17 during manual control; and it
  hands control to AVA's planner — the opposite of the project direction).
- No attempt to make the on-device 1B LLM do anything agentic (settled 2026-07-04; voice/LLM = cloud).

## 8. Gotchas the implementer MUST know (all learned the hard way)

- **G1 — Valetudo holds the last velocity.** Stopping = actively sending zero. A watchdog that merely
  stops sending does nothing. (Cost a live "fighting AVA" incident.)
- **G2 — DDS discovery takes 10–12 s** for a fresh node on this box. Any "no data → abort" warm-up gate
  needs ≥20 s. `ros2 topic hz/list` needs long timeouts too — an empty first answer is usually discovery,
  not an outage.
- **G3 — slam_toolbox is a lifecycle node** — without configure+activate it sits silent. `slam_lifecycle_up.sh`
  handles it; any new lifecycle node needs the same treatment (Nav2's lifecycle_manager does it for nav2).
- **G4 — deserialize_map on a ~zero-node pose graph SEGFAULTS slam_toolbox** (crash loop). The
  `MIN_RESUME_BYTES` guard in `q6a-map-persist` covers the known case; if slam_toolbox ever crash-loops
  right after a "will resume" log: `systemctl stop q6a-map-persist`, delete `maps/apartment.posegraph*`.
- **G5 — rclpy shutdown**: you cannot make ROS service calls from a `finally:` after SIGINT (context
  already dead), and disabling rclpy's SIGINT handler breaks `spin()` interruption entirely (process hangs
  to SIGKILL). Periodic-timer persistence only; no "final save on shutdown".
- **G6 — the fanoff LiDAR gate is load-bearing.** During manual control the turret is parked by default →
  `/scan` goes silent → everything above starves. For any driving session:
  `ssh robot-wifi 'pkill -f fanoff_flag; touch /tmp/lidar_allow'`, and restore the daemon after
  (`nohup setsid sh /data/fanoff_flag.sh ...` + `rm /tmp/lidar_allow`). A future `q6a-nav` session-manager
  could automate this; until then it's a manual checklist item.
- **G7 — `pkill -f` self-match**, including the subtle variant: in a multi-line `ssh host '...'` script the
  remote shell's cmdline contains EVERY line, so a bracket-escaped pattern still matches a *later* line
  mentioning the same filename and kills the whole session. Separate SSH calls for pkill vs start.
- **G8 — wheel odometry lies during in-place pivots** (slip). Use `/odom_laser` or LiDAR bearings as
  rotation ground truth. Straight-line short-distance wheel odom is fine (blind-creep uses it).
- **G9 — MiDaS blind zone**: never treat a low floor-drop reading at close range as "clear". Latched
  hazard states clear on direction change or explicit re-verification, never on signal disappearance.
- **G10 — sim-to-real placement variance is real**: nominally identical hand placements produced 5–35 cm
  differences in stop distance. Build margins (mask rim width, standoff distances) at the ≥15 cm scale,
  and never tune a threshold to the edge of one good run.
- **G11 — REST schema**: manual control is `{"action": ...}` (NOT `{"operation": ...}` → 400);
  move vector is `{"velocity": 0..1, "angle": deg}`.
- **G12 — battery**: Valetudo's charging flag is broken on this model; use `/battery` (AVA `charge_state`
  via valetudo-bridge). `q6a-brownout` already handles low-battery poweroff — don't duplicate.

## 9. Suggested execution order & effort

| Order | Phase | New files | Risk |
|---|---|---|---|
| 1 | 0.1 resume verify | — | low, but BLOCKING if it fails |
| 2 | 1 cmd_vel + mux | `q6a_cmd_vel_bridge.py`, `twist_mux.yaml`, 2 units, cliff_guard edit | medium (drive-by-wire swap) — test with wheels-off-ground first |
| 3 | 2 Foxglove | 1 unit, layout json | trivial |
| 4 | 0.2 camera calib | env edits | low |
| 5 | 3 room map + masks | 2 mask pairs, rooms doc | low code, careful hand-work |
| 6 | 4 Nav2 | `nav2_params.yaml`, `q6a-nav2.service` (+launch) | high tuning effort |
| 7 | 5 cliff nav | `q6a_cliff_scan.py` + unit | supervised live tests, human present |
| 8 | 6 objects | `q6a_goto_object.py`, `rooms.yaml`, objmap edit | low |

Every phase ends with: verify → CHANGELOG entry → commit (incl. every deployed file) → push.

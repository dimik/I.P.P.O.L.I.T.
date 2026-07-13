# IPPOLIT companion software architecture — production-grade ROS 2 plan

**Status: ACTIVE PLAN (2026-07-12, rev 2).** Rev 1 of this doc planned the navigation features but kept
the historical "loose scripts + hand-managed systemd units" convention. **That convention is now
superseded by explicit user direction**: the solution must be architected as a whole, production-grade
ROS 2 system — proper workspace, packages, interfaces, launch, tests, CI, deployment — not a set of
manually managed scripts. This rev is the full plan: Part A builds the engineering foundation, Part B
builds the robot features (room navigation, object mapping, stairwell safety, visualization) on top of it.

Written to be executed incrementally by a simpler model: every phase has exact deliverables, a
verification gate, and references the landmine list (§9). **Read §9 before touching anything.**

Non-negotiable project constraints (unchanged):
- Everything runs on the Q6A companion; the robot keeps only AVA/Valetudo + LD_PRELOAD taps + ROS-free
  forwarders. No ROS on the robot.
- AVA is used minimally — but it owns the motors, so Valetudo's `HighResolutionManualControlCapability`
  REST endpoint is the single unavoidable actuation path. The architecture funnels ALL motion through
  exactly one node so that surface stays minimal and swappable.
- Sensor access stays via the existing taps (`/scan` ring-forward, camera siphon, MCU decode) — those are
  robot-side and out of scope here except as driver-package wrappers.

---

## 1. Where we are vs where production-grade points (gap summary)

What exists **works and is live-validated** (SLAM chain, object map, persistence, edge-safety behaviors —
see rev-1 inventory, now in §8 appendix), but structurally it is prototype-grade:

| Area | Today | Production practice (grounded via 2025/2026 ROS-Industrial / community guidance) |
|---|---|---|
| Code organization | ~15 loose Python files in `scripts/companion/`, hand-`scp`'d to `/home/radxa/ros/` | colcon workspace, ament packages with single responsibilities, versioned deploys |
| Configuration | env vars in `/etc/default/ippolit-robot` + constants edited in-file | declared ROS parameters loaded from per-node YAML in a bringup package |
| Interfaces | JSON blobs on `std_msgs/String` (`/vision/detections`, `/vision/floor`, `/object_map`, `/mcu/triggers`) | standard msgs (`vision_msgs`, `sensor_msgs/Range`) + a small `ippolit_interfaces` package for the rest |
| Robot model | ad-hoc static TF publishes, geometry constants (BODY_R) duplicated across scripts | URDF/xacro + `robot_state_publisher`; one source of truth for geometry |
| Startup | 12 independent systemd units, ordering by `After=` guesswork, lifecycle handled by a bash poller | launch files (XML preferred as the launch front-end) + lifecycle management; systemd supervises ONE launch per subsystem |
| Actuation | every drive script does its own raw REST PUTs; safety node races them | one `cmd_vel` sink node; `twist_mux` arbitration; Nav2 on top |
| Testing / CI | none; every regression found live on hardware | pytest per package, `launch_testing` smoke, lint; GitHub Actions colcon build+test on every push |
| Deployment | `scp` file-by-file, drift between repo and device found repeatedly (mcu_node decode gap, stray drop-in) | one deploy script: git pull → `rosdep install` → `colcon build` → restart; device state fully derived from the repo |
| Observability | ssh + journalctl + ad-hoc monitor scripts | `/diagnostics` (diagnostic_updater + aggregator), rolling rosbag2/MCAP incident recorder, Foxglove |

The repeated real-world failures this structure caused: the deployed `mcu_node.py` silently missing a
month of decode work; a stray systemd drop-in overriding a unit edit; three drive scripts and a safety
node fighting over the REST endpoint; constants like `MAX_SAFE_VEL`/camera FOV duplicated and drifting.
Part A eliminates these classes of failure, not just instances.

---

## 2. Target architecture

### 2.1 Workspace and packages (new repo layout)

```
ros2_ws/src/
  ippolit_interfaces/     # msg/srv: FloorDrop.msg, MappedObject.msg, MappedObjectArray.msg,
                          #   McuTriggers.msg, CliffState.msg  (ament_cmake, msgs only)
  ippolit_description/    # URDF/xacro (base_link, laser, camera, wheel geometry, BODY_R),
                          #   robot_state_publisher config  (replaces ad-hoc static TFs)
  ippolit_drivers/        # lds_scan_node, mcu_node, valetudo_bridge, audio_bridge
                          #   (hardware/robot I/O only — no business logic)
  ippolit_control/        # cmd_vel_bridge (Twist→Valetudo REST; THE only actuation surface),
                          #   twist_mux config
  ippolit_safety/         # cliff_guard, cliff_scan (virtual-obstacle publisher), estop logic
  ippolit_perception/     # vision node (YOLO+ByteTrack+MiDaS), objmap node
  ippolit_localization/   # laser_odom (custom ICP, kept — see D3), map_persist
  ippolit_navigation/     # nav2 params, costmap filter masks, goto_object action client
  ippolit_teleop/         # (thin) foxglove teleop remaps, joystick later
  ippolit_bringup/        # launch/*.launch.xml, config/*.yaml (ALL node params), systemd templates,
                          #   deploy script, rosbag recorder config
```

Rules: ament_python for the Python nodes (entry points in `setup.py`, no more `python3 /path/file.py`);
`ippolit_interfaces` is ament_cmake (message generation); every node = one class, ROS wiring separated
from core logic (testable without rclpy where practical); parameters **declared** with types/descriptions
and loaded from `ippolit_bringup/config/<node>.yaml` — env vars are retired except `ROBOT_ADDR`-class
machine-local values, which move to one `machine.env` sourced by the systemd unit.

### 2.2 Interfaces (retiring JSON-on-String)

| Today | Becomes |
|---|---|
| `/vision/detections` JSON String | `vision_msgs/Detection2DArray` (standard; label/score/bbox/track id in `id`) |
| `/vision/floor` JSON String | `ippolit_interfaces/FloorDrop` (per-sector drop + sharpness + header) |
| `/object_map` JSON String | `ippolit_interfaces/MappedObjectArray` (class, pose, n, conf, room) + keep RViz/Foxglove `MarkerArray` |
| `/mcu/triggers` JSON String | `ippolit_interfaces/McuTriggers` (named bools) — `/cliff`, `/bumper`, `/wheel_floating` stay `Bool` (they're fine) |
| persistence files | unchanged (posegraph + JSON on disk is an implementation detail of map_persist/objmap) |

Migration is per-topic with a compatibility window: new typed topic published alongside the JSON one until
all consumers are ported, then the JSON publisher is deleted (grep the repo to confirm zero subscribers).

### 2.3 Runtime graph (unchanged in shape from rev 1 — now expressed as packages)

```
 drivers: lds_scan → /scan          mcu → /imu /odom/wheel /cliff /wheel_floating (+McuTriggers)
          valetudo_bridge → /battery /robot/status
 localization: laser_odom → odom→base_link ;  slam_toolbox → /map, map→odom, /pose ;  map_persist
 description: robot_state_publisher → base_link→laser, base_link→camera (from URDF)
 perception: vision → Detection2DArray + FloorDrop ;  objmap → MappedObjectArray + markers
 safety: cliff_guard → /cmd_vel_safety (zero-Twist hold) + REST disable backstop
         cliff_scan → /virtual_cliff_scan (synthetic LaserScan from FloorDrop, latched per §9-G9)
 navigation: nav2 (planner/controller/BT/behaviors, lifecycle-managed)
             costmaps: static(/map) + obstacle(/scan) + obstacle(/virtual_cliff_scan, local) + inflation
             + KeepoutFilter (stairwell hole+rim, lethal) + SpeedFilter (caution zone ~40%)
             → /cmd_vel_nav
 control: twist_mux (safety 100 > teleop 50 > nav 10) → /cmd_vel → cmd_vel_bridge → Valetudo REST
          bridge: MAX_SAFE_VEL clamp, explicit-zero watchdog (G1), enable/disable ownership, ~6.6 Hz
 viz: foxglove_bridge :8765 (websocket; LOCALHOST-only DDS makes off-board RViz a non-starter — D5)
 observability: diagnostic_updater in every driver → /diagnostics → aggregator; rosbag2 MCAP
          rolling recorder (snapshot service) for incident capture
```

### 2.4 Launch & process supervision

- `ippolit_bringup/launch/`: `drivers.launch.xml`, `localization.launch.xml`, `perception.launch.xml`,
  `safety_control.launch.xml`, `navigation.launch.xml` (wraps Nav2 bringup + filter mask servers),
  `viz.launch.xml`, and a top-level `robot.launch.xml` including them all.
- systemd shrinks from ~12 hand-ordered units to **4 supervised groups**, each `ExecStart=ros2 launch ...`:
  `ippolit-core` (drivers+description+localization+safety+control), `ippolit-perception`,
  `ippolit-nav`, `ippolit-viz`. Groups = restart blast-radius boundaries (perception can crash-loop
  without taking the safety chain down). `Restart=on-failure`, `KillSignal=SIGINT` (rclpy needs it),
  journald logging as today.
- Lifecycle: nav2's own lifecycle manager handles the nav set; `slam_toolbox`'s configure/activate moves
  from the bash poller into the launch file (launch lifecycle transition events); custom nodes stay
  regular nodes unless a real bring-up ordering need appears (don't cargo-cult lifecycle everywhere).

### 2.5 Testing & CI

- Unit: pytest per package for the logic that has burned us — Twist→(velocity,angle) mapping + clamps,
  caution-zone latch state machine, objmap merge/dedup, MIN_RESUME_BYTES guard, mask coordinate math.
- Integration: `launch_testing` smoke — bring up drivers+localization with a recorded `/scan` bag input,
  assert expected topics publish and TF tree resolves `map→base_link` (catches the "deployed but silent"
  class).
- Lint: `ament_flake8` + `ament_pep257` as test dependencies.
- CI (GitHub Actions): on every push/PR — `ubuntu-24.04` runner, install ros-jazzy-ros-base via apt,
  `rosdep install`, `colcon build`, `colcon test`. This validates source + interfaces on amd64; the Q6A
  is arm64 but ament_python + msgs are arch-independent (add a qemu arm64 job only if a compiled package
  ever appears). Hardware-in-loop stays manual by design — CI gates structure, not physics.

### 2.6 Deployment (D4)

Native build on the Q6A (12 GB RAM / 8 cores — colcon of pure-Python packages takes seconds; Debian
packages remain the preferred channel for upstream deps, per ROS guidance). One idempotent script in the
repo, `ippolit_bringup/scripts/deploy.sh`, run **on the Q6A**:

```
git -C ~/ippolit pull --ff-only            # repo clone ON the device (replaces scp-drift forever)
rosdep install --from-paths ~/ippolit/ros2_ws/src -y
cd ~/ippolit/ros2_ws && colcon build --symlink-install
sudo systemctl daemon-reload && sudo systemctl restart ippolit-core ippolit-perception ippolit-nav ippolit-viz
```

Releases = git tags (`v0.x`); the device runs a tag, not a branch tip, once stable. Docker (multi-stage
cacher/builder/runner) is the documented **future** option if a second robot or an x86 dev-parity need
appears — deliberately not now: one robot, native deps already proven, containers would add a layer
between us and the NPU/camera stack for zero current benefit. Rollback = `git checkout <prev-tag> && deploy.sh`.

### 2.7 Observability

- `diagnostic_updater` in every driver: scan rate, ring-forward connection state, MCU frame rate, REST
  reachability, battery; `diagnostic_aggregator` publishes a single tree Foxglove renders natively.
- **Rolling incident recorder**: rosbag2 MCAP, snapshot mode (RAM ring, service-triggered dump), topics:
  `/scan /pose /cmd_vel* /cliff* /wheel_floating FloorDrop /diagnostics`. Every "it fought AVA again"
  moment this week was reconstructed from grep-ing text logs; a bag dump makes those one-click analyses.
  cliff_guard's e-stop path triggers a snapshot automatically.

---

## 3. Decision records

- **D1 (supersedes rev-1/companion-autonomy "no colcon" convention):** full colcon workspace + ament
  packages. Rationale: user direction to production grade; the loose-script model demonstrably caused
  deploy drift, config drift, and duplicated constants. The old convention is retired everywhere, not
  case-by-case.
- **D2 — single actuation node** (`cmd_vel_bridge`) + `twist_mux`: eliminates the REST-race bug class by
  construction; keeps the whole AVA dependency behind one standard interface.
- **D3 — keep the custom ICP laser odom for now.** Evaluated alternatives: `robot_localization` EKF
  (wheel+IMU fusion) and community 2-D laser odometry (rf2o/kiss-icp). Ours is live-validated on this
  exact sensor; wheel odom slips in pivots (measured), so an EKF fed by it needs careful covariance work.
  Revisit only if laser odom becomes the accuracy bottleneck after Phase F3 mapping. Not a blocker.
- **D4 — native-on-device builds, no Docker yet** (see 2.6).
- **D5 — Foxglove over RViz** for off-board viz: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` is deliberate
  (DDS stability); foxglove_bridge exports over one websocket without touching DDS. RViz possible later
  via CycloneDDS static peers; nothing depends on it.
- **D6 — stairwell = caution zone, not blanket no-go** (explicit user direction): small lethal
  KeepoutFilter over the physical hole + ~15 cm rim (matches measured stop-distance variance), broader
  SpeedFilter (~40 %) caution zone, PLUS dynamic MiDaS virtual obstacles, PLUS reactive reflexes. Four
  independent layers because the hole is invisible to the 2-D LiDAR (reads as free space) and MiDaS goes
  blind at the boundary — authored map data and sensed layers cover each other's failure modes.
- **D7 — XML launch files** (current community guidance: Python launch wasn't meant as the everyday
  front-end; XML keeps them declarative). Python launch only where logic is unavoidable (nav2 wrap).

---

## 4. Part A — engineering foundation (phases A0–A5)

Each phase = PR-sized, behavior-preserving unless stated, ends with: verify → CHANGELOG → commit → push.

- **A0 — workspace scaffold. ✅ DONE (2026-07-12).** Create `ros2_ws/src/` with all packages (empty nodes
  OK), CI workflow, `colcon build && colcon test` green in CI and on the Q6A. Nothing deployed yet.
  ✅ Build clean on device (10/10 packages); 38/38 tests pass on device (0 errors, 0 failures); URDF valid
  + `robot_state_publisher` actually runs and publishes correct TF; CI workflow added (not yet observed
  green on a real push at time of writing — confirm on the next PR). Found and fixed 3 real bugs along
  the way, not just scaffolding — see G13-G15.
- **A1 — wrap existing nodes.** Move each script into its package **unchanged in logic**, add entry
  points, keep old topics/params (env vars still read as fallback). Deploy via new `deploy.sh`; replace
  the 12 units with the 4 group units running launch files. Old `/home/radxa/ros/*.py` copies deleted
  after verification.
  ✅ DONE (2026-07-12). `ippolit_drivers`: all 4 foundational nodes migrated and cut over in
  production — `audio_bridge`, `mcu_node`, `lds_scan_node` (+ `lds_decode` helper), `valetudo_bridge`
  (first argparse-based node, uses `<node args="...">` not `<param>`). `ippolit_safety`: `cliff_guard`
  (wheel-drop hard e-stop + MiDaS `/cliff/ahead` advisory) migrated and cut over. `ippolit_localization`:
  `q6a_laser_odom` + `q6a_map_persist` migrated and cut over (`slam_toolbox` itself deliberately stays
  on its own standalone `q6a-slam-toolbox.service` for now — see G3). `ippolit_perception`: all four
  nodes migrated and cut over — `q6a_announce`, `q6a_vision` (+ folded-in `q6a_yolo`/`q6a_bytetrack`
  helper submodules, no more `sys.path` hack), `q6a_objmap`. Every legacy standalone systemd unit
  stopped+disabled; single clean instance of every node confirmed. All 38 tests across 10 packages
  green (`colcon test-result --all`) — see G20 (real flake8/pep257 lint debt) and G21 (XML launch
  `<env>` replaces, doesn't append — broke `q6a_vision`'s NPU lib path) for what was found along
  the way.
  ✅ Full stack up via `ros2 launch ippolit_bringup robot.launch.xml`; same topics/rates as before
  (compare `ros2 topic hz` for `/scan`, `/pose`, `/vision/detections`); reboot test passes; slam lifecycle
  transition handled by launch (bash poller retired).
- **A2 — parameters. ✅ DONE (2026-07-12).** Declare all tunables as ROS params with YAML in
  `ippolit_bringup/config/`; document each with description strings. Env-var reads deleted. The
  safety constants (caution thresholds, `min_resume_bytes`) get validation (rejected if out of
  proven ranges). (`MAX_SAFE_VEL` lives in `q6a_creep_test.py`, a manual-testing script outside
  ros2_ws — out of this phase's scope, unaffected.)
  ✅ All 6 nodes with env-var tunables converted (`cliff_guard`, `q6a_laser_odom`,
  `q6a_map_persist`, `q6a_announce`, `q6a_objmap`, `q6a_vision`) — the 4 A1 driver nodes already
  used declared parameters exclusively. `ros2 param get` per node matches
  `ippolit_bringup/config/<node>.yaml`; grep confirms `ROBOT_ADDR` is the only `os.environ` read
  left anywhere in `ros2_ws` (the one sanctioned machine-local exception). Safety ranges verified
  live: an out-of-range `ros2 param set` is rejected, not silently clamped.
- **A3 — interfaces. ✅ DONE (2026-07-13).** Create `ippolit_interfaces`, port topics per §2.2 with
  the compatibility window.
  ✅ All four topics ported (`/mcu/triggers`, `/vision/detections`, `/vision/floor`,
  `/object_map`) — the `.msg` definitions already existed from A0 and needed no changes. Given a
  single-repo/single-developer project where every consumer is known, did a direct atomic
  per-topic migration instead of the suggested publish-both window (grepped first to confirm the
  full consumer list per topic; two had zero in-repo subscribers). `ros2 topic type`/`echo` show
  typed data live for all four; `/vision/floor`'s FloorDrop fields are now genuinely Plot-panel
  ready, closing F2's previously-flagged gap. JSON publishers are gone (not kept alongside).
  Found an untracked `/tmp/cliff_monitor.py` debug script (not ours, pre-existing, still running)
  whose `/mcu/triggers` subscription is now a harmless silent type-mismatch — flagged for the
  user, not touched.
- **A4 — URDF + robot_state_publisher.** Measure/encode geometry once (wheel base, BODY_R→radius, laser
  and camera poses — the camera yaw/HFOV calibration from F0 feeds this). All static TF publishes and
  duplicated geometry constants removed in favor of TF lookups / one xacro property file.
  ✅ DONE (2026-07-13). Measured live with a tape measure: chassis dia 0.350 m (spec, confirmed) →
  radius 0.175 m; LDS scan-slot 0.095 m above floor at chassis center; OV8856 camera 0.16 m forward,
  0.06 m high, on the centerline; camera yaw 1.8° (from F0(b)/G26). All encoded in a single xacro
  `<property>` block in `ippolit_description/urdf/ippolit.urdf.xacro` — the one source of geometry
  truth. Removed the ad-hoc static `base_link→laser` TF that `q6a_laser_odom` used to publish itself
  (it was an identity transform duplicating what the URDF now owns — two static publishers of one
  transform is a real bug, cf. G25's `/map` note); `robot_state_publisher` is now the sole publisher
  of `/tf_static` (verified: publisher count 1). `q6a_objmap` no longer has a `cam_yaw_deg` param — it
  reads the camera yaw back from the URDF via a TF lookup (`base_link→camera_link`), cached, with a
  0-yaw fallback while `robot_state_publisher` isn't up. Verified live: `view_frames` shows
  `odom→base_link` (dynamic) + `base_link→{laser,camera_link}` (static, `default_authority`);
  `tf2_echo` confirms laser at z=0.095 and camera at (0.16,0,0.06) yaw 1.8°; objmap logged
  "camera yaw from TF (camera_link): 1.80 deg". 20/20 `colcon test` green (4 new `yaw_from_quaternion`
  cases). Wheel base (a diff-drive kinematic constant) was NOT measured/encoded — we drive via
  Valetudo REST, not direct wheel velocities, and `/odom_laser` is scan-matched not wheel-derived, so
  no consumer needs it yet; deferred until something does (e.g. an F4 controller that wants it).
  NB the full `map→odom→base_link` chain only closes while the LiDAR turret is spinning (slam needs
  live scans for `map→odom`); at idle the fanoff gate parks the turret so the tree shows as two
  unconnected halves — expected, not an A4 regression.
- **A5 — tests + observability.** The §2.5 test set green in CI; diagnostics + aggregator live; rolling
  MCAP recorder unit + snapshot service; cliff e-stop wired to auto-snapshot.
  🔶 IN PROGRESS (2026-07-13). **Done + live-verified:** `diagnostic_updater` tasks on 4 nodes
  (`lds_scan_node` ring/tap connection, `mcu_node` frame staleness, `valetudo_bridge` dual-SSE
  reachability, `cliff_guard` wheel-drop state — WARN not ERROR while tripped, since that's the
  safety system doing its job); `diagnostic_aggregator` (`analyzers` node, `Drivers`+`Safety`
  buckets) confirmed live as `Aggregation: OK` with both buckets healthy; rolling MCAP snapshot
  recorder (`ros2 bag record --snapshot-mode`, RAM ring, `rosbag2_interfaces/srv/Snapshot`) wired
  into `viz.launch.xml`, with `cliff_guard` auto-triggering a snapshot on wheel-drop — verified via
  a manual service call and via `cliff_guard`'s own trigger path. Found and fixed the safety-relevant
  G23 regression along the way (see above and the gotchas list). 47/47 `colcon test` green.
  **Also done + live-verified (follow-up, same day):** `audio_bridge` now has a `diagnostic_updater`
  task (reports the last utterance's outcome — OK/idle or ERROR with detail); `q6a_objmap`'s
  merge/dedup logic pulled into a plain `merge_object()` function with 7 new pytest cases;
  `q6a_map_persist`'s `min_resume_bytes` guard pulled into a plain `resume_decision()` function with
  5 new pytest cases. 59/59 `colcon test` green; `ippolit-core`+`ippolit-perception` restarted clean,
  confirmed identical resume behavior on the real (still-too-small) saved posegraph, confirmed the
  new diagnostic live on `/diagnostics` and correctly bucketed under `Other` in `/diagnostics_agg`.
  **Not user-verified** (needs the user physically present): pulling the LiDAR ring cable to confirm its
  diagnostic flips to ERROR within 5 s, and opening a triggered snapshot bag in an actual Foxglove
  client (no Foxglove client available in this environment — `foxglove-layout.json` itself is also
  still unverified per F2). The two ✅ acceptance lines below are therefore the phase's *target*
  criteria, not yet independently confirmed by the user.
  ✅ Foxglove diagnostics panel shows all-OK tree; pulling the LiDAR ring cable flips its diagnostic to
  ERROR within 5 s; a triggered snapshot bag opens in Foxglove.

## 5. Part B — robot features (phases F0–F6)

Same functional content as rev 1, now landing inside the packages. Preconditions: F1 needs A1; F4+ needs
A2 (param-driven nav tuning) and ideally A3/A4.

- **F0 — close verification debt.** (a) Real map resume (task #22): drive ≥1 min, posegraph ≫50 KB,
  restart slam+persist, confirm resumed map and NO segfault (G4 — if a real graph also crashes, STOP and
  rework persistence before anything downstream). (b) Camera bearing/FOV calibration against a known
  object → values recorded for A4's URDF.
  ✅ DONE (2026-07-13). **(a):** no dedicated coverage drive ever happened; instead, the many small
  calibration test-drive segments from the first session (G24) cumulatively built a real, substantial
  pose graph (4091450B — nowhere near the "trivial empty graph" crash scenario G4 warned about)
  without anyone intending it as a mapping drive. A Q6A power-cycle between sessions then restarted
  `slam_toolbox` fresh (its own standalone `q6a-slam-toolbox.service`, independent of `ippolit-core`),
  which forced the very first genuine cold resume attempt against that file. Result: found + fixed a
  real crash (G25 — `q6a_map_persist` had been silently dead for ~9.5h after crashing on the first
  real `deserialize_map` response), then re-verified live: resumed map is 420x428 cells @ 0.05m
  (~21m x 21m), `slam_toolbox` stayed in `active` lifecycle state throughout, no segfault.
  **(b):** first attempt (paced/eyeballed chair placement) produced a physically implausible 167°
  implied FOV — redone properly with a tape measure (chair at exactly 1.00m dead-ahead, then 1.00m
  fwd + 0.30m left) and averaged detection reads at each position; solved cleanly to `cam_hfov_deg=
  116.7` (close to the prior 110° spec guess) and `cam_yaw_deg=1.8` (negligible). Deployed to
  `q6a_objmap.yaml`, rebuilt (9/9 `colcon test` green), restarted `ippolit-perception`, confirmed live
  in the startup log (`HFOV=117deg`). See G26 for the full story and its caveat (only verified over a
  ±17° bearing range — treat wider bearings as extrapolated). Also surfaced a real, separate finding
  needing its own fix before F4: `/map` has two publishers (`slam_toolbox` + `valetudo_bridge`) — see
  G25's last paragraph.
- **F1 — actuation layer** (`ippolit_control`): `cmd_vel_bridge` per §2.3 (clamp, explicit-zero watchdog
  G1, persistent ~6.6 Hz sender G-rate, enable/disable ownership; reverse unsupported until calibrated),
  Twist mapping calibrated against `/odom_laser` (G8), `twist_mux` config, cliff_guard ported to
  `/cmd_vel_safety` (keeps REST-disable backstop), `q6a_drive` behavior re-implemented as a cmd_vel
  publisher. Supervised-only tools (`edge_follow`, `creep_test`) migrate last.
  🔶 IN PROGRESS (2026-07-13). `cmd_vel_bridge` + `twist_mux` + `cliff_guard`'s `/cmd_vel_safety`
  publish all built and software-verified (47/47 tests incl. 9 new pytest cases for the
  Twist→(velocity,angle) mapping+clamps; both nodes start clean under `ippolit-core` and confirmed
  to stay completely inert — zero REST calls — with nothing yet publishing `/cmd_vel_teleop`/
  `/cmd_vel_nav`). **Deliberately NOT done** (needs the user physically present — deferred
  alongside F0 this session): the actual `linear_scale`/`angular_to_deg_scale` calibration
  against `/odom_laser`, and this phase's own acceptance criteria below (all require live driving).
  `q6a_drive`'s reimplementation as a `/cmd_vel_teleop` publisher is also deferred (lower priority;
  only meaningful once the mapping is calibrated).
  ⚠️ **Correction (found in A5, see G23):** `cliff_guard`'s `/cmd_vel_safety` publish, listed above as
  built, was actually a **silent no-op from the moment it was built** — `twist_mux`'s default
  `use_stamped: true` type-mismatched it against `cliff_guard`'s plain `Twist` messages, so this
  phase's own "lifted-wheel test zeroes /cmd_vel" acceptance line below was never actually true
  during F1; it was only the REST-disable backstop (not the Twist path) doing the job. Fixed and
  re-verified in A5 — see G23 and the CHANGELOG's A5 entry for the full incident.
  ✅ Teleop Twist drives; killing publisher stops <0.5 s; lifted-wheel test zeroes /cmd_vel regardless of
  other publishers (re-verified true after the G23 fix, not just at original F1 write-up time).
  🔶 **Update (first live drive session, 2026-07-13, see G24):** did the deferred physical
  calibration. Found + fixed a real bug live (rotation-only Twist commands never enabled manual
  control at all — G24 #1). Also discovered the linear AND angular Twist->Valetudo mapping are both
  genuinely nonlinear, not just an unknown scale factor (G24 #2) — a single scalar calibration is
  necessarily rough. Deployed working (not precision) values: `linear_scale=1.7`,
  `angular_to_deg_scale=3.0`. User-observed ground truth (eyes on the robot): a 0.15 cmd forward
  drive covered "like 20cm" over 3s (~0.067 m/s), confirming the odometry-based measurement was
  real, not a measurement artifact. Did NOT reach `q6a_drive`'s `/cmd_vel_teleop` reimplementation
  or a proper multi-point nonlinear calibration sweep this session — see G24 for what a real fix
  would need. LiDAR-turret gate gotcha found and worked around live: Valetudo's `manual_control`
  status is in the fanoff shim's `BLOCKED_STATES`, so the on-robot `fanoff_flag.sh` daemon actively
  parks the LiDAR turret during manual driving by design (originally built for quiet human-joystick
  driving with no LiDAR need) — this silently starves `/scan` and therefore `/odom_laser` and SLAM
  during any ROS teleop session unless overridden. Worked around per the daemon's own documented
  manual-override path (`pkill fanoff_flag` + `: > /tmp/lidar_allow`), restored the daemon to normal
  automatic gating after the session. Worth a permanent fix later: either add a ROS-teleop-aware
  state to the gate, or have `cmd_vel_bridge` manage the override itself instead of a manual step
  each session.
- **F2 — visualization**: foxglove_bridge in `ippolit-viz` group; repo-committed layout
  (`docs/foxglove-layout.json`): map+masks, 3-D (markers/pose/scan/TF), FloorDrop plots, camera MJPEG
  panel (robot :8090), teleop→`/cmd_vel_teleop`, diagnostics.
  🔶 IN PROGRESS (2026-07-13). `foxglove_bridge` deployed+enabled in `ippolit-viz`, verified live:
  websocket listening on `0.0.0.0:8765`, advertising 60+ topics. `docs/foxglove-layout.json`
  authored (3D panel doing map+TF+scan+markers+pose in one, Raw Messages panels for
  `/vision/floor` + `/object_map` + a `/diagnostics` placeholder, a working Teleop panel to
  `/cmd_vel_teleop`) but **NOT verified against a real Foxglove client** (none available in this
  environment — open it and treat any needed fixes as the real acceptance test). Two items are
  real, not-yet-buildable gaps rather than oversights: true FloorDrop **plotting** needs A3's
  typed messages (JSON-on-String isn't plottable), and a **camera panel** needs a new
  MJPEG→`sensor_msgs/Image` bridge node that doesn't exist yet (`q6a_vision` only serves plain
  HTTP MJPEG, no ROS image topic).
- **F3 — map the room**: teleop coverage drive at ≤0.3 (turret gate G6!), loop closure confirmed in logs
  (first real validation), save/resume verified with the full-room graph; author stairwell masks
  (keepout: hole+15 cm rim lethal; speed: ~0.8 m zone at 40 %) with documented pixel↔world math; hole
  located via supervised `/pose` readings + recorded `/cliff/ahead` events.
- **F4 — Nav2 bringup** (`ippolit_navigation` + nav group unit): params per §2.3; controller RPP with
  `max_vel_x` = calibrated m/s equivalent of Valetudo 0.4; `/cmd_vel`→`/cmd_vel_nav` remap (Nav2 NEVER
  touches REST); expect a laggy local costmap at 5 Hz scan — keep speeds low, reactive layers cover.
  ✅ Repeatable A→B ±0.15 m, masks honored (watch costmap overlays).
- **F5 — cliff-aware navigation**: `cliff_scan` virtual-obstacle node (FloorDrop → short-range synthetic
  LaserScan, latched ≥10 s, cleared only on >15° heading change per G9). Supervised test triple: goal
  across speed zone (slows), goal inside keepout (refused), hand-placed aimed at hole off-mask (virtual
  scan diverts). Twice each, human ready to catch.
- **F6 — objects**: room tagging (`rooms.yaml` rectangles → objmap stamps room), `goto_object` action
  client (MappedObjectArray query → 0.6 m standoff pose → `NavigateToPose`) — the hook the cloud voice
  worker's `goto-object` action already emits.
- **F7 (stretch, do not start early)**: frontier exploration, metric MiDaS scaling, multi-floor.

Suggested order: A0→A1→F0→F1→F2→A2→F3→A3→A4→F4→F5→A5→F6. (Foundation first where it de-risks features;
features early where they unblock verification debt; A5 before the heavy live-testing of F5 so incident
bags exist.)

---

## 6. What we deliberately do NOT do (unchanged)

No ROS on the robot; no new AVA shims for motion (REST manual control only); no Valetudo GoTo for
autonomy (work_mode 17 + wrong direction); no on-device LLM agents (cloud voice worker instead); no
Docker yet (D4); no exploration before F0–F6 done.

## 7. Success criteria (the user-visible definition of done)

1. From a cold boot: one `systemctl` tree brings up everything; Foxglove connects and shows map, robot
   pose, objects, diagnostics — no ssh needed for routine operation.
2. "Map the room": teleop drive from Foxglove produces a persistent full-room map + object layer that
   survives reboots.
3. Click a nav goal in Foxglove → robot drives there, slowing in the stairwell caution zone, never
   entering the hole rim, with three sensed safety layers behind the map.
4. Repo = single source of truth: device state is `git tag` + `deploy.sh`, CI green, every constant in
   version-controlled YAML, incident bags on every e-stop.

## 8. Appendix — validated-behavior inventory (carried from rev 1)

Working today: `/scan` (5 Hz, bearing calibrated 2026-07-12), laser-ICP odom, slam_toolbox (+lifecycle
poller), map/objmap persistence (+MIN_RESUME_BYTES segfault guard), YOLO+ByteTrack+MiDaS vision, object
map (allowlist/dedup/persistence-gate), full MCU Triggers decode, cliff_guard (advisory + e-stop),
edge-follow controller (corner logic validated), creep-test v7 (caution latch + wheel_floating pause +
odometry blind-creep, landings 5–13 cm, variance up to 35 cm open item), MAX_SAFE_VEL=0.4 proven recovery
envelope (1.0 fails), MiDaS edge calibration table (65→5 cm), thermal enclosure headroom, cloud voice
worker. Battery telemetry via `/battery` (Valetudo charging flag broken on this model). IR floor sensors
conclusively useless for early warning (co-fire with wheel-drop).

## 9. Gotchas (G1–G26) — G1-G12 unchanged from rev 1, G13-G26 found live during A0/A1/F1/A5/F0; MUST READ

- **G1** Valetudo holds the last velocity — stopping requires actively sending zero; a silent watchdog is
  not a stop.
- **G2** DDS discovery takes 10–12 s for fresh nodes here — warm-up gates ≥20 s; empty first `ros2 topic`
  answers are usually discovery, not outages.
- **G3** slam_toolbox (Jazzy) is a lifecycle node — unconfigured = silent. Launch-file transitions own
  this after A1.
- **G4** `deserialize_map` on a ~zero-node pose graph SEGFAULTS slam_toolbox (crash loop). Guard exists
  (`MIN_RESUME_BYTES`); on crash-loop after a "will resume" log: stop map-persist, delete the posegraph pair.
- **G5** No ROS service calls from `finally:` after SIGINT (context dead), and disabling rclpy's SIGINT
  handler hangs `spin()` to SIGKILL. Periodic-timer persistence only.
- **G6** The robot-side fanoff LiDAR gate is load-bearing: manual control parks the turret → `/scan`
  starves everything. Driving sessions need `pkill -f fanoff_flag; touch /tmp/lidar_allow` on the robot,
  restore after. Candidate for automation in `ippolit_drivers` later (SSH toggle from the bridge).
- **G7** `pkill -f` self-match, incl. the multi-line-SSH variant (remote shell's cmdline contains every
  line — a later mention of the filename matches). Separate SSH calls for kill vs start.
- **G8** Wheel odometry lies during in-place pivots. Rotation ground truth = `/odom_laser` / LiDAR
  bearings. Short straight-line wheel odom is fine.
- **G9** MiDaS blind zone: never treat a low floor-drop reading at close range as "clear". Latched
  hazards clear on heading change or explicit re-verification, never on signal disappearance.
- **G10** Hand-placement variance is ≥15 cm-scale: build mask rims/standoffs accordingly; never tune a
  threshold to the edge of one good run.
- **G11** REST schema: `{"action": ...}` (not `operation`); move vector `{"velocity": 0..1, "angle": deg}`.
- **G12** Battery: use `/battery` (AVA charge_state); Valetudo's charging flag is broken on the D10S Pro;
  `q6a-brownout` already owns low-battery poweroff.
- **G13** (found in A0) `colcon test` silently runs ZERO tests for ament_python packages on this
  Python 3.12/setuptools combo — auto-detection keys off `setup.py`'s `tests_require`, but setuptools
  now silently drops that field before colcon reads it back (the "Unknown distribution option" warning
  is the tell). Always pass `--python-testing pytest` explicitly to `colcon test` (done in the CI
  workflow; do the same for any local/manual test run). Do NOT "fix" the warning by removing
  `tests_require` — that field still has to be present for colcon, the warning is cosmetic.
- **G14** `package.xml` (format 3) element ORDER is validated by `xmllint` and matters:
  `member_of_group` must come after all `depend`/`test_depend` tags, immediately before `export`, or the
  package fails lint with a schema error that only shows up under `colcon test`, not `colcon build`.
- **G15** XML launch `<param>` values built from `$(command '...')` substitutions (e.g. xacro-expanded
  URDF text passed to `robot_state_publisher`) need an explicit `type="str"` attribute — without it,
  launch's automatic type inference can choke on multi-line content and fail at RUN time (not parse
  time) with "Failed to convert". `ros2 launch <file> --show-args` parsing cleanly is not sufficient
  proof a launch file works; always also run it (even briefly, `timeout Ns ros2 launch ...`) and check
  the node actually comes up.
- **G16** (found in A1) **XML comments can never contain a literal `--` (double hyphen), and can't end in
  `-` before `-->`** — this is a plain XML spec rule, nothing ROS-specific, but this whole project's prose
  style uses "--" constantly as a parenthetical separator (in every doc, docstring, and now launch-file
  comment), so it hit on the very first real `.launch.xml` edit. Symptom is a confusing multi-parser
  traceback (`InvalidFrontendLaunchFileError` wrapping an `xml.etree.ElementTree.ParseError: not
  well-formed (invalid token)` at the `--` position, PLUS a red-herring `SyntaxError: invalid character
  '—'` from a fallback parser if an em-dash happens to be nearby — chasing the em-dash first cost real
  time; the em-dash was never the actual problem, `encoding="UTF-8"` is good practice regardless but
  didn't fix this). Fixed project-wide by replacing every `--` inside a comment BODY with a single hyphen
  (`sed`-ing the delimiters `<!--`/`-->` themselves by accident is its own trap — they contain `--` too;
  fix comment bodies with a script that captures `<!--(.*?)-->` and only touches the captured group).
  **Also a live process-hygiene lesson surfaced by the same incident**: the failure was initially masked
  because the OLD, still-running production node of the same name answered `ros2 node list` queries,
  making it look like the NEW launch had started when it had actually errored out immediately — always
  check `systemctl is-active <old-unit>` / stop the old node BEFORE trusting `ros2 node list` during a
  migration verification. And: `ros2 launch <file> --show-args` parsing cleanly is not proof a *different*
  file in the same include chain also parses — test the exact file/command that will actually run.
- **G17** (found in A1) `colcon build --symlink-install` generates `install/` content (including
  `local_setup.bash`) with **absolute-path symlinks back into `src/`**. Moving/renaming the workspace root
  after building (e.g. promoting a test dir like `~/ippolit_ws_test` to its permanent location `~/ippolit`)
  leaves every symlink dangling — the package still "builds" fine on inspection but `ros2 launch`
  crash-loops with `Package '<pkg>' not found` because `local_setup.bash` can't be read. Fix: rebuild
  (`rm -rf build install log && colcon build --symlink-install`) at the final path; don't `mv` a built tree.
- **G18** (found in A1, migrating `audio_bridge`) **`audio_bridge`'s SSH-based TTS round-trip genuinely
  takes ~10-15s** (SSH to the robot + Piper synth + ffmpeg + mediad playback) — checking logs or asking "did
  you hear it" within a few seconds of publishing to `/robot/speak` will read as a total failure when it's
  actually just still in flight. This single timing mistake produced an hour-long false debugging trail
  during the migration (chased a `colcon test`-style discovery-range mismatch between an ad-hoc SSH test
  shell — which defaults to `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` — and the services, which use
  `LOCALHOST` via `/etc/default/ippolit-robot`; then chased a thread-pileup theory from `_speak()`'s
  serializing lock — both were real, secondary observations, NEITHER was the actual cause). **The fix that
  actually mattered: wait 15-20s after publishing before checking the log or asking whether audio played.**
  The discovery-range mismatch is still worth matching in ad-hoc test shells (`export
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before any manual
  `ros2 <verb>` query) since it's a genuine inconsistency, just not the culprit here. Broader lesson: when
  a live verification seems to fail, check the SIMPLEST explanation (wrong timeout) before reaching for
  DDS-layer theories.
- **G19** (found in A1, migrating `q6a_announce`) when a node's logic **decays state on the ABSENCE of a
  condition** (here: `q6a_announce`'s per-label hit counter decrements on every message that doesn't
  contain the label), a synthetic test message competes against whatever REAL publisher is also live on
  that topic. `q6a_vision` publishes real detections at ~8Hz continuously; injecting occasional synthetic
  "chair" messages at a much lower rate just got decayed back down between injections by the real
  (chair-free) frames, silently preventing the test from ever crossing the persistence threshold. Fix:
  `sudo systemctl stop <real-publisher>` for the duration of an isolated synthetic test, then restart it
  — don't assume a competing real data source is idle just because the test doesn't reference it.
- **G20** (found in A1, migrating `mcu_node`/`lds_scan_node`/`valetudo_bridge`) `colcon test` can surface
  **genuine, large-scale lint debt**, not just another G16-style XML false alarm — don't assume every
  `colcon test` failure is a repeat of a known gotcha; read the actual error list first. The first run of
  `colcon test --python-testing pytest` against the full `ippolit_drivers`/`ippolit_perception` set (never
  previously run against more than one file at a time) reported 148+14 real `ament_flake8`/`ament_pep257`
  violations across every driver copied so far, because the pre-ROS `scripts/robot|companion/` originals
  were never written against ROS's stricter lint config. Categories: line length 99, `flake8-import-order`
  (**all non-stdlib imports — rclpy, message packages, and even same-package self-imports like `from
  ippolit_drivers import lds_decode` — sort as ONE alphabetical block, not grouped by origin with blank
  lines**), `flake8-quotes` (single-quote preference), `flake8-comprehensions`, ambiguous single-letter
  names, multi-statement one-liners, and — the least obvious — `ament_pep257`'s **D213** convention:
  a multi-line docstring's summary must start on the *second* line (`"""` alone on its own line), the
  opposite of this project's established "summary immediately after `"""`" style used everywhere else in
  the repo (docs, other scripts). Also re-hit a subtler variant of the wrapping trap from G16: a
  comment-only continuation line must match the indentation of the **following** code line exactly, not
  just look visually aligned under the code above it, or `pycodestyle` flags E114/E116. Fix for all of
  this is mechanical (reflow, no logic changes) but real — budget time for it on every future node
  migration, and better: **write new ROS Python source flake8/pep257-compliant from the start** rather
  than copying a pre-ROS script verbatim and fixing lint after the fact. Confirmed cheap when applied
  (migrating `cliff_guard` next): writing it compliant up front left only ONE `colcon test` failure —
  `I101` import-name ordering, which `flake8-import-order` sorts **case-insensitively**, so a lowercase
  name (e.g. `qos_profile_sensor_data`) can sort before CamelCase names from the same module
  (`QoSDurabilityPolicy` etc.) in a single multi-name import line. Worth checking on any future
  multi-name import from `rclpy.qos` or similarly mixed-case modules. Also caught (migrating
  `q6a_vision`/`q6a_objmap`/`q6a_yolo`/`q6a_bytetrack`): `ament_pep257`'s **D403** flags any
  docstring whose first word is a mixed-case acronym ("IoU", "LiDAR", "MiDaS", etc.) — pydocstyle
  compares the word against Python's `.capitalize()` output (which lowercases everything after the
  first letter), so "IoU" != "Iou" and it's flagged as "improperly capitalized" even though it
  looks fine to a human. Fix: rephrase so the docstring's first word is a normal English word
  ("Compute IoU between..." not "IoU between...").
- **G21** (found in A1, cutover of `q6a_vision`) **ROS 2 XML launch's `<env name="" value=""/>`
  action REPLACES the named environment variable — it does NOT append.** `q6a_vision` needs
  `LD_LIBRARY_PATH` to include the QNN/QAIRT native lib dir for `qai_appbuilder` (Hexagon NPU). The
  pre-migration systemd unit got this right only by accident: its `Environment=LD_LIBRARY_PATH=
  <qnn-path>` ran *before* `ExecStart`'s `source /opt/ros/jazzy/setup.bash`, and ROS's own setup
  script *prepends* its lib dir onto whatever was already set — so the final value had both. Setting
  the identical value via a node-scoped `<env>` in the new `perception.launch.xml` runs *after* the
  systemd unit's setup.bash has already populated `LD_LIBRARY_PATH` with ROS's own libs
  (`librcl_action.so` etc.), so the plain `<env value="<qnn-path>"/>` overwrote and lost them —
  `q6a_vision` crash-looped immediately on `import rclpy` itself. Caught fast because the node was
  simply absent from `ros2 node list` after the restart (an absent node is easier to catch than a
  silently-wrong one — always check the full expected node list after any launch-file env change).
  Fix: append instead of replace, `value="$(env LD_LIBRARY_PATH ''):<qnn-path>"`. Check any future
  `<env>` use in a launch file for the same replace-vs-append trap, especially for any variable
  ROS/rclpy itself depends on (`LD_LIBRARY_PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, etc.).
- **G22** (found in F1, wiring `twist_mux`) an empty `locks: {}` in a `twist_mux` params YAML
  **crashes the node at startup** (`terminate called after throwing an instance of
  'rclcpp::exceptions::InvalidParameterValueException'` / `parameter_value_from failed for
  parameter 'locks': No parameter value set`) — its C++ parameter parsing apparently can't infer a
  type from a present-but-empty YAML mapping. Fix: omit the `locks` key entirely when there are no
  locks to configure, rather than declaring it empty. Same family of lesson as G16/G21: a
  seemingly-inert placeholder value (an empty comment-free block, an unset-but-declared env var)
  can crash a vendor node in a way that's non-obvious from the YAML alone — always start a newly
  wired third-party node once and grep its journal, don't assume the config "looks right" is
  proof it runs.
- **G23** (found in A5, testing the rosbag snapshot recorder) — **SAFETY-RELEVANT**: this version of
  `twist_mux` defaults its `use_stamped` parameter to `true` when it isn't explicitly declared, so it
  subscribes/republishes `geometry_msgs/TwistStamped` everywhere — silently type-mismatching
  `cliff_guard`'s and `cmd_vel_bridge`'s plain `geometry_msgs/Twist` publishers/subscribers. A ROS 2
  type mismatch on a topic produces **no error, no crash, no log line** — the two ends simply never
  see each other's messages. Net effect: **`cliff_guard`'s `/cmd_vel_safety` zero-Twist wheel-drop
  stop had been completely non-functional since F1 was built**, despite F1's CHANGELOG entry claiming
  it worked (that claim was based on the node starting cleanly and REST-disable still acting as a
  backstop, not on the Twist path itself being verified end-to-end). Found only by inspecting
  `ros2 topic type /cmd_vel_safety` while wiring up A5's bag recorder and noticing **two** message
  types registered on one topic name — an INFO log line from an earlier F1 test session
  (`"use_stamped" is not declared as parameter, defaulting to "true"`) had been logged and seen at
  the time but its significance wasn't recognized until this later discrepancy forced a closer look.
  Fix: declare `use_stamped: false` explicitly in `twist_mux.yaml`. Verified fix end-to-end: rebuilt,
  redeployed, restarted `ippolit-core`, confirmed `ros2 topic type` shows a single
  `geometry_msgs/msg/Twist` on `/cmd_vel`, `/cmd_vel_safety`, `/cmd_vel_teleop`; published a test
  zero-Twist to `/cmd_vel_teleop` and confirmed it reached `/cmd_vel` via `twist_mux` and was received
  by `cmd_vel_bridge`. Lesson (extends G21/G22's family): **a vendor node's parameter defaults can
  silently change the wire *type*, not just a value** — after wiring any third-party node that
  bridges/republishes a message, always cross-check `ros2 topic type` on both sides of the bridge, not
  just that the node started and topics exist. See the CHANGELOG's A5 entry ("Correction first") for
  the full incident writeup.
- **G24** (found live, first physical F0/F1 drive session, 2026-07-13) two related findings from the
  first real teleop drive:
  1. **Bug**: `cmd_vel_bridge`'s lazy-enable condition checked `vel > 0.0` only. A pure-rotation
     command (`linear.x=0`, nonzero `angular.z`) has `vel==0`, so it never called `{"action":"enable"}`
     and never called `move` at all — a rotation-only Twist silently did nothing on the real robot
     (no REST call, no turret spin-up, no rotation), with no error anywhere in the chain. Same idle
     timer bug alongside it: `zero_since_t` was reset only by `vel>0`, so a sustained pure-rotation
     session would have incorrectly started its 30s idle-disable countdown from tick one, disabling
     manual control mid-rotation. Fixed by pulling the check into a plain `is_commanding_motion(vel,
     angle)` function (`vel > 0.0 or angle != 0.0`) and using it for both the enable AND the idle-timer
     reset. 4 new pytest regression cases. Verified live: rotation commands now log "manual control
     enabled" and physically rotate the robot.
  2. **Real hardware nonlinearity, not just an uncalibrated scale**: live `/odom_laser` measurements
     showed BOTH the angle->turn-rate response AND the linear velocity->real-speed response are
     genuinely nonlinear, not just an unknown linear scale factor. Angular: commanding a modest angle
     (~17deg sent) produced almost no measured rotation (0.014 rad/s), while the `max_angle_deg` clamp
     (45deg) produced much more (0.15 rad/s) — a ~2.6x change in angle produced an ~11x change in
     turn rate, and that same "rotation-only" command also produced real translation (0.36m over 6s)
     despite commanded `vel==0`: this API does not do a clean in-place pivot, it behaves more like a
     wide curve even at zero linear velocity. Linear: two initial calibration points (vel_valetudo
     0.10->0.055 m/s real, 0.20->0.127 m/s real) implied a roughly consistent scale (~1.7), but a
     THIRD point at a similar magnitude (0.255, after deploying that scale) measured dramatically
     slower (~0.067 m/s real, user-confirmed by eye: "drove forward like 20cm" over 3s) — inconsistent
     with a single linear scale factor across the whole range. Root cause not isolated this session
     (candidates: real motor-response nonlinearity/deadband, floor-surface change as the robot moved
     across the room during testing, or accel-ramp eating a bigger fraction of the shorter 3s test
     window vs the original 5s ones — not distinguished). **Consequence: `linear_scale` (1.7) and
     `angular_to_deg_scale` (3.0) in `cmd_vel_bridge.yaml` are deliberately ROUGH working values, not
     a precision fit** — good enough for cautious teleop, NOT sufficient for F4's Nav2 tuning without
     redoing this as a proper multi-point (ideally many-point) calibration sweep first, ideally on a
     single consistent floor surface with longer (5-10s+) segments to dilute ramp-up error.
- **G25** (found live, F0's map-resume test, 2026-07-13) — **`q6a_map_persist` crashed the whole node
  the first time a real `deserialize_map` call ever actually completed**, and had been silently dead
  for ~9.5 hours before anyone noticed (a Q6A power-cycle restarted `slam_toolbox` — its own separate
  `q6a-slam-toolbox.service` — which triggered the very first genuine resume attempt against a
  substantial, real 4MB `.posegraph` file; `q6a_map_persist` itself, part of the `ippolit-core`
  systemd group, had also just restarted moments earlier). Root cause: `on_resume_done` assumed
  `DeserializePoseGraph.Response` has a `result` field, by analogy with `SerializePoseGraph`/
  `SaveMap` (which do have one) — but `ros2 interface show slam_toolbox/srv/DeserializePoseGraph`
  shows its response has **no fields at all**. `res.result` raised `AttributeError:
  'DeserializePoseGraph_Response' object has no attribute 'result'`, which was never caught (only
  the `fut.result()` call itself was wrapped in `try/except`), so the whole node process died
  (exit code 1) the moment its retry timer got a real response instead of a "not ready yet" no-op —
  meaning **zero periodic map/objmap saves happened for the whole time the node was down**, silently.
  Fixed: since there's no result code to check, any response that doesn't raise counts as success.
  Verified live: `q6a_map_persist` restarted, resumed the real 4MB pose graph, logged
  "resumed saved map", and stayed alive; `slam_toolbox` itself never crashed (lifecycle state
  `active`); the resumed map queried via `/slam_toolbox/dynamic_map` is 420x428 cells @ 0.05m
  (~21m x 21m) — a real, substantial map, not a trivial one. **This is F0(a)'s actual acceptance
  test, satisfied for real** (see the F0 phase-table entry). Lesson: don't assume a vendor service's
  response shape by analogy with a sibling service in the same package — `ros2 interface show` each
  one individually, especially ones whose success/failure signal is only exercised by a real,
  late-arriving response (a "not ready, retry" no-op path can look like it's working for a long time
  before the real payload ever reaches the handler). **Also found, not yet fixed**: `/map` currently
  has TWO publishers (`slam_toolbox` and `valetudo_bridge`) — a real topic collision. Any subscriber
  (Foxglove, Nav2 later) gets whichever one happens to publish more recently, so map display/behavior
  could silently flip between the two independent sources. Needs a remap (e.g. `slam_toolbox`'s own
  map onto a distinct topic, or retiring `valetudo_bridge`'s `/map` publisher now that `slam_toolbox`
  is the real map source) before F4's Nav2 bringup, which will consume `/map` directly.
- **G26** (found live, F0(b)'s camera calibration, 2026-07-13) an eyeballed/paced two-point bearing
  calibration produced a physically implausible result — placing a chair "about 1m away" then "about
  1m to the left" and solving the code's linear `bearing = bear_sign*offset_frac*hfov + cam_yaw` model
  for those two points implied a **167° horizontal FOV**, wildly unrealistic for this non-fisheye
  lens. Root cause: imprecise placement (paced by eye, not measured) compounds badly at large bearing
  angles in this linear model. Redone properly with a tape measure: chair at exactly 1.00m dead-ahead,
  then 1.00m forward + 0.30m left (true bearing = atan2(0.30, 1.00) = 16.7°, using 3 consecutive
  detection reads averaged at each position to smooth single-frame jitter) — this solved cleanly to
  **HFOV=116.7°** (reassuringly close to the code's prior 110° spec-based guess — a good sanity check
  that the fix, not the model, was the problem) and **cam_yaw=1.8°** (near enough to zero to treat as
  negligible camera misalignment). Lesson: for any bearing/FOV calibration using this linear
  pixel-offset model, use a tape measure and keep the calibration offset SMALL (here, ±17°) — large
  paced-off angles amplify placement error nonlinearly and can produce a nonsensical fit that LOOKS
  like real data (two clean numbers in, two clean numbers out) but is actually garbage-in-garbage-out.
  **Caveat carried forward**: this calibration is only verified over a ±17° bearing range from
  center; trust it there, treat wider bearings (near the edges of the 116.7° FOV) as extrapolated,
  not verified — a similar small-angle-only caveat to G8's rotation-odometry gotcha.

**Deferred-physical-test pattern (established during F0/F1, 2026-07-13):** when a phase's next
step requires physically driving the robot or aiming its camera and the user isn't set up to
supervise that right now, split the phase: implement and verify everything that's software-only
(build, lint, unit tests, live node startup with zero physical side effects — e.g. `cmd_vel_bridge`
starting but never calling REST `enable` because nothing publishes `/cmd_vel` yet), mark the phase
🔶 IN PROGRESS with an explicit list of what still needs the user present, and move on to the next
phase per the suggested order rather than blocking. Do not fake-verify the physical parts (no
"probably fine" live driving tests without the user watching) — say plainly what's unverified.

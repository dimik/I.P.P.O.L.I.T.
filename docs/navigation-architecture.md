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
  ✅ Full stack up via `ros2 launch ippolit_bringup robot.launch.xml`; same topics/rates as before
  (compare `ros2 topic hz` for `/scan`, `/pose`, `/vision/detections`); reboot test passes; slam lifecycle
  transition handled by launch (bash poller retired).
- **A2 — parameters.** Declare all tunables as ROS params with YAML in `ippolit_bringup/config/`;
  document each with description strings. Env-var reads deleted. The safety constants (`MAX_SAFE_VEL`,
  caution thresholds, MIN_RESUME_BYTES) get validation (rejected if out of proven ranges).
  ✅ `ros2 param dump` per node matches YAML; grep confirms no `os.environ` left outside machine.env.
- **A3 — interfaces.** Create `ippolit_interfaces`, port topics per §2.2 with the compatibility window.
  ✅ `ros2 topic echo` shows typed data; Foxglove plots FloorDrop fields directly; JSON publishers removed.
- **A4 — URDF + robot_state_publisher.** Measure/encode geometry once (wheel base, BODY_R→radius, laser
  and camera poses — the camera yaw/HFOV calibration from F0 feeds this). All static TF publishes and
  duplicated geometry constants removed in favor of TF lookups / one xacro property file.
  ✅ `ros2 run tf2_tools view_frames` shows the full tree sourced from URDF; objmap bearing math consumes
  the camera frame from TF.
- **A5 — tests + observability.** The §2.5 test set green in CI; diagnostics + aggregator live; rolling
  MCAP recorder unit + snapshot service; cliff e-stop wired to auto-snapshot.
  ✅ Foxglove diagnostics panel shows all-OK tree; pulling the LiDAR ring cable flips its diagnostic to
  ERROR within 5 s; a triggered snapshot bag opens in Foxglove.

## 5. Part B — robot features (phases F0–F6)

Same functional content as rev 1, now landing inside the packages. Preconditions: F1 needs A1; F4+ needs
A2 (param-driven nav tuning) and ideally A3/A4.

- **F0 — close verification debt.** (a) Real map resume (task #22): drive ≥1 min, posegraph ≫50 KB,
  restart slam+persist, confirm resumed map and NO segfault (G4 — if a real graph also crashes, STOP and
  rework persistence before anything downstream). (b) Camera bearing/FOV calibration against a known
  object → values recorded for A4's URDF.
- **F1 — actuation layer** (`ippolit_control`): `cmd_vel_bridge` per §2.3 (clamp, explicit-zero watchdog
  G1, persistent ~6.6 Hz sender G-rate, enable/disable ownership; reverse unsupported until calibrated),
  Twist mapping calibrated against `/odom_laser` (G8), `twist_mux` config, cliff_guard ported to
  `/cmd_vel_safety` (keeps REST-disable backstop), `q6a_drive` behavior re-implemented as a cmd_vel
  publisher. Supervised-only tools (`edge_follow`, `creep_test`) migrate last.
  ✅ Teleop Twist drives; killing publisher stops <0.5 s; lifted-wheel test zeroes /cmd_vel regardless of
  other publishers.
- **F2 — visualization**: foxglove_bridge in `ippolit-viz` group; repo-committed layout
  (`docs/foxglove-layout.json`): map+masks, 3-D (markers/pose/scan/TF), FloorDrop plots, camera MJPEG
  panel (robot :8090), teleop→`/cmd_vel_teleop`, diagnostics.
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

## 9. Gotchas (G1–G18) — G1-G12 unchanged from rev 1, G13-G18 found live during A0/A1; MUST READ

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

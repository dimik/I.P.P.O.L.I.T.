# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from), **`docs/navigation-architecture.md` (the navigation/mapping/stairwell/viz plan — the
current active roadmap)**.

---

## 2026-07-14 — F4 stage-3 prep: collision_monitor + ippolit-nav unit; Nav2-needs-turret finding (G31)

Prepping the first autonomous drive:
- Installed the `ippolit-nav` systemd unit on the device (disabled — startable on demand; Nav2 isn't
  production-auto yet).
- Added a **`collision_monitor`** safety backstop between the planner and the robot: RPP/behaviors now
  publish `/cmd_vel_raw` → collision_monitor (watches `/scan` directly, independent of the costmaps;
  circular stop zone r=0.28 m + slowdown r=0.45 m, base_link) → `/cmd_vel_nav` → twist_mux →
  cmd_vel_bridge. Verified live it activates, loads both zones, and publishes `/polygon_stop` +
  `/polygon_slowdown`; both `/cmd_vel_raw` and `/cmd_vel_nav` are plain `Twist` (re-checked the G23
  trap on the new wiring). Its stop *behavior* gets exercised in the drive test (needs a goal +
  obstacle).

**G31 (important operational finding):** Nav2 will NOT activate unless the LiDAR turret is ALREADY
spinning at launch. The global costmap needs `map->base_link` to activate → needs slam's `map->odom`
→ needs fresh `/scan`. Turret parked at launch ⇒ `planner_server` fails to activate ("transform
base_link→map did not become available") ⇒ lifecycle_manager aborts the whole bringup (no retry).
Correct order: override the fanoff gate, spin the turret (teleop), THEN launch Nav2 — verified it then
activates all nodes cleanly (controller/planner/behavior/bt_navigator/collision_monitor all active).
Recorded as G31, with the durable-fix idea (make slam publish `map->odom` continuously so Nav2 can
activate independent of live scans).

**Stage 3 remaining (supervised autonomous drive):** first `NavigateToPose` + RPP tuning. Open
coordination issue to solve there: keeping the turret spun through the teleop→RPP handoff (teleop prio
50 > nav 10). Also still wants the G24 velocity calibration. Cleaned up after: Nav2 stopped, gate
daemon restored, robot idle.

---

## 2026-07-14 — F4 stage 2: Nav2 costmaps + planning verified live (supervised, no autonomous motion)

Verified the Nav2 stack end-to-end for planning, with the LiDAR turret kept spinning by a gentle
in-place teleop rotation (user supervising). Confirmed live:
- `/scan` flows, `map->base_link` resolves (robot localized in the map).
- Global costmap populates: 790x428 @ 0.05m — the full slam_toolbox `/map` loaded into the static
  layer.
- Local costmap populates: 80x80 @ 0.05m (4x4 m rolling window from `/scan`).
- **`ComputePathToPose` action SUCCEEDED** (error_code 0, valid path returned) to a goal ~0.7 m
  ahead — the full localization → costmaps → NavFn planner pipeline works.

The earlier turret-parked failure ("extrapolation into the past", `map->base_link` unresolved) was
confirmed to be purely the stale-scan artifact; it clears the instant the LiDAR spins. No drive goals
issued — the only motion was the turret-alive rotation. Cleaned up after: Nav2 stopped, fanoff gate
daemon restored, robot idle.

Two follow-ups noted: the `ippolit-nav` systemd unit isn't installed on the device yet (Nav2 was
launched manually for these tests); and stage 3 = a first supervised `NavigateToPose` goal + RPP
tuning (the actual autonomous-drive step), which also wants the G24 velocity calibration and a
`collision_monitor` backstop first.

---

## 2026-07-14 — F4 stage 1: Nav2 bringup scaffold (software, no motion)

After confirming AVA's native autonomy is fully locked by work_mode 17 (start/goto/home all return
HTTP 400 — tested live), the companion has to provide autonomy, so started the Nav2 stack (F4).

Stage 1 (software scaffold, no driving): wrote `ippolit_bringup/config/nav2.yaml` +
`navigation.launch.xml`. Design: RPP controller (not the default MPPI — lighter for the Q6A and fits
this robot: drives straight, turns in place, no reverse), NavFn planner, circular footprint r=0.175
(A4 URDF), odom `/odometry/filtered` (the wheel+IMU EKF), costmap obstacle source `/scan`, global
static layer from slam_toolbox's latched `/map`. NO amcl, NO map_server — slam_toolbox already
provides `/map` + `map->odom` (online SLAM localization). Velocity out on `/cmd_vel_nav` → twist_mux
(prio 10) → cmd_vel_bridge → Valetudo REST (Nav2 never touches REST directly, per D2). Added the
nav2 exec deps to `ippolit_bringup/package.xml`.

**Verified live:** stack builds, launches, and the lifecycle **activates** (`controller_server`
`active`, RPP plugin loaded, all nav nodes up); **`/cmd_vel_nav` is `geometry_msgs/msg/Twist`** — the
G23-class TwistStamped mismatch is avoided (Jazzy Nav2 defaults to plain Twist here). No goals issued
→ robot did not move.

**Stage 2 (pending, needs the turret spinning = supervised driving):** costmaps + planning can't be
verified with the turret parked — no fresh `map->odom`/`/scan`, so `map->base_link` won't resolve and
the costmaps sit on "extrapolation into the past." Inherent to the LiDAR-gate design (turret only
spins in active modes; cleaning is work_mode-17-blocked). Stage 2 = drive to keep the turret spinning,
confirm costmaps populate + `ComputePathToPose` plans (plan check needs no motion), then a first
supervised `NavigateToPose` + RPP tuning. Still deferred: G24 velocity calibration for a real
`max_vel_x`, keepout/speed filter masks, `virtual_cliff_scan` (F5), `collision_monitor`.

`ippolit-nav` systemd service left inactive (launched manually for the test, then stopped clean).

---

## 2026-07-13 — Retire q6a_laser_odom (reverses D3)

Retired the custom ICP laser odometry node, following the G30 finding that its point-to-point ICP
drifts in yaw while the wheel+IMU EKF (G28) is accurate. Removed the node from
`localization.launch.xml`, dropped its `setup.py` entry point and `q6a_laser_odom.yaml` config, and
deleted the source (recoverable from git). Reverses decision D3 ("keep the custom ICP for now").

Nothing depended on it: `/odom_laser` had zero subscribers once the EKF took over `odom->base_link`,
and it hadn't published TF since the EKF landed. The LiDAR itself is untouched — `/scan` still has 4
consumers (slam_toolbox for mapping + map->odom, q6a_objmap for range, cliff_guard, the incident
recorder); only this one redundant/drifty consumer is gone.

Verified live: clean rebuild + restart, `q6a_laser_odom` node/executable/topic all absent,
`/odometry/filtered` still @ 30 Hz, EKF remains the sole `odom->base_link` publisher, aggregator OK,
zero errors. 7/7 colcon test green. Localization is now: EKF (wheel+IMU) → odom->base_link;
slam_toolbox → map->odom + /map + /pose; map_persist → save/resume.

---

## 2026-07-13 — Fix /battery charging flag (broken since the companion migration)

Found while the robot was charging: `/battery` reported `power_supply_status: 0` (UNKNOWN) even
though the robot was docked and charging. Root cause: `valetudo_bridge`'s charge detection read
`/tmp/charge_state` from its LOCAL filesystem, but that file is written by an AVA poller on the
ROBOT — and this node moved to the Q6A companion (2026-07-08), where the file doesn't exist. So the
charging flag has silently read UNKNOWN ever since (battery *percentage* was unaffected — it comes
from Valetudo's SSE, not the file). The MCU battery packet (0x2B) that could have been an alternate
source doesn't exist on this hardware (`/mcu/battery` is empty), confirming that path is dead.

Fix: derive charging from the Valetudo `StatusStateAttribute` the node already receives via SSE —
`docked` => CHARGING (or FULL at >=100%), anything else => DISCHARGING. No file, no ssh, no
robot-side change; companion-native. It's a proxy (a docked-but-faulted-charger edge case would
misreport) but vastly better than always-UNKNOWN, and full-at-dock is handled.

Verified live while charging: `/battery` now reports `power_supply_status: 1` (CHARGING) with the
robot docked at 24%. 2/2 colcon test green. (Also corrects a stale assumption in the
`project_robot_audio_battery` memory, which described the now-dead /tmp/charge_state approach.)

---

## 2026-07-13 — Odom-source investigation: laser_odom ICP drifts in yaw; wheel+IMU EKF is correct (G30)

Investigated the "erratic driving" (G29) by comparing all four pose sources during straight-forward
drives, logging wheel/laser/EKF/Valetudo at 5 Hz plus integrating the raw gyro as an independent
arbiter. Result — the discrepancy is entirely in YAW, and it's the LASER odometry that's wrong:

| source | yaw over a straight 6 s drive |
|---|---|
| raw gyro (integrated) | ~0° (correct — direct sensor, read +127° on a real pivot) |
| wheel odom | ~0° |
| EKF (wheel+IMU) | ~0° |
| **laser scan-match (q6a_laser_odom ICP)** | **−4°/drive drift (~15° over three)** |
| Valetudo robot_position | frozen the entire time |

The user visually confirmed the drive was straight. So `q6a_laser_odom`'s naive point-to-point ICP
has a systematic **rotation-drift bias**, which curves its integrated path — and it was the reference
I'd trusted in G24. Recorded as **G30**.

Consequences:
- The **wheel+IMU EKF (previous entry) is validated** as the correct odom source — switching to it
  was the right call. `q6a_laser_odom` is now both redundant (EKF owns the TF) and unreliable even as
  a reference; it should be retired as an odom source (its drift does NOT affect slam_toolbox, which
  does its own correlative scan-matching + loop closure for map->odom).
- **G29 revised**: the robot actually drives reasonably straight (gyro + eyes agree). The
  "curving/erratic-rotation" impression was largely laser-odom drift. What's real: variable *distance*
  per identical command (~0.14–0.38 m — the G24 speed nonlinearity), and one unexplained ~75° turn on
  an earlier run (a probable one-off physical event, since the gyro can't miss a real rotation).
- **Valetudo is confirmed useless as a live odom source** during manual_control (pose frozen).

**Command-path cross-check (addressing the "is the command mapping wrong?" question):** logged
`/cmd_vel` alongside the gyro during three more forward drives — commanded yaw rate was EXACTLY 0°/s
throughout (max abs 0.0), gyro integrated +3.9° total, wheel +0.5°, and the user visually confirmed
all three drives went straight with no curve. So the cmd_vel→Valetudo mapping is clean; it does not
inject rotation. An earlier "drove, rotated, drove" impression traced to MIXED test commands from the
EKF phase (a forward, then a deliberate rotation test, then forward), not a forward command turning.

No production code changed this pass (investigation only). Kept the diagnostic tools under
`scripts/companion/diag/` (`odom_compare.py`, `gyro_yaw_check.py`, `cmdprobe.py`) for reuse.
Recommended follow-up: retire `q6a_laser_odom` from the launch, and steady the forward-speed
calibration (G24) — heading is now trustworthy and driving is straight, so a mapping drive (F3) is
viable.

Note: robot battery down to ~16% (not charging) during this session — dock before further drives.

---

## 2026-07-13 — EKF wheel+IMU odometry (robot_localization); exposed erratic driving (G28, G29)

Stood up a `robot_localization` EKF fusing `/odom/wheel` + `/imu/data` into a smooth
`odom->base_link`, replacing `q6a_laser_odom`'s scan-matching as the odom-frame source. Motivated by
the earlier question "should we use AVA/Valetudo wheel odometry?": Valetudo exposes none (only a
SLAM pose, frozen during manual_control), but AVA's MCU wheel odometry was already decoded and live
on `/odom/wheel` at 50 Hz (unused), and `/imu/data` (gyro) at 90 Hz. The big win: wheel+IMU odom
works WITHOUT the LiDAR, so it stays valid during manual_control when the fanoff gate parks the
turret and scan-matching goes blind.

Changes:
- Added `imu_link` to the URDF (EKF needs the IMU frame in TF; identity offset is correct for 2D
  yaw-rate fusion).
- `mcu_node` now populates measurement covariances on `/odom/wheel` (pose) and `/imu/data`
  (angular velocity) — mandatory for RL, which can't fuse zero-covariance messages.
- New `ekf.yaml` (two_d_mode, world=odom, fuse wheel x/y/yaw + IMU yaw-rate) + `ekf_node` wired into
  `localization.launch.xml`.
- `q6a_laser_odom` gains a `publish_tf` param, set false: it still publishes `/odom_laser` as a
  scan-matched reference but no longer broadcasts `odom->base_link` (the EKF owns it — avoids the
  two-parents-for-base_link bug, G27).

**Verified live:** EKF up, `/odometry/filtered` @ 30 Hz, sole `odom->base_link` publisher (laser_odom
off `/tf`), both inputs subscribed, no measurement-rejection/transform warnings. After fixing G28
(see below), EKF translation matches raw wheel odom exactly (0.219 m vs 0.218 m on a test drive).

**G28 — differential-mode over-report:** first config used `odom0_differential: true` and
over-reported translation ~2x (0.55 m for a ~0.22 m drive). Differential mode is for reconciling
multiple absolute-pose sources, not a lone wheel source; fusing absolutely (`differential: false`)
fixed it. Recorded as G28.

**G29 — the robot drives erratically (the real finding):** during the drive-tests, identical
zero-angular forward commands produced wildly inconsistent real motion run-to-run — 0.22 m straight,
then 0.31 m, then one run went forward + spontaneously turned ~75° CCW, then 0.14 m nearly straight.
This is an ACTUATION problem, not odometry: the EKF faithfully tracked each actual path (the
75°-turn run's chord matched; the straight run read +0.5° yaw). So the odometry/mapping stack is
trustworthy; the robot's *motion* under open-loop manual control is not. Recorded as G29. Implication:
F3/F4 need closed-loop heading control (now feasible — EKF/IMU yaw is trustworthy) or the erratic
actuation root-caused first. IMU vs wheel yaw agreed exactly on every turn, suggesting the Dreame
MCU may already gyro-fuse its yaw internally — our IMU fusion is then confirmatory/robustness
insurance rather than a big immediate gain, but harmless and the correct extensible architecture.

Left deployed (EKF owns odom). Not yet validated: slam_toolbox building a map on top of the EKF odom
during a real turret-on mapping drive (interfaces are correct; needs F3).

---

## 2026-07-13 — Fix /map double-publisher: trim valetudo_bridge to status+battery (G27)

Fixing the `/map` double-publisher flagged in the last A4 work turned up a bigger root cause:
`valetudo_bridge` was still a LEGACY full-stack node, publishing `/map`, `/odom`, AND a
`map->base_link` TF derived from Valetudo's own SLAM -- all left over from before the companion
grew its own `slam_toolbox` + `q6a_laser_odom` + `robot_state_publisher` stack, and all now in
conflict with it:
- `/map` collided with slam_toolbox's (and could feed slam's own map_saver the wrong grid -- slam
  subscribes to /map for `use_map_saver`).
- The `map->base_link` TF was the more dangerous one: it gave `base_link` TWO parents (`map` from
  valetudo, `odom` from laser_odom), corrupting the single-parent TF chain tf2 assumes -- a bug
  that would surface as flickering localization during F4 nav, silently (no error/crash).
- `/odom` from valetudo was useless anyway: Valetudo's `robot_position` is FROZEN during
  `manual_control` (the whole reason q6a_laser_odom exists).

Fix: trimmed `valetudo_bridge` to exactly the role the architecture doc's §2.3 data-flow already
assigns it -- `/robot/status` + `/battery` only -- removing the `/map`/`/odom`/TF publishers and
their now-dead map-SSE + pose machinery (map grid decode, TransformBroadcaster, Odometry). The node
is now ~90 lines lighter and single-purpose. The Valetudo map is recoverable from git history if
ever wanted as a reference topic.

**Verified live:** `/map` publisher count 1 (slam_toolbox); valetudo_bridge absent from `/tf` and
`/odom` (both now have 0 conflicting publishers from it); `/battery` still flowing (26%);
`/robot/status` publisher present (event-driven, publishes on state change); valetudo diagnostic
OK ("attributes SSE stream connected"); node came up clean, no errors. 2/2 colcon test green.
Recorded as G27, with the general lesson (recurring with G23/G25): when a subsystem is superseded,
actively retire its overlapping outputs -- a legacy node still publishing a shared topic/TF is a
silent collision waiting to bite.

Note: `q6a_objmap` still subscribes to `/odom` as an optional fallback pose source; that's now
unpublished, so objmap relies on slam's `/pose` (the correct map-frame source). Harmless -- at idle
objmap can't place objects anyway (parked turret → no scan range), and during active driving slam
`/pose` is authoritative. Left as a legitimate optional hook, not dead-wired.

---

## 2026-07-13 — Phase A4 DONE: URDF as the single source of robot geometry

Measured the robot's real geometry once (tape measure) and encoded it in
`ippolit_description/urdf/ippolit.urdf.xacro` as the single source of truth, in one xacro
`<property>` block:
- chassis diameter 0.350 m (Dreame D10s Pro spec, confirmed) → radius 0.175 m
- LDS turret scan-slot 0.095 m above floor, at the chassis center (x=y=0)
- OV8856 camera 0.16 m forward of center, 0.06 m above floor, on the centerline
- camera yaw offset 1.8° (from F0(b)/G26)

**Removed duplicated geometry / static-TF publishers** (the core of A4):
- `q6a_laser_odom` no longer publishes its own `base_link→laser` static TF (it was an identity
  transform; the URDF now owns it, with the real 0.095 m height). Two static publishers of one
  transform with different values is a real bug -- same family as G25's `/map` double-publisher.
  `robot_state_publisher` is now the sole `/tf_static` publisher (verified: publisher count 1).
  Dropped the now-orphaned `laser_frame` param from the node + its yaml.
- `q6a_objmap` no longer has a `cam_yaw_deg` param -- it reads the camera yaw back from the URDF via
  a `base_link→camera_link` TF lookup (cached, since it's static; 0-yaw fallback while
  robot_state_publisher isn't up yet -- a negligible, self-correcting 1.8° error, not a crash).
  `cam_hfov_deg` stays a param (it's a sensor intrinsic, not a frame pose). Added a pure
  `yaw_from_quaternion` helper with 4 unit tests.

**Verified live:** `ros2 run tf2_tools view_frames` shows `odom→base_link` (dynamic, ~32 Hz from
laser_odom) + `base_link→{laser,camera_link}` (static, `default_authority` = robot_state_publisher);
`tf2_echo` confirms laser at z=0.095 and camera at (0.16, 0, 0.06) with 1.8° yaw; objmap logged
"camera yaw from TF (camera_link): 1.80 deg" -- i.e. its bearing math now consumes the camera frame
from TF, exactly A4's acceptance criterion. 20/20 `colcon test` green across the touched packages.

**Deliberately not done:** wheel base (a diff-drive kinematic constant) -- we drive via Valetudo
REST, not direct wheel velocities, and `/odom_laser` is scan-matched not wheel-derived, so nothing
consumes it yet; deferred until an F4 controller wants it. Also noted (not a regression): the full
`map→odom→base_link` chain only closes while the LiDAR turret spins; at idle the fanoff gate parks
the turret so the TF tree reads as two unconnected halves until we drive.

---

## 2026-07-13 — F0 fully DONE: camera FOV/yaw calibrated with a tape measure (G26)

F0(b), the last open item of F0, is done: camera bearing/FOV calibration against a known object.

**First attempt failed sanity-check:** placed a chair "about 1m away" then "about 1m to the left"
(paced, not measured) and solved the code's linear bearing model for those two points -- implied a
**167° horizontal FOV**, obviously wrong for this lens. Recognized this as garbage-in rather than
deploying it.

**Redone properly:** chair at exactly 1.00m dead-ahead (tape-measured), then 1.00m forward + 0.30m
left (tape-measured; true bearing = atan2(0.30,1.00) = 16.7°), 3 detection reads averaged at each
position to smooth single-frame jitter. Solved cleanly: `cam_hfov_deg=116.7` (reassuringly close to
the code's prior 110° spec-based guess -- confirms the FIX, not the underlying model, was the
problem with the first attempt) and `cam_yaw_deg=1.8` (near enough to zero to treat as negligible
camera misalignment). Deployed to `q6a_objmap.yaml`, rebuilt (9/9 `colcon test` green), restarted
`ippolit-perception`, confirmed live in the startup log (`HFOV=117deg`).

Recorded as **G26**, including the caveat that this calibration is only verified over a ±17° bearing
range from center -- wider bearings (toward the edges of that 116.7° FOV) are extrapolated, not
independently verified.

With this, **F0 is fully done**: (a) real map-resume (done last entry, found+fixed G25's crash) and
(b) camera calibration (this entry) are both closed. Remaining open item from that work: `/map` still
has two competing publishers (`slam_toolbox` + `valetudo_bridge`) -- needs a fix before F4's Nav2
bringup, tracked in G25.

---

## 2026-07-13 — F0(a) map-resume test passes; found + fixed a real q6a_map_persist crash (G25)

F0(a)'s acceptance test ("drive, build a real pose graph, restart slam+persist, confirm resume with
no segfault") passed for real, though not the way planned: no dedicated coverage drive happened this
session. Instead, the many short calibration test-drive segments from the earlier F0/F1 session
(G24) cumulatively left a real, substantial 4091450-byte `.posegraph` on disk. A Q6A power-cycle
between sessions then restarted `slam_toolbox` (its own standalone `q6a-slam-toolbox.service`,
independent of `ippolit-core`) fresh -- the very first genuine cold resume attempt against that file.

**Found + fixed a real crash (G25):** `q6a_map_persist` had been silently dead for ~9.5 hours. Root
cause: `on_resume_done` assumed `DeserializePoseGraph.Response` has a `result` field, by analogy
with `SerializePoseGraph`/`SaveMap` (which do have one) -- but its response has **no fields at all**
(confirmed via `ros2 interface show`). `res.result` raised an uncaught `AttributeError` the moment a
real response (not just a "not ready, retry" no-op) reached the handler, killing the whole node --
meaning zero periodic map/objmap saves happened the entire time it was down, with nothing announcing
the failure beyond a systemd journal entry nobody was watching. Fixed: since there's no result code
to check, any response that doesn't raise counts as success.

**Verified live after the fix:** restarted `ippolit-core`; `q6a_map_persist` resumed the real 4MB
pose graph and logged "resumed saved map"; `slam_toolbox` itself never crashed throughout (lifecycle
state `active` both before and after); queried the resumed map directly via
`/slam_toolbox/dynamic_map` -- 420x428 cells @ 0.05m (~21m x 21m), a real substantial map, not a
trivial one. 7/7 `colcon test` green after the fix.

**Also found, not yet fixed:** `/map` currently has TWO publishers (`slam_toolbox` and
`valetudo_bridge`) -- a real topic collision most likely invisible until F4's Nav2 bringup starts
consuming `/map` directly and gets whichever source happened to publish most recently. Needs a
remap or retiring one of the two publishers before then.

**Still outstanding for F0:** (b) camera bearing/FOV calibration against a known object -- needs a
dedicated physical setup that hasn't happened yet.

---

## 2026-07-13 — First live F0/F1 drive session: found + fixed a real bug, rough calibration (G24)

First physical teleop drive with the ROS stack. Goal was F1's Twist->Valetudo calibration plus
F0's map-resume test; got through the former (partially) and ran out of session time before the
latter.

**Bug found + fixed live:** `cmd_vel_bridge`'s lazy manual-control-enable checked `vel > 0.0`
only, so a pure-rotation command (`linear.x=0`, nonzero `angular.z`) never enabled manual control
and never sent any move command at all — silently a complete no-op on the real robot (confirmed:
no REST call, no turret spin-up, no rotation). The same check also gated the idle-disable timer
reset, so a sustained rotation-only session would have incorrectly started counting down to
auto-disable from the first tick. Fixed by extracting a plain `is_commanding_motion(vel, angle)`
function (`vel > 0.0 or angle != 0.0`) used for both. 4 new pytest regression cases (15/15 green).
Verified live: rotation commands now log "manual control enabled" and physically rotate the robot.

**LiDAR-gate gotcha, worked around:** Valetudo's `manual_control` status is in the on-robot
`fanoff_flag.sh` daemon's `BLOCKED_STATES` (it parks the LiDAR turret during manual driving by
design — that daemon predates the ROS stack and was built for quiet human-joystick driving with no
LiDAR need). This silently starves `/scan` -> `/odom_laser` -> SLAM during any ROS teleop session
unless overridden. Worked around per the daemon's own documented manual-override
(`pkill fanoff_flag` + `: > /tmp/lidar_allow`); restored the daemon to normal automatic gating
after the session. Flagged as worth a permanent fix (either the gate should know about ROS teleop,
or `cmd_vel_bridge` should own the override itself instead of a manual per-session step).

**Calibration attempted, found genuinely nonlinear (G24):** two initial linear-speed data points
(vel_valetudo 0.10->0.055 m/s, 0.20->0.127 m/s real, via chained `/odom_laser` measurements)
suggested a roughly consistent scale (~1.7). Deployed `linear_scale=1.7`/`angular_to_deg_scale=3.0`
(the latter chosen so typical commanded turn rates saturate at the 45deg clamp, since the raw
angle->turn-rate response measured strongly nonlinear -- weak below ~17deg sent, then ramping
fast). A confirmation drive at the new calibration measured far slower than predicted; **the user
directly watched the next test and confirmed it live** ("drove forward like 20cm" over 3s, ~0.067
m/s) -- so this is a real hardware/nonlinearity effect, not an odometry measurement glitch.
Root cause not isolated (candidates: genuine motor-response nonlinearity/deadband, a floor-surface
change as the robot moved across the room, or ramp-up eating more of the shorter 3s test window)
-- flagged honestly as unresolved. **Current values are working-but-rough, not a precision fit**;
do not trust them for F4's Nav2 tuning without a proper multi-point recalibration first.

**Decision (user's call): stopped here for today** rather than chase calibration precision
further or push into the coverage/map-resume drive with an imprecise mapping. F0's map-resume test
(a) and camera FOV calibration (b) remain outstanding for the next physical session -- next time,
go straight to the coverage drive per the doc's updated F0 note (map-building doesn't need
precise velocity calibration, only that the robot moves and the LiDAR keeps working).

Companion (Q6A) and robot were left in a safe idle state; the Q6A was powered off cleanly at the
user's request to end the session. Valetudo/AVA has no software poweroff capability (checked --
not present in the capabilities list), so the robot still needs a physical intervention or dock to
fully power down if desired later.

---

## 2026-07-13 — Phase A5 follow-up: closed the three remaining gaps (audio diag + objmap/map_persist tests)

Closed all three items the A5 entry below explicitly flagged as "not done":
- **`audio_bridge` diagnostic_updater**: reports the outcome of the LAST utterance (there's no
  persistent connection to watch -- each utterance is its own ssh round-trip) -- OK if idle/none
  spoken yet or the last one succeeded, ERROR with the failure detail otherwise (an ssh/robot-link
  problem is the likely cause of a failure, not a one-off fluke).
- **`q6a_objmap` merge/dedup unit tests**: pulled the inline merge logic out of `ObjMap.merge` into
  a plain module-level `merge_object(objects, cls, x, y, conf, merge_dist)` function (same pattern
  as F1's `clamp`/`twist_to_valetudo`) so it's testable without a live rclpy Node. 7 new pytest
  cases: new-entry creation, same-class-within-distance merge + running-average position, distance
  exactly at the merge_dist boundary does NOT merge (strict `<`), far same-class detections stay
  separate, different classes never merge even at the same position, confidence keeps the max seen
  (not the latest), and a 3-observation running average.
- **`q6a_map_persist` `min_resume_bytes` guard unit tests**: pulled the resume-size classification
  out of `MapPersist.__init__` into a plain `resume_decision(size, min_resume_bytes)` function
  returning `'empty'/'refuse'/'resume'`. 5 new pytest cases, including the two states that must
  never be confused: a 0-byte file (nothing to resume) vs. a small-but-nonzero file (refuse --
  the actual crash-risk case from the docstring's CRASH FOUND note).

**Verified live** (via `ippolit-lan`, the wired link from the Odyssey -- this session had no path
to the `q6a`/`ippolit` mDNS alias, only the `ippolit-lan` static-IP alias worked): `colcon build` +
`colcon test` for `ippolit_perception`/`ippolit_localization`/`ippolit_drivers` -- 59/59 tests green
(caught two of my own D213 docstring violations and a test bug of my own, mirror of G20, fixed
before commit -- see below). Restarted `ippolit-core` + `ippolit-perception`; confirmed all three
nodes came back clean with no tracebacks from the refactors: `q6a_map_persist` correctly refused to
resume the existing 7769B posegraph (identical decision to the pre-refactor code -- proves the
`resume_decision` extraction didn't change behavior), `q6a_objmap` loaded cleanly, `audio_bridge`'s
new diagnostic reads `idle -- no utterances spoken yet this run` on `/diagnostics`, and
`/diagnostics_agg` correctly buckets it under `Other` (not in the `Drivers`/`Safety` allowlists --
expected, not an error).

**Lesson (mirrors G20):** wrote both new docstrings with the "summary right after `"""`" style used
everywhere in this project's non-ROS docs/scripts, and `ament_pep257`'s D213 flagged both -- same
gotcha as G20, still easy to trip even when you know the rule, because it's the opposite of this
project's general house style. Also had a genuine test bug (not a lint issue): my own
`merge_dist`/point-spacing choice in one running-average test landed exactly on the strict `<`
boundary, so the second observation never actually merged -- `colcon test` caught it immediately.

---

## 2026-07-13 — Phase A5: diagnostics + aggregator + rolling MCAP recorder; CORRECTION to F1's claim (G23)

**Correction first:** F1's changelog entry claimed cliff_guard's `/cmd_vel_safety` zero-Twist hold
was a working second stop layer alongside the REST-disable backstop. **It was not.** While
building this phase's rosbag recorder and inspecting `ros2 topic type` output, found that
`ros-jazzy-twist-mux` (this version) defaults `use_stamped` to `true` when the parameter isn't
declared — meaning `twist_mux` was silently expecting/publishing `geometry_msgs/TwistStamped`
everywhere, while `cliff_guard` and `cmd_vel_bridge` both use plain `geometry_msgs/Twist`. ROS
2/DDS simply does not match publishers and subscribers of different types on the same topic name
— no error, no crash, just zero data flow. So `/cmd_vel_safety` and `cmd_vel_bridge`'s `/cmd_vel`
subscription have been dead since F1 was written. Recorded as **G23**. Fixed by explicitly setting
`use_stamped: false` in `twist_mux.yaml`. Verified live this time: `ros2 topic type` shows a
single `geometry_msgs/msg/Twist` on every `/cmd_vel*` topic, and a zero-Twist published to
`/cmd_vel_teleop` is confirmed reaching `/cmd_vel` through `twist_mux` and being received by
`cmd_vel_bridge` (which correctly stayed disabled, zero REST calls, since the value was zero).
**Lesson: an INFO-level startup log line ("defaulting to X") is worth reading, not skimming past
— this one was visible during F1's own verification and wasn't caught until a downstream tool
(the bag recorder) surfaced the consequence.**

With that fixed, built out the rest of A5 (`docs/navigation-architecture.md` §2.7):
- **`diagnostic_updater`** in the four nodes it matters most for right now: `lds_scan_node`
  (reports the shm ring/tap CONNECTION state, deliberately not scan rate/frequency — 0 Hz while
  the turret is parked is normal, not a fault, so a naive `FrequencyStatus` would false-positive
  constantly), `mcu_node` (ANY MCU frame within the last few seconds, since Status20ms flows
  continuously even when idle/docked, unlike the IMU stream), `valetudo_bridge` (both SSE streams
  connected — REST reachability, not event frequency, for the same idle-robot reason), and
  `cliff_guard` (the wheel-drop e-stop state itself — WARN, not ERROR, while tripped, since the
  safety system is doing exactly its job, not malfunctioning).
- **`diagnostic_aggregator`** (`ippolit_bringup/config/diagnostic_aggregator.yaml`) bucketing
  these into `Drivers`/`Safety` groups, published on `/diagnostics_agg`.
  `audio_bridge`'s diagnostic is not yet built (remaining work, noted below).
- **Rolling incident recorder**: `ros2 bag record --snapshot-mode` (rosbag2's own built-in
  RAM-ring snapshot mode -- no separate package needed) recording `/scan /pose /cmd_vel*
  /cliff* /wheel_floating /vision/floor /diagnostics`, wired via `<executable>` in
  `viz.launch.xml` (`ros2 bag record` is a CLI wrapper, not a `ros2 run`-able node). Deliberately
  no fixed `-o` path -- rosbag2 refuses to write into an existing bag directory, so a second
  restart with a fixed path would fail; left unset so it defaults to a fresh timestamped folder
  in `cwd` (`/home/radxa/ros/bags`) every run. `cliff_guard` now calls
  `/rosbag_snapshot_recorder/snapshot` automatically on a real wheel-drop trip, in addition to
  its two stop layers.

**Verified live:** all 47 tests pass; both affected systemd groups restarted; `/diagnostics_agg`
shows `Aggregation: OK` with `Drivers`/`Safety` both `OK` and the right per-node detail underneath;
manually called the production snapshot service and confirmed a real ~279KB MCAP bag was written
and playable-format (two-file split, matching mcap's own segment convention). Cannot verify the
doc's exact physical acceptance tests myself (pulling the LiDAR ring cable; opening a bag in the
Foxglove GUI) -- the underlying mechanisms are proven live, but those two specific checks need the
user.

**Explicitly NOT done, flagged not hidden:** `audio_bridge`'s diagnostic_updater (per §2.7's "every
driver"); additional pytest unit tests for `q6a_objmap`'s merge/dedup and
`q6a_map_persist`'s `min_resume_bytes` guard logic (only F1's Twist-mapping tests exist so far).

---

## 2026-07-13 — Phase A3 COMPLETE: typed interfaces retire JSON-on-String topics

Ported all four JSON-on-String topics per the architecture doc's §2.2 table to typed messages:
`/mcu/triggers` (`std_msgs/String` -> `ippolit_interfaces/McuTriggers`, `mcu_node.py`),
`/vision/detections` (-> `vision_msgs/Detection2DArray`, `q6a_vision.py`), `/vision/floor` (->
`ippolit_interfaces/FloorDrop`, `q6a_vision.py`), `/object_map` (->
`ippolit_interfaces/MappedObjectArray`, `q6a_objmap.py`). All four `.msg` definitions already
existed from A0's scaffold and needed no changes — their fields matched the actual JSON payloads
exactly.

Grepped the whole `ros2_ws` first to find every consumer before touching a publisher: `mcu/triggers`
and `object_map` have zero in-repo subscribers (pure diagnostic/future-consumer topics), so no
downstream code needed updating for those two. `/vision/detections` is subscribed by both
`q6a_announce` and `q6a_objmap`; `/vision/floor` by `cliff_guard` — all three updated to read the
typed fields directly (no more `json.loads`).

Given this is a single-repo, single-developer project where every consumer is code I control, did
a direct atomic per-topic migration rather than the doc's suggested compatibility window
(publish-both-briefly): no external/uncontrolled consumer exists to protect during a transition,
so the extra publish-both machinery would just be built and immediately deleted again.

Two real design decisions along the way: (1) `vision_msgs/Detection2D`'s `BoundingBox2D` is
center+size, not corner (x1,y1,x2,y2) — a genuine format conversion, not just a rename; (2) MiDaS
per-detection disparity (the JSON's `disp` field) has no natural home in `vision_msgs` and nothing
downstream ever consumed it (confirmed by grep) — dropped from the ROS-published message rather
than forced into an unrelated field; it still exists internally for the `:8093` annotated view.
(3) `Detection2DArray` carries pixel bbox coordinates but not frame width/height, which
`q6a_objmap`'s bearing calc needs to normalize the bbox x-center — added a new `img_width`
parameter (default 672, the fixed OV8856/camstream resolution) rather than inventing a
non-standard message field.

**Verified live:** all 47 tests pass; restarted `ippolit-core` + `ippolit-perception`, confirmed
every topic now reports its typed `ros2 topic type`, and read real data off each: `/vision/floor`
(FloorDrop numeric fields), `/object_map` + `/vision/detections` (typed empty arrays, correct
given the robot is currently idle with no detections), `/mcu/triggers` (full McuTriggers field
dump via explicit-type `ros2 topic echo`). No new errors in either group's journal beyond the
already-known-harmless QNN loader probe pattern (see prior entries).

**Found and flagging, not fixing:** `/tmp/cliff_monitor.py` — an untracked (not in git), ad-hoc
debug script from earlier manual testing, still running on the Q6A (orphaned, PPID 1) — subscribes
to `/mcu/triggers` expecting the old `String`/JSON type. Its subscription is now a silent type
mismatch (ROS 2/DDS simply won't match publishers and subscribers of different types on the same
topic name), so it stops receiving `d_view_*` updates. Its other subscriptions (`/cliff`,
`/bumper`, etc., still `Bool`) are unaffected. Not part of the production system and not touched —
flagged for the user to decide whether to kill it or leave it.

Next per `docs/navigation-architecture.md`'s suggested order: A4 (URDF + robot_state_publisher
real geometry) — also pure software, no physical driving required, unlike F3/F0's still-deferred
physical tests.

---

## 2026-07-13 — Phase F2 (visualization): foxglove_bridge deployed and verified; layout JSON authored but unverified

Installed `ros-jazzy-foxglove-bridge` on the Q6A, wired it into the new `ippolit-viz` systemd
group (the 4th and last restart blast-radius group — a viz-side crash/reconnect never touches
safety/actuation or perception), and deployed+enabled the unit for the first time this phase
(previous phases only built A0's stub). `D5` unaffected: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
keeps DDS itself host-local, but `foxglove_bridge`'s own websocket (port 8765, `address=0.0.0.0`,
both package defaults) is a separate cross-host bridge an off-board Foxglove client can reach over
the LAN regardless.

**Verified live (real, ROS-side):** `foxglove_bridge` starts clean under `ippolit-viz`, advertises
every topic currently on the graph (60+ channels seen in the log — `/map`, `/scan`, `/cliff*`,
`/object_map`, `/vision/floor`, `/cmd_vel*`, etc.), and the websocket port is confirmed listening
on `0.0.0.0:8765`. All 47 tests still pass; apt's post-install trigger unexpectedly restarted
`ippolit-perception` as a side effect of installing the new package (a shared-library dependency
overlap) — confirmed it recovered cleanly with all 3 nodes back up.

**`docs/foxglove-layout.json` was authored from schema knowledge, NOT verified against a real
Foxglove client** — no browser/Foxglove desktop app is available in this environment, so unlike
everything above, "the layout loads and every panel renders correctly" is unconfirmed. Treat it
as a strong first draft: open it in Foxglove, expect to need small fixes (panel config schemas do
shift between Foxglove versions), and treat that as the actual acceptance test for this file.

The layout covers what's achievable with TODAY's topics: one 3D panel doing double duty for
"map+masks" and "3-D (markers/pose/scan/TF)" (Foxglove's 3D panel natively renders
`nav_msgs/OccupancyGrid`, so there's no separate 2-D map panel type — RViz-style composition, one
panel), Raw Messages panels for `/vision/floor` and `/object_map` (both are still JSON-on-String
per A3's not-yet-done topic-typing, so they show as a JSON tree, not a proper numeric Plot panel
-- revisit once A3 lands typed `FloorDrop`/`MappedObjectArray` messages), a placeholder Raw
Messages panel for `/diagnostics` (nothing publishes it yet -- inert until A5), and a Teleop panel
publishing to `/cmd_vel_teleop` (real and functional today, given F1's `twist_mux` already
subscribes there).

**Explicitly NOT done / real gaps, not oversights:** no camera panel. `q6a_vision` only exposes
its feed as plain HTTP MJPEG (`:8090` raw, `:8093` annotated) — Foxglove's Image panel needs a
`sensor_msgs/Image` or `CompressedImage` *topic*, and no node currently republishes the camera
that way. Bridging MJPEG into a proper ROS image topic is a real, separate piece of work, not
something implicit in "foxglove_bridge + a layout file" — flagged here rather than faked with an
unsupported panel type.

---

## 2026-07-13 — Phase F1 (core actuation layer): cmd_vel_bridge + twist_mux built, software-verified; physical calibration deferred

Built the D2 "single actuation node" the architecture doc calls for: `cmd_vel_bridge`
(`ippolit_control`) is now the *only* node that calls Valetudo's
`HighResolutionManualControlCapability` REST endpoint to actually drive, subscribing `/cmd_vel`
(the output of a new `twist_mux` instance merging `/cmd_vel_safety` priority 100 >
`/cmd_vel_teleop` 50 > `/cmd_vel_nav` 10, the last not wired to anything until F4). `cliff_guard`
now ALSO publishes a zero `Twist` on `/cmd_vel_safety` at 10 Hz while wheel-tripped (on top of,
not instead of, its existing direct REST-disable backstop) — two independent stop mechanisms so
one failing (e.g. `cmd_vel_bridge`'s REST calls wedged) doesn't remove the other.

`cmd_vel_bridge` implements every piece of the F1 spec: the G1 explicit-zero watchdog (Valetudo
holds the last velocity — ceasing to publish is not a stop, so it actively forces zero after
`watchdog_timeout_s` with no fresh `/cmd_vel`), the ~6.6 Hz persistent sender rate (matching the
pre-ROS `q6a_drive.py`'s empirically-tuned value), lazy enable-on-first-real-command +
idle-auto-disable ownership of the REST capability (never proactively enables at startup, so an
idle system with nothing publishing `/cmd_vel` never arms manual control or spins up the turret),
`max_safe_vel` clamped and range-validated at 0.4 (the same validated envelope as
`q6a_creep_test.py`), and reverse refused (negative `linear.x` clamps to 0) since the sign/scale
isn't calibrated yet.

**The Twist -> Valetudo (velocity, angle) mapping itself is an explicit, documented
PLACEHOLDER** (`linear_scale`/`angular_to_deg_scale`, both defaulted to 1.0) — this genuinely
needs verification against `/odom_laser` (G8) with the real robot driving, which the user
deferred alongside F0's physical map-resume test this session (not set up to supervise a
physical drive right now). Per the doc's testing plan, the mapping+clamp logic itself was
factored into a pure `twist_to_valetudo()` function with no rclpy dependency and covered by 9
new pytest unit tests (forward scale+clamp, reverse rejection, angular deg conversion+clamp
both directions, zero-Twist passthrough) — this is what COULD be verified without physically
moving the robot, and was.

Verified live (software only, zero physical motion): `colcon test` 47/47 green (up from 38);
`ros2 param get` matches deployed YAML; both `twist_mux` and `cmd_vel_bridge` start cleanly under
`ippolit-core` and stay completely inert (no REST calls at all, confirmed via journal grep) since
nothing publishes `/cmd_vel_teleop`/`/cmd_vel_nav` yet and `cliff_guard` isn't tripped;
`cliff_guard`'s existing `/cliff/ahead` behavior unaffected by its edit. Hit one new gotcha: an
empty `locks: {}` in `twist_mux.yaml` crashes the node at startup
(`parameter_value_from failed for parameter 'locks': No parameter value set`) — twist_mux needs
the `locks` key omitted entirely when there are none to configure, not present-but-empty.

**Explicitly NOT done this session** (needs the user physically present, paired with F0's
deferred physical test): the actual linear/angular scale calibration against `/odom_laser`;
live driving verification (teleop Twist actually drives; killing the publisher stops <0.5s;
lifted-wheel test zeroes `/cmd_vel` regardless of other publishers — the doc's stated F1
acceptance criteria); and `q6a_drive.py`'s reimplementation as a `/cmd_vel_teleop` publisher
(lower priority — it's a supervised manual tool, and reimplementing it is only meaningful once
the mapping above is calibrated). `docs/navigation-architecture.md`'s F1 entry is marked
🔶 IN PROGRESS, not done, reflecting this.

---

## 2026-07-12 — Phase A2 COMPLETE: declared ROS parameters replace env-var configuration

Converted every remaining `os.environ.get(...)` tunable across all six nodes that had them
(`cliff_guard`, `q6a_laser_odom`, `q6a_map_persist`, `q6a_announce`, `q6a_objmap`, `q6a_vision`)
into declared ROS parameters with type + description, loaded from a matching
`ippolit_bringup/config/<node>.yaml` via `<param from="..."/>` in each node's launch entry.
`audio_bridge`, `mcu_node`, `lds_scan_node`, and `valetudo_bridge` (A1) already used declared
parameters exclusively, so no changes were needed there.

`ROBOT_ADDR` is the one deliberate exception: per the architecture doc's A2 rule, machine-local
deployment values stay outside the ROS parameter system as env vars sourced from
`/etc/default/ippolit-robot`. Grepping the whole workspace afterward confirms it is now the
*only* `os.environ` read left in any node — Phase A2's stated completion criterion.

Safety-critical constants got range validation via `ParameterDescriptor`
(`floating_point_range`/`integer_range`), not just a declaration: `cliff_guard`'s MiDaS/LiDAR
cliff thresholds (deliberately narrow bands around the 2026-07-09 stairwell calibration, not
wide-open) and `q6a_map_persist`'s `min_resume_bytes` (floor of 1024B keeps the G4 crash-guard
meaningful — see that node's CRASH FOUND note). Verified live: `ros2 param set /cliff_guard
midas_stop 5.0` is correctly rejected ("Parameter midas_stop out of range Min: 0.2, Max: 0.6").

`q6a_vision.py` needed the most restructuring: its module-level `puller()` thread and the MJPEG
view HTTP server previously read module-global constants (`MJPEG_URL`, `VALID_ROWS`,
`VIEW_PORT`) set once at import time from env vars. Since ROS parameters are per-node-instance,
both were moved to start from inside `VisionNode.__init__` (after parameters are declared and
resolved), with `puller()` changed to accept `(mjpeg_url, valid_rows)` as explicit arguments
instead of reading globals — keeping it a plain, testable function with no rclpy dependency.
Same treatment for `q6a_laser_odom.py`'s `icp()` function (now takes `iters`/`max_corr`/`min_pts`
as explicit arguments with plain literal defaults, rather than defaulting to module constants
computed from env vars at import time).

List-valued env vars (`q6a_announce`'s `ALLOW`, `q6a_objmap`'s `ALLOW`) became `string[]`
parameters (a YAML list) rather than a single comma-separated string — more idiomatic for ROS
params and matches how they're naturally expressed in the config YAML. `q6a_objmap`'s `DYNAMIC`
set (built from an env var but never actually referenced anywhere in the file — dead code
predating this session) was dropped rather than carried forward as a phantom parameter.

All 38 tests across 10 packages pass. Verified live post-restart: every converted node's
parameters checked with `ros2 param get` against the deployed YAML, and each systemd group
(`ippolit-core`, `ippolit-perception`) restarted and confirmed healthy — single clean instance of
every node, `/cliff/ahead`, `/vision/detections`, `/vision/floor` all publishing correctly,
`q6a_vision`'s NPU/YOLO+MiDaS inference still live (same benign QNN loader probe/fallback
messages as before — not a regression, see G21).

Next per `docs/navigation-architecture.md`'s suggested order: F0-F3 (feature track — map/camera
calibration verification, cmd_vel bridge, Foxglove visualization, full-room mapping drive) or A3
(typed interfaces retiring JSON-on-String topics).

---

## 2026-07-12 — Phase A1 COMPLETE: q6a_vision + q6a_objmap migrated into ippolit_perception; G21 found

Migrated the last two A1 nodes: `q6a_vision.py` (robot camera -> YOLO(NPU)+ByteTrack+MiDaS ->
`/vision/detections` + `/vision/floor` + annotated `:8093` MJPEG) and `q6a_objmap.py` (semantic
object map fusion + disk persistence). Also folded `q6a_yolo.py` and `q6a_bytetrack.py` — until
now standalone helper scripts living in `~/` on the Q6A, imported via a `sys.path.insert` hack —
into the package proper as real submodules (`ippolit_perception/q6a_yolo.py`,
`ippolit_perception/q6a_bytetrack.py`), removing the path hack entirely. All four files written
flake8/pep257-compliant from the start per G20; only a handful of E127 continuation-indent slips
and D403 (docstrings starting with a mixed-case acronym like "IoU"/"LiDAR"/"MiDaS" fail
pydocstyle's `.capitalize()` comparison — rephrase to start with a normal word) needed fixing
after the first deploy.

**G21 (found live, cutover of `q6a_vision`): ROS 2 XML launch's `<env>` action REPLACES the named
variable, it does not append.** `q6a_vision` needs `LD_LIBRARY_PATH` to include the QNN/QAIRT
native lib dir for `qai_appbuilder` (Hexagon NPU) — the pre-migration systemd unit got this right
by accident, since `Environment=LD_LIBRARY_PATH=<qnn-path>` ran *before* `ExecStart`'s `source
/opt/ros/jazzy/setup.bash`, and ROS's setup script itself *prepends* its own lib dir onto
whatever was already there. Setting the same value via a node-scoped `<env>` in
`perception.launch.xml` runs *after* the systemd unit's setup.bash has already populated
`LD_LIBRARY_PATH` with ROS's own libs (`librcl_action.so` etc.) — so the plain `<env
value="...">` wiped them out, and `q6a_vision` crash-looped on `import rclpy` itself
(`ImportError: librcl_action.so: cannot open shared object file`). Caught immediately because the
node was simply absent from `ros2 node list` after the first restart. Fixed by appending instead
of replacing: `value="$(env LD_LIBRARY_PATH ''):<qnn-path>"`. Worth checking any future `<env>`
use for the same replace-vs-append trap, especially for any variable ROS/rclpy itself depends on.

Given `q6a_vision` holds an exclusive NPU/HTP session, verification used a different pattern than
prior migrations: rather than briefly overlapping old+new instances, the legacy
`q6a-vision.service`/`q6a-objmap.service` were stopped FIRST, the new nodes verified standalone
(confirmed live YOLO+MiDaS inference at ~8Hz, `/vision/detections`+`/vision/floor` publishing real
data), then cut over via `ippolit-perception.service` with zero overlap. A vendor QNN loader
`libQnnHtpV68Skel.so.2` "not found" message during startup is a harmless versioned-then-unversioned
probe/fallback in the underlying fastrpc stack, not a real error (confirmed by the immediate
successful fallback + working inference right after).

**Phase A1 is now fully complete**: `ippolit_drivers`, `ippolit_safety`, `ippolit_localization`,
and `ippolit_perception` all migrated and cut over in production; every legacy standalone
systemd unit stopped+disabled; single clean instance of every node confirmed. Next per
`docs/navigation-architecture.md`: A2 (declared ROS parameters replacing env-var reads) or
resuming the feature-track work (F0-F3) — see the doc's suggested interleaved order.

---

## 2026-07-12 — Phase A1: q6a_laser_odom + q6a_map_persist migrated into ippolit_localization, cut over

Migrated the last two localization scripts: `q6a_laser_odom.py` (LiDAR scan-matching ICP odometry,
publishes `/odom_laser` + `odom->base_link` TF) and `q6a_map_persist.py` (slam_toolbox pose-graph
serialize/deserialize, the G4 near-empty-graph segfault guard). Both written flake8/pep257-compliant
from the start per G20. `slam_toolbox` itself (the vendor `async_slam_toolbox_node`) deliberately
stays on its own standalone `q6a-slam-toolbox.service` for now — its lifecycle configure/activate
dance is a separate concern from this migration's scope (see G3); only updated that unit's `After=`
line to point at `ippolit-core.service` instead of the now-retired `q6a-laser-odom.service`.

Given `q6a_map_persist`'s known segfault risk (G4: `deserialize_map` on a near-empty pose graph
crashes `slam_toolbox`), verification was extra careful: checked the current saved pose-graph size
first (7769B, well under `MIN_RESUME_BYTES`'s 50KB threshold) so the guard would apply unchanged,
then launched both new nodes standalone with remapped names against the live system, confirmed
`q6a_map_persist_test` correctly refused to resume the small graph (identical log line to
production) and a `serialize_map` call succeeded after one `SAVE_PERIOD_S` cycle, before cutting
over. Old `q6a-laser-odom.service`/`q6a-map-persist.service` stopped+disabled;
`q6a-slam-toolbox.service` confirmed still `active` (lifecycle state unaffected) after cutover.

`ippolit_localization` is now migrated. Remaining A1 work: the rest of `ippolit_perception`
(`q6a_vision.py`, `q6a_objmap.py`, still on old standalone units).

---

## 2026-07-12 — Phase A1: cliff_guard migrated into ippolit_safety, cut over in production; written lint-clean from the start

Migrated `cliff_guard.py` (SAFETY: wheel-drop hard e-stop + MiDaS advisory `/cliff/ahead`) into
`ippolit_safety`, wired into the new `safety_control.launch.xml` (part of the `ippolit-core`
systemd group, already included by `core.launch.xml`), and cut over in production. Since this is
the safety-critical node, verification was extra careful: launched the new node standalone first
with a remapped node name + remapped `/cliff/ahead` topic against the live system (real `/cliff`,
`/bumper`, `/vision/floor`, `/scan` subscriptions — read-only, no collision risk), confirmed no
exceptions over ~18s with real data flowing and the expected startup log/latched initial publish,
before restarting `ippolit-core` and cutting over from the standalone `q6a-cliff-guard.service`.
Single clean `/cliff_guard` instance and correct `/cliff/ahead` output confirmed afterward.

Applied G20's lesson directly this time: wrote the migrated file flake8/pep257-compliant from the
start (99-char lines, single quotes, D213 docstrings, no multi-statement one-liners, non-stdlib
imports as one alphabetical block) instead of copying the pre-ROS script verbatim and fixing lint
after the fact. Cost was near-zero — one remaining `colcon test` failure on the first deploy,
`I101` import-name ordering (`rclpy.qos`'s multi-name import needs `qos_profile_sensor_data` first:
flake8-import-order sorts names **case-insensitively**, so a lowercase name can sort before
CamelCase names in the same import — worth remembering for any future multi-name import from
`rclpy.qos` or similar mixed-case modules). Fixed, and all 38 tests across 10 packages passed
clean on the very next deploy.

`ippolit_safety` is now migrated. Next per `docs/navigation-architecture.md`'s A1 order:
`ippolit_localization` (`q6a_laser_odom.py`, `q6a_map_persist.py`), then the rest of
`ippolit_perception` (`q6a_vision.py`, `q6a_objmap.py`).

---

## 2026-07-12 — Phase A1 finishes the four foundational drivers: mcu_node, lds_scan_node, valetudo_bridge migrated + cut over; G20 (real lint debt, not another false alarm)

Completed A1's `ippolit_drivers` package: `mcu_node.py`, `lds_scan_node.py` (+ its `lds_decode.py`
helper), and `valetudo_bridge.py` all moved in unchanged in logic, wired into `drivers.launch.xml`
(the `ippolit-core` systemd group), and cut over in production. `valetudo_bridge` is the first node
in this migration to use plain `argparse` (`--host`) instead of declared ROS parameters, which meant
passing it via the `<node>` element's `args=` attribute rather than a `<param>` child — a new launch
pattern, not previously needed by the other three drivers.

**mcu_node + lds_scan_node** followed the now-familiar old+new overlap verification pattern (both
briefly ran alongside their pre-migration standalone-script equivalents under distinct test node
names, confirmed healthy, then the old systemd units were stopped+disabled and a single clean
instance confirmed). The safety-critical `angle_offset_deg=43.0` LiDAR bearing calibration
(2026-07-12's sign-and-offset fix, see `lds_scan_node.py`'s own header) was carried over exactly and
is called out with an explicit "do not lose this" comment in `drivers.launch.xml`.

**valetudo_bridge** cutover: launched standalone first with remapped topics
(`/map_test2` etc.) against the real robot, confirmed `/map` (176x63 grid, correct resolution/origin),
`/battery` (correct percentage), and `map->base_link` TF all matched the pre-migration bridge's
output, then restarted `ippolit-core.service` (which now includes `valetudo_bridge` in
`drivers.launch.xml` for the first time), verified the brief old+new node-name overlap, and
stopped+disabled the legacy standalone `valetudo-bridge.service`. Single `/valetudo_bridge` instance
confirmed afterward.

**G20 — `colcon test --python-testing pytest` surfaced 148+14 REAL flake8/pep257 violations, not
another XML-comment false alarm.** Having hit G16 (literal `--` in XML comments) three times, the
plan was to make `colcon test` (which includes `ament_xmllint`) part of every deploy from here on.
First run after adding `valetudo_bridge` reported "4 failures" across `ippolit_drivers` and
`ippolit_perception` — this time genuine accumulated lint debt in every driver file copied so far
(`audio_bridge.py`, `lds_decode.py`, `lds_scan_node.py`, `mcu_node.py`, the new `valetudo_bridge.py`,
and `q6a_announce.py`), never previously checked because `colcon test` hadn't been run against the
full set before. Categories hit: `ament_flake8`'s ROS-standard plugin set (line length 99,
`flake8-import-order` — all non-stdlib imports must sort as ONE alphabetical block, not
grouped-by-origin; `flake8-quotes` single-quote preference; `flake8-comprehensions`; ambiguous
single-letter names; multi-statement one-liners), and `ament_pep257`'s **D213** convention (a
multi-line docstring's summary must start on the *second* line, i.e. `"""` alone on its own line —
the opposite of this project's established "summary immediately after `"""`" style used everywhere
else in the repo, e.g. `docs/*.md` prose). Fixed by reflowing every affected file (no logic changes)
and moving all 7 flagged module/function/method docstrings to the D213 form. Hit the same
comment-continuation-indentation trap as G16bis twice more while reflowing wrapped comments (a
comment-only continuation line must match the indentation of the **following** code line, not just
look visually aligned) — caught both via a local heuristic script before redeploying. All 38 tests
across all 10 packages pass clean now (`colcon test-result --all` → 0 failures). **Going forward, ROS
Python source in this repo should be written flake8/pep257-compliant from the start** (99-char lines,
single quotes, D213 docstrings, one statement per line) rather than copied verbatim from the
pre-ROS `scripts/robot|companion/` originals and fixed after the fact.

`ippolit_drivers` (all 4 foundational nodes) and the perception package are now both migrated and
production-cut-over. Next: `ippolit_safety`/`cliff_guard.py` (the wheel-drop e-stop + MiDaS advisory
layer — safety-critical), then `ippolit_localization` (`q6a_laser_odom.py`, `q6a_map_persist.py`) and
the rest of `ippolit_perception` (`q6a_vision.py`, `q6a_objmap.py`), per
`docs/navigation-architecture.md`'s phase order.

---

## 2026-07-12 — A1 continues: q6a_announce migrated, real-vs-synthetic testing gotcha (G19)

Second node migration: `q6a_announce.py` moved into `ippolit_perception` (better fit than the earlier
guess of `ippolit_safety` — it narrates detected objects, unrelated to safety) unchanged in logic, wired
into `perception.launch.xml` / `ippolit-perception.service`. Old standalone `q6a-announce.service` stopped
and disabled after the new one was verified working.

Verification initially looked broken again (repeated synthetic `/vision/detections` publishes never
triggered an announcement) — but this time the cause was simpler and specific to this node: `q6a_vision`'s
real detection stream runs continuously at ~8Hz, and `q6a_announce`'s persistence counter *decays* on
every message that doesn't contain the target label. Sparse synthetic test injections were getting decayed
back down by the real (chair-free) frames between injections. Fixed the test, not the node: stopped
`q6a-vision` for the duration of an isolated test, confirmed "announce: I see a chair" fired and the full
chain (`q6a_announce` -> `/robot/speak` -> `audio_bridge` -> spoken audio) worked end-to-end, then
restarted `q6a-vision` normally. Recorded as G19 — a different class of testing pitfall than G18's "just
wait longer": here the fix was isolating the test from a competing real data source, not patience.

---

## 2026-07-12 — Phase A1 begins: audio_bridge migrated end-to-end, real production cutover, 3 more gotchas

First node migration of A1 (task #23): `audio_bridge.py` moved into `ippolit_drivers` unchanged in logic,
wired into `drivers.launch.xml` (part of the `ippolit-core` systemd group), and **actually cut over in
production** — the old standalone `audio-bridge.service` is stopped and disabled; `/robot/speak` is now
served by the packaged node via `ippolit-core.service`.

This one node took far longer than expected because of three more real bugs (not migration-logic bugs —
the Python was copied unchanged) found via live verification, all recorded as G16-G18:

- **G16**: XML comments can't contain a literal `--` — this whole project's prose style uses it
  constantly, so it broke on the very first real `.launch.xml` edit. An em-dash red herring (a secondary
  parser's error, chased first, cost real time) turned out to be irrelevant; `encoding="UTF-8"` is good
  practice but wasn't the fix. Fixed project-wide with a script that only touches comment bodies (blindly
  sed-ing `--` also corrupts the `<!--`/`-->` delimiters themselves — hit that too, then fixed properly).
- **G17**: `colcon build --symlink-install` embeds absolute-path symlinks into `install/`; promoting a
  test workspace to its permanent path via `mv` leaves them all dangling. Fix: rebuild at the final path.
- **G18**: the real reason "nothing" seemed to happen after publishing a test utterance for over an
  hour of debugging: the SSH+Piper+ffmpeg round-trip genuinely takes ~10-15s, and every verification
  check was impatient (a few seconds). This sent the investigation down two mostly-unnecessary paths (a
  real but non-causal discovery-range mismatch between an ad-hoc test shell and the services; a red-herring
  thread-pileup theory) before landing on "wait longer" as the actual fix. Both dead ends are documented
  so they don't get re-chased, but the headline lesson is simpler: check the timeout before the theory.

Also production-restored the old `audio-bridge.service` mid-investigation as soon as the new one looked
broken, rather than leaving live TTS down while debugging — confirmed working again immediately, then
re-cut-over once the real fix (patience) was found and verified live (user confirmed hearing two
consecutive test utterances through the new production path).

Next: continue A1 with the remaining drivers (`lds_scan_node`, `mcu_node`, `valetudo_bridge`) and the
other packages (`ippolit_safety`/`cliff_guard`, `ippolit_localization`/laser-odom+map-persist,
`ippolit_perception`/vision+objmap), applying the same rigor (build, deploy, live-verify with correct
patience, disable the old unit only after confirming the new one works).

---

## 2026-07-12 — Phase A0: colcon workspace scaffold, live-validated on the Q6A + a real ecosystem gotcha found

First execution step of the architecture plan (task #23): `ros2_ws/src/` with all 10 packages from
`docs/navigation-architecture.md` §2.1 — `ippolit_interfaces` (ament_cmake, real msg definitions:
`FloorDrop`, `MappedObject(Array)`, `McuTriggers` ported field-for-field from `mcu_node.py`'s
`_TRIGGERS_BOOL_BITS`), `ippolit_description` (ament_cmake, a placeholder URDF/xacro + a
`robot_state_publisher` launch — geometry values are honest placeholders pending A4/F0.2 calibration),
`ippolit_bringup` (ament_cmake: 7 XML launch stubs, the 4 group systemd units, `deploy.sh`, `config/`
placeholder), and the 7 ament_python node packages (`drivers/control/safety/perception/localization/
navigation/teleop`) — currently empty node bodies per A0's scope, wrapped with real logic in A1. Added
`.github/workflows/ros2_ci.yml` (colcon build+test on ubuntu-24.04 via `ros-tooling/action-ros-ci`).

**Live-validated end-to-end on the Q6A** (the only machine with ROS 2 Jazzy — this dev box has none):
`rosdep install` (all deps already present), `colcon build` (clean, 10/10 packages), `colcon test`
(38 tests, 0 errors, 0 failures after two real fixes below), message generation (`ros2 interface show`),
URDF validity (`xacro` + parse check), and — going beyond A0's own bar — actually **running**
`core.launch.xml` and confirming `robot_state_publisher` starts and publishes a correct `base_link→laser`
TF (0, 0, 0.09) matching the xacro joint definition exactly.

**Three real bugs found and fixed, not just scaffolding:**
1. **`colcon test` silently ran ZERO tests for every ament_python package**, reporting Python `unittest`'s
   "Ran 0 tests / NO TESTS RAN" instead of any pytest output. Root cause: `colcon test`'s auto-detection
   of the pytest step extension keys off `setup.py`'s `tests_require` field — but the setuptools version
   in this Python 3.12 environment has deprecated/silently drops `tests_require` before colcon can read it
   back (hence the "Unknown distribution option" warning seen on every build), so auto-detection always
   falls through to the legacy `setuppy_test` step, which finds nothing. **Fix: force the extension
   explicitly with `--python-testing pytest`** (added to the CI workflow's `colcon-defaults` and to
   `docs/navigation-architecture.md` §9 as a new gotcha) — this is an ecosystem-version issue, not
   something fixable by editing package files, so it has to be forced at the `colcon test` call site
   every time. (Initially "fixed" the cosmetic warning by *removing* `tests_require` entirely — that was
   wrong, since colcon still needs the field present even though setuptools ignores it; reverted.)
2. **`ippolit_interfaces/package.xml` failed `xmllint`**: `<member_of_group>` was placed before the
   `<depend>` tags, violating package_format3's required element ordering. Fixed by moving it to just
   before `<export>`.
3. **`core.launch.xml` failed at runtime** (not just parse time) with "Failed to convert" on the
   `robot_description` param: XML launch's automatic parameter-type inference chokes on the multi-line
   xacro-expanded URDF string. Fixed with an explicit `type="str"` attribute on the `<param>` tag.

All three are now recorded in `docs/navigation-architecture.md` §9 so they don't get rediscovered.
Next: A1 (wrap the existing scripts into these packages, unchanged in logic) or F0 (verify real map
resume / calibrate the camera) — see the doc's suggested interleaved order.

---

## 2026-07-12 (rev 2) — Architecture plan rewritten to production-grade ROS 2 engineering

`docs/navigation-architecture.md` substantially revised per explicit user direction: architect the whole
solution as production-grade robot software, not a manually-managed script collection. Grounded against
current (2025/2026) ROS 2 community/industry guidance via web research.

**The big decision (D1): the historical "loose scripts + hand-scp'd files + 12 hand-ordered systemd
units" convention is RETIRED.** New foundation (Part A, phases A0-A5): colcon workspace with ~10
single-responsibility ament packages (`ippolit_interfaces/description/drivers/control/safety/perception/
localization/navigation/bringup`), declared ROS parameters from YAML (env vars retired), real message
types replacing every JSON-on-String topic (vision_msgs + a small ippolit_interfaces pkg), URDF +
robot_state_publisher as the single source of geometry truth (BODY_R etc. currently duplicated across
scripts), XML launch files with systemd shrunk to 4 supervised groups, pytest + launch_testing + lint in
GitHub Actions CI, an idempotent on-device deploy script (git pull + rosdep + colcon build — kills the
scp-drift failure class that bit us twice), diagnostics (/diagnostics + aggregator) and a rolling MCAP
incident recorder auto-snapshotted on e-stop.

**Decision records D1-D7** capture the judgment calls: keep the custom ICP laser odom for now (validated;
revisit vs rf2o/kiss-icp/robot_localization only if it becomes the bottleneck), native on-device builds
over Docker (one robot, NPU/camera stack friction, documented as future option), Foxglove over RViz
(localhost-only DDS is deliberate), stairwell as caution-zone-not-no-go (4 independent layers).

Feature phases F0-F6 (cmd_vel bridge + twist_mux, Foxglove, room mapping + stairwell masks, Nav2,
cliff-aware nav, go-to-object) unchanged in content from rev 1 but now land inside the packages, with an
interleaved A/F execution order. Rev-1's validated-behavior inventory and the G1-G12 live-learned gotchas
carried over intact.

---

## 2026-07-12 — Navigation architecture plan: docs/navigation-architecture.md

Full review of the current state + a phased, step-by-step architecture doc for the next stage: room
navigation (Nav2), full-room mapping with objects, proper stairwell handling, and live visualization —
written to be executable incrementally by a simpler model.

**Key architectural decisions recorded there:**
- **Single actuation node** (`q6a_cmd_vel_bridge`, new): geometry_msgs/Twist on `/cmd_vel` → Valetudo
  manual-control REST, with the MAX_SAFE_VEL=0.4 clamp, explicit-zero watchdog (Valetudo holds the last
  velocity), and enable/disable ownership. The ONLY AVA touchpoint; everything upstream is standard ROS.
- **twist_mux** for command arbitration (safety > teleop > nav) — eliminates by construction the
  REST-race "fighting" class of bug that burned a day of live testing.
- **Stairwell = Nav2 costmap filters**, per the user's "caution zone, not no-go" direction: a small
  lethal KeepoutFilter over the physical hole + rim, a broader SpeedFilter caution zone around it, PLUS a
  dynamic `q6a_cliff_scan` virtual-obstacle node (MiDaS drop → synthetic LaserScan → local costmap) and
  the existing reactive layers (wheel_floating pause, wheel-drop e-stop, AVA recovery at ≤0.4). Four
  independent layers; the map annotation is authored, not sensed, because a 2-D LiDAR sees the hole as
  free space.
- **Foxglove (foxglove_bridge websocket) over RViz** for visualization — the deliberate
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST setting makes off-board RViz a non-starter without DDS surgery;
  a single on-board websocket sidesteps it entirely and adds teleop/map/3D panels.
- Phases 0-7 with per-step verification gates, plus a "gotchas" section (G1-G12) capturing every landmine
  learned in the recent sessions (Valetudo velocity-hold, DDS discovery latency, lifecycle nodes, the
  deserialize segfault, rclpy shutdown limits, the load-bearing fanoff gate, pkill self-match, wheel-odom
  pivot slip, MiDaS blind zone, placement variance, REST schemas).

**Verified while writing it**: nav2-bringup/costmap-2d/map-server are already installed on the Q6A;
twist-mux and foxglove-bridge are not (install steps included); env file confirms CycloneDDS +
localhost-only discovery.

---

## 2026-07-12 — Map + object-map persistence, and a real slam_toolbox crash found + mitigated

**Context**: neither the SLAM occupancy grid nor the semantic object map survived a reboot/restart before
today — both lived only in memory. This was flagged as the single biggest gap in the mapping pipeline
("what's left from SLAM and mapping the apartment") and tackled directly.

**Object map (`q6a_objmap.py`)**: added `Q6A_OBJMAP_FILE` (default `/home/radxa/ros/maps/object_map.json`).
Loads on startup if present, saves every 30s and (best-effort) on clean shutdown, atomic write (temp file
+ `os.replace`) so a mid-write crash can't corrupt the persisted file. Straightforward, no ROS-service
dependency -- this half worked cleanly on first deploy.

**SLAM map**: iterated through two designs.
1. **First cut**: bash scripts (`slam_save_map.sh` on `ExecStop`, a deserialize call added to
   `slam_lifecycle_up.sh`'s `ExecStartPost`) calling slam_toolbox's own standard `serialize_map`/
   `deserialize_map`/`save_map` services -- the same services RViz's SlamToolboxPlugin buttons call.
   Not a custom protocol, just automating an existing one via the systemd hooks already used for
   slam_toolbox's Jazzy lifecycle (configure->activate) dance.
2. **User asked why not a dedicated ROS node instead of shell+systemd hooks.** Agreed and rebuilt as
   `q6a_map_persist.py` + `q6a-map-persist.service`: same three services, now owned by one node with its
   own service clients, a startup resume-retry timer, and a periodic save timer. Reverted the bash-hook
   version entirely (`slam_save_map.sh` deleted, `ExecStop` removed from `q6a-slam-toolbox.service`,
   `slam_lifecycle_up.sh` back to lifecycle-only).

**Dead end during that rebuild**: tried adding a synchronous "final save on clean SIGINT" in the node's
`finally:` block. Every attempt failed with "rcl node's context is invalid" -- rclpy's default SIGINT
handler tears the context down before `finally:` runs. Tried disabling that handler
(`signal_handler_options=SignalHandlerOptions.NO`) so the context would still be alive -- this was WORSE:
without it, plain SIGINT doesn't reliably interrupt the blocking rcl wait inside `spin()` at all, so the
process just hung until systemd's `TimeoutStopSec` elapsed and SIGKILL'd it (confirmed live: a restart
that should take ~1s took ~45s with zero log output, graceful or otherwise). Reverted; the node relies
solely on its periodic save timer, accepting a bounded staleness window equal to the save period rather
than fighting rclpy's shutdown internals.

**⚠️ Real crash found and mitigated**: live-testing the resume path, `slam_toolbox` started
**segfaulting in a repeating Configuring->Activating->crash loop**. Root-caused by isolation (stopping
`q6a-map-persist` immediately stabilized slam_toolbox) to `deserialize_map` being called against a saved
pose graph that had **zero real scan nodes** -- every rapid restart-cycle test that day had the robot
sitting still, so every "saved map" was an empty graph, and `match_type=START_AT_FIRST_NODE` against an
empty graph reliably crashes slam_toolbox's C++ process. Mitigated with a size heuristic
(`MIN_RESUME_BYTES`, default 50KB): the node now refuses to attempt `deserialize_map` if the saved
`.posegraph` file is suspiciously small to contain real scan data, logging why and starting from empty
instead. This is a heuristic, not a real fix for the underlying crash -- if slam_toolbox ever
crash-loops again right after "will resume" is logged, suspect this bug first, `systemctl stop
q6a-map-persist` immediately, and delete the saved `.posegraph`/`.data` pair. **Real end-to-end resume
(loading an actual multi-node graph from a real drive) is still unverified** -- everything tested today
was against trivial/empty graphs; this needs a real mapping drive followed by a restart to confirm.

**Side effect, unavoidable**: restarting `q6a-objmap` to deploy its persistence fix wiped out that
service's *in-memory* object map accumulated from earlier (pre-persistence) sessions -- 16 objects
(refrigerator, tv, several chairs, dining table, some with 500-900 observations) were lost, since the old
code had no way to save them before the restart. Persistence now works going forward; that specific
accumulated map needs a new drive to rebuild.

---

## 2026-07-12 (final) — LATCH_CENTER lowered then rolled back: stationary calibration doesn't fully predict
in-motion behavior

Tried lowering `LATCH_CENTER` 0.60 -> 0.55 to fix the recurring near-miss pattern (center peaking at
0.59-0.60, just under the old threshold, and never latching). Live-tested at 0.55: latch fired almost
immediately after caution entry this time (center jumped 0.00 -> 0.43 -> 0.54 -> 0.57 in ~3 ticks, ~0.4s),
and blind-creep completed cleanly to 5.2cm traveled -- but the user tape-measured the actual final distance
at **35cm from the true edge**, at the SAME physical location and setup as the two earlier successful runs
that landed at ~5-13cm. That's a real, unexplained discrepancy between the stationary tape-measure
calibration and how the signal behaves while actually driving (possibly approach-angle sensitivity, or
processing latency between frame capture and the published reading, or something else not yet diagnosed) --
not just threshold noise on the scale seen elsewhere today. Rather than keep tuning blindly without
understanding the cause, rolled `LATCH_CENTER` back to 0.60 (the value behind both actual successful
landings), accepting the occasional missed-latch fallback (safe -- continuous caution creep to wheel-drop)
over a threshold that's now demonstrated it can also land 30cm+ off in the "too conservative" direction.

**Open item for next session**: the blind-creep distance has ranged from 5.2cm to 35cm across attempts at
supposedly the same setup -- every miss so far has erred toward stopping FARTHER from the edge (safe
direction, never closer/riskier), but the spread is too wide to trust for precision close-approach work
yet. Needs either an in-motion calibration pass (not just stationary) or a hypothesis test for what's
actually driving the variance (approach angle, latency, floor lighting) before further threshold tuning is
likely to help.

---

## 2026-07-12 (even later) — blind-creep latch AND-gate fragility fix + vel=1.0 incident does NOT reproduce
under the caution zone

**Latch AND-gate was too brittle.** First live test of the odometry blind-creep never actually latched: on a
run where caution triggered and drove cleanly, `center` and `sharp` kept narrowly missing their thresholds
on the SAME tick (e.g. `center=0.60/sharp=13.5`, next tick `center=0.56/sharp=14.0`) — the robot drove
continuously at caution speed all the way to a safe wheel-drop stop, but blind-creep never got a chance to
run. Loosened `LATCH_SHARP` 14.0 -> 8.0 (closer to how `MIN_SHARP` is just a noise-reject gate for caution
entry, rather than a second precision requirement) since `center` crossing 0.60 is already the primary
confidence signal. Also nudged `BASE_ENTER` 0.52 -> 0.56 and `BLIND_CREEP_M` 0.07 -> 0.05 after two clean
runs landed at slightly different final distances (user: 2nd run "started to slow down too early" and
"stopped a bit further" than the 1st) — inherent sensor noise, expect this to reduce but not eliminate
run-to-run variance. **Verified live again at vel=0.4 (max cap)**: caution + latch + blind-creep all fired
cleanly, landed at 5.2cm traveled since latch, no wheel-drop, no fighting.

**User then asked to explicitly bypass `MAX_SAFE_VEL` and retest at true vel=1.0** — the exact speed that
caused the original wheel-hang incident earlier the same day. Flagged this clearly before doing anything
(this is the documented cause of a real incident, and the caution-zone/blind-creep logic was only ever
designed/tested at ≤0.4) and got explicit confirmation, plus confirmed the user was physically ready to
catch the robot. Added a `--force-unsafe-velocity` opt-in flag (bypasses the clamp only when explicitly
passed; default behavior for any future run is unchanged) rather than editing the safety constant directly.

**Result: the incident did NOT reproduce.** Even with the raw velocity uncapped, the caution zone triggered
promptly (`center=0.51`, well before the true edge) and dropped the ACTUAL driving speed to `CAUTION_VEL`
(0.15) for essentially the entire remaining approach (~7.5s) — meaning the robot's real momentum at the
moment of wheel-drop was based on 0.15, not 1.0. Blind-creep's latch didn't quite fire this run either
(`center` peaked at 0.59, just under 0.60 -- another near-miss, noted for further LATCH_CENTER tuning), but
wheel-drop stopped it cleanly regardless. **User confirmed: "no repeat of the incident."** This validates
that the day's caution-zone work is a real, independent safety layer -- it protected against the original
failure mode even when the speed cap that was ALSO built in response to that incident was deliberately
bypassed.

---

## 2026-07-12 (latest) — MILESTONE: odometry blind-creep — first-ever planned close-approach stop, not a
wheel-drop recovery

**Stationary MiDaS calibration pass** (`scripts/companion/q6a_creep_test.py` region + the throwaway
`midas_calib.py` logger): robot placed by hand at 8 tape-measured distances from a real edge (65/50/40/30/
20/15/10/5cm), `/vision/floor` center+sharp logged at each, stationary, no driving:

| Distance | center (avg) | sharp (avg) |
|---|---|---|
| 65cm | 0.28 | 2.8 |
| 50cm | 0.45 | 5.6 |
| 40cm | 0.41 | 6.8 |
| 30cm | 0.41 | 7.9 |
| 20cm | 0.65 | 16.1 |
| 15cm | 0.68 | 17.2 |
| 10cm | 0.54 | 12.1 (declining) |
| 5cm | 0.06 | 2.4 (fully blind) |

Findings: the strongest/most confident reading is **~15-20cm out** (center 0.65-0.68, sharp 16-17); decline
starts between 15cm and 10cm; fully blind by 5cm (matches every wheel-drop-time reading logged all day).
`center` is noisier than expected in the 30-50cm band (barely moves, sometimes non-monotonic) — `sharp`
tracks proximity more cleanly there, though both converge to the same story near the edge.

**Implemented odometry blind-creep on top of the (already-validated) event-driven caution zone**: once
`center`/`sharp` cross into the confirmed-strong band (`LATCH_CENTER=0.60`, `LATCH_SHARP=14.0`), latch the
current `/odom/wheel` position and STOP trusting MiDaS's live reading entirely — from that instant, creep
`BLIND_CREEP_M` (default 0.07m, tunable via `--blind-creep`) tracked purely by odometry distance from the
latch point. Deliberately conservative (targets landing ~8-13cm from the true edge, not the full 5-10cm
goal, given the calibration is only 8 points). Straight-line wheel odometry is trusted here specifically
because the previously-established unreliability was for IN-PLACE PIVOTS (wheel slip during rotation), not
forward travel. `/wheel_floating` reactive pausing stays active during blind creep (doesn't depend on
MiDaS); wheel-drop (`/cliff`) remains the final, unconditional safety net throughout.

**Verified live, first try**: caution entered cleanly (event mode, continuous drive, no fighting), latched
at `center=0.63 sharp=14.7`, blind-crept smoothly from 0.0cm to 7.0cm over ~2.3s, and **stopped itself on
the distance target** — `STOP: BLIND-CREEP target reached (7.0cm since latch)`. **No wheel-drop fired.**
User confirmed both that it stopped safely on its own and that the measured real distance to the edge was
in the expected ~8-13cm range. This is the first time this project has stopped the robot at a *planned*
safe distance from a real edge, rather than relying on AVA's own wheel-drop recovery as the terminal event.

---

## 2026-07-12 — q6a_creep_test.py v7: speed-scaled MiDaS caution zone (not a stop-ramp) + hard velocity ceiling

Follow-up to the max-velocity (1.0) near-miss (robot's momentum carried it past the point where AVA's own
wheel-drop recovery could work, leaving it hanging at the edge with wheels off the ground — recovered by
hand). Two fixes, driven directly by user guidance on what a real driving strategy should look like:

1. **`MAX_SAFE_VEL = 0.4` clamped unconditionally in code** (not just advised) — every velocity ≤0.4 tested
   let AVA's wheel-drop recovery work reliably; 1.0 didn't. The ceiling being advisory-only is exactly what
   caused the incident, so `CreepTest.__init__` now clamps `self.vel = min(vel, MAX_SAFE_VEL)` regardless
   of what `--velocity` requests.
2. **MiDaS caution zone reintroduced, but NOT as a stop-ramp.** User's explicit correction: a detected edge
   should mean "drive with caution at reduced speed," not "this location is now off-limits" — a permanent
   no-go would break the actual edge-following use case (traveling parallel to a drop at 5-10cm). New
   design: `enter_thresh(v) = 0.50 - 0.15*(v/MAX_SAFE_VEL)` — the trigger point moves earlier (lower
   center threshold = farther out) as commanded speed rises, approximating a physics-based stopping
   distance without a true metric MiDaS calibration (MiDaS is a relative disparity signal, not calibrated
   to real distance — a calibration pass was offered and explicitly deferred this session, "skip
   calibration, use rough estimates for now").
3. **Direction-change re-arm**: the MiDaS blind spot is specific to the current approach angle, not the
   physical location. `move()` now clears the caution latch whenever the commanded angle changes by more
   than `REARM_DEG=15`, so a new heading gets a fresh MiDaS assessment instead of trusting a stale one.

**Two more bugs found and fixed via live testing the same day, both re-deriving lessons from v1-v5:**
- **Removed the center-based caution EXIT.** First cut cleared caution once `center` dropped back below
  threshold (with hysteresis) — but live-tested, it exited right at the true edge (`center=0.06`, the known
  MiDaS blind spot, misread as "clear floor") and resumed full 0.3 velocity for ~1.3s before wheel-drop —
  reproducing the exact v1-v5 "fighting AVA" failure at a caution-zone level. Fix: caution is now a one-way
  latch — once triggered it holds `CAUTION_VEL` until the direction-change re-arm or the run ends
  (wheel-drop/time bound). No exit on a low reading, ever.
- **Added pulse-and-settle inside caution** (`PULSE_ON_S=0.4` / `PULSE_OFF_S=0.5`). Even a *constant* 0.15
  creep still visibly fought AVA a couple of times right at the true edge before the final wheel-drop —
  user confirmed live ("was trying to fight a couple of times then stopped"). Some protective reflex (not
  necessarily the same signal as our decoded `/cliff` bit) nudges the robot back, and continuously
  re-commanding forward every tick just re-pushes into it. Switched caution to short driving bursts
  separated by an explicit `vel=0` pause, giving the reflex room to settle instead of being fought every
  cycle. **Verified live, twice** (once cut short by an oversized outer test-harness timeout on my end, not
  a robot issue; once to completion) — the second full run held the pulse pattern cleanly through ~6s of
  caution and stopped in one clean wheel-drop event, **user-confirmed: "Yes, clean this time."**

**Still open** (discussed, not yet implemented): getting reliably close (5-10cm) to a bare edge needs an
odometry-based "blind creep" for the final approach once MiDaS's last confident reading is latched — MiDaS
itself goes blind exactly in that range. This needs one calibration data point (real distance from "MiDaS's
last strong/sharp reading" to the true edge) that the user deferred; noted as the next concrete step.

---

## 2026-07-12 (later) — q6a_creep_test.py: found the real earlier signal (`/wheel_floating`), settled on
continuous-in-caution as clean, entry-threshold saga, two operational bugs found and fixed

Continuation of the same day's work above. User pushed back on the pulse-and-settle result ("it was moving
with jerks. not smoothly. can it drive smooth just slower?"), which led to a chain of live tests narrowing
down what the actual fix needed to be:

- **Continuous, slower (`CAUTION_VEL=0.08`, no pulsing): still fought.** Falsified "slower avoids
  provoking the reflex" — confirmed it's specifically about reaching zero, not magnitude.
- **Smooth sinusoidal soft-pulse (`--soft`, oscillating `0.02<->0.15`, never a discrete step): still
  fought**, live-confirmed by the user picking "same fighting as before" — nails down that even a *smooth*
  nonzero trough doesn't release whatever reflex fires; only an actual return to zero does. Reverted the
  default back to the hard on/off pulse (jerky, but the only thing verified clean at that point).
- **Entry threshold saga**: user reported caution triggering "~half a meter before the edge" (`BASE_ENTER`
  0.50) — raised to 0.62 (still ~20-30cm out per user estimate) — raised again to 0.68 — which then live-
  tested to **never trigger at all** in one run (`center` peaked at 0.56, threshold was 0.64, robot drove
  full speed the whole way to the edge and "was fighting 3 times" right at the end, completely outside any
  caution logic). Lesson: `center`'s peak-before-blind-spot is noisy run-to-run (~0.5-0.76 seen across
  today's tests), so a threshold tuned for "trigger closer" risks never triggering — worse than triggering
  early, since a miss reproduces the exact full-speed-to-edge danger this whole feature exists to prevent.
  Settled on `BASE_ENTER=0.52`, biased toward reliably triggering over precisely-timed triggering.
- **Found the actual missing signal, `/wheel_floating`.** Our `/cliff` subscription only ever watched the
  downward IR sensors (Triggers byte[1]) — already established to co-fire only AT the final wheel-drop
  instant. There's a separate, dedicated `/wheel_floating` topic (Triggers byte[0] bits 6-7, decoded
  2026-07-11) that no test this entire day had ever subscribed to. Added a new `event` mode (now default):
  drive continuously at `CAUTION_VEL`, pause (vel=0) only when `/wheel_floating` actually fires, hold for
  `WF_SETTLE_S=0.6` after it clears. `--pulse` (old hard on/off) and `--soft` kept as fallback/reference.
- **Verified live, twice, clean**: at both `--velocity 0.3` and `--velocity 0.4` (the `MAX_SAFE_VEL` cap),
  caution triggered correctly and drove **continuously** the whole way — `/wheel_floating` never even
  fired, no fighting either time, single clean wheel-drop stop both times. This also resolves the earlier
  apparent contradiction (continuous 0.15 fought once, was clean another time): most likely just run-to-run
  physical variability in hand-placement, not a deterministic bug — with entry timing and warm-up fixed,
  every subsequent continuous-mode test has been clean.

**Two operational bugs hit and fixed along the way (not robot-behavior bugs, but real live-test blockers):**
- **`pkill -f` self-kill, subtler variant.** Documented before for camstream/fanoff (a pkill pattern
  matching its own invocation's cmdline), but hit a new form here: since `ssh host 'multi-line script'`
  passes the WHOLE script as one argument to the remote `bash -c`, that shell's own `/proc/self/cmdline`
  contains every line of the script — including a *later*, unrelated, unbracketed occurrence of the same
  filename (a `nohup python3 /tmp/cliff_monitor.py &` line further down). `pkill -f "[c]liff_monitor.py"`
  (the usual bracket-escape trick) still matched that later occurrence and killed the invoking shell,
  silently dropping the whole SSH session with no output. Fix: never combine a `pkill -f <name>` with a
  later literal occurrence of `<name>` in the *same* multi-line SSH command — split them into separate SSH
  invocations.
- **DDS discovery latency was masquerading as a sensor outage.** The script's warm-up gave up after 8s if
  `/scan` hadn't arrived, but a fresh rclpy node's discovery of an existing publisher can legitimately take
  ~10-12s on this setup (matches the previously-documented FastDDS re-match fragility). Confirmed the LiDAR
  itself was fine throughout (ring buffer's write-pointer advancing at full rate; `ros2 topic hz /scan`
  succeeded once given a longer window) — raised the warm-up timeout 8.0s -> 20.0s.

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

## 2026-07-12 — CRITICAL SAFETY FINDING: AVA's wheel-drop recovery has a speed limit, not unconditional

Ran the simplified constant-speed script (no MiDaS, wheel-drop as the only stop) at maximum velocity (1.0)
per user request. Wheel-drop fired cleanly and the script hard-stopped correctly (matching the fix) --
but the robot's MOMENTUM at that speed carried it PAST the point where AVA's own recovery could work at
all. Wheels lost contact entirely and the robot was left hanging at the edge, unable to self-recover --
needed a physical rescue by hand.

**This is the key finding: AVA's own wheel-drop recovery, which worked reliably at every lower speed
tested today (0.4 and below, repeatedly, including oscillation cycles), is NOT unconditionally reliable.**
It has a speed-dependent limit -- sufficient momentum can overwhelm it before it can act, leaving the
robot stuck (wheels off the ground) rather than safely backed away. This directly changes the risk picture
for "rely on AVA's reflex alone" as a design: it is a real, working backstop at moderate speeds, but not a
guarantee at all speeds.

**Recommendation going forward:** any further supervised edge-testing with reduced/no stop-gates should
stay at speeds where AVA's recovery has been proven to work (<=0.4, validated repeatedly today), not at
maximum velocity. This was a one-off max-speed probe specifically requested to see the behavior at the
extreme -- now answered, and answered as "don't do this again without more caution."

No code changes from this entry -- purely a documented safety finding from live testing.

---

## 2026-07-12 — q6a_creep_test.py simplified: MiDaS slowing removed entirely, wheel-drop is a real hard stop

After the second "fighting AVA" incident (confirmed live even after fixing the pause-must-command-zero-
velocity bug), user's direct assessment: no improvement, because the underlying problem is MiDaS going
blind right at the boundary -- no ramp-parameter tuning fixes a genuine sensor blind spot. Decision:
remove the MiDaS-based ramp entirely rather than keep chasing it.

**Rewrote the script from scratch, much simpler:** constant commanded velocity (no ramp, no floor, no
cooldown logic). Wheel-drop (/cliff, AVA's own signal) is now the ONLY stop condition besides the time
bound and a stale-scan abort, and it IS a genuine hard stop again (raises SystemExit, ends the run) --
not the pause-and-resume behavior from the previous MiDaS-ramp design, since there's no ramp logic left to
resume into. This removes the whole class of "resumes full speed right at the edge" bug since the robot
now only ever drives at one constant speed regardless of proximity, with wheel-drop cutting it off cleanly
when it fires. /vision/floor subscription removed too (no longer used for anything).

Files: `scripts/companion/q6a_creep_test.py` (full rewrite, much smaller/simpler).

---

## 2026-07-12 — CRITICAL FIX: creep-test was fighting AVA's own wheel-drop recovery ("trying to suicide")

Live incident with wheel-drop fully removed: the robot approached the edge, AVA's OWN independent wheel-drop
detection kicked in and backed it away automatically -- but q6a_creep_test.py kept issuing forward move
commands every tick regardless of AVA's state, so it immediately pushed the robot toward the edge again the
moment AVA backed off. This repeated: approach -> AVA saves it -> our script undoes the save -> approach
again. User's own words: "it was looking like it was hitting the edge and drive back then hitting the edge
again and drive back again trying to suicide."

**This finally gives us the confirmation the 2026-07-12 log investigation couldn't find: AVA genuinely has
its own independent wheel-drop detection + automatic backward recovery**, completely outside our software.
But our script was actively undermining it by refusing to acknowledge AVA had taken protective action.

**Fixed:** q6a_creep_test.py now subscribes to /cliff again and PAUSES its own forward commands (does not
raise SystemExit, does not end the run) whenever AVA's wheel-drop is active, plus a 2s cooldown after it
clears, before resuming the MiDaS-ramped approach. This stops us from fighting AVA's recovery without
reintroducing a hard stop that ends the test outright -- matches the user's exact ask: "we should recognize
when ava stopped it and do not try to move forward again."

Also (separately, per feedback): raised speeds again (max 0.4, floor 0.15, up from 0.25/0.05) since both
felt too slow; user wants a real capped reduction, not a near-crawl -- floor should make actual progress
so AVA's own wheel-drop is what ultimately stops it, not our software running out of ground to cover.

Files: `scripts/companion/q6a_creep_test.py`.

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

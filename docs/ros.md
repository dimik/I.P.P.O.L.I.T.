# ROS 2 integration

How the robot's data reaches ROS 2, what's published, and how to consume it.

**Decided architecture (see below):** the **Q6A runs all of ROS**; the robot is a pure data-source
appliance exposing **HTTP/RTSP** (no ROS in the production path). ROS 2 **Jazzy** is also installed
in the robot's chroot (`/data/chroot`, `source /opt/ros/jazzy/setup.bash`) — currently used to run
the bridge there, which works but is the **fallback / test rig**, not production. We're ROS 2 (DDS,
no master); the robot has no vendor ROS (AVA is MIIO/serial), so there's one ROS graph and it lives
on the Q6A.

## What's live today: Valetudo → ROS bridge ✅

`scripts/robot/valetudo_bridge.py` (runs in the chroot) bridges Valetudo's HTTP API into ROS —
**no MQTT broker**. It seeds the map once via REST, then is **push-driven via SSE** (no polling).

| Topic | Type | QoS | Source | Rate |
|-------|------|-----|--------|------|
| `/map` | `nav_msgs/OccupancyGrid` | RELIABLE + **TRANSIENT_LOCAL** (latched) | Valetudo `/state/map` floor(0)/wall(100)/unknown(-1) | on map change |
| `/odom` | `nav_msgs/Odometry` | default (volatile) | `robot_position` entity (pose only; **no twist**) | 2 Hz heartbeat |
| TF `map`→`base_link` | tf2 | — | same pose | 2 Hz heartbeat |
| `/robot/status` | `std_msgs/String` | default | `StatusStateAttribute` (`value/flag`) | on attr change |

Plus **`/scan`** (`sensor_msgs/LaserScan`) from a *separate* pipeline — the robot's own 360° lidar via
`libserialtap.so` → tmpfs shm ring → `lds_scan_node.py`. Publishes only while the turret spins (active
nav); zero overhead when gated off. See "Tap raw ttyS3" below + `docs/sensors.md`.

**Design (why this shape):** Valetudo's map SSE is *push-on-change with no initial snapshot*, so a
docked/just-started robot would have no map until it moved → we GET the map once to seed, then ride
the SSE. `/odom`+TF are republished by a **2 Hz in-memory heartbeat** (cached pose, no HTTP) so they
stay fresh while idle — nav2 needs continuous TF. Net: zero idle HTTP traffic, instant updates while
cleaning, continuous odom/TF always.

**Coordinate convention:** Valetudo is **mm, +y DOWN** (image coords); ROS (REP-103) is **m, +y UP**.
So `x = x_mm/1000`, `y = -y_mm/1000`, `yaw = -angle_deg`. ⚠️ The yaw sign/offset is v1 — eyeball the
`base_link` heading arrow in RViz once and flip if needed.

**Run / control (on the robot):**
```sh
sh /data/valetudo_bridge.sh start|stop|status     # auto-starts at boot via _root_postboot.sh
```

**Consume (on the robot chroot):**
```sh
source /opt/ros/jazzy/setup.bash
ros2 topic echo /odom --once
ros2 topic echo /tf --once
# /map is LATCHED — the subscriber MUST match QoS or it sees nothing:
ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local --field info
```

## Decided architecture: robot = appliance, Q6A = all ROS

**The robot runs NO ROS in the production path.** It's a pure data-source appliance exposing
**HTTP/RTSP**; the Q6A runs the **entire** ROS 2 graph. The robot↔Q6A interface is plain HTTP/RTSP,
**not DDS** — so there's no cross-host DDS to configure, no master, no domain bridging.

```
Robot (appliance — no ROS)                 Q6A (ALL ROS, one DDS domain)
  AVA + Valetudo  (REST/SSE :80)  ──HTTP──►  valetudo_bridge.py --host http://192.168.10.1
  go2rtc          (RTSP :8554)    ──RTSP──►  camera→image node (optional)
  LD_PRELOAD shims + camstream               nav2, SLAM, vision, AI/LLM, planning
```

Why (not ROS 1 dual-masters, not on-robot ROS): we're **ROS 2 (DDS, no master)**, and the robot has
**no vendor ROS** — AVA speaks MIIO/serial, so there's nothing to bridge *between* two ROS graphs;
there's one graph (ours) and it should live on the Q6A. The Allwinner A7 is weak and `/data` eMMC
**wedged under write load** this session (sysrq reboot) → keep ROS/DDS off the robot. Running the
bridge on the robot (the current state) is the worst of both — ROS load on the robot AND still
needing cross-host DDS; relocating it to the Q6A removes both.

**Migration (when the Q6A is online):** `valetudo_bridge.py` is host-agnostic — run it on the Q6A
with `--host http://<robot-ip>`; same idea for a camera→image node off the go2rtc RTSP URL. Then
disable the robot-side bridge in `_root_postboot.sh`. The chroot-ROS bridge stays as a robot-local
test rig / fallback (not the production path).

## The raw `/scan` gap — how to proceed

Valetudo exposes the **processed map + pose, not raw laser ranges**, and AVA owns `/dev/ttyS3` (the
LDS) exclusively — so there's no clean `sensor_msgs/LaserScan` source. Decide by *why* you need it:

- **Map-based nav (localize + global plan)** → **don't need `/scan`.** nav2 works against `/map` +
  AVA's pose (published as `/odom`+TF). AVA already does the SLAM; we consume it. ← v1.
- **Dynamic obstacle avoidance** (things not in the map) → use the **camera** (already streamed via
  go2rtc) + the **Q6A NPU** as a vision obstacle layer. Leverages what we have; zero robot risk.
- **Real live 360° scan / own SLAM** → add a **dedicated USB lidar on the Q6A** (LD06/LD19/RPLiDAR,
  ~$70) → native LaserScan from its driver. Robust, standard; the robot's lidar stays AVA's.
- **Tap raw ttyS3 in AVA** → **DONE — `/scan` is LIVE** (2026-06-19). The earlier "read() breaks AVA"
  wall was a bug in *our* shim, not AVA: the freestanding `read()` returned raw `-errno` instead of
  glibc's `-1`+`errno`, so AVA's non-blocking loops choked. Fixed with `__errno_location`. The
  production path: `libserialtap.so` (LD_PRELOAD, passive errno-correct read-tap on ttyS3, isolates the
  fd via `/proc/self/fd`) tees into a tmpfs shm ring → `lds_scan_node.py` decodes the LDS frames →
  `sensor_msgs/LaserScan` on `/scan`. The robot's *own* 360° lidar, no extra hardware. Boot-persistent
  and verified across a reboot; zero overhead when the turret is gated off. See `docs/sensors.md` for
  the frame format and `scripts/robot/{serialtap.c,lds_scan_node.py}`.

**Recommendation:** ship v1 nav on `/map` + pose now; for live 360° sensing, the robot's own lidar
via the ttyS3 tap is now the preferred path (no extra hardware) — vs. a USB lidar on the Q6A as the
zero-AVA-risk fallback.

## Raw IMU + wheel-odom over ttyS4 — LIVE

`libserialtap.so` also tees AVA's MCU stream on `/dev/ttyS4` (`3c…3e` frames, Modbus-CRC) into
`/tmp/mcu_ring.buf`; **`scripts/robot/mcu_node.py`** decodes it and publishes:
- **`/imu/data`** (`sensor_msgs/Imu`) — BMI-class gyro + accel (`imu_type=2`). Orientation left unknown
  (no magnetometer; covariance[0]=-1); angular_velocity + linear_acceleration for EKF fusion. Gyro
  bias auto-calibrated at startup (assumes still/docked). ~35 Hz idle (MCU throttles Status10ms).
- **`/odom/wheel`** (`nav_msgs/Odometry`) — raw wheel dead-reckoning (separate from the bridge's
  SLAM `/odom`; for `robot_localization` EKF, not direct nav).

**Scalings are D10s-MEASURED, not Z10-borrowed** (the Z10 firmware pre-scaled; the D10s sends raw
LSB): accel `/16384` (±2 g — confirmed by gravity = 9.807 m/s² at rest), gyro `/65.536` (±500°/s —
confirmed by two spin cross-checks of `gyro_z` integral vs wheel-odom yaw: 0.01526/0.01527 °/s·LSB,
both directions; see `scripts/robot/spin_calib.py`). Both are node params, re-tunable without rebuild.
Axis/sign vs `base_link` is a v1 passthrough — verify in RViz. `mcutap.c` is superseded by serialtap.

## Roadmap

- [x] `/map`, `/odom`, TF, `/robot/status` from Valetudo (broker-free, SSE-driven)
- [ ] **relocate the bridge to the Q6A** (`--host http://<robot-ip>`), then disable the robot-side
      one — no cross-host DDS needed (robot↔Q6A is HTTP/RTSP)
- [ ] nav2 bringup on the Q6A consuming `/map` + TF; goals → Valetudo REST move commands
- [ ] camera into ROS (the go2rtc RTSP/WebRTC feed → an `image` topic if wanted)
- [ ] live obstacle sensing on the Q6A when needed: camera+NPU layer, or a dedicated USB lidar
- [x] raw `/scan` via ttyS3 read-tap (`libserialtap.so` → shm ring → `lds_scan_node.py`) — **LIVE**,
      boot-persistent, the robot's own 360° lidar
- [x] `/imu/data` + `/odom/wheel` via the ttyS4 tap (`libserialtap.so` → shm ring → `mcu_node.py`) —
      **LIVE**, boot-persistent; scalings D10s-measured (accel /16384, gyro /65.536)
- [ ] `robot_localization` EKF on the Q6A fusing `/odom/wheel` + `/imu/data` (and verify IMU axis
      alignment vs `base_link` in RViz)

## Gotchas

- `/map` is latched (TRANSIENT_LOCAL) — `ros2 topic echo` shows nothing unless you pass
  `--qos-durability transient_local --qos-reliability reliable`.
- The bridge is a chroot process; launch it `setsid`-detached (the launcher handles this) or it dies
  when the ssh session closes.
- It needs Valetudo's HTTP API up; at boot it retries until Valetudo answers.

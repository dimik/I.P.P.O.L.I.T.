# CLAUDE.md — dreame-cortex project knowledge

## Project goal

Turn a Dreame D10s Pro robot vacuum into an open AI platform:
- Cloud-free operation via Valetudo
- AI companion board (Radxa Dragon Q6A) for vision, navigation, audio
- ROS 2 Jazzy for all AI/autonomy — runs entirely on the companion board
- Robot hardware (motors, LiDAR, SLAM) stays controlled by the proprietary AVA daemon

---

## Hardware

### Robot — Dreame D10s Pro (model r2250)
- SoC: AllWinner MR813, quad-core Cortex-A7 @ 1.2GHz, aarch64
- WiFi: Realtek 8189fs — **2.4GHz only, single radio** (cannot do AP + STA on different channels simultaneously)
- LiDAR (LDS turret): on `/dev/ttyS3` @ 230400 — read by AVA; tapped read-only via `libldstap.so` →
  `/scan` (LDS protocol fully decoded — see `docs/sensors.md`). MCU (motors/IMU/odom) is on `/dev/ttyS4`.
- Camera: OV8856 MIPI sensor (`/dev/video0`,`/dev/video2`, multi-plane sunxi-vin). AVA owns it; live frames are siphoned read-only via `camsiphon.so` — see `docs/sensors.md`.
- Speaker: SUNXI-CODEC ALSA device — `hw:0,0`, playback via `aplay`
- Root filesystem: **squashfs (physically read-only)** — cannot be remounted RW
- Writable partition: `/data/` (ext4, ~3.3GB total, ~2GB free after setup)
- SSH access: `root@192.168.1.213` (home WiFi), `root@192.168.5.1` (robot AP mode)
- SSH key: `~/.ssh/id_rsa_dreame`

### Companion — Radxa Dragon Q6A
- SoC: Qualcomm QCS6490
- CPU: 1× Kryo Prime @ 2.7GHz + 3× Gold @ 2.4GHz + 4× Silver @ 1.9GHz
- NPU: **12 TOPS** (Hexagon DSP, 6th-gen Qualcomm AI Engine)
- GPU: Adreno 643L (OpenCL, Vulkan)
- RAM: up to 16GB LPDDR5
- Connectivity: GbE, WiFi 6, BT 5.4, USB 3.1 + 3× USB 2.0, 3× MIPI CSI, 40-pin GPIO
- Power: 12V, 18–30W (powered from robot 14.8V battery via 12V buck converter)
- OS: Ubuntu 24.04 (Armbian) — ROS 2 Jazzy Tier 1 native
- Size: 85×56mm — fits in D10s Pro top compartment

### Physical link
Robot USB 2.0 host → USB-Ethernet adapter → Cat5e → Dragon Q6A GbE
- Robot static IP on link: `192.168.10.1`
- Dragon Q6A static IP on link: `192.168.10.2`

---

## Software stack

### On the robot (Dreame firmware)
- **AVA daemon**: closed-source binary, owns ALL hardware (motors, LiDAR SLAM, sensors, path planning)
- **`dummycloud` socket**: AVA's command/state interface, normally forwarded to Dreame cloud
- **Valetudo v2026.05.0**: intercepts `dummycloud` socket, exposes REST API (port 80) and MQTT (port 1883)
- **Init system**: BusyBox with `/etc/rc.sysinit` — two hook points in `/data/`

### Boot hooks (`/data/`)
| File | Timing | Role |
|------|--------|------|
| `_root.sh` | Early — before `wifi.sh` | Bind-mount wpa_supplicant config |
| `_root_postboot.sh` | Late — after all services | DHCP, chroot mounts, start Valetudo |

### WiFi (critical — read before touching)

**Home network topology:**
- Laptop connects to `5K` (5GHz, 802.11ac) — Claude/SSH sessions run from here
- Robot connects to `4K` (2.4GHz) — robot WiFi is 2.4GHz only
- These are the same router, same password, different bands/SSIDs
- SSH to robot at home: `root@192.168.1.213` (via 4K DHCP)
- SSH to robot in AP mode: `root@192.168.5.1`

**wpa_supplicant config bind-mount:**
The init script `wpa_supplicant.sh` checks for `/usr/bin/wifi_manager`. When it exists (it does on r2250), it reads from `/etc/wifi/wpa_supplicant.conf` — a **read-only squashfs file with no network entries** — instead of `/data/config/wifi/wpa_supplicant.conf`.

Fix: `_root.sh` writes our config to `/data/config/wifi/wpa_supplicant.conf` then does:
```sh
mount --bind /data/config/wifi/wpa_supplicant.conf /etc/wifi/wpa_supplicant.conf
```
This must run **before** `wpa_supplicant.sh`. See `docs/wifi-hack.md` for full explanation.

**CRITICAL — `/etc/miio` is a symlink:**
`/etc/miio` in squashfs is a **symlink → `/data/config/miio/`**. This means:
- Files created in `/data/config/miio/` are immediately visible at `/etc/miio/`
- No bind-mount of `/etc/miio/` is needed or useful — the symlink already provides write access
- `mount --bind /data/config/miio /etc/miio` resolves the symlink and mounts `/data/config/miio` over itself (no-op in practice)
- `miio_client_helper_nomqtt.sh` checks `[ -f /etc/miio/wifi.conf ]` (via `WIFI_CONF_FILE=/etc/miio/wifi.conf`) to decide if the device is provisioned

**Deployment — e2e script requirement:**
Connecting the laptop to the robot AP (`dreame-vacuum-r2250_miap8E6A`) drops the internet connection. All deploy scripts must be completely self-contained background scripts that: connect to AP → do work → reconnect to home WiFi (`5K`). Never try to SSH interactively while switching networks. See `/tmp/robot-deploy.sh` for the pattern.

### Ubuntu 24.04 chroot (`/data/chroot/`)
- Full Ubuntu 24.04.4 arm64 base rootfs
- ROS 2 Jazzy installed (but NOT used — Dragon Q6A handles all ROS)
- Enter: `ssh dreame-home` then `sh /data/chroot.sh`
- apt fix required: `APT::Sandbox::User "root"` in `/etc/apt/apt.conf.d/00no-sandbox`
  (apt's `_apt` sandbox user cannot do DNS on kernel 4.9.191)

### Valetudo config
- Binary: `/data/valetudo` (37MB, aarch64)
- Config: `/data/valetudo_config/valetudo.json`
- Web UI: `http://192.168.1.213`
- Robot implementation: `DreameD10SProValetudoRobot` (auto-detected)
- MQTT: disabled by default — enable in valetudo.json for Dragon Q6A bridge

---

## Architecture

```
Dreame D10s Pro                              Radxa Dragon Q6A
────────────────────────────────             ──────────────────────────────────
AVA  ──► LiDAR / motors / SLAM
Valetudo REST (port 80)  ◄────────── REST ────── navigation commands
Valetudo REST+SSE (port 80) ─ REST/SSE ───────►  valetudo_bridge.py (chroot ROS, broker-free)
                                                    └─► /odom /map /robot/status
AVA /dev/ttyS3 ──► libldstap.so (LD_PRELOAD read-tap) ──► /tmp/lds_ring.buf ──► lds_scan_node.py
                                                                              └─► /scan  ✅

AVA /dev/video2 ──► camsiphon.so (LD_PRELOAD, read-only DQBUF tap) ──► NV21 frames
   ├─► one-shot:  /tmp/cam_grab  → /tmp/cam_frame.raw  (PNG via nv21_to_png.py)
   └─► stream:    /tmp/cam_stream → RAM ring /tmp/cam_stream.buf ──► camstream (cedar HW-JPEG)
                                                              ──► MJPEG over HTTP :8090  ✅
   (the robot's OV8856 is AVA-owned; we siphon the raw frames AVA already captures — see docs/sensors.md)

/dev/snd (ALSA) ◄──────────────── TCP socket ───── audio_server.py
audio_server.py (port 9999)                          └─ TTS WAV data

Ubuntu chroot (idle, available)             inference_node (YOLOv8, NPU)
                                            nav2_stack
                                            behavior_node
                                            tts_node (Piper)
```

### Data flows

| Stream | Source | Protocol | Destination |
|--------|--------|----------|-------------|
| Camera | `/dev/video0` V4L2 | GStreamer UDP H.264 | Dragon Q6A camera_node |
| Map | Valetudo REST seed + **map SSE** | `valetudo_bridge.py` (chroot ROS) | `/map` OccupancyGrid (latched) ✅ |
| Robot pose | Valetudo SSE `robot_position` | `valetudo_bridge.py` (2 Hz heartbeat) | `/odom` + `map→base_link` TF ✅ |
| Status | Valetudo attributes SSE | `valetudo_bridge.py` | `/robot/status` ✅ |
| **LiDAR** | `/dev/ttyS3` LDS read-tap | `libldstap.so` → shm ring → `lds_scan_node.py` | `/scan` LaserScan (robot's own 360° lidar) ✅ |

(Bridge is broker-free + SSE-driven, no polling — full details, QoS, frames, cross-host DDS plan in `docs/ros.md`.)
| Nav commands | Dragon Q6A Nav2 | Valetudo REST API | AVA motors via Valetudo |
| Audio | Dragon Q6A TTS | TCP socket (WAV) | aplay on robot speaker |

---

## Key constraints and gotchas

1. **WiFi is 2.4GHz only** — robot connects to `4K` SSID (2.4GHz). The `5K` SSID (5GHz) won't be seen by the robot.
2. **Single radio** — robot AP (hostapd on wlan0) and STA mode (wpa_supplicant) cannot run simultaneously on different channels. Our fix: kill hostapd at boot, use wlan0 as STA only.
3. **BusyBox wget cannot follow HTTPS redirects** — always use `curl -L` on the robot, or download on laptop and `scp`
4. **Kernel 4.9.191** — Ubuntu 24.04 glibc 2.39 mostly works, but apt's sandbox user (`_apt`) cannot do DNS — requires `APT::Sandbox::User "root"` workaround
5. **AVA owns the sensor serial ports exclusively** — `/dev/ttyS4` (MCU: motors/IMU/odom, `3c…3e`) and `/dev/ttyS3` (LiDAR LDS, `55 aa 03 08`). You can't open them yourself. Instead, **passive LD_PRELOAD read-taps** copy the bytes AVA already read: `libldstap.so` tees ttyS3 → `/scan` (live). The key to AVA-safety is the **errno contract** — a freestanding `read()` MUST return `-1`+errno, not raw `-errno`, or AVA's non-blocking loops choke (this broke the first tap, `mcutap`). See `docs/sensors.md` / `docs/ros.md`. Processed map/pose still come via Valetudo's REST/SSE bridge.
6. **squashfs root** — any change to system files requires either a bind mount from `/data/` or a chroot. Never attempt `mount -o remount,rw /`. `/etc/miio` is a SYMLINK to `/data/config/miio/` (so it's already writable via /data/).
7. **exec_monitor.sh** watches only the `ava` process — it does NOT restart hostapd if killed. Safe to kill hostapd permanently.
8. **firmware 1413 IoT flag** — AVA does NOT connect to `miio_agent` (TCP 54320) at boot unless `/data/config/ava/iot.flag` contains `miiot`. This flag is normally written during cloud provisioning (which we bypass via Valetudo). Without it, ALL Valetudo MIIO property/action commands time out. Fix: `_root_postboot.sh` writes `miiot` to the flag and calls `avacmd iot '{"type":"iot","notify":"open_server"}'` at boot.
9. **work_mode 17 persists at boot** — AVA enters work_mode 17 (RemoteCtrlMode) on every boot via the MIIO provisioning flow. (Note: the HTTP 400s we hit on manual control were a wrong REST payload — `{"operation":...}` instead of `{"action":...}` — not work_mode.) See `### work_mode 17 — root cause and investigation` below.
10. **_root.sh has hardcoded WiFi credentials** — the `4K` SSID PSK is hardcoded. Deploying from repo (which may have placeholder `YOUR_SSID`/`YOUR_HEX_PSK`) will break WiFi. Always verify credentials before deploy.
11. **Bind-mount a FILE over a non-existent squashfs path = boot failure** — if you try `mount --bind /data/file /etc/miio/nonexistent` where `nonexistent` doesn't exist in squashfs, the bind-mount silently fails but breaks init. Always bind-mount directories, or ensure the target file exists in squashfs first.

---

## Robot internals and configuration

### AVA daemon internals

AVA (`/ava/bin/ava`) is a closed-source C++ binary running as PID 1-level process on the robot. It owns all hardware:
- Motors (wheels, side brush, main brush, fan/vacuum, mop pump) + IMU/odom via the MCU on `/dev/ttyS4` (fd 24; `3c…3e`-framed protocol with Modbus CRC16 — see the MCU serial section)
- LiDAR (LDS turret) via `/dev/ttyS3` (fd 26; `55 aa 03 08` frames — see `docs/sensors.md`)
- SLAM, path planning, behavior tree

AVA is structured as a behavior tree with named nodes. Nodes communicate via an **in-process** pub/sub bus (`ava::Publisher<…>`; e.g. `LaserWitPose`) and expose a command interface through `/tmp/avacmd.socket`. Recon (2026-06-19) confirmed AVA exposes **no external IPC for raw sensor data** (`/dev/shm` empty; only `avacmd`/`avaexec`/`videomonitor` unix sockets), and the IMU has **no kernel `iio`/`input` driver** — it's MCU-serial only. Hence raw `/scan` + IMU come from passive read-taps, not an IPC subscription.

**Key AVA file descriptors:**
| fd | Device | Purpose |
|----|--------|---------|
| 24 | `/dev/ttyS4` | MCU serial — motors, fan, brushes, IMU/sensors (`3c…3e` protocol) |
| 26 | `/dev/ttyS3` | LiDAR LDS @ 230400 (`55 aa 03 08` frames); read-only, tapped by `libldstap.so` → `/scan`. Reads only while the turret spins (active nav). |

**Key AVA config files (on `/data/`, writable):**
| File | Purpose |
|------|---------|
| `/data/config/ava/clean_parameter.json` | Persistent cleaning settings, read at boot by porphyrion (clb) node |
| `/data/config/ava/iot.flag` | Must contain `miiot` for AVA to connect to miio_agent |
| `/data/config/ava/iot_conf.json` | IoT silent mode: `{"EnableSilent":0,"BaseUrl":""}` |
| `/data/Robot_Para.yaml` | Motor calibration offsets (not on/off control) |
| `/ava/conf/setting.yaml` | Behavior tree config (read-only squashfs; bind-mount from /data/ to override) |
| `/ava/conf/r2250.conf` | AVA node configuration; `max_average_speed:300` for porphyrion |

### avacmd — AVA node command interface

`avacmd <node_name> '<json>'` sends a command to a named AVA node via `/tmp/avacmd.socket`. All responses are JSON.

**Known nodes and their commands:**

| Node | avacmd name | Useful commands |
|------|-------------|----------------|
| msg_cvt (MIIO translator) | `msg_cvt` | `{"type":"msgCvt","cmd":"get_prop","prop":"work_mode"}` → `{"value":"N","ret":"ok"}` |
| msg_cvt | `msg_cvt` | `{"type":"msgCvt","cmd":"get_prop","prop":"clean_mode"}` → `{"value":"0","ret":"ok"}` (read-only via avacmd) |
| msg_cvt | `msg_cvt` | `{"type":"msgCvt","cmd":"status_idle"}` → `{"ret":"ok"}` |
| IoT | `iot` | `{"type":"iot","notify":"open_server"}` — connect AVA to miio_agent (needed after boot on fw 1413) |
| porphyrion (BT node) | `clb` | Network connect mode: `{"type":"clb","cmd":"report_network_connect_mode","mode":N}` |

**`{}` means unrecognized command**, not success. `{"ret":"ok"}` means recognized and executed.

**Important limitations**: `avacmd msg_cvt set_prop clean_mode 1` and all variant formats return `{}` — msg_cvt only exposes get_prop for cleaning properties, not set_prop. Direct avacmd cannot change the clean mode at runtime.

**avacmd log**: `/tmp/log/ava_cmd.log` — real-time log of all avacmd calls and responses.

### MIIO protocol stack

```
Valetudo (UDP 8053) ↔ miio_client (UDP 54321, TCP 54322/54323) ↔ miio_agent (TCP 54320) ↔ AVA
```

- **miio_client TCP 54323**: provisioning helper channel — `miio_send_line` writes here. NOT for device property commands. Only handles `_internal.*` MIIO methods.
- **miio_client UDP 54321**: receives commands from Valetudo, forwards to miio_agent.
- **miio_agent TCP 54320**: AVA connects here to receive MIIO commands from cloud/Valetudo.
- Without `iot.flag = miiot`, AVA never connects to miio_agent → all commands time out.

**MIIO property mappings (siid:piid):**

| MIIO property | siid | piid | Values |
|---------------|------|------|--------|
| work_mode | 2 | 4 | see work mode table below |
| fan_speed | 4 | 4 | 0=off, 1=low, 2=medium, 3=high, 4=max (Valetudo "low"→0, "medium"→1, "high"→2, "max"→3) |
| clean_mode | 4 | 7 | 0=sweep (fan ON), 1=mop-only (fan OFF), 2=sweep+mop |
| water_grade | ? | ? | low/medium/high |

**Work modes (`work_mode` from avacmd):** — see `### work_mode 17` section for full table and investigation

### work_mode 17 — root cause and investigation

**Symptom**: On every boot, AVA immediately enters work_mode 17 (RemoteCtrlMode). Most Valetudo REST capabilities return HTTP 400 in this state. NOTE: the manual-control 400s were a wrong REST payload (`{"operation":...}` vs the correct `{"action":...}`), NOT work_mode.

**MIIO provisioning flow (root cause):**
```
miio_client starts
  → sends "_internal.request_dinfo" to miio_client_helper_nomqtt.sh
  → helper reads /etc/miio/wifi.conf (which is /data/config/miio/wifi.conf via symlink)
  → if file MISSING: sends params:0 (not provisioned)
     → miio_client: STATE_OT_CONFIG_DONE → STATE_WIFI_AP_MODE
     → opens WiFi AP for provisioning (30 min countdown)
     → triggers work_mode 17 in AVA ("app can drive robot to dock")
  → if file EXISTS: sends params:1 (provisioned) → STA mode → no work_mode 17
```

**Key files:**
- `WIFI_CONF_FILE=/etc/miio/wifi.conf` (symlink → `/data/config/miio/wifi.conf`)
- `/usr/bin/config`: defines WIFI_CONF_FILE (falls back to /etc/miio/wifi.conf if unset)
- `/usr/bin/miio_client_helper_nomqtt.sh`: `get_bind_status()` checks `[ -f $WIFI_CONF_FILE ]`

**Current fix in `_root.sh`:**
```sh
[ ! -f /data/config/miio/wifi.conf ] && printf 'ssid="configured"\nkey_mgmt=WPA\n' > /data/config/miio/wifi.conf
```
This creates wifi.conf before miio_client starts (at t=3s; miio_client starts at t=9s).

**Current status (UNRESOLVED):** Even with wifi.conf created and confirmed accessible (`test -f /etc/miio/wifi.conf` returns YES), the MIIO helper still sends params:0 and miio_client enters AP mode → work_mode 17. User.log shows this at t=9-11s boot time:
```
STATE: [STATE_OT_CONFIG_DONE] -> [STATE_WIFI_AP_MODE]
wifi enter AP mode
ap will close in 1799s  (←30 min countdown)
```

**Investigation attempts (all failed to prevent work_mode 17):**
- Created `/data/config/miio/wifi.conf` with dummy content → file exists but params:0 still sent
- Directory bind-mount `/data/config/miio` → `/etc/miio` → no-op (symlink resolves to same path)
- `avacmd msg_cvt '{"type":"msgCvt","cmd":"status_idle"}'` → `{"ret":"fail"}` in work_mode 17
- `curl PUT …/HighResolutionManualControlCapability {"operation":"disable"}` → HTTP 400 (wrong schema; correct is `{"action":"disable"}`)
- `avacmd clb {"cmd":"report_network_connect_mode","mode":0}` → `{}` (not recognized)
- Valetudo `{"command":"home"}` → HTTP 400 (Bad Request) in work_mode 17

**WRONG approach that broke the robot:**
The first fix attempt tried to bind-mount a FILE over a non-existent path:
```sh
mount --bind /data/config/miio/wifi.conf /etc/miio/wifi.conf  # WRONG!
```
`/etc/miio/wifi.conf` doesn't exist in squashfs. Bind-mounting a file over a non-existent target caused the robot to fail to boot (couldn't connect to home WiFi). Recovery: connect to robot AP mode at 192.168.5.1, deploy fixed script.

**Correct bind-mount rule:** Only bind-mount OVER PATHS THAT EXIST in squashfs. For files, the target must already exist. For directories, the target must exist. Never create new squashfs paths via bind-mount.

**Open questions:**
- Why does the helper return params:0 even when wifi.conf exists? May be a shell environment issue, timing, or the `source /usr/bin/config` command failing silently.
- Does the Dreame firmware have a state machine that ALWAYS starts in work_mode 17 and transitions to 6 only after cloud confirm? If so, with Valetudo (no cloud), it stays at 17 forever.

**Work-around while unresolved:**
`FanSpeedControlCapability "low"` (MIIO siid:4 piid:4=0) set at boot via Valetudo. AVA uses stored fan_speed=0 when entering work_mode 17, keeping fan off. See `### Permanent vacuum fan disable` below.

**Work modes (`work_mode` from avacmd):**
| Value | Meaning |
|-------|---------|
| 6 | Docked, idle (target state) |
| 9 | Intermediate provisioning state (seen during AP mode before work_mode 17 kicks in) |
| 13 | Intermediate state during HighResolutionManualControlCapability enable transition (17→13→17) |
| 14 | Dock activity (auto-empty, self-cleaning, etc.) |
| 17 | Remote control / provisioning mode — entered during Dreame WiFi provisioning flow OR HighResolutionManualControlCapability enable |



`/data/config/ava/clean_parameter.json` — AVA's porphyrion (clb) node reads this at boot to initialize the behavior tree blackboard. AVA continuously overwrites this file during operation (mirroring its in-memory state).

**Key fields:**

| Field | Values | Effect |
|-------|--------|--------|
| `CleanMode` | 0=sweep, 1=mop-only, 2=sweep+mop | boot cleaning mode; does NOT gate the fan in manual/remote mode (the MCU SetCleaning `f3` does) |
| `CleanMop` | 0/1 | Whether mop pad is installed |
| `CleanCarPetPress` | 0/1 | Carpet boost |
| `StreamerSwitch` | 0/3 | Streamline feature |
| `CarpetPressState` | 0/1/2 | Carpet mode sensitivity (low/med/high) |
| `SwitchSet[CleanType]` | 0/1 | Clean type (linked to CleanMode) |

**Default (sweep mode, fan ON):**
```json
{"CleanMode":0,"CleanMop":1,"CleanBreakPonitStart":0,"CleanCarPetPress":0,"CleanWashMopTime":0,"StreamerSwitch":3,"MopSwitch":0,"CustomeSwitch":0,"ChildLock":0,"CarpetPressState":2,"MopMode":0,"UploadMap":1,"YmodeSwitch":0,"MultiMapReloc":0,"SwitchSet":[{"k":"AutoDry","v":1},{"k":"CleanType","v":0},{"k":"FillinLight","v":1},{"k":"LessColl","v":1},{"k":"MopScalable","v":1},{"k":"StainIdentify","v":1}]}
```

**Mop-only (fan OFF permanently):**
```json
{"CleanMode":1,"CleanMop":1,"CleanBreakPonitStart":0,"CleanCarPetPress":0,"CleanWashMopTime":0,"StreamerSwitch":3,"MopSwitch":0,"CustomeSwitch":0,"ChildLock":0,"CarpetPressState":2,"MopMode":0,"UploadMap":1,"YmodeSwitch":0,"MultiMapReloc":0,"SwitchSet":[{"k":"AutoDry","v":1},{"k":"CleanType","v":1},{"k":"FillinLight","v":1},{"k":"LessColl","v":1},{"k":"MopScalable","v":1},{"k":"StainIdentify","v":1}]}
```

### AVA debug logs

Key log files that are NOT user.log (which is from previous boots and goes silent on current boot because miio_client logs to /dev/null):

| Log file | Updated | Contents |
|----------|---------|----------|
| `/tmp/log/log_0` | Current boot | AVA's WritePropInt/WritePropString internal property writes, camera AI, LiDAR |
| `/tmp/log/ava_cmd.log` | Current boot | All avacmd invocations and responses |
| `/tmp/log/trace_sync.log` | Current boot | AVA behavior tree pub/cond trace (low-level) |
| `/data/log/msg_cvt.log` | Persistent | All `msg_cvt.sh` invocations (set_device_time, poweroff, iot_state) |
| `/data/log/wifi.log` | Current boot | WiFi connection events |
| `/data/log/fds.log` | Current boot | FDS (firmware download) events |
| `/tmp/postboot.log` | Current boot | Our `_root_postboot.sh` run log |
| `/tmp/valetudo.log` | Current boot | Valetudo process log |

**`miio_client` (PID ~1601) logs to `/dev/null`** on current firmware — all stdout/stderr discarded. user.log from previous boots persists but is stale.

**`/tmp/log/log_0` — WritePropInt/WritePropString format:**
```
[WritePropInt|140] type=N, value=V, from=0, sync=0  ← sets integer property type=N to value V
[WritePropString:161] type=0, len=L, value:P,{json}  ← sets property piid=P (string) to json
```

**Boot sequence (from log_0):**
1. AVA reads `clean_parameter.json` → WritePropInt loads: `type=0 value=1` (CleanMode=1 mop-only), `type=1 value=1` (CleanMop), `type=17 value=2` (CarpetPressState), `type=23 value=3` (StreamerSwitch) etc.
2. Camera AI + LiDAR init → hundreds of `[CL2] Kernel` lines (OpenCL GPU kernels)
3. Valetudo connects (dummycloud at t≈22s after boot) → AVA receives piid:13 enable:
   - `WritePropInt type=0 value=0` → CleanMode set to 0 (sweep)
   - `WritePropString piid:13 {"spdv":0,"spdw":0,"audio":"true","random":N}` → RemoteCtrlMode entered
4. Periodic keepalive piid:13 writes with `audio:false`

Note: `CleanMode` (`WritePropInt type=0`) is a property-store/behavior-tree value; it does NOT gate the vacuum fan in manual/remote mode. The fan is driven by the MCU `SetCleaning` packet — see the "Vacuum fan + LiDAR quieting" section. (Earlier theories that holding CleanMode=1 or `only_mop`=1 would stop the fan were disproven.)

---

### Vacuum fan + LiDAR quieting (SOLVED — MCU command filter)

**Goal**: drive the robot in manual navigation (`HighResolutionManualControlCapability`) silently — no vacuum fan and no spinning LiDAR turret — while keeping the LiDAR available in every other mode.

**Final design (deployed & verified 2026-06-15):** an `LD_PRELOAD` shim on AVA (`fanoff_shim.c`) rewrites two MCU command types on `/dev/ttyS4`:
- **Vacuum fan — OFF in every mode (unconditional).** `SetCleaning` (type `0x01`) payloads are rewritten to the docked-idle pattern `00 01 00 00 00`, so fan/brush/pump never spin. No flag, no daemon, no race on the loud motor.
- **LiDAR turret — off ONLY in manual nav.** `_CtrlMcuCMD` (type `0x14`) subcmd `0x04` (LDS motor: `01`=spin, `00`=park) is forced to `00`, but only while the LiDAR is *blocked*. The shim blocks the LiDAR BY DEFAULT and allows it while `/tmp/lidar_allow` exists. The gate daemon (`fanoff_flag.sh`) creates that flag in active non-manual modes and removes it for manual_control/idle/docked — so the turret keeps running for mapping/go-to/etc. and is parked only during manual driving.

**Why this shape (race-free).** An earlier "filter only when status==manual_control" flag let AVA's first fan-on `SetCleaning` through before a 1 Hz poller set the flag → the fan blipped at full power for ~1s. Making the fan unconditional removes that. Making the LiDAR blocked-by-default means manual_control AND idle are both "blocked", so entering manual nav never transitions allowed→blocked mid-session → no turret spin-up blip either.

**Fan byte detail.** `SetCleaning` payload `f1..f5`: idle=`00 01 00 00 00`; active=`55 58 03 00 00` (low/med) / `55 58 05 00 00` (max). `f1`/`f2` are the base fan/brush/pump power; `f3` is the fan boost tier (a Valetudo fan-preset sweep changed only `f3`). Zeroing only `f3` left the fan at base speed, so the shim forces the whole payload to idle. (Killing only the vacuum while keeping brushes would need the exact fan byte among `f1`/`f2` via `mcu.bin` RE — unnecessary here since the fan is off globally.)

**LiDAR gate is event-driven.** `fanoff_flag.sh` holds ONE Valetudo SSE stream (`GET /api/v2/robot/state/attributes/sse`) and reacts to pushed `StateAttributesUpdated` events — no polling / no `sleep`. It sets `/tmp/lidar_allow` for active statuses (cleaning/returning/moving/…) and removes it for manual_control/idle/docked/paused/error. Manual override: `touch /tmp/lidar_allow` to force the turret on; stop the daemon + leave the flag absent to keep it always off.

**Manual-nav REST payload** is `{"action":"enable"}` / `{"action":"disable"}` (the `{"operation":...}` form returns HTTP 400 — the real cause of earlier "rejected" results, not work_mode). Move: `{"action":"move","vector":{"velocity":0..1,"angle":N}}`.

**Voice prompt ("Start remote controlled cleaning").** Entering manual nav makes AVA play that voice prompt. Silenced by muting the speaker: `PUT SpeakerVolumeControlCapability {"action":"set_volume","value":0}` (mutes ALL prompts; for a rover that's fine). Set live 2026-06-15. Dreame normally persists volume across reboot — if a reboot resets it, add a one-shot `set_volume 0` after Valetudo is up in `_root_postboot.sh`. Hardware fallback if ever needed: `amixer` is at `/bin/amixer` on the host. (Targeting only that one prompt would mean swapping its WAV — not worth it.)

**Verified on the wire.** Manual nav → `SetCleaning` `00 01 00 00 00` (fan off) + ttyS3 LDS reads ~0 (turret parked) + `_CtrlMcuCMD 04 00`, with no fan/LiDAR start-up blip. With `/tmp/lidar_allow` present the LiDAR spins (~1700 reads/3s) while `SetCleaning` stays `00 01` — proving the fan is unconditional and the LiDAR is gated. MotorCtrl (driving) flows throughout; AVA healthy.

**Deploy / persistence.**
- `deploy_ava_shims.sh` builds a patched `ava.sh` (`export LD_PRELOAD="<shim list>"`), bind-mounts it over `/etc/rc.d/ava.sh`, restarts AVA. The LD_PRELOAD list (fanoff filter + camsiphon if present) is the shared injection MECHANISM; each shim is an independent feature.
- `_root.sh` re-establishes that bind-mount early at boot; `_root_postboot.sh` launches the SSE gate daemon after Valetudo starts. Both persist across reboot.
- Build with `build_ava_shims.sh` (freestanding, glibc-2.23-safe; builds fanoff + camsiphon). RE artifacts: `~/dreame-re/{mcu.bin,node_signal.so}`; protocol ref `~/dreame_mcu_protocol` (alufers).

**Architecture (table-driven).** `fanoff_shim.c` is layered: raw syscalls → Modbus CRC16 → frame codec (3c..3e + `?` escaping) → **policy `RULES[]` table** → write/writev hooks. Each subsystem is ONE declarative rule — match `type` (+ optional first payload byte), a rewrite `action` (`REWRITE_SETCLEANING_IDLE` / `REWRITE_ZERO_BYTE`), and a `gate` (`GATE_ALWAYS` or `GATE_UNLESS_FLAG <path>`). To disable another subsystem later, add a rule — nothing else changes. (Do NOT gate the IMU — AVA needs it to drive.) The const table relocates correctly under the `-nostdlib` build (verified: AVA loads + runs).

**Language choice (decided 2026-06-15).** The shim must be a native C-ABI `.so` LD_PRELOAD-ed into AVA (hooks libc `write`/`writev`, runs in AVA's glibc-2.23 address space on every MCU write) — so **C is the only practical fit**, and freestanding C neatly dodges the 2.23-vs-2.39 glibc mismatch with zero deps. **Python** can't be interposed in-process (would require an external pty proxy we rejected; no host Python anyway). **Nim→C** could emit a `.so` but you'd have to strip its runtime/GC to go freestanding and add a cross-toolchain — more complexity for a ~200-line file. The **`dreame_mcu_protocol` repo is Python for *offline* sniffing/decoding** (parses strace over SSH) — it can't run inside AVA; we already ported its CRC16 + packet defs into the shim and keep it as a decode/reference tool. The **gate** stays POSIX `sh` (no deps, runs on BusyBox host). Reserve Python/Rust for **Q6A companion** software (ROS/nav/vision), which isn't bound by AVA's runtime.

**Implementation kit** (`scripts/robot/`): `fanoff_shim.c` (table-driven filter), `fanoff_flag.sh` (SSE LiDAR gate), `build_ava_shims.sh` (builds all AVA preload shims), `deploy_ava_shims.sh` (live install of the shim list), `capture_cleanset.sh`.

**Dead ends — do NOT retry** (none gate the fan in manual/remote mode): `clean_parameter.json` `CleanMode`, an `only_mop` heap patch, ptrace-patching `node_porphyrion.so`, the `FanSpeedControlCapability` boot loop. Removed from the boot path and repo.

### Camera video stream (cedar HW encoder) ✅ MJPEG live

Local, cloud-free live video from the AVA-owned OV8856. Pipeline:
`AVA /dev/video2 → camsiphon (read-only DQBUF tap) → RAM ring /tmp/cam_stream.buf (double-buffered, tmpfs) → camstream (cedar HW-JPEG) → multipart/x-mixed-replace HTTP :8090`.

- **Auto-starts at boot** (`_root_postboot.sh`) and is **client-gated**: camstream idles in `accept()` with the ring OFF (zero AVA overhead); on each HTTP client it sets `/tmp/cam_stream` (camsiphon starts filling) and clears it on disconnect. So it's always reachable but costs nothing when nobody's watching. View at `http://<robot-ip>:8090/` (browser / VLC / `ffplay`). Manual control: `sh /data/camstream.sh start|stop|status`. Verified ~14 fps, NV21 672×504 → JPEG ~28 KB/frame. Read-only — AVA/camera/ISP untouched.
- **camsiphon stream egress**: gated by `/tmp/cam_stream` (set by camstream only while a viewer is connected); copies every frame into a RAM ring (no flash writes, no AVA stall — writes the inactive slot then flips `latest`). Zero overhead when the flag is absent. (One-shot grab via `/tmp/cam_grab` still works independently.)
- **Gotchas baked into the scripts**: camstream ignores `SIGHUP` (survives the launching ssh/boot shell) and the launcher must `pkill -f /opt/camstream` (NOT `-f camstream`, which matches the `camstream.sh` script's own cmdline and self-kills); it's `setsid`-detached. Single viewer at a time (go2rtc below fans out to many).

**H.264 / RTSP / WebRTC via go2rtc (on-robot, SOFTWARE x264).** `go2rtc` + static `ffmpeg` (libx264) in the chroot restream the MJPEG as H.264. Pipeline: `camstream MJPEG :8090 → go2rtc exec:ffmpeg (mpjpeg demux → libx264 ultrafast/zerolatency) → RTSP rtsp://<ip>:8554/dreame + WebRTC http://<ip>:1984/`. Auto-starts at boot (`_root_postboot.sh`); manual: `sh /data/go2rtc.sh start|stop|status`. Config: `/data/chroot/opt/go2rtc.yaml` (an `exec:` source — go2rtc otherwise misdetects the http feed as plain MJPEG and the transcode fails with "Output file does not contain any stream"; input MUST be `-f mpjpeg`, the multipart demuxer). **On-demand gates the whole chain**: no go2rtc viewer → go2rtc reaps the ffmpeg source → MJPEG pull stops → camsiphon ring off → zero idle cost on both CPU and AVA. go2rtc fans one source out to many RTSP/WebRTC viewers (lifts camstream's single-viewer limit). Binaries: `/opt/go2rtc` (1.9.14 arm64), `/opt/ffmpeg` (7.0.2 static, libx264) — `build_ava_shims.sh` does not fetch these; they were `wget`'d into the chroot.
- **NO hardware H.264 is available on this robot.** Verified: only `/dev/video0`+`/dev/video2` exist (both camera *capture*, major 81); there is **no V4L2 M2M encoder device**, so ffmpeg's `h264_v4l2m2m` has nothing to bind. The only HW H.264 path is the cedar encoder, which is locked (see below). So go2rtc's H.264 is **software libx264 on the CPU** (672×504 is light, but it's not free while a viewer is connected). The cedar HW is still used for the *JPEG* step (camstream).
- **camstream** (`/opt/camstream` in chroot): mmaps the ring, inits the **CedarX HW encoder** once, JPEG-encodes the latest frame per HTTP part. Runs INSIDE the chroot (glibc 2.39) linking the host's **vendor encoder libs** (glibc 2.23, backward-compatible) at `/data/chroot/opt/venc/` — `libvencoder/libvenc_codec/libawh264/libVE/libMemAdapter/libcdc_base` — via `/dev/cedar_dev`+`/dev/ion` (visible in the chroot). The launcher bind-mounts host `/tmp` into the chroot so both sides see the ring.
- **CedarX VideoEncoder ABI** (reverse-engineered, see `cedar_enc.c`): `VideoEncCreate(codec) → VideoEncSetParameter → VideoEncInit(&VencBaseConfig) → AllocInputBuffer → {GetOneAllocInputBuffer; memcpy Y@0,C@W*H; FlushCache; AddOneInputBuffer; VideoEncodeOneFrame; GetOneBitstreamFrame → Annex-B/JPEG bytes; Free…} → loop`. `VencBaseConfig`: `eInputFormat@24` (NV21=`VENC_PIXEL_YVU420SP`=1), `memops@32`(=`MemAdapterGetOpsS()`), `veOpsS@40`(NULL — lib inits VE itself) — offsets confirmed by disassembling `VideoEncInit`.
- **H.264 status — BLOCKED on SPS/PPS (catch-22), MJPEG is the shipping path.** Detailed RE findings (CedarC v1.2.0):
  - HW encoder produces valid Annex-B IDR/P **slices** (`VideoEncodeOneFrame→0`, flag=keyframe), but in default config never materializes **SPS/PPS** header bytes.
  - `H264GetParameter` SPS/PPS getter is index **`0x501`** (struct `{u32 nLength@0; u8* pBuffer@8}`; copies `2N/3` SPS + `N/3` PPS). It **segfaults** because its context buffers (`ctx[5664]`/`[5680]`) are NULL — SPS never generated.
  - SPS generation (`h264InitSpsPps`, which writes to the bitstream + a `pExtraData` copy) is **gated inside `H264InitVer2`** by context byte `[ctx+966]` (skips if ∈{1,2,3,4}).
  - The H.264 param setter is index **`0x106`** (`VENC_IndexParamH264Param`; handler reads only `arg[0],arg[4],arg[8]`→`ctx[1552/1556/1560]`). **Setting it hangs the HW encoder** ("h264 encoder wait interrupt overtime"), regardless of values — so forcing the profile to trigger SPS gen breaks encode. Catch-22.
  - **Synthesis** (`h264_headers.py`): the encoder uses **CABAC + `deblocking_filter_control_present=1`** (brute-force decode-verify dropped meandiff 92→40 when those were set). But the decode is still **structured corruption**, likely because the default-config slice itself is under-specified without the profile param. So synthesis can't fully recover it.
  - **Dynamic analysis done (gdb, 2026-06-17) — both routes dead-end:**
    - *Direct encoder*: gdb on `cedar_enc` shows the SPS builders (`h264InitSpsPps`, `H264InitVer2`, `_InitSPS`, `h264_init_sps_pps`) are **never called** in a `VideoEncCreate→SetParameter→VideoEncInit→VideoEncodeOneFrame` flow (only `rc_init_sequence` fires). SPS is simply never built → `0x501` NULL. The one param that changes the H264 path, `0x106`, **hangs the HW encoder**. So the direct API can't be coaxed into emitting headers without the stock streamer's full (unknown) setup.
    - *Trace AVA's working path* (the right idea): **blocked**. The encoder libs `dlopen` only on a video session, and triggering one via the `videomonitor` socket (`{"method":"open_camera"}` and `{"method":"recordVideo",...}`) **crashes AVA in `RealyVideoMonitor` BEFORE libvencoder loads** (no `VideoEnc*` calls captured; needs cloud auth/context we don't have). A real session needs Dreame's cloud/Agora handshake. gdb tracing itself works (chroot-gdb attaches to AVA, resolves `libvencoder`/`libvenc_codec` symbols, pending breakpoints) — there's just nothing to trace without a non-crashing trigger.
  - **Conclusion / SOLVED via transcode**: cedar-native H.264 needs Dreame's cloud creds or much deeper `RealyVideoMonitor` RE — abandoned. Instead we **transcode the MJPEG to H.264** (the community pattern: bypass Dreame's video stack, don't unlock it). Implemented **on-robot** with go2rtc + ffmpeg/libx264 (see "H.264 / RTSP / WebRTC via go2rtc" above) — software x264, since no HW H.264 exists on this device. A LAN host (NAS/HA) running the same go2rtc/ffmpeg is the zero-robot-CPU alternative.
  - Tools ready for future RE: `cedar_enc.c` (encode + `0x501`/SetParameter probes via `H264IDX`/`H264PROF` env), `h264_headers.py`, `vmread.c` (videomonitor nanomsg send/recv), gdb scripts in `/data/chroot/opt/*.gdb`, vendor libs at `~/dreame-re/venc/`.
- **Build**: `build_ava_shims.sh` compiles `camstream` (needs `/data/chroot/opt/venc/` populated). JPEG encoder currently emits 672×**672** (height padding) — valid scene is the top 504 rows; cosmetic.

### Valetudo REST API capabilities

Base URL: `http://192.168.1.213` (or `http://localhost` from on-robot)

**Robot state**: `GET /api/v2/robot/state/attributes` — returns array of attribute objects (fan_speed, mop, water_grade, etc.)

**Available capabilities** (`GET /api/v2/robot/capabilities`):

| Capability | Endpoint | GET returns | PUT body |
|------------|----------|-------------|----------|
| BasicControlCapability | `/api/v2/robot/capabilities/BasicControlCapability` | — | `{"command":"start"/"stop"/"pause"/"home"}` |
| FanSpeedControlCapability | `/api/v2/robot/capabilities/FanSpeedControlCapability/preset` | `{"value":"low"}` | `{"name":"low"/"medium"/"high"/"max"}` → MIIO siid:4 piid:4 |
| WaterUsageControlCapability | `/api/v2/robot/capabilities/WaterUsageControlCapability/preset` | `{"value":"low"}` | `{"name":"low"/"medium"/"high"}` |
| HighResolutionManualControlCapability | `/api/v2/robot/capabilities/HighResolutionManualControlCapability` | — | `{"action":"enable"/"disable"}` or `{"action":"move","vector":{"velocity":0..1,"angle":N}}` |
| QuirksCapability | `/api/v2/robot/capabilities/QuirksCapability` | quirk array | PUT `{"id":"...","value":"low"/"medium"/"high"}` — only exposes Carpet Mode Sensitivity |
| CarpetModeControlCapability | `/api/v2/robot/capabilities/CarpetModeControlCapability` | `{"enabled":false}` | `{"enabled":true/false}` |
| CleanRouteControlCapability | `/api/v2/robot/capabilities/CleanRouteControlCapability` | `{"route":"normal"}` | `{"route":"normal"/"..."}` |
| LocateCapability | `/api/v2/robot/capabilities/LocateCapability` | — | PUT (no body) — robot beeps |
| KeyLockCapability | `/api/v2/robot/capabilities/KeyLockCapability` | `{"enabled":false}` | `{"enabled":true/false}` |
| DoNotDisturbCapability | `/api/v2/robot/capabilities/DoNotDisturbCapability` | DND schedule | schedule object |
| MapSegmentationCapability | `/api/v2/robot/capabilities/MapSegmentationCapability` | — | segment clean commands |
| ZoneCleaningCapability | `/api/v2/robot/capabilities/ZoneCleaningCapability` | — | zone clean commands |
| GoToLocationCapability | `/api/v2/robot/capabilities/GoToLocationCapability` | — | `{"coordinates":{"x":N,"y":N}}` |
| SpeakerVolumeControlCapability | `/api/v2/robot/capabilities/SpeakerVolumeControlCapability` | `{"volume":N}` | `{"action":"set_volume","value":0-100}` — NOTE: `{"volume":N}` / `{"value":N}` both 400; the action wrapper is required |
| ConsumableMonitoringCapability | `/api/v2/robot/capabilities/ConsumableMonitoringCapability` | consumable stats | — |
| TotalStatisticsCapability | `/api/v2/robot/capabilities/TotalStatisticsCapability` | all-time stats | — |

**No clean mode (sweep/mop-only/sweep+mop) capability is exposed by Valetudo** for this robot — controlled only via `clean_parameter.json` (see above).

### HighResolutionManualControlCapability (manual driving)

Joystick-style manual control. **Verified REST schema** (Valetudo uses `action`, not `operation`):

```bash
# enable / disable
curl -X PUT http://192.168.1.213/api/v2/robot/capabilities/HighResolutionManualControlCapability \
  -H 'Content-Type: application/json' -d '{"action":"enable"}'      # or {"action":"disable"}

# move (Valetudo schema: velocity 0..1, angle degrees)
curl -X PUT http://192.168.1.213/api/v2/robot/capabilities/HighResolutionManualControlCapability \
  -H 'Content-Type: application/json' -d '{"action":"move","vector":{"velocity":0.2,"angle":0}}'
```

`{"operation":...}` / top-level `velocity`/`angle` are WRONG and return HTTP 400. Enabling manual control spins the vacuum fan at the MCU — suppressed by the SetCleaning filter (see the Vacuum fan disable section).

### MCU & LDS serial protocol (reference)

Two independent serial links from the SoC (AVA):
- **`/dev/ttyS4` — MCU** (motors: wheels, vacuum fan, brushes, pump; plus IMU/sensor telemetry). AVA opens it on a dynamic fd (observed fd 24). Per the alufers Z10 repo the MCU link is **115200**; D10s value not independently confirmed.
- **`/dev/ttyS3` — LDS / LiDAR**, **230400**. AVA opens it (observed fd 26); read-mostly (the turret streams scans; AVA rarely writes). LDS frames are a *different* format from the MCU link (old docs mislabeled it "MCU 55 AA protocol"): a **40-byte fixed frame** `55 aa 03 08 | speed | startAngle | 8×(dist_mm,qual) | endAngle | 6-byte timestamp` — **fully decoded + validated, no checksum**. Full table + the decoder/tap in `docs/sensors.md`; published as `/scan` via `libldstap.so` + `lds_scan_node.py`.

RE'd from `github.com/alufers/dreame_mcu_protocol` (cloned `~/dreame_mcu_protocol`; written for the Z10 Pro — same protocol family as the D10s; framing/CRC/SetCleaning/_CtrlMcuCMD all re-verified on our D10s). Firmware dumped at `~/dreame-re/mcu.bin` (GD32F303-class MCU, FreeRTOS) — import to Ghidra as Raw Binary, ARM Cortex LE, base `0x08000000`. AVA-side node that builds messages: `~/dreame-re/node_signal.so`.

**MCU frame format** (ttyS4):
```
3c <len> <type> <payload[len]> <crc_hi> <crc_lo> 3e
```
- `3c`='<' start, `3e`='>' end, `3f`='?' escape (a literal `3c`/`3e`/`3f` in the body is prefixed with `3f`).
- `crc` = **Modbus CRC16** over the unescaped `[len, type, payload]`, stored big-endian (repo `crc_util.py`; ported into `fanoff_shim.c`; reproduces every captured frame). The MCU occasionally emits corrupt frames not starting with `3c` — resync on the delimiters.

**SoC→MCU packets** (repo `TYPES_TO_MCU`; ✓ = re-verified on our D10s):
| type | name | payload | notes |
|------|------|---------|-------|
| 0x00 | MotorCtrl ✓ | flag:u8, linear:f32, rot:f32 | wheel velocities (driving) — `3c 09 00 …`; all-zero when stationary |
| 0x01 | SetCleaning ✓ | f1..f5:u8 | fan/brush/pump. **f3=vacuum fan level** (low/med=3,max=5,off=0); f1/f2 base power; idle=`00 01 00 00 00` |
| 0x02 | SetButtonLEDState | state:u8 | LED state; doubles as heartbeat |
| 0x04 | SetOdometer | op:u8 + 3×u32 + u8 | |
| 0x11 | SetLDSCalibration | x,y,angle:f32 | (calib JSON carried via msg 0x10) |
| 0x14 | _CtrlMcuCMD ✓ | subcmd:u8, value:u8 | MCU signal control. **subcmd 0x04 = LDS turret motor** (1=spin/0=park); other subcmds drive IR/cliff switches (`RobotIRSwitch`) |
| 0x1d | LaserOrTofControl | reset_trans:u8, value:u8 | 6=laser reset, 1=tof reset, 4=camera-stereo reset |
| 0x1f | CalibrateIMU | op:u8 | 1=start, 5=query (replies 0x11) |
| 0x0f | Pong | u32 | latency reply to MCU Ping |

**MCU→SoC packets** (repo `TYPES_FROM_MCU`):
| type | name | notes |
|------|------|-------|
| 0x00 | Triggers | bumpers, cliff/IR, dock; per-motor over-current + error bits (fan_error, lidar_error, …) — handy for diagnostics |
| 0x01 | Status20ms | odometry x/y/yaw, wheel vel, roller+sidebrush current (length differs D10s vs Z10) |
| 0x02 | Status10ms | IMU gyro/accel + wheel distances |
| 0x03 | Status100ms | pitch/roll, wheel current, dust/water/hepa/carpet bits |
| 0x05 | Status500ms | RTC timestamp |
| 0x07 | McuFwVersionInfo | git hash + version |
| 0x0b | lidar | 1=start lidar calibrate/spinup, 2=stop |
| 0x0f | PingMsg | latency probe (reply Pong 0x0f) |
| 0x27 | McuLog | 12B; AVA saves to /data/log/mculog.bin |
| 0x29 | HwInfo | MCU/IMU/charger/app type ids |
| 0x2b | BatteryStatus | voltage, current, temp, charge, SoC% |

**AVA node architecture** (one `.so` per node in `/ava/lib/`, nanomsg IPC):
- `node_com.so` — link layer: serial connect + framing + CRC only (no packet semantics).
- `node_signal.so` — "HAL": builds/parses MCU message *contents* (`AvaNodeSignal::CleanSetProcess`→SetCleaning, `MoveControlProcess`→MotorCtrl); also maps raw LDS data upward.
- `node_lds.so` — low-level LDS serial.
- `node_cmd.so` — serves `avacmd` on `/tmp/avacmd.socket`.
- `liberos_tactics_tree.so` — the behavior tree (BehaviorTree.CPP): `RobotMcuSignalCtrl{RobotIRSwitch(Cliff/Front), LDS_Switch}`, `ChangeRobotModeTo{…Remote, Auto, BackHome, FastMapBuild…}`, escape/warning nodes, etc.

To modify the stream, interpose via `LD_PRELOAD` on AVA (see "Vacuum fan + LiDAR quieting") — do not open the serial directly (conflicts with AVA).

### avaexec socket

`/tmp/avaexec.socket` — nanomsg IPC socket. `exec_proc` node listens and executes shell commands via `system()`. Used by `msg_cvt.sh` for network/provisioning tasks. Not directly useful for cleaning mode control.

`msg_cvt.sh` at `/ava/script/msg_cvt.sh` handles: start_up, location, play, net_error, set_device_time, iot_state, iot_restore, iot_reset, download_file, reset_device, add_ap, del_ap, get_ap.

---

## Useful commands

```bash
# SSH to robot
ssh dreame-home          # home network (192.168.1.213) via 4K (2.4GHz)
ssh dreame               # robot AP mode (192.168.5.1)

# Enter Ubuntu chroot on robot
ssh dreame-home 'sh /data/chroot.sh'

# Check Valetudo status
ssh dreame-home 'cat /tmp/valetudo.log | tail -20'

# Check WiFi connection
ssh dreame-home 'wpa_cli -iwlan0 status'

# View boot logs (structured now)
ssh dreame-home 'cat /tmp/root_sh.log'      # early boot log
ssh dreame-home 'cat /tmp/postboot.log'     # postboot sequence log

# Check work_mode and fan speed
ssh dreame-home 'avacmd msg_cvt '"'"'{"type":"msgCvt","cmd":"get_prop","prop":"work_mode"}'"'"''
ssh dreame-home 'curl -s http://localhost/api/v2/robot/capabilities/FanSpeedControlCapability/preset'

# Set fan to off (Valetudo "low" = MIIO 0 = off)
ssh dreame-home 'curl -s -X PUT http://localhost/api/v2/robot/capabilities/FanSpeedControlCapability/preset \
  -H "Content-Type: application/json" -d '"'"'{"name":"low"}'"'"''

# --- fanoff system (vacuum fan off always; LiDAR off only in manual nav) ---
# Is the shim loaded into AVA?
ssh dreame-home 'grep -q libfanoff_filter.so /proc/$(pidof ava)/maps && echo loaded || echo MISSING'
# Is the event-driven LiDAR gate daemon running (holds the Valetudo SSE stream)?
ssh dreame-home 'ps w | grep "[f]anoff_flag"'
# LiDAR gate state: present = allowed (non-manual mode), absent = blocked (manual/idle)
ssh dreame-home 'ls /tmp/lidar_allow 2>/dev/null && echo allowed || echo "blocked"'
# Verify on the wire during manual nav (expect SetCleaning 00 01 = fan off, CtrlMcu 14 04 00 = LiDAR off):
ssh dreame-home 'A=$(pidof ava); timeout 3 chroot /data/chroot strace -f -e trace=write -xx -s64 -p $A -o /tmp/x 2>/dev/null; grep -aoE "x3c.x05.x01.x..\x..|x3c.x02.x14.x04.x.." /data/chroot/tmp/x | sort | uniq -c'
# Rebuild + reload shim after editing scripts/robot/fanoff_shim.c (scp it to /data first):
ssh dreame-home 'sh /data/build_ava_shims.sh && killall -9 ava'   # ava.sh relaunches with the new shim
# Restart the LiDAR gate daemon (without reboot):
ssh dreame-home 'pkill -f fanoff_flag; setsid sh /data/fanoff_flag.sh </dev/null >/dev/null 2>&1 &'
# Mute voice prompts (e.g. manual-nav "Start remote controlled cleaning"):
ssh dreame-home 'curl -s -X PUT http://localhost/api/v2/robot/capabilities/SpeakerVolumeControlCapability -H "Content-Type: application/json" -d '"'"'{"action":"set_volume","value":0}'"'"''

# Deploy scripts to robot (e2e: AP connect → deploy → reboot → reconnect 5K)
bash /tmp/robot-deploy.sh     # see scripts/robot/deploy.sh for the template

# Test camera
ssh dreame-home 'v4l2-ctl --device=/dev/video0 --info'

# Play test audio on robot
ssh dreame-home 'aplay -D hw:0,0 /usr/share/sounds/alsa/Front_Left.wav'

# Check miio state transitions at boot
ssh dreame-home 'grep -E "(STATE|AP_mode|wifi_conf)" /data/log/user.log | tail -20'

# Check if robot is in provisioning/AP mode
ssh dreame-home 'grep "ap will close" /data/log/user.log | tail -3'
```

---

## Repository layout

```
scripts/
  robot/
    _root.sh               deploy to /data/_root.sh on robot (CONTAINS WIFI CREDENTIALS)
    _root_postboot.sh      deploy to /data/_root_postboot.sh on robot
    chroot.sh              deploy to /data/chroot.sh on robot
    camsiphon.c            LD_PRELOAD camera siphon: AVA's live NV21 frames at VIDIOC_DQBUF (read-only) ✅
                           one-shot grab (/tmp/cam_grab) + continuous RAM-ring stream (/tmp/cam_stream)
    cam_grab.sh            robot-side: touch /tmp/cam_grab -> camsiphon writes /tmp/cam_frame.raw (NV21)
    nv21_to_png.py         workstation: convert a siphoned NV21 frame -> PNG (PIL)
    camstream.c            cedar HW-JPEG MJPEG-over-HTTP server (chroot-native; reads the RAM ring) ✅
    camstream.sh           robot-side launcher: start/stop/status the MJPEG stream on :8090
    go2rtc.yaml            go2rtc config: MJPEG -> H.264 (exec:ffmpeg libx264) -> RTSP/WebRTC
    go2rtc.sh              robot-side launcher: start/stop/status go2rtc (:8554 RTSP, :1984 WebRTC)
    rollback.sh            restore stock AVA (undo shims+daemons) for the current boot
    cedar_enc.c            cedar HW encoder test/tool (NV21 -> JPEG works; H.264 IDR encodes, SPS/PPS TBD)
    h264_headers.py        workstation: synthesize H.264 SPS/PPS (for the in-progress H.264 path)
    camera_stream.sh       (unused) GStreamer stub — gst not installed; video0 deadlocks while AVA streams
    v4l2grab.c             multi-plane V4L2 grabber -> PPM (dead end: video0 reconfigure deadlocks the ISP)
    vmread.c               nanomsg-PAIR probe for /tmp/videomonitor.socket (RE of the cloud video relay)
    audio_server.py        run in robot chroot to serve audio playback
    dreame-wifi-setup.sh   e2e script: connect AP → deploy → reconnect 5K
    fanoff_shim.c          LD_PRELOAD shim: fan off always + LiDAR off when blocked (freestanding)
    fanoff_flag.sh         event-driven (Valetudo SSE) LiDAR gate: /tmp/lidar_allow in non-manual modes
    valetudo_bridge.py     Valetudo REST -> ROS 2: /map (OccupancyGrid), /odom + TF, /robot/status
    valetudo_bridge.sh     robot-side launcher: start/stop/status the ROS bridge (chroot ROS 2 Jazzy)
    ldstap.c               LD_PRELOAD read-tap: tees AVA's ttyS3 (LiDAR) -> tmpfs shm ring (freestanding)
    lds_decode.py          LDS frame decoder (reference) + strace-capture analyzer
    lds_scan_node.py       chroot-ROS node: shm ring -> sensor_msgs/LaserScan on /scan
    mcutap.c               MCU ttyS4 read-tap (errno-corrected; read interposition now AVA-safe — unbuilt: needs shm-ring tee, see ldstap.c)
    build_ava_shims.sh     compile fanoff + camsiphon + ldstap .so + camstream in chroot, glibc-2.23-safe
    capture_cleanset.sh    capture MCU 3c..3e frames across fan states
    deploy_ava_shims.sh    bind-mount patched ava.sh exporting the LD_PRELOAD shim list, restart + verify
  companion/
    install_ros2.sh        run on Dragon Q6A to install ROS 2 Jazzy
robot/
  boot/README.md           deployment instructions for boot hooks
  valetudo/valetudo.json   Valetudo configuration
  config/
    ava/
      clean_parameter.json versioned snapshot of /data/config/ava/clean_parameter.json
      iot_conf.json        versioned snapshot of /data/config/ava/iot_conf.json
    miio/
      wifi.conf            versioned snapshot of /data/config/miio/wifi.conf (dummy content)
companion/
  ros2/                    ROS 2 node packages (valetudo_bridge, camera_node, etc.)
docs/
  hardware.md              wiring, power, physical setup
  wifi-hack.md             detailed explanation of the wpa_supplicant fix
  sensors.md               sensors & data access (LiDAR, camera, IMU, mic) + how to read each
  ros.md                   ROS 2 integration: Valetudo bridge topics/QoS/frames, chroot ROS,
                           cross-host DDS to the Q6A, parked IMU, roadmap
  device-test-checklist.md manual smoke test after shim/stream/bridge changes
```

**Important**: `scripts/robot/_root.sh` contains the WiFi PSK for the 4K network (2.4GHz home). Do not replace with placeholder values when deploying — this breaks WiFi connectivity.

**Robot config versioning**: Key config files from `/data/config/` are copied to `robot/config/` so changes can be tracked and rolled back. To update: `scp dreame-home:/data/config/ava/clean_parameter.json robot/config/ava/`.

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
- LiDAR: on `/dev/ttyS4` — read exclusively by AVA, proprietary binary protocol
- Camera: OV8856 MIPI sensor, exposed as `/dev/video0` and `/dev/video2` via V4L2
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
Valetudo MQTT (port 1883) ──── MQTT ──────────►  valetudo_bridge node
                                                    └─► /scan /odom /map

/dev/video0 ──► camera_stream.sh ─── UDP/H.264 ──► camera_node
                                                    └─► /camera/image_raw

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
| LiDAR/map | Valetudo MQTT | MQTT → ROS bridge | `/scan`, `/map` topics |
| Robot pose | Valetudo MQTT | MQTT → ROS bridge | `/odom` topic |
| Nav commands | Dragon Q6A Nav2 | Valetudo REST API | AVA motors via Valetudo |
| Audio | Dragon Q6A TTS | TCP socket (WAV) | aplay on robot speaker |

---

## Key constraints and gotchas

1. **WiFi is 2.4GHz only** — robot connects to `4K` SSID (2.4GHz). The `5K` SSID (5GHz) won't be seen by the robot.
2. **Single radio** — robot AP (hostapd on wlan0) and STA mode (wpa_supplicant) cannot run simultaneously on different channels. Our fix: kill hostapd at boot, use wlan0 as STA only.
3. **BusyBox wget cannot follow HTTPS redirects** — always use `curl -L` on the robot, or download on laptop and `scp`
4. **Kernel 4.9.191** — Ubuntu 24.04 glibc 2.39 mostly works, but apt's sandbox user (`_apt`) cannot do DNS — requires `APT::Sandbox::User "root"` workaround
5. **AVA owns `/dev/ttyS4`** — LiDAR cannot be read directly without conflicting with AVA. Use Valetudo MQTT for map/scan data instead.
6. **squashfs root** — any change to system files requires either a bind mount from `/data/` or a chroot. Never attempt `mount -o remount,rw /`. `/etc/miio` is a SYMLINK to `/data/config/miio/` (so it's already writable via /data/).
7. **exec_monitor.sh** watches only the `ava` process — it does NOT restart hostapd if killed. Safe to kill hostapd permanently.
8. **firmware 1413 IoT flag** — AVA does NOT connect to `miio_agent` (TCP 54320) at boot unless `/data/config/ava/iot.flag` contains `miiot`. This flag is normally written during cloud provisioning (which we bypass via Valetudo). Without it, ALL Valetudo MIIO property/action commands time out. Fix: `_root_postboot.sh` writes `miiot` to the flag and calls `avacmd iot '{"type":"iot","notify":"open_server"}'` at boot.
9. **work_mode 17 persists at boot** — AVA enters work_mode 17 (RemoteCtrlMode) on every boot via the MIIO provisioning flow. Most Valetudo capabilities return HTTP 400 in this state. See `### work_mode 17 — root cause and investigation` below.
10. **_root.sh has hardcoded WiFi credentials** — the `4K` SSID PSK is hardcoded. Deploying from repo (which may have placeholder `YOUR_SSID`/`YOUR_HEX_PSK`) will break WiFi. Always verify credentials before deploy.
11. **Bind-mount a FILE over a non-existent squashfs path = boot failure** — if you try `mount --bind /data/file /etc/miio/nonexistent` where `nonexistent` doesn't exist in squashfs, the bind-mount silently fails but breaks init. Always bind-mount directories, or ensure the target file exists in squashfs first.

---

## Robot internals and configuration

### AVA daemon internals

AVA (`/ava/bin/ava`) is a closed-source C++ binary running as PID 1-level process on the robot. It owns all hardware:
- Motors (wheels, side brush, main brush, fan/vacuum, mop pump) via `/dev/ttyS3` (MCU serial, 230400 baud, binary `55 AA`-prefixed protocol, fd 26 of ava process)
- LiDAR via `/dev/ttyS4` (fd 24 of ava process)
- SLAM, path planning, behavior tree

AVA is structured as a behavior tree with named nodes. Nodes communicate via pub/sub (nanomsg) and expose a command interface through `/tmp/avacmd.socket`.

**Key AVA file descriptors:**
| fd | Device | Purpose |
|----|--------|---------|
| 24 | `/dev/ttyS4` | LiDAR serial |
| 26 | `/dev/ttyS3` | MCU (motors, sensors) serial |

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

**Symptom**: On every boot, AVA immediately enters work_mode 17 (RemoteCtrlMode). Most Valetudo REST capabilities return HTTP 400 in this state. `{"operation":"disable"}` on HighResolutionManualControlCapability also returns 400.

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
- `curl PUT /api/v2/robot/capabilities/HighResolutionManualControlCapability {"operation":"disable"}` → HTTP 400
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
| `CleanMode` | 0=sweep, 1=mop-only, 2=sweep+mop | **1 permanently disables vacuum fan** |
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
   - `WritePropInt type=0 value=0` → **CleanMode RESET to 0 (sweep)** ← fan activates here
   - `WritePropString piid:13 {"spdv":0,"spdw":0,"audio":"true","random":N}` → RemoteCtrlMode entered
4. Periodic keepalive piid:13 writes with `audio:false`

**Critical: AVA resets CleanMode to 0 (sweep mode = fan ON) every time piid:13 is written** — i.e., every time Valetudo sends a remote control command. This overrides our boot-time `clean_parameter.json` CleanMode:1 patch, and also overrides `set_only_mop.py`'s `only_mop=1` (which is a separate blackboard key from CleanMode type=0).

---

### Permanent vacuum fan disable

**Goal**: Use `HighResolutionManualControlCapability` (manual driving) without the vacuum fan running.

**Root cause — UPDATED after debugging:**

When Valetudo starts and connects to dummycloud (t≈22s after boot), it sends `WritePropString piid:13 {"spdv":0,"spdw":0,"audio":"true","random":N}` — the initial RemoteCtrlMode enable. AVA processes this and:
1. Executes `WritePropInt type=0 value=0` → resets CleanMode to 0 (sweep mode = fan enabled)
2. Enters RemoteCtrlMode (work_mode 17)
3. Fan activates via MCU

This happens BEFORE our postboot fan_speed=0 set (t≈90s). Setting `FanSpeedControlCapability "low"` after this point:
- Returns "OK" from Valetudo
- Valetudo updates its internal cache to rawValue=0
- But the state REVERTS to rawValue=3 (max) after some time, suggesting AVA overrides it or the command doesn't affect the running fan in RemoteCtrlMode
- **Fan continues running** — confirmed by user

**`HighResolutionManualControlCapability enable` returns HTTP 400** — because robot is ALREADY in RemoteCtrlMode from Valetudo's automatic startup init. This is NOT a blocking error; the robot IS in remote control mode.

**piid:13 vs piid:15**: CLAUDE.md previously said piid:15 for remote control. Actual internal AVA log shows piid:13. These may be different numbering systems (MIIO siid/piid vs AVA internal property index).

**Valetudo fan speed preset mapping** (D10s Pro — offset by 1 from MIIO):
| Valetudo preset | MIIO siid:4 piid:4 | Fan behavior |
|---|---|---|
| `low` | 0 | Fan off (stored preset) |
| `medium` | 1 | Low speed |
| `high` | 2 | Medium speed |
| `max` | 3 | High speed |

**Method 1 — FanSpeedControlCapability at boot (PARTIALLY WORKING, unreliable)**:
Set `FanSpeedControlCapability` preset to `"low"` (= MIIO 0 = off) after Valetudo starts. This sets the stored fan_speed preset in AVA's MIIO property store. However:
- Fan is already running from t=22s (Valetudo's piid:13 init), set doesn't run until t≈90s
- State may revert to rawValue=3 (max) — suggesting the command doesn't affect the currently running fan in RemoteCtrlMode
- Still implemented in `_root_postboot.sh` (30×3s retry loop) as a safety measure

**Method 2 — `set_only_mop.py` daemon (does NOT suppress fan in work_mode 17)**:
`set_only_mop.py` holds AVA BT blackboard `only_mop=1` via heap patch. Does NOT prevent the fan. Root cause confirmed: `only_mop` is a SEPARATE blackboard key from `CleanMode` (WritePropInt type=0). The daemon holds `only_mop=1` but AVA still resets `CleanMode=0` (sweep) on every piid:13 write. Daemon kept running — prevents fan in non-RemoteCtrlMode sweep mode.

**Method 3 — `clean_parameter.json` CleanMode:1 (partial)**:
Loads at boot, sets CleanMode=1. But AVA actively RESETS CleanMode to 0 when it processes piid:13 commands. Does NOT prevent fan in work_mode 17.

**Note on CleanMode:1**: Kept in `_root.sh` as a safety net, but it does NOT control the fan in work_mode 17.

**Note on manual control speed**: CleanMode:1 (mop-only) does NOT reduce manual driving speed. Manual control wheel speed is governed by `spdv`/`spdw` MIIO parameters (`remote_params` blackboard), not by mop speed limits.

**Test result (under investigation):**
Live monitoring captured the HighResolutionManualControlCapability enable sequence:
- work_mode: 17 (provisioning) → 13 (intermediate) → 17 (RemoteCtrlMode proper)
- log_0: `WritePropInt type=0 value=0` (CleanMode=0/sweep reset) then piid:13 `{"spdv":0,"spdw":0,"audio":"true","random":554}`
- `fan_speed` Valetudo state: stayed "low" (rawValue=0) throughout enable sequence
- Conclusion pending user confirmation: if fan was silent, `FanSpeedControlCapability "low"` DOES suppress the fan even with CleanMode=0 in RemoteCtrlMode. If fan still ran, the MIIO fan_speed preset doesn't affect the motor in RemoteCtrlMode.

**If still failing — next approaches to investigate:**
1. **Heap-patch CleanMode (WritePropInt type=0)**: Extend `set_only_mop.py` to also hold the integer property type=0 (CleanMode) at value=1 (mop-only) continuously. Different from `only_mop` — need to find the integer property array address in AVA's heap/data segment.
2. **Block piid:13 fan activation**: Intercept the miio_agent→AVA TCP 54320 channel and filter/modify piid:13 messages to prevent the CleanMode reset.
3. **MCU CLEANSET intercept**: Intercept `/dev/ttyS3` writes (fd 26 of AVA) to suppress fan motor commands. Risk: conflicts with AVA.

**What doesn't work**:
- `avacmd msg_cvt '{"type":"msgCvt","cmd":"set_prop","prop":"clean_mode","value":"1"}'` → `{}` (not supported)
- `avacmd clb '{"type":"clb","cmd":"set_clean_mode","value":1}'` → `{}` (not exposed)
- `miio_send_line '{"method":"set_properties","params":[{"siid":4,"piid":7,"value":1}]}'` → only sends to provisioning helper port (54323), not device command channel
- `chattr +i clean_parameter.json` → ext4 on `/data/` does not support immutable flag on kernel 4.9.191
- `CleanMode:1` in `clean_parameter.json` → AVA resets to CleanMode=0 on every piid:13 write
- `only_mop=1` heap patch (set_only_mop.py) → patches different key, CleanMode still reset to 0
- `FanSpeedControlCapability "low"` at boot (t≈90s) → fan already running since t≈22s; command may not affect running fan in RemoteCtrlMode

### Valetudo REST API capabilities

Base URL: `http://192.168.1.213` (or `http://localhost` from on-robot)

**Robot state**: `GET /api/v2/robot/state/attributes` — returns array of attribute objects (fan_speed, mop, water_grade, etc.)

**Available capabilities** (`GET /api/v2/robot/capabilities`):

| Capability | Endpoint | GET returns | PUT body |
|------------|----------|-------------|----------|
| BasicControlCapability | `/api/v2/robot/capabilities/BasicControlCapability` | — | `{"command":"start"/"stop"/"pause"/"home"}` |
| FanSpeedControlCapability | `/api/v2/robot/capabilities/FanSpeedControlCapability/preset` | `{"value":"low"}` | `{"name":"low"/"medium"/"high"/"max"}` → MIIO siid:4 piid:4 |
| WaterUsageControlCapability | `/api/v2/robot/capabilities/WaterUsageControlCapability/preset` | `{"value":"low"}` | `{"name":"low"/"medium"/"high"}` |
| HighResolutionManualControlCapability | `/api/v2/robot/capabilities/HighResolutionManualControlCapability` | — | `{"operation":"enable"/"disable"/"move","velocity":N,"angle":N}` |
| QuirksCapability | `/api/v2/robot/capabilities/QuirksCapability` | quirk array | PUT `{"id":"...","value":"low"/"medium"/"high"}` — only exposes Carpet Mode Sensitivity |
| CarpetModeControlCapability | `/api/v2/robot/capabilities/CarpetModeControlCapability` | `{"enabled":false}` | `{"enabled":true/false}` |
| CleanRouteControlCapability | `/api/v2/robot/capabilities/CleanRouteControlCapability` | `{"route":"normal"}` | `{"route":"normal"/"..."}` |
| LocateCapability | `/api/v2/robot/capabilities/LocateCapability` | — | PUT (no body) — robot beeps |
| KeyLockCapability | `/api/v2/robot/capabilities/KeyLockCapability` | `{"enabled":false}` | `{"enabled":true/false}` |
| DoNotDisturbCapability | `/api/v2/robot/capabilities/DoNotDisturbCapability` | DND schedule | schedule object |
| MapSegmentationCapability | `/api/v2/robot/capabilities/MapSegmentationCapability` | — | segment clean commands |
| ZoneCleaningCapability | `/api/v2/robot/capabilities/ZoneCleaningCapability` | — | zone clean commands |
| GoToLocationCapability | `/api/v2/robot/capabilities/GoToLocationCapability` | — | `{"coordinates":{"x":N,"y":N}}` |
| SpeakerVolumeControlCapability | `/api/v2/robot/capabilities/SpeakerVolumeControlCapability` | `{"value":N}` | `{"value":0-100}` |
| ConsumableMonitoringCapability | `/api/v2/robot/capabilities/ConsumableMonitoringCapability` | consumable stats | — |
| TotalStatisticsCapability | `/api/v2/robot/capabilities/TotalStatisticsCapability` | all-time stats | — |

**No clean mode (sweep/mop-only/sweep+mop) capability is exposed by Valetudo** for this robot — controlled only via `clean_parameter.json` (see above).

### HighResolutionManualControlCapability (manual driving)

Enables joystick-style manual control. Sends MIIO property `{"siid":X,"piid":15,"value":{"spdv":N,"spdw":N,"audio":"true","random":N}}`. Puts AVA into work_mode 17.

**Enabling from REST:**
```bash
curl -X PUT http://192.168.1.213/api/v2/robot/capabilities/HighResolutionManualControlCapability \
  -H 'Content-Type: application/json' \
  -d '{"operation":"enable"}'
```

**Moving:**
```bash
curl -X PUT http://192.168.1.213/api/v2/robot/capabilities/HighResolutionManualControlCapability \
  -H 'Content-Type: application/json' \
  -d '{"operation":"move","velocity":N,"angle":N}'
```

`velocity`: forward/backward speed (positive=forward), `angle`: turning rate (positive=left).

**Warning**: Without CleanMode:1 (mop-only), enabling manual control also starts the vacuum fan.

### MCU serial communication (ttyS3)

AVA communicates with the MCU (motor controller) via `/dev/ttyS3` at 230400 baud. Protocol: binary, `55 AA`-prefixed framing. AVA holds fd 26 open to this device.

**MCU pub/sub message types** (internal to AVA, not directly accessible externally):
- `CLEANSET` — sets motor speeds (fan, brush, pump)
- `MOVE_CTRL` — sets wheel velocity
- `BUTTONLED` — controls LEDs
- `ROBOTMODE` — sets robot operating mode

Direct MCU serial access would conflict with AVA. Use avacmd/Valetudo API instead.

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
ssh dreame-home 'cat /tmp/only_mop.log'     # fan suppress daemon log

# Check work_mode and fan speed
ssh dreame-home 'avacmd msg_cvt '"'"'{"type":"msgCvt","cmd":"get_prop","prop":"work_mode"}'"'"''
ssh dreame-home 'curl -s http://localhost/api/v2/robot/capabilities/FanSpeedControlCapability/preset'

# Set fan to off (Valetudo "low" = MIIO 0 = off)
ssh dreame-home 'curl -s -X PUT http://localhost/api/v2/robot/capabilities/FanSpeedControlCapability/preset \
  -H "Content-Type: application/json" -d '"'"'{"name":"low"}'"'"''

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
    camera_stream.sh       run in robot chroot to stream /dev/video0
    audio_server.py        run in robot chroot to serve audio playback
    dreame-wifi-setup.sh   e2e script: connect AP → deploy → reconnect 5K
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
```

**Important**: `scripts/robot/_root.sh` contains the WiFi PSK for the 4K network (2.4GHz home). Do not replace with placeholder values when deploying — this breaks WiFi connectivity.

**Robot config versioning**: Key config files from `/data/config/` are copied to `robot/config/` so changes can be tracked and rolled back. To update: `scp dreame-home:/data/config/ava/clean_parameter.json robot/config/ava/`.

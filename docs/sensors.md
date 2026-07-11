# Dreame D10s Pro (r2250) — sensors & data access

What the robot can sense, where each stream lives, and how to read it for the AI-rover build.
All verified live on 2026-06-15/16.

| Sensor | Device / source | Owner | Independently readable? |
|--------|-----------------|-------|--------------------------|
| LiDAR (LDS) | `/dev/ttyS3` @ 230400 | AVA (`node_lds.so`) | via Valetudo map/MQTT, or tap ttyS3 |
| Camera (OV8856) | `/dev/video0`+`/dev/video2` (sunxi-vin) | AVA (pipeline bound to video2) | not live (sensor exclusive); JPEGs saved by AVA — see below |
| IMU + odometry + currents | MCU stream on `/dev/ttyS4` | AVA (`node_signal.so`) | yes — tap ttyS4 (strace/shim) or decode |
| Microphone | — | — | none (codec capture errors out) |

---

## LiDAR turret (LDS)

A 360° spinning laser distance sensor on **`/dev/ttyS3` @ 230400 baud**, read by `node_lds.so`
(then `node_lds_calibration.so` → `node_lidar_slam.so`, all in-process inside AVA). This is a
*different* protocol from the MCU link and is what older notes mislabeled as the "MCU 55 AA protocol".

The **two holes** in the turret are the laser **emitter (TX)** window and the **receiver/photodiode
(RX)** lens — standard triangulation-LiDAR layout (project a dot, read its offset reflection,
distance = triangulation).

### Decoded LDS frame format (2026-06-19)

Reverse-engineered from a live capture (errno-safe `strace -e read=26` of AVA while the turret spun
in forced-allow manual mode — see the capture method below). **40-byte fixed frame**, little-endian,
back-to-back, no inter-frame gap:

| Off | Len | Field | Notes |
|----:|----:|-------|-------|
| 0   | 2   | sync `55 aa` | frame start |
| 2   | 2   | type `03 08` | constant (frame/packet type) |
| 4   | 2   | rotation speed | LE16; ~0x5E0A steady (settles from higher during spin-up) |
| 6   | 2   | **start angle** | LE16; **unit = 360°/65536** (≈0.0055°); increments ~57/sample |
| 8   | 24  | **8 samples** | each `[dist:LE16 mm][quality:1]`; `dist==0x8000` = no return/invalid |
| 32  | 2   | **end angle** | LE16; `endAngle[n] ≈ startAngle[n+1]` (continuous) |
| 34  | 6   | timestamp | ~40-bit **monotonic counter** (+~251 M/frame). **No checksum, no end marker.** |

**Sync on the 4-byte `55 aa 03 08`** (sync + constant type). There is NO per-frame checksum and NO
trailing marker — a short early capture made byte 39 look like a constant `0xa4`, but that was just
the timestamp's high byte at that moment (live captures show it varying, e.g. ending `…86 08`). The
brute-force "failure" *was* the result: `[34:39]` is strictly monotonic (1187/1187 steps), i.e. a
counter, not a checksum. Frame integrity is the `55 aa 03 08` sync + per-sample quality bytes.

8 samples span `[startAngle, endAngle]`; `angle[k] = startAngle + (endAngle-startAngle)*k/8`.
Validated two ways: a 4 s capture (1188 frames, 7443 points) reconstructs a coherent top-down room
scan, AND the scan cross-checks against Valetudo's SLAM walls — recovered orientation (LDS angle-0 →
world 150°) matches the robot's heading (146°) within 4°, near-wall distances agree to ~16 mm median.

### Capturing raw LDS frames safely (no AVA restart)

AVA owns ttyS3 exclusively and the turret only spins during active (non-manual) nav, so:
1. Stop the `fanoff_flag.sh` gate daemon(s) and force `: > /tmp/lidar_allow` (the fanoff shim then
   permits the turret).
2. `PUT {"action":"enable"}` to `HighResolutionManualControlCapability` — robot stays **stationary**
   (no move vector sent), no vacuuming, but AVA spins the turret and reads ttyS3.
3. Capture: `chroot /data/chroot strace -f --seccomp-bpf -e trace=read -e read=26 -p $(pidof ava)`
   — `--seccomp-bpf` keeps overhead negligible (AVA shrugged it off; pid unchanged).
4. **Always restore:** disable manual, `rm /tmp/lidar_allow`, restart one `fanoff_flag.sh`.

### Read path for the rover

AVA owns ttyS3 for SLAM. Two routes to lidar data, in order of risk:
- **Processed map/pose (live, zero risk):** the **Valetudo→ROS bridge** (REST/SSE, broker-free —
  `/map`, `/odom`; see `docs/ros.md`). Valetudo exposes **no raw `/scan`**.
- **Raw `/scan` via passive read-tap (LIVE):** `libserialtap.so` (LD_PRELOAD, errno-correct) tees AVA's
  ttyS3 reads into a tmpfs shm ring (`/tmp/lds_ring.buf`); `lds_scan_node.py` (chroot ROS) decodes
  and publishes `sensor_msgs/LaserScan` on `/scan`. Boot-persistent (preloaded via `_root.sh`, node
  launched by `_root_postboot.sh`); verified across a reboot. **Zero overhead when the turret is
  gated off** (no ttyS3 reads → empty ring). Calibration: `base_link bearing = -lds_deg + offset`
  (handedness −1; offset ~0–5° — tune in RViz). See `serialtap.c` / `lds_scan_node.py` / `docs/ros.md`.

---

## Camera

- **One physical sensor: OmniVision `ov8856_mipi`** (~8 MP MIPI), via the Allwinner **sunxi-vin**
  pipeline (`sunxi_csi` → `sunxi_mipi` → `sunxi_isp` → `sunxi_scaler` → video node).
- Exposed as **two V4L2 nodes**: `/dev/video0` (`vin_video0`) and `/dev/video2` (`vin_video2`)
  — same sensor, two VIN paths.
- **AVA holds `/dev/video2` open** (plus `media0`, `isp`, `h3a` subdevs) and runs object AI on it:
  loaded nodes `node_camera_ai.so`, `node_camera_streamer.so`, `node_cameracalibr_ai.so`,
  `node_ai_area_recognition.so`. So the camera is **powered/in use by default** while AVA runs.
- The Z10's separate IR-cam + line-laser on `/dev/ttyS2` (`node_camera_laser.so`) is **NOT used**
  on this D10s — that node isn't loaded and ttyS2 is idle.

### Direct capture from /dev/video0 — blocked at runtime
`sunxi-vin` is a **multi-plane** V4L2 device (`V4L2_CAP_VIDEO_CAPTURE_MPLANE`). `/dev/video0` is
free to `open()` and enumerates a long format list (NV12/NV21/YUYV, raw Bayer BA81/RGGB/…, RGB),
but it comes up **unconfigured** (`G_FMT` → `0x0`, `num_planes=0`) and `S_FMT` returns `EINVAL`:
the media-controller links (sensor→csi→isp→scaler→video0) are not set up, and there's only one
sensor — which AVA has bound to the `video2` pipeline. Independent live capture would require
`media-ctl` (not installed on the robot) to wire the pipeline **and** the sensor freed from AVA.
The capture helper `scripts/robot/v4l2grab.c` (multi-plane V4L2 → PPM) is committed for if/when
we set that up (install `v4l-utils`+`media-ctl` in the chroot and pause AVA's camera).

### Easy frame source — AVA's AI image dumps  ✅
AVA continuously saves obstacle/furniture detection frames as JPEG to
**`/data/ai_offline_collection/*.jpg`** — real OV8856 frames, **672×504, color baseline JPEG**
(the AI-pipeline output size; sensor native is higher). Grab the latest without disturbing AVA:
```sh
ssh dreame-wifi 'cd /data/ai_offline_collection; ls -t *.jpg | head -1'   # newest
# pull (BusyBox has no sftp): base64 over ssh
ssh dreame-wifi 'base64 /data/ai_offline_collection/<file>.jpg' | base64 -d > frame.jpg
```
Sample frames pulled to `~/dreame-camera/` show the robot's low, wide-FOV view of the room.

### Live camera control — reverse-engineered (and why it's a dead end for us)
AVA's camera streamer (`node_camera_streamer.so`) contains two roles: `AvaNodeCameraStreamer`
(avacmd node **`streamer`**) and `RealyVideoMonitor` (the cloud video-monitor relay).

- **`avacmd streamer` control — SAFE.** `avacmd streamer '{"cmd":"get_camera_state"}'` → `{"state":"open"}`.
  Commands `open_camera`/`close_camera`/`get_camera_state`; params `width`/`height`/`fps_in`/`fps_out`/
  `enable_photo_thread`/`enable_sync`. **Verified stable** — `open_camera` (even with new width/height/fps)
  ran with AVA staying up 30 s+, same PID, Valetudo 200. (An earlier reboot I'd blamed on this was
  *coincidental* — it does NOT crash AVA.) But `enable_photo_thread` produced **no files while idle**:
  frames only flow when AVA is actively pulling them (navigation), same as the AI dumps.

- **`/tmp/videomonitor.socket` is a nanomsg PAIR (`ipc://`) endpoint** — the 8-byte blob is the nanomsg
  **SP greeting** (`\0SP\0` + protocol `0x0010`=PAIR + reserved), not a frame header. A raw reader gets
  stuck because it must send its own greeting back. With a proper greeting the handshake completes —
  but AVA then sends nothing unprompted (`RealyVideoMonitor` is a *relay*, it waits for commands).
- **⚠️ Sending a JSON command into that socket CRASHED AVA** (2026-06-16): a single
  `{"method":"takephoto"}` over the PAIR socket made AVA close the connection and the `ava` process
  died (watchdog restarted it ~24 s later; **no system reboot**, and the fanoff shim auto-reloaded via
  the `ava.sh` bind-mount). So the relay socket is **not** safely pokeable.
- **It's a cloud feature anyway.** The node uses **Agora RTC** for live video and uploads photos to
  `https://aiphotos.dreame.tech` / an OSS URL (`VM_UploadPhotoToServer`, `CAMERA_PQKL_IMG_UPLOAD`).
  Command vocab: `method`/`action`/`type`/`start`/`end`/`takephoto`/`monitor`/`open`/`upload`. The live
  feed is designed for cloud video-calls + cloud photo upload — **not a local raw stream**.

**Conclusion:** the onboard OV8856 cannot safely provide a local live feed. `avacmd streamer` is safe but
yields no idle frames; the videomonitor relay is cloud/Agora-oriented and crashes AVA when poked. **Use:**
1. **AI JPEG dumps** (`/data/ai_offline_collection`, read-only) — real frames during navigation. Zero risk.
2. **Dedicated camera on the Q6A** (3× MIPI CSI) — *the recommended path* for the rover's real-time vision;
   the robot's OV8856 stays with AVA for obstacle avoidance.

(`scripts/robot/vmread.c` = nanomsg-PAIR socket probe used for this RE.)

### Live video stream — why it can't be "mimicked from the cloud"
The live video is **not** carried over the control cloud Valetudo replaces (dummycloud/MIIO). It's a
*separate cloud RTC service*: `node_camera_streamer` HW-encodes H.264 (`/dev/cedar_dev` +
`libvencoder`/`libawh264`/`libOmxVenc`) and publishes via **`libagora-rtc-sdk.so`** to **Agora / Alibaba
Cloud Link Visual**, authenticated with the device triple in
`/mnt/private/ULI/factory/video_{device_name,device_secret,product_key}.txt` (+ `AliLicOps.sh` license).
The binary has **no local output** (no rtsp/udp/sdp/file targets — Agora only), and the feature is dormant
(libagora not loaded, 0 `videomonitor.socket` clients) until a cloud session starts.

So the cloud never *consumed* the stream — it brokered an Agora channel and the app subscribed via Agora's
servers. "Mimicking" it means becoming the Agora subscriber: provision AVA with your own Agora app-id/token
and consume with the Agora SDK over the internet — still cloud, heavy (credential reversal + Agora account).
Valetudo never implemented video and can't without re-brokering Agora.

**Only true LOCAL video path:** our own pipeline — siphon AVA's frames → `/dev/cedar_dev` HW encode → local
HTTP/RTSP. The libs (`libvencoder`, `libawh264`, …) and the encoder device are present. We must NOT *take* the
camera: reconfiguring the pipeline while AVA streams **deadlocks the kernel** (uninterruptible D-state, needs
reboot) — verified. The solution is to feed the encoder from the read-only siphon instead. **✅ DONE — see the
cedar MJPEG stream below.**

### ✅ WORKING: read-only frame siphon (`camsiphon.so`)
AVA continuously dequeues raw frames from `/dev/video2` (V4L2 mmap streaming, multi-plane, ~14 fps) for its
obstacle AI — even while docked. `scripts/robot/camsiphon.c` is a freestanding LD_PRELOAD lib (loaded into
AVA alongside `libfanoff_filter.so`) that hooks `open`/`openat`/`mmap`/`ioctl` and, on each `VIDIOC_DQBUF`,
**copies out the frame AVA just captured** — purely read-only (it calls the real syscalls and only *reads*
AVA's buffer). **AVA is completely unaffected** (verified: Valetudo 200, capturing normally; no crash/deadlock).

- Format discovered from AVA's `S_FMT`: **NV21, 672×504** (`/tmp/cam_fmt.txt`).
- On-demand grab: `touch /tmp/cam_grab` → next frame written to `/tmp/cam_frame.raw` (516096-byte buffer;
  NV21 payload 508032). Pull + convert (see `~/dreame-camera/`, NV21→PPM/PNG works).
- Loaded via the same `ava.sh` `LD_PRELOAD` bind-mount: `LD_PRELOAD="…/libfanoff_filter.so …/libcamsiphon.so"`.
- **Verified 2026-06-16**: produced a correct, current color frame of the room, zero AVA disruption.

**This is the recommended way to get the robot's camera locally** — no cloud, no ISP fight, no risk.

### ✅ WORKING: live MJPEG stream via the cedar HW encoder (`camstream`)
camsiphon also has a **continuous stream mode**: when `/tmp/cam_stream` exists it copies every frame into a
RAM-backed double-buffered ring (`/tmp/cam_stream.buf`, tmpfs — no flash writes, no AVA stall). `scripts/robot/
camstream.c` runs inside the Ubuntu chroot, mmaps the ring, and HW-JPEG-encodes each frame with the **Allwinner
CedarX encoder** (`/dev/cedar_dev`+`/dev/ion`, vendor libs at `/data/chroot/opt/venc/`), serving
`multipart/x-mixed-replace` over HTTP.

- **Run**: `sh /data/camstream.sh start` → `http://<robot-ip>:8090/` (browser / VLC / ffplay). `stop`/`status` too.
- **Verified 2026-06-17**: ~14 fps, NV21 672×504 → JPEG ~28 KB/frame, AVA undisturbed.
- Chroot is glibc 2.39, vendor libs are glibc 2.23 — they load fine (glibc is backward-compatible); the encoder
  binary just has to *run* inside the chroot. Launcher bind-mounts host `/tmp` into the chroot to share the ring.
- **CedarX encoder ABI** reverse-engineered in `scripts/robot/cedar_enc.c` (the standalone encode test/tool).
- **H.264/RTSP**: encoder produces valid IDR/P slices, but this CedarX build doesn't materialize SPS/PPS headers
  (getter index `0x501` has a NULL context buffer; generation is gated in `H264InitVer2`). Synthesized headers
  (`h264_headers.py`) don't match the encoder's internal SPS → decoder shows gray. MJPEG ships now; H.264 needs
  that gate cracked (likely a `VENC_IndexParamH264Param` profile-set). Vendor libs saved to `~/dreame-re/venc/`.

A **dedicated Q6A camera** is still better for high-rate/high-res rover vision, but the onboard feed now streams.

(Earlier "the onboard camera can't be tapped safely" conclusion is SUPERSEDED by camsiphon — the read-only
DQBUF tap + cedar HW-JPEG is the safe path; the cloud/Agora live-feed and the ISP-seizure routes remain dead ends.)

`scripts/robot/camera_stream.sh` (GStreamer off `/dev/video0`) and `scripts/robot/v4l2grab.c` remain
for if we ever set up an independent `/dev/video0` pipeline (`media-ctl` + `v4l-utils`/`gst`, sensor
freed) — but that contends with AVA for the sensor and risks the same instability.

---

## Microphone — none (definitively; both paths checked)

The D10s Pro has **no functional microphone**. The SoC supports two mic paths and both are dead here:

1. **Analog codec ADC** (`SUNXI-CODEC`, `hw:0,0`, `capture 1`). By default its capture controls are
   off (`ADC volume`, `MIC1/MIC2 gain` = 0, boost switches off), so `arecord` returns
   `read error: Input/output error`. **Enable them** (`amixer sset 'ADC volume'/'MIC1 gain volume'
   100%`, `'ADCL/ADCR Input MIC1/2 Boost' on`) and capture *succeeds* — but yields only a **flat DC
   floor (~235/32767, avg≈peak), which scales with gain and does NOT respond to a loud sound source
   in the room** (verified with a TV playing). That's ADC self-noise, not audio → **nothing wired to
   MIC1/MIC2**.
2. **Digital mic (DMIC)**. The SoC has a dedicated DMIC block (`dmic-controller@0x05095000`, machine
   `allwinner,sunxi-dmic-machine`, `sound@2`) — but it's **`status = "disabled"` in the device tree**,
   so it never registers as an ALSA card (only card 0 `audiocodec` exists; no DMIC in `dmesg`). The
   disabled node is the board's way of saying no DMIC chip is fitted.

So the robot is **speaker-out only** (voice prompts). NOTE: the speaker mute we use
(`PUT SpeakerVolumeControlCapability {"action":"set_volume","value":0}`) is the **playback** path
(`DAC/HpSpeaker/LINEOUT` controls) — entirely separate from capture; it does not affect (and is not
the cause of) the mic situation. The cloud video stack ships strings for two-way audio, but on this
model there's no mic to feed it. **For audio on the rover, add a USB mic on the Q6A companion**, not
the robot codec. (Only remaining robot-side avenue: flip the DMIC DT node to `okay` and reboot —
high risk, ~zero reward since no chip is fitted; don't.)

---

## IMU, odometry & motion — rich, readable

Everything streams from the MCU → SoC on **`/dev/ttyS4`** (`3c..3e` framed, Modbus CRC16 — see
`../CLAUDE.md` "MCU & LDS serial protocol"). Read it read-only with `strace` on AVA, decode with
`github.com/dimik/dreame_mcu_protocol/mcu_packets.py`, or forward a copy from the `fanoff_shim`. Packet set:

**2026-07-11: every known packet type is now decoded and published** (`mcu_node.py`) — nothing from the
MCU protocol is silently dropped anymore, including types we can't semantically interpret (they land on
`/mcu/unknown` as raw hex, logged once per distinct type+length so nothing goes unnoticed).

| Packet (type) | Rate | Data | ROS exposure |
|---|---|---|---|
| `Status10ms` (0x02) | 100 Hz | 3-axis **gyro** (°/s), 3-axis **accel**, L/R wheel-distance delta | `/imu/data`; `leftDis`/`rightDis` -> `/mcu/status10` (JSON) |
| `Status20ms` (0x01) | 50 Hz | **odometry** x, y, yaw + L/R wheel velocity, `edgeDis`, roller/side-brush current | `/odom/wheel`, `/cliff/edge_dist`; roller/side current -> `/mcu/status20` (JSON) |
| `Status100ms` (0x03) | 10 Hz | **pitch / roll**, wheel current, dust/water/HEPA/carpet present bits | `/mcu/status100` (JSON) + `/dustbin_missing` |
| `Triggers` (0x00) | event | bumpers, cliff/IR (front+rear, per-position), dock contact, per-motor over-current + error bits | `/cliff`, `/bumper`, `/cliff/front`, `/cliff/rear`, `/wheel_floating`, `/mcu/error` (any error/overcurrent bit), `/mcu/triggers` (full field dict, not just nonzero) |
| `BatteryStatus` (0x2B) | ? (existence on THIS hardware unconfirmed) | native voltage(V)/current(mA, **unsigned** — no direction bit)/temperature(°C)/charge_voltage(V)/SoC(%) | `/mcu/battery` (JSON) — untested whether this robot even emits it |
| `HwInfo` (0x29) | rare | mcu/imu/imu2/charge/app type IDs (static hardware identification) | `/mcu/hwinfo` (JSON, latched) |
| `McuFwVersionInfo` (0x07) | rare | MCU firmware git hash + version string | `/mcu/fw_version` (JSON, latched) |
| `PingMsg` (0x0F) | ? | MCU heartbeat/latency probe (we never reply Pong — read-only tap) | `/mcu/ping` (JSON) |
| `Status500ms` (0x05) | 2 Hz | sequence counter + RTC unix timestamp | `/mcu/status500` (JSON) |
| `ShutdownMsg` (0x10) | event | no data — occurrence itself signals a poweroff sequence in progress | `/mcu/shutdown_event` (latched, timestamp only) |
| `McuLog` (0x27) | event | 12 raw bytes, format undocumented (AVA itself just saves it to `/data/log/mculog.bin`) | `/mcu/log_raw` (hex string) |
| `FactoryTest` (0x04) | rare | `wifi_ssid_id` (only meaningful during factory testing) | `/mcu/factory_test` |
| `0x24` (undocumented upstream — "something connected with the battery temperature", 1 byte) | ~2 Hz | flag byte; **live-captured 2026-07-11 while charging: constant 0x00 (30 frames/15s)** — only the "OK" value observed, meaning of nonzero unconfirmed | `/mcu/battery_temp_flag` (Bool, best-effort — treat nonzero as a warning) |
| everything else (~11 type bytes: 0x06, 0x0B, 0x0D, 0x11-0x13, 0x20, 0x21, 0x23, 0x25, 0x26, 0x28, and any length mismatch on a known type) | — | **undecoded even in the reference RE repo** — no known field layout. Live-captured while charging: `0x12` (7B) is a monotonic timestamp/counter (changes every frame), `0x23` (5B) and `0x26` (2B, ~50Hz) were constant the whole capture (`40 00 00 00 00` / `07 01`) — neither looks battery-related | `/mcu/unknown` (raw type + hex payload) — surfaced, not dropped |

**BatteryStatus (0x2B) — CONFIRMED ABSENT on this hardware (2026-07-11), root cause investigated.**
Captured 90s of raw MCU traffic while the robot actively charged (35%->40%) — **zero 0x2B frames**, packet-
type histogram scaled perfectly proportionally between a 15s and a 90s run (same 9 types throughout, nothing
rare/new appearing) — rules out "just a slow periodic broadcast we missed," not just "untested."

**Root cause, now DECISIVELY narrowed down (2026-07-11, follow-up investigation):**
- PMIC is an **AXP806** (`dmesg`: "AXP20x variant AXP806 found", i2c 6-0036) — regulator/LDO-only, no fuel-
  gauge function. A dead end: explains why `axp803-battery-power-supply` never binds (wrong PMIC variant),
  but this chip has nothing to do with the actual charge IC (see below).
- **Found the real charge IC: `BQ24725` appears as a literal string in `/ava/bin/ava`** — matches the MCU
  protocol's own `HwInfo.charge_type` enum (`NONE/BQ24725/PWM`) exactly, so it's genuinely the hardware.
  BUT it has **no device-tree node at all** (`find /sys/firmware/devicetree/base -iname "*bq24*"` -> empty)
  — nothing for a kernel driver to bind to, from either side.
- **Decisive test: inspected AVA's actual open file descriptors** (`/proc/<ava-pid>/fd/`, not just a
  syscall-trace snapshot window that could miss a boot-time-only open). Result: `/dev/null`, `/dev/video2`,
  `/dev/ttyS4` (MCU), `/dev/ttyS3` (LiDAR), `/dev/media0` + v4l-subdevs (camera), `/dev/dri/renderD128`
  (GPU). **Zero I2C fds. No GPADC/input fd either** — this also rules out the SoC-GPADC-direct-read theory
  from the initial investigation (superseded).

**Conclusion: BQ24725 must be wired to the separate motor-control MCU (ttyS4), not the SoC — AVA has no
direct hardware path to it at all.** So there is nothing to "install a proper driver" for: not for AXP806
(wrong chip, no fuel-gauge silicon), not for BQ24725 (no devicetree node, and it's not even on a bus the
SoC's kernel can reach). Whatever `avacmd battery`/`charge_state` report is necessarily relayed from the MCU
over the same `3c..3e` serial protocol we already tap — `/mcu/battery`'s emptiness isn't a missing driver,
it's an unsolved piece of that SAME protocol (either a request/response exchange we haven't caught passively,
or the percentage is embedded in a field of a packet type we've mislabeled). Richer battery telemetry, if
ever pursued, is more MCU-protocol RE (same class of work as the Triggers bit-map), not kernel/driver work.
`avacmd charge_state`/`battery` (already used by `valetudo_bridge.py` for `/battery`) remain the sources
of truth.

Same capture also cross-validated two bits: `Triggers`' `dock_sta` read `1` for all 150 frames (robot
genuinely docked) and `Status100ms`'s `dust_container_missing` read `False` for all 150 frames (dustbin
genuinely installed) — both check out against physical reality. Also learned `Triggers` flows while just
docked/charging, not only during active manual control as the 2026-07-09 note assumed.

**Architectural note:** this is a read-only tap (we only decode what AVA already reads from the MCU); we
never craft outbound MCU frames ourselves — driving commands go through Valetudo's REST API to AVA, which
talks to the MCU on its own. So there's no equivalent "gap" on the outbound (`ToMcu_*`) side to close.

Live-decoded `Status10ms` frame (robot stationary, sitting flat):
```
gyro  raw x=-4 y=-18 z=16        -> ~0 (not rotating)
accel raw x=-243 y=38 z=16411    -> accel_z 16411/16384 = 1.002 g (gravity on Z, upright)
wheel dist delta L=0 R=0         -> not moving
```
(Status20ms/100ms byte layout differs slightly from the Z10 reference; gyro/accel scaling needs a
calibration pass, but structure + values are confirmed sane.)

**TODO — IMU calibration constants** (before using the IMU for nav/fusion on the Q6A):
- **Chip is a Bosch BMI055** (per `github.com/dimik/dreame_mcu_protocol/dreame_z10_notes.md`) — so scales come from
  its datasheet once the configured range is known; no blind solving needed.
- **r2250 sends raw BMI055 LSB** (our `accel_z=16411 ≈ 16384 LSB/g` = ±2 g range). This **differs from
  the Z10 repo's `Status10ms` decode** (`mcu_packets.py`: `gyro /= 100` → °/s, `accel /= 1000` → g,
  i.e. the Z10 firmware pre-scales to centi-°/s and milli-g). So don't reuse the Z10 divisors verbatim.
- Accel: ±2 g ⇒ **16384 LSB/g** (confirmed upright). Verify ±X/±Y by tilting.
- Gyro: confirm the BMI055 range register → datasheet **LSB/(°/s)** (e.g. ±2000°/s→16.4, ±125°/s→262.4);
  a slow known-rotation run only *validates* it. Then capture a ~30 s **static run** for the per-axis
  **zero-rate bias** (subtract it — else heading drifts).
- On-device helpers exist: `avacmd msg_cvt '{"type":"msgCvt","cmd":"calibration","value":"gyro20"}'`
  (also `gyro21`, `light1x/2x`) — Z10-documented; worth probing whether r2250 honors them.
- Record the final `{accel_scale, gyro_scale, gyro_bias[3]}` here and in the Q6A `imu_node` config.

IMU/compass ICs per the Z10 reference (D10s may differ): gyro XV7001, IMU BMI055, compass QMCX983.

**MCU IMU + odom tap status (2026-06-19): LIVE.** `libserialtap.so` tees AVA's ttyS4 reads into
`/tmp/mcu_ring.buf`; `mcu_node.py` publishes `/imu/data` + `/odom/wheel` (see `docs/ros.md`). The key
to AVA-safety was the **errno contract** — the first tap (`mcutap.c`) broke AVA's startup not because
interposing `read()` is unsafe, but because the freestanding shim returned raw `-errno` instead of
glibc's `-1`+`errno`, so AVA's non-blocking loops choked. Fixed via `__errno_location`; the tap also
uses a shm ring (memcpy, no per-frame syscall) instead of `mcutap`'s `sendto`. `mcutap.c` is superseded.

**IMU chip + scalings (D10s-MEASURED — the Z10's differ).** `HwInfo` (pkt 0x29) reports
`imu_type=2` (mcu_type=1, no 2nd IMU). The D10s sends **raw sensor LSB** (the Z10 firmware pre-scaled
to mg/centideg), so the Z10's `/1000`,`/100` are wrong here:
- **accel = raw/16384 g** (±2 g, 16-bit) — confirmed by gravity: `linear_acceleration.z = 9.807 m/s²`
  (= 1.000 g) at rest.
- **gyro = raw/65.536 °/s** (±500°/s) — confirmed by two spin cross-checks (`spin_calib.py`):
  `gyro_z` integral vs wheel-odom yaw gave 0.01526 / 0.01527 °/s·LSB in both rotation directions.

**Status10ms (IMU, 0x02) is emitted ONLY when the robot is ACTIVE** (moving/cleaning) — when
docked/idle the MCU sends only wheel odom (0x01), no IMU at all. So `/imu/data` publishes only during
activity, and a "hold still at startup" bias calibration can't work; `mcu_node` instead uses
**adaptive bias** (updates only while it detects the robot still, publishes always). `mcu_node` params
`gyro_scale`/`accel_scale`/`still_thresh_dps` allow re-tuning without a rebuild.

### Triggers (0x00) — full bit map (found 2026-07-11, re-cloning `dreame_mcu_protocol`)

The 2026-07-09 cliff work only inspected byte[0] ("bump") and byte[1] ("cliff") as single OR'd bytes, and
concluded byte[1] was **wheel-drop** because holding the robot at a real edge (wheels grounded) read 0.
Re-cloning `github.com/dimik/dreame_mcu_protocol` and reading its `Triggers` class (7-byte payload, exact
match) shows this was **mislabeled** — byte[1] is genuinely the forward/rear **drop-view IR cliff sensors**,
just never broken out per-position. Validated against our own captures + historical calibration values
(all consistent, see CHANGELOG 2026-07-11):

| Byte | Bits (LSB=0) | Field |
|---|---|---|
| 0 | 0-3 | `key1..4` (physical buttons, unused here) |
| 0 | 4, 5 | `left_bumper`, `right_bumper` — matches old "0x10 = bumper" |
| 0 | 6, 7 | `left_wheel_floating`, `right_wheel_floating` — explains old "0xc0/0xd0 under hard push": a hard shove can rock the chassis and briefly unweight the wheels, alongside the bumper hit |
| 1 | 0-3 | `d_view_lf`, `d_view_lmf`, `d_view_rmf`, `d_view_rf` — **the real front cliff-IR sensors** (matches old "0x0f when fully lifted" = all four; "0x08" = one corner) |
| 1 | 4, 5 | `d_view_lb`, `d_view_rb` — rear cliff-IR |
| 1 | 6, 7 | `mag_signal_left/right` — boundary-strip sensor, unused (no magnetic strips on this unit) |
| 2, 3 | — | `ir_dock_*` (3-bit signal code) + `ir_field_*` (bool) ×4 quadrants — dock-homing IR beacon channels; explains the rapid ambient flicker seen in raw captures, **unrelated to cliff** |
| 4 | 32-39 | `dock_sta`, `lds_button1/2`, `side/roll/pump_error` |
| 5, 6 | 40-55 | overcurrent + error bits (side/roll/fan/pump/wheels/lidar/vel/mag/imu/charge) |

`scripts/robot/mcu_node.py`'s `decode_triggers()` implements this (pure bit math, no dependency), publishing
`/cliff/front` (OR of the 4 front `d_view_*`), `/cliff/rear`, `/wheel_floating`, and the full nonzero-field
dict as JSON on `/mcu/triggers` — additive; the original `/cliff` (byte[1]!=0) and `/bumper` (byte[0]!=0)
keep their exact old semantics for the existing safety consumers (`cliff_guard.py`, `q6a_drive.py`).

**Open/unverified:** whether the front `d_view_*` sensors actually trip while the robot is held stationary
at a real edge (the 2026-07-09 test read 0 throughout — possibly because AVA only samples/transmits them
while actively driving, or the hold geometry didn't put the sensor windows themselves over the void). Needs
a live re-test with the robot **driving** toward a real edge, watching `/cliff/front` and the per-position
bits in `/mcu/triggers`.

**`edgeDis` (Status20ms, int16, mm) — decoded since day one but never published.** A CONTINUOUS reading
(50 Hz, not event-gated like Triggers), now published as `/cliff/edge_dist` (m). Meaning/threshold is
**untested** — could be a single dedicated distance-to-floor rangefinder (more useful for graded
edge-following than the boolean `d_view_*` bits), or something else entirely. Calibrate against a known
floor distance vs a real edge before trusting it.

---

## Battery / charging

The battery **level** works fine through Valetudo (updates correctly, e.g. 91→100% while charging),
but Valetudo's **charging FLAG is stuck at `none`** on the D10S Pro — even while actively charging.
That is a **Valetudo mapping gap** (its `DreameD10SProValetudoRobot` reads the level but never maps
the charging state for this firmware), **not** anything in our setup.

AVA itself reports the truth via the **`charge_state`** property (values `charging` / `not charging`):
```sh
avacmd msg_cvt '{"type":"msgCvt","cmd":"get_prop","prop":"charge_state"}'   # -> {"value":"charging","ret":"ok"}
avacmd msg_cvt '{"type":"msgCvt","cmd":"get_prop","prop":"battery"}'        # -> {"value":"100","ret":"ok"}
```
(AVA internally: `avanodemsgcvt::MsgBattery::SetBatteryLevel(int level, bool charging)`.) Note the
**D10S Pro does NOT emit the MCU `BatteryStatus` packet (0x2B)** the Z10 does, so charging can't be
read off the MCU serial stream — use `charge_state` via `avacmd`.

**Correct `/battery` in ROS:** a host poller (in `_root_postboot.sh`) writes `charge_state` to
`/tmp/charge_state` every 15 s; `valetudo_bridge.py` combines it with the Valetudo level and publishes
`sensor_msgs/BatteryState` on **`/battery`** (`POWER_SUPPLY_STATUS` CHARGING / FULL / DISCHARGING) —
bypassing Valetudo's broken flag. Republished by the bridge heartbeat (the attributes SSE only fires
on change, so docked+full would otherwise never publish).

---

## Recommended data plumbing for the rover (Radxa Q6A)

- **Map + pose** → Valetudo **REST/SSE** → `valetudo_bridge.py` → ROS `/map`, `/odom` + TF (live; `docs/ros.md`). Raw `/scan` not exposed by Valetudo — TBD.
- **Camera** → **DONE**: camsiphon (read-only `/dev/video2` tap) → camstream (cedar HW-JPEG, MJPEG
  :8090) → go2rtc (H.264 RTSP `:8554` / WebRTC `:1984`). See the camera-stream sections above.
  (The old "wire a `/dev/video0` stream via media-ctl+gst" idea is a **dead end** — reconfiguring
  video0 while AVA holds the sensor deadlocks the ISP/kernel — and is now superseded; don't.)
- **IMU / odometry / currents / triggers / battery** → tap the MCU stream on `ttyS4`
  (read-only strace or a small forwarder), decode with `mcu_packets.py`.
- **Mic** → not available.

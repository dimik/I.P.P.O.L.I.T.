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

A 360° spinning laser distance sensor on **`/dev/ttyS3` @ 230400 baud**, read by `node_lds.so`.
Frame format is **`55 aa` + 36 data bytes** (38 B total) — angle + distance points per rotation.
This is a *different* protocol from the MCU link and is what older notes mislabeled as the
"MCU 55 AA protocol".

The **two holes** in the turret are the laser **emitter (TX)** window and the **receiver/photodiode
(RX)** lens — standard triangulation-LiDAR layout (project a dot, read its offset reflection,
distance = triangulation).

Read path for the rover: AVA owns ttyS3 for SLAM, so consume the processed scan/map/pose via
the **Valetudo MQTT bridge** (`/scan`, `/map`, `/odom`) rather than tapping the raw serial.

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
ssh dreame-home 'cd /data/ai_offline_collection; ls -t *.jpg | head -1'   # newest
# pull (BusyBox has no sftp): base64 over ssh
ssh dreame-home 'base64 /data/ai_offline_collection/<file>.jpg' | base64 -d > frame.jpg
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

## Microphone — none

The ALSA codec (`SUNXI-CODEC`, `hw:0,0`) advertises a capture channel (`capture 1`), but recording
from it returns `read error: Input/output error` and an empty file — **no microphone is wired/routed**.
The robot is speaker-out only (voice prompts). Speaker mute: `PUT SpeakerVolumeControlCapability
{"action":"set_volume","value":0}`.

---

## IMU, odometry & motion — rich, readable

Everything streams from the MCU → SoC on **`/dev/ttyS4`** (`3c..3e` framed, Modbus CRC16 — see
`../CLAUDE.md` "MCU & LDS serial protocol"). Read it read-only with `strace` on AVA, decode with
`~/dreame_mcu_protocol/mcu_packets.py`, or forward a copy from the `fanoff_shim`. Packet set:

| Packet (type) | Rate | Data |
|---|---|---|
| `Status10ms` (0x02) | 100 Hz | 3-axis **gyro** (°/s), 3-axis **accel**, L/R wheel-distance delta |
| `Status20ms` (0x01) | 50 Hz | **odometry** x, y, yaw + L/R wheel velocity + roller/side-brush current |
| `Status100ms` (0x03) | 10 Hz | **pitch / roll**, wheel current, dust/water/HEPA/carpet present bits |
| `Triggers` (0x00) | event | bumpers, cliff/IR, dock contact, per-motor over-current + error bits |
| `BatteryStatus` (0x2b) | — | voltage, current, temperature, charge, SoC % |

Live-decoded `Status10ms` frame (robot stationary, sitting flat):
```
gyro  raw x=-4 y=-18 z=16        -> ~0 (not rotating)
accel raw x=-243 y=38 z=16411    -> accel_z 16411/16384 = 1.002 g (gravity on Z, upright)
wheel dist delta L=0 R=0         -> not moving
```
(Status20ms/100ms byte layout differs slightly from the Z10 reference; gyro/accel scaling needs a
calibration pass, but structure + values are confirmed sane.)

IMU/compass ICs per the Z10 reference (D10s may differ): gyro XV7001, IMU BMI055, compass QMCX983.

---

## Recommended data plumbing for the rover (Radxa Q6A)

- **LiDAR scan + map + pose** → Valetudo **MQTT** → ROS `/scan`, `/map`, `/odom`.
- **Camera** → for now, poll `/data/ai_offline_collection/*.jpg`; longer-term, wire a real
  `/dev/video0` stream (needs `media-ctl` + `gst`/`ffmpeg` installed, sensor freed).
- **IMU / odometry / currents / triggers / battery** → tap the MCU stream on `ttyS4`
  (read-only strace or a small forwarder), decode with `mcu_packets.py`.
- **Mic** → not available.

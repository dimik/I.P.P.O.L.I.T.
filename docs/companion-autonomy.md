# Companion Autonomy — architecture & decisions (robot-brain migration)

The Radxa Dragon **Q6A companion is the robot's full autonomy brain**: it runs all of ROS 2, ingests the
robot's sensors over the USB link, does perception (YOLO + MiDaS depth) + a semantic object map, and drives
the robot via Valetudo. The **Dreame D10s Pro robot runs no ROS** — only its AVA/Valetudo stack plus a few
ROS-free helpers we inject. This doc records the architecture and the key decisions behind it. Newest
decisions first; see `CHANGELOG.md` for the dated build log.

## Link
Robot ↔ Q6A over the **USB-gadget CDC-ECM** link: robot `usb0`=192.168.10.1, Q6A auto-DHCPs
`enx…`=192.168.10.2, ~0.68 ms, plug-and-play (robot boot hook brings up the gadget + a `usb0` dnsmasq).
From the Q6A: Valetudo REST at `http://192.168.10.1`, and `ssh robot-usb` (dreame key). See `docs/usb-gadget.md`.

## Where things run
| Runs on the **companion (Q6A)** | Runs on the **robot** |
|---|---|
| All ROS 2 Jazzy nodes (systemd services) | AVA + Valetudo (unchanged) |
| `valetudo-bridge` → `/map` `/odom` `/robot/status` `/battery` TF | LD_PRELOAD serial taps (`libserialtap.so` ttyS3, MCU tap ttyS4) → tmpfs rings |
| `mcu-node` → `/imu/data` `/odom/wheel` | `ring_forward.py` (ROS-free): tap rings → TCP (LDS :9901, MCU :9902) |
| `lds-scan-node` → `/scan` | `speak.py` (ROS-free): piper/espeak + ffmpeg → localhost mediad |
| `audio-bridge` (`/robot/speak` → robot `speak.py` over ssh) | camstream MJPEG `:8090` (OV8856 forward camera) |
| `q6a-vision` → `/vision/detections` + annotated `:8093` | mediad (audio, binds 127.0.0.1) |

Sensor data path: robot serial tap → tmpfs ring → `ring_forward.py` (TCP over USB) → companion ROS node.
The taps must run on the robot (they interpose AVA's serial); everything downstream is on the companion.

## Decisions

### D6 (2026-07-08) — Robot OV8856 camera, not the Q6A IMX296, for robot-perception
The companion's vision (YOLO + depth + object map) consumes the **robot's forward OV8856** via its MJPEG
(`:8090` over USB), not the Q6A's IMX296. **Why:** side-by-side frame grab — the OV8856 gives a clean
forward-facing room view (TV, table, chairs, bed, floor all visible) at robot height, exactly what object
recognition + obstacle avoidance need; the IMX296's FOV depends on how the board sits in the compartment and
it was delivering only noise (a post-reboot CAMSS bringup fault). **Bonus:** the robot already hands us JPEG,
so the Q6A skips the whole GPU-ISP/demosaic stack — vision is just decode → YOLO. The IMX296 pipeline stays
in-tree (it works when its overlay boots cleanly) but is off the critical path.

### D5 (2026-07-07) — Retire the on-device LLM; LLM needs go to cloud
`q6a-llmd` (Llama-3.2-1B on the NPU) is **disabled** (freed ~1.8 GB for the autonomy stack). The v68 NPU caps
at ~1B, which is too weak for agentic/robot reasoning (the offline agent was already dropped). LLM/agentic
work will use a **cloud model (planned: Cloudflare Workers AI)** via a thin companion client. Reversible.
See the `project_q6a_retire_llm` memory + `CHANGELOG`.

### D4 (2026-07-07) — ROS lives entirely on the companion; robot is ROS-free
ROS 2 Jazzy was **removed from the robot chroot** (147 MB) and all nodes relocated to the Q6A as systemd
services. The robot keeps only ROS-free helpers (`ring_forward.py`, `speak.py`) + the LD_PRELOAD taps. **Why:**
one place for the autonomy brain, robot stays lean, and the companion has the RAM/compute (esp. with the LLM
retired). Rings/serial stay on the robot (taps interpose AVA); a thin TCP forwarder bridges them.

### D3 — Drive via Valetudo GoTo/segments, not companion-side manual-control (for now)
High-level nav ("go to kitchen") resolves a room name → Valetudo **map segment** → `GoToLocation` /
segment-clean; **AVA path-plans and drives**. **Why:** the robot's `fanoff` gate **parks the LiDAR turret
during `HighResolutionManualControl`** — so companion Nav2 driving via manual-control move commands would
blind its own `/scan` (robot navigates + docks blind — hit 2026-06-19). Valetudo GoTo keeps the LiDAR
spinning and reuses AVA's proven navigation; the companion picks goals and adds obstacle awareness. Full
companion-side Nav2 driving is deferred until the LiDAR-gate interaction is solved.

### D2 — Reuse Valetudo's SLAM map; companion adds a local + semantic layer
AVA/Valetudo already SLAMs; the companion consumes its map + pose via `valetudo-bridge` (global) and adds a
**local costmap** (`/scan` + depth) for obstacle awareness — no duplicate SLAM (rejected as wasteful). A
**semantic object map** (D1) is layered on top.

### D1 — Semantic object map (planned)
Project `q6a-vision` detections (+ MiDaS metric depth for range + robot pose from the bridge) into the map
frame → a **persistent object layer** (table/chairs/bed/…) keyed to Valetudo's room **segments**, so the
robot "knows" what's in each room and supports queries like "go to the kitchen table".

## Roadmap (phases)
- **P1 ✅ ROS → companion**: bridge, `/scan`, `/imu`+`/odom`, audio all relocated; robot ROS removed.
- **P2 (in progress) perception**: ✅ camera decision + `q6a-vision` (robot cam → YOLO+ByteTrack → detections);
  next: MiDaS metric depth (fuse with `/scan`), then the semantic object map, then "go to kitchen".
- **P3 autonomy/power**: local costmap + goal-driving via Valetudo; brownout daemon; dock pass-through test.

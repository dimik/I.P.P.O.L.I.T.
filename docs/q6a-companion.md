# Q6A companion — status & findings (as of 2026-07-04)

Consolidated handoff for the Radxa Dragon Q6A (QCS6490) companion work. Deep details live in
`CLAUDE.md` and `scripts/companion/`; this is the map + the hard-won findings + how to resume.

---

## 1. Platform state

| Thing | State |
|---|---|
| **Odyssey** (Seeed X86J4125, x86_64, Ubuntu) | dev host; on home WiFi `192.168.1.150` |
| **Q6A** (QCS6490, aarch64, Radxa OS / Ubuntu 24.04, kernel 6.18 qcom) | companion; **powered ON / in use** (LLM daemon + IMX296 camera active) |
| **Wired link Odyssey↔Q6A** | direct cable, static `192.168.20.0/24` (Odyssey `.1` / Q6A `.2`), NM profiles `q6a-lan`/`odyssey-lan`, `never-default`. Gigabit, ~1.7 ms, ~107 MB/s. |
| **Q6A WiFi** | `radxa-dragon-q6a.local` on home LAN (DHCP ~192.168.1.243); mDNS was flaky — prefer the wired IP |
| **SSH** | from Odyssey: **`ssh ippolit-lan`** (wired, key `~/.ssh/id_ed25519_ippolit`) or `ssh ippolit` (wifi/mDNS fallback) |
| **Auto-services on Q6A** (autostart at boot) | `q6a-llmd` (LLM daemon), `llama-prewarm`, `docker`, wired link |

Robot (Dreame D10s Pro) was OFFLINE this whole session — no live robot tests done.

## 2. Networking (see `CLAUDE.md` Q6A access section)
- The original "LAN broken, no light" on the Odyssey was **not a driver issue** — the Intel I211 + `igb`
  were fine; it was the far-end/jack. Both Odyssey ethernet ports proven good at gigabit.
- Built the dedicated wired Odyssey↔Q6A link on `192.168.20.0/24` (see above). `never-default` so neither
  box's internet routing changes.

## 3. Power & thermal (the biggest gotchas)
- **GNOME auto-suspend** (headless image suspends ~20 min idle; SSH doesn't reset the timer) dropped the
  board off the network. **Fixed:** `systemctl mask sleep.target suspend.target hibernate.target
  hybrid-sleep.target` + locked GNOME dconf.
- **The Genie daemon's `"poll": true` busy-spun ~2.5 CPU cores 24/7** (HTP polling), pinning idle to ~90 °C
  → this pre-heated the board into a **110 °C thermal shutdown** during a GPU test. Root cause was the
  daemon, NOT cooling. **Fixed with `"poll": false`** (idle 247%→~5% CPU, ~90 °C→~66 °C, same latency).
- **True thermal envelope (daemon fixed, PRE-enclosure):** idle ~66 °C; sustained NPU 1B load peaks ~80 °C
  (10 °C under the 90 °C hot-trip, 30 °C under 110 °C critical). NPU is the coolest/most-efficient path.
- Passive cooling only (no fan); sustained GPU/CPU 3B compute WILL thermal-shut-down — needs active cooling.
- **Thermal enclosure installed 2026-07-12 — big improvement.** Steady-state reading with the FULL active
  autonomy stack running simultaneously (q6a-vision YOLO+MiDaS ~55% CPU, slam_toolbox, laser-odom, objmap,
  cliff-guard, announce, valetudo-bridge, mcu-node, lds-scan-node, audio-bridge — all 11 services active,
  9 min uptime): CPU cores 55-58 °C, GPU/NPU 50-52 °C, overall range 47-58 °C. That's COOLER than the old
  PRE-enclosure idle baseline (66 °C) while under active full-stack load — roughly 8-15 °C headroom gained
  vs the old idle point, 20 °C+ vs the old active-load point. Substantially more thermal margin now before
  the 90 °C hot-trip. LLM section above (retired, kept as reference) predates the enclosure.

## 4. ROS 2
- **ROS 2 Jazzy installed** on the Q6A (`ros-base` + Nav2), via `scripts/companion/install_ros2.sh`
  (hardened: sudo-correct, `universe`, real-user `.bashrc`). Not yet integrated with the robot.

## 5. Local LLM — the working setup (NPU via Genie)
**This is the usable on-device LLM today.** Setup: `scripts/companion/setup_npu_llm.sh`.
- **Llama 3.2 1B, Hexagon-v68 quantized**, run via Qualcomm **Genie** (`genie-t2t-run` / `libGenie.so`).
  Downloaded pre-compiled from Radxa (`radxa/Llama3.2-1B-4096-qairt-v68`) — no on-device compile.
- **Resident daemon** `q6a-llmd` (`scripts/companion/q6a_llmd.py`, Python+ctypes over libGenie): loads the
  model once, serves prompts over a Unix socket. It now runs a **from-source `libGenie.so` with an
  adaptive-spin threadpool patch at `poll:true`** → **0.00 idle cores + full speed** (no busy-spin, no 90 °C).
  Full build recipe/benchmark in `scripts/companion/qnn/PHASE3.md`. (The stock bundle libGenie still needs
  `poll:false`; ours supersedes it.)
- **Commands:** `q6a-llm "..."` (fast, ~0.47 s short prompts) on the board and on the Odyssey
  (`q6a-llm-remote`).
- **Speed:** ~9.8 tok/s decode (the real ceiling for a 1B on this bandwidth-bound chip, GPU or NPU).
  Short prompt latency ~0.45 s; `total ≈ 0.8 s + tokens/12` end-to-end from the Odyssey.
- The bare 1B **has no tools/internet and hallucinates facts** — that's inherent to a 1B. The offline
  MCP agent that tried to fix this was **dropped** (see §7); real agentic quality needs cloud/v73+.

## 6. Model-runtime map (what runs where, and why)
- **NPU (Hexagon v68)** runs ONLY pre-compiled v68 QNN context binaries — not arbitrary models. Only
  ≤1 B v68 LLM bundles exist for this chip. Genie exposes `poll:true/false` only.
- **Bigger LLMs can't go on the v68 NPU — confirmed ≤1B ceiling.** No `qai-hub-models` LLM recipe targets
  `qcs6490` (all v73+); Radxa prebuilds only ≤1B; aidevhome's 7B is a v73 build. Bigger → cloud or v73+ HW.
  Full evidence + the quantization/export pipeline in `docs/q6a-llm-toolchain.md`.
- **GPU (Adreno 643)** — the **Qualcomm OpenCL driver is `apt install qcom-adreno-cl1`** (from the enabled
  `ubuntu-qcom-iot` PPA) and works on the **stock mainline `msm` kernel** via dma-heap (NO KGSL swap —
  contradicts the "GPU unusable" lore). llama.cpp `-DGGML_OPENCL=ON` runs any GGUF. BUT: **1 B ≈ 11.7 tok/s
  = same as the NPU** (both bandwidth-bound on shared LPDDR5 — GPU is NOT faster), and **3 B full-offload
  thermal-crashes the board**. See `scripts/companion/gpu/setup_gpu_llm.sh`.
- **CPU (llama.cpp)** — any GGUF, ~few tok/s, offline, no key.
- **Cloud (free Gemini/Groq key)** — far more capable; the pragmatic "smarter brain" (needs online).
- **Decode is memory-bandwidth bound** — the "12 TOPS" NPU spec and "20-55 tok/s" community figures are
  irrelevant here; this chip's real ceiling is ~9.8 tok/s for a 1 B (GPU or NPU).

## 7. Offline MCP agent — TRIED AND DROPPED (2026-07-04)
Built an offline MCP agent (1B + `mcp_websearch.py` DuckDuckGo + `mcp_robot.py` Valetudo REST + a ReAct
`agent.py`) and **removed it** (code deleted from repo + Q6A). **The 1 B is too weak to be a reliable
agent** — even with heavy scaffolding (2-stage tool selection, arg-prefill, forced stop) it hallucinates
tool names, empties args, defaults to tool #1, and confabulates facts (got the Nvidia CEO wrong, parroted
result titles).
- **Durable lesson:** the on-device 1B is a fast, well-scoped *text generator*, not an agent. Since v68
  caps the NPU at ~1B (§6 — no 3B+ path on this chip), agentic reasoning must run on **cloud (Gemini/Groq
  free tier)** or a **v73+ board**. For the ROS head, use event-driven *narrow* handlers, not free tool choice.

## 8. Adaptive polling — SOLVED via from-source Genie (`scripts/companion/qnn/PHASE3.md`)
Goal was adaptive polling (spin during decode, idle cool between) without Genie's 24/7 busy-spin.
- **RESOLVED (2026-07-04):** the QAIRT SDK ships **full Genie source** (`examples/Genie/Genie`) — it is NOT
  closed. Genie's `poll` is its worker-**threadpool** busy-spin. We patched `threadpool.cpp` for **time-based
  adaptive spinning**, built `libGenie.so` from source (`-O3`), and **deployed it into `q6a-llmd`** → 0.00
  idle cores + ~9.8 tok/s. This *supersedes* the earlier QNN-direct plan.
- **Abandoned (superseded):** the QNN-direct runtime via `dimik/qai-appbuilder` (Phase 3 in old notes). Its
  only purpose was adaptive polling; the decode path hit a DMA/KV-residency blocker and is no longer needed.
  Recon kept in `scripts/companion/qnn/` for reference.
- **Deep toolchain reference** (QNN/QAIRT/AIMET, LLM quantization+export, distribution, the v68 ≤1B ceiling):
  `docs/q6a-llm-toolchain.md`.

## 8b. Camera — Sony IMX296 global shutter (WORKING) → `docs/q6a-camera.md`
IMX296 GS camera captures live frames on the Q6A (CAMSS). Built the missing `imx296.ko` + a corrected overlay
that **fixes the boot-loop that blocks Radxa's own camera overlays** (strip the `linux,cma` fragment).
CAM2 live-verified; CAM3 overlay committed + validated. Setup: `scripts/companion/camera/{build,deploy}_imx296.sh [2|3]`.

## 9. Command cheat-sheet
```
ssh ippolit-lan                      # Odyssey -> Q6A (wired)
q6a-llm  "one sentence: ..."         # fast local LLM (Genie/NPU adaptive libGenie), no tools
systemctl status q6a-llmd            # the LLM daemon (from-source adaptive libGenie, poll:true, cool)
# thermal:  for z in /sys/class/thermal/thermal_zone*; do echo $(cat $z/type)=$(($(cat $z/temp)/1000))C; done
```

## 10. Open items / resume points
- **LLM adaptive polling: DONE** (from-source Genie, deployed). **v68 LLM ceiling: ~1B, confirmed** — bigger
  models need cloud or v73+ HW (see `docs/q6a-llm-toolchain.md`). No open QNN work.
- **Camera**: CAM2 IMX296 live; **CAM3** ready (`build/deploy_imx296.sh 3`) — live-verify when the 2nd cam is in.
- **Robot integration**: relocate `valetudo_bridge.py` to the Q6A; ROS `behavior_node` as the event-driven
  head (subscribe /battery,/status,/detections,/voice → LLM → Nav2/speak). Robot was offline all session.
- **Mic**: no onboard mic — add a **USB mic** (ReSpeaker USB array for far-field, or a cheap USB mic to
  prototype) for STT → the LLM. Avoid I2S mics (INMP441) — hard on the QCS6490 LPASS.
- **Smarter brain**: free Gemini/Groq key (online) or CPU-3B (offline, slow); the NPU 1 B is the efficient
  on-device default.

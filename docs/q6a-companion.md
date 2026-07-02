# Q6A companion — status & findings (as of 2026-07-03)

Consolidated handoff for the Radxa Dragon Q6A (QCS6490) companion work. Deep details live in
`CLAUDE.md` and `scripts/companion/`; this is the map + the hard-won findings + how to resume.

---

## 1. Platform state

| Thing | State |
|---|---|
| **Odyssey** (Seeed X86J4125, x86_64, Ubuntu) | dev host; on home WiFi `192.168.1.150` |
| **Q6A** (QCS6490, aarch64, Radxa OS / Ubuntu 24.04, kernel 6.18 qcom) | companion; **currently powered OFF** (shut down end of this session) |
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
- **True thermal envelope (daemon fixed):** idle ~66 °C; sustained NPU 1B load peaks ~80 °C (10 °C under
  the 90 °C hot-trip, 30 °C under 110 °C critical). NPU is the coolest/most-efficient path.
- Passive cooling only (no fan); sustained GPU/CPU 3B compute WILL thermal-shut-down — needs active cooling.

## 4. ROS 2
- **ROS 2 Jazzy installed** on the Q6A (`ros-base` + Nav2), via `scripts/companion/install_ros2.sh`
  (hardened: sudo-correct, `universe`, real-user `.bashrc`). Not yet integrated with the robot.

## 5. Local LLM — the working setup (NPU via Genie)
**This is the usable on-device LLM today.** Setup: `scripts/companion/setup_npu_llm.sh`.
- **Llama 3.2 1B, Hexagon-v68 quantized**, run via Qualcomm **Genie** (`genie-t2t-run` / `libGenie.so`).
  Downloaded pre-compiled from Radxa (`radxa/Llama3.2-1B-4096-qairt-v68`) — no on-device compile.
- **Resident daemon** `q6a-llmd` (`scripts/companion/q6a_llmd.py`, Python+ctypes over libGenie): loads the
  model once, serves prompts over a Unix socket. **`poll:false` is mandatory** (see §3).
- **Commands:** `q6a-llm "..."` (fast, ~0.47 s short prompts) on the board and on the Odyssey
  (`q6a-llm-remote`); `q6a-agent "..."` = tool-using agent (see §7).
- **Speed:** ~9.6 tok/s generation with poll:false (~12 with poll:true, but that busy-spins/overheats).
  Short prompt latency ~0.45 s; `total ≈ 0.8 s + tokens/12` end-to-end from the Odyssey.
- The **bare `q6a-llm` has no tools/internet** → hallucinates facts; that's expected (use `q6a-agent`).

## 6. Model-runtime map (what runs where, and why)
- **NPU (Hexagon v68)** runs ONLY pre-compiled v68 QNN context binaries — not arbitrary models. Only
  ≤1 B v68 LLM bundles exist for this chip. Genie exposes `poll:true/false` only.
- **Bigger/newer models can't easily go on the NPU:** the modern LLM→QNN compile pipeline needs **v73+**;
  the QCS6490 is **v68** (2021-gen silicon in a 2025 board). Quantizing 3 B to v68 is possible in principle
  but hard/uncertain and needs a beefy x86 host (not the Odyssey).
- **GPU (Adreno 643)** — the **Qualcomm OpenCL driver is `apt install qcom-adreno-cl1`** (from the enabled
  `ubuntu-qcom-iot` PPA) and works on the **stock mainline `msm` kernel** via dma-heap (NO KGSL swap —
  contradicts the "GPU unusable" lore). llama.cpp `-DGGML_OPENCL=ON` runs any GGUF. BUT: **1 B ≈ 11.7 tok/s
  = same as the NPU** (both bandwidth-bound on shared LPDDR5 — GPU is NOT faster), and **3 B full-offload
  thermal-crashes the board**. See `scripts/companion/gpu/setup_gpu_llm.sh`.
- **CPU (llama.cpp)** — any GGUF, ~few tok/s, offline, no key.
- **Cloud (free Gemini/Groq key)** — far more capable; the pragmatic "smarter brain" (needs online).
- **Decode is memory-bandwidth bound** — the "12 TOPS" NPU spec and "20-55 tok/s" community figures are
  irrelevant here; this chip's real ceiling is ~12 tok/s for a 1 B (GPU or NPU).

## 7. Offline MCP agent (`scripts/companion/agent/`)
- The local 1 B drives 2 real MCP servers — `mcp_websearch.py` (DuckDuckGo) + `mcp_robot.py` (Valetudo REST)
  — via `agent.py` (ReAct loop, MCP client). `q6a-agent "..."` (board + Odyssey `q6a-agent-remote`).
- **The 1 B is a weak agent** — needs heavy scaffolding (2-stage tool selection by number → inject valid
  name; arg-prefill; stop-on-first-observation). Works for narrow single-tool tasks; unreliable for
  open-ended reasoning (got the Nvidia CEO wrong, parroted result titles). For real agentic quality → cloud.

## 8. QNN-direct via QAI AppBuilder — bypassing Genie (`scripts/companion/qnn/`)
Goal: own the inference path + the HTP perf infra (**adaptive polling**), which Genie doesn't expose.
- **SDK without a Qualcomm account** (their SDK needs company verification — blocked): extracted QAIRT 2.42
  from Radxa's **`radxazifeng278/qairt-npu-v68`** docker image → `~/qairt_2.42.0.251225` (path MUST contain
  the version string). Built the aarch64/v68 `qai_appbuilder` wheel from the `dimik/qai-appbuilder` fork.
- **Adaptive polling WORKS on v68** (Phase 2): `setPowerConfig(RPC_POLLING_TIME)` + `ADAPTIVE_POLLING_TIME`
  both return rc=0 — **despite the SDK header saying "v69+".** Patch in the fork's `QnnInferenceEngine.cpp`.
- **CANNOT bolt adaptive polling onto Genie** (tested, refuted): Genie is closed-source; you can't create a
  2nd QNN HTP backend in its process (DSP transport conflict) or reach its own backend. Adaptive polling for
  the LLM only comes via the QNN-direct runtime (Phase 3).
- **Phase 3 — QNN-direct LLM runtime** (`qnn_llm.py`, spec in `PHASE3.md`):
  - Binary has 2 graphs: idx0 `ar128_cl4096` (prefill), idx1 `ar1_cl4096` (decode); driven via
    `QNNContext.Inference(inputs, "burst", graphIndex)`.
  - **Prefill PROVEN CORRECT** (`qnn_prefill_test.py`): "The capital of France is" → " Paris" (top-1). All
    conventions validated (llama3 RoPE θ=500000+scaling, causal mask `-100/0` with `[past,current]` layout,
    by-name KV remap, qai float→quant, greedy).
  - **BLOCKER:** repeated decode fails — 1st decode step OK, 2nd → `Dma execution failed on the skel side
    result=1100` (re-DMA'ing 67 M-element KV tensors per token). **Fix = ShareMemory-backed persistent KV
    buffers** (keep KV resident on the DSP, update in place). That's the next task.

## 9. Command cheat-sheet
```
ssh ippolit-lan                      # Odyssey -> Q6A (wired)
q6a-llm  "one sentence: ..."         # fast local LLM (Genie/NPU), no tools
q6a-agent "who won the 2022 WC?"     # tool-using agent (web + robot), slower
systemctl status q6a-llmd            # the LLM daemon (poll:false)
# thermal:  for z in /sys/class/thermal/thermal_zone*; do echo $(cat $z/type)=$(($(cat $z/temp)/1000))C; done
```

## 10. Open items / resume points
- **Phase 3 decode**: rework `qnn_llm.py` decode KV to qai_appbuilder **ShareMemory** (the one blocker to a
  working Genie-free runtime with adaptive polling → ~12 tok/s cool). Everything else is proven.
- **Robot integration**: relocate `valetudo_bridge.py` to the Q6A; ROS `behavior_node` as the event-driven
  head (subscribe /battery,/status,/detections,/voice → LLM → Nav2/speak). Robot was offline all session.
- **Mic**: no onboard mic — add a **USB mic** (ReSpeaker USB array for far-field, or a cheap USB mic to
  prototype) for STT → the LLM. Avoid I2S mics (INMP441) — hard on the QCS6490 LPASS.
- **Smarter brain**: free Gemini/Groq key (online) or CPU-3B (offline, slow); the NPU 1 B is the efficient
  on-device default.

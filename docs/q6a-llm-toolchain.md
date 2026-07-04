# Q6A LLM/NPU toolchain — QNN, QAIRT, AIMET, quantization & export (deep reference)

Everything we learned getting LLMs onto the Radxa Dragon Q6A's **Hexagon v68** NPU (QCS6490). Read this
before attempting any new model, quantization, or SDK work — it will save days. Companion docs:
`scripts/companion/qnn/PHASE3.md` (the adaptive-polling Genie build) and `docs/q6a-camera.md` (CAMSS/camera).

---

## 0. TL;DR / decision matrix — "how do I run model X on the Q6A?"

| Goal | Verdict | Path |
|---|---|---|
| Small LLM (≤1B) on NPU | ✅ works today | Radxa prebuilt v68 Genie bundle (ModelScope) + our adaptive `q6a-llmd` |
| 3B/7B LLM on the **v68 NPU** | ❌ not feasible | v68 tops out at ~1B (see §5). Use cloud or v73+ HW. |
| Quantize/convert a **CNN/ONNX** for v68 | ✅ | QAIRT `qairt-converter`+`qairt-quantizer` on an **AVX2 x86** host (NOT the Odyssey) → `qnn-context-binary-generator` for v68 |
| Quantize a **new LLM** for v68 | ⚠️ hard/cloud | AIMET/AI Hub does the quantize+split; then compile for `qcs6490`. Heavy; needs GPU/cloud. |
| "Bigger brain" for the robot | ✅ | Cloud LLM (Gemini/Groq free tier) — the on-device 1B is a scoped text gen, not an agent |

---

## 1. The stack (what sits on what)

- **QAIRT** (Qualcomm AI Runtime) = the umbrella SDK. Contains **QNN**, **Genie**, **SNPE**, and the host
  tools (converters, quantizer, context-binary generator). Version on the Q6A: **2.42.0.251225** at
  `/home/radxa/qairt_2.42.0.251225`.
- **QNN** = the mid-level C API to the Hexagon HTP (tensor graphs, context binaries, power configs).
- **Genie** = the high-level **LLM runtime** built on QNN (tokenizer, KV-cache residency, prefill/decode,
  sampling, chat templating). **Its full source ships in the SDK** (`examples/Genie/Genie`) and is buildable.
- **CAMSS** = the camera V4L2 subsystem (unrelated to LLMs — see `docs/q6a-camera.md`).
- **AIMET** = the offline model-quantization toolkit (PTQ/QAT); separate from QAIRT, used before conversion.

Silicon reality: **QCS6490 = Hexagon v68 (2021-gen)**. Qualcomm's modern LLM tooling targets **v73+**
(Snapdragon 8 Gen 2/3, X Elite, QCS9075). This single fact drives most of the limits below.

---

## 2. QNN + Genie runtime findings

### Adaptive polling (SOLVED — full writeup in `scripts/companion/qnn/PHASE3.md`)
- Genie's `poll:true/false` config is its **worker-threadpool busy-spin**, NOT the QNN HTP RPC power config.
  `poll:true` spins ~2.5 CPU cores 24/7 (~90 °C on the passively-cooled board); `poll:false` blocks (cool,
  slight wake latency, ~9.5 tok/s).
- We built `libGenie.so` **from the SDK source** with a **time-based adaptive-spin threadpool patch** (spin
  100 ms after last work, then block on the condvar) at `-O3`, and deployed it into `q6a-llmd` at
  `poll:true` → **0.00 idle cores + ~9.8 tok/s decode**. This is the current production daemon.
- Dead end we ruled out: injecting the QNN `setPowerConfig(ADAPTIVE_POLLING_TIME)` into Genie's process
  (a 2nd HTP backend fails — Genie owns the DSP/FastRPC session). Genie being "closed" was **wrong** — it's
  buildable, which is what unlocked this.

### QNN context binaries are BOTH version- and arch-specific (this bites hard — err 5005)
A `.serialized.bin` / `.dtbo`-style context binary embeds a **QNN build version** and a **dspArch**. Loading
an incompatible one fails with **`Could not create context from binary ... err 5005`** (after host allocation
succeeds). Verified:
- **Version window is narrow.** A **v2.40** binary loads on the 2.42 runtime; a **v2.29** binary does **not**
  (13 versions apart). Match the runtime to the binary's era (or rebuild the binary).
- **Arch must match exactly.** A **dspArch=73** (v73 / X-Elite, socModel 60 = SC8380) binary will **never**
  load on our **v68** — no runtime version fixes an arch mismatch. Inspect any binary with
  `qnn-context-binary-utility --context_binary X --json_file out.json` (aarch64 build is in the SDK bin dir)
  → read `info/contextMetadata/info/dspArch`, `info/socModel`, `info/coreApiVersion`. Or quick: `strings X | grep -E "v2\.[0-9]+\.[0-9]+"`.

### cdsp wedging / recovery
Unclean fastrpc-client exits (`kill -9`, crash, restart-storm) orphan a cDSP process-domain → `err 5005` /
`remote_munmap64 failed` / hangs for the next client. The sysfs remoteproc SSR (`echo stop >
/sys/class/remoteproc/remoteproc1/state`) **hangs on this kernel** — recovery = **reboot**. Prevent it:
clean `systemctl stop`, one NPU client at a time, never `-9`. (Details in `PHASE3.md`.)

---

## 3. QAIRT SDK — obtaining it + the host toolchain

### Distribution channels (the Qualcomm portal gate is avoidable)
- ❌ The classic Qualcomm portal / QPM requires **company verification** (blocked for us).
- ✅ **"Qualcomm AI Runtime Community"** is a **direct public download**, no gate:
  `https://softwarecenter.qualcomm.com/api/download/software/sdks/Qualcomm_AI_Runtime_Community/All/<ver>/v<ver>.zip`
  (HEAD is 403; a ranged GET works — verified 2.34.0.250424, 2.36.0.250627, 2.40.0.251030 all downloadable).
- ✅ **Radxa docker image** `radxazifeng278/qairt-npu-v68` bundles the whole 2.42 v68 stack.
- ✅ Already **pre-installed** on the Q6A at `~/qairt_2.42.0.251225`.
- **`qai-appbuilder`** (`quic/ai-engine-direct-helper`, `pip install qai-appbuilder`) is the *runtime* Python
  wrapper only — it does NOT contain the converter/quantizer host tools (it points you to QAI Hub for quant).

### The x86 host quantization/conversion toolchain (present in the SDK)
`bin/x86_64-linux-clang/` (Python front-ends over `lib/python/{qti,qairt,snpe}` + `lib/x86_64-linux-clang/*.so`):
- `qairt-converter` + `qairt-quantizer` (modern unified PTQ path)
- `qnn-onnx-converter` / `qnn-pytorch-converter` / `qnn-tensorflow-converter` / `qnn-tflite-converter`
- `qnn-model-lib-generator`, `qnn-context-binary-generator` (compile a DLC → HTP context binary for a `dsp_arch`)
- `qnn-genai-transformer-composer` (the LLM/transformer export path), `snpe-dlc-quantize`, LoRA tools

### Standing it up on the Odyssey (x86) — and the AVX2 WALL ⚠️
We built an isolated env on the Seeed Odyssey (`~/qairt-x86/`):
- `uv` → **Python 3.10.20** venv (SDK supports 3.8/3.10 only; the Odyssey's system Python 3.14 is too new).
- Copied the SDK's x86 subset (~1 GB); resolved the native lib chain **without root** by `apt-get download`
  + `dpkg-deb -x` of `libc++1`, `libc++abi1`, `llvm-libunwind1` (LLVM `libunwind.so.1`, NOT the nongnu
  `libunwind8`), plus uv's `libpython3.10.so.1.0` → all on `LD_LIBRARY_PATH`.
- **Version pinning gotcha:** `onnx==1.16.2` + `protobuf==4.25.3`. onnx ≥1.18 removed `onnx.mapping` (the SDK
  imports it → silent `onnx=None` → `AttributeProto` errors); onnx <1.15 is too old. protobuf must satisfy both.
- ✅ **`qairt-converter` works** (ONNX→DLC verified on a toy model).
- ❌ **`qairt-quantizer` SIGILLs** — `Illegal instruction`. Root cause: the calibration backend `libQnnCpu.so`
  is packed with **AVX2** (147k AVX/YMM instrs), but the Odyssey's **Celeron J4125 has only SSE4.1/4.2** — no
  AVX. Not fixable in software. **Quantization must run on an AVX2 box, the Q6A's ARM cores, or QAI Hub.**

---

## 4. AIMET (AI Model Efficiency Toolkit)

- Role: offline **quantization** (PTQ: CLE, AdaRound, bias-correction; QAT) — needed for good W4A16/W8A8,
  especially LLMs where naïve rounding collapses quality. Runs before QAIRT conversion.
- **Two flavors:**
  - **AIMET Pro** — `aimetpro-release-<ver>.torch-<backend>-release.tar.gz`, distributed via QPM (gated).
    The SDK's `bin/aimet_env_setup.sh` expects exactly this tarball (min version 1.30) and builds a venv.
  - **OSS AIMET** — `quic/aimet`, free pip wheels (`aimet-torch`, `aimet-onnx`). This is the "download and
    use" one. GPU wheels need CUDA; **CPU-only on the Odyssey** (no NVIDIA GPU) → slow, and can't hold a large
    LLM in 7 GB RAM anyway.
- For LLMs, Qualcomm's blessed path is **AI Hub / `qai-hub-models`** (which orchestrates the quantize+export
  on their hardware) rather than hand-driving AIMET locally.

---

## 5. LLM quantization & export — the real pipeline, and the v68 ceiling

### "Quantize + export an LLM" is NOT the CNN two-step
For a CNN, `qairt-converter` + `qairt-quantizer` is minutes. For an LLM it's a specialized pipeline:
1. **Quantize to W4A16** — 4-bit weights need calibration (AWQ/GPTQ-style), needing the **full fp16 model in
   RAM** (~14 GB for 7B, ~6 GB for 3B) + compute. ← this is the wall.
2. **Graph-split** — a big model doesn't fit one HTP context on v68 (limited VTCM), so it's split into N
   weight-sharing sub-graphs (e.g. a 7B = 5× `model-N.bin`).
3. **Compile each split → a per-arch context binary** (`qnn-context-binary-generator`, `dsp_arch=v68`).
4. **Author the Genie config** (`ssd-q1`/context config, KV cache, `rope-theta`, tokenizer, prompt format).

Steps 2–4 we can do. **Step 1 is the wall** — it needs a **GPU / big-RAM x86** box. Neither the Odyssey
(no AVX2, 7 GB) nor the Q6A (11 GB, ARM) can quantize a 3B/7B. That's exactly why **QAI Hub** (cloud) exists.

### qai-hub-models (the recipe library) + QAI Hub (cloud compiler)
- Export command shape:
  `python -m qai_hub_models.models.<model>.export --chipset qualcomm-qcs6490 --target-runtime qnn_context_binary --quantize w4a16`
  → AI Hub does the quantize+split+compile **on real Snapdragon HW in the cloud** and returns artifacts
  compiled against a **current** QNN version (so no 2.29-vs-2.42 or arch headaches).
- **QAI Hub** needs a free account + API token (`qai-hub configure --api_token …`; config at
  `~/.qai_hub/client.ini`). It's a *different, ungated* signup from the SDK portal. QCS6490 is a valid
  target ("Dragonwing RB3 Gen 2", chipset id `qualcomm-qcs6490`). Token was configured this session.
- **BUT — the v68 LLM ceiling (definitive):** *no* `qai-hub-models` LLM recipe lists `qcs6490` as a supported
  chipset. `llama_v3_2_3b_instruct`, `qwen2_7b_instruct`, `phi_3_5_mini`, `ministral_3_3b`, `gemma_4_e2b`,
  `qwen3_0_6b` — all target v73+ only (`snapdragon-8-elite(-gen5)`, `x-elite`, `x2-elite`, `sa8775p`,
  `qcs9075`). There is **no Qwen2.5-3B recipe at all**. So AI Hub won't hand you a v68 3B out of the box.

### Definitive evidence the QCS6490/v68 tops out at ~1B on-device
| Source | Biggest LLM for v68 |
|---|---|
| Radxa prebuilts (ModelScope) | **1B** (Llama-3.2-1B) / 0.5B (Qwen2.5-0.5B) — Radxa's own custom v68 builds |
| aidevhome (Radxa model host) | 7B exists — but it's a **v73** build (SC8380 X-Elite), won't load on v68 |
| qai-hub-models — any LLM recipe | **none** target qcs6490 (all v73+) |

Bigger LLMs need **v73+ hardware** (e.g. Radxa Airbox / QCS8550-class) or **cloud**.

### Model formats & the "download and run" ecosystem
- Genie runs **`weight_sharing_model_N_of_N.serialized.bin`** (multi-part, weight-shared) + `tokenizer.json`
  + a Genie `config.json` (dialog type, e.g. `ssd-q1` for speculative decoding; context/sampler/engine).
- **`DLC2BIN` / `ONNX2BIN`** (in `ai-engine-direct-helper`, radxa-dev fork) convert a DLC/ONNX → the
  platform-optimized Genie `.bin`.
- **`GenieAPIService`** (same fork) is an OpenAI-compatible server over Genie — a nicer front-end than our
  raw-socket `q6a-llmd`, and the route to bigger models on v73+ boards.
- Prebuilt v68 models: Radxa **ModelScope** (`radxa/Llama3.2-1B-4096-qairt-v68`, `radxa/Qwen2.5-0.5B-v68`).

---

## 6. What we actually run today
- **On the Q6A NPU:** Llama-3.2-1B via the from-source **adaptive** `libGenie.so` in `q6a-llmd` (poll:true,
  0 idle cores, ~9.8 tok/s). Client: `q6a-llm "…"` (local) / from the Odyssey (`q6a-llm-remote`).
- **The offline MCP agent was dropped** — the 1B hallucinates as an agent. Agentic reasoning → cloud/v73+.
- **On the Odyssey (x86):** the QAIRT converter works; the quantizer doesn't (AVX2). Useful as a
  conversion / model-prep host; real quantization → AVX2 box / QAI Hub.

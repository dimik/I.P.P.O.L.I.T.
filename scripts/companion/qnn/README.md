# Direct QNN on the Q6A via QAI AppBuilder (bypassing Genie)

Runtime for running QNN **v68** context binaries directly (no Genie), to control the HTP perf
infrastructure (**adaptive polling**) and own the inference path. Build: `setup_qai_appbuilder.sh`.
Smoke/recon: `qab_smoke.py`.

## Why (2026-07-02)
- Genie's `"poll": true` busy-spins ~2.5 CPU cores at idle (→ ~90 °C on this passive board); `poll:false`
  fixes the heat but costs ~20 % tok/s (12 → 9.6), and **adaptive polling is not reachable through Genie**
  (it's a `QnnHtpPerfInfrastructure` C-API power config). Going QNN-direct exposes it.
- `QnnInferenceEngine.cpp` (fork) already calls `perfInfra.setPowerConfig(... DCVS_V3 ...)` — the exact hook
  to add `QNN_HTP_PERF_INFRASTRUCTURE_POWER_CONFIGOPTION_ADAPTIVE_POLLING_TIME` (**Phase 2**, not yet done).

## SDK access — no Qualcomm account
Qualcomm's "free" QAIRT SDK requires **company verification** (blocked). Workaround: the QAIRT 2.42 SDK is
extracted from Radxa's **`radxazifeng278/qairt-npu-v68`** docker image (arm64, ~3.9 GB) → `~/qairt_2.42.0.251225`.
Gotcha: `QNN_SDK_ROOT` path **must contain the version string** (setup.py parses it).

## Status
- ✅ aarch64/v68 `qai_appbuilder` wheel built from source & installed; bundles `libQnnHtpV68Skel/Stub.so`.
- ✅ **QNN-direct proven**: `QNNContext(...)` loads the Llama-3.2-1B v68 context binary onto the NPU (HTP),
  no Genie, ~3 s `model_initialize`. API: `QNNConfig.Config(Runtime.HTP, ...)`, `QNNContext(name, bin)`,
  `PerfProfile.SetPerfProfileGlobal(...)`, `.Inference(...)`.

## Llama-3.2-1B v68 context-binary structure (Phase-3 recon)
Graph **`ar128_cl4096_1_of_1`** — chunked prefill (128-token chunk, 4096 context):
- **Inputs (36):** `input_ids`[1,128] int32; per-layer KV cache ×16 → `past_key_N_in`[8,1,64,3968] +
  `past_value_N_in`[8,1,3968,64] (ufp8); RoPE `position_ids_cos`/`_sin`[1,1,128,32]; `attention_mask`[1,1,128,4096].
- **Outputs (33):** per-layer updated KV ×16 (`past_*_out` [8,1,128,64]/[8,1,64,128]); `logits`[1,128,128256].
- **Model params:** 16 layers, 8 KV heads (GQA), head_dim 64, ctx 4096, chunk 128, vocab 128256, **ufp8 KV cache**.

## To build a QNN-direct LLM runtime (Phase 3 — not done)
Feed input_ids + KV + RoPE cos/sin + causal mask through the graph; ring-manage the **ufp8** KV cache across
128-token prefill chunks, then autoregressive decode; tokenize with the bundle's `tokenizer.json` (BPE);
sample logits. This is the multi-week part Genie otherwise does for us.

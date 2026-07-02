# Phase 3 — QNN-direct LLM runtime (spec + progress)

Goal: run Llama-3.2-1B on the NPU **without Genie**, via `qai_appbuilder` (QNN-direct), so we own the
perf infra (adaptive polling → ~12 tok/s *and* cool). Replaces what Genie does: tokenize, prefill/decode
graph orchestration, KV-cache management, sampling.

## Foundation (DONE)
- **Two graphs in `weight_sharing_model_1_of_1.serialized.bin`** (one QNNContext drives both via
  `Inference(inputs, perf, graphIndex)`):
  - **idx 0 = `ar128_cl4096`** (PREFILL, 128-token chunk). Inputs: `input_ids`[1,128] int32; per-layer
    KV ×16 `past_key_N_in`[8,1,64,**3968**]/`past_value_N_in`[8,1,**3968**,64] ufp8; `position_ids_cos`/`_sin`
    [1,1,128,32]; `attention_mask`[1,1,128,4096]. Outputs: KV ×16 `past_*_out`[8,1,128,64]/[8,1,64,128];
    `logits`[1,128,128256].
  - **idx 1 = `ar1_cl4096`** (DECODE, 1 token). Inputs: `input_ids`[1,1]; KV ×16 [8,1,64,**4095**]/[8,1,**4095**,64]
    ufp8; cos/sin[1,1,1,32]; mask[1,1,1,4096]. Outputs: KV ×16 [8,1,1,64]/[8,1,64,1]; `logits`[1,1,128256].
- **Model:** 16 layers, 8 KV heads (GQA), head_dim 64, ctx 4096, vocab 128256, bos 128000, eos 128009.
- **KV cache is ufp8 passthrough** — `past_*_out` feeds `past_*_in` verbatim (opaque uint8, NO dequant).
  The prefill window is 3968 (=4096−128), decode window 4095 (=4096−1); we ring/pad the cache to match.
- **Tokenizer:** `tokenizer.json` via HF `tokenizers` (installed). vocab 128256; specials verified.
- **Perf:** adaptive polling patch in `QnnInferenceEngine.cpp` (accepted on v68, rc=0).

## Runtime algorithm (TO BUILD)
1. Build prompt with the Llama-3.2 chat template; tokenize → ids.
2. **Prefill** (graphIndex=0): process ids in 128-token chunks. Per chunk build input_ids(128, pad last),
   RoPE cos/sin for the chunk's positions (rope-theta 500000, llama3 scaling factor 32), causal mask over
   the 4096 window; feed KV-in (3968 window); collect KV-out (128 new slots) → append to the cache.
3. Take the last real token's logits → **greedy** argmax (config top-k=1) = first generated token.
4. **Decode** (graphIndex=1): loop — input_ids[1,1]=last token; cos/sin for its position; mask; KV-in
   (4095 window); get logits[1,1,vocab] + 1 KV slot; argmax; append; stop on eos(128009) or max tokens.
5. Detokenize the generated ids.

## Remaining sub-steps
- [ ] 3a RoPE cos/sin builder + causal mask builder (shapes above)
- [ ] 3b KV-cache ring buffer (ufp8 bytes; prefill 3968 ↔ decode 4095 window handling)
- [ ] 3c prefill loop (graphIndex=0) wired through `QNNContext.Inference`
- [ ] 3d decode loop (graphIndex=1) + greedy sampling + eos stop
- [ ] 3e wrap as a socket daemon like q6a-llmd (drop-in, but QNN-direct + adaptive polling)
- [ ] 3f measure: idle CPU (adaptive polling) + tok/s vs Genie's 9.6/12

## Notes / risks
- ufp8 KV quant encodings: since out→in is verbatim, no scale/offset needed for passthrough. The graph
  handles quant internally. Only input_ids/cos/sin/mask/logits are float/int we construct.
- `Inference` input order = `getInputName()` order (NOT declaration order) — build arrays in that order.
- This is the multi-day part Genie otherwise hides. Foundation is proven; the loop is the work.

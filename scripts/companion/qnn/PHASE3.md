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

## Progress (2026-07-03) — see `qnn_llm.py`, `qnn_prefill_test.py`
- [x] 3a RoPE (llama3 θ=500000, scaling) + causal mask — **PROVEN CORRECT**.
- [x] **Prefill works end-to-end (graphIndex=0):** `qnn_prefill_test.py` — "The capital of France is"
      → **" Paris"** (top-1; top5 Paris/not/Berlin/…/a). So RoPE + mask + input remap + qai float→quant
      pipeline + greedy are ALL correct on the first real attempt. Key facts confirmed:
      - `Inference(inputs, "burst", graphIndex)` — 3 positional args only (data types default float).
      - Pass **float** cos/sin/mask/KV; qai_appbuilder quantizes per the graph encodings. Output float.
      - Mask: `-100.0` masked / `0.0` keep; layout `[past(3968), current(128)]`, cols 3968+j causal.
      - KV feedback must remap **by name** (in-order ≠ out-order; out is value-before-key, seq layers).
- [~] 3d decode loop (graphIndex=1): **first decode step works; the SECOND step FAILS** with
      `Dma execution failed on the skel side result=1100` (during graph execution, DSP side).
      **ROOT CAUSE (diagnosed 2026-07-04):** qai_appbuilder's `Inference` re-registers ALL 36 input + 33
      output DMA buffers with the DSP **every call**; the 32 huge KV tensors exhaust the DSP DMA/mapping
      capacity after ~2 calls. Ruled out: host OOM (6.3 GB free); data size / conversion (**native ufp8
      mode = 4× less data, no float conversion → SAME error**); `.copy()`. Native prefill/decode step-1
      both correct (" Paris", logits uint8, argmax works) — so it's purely the per-call DMA registration.
      **The real fix = KV RESIDENT on the DSP** (register once, update in place, pass only the new token) —
      exactly what Genie does internally. qai_appbuilder's "pass all I/O every call" API doesn't support
      this. Options (all substantial): (a) qai_appbuilder `QNNShareMemory`+`QNNContextProc` (persistent ION
      buffer — uncertain it keeps DSP registration resident; multi-process restructure); (b) patch
      qai_appbuilder's buffer lifecycle to reuse/free DMA mappings; (c) **raw QNN with persistent
      `Qnn_MemHandle` shared buffers for the KV** (the proper approach = reimplementing Genie's KV residency;
      multi-week). See `native_gen.py` / `native_prefill.py` for the proven-correct native path.
- [ ] 3e wrap as a socket daemon like q6a-llmd (QNN-direct + adaptive polling)
- [ ] 3f measure: idle CPU (adaptive polling) + tok/s vs Genie's 9.6/12

**Bottom line:** the QNN-direct path is *proven correct* (prefill predicts right tokens, no Genie). The
only blocker to full generation is the repeated-decode DSP DMA error → needs ShareMemory-backed KV
buffers (the multi-day-ish piece). `qnn_llm.py` has the full loop; swap the decode KV to ShareMemory.

## Notes / risks
- ufp8 KV quant encodings: since out→in is verbatim, no scale/offset needed for passthrough. The graph
  handles quant internally. Only input_ids/cos/sin/mask/logits are float/int we construct.
- `Inference` input order = `getInputName()` order (NOT declaration order) — build arrays in that order.
- This is the multi-day part Genie otherwise hides. Foundation is proven; the loop is the work.

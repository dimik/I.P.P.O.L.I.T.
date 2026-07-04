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

## Option 1 RULED OUT + the proper fix (researched 2026-07-04)
- **QNNShareMemory/QNNContextProc will NOT fix the DSP DMA exhaustion.** `CreateShareMem` uses
  `ipc::SharedRegion` (host `mmap` shared memory for the multi-PROCESS model), and `QnnInferenceEngine.cpp`
  has ZERO `QnnMem`/`MemHandle`/`rpcmem` usage — qai_appbuilder always uses `clientBuf` tensors (re-registered
  to the DSP every execute). ShareMemory only removes host-side inter-process copies, not the per-call DSP
  registration that exhausts on step 2.
- **Proper fix = QNN HTP shared buffers (MemHandle)** — register KV buffers ONCE, reuse across executes:
  `rpcmem_alloc(HEAP_ID_SYSTEM=25, FLAGS=1)` → `rpcmem_to_fd` → `Qnn_MemDescriptor_t` → `QnnMem_register`
  → `Qnn_MemHandle_t`; set tensor `memType=QNN_TENSORMEMTYPE_MEMHANDLE`, `memHandle=…`, `clientBuf=null`.
  (Ref: QNN "HTP Shared Buffer Tutorial", docs.qualcomm.com 80-63442-50.) This is what Genie does internally.
- **Remaining path:** OPTION 2 — patch qai_appbuilder's IO-tensor code (QnnInferenceEngine executeGraphs/IO
  setup) to allocate rpcmem + QnnMem_register the KV tensors as MemHandles and keep them resident across
  prefill(g0)+decode(g1); update the new KV slot in place. OR OPTION 3 raw-QNN. Both = the multi-week core.
  Genie/QAIRT offer no shortcut (confirmed). qai_appbuilder gives us working graph/context plumbing to build
  option 2 on, which is more tractable than raw QNN.

## BREAKTHROUGH (2026-07-04): Genie is buildable from source; "poll" = threadpool spin
The QAIRT SDK ships the **full Genie source** at `examples/Genie/Genie/` (CMake + README:
"recreate the Genie library from source") — libGenie is NOT closed. Two key realizations:
- **Genie's `poll` is its own worker-THREADPOOL spin, NOT the QNN HTP RPC-polling power config**
  (Genie source has zero `setPowerConfig`). `threadpool.cpp::loop()`:
  `if (!_jobs.empty()) run; else if (_poll) __cpu_relax()  // spin forever = 247% idle; else cond.wait()`.
  So poll:true = threads busy-spin between jobs (the 90 °C idle); poll:false = block (cool, +wake latency).
- **Adaptive polling = a ~4-line threadpool patch** (bounded spin then block):
  `else if (_poll && idle_spins++ < LIMIT) __cpu_relax(); else { cond.wait(lock); idle_spins=0; }`
  → spin during active decode (poll:true speed ~12 tok/s), block when idle (cool). Enqueue already
  notifies the condition, so blocked threads wake on new jobs.

**This SUPERSEDES options 1/2/3.** Plan: patch `threadpool.cpp`, rebuild `libGenie.so` from the SDK source,
swap it into the working `q6a-llmd` daemon (config poll:true) → Genie's proven resident-KV decode +
adaptive polling, no reimplementation. (Note: the earlier qai_appbuilder QNN-direct + MemHandle path
is moot for this goal — the fix lives in Genie's threadpool, and Genie is buildable.)

## DONE (2026-07-04): adaptive polling built from source and DEPLOYED to q6a-llmd
Built `libGenie.so` from the SDK source, patched the threadpool for adaptive spinning, and deployed it
into the live daemon. **The full win is achieved and running.** Exact patched files saved under
`genie-build/` (threadpool.cpp, threadpool.hpp, Genie-CMakeLists.txt, q6a-llmd-adaptive.conf).

### The adaptive patch (final form — time-based, in `threadpool.cpp::loop()`)
```cpp
auto _last_work = std::chrono::steady_clock::now();      // before the while loop
// ... after dispatching a job j(): _last_work = std::chrono::steady_clock::now();
// no-jobs branch:
if (_poll && (std::chrono::steady_clock::now() - _last_work) < std::chrono::milliseconds(100)) {
  lock.unlock(); __cpu_relax();                          // spin only within 100ms of last work
} else {
  _mutex_condition.wait(lock); lock.unlock();            // idle >100ms → block (cool)
  _last_work = std::chrono::steady_clock::now();
}
```
Requires `#include <chrono>` (added after the threadpool.hpp include). `enqueue()` already
`notify_one()`s unconditionally under the queue mutex, so the block/wake is race-free — a job pushed
during the spin→block transition is seen on the next `!_jobs.empty()` check.

### Building it (all Windows-port hurdles resolved in `Genie-CMakeLists.txt`)
- Toolchain: rustup cargo ≥1.80 (apt cargo 1.75 too old for rayon-core); keep Cargo.lock v4.
- CMake patches: `TOKENIZERS_RUST_TARGET`/`TARGET_ARCH` → `aarch64-unknown-linux-gnu`;
  `tokenizers_capi.lib`→`libtokenizers_capi.a`; `GENIE_API=__declspec(dllexport)`→ empty; EXCLUDE
  `windows/DynamicLoading.cpp` (not linux); `STATIC_LIBS_FOR_RUST` → `gcc_s;util;rt;pthread;m;dl;<abs>/libonig.a`.
- **CRITICAL — optimization:** the SDK example CMake sets NO build type → defaults to `-O0`. Configure
  with `-DCMAKE_BUILD_TYPE=Release` (→ `-O3 -DNDEBUG`). At `-O0` decode ran ~8.3; at `-O3` it matches
  the prebuilt bundle. Build: `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8`.

### Definitive benchmark — there was NEVER a throughput regression
The earlier "~12 tok/s" was a stale/optimistic reading; the real reproducible rate for this Llama-3.2-1B
on the Q6A (Hexagon v68) is ~9.4–9.8 tok/s regardless of build or poll mode. Apples-to-apples
(`genie-t2t-run --profile`, poll:true, identical prompt):

| Build | decode-rate | idle CPU |
|---|---|---|
| Bundle 2.40 (prebuilt) | 9.38 tok/s | 2.55 cores 🔥 (~90 °C busy-spin) |
| **Our 2.42 `-O3` + adaptive** | **9.81 tok/s** | **0.00 cores** ❄️ |

Our from-source build slightly *beats* the bundle AND idles cool. Adaptive polling delivers poll:true's
spin-during-active-decode behavior with zero idle burn.

### Deployment into q6a-llmd
- Copied our `build/lib/libGenie.so` (2.42, `-O3`, adaptive) over the daemon's; set config
  `htp-model-config-llama32-1b-gqa.json` → `"poll": true`.
- systemd drop-in `q6a-llmd.service.d/adaptive.conf` sets the daemon's lib env to the **proven 2.42 combo**:
  `LD_LIBRARY_PATH=<Genie>/build/lib:<SDK2.42>/lib/aarch64-oe-linux-gcc11.2`,
  `ADSP_LIBRARY_PATH=<SDK2.42>/lib/hexagon-v68/unsigned`. (Our 2.42 libGenie needs 2.42 QnnSystem — the
  2.40 QNN libs still in `~/llama-1b` must NOT be first in LD, or you get `getQnnSystemInterface FAILED`.)
- Verified: `model loaded on NPU; ready`; idle **0.000 cores**; end-to-end query
  "Name three primary colors." → "Red, blue, and yellow." in 1.0s.

### Two operational gotchas (cost real time — remember them)
1. **Wedged cdsp session:** rapidly restarting the daemon while a prior instance held the DSP left an
   orphaned fastrpc session on domain 3 (`Create From Binary List Async ... err 5005`,
   `remote_munmap64 failed`, `reverse module apps_mem already found refs 2`). It survives `pkill -9`;
   even standalone `genie-t2t-run` then hangs at context creation. **Fix = reboot the Q6A** (clears cdsp);
   the daemon comes back healthy on boot. Avoid restart storms — always `stop` + confirm no NPU client
   before the next start.
2. **Daemon socket protocol:** `q6a_llmd.py handle()` reads the prompt **until the client half-closes**
   (`recv` loop until EOF). A test client MUST `s.shutdown(socket.SHUT_WR)` after `sendall` or the daemon
   blocks forever in `recv` and the query never runs. (Agent harness prefix `\x01RAW\x01` = pre-formatted
   prompt, else it wraps in the Llama-3 chat TEMPLATE.)

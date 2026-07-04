# Q6A — QAIRT 2.46 stack: findings & the safe migration path

Investigation (2026-07-05) into moving the Q6A off the manual **QAIRT 2.42** tarball onto a newer **2.46**
stack. Conclusion up front: **2.46 works and is safely installable, but buys no capability here** — keep
2.42 unless you specifically want a unified/newer stack. This doc records *why*, and the exact safe path
if we ever do migrate.

## TL;DR
- **YOLOv11 cannot run on the v68 NPU at ANY QAIRT version.** Its C2PSA attention `MatMul` requires HTP
  arch **≥73**; v68 is rejected at **both 2.42 and 2.46** with the explicit error
  `has incorrect Value 68, expected >= 73` on `/model/10/m/0/attn/MatMul`. It's a **hardware** gate (v73+
  matrix engine), not a version issue. → **Use YOLOv8** (no attention; composes fine on v68). Already deployed.
- **The 1B LLM is identical** on 2.42 vs 2.46 (chip is v68-capped at ~1B). Stock 2.46 Genie measured
  **8.1 tok/s** decode vs the **~9.8 tok/s** of our from-source *adaptive* libGenie on 2.42. So stock 2.46
  is a touch slower unless we rebuild the adaptive patch.
- **The clean `apt install` is blocked** on this board by a fastrpc fork collision (below). But the 2.46
  libraries run fine against Radxa's existing fastrpc — proven by running the 1B on 2.46. So a migration,
  if wanted, uses the **extracted debs** (`~/qairt-2.46`), not `apt`.

## The fastrpc conflict (why `apt install qairt-libs` fails)
```
qcom-fastrpc1 : Breaks: fastrpc but 1.0.3-1 is to be installed
```
- The board runs **Radxa's own `fastrpc`** (v1.0.4, Maintainer *Radxa Computer Co.*, repo
  `radxa-repo.github.io/qcs6490-noble`), which provides `/usr/lib/libcdsprpc.so`. It's a fork of Qualcomm's
  open-source [github.com/qualcomm/fastrpc](https://github.com/qualcomm/fastrpc) with **qcs6490 patches** —
  Radxa packaged it because Qualcomm's official fastrpc didn't work on the Q6A.
- The qcom PPA's `qairt-libs` depends on `qcom-fastrpc-dev → qcom-fastrpc1` (v1.0.15, Qualcomm's build for
  Thundercomm/RB3), which declares `Breaks: fastrpc`. Two competing userspace fastrpc for the same board.
- **Do NOT force-swap to `qcom-fastrpc1`** — Radxa forked specifically because Qualcomm's didn't work here;
  swapping risks breaking the DSP/cdsp for both the LLM and camera (reboot/recovery). Qualcomm is
  upstreaming qcs6490 support, so this may resolve itself in a future release.
- **Key point:** the conflict is *packaging metadata only*. The 2.46 QNN/Genie libs are runtime-compatible
  with Radxa's fastrpc (verified — see below). So skip `apt` and use the extracted debs.

## Where the pieces come from
- **2.46 runtime + tools + dsp binaries + headers:** the qcom PPA (`ubuntu-qcom-iot/qcom-ppa`) packages
  `qairt-libs / qairt-tools / qairt-dsp-binaries / qairt-headers` (v2.46.0-0ubuntu1~bpo24.04.1). Includes
  `libGenie.so`, `libQnnHtp.so`, `libQnnModelDlc.so`, v68 skels, `genie-t2t-run`,
  `qnn-context-binary-generator`. Extract without installing:
  `apt-get download qairt-libs qairt-tools qairt-dsp-binaries qairt-headers` then
  `dpkg-deb -x <deb> ~/qairt-2.46`.
- **Genie C++ source (for the adaptive-polling patch)** is NOT on GitHub standalone and NOT in the apt
  packages (binaries + API headers only). It ships **inside the QAIRT Community SDK** at
  `examples/Genie/Genie/` (same CMake tree as 2.42). Public download (verified HTTP 206):
  ```
  https://softwarecenter.qualcomm.com/api/download/software/sdks/Qualcomm_AI_Runtime_Community/All/2.46.0.260424/v2.46.0.260424.zip
  ```
  (URL uses the date `260424`, not the full framework timestamp `260424121129`.) The public GitHub repos
  `qualcomm/qai-appbuilder` / `quic/ai-engine-direct-helper` are QNN *wrappers*, not the Genie lib source.

## Verified on 2.46 (isolated, no system changes)
- Extracted 2.46 to `~/qairt-2.46/usr/{lib,bin,share}`; `qnn-net-run --version` → `v2.46.0.260424121129`.
- Ran the **existing 2.30-format 1B context binary** on 2.46 `genie-t2t-run` (libGenie 1.18.0) using
  `~/qairt-2.46` libs + Radxa's fastrpc + the 2.46 v68 skel → correct generation, **8.1 tok/s** decode,
  59.6 tok/s prefill, TTFT 369 ms. So 2.46 runs the LLM on the v68 NPU with no fastrpc swap.
- `env.sh` written at `~/qairt-2.46/env.sh` (sets `LD_LIBRARY_PATH`, `ADSP_LIBRARY_PATH` to the v68 cdsp
  skel dir, `PATH`).

## The safe migration path (if we ever do it) — NO apt, NO fastrpc swap
1. Keep the 2.42 tarball + current `q6a-llmd` as rollback until proven.
2. `~/qairt-2.46` already extracted; `source ~/qairt-2.46/env.sh` for the 2.46 runtime.
3. Download the 2.46 Community SDK (URL above), unzip, `cd examples/Genie/Genie`, re-apply the adaptive
   threadpool patch (bounded-spin-then-block in `threadpool.cpp::loop()` — see
   `scripts/companion/qnn/PHASE3.md`), build `-O3` → `libGenie.so` (2.46, adaptive). Keeps ~9.8 tok/s.
4. Point `q6a-llmd` at the 2.46 libs + adaptive libGenie (systemd `Environment=` for
   `LD_LIBRARY_PATH`/`ADSP_LIBRARY_PATH`). The 2.30 1B binary loads fine on 2.46. Verify tok/s.
5. Camera/YOLO: rebuild the YOLOv8 context binary with 2.46 tools (`qnn-context-binary-generator`), or keep
   the working 2.42 appbuilder path. (YOLOv11 stays impossible either way.)
6. Only after both are proven on 2.46, retire the 2.42 tarball.

**Recommendation:** don't migrate for capability — there is none. Do it only as deliberate housekeeping to
unify on a newer, PPA-adjacent version. The 2.42 stack + YOLOv8 is the working baseline.

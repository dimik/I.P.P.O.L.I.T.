#!/bin/bash
# Build the on-device MiDaS-V2 monocular depth model for the Q6A (Hexagon v68 NPU). Plan P2.3.
#
# WHY MiDaS-V2 (not Depth-Anything): v68 can't compose attention MatMul on QAIRT 2.42 (same wall that
# blocks YOLOv11 and the 7B LLM — needs HTP arch >=73). MiDaS-v21-small is a MobileNetV2 encoder + conv
# decoder, NO attention -> composes cleanly on v68. Depth-Anything (v1/v2/v3) are ViT-based -> would hit
# the arch>=73 wall. The review cites MiDaS-V2 at 4.117 ms w8a8 on QCS6490 for exactly this reason.
#
# Same 3-hop toolchain as build_yolo.sh (AI Hub is 2.45+, device runtime is pinned 2.42):
#   AI-Hub quantized ONNX -> x86 qairt-converter (2.42 DLC, no AVX2) -> v68 context binary on the Q6A.
#
# Model I/O: input NCHW [1,3,256,256] RGB in [0,1] (DEFAULT_HEIGHT/WIDTH=256); output is a single-channel
# INVERSE-depth (disparity) map [1,256,256], affine-invariant (needs LiDAR/floor-plane scale to become
# metric — that's the runtime step, not this build).
#
# Usage:  PRECISION=w8a8 ./build_depth.sh [MODEL]     MODEL defaults to midas_depth_w8a8
# Run ON the Odyssey. Produces models/<MODEL>.bin (context binary) committed for turnkey deploy.
# THIS SCRIPT ONLY BUILDS THE MODEL — it adds no sustained device load (one-shot compile). The runtime
# depth process + 3-accelerator coexistence test are a separate, thermally-gated step.
set -euo pipefail
MODEL="${1:-midas_depth_w8a8}"
PRECISION="${PRECISION:-w8a8}"       # w8a8 for the 4 ms path; validate it composes on v68 at step 3
Q6A="ippolit-lan"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="${QHM_VENV:-$HOME/qhm-venv/bin/python}"
QAIRT_X86="$HOME/qairt-x86"
WORK="$HOME/${MODEL}-q6a-onnx"
SDK_REMOTE="~/qairt_2.42.0.251225"; ABI="aarch64-oe-linux-gcc11.2"

echo "== 1/4 fetch the pre-quantized w8a8 ONNX from AI Hub =="
# WHY --fetch-static-assets (not a from-scratch quantize like build_yolo.sh): MiDaS's w8a8 export path
# calibrates on the NYUV2 dataset, which is PRIVATE (manual Kaggle download) -> a from-scratch export dies
# with UnfetchableDatasetError. `--fetch-static-assets` downloads Qualcomm's OFFICIAL pre-quantized w8a8
# ONNX instead (uint8 image[1,3,256,256] -> uint8 depth_estimates[1,1,256,256], QDQ, no attention ops).
# For DOMAIN-MATCHED accuracy later, re-quantize on real robot frames via qai_hub.submit_quantize_job(
# calibration_data={"image":[<uint8 NCHW frames>]}, weights_dtype=INT8, activations_dtype=INT8) and skip
# this fetch. QAIHM_CI=1 auto-accepts the isl-org repo clone (deps: geffnet==1.0.2, timm==1.0.15).
mkdir -p "$WORK"
QAIHM_CI=1 QAIHM_DEV_MODE=1 "$VENV" -m qai_hub_models.models.midas.export \
  --target-runtime onnx --chipset qualcomm-qcs6490 --precision "$PRECISION" --fetch-static-assets \
  --skip-profiling --skip-inferencing --output-dir "$WORK"
ZIP="$(find "$WORK" -name 'midas-onnx-*.zip' | head -1)"
[ -n "$ZIP" ] && ( cd "$WORK" && unzip -o -q "$ZIP" )
ONNX="$(find "$WORK" -name '*.onnx' | head -1)"      # carries external weights (midas.data) alongside
[ -n "$ONNX" ] || { echo "FAIL: no ONNX fetched (see output above)"; exit 1; }
echo "   ONNX: $ONNX"

echo "== 2/4 convert ONNX -> 2.42 DLC (x86 qairt-converter, no AVX2 needed) =="
( source "$QAIRT_X86/env.sh" 2>/dev/null
  qairt-converter --input_network "$ONNX" --output_path "$WORK/${MODEL}_242.dlc" > "$WORK/convert.log" 2>&1 ) || true
tr '\r' '\n' < "$WORK/convert.log" | grep -aE "CONVERSION_SUCCESS|WRITE_SUCCESS|CONVERSION_FAIL|- ERROR -" | tail -3 || true
[ -s "$WORK/${MODEL}_242.dlc" ] || { echo "FAIL: converter produced no DLC (see $WORK/convert.log)"; exit 1; }
cp "$WORK/${MODEL}_242.dlc" "$REPO_DIR/models/${MODEL}_242.dlc"

echo "== 3/4 build v68 context binary ON the Q6A (THE v68 GATE) =="
scp -q "$WORK/${MODEL}_242.dlc" "$Q6A:~/${MODEL}_242.dlc"
ssh "$Q6A" "SDK=$SDK_REMOTE; export LD_LIBRARY_PATH=\$SDK/lib/$ABI; export ADSP_LIBRARY_PATH=\$SDK/lib/hexagon-v68/unsigned
  rm -rf ~/depth_ctx && mkdir -p ~/depth_ctx
  \$SDK/bin/$ABI/qnn-context-binary-generator --model \$SDK/lib/$ABI/libQnnModelDlc.so \
    --dlc_path ~/${MODEL}_242.dlc --backend \$SDK/lib/$ABI/libQnnHtp.so \
    --binary_file ${MODEL} --output_dir ~/depth_ctx 2>&1 | grep -iE 'ERROR|error code|graph' || true
  [ -s ~/depth_ctx/${MODEL}.bin ] && cp ~/depth_ctx/${MODEL}.bin ~/${MODEL}.bin && ls -la ~/${MODEL}.bin || { echo 'FAIL: no context binary — likely a v68 compose failure (op needs arch>=73)'; exit 1; }"

echo "== 4/4 fetch the context binary into the repo (turnkey) =="
scp -q "$Q6A:~/${MODEL}.bin" "$REPO_DIR/models/${MODEL}.bin"
echo "DONE. models/${MODEL}.bin ready (w8a8 MiDaS-V2 composed on v68). Runtime integration is the next step."

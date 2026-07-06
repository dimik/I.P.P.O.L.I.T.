#!/bin/bash
# Build the on-device YOLO COCO detector for the Q6A (Hexagon v68 NPU).
#
# WHY THIS IS NOT ONE STEP (hard-won):
#  - Qualcomm AI Hub only builds artifacts for QAIRT 2.45/2.46/2.47; the Q6A runtime + qai_appbuilder
#    are pinned at 2.42 (upgrading would disturb the working Genie LLM daemon). A 2.45 DLC/context-binary
#    will NOT load on 2.42 (err "dlc handle code 1002").
#  - So we take a QUANTIZED ONNX (w8a16) from AI Hub and convert it to a 2.42 DLC OURSELVES with the
#    x86 qairt-converter on the Odyssey (~/qairt-x86, 2.42). The converter (unlike qairt-quantizer)
#    does NOT need AVX2, so it runs on the J4125. QDQ encodings are preserved -> quantized 2.42 DLC.
#  - YOLOv11 does NOT work: its C2PSA attention MatMul needs HTP arch >=73; v68 can't compose it on
#    2.42. YOLOv8 has no attention -> composes cleanly on v68. (Same "v68 is old" wall as the 7B LLM.)
#  - The model input is NCHW [1,3,640,640], values [0,1]; padding is BOTTOM-RIGHT letterbox (centered
#    padding tanks the scores). Outputs: scores[8400], class_idx[8400], boxes[8400,4] (xyxy in 640 space).
#
# Usage:  ./build_yolo.sh [MODEL]     MODEL defaults to yolov8_det
# Run ON the Odyssey. Produces models/<MODEL>.bin (context binary) committed for turnkey deploy.
set -euo pipefail
MODEL="${1:-yolov8_det}"
# Precision: w8a16 (current, safe) or w8a8 (P1.2: ~half the infer time via native int8 I/O, but int8
# activations may cost accuracy AND may fail to compose on v68 at step 3 -> validate before deploying).
# A w8a8 build should output a DIFFERENT name so it can be A/B-tested without clobbering the live w8a16 .bin:
#   PRECISION=w8a8 ./build_yolo.sh yolov8_det_w8a8
PRECISION="${PRECISION:-w8a16}"
Q6A="ippolit-lan"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="${QHM_VENV:-$HOME/qhm-venv/bin/python}"
QAIRT_X86="$HOME/qairt-x86"
WORK="$HOME/${MODEL}-q6a-onnx"
SDK_REMOTE="~/qairt_2.42.0.251225"; ABI="aarch64-oe-linux-gcc11.2"

echo "== 1/4 export quantized ONNX ($PRECISION) from AI Hub =="
mkdir -p "$WORK"
# NB: the AI-Hub export module name is the base model (yolov8_det); MODEL may be a build alias (e.g.
# yolov8_det_w8a8) used only for output naming. Strip a trailing _w8a8/_w8a16 to get the module.
EXPORT_MODULE="${MODEL%_w8a8}"; EXPORT_MODULE="${EXPORT_MODULE%_w8a16}"
"$VENV" -m qai_hub_models.models.${EXPORT_MODULE}.export \
  --target-runtime onnx --chipset qualcomm-qcs6490 --precision "$PRECISION" \
  --skip-profiling --skip-inferencing --output-dir "$WORK"
ONNX="$(find "$WORK" -name "${EXPORT_MODULE}.onnx" | head -1)"
echo "   ONNX: $ONNX"

echo "== 2/4 convert ONNX -> 2.42 DLC (x86 qairt-converter, no AVX2 needed) =="
( source "$QAIRT_X86/env.sh" 2>/dev/null
  qairt-converter --input_network "$ONNX" --output_path "$WORK/${MODEL}_242.dlc" 2>&1 \
    | grep -iE "CONVERSION_SUCCESS|WRITE_SUCCESS|CONVERSION_FAIL|ERROR:" | grep -vi WARNING )
cp "$WORK/${MODEL}_242.dlc" "$REPO_DIR/models/${MODEL}_242.dlc"

echo "== 3/4 build v68 context binary ON the Q6A =="
scp -q "$WORK/${MODEL}_242.dlc" "$Q6A:~/${MODEL}_242.dlc"
ssh "$Q6A" "SDK=$SDK_REMOTE; export LD_LIBRARY_PATH=\$SDK/lib/$ABI; export ADSP_LIBRARY_PATH=\$SDK/lib/hexagon-v68/unsigned
  rm -rf ~/yolo_ctx && mkdir -p ~/yolo_ctx
  \$SDK/bin/$ABI/qnn-context-binary-generator --model \$SDK/lib/$ABI/libQnnModelDlc.so \
    --dlc_path ~/${MODEL}_242.dlc --backend \$SDK/lib/$ABI/libQnnHtp.so \
    --binary_file ${MODEL} --output_dir ~/yolo_ctx 2>&1 | grep -iE 'ERROR|error code' || true
  cp ~/yolo_ctx/${MODEL}.bin ~/${MODEL}.bin && ls -la ~/${MODEL}.bin"

echo "== 4/4 fetch the context binary into the repo (turnkey) =="
scp -q "$Q6A:~/${MODEL}.bin" "$REPO_DIR/models/${MODEL}.bin"
cp "$WORK"/*/labels.txt "$REPO_DIR/models/coco_labels.txt" 2>/dev/null || \
  find "$WORK" -name labels.txt -exec cp {} "$REPO_DIR/models/coco_labels.txt" \; 2>/dev/null || true
echo "DONE. models/${MODEL}.bin ready. Deploy: scp models/${MODEL}.bin $Q6A:~/  + coco_labels.txt"

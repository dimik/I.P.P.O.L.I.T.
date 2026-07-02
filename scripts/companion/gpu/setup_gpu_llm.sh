#!/bin/bash
# Run LLMs on the Radxa Q6A's Adreno 643 GPU via the Qualcomm OpenCL backend (llama.cpp).
#
# KEY FINDING (2026-07-02): the *proprietary* Qualcomm Adreno OpenCL driver is
# PACKAGED in the already-enabled `ubuntu-qcom-iot` PPA, and it works on the STOCK
# mainline `msm` kernel via dma-heap -- NO KGSL / kernel swap needed. This
# contradicts the common "Adreno is unusable on the Q6A Ubuntu image" consensus
# (which assumed you must swap to a KGSL kernel). No blob extraction, no building
# the driver -- it's an apt install.
#
# RESULTS (measured, Llama-3.2-*-Instruct Q4_K_M, `-ngl 99`):
#   1B  -> ~11.7 tok/s generation, ~82 tok/s prompt  == the NPU (~12 tok/s).
#          Decode is memory-bandwidth bound and the GPU+NPU share the ~40-50 GB/s
#          LPDDR5, so the GPU is NOT faster than the NPU. (Community "20-55 tok/s"
#          figures are flagship Snapdragons w/ LPDDR5X, not this 2021 Adreno 643.)
#   3B  -> CRASHED the board: THERMAL SHUTDOWN. Sustained GPU load drove the SoC to
#          the 110 C critical trip (hot=90 C; board idles ~65-70 C, passively cooled
#          and enclosed) -> PMIC emergency power-off. DO NOT run 3B+ full-offload
#          without active cooling. The NPU is the coolest/most-efficient path.
set -euo pipefail

echo "=== 1. Qualcomm Adreno OpenCL driver + GPU firmware (ubuntu-qcom-iot PPA) ==="
sudo apt-get update
sudo apt-get install -y qcom-adreno-cl1 linux-firmware-dragonwing
# Dev headers. NOTE: qcom-adreno-cl-dev CONFLICTS with generic opencl-*-headers,
# so install it ALONE (do not add opencl-headers/opencl-clhpp-headers).
sudo apt-get install -y qcom-adreno-cl-dev
echo "--- verify the GPU is an OpenCL device (expect: QUALCOMM Adreno) ---"
timeout 15 clinfo | grep -iE "Platform Name|Device Name" | head

echo "=== 2. build tools ==="
sudo apt-get install -y git build-essential cmake

echo "=== 3. build llama.cpp with the OpenCL (Adreno) backend ==="
[ -d "$HOME/llama.cpp" ] || git clone --depth 1 https://github.com/ggml-org/llama.cpp "$HOME/llama.cpp"
cd "$HOME/llama.cpp"
cmake -B build-cl -DGGML_OPENCL=ON
cmake --build build-cl -j"$(nproc)" --target llama-cli llama-bench

cat <<'NOTE'
=== DONE. Run with all layers on the GPU (-ngl 99): ===
  ./build-cl/bin/llama-cli  -m <model.gguf> -ngl 99 -p "your prompt"
  ./build-cl/bin/llama-bench -m <model.gguf> -ngl 99

  Startup prints: ggml_opencl: device: 'QUALCOMM Adreno(TM) 635 ...'  (= the 643)

WARNING: watch temps. `for z in /sys/class/thermal/thermal_zone*; do echo $(cat $z/type)=$(($(cat $z/temp)/1000))C; done`
  Critical trip = 110 C. Do NOT run 3B+ full-offload on this passively-cooled board -> thermal shutdown.
  For sustained work use the NPU (see setup_npu_llm.sh) or add active cooling.
NOTE

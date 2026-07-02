#!/bin/bash
# Build QAI AppBuilder (QNN-DIRECT runtime, bypassing Genie) for the Radxa Q6A
# (QCS6490, Hexagon v68). Going direct on QNN exposes QnnHtpPerfInfrastructure
# (adaptive polling + full perf control) and lets us own the inference path.
#
# Gets the QAIRT SDK WITHOUT a Qualcomm account (their "free" SDK requires COMPANY
# VERIFICATION) by extracting it from Radxa's qairt-npu-v68 docker image.
# Verified 2026-07-02 on the Q6A: builds, installs, imports, and QNNContext loads
# a v68 context binary onto the NPU (no Genie).
set -euo pipefail

IMG=radxazifeng278/qairt-npu-v68:v1.2
# NOTE: QNN_SDK_ROOT path MUST contain the version string — setup.py parses the
# version out of the path (a plain ~/qairt fails with "Cannot extract version").
SDK="$HOME/qairt_2.42.0.251225"
FORK="${FORK:-https://github.com/dimik/qai-appbuilder.git}"

echo "=== 1. Docker + pull the QAIRT-v68 image (~3.9 GB, no Qualcomm login) ==="
command -v docker >/dev/null || { sudo apt-get install -y docker.io; sudo systemctl enable --now docker; sudo usermod -aG docker "$USER"; }
sudo docker pull "$IMG"

echo "=== 2. extract the QAIRT 2.42 SDK to a VERSIONED path ==="
if [ ! -d "$SDK" ]; then
  cid=$(sudo docker create "$IMG")
  sudo docker cp "$cid:/root/qairt/2.42.0.251225" "$SDK"
  sudo docker rm "$cid" >/dev/null
  sudo chown -R "$USER:$USER" "$SDK"
fi

echo "=== 3. clone the fork + ONLY the pybind11 submodule (skip heavy genie externals) ==="
[ -d "$HOME/qai-appbuilder" ] || git clone "$FORK" "$HOME/qai-appbuilder"
cd "$HOME/qai-appbuilder"
git submodule update --init pybind/pybind11

echo "=== 4. build deps + build the aarch64/v68 wheel ==="
sudo apt-get install -y python3.12-dev build-essential cmake
pip3 install --user --break-system-packages wheel==0.45.1 setuptools==80.9.0 pybind11==2.13.6 build==1.4.0
env QNN_SDK_ROOT="$SDK" QAI_TOOLCHAINS=aarch64-oe-linux-gcc11.2 python3 -m build -w
pip3 install --user --break-system-packages --force-reinstall dist/qai_appbuilder-*.whl

echo "=== DONE. QNN-direct runtime installed (wheel bundles libQnnHtpV68Skel/Stub). ==="
echo "Smoke test / recon:  python3 $(dirname "$0")/qab_smoke.py"

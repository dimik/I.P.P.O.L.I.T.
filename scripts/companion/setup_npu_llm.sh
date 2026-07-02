#!/bin/bash
# Set up a local LLM on the Radxa Dragon Q6A's Hexagon NPU (QCS6490, v68).
# Downloads Radxa's PRE-COMPILED Qualcomm-Genie bundle (genie-t2t-run + QNN HTP
# runtime + v68-quantized weights), installs prompt helpers, and stands up a
# RESIDENT Genie daemon so prompts don't pay the ~1.2s model reload each call.
# Run as `radxa` on the board. Deploy this whole scripts/companion/ dir alongside it.
#
# Verified 2026-07-02 (Llama 3.2 1B):
#   one-shot cold ~5s, one-shot warm ~2.8s, RESIDENT DAEMON short reply ~0.47s.
#   Generation is ~10-15 tok/s (NPU), so long answers still scale with length.
#
# Custom models (other bases/sizes): QCS6490 is Hexagon v68; the modern AI-Hub
# LLM pipeline needs v73+, so only small Qwen2/2.5/Llama/Phi/Gemma-2 architectures
# are compilable to v68, via the older QAIRT-v68 toolchain on an x86 host.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
MODEL_REPO="${MODEL_REPO:-radxa/Llama3.2-1B-4096-qairt-v68}"
MODEL_DIR="${MODEL_DIR:-$HOME/llama-1b}"
GENIE_CONFIG="${GENIE_CONFIG:-htp-model-config-llama32-1b-gqa.json}"

echo "=== 1. modelscope (user-level; Ubuntu 24.04 is PEP668-managed) ==="
pip3 install --user --break-system-packages -q -U modelscope

echo "=== 2. download pre-compiled v68 bundle -> $MODEL_DIR (~1.7 GB) ==="
mkdir -p "$MODEL_DIR"
"$HOME/.local/bin/modelscope" download --model "$MODEL_REPO" --local_dir "$MODEL_DIR"
chmod +x "$MODEL_DIR/genie-t2t-run"

# CRITICAL for the resident daemon: Radxa's config ships "poll": true, which makes
# the HTP backend BUSY-WAIT ~2.5 CPU cores CONTINUOUSLY once the model is loaded
# (not just during generation) -> ~90C idle on this passively-cooled board and
# eventual thermal shutdown. poll:false is interrupt-driven (idle ~5% CPU, ~66C)
# with no measurable latency cost for a resident daemon. (Verified 2026-07-02.)
sed -i 's/"poll": true/"poll": false/' "$MODEL_DIR/$GENIE_CONFIG"

echo "=== 3. deploy resident daemon + helpers ==="
cp "$HERE/q6a_llmd.py" "$MODEL_DIR/q6a_llmd.py"
mkdir -p "$HOME/.local/bin"
install -m 755 "$HERE/q6a-llm" "$HOME/.local/bin/q6a-llm"          # socket client (fast path)
# one-shot fallback helper (works even if the daemon is down)
cat > "$HOME/.local/bin/q6a-llm-oneshot" <<EOF
#!/bin/bash
set -e
cd "$MODEL_DIR"
export LD_LIBRARY_PATH="$MODEL_DIR:\$LD_LIBRARY_PATH" ADSP_LIBRARY_PATH="$MODEL_DIR"
MSG="\$*"; [ -z "\$MSG" ] && { echo "usage: q6a-llm-oneshot <prompt>"; exit 1; }
P=\$(printf "<|begin_of_text|><|start_header_id|>user<|end_header_id|>\n\n%s<|eot_id|><|start_header_id|>assistant<|end_header_id|>\n\n" "\$MSG")
exec ./genie-t2t-run -c $GENIE_CONFIG -p "\$P"
EOF
chmod +x "$HOME/.local/bin/q6a-llm-oneshot"

echo "=== 4. install + enable systemd units (prewarm + resident daemon) ==="
sudo cp "$HERE/systemd/llama-prewarm.service" "$HERE/systemd/q6a-llmd.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now llama-prewarm.service
sudo systemctl enable --now q6a-llmd.service

echo "=== DONE. Ensure ~/.local/bin is on PATH, then: q6a-llm \"your prompt\" ==="
echo "    resident daemon: systemctl status q6a-llmd  |  one-shot: q6a-llm-oneshot \"...\""

#!/bin/bash
# Deploy + enable an IMX296 overlay on the Radxa Dragon Q6A. Run ON the Q6A after build_imx296.sh.
# Usage:  ./deploy_imx296.sh [CAM]     CAM = 2 (default) or 3.
# Enables it via the SAME path rsetup uses (embloader/EDK2) — the only correct way on this board.
#
# ⚠️ Reboots into a camera overlay. On a headless board a bad overlay = boot loop; recover by booting a
#    microSD and undoing steps 2-3 below (the NVMe rootfs is NOT wiped). These overlays are validated
#    (offline fdtoverlay merge keeps cdsp/video/zap reserved-memory intact). See docs/q6a-camera.md.
set -euo pipefail
CAM="${1:-2}"; [[ "$CAM" =~ ^[23]$ ]] || { echo "CAM must be 2 or 3"; exit 1; }
KVER="$(uname -r)"; HERE="$(cd "$(dirname "$0")" && pwd)"
OVL="qcs6490-radxa-dragon-q6a-cam${CAM}-imx296"; EFI="/boot/efi/RadxaOS/$KVER"

echo "== 1. pin the base DTB (prevents the en7581 wrong-DTB trap on BLS regen) =="
echo "qcom/qcs6490-radxa-dragon-q6a.dtb" | sudo tee /etc/kernel/devicetree >/dev/null

echo "== 2. install the overlay into all dtbo locations (as .disabled) =="
for d in "$EFI/dtbo" /boot/dtbo "/usr/lib/linux-image-$KVER/qcom/overlays"; do
  sudo cp "$HERE/$OVL.dtbo" "$d/$OVL.dtbo.disabled" 2>/dev/null || true
  sudo grep -q "$OVL" "$d/managed.list" 2>/dev/null || echo "$OVL.dtbo" | sudo tee -a "$d/managed.list" >/dev/null 2>&1 || true
done

echo "== 3. enable it the rsetup way (edk2/embloader; hwid.sh provides get_product_id) =="
sudo bash -c '
  source /usr/lib/librtui/utils/utils.sh
  source /usr/lib/rsetup/mod/hwid.sh
  source /usr/lib/rsetup/cli/rconfig.sh
  source /usr/lib/rsetup/cli/overlay-menu.sh
  enable_overlays '"$OVL"'.dtbo'

echo "== 4. VERIFY boot config before reboot (abort if wrong) =="
E="/boot/efi/loader/entries/RadxaOS-$KVER.conf"
sudo grep -qE "^devicetree /RadxaOS/$KVER/qcs6490-radxa-dragon-q6a.dtb" "$E" \
  || { echo "FATAL: base devicetree line is not qcs6490 — do NOT reboot"; exit 1; }
sudo grep -q "$OVL.dtbo" "$E" || { echo "FATAL: overlay not referenced in BLS entry"; exit 1; }
echo "OK: base=qcs6490 + CAM${CAM} overlay referenced. 'sudo reboot', then use the capture recipe in"
echo "docs/q6a-camera.md (CAM2 -> csiphy2/csid0/vfe0_rdi0//dev/video0 ; CAM3 -> csiphy3, pick a free csid/rdi)."

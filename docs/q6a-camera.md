# Q6A MIPI camera — Sony IMX296 global-shutter (bring-up, 2026-07-04)

**Status: WORKING.** A Sony **IMX296** (color/LQ, global shutter, 1.58 MP) global-shutter camera captures
live frames on the Radxa Dragon Q6A (QCS6490, Hexagon **v68**) via the mainline **qcom-camss** V4L2 stack.
This required building the missing sensor driver and — crucially — **fixing the camera-overlay boot-loop
that blocks everyone on this board** (Radxa's own shipped camera overlays brick it too).

Artifacts: `scripts/companion/camera/` (`build_imx296.sh`, `deploy_imx296.sh`, the overlay `.dts`/`.dtbo`).

## Hardware
- Q6A has 3 CSI connectors: 1× 4-lane (31-pin Radxa FPC, "CAM1") + **2× 2-lane 15-pin, Raspberry-Pi-CSI
  compatible ("CAM2"/"CAM3")**. InnoMaker IMX296 MIPI modules ship with the 15-pin RPi FPC → plug straight
  into CAM2/CAM3. (Contacts face the PCB; the flip-lock actuator lifts up.) This build targets **CAM2**.
- IMX296 is a **1-lane** MIPI sensor (like the Raspberry Pi Global Shutter Camera), i2c address **0x1a**,
  INCK **37.125 MHz**, MIPI **1188 Mbps** (link-freq 594 MHz), output **SBGGR10 1456×1088**.

## What was needed (none of this ships working)
1. **Driver:** the stock image has CAMSS + imx219/imx412(577)/imx214, but **no imx296**. Built `imx296.ko`
   out-of-tree from mainline `drivers/media/i2c/imx296.c` (v6.18 tag) against `linux-headers-radxa-dragon-q6a`.
2. **Overlay:** adapted from Radxa's `cam2-radxa-camera-8m-219` (imx219) overlay, with four fixes:
   - **Removed the `linux,cma` reserved-memory fragment.** THIS is the fix for the boot-loop
     ([radxa-build/radxa-dragon-q6a#4](https://github.com/radxa-build/radxa-dragon-q6a/issues/4)): the
     overlay's 128 MB `linux,cma` (`linux,cma-default`) collides with the firmware reservations and the
     kernel dies reserving `cdsp@8e000000` / `video@8fe00000` / `zap@90300000` → reboot loop. Dropping the
     fragment lets CAMSS use the system CMA; validated offline (`fdtoverlay` merge keeps all three intact).
   - `compatible = "sony,imx296"`, `reg = <0x1a>`.
   - `clock-names = "inck"` — the driver does `devm_v4l2_sensor_clk_get(dev, "inck")`; the imx219 template's
     `"ext_cam_clk_imx219"` name → `-ENOENT: failed to get clock` → probe fails. mclk fixed-clock 37.125 MHz.
   - `data-lanes = <1>` (sensor) / `<0>` (csiphy) — **1-lane**. The imx219 template's 2 lanes make the
     CSIPHY wait on a non-existent 2nd lane → STREAMON succeeds but **0 frames**.
3. **Enable path:** this board boots via **embloader** (systemd-boot/EDK2), NOT extlinux. Overlays are
   enabled by a `devicetree-overlay` line in the BLS entry (`/boot/efi/loader/entries/RadxaOS-<ver>.conf`)
   + the `.dtbo` (enabled, no `.disabled`) in `/boot/efi/RadxaOS/<ver>/dtbo/`. `rsetup` writes these; the
   deploy script replicates it non-interactively (sourcing `hwid.sh` for `get_product_id`).
   - **⚠️ en7581 trap:** enabling an overlay can trigger a BLS-entry regen that picks the *wrong* DTB
     (`en7581-evb.dtb`, a MediaTek board — kernel ships all vendors' DTBs). If the base `devicetree` line
     isn't `qcs6490-radxa-dragon-q6a.dtb`, the board won't boot. Fix = pin `/etc/kernel/devicetree`
     (deploy script does this) and always verify the BLS entry before rebooting.

## Build + deploy (on the Q6A)
```bash
cd scripts/companion/camera
./build_imx296.sh     # fetch imx296.c, build imx296.ko + depmod, compile the overlay
./deploy_imx296.sh    # pin DTB, install overlay, enable via rsetup path, verify BLS entry
sudo reboot           # boots in ~24 s; on brick, recover via microSD (rootfs NOT wiped)
```
On boot: `dmesg | grep imx296` → `found IMX296LQ (NN.NC)`; sensor ACKs at `0x1a` on the CCI bus (`i2cdetect
-y -r 18`); `/dev/media0` + `/dev/video*` appear.

## Capture recipe (CAMSS pipeline: sensor → csiphy2 → csid0 → vfe0_rdi0 → /dev/video0)
```bash
M="media-ctl -d /dev/media0"
$M -l '"msm_csiphy2":1 -> "msm_csid0":0 [1]'
$M -l '"msm_csid0":1 -> "msm_vfe0_rdi0":0 [1]'
for e in '"imx296 18-001a":0' '"msm_csiphy2":0' '"msm_csiphy2":1' \
         '"msm_csid0":0' '"msm_csid0":1' '"msm_vfe0_rdi0":0'; do
  $M -V "$e [fmt:SBGGR10_1X10/1456x1088]"
done
# RDI only supports PACKED 10-bit Bayer -> pixelformat pBAA (NOT unpacked BG10, which EPIPEs on STREAMON)
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1456,height=1088,pixelformat=pBAA
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=5 --stream-to=/tmp/imx296.raw
# frame = 1456×1088 packed 10-bit, stride-aligned ≈ 1,984,512 bytes/frame
```
Verified: 5 live frames, ~1.98 MB each, pixel values vary frame-to-frame (real stream). Default exposure is
dark (mean ~18/255) — raise with `v4l2-ctl -d /dev/v4l-subdev<sensor>` (or the imx296 subdev) exposure/gain.

## Gotchas summary (all cost real time)
| Symptom | Cause | Fix |
|---|---|---|
| Boot loop after enabling camera | overlay `linux,cma` collides w/ cdsp/video/zap reservations | strip the `linux,cma` fragment |
| Board won't boot, wrong DTB | BLS regen picked `en7581-evb.dtb` | pin `/etc/kernel/devicetree` = qcs6490; verify BLS entry |
| `probe failed -ENOENT: failed to get clock` | driver wants clock named `inck` | `clock-names = "inck"` |
| STREAMON `-EPIPE` (Broken pipe) | video node format ≠ pad packing | use `pBAA` (packed), not `BG10` |
| STREAMON ok but 0 frames (hang) | 2-lane config, sensor drives 1 | `data-lanes = <1>` (sensor) / `<0>` (csiphy) |

## CAM3 (second camera) — ready to go
A second IMX296 on the **CAM3** connector is a one-liner — the overlay is committed and validated
(offline `fdtoverlay` merge: cdsp/video/zap intact, no CMA, sony,imx296@0x1a, 1-lane, `inck`):
```bash
cd scripts/companion/camera
./build_imx296.sh 3 && ./deploy_imx296.sh 3 && sudo reboot
```
CAM3 differs from CAM2 only in the CCI bus and CSIPHY: **CAM3 = `cci1_i2c1`** (vs CAM2 `cci1_i2c0`), and its
sensor binds to a **different CSIPHY** (CAM2 = `msm_csiphy2`). **⚠️ CAM3 gotcha (already fixed in the committed
overlay):** Radxa's cam3 template names the mclk `ext_cam_clk_imx219_**1**` (note the `_1` suffix) — the
overlay renames it to `inck` regardless, but if you regenerate from the template, match `ext_cam_clk*`.

Capture on CAM3 (find its CSIPHY first, then pick a *free* csid/rdi so it doesn't clash with CAM2):
```bash
media-ctl -d /dev/media0 -p | grep -B1 "imx296"          # shows "<- imx296 ...":0 under msm_csiphyN
# say it is csiphy3 -> route via csid1 -> vfe0_rdi1 -> /dev/video1:
M="media-ctl -d /dev/media0"
$M -l '"msm_csiphy3":1 -> "msm_csid1":0 [1]'
$M -l '"msm_csid1":1 -> "msm_vfe0_rdi1":0 [1]'
for e in '"imx296 <bus>-001a":0' '"msm_csiphy3":0' '"msm_csiphy3":1' '"msm_csid1":0' '"msm_csid1":1' '"msm_vfe0_rdi1":0'; do
  $M -V "$e [fmt:SBGGR10_1X10/1456x1088]"; done
v4l2-ctl -d /dev/video1 --set-fmt-video=width=1456,height=1088,pixelformat=pBAA \
  --stream-mmap --stream-count=5 --stream-to=/tmp/imx296_cam3.raw
```
Both cameras can run at once (CAM2→csiphy2→csid0→rdi0→video0, CAM3→csiphy3→csid1→rdi1→video1) — the CSIPHY,
CSID, and RDI resources are distinct. (CAM3 overlay committed + offline-validated; not yet live-captured — the
CSIPHY/csid/rdi numbers above are the expected mapping, confirm with `media-ctl -p` after enabling.)

---

## Color pipeline (`q6a_camstream.py`)

The IMX296 here is a **color BGGR Bayer** sensor (the two green Bayer phases match each other and read
~1.6× red/blue — that is the colour-filter array + sensor QE, not a defect). Getting neutral colour is
**deterministic**, not per-frame guessing:

- **Black level 56**, then **fixed raw white-balance gains R×1.60 / B×1.52 / G×1.0**, applied at the raw
  Bayer level *before* demosaic. Measured from raw phase statistics; scene-independent. This alone moves
  the centre from R/G 0.61 (green) to ~1.04 (neutral). These live as constants in `q6a_camstream.py`.
- Residual **radial magenta-centre / green-edge** is genuine lens shading. `--calibrate` (aim at a
  uniform, evenly-lit grey/white surface) fits a **smooth quadratic, green-relative** gain map (green
  untouched → no vignette/noise amplification), saved to `imx296_wb.npz` and applied at load.
- **Do not** reintroduce per-frame gray-world or a raw per-pixel flat-field — both are fragile and were
  explicitly rejected. The fixed WB + optional smooth shading map is the whole story.

Calibrate: `./view_q6a_cam.sh calibrate 2` (grey card) → writes/commits `imx296_wb.npz`.

## On-device YOLO detection (`q6a_yolo.py` + `build_yolo.sh`)

COCO object detection on the **Hexagon v68 NPU**, ~21 ms/inference, drawn as an overlay on the stream
(`detector_loop`, runs only while a viewer is connected; coexists with the `q6a-llmd` LLM daemon — two
HTP contexts are fine). `--no-yolo` disables it.

**The version maze (why it is not one `pip`/export step):**
- Qualcomm **AI Hub only builds QAIRT 2.45/2.46/2.47**. The Q6A runtime + `qai_appbuilder` are **2.42**
  (upgrading would disturb the working Genie stack; `pip` `qai_appbuilder` caps at 2.40). A 2.45 DLC or
  context binary **will not load on 2.42** (`Failed to create dlc handle … code 1002`).
- **Fix:** export a **w8a16 QDQ ONNX** from AI Hub, then convert **ONNX → 2.42 DLC** ourselves with the
  x86 `qairt-converter` in `~/qairt-x86` (2.42) on the Odyssey. The **converter does *not* need AVX2**
  (unlike `qairt-quantizer`, which SIGILLs on the J4125), and it preserves the QDQ encodings → a
  quantized 2.42 DLC. Then `qnn-context-binary-generator` on the Q6A builds the **v68 context binary**.
- **YOLOv11 does not work on v68/2.42:** its C2PSA attention `MatMul` requires HTP arch **≥73** (`has
  incorrect Value 68, expected >= 73`) — the same "v68 is old" wall as the 7B LLM. **Use YOLOv8** (no
  attention block → composes cleanly).
- **Model I/O (qai-hub YOLO):** input `image` **NCHW [1,3,640,640], values [0,1]**, **bottom-right
  letterbox** (centered padding roughly halves the scores). Outputs `scores[8400]`, `class_idx[8400]`,
  `boxes[8400,4]` (xyxy in 640-space). `qai_appbuilder` does float I/O — feed `[0,1]` floats, it
  quantises the input and dequantises the outputs. Post-process = threshold (~0.30) + NMS + unletterbox.

Rebuild the model end-to-end: `./build_yolo.sh yolov8_det` (Odyssey). Turnkey deploy of the prebuilt
`models/yolov8_det.bin` happens automatically via `./view_q6a_cam.sh`.

## Hardware camera acceleration on the Q6A — investigated, and why it's CPU/GPU (2026-07-05)

The software ISP (demosaic+destripe+tone-map in numpy) costs **~342 ms/frame full-res (~3 fps)**; `--fast`
half-res is ~52 ms (~19 fps). We deep-investigated using the hardware ISP/encoder and it is **not
available to us on mainline** — the documented path is RDI raw + demosaic in userspace on CPU/GPU.

- **The QCS6490 has a Spectra 570L ISP + Venus encoder** (`platform:acb3000.isp`, `platform:qcom-venus`,
  `/dev/video17`). But on **mainline `qcom-camss` it is RDI (raw Bayer) only.** The Titan ISP pixel
  pipeline (demosaic/AWB/tone) is driven by an **embedded CPU fed an undocumented, proprietary command
  stream** — kernel docs state supporting it "is beyond the current scope of CAMSS due to the amount of
  work... and the lack of documentation for the CPU command stream," and recommend userspace CPU/GPU
  post-processing. `camss-vfe-480.c`/`-680.c` (our Titan VFE) are RDI-only (`MODE_MIPI_RAW` hardcoded).
  The `msm_vfe3_pix`/`vfe4_pix` pads *enumerate* and even advertise UYVY, but `STREAMON` **hangs / yields
  0 frames** (verified experiment) — the pixel pipeline can't be programmed. Ref:
  [kernel camss docs](https://docs.kernel.org/admin-guide/media/qcom_camss.html),
  [sc7280 support (LWN)](https://lwn.net/Articles/1001452/).
- **The only route to the hardware ISP is CAMX** (`qcom-camx` + `qtiqmmfsrc`/QMMF, the Titan command-stream
  provider). Ruled out for us: (1) it needs a **proprietary CHI-CDK sensor bring-up for the IMX296** — the
  PPA ships only compiled `com.qti.sensormodule.*.bin` (imx415/476/481/519/577/586/686/766, ov9282, s5k*),
  no editable XML/`buildbins`, and the source CHI-CDK is gated; (2) the **Venus encoder reboots this board**
  on our firmware (`6.0.251230`; needs UEFI *Hypervisor Override* + fw ≥`6.0.260120`, ≤720p) —
  [Radxa forum](https://forum.radxa.com/t/gstreamer-encoder-usage-reboot-the-board/29828); (3) `qcom-camx`
  pulls `qcom-fastrpc`, which **conflicts with Radxa's `fastrpc`** (the same clash that forced our fsck).
- **Conclusion: demosaic in userspace (kernel-blessed path). Move the CPU-heavy demosaic to the Adreno GPU
  via OpenCL** (the stack already used for llama.cpp) → full-res at speed, no vendor stack, no board risk.
- **Viewer note:** view the MJPEG stream with **VLC/mpv, NOT Firefox** — snap Firefox leaks decoded frames
  into shmem (~800 MB) and the OOM killer stops it (kernel `oom-kill … task=snap.firefox`, ~845 MB shmem).

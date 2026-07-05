# Camera color tuning profiles

Ready-made, lab-calibrated camera tuning files (Raspberry Pi libcamera format). The streamer reads
`tuning/<camera-model>.json` at startup and applies its **CCM** (color correction matrix, `rpi.ccm`,
interpolated to a target colour temperature) — the cross-channel colour science that a per-channel
white balance can't do. This replaces per-unit guessing with a professional calibration.

- `imx296.json` — Sony IMX296 (our sensor), from Raspberry Pi's Global-Shutter-camera tuning
  (`raspberrypi/libcamera` `src/ipa/rpi/vc4/data/imx296.json`). Only `rpi.ccm` (+ `rpi.black_level`)
  is used here; the rest (AGC/AWB/ALSC/…) is RPi-pipeline-specific and ignored.

**To add a camera:** drop its RPi tuning JSON here as `<model>.json` and run with
`--camera-model <model>` (default: `imx296`). Source tuning files:
`github.com/raspberrypi/libcamera/tree/main/src/ipa/rpi/vc4/data`.

# Camera tuning — PROVENANCE (not used at runtime)

The runtime reads `../profiles/<model>.json` (self-contained: geometry, CFA, MIPI format, colour
defaults, AE bounds, and the CCM). This folder keeps the **original Raspberry Pi libcamera tuning**
files the CCM matrices were extracted from, for traceability / re-extraction:

- `imx296.json` — Raspberry Pi Global-Shutter-camera tuning (`raspberrypi/libcamera`
  `src/ipa/rpi/vc4/data/imx296.json`). Its `rpi.ccm.ccms` were copied into `profiles/imx296.json`.

**To add a camera:** create `profiles/<model>.json` (copy imx296.json and edit geometry/CFA/CCM).
Optionally drop the source RPi tuning here for provenance.

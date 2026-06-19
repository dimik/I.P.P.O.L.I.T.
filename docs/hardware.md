# Hardware setup

## Boards

| Board | Role | SoC | Notes |
|-------|------|-----|-------|
| Dreame D10s Pro (r2250) | Robot | AllWinner MR813 (quad Cortex-A7 @ 1.2GHz) | 3.3GB /data partition |
| Radxa Dragon Q6A | Companion / AI | Qualcomm QCS6490 (Kryo, 12 TOPS NPU) | 85×56mm, runs all ROS 2 |

## Physical connection (Q6A ↔ robot)

⚠️ **The robot exposes only ONE USB port — the OTG/debug port** (`usbc0` = `allwinner,sunxi-otg-manager`,
used for rooting). The SoC's 2nd USB controller (`usbc1`/`ehci1`) is enabled in the Tina BSP but **not
wired to a connector** (bare DT node — no port_type/detect/VBUS-drive GPIO; `usb1-vbus` is a fixed
stub; only the two root hubs in `/sys/kernel/debug/usb/devices`; nothing enumerates). So the old
"USB-host → USB-Ethernet → GbE" plan below **does not apply** — there is no spare host port.

Link options (in order of preference):
1. **USB gadget-Ethernet (wired, one cable) — needs a kernel module:** robot OTG in *device* mode →
   USB NIC to the Q6A (USB host). The gadget **core** is built-in (`USB_GADGET/LIBCOMPOSITE/CONFIGFS=y`,
   `USB_SUNXI_UDC0=y`; a `mass_storage` configfs gadget binds to the UDC fine — path verified) but **no
   ethernet function is compiled** (`CONFIG_USB_CONFIGFS_ECM/NCM/RNDIS=n`, no `g_ether`; no host-side
   `usbnet`/`r8152` either). Build a NIC gadget module vs the r2250 kernel source (Tina 4.9.191) + the
   saved `kernel/config-4.9.191.txt` (`MODVERSIONS=n` → only vermagic must match). **Use CDC-NCM**
   (`usb_f_ncm`), not ECM: ECM ~20–40 MB/s (one frame/transfer), NCM ~35–45 MB/s (aggregated).
   **Bus ceiling: USB 2.0 ≈ 40–45 MB/s** (no USB 3 on the robot). FunctionFS (`CONFIG_USB_F_FS=y`) is a
   no-kernel-build fallback (userspace raw-bulk tunnel). Feasible but a side-project.
2. **WiFi (simplest, works today):** both on the LAN; Q6A reaches the robot at `192.168.1.213`.
3. OTG→host (ID-grounded adapter) + USB-Ethernet dongle — possible but occupies the debug port,
   VBUS-on-that-port unverified.

Static IPs on a dedicated (gadget-Ethernet) link: robot `192.168.10.1`, Q6A `192.168.10.2`.
~~Robot USB 2.0 host port → USB-Ethernet adapter → Cat5e → Q6A GbE~~ (assumed a host port that the
D10s Pro does not expose).

## Power

The Dragon Q6A requires 12V, 18–30W. The robot battery is 14.8V nominal (4S LiPo).

```
Robot battery terminals (14.8V)
  └─► 12V buck converter (e.g. Mini360 or LM2596)
        └─► Dragon Q6A USB-C power input
```

The robot's USB 2.0 port cannot power the Dragon Q6A (insufficient current).

## Robot hardware interfaces

| Interface | Device | Used by |
|-----------|--------|---------|
| LiDAR (LDS turret) | `/dev/ttyS3` @ 230400 | AVA (SLAM); tapped read-only via `libserialtap.so` → `/scan` (see `docs/sensors.md`) |
| MCU (motors/IMU/odom) | `/dev/ttyS4` | AVA (`3c…3e` protocol); tappable via the same read-tap mechanism |
| Camera | `/dev/video0`, `/dev/video2` | OV8856 MIPI, V4L2 accessible |
| Speaker | `/dev/snd/pcmC0D0p` | SUNXI-CODEC, ALSA `hw:0,0` |
| WiFi | `wlan0` (Realtek 8189fs) | 2.4GHz only, single radio |

## Robot software stack

```
squashfs (read-only)   /
ext4 (writable)        /data/          3.3GB
  ├─ _root.sh                          early boot hook
  ├─ _root_postboot.sh                 late boot hook
  ├─ valetudo                          Valetudo binary (v2026.05.0)
  ├─ valetudo_config/valetudo.json
  └─ chroot/                           Ubuntu 24.04.4 arm64
       └─ (ROS 2 Jazzy installed but not used — Dragon Q6A handles ROS)
```

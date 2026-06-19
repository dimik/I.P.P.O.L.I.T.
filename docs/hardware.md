# Hardware setup

## Boards

| Board | Role | SoC | Notes |
|-------|------|-----|-------|
| Dreame D10s Pro (r2250) | Robot | AllWinner MR813 (quad Cortex-A7 @ 1.2GHz) | 3.3GB /data partition |
| Radxa Dragon Q6A | Companion / AI | Qualcomm QCS6490 (Kryo, 12 TOPS NPU) | 85×56mm, runs all ROS 2 |

## Physical connection

```
Robot USB 2.0 host port
  └─► USB-Ethernet adapter (any RTL8152/AX88179)
        └─► Cat5e patch cable
              └─► Dragon Q6A Gigabit Ethernet port
```

Assign static IPs on the dedicated link:
- Robot: `192.168.10.1`
- Dragon Q6A: `192.168.10.2`

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
| LiDAR (LDS turret) | `/dev/ttyS3` @ 230400 | AVA (SLAM); tapped read-only via `libldstap.so` → `/scan` (see `docs/sensors.md`) |
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

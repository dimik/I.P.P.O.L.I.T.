# dreame-cortex

Turning a Dreame robot vacuum into an AI platform — Valetudo integration, ROS 2 on companion board, computer vision, and custom navigation.

## Hardware

- **Robot**: Dreame D10s Pro (model r2250, AllWinner MR813 SoC, aarch64)
- **Companion board**: Radxa Dragon Q6A (Qualcomm QCS6490, 12 TOPS NPU, Adreno 643L GPU)
- **Firmware**: [Valetudo](https://valetudo.cloud) 2026.05.0 — cloud-free, local control

## Architecture

```mermaid
flowchart LR
    subgraph robot["Dreame D10s Pro (r2250)"]
        ava["AVA daemon\nhardware control"]
        valetudo["Valetudo\nREST :80 · MQTT :1883"]
        cam["/dev/video0\nOV8856 camera"]
        spk["/dev/snd\nALSA speaker"]
        gst["camera_stream.sh\nGStreamer H.264"]
        audio["audio_server.py\nTCP :9999"]
        ava <--> valetudo
        cam --> gst
        audio --> spk
    end

    subgraph companion["Radxa Dragon Q6A (12 TOPS NPU)"]
        bridge["valetudo_bridge\nMQTT → ROS 2"]
        camera_node["camera_node\n/camera/image_raw"]
        infer["inference_node\nYOLOv8 · NPU"]
        nav2["Nav2 stack"]
        behavior["behavior_node"]
        tts["tts_node\nPiper TTS"]
        bridge --> nav2
        camera_node --> infer
        infer --> behavior
        nav2 --> behavior
        behavior --> tts
    end

    valetudo -- "MQTT" --> bridge
    behavior -- "REST API" --> valetudo
    gst -- "UDP/H.264\n192.168.10.x" --> camera_node
    tts -- "WAV · TCP" --> audio
```

**Link:** Robot USB 2.0 → USB-Ethernet adapter → Dragon Q6A GbE (dedicated `192.168.10.0/24`)  
**Power:** Robot battery 14.8V → 12V buck converter → Dragon Q6A USB-C

## Repository structure

```
scripts/
  robot/
    _root.sh              early boot hook — WiFi bind-mount (deploy to /data/)
    _root_postboot.sh     late boot hook — DHCP, chroot mounts, Valetudo
    chroot.sh             enter Ubuntu 24.04 chroot on the robot
    camera_stream.sh      GStreamer pipeline: /dev/video0 → UDP H.264
    audio_server.py       TCP server: receives WAV from companion, plays via aplay
  companion/
    install_ros2.sh       install ROS 2 Jazzy on Dragon Q6A
robot/
  boot/README.md          deployment instructions for boot hooks
  valetudo/valetudo.json  Valetudo configuration
companion/
  ros2/                   ROS 2 node packages (valetudo_bridge, camera_node, …)
docs/
  hardware.md             wiring, power, physical mounting
  wifi-hack.md            why the bind-mount trick is needed for WiFi
```

## Quick start

See [docs/hardware.md](docs/hardware.md) for wiring and physical setup.  
See [robot/boot/README.md](robot/boot/README.md) for deploying boot hooks to the robot.  
See [scripts/companion/install_ros2.sh](scripts/companion/install_ros2.sh) for Dragon Q6A ROS 2 setup.

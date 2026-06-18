# Device test checklist

Manual smoke test after changing a shim, the boot hooks, or the camera stack. There is **no CI**
(the build runs inside the robot's glibc-2.23 chroot and tests need the physical robot + AVA), so
this is the verification gate. Run from the workstation; robot is `dreame-home` (192.168.1.213).

Rollback any time with `sh /data/rollback.sh` (restores stock AVA for the current boot).

## 1. Build + deploy shims
- [ ] `scp scripts/robot/{fanoff_shim.c,camsiphon.c,camstream.c,build_ava_shims.sh} dreame-home:/data/` (or `cat | ssh`)
- [ ] `ssh dreame-home 'sh /data/build_ava_shims.sh'` → builds `libfanoff_{log,filter}.so`, `libcamsiphon.so`, `/opt/camstream`; readelf shows **no GLIBC verneed** (freestanding) for the shims.
- [ ] Reload: `ssh dreame-home 'killall -9 ava'` then `'/etc/rc.d/ava.sh force'` (watchdog does NOT auto-relaunch a killed AVA).

## 2. Shims loaded into AVA
- [ ] `A=$(ssh dreame-home pidof ava)` non-empty.
- [ ] `ssh dreame-home "grep -c libfanoff_filter /proc/$A/maps"` ≥ 1, and `libcamsiphon` ≥ 1.

## 3. Fan / LiDAR (manual nav) — the core feature
- [ ] Enable manual nav in Valetudo (or `PUT HighResolutionManualControlCapability {"action":"enable"}`).
- [ ] **Fan stays SILENT** (listen) and on the wire `SetCleaning` = `00 01 00 00 00`.
- [ ] **LiDAR turret parked** during manual nav (`/tmp/lidar_allow` absent; ttyS3 reads ~0).
- [ ] Driving still responds (MotorCtrl flows); brush/pump respond.
- [ ] Start a normal clean → **LiDAR spins** again (gate sets `/tmp/lidar_allow`), fan still off.
- [ ] No fan/LiDAR start-up blip when toggling modes.

## 4. Camera — MJPEG + H.264/WebRTC
- [ ] `ssh dreame-home 'sh /data/camstream.sh status'` → running; `sh /data/go2rtc.sh status` → running.
- [ ] MJPEG: open `http://192.168.1.213:8090/` (browser) → live video. Or `curl --max-time 3 .../ -o m.bin` → contains `FFD8` JPEG frames.
- [ ] H.264 RTSP: `gst-launch-1.0 rtspsrc location=rtsp://192.168.1.213:8554/dreame ! rtph264depay ! h264parse ! avdec_h264 ! … ` decodes frames.
- [ ] WebRTC: `http://192.168.1.213:1984/` → "video+audio = simple viewer" shows video (audio is silent — no mic on this model, by design).
- [ ] **Gating**: with NO viewer, `ssh dreame-home 'ls /tmp/cam_stream'` → absent (camsiphon idle, zero AVA overhead). Connect a viewer → flag appears; disconnect → flag clears.
- [ ] Multi-client: 2–3 simultaneous RTSP/WebRTC viewers all play (go2rtc fans out one transcode).

## 5. Stock sanity + reboot persistence
- [ ] Valetudo UI reachable; robot reports state; a normal clean start/stop works.
- [ ] Reboot (`reboot`, or `reboot -f` / sysrq `b` if `/data` I/O is wedged) → after boot: AVA up, shims loaded (§2), camstream + go2rtc auto-started (§4), manual-nav fan-off still holds (§3).

## 6. Rollback works
- [ ] `ssh dreame-home 'sh /data/rollback.sh'` → reports "no shims loaded — AVA is stock"; fan behaves stock (spins in manual nav again); camera daemons stopped. Reboot re-applies the shims.

## Watch-outs (learned the hard way)
- `kill -9 ava` does **not** auto-relaunch — use `/etc/rc.d/ava.sh force`.
- Never run `v4l2grab` / reconfigure `/dev/video0` while AVA streams — **deadlocks the kernel** (D-state, reboot only).
- Sustained `/data` (eMMC) write load can wedge the FS (jbd2 + procs in D-state) — recover with sysrq reboot; keep chroot build/deploy churn lean.
- Launcher `pkill` patterns must target `/opt/<binary>`, not the bare name (else the `*.sh` script self-matches and SIGKILLs itself).

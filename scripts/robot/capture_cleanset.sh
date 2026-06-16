#!/bin/sh
# capture_cleanset.sh — Phase-1 capture harness for the MCU CLEANSET fan-byte filter.
#
# Deploys the LOG-mode shim onto AVA, drives the robot through known fan states, and pulls
# the captured 55 AA frames for offline decode. The decoded frames feed the FRAME SPEC
# constants in fanoff_shim.c (CLEANSET_CMD, FAN_OFFSET, checksum). See CLAUDE.md.
#
# PREREQUISITES (run first):
#   1. scp scripts/robot/fanoff_shim.c build_ava_shims.sh   dreame-home:/data/
#   2. ssh dreame-home 'sh /data/build_ava_shims.sh'        # builds /data/lib/libfanoff_log.so
#   3. Patched ava.sh in place exporting LD_PRELOAD=/data/lib/libfanoff_log.so (see NOTE).
#
# NOTE on injection: AVA is launched by /etc/rc.d/ava.sh (squashfs). To preload the LOG shim
# for a capture WITHOUT a reboot, restart AVA manually with the env set:
#     killall -9 ava
#     cd /data && LD_PRELOAD=/data/lib/libfanoff_log.so ava -f /ava/conf/r2250.conf force &
# (For the production FILTER shim we instead bind-mount a patched ava.sh at boot — see
#  deploy_ava_shims.sh — so the watchdog's restarts also get the preload.)
#
# !! REQUIRES A HUMAN: you must listen and confirm fan ON/OFF at each step, and ideally set
#    each fan-speed preset so each speed maps to a distinct byte value. Robot stays docked
#    /stationary throughout (manual-control enable spins the fan with spdv=0).
#
# Usage: sh capture_cleanset.sh   (run ON the robot, or via ssh dreame-home 'sh -s' < this)

LOG=/tmp/mcu_tx.log
API=http://localhost/api/v2/robot/capabilities

echo "=== fan-off baseline (idle/docked). Confirm fan is SILENT. Capturing 5s... ==="
: > "$LOG"
sleep 5
cp "$LOG" /tmp/cap_fanoff.bin
echo "saved /tmp/cap_fanoff.bin ($(wc -c < /tmp/cap_fanoff.bin) bytes)"

echo "=== enabling manual control (fan should turn ON, robot stationary) ==="
curl -s -X PUT "$API/HighResolutionManualControlCapability" \
  -H 'Content-Type: application/json' -d '{"operation":"enable"}'
echo "  -> LISTEN: is the fan running now? Capturing 5s..."
: > "$LOG"
sleep 5
cp "$LOG" /tmp/cap_fanon.bin
echo "saved /tmp/cap_fanon.bin ($(wc -c < /tmp/cap_fanon.bin) bytes)"

# Optional: sweep fan-speed presets to map speed -> byte (each preset = distinct fan byte).
for SP in low medium high max; do
    echo "=== fan preset $SP ==="
    curl -s -X PUT "$API/FanSpeedControlCapability/preset" \
      -H 'Content-Type: application/json' -d "{\"name\":\"$SP\"}"
    : > "$LOG"; sleep 3
    cp "$LOG" "/tmp/cap_fan_$SP.bin"
    echo "saved /tmp/cap_fan_$SP.bin ($(wc -c < /tmp/cap_fan_$SP.bin) bytes)"
done

echo "=== disabling manual control ==="
curl -s -X PUT "$API/HighResolutionManualControlCapability" \
  -H 'Content-Type: application/json' -d '{"operation":"disable"}'

echo
echo "=== DONE. Pull captures to the laptop for decode: ==="
echo "  scp dreame-home:/tmp/cap_*.bin ./captures/"
echo "Decode hint: frames are separated by the 0xFF 0xFF marker the shim writes."
echo "  xxd /tmp/cap_fanon.bin | less   # find 55 AA frames, diff fanoff vs fanon"
echo "Look for: the CLEANSET command id (byte after 55 AA <len>), the byte that tracks"
echo "fan speed across cap_fan_{low,medium,high,max}.bin, and the trailing checksum byte."

#!/bin/sh
# fanoff_flag.sh — LiDAR gate for the fanoff shim (event-driven via Valetudo SSE).
#
# The fanoff shim filters the vacuum fan UNCONDITIONALLY (off in every mode) and blocks the
# LiDAR turret BY DEFAULT, allowing it only while /tmp/lidar_allow exists. This gate keeps the
# LiDAR running in active non-manual modes (mapping / go-to / autonomous nav) and blocks it for
# manual_control / idle / docked / parked.
#
# Event-driven: it holds a single Server-Sent-Events stream
# (GET /api/v2/robot/state/attributes/sse) and reacts the instant Valetudo pushes a state change
# — no polling, no `sleep` between checks. The `while` is only a reconnect supervisor: curl
# blocks inside the stream for its whole lifetime and we only loop to re-open it if it drops
# (e.g. Valetudo restart). Reacting instantly also makes the block/allow transition crisper than
# a timed poll.
#
# Race note: manual_control AND idle both map to "block", so entering manual navigation never
# flips the LiDAR allowed->blocked mid-session — the turret is simply never spun up in manual
# nav, so there is no spin-up blip.
#
# Launched in the background from _root_postboot.sh after Valetudo starts.

ALLOW=/tmp/lidar_allow
URL=http://localhost/api/v2/robot/state/attributes/sse

while true; do
    curl -sN -H "Accept: text/event-stream" "$URL" 2>/dev/null | while IFS= read -r line; do
        case "$line" in
            data:*)
                case "$line" in
                    # manual / parked statuses -> block LiDAR
                    *'"value":"manual_control"'* | *'"value":"idle"'* | *'"value":"docked"'* \
                        | *'"value":"paused"'* | *'"value":"error"'* | *'"value":"sleeping"'*)
                        rm -f "$ALLOW" ;;
                    # has a status, and it isn't a blocked one -> active mode -> allow LiDAR
                    *StatusStateAttribute*)
                        : > "$ALLOW" ;;
                esac ;;
        esac
    done
    sleep 2   # stream dropped (e.g. Valetudo restart) — reconnect
done

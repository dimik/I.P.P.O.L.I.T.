#!/bin/sh
# fanoff_flag.sh — LiDAR gate for the fanoff shim (event-driven via Valetudo SSE).
#
# The fanoff shim filters the vacuum fan UNCONDITIONALLY (off in every mode) and blocks the
# LiDAR turret BY DEFAULT, allowing it only while /tmp/lidar_allow exists. This gate keeps the
# LiDAR running in active non-manual modes (mapping / go-to / autonomous nav) and blocks it
# while the robot is manually driven or parked.
#
# It is EVENT-DRIVEN: a single Server-Sent-Events connection
# (GET /api/v2/robot/state/attributes/sse) stays open and the read blocks until Valetudo pushes
# a StateAttributesUpdated event — there is no polling and no sleep on the hot path. The outer
# `while` is ONLY a reconnect supervisor: `curl` blocks inside the stream for the connection's
# entire lifetime, so the loop body runs just once per disconnect (e.g. a Valetudo restart) —
# possibly never. (A long-lived connection can always drop, so some respawn is unavoidable;
# this is a supervisor, not a busy poll.)
#
# Race note: manual_control AND idle are both in BLOCKED_STATES, so entering manual navigation
# never flips the LiDAR allowed->blocked mid-session — the turret is simply never spun up during
# manual nav, so there is no spin-up blip.
#
# Launched in the background from _root_postboot.sh after Valetudo starts.

ALLOW=/tmp/lidar_allow
SSE_URL=http://localhost/api/v2/robot/state/attributes/sse

# Robot statuses in which the LiDAR is kept OFF (manual driving + parked). Every other status
# (cleaning / returning / moving / mapping / ...) is treated as "active" and allows the LiDAR.
BLOCKED_STATES="manual_control idle docked paused error sleeping"

# Is the LiDAR blocked for this SSE data line? (line contains the full state attributes JSON)
status_is_blocked() {
    for st in $BLOCKED_STATES; do
        case "$1" in
            *"\"value\":\"$st\""*) return 0 ;;
        esac
    done
    return 1
}

while true; do
    curl -sN -H "Accept: text/event-stream" "$SSE_URL" 2>/dev/null | while IFS= read -r line; do
        case "$line" in
            data:*StatusStateAttribute*)
                if status_is_blocked "$line"; then
                    rm -f "$ALLOW"        # manual / parked -> block LiDAR
                else
                    : > "$ALLOW"          # active mode    -> allow LiDAR
                fi ;;
        esac
    done
    sleep 2   # reached only if the SSE stream dropped — reconnect
done

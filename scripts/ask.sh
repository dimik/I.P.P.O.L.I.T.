#!/bin/sh
# ask.sh — talk to IPPOLIT by text: Worker LLM answers, the robot speaks the reply (Piper).
#   scripts/ask.sh "how is your battery"
#   scripts/ask.sh "jedź do telewizora"          # replies in Polish with the gosia voice
# Flow: /text on the voice Worker (context = live battery/status from Valetudo) -> {reply, voice}
#       -> ssh Q6A -> ssh robot -> chroot speak.py (piper -> mediad). docs/voice-cloud.md.
# Env overrides: EP (worker URL), Q6A (ssh target, e.g. radxa@192.168.1.243), ROBOT_SSH
#       (alias ON the Q6A: robot-wifi|robot-usb), ROBOT_HTTP (Valetudo base for context).
# NB: actions in the reply are printed but NOT executed — that's q6a_voice.py's job.
set -eu
[ $# -ge 1 ] || { echo "usage: $0 \"text to say to the robot\"" >&2; exit 2; }

DIR=$(cd "$(dirname "$0")/.." && pwd)
EP=${EP:-https://ippolit-voice.poklonskiydmitry.workers.dev}
Q6A=${Q6A:-q6a}
ROBOT_SSH=${ROBOT_SSH:-robot-wifi}
ROBOT_HTTP=${ROBOT_HTTP:-http://192.168.1.213}
TOK=${AUTH_TOKEN:-$(sed -n 's/^AUTH_TOKEN=//p' "$DIR/cloud/voice-worker/.dev.vars" 2>/dev/null)}
[ -n "$TOK" ] || { echo "no AUTH_TOKEN (set env or cloud/voice-worker/.dev.vars)" >&2; exit 1; }

# best-effort live context from Valetudo (battery/status); empty context if unreachable
CTX=$(curl -sm 3 "$ROBOT_HTTP/api/v2/robot/state/attributes" 2>/dev/null | python3 -c '
import json,sys
ctx={}
try:
    for a in json.load(sys.stdin):
        if a.get("__class")=="BatteryStateAttribute": ctx["battery"]=a.get("level")
        if a.get("__class")=="StatusStateAttribute": ctx["status"]=a.get("value")
except Exception: pass
print(json.dumps(ctx))' 2>/dev/null || echo '{}')

RESP=$(printf '%s' "$1" | python3 -c '
import json,sys; print(json.dumps({"text": sys.stdin.read(), "context": json.loads(sys.argv[1])}))' "$CTX" |
  curl -sm 60 "$EP/text" -H "Authorization: Bearer $TOK" -H 'Content-Type: application/json' --data-binary @-)

echo "$RESP" | python3 -c 'import json,sys; d=json.load(sys.stdin)
print("reply :", d.get("reply")); print("voice :", d.get("voice"))
print("actions:", json.dumps(d.get("actions"))) if d.get("actions") else None'

echo "$RESP" | python3 -c 'import json,sys; d=json.load(sys.stdin)
sys.stdout.write("%s: %s" % (d.get("voice","amy"), d.get("reply","")))' |
  ssh -i ~/.ssh/id_ed25519_q6a -o ConnectTimeout=8 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$Q6A" \
    "ssh -o ConnectTimeout=8 $ROBOT_SSH 'SPEAK_VOL=90 chroot /data/chroot python3 /opt/speak.py'" 2>&1 | grep -v '^Warning:' || true

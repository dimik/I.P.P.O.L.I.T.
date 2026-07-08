# Voice control via Cloudflare Workers AI (STT + LLM)

Cloud brain for voice-controlling IPPOLIT, replacing the retired on-device 1B LLM (see
`docs/companion-autonomy.md` — "cloud (Cloudflare Workers AI planned)"). A personal-account
Cloudflare Worker does speech-to-text + intent parsing in **one HTTPS round trip**; everything
robot-side stays on the Q6A companion and reuses infrastructure we already run
(`/robot/speak`, `/object_map`, Valetudo REST).

## Decision summary

| Decision | Choice | Why |
|----------|--------|-----|
| Where the mic lives | **USB mic on the Q6A** | The D10s Pro has **no functional microphone** (both codec + DMIC paths dead — `docs/sensors.md`); the Q6A has no onboard mic either. ReSpeaker USB array for far-field, or any USB mic to start. |
| STT | `@cf/openai/whisper-large-v3-turbo` on Workers AI | Multilingual, auto language detect, `vad_filter`, $0.00051/audio-minute. On-device Whisper on the Q6A would fight the vision pipeline for CPU/thermal headroom (NPU throttle ladder is already load-bearing). |
| LLM | `@cf/meta/llama-3.3-70b-instruct-fp8-fast` | On the Workers AI **JSON Mode supported list** (schema-forced output → no parse failures) + strong multilingual intent parsing. Far above the "1B too weak for agentic decisions" line that killed the on-device agent. ⚠️ llama-4-scout was tried first and **silently ignores `response_format`** (not on the JSON-mode list) — it free-styles the JSON shape; don't switch models without checking that list. |
| API shape | **One Worker, one round trip**: audio in → `{transcript, reply, voice, actions}` out | Two separate calls (STT, then chat) would double latency and put the intent loop on the companion. The Worker chains Whisper → Llama internally on Cloudflare's network. |
| Tool use | **`guided_json` single-shot, NOT a tool-call loop** | Commands are narrow (dock/stop/goto/answer). A schema-forced single response is cheaper, faster, and immune to the hallucinated-tool-name failure mode we hit with the offline agent. |
| Auth | Shared bearer secret (`AUTH_TOKEN` Worker secret ↔ `/etc/default/ippolit-voice`) | Single client, personal account. Rotate with `wrangler secret put`. |
| TTS | **Stays as-is** (robot Piper/espeak via `/robot/speak` → audio_bridge → mediad) | Already deployed and serialized with AVA's own prompts. Cloud TTS (Deepgram Aura/MeloTTS) is a later option if Piper's ~RTF 1 feels slow. |

## Status — deployed & verified (2026-07-08)

Live at **`https://ippolit-voice.poklonskiydmitry.workers.dev`** (personal account
poklonskiydmitry@gmail.com, free tier). Verified end-to-end the same day:

- `/healthz` ✅; missing/wrong bearer → 401 ✅.
- `/text` intent suite ✅: "go to the chair" → `goto_point{1.9,-0.4}`; "how is your battery" →
  answered from context, no action; "bring me a beer" → correctly refuses (unknown object, no
  action); "jedź do telewizora" → **Polish reply, `gosia` voice, correct tv coordinates**;
  "stop right now" → `stop`.
- `/voice` with real spoken audio (macOS `say` → 16 kHz WAV) ✅: perfect transcript, `en`
  detected, correct action — **3.8 s round trip** (upload + Whisper + Llama).
- Robot spoke a cloud-generated reply through Piper via `scripts/ask.sh` ✅ (full chain:
  Worker → Q6A → robot `speak.py` → mediad).
- `LocateCapability` payload verified live: `{"action":"locate"}` → 200, empty body → 400
  (CLAUDE.md table corrected).

**The bearer secret** lives in `cloud/voice-worker/.dev.vars` (gitignored) and as the Worker's
`AUTH_TOKEN` secret; copy it into `/etc/default/ippolit-voice` on the Q6A when deploying the mic
node. Rotate: `openssl rand -hex 32` → `wrangler secret put AUTH_TOKEN` → update both copies.

Two implementation gotchas hit during bring-up (both handled in `src/index.ts`, kept here so they
aren't re-learned):
1. **JSON mode is model-gated** — models not on the supported list (llama-4-scout) silently ignore
   `response_format` and invent their own JSON nesting.
2. **In JSON mode the AI binding returns `response` as an already-parsed object**, not a string —
   `JSON.parse` on it throws. The Worker handles both shapes.

Still pending (needs hardware / a live drive): USB mic on the Q6A, and one calibration run of the
`goto_point` meters→mm sign convention (deployment checklist step 6).

## Usage without a mic (today)

**Text → robot speaks the reply (Piper)** — `scripts/ask.sh`:

```sh
scripts/ask.sh "how is your battery"        # answers with live battery from Valetudo
scripts/ask.sh "jedź do telewizora"         # Polish reply in the gosia voice
# env overrides: EP, Q6A (ssh target, e.g. radxa@192.168.1.243 when mDNS is flaky),
#                ROBOT_SSH (robot-wifi|robot-usb, alias ON the Q6A), ROBOT_HTTP
```

It fetches battery/status from Valetudo as context, calls `/text`, prints `reply/voice/actions`,
and pipes `voice: reply` over ssh (Q6A → robot → `chroot speak.py`). Actions are **printed, not
executed** — driving belongs to `q6a_voice.py`.

**Raw curl** (see `cloud/voice-worker/README.md` for the `/voice` audio variant):

```sh
source <(sed 's/^/export /' cloud/voice-worker/.dev.vars)
curl -s https://ippolit-voice.poklonskiydmitry.workers.dev/text \
  -H "Authorization: Bearer $AUTH_TOKEN" -H 'Content-Type: application/json' \
  -d '{"text":"go to the chair","context":{"battery":84,"status":"docked",
       "objects":[{"label":"chair","x":1.9,"y":-0.4}]}}' | jq
```

## Architecture

```
      Q6A companion (ROS 2 Jazzy)                    Cloudflare (personal account)
┌───────────────────────────────────────┐    ┌──────────────────────────────────────┐
│ USB mic ─► q6a_voice.py               │    │ ippolit-voice Worker                 │
│   arecord 16k mono + energy VAD       │    │  POST /voice  (Bearer AUTH_TOKEN)    │
│   (or push-to-talk: /voice/trigger)   │    │   1. Whisper large-v3-turbo (STT)    │
│        │  WAV b64 + context ───HTTPS──┼────┼─► 2. Llama-4-Scout, guided_json      │
│        │                              │    │      system prompt = robot persona   │
│   context:                            │    │      + live context JSON             │
│    /object_map (q6a_objmap)  ─────────┤    │   3. {transcript, reply, voice,      │
│    Valetudo REST attrs (battery,      │ ◄──┼──────actions[]}                      │
│      status via ROBOT_ADDR)           │    └──────────────────────────────────────┘
│        ▼                              │
│   reply ─► /robot/speak ─► audio_bridge ─ssh─► robot speak.py (piper/espeak→mediad)
│   actions ─► Valetudo REST (robot):   │
│     dock/stop/pause ─► BasicControl   │
│     locate          ─► Locate         │
│     goto_point(m)   ─► GoTo (mm)      │
└───────────────────────────────────────┘
```

The robot itself is untouched: it already exposes everything needed (Valetudo REST on port 80,
TTS via the existing `/robot/speak` path). No new robot-side code.

### Request / response contract

`POST /voice` (or `/text` to skip STT — testing and typed commands):

```jsonc
// request
{
  "audio_b64": "<16 kHz mono WAV>",
  "language_hint": "en",              // optional; Whisper auto-detects otherwise
  "context": {                        // best-effort, built fresh per utterance
    "battery": 84, "status": "docked",
    "objects": [{"label": "chair", "x": 1.9, "y": -0.4}]   // ROS map-frame meters, from /object_map
  },
  "history": [{"role": "user", "content": "..."}]          // last ≤6 turns, kept client-side
}
// response
{
  "transcript": "go to the chair", "language": "en",
  "reply": "On my way to the chair.",
  "voice": "amy",                     // amy|thorsten|gosia|davefx|espeak — the robot's actual voices
  "actions": [{"type": "goto_point", "x": 1.9, "y": -0.4, "label": "chair"}]
}
```

Action vocabulary (schema-enforced in the Worker, executed by `q6a_voice.py`):

| action | executed as | notes |
|--------|-------------|-------|
| `dock` / `stop` / `pause` | `BasicControlCapability` `home`/`stop`/`pause` | |
| `locate` | `LocateCapability` `{"action":"locate"}` | robot beeps ("where are you") |
| `goto_point {x,y}` | `GoToLocationCapability` `{"coordinates":{x·1000, −y·1000}}` | LLM emits **ROS map meters** (object-map frame); companion converts via the inverse of `valetudo_bridge.py`'s transform (`/1000`, y-flip). ⚠️ verify sign/scale on the live map once before trusting it. |
| `none` | nothing | ambiguous request → the reply asks a clarifying question instead |

Language handling: the LLM replies in the user's language when a Piper voice exists (en/de/pl/es →
amy/thorsten/gosia/davefx) and picks that voice; anything else falls back to English. **Piper does
not translate** (`docs/audio.md`) — that's why voice choice lives with the LLM, which writes the
reply in the matching language.

### Why context-in-prompt instead of tool-calls for queries

Battery, status, and the object map are small (a few hundred tokens) and change slowly, so the
companion snapshots them into every request. "How's your battery?" / "What do you see?" are then
answered in the same single LLM call — no second round trip, no tool loop. If context ever grows
past ~2k tokens (big object maps), switch to sending only object labels + a `find_object` action
the companion resolves locally.

## Repo layout

```
cloud/voice-worker/          the Cloudflare Worker (deploy: see its README.md)
  wrangler.jsonc             AI binding, name ippolit-voice
  src/index.ts               /voice /text /healthz — Whisper → Llama JSON-mode
  .dev.vars                  AUTH_TOKEN (gitignored — the live bearer secret)
scripts/
  ask.sh                     workstation: text → Worker → robot speaks the reply (no mic needed)
scripts/companion/
  q6a_voice.py               mic capture + VAD → Worker → /robot/speak + Valetudo actions
  systemd/q6a-voice.service  unit (EnvironmentFile /etc/default/ippolit-voice)
  systemd/ippolit-voice.env.example   VOICE_ENDPOINT / VOICE_TOKEN / VOICE_MIC template
```

## Deployment checklist

**Cloud (once, from the Mac):**
1. `cd cloud/voice-worker && npm i -D wrangler && npx wrangler login` (personal account)
2. `openssl rand -hex 32` → `npx wrangler secret put AUTH_TOKEN`
3. `npx wrangler deploy` → note the `https://ippolit-voice.<subdomain>.workers.dev` URL
4. Smoke-test the `/text` path (curl in `cloud/voice-worker/README.md`)

**Companion:**
1. Plug in a USB mic; `arecord -L` to find the device; sanity: `arecord -D <dev> -f S16_LE -r 16000 -c1 -d 3 t.wav`
2. `scp scripts/companion/q6a_voice.py q6a:~/ros/`
3. Create `/etc/default/ippolit-voice` from `ippolit-voice.env.example` (endpoint, token, mic; `chmod 600`)
4. `scp scripts/companion/systemd/q6a-voice.service q6a:` → `/etc/systemd/system/` →
   `sudo systemctl daemon-reload && sudo systemctl enable --now q6a-voice`
5. Verify: say "what's your battery level" → spoken answer; `journalctl -u q6a-voice -f` shows
   `heard: ...`; `ros2 topic echo /voice/transcript` for debugging.
6. **Calibrate `goto_point` once**: "go to the <object>" with a known object; if the robot heads the
   wrong way, fix the sign convention in `q6a_voice.execute()` (and this doc).

## Cost & latency budget

- **Free tier: 10,000 neurons/day.** A typical command ≈ 5 s audio (Whisper: fraction of a cent-
  equivalent in neurons) + ~600 input / ~80 output LLM tokens. Order-of-magnitude: **hundreds of
  voice commands/day fit inside the free allocation**; heavy days cost cents.
- **Latency ≈ 2–4 s end-to-end**: VAD tail 0.8 s + upload ~160 KB WAV (fine even on the robot's
  WiFi path) + Whisper ~1 s + Llama ~1 s + Piper synth on the robot (~RTF 1, `espeak` voice is
  instant). First request after idle adds 1–3 s model cold start.
- Trim options if it feels slow: `whisper-tiny-en` (English-only, faster), shorter VAD tail,
  espeak default voice, or Q6A-side Piper synth (`companion/tts_speak.sh` path).

## Security

- Worker URL is public but useless without the bearer secret; 401 on mismatch, `/healthz` is the
  only open route. Secret lives in a Worker secret + `/etc/default/ippolit-voice` (600, root) —
  **never in the repo** (GitHub remote).
- Audio leaves the LAN (that's the point) — no always-on streaming though: only VAD-gated
  utterances are uploaded, nothing is stored by the Worker.
- Optional hardening later: route via **AI Gateway** (per-model logs, caching, rate limits) or a
  Cloudflare Access service token instead of the bearer header.

## Failure modes

| Failure | Behavior |
|---------|----------|
| Cloud/WiFi down | `q6a_voice` says "cloud unreachable" (espeak, instant) and keeps listening. No offline command fallback by design — the on-device 1B was retired for exactly this role. |
| Silence / noise triggers VAD | Whisper returns empty text → Worker returns without calling the LLM (no neurons burned), companion stays quiet. |
| Robot speaker triggers the mic | `busy_until` mutes the VAD for the estimated speech duration after each reply. If echo still self-triggers, lower mic gain or switch `listen_mode:=trigger`. |
| Valetudo command fails (e.g. work_mode quirk) | Spoken "command failed" + warn in the journal. |
| Free-tier exhausted (HTTP 7505/429) | Treated as "cloud unreachable"; resets daily. |

## Roadmap / phases

1. **Now (this doc)**: VAD or push-to-talk (`ros2 topic pub --once /voice/trigger std_msgs/msg/Empty {}`),
   dock/stop/pause/locate/goto-object, status Q&A, 4-language replies.
2. **Wake word**: openWakeWord on the Q6A CPU ("hey Ippolit") gating the VAD — removes false
   triggers, no cloud cost when idle. Revisit mic hardware (ReSpeaker array) for far-field.
3. **Room targets**: compute Valetudo segment centroids on the companion (from the map JSON the
   bridge already parses) and add them to context → "go to the kitchen".
4. **Richer actions**: camera describe ("what's in front of you" → attach a `q6a-vision` frame to
   a vision-capable model, e.g. `llama-3.2-11b-vision-instruct` — the main LLM path stays
   JSON-mode), manual-drive verbs, patrol routines.
5. **Streaming/conversational**: Deepgram Flux ("built for voice agents") + Aura TTS on the same
   Workers AI account if turn-based ever feels too slow.

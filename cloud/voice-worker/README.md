# ippolit-voice — Cloudflare Worker (STT + LLM for voice control)

One HTTPS round trip: WAV → Whisper (`@cf/openai/whisper-large-v3-turbo`) → Llama 3.3 70B
(`@cf/meta/llama-3.3-70b-instruct-fp8-fast`, JSON mode) → `{transcript, reply, voice, actions}`.
Full architecture + companion integration: [`../../docs/voice-cloud.md`](../../docs/voice-cloud.md).

**Deployed & verified 2026-07-08**: `https://ippolit-voice.poklonskiydmitry.workers.dev`
(~3.8 s /voice round trip). Bearer secret: `.dev.vars` here (gitignored) = the Worker's
`AUTH_TOKEN` secret. ⚠️ Model swap gotcha: the LLM **must** be on the Workers AI *JSON Mode*
supported list — llama-4-scout is not and silently ignores `response_format`; also note the AI
binding returns `response` as an already-parsed **object** in JSON mode (index.ts handles both).

## Deploy (personal account, free tier)

```sh
cd cloud/voice-worker
npm i -D wrangler typescript @cloudflare/workers-types   # first time
npx wrangler login                                       # personal Cloudflare account
openssl rand -hex 32                                     # generate the shared secret
npx wrangler secret put AUTH_TOKEN                       # paste it
npx wrangler deploy                                      # -> https://ippolit-voice.<subdomain>.workers.dev
```

Dev loop: `npx wrangler dev --remote` (AI models only run remotely, `--remote` is mandatory).

## Test

```sh
EP=https://ippolit-voice.<subdomain>.workers.dev
TOK=<the secret>

# no-STT path (typed command):
curl -s $EP/text -H "Authorization: Bearer $TOK" -H 'Content-Type: application/json' -d '{
  "text": "go to the chair",
  "context": {"battery": 84, "status": "docked",
              "objects": [{"label": "chair", "x": 1.9, "y": -0.4}]}
}' | jq

# full voice path (16 kHz mono WAV):
base64 -i cmd.wav | jq -Rs '{audio_b64: .}' | \
  curl -s $EP/voice -H "Authorization: Bearer $TOK" -H 'Content-Type: application/json' -d @- | jq
```

Expected: `{"transcript":"go to the chair","reply":"On my way to the chair.","voice":"amy",
"actions":[{"type":"goto_point","x":1.9,"y":-0.4,"label":"chair"}]}`.

## Notes

- Auth = single shared bearer secret (`AUTH_TOKEN`); rotate with `wrangler secret put`. The companion
  reads it from `/etc/default/ippolit-voice` (never commit it — this repo is on GitHub).
- Free tier = 10,000 neurons/day; a 5-second command (STT + ~500 LLM tokens) costs a small fraction of
  that — hundreds of commands/day fit. Cost math in the doc.
- `goto_point` coordinates are ROS map-frame meters; the **companion** converts to Valetudo GoTo mm.

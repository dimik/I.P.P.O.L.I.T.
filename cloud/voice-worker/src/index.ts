// IPPOLIT voice endpoint — one round trip: audio -> Whisper STT -> Llama (guided JSON) -> {reply, actions}.
// Called by scripts/companion/q6a_voice.py on the Q6A. Architecture: docs/voice-cloud.md.
//
// POST /voice  {audio_b64, language_hint?, context?, history?} -> {transcript, language, reply, voice, actions}
// POST /text   {text, language_hint?, context?, history?}      -> same, minus STT (testing / typed commands)
// GET  /healthz

export interface Env {
  AI: Ai;
  AUTH_TOKEN: string; // wrangler secret put AUTH_TOKEN
}

const STT_MODEL = '@cf/openai/whisper-large-v3-turbo';
// NB: must be a model on the Workers AI "JSON Mode" supported list — llama-4-scout is NOT (it
// silently ignores response_format and free-styles the JSON shape; hit 2026-07-08).
const LLM_MODEL = '@cf/meta/llama-3.3-70b-instruct-fp8-fast';

// Voices the robot can actually speak (audio_bridge -> robot piper/espeak). Keep in sync with docs/audio.md.
const VOICES = ['amy', 'thorsten', 'gosia', 'davefx', 'espeak'] as const;

// Actions the companion executes (q6a_voice.py). goto_point is in ROS map-frame METERS —
// the companion converts to Valetudo GoTo mm (x*1000, -y*1000).
const RESPONSE_SCHEMA = {
  type: 'object',
  properties: {
    reply: { type: 'string', description: 'Short spoken reply to the user, in their language if supported' },
    voice: { type: 'string', enum: [...VOICES] },
    actions: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          type: { type: 'string', enum: ['dock', 'stop', 'pause', 'locate', 'goto_point', 'none'] },
          x: { type: 'number', description: 'goto_point only: ROS map-frame x in meters' },
          y: { type: 'number', description: 'goto_point only: ROS map-frame y in meters' },
          label: { type: 'string', description: 'goto_point only: what the target is, for logging/confirmation' },
        },
        required: ['type'],
      },
    },
  },
  required: ['reply', 'voice', 'actions'],
} as const;

interface RobotContext {
  battery?: number;
  status?: string;
  pose?: { x: number; y: number };
  segments?: { id: string; name: string }[];
  objects?: { label: string; x: number; y: number; [k: string]: unknown }[];
}

function systemPrompt(ctx: RobotContext | undefined): string {
  return `You are IPPOLIT, a Dreame robot vacuum converted into an autonomous AI rover (vacuum fan permanently disabled — you drive, see and talk, you do not clean). You receive one transcribed voice command and must answer with JSON only, matching the given schema.

Rules:
- "reply" is SPOKEN through a speaker: keep it to one short sentence, no markdown, no emoji.
- Reply in the user's language when it is English/German/Polish/Spanish and pick the matching voice (amy=English, thorsten=German, gosia=Polish, davefx=Spanish). Any other language: reply in English with voice "amy".
- Only emit movement actions the user clearly asked for. If the request is ambiguous or the target is unknown, emit no action (type "none") and ask a short clarifying question instead.
- "go to <thing>": find the thing in context.objects and emit goto_point with its x/y (meters) and its label. If it is not in the object list, say you do not know where it is.
- "come home" / "dock" / "charge" -> dock. "stop" / "freeze" / "halt" -> stop. "where are you" -> locate (the robot beeps) plus a spoken answer from context. Battery/status questions: answer directly from context, no action.
- Answer questions about what you can see using context.objects.

Robot state (JSON): ${JSON.stringify(ctx ?? {})}`;
}

type Msg = { role: 'system' | 'user' | 'assistant'; content: string };

async function runBrain(env: Env, text: string, ctx: RobotContext | undefined, history: Msg[] | undefined) {
  const messages: Msg[] = [
    { role: 'system', content: systemPrompt(ctx) },
    ...(history ?? []).slice(-6).filter((m) => m.role !== 'system'),
    { role: 'user', content: text },
  ];
  const res = (await env.AI.run(LLM_MODEL as Parameters<Ai['run']>[0], {
    messages,
    max_tokens: 300,
    temperature: 0.2,
    response_format: { type: 'json_schema', json_schema: RESPONSE_SCHEMA },
  } as never)) as { response?: string | Record<string, unknown> };
  try {
    // In JSON mode the binding returns response as an already-parsed object (string otherwise).
    let parsed = typeof res.response === 'string' ? JSON.parse(res.response) : (res.response ?? {});
    // Defensive unwrap: weaker schema adherence sometimes nests the whole object under "reply".
    if (parsed.reply && typeof parsed.reply === 'object') parsed = { ...parsed, ...parsed.reply };
    return {
      reply: String(parsed.reply ?? ''),
      voice: VOICES.includes(parsed.voice) ? parsed.voice : 'amy',
      actions: Array.isArray(parsed.actions) ? parsed.actions.filter((a: { type?: string }) => a?.type && a.type !== 'none') : [],
    };
  } catch {
    // Guided JSON should prevent this; degrade to a speak-only answer.
    const raw = typeof res.response === 'string' ? res.response : 'Sorry, I had trouble thinking about that.';
    return { reply: raw, voice: 'amy', actions: [] };
  }
}

function unauthorized() {
  return Response.json({ error: 'unauthorized' }, { status: 401 });
}

export default {
  async fetch(request: Request, env: Env): Promise<Response> {
    const url = new URL(request.url);
    if (url.pathname === '/healthz') return Response.json({ ok: true });

    const auth = request.headers.get('Authorization') ?? '';
    if (!env.AUTH_TOKEN || auth !== `Bearer ${env.AUTH_TOKEN}`) return unauthorized();
    if (request.method !== 'POST') return Response.json({ error: 'POST only' }, { status: 405 });

    let body: {
      audio_b64?: string;
      text?: string;
      language_hint?: string;
      context?: RobotContext;
      history?: Msg[];
    };
    try {
      body = await request.json();
    } catch {
      return Response.json({ error: 'invalid JSON body' }, { status: 400 });
    }

    let transcript = '';
    let language = body.language_hint ?? '';

    if (url.pathname === '/voice') {
      if (!body.audio_b64) return Response.json({ error: 'audio_b64 required' }, { status: 400 });
      const stt = (await env.AI.run(STT_MODEL as Parameters<Ai['run']>[0], {
        audio: body.audio_b64,
        task: 'transcribe',
        ...(body.language_hint ? { language: body.language_hint } : {}),
        vad_filter: true,
        initial_prompt:
          'Voice commands to IPPOLIT, a robot vacuum rover: stop, dock, go to the kitchen, go to the chair, where are you, battery.',
      } as never)) as { text?: string; transcription_info?: { text?: string; language?: string } };
      transcript = (stt.text ?? stt.transcription_info?.text ?? '').trim();
      language = stt.transcription_info?.language ?? language;
      if (!transcript) {
        // Don't spend LLM neurons on silence/noise.
        return Response.json({ transcript: '', language, reply: '', voice: 'amy', actions: [] });
      }
    } else if (url.pathname === '/text') {
      transcript = (body.text ?? '').trim();
      if (!transcript) return Response.json({ error: 'text required' }, { status: 400 });
    } else {
      return Response.json({ error: 'not found' }, { status: 404 });
    }

    const brain = await runBrain(env, transcript, body.context, body.history);
    return Response.json({ transcript, language, ...brain });
  },
} satisfies ExportedHandler<Env>;

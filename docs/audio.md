# Audio playback (speaker / voice / TTS)

How to play arbitrary audio — beeps, voice, TTS — on the robot's built-in speaker, reusing Dreame's
own audio stack so it never fights AVA for the ALSA codec.

## The Dreame audio architecture (reverse-engineered)

```
mda_cli "single,<path-no-ext>,<vol>"  →  mediad (pid, TCP 127.0.0.1:10100)  →  /ava/script/mediad_script.sh
                                                                                  ├─ amixer -D hw:audiocodec 'LINEOUT volume'  (MR813)
                                                                                  └─ ogg123 <path>.ogg  →  ALSA codec  →  speaker
```

- **`mediad`** (started `mediad -c /ava/script/mediad_script.sh`, monitored by `mediad_monitor.sh`)
  is the media daemon. It **listens on TCP `127.0.0.1:10100`** and owns the speaker.
- **`mda_cli "<message>"`** just sends a string to `:10100`. Messages AVA uses:
  - `play,<path>` — play
  - `single,<path>,<vol>` — play one file at a volume, restoring volume after
  - `ret` — (return/restore)
  - The `<path>` is given **WITHOUT extension** — `mediad_script.sh` appends `.ogg`.
- **`mediad_script.sh`** kills any running `ogg123` first (so audio is **serialized** — your sound and
  AVA's own prompts queue through one daemon, never overlapping on the codec), sets volume via
  `amixer -D hw:audiocodec` (max 31 on this MR813 board), then `ogg123 <path>.ogg`.

**Why go through mediad, not `aplay`/`ogg123` directly:** `aplay` opens ALSA `hw:0,0` directly and can
collide with AVA's `ogg123` (device busy / cut-off). mediad is the one owner — no contention, plus
free volume handling. (The repo's old `aplay`-based `audio_server.py` is superseded by this.)

**Wire protocol** (confirmed): connect to `127.0.0.1:10100`, send the message string — no terminator
needed. The chroot can reach it (shares the host network), so no `mda_cli` binary is required there.

## Play something — quickest path

```sh
# any OGG in /tmp:
oggenc -Q input.wav -o /tmp/hello.ogg          # WAV -> OGG (oggenc is on the robot)
mda_cli "single,/tmp/hello,70"                 # plays /tmp/hello.ogg @ vol 70  (NOTE: no .ogg)
```
Generate a tone from nothing (ffmpeg is in the chroot at `/opt/ffmpeg`):
```sh
chroot /data/chroot /opt/ffmpeg -f lavfi -i "sine=frequency=660:duration=1.5" -ar 16000 /tmp/t.wav
oggenc -Q /tmp/t.wav -o /tmp/t.ogg && mda_cli "single,/tmp/t,70"
```

## ROS integration — `audio_bridge.py` (multi-voice text→speech + files)

`scripts/robot/audio_bridge.py` (chroot ROS, started by `_root_postboot.sh`) subscribes
**`/robot/speak`** (`std_msgs/String`); the message is one of:
- **`text`** → spoken with the **default voice** (`default_voice` param, default `amy`)
- **`<voice>: text`** → spoken with that voice (per-message selection)
- a readable **`.ogg` path** → played as-is
- **`"stop"`** → `killall ogg123`

```sh
ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'Docking complete'}"        # default voice
ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'gosia: Dzień dobry'}"        # Polish
ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'es: Hola Dmitry'}"           # Spanish
ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'espeak: quick beep'}"        # instant
ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: /tmp/hello.ogg}"              # play file
```

**Voices** (the `VOICES` map in the node; keep in sync with `/data/chroot/opt/piper/voices/`):

| key | engine | language | aliases |
|-----|--------|----------|---------|
| `amy` | Piper (neural) | English (US ♀) | `en`, `english` |
| `thorsten` | Piper | German | `de`, `german` |
| `gosia` | Piper | Polish | `pl`, `polish` |
| `davefx` | Piper | Spanish | `es`, `spanish` |
| `espeak` | espeak-ng | any (robotic, **instant**) | — |

⚠️ **Piper does NOT translate** — send text *already in the target language* (the model only
pronounces). For English→other, translate first (an LLM/translator upstream), then send the result.
Params: `volume` (90), `default_voice` (`amy`), `espeak_speed` (155), `espeak_amp` (200).

## On-robot TTS engines

**Piper** (neural, natural) — binary + voices live at **`/data/chroot/opt/piper`** (eMMC, persists;
~281 MB = ~50 MB runtime + ~60 MB/voice). On the A7 it's ~RTF 1 for `low` voices, ~RTF 1.7 for
`medium` (so ~9–10 s for a sentence, incl. ~2.7 s model load). Install / add voices:
```sh
# runtime (once): download piper_linux_aarch64.tar.gz (github.com/rhasspy/piper releases) -> /data/chroot/opt/piper
# a voice: curl -L huggingface.co/rhasspy/piper-voices/resolve/main/<lang>/<...>/<voice>.onnx{,.json} -> .../voices/
```
Synthesis is piped (no temp WAV): `piper --model V --output_file - | /opt/ffmpeg -c:a libvorbis x.ogg`.

**espeak-ng** (robotic, instant) — apt-installed in the chroot, used for the `espeak` voice:
```sh
chroot /data/chroot apt-get -o APT::Sandbox::User=root install -y --no-install-recommends espeak-ng
```
espeak supports on-the-fly accents (`en-us`, `en-gb`, `en-gb-scotland`, foreign-accented English…).

## Natural voices on the companion (optional)

The A7 is slow for Piper; for snappy natural speech, synthesize on the Q6A and ship the OGG over —
`companion/tts_speak.sh`: `LLM/text → Piper → WAV → oggenc → robot:/tmp → mediad`.

## Caveats

- **OGG only** through mediad (it appends `.ogg` + runs `ogg123`). Encode WAV→OGG with `oggenc`.
- **Serialized**: a new play kills the previous `ogg123` — no overlap/mixing (matches AVA's behavior).
- Volume is 0–31 on MR813 but `mda_cli` takes the AVA-scale value; the bridge passes it through.

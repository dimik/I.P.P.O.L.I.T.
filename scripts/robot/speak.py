#!/usr/bin/env python3
"""speak.py — ROS-free TTS + play on the robot. Invoked over ssh by the companion audio_bridge ROS node.

Reads the utterance from stdin (a /robot/speak message: "text", "<voice>: text", "/path.ogg", or "stop"),
synthesizes via piper/espeak-ng -> ffmpeg libvorbis -> /tmp/_spoke.ogg, and triggers the robot's mediad
(127.0.0.1:10100) to play it. Runs in the robot chroot (has /opt/piper, /opt/ffmpeg, espeak-ng, and the
localhost mediad). NO ROS — the ROS /robot/speak subscription lives on the companion (audio_bridge.py),
which pipes the message text to `chroot /data/chroot python3 /opt/speak.py` over the USB link.

This is the synth/mediad half of the old on-robot audio_bridge.py, split out so ROS can leave the robot.
"""
import os, socket, subprocess, sys

MEDIAD = ('127.0.0.1', 10100)
FFMPEG = '/opt/ffmpeg'
PIPER = '/opt/piper/piper'
PIPER_DIR = '/opt/piper'
TTS_OGG = '/tmp/_spoke.ogg'
VOL = int(os.environ.get('SPEAK_VOL', '90'))
SPEED = int(os.environ.get('SPEAK_ESPEAK_SPEED', '155'))
AMP = int(os.environ.get('SPEAK_ESPEAK_AMP', '200'))
DEFAULT_VOICE = os.environ.get('SPEAK_DEFAULT_VOICE', 'amy')

VOICES = {
    'amy':      ('piper', '/opt/piper/voices/amy.onnx'),
    'thorsten': ('piper', '/opt/piper/voices/de.onnx'),
    'gosia':    ('piper', '/opt/piper/voices/pl.onnx'),
    'davefx':   ('piper', '/opt/piper/voices/es.onnx'),
    'espeak':   ('espeak', 'en-us+f3'),
}
ALIASES = {'en': 'amy', 'english': 'amy', 'de': 'thorsten', 'german': 'thorsten',
           'pl': 'gosia', 'polish': 'gosia', 'es': 'davefx', 'spanish': 'davefx'}


def send_mediad(msg):
    s = socket.socket(); s.settimeout(2.0)
    s.connect(MEDIAD); s.sendall(msg.encode()); s.close()


def synth(engine, model, text):
    if engine == 'piper':
        cmd, cwd = [PIPER, '--model', model, '--output_file', '-'], PIPER_DIR
    else:
        cmd, cwd = ['espeak-ng', '-v', model, '-s', str(SPEED), '-a', str(AMP), '--stdout'], None
    p1 = subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    p2 = subprocess.Popen([FFMPEG, '-hide_banner', '-loglevel', 'error', '-y', '-i', 'pipe:0',
                           '-c:a', 'libvorbis', TTS_OGG], stdin=p1.stdout,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    p1.stdout.close()
    p1.stdin.write(text.encode('utf-8')); p1.stdin.close()
    p2.wait(timeout=60); p1.wait(timeout=60)
    return p2.returncode == 0


def main():
    text = sys.stdin.read().strip()
    if not text:
        return 0
    if text == 'stop':
        subprocess.run(['killall', 'ogg123'], capture_output=True)
        return 0
    if text.endswith('.ogg') and os.path.exists(text):
        send_mediad(f'single,{text[:-4]},{VOL}'); return 0
    voice = DEFAULT_VOICE
    if ':' in text:
        head, _, rest = text.partition(':')
        key = head.strip().lower()
        if key in VOICES or key in ALIASES:
            voice = ALIASES.get(key, key); text = rest.strip()
    engine, model = VOICES.get(voice, VOICES[DEFAULT_VOICE])
    if text and synth(engine, model, text):
        send_mediad(f'single,{TTS_OGG[:-4]},{VOL}')
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())

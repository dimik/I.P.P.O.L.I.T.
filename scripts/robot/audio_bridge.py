#!/usr/bin/env python3
"""audio_bridge.py — robot speaker bridge: speak text (multi-voice TTS) and play files, via mediad.

Subscribes /robot/speak (std_msgs/String). The message is one of:
  - "text"                  -> spoken with the default voice
  - "<voice>: text"         -> spoken with that voice (per-message selection)
  - "/path/to/file.ogg"     -> played as-is
  - "stop"                  -> stops playback

Voices (see VOICES below): Piper neural voices (natural, ~real-time on the A7) for EN/DE/PL/ES, plus
espeak-ng (instant, robotic). Examples:
  ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'Docking complete'}"        # default
  ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'gosia: Dzień dobry'}"       # Polish
  ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'es: Hola Dmitry'}"          # Spanish
  ros2 topic pub --once /robot/speak std_msgs/msg/String "{data: 'espeak: quick beep test'}"  # instant

NOTE: Piper does NOT translate — send text already in the target language (the model just pronounces).
Pipeline (no temp WAV): piper/espeak (WAV on stdout) | ffmpeg -c:a libvorbis -> /tmp/_spoke.ogg ->
mediad (TCP 127.0.0.1:10100), serialized with AVA's prompts, no ALSA contention. See docs/audio.md.

Run: source /opt/ros/jazzy/setup.bash && python3 audio_bridge.py
"""
import os
import socket
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

MEDIAD = ('127.0.0.1', 10100)
FFMPEG = '/opt/ffmpeg'
PIPER = '/opt/piper/piper'
PIPER_DIR = '/opt/piper'
TTS_OGG = '/tmp/_spoke.ogg'

# voice key -> (engine, model)   [piper model path | espeak voice name]
VOICES = {
    'amy':      ('piper', '/opt/piper/voices/amy.onnx'),   # English  (US female)
    'thorsten': ('piper', '/opt/piper/voices/de.onnx'),    # German
    'gosia':    ('piper', '/opt/piper/voices/pl.onnx'),    # Polish
    'davefx':   ('piper', '/opt/piper/voices/es.onnx'),    # Spanish
    'espeak':   ('espeak', 'en-us+f3'),                    # instant, robotic
}
ALIASES = {'en': 'amy', 'english': 'amy', 'de': 'thorsten', 'german': 'thorsten',
           'pl': 'gosia', 'polish': 'gosia', 'es': 'davefx', 'spanish': 'davefx'}


class AudioBridge(Node):
    def __init__(self):
        super().__init__('audio_bridge')
        self.declare_parameter('volume', 90)
        self.declare_parameter('default_voice', 'amy')
        self.declare_parameter('espeak_speed', 155)
        self.declare_parameter('espeak_amp', 200)
        self.vol = int(self.get_parameter('volume').value)
        self.default_voice = self.get_parameter('default_voice').value
        self.speed = int(self.get_parameter('espeak_speed').value)
        self.amp = int(self.get_parameter('espeak_amp').value)
        self.create_subscription(String, '/robot/speak', self.on_speak, 10)
        self.get_logger().info(f'audio_bridge up; default_voice={self.default_voice}; '
                               f'voices={list(VOICES)}; use "<voice>: text", a .ogg path, or "stop"')

    def send_mediad(self, msg):
        try:
            s = socket.socket(); s.settimeout(2.0)
            s.connect(MEDIAD); s.sendall(msg.encode()); s.close()
            return True
        except Exception as e:
            self.get_logger().warn(f'mediad send failed: {e}')
            return False

    def synth(self, engine, model, text):
        """piper/espeak -> WAV on stdout -> ffmpeg libvorbis -> TTS_OGG (no temp WAV file)."""
        if engine == 'piper':
            cmd, cwd = [PIPER, '--model', model, '--output_file', '-'], PIPER_DIR
        else:
            cmd, cwd = ['espeak-ng', '-v', model, '-s', str(self.speed),
                        '-a', str(self.amp), '--stdout'], None
        try:
            p1 = subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL)
            p2 = subprocess.Popen([FFMPEG, '-hide_banner', '-loglevel', 'error', '-y',
                                   '-i', 'pipe:0', '-c:a', 'libvorbis', TTS_OGG],
                                  stdin=p1.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            p1.stdout.close()                       # let p2 get EOF when p1 exits
            p1.stdin.write(text.encode('utf-8')); p1.stdin.close()
            p2.wait(timeout=60); p1.wait(timeout=60)
            return p2.returncode == 0
        except Exception as e:
            self.get_logger().warn(f'TTS failed: {e}')
            try:
                p1.kill(); p2.kill()
            except Exception:
                pass
            return False

    def on_speak(self, m):
        text = m.data.strip()
        if not text:
            return
        if text == 'stop':
            subprocess.run(['killall', 'ogg123'], capture_output=True)
            return
        if text.endswith('.ogg') and os.path.exists(text):
            if self.send_mediad(f'single,{text[:-4]},{self.vol}'):
                self.get_logger().info(f'playing {text}')
            return
        # per-message voice selection: "<voice>: text"
        voice = self.default_voice
        if ':' in text:
            head, _, rest = text.partition(':')
            key = head.strip().lower()
            if key in VOICES or key in ALIASES:
                voice = ALIASES.get(key, key)
                text = rest.strip()
        engine, model = VOICES.get(voice, VOICES[self.default_voice])
        if text and self.synth(engine, model, text):
            self.send_mediad(f'single,{TTS_OGG[:-4]},{self.vol}')
            self.get_logger().info(f'[{voice}] {text[:60]!r}')


def main():
    rclpy.init()
    node = AudioBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

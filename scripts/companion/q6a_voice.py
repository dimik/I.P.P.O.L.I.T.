#!/usr/bin/env python3
"""q6a_voice.py (companion) — USB-mic voice control via the Cloudflare voice Worker.

Listens on a USB mic (energy VAD or /voice/trigger), ships each utterance as WAV to the
ippolit-voice Worker (Whisper STT + Llama intent -> {reply, voice, actions}), speaks the reply
through /robot/speak (audio_bridge -> robot piper/mediad) and executes the actions against the
robot's Valetudo REST. One HTTPS round trip per utterance. Architecture: docs/voice-cloud.md.

Config comes from /etc/default/ippolit-voice (VOICE_ENDPOINT, VOICE_TOKEN — never committed) and
/etc/default/ippolit-robot (ROBOT_ADDR). Run:
  source /opt/ros/jazzy/setup.bash && VOICE_ENDPOINT=... VOICE_TOKEN=... python3 q6a_voice.py

Actions executed (schema owned by cloud/voice-worker/src/index.ts):
  dock|stop|pause -> BasicControlCapability; locate -> LocateCapability;
  goto_point{x,y} (ROS map meters) -> GoToLocationCapability mm: (x*1000, -y*1000)
  (inverse of valetudo_bridge.py's /1000 + y-flip — verify signs on the live map once).
"""
import base64
import io
import json
import os
import struct
import subprocess
import threading
import time
import urllib.request
import wave

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Empty, String

RATE = 16000
FRAME_MS = 30
FRAME_BYTES = RATE * 2 * FRAME_MS // 1000   # S16LE mono


class Q6AVoice(Node):
    def __init__(self):
        super().__init__('q6a_voice')
        self.declare_parameter('endpoint', os.environ.get('VOICE_ENDPOINT', ''))
        self.declare_parameter('token', os.environ.get('VOICE_TOKEN', ''))
        self.declare_parameter('robot_host', os.environ.get('ROBOT_ADDR', '192.168.10.1'))
        self.declare_parameter('mic_device', 'default')      # arecord -D; e.g. plughw:CARD=Device,DEV=0
        self.declare_parameter('language_hint', '')          # ISO 639-1; '' = Whisper auto-detect
        self.declare_parameter('listen_mode', 'vad')         # 'vad' | 'trigger' (push-to-talk via /voice/trigger)
        self.declare_parameter('vad_rms', 700)               # speech threshold on int16 RMS — tune per mic
        self.declare_parameter('silence_ms', 800)            # end-of-utterance silence
        self.declare_parameter('max_utterance_s', 12)
        p = lambda n: self.get_parameter(n).value
        self.endpoint = str(p('endpoint')).rstrip('/')
        self.token = str(p('token'))
        self.robot = f"http://{p('robot_host')}"
        self.mic = str(p('mic_device'))
        self.lang = str(p('language_hint'))
        self.mode = str(p('listen_mode'))
        self.vad_rms = int(p('vad_rms'))
        self.sil_frames = int(p('silence_ms')) // FRAME_MS
        self.max_frames = int(p('max_utterance_s')) * 1000 // FRAME_MS
        if not self.endpoint or not self.token:
            raise SystemExit('VOICE_ENDPOINT / VOICE_TOKEN not set (see /etc/default/ippolit-voice)')

        self.pub_speak = self.create_publisher(String, '/robot/speak', 10)
        self.pub_tx = self.create_publisher(String, '/voice/transcript', 10)   # debug/other consumers
        self.create_subscription(String, '/object_map', self.on_objmap, 10)
        self.create_subscription(Empty, '/voice/trigger', self.on_trigger, 10)
        self.objects = []
        self.history = []                 # short conversational memory, [{'role','content'}]
        self.triggered = threading.Event()
        self.busy_until = 0.0             # mute the VAD while we speak (mic hears the robot speaker)
        threading.Thread(target=self.capture_loop, daemon=True).start()
        self.get_logger().info(f'q6a_voice up: mode={self.mode} mic={self.mic} -> {self.endpoint}')

    # ---- ROS inputs -------------------------------------------------------
    def on_objmap(self, m):
        try:
            self.objects = json.loads(m.data).get('objects', [])
        except (ValueError, AttributeError):
            pass

    def on_trigger(self, _):
        self.triggered.set()

    # ---- mic capture (arecord + energy VAD) -------------------------------
    def capture_loop(self):
        while True:
            try:
                self._capture_session()
            except Exception as e:            # arecord died / mic unplugged — retry
                self.get_logger().warn(f'capture: {e}; retrying in 3s')
                time.sleep(3)

    def _capture_session(self):
        proc = subprocess.Popen(
            ['arecord', '-q', '-D', self.mic, '-f', 'S16_LE', '-c', '1', '-r', str(RATE), '-t', 'raw'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        preroll = []                                    # ~300 ms so the first word isn't clipped
        utter, silent = None, 0
        while True:
            frame = proc.stdout.read(FRAME_BYTES)
            if len(frame) < FRAME_BYTES:
                raise RuntimeError('arecord EOF')
            now = time.monotonic()
            n = len(frame) // 2
            rms = (sum(s * s for s in struct.unpack(f'<{n}h', frame)) / n) ** 0.5
            speech = rms >= self.vad_rms and now >= self.busy_until
            if self.mode == 'trigger' and utter is None and not self.triggered.is_set():
                speech = False
            if utter is None:
                preroll.append(frame)
                if len(preroll) > 10:
                    preroll.pop(0)
                if speech:
                    utter, silent = list(preroll), 0
            else:
                utter.append(frame)
                silent = 0 if speech else silent + 1
                if silent >= self.sil_frames or len(utter) >= self.max_frames:
                    self.triggered.clear()
                    if len(utter) - silent > 8:         # ≥ ~0.25 s of actual speech
                        self.handle_utterance(b''.join(utter))
                    utter, preroll = None, []

    # ---- one utterance: -> Worker -> speak + act --------------------------
    def handle_utterance(self, pcm: bytes):
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(RATE); w.writeframes(pcm)
        req = {'audio_b64': base64.b64encode(buf.getvalue()).decode(),
               'context': self.build_context(), 'history': self.history[-6:]}
        if self.lang:
            req['language_hint'] = self.lang
        try:
            resp = self._post('/voice', req)
        except Exception as e:
            self.get_logger().warn(f'worker unreachable: {e}')
            self.say('espeak: cloud unreachable')
            return
        tx = resp.get('transcript', '')
        if not tx:
            return                                       # silence/noise — Worker skipped the LLM
        self.pub_tx.publish(String(data=tx))
        self.get_logger().info(f'heard: {tx!r} -> actions={resp.get("actions")}')
        self.history += [{'role': 'user', 'content': tx},
                         {'role': 'assistant', 'content': resp.get('reply', '')}]
        self.history = self.history[-8:]
        reply = resp.get('reply', '')
        if reply:
            self.say(f"{resp.get('voice', 'amy')}: {reply}")
        for a in resp.get('actions', []):
            self.execute(a)

    def build_context(self):
        ctx = {'objects': [{'label': o['cls'], 'x': o['x'], 'y': o['y']} for o in self.objects]}
        try:                                             # battery + status straight from Valetudo
            attrs = json.loads(urllib.request.urlopen(
                f'{self.robot}/api/v2/robot/state/attributes', timeout=3).read())
            for a in attrs:
                if a.get('__class') == 'BatteryStateAttribute':
                    ctx['battery'] = a.get('level')
                if a.get('__class') == 'StatusStateAttribute':
                    ctx['status'] = a.get('value')
        except Exception:
            pass                                         # context is best-effort
        return ctx

    def say(self, text):
        self.busy_until = time.monotonic() + 3 + len(text) * 0.07   # rough speech duration mute
        self.pub_speak.publish(String(data=text))

    def execute(self, a):
        t = a.get('type')
        try:
            if t in ('dock', 'stop', 'pause'):
                cmd = {'dock': 'home', 'stop': 'stop', 'pause': 'pause'}[t]
                self._valetudo('BasicControlCapability', {'command': cmd})
            elif t == 'locate':
                self._valetudo('LocateCapability', {'action': 'locate'})
            elif t == 'goto_point':
                self._valetudo('GoToLocationCapability', {'coordinates':
                               {'x': round(float(a['x']) * 1000), 'y': round(-float(a['y']) * 1000)}})
            else:
                self.get_logger().warn(f'unknown action {a}')
        except Exception as e:
            self.get_logger().warn(f'action {t} failed: {e}')
            self.say('espeak: command failed')

    def _valetudo(self, cap, body):
        r = urllib.request.Request(f'{self.robot}/api/v2/robot/capabilities/{cap}',
                                   data=json.dumps(body).encode(), method='PUT',
                                   headers={'Content-Type': 'application/json'})
        urllib.request.urlopen(r, timeout=10).read()

    def _post(self, path, obj):
        r = urllib.request.Request(self.endpoint + path, data=json.dumps(obj).encode(),
                                   headers={'Content-Type': 'application/json',
                                            'Authorization': f'Bearer {self.token}'})
        return json.loads(urllib.request.urlopen(r, timeout=30).read())


def main():
    rclpy.init()
    try:
        rclpy.spin(Q6AVoice())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

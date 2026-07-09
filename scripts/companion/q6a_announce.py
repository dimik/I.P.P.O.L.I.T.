#!/usr/bin/env python3
"""q6a_announce.py — speak recognized objects (companion).

Watches YOLO detections and has the robot say what it sees ("I see a chair") through its own Piper
voice. Subscribes /vision/detections (from q6a-vision) and publishes /robot/speak (std_msgs/String),
which audio-bridge pipes to the robot's speak.py over the USB/WiFi link.

Debounced so it narrates rather than spams:
  - MIN_CONF   : only speak confident detections
  - MIN_HITS   : a label must persist across a few frames first (kills single-frame false positives)
  - COOLDOWN   : don't repeat the same label within this many seconds
  - MIN_GAP    : global spacing between any two utterances (Piper + the serialized bridge are ~1-2 s/utt)

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 q6a_announce.py
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MIN_CONF = float(os.environ.get('Q6A_ANNOUNCE_MIN_CONF', '0.5'))
MIN_HITS = int(os.environ.get('Q6A_ANNOUNCE_MIN_HITS', '3'))       # frames a label must persist before speaking
COOLDOWN = float(os.environ.get('Q6A_ANNOUNCE_COOLDOWN', '25'))    # per-label repeat suppression (s)
MIN_GAP = float(os.environ.get('Q6A_ANNOUNCE_MIN_GAP', '3.0'))     # global spacing between utterances (s)
PHRASE = os.environ.get('Q6A_ANNOUNCE_PHRASE', 'I see a {label}')  # {label} substituted
# say "an" before vowel-initial labels for the default phrase; harmless if the phrase is customized
_VOWEL = ('a', 'e', 'i', 'o', 'u')


class Announcer(Node):
    def __init__(self):
        super().__init__('q6a_announce')
        self.hits = {}            # label -> persistence counter (grows when seen, decays when not)
        self.last_said = {}       # label -> monotonic time last announced
        self.last_utter = 0.0     # monotonic time of the last utterance (any label)
        self.pub = self.create_publisher(String, '/robot/speak', 10)
        self.create_subscription(String, '/vision/detections', self.on_dets, 10)
        self.get_logger().info(
            f'q6a_announce up: /vision/detections -> /robot/speak '
            f'(min_conf={MIN_CONF}, min_hits={MIN_HITS}, cooldown={COOLDOWN}s, phrase="{PHRASE}")')

    def on_dets(self, msg):
        try:
            dets = json.loads(msg.data).get('dets', [])
        except Exception:
            return
        seen = {d['label'] for d in dets if d.get('conf', 0) >= MIN_CONF and d.get('label')}
        # persistence: bump seen labels, decay the rest
        for lab in list(self.hits) + list(seen):
            if lab in seen:
                self.hits[lab] = min(self.hits.get(lab, 0) + 1, MIN_HITS + 2)
            else:
                self.hits[lab] = max(self.hits.get(lab, 0) - 1, 0)
        now = time.monotonic()
        if now - self.last_utter < MIN_GAP:
            return                                     # keep utterances spaced out
        # pick the most-persistent eligible label that is off cooldown
        ready = [lab for lab, h in self.hits.items()
                 if h >= MIN_HITS and (now - self.last_said.get(lab, -1e9)) >= COOLDOWN]
        if not ready:
            return
        lab = max(ready, key=lambda l: self.hits[l])
        self.speak(lab)
        self.last_said[lab] = now
        self.last_utter = now

    def speak(self, label):
        phrase = PHRASE
        if PHRASE == 'I see a {label}' and label[:1].lower() in _VOWEL:
            phrase = 'I see an {label}'
        text = phrase.format(label=label)
        self.pub.publish(String(data=text))
        self.get_logger().info(f'announce: "{text}"')


def main():
    rclpy.init()
    node = Announcer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

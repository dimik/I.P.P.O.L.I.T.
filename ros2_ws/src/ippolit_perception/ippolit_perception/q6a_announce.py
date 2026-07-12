#!/usr/bin/env python3
"""
q6a_announce.py — speak recognized objects (companion).

Watches YOLO detections and has the robot say what it sees ("I see a chair") through its own
Piper voice. Subscribes /vision/detections (vision_msgs/Detection2DArray, typed per A3 -- was a
JSON String) and publishes /robot/speak (std_msgs/String), which audio-bridge pipes to the
robot's speak.py over the USB/WiFi link.

Debounced so it narrates rather than spams:
  - min_conf   : only speak confident detections
  - min_hits   : a label must persist across a few frames first (kills single-frame false
                 positives)
  - cooldown   : don't repeat the same label within this many seconds
  - min_gap    : global spacing between any two utterances (Piper + the serialized bridge are
                 ~1-2 s/utt)

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 q6a_announce.py

Parameters are declared below (see ippolit_bringup/config/q6a_announce.yaml for the deployed
values); this replaces the earlier Q6A_ANNOUNCE_* environment-variable reads (A2).
"""
import time

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

_DEFAULT_ALLOW = [
    'person', 'chair', 'couch', 'bed', 'dining table', 'tv', 'refrigerator', 'oven',
    'microwave', 'sink', 'toilet', 'potted plant', 'book', 'clock', 'vase', 'bench', 'suitcase',
]
# say "an" before vowel-initial labels for the default phrase; harmless if the phrase is
# customized
_VOWEL = ('a', 'e', 'i', 'o', 'u')


class Announcer(Node):
    def __init__(self):
        super().__init__('q6a_announce')
        self.declare_parameter(
            'min_conf', 0.6,
            ParameterDescriptor(
                description='Only speak detections at or above this confidence (real objects '
                            'are typically >=0.62).',
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter(
            'min_hits', 5,
            ParameterDescriptor(
                description='Frames a label must persist before it is spoken.',
                integer_range=[IntegerRange(from_value=1, to_value=100)]))
        # Class allowlist: the model hallucinates implausible classes on textured floor ("cat",
        # "laptop") that peak even >0.6, so a confidence gate can't fully kill them. This robot
        # maps a room, so only speak/keep plausible indoor furniture/appliances/people;
        # everything else is ignored regardless of confidence.
        self.declare_parameter(
            'allow_labels', _DEFAULT_ALLOW,
            ParameterDescriptor(description='Lower-cased YOLO labels eligible to be announced.'))
        self.declare_parameter(
            'cooldown', 25.0,
            ParameterDescriptor(
                description='Seconds before the same label can be announced again.',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=600.0)]))
        self.declare_parameter(
            'min_gap', 3.0,
            ParameterDescriptor(
                description='Minimum seconds between any two utterances (any label).',
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=60.0)]))
        self.declare_parameter(
            'phrase', 'I see a {label}',
            ParameterDescriptor(description='Format string; {label} is substituted.'))

        self.min_conf = self.get_parameter('min_conf').value
        self.min_hits = self.get_parameter('min_hits').value
        self.allow = {lab.strip().lower() for lab in self.get_parameter('allow_labels').value
                      if lab.strip()}
        self.cooldown = self.get_parameter('cooldown').value
        self.min_gap = self.get_parameter('min_gap').value
        self.phrase = self.get_parameter('phrase').value

        # label -> persistence counter (grows when seen, decays when not)
        self.hits = {}
        self.last_said = {}       # label -> monotonic time last announced
        self.last_utter = 0.0     # monotonic time of the last utterance (any label)
        self.pub = self.create_publisher(String, '/robot/speak', 10)
        self.create_subscription(Detection2DArray, '/vision/detections', self.on_dets, 10)
        self.get_logger().info(
            f'q6a_announce up: /vision/detections -> /robot/speak '
            f'(min_conf={self.min_conf}, min_hits={self.min_hits}, cooldown={self.cooldown}s, '
            f'phrase="{self.phrase}")')

    def on_dets(self, msg):
        seen = set()
        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            if conf >= self.min_conf and label and label.lower() in self.allow:
                seen.add(label)
        # persistence: bump seen labels, decay the rest
        for lab in list(self.hits) + list(seen):
            if lab in seen:
                self.hits[lab] = min(self.hits.get(lab, 0) + 1, self.min_hits + 2)
            else:
                self.hits[lab] = max(self.hits.get(lab, 0) - 1, 0)
        now = time.monotonic()
        if now - self.last_utter < self.min_gap:
            return                                     # keep utterances spaced out
        # pick the most-persistent eligible label that is off cooldown
        ready = [lab for lab, h in self.hits.items()
                 if h >= self.min_hits and (now - self.last_said.get(lab, -1e9)) >= self.cooldown]
        if not ready:
            return
        lab = max(ready, key=lambda candidate: self.hits[candidate])
        self.speak(lab)
        self.last_said[lab] = now
        self.last_utter = now

    def speak(self, label):
        phrase = self.phrase
        if self.phrase == 'I see a {label}' and label[:1].lower() in _VOWEL:
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

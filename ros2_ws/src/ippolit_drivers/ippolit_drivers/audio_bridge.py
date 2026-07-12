#!/usr/bin/env python3
"""
audio_bridge.py (companion) — ROS /robot/speak -> robot TTS over the USB link.

Subscribes /robot/speak (std_msgs/String) on the Q6A and pipes each utterance to the robot's
ROS-free speak.py (piper/espeak + ffmpeg -> localhost mediad) over ssh. Relocated off the robot
chroot ROS (2026-07-08): the ROS subscription lives on the companion; synth stays on the robot
(reuses its /opt/piper + /opt/ffmpeg + mediad, which binds 127.0.0.1 only, so it can't be hit
from the Q6A directly).

Run: source /opt/ros/jazzy/setup.bash && python3 audio_bridge.py --ros-args -p robot_ssh:=robot-usb

Publishes /diagnostics (A5): there's no persistent connection to watch (each utterance is its own
ssh round-trip), so this reports the outcome of the LAST utterance -- OK if none has been spoken
yet (idle, nothing to report) or the last one succeeded, ERROR if it failed (the ssh link to the
robot, not just this one utterance, is the likely cause).
"""
import subprocess
import threading

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import FunctionDiagnosticTask, Updater
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class AudioBridge(Node):
    def __init__(self):
        super().__init__('audio_bridge')
        self.declare_parameter('robot_ssh', 'robot-usb')      # ssh alias for the robot over USB
        self.declare_parameter('volume', 90)
        self.declare_parameter('default_voice', 'amy')
        self.robot = self.get_parameter('robot_ssh').value
        self.vol = int(self.get_parameter('volume').value)
        self.dv = self.get_parameter('default_voice').value
        self._lock = threading.Lock()             # serialize utterances (one speak.py at a time)
        self.last_speak_ok = None                  # None = nothing spoken yet this run
        self.last_speak_err = ''
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('audio_bridge')
        self.diag_updater.add(FunctionDiagnosticTask('audio_bridge speak link', self._diag))
        self.create_subscription(String, '/robot/speak', self.on_speak, 10)
        self.get_logger().info(
            f'audio_bridge (companion) up; /robot/speak -> {self.robot}:/opt/speak.py')

    def _diag(self, stat):
        if self.last_speak_ok is None:
            stat.summary(DiagnosticStatus.OK, 'idle -- no utterances spoken yet this run')
        elif self.last_speak_ok:
            stat.summary(DiagnosticStatus.OK, 'last utterance succeeded')
        else:
            stat.summary(DiagnosticStatus.ERROR, f'last utterance failed: {self.last_speak_err}')
        return stat

    def on_speak(self, m):
        text = m.data.strip()
        if not text:
            return
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _speak(self, text):
        with self._lock:
            try:
                r = subprocess.run(
                    ['ssh', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=no', self.robot,
                     f'SPEAK_VOL={self.vol} SPEAK_DEFAULT_VOICE={self.dv} '
                     f'chroot /data/chroot python3 /opt/speak.py'],
                    input=text.encode('utf-8'), timeout=75,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                if r.returncode != 0:
                    err = r.stderr.decode(errors='replace')[:140]
                    self.last_speak_ok, self.last_speak_err = False, f'rc={r.returncode}: {err}'
                    self.get_logger().warn(f'speak rc={r.returncode}: {err}')
                else:
                    self.last_speak_ok = True
                    self.get_logger().info(f'spoke: {text[:60]!r}')
            except Exception as e:
                self.last_speak_ok, self.last_speak_err = False, str(e)
                self.get_logger().warn(f'speak error: {e}')


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

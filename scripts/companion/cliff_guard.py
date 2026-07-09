#!/usr/bin/env python3
"""cliff_guard.py — SAFETY backstop: hard-stop the robot when a downward cliff sensor trips.

The robot lives on the 2nd floor near a ladder/stairs. A horizontal LiDAR cannot see a down-staircase
(a drop reads as "open"), so fall protection comes from the robot's own downward IR cliff sensors. mcu_node
decodes them from the MCU Triggers frame and publishes /cliff (Bool). This node reacts: on a rising edge to
True it immediately DISABLES HighResolutionManualControl over Valetudo REST (stopping the drive) and speaks a
warning. It is the software backstop to AVA's built-in cliff avoidance.

Latches on trip (won't silently resume); re-arms when /cliff clears (robot back on solid floor). The driver
must re-enable manual control after a trip. Robot REST address from $ROBOT_ADDR (matches the other services).

Run: source /opt/ros/jazzy/setup.bash && ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 cliff_guard.py
"""
import json
import os
import threading
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

ROBOT_ADDR = os.environ.get('ROBOT_ADDR', '192.168.10.1')
CAP = f'http://{ROBOT_ADDR}/api/v2/robot/capabilities/HighResolutionManualControlCapability'


class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')
        self.pub_speak = self.create_publisher(String, '/robot/speak', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/cliff', self.on_cliff, latched)
        self.tripped = False
        self.get_logger().info(f'cliff_guard up: /cliff -> DISABLE manual control @ {ROBOT_ADDR} (fall backstop)')

    def on_cliff(self, m):
        if m.data and not self.tripped:
            self.tripped = True
            self.get_logger().error('CLIFF DETECTED — hard-stopping (disable manual control)')
            threading.Thread(target=self.estop, daemon=True).start()
            self.pub_speak.publish(String(data='Cliff detected. Stopping.'))
        elif not m.data and self.tripped:
            self.tripped = False
            self.get_logger().warn('cliff cleared — guard re-armed (re-enable manual control to drive)')

    def estop(self):
        body = json.dumps({'action': 'disable'}).encode()
        for _ in range(3):                 # repeat — a single PUT can drop over WiFi
            try:
                req = urllib.request.Request(CAP, data=body, method='PUT',
                                             headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=1.0).read()
            except Exception as e:
                self.get_logger().warn(f'estop PUT failed: {e}')


def main():
    rclpy.init()
    node = CliffGuard()
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

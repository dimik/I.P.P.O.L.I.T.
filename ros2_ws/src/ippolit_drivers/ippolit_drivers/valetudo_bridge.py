#!/usr/bin/env python3
"""
valetudo_bridge.py — Valetudo REST/SSE -> ROS 2 status + battery bridge.

Runs on the Q6A companion (ROS 2 Jazzy). Bridges the robot's high-level state from Valetudo into
ROS WITHOUT an MQTT broker and WITHOUT polling: one REST fetch seeds the initial attributes, then a
single Server-Sent-Events stream pushes updates.

Publishes:
  /robot/status  std_msgs/String        StatusStateAttribute  "<value>/<flag>"
  /battery       sensor_msgs/BatteryState  level + AVA charge_state

SCOPE (A4, 2026-07-13): this node used to ALSO publish /map (OccupancyGrid), /odom (Odometry), and
a map->base_link TF derived from Valetudo's own SLAM. Those are all GONE now — the companion's own
stack owns them: slam_toolbox owns /map + map->odom, q6a_laser_odom owns odom->base_link, and
robot_state_publisher owns the static base_link->{laser,camera_link} frames (from the URDF).
Keeping the old publishers here was an active bug: /map collided with slam_toolbox's (and could
feed slam's own map_saver the wrong grid), and the map->base_link TF gave base_link TWO parents
(map from here, odom from laser_odom), corrupting the TF tree. Valetudo's robot_position is also
FROZEN during manual_control anyway (the whole reason q6a_laser_odom exists), so its pose was
useless for a manual mapping drive. This bridge is now purely the status/battery path the
architecture doc assigns it. The Valetudo map is still recoverable from git history if ever wanted
as a reference topic.

Run:  source /opt/ros/jazzy/setup.bash && python3 valetudo_bridge.py [--host http://127.0.0.1]

Publishes /diagnostics (A5): OK while the attributes SSE stream is connected -- REST reachability
is what actually matters here, not event frequency (a docked/idle robot can go a long time between
real attribute changes without anything being wrong).
"""
import argparse
import json
import sys
import threading
import time
import urllib.request

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import FunctionDiagnosticTask, Updater
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class ValetudoBridge(Node):
    def __init__(self, host):
        super().__init__('valetudo_bridge')
        self.host = host.rstrip('/')
        self.pub_status = self.create_publisher(String, '/robot/status', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/battery', 10)
        # cached battery %, updated by the attrs SSE/seed, republished by the heartbeat
        self.batt_level = None
        self.docked = None          # cached from the Valetudo status -> the /battery charge signal
        self.attrs_sse_connected = False
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('valetudo_bridge')
        self.diag_updater.add(FunctionDiagnosticTask('Valetudo REST/SSE reachability', self._diag))
        # heartbeat: republish cached /battery at 2 Hz so it stays fresh while idle (the SSE only
        # fires on change, so docked+full would otherwise never re-publish; no HTTP here).
        self.create_timer(0.5, self.publish_battery)

        # seed the initial attributes once via REST (the SSE sends no snapshot on connect)
        try:
            self.handle_attrs(self.fetch('/api/v2/robot/state/attributes'))
        except Exception as e:
            self.get_logger().warn(f'initial attrs seed failed: {e}')

        # then push-driven: one SSE stream for state attributes (no polling)
        threading.Thread(
            target=self.sse_loop,
            args=('/api/v2/robot/state/attributes/sse', 'StateAttributesUpdated',
                  self.handle_attrs, 'attrs_sse_connected'),
            daemon=True).start()
        self.get_logger().info(f'valetudo_bridge up (status + battery, REST seed + SSE) on '
                               f'{self.host}')

    # ---- HTTP ----
    def fetch(self, path):
        with urllib.request.urlopen(self.host + path, timeout=4) as r:
            return json.load(r)

    def _diag(self, stat):
        if self.attrs_sse_connected:
            stat.summary(DiagnosticStatus.OK, 'attributes SSE stream connected')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Valetudo attributes SSE down -- unreachable?')
        stat.add('attrs_sse_connected', str(self.attrs_sse_connected))
        return stat

    def sse_loop(self, path, want_event, handler, connected_attr):
        """Long-lived SSE: accumulate event/data lines, dispatch on blank, reconnect on drop."""
        while rclpy.ok():
            try:
                with urllib.request.urlopen(self.host + path, timeout=None) as resp:
                    setattr(self, connected_attr, True)
                    event, data = None, []
                    for raw in resp:
                        if not rclpy.ok():
                            return
                        line = raw.decode('utf-8', 'replace').rstrip('\r\n')
                        if line.startswith('event:'):
                            event = line[6:].strip()
                        elif line.startswith('data:'):
                            data.append(line[5:].strip())
                        elif line == '':                     # end of one SSE event
                            if event == want_event and data:
                                try:
                                    handler(json.loads('\n'.join(data)))
                                except Exception as e:
                                    self.get_logger().warn(f'{path} parse: {e}')
                            event, data = None, []
            except Exception as e:
                setattr(self, connected_attr, False)
                if rclpy.ok():
                    self.get_logger().warn(f'{path} dropped ({e}); reconnecting')
                    time.sleep(2.0)

    # ---- publishers ----
    def handle_attrs(self, attrs):
        st = next((a for a in attrs if a.get('__class') == 'StatusStateAttribute'), None)
        if st:
            self.pub_status.publish(String(data=f"{st.get('value')}/{st.get('flag')}"))
            # 'docked' is our charge signal (see publish_battery). Cache it here — this is the
            # only place the status flows in; the heartbeat republishes /battery off the cache.
            self.docked = st.get('value') == 'docked'
        # cache the battery level from the attributes SSE; /battery is republished by the
        # heartbeat (the SSE only fires on change, so docked+full would otherwise never publish).
        bat = next((a for a in attrs if a.get('__class') == 'BatteryStateAttribute'), None)
        if bat is not None and bat.get('level') is not None:
            self.batt_level = int(bat['level'])

    def publish_battery(self):
        """
        Publish /battery from the cached level + docked status as the charge signal.

        Two D10S Pro quirks force this: Valetudo's own charging FLAG is stuck 'none' (a mapping
        gap), and the old workaround (AVA writes /tmp/charge_state, we read it) BROKE when this
        node moved from the robot chroot to the Q6A companion -- /tmp/charge_state lives on the
        robot, not here, so it always read absent -> UNKNOWN. Instead we use the Valetudo status
        we already receive: 'docked' => charging (or FULL at >=100%), anything else => discharging.
        A proxy (docked-but-idle-at-full is FULL, which we handle; a docked-but-faulted-charger
        edge case would misreport, but that's rare and far better than always-UNKNOWN).
        """
        if self.batt_level is None:
            return
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.percentage = self.batt_level / 100.0
        b.present = True
        if self.docked is None:
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif self.docked:
            b.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_FULL if self.batt_level >= 100
                else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='http://127.0.0.1')
    a, ros_argv = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_argv)
    node = ValetudoBridge(a.host)
    try:
        # no timers beyond the battery heartbeat; the SSE thread drives status updates
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

#!/usr/bin/env python3
"""
q6a_map_persist.py — slam_toolbox map persistence (companion), as a dedicated ROS node.

Replaces the earlier bash+systemd-hook approach (slam_save_map.sh called from ExecStop,
deserialize called from slam_lifecycle_up.sh's ExecStartPost) with a single node that owns the
whole persistence lifecycle via slam_toolbox's own standard service API:
  /slam_toolbox/serialize_map    (SerializePoseGraph) -- full pose-graph state, used to RESUME
  /slam_toolbox/deserialize_map  (DeserializePoseGraph) -- loads that state back in
  /slam_toolbox/save_map         (SaveMap) -- plain occupancy-grid image (yaml+pgm), viewing only

These are the same services RViz's SlamToolboxPlugin panel calls when you click its Save/Serialize
buttons -- nothing about the ROS API here is custom, only that this node calls them automatically
instead of a human doing it by hand.

Lifecycle:
  - On startup: retries `deserialize_map` (if a saved pose graph exists AND looks non-trivial, see
    MIN_RESUME_BYTES below) until slam_toolbox is active and accepts it, or DESERIALIZE_TIMEOUT_S
    elapses (then gives up and lets slam_toolbox map from empty -- non-fatal). Uses
    match_type=START_AT_FIRST_NODE, i.e. the loaded graph is anchored at its own saved origin:
    this assumes the robot is placed back at roughly the position it was in when the map was last
    saved. There's no relocalization-against-live-scan step yet.

  ⚠️ CRASH FOUND LIVE (2026-07-12): calling deserialize_map against a saved pose graph with ZERO
  real scan nodes (e.g. serialized while the robot never moved -- exactly what every rapid
  restart-cycle test that day produced) reliably SEGFAULTS slam_toolbox's C++ process. Confirmed
  by isolation: stopping this node immediately stabilized slam_toolbox; a repeated
  Configuring->Activating->segfault crash loop only happened while this node kept retrying
  deserialize_map against that empty graph. Mitigated with MIN_RESUME_BYTES (skip deserialize
  entirely if the saved .posegraph file is suspiciously small to contain real scan data) but this
  is a size HEURISTIC, not a real fix for the underlying crash -- if you ever see slam_toolbox
  crash-looping right after this node logs "will resume", suspect this bug first, `systemctl stop
  q6a-map-persist` immediately, and delete the saved .posegraph/.data pair.
  - Every SAVE_PERIOD_S: calls serialize_map + save_map to persist the current state. This is the
    ONLY save trigger -- a "final save on clean shutdown" was tried and reverted (see the note in
    main()): rclpy's default SIGINT handler tears the context down before a `finally:` block can
    make a service call, and disabling that handler to work around it broke prompt shutdown
    entirely instead. So a clean stop/restart loses at most the last SAVE_PERIOD_S of updates,
    same as a hard power-cut would -- a simple, honest bound rather than a fragile attempt at a
    perfect save.

Still needs `slam_lifecycle_up.sh` to exist and run from ExecStartPost -- this node does NOT do
the configure->activate lifecycle dance itself (that's a startup/process-wiring concern tied to
Jazzy's slam_toolbox being a lifecycle node, arguably still fine as a shell hook); it only owns
save/load.
"""
import os
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, SaveMap, SerializePoseGraph
from std_msgs.msg import String

MAP_DIR = os.environ.get('Q6A_MAP_DIR', '/home/radxa/ros/maps')
BASE = os.path.join(MAP_DIR, 'apartment')
SAVE_PERIOD_S = float(os.environ.get('Q6A_MAP_SAVE_PERIOD_S', '30.0'))
DESERIALIZE_TIMEOUT_S = float(os.environ.get('Q6A_MAP_DESERIALIZE_TIMEOUT_S', '60.0'))
DESERIALIZE_RETRY_S = 3.0
MATCH_START_AT_FIRST_NODE = 1
# A pose graph serialized with zero real scan nodes (robot never moved) was observed at ~7.8KB
# and reliably SEGFAULTED slam_toolbox on deserialize_map -- see the docstring's CRASH FOUND note.
# A graph with actual scan data (even a short drive) should be far larger (each scan alone is
# hundreds of range floats). This is a crude size heuristic, not a real fix, but cheap insurance
# against hitting the same crash with a similarly-degenerate saved file.
MIN_RESUME_BYTES = int(os.environ.get('Q6A_MAP_MIN_RESUME_BYTES', '51200'))   # 50KB


class MapPersist(Node):
    def __init__(self):
        super().__init__('q6a_map_persist')
        os.makedirs(MAP_DIR, exist_ok=True)
        self.cli_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.cli_deserialize = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.cli_save_map = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.resumed = False
        self.deserialize_deadline = time.monotonic() + DESERIALIZE_TIMEOUT_S
        pg = BASE + '.posegraph'
        size = os.path.getsize(pg) if os.path.exists(pg) else 0
        if size == 0:
            self.resumed = True   # nothing to resume -- treat as "done" so we don't keep retrying
            self.get_logger().info(f'no saved map at {pg} -- starting from empty')
        elif size < MIN_RESUME_BYTES:
            # see MIN_RESUME_BYTES / the docstring's CRASH FOUND note -- refusing to deserialize
            # a graph this small is what stopped a real crash loop
            self.resumed = True
            self.get_logger().warn(
                f'saved map at {pg} is only {size}B (< {MIN_RESUME_BYTES}B) -- refusing to '
                f'resume (looks like it has no real scan data; a prior graph this size crashed '
                f'slam_toolbox on deserialize_map). Starting from empty. Delete {pg} and '
                f'{BASE}.data if this is stale.')
        else:
            self.get_logger().info(
                f'saved map found at {pg} ({size}B) -- will resume once slam_toolbox is active '
                f'(retrying up to {DESERIALIZE_TIMEOUT_S:.0f}s)')
            self.create_timer(DESERIALIZE_RETRY_S, self.try_resume)
        self.create_timer(SAVE_PERIOD_S, self.save)
        self.get_logger().info(f'q6a_map_persist up (base={BASE}, save every {SAVE_PERIOD_S}s)')

    def try_resume(self):
        if self.resumed:
            return
        if time.monotonic() > self.deserialize_deadline:
            self.resumed = True
            self.get_logger().warn(
                f'gave up resuming after {DESERIALIZE_TIMEOUT_S:.0f}s -- '
                f'slam_toolbox will map from empty this session')
            return
        if not self.cli_deserialize.service_is_ready():
            return   # slam_toolbox not active yet (lifecycle not configured) -- retry later
        req = DeserializePoseGraph.Request()
        req.filename = BASE
        req.match_type = MATCH_START_AT_FIRST_NODE
        fut = self.cli_deserialize.call_async(req)
        fut.add_done_callback(self.on_resume_done)

    def on_resume_done(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'deserialize_map call failed: {e} -- will retry')
            return
        if res.result == 0:
            self.resumed = True
            self.get_logger().info(f'resumed saved map from {BASE}.posegraph')
        else:
            self.get_logger().warn(f'deserialize_map returned result={res.result} -- will retry')

    def save(self):
        if self.cli_serialize.service_is_ready():
            req = SerializePoseGraph.Request()
            req.filename = BASE
            self.cli_serialize.call_async(req).add_done_callback(self._log_serialize)
        if self.cli_save_map.service_is_ready():
            req = SaveMap.Request()
            req.name = String(data=BASE)
            self.cli_save_map.call_async(req).add_done_callback(self._log_save_map)

    def _log_serialize(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'serialize_map failed: {e}')
            return
        if res.result == 0:
            self.get_logger().info(f'serialized pose graph -> {BASE}.posegraph/.data')
        else:
            self.get_logger().warn(f'serialize_map returned result={res.result}')

    def _log_save_map(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'save_map failed: {e}')
            return
        if res.result != 0:
            self.get_logger().warn(f'save_map returned result={res.result} (non-fatal)')


def main():
    # NOTE on a dead end (2026-07-12): tried disabling rclpy's default SIGINT handler
    # (signal_handler_options=SignalHandlerOptions.NO) so a "final save on shutdown" in
    # `finally:` below would have a still-valid context to call services with (the default
    # handler otherwise tears the context down the instant SIGINT arrives, before `finally:`
    # runs -- confirmed live: every attempt failed with "rcl node's context is invalid"). That
    # "fix" was WORSE: without rclpy's handler, plain Python SIGINT doesn't reliably interrupt
    # the blocking rcl wait inside spin() at all, so the process just hung until systemd's
    # TimeoutStopSec elapsed and SIGKILL'd it -- confirmed live (a restart that should take ~1s
    # took ~45s and produced no log output at all, graceful or otherwise). Reverted. Conclusion:
    # a synchronous final-save-on-SIGINT isn't reliably achievable here without much deeper
    # surgery on rclpy's shutdown sequence. Relying on the periodic SAVE_PERIOD_S timer only -- a
    # clean stop/restart loses at most that long of updates, which is an acceptable, simple,
    # honest bound.
    rclpy.init()
    node = MapPersist()
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

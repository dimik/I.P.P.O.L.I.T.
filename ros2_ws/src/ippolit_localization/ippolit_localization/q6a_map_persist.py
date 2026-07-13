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
    the min_resume_bytes param below) until slam_toolbox is active and accepts it, or the
    deserialize_timeout_s param elapses (then gives up and lets slam_toolbox map from empty --
    non-fatal). Uses match_type=START_AT_FIRST_NODE, i.e. the loaded graph is anchored at its own
    saved origin: this assumes the robot is placed back at roughly the position it was in when
    the map was last saved. There's no relocalization-against-live-scan step yet.

  ⚠️ CRASH FOUND LIVE (2026-07-12): calling deserialize_map against a saved pose graph with ZERO
  real scan nodes (e.g. serialized while the robot never moved -- exactly what every rapid
  restart-cycle test that day produced) reliably SEGFAULTS slam_toolbox's C++ process. Confirmed
  by isolation: stopping this node immediately stabilized slam_toolbox; a repeated
  Configuring->Activating->segfault crash loop only happened while this node kept retrying
  deserialize_map against that empty graph. Mitigated with the min_resume_bytes param (skip
  deserialize entirely if the saved .posegraph file is suspiciously small to contain real scan
  data) but this is a size HEURISTIC, not a real fix for the underlying crash -- if you ever see
  slam_toolbox crash-looping right after this node logs "will resume", suspect this bug first,
  `systemctl stop q6a-map-persist` immediately, and delete the saved .posegraph/.data pair.
  - Every save_period_s: calls serialize_map + save_map to persist the current state. This is the
    ONLY save trigger -- a "final save on clean shutdown" was tried and reverted (see the note in
    main()): rclpy's default SIGINT handler tears the context down before a `finally:` block can
    make a service call, and disabling that handler to work around it broke prompt shutdown
    entirely instead. So a clean stop/restart loses at most the last save_period_s of updates,
    same as a hard power-cut would -- a simple, honest bound rather than a fragile attempt at a
    perfect save.

Still needs `slam_lifecycle_up.sh` to exist and run from ExecStartPost -- this node does NOT do
the configure->activate lifecycle dance itself (that's a startup/process-wiring concern tied to
Jazzy's slam_toolbox being a lifecycle node, arguably still fine as a shell hook); it only owns
save/load.

Parameters are declared below (see ippolit_bringup/config/q6a_map_persist.yaml for the deployed
values); this replaces the earlier Q6A_MAP_* environment-variable reads (A2).
"""
import os
import time

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, SaveMap, SerializePoseGraph
from std_msgs.msg import String

DESERIALIZE_RETRY_S = 3.0
MATCH_START_AT_FIRST_NODE = 1


def resume_decision(size, min_resume_bytes):
    """
    Classify a saved .posegraph file's on-disk byte size for the startup resume guard.

    Pulled out of MapPersist.__init__ as a plain function so the min_resume_bytes SAFETY guard
    (see the module docstring's CRASH FOUND note) is testable without a live rclpy Node.
    Returns 'empty' (no file / zero bytes -- nothing to resume), 'refuse' (file exists but is
    smaller than min_resume_bytes -- the crash-risk case), or 'resume' (safe to attempt).
    """
    if size == 0:
        return 'empty'
    if size < min_resume_bytes:
        return 'refuse'
    return 'resume'


class MapPersist(Node):
    def __init__(self):
        super().__init__('q6a_map_persist')
        self.declare_parameter(
            'map_dir', '/home/radxa/ros/maps',
            ParameterDescriptor(description='Directory holding the persisted map files.'))
        self.declare_parameter(
            'map_name', 'apartment',
            ParameterDescriptor(description='Filename stem for the saved posegraph/pgm/yaml.'))
        self.declare_parameter(
            'save_period_s', 30.0,
            ParameterDescriptor(
                description='Seconds between periodic serialize_map + save_map calls.',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=600.0)]))
        self.declare_parameter(
            'deserialize_timeout_s', 60.0,
            ParameterDescriptor(
                description='Give up retrying deserialize_map after this many seconds and let '
                            'slam_toolbox map from empty.',
                floating_point_range=[FloatingPointRange(from_value=1.0, to_value=600.0)]))
        self.declare_parameter(
            'min_resume_bytes', 51200,
            ParameterDescriptor(
                description=(
                    'SAFETY: a saved .posegraph file smaller than this is refused for resume -- '
                    'a graph with zero real scan nodes reliably SEGFAULTS slam_toolbox on '
                    'deserialize_map (see the module docstring CRASH FOUND note). Do not set '
                    'this to 0; the range floor keeps it a meaningful guard.'),
                integer_range=[IntegerRange(from_value=1024, to_value=10_000_000)]))

        map_dir = self.get_parameter('map_dir').value
        map_name = self.get_parameter('map_name').value
        self.save_period_s = self.get_parameter('save_period_s').value
        self.deserialize_timeout_s = self.get_parameter('deserialize_timeout_s').value
        self.min_resume_bytes = self.get_parameter('min_resume_bytes').value
        self.base = os.path.join(map_dir, map_name)

        os.makedirs(map_dir, exist_ok=True)
        self.cli_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.cli_deserialize = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self.cli_save_map = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.resumed = False
        self.deserialize_deadline = time.monotonic() + self.deserialize_timeout_s
        pg = self.base + '.posegraph'
        size = os.path.getsize(pg) if os.path.exists(pg) else 0
        decision = resume_decision(size, self.min_resume_bytes)
        if decision == 'empty':
            self.resumed = True   # nothing to resume -- treat as "done" so we don't keep retrying
            self.get_logger().info(f'no saved map at {pg} -- starting from empty')
        elif decision == 'refuse':
            # see min_resume_bytes / the docstring's CRASH FOUND note -- refusing to deserialize
            # a graph this small is what stopped a real crash loop
            self.resumed = True
            self.get_logger().warn(
                f'saved map at {pg} is only {size}B (< {self.min_resume_bytes}B) -- refusing to '
                f'resume (looks like it has no real scan data; a prior graph this size crashed '
                f'slam_toolbox on deserialize_map). Starting from empty. Delete {pg} and '
                f'{self.base}.data if this is stale.')
        else:
            self.get_logger().info(
                f'saved map found at {pg} ({size}B) -- will resume once slam_toolbox is active '
                f'(retrying up to {self.deserialize_timeout_s:.0f}s)')
            self.create_timer(DESERIALIZE_RETRY_S, self.try_resume)
        self.create_timer(self.save_period_s, self.save)
        self.get_logger().info(
            f'q6a_map_persist up (base={self.base}, save every {self.save_period_s}s)')

    def try_resume(self):
        if self.resumed:
            return
        if time.monotonic() > self.deserialize_deadline:
            self.resumed = True
            self.get_logger().warn(
                f'gave up resuming after {self.deserialize_timeout_s:.0f}s -- '
                f'slam_toolbox will map from empty this session')
            return
        if not self.cli_deserialize.service_is_ready():
            return   # slam_toolbox not active yet (lifecycle not configured) -- retry later
        req = DeserializePoseGraph.Request()
        req.filename = self.base
        req.match_type = MATCH_START_AT_FIRST_NODE
        fut = self.cli_deserialize.call_async(req)
        fut.add_done_callback(self.on_resume_done)

    def on_resume_done(self, fut):
        # G25: unlike SerializePoseGraph/SaveMap, DeserializePoseGraph.Response has NO fields at
        # all (confirmed via `ros2 interface show`) -- there is no `.result` code to check. The
        # original code assumed one by analogy with the other two services and crashed the whole
        # node with an AttributeError the first time a real deserialize actually completed. A
        # future rclpy/service-call exception is still the only failure signal available here.
        try:
            fut.result()
        except Exception as e:
            self.get_logger().warn(f'deserialize_map call failed: {e} -- will retry')
            return
        self.resumed = True
        self.get_logger().info(f'resumed saved map from {self.base}.posegraph')

    def save(self):
        if self.cli_serialize.service_is_ready():
            req = SerializePoseGraph.Request()
            req.filename = self.base
            self.cli_serialize.call_async(req).add_done_callback(self._log_serialize)
        if self.cli_save_map.service_is_ready():
            req = SaveMap.Request()
            req.name = String(data=self.base)
            self.cli_save_map.call_async(req).add_done_callback(self._log_save_map)

    def _log_serialize(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'serialize_map failed: {e}')
            return
        if res.result == 0:
            self.get_logger().info(f'serialized pose graph -> {self.base}.posegraph/.data')
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
    # surgery on rclpy's shutdown sequence. Relying on the periodic save_period_s timer only -- a
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

# Per-node parameter YAML

One `<node_name>.yaml` per node with declared ROS parameters (Phase A2), loaded by the matching
`<node>` entry in `../launch/*.launch.xml` via `<param from="$(find-pkg-share ippolit_bringup)/config/<node_name>.yaml"/>`.
Deployed values only — the parameter descriptions/types/valid ranges live in each node's
`declare_parameter(...)` call, not here. `ROBOT_ADDR`-class machine-local deployment values
(host/IP, native library search paths) stay as env vars sourced from `/etc/default/ippolit-robot`
or set node-scoped in the launch file — see each node's module docstring for which category a
given knob falls into.

Files:
- `cliff_guard.yaml` — SAFETY-CRITICAL MiDaS/LiDAR cliff-detection thresholds. Do not change
  without re-verifying against a real stairwell drive (see `cliff_guard.py`'s docstring).
- `q6a_map_persist.yaml` — includes the SAFETY-CRITICAL `min_resume_bytes` deserialize guard
  (see `q6a_map_persist.py`'s CRASH FOUND note).
- `q6a_laser_odom.yaml`, `q6a_announce.yaml`, `q6a_objmap.yaml`, `q6a_vision.yaml` — regular
  tuning knobs, safe to adjust and redeploy.

#!/usr/bin/env python3
"""
q6a_vision.py — companion vision node.

Robot OV8856 -> YOLO(NPU)+ByteTrack -> ROS detections + view.

Pulls the robot's forward camera over the USB link (MJPEG on :8090), runs the w8a8 YOLOv8 on the
Hexagon NPU with ByteTrack for stable track IDs, publishes detections on ROS /vision/detections
(vision_msgs/Detection2DArray, typed per A3), and serves an annotated MJPEG on :8093 so you can
watch it. The robot already hands us JPEG frames, so the Q6A skips the GPU-ISP/demosaic stack
entirely — just decode -> YOLO.

Decision 2026-07-08: use the robot OV8856 (forward-facing, sees the room) over the Q6A IMX296 for
robot-perception — see docs/companion-autonomy.md.

Run: source /opt/ros/jazzy/setup.bash && \
     LD_LIBRARY_PATH=~/qairt_2.42.0.251225/lib/aarch64-oe-linux-gcc11.2 \
     ADSP_LIBRARY_PATH=~/qairt_2.42.0.251225/lib/hexagon-v68/unsigned python3 q6a_vision.py

Parameters are declared below (see ippolit_bringup/config/q6a_vision.yaml for the deployed
values); this replaces the earlier Q6A_CAM_*/Q6A_YOLO_CONF/Q6A_VISION_* environment-variable reads
(A2). LD_LIBRARY_PATH/ADSP_LIBRARY_PATH stay machine-local env vars (QNN/QAIRT native lib paths,
not node tunables) set node-scoped in perception.launch.xml.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import os
import threading
import time
import urllib.request

from ippolit_interfaces.msg import FloorDrop
from ippolit_perception.q6a_bytetrack import ByteTracker
from ippolit_perception.q6a_yolo import YoloDetector
import numpy as np
from PIL import Image, ImageDraw
from qai_appbuilder import DataType, QNNContext
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

MIDAS_RES = 256                                                # MiDaS-v21-small is 256x256
_PALETTE = [(255, 64, 64), (64, 200, 64), (64, 160, 255), (255, 200, 0),
            (255, 64, 255), (0, 220, 220), (255, 128, 0), (160, 96, 255)]


class Shared:
    rgb = None            # latest decoded frame (valid rows), numpy HxWx3 RGB
    dets = []             # [(x1,y1,x2,y2,label,conf,track_id,disp)]
    depth = None          # latest MiDaS 256x256 relative disparity map (for obstacle use later)
    annot = None          # latest annotated JPEG bytes (for the view server)
    seq = 0
    lock = threading.Lock()


def puller(mjpeg_url, valid_rows):
    """Pull the robot MJPEG stream, decode the newest JPEG into Shared.rgb (valid rows only)."""
    while True:
        try:
            r = urllib.request.urlopen(mjpeg_url, timeout=10)
            buf = b''
            while True:
                chunk = r.read(16384)
                if not chunk:
                    break
                buf += chunk
                while True:
                    i = buf.find(b'\xff\xd8')
                    j = buf.find(b'\xff\xd9', i + 2) if i >= 0 else -1
                    if i < 0 or j < 0:
                        break
                    jpg = buf[i:j + 2]
                    buf = buf[j + 2:]
                    try:
                        arr = np.asarray(Image.open(io.BytesIO(jpg)).convert('RGB'))[:valid_rows]
                        Shared.rgb = np.ascontiguousarray(arr)
                    except Exception:
                        pass
                if len(buf) > 4_000_000:
                    buf = buf[-1_000_000:]
        except Exception as e:
            print(f'[vision] mjpeg pull error: {e}; retrying', flush=True)
            time.sleep(1.0)


def _band_step(band):
    """
    Compute (max_step, step_at, sharpness, profile) for a floor band.

    row-medians bottom->far in 16 bins. max_step = largest RELATIVE fall between adjacent bins
    (MiDaS is affine-invariant -> relative). But a smooth floor's perspective decay ALSO has a
    steepest step (~0.24), so max_step alone false-alarms. sharpness = max_step / median|step|
    distinguishes a true drop-off (one bin falls far, neighbours smooth -> ratio ~5-7) from a
    smooth gradient (all steps similar -> ~1.1-1.8). A real cliff needs BOTH high.
    """
    rows = np.median(band, axis=1)[::-1]                   # [0] = bottom-most (nearest floor)
    idx = np.linspace(0, len(rows) - 1, 16).astype(int)
    prof = rows[idx]
    steps = (prof[:-1] - prof[1:]) / (prof[:-1] + 1e-6)    # relative fall per bin, near -> far
    mx = float(steps.max())
    at = int(steps.argmax())
    sharp = mx / (float(np.median(np.abs(steps))) + 1e-3)
    return round(mx, 3), at, round(sharp, 2), [round(float(v), 1) for v in prof]


def floor_profile(dmap):
    """
    Floor-band drop-off (stair) detection for cliff_guard — full-width AND per-sector.

    Bottom 45% of the MiDaS map = the floor ahead. A drop-off makes the floor plane jump far, a
    sharp step in the profile. Per-sector (left/center/right column thirds) so a consumer can
    tell a drop dead-ahead (center -> don't drive forward) from a drop to the side (edge
    alongside -> can travel parallel). max_step = largest relative fall; step_at = which bin
    (0=nearest -> ~closer edge).
    """
    h, w = dmap.shape
    band = dmap[int(h * 0.55):, :].astype(np.float32)
    ms, sa, sh, prof = _band_step(band)
    t = w // 3
    ls, la, lsh, _ = _band_step(band[:, :t])
    cs, ca, csh, _ = _band_step(band[:, t:2 * t])
    rs, ra, rsh, _ = _band_step(band[:, 2 * t:])
    return {
        'profile': prof,
        'max_step': ms, 'step_at': sa, 'sharp': sh,
        # per sector: [max_step, step_at, sharpness]
        'sectors': {'left': [ls, la, lsh], 'center': [cs, ca, csh], 'right': [rs, ra, rsh]},
        'frame_med': round(float(np.median(dmap)), 1),
    }


def _depth_rgb(dmap, w, h):
    """Colorize the MiDaS disparity map (jet-ish: near=red -> far=blue), resized to (w,h)."""
    if dmap is None:
        return np.zeros((h, w, 3), np.uint8)
    t = dmap.astype(np.float32) / 255.0
    rgb = np.stack([np.clip(1.5 - np.abs(4 * t - 3), 0, 1),
                    np.clip(1.5 - np.abs(4 * t - 2), 0, 1),
                    np.clip(1.5 - np.abs(4 * t - 1), 0, 1)], axis=-1)
    return np.asarray(Image.fromarray((rgb * 255).astype(np.uint8)).resize((w, h), Image.NEAREST))


def annotate(rgb, dets, dmap):
    """Composite view: [YOLO-boxed RGB | MiDaS depth colormap] side by side, for the :8093 view."""
    im = Image.fromarray(rgb)
    d = ImageDraw.Draw(im)
    for x1, y1, x2, y2, lab, cf, tid, dep in dets:
        col = _PALETTE[(tid if tid else hash(lab)) % len(_PALETTE)]
        d.rectangle([x1, y1, x2, y2], outline=col, width=3)
        tag = f'#{tid} {lab} {cf:.2f}'
        if dep >= 0:
            tag += f' d{dep}'   # d = relative disparity (higher=nearer)
        d.text((x1 + 2, y1 + 1), tag, fill=col)
    d.text((4, 4), 'YOLO+ByteTrack', fill=(255, 255, 0))
    h, w = rgb.shape[0], rgb.shape[1]
    right = _depth_rgb(dmap, w, h)
    rim = Image.fromarray(right)
    ImageDraw.Draw(rim).text((4, 4), 'MiDaS depth (red=near)', fill=(255, 255, 255))
    composite = np.hstack([np.asarray(im), np.asarray(rim)])
    b = io.BytesIO()
    Image.fromarray(composite).save(b, 'JPEG', quality=80)
    return b.getvalue()


class ViewHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            self.connection.settimeout(10.0)
            while True:
                j = Shared.annot
                if j:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n')
                time.sleep(0.1)
        except Exception:
            pass


class VisionNode(Node):
    def __init__(self):
        super().__init__('q6a_vision')
        self.declare_parameter(
            'mjpeg_url', 'http://192.168.10.1:8090/',
            ParameterDescriptor(description='Robot camera MJPEG stream URL.'))
        self.declare_parameter(
            'valid_rows', 504,
            ParameterDescriptor(
                description='Rows to keep from each frame (camstream pads 672x504->672x672).',
                integer_range=[IntegerRange(from_value=1, to_value=4096)]))
        self.declare_parameter(
            'yolo_conf', 0.30,
            ParameterDescriptor(
                description=(
                    'Detector confidence floor (0.1 leaked junk; real furniture is >=0.62, '
                    'floor false-positives like cat/laptop sit ~0.44-0.55).'),
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter(
            'view_port', 8093,
            ParameterDescriptor(
                description='TCP port serving the annotated MJPEG view.',
                integer_range=[IntegerRange(from_value=1, to_value=65535)]))
        self.declare_parameter(
            'det_fps', 8.0,
            ParameterDescriptor(
                description='Target detection loop rate (Hz); 0 disables throttling.',
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=60.0)]))
        self.declare_parameter(
            'midas_bin', os.path.expanduser('~/midas_depth_w8a8.bin'),
            ParameterDescriptor(description='Path to the MiDaS w8a8 QNN context binary.'))
        self.declare_parameter(
            'enable_depth', True,
            ParameterDescriptor(description='Run MiDaS depth alongside YOLO.'))

        self.mjpeg_url = self.get_parameter('mjpeg_url').value
        self.valid_rows = self.get_parameter('valid_rows').value
        self.conf = self.get_parameter('yolo_conf').value
        self.view_port = self.get_parameter('view_port').value
        self.det_fps = self.get_parameter('det_fps').value
        self.midas_bin = self.get_parameter('midas_bin').value
        self.enable_depth = self.get_parameter('enable_depth').value

        self.pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        # MiDaS floor profile (cliff cue)
        self.pub_floor = self.create_publisher(FloorDrop, '/vision/floor', 10)
        self.det = YoloDetector(conf=self.conf)     # Configs the HTP backend + loads YOLO context
        self.labels = self.det.labels
        self.tracker = ByteTracker(high_thresh=0.4, low_thresh=self.conf)
        self.midas = None
        if self.enable_depth and os.path.exists(self.midas_bin):  # 2nd NPU ctx, same process (OK)
            self.midas = QNNContext('midas_depth_w8a8', self.midas_bin,
                                    input_data_type=DataType.NATIVE,
                                    output_data_type=DataType.NATIVE)
            self.get_logger().info('MiDaS depth context loaded (per-detection relative disparity)')
        midas_tag = '+MiDaS' if self.midas else ''
        self.get_logger().info(
            f'q6a_vision up: {self.mjpeg_url} -> YOLO(NPU)+ByteTrack{midas_tag} '
            f'-> /vision/detections + :{self.view_port}')
        threading.Thread(
            target=puller, args=(self.mjpeg_url, self.valid_rows), daemon=True).start()
        srv = ThreadingHTTPServer(('0.0.0.0', self.view_port), ViewHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        threading.Thread(target=self.infer_loop, daemon=True).start()

    def depth_map(self, rgb):
        """Compute MiDaS inverse-depth (disparity) at 256x256; higher=nearer, affine-inv. (D1)."""
        if self.midas is None:
            return None
        small = np.ascontiguousarray(
            np.asarray(Image.fromarray(rgb).resize((MIDAS_RES, MIDAS_RES), Image.BILINEAR))
            .transpose(2, 0, 1)[None].astype(np.uint8))
        out = self.midas.Inference([small])
        arr = np.asarray(out[0] if isinstance(out, (list, tuple)) else out, dtype=np.uint8)
        return arr.reshape(MIDAS_RES, MIDAS_RES)

    @staticmethod
    def _det_disp(dmap, x1, y1, x2, y2, w, h):
        """Median MiDaS disparity inside a bbox (bbox in frame px -> 256 grid). -1 if no depth."""
        if dmap is None:
            return -1
        sx, sy = MIDAS_RES / w, MIDAS_RES / h
        patch = dmap[max(0, int(y1 * sy)):int(y2 * sy) + 1, max(0, int(x1 * sx)):int(x2 * sx) + 1]
        return int(np.median(patch)) if patch.size else -1

    def infer_loop(self):
        period = 1.0 / self.det_fps if self.det_fps > 0 else 0.0
        while rclpy.ok():
            t0 = time.time()
            rgb = Shared.rgb
            if rgb is None:
                time.sleep(0.05)
                continue
            try:
                out = self.det.infer(rgb)
            except Exception as e:
                self.get_logger().warn(f'infer error: {e}')
                time.sleep(0.2)
                continue
            boxes = [(d[0], d[1], d[2], d[3]) for d in out]
            scores = [d[5] for d in out]
            clsi = [self.labels.index(d[4]) if d[4] in self.labels else -1 for d in out]
            tracked = self.tracker.update(boxes, scores, clsi)   # (x1,y1,x2,y2,score,cls_idx,tid)
            h, w = int(rgb.shape[0]), int(rgb.shape[1])
            dmap = self.depth_map(rgb)                            # 256x256 disparity (or None)
            Shared.depth = dmap
            if dmap is not None:                                  # floor-drop cue for cliff_guard
                fm = floor_profile(dmap)
                sectors = fm['sectors']
                floor_msg = FloorDrop()
                floor_msg.header.stamp = self.get_clock().now().to_msg()
                floor_msg.left, _, floor_msg.left_sharp = sectors['left']
                floor_msg.center, _, floor_msg.center_sharp = sectors['center']
                floor_msg.right, _, floor_msg.right_sharp = sectors['right']
                self.pub_floor.publish(floor_msg)
            dets = []
            for (x1, y1, x2, y2, cf, ci, tid) in tracked:
                lab = self.labels[int(ci)] if 0 <= int(ci) < len(self.labels) else str(int(ci))
                disp = self._det_disp(dmap, x1, y1, x2, y2, w, h)
                dets.append((int(x1), int(y1), int(x2), int(y2), lab, float(cf), int(tid), disp))
            Shared.dets = dets
            Shared.annot = annotate(rgb, dets, dmap)
            Shared.seq += 1
            self.publish(dets)
            dt = time.time() - t0
            if period and dt < period:
                time.sleep(period - dt)

    def publish(self, dets):
        """
        Publish tracked detections as vision_msgs/Detection2DArray.

        MiDaS disparity (the last tuple element) isn't published here -- vision_msgs has no
        natural field for it, and nothing downstream currently consumes it (it only drives the
        local :8093 annotated view). Bbox is stored as vision_msgs' native center+size form, not
        the internal (x1,y1,x2,y2) corners.
        """
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x1, y1, x2, y2, lab, cf, tid, _dep) in dets:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = lab
            hyp.hypothesis.score = float(cf)
            det.results.append(hyp)
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            det.id = str(tid)
            msg.detections.append(det)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VisionNode()
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

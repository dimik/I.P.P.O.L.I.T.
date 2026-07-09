#!/usr/bin/env python3
"""q6a_vision.py — companion vision node: robot OV8856 -> YOLO(NPU)+ByteTrack -> ROS detections + view.

Pulls the robot's forward camera over the USB link (MJPEG on :8090), runs the w8a8 YOLOv8 on the Hexagon
NPU with ByteTrack for stable track IDs, publishes detections on ROS /vision/detections (JSON String), and
serves an annotated MJPEG on :8093 so you can watch it. The robot already hands us JPEG frames, so the Q6A
skips the GPU-ISP/demosaic stack entirely — just decode -> YOLO.

Decision 2026-07-08: use the robot OV8856 (forward-facing, sees the room) over the Q6A IMX296 for
robot-perception — see docs/companion-autonomy.md.

Run: source /opt/ros/jazzy/setup.bash && \
     LD_LIBRARY_PATH=~/qairt_2.42.0.251225/lib/aarch64-oe-linux-gcc11.2 \
     ADSP_LIBRARY_PATH=~/qairt_2.42.0.251225/lib/hexagon-v68/unsigned python3 q6a_vision.py
"""
import io, os, sys, json, time, threading, urllib.request
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.expanduser('~'))
from q6a_yolo import YoloDetector
from q6a_bytetrack import ByteTracker
from qai_appbuilder import QNNContext, DataType

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MJPEG_URL = os.environ.get('Q6A_CAM_URL', 'http://192.168.10.1:8090/')
VALID_ROWS = int(os.environ.get('Q6A_CAM_VALID_ROWS', '504'))   # camstream pads 672x504 -> 672x672
CONF = float(os.environ.get('Q6A_YOLO_CONF', '0.1'))            # low: ByteTrack recovers, spawns only >=0.4
VIEW_PORT = int(os.environ.get('Q6A_VISION_PORT', '8093'))
DET_FPS = float(os.environ.get('Q6A_VISION_FPS', '8'))
MIDAS_BIN = os.path.expanduser('~/midas_depth_w8a8.bin')
MIDAS_RES = 256                                                # MiDaS-v21-small is 256x256
DEPTH = os.environ.get('Q6A_VISION_DEPTH', '1') != '0'         # run MiDaS depth alongside YOLO
_PALETTE = [(255, 64, 64), (64, 200, 64), (64, 160, 255), (255, 200, 0),
            (255, 64, 255), (0, 220, 220), (255, 128, 0), (160, 96, 255)]


class Shared:
    rgb = None            # latest decoded frame (valid rows), numpy HxWx3 RGB
    dets = []             # [(x1,y1,x2,y2,label,conf,track_id,disp)]
    depth = None          # latest MiDaS 256x256 relative disparity map (for obstacle use later)
    annot = None          # latest annotated JPEG bytes (for the view server)
    seq = 0
    lock = threading.Lock()


def puller():
    """Pull the robot MJPEG stream, decode the newest JPEG into Shared.rgb (cropped to the valid rows)."""
    while True:
        try:
            r = urllib.request.urlopen(MJPEG_URL, timeout=10)
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
                    jpg = buf[i:j + 2]; buf = buf[j + 2:]
                    try:
                        arr = np.asarray(Image.open(io.BytesIO(jpg)).convert('RGB'))[:VALID_ROWS]
                        Shared.rgb = np.ascontiguousarray(arr)
                    except Exception:
                        pass
                if len(buf) > 4_000_000:
                    buf = buf[-1_000_000:]
        except Exception as e:
            print(f'[vision] mjpeg pull error: {e}; retrying', flush=True)
            time.sleep(1.0)


def _band_step(band):
    """(max_step, step_at, profile) for a floor band: row-medians bottom->far in 16 bins, largest
    RELATIVE fall between adjacent bins (MiDaS is affine-invariant -> relative, not absolute)."""
    rows = np.median(band, axis=1)[::-1]                   # [0] = bottom-most (nearest floor)
    idx = np.linspace(0, len(rows) - 1, 16).astype(int)
    prof = rows[idx]
    steps = (prof[:-1] - prof[1:]) / (prof[:-1] + 1e-6)    # relative fall per bin, near -> far
    return round(float(steps.max()), 3), int(steps.argmax()), [round(float(v), 1) for v in prof]


def floor_profile(dmap):
    """Floor-band drop-off (stair) detection for cliff_guard — full-width AND per-sector.

    Bottom 45% of the MiDaS map = the floor ahead. A drop-off makes the floor plane jump far, a sharp
    step in the profile. Per-sector (left/center/right column thirds) so a consumer can tell a drop
    dead-ahead (center -> don't drive forward) from a drop to the side (edge alongside -> can travel
    parallel). max_step = largest relative fall; step_at = which bin (0=nearest -> ~closer edge).
    """
    h, w = dmap.shape
    band = dmap[int(h * 0.55):, :].astype(np.float32)
    ms, sa, prof = _band_step(band)
    t = w // 3
    ls, la, _ = _band_step(band[:, :t])
    cs, ca, _ = _band_step(band[:, t:2 * t])
    rs, ra, _ = _band_step(band[:, 2 * t:])
    return {
        'profile': prof,
        'max_step': ms, 'step_at': sa,
        'sectors': {'left': [ls, la], 'center': [cs, ca], 'right': [rs, ra]},   # [max_step, step_at]
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
    """Composite view: [YOLO-boxed RGB | MiDaS depth colormap] side by side, for the :8093 stream."""
    im = Image.fromarray(rgb); d = ImageDraw.Draw(im)
    for x1, y1, x2, y2, lab, cf, tid, dep in dets:
        col = _PALETTE[(tid if tid else hash(lab)) % len(_PALETTE)]
        d.rectangle([x1, y1, x2, y2], outline=col, width=3)
        tag = f'#{tid} {lab} {cf:.2f}' + (f' d{dep}' if dep >= 0 else '')   # d = relative disparity (higher=nearer)
        d.text((x1 + 2, y1 + 1), tag, fill=col)
    d.text((4, 4), 'YOLO+ByteTrack', fill=(255, 255, 0))
    h, w = rgb.shape[0], rgb.shape[1]
    right = _depth_rgb(dmap, w, h)
    rim = Image.fromarray(right); ImageDraw.Draw(rim).text((4, 4), 'MiDaS depth (red=near)', fill=(255, 255, 255))
    composite = np.hstack([np.asarray(im), np.asarray(rim)])
    b = io.BytesIO(); Image.fromarray(composite).save(b, 'JPEG', quality=80)
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
        self.pub = self.create_publisher(String, '/vision/detections', 10)
        self.pub_floor = self.create_publisher(String, '/vision/floor', 10)   # MiDaS floor profile (cliff cue)
        self.det = YoloDetector(conf=CONF)          # Configs the HTP backend + loads the YOLO context
        self.labels = self.det.labels
        self.tracker = ByteTracker(high_thresh=0.4, low_thresh=CONF)
        self.midas = None
        if DEPTH and os.path.exists(MIDAS_BIN):     # 2nd NPU context in the same process (verified OK)
            self.midas = QNNContext('midas_depth_w8a8', MIDAS_BIN,
                                    input_data_type=DataType.NATIVE, output_data_type=DataType.NATIVE)
            self.get_logger().info('MiDaS depth context loaded (per-detection relative disparity)')
        self.get_logger().info(f'q6a_vision up: {MJPEG_URL} -> YOLO(NPU)+ByteTrack{"+MiDaS" if self.midas else ""} '
                               f'-> /vision/detections + :{VIEW_PORT}')
        threading.Thread(target=self.infer_loop, daemon=True).start()

    def depth_map(self, rgb):
        """MiDaS inverse-depth (disparity) at 256x256; higher = nearer. affine-invariant (see D1)."""
        if self.midas is None:
            return None
        small = np.ascontiguousarray(
            np.asarray(Image.fromarray(rgb).resize((MIDAS_RES, MIDAS_RES), Image.BILINEAR))
            .transpose(2, 0, 1)[None].astype(np.uint8))
        out = self.midas.Inference([small])
        return np.asarray(out[0] if isinstance(out, (list, tuple)) else out, dtype=np.uint8).reshape(MIDAS_RES, MIDAS_RES)

    @staticmethod
    def _det_disp(dmap, x1, y1, x2, y2, w, h):
        """Median MiDaS disparity inside a bbox (bbox in frame px -> 256 grid). -1 if no depth."""
        if dmap is None:
            return -1
        sx, sy = MIDAS_RES / w, MIDAS_RES / h
        patch = dmap[max(0, int(y1 * sy)):int(y2 * sy) + 1, max(0, int(x1 * sx)):int(x2 * sx) + 1]
        return int(np.median(patch)) if patch.size else -1

    def infer_loop(self):
        period = 1.0 / DET_FPS if DET_FPS > 0 else 0.0
        while rclpy.ok():
            t0 = time.time()
            rgb = Shared.rgb
            if rgb is None:
                time.sleep(0.05); continue
            try:
                out = self.det.infer(rgb)
            except Exception as e:
                self.get_logger().warn(f'infer error: {e}'); time.sleep(0.2); continue
            boxes = [(d[0], d[1], d[2], d[3]) for d in out]
            scores = [d[5] for d in out]
            clsi = [self.labels.index(d[4]) if d[4] in self.labels else -1 for d in out]
            tracked = self.tracker.update(boxes, scores, clsi)   # (x1,y1,x2,y2,score,cls_idx,tid)
            h, w = int(rgb.shape[0]), int(rgb.shape[1])
            dmap = self.depth_map(rgb)                            # 256x256 relative disparity (or None)
            Shared.depth = dmap
            if dmap is not None:                                  # floor-drop cue for cliff_guard
                fm = floor_profile(dmap)
                fm['stamp'] = self.get_clock().now().nanoseconds
                self.pub_floor.publish(String(data=json.dumps(fm)))
            dets = []
            for (x1, y1, x2, y2, cf, ci, tid) in tracked:
                lab = self.labels[int(ci)] if 0 <= int(ci) < len(self.labels) else str(int(ci))
                disp = self._det_disp(dmap, x1, y1, x2, y2, w, h)
                dets.append((int(x1), int(y1), int(x2), int(y2), lab, float(cf), int(tid), disp))
            Shared.dets = dets
            Shared.annot = annotate(rgb, dets, dmap)
            Shared.seq += 1
            self.publish(dets, rgb.shape)
            dt = time.time() - t0
            if period and dt < period:
                time.sleep(period - dt)

    def publish(self, dets, shape):
        msg = String()
        msg.data = json.dumps({
            'stamp': self.get_clock().now().nanoseconds,
            'w': int(shape[1]), 'h': int(shape[0]),
            'dets': [{'label': l, 'conf': round(cf, 3), 'bbox': [x1, y1, x2, y2], 'id': tid, 'disp': dep}
                     for (x1, y1, x2, y2, l, cf, tid, dep) in dets],
        })
        self.pub.publish(msg)


def main():
    threading.Thread(target=puller, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', VIEW_PORT), ViewHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
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

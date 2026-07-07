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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MJPEG_URL = os.environ.get('Q6A_CAM_URL', 'http://192.168.10.1:8090/')
VALID_ROWS = int(os.environ.get('Q6A_CAM_VALID_ROWS', '504'))   # camstream pads 672x504 -> 672x672
CONF = float(os.environ.get('Q6A_YOLO_CONF', '0.1'))            # low: ByteTrack recovers, spawns only >=0.4
VIEW_PORT = int(os.environ.get('Q6A_VISION_PORT', '8093'))
DET_FPS = float(os.environ.get('Q6A_VISION_FPS', '8'))
_PALETTE = [(255, 64, 64), (64, 200, 64), (64, 160, 255), (255, 200, 0),
            (255, 64, 255), (0, 220, 220), (255, 128, 0), (160, 96, 255)]


class Shared:
    rgb = None            # latest decoded frame (valid rows), numpy HxWx3 RGB
    dets = []             # [(x1,y1,x2,y2,label,conf,track_id)]
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


def annotate(rgb, dets):
    im = Image.fromarray(rgb); d = ImageDraw.Draw(im)
    for x1, y1, x2, y2, lab, cf, tid in dets:
        col = _PALETTE[(tid if tid else hash(lab)) % len(_PALETTE)]
        d.rectangle([x1, y1, x2, y2], outline=col, width=3)
        d.text((x1 + 2, y1 + 1), f'#{tid} {lab} {cf:.2f}', fill=col)
    b = io.BytesIO(); im.save(b, 'JPEG', quality=80)
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
        self.det = YoloDetector(conf=CONF)
        self.labels = self.det.labels
        self.tracker = ByteTracker(high_thresh=0.4, low_thresh=CONF)
        self.get_logger().info(f'q6a_vision up: {MJPEG_URL} -> YOLO(NPU)+ByteTrack -> /vision/detections + :{VIEW_PORT}')
        threading.Thread(target=self.infer_loop, daemon=True).start()

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
            dets = [(int(x1), int(y1), int(x2), int(y2),
                     self.labels[int(ci)] if 0 <= int(ci) < len(self.labels) else str(int(ci)),
                     float(cf), int(tid)) for (x1, y1, x2, y2, cf, ci, tid) in tracked]
            Shared.dets = dets
            Shared.annot = annotate(rgb, dets)
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
            'dets': [{'label': l, 'conf': round(cf, 3), 'bbox': [x1, y1, x2, y2], 'id': tid}
                     for (x1, y1, x2, y2, l, cf, tid) in dets],
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

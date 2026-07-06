"""On-device YOLOv11 COCO detector for the Q6A (Hexagon v68 NPU).

Runs a QNN context binary (built on-device from the AI-Hub qnn_dlc export, w8a16) via QAIRT 2.42 /
qai_appbuilder. The model is exported with a fused detection head, so its outputs are already decoded:
    boxes     (8400, 4)  xyxy in the 640x640 letterboxed input space
    scores    (8400,)    max class confidence 0..1
    class_idx (8400,)    COCO class index 0..79
We only threshold + NMS + map boxes back to the source-frame coordinates.

qai_appbuilder does float I/O: we feed normalised RGB in [0,1] (it quantises to the graph's uint16
input) and it returns dequantised float outputs. Import is lazy so the streamer still runs if the NPU
stack or the model binary is missing (detector just stays disabled).
"""
import os
import numpy as np
from PIL import Image

# Prefer the w8a8 build (~45% faster HTP inference, ~22->~12ms; identical on confident detections, softer on
# marginal <0.5-conf ones). Fall back to the w8a16 build if the w8a8 binary isn't deployed.
_W8A8 = os.path.expanduser("~/yolov8_det_w8a8.bin")
MODEL_BIN = _W8A8 if os.path.exists(_W8A8) else os.path.expanduser("~/yolov8_det.bin")
LABELS_TXT = os.path.expanduser("~/coco_labels.txt")
IN = 640                                                # model input size


def _nms(boxes, scores, iou_thr):
    """Greedy non-max suppression. boxes: (N,4) xyxy. Returns kept indices."""
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    area = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]; keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]]); yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]]); yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0, xx2 - xx1); h = np.maximum(0, yy2 - yy1)
        inter = w * h
        iou = inter / (area[i] + area[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return keep


class YoloDetector:
    def __init__(self, model=MODEL_BIN, labels=LABELS_TXT, conf=0.30, iou=0.45):
        from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel, DataType
        QNNConfig.Config(Runtime.HTP, LogLevel.WARN, ProfilingLevel.OFF)  # bundled 2.42 v68 backend
        # w8a8's input is quantized uint8 (scale 1/255), so feeding the raw uint8 letterbox as NATIVE input
        # is bit-identical to the float[0,1] path (verified: same boxes/conf) but replaces the ~5-8ms
        # copyFromFloatToNative quantize with a ~0.3ms memcpy. Only valid for the 8-bit-input w8a8 model;
        # the w8a16 fallback keeps float I/O (its native input is 16-bit, a different encoding).
        self.native = str(model).endswith("_w8a8.bin")
        if self.native:
            self.ctx = QNNContext("yolov8_det", model,
                                  input_data_type=DataType.NATIVE, output_data_type=DataType.FLOAT)
        else:
            self.ctx = QNNContext("yolov8_det", model)   # YOLOv8 (v11 doesn't run on v68); float I/O
        self.conf = conf
        self.iou = iou
        self.labels = ([l.strip() for l in open(labels) if l.strip()]
                       if os.path.exists(labels) else [str(i) for i in range(80)])
        try:
            self.onames = [str(n).lower() for n in self.ctx.getOutputName()]
        except Exception:
            self.onames = None

    def _letterbox(self, rgb):
        """Resize keeping aspect ratio, pad bottom-right to IN x IN (qai-hub YOLO convention -> the
        model scores much higher this way than with centered padding). Returns (img, scale, 0, 0)."""
        h, w = rgb.shape[:2]
        s = min(IN / w, IN / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        im = np.asarray(Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR))
        pad = np.full((IN, IN, 3), 114, np.uint8)
        pad[:nh, :nw] = im
        return pad, s, 0, 0

    def _map_outputs(self, out):
        """Return (boxes(N,4), scores(N,), class_idx(N,)) regardless of graph output order."""
        arrs = [np.asarray(o, dtype=np.float32).ravel() for o in out]
        if self.onames and len(self.onames) == len(arrs):                 # map by name if available
            by = dict(zip(self.onames, arrs))
            b = next((v for k, v in by.items() if "box" in k), None)
            sc = next((v for k, v in by.items() if "score" in k or "conf" in k), None)
            cl = next((v for k, v in by.items() if "class" in k or "idx" in k or "label" in k), None)
            if b is not None and sc is not None and cl is not None:
                return b.reshape(-1, 4), sc, cl
        # fallback by shape/values: the 4N array is boxes; of the two N arrays, the integer-valued one
        # (all in [0,80)) is class_idx and the other is scores
        boxes = next((a.reshape(-1, 4) for a in arrs if a.size % 4 == 0 and a.size >= 32000), None)
        n = boxes.shape[0] if boxes is not None else 8400
        flats = [a for a in arrs if a.size == n]
        cl = next((a for a in flats if a.size and a.max() < 80 and np.allclose(a, np.round(a), atol=1e-2)), None)
        sc = next((a for a in flats if a is not cl), None)
        return boxes, sc, cl

    def infer(self, rgb):
        """rgb: (H,W,3) uint8. Returns list of (x1,y1,x2,y2,label,conf) in source-frame pixels."""
        lb, s, ox, oy = self._letterbox(rgb)
        # model input is NCHW [1,3,640,640]. NATIVE path: feed raw uint8 (the graph's input quant = scale
        # 1/255, so uint8 pixels ARE the native tensor) -> a memcpy, no float quantize. FLOAT path (w8a16):
        # values [0,1], appbuilder quantises float->native for us.
        if self.native:
            x = np.ascontiguousarray(lb.transpose(2, 0, 1)).reshape(1, 3, IN, IN)   # uint8
        else:
            x = np.ascontiguousarray((lb.astype(np.float32) / 255.0).transpose(2, 0, 1)).reshape(1, 3, IN, IN)
        out = self.ctx.Inference([x])
        boxes, scores, cls = self._map_outputs(out)
        if boxes is None or scores is None or cls is None:
            return []
        m = scores >= self.conf
        if not m.any():
            return []
        boxes, scores, cls = boxes[m], scores[m], cls[m].astype(int)
        keep = _nms(boxes, scores, self.iou)
        H, W = rgb.shape[:2]
        dets = []
        for i in keep:
            x1, y1, x2, y2 = boxes[i]
            x1 = (x1 - ox) / s; y1 = (y1 - oy) / s; x2 = (x2 - ox) / s; y2 = (y2 - oy) / s
            x1 = max(0, min(W - 1, x1)); x2 = max(0, min(W - 1, x2))
            y1 = max(0, min(H - 1, y1)); y2 = max(0, min(H - 1, y2))
            c = cls[i]
            label = self.labels[c] if 0 <= c < len(self.labels) else str(c)
            dets.append((int(x1), int(y1), int(x2), int(y2), label, float(scores[i])))
        return dets


if __name__ == "__main__":               # standalone test: q6a_yolo.py <image-or-raw>
    import sys, time
    d = YoloDetector()
    img = np.asarray(Image.open(sys.argv[1]).convert("RGB")) if len(sys.argv) > 1 else \
        (np.random.rand(1088, 1456, 3) * 255).astype(np.uint8)
    t = time.time(); dets = d.infer(img); dt = time.time() - t
    print(f"{len(dets)} detections in {dt*1000:.0f} ms")
    for x1, y1, x2, y2, lab, cf in dets:
        print(f"  {lab:16s} {cf:.2f}  [{x1},{y1},{x2},{y2}]")

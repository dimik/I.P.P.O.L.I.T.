"""
Lightweight ByteTrack for the Q6A detector (numpy-only, no scipy).

Assigns persistent track IDs to per-frame YOLO detections and smooths/predicts boxes with a
constant-velocity Kalman filter. ByteTrack's trick: associate HIGH-confidence detections first,
then recover tracks with the LEFTOVER low-confidence detections a plain tracker would discard —
so an object briefly seen at 0.3 conf keeps its ID instead of flickering. Matching is per-class,
greedy on IoU (Hungarian-free: near optimal for the <50 mostly-disjoint boxes we see, and scipy
isn't on the device).

Runs in the detector process right after infer() (<1 ms for tens of boxes on a Silver core).
Output rows carry a stable track_id that flows through the detection shm to the overlay +
downstream.
"""
import numpy as np


def _iou_matrix(a, b):
    """Compute IoU between every box in a (N,4 xyxy) and b (M,4 xyxy) -> (N,M)."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), np.float32)
    a = a[:, None, :]
    b = b[None, :, :]
    ix1 = np.maximum(a[..., 0], b[..., 0])
    iy1 = np.maximum(a[..., 1], b[..., 1])
    ix2 = np.minimum(a[..., 2], b[..., 2])
    iy2 = np.minimum(a[..., 3], b[..., 3])
    iw = np.clip(ix2 - ix1, 0, None)
    ih = np.clip(iy2 - iy1, 0, None)
    inter = iw * ih
    aa = np.clip(a[..., 2] - a[..., 0], 0, None) * np.clip(a[..., 3] - a[..., 1], 0, None)
    ab = np.clip(b[..., 2] - b[..., 0], 0, None) * np.clip(b[..., 3] - b[..., 1], 0, None)
    return (inter / (aa + ab - inter + 1e-9)).astype(np.float32)


def _greedy_match(iou, thr):
    """Greedy IoU matching. Returns (matches [(i,j)], unmatched_rows, unmatched_cols)."""
    matches = []
    if iou.size:
        iou = iou.copy()
        while True:
            i, j = np.unravel_index(np.argmax(iou), iou.shape)
            if iou[i, j] < thr:
                break
            matches.append((i, j))
            iou[i, :] = -1
            iou[:, j] = -1
    mr = {m[0] for m in matches}
    mc = {m[1] for m in matches}
    ur = [i for i in range(iou.shape[0]) if i not in mr]
    uc = [j for j in range(iou.shape[1]) if j not in mc]
    return matches, ur, uc


class _Track:
    """Constant-velocity Kalman box tracker. State x = [cx, cy, w, h, vcx, vcy, vw, vh]."""

    _next_id = 1

    def __init__(self, box, score, cls):
        cx, cy, w, h = _to_cxcywh(box)
        self.x = np.array([cx, cy, w, h, 0, 0, 0, 0], np.float32)
        self.P = np.diag([10, 10, 10, 10, 1e4, 1e4, 1e4, 1e4]).astype(np.float32)
        self.cls = int(cls)
        self.score = float(score)
        self.id = _Track._next_id
        _Track._next_id += 1
        self.time_since_update = 0     # frames since a real detection matched
        self.hits = 1                  # total matched detections
        self.age = 0

    # constant-velocity transition (dt = 1 frame) + modest process/measurement noise
    _F = np.eye(8, dtype=np.float32)
    _F[0, 4] = _F[1, 5] = _F[2, 6] = _F[3, 7] = 1.0
    _H = np.zeros((4, 8), np.float32)
    _H[0, 0] = _H[1, 1] = _H[2, 2] = _H[3, 3] = 1.0
    _Q = np.diag([1, 1, 1, 1, 10, 10, 10, 10]).astype(np.float32)
    _R = np.diag([1, 1, 10, 10]).astype(np.float32)

    def predict(self):
        self.x = _Track._F @ self.x
        self.P = _Track._F @ self.P @ _Track._F.T + _Track._Q
        self.x[2] = max(self.x[2], 1.0)   # keep w,h positive
        self.x[3] = max(self.x[3], 1.0)
        self.age += 1
        self.time_since_update += 1

    def update(self, box, score, cls):
        z = np.array(_to_cxcywh(box), np.float32)
        y = z - _Track._H @ self.x
        S = _Track._H @ self.P @ _Track._H.T + _Track._R
        K = self.P @ _Track._H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(8, dtype=np.float32) - K @ _Track._H) @ self.P
        self.score = float(score)
        self.cls = int(cls)
        self.time_since_update = 0
        self.hits += 1

    def box(self):
        return _to_xyxy(self.x[:4])


def _to_cxcywh(b):
    return (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, b[2] - b[0], b[3] - b[1]


def _to_xyxy(c):
    cx, cy, w, h = c
    return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], np.float32)


class ByteTracker:
    def __init__(self, high_thresh=0.5, low_thresh=0.1, match_iou=0.2, max_age=30, min_hits=2):
        self.high = high_thresh          # detections >= this drive the first association + spawn
        self.low = low_thresh            # detections in [low, high) only RECOVER existing tracks
        self.match_iou = match_iou
        self.max_age = max_age           # drop a track after this many frames with no detection
        self.min_hits = min_hits         # confirm a track (emit its id) after this many hits
        self.tracks = []

    def update(self, boxes, scores, classes):
        """
        Update tracks with a new frame's detections.

        boxes (N,4 xyxy), scores (N,), classes (N,). Returns list of
        (x1,y1,x2,y2,score,class_idx,track_id) for confirmed, currently-visible tracks.
        """
        boxes = np.asarray(boxes, np.float32).reshape(-1, 4)
        scores = np.asarray(scores, np.float32).reshape(-1)
        classes = np.asarray(classes, np.int32).reshape(-1)
        for t in self.tracks:
            t.predict()

        hi = scores >= self.high
        lo = (scores >= self.low) & ~hi
        remaining = list(range(len(self.tracks)))

        # --- stage 1: high-confidence detections vs all tracks (per class) ---
        det_hi = np.where(hi)[0]
        remaining = self._associate(det_hi, remaining, boxes, scores, classes, spawn=True)
        # --- stage 2: leftover tracks vs low-confidence detections (recover, no spawn) ---
        det_lo = np.where(lo)[0]
        remaining = self._associate(det_lo, remaining, boxes, scores, classes, spawn=False)

        # age out dead tracks
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

        out = []
        for t in self.tracks:
            if t.time_since_update == 0 and (t.hits >= self.min_hits or t.age < self.min_hits):
                b = t.box()
                out.append((float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                            t.score, t.cls, t.id))
        return out

    def _associate(self, det_idx, track_idx, boxes, scores, classes, spawn):
        """Match the given detection indices to the given track indices (same-class, greedy)."""
        if len(det_idx) == 0:
            return track_idx
        avail = list(track_idx)
        if avail:
            tb = np.array([self.tracks[i].box() for i in avail], np.float32)
            tc = np.array([self.tracks[i].cls for i in avail])
            db = boxes[det_idx]
            dc = classes[det_idx]
            iou = _iou_matrix(tb, db)
            iou[tc[:, None] != dc[None, :]] = 0.0          # forbid cross-class matches
            matches, ur, uc = _greedy_match(iou, self.match_iou)
            for ti, di in matches:
                d = det_idx[di]
                self.tracks[avail[ti]].update(boxes[d], scores[d], classes[d])
            leftover_tracks = [avail[i] for i in ur]
            leftover_dets = [det_idx[j] for j in uc]
        else:
            leftover_tracks = []
            leftover_dets = list(det_idx)
        if spawn:
            for d in leftover_dets:
                self.tracks.append(_Track(boxes[d], scores[d], classes[d]))
        return leftover_tracks

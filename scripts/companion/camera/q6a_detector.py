"""Separate-process YOLO detector for the Q6A (NPU only).

Reads the latest camera frame from shared memory (written by the GPU-ISP streamer) and writes
detections back. Running the NPU here — in a *different process* from the Adreno GPU ISP — is what
lets the two accelerators run truly concurrently: in one process the OpenCL and QNN/fastrpc userspace
stacks corrupt shared allocator state and segfault (that was the reason for the old in-process ACCEL
lock). Separate address spaces + the kernel's multi-client arbitration = no lock, full concurrency.

Shared-memory layout (must match q6a_camstream.py):
  q6a_frame : H*W*3 uint8 RGB  (single writer = streamer)
  q6a_ctrl  : [0]=frame_seq u64  [8]=det_seq u64  [16]=det_count i32  [32:]=MAX_DET x 7 f32
              det row = (x1, y1, x2, y2, conf, class_idx, track_id)   # track_id from ByteTrack (P2.1)
"""
import os, signal, time
import numpy as np
from multiprocessing import shared_memory, resource_tracker

W, H = 1456, 1088
MAX_DET = 32
CTRL_OFF = 32
DET_FPS = float(os.environ.get("Q6A_DET_FPS", "10"))   # 0 = unlimited (run at the NPU inference ceiling)
MIN_PERIOD = (1.0 / DET_FPS) if DET_FPS > 0 else 0.0    # min seconds between inferences (NPU duty-cycle cap)

_stop = False


def _on_signal(signum, frame):
    global _stop
    _stop = True                               # break the loop -> the finally block releases the NPU + shm


def _untrack(shm):
    """Detach a shm segment we ATTACHED (streamer is the owner). Py3.12's resource_tracker unlinks every
    segment it knows about at process exit, so without this the detector dying would destroy the streamer's
    live q6a_frame/q6a_ctrl. We only ever read/write them; the streamer creates and unlinks them."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


def main():
    from q6a_yolo import YoloDetector
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    fshm = cshm = None
    for _ in range(300):                       # wait for the streamer to create the shm
        try:
            fshm = shared_memory.SharedMemory(name="q6a_frame")
            cshm = shared_memory.SharedMemory(name="q6a_ctrl")
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if fshm is None:
        print("[detector] shm not found; exiting", flush=True); return
    _untrack(fshm); _untrack(cshm)             # never let our exit unlink the streamer's segments
    fseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=0)
    dseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=8)
    dcnt = np.ndarray((1,), np.int32, buffer=cshm.buf, offset=16)
    ow_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=24)
    oh_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=26)
    dbuf = np.ndarray((MAX_DET, 7), np.float32, buffer=cshm.buf, offset=CTRL_OFF)
    for _ in range(300):                       # wait for the streamer to publish output dims
        if int(ow_a[0]) > 0 and int(oh_a[0]) > 0: break
        time.sleep(0.05)
    ow, oh = int(ow_a[0]) or W, int(oh_a[0]) or H
    frame = np.ndarray((oh, ow, 3), np.uint8, buffer=fshm.buf)
    print(f"[detector] frame {ow}x{oh}", flush=True)

    # conf=0.1 so ByteTrack gets the LOW-confidence pool it needs for the recovery stage (it only spawns
    # tracks from high-conf dets, so low-conf boxes can't create false tracks — they only extend existing).
    det = YoloDetector(conf=0.1)
    labels = det.labels
    from q6a_bytetrack import ByteTracker
    tracker = ByteTracker(high_thresh=0.4, low_thresh=0.1)
    print("[detector] YOLO ready on NPU (separate process, no lock)", flush=True)
    print(f"[detector] rate cap: {DET_FPS:g} fps" if MIN_PERIOD else "[detector] rate: unlimited (NPU ceiling)", flush=True)
    last = 0; t_prev = 0.0
    try:
      while not _stop:
        s = int(fseq[0])
        if s == last:
            time.sleep(0.004); continue        # no new frame
        # Rate cap: don't infer more than DET_FPS/s. We still always take the FRESHEST frame (frame-drop),
        # just less often -> lower NPU duty (cooler, frees the HTP for the LLM) with no visual loss since the
        # streamer persists the last boxes between updates.
        if MIN_PERIOD:
            wait = MIN_PERIOD - (time.time() - t_prev)
            if wait > 0:
                time.sleep(wait); continue     # loop back and re-read fseq -> newest frame after the wait
        last = s
        img = frame.copy()                      # snapshot
        if int(fseq[0]) != s:                   # streamer overwrote it mid-copy -> skip, grab next
            continue
        t_prev = time.time()
        try:
            out = det.infer(img)                # NPU inference (concurrent with the GPU ISP process)
        except Exception as e:
            print("[detector] infer error:", e, flush=True); time.sleep(0.2); continue
        # ByteTrack: assign stable track IDs (Kalman predict + two-stage IoU association). <1ms.
        boxes = [(d[0], d[1], d[2], d[3]) for d in out]
        scores = [d[5] for d in out]
        clsi = [labels.index(d[4]) if d[4] in labels else -1 for d in out]
        tracked = tracker.update(boxes, scores, clsi)     # (x1,y1,x2,y2,score,cls_idx,track_id)
        n = min(len(tracked), MAX_DET)
        # seqlock publish (mirror of the frame channel): ODD while writing dbuf+dcnt, EVEN when done,
        # so the streamer never overlays a half-written detection set (rows not matching dcnt).
        sq = int(dseq[0])
        dseq[0] = sq + 1                        # odd: write in progress
        for i in range(n):
            x1, y1, x2, y2, cf, ci, tid = tracked[i]
            dbuf[i] = (x1, y1, x2, y2, cf, ci, tid)
        dcnt[0] = n
        dseq[0] = sq + 2                        # even: complete
    finally:
        print("[detector] shutting down: releasing NPU context + shm", flush=True)
        try: det.ctx.release()                  # free the QNN/HTP context cleanly (don't leave fastrpc state)
        except Exception as e: print("[detector] ctx release:", e, flush=True)
        for s in (fshm, cshm):
            try: s.close()                       # detach only (unregistered above) — the streamer unlinks
            except Exception: pass


if __name__ == "__main__":
    main()

"""Separate-process YOLO detector for the Q6A (NPU only).

Reads the latest camera frame from shared memory (written by the GPU-ISP streamer) and writes
detections back. Running the NPU here — in a *different process* from the Adreno GPU ISP — is what
lets the two accelerators run truly concurrently: in one process the OpenCL and QNN/fastrpc userspace
stacks corrupt shared allocator state and segfault (that was the reason for the old in-process ACCEL
lock). Separate address spaces + the kernel's multi-client arbitration = no lock, full concurrency.

Shared-memory layout (must match q6a_camstream.py):
  q6a_frame : H*W*3 uint8 RGB  (single writer = streamer)
  q6a_ctrl  : [0]=frame_seq u64  [8]=det_seq u64  [16]=det_count i32  [32:]=MAX_DET x 6 f32
              det row = (x1, y1, x2, y2, conf, class_idx)
"""
import os, time
import numpy as np
from multiprocessing import shared_memory

W, H = 1456, 1088
MAX_DET = 32
CTRL_OFF = 32
DET_FPS = float(os.environ.get("Q6A_DET_FPS", "10"))   # 0 = unlimited (run at the NPU inference ceiling)
MIN_PERIOD = (1.0 / DET_FPS) if DET_FPS > 0 else 0.0    # min seconds between inferences (NPU duty-cycle cap)


def main():
    from q6a_yolo import YoloDetector
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
    fseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=0)
    dseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=8)
    dcnt = np.ndarray((1,), np.int32, buffer=cshm.buf, offset=16)
    ow_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=24)
    oh_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=26)
    dbuf = np.ndarray((MAX_DET, 6), np.float32, buffer=cshm.buf, offset=CTRL_OFF)
    for _ in range(300):                       # wait for the streamer to publish output dims
        if int(ow_a[0]) > 0 and int(oh_a[0]) > 0: break
        time.sleep(0.05)
    ow, oh = int(ow_a[0]) or W, int(oh_a[0]) or H
    frame = np.ndarray((oh, ow, 3), np.uint8, buffer=fshm.buf)
    print(f"[detector] frame {ow}x{oh}", flush=True)

    det = YoloDetector()
    labels = det.labels
    print("[detector] YOLO ready on NPU (separate process, no lock)", flush=True)
    print(f"[detector] rate cap: {DET_FPS:g} fps" if MIN_PERIOD else "[detector] rate: unlimited (NPU ceiling)", flush=True)
    last = 0; t_prev = 0.0
    while True:
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
        n = min(len(out), MAX_DET)
        # seqlock publish (mirror of the frame channel): ODD while writing dbuf+dcnt, EVEN when done,
        # so the streamer never overlays a half-written detection set (rows not matching dcnt).
        sq = int(dseq[0])
        dseq[0] = sq + 1                        # odd: write in progress
        for i in range(n):
            x1, y1, x2, y2, lab, cf = out[i]
            ci = labels.index(lab) if lab in labels else -1
            dbuf[i] = (x1, y1, x2, y2, cf, ci)
        dcnt[0] = n
        dseq[0] = sq + 2                        # even: complete


if __name__ == "__main__":
    main()

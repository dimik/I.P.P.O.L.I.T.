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
import time
import numpy as np
from multiprocessing import shared_memory

W, H = 1456, 1088
MAX_DET = 32
CTRL_OFF = 32


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
    frame = np.ndarray((H, W, 3), np.uint8, buffer=fshm.buf)
    fseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=0)
    dseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=8)
    dcnt = np.ndarray((1,), np.int32, buffer=cshm.buf, offset=16)
    dbuf = np.ndarray((MAX_DET, 6), np.float32, buffer=cshm.buf, offset=CTRL_OFF)

    det = YoloDetector()
    labels = det.labels
    print("[detector] YOLO ready on NPU (separate process, no lock)", flush=True)
    last = 0
    while True:
        s = int(fseq[0])
        if s == last:
            time.sleep(0.004); continue        # no new frame
        last = s
        img = frame.copy()                      # snapshot
        if int(fseq[0]) != s:                   # streamer overwrote it mid-copy -> skip, grab next
            continue
        try:
            out = det.infer(img)                # NPU inference (concurrent with the GPU ISP process)
        except Exception as e:
            print("[detector] infer error:", e, flush=True); time.sleep(0.2); continue
        n = min(len(out), MAX_DET)
        for i in range(n):
            x1, y1, x2, y2, lab, cf = out[i]
            ci = labels.index(lab) if lab in labels else -1
            dbuf[i] = (x1, y1, x2, y2, cf, ci)
        dcnt[0] = n
        dseq[0] += 1                            # publish


if __name__ == "__main__":
    main()

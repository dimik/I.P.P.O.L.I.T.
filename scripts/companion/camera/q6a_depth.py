"""Separate-process MiDaS-V2 monocular depth for the Q6A (NPU only). Plan P2.3 runtime.

Third accelerator process (like q6a_detector.py): reads the latest camera frame from the streamer's shared
memory and writes an inverse-depth map back to its own shm. Runs the NPU in a *different process* from the
Adreno GPU ISP and from the YOLO detector — separate address spaces + the kernel's multi-client arbitration
give true concurrency with no lock (proven: detector+depth coexist, no dmabuf leak, ~7.6 ms depth under
contention). The 2026-07-07 coexistence test validated depth+detector; a healthy resident LLM as a 3rd
active context is still gated on the pinned-memory re-measure (LLM ~1.7 GB + detector + depth vs 12 GB).

Shared memory:
  q6a_frame : oh*ow*3 uint8 RGB              (read; single writer = streamer)
  q6a_ctrl  : [0]=frame_seq u64 ...          (read fseq for the frame seqlock; layout in q6a_detector.py)
  q6a_depth : [0]=depth_seq u64  [8]=dw u16 [10]=dh u16  [16]=scale f32 [20]=shift f32  [64:]=dh*dw uint8
              depth map = MiDaS INVERSE-depth (disparity), affine-invariant. scale/shift are the metric
              affine fit (metric_depth = 1/(scale*disp + shift)); 0/0 = unscaled (no LiDAR fit yet).

Model I/O (pre-quantized w8a8, native uint8): image[1,3,256,256] -> depth_estimates[1,1,256,256].
Input dequant is scale=0.00487531 zp=24, so raw uint8 pixels ~= pixel/255; the small approximation is
absorbed by the affine metric rescale (MiDaS output is affine-invariant). See build_depth.sh.
"""
import os, signal, time
import numpy as np
from multiprocessing import shared_memory, resource_tracker
from PIL import Image

DEPTH_RES = 256                                            # MiDaS-v21-small input/output is 256x256
DEPTH_OFF = 64                                             # depth map starts here (header in [0,64))
DEPTH_FPS = float(os.environ.get("Q6A_DEPTH_FPS", "5"))    # cap NPU depth duty (0 = unlimited)
MIN_PERIOD = (1.0 / DEPTH_FPS) if DEPTH_FPS > 0 else 0.0
MODEL = os.path.expanduser("~/midas_depth_w8a8.bin")

_stop = False


def _on_signal(signum, frame):
    global _stop
    _stop = True                                           # break loop -> finally releases NPU + shm


def _untrack(shm):
    """We only ATTACH the streamer's segments; Py3.12 resource_tracker would unlink them on our exit."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


def main():
    from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel, DataType
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    fshm = cshm = dshm = None
    for _ in range(300):                                   # wait for the streamer to create the shm
        try:
            fshm = shared_memory.SharedMemory(name="q6a_frame")
            cshm = shared_memory.SharedMemory(name="q6a_ctrl")
            dshm = shared_memory.SharedMemory(name="q6a_depth")
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if fshm is None or dshm is None:
        print("[depth] shm not found; exiting", flush=True); return
    _untrack(fshm); _untrack(cshm); _untrack(dshm)         # never unlink the streamer's segments on our exit

    fseq = np.ndarray((1,), np.uint64, buffer=cshm.buf, offset=0)
    ow_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=24)
    oh_a = np.ndarray((1,), np.uint16, buffer=cshm.buf, offset=26)
    for _ in range(300):                                   # wait for the streamer to publish frame dims
        if int(ow_a[0]) > 0 and int(oh_a[0]) > 0: break
        time.sleep(0.05)
    ow, oh = int(ow_a[0]) or DEPTH_RES, int(oh_a[0]) or DEPTH_RES
    frame = np.ndarray((oh, ow, 3), np.uint8, buffer=fshm.buf)

    dseq = np.ndarray((1,), np.uint64, buffer=dshm.buf, offset=0)
    dw_a = np.ndarray((1,), np.uint16, buffer=dshm.buf, offset=8)
    dh_a = np.ndarray((1,), np.uint16, buffer=dshm.buf, offset=10)
    dmap = np.ndarray((DEPTH_RES, DEPTH_RES), np.uint8, buffer=dshm.buf, offset=DEPTH_OFF)
    dw_a[0] = DEPTH_RES; dh_a[0] = DEPTH_RES
    print(f"[depth] frame {ow}x{oh} -> depth {DEPTH_RES}x{DEPTH_RES}", flush=True)

    QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.OFF)   # bundled 2.42 v68 backend
    ctx = QNNContext("midas_depth_w8a8", MODEL,
                     input_data_type=DataType.NATIVE, output_data_type=DataType.NATIVE)
    print("[depth] MiDaS ready on NPU (separate process, no lock)", flush=True)
    print(f"[depth] rate cap: {DEPTH_FPS:g} fps" if MIN_PERIOD else "[depth] rate: unlimited (NPU ceiling)", flush=True)
    last = 0; t_prev = 0.0
    try:
      while not _stop:
        s = int(fseq[0])
        if s == last:
            time.sleep(0.004); continue                    # no new frame
        if MIN_PERIOD:
            wait = MIN_PERIOD - (time.time() - t_prev)
            if wait > 0:
                time.sleep(wait); continue                 # re-read fseq after the wait -> newest frame
        last = s
        img = frame.copy()                                 # snapshot the RGB frame
        if int(fseq[0]) != s:                              # streamer overwrote mid-copy -> skip
            continue
        t_prev = time.time()
        try:
            # resize to 256x256, NCHW uint8 (native input; dequant ~= /255, affine-absorbed downstream)
            small = np.asarray(Image.fromarray(img).resize((DEPTH_RES, DEPTH_RES), Image.BILINEAR))
            x = np.ascontiguousarray(small.transpose(2, 0, 1)[None], dtype=np.uint8)   # (1,3,256,256)
            out = ctx.Inference([x])
        except Exception as e:
            print("[depth] infer error:", e, flush=True); time.sleep(0.2); continue
        d = np.asarray(out[0] if isinstance(out, (list, tuple)) else out, dtype=np.uint8).reshape(DEPTH_RES, DEPTH_RES)
        # seqlock publish (odd while writing, even when done) — mirror of the detection channel.
        sq = int(dseq[0]); dseq[0] = sq + 1
        dmap[:] = d
        dseq[0] = sq + 2
    finally:
        print("[depth] shutting down: releasing NPU context + shm", flush=True)
        try: ctx.release()
        except Exception as e: print("[depth] ctx release:", e, flush=True)
        for s in (fshm, cshm, dshm):
            try: s.close()                                 # detach only (unregistered above)
            except Exception: pass


if __name__ == "__main__":
    main()

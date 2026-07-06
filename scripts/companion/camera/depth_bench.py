"""Standalone MiDaS-V2 w8a8 depth benchmark on the Q6A NPU (isolation test, no detector running)."""
import os, time, numpy as np
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel, DataType

MODEL = os.path.expanduser("~/midas_depth_w8a8.bin")
QNNConfig.Config(Runtime.HTP, LogLevel.WARN, ProfilingLevel.OFF)   # bundled 2.42 v68 backend
# ONNX I/O were both uint8 (native quantized): input image[1,3,256,256] u8 -> depth[1,1,256,256] u8
ctx = QNNContext("midas_depth_w8a8", MODEL,
                 input_data_type=DataType.NATIVE, output_data_type=DataType.NATIVE)
print("[depth] context loaded on HTP", flush=True)

x = (np.random.rand(1, 3, 256, 256) * 255).astype(np.uint8)   # dummy uint8 NCHW frame
# warmup
for _ in range(5):
    out = ctx.Inference([x])
# timed
N = 50
t0 = time.time()
for _ in range(N):
    out = ctx.Inference([x])
dt = (time.time() - t0) / N * 1000.0
o = np.array(out[0]) if isinstance(out, (list, tuple)) else np.array(out)
print(f"[depth] inference: {dt:.2f} ms/frame over {N} iters", flush=True)
print(f"[depth] output size={o.size} (expect 256*256={256*256}), min={o.min()} max={o.max()}", flush=True)
try:
    ctx.release()   # clean HTP teardown — else this bench orphans its fastrpc PD on the cDSP (unreclaimable)
except Exception as e:
    print("[depth] ctx release:", e, flush=True)

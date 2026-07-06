"""P2.3 prerequisite: 3-process NPU coexistence + dmabuf-growth test.
Runs MiDaS depth at ~10 Hz alongside the LIVE detector + q6a-llmd, fires periodic LLM queries to force
3-way NPU/cDSP contention, and samples MemAvailable / dmabuf bytes / temp / depth latency. Self-aborts at
84 C (well under the 88 C park rung) so it never risks the board."""
import os, sys, time, glob, subprocess, numpy as np

DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 120
ABORT_C = 84.0

def max_temp():
    hi = 0
    for p in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            v = int(open(p).read())
            if v > hi: hi = v
        except Exception: pass
    return hi / 1000.0

def mem_avail_mb():
    for ln in open("/proc/meminfo"):
        if ln.startswith("MemAvailable"):
            return int(ln.split()[1]) // 1024

def dmabuf_bytes():
    try:  # bufinfo has non-UTF8 bytes -> read raw + decode ignoring errors
        raw = subprocess.run(["sudo", "cat", "/sys/kernel/debug/dma_buf/bufinfo"],
                             capture_output=True, timeout=8).stdout.decode("utf-8", "ignore")
        for ln in raw.splitlines():
            if "Total" in ln and "bytes" in ln:       # "Total N objects, M bytes"
                return int(ln.split()[1]), int(ln.split(",")[1].strip().split()[0])
    except Exception as e:
        print(f"[coexist] dmabuf read err: {e}", flush=True)
    return None, None

Q6A_LLM = os.path.expanduser("~/.local/bin/q6a-llm")
def fire_llm():
    try:
        t0 = time.time()
        r = subprocess.run([Q6A_LLM, "Reply with one word: ok"], capture_output=True, text=True, timeout=20)
        return time.time() - t0, (r.returncode == 0 and len(r.stdout.strip()) > 0)
    except Exception:
        return None, False

# --- load depth context (3rd NPU/HTP context; detector + LLM already resident) ---
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel, DataType
QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.OFF)
ctx = QNNContext("midas_depth_w8a8", os.path.expanduser("~/midas_depth_w8a8.bin"),
                 input_data_type=DataType.NATIVE, output_data_type=DataType.NATIVE)
x = (np.random.rand(1, 3, 256, 256) * 255).astype(np.uint8)
print("[coexist] depth context loaded as 3rd NPU process; detector+LLM live", flush=True)

b0 = dmabuf_bytes(); m0 = mem_avail_mb(); t0 = max_temp()
print(f"[coexist] baseline: mem_avail={m0}MB dmabuf={b0[0]}obj/{b0[1]}B temp={t0:.1f}C", flush=True)

lat = []; llm_lat = []; llm_ok = 0; llm_n = 0; errs = 0
mem_series = [m0]; dmabuf_series = [b0[1]]; temp_hi = t0
start = time.time(); last_sample = 0; last_llm = 0; aborted = False
while time.time() - start < DURATION:
    try:
        ti = time.time(); ctx.Inference([x]); lat.append((time.time() - ti) * 1000)
    except Exception as e:
        errs += 1; print(f"[coexist] depth infer error: {e}", flush=True)
    now = time.time()
    if now - last_sample >= 5:
        last_sample = now
        t = max_temp(); temp_hi = max(temp_hi, t)
        m = mem_avail_mb(); b = dmabuf_bytes()
        mem_series.append(m); dmabuf_series.append(b[1] if b[1] else dmabuf_series[-1])
        el = now - start
        print(f"[coexist] t+{el:4.0f}s mem_avail={m}MB dmabuf={b[1]}B temp={t:.1f}C depth_med={np.median(lat[-50:]):.1f}ms", flush=True)
        if t >= ABORT_C:
            print(f"[coexist] ABORT: {t:.1f}C >= {ABORT_C}C — stopping before the park rung", flush=True)
            aborted = True; break
    if now - last_llm >= 15:                 # force LLM NPU decode -> real 3-way contention
        last_llm = now
        dl, ok = fire_llm(); llm_n += 1; llm_ok += int(ok)
        if dl: llm_lat.append(dl)
        print(f"[coexist] LLM query #{llm_n}: {'ok' if ok else 'FAIL'} {dl:.2f}s" if dl else f"[coexist] LLM query #{llm_n}: FAIL", flush=True)
    time.sleep(max(0, 0.1 - (time.time() - now)))    # ~10 Hz depth cadence

print("\n===== COEXISTENCE SUMMARY =====", flush=True)
print(f"duration={time.time()-start:.0f}s aborted={aborted} depth_infers={len(lat)} depth_errs={errs}", flush=True)
if lat: print(f"depth latency: median={np.median(lat):.1f}ms p95={np.percentile(lat,95):.1f}ms (isolation was 5.28ms)", flush=True)
print(f"LLM queries: {llm_ok}/{llm_n} ok" + (f", latency med={np.median(llm_lat):.1f}s" if llm_lat else ""), flush=True)
print(f"mem_avail: start={mem_series[0]}MB end={mem_series[-1]}MB min={min(mem_series)}MB delta={mem_series[-1]-mem_series[0]}MB", flush=True)
if all(dmabuf_series): print(f"dmabuf: start={dmabuf_series[0]}B end={dmabuf_series[-1]}B delta={dmabuf_series[-1]-dmabuf_series[0]}B", flush=True)
print(f"temp: baseline={t0:.1f}C peak={temp_hi:.1f}C", flush=True)

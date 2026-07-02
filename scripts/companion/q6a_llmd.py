#!/usr/bin/env python3
"""q6a_llmd.py - resident Qualcomm Genie daemon for the Radxa Dragon Q6A.

Loads Llama 3.2 1B onto the Hexagon NPU ONCE (paying the ~1.2s graph-init a
single time at startup), then answers prompts over a Unix socket with token
streaming. Eliminates the per-call model reload that made genie-t2t-run ~2.8s.

Talks to the bundled libGenie.so (1.13.0) via ctypes -- no compiler, no
version mismatch. Run with cwd = model dir and LD_LIBRARY_PATH/ADSP_LIBRARY_PATH
pointing at it (the systemd unit sets these).
"""
import ctypes, os, sys, socket, threading, signal

MODEL_DIR = os.environ.get("Q6A_LLM_DIR", os.path.expanduser("~/llama-1b"))
CONFIG    = os.environ.get("Q6A_LLM_CONFIG", "htp-model-config-llama32-1b-gqa.json")
SOCK_PATH = os.environ.get("Q6A_LLM_SOCK", "/tmp/q6a-llm.sock")

os.chdir(MODEL_DIR)                                    # relative paths in the config
os.environ.setdefault("ADSP_LIBRARY_PATH", MODEL_DIR)  # push V68 skel to the cDSP

lib = ctypes.CDLL(os.path.join(MODEL_DIR, "libGenie.so"), mode=ctypes.RTLD_GLOBAL)

Handle = ctypes.c_void_p
SUCCESS = 0
SENTENCE_COMPLETE = 0

lib.GenieDialogConfig_createFromJson.argtypes = [ctypes.c_char_p, ctypes.POINTER(Handle)]
lib.GenieDialogConfig_createFromJson.restype  = ctypes.c_int
lib.GenieDialog_create.argtypes = [Handle, ctypes.POINTER(Handle)]
lib.GenieDialog_create.restype  = ctypes.c_int
lib.GenieDialog_reset.argtypes  = [Handle]
lib.GenieDialog_reset.restype   = ctypes.c_int
# void cb(const char* response, int sentenceCode, const void* userData)
CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
lib.GenieDialog_query.argtypes = [Handle, ctypes.c_char_p, ctypes.c_int, CALLBACK, ctypes.c_void_p]
lib.GenieDialog_query.restype  = ctypes.c_int


def _die(msg, st=None):
    sys.stderr.write(f"[q6a-llmd] {msg}" + (f" (status={st})" if st is not None else "") + "\n")
    sys.exit(1)


# ---- load the model ONCE ----
with open(CONFIG, "rb") as f:
    cfg_json = f.read()
cfg = Handle()
st = lib.GenieDialogConfig_createFromJson(cfg_json, ctypes.byref(cfg))
if st != SUCCESS:                                     # some builds accept a path instead
    st = lib.GenieDialogConfig_createFromJson(CONFIG.encode(), ctypes.byref(cfg))
    if st != SUCCESS:
        _die("GenieDialogConfig_createFromJson failed", st)
dlg = Handle()
st = lib.GenieDialog_create(cfg, ctypes.byref(dlg))
if st != SUCCESS:
    _die("GenieDialog_create failed", st)
sys.stderr.write("[q6a-llmd] model loaded on NPU; ready\n")
sys.stderr.flush()

TEMPLATE = ("<|begin_of_text|><|start_header_id|>user<|end_header_id|>\n\n"
            "{msg}<|eot_id|><|start_header_id|>assistant<|end_header_id|>\n\n")

npu_lock = threading.Lock()   # the dialog/NPU is single: serialize queries


def handle(conn):
    try:
        conn.settimeout(120)
        data = b""
        while True:                                   # read until the client half-closes
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        if not data:
            return
        if data.startswith(b"\x01RAW\x01"):           # agent harness: already-formatted prompt
            prompt = data[len(b"\x01RAW\x01"):]
        else:                                         # plain client: wrap one user turn
            msg = data.decode("utf-8", "replace").rstrip("\n")
            if not msg:
                return
            prompt = TEMPLATE.format(msg=msg).encode("utf-8")

        def cb(resp, code, ud):
            if resp:
                try:
                    conn.sendall(resp)           # stream raw bytes as tokens arrive
                except OSError:
                    pass
        c_cb = CALLBACK(cb)
        with npu_lock:
            lib.GenieDialog_reset(dlg)           # stateless per request
            st = lib.GenieDialog_query(dlg, prompt, SENTENCE_COMPLETE, c_cb, None)
        if st != SUCCESS:
            conn.sendall(f"\n[q6a-llmd: query status {st}]\n".encode())
    except Exception as e:                       # noqa: BLE001 - report to client
        try:
            conn.sendall(f"\n[q6a-llmd error: {e}]\n".encode())
        except OSError:
            pass
    finally:
        try:
            conn.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        conn.close()


if os.path.exists(SOCK_PATH):
    os.unlink(SOCK_PATH)
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(SOCK_PATH)
os.chmod(SOCK_PATH, 0o660)
srv.listen(8)


def _shutdown(*_a):
    try:
        os.unlink(SOCK_PATH)
    except OSError:
        pass
    os._exit(0)


signal.signal(signal.SIGTERM, _shutdown)
signal.signal(signal.SIGINT, _shutdown)

while True:
    c, _ = srv.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()

import ctypes, os, time, json
MD="/home/radxa/llama-1b"; os.chdir(MD); os.environ["ADSP_LIBRARY_PATH"]=MD
g=ctypes.CDLL(os.path.join(MD,"libGenie.so"), mode=ctypes.RTLD_GLOBAL)
H=ctypes.c_void_p
g.GenieDialogConfig_createFromJson.argtypes=[ctypes.c_char_p,ctypes.POINTER(H)]; g.GenieDialogConfig_createFromJson.restype=ctypes.c_int
g.GenieDialog_create.argtypes=[H,ctypes.POINTER(H)]; g.GenieDialog_create.restype=ctypes.c_int
g.GenieDialog_setPerformancePolicy.argtypes=[H,ctypes.c_int]; g.GenieDialog_setPerformancePolicy.restype=ctypes.c_int
CB=ctypes.CFUNCTYPE(None,ctypes.c_char_p,ctypes.c_int,ctypes.c_void_p)
g.GenieDialog_query.argtypes=[H,ctypes.c_char_p,ctypes.c_int,CB,ctypes.c_void_p]; g.GenieDialog_query.restype=ctypes.c_int
g.GenieDialog_reset.argtypes=[H]; g.GenieDialog_reset.restype=ctypes.c_int
cfg=json.load(open("htp-model-config-llama32-1b-gqa.json")); cfg["dialog"]["engine"]["backend"]["QnnHtp"]["poll"]=False
open("/tmp/pf.json","w").write(json.dumps(cfg))
c=H(); assert g.GenieDialogConfig_createFromJson(open("/tmp/pf.json","rb").read(),ctypes.byref(c))==0
d=H(); assert g.GenieDialog_create(c,ctypes.byref(d))==0
P="<|begin_of_text|><|start_header_id|>user<|end_header_id|>\n\nWrite one detailed paragraph about robots.<|eot_id|><|start_header_id|>assistant<|end_header_id|>\n\n"
def bench(label):
    n=[0]
    def cb(r,code,u):
        if r: n[0]+=1
    cbc=CB(cb); g.GenieDialog_reset(d)
    t=time.time(); g.GenieDialog_query(d,P.encode(),0,cbc,None); dt=time.time()-t
    print(f"{label}: ~{n[0]} chunks in {dt:.1f}s = {n[0]/dt:.1f} chunks/s")
bench("poll:false default policy")
for name,val in [("SUSTAINED_HIGH_PERF",20),("BURST",10),("HIGH_PERFORMANCE",30)]:
    rc=g.GenieDialog_setPerformancePolicy(d,val); print(f"setPerformancePolicy({name})=rc{rc}")
    bench(f"poll:false + {name}")

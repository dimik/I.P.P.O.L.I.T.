import os, math, numpy as np, qai_appbuilder
from tokenizers import Tokenizer
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel
MD="/home/radxa/llama-1b"; os.environ.setdefault("ADSP_LIBRARY_PATH",os.path.join(os.path.dirname(qai_appbuilder.__file__),"libs"))
HEAD=64; PAST=3968; CHUNK=128; CTX=4096; NL=16; NKV=8
IN=['input_ids']+sum([[f'past_key_{i}_in',f'past_value_{i}_in'] for i in [11,1,12,13,14,15,0,2,3,4,5,6,7,8,9,10]],[])+['position_ids_cos','position_ids_sin','attention_mask']
tok=Tokenizer.from_file(MD+"/tokenizer.json")
ids=tok.encode("The capital of France is", add_special_tokens=False).ids
ids=[128000]+ids            # begin_of_text
n=len(ids)
# llama3 rope inv_freq
dim=HEAD; inv=1.0/(500000.0**(np.arange(0,dim,2)/dim)); factor,lo,hi,old=32.0,1.0,4.0,8192.0
lw,hw=old/lo,old/hi; nf=[]
for f in inv:
    wl=2*math.pi/f
    nf.append(f if wl<hw else (f/factor if wl>lw else ((1-((old/wl-lo)/(hi-lo)))*f/factor+((old/wl-lo)/(hi-lo))*f)))
inv=np.array(nf,np.float32)                       # [32]
pos=np.arange(CHUNK,dtype=np.float32)[:,None]*inv[None,:]
cos=np.cos(pos).reshape(1,1,CHUNK,32).astype(np.float32); sin=np.sin(pos).reshape(1,1,CHUNK,32).astype(np.float32)
input_ids=np.zeros((1,CHUNK),np.int32); input_ids[0,:n]=ids
# causal mask [1,1,128,4096]: cols 0..3967 past(empty->mask), 3968..4095 current causal
mask=np.full((1,1,CHUNK,CTX),-100.0,np.float32)
for i in range(CHUNK):
    for j in range(i+1): mask[0,0,i,PAST+j]=0.0
zk=np.zeros((NKV,1,HEAD,PAST),np.float32); zv=np.zeros((NKV,1,PAST,HEAD),np.float32)
arrs={'input_ids':input_ids,'position_ids_cos':cos,'position_ids_sin':sin,'attention_mask':mask}
for i in range(NL): arrs[f'past_key_{i}_in']=zk; arrs[f'past_value_{i}_in']=zv
QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.BASIC)
m=QNNContext("llama",MD+"/models/weight_sharing_model_1_of_1.serialized.bin")
out=m.Inference([arrs[k] for k in IN], "burst", 0, "float", "float")   # graphIndex 0 = prefill
logits=np.array(out[-1]).reshape(1,CHUNK,-1)      # last output = logits
nxt=int(logits[0,n-1].argmax())
print("prompt:", repr(tok.decode(ids)))
print("next-token id:", nxt, "->", repr(tok.decode([nxt])))
top5=logits[0,n-1].argsort()[-5:][::-1]
print("top5:", [(int(t), repr(tok.decode([int(t)]))) for t in top5])

import os, math, numpy as np, qai_appbuilder
from tokenizers import Tokenizer
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel
MD="/home/radxa/llama-1b"; os.environ.setdefault("ADSP_LIBRARY_PATH",os.path.join(os.path.dirname(qai_appbuilder.__file__),"libs"))
HEAD=64; PPRE=3968; CHUNK=128; CTX=4096; NL=16; NKV=8
LORD=[11,1,12,13,14,15,0,2,3,4,5,6,7,8,9,10]
IN=['input_ids']+sum([[f'past_key_{i}_in',f'past_value_{i}_in'] for i in LORD],[])+['position_ids_cos','position_ids_sin','attention_mask']
OUT=sum([[f'past_value_{i}_out',f'past_key_{i}_out'] for i in range(NL)],[])+['logits']
CS=0.007843137718737125  # cos/sin scale (offset -127)
def qcs(x): return np.clip(np.round(x/CS+127),0,255).astype(np.uint8)
tok=Tokenizer.from_file(MD+"/tokenizer.json")
dim=HEAD; inv=1.0/(500000.0**(np.arange(0,dim,2)/dim)); fa,lo,hi,old=32.0,1.0,4.0,8192.0; lw,hw=old/lo,old/hi
INV=np.array([f if 2*math.pi/f<hw else (f/fa if 2*math.pi/f>lw else (1-((old/(2*math.pi/f)-lo)/(hi-lo)))*f/fa+((old/(2*math.pi/f)-lo)/(hi-lo))*f) for f in inv],np.float32)
ids=[128000]+tok.encode("The capital of France is",add_special_tokens=False).ids; n=len(ids)
a={'input_ids':np.zeros((1,CHUNK),np.int32)}; a['input_ids'][0,:n]=ids
pp=np.arange(CHUNK,np.float32:=np.float32)[:,None]*INV[None,:]
a['position_ids_cos']=qcs(np.cos(pp)).reshape(1,1,CHUNK,32); a['position_ids_sin']=qcs(np.sin(pp)).reshape(1,1,CHUNK,32)
mk=np.zeros((1,1,CHUNK,CTX),np.uint8)   # 0 = masked; 255 = attend
for i in range(CHUNK):
    for j in range(i+1): mk[0,0,i,PPRE+j]=255
a['attention_mask']=mk
for i in range(NL): a[f'past_key_{i}_in']=np.zeros((NKV,1,HEAD,PPRE),np.uint8); a[f'past_value_{i}_in']=np.zeros((NKV,1,PPRE,HEAD),np.uint8)
QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.BASIC)
m=QNNContext("llama",MD+"/models/weight_sharing_model_1_of_1.serialized.bin")
o=m.Inference([a[k] for k in IN],"burst",0,"native","native")
out={k:np.array(v) for k,v in zip(OUT,o)}
lg=out['logits'].reshape(CHUNK,-1)
nxt=int(lg[n-1].argmax()); print("NATIVE next-token:",repr(tok.decode([nxt])), "| logits dtype:", out['logits'].dtype, "shape:", out['logits'].shape)

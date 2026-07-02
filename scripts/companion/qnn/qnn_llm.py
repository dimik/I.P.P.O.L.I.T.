import os, math, time, numpy as np, qai_appbuilder
from tokenizers import Tokenizer
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel
MD="/home/radxa/llama-1b"; os.environ.setdefault("ADSP_LIBRARY_PATH",os.path.join(os.path.dirname(qai_appbuilder.__file__),"libs"))
HEAD=64; PPRE=3968; PDEC=4095; CHUNK=128; CTX=4096; NL=16; NKV=8; EOS=128009
LORDER=[11,1,12,13,14,15,0,2,3,4,5,6,7,8,9,10]
IN=['input_ids']+sum([[f'past_key_{i}_in',f'past_value_{i}_in'] for i in LORDER],[])+['position_ids_cos','position_ids_sin','attention_mask']
OUT=sum([[f'past_value_{i}_out',f'past_key_{i}_out'] for i in range(NL)],[])+['logits']
tok=Tokenizer.from_file(MD+"/tokenizer.json")
dim=HEAD; inv=1.0/(500000.0**(np.arange(0,dim,2)/dim)); factor,lo,hi,old=32.0,1.0,4.0,8192.0
lw,hw=old/lo,old/hi; nf=[]
for f in inv:
    wl=2*math.pi/f; nf.append(f if wl<hw else (f/factor if wl>lw else ((1-((old/wl-lo)/(hi-lo)))*f/factor+((old/wl-lo)/(hi-lo))*f)))
INV=np.array(nf,np.float32)
def cs(positions):
    p=np.array(positions,np.float32)[:,None]*INV[None,:]
    return np.cos(p).astype(np.float32), np.sin(p).astype(np.float32)
QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.BASIC)
m=QNNContext("llama",MD+"/models/weight_sharing_model_1_of_1.serialized.bin")
def run(gi, arrs):
    o=m.Inference([arrs[k] for k in IN], "burst", gi)
    return {k:np.array(v) for k,v in zip(OUT,o)}
def generate(prompt, maxtok=40):
    ids=[128000]+tok.encode(prompt, add_special_tokens=False).ids; n=len(ids)
    assert n<=CHUNK
    # ---- prefill (graph 0) ----
    a={'input_ids':np.zeros((1,CHUNK),np.int32)}; a['input_ids'][0,:n]=ids
    c,s=cs(range(CHUNK)); a['position_ids_cos']=c.reshape(1,1,CHUNK,32); a['position_ids_sin']=s.reshape(1,1,CHUNK,32)
    mk=np.full((1,1,CHUNK,CTX),-100.0,np.float32)
    for i in range(CHUNK):
        for j in range(i+1): mk[0,0,i,PPRE+j]=0.0
    a['attention_mask']=mk
    for i in range(NL): a[f'past_key_{i}_in']=np.zeros((NKV,1,HEAD,PPRE),np.float32); a[f'past_value_{i}_in']=np.zeros((NKV,1,PPRE,HEAD),np.float32)
    out=run(0,a)
    # decode KV buffers (4095), fill slots 0..n-1 from prefill out
    kbuf={i:np.zeros((NKV,1,HEAD,PDEC),np.float32) for i in range(NL)}
    vbuf={i:np.zeros((NKV,1,PDEC,HEAD),np.float32) for i in range(NL)}
    for i in range(NL):
        kbuf[i][:,:,:,:n]=out[f'past_key_{i}_out'][:,:,:,:n]
        vbuf[i][:,:,:n,:]=out[f'past_value_{i}_out'][:,:,:n,:]
    nxt=int(out['logits'].reshape(1,CHUNK,-1)[0,n-1].argmax())
    gen=[nxt]; p=n
    # ---- decode loop (graph 1) ----
    for _ in range(maxtok):
        if nxt==EOS: break
        a={'input_ids':np.array([[nxt]],np.int32)}
        c,s=cs([p]); a['position_ids_cos']=c.reshape(1,1,1,32); a['position_ids_sin']=s.reshape(1,1,1,32)
        mk=np.full((1,1,1,CTX),-100.0,np.float32); mk[0,0,0,:p]=0.0; mk[0,0,0,PDEC]=0.0  # keys 0..p-1 + current(col4095)
        a['attention_mask']=mk
        for i in range(NL): a[f"past_key_{i}_in"]=kbuf[i].copy(); a[f"past_value_{i}_in"]=vbuf[i].copy()
        out=run(1,a)
        for i in range(NL):
            kbuf[i][:,:,:,p]=out[f'past_key_{i}_out'][:,:,:,0]; vbuf[i][:,:,p,:]=out[f'past_value_{i}_out'][:,:,0,:]
        nxt=int(out['logits'].reshape(1,-1).argmax()); p+=1
        if nxt!=EOS: gen.append(nxt)
    return tok.decode(gen)
if __name__=="__main__":
    import sys
    q=sys.argv[1] if len(sys.argv)>1 else "The capital of France is"
    t=time.time(); txt=generate(q); dt=time.time()-t
    print("PROMPT:", q); print("OUTPUT:", txt); print(f"[{dt:.1f}s]")

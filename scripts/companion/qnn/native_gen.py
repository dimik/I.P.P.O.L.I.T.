import os, math, time, sys, numpy as np, qai_appbuilder
from tokenizers import Tokenizer
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel
MD="/home/radxa/llama-1b"; LIBS=os.path.join(os.path.dirname(qai_appbuilder.__file__),"libs"); os.environ.setdefault("ADSP_LIBRARY_PATH",LIBS)
HEAD=64; PPRE=3968; PDEC=4095; CHUNK=128; CTX=4096; NL=16; NKV=8; EOS=128009
LORD=[11,1,12,13,14,15,0,2,3,4,5,6,7,8,9,10]
IN=['input_ids']+sum([[f'past_key_{i}_in',f'past_value_{i}_in'] for i in LORD],[])+['position_ids_cos','position_ids_sin','attention_mask']
OUT=sum([[f'past_value_{i}_out',f'past_key_{i}_out'] for i in range(NL)],[])+['logits']
CS=0.007843137718737125
def qcs(x): return np.clip(np.round(x/CS+127),0,255).astype(np.uint8)
tok=Tokenizer.from_file(MD+"/tokenizer.json")
dim=HEAD; inv=1.0/(500000.0**(np.arange(0,dim,2)/dim)); fa,lo,hi,old=32.0,1.0,4.0,8192.0; lw,hw=old/lo,old/hi
INV=np.array([f if 2*math.pi/f<hw else (f/fa if 2*math.pi/f>lw else (1-((old/(2*math.pi/f)-lo)/(hi-lo)))*f/fa+((old/(2*math.pi/f)-lo)/(hi-lo))*f) for f in inv],np.float32)
def cs(ps): p=np.array(ps,np.float32)[:,None]*INV[None,:]; return qcs(np.cos(p)),qcs(np.sin(p))
QNNConfig.Config(Runtime.HTP, LogLevel.ERROR, ProfilingLevel.BASIC)
m=QNNContext("llama",MD+"/models/weight_sharing_model_1_of_1.serialized.bin",LIBS+"/libQnnHtp.so",LIBS+"/libQnnSystem.so",False,"native","native")
def run(gi,a): return {k:np.array(v) for k,v in zip(OUT, m.Inference([a[k] for k in IN],"burst",gi))}
def generate(prompt, maxtok=40):
    ids=[128000]+tok.encode(prompt,add_special_tokens=False).ids; n=len(ids); assert n<=CHUNK
    a={'input_ids':np.zeros((1,CHUNK),np.int32)}; a['input_ids'][0,:n]=ids
    c,s=cs(range(CHUNK)); a['position_ids_cos']=c.reshape(1,1,CHUNK,32); a['position_ids_sin']=s.reshape(1,1,CHUNK,32)
    mk=np.zeros((1,1,CHUNK,CTX),np.uint8)
    for i in range(CHUNK):
        for j in range(i+1): mk[0,0,i,PPRE+j]=255
    a['attention_mask']=mk
    for i in range(NL): a[f'past_key_{i}_in']=np.zeros((NKV,1,HEAD,PPRE),np.uint8); a[f'past_value_{i}_in']=np.zeros((NKV,1,PPRE,HEAD),np.uint8)
    out=run(0,a)
    kb={i:np.zeros((NKV,1,HEAD,PDEC),np.uint8) for i in range(NL)}; vb={i:np.zeros((NKV,1,PDEC,HEAD),np.uint8) for i in range(NL)}
    for i in range(NL): kb[i][:,:,:,:n]=out[f'past_key_{i}_out'][:,:,:,:n]; vb[i][:,:,:n,:]=out[f'past_value_{i}_out'][:,:,:n,:]
    nxt=int(out['logits'].reshape(CHUNK,-1)[n-1].argmax()); gen=[nxt]; p=n
    for _ in range(maxtok):
        if nxt==EOS: break
        a={'input_ids':np.array([[nxt]],np.int32)}
        c,s=cs([p]); a['position_ids_cos']=c.reshape(1,1,1,32); a['position_ids_sin']=s.reshape(1,1,1,32)
        mk=np.zeros((1,1,1,CTX),np.uint8); mk[0,0,0,:p]=255; mk[0,0,0,PDEC]=255; a['attention_mask']=mk
        for i in range(NL): a[f'past_key_{i}_in']=kb[i]; a[f'past_value_{i}_in']=vb[i]
        out=run(1,a)
        if 'logits' not in out: print("DECODE FAILED at token",len(gen)); break
        for i in range(NL): kb[i][:,:,:,p]=out[f'past_key_{i}_out'][:,:,:,0]; vb[i][:,:,p,:]=out[f'past_value_{i}_out'][:,:,0,:]
        nxt=int(out['logits'].reshape(-1).argmax()); p+=1
        if nxt!=EOS: gen.append(nxt)
    return gen
if __name__=="__main__":
    q=sys.argv[1] if len(sys.argv)>1 else "The capital of France is"
    t=time.time(); g=generate(q, 40); dt=time.time()-t
    print("PROMPT:",q); print("OUTPUT:",repr(tok.decode(g))); print(f"[{len(g)} tokens in {dt:.1f}s = {len(g)/dt:.1f} tok/s]")

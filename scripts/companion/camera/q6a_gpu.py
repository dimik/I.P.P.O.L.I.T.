"""GPU (Adreno OpenCL) Bayer demosaic for the Q6A.

The mainline camss ISP can't demosaic on this Titan SoC (RDI-only — see docs/q6a-camera.md), so the
demosaic runs in userspace. In pure numpy it costs ~290 ms/frame; this offloads it to the Adreno 635 via
OpenCL. One kernel does black-level subtract + raw white balance + full-res bilinear BGGR demosaic and
returns float32 RGB (the CPU still does the cheap destripe + tone-map, which need global stats).

Needs pyopencl + the Adreno driver registered as an ICD:
    echo /usr/lib/aarch64-linux-gnu/libOpenCL_adreno.so.1 | sudo tee /etc/OpenCL/vendors/adreno.icd
Import is lazy in the streamer, so if OpenCL is unavailable the pipeline falls back to numpy.
"""
import os
os.environ.setdefault("PYOPENCL_COMPILER_OUTPUT", "0")
import numpy as np
import pyopencl as cl

_SRC = r'''
inline float bget(__global const ushort* b, int x, int y, int W, int H, float bl){
    x = clamp(x, 0, W-1); y = clamp(y, 0, H-1);
    float v = (float)b[y*W + x] - bl;
    return v < 0.0f ? 0.0f : v;
}
// read a pixel straight from the packed MIPI RAW10 buffer (pBAA: 4 px in 5 bytes, stride S) -> unpack on GPU
inline float bget_packed(__global const uchar* p, int x, int y, int W, int H, int S, float bl){
    x = clamp(x, 0, W-1); y = clamp(y, 0, H-1);
    int g = x >> 2, wi = x & 3, base = y*S + g*5;
    int val = ((int)p[base + wi] << 2) | ((p[base + 4] >> (2*wi)) & 3);
    float v = (float)val - bl;
    return v < 0.0f ? 0.0f : v;
}
// BGGR bilinear demosaic + black level + raw white balance -> interleaved float RGB
__kernel void demosaic(__global const ushort* bayer, __global float* rgb,
                       const int W, const int H, const float bl,
                       const float wr, const float wg, const float wb){
    int x = get_global_id(0), y = get_global_id(1);
    if (x >= W || y >= H) return;
    int bx = x & 1, by = y & 1;
    float C = bget(bayer, x, y, W, H, bl), R, G, B;
    float g4 = 0.25f*(bget(bayer,x-1,y,W,H,bl)+bget(bayer,x+1,y,W,H,bl)
                     +bget(bayer,x,y-1,W,H,bl)+bget(bayer,x,y+1,W,H,bl));
    float d4 = 0.25f*(bget(bayer,x-1,y-1,W,H,bl)+bget(bayer,x+1,y-1,W,H,bl)
                     +bget(bayer,x-1,y+1,W,H,bl)+bget(bayer,x+1,y+1,W,H,bl));
    float hh = 0.5f*(bget(bayer,x-1,y,W,H,bl)+bget(bayer,x+1,y,W,H,bl));
    float vv = 0.5f*(bget(bayer,x,y-1,W,H,bl)+bget(bayer,x,y+1,W,H,bl));
    if (by==0 && bx==0){ B=C; G=g4; R=d4; }              // B site
    else if (by==1 && bx==1){ R=C; G=g4; B=d4; }         // R site
    else if (by==0 && bx==1){ G=C; B=hh; R=vv; }         // G on blue row
    else { G=C; R=hh; B=vv; }                            // G on red row
    int o = (y*W + x)*3;
    rgb[o]=R*wr; rgb[o+1]=G*wg; rgb[o+2]=B*wb;
}
// full ISP from PACKED RAW10: unpack + demosaic + WB + optional shading + tone map -> uint8 RGB
__kernel void isp(__global const uchar* pk, __global const float* shade, __global uchar* out,
                  const int W, const int H, const int S, const float bl,
                  const float wr, const float wg, const float wb,
                  const float scale, const int use_shade){
    int x = get_global_id(0), y = get_global_id(1);
    if (x >= W || y >= H) return;
    int bx = x & 1, by = y & 1;
    float C = bget_packed(pk, x, y, W, H, S, bl), R, G, B;
    float g4 = 0.25f*(bget_packed(pk,x-1,y,W,H,S,bl)+bget_packed(pk,x+1,y,W,H,S,bl)
                     +bget_packed(pk,x,y-1,W,H,S,bl)+bget_packed(pk,x,y+1,W,H,S,bl));
    float d4 = 0.25f*(bget_packed(pk,x-1,y-1,W,H,S,bl)+bget_packed(pk,x+1,y-1,W,H,S,bl)
                     +bget_packed(pk,x-1,y+1,W,H,S,bl)+bget_packed(pk,x+1,y+1,W,H,S,bl));
    float hh = 0.5f*(bget_packed(pk,x-1,y,W,H,S,bl)+bget_packed(pk,x+1,y,W,H,S,bl));
    float vv = 0.5f*(bget_packed(pk,x,y-1,W,H,S,bl)+bget_packed(pk,x,y+1,W,H,S,bl));
    if (by==0 && bx==0){ B=C; G=g4; R=d4; }
    else if (by==1 && bx==1){ R=C; G=g4; B=d4; }
    else if (by==0 && bx==1){ G=C; B=hh; R=vv; }
    else { G=C; R=hh; B=vv; }
    R*=wr; G*=wg; B*=wb;
    int o = (y*W + x)*3;
    if (use_shade){ R*=shade[o]; G*=shade[o+1]; B*=shade[o+2]; }
    R = 255.0f*native_powr(clamp(R*scale/255.0f, 0.0f, 1.0f), 0.7f);   // tone map: scale to target + gamma
    G = 255.0f*native_powr(clamp(G*scale/255.0f, 0.0f, 1.0f), 0.7f);
    B = 255.0f*native_powr(clamp(B*scale/255.0f, 0.0f, 1.0f), 0.7f);
    out[o]=(uchar)(R+0.5f); out[o+1]=(uchar)(G+0.5f); out[o+2]=(uchar)(B+0.5f);
}
// 2x2 BINNED ISP: one output pixel per Bayer quad (uses real photosites, averages the 2 greens ->
// ~2x less noise, no demosaic interpolation) + WB + tone map. Output is half-res (OW=W/2, OH=H/2).
__kernel void isp_bin(__global const uchar* pk, __global const float* shade, __global uchar* out,
                      const int W, const int H, const int OW, const int OH, const int S, const float bl,
                      const float wr, const float wg, const float wb, const float scale, const int use_shade,
                      __global const float* dcol, __global const float* drow, const int use_ds){
    int ox = get_global_id(0), oy = get_global_id(1);
    if (ox >= OW || oy >= OH) return;
    int x = ox*2, y = oy*2;
    float B = bget_packed(pk, x,   y,   W, H, S, bl);              // BGGR quad (unpacked on GPU)
    float G = 0.5f*(bget_packed(pk, x+1, y, W, H, S, bl) + bget_packed(pk, x, y+1, W, H, S, bl));
    float R = bget_packed(pk, x+1, y+1, W, H, S, bl);
    R*=wr; G*=wg; B*=wb;
    if (use_shade){ int so=(y*W + x)*3; R*=shade[so]; G*=shade[so+1]; B*=shade[so+2]; }  // radial color-shading
    int o = (oy*OW + ox)*3;
    float ro=255.0f*native_powr(clamp(R*scale/255.0f,0.0f,1.0f),0.7f);
    float go=255.0f*native_powr(clamp(G*scale/255.0f,0.0f,1.0f),0.7f);
    float bo=255.0f*native_powr(clamp(B*scale/255.0f,0.0f,1.0f),0.7f);
    if (use_ds){                                                   // FPN destripe fused inline (no 2nd pass)
        ro -= dcol[ox*3]  +drow[oy*3];
        go -= dcol[ox*3+1]+drow[oy*3+1];
        bo -= dcol[ox*3+2]+drow[oy*3+2];
    }
    out[o]  =(uchar)clamp(ro+0.5f,0.0f,255.0f);
    out[o+1]=(uchar)clamp(go+0.5f,0.0f,255.0f);
    out[o+2]=(uchar)clamp(bo+0.5f,0.0f,255.0f);
}
// destripe (FPN) on the uint8 image, all on GPU: column/row sums -> (CPU smooths) -> subtract
__kernel void col_sum(__global const uchar* im, __global int* cs, const int OW, const int OH){
    int x = get_global_id(0), c = get_global_id(1);
    if (x >= OW || c >= 3) return;
    int s = 0;
    for (int y = 0; y < OH; y++) s += im[(y*OW + x)*3 + c];
    cs[x*3 + c] = s;
}
__kernel void row_sum(__global const uchar* im, __global int* rs, const int OW, const int OH){
    int y = get_global_id(0), c = get_global_id(1);
    if (y >= OH || c >= 3) return;
    int s = 0;
    for (int x = 0; x < OW; x++) s += im[(y*OW + x)*3 + c];
    rs[y*3 + c] = s;
}
// correction = (per-line mean) - (box-smoothed per-line mean), computed fully on GPU (no CPU round-trip)
__kernel void corr(__global const int* sum, __global float* out, const int N, const int DIV, const int win){
    int i = get_global_id(0), c = get_global_id(1);
    if (i >= N || c >= 3) return;
    int h = win/2; float s = 0.0f;
    for (int k = -h; k <= h; k++){ int ii = clamp(i+k, 0, N-1); s += (float)sum[ii*3 + c]; }
    out[i*3 + c] = ((float)sum[i*3 + c] - s/(float)(2*h+1)) / (float)DIV;   // high-freq part / DIV
}
__kernel void destripe_sub(__global uchar* im, __global const float* dcol, __global const float* drow,
                           const int OW, const int OH){
    int x = get_global_id(0), y = get_global_id(1);
    if (x >= OW || y >= OH) return;
    int o = (y*OW + x)*3;
    for (int c = 0; c < 3; c++){
        float v = (float)im[o+c] - dcol[x*3+c] - drow[y*3+c];
        im[o+c] = (uchar)clamp(v, 0.0f, 255.0f);
    }
}
'''


class GpuDemosaic:
    def __init__(self, W, H):
        self.W, self.H = W, H
        dev = cl.get_platforms()[0].get_devices()[0]
        self.ctx = cl.Context(devices=[dev])
        self.q = cl.CommandQueue(self.ctx)
        self.prg = cl.Program(self.ctx, _SRC).build(options=["-cl-fast-relaxed-math"])
        self.knl = cl.Kernel(self.prg, "demosaic")     # build once, reuse (avoid per-call retrieval)
        self.isp_knl = cl.Kernel(self.prg, "isp")
        self.isp_bin_knl = cl.Kernel(self.prg, "isp_bin")
        self.col_sum_knl = cl.Kernel(self.prg, "col_sum")
        self.row_sum_knl = cl.Kernel(self.prg, "row_sum")
        self.corr_knl = cl.Kernel(self.prg, "corr")
        self.destripe_knl = cl.Kernel(self.prg, "destripe_sub")
        mf = cl.mem_flags
        self.OW, self.OH = W // 2, H // 2
        self.d_in = cl.Buffer(self.ctx, mf.READ_ONLY, W * H * 2)
        self.d_out = cl.Buffer(self.ctx, mf.WRITE_ONLY, W * H * 3 * 4)
        self.d_u8 = cl.Buffer(self.ctx, mf.READ_WRITE, W * H * 3)         # RW: destripe edits in place
        self.d_u8_bin = cl.Buffer(self.ctx, mf.READ_WRITE, self.OW * self.OH * 3)
        self.d_colsum = cl.Buffer(self.ctx, mf.READ_WRITE, W * 3 * 4)
        self.d_rowsum = cl.Buffer(self.ctx, mf.READ_WRITE, H * 3 * 4)
        self.d_dcol = cl.Buffer(self.ctx, mf.READ_ONLY, W * 3 * 4)
        self.d_drow = cl.Buffer(self.ctx, mf.READ_ONLY, H * 3 * 4)
        self.out = np.empty((H, W, 3), np.float32)
        self.u8 = np.empty((H, W, 3), np.uint8)
        self.u8_bin = np.empty((self.OH, self.OW, 3), np.uint8)
        self.d_shade = None
        self.dev_name = dev.name.strip()
        # FPN (fixed-pattern column/row noise) is STATIC per sensor/gain, so the correction changes
        # slowly. Recompute the col/row correction only every `destripe_period` frames (the expensive
        # col_sum+row_sum+corr reductions) and just APPLY the cached correction every frame (cheap
        # destripe_sub). ~3x cheaper destripe with no visible difference. 1 = recompute every frame.
        self.destripe_period = 8
        self._dstr_ctr = 0

    def _apply_destripe(self, d_u8, ow, oh, win=41):
        """FPN destripe entirely on GPU: col/row sums -> corr (mean-smooth) -> subtract. No CPU round-trip.
        The correction (d_dcol/d_drow) is cached and refreshed every destripe_period frames (static FPN)."""
        if self._dstr_ctr % self.destripe_period == 0:
            self.col_sum_knl(self.q, (ow, 3), None, d_u8, self.d_colsum, np.int32(ow), np.int32(oh))
            self.row_sum_knl(self.q, (oh, 3), None, d_u8, self.d_rowsum, np.int32(ow), np.int32(oh))
            self.corr_knl(self.q, (ow, 3), None, self.d_colsum, self.d_dcol, np.int32(ow), np.int32(oh), np.int32(win))
            self.corr_knl(self.q, (oh, 3), None, self.d_rowsum, self.d_drow, np.int32(oh), np.int32(ow), np.int32(win))
        self._dstr_ctr += 1
        self.destripe_knl(self.q, (ow, oh), None, d_u8, self.d_dcol, self.d_drow, np.int32(ow), np.int32(oh))

    def _refresh_destripe(self, d_u8, ow, oh, win=41):
        """Recompute the FPN col/row correction (d_dcol/d_drow) from a RAW (un-destriped) frame.
        Called every destripe_period frames; the correction is then applied inline by the isp_bin kernel."""
        self.col_sum_knl(self.q, (ow, 3), None, d_u8, self.d_colsum, np.int32(ow), np.int32(oh))
        self.row_sum_knl(self.q, (oh, 3), None, d_u8, self.d_rowsum, np.int32(ow), np.int32(oh))
        self.corr_knl(self.q, (ow, 3), None, self.d_colsum, self.d_dcol, np.int32(ow), np.int32(oh), np.int32(win))
        self.corr_knl(self.q, (oh, 3), None, self.d_rowsum, self.d_drow, np.int32(oh), np.int32(ow), np.int32(win))

    def isp_bin(self, packed, stride, bl, wr, wg, wb, scale, destripe=False):
        """2x2 binned ISP from PACKED RAW10 bytes -> (H/2,W/2,3) uint8 (unpack on GPU; lower noise).
        Destripe is FUSED into the kernel: the cached col/row FPN correction is subtracted inline in the
        output write (no separate full-image pass). On every `destripe_period`-th frame the kernel renders
        WITHOUT destripe and the correction is recomputed from that raw frame for the next N frames (a
        static-FPN approximation; the one un-destriped frame per period is imperceptible)."""
        cl.enqueue_copy(self.q, self.d_in, np.frombuffer(packed, np.uint8), is_blocking=False)
        refresh = destripe and (self._dstr_ctr % self.destripe_period == 0)
        use_ds = 1 if (destripe and not refresh) else 0
        use_sh = 1 if self.d_shade is not None else 0
        sh = self.d_shade if use_sh else self.d_in         # dummy (unused) buffer when no shade map
        self.isp_bin_knl(self.q, (self.OW, self.OH), None, self.d_in, sh, self.d_u8_bin,
                         np.int32(self.W), np.int32(self.H), np.int32(self.OW), np.int32(self.OH),
                         np.int32(stride), np.float32(bl),
                         np.float32(wr), np.float32(wg), np.float32(wb), np.float32(scale), np.int32(use_sh),
                         self.d_dcol, self.d_drow, np.int32(use_ds))
        if destripe:
            self._dstr_ctr += 1
            if refresh:
                self._refresh_destripe(self.d_u8_bin, self.OW, self.OH)   # for the next N frames
        cl.enqueue_copy(self.q, self.u8_bin, self.d_u8_bin, is_blocking=False)
        self.q.finish()
        return self.u8_bin

    def set_shade(self, shade):
        """Upload a (H,W,3) float32 color-shading gain map to the GPU once (or None to disable)."""
        if shade is None:
            self.d_shade = None; return
        s = np.ascontiguousarray(shade, np.float32)
        self.d_shade = cl.Buffer(self.ctx, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=s)

    def isp(self, packed, stride, bl, wr, wg, wb, scale, destripe=False):
        """Full ISP on the GPU from PACKED RAW10 bytes -> (H,W,3) uint8 (unpack+demosaic+WB+shade+tonemap)."""
        cl.enqueue_copy(self.q, self.d_in, np.frombuffer(packed, np.uint8), is_blocking=False)
        use = 1 if self.d_shade is not None else 0
        sh = self.d_shade if use else self.d_in           # dummy (unused) buffer when no shade
        self.isp_knl(self.q, (self.W, self.H), None, self.d_in, sh, self.d_u8,
                     np.int32(self.W), np.int32(self.H), np.int32(stride), np.float32(bl),
                     np.float32(wr), np.float32(wg), np.float32(wb),
                     np.float32(scale), np.int32(use))
        if destripe:
            self._apply_destripe(self.d_u8, self.W, self.H)
        cl.enqueue_copy(self.q, self.u8, self.d_u8, is_blocking=False)
        self.q.finish()
        return self.u8

    def demosaic(self, px, bl, wr, wg, wb):
        """px: (H,W) uint16 Bayer -> (H,W,3) float32 RGB (black-level + WB + demosaic on the GPU)."""
        px = np.ascontiguousarray(px, np.uint16)
        cl.enqueue_copy(self.q, self.d_in, px)
        self.knl(self.q, (self.W, self.H), None, self.d_in, self.d_out,
                 np.int32(self.W), np.int32(self.H), np.float32(bl),
                 np.float32(wr), np.float32(wg), np.float32(wb))
        cl.enqueue_copy(self.q, self.out, self.d_out)
        self.q.finish()
        return self.out


if __name__ == "__main__":                 # standalone correctness + timing vs numpy
    import time, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) or ".")
    import q6a_camstream as C
    raw = open("/dev/shm/q6a_cap.raw", "rb").read(C.FRAME) if os.path.exists("/dev/shm/q6a_cap.raw") \
        else (np.random.rand(C.FRAME) * 255).astype("uint8").tobytes()
    px = C.unpack_raw10(raw)
    g = GpuDemosaic(C.W, C.H); print("GPU:", g.dev_name)
    # correctness vs the numpy path (black-level + WB + demosaic)
    pxf = px.astype(np.float32) - C.BLACK_LEVEL; np.clip(pxf, 0, None, out=pxf)
    pxf[1::2, 1::2] *= C.WB_R; pxf[0::2, 0::2] *= C.WB_B
    cpu = C.demosaic_bggr(pxf)
    gpu = g.demosaic(px, C.BLACK_LEVEL, C.WB_R, C.WB_G, C.WB_B)
    print("max abs diff GPU vs numpy:", float(np.abs(cpu - gpu).max()))
    for name, fn in [("GPU demosaic", lambda: g.demosaic(px, C.BLACK_LEVEL, C.WB_R, C.WB_G, C.WB_B))]:
        t = time.time()
        for _ in range(20): fn()
        print("%s: %.1f ms/frame" % (name, (time.time() - t) / 20 * 1000))

/* ============================================================================================
 * cedar_enc.c — one-frame NV21 -> H.264 encode test using the Allwinner CedarX VideoEncoder.
 *
 * Runs INSIDE the robot's Ubuntu chroot (glibc 2.39), linked against the host's vendor encoder
 * libs (libvencoder/libvenc_codec/libawh264/libVE/libMemAdapter, glibc 2.23 — backward-compatible
 * so they load fine under 2.39). The HW video engine is reached via /dev/cedar_dev + /dev/ion,
 * both visible in the chroot. This is a SEPARATE process fed RAM buffers: a wrong struct offset
 * just segfaults this process — it cannot disturb AVA or the camera (no video0/ISP touched).
 *
 * Goal: read /tmp/cam_frame.raw (NV21 672x504, as produced by camsiphon), encode ONE frame,
 * write the H.264 Annex-B bytes to /tmp/test.h264. Validate the .h264 has SPS/PPS/IDR NALs.
 *
 * Build (in chroot):  gcc-13 cedar_enc.c -L/opt/venc -lvencoder -lMemAdapter -lVE -o cedar_enc
 * Run   (in chroot):  LD_LIBRARY_PATH=/opt/venc ./cedar_enc 672 504 /tmp/cam_frame.raw /tmp/test.h264
 *
 * ABI: canonical CedarX layout. VencBaseConfig.memops/veOpsS/pVeOpsSelf confirmed at offsets
 * 32/40/48 by disassembling VideoEncInit. Structs are padded generously so trailing
 * version-drift fields stay zeroed (= library defaults).
 * ============================================================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

typedef enum {
    VENC_CODEC_H264 = 0, VENC_CODEC_JPEG, VENC_CODEC_H264_VBR,
    VENC_CODEC_H265, VENC_CODEC_H265_VBR, VENC_CODEC_JPEG_VBR
} VENC_CODEC_TYPE;

typedef enum {
    VENC_PIXEL_YUV420SP = 0,  /* NV12 (UV) */
    VENC_PIXEL_YVU420SP,      /* NV21 (VU) <- our camera */
    VENC_PIXEL_YUV420P, VENC_PIXEL_YVU420P,
    VENC_PIXEL_YUV422SP, VENC_PIXEL_YVU422SP
} VENC_PIXEL_FMT;

/* SetParameter index ids (canonical order). */
typedef enum {
    VENC_IndexParamBitrate = 0,
    VENC_IndexParamFramerate,
    VENC_IndexParamMaxKeyInterval
} VENC_INDEXTYPE;

typedef struct VencBaseConfig {
    unsigned char  bEncH264Nalu;     /* off 0  */
    unsigned int   nInputWidth;      /* off 4  */
    unsigned int   nInputHeight;     /* off 8  */
    unsigned int   nStride;          /* off 12 */
    unsigned int   nDstWidth;        /* off 16 */
    unsigned int   nDstHeight;       /* off 20 */
    VENC_PIXEL_FMT eInputFormat;     /* off 24 (confirmed: color_format read from here) */
    unsigned int   _reserved28;      /* off 28 */
    void*          memops;           /* off 32 (confirmed) */
    void*          veOpsS;           /* off 40 (confirmed) */
    void*          pVeOpsSelf;       /* off 48 (confirmed) */
    unsigned char  _pad[160];        /* trailing drift fields -> 0 = defaults */
} VencBaseConfig;

typedef struct VencAllocateBufferParam {
    unsigned int nBufferNum;
    unsigned int nSizeY;
    unsigned int nSizeC;
} VencAllocateBufferParam;

typedef struct VencInputBuffer {
    unsigned long  nID;          /* off 0  */
    long long      nPts;         /* off 8  */
    unsigned int   nFlag;        /* off 16 */
    unsigned char* pAddrPhyY;    /* off 24 */
    unsigned char* pAddrPhyC;    /* off 32 */
    unsigned char* pAddrVirY;    /* off 40 */
    unsigned char* pAddrVirC;    /* off 48 */
    unsigned char  _pad[256];    /* crop/roi/dma fields we don't use */
} VencInputBuffer;

typedef struct VencOutputBuffer {
    int            nID;          /* off 0  */
    long long      nPts;         /* off 8  */
    unsigned int   nFlag;        /* off 16 */
    unsigned int   nSize0;       /* off 20 */
    unsigned int   nSize1;       /* off 24 */
    unsigned char* pData0;       /* off 32 */
    unsigned char* pData1;       /* off 40 */
    unsigned char  _pad[64];
} VencOutputBuffer;

extern void* VideoEncCreate(VENC_CODEC_TYPE);
extern void  VideoEncDestroy(void*);
extern int   VideoEncInit(void*, VencBaseConfig*);
extern int   VideoEncUnInit(void*);
extern int   VideoEncSetParameter(void*, VENC_INDEXTYPE, void*);
extern int   VideoEncGetParameter(void*, int, void*);
extern int   AllocInputBuffer(void*, VencAllocateBufferParam*);
extern int   GetOneAllocInputBuffer(void*, VencInputBuffer*);
extern int   FlushCacheAllocInputBuffer(void*, VencInputBuffer*);
extern int   ReturnOneAllocInputBuffer(void*, VencInputBuffer*);
extern int   AddOneInputBuffer(void*, VencInputBuffer*);
extern int   AlreadyUsedInputBuffer(void*, VencInputBuffer*);
extern int   VideoEncodeOneFrame(void*);
extern int   GetOneBitstreamFrame(void*, VencOutputBuffer*);
extern int   FreeOneBitStreamFrame(void*, VencOutputBuffer*);
extern void* MemAdapterGetOpsS(void);
extern void* GetVeOpsS(void);

#define LOG(...) do { fprintf(stderr, "[cedar_enc] " __VA_ARGS__); fprintf(stderr, "\n"); } while (0)

int main(int argc, char** argv) {
    if (argc < 5) { LOG("usage: %s W H in.nv21 out.h264", argv[0]); return 2; }
    int W = atoi(argv[1]), H = atoi(argv[2]);
    const char* inpath = argv[3];
    const char* outpath = argv[4];
    int pixfmt = (argc > 5) ? atoi(argv[5]) : 0;   /* VENC_PIXEL_FMT; 0=NV12 1=NV21 2=YU12 3=YV12 */
    unsigned ysz = (unsigned)W * H, csz = ysz / 2;

    /* load the raw NV21 frame */
    int fd = open(inpath, O_RDONLY);
    if (fd < 0) { LOG("open %s failed", inpath); return 1; }
    unsigned char* raw = malloc(ysz + csz);
    if (read(fd, raw, ysz) != (long)ysz || read(fd, raw + ysz, csz) != (long)csz) {
        LOG("short read (need %u + %u bytes)", ysz, csz); return 1;
    }
    close(fd);
    LOG("loaded %s : Y=%u C=%u (%dx%d NV21)", inpath, ysz, csz, W, H);

    int codec = (argc > 6 && strcmp(argv[6], "probe") != 0) ? atoi(argv[6]) : 0;  /* 0=H264 1=JPEG */
    void* enc = VideoEncCreate((VENC_CODEC_TYPE)codec);
    LOG("VideoEncCreate(codec=%d) -> %p", codec, enc);
    if (!enc) return 1;

    /* configure rate/framerate/keyint (best-effort; ignore if index differs) */
    int bitrate = 2000000, fps = 15, keyint = 30;
    VideoEncSetParameter(enc, VENC_IndexParamFramerate, &fps);
    VideoEncSetParameter(enc, VENC_IndexParamBitrate, &bitrate);
    VideoEncSetParameter(enc, VENC_IndexParamMaxKeyInterval, &keyint);

    VencBaseConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.bEncH264Nalu = 0;   /* 0 = clean Annex-B (00 00 00 01) output */
    cfg.nInputWidth = W;  cfg.nInputHeight = H;  cfg.nStride = W;
    cfg.nDstWidth = W;    cfg.nDstHeight = H;
    cfg.eInputFormat = (VENC_PIXEL_FMT)pixfmt;   /* sweepable: 0=NV12 1=NV21 2=YU12 3=YV12 ... */
    cfg.memops = MemAdapterGetOpsS();            /* veOpsS left NULL — lib inits VE itself */
    LOG("memops=%p eInputFormat=%d", cfg.memops, pixfmt);

    int r = VideoEncInit(enc, &cfg);
    LOG("VideoEncInit -> %d", r);
    if (r != 0) return 1;


    VencAllocateBufferParam bp;
    memset(&bp, 0, sizeof(bp));
    bp.nBufferNum = 4; bp.nSizeY = ysz; bp.nSizeC = csz;
    r = AllocInputBuffer(enc, &bp);
    LOG("AllocInputBuffer(num=%u, Y=%u, C=%u) -> %d", bp.nBufferNum, bp.nSizeY, bp.nSizeC, r);
    if (r != 0) return 1;

    VencInputBuffer ib; memset(&ib, 0, sizeof(ib));
    r = GetOneAllocInputBuffer(enc, &ib);
    LOG("GetOneAllocInputBuffer -> %d  virY=%p virC=%p phyY=%p",
        r, ib.pAddrVirY, ib.pAddrVirC, ib.pAddrPhyY);
    if (r != 0 || !ib.pAddrVirY || !ib.pAddrVirC) return 1;

    memcpy(ib.pAddrVirY, raw, ysz);
    memcpy(ib.pAddrVirC, raw + ysz, csz);
    ib.nPts = 0;
    FlushCacheAllocInputBuffer(enc, &ib);
    r = AddOneInputBuffer(enc, &ib);
    LOG("AddOneInputBuffer -> %d", r);

    r = VideoEncodeOneFrame(enc);
    LOG("VideoEncodeOneFrame -> %d", r);

    AlreadyUsedInputBuffer(enc, &ib);
    ReturnOneAllocInputBuffer(enc, &ib);

    VencOutputBuffer ob; memset(&ob, 0, sizeof(ob));
    r = GetOneBitstreamFrame(enc, &ob);
    LOG("GetOneBitstreamFrame -> %d  size0=%u size1=%u data0=%p data1=%p flag=0x%x",
        r, ob.nSize0, ob.nSize1, ob.pData0, ob.pData1, ob.nFlag);
    if (r != 0) return 1;

    int ofd = open(outpath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    long total = 0;
    if (ob.nSize0 && ob.pData0) { write(ofd, ob.pData0, ob.nSize0); total += ob.nSize0; }
    if (ob.nSize1 && ob.pData1) { write(ofd, ob.pData1, ob.nSize1); total += ob.nSize1; }
    close(ofd);
    LOG("wrote %ld bytes -> %s", total, outpath);

    FreeOneBitStreamFrame(enc, &ob);

    /* probe: find the GetParameter index that returns the SPS/PPS header (now that the
     * encoder has produced an IDR, headers should be available). argv[6]="probe" enables it. */
    if (argc > 6 && strcmp(argv[6], "probe") == 0) {
        /* index 0x501 = SPS/PPS getter. arg = {unsigned int nLength@0; unsigned char* pBuffer@8}.
         * copies 2*nLength/3 SPS bytes then nLength/3 PPS bytes into pBuffer. */
        unsigned int N = (argc > 7) ? (unsigned)atoi(argv[7]) : 384;
        unsigned char* buf = malloc(16384); memset(buf, 0xAA, 16384);
        unsigned char arg[64]; memset(arg, 0, sizeof(arg));
        *(unsigned int*)  (arg + 0) = N;            /* nLength */
        *(unsigned char**)(arg + 8) = buf;          /* pBuffer */
        int gr = VideoEncGetParameter(enc, 0x501, arg);
        unsigned int spsregion = 2 * N / 3;
        LOG("GetParameter(0x501, N=%u) -> %d  (SPS region [0..%u), PPS region [%u..%u))",
            N, gr, spsregion, spsregion, N);
        LOG("  buf[0..23]:   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
            "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
            buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],
            buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19],buf[20],buf[21],buf[22],buf[23]);
        LOG("  buf[%u..+15]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
            spsregion, buf[spsregion+0],buf[spsregion+1],buf[spsregion+2],buf[spsregion+3],
            buf[spsregion+4],buf[spsregion+5],buf[spsregion+6],buf[spsregion+7],buf[spsregion+8],
            buf[spsregion+9],buf[spsregion+10],buf[spsregion+11],buf[spsregion+12],buf[spsregion+13],
            buf[spsregion+14],buf[spsregion+15]);
        /* also: find where the 0xAA padding starts in each region = actual lengths */
        int e0 = 0; while (e0 < (int)spsregion && buf[e0] != 0xAA) e0++;
        LOG("  SPS region: %d non-pad bytes before padding", e0);
        free(buf);
    }

    VideoEncUnInit(enc);
    VideoEncDestroy(enc);
    LOG("done");
    return 0;
}

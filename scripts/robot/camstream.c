/* ============================================================================================
 * camstream.c — cedar HW-JPEG MJPEG-over-HTTP server for the Dreame camera.
 *
 * Runs INSIDE the robot's Ubuntu chroot (glibc 2.39), linked against the vendor CedarX encoder
 * libs (see build_ava_shims/cedar_enc.c for the ABI story). Pipeline:
 *
 *   AVA --(V4L2 DQBUF)--> camsiphon --(RAM ring /tmp/cam_stream.buf)--> camstream --(cedar JPEG)
 *        --> multipart/x-mixed-replace over HTTP :8090
 *
 * camsiphon copies every captured NV21 frame into a double-buffered shared ring (gated by the
 * /tmp/cam_stream flag). We mmap that ring, JPEG-encode the latest frame with the HW video engine,
 * and push it to any connected browser. Read-only w.r.t. AVA; the camera/ISP are never touched.
 *
 * Build (chroot): gcc-13 camstream.c -L/opt/venc -lvencoder -lvenc_codec -lvenc_base \
 *                   -lMemAdapter -lVE -lcdc_base -Wl,-rpath,/opt/venc -o camstream
 * Run   (chroot): LD_LIBRARY_PATH=/opt/venc ./camstream 8090
 * View:           http://<robot-ip>:8090/   (MJPEG; works in any browser / VLC / ffplay)
 * ============================================================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* ---- CedarX VideoEncoder ABI (validated in cedar_enc.c) ---- */
typedef enum { VENC_CODEC_H264 = 0, VENC_CODEC_JPEG } VENC_CODEC_TYPE;
typedef enum { VENC_PIXEL_YUV420SP = 0, VENC_PIXEL_YVU420SP /*NV21*/ } VENC_PIXEL_FMT;
typedef struct {
    unsigned char  bEncH264Nalu;
    unsigned int   nInputWidth, nInputHeight, nStride, nDstWidth, nDstHeight;
    VENC_PIXEL_FMT eInputFormat;   /* off 24 */
    unsigned int   _reserved28;
    void *memops, *veOpsS, *pVeOpsSelf;   /* off 32/40/48 */
    unsigned char  _pad[160];
} VencBaseConfig;
typedef struct { unsigned int nBufferNum, nSizeY, nSizeC; } VencAllocateBufferParam;
typedef struct {
    unsigned long nID; long long nPts; unsigned int nFlag;
    unsigned char *pAddrPhyY, *pAddrPhyC, *pAddrVirY, *pAddrVirC;
    unsigned char _pad[256];
} VencInputBuffer;
typedef struct {
    int nID; long long nPts; unsigned int nFlag, nSize0, nSize1;
    unsigned char *pData0, *pData1; unsigned char _pad[64];
} VencOutputBuffer;
extern void* VideoEncCreate(VENC_CODEC_TYPE);
extern int   VideoEncInit(void*, VencBaseConfig*);
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

/* ---- shared ring (must match camsiphon.c) ---- */
#define SHM_PATH  "/tmp/cam_stream.buf"
#define SHM_SLOT  (1UL << 20)
#define SHM_HDR   4096UL
#define SHM_TOTAL (SHM_HDR + 2 * SHM_SLOT)

static volatile unsigned int *g_hdr;   /* [0]=latest [1]=seq [2]=w [3]=h [4]=size */
static unsigned char *g_base;
static void *g_enc;

#define LOG(...) do { fprintf(stderr, "[camstream] " __VA_ARGS__); fprintf(stderr, "\n"); } while (0)

static void msleep(int ms) { struct timespec t = { ms/1000, (long)(ms%1000)*1000000L }; nanosleep(&t, 0); }

/* encode one NV21 frame (Y at +0, C at +W*H) to JPEG; returns bitstream into *out/*len (lib-owned
 * until FreeOneBitStreamFrame). Returns 0 on success. */
static int encode_jpeg(unsigned char *nv21, int W, int H, VencOutputBuffer *ob) {
    unsigned ysz = (unsigned)W * H, csz = ysz / 2;
    VencInputBuffer ib; memset(&ib, 0, sizeof(ib));
    if (GetOneAllocInputBuffer(g_enc, &ib) != 0 || !ib.pAddrVirY || !ib.pAddrVirC) return -1;
    memcpy(ib.pAddrVirY, nv21, ysz);
    memcpy(ib.pAddrVirC, nv21 + ysz, csz);
    ib.nPts = 0;
    FlushCacheAllocInputBuffer(g_enc, &ib);
    if (AddOneInputBuffer(g_enc, &ib) != 0) return -2;
    if (VideoEncodeOneFrame(g_enc) != 0) { AlreadyUsedInputBuffer(g_enc,&ib); ReturnOneAllocInputBuffer(g_enc,&ib); return -3; }
    AlreadyUsedInputBuffer(g_enc, &ib);
    ReturnOneAllocInputBuffer(g_enc, &ib);
    memset(ob, 0, sizeof(*ob));
    if (GetOneBitstreamFrame(g_enc, ob) != 0) return -4;
    return 0;
}

static int send_all(int fd, const void *buf, int len) {
    const char *p = buf; int left = len;
    while (left > 0) { int n = send(fd, p, left, MSG_NOSIGNAL); if (n <= 0) return -1; p += n; left -= n; }
    return 0;
}

int main(int argc, char **argv) {
    int port = (argc > 1) ? atoi(argv[1]) : 8090;
    signal(SIGPIPE, SIG_IGN);

    int sfd = open(SHM_PATH, O_RDWR);
    if (sfd < 0) { LOG("open %s failed (is /tmp/cam_stream set + camsiphon streaming?)", SHM_PATH); return 1; }
    g_base = mmap(0, SHM_TOTAL, PROT_READ | PROT_WRITE, MAP_SHARED, sfd, 0);
    if (g_base == MAP_FAILED) { LOG("mmap failed"); return 1; }
    g_hdr = (volatile unsigned int *)g_base;

    LOG("waiting for first frame...");
    for (int i = 0; i < 100 && g_hdr[4] == 0; i++) msleep(100);
    int W = g_hdr[2] ? (int)g_hdr[2] : 672, H = g_hdr[3] ? (int)g_hdr[3] : 504;
    LOG("frames flowing: %dx%d size=%u seq=%u", W, H, g_hdr[4], g_hdr[1]);

    /* init cedar JPEG encoder once */
    g_enc = VideoEncCreate(VENC_CODEC_JPEG);
    if (!g_enc) { LOG("VideoEncCreate failed"); return 1; }
    VencBaseConfig cfg; memset(&cfg, 0, sizeof(cfg));
    cfg.nInputWidth = W; cfg.nInputHeight = H; cfg.nStride = W;
    cfg.nDstWidth = W;  cfg.nDstHeight = H;
    cfg.eInputFormat = VENC_PIXEL_YVU420SP;
    cfg.memops = MemAdapterGetOpsS();
    if (VideoEncInit(g_enc, &cfg) != 0) { LOG("VideoEncInit failed"); return 1; }
    VencAllocateBufferParam bp = { 4, (unsigned)W * H, (unsigned)W * H / 2 };
    if (AllocInputBuffer(g_enc, &bp) != 0) { LOG("AllocInputBuffer failed"); return 1; }
    LOG("cedar JPEG encoder ready");

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    if (bind(srv, (struct sockaddr*)&a, sizeof(a)) < 0) { LOG("bind :%d failed: %s", port, strerror(errno)); return 1; }
    listen(srv, 4);
    LOG("MJPEG server on http://0.0.0.0:%d/", port);

    unsigned char *framebuf = malloc(SHM_SLOT);
    const char *hdr =
        "HTTP/1.0 200 OK\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

    for (;;) {
        int c = accept(srv, 0, 0);
        if (c < 0) continue;
        LOG("client connected");
        char req[1024]; recv(c, req, sizeof(req), 0);   /* drain request line(s) */
        if (send_all(c, hdr, (int)strlen(hdr)) < 0) { close(c); continue; }

        unsigned last = g_hdr[1] - 1; int frames = 0;
        for (;;) {
            unsigned seq = g_hdr[1];
            if (seq == last) { msleep(8); continue; }    /* wait for a fresh frame */
            last = seq;
            unsigned idx = g_hdr[0], size = g_hdr[4];
            if (size == 0 || size > SHM_SLOT) { msleep(8); continue; }
            memcpy(framebuf, g_base + SHM_HDR + (unsigned long)idx * SHM_SLOT, size);

            VencOutputBuffer ob;
            if (encode_jpeg(framebuf, W, H, &ob) != 0) { msleep(20); continue; }

            char part[160];
            int pl = snprintf(part, sizeof(part),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                ob.nSize0 + ob.nSize1);
            int bad = send_all(c, part, pl)
                   || (ob.nSize0 && send_all(c, ob.pData0, ob.nSize0))
                   || (ob.nSize1 && send_all(c, ob.pData1, ob.nSize1))
                   || send_all(c, "\r\n", 2);
            FreeOneBitStreamFrame(g_enc, &ob);
            if (bad) break;
            if ((++frames % 30) == 0) LOG("streamed %d frames", frames);
            msleep(66);   /* ~15 fps cap */
        }
        close(c);
        LOG("client disconnected (%d frames)", frames);
    }
    return 0;
}

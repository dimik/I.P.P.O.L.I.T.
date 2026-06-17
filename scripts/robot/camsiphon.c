/*
 * camsiphon.c — LD_PRELOAD frame siphon for AVA's camera (Dreame D10s Pro, /dev/video2).
 * ============================================================================================
 * READ-ONLY tap: AVA continuously captures raw frames from /dev/video2 (V4L2 mmap streaming,
 * multi-plane, ~14 fps) for its obstacle AI. This shim hooks open/openat/mmap/ioctl inside AVA
 * and, whenever AVA dequeues a frame (VIDIOC_DQBUF), copies that buffer out — without changing
 * AVA's flow at all (we call the real syscalls and only *read* the buffer AVA already owns). So
 * AVA keeps working; we just observe. No ISP seizure, no cloud, no encoder needed.
 *
 * Two egress paths, both zero-overhead unless their flag file exists:
 *   1. One-shot grab — when /tmp/cam_grab exists, copy ONE frame to /tmp/cam_frame.raw (+ remove
 *      the flag) and write /tmp/cam_fmt.txt ("WxH pixfmt=XXXX planesize=N").
 *   2. Continuous stream — when /tmp/cam_stream exists, copy EVERY frame into a RAM-backed shared
 *      ring (/tmp/cam_stream.buf, double-buffered) for the cedar MJPEG server to read. No flash
 *      writes, no AVA stall (we memcpy into the inactive slot, then flip the latest index).
 *
 * Freestanding (-nostdlib): AVA links glibc 2.23 but we build in a glibc-2.39 chroot, so no
 * libc — raw svc syscalls + kernel uapi struct defs only. Exports open/openat/mmap/ioctl.
 * Built/loaded alongside libfanoff_filter.so (LD_PRELOAD list). See build_ava_shims.sh.
 * ============================================================================================
 */
#include <linux/videodev2.h>
#include <stddef.h>

/* ---- aarch64 raw syscalls (no libc) ---- */
#define SYS_openat 56
#define SYS_mmap   222
#define SYS_ioctl  29
#define SYS_faccessat 48
#define SYS_write  64
#define SYS_close  57
#define SYS_unlinkat 35
#define SYS_renameat 38
#define SYS_ftruncate 46
#define AT_FDCWD   (-100)

static long sys1(long n,long a){register long x8 asm("x8")=n,x0 asm("x0")=a;asm volatile("svc #0":"+r"(x0):"r"(x8):"memory","cc");return x0;}
static long sys3(long n,long a,long b,long c){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2):"memory","cc");return x0;}
static long sys4(long n,long a,long b,long c,long d){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c,x3 asm("x3")=d;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2),"r"(x3):"memory","cc");return x0;}
static long sys6(long n,long a,long b,long c,long d,long e,long f){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c,x3 asm("x3")=d,x4 asm("x4")=e,x5 asm("x5")=f;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2),"r"(x3),"r"(x4),"r"(x5):"memory","cc");return x0;}

static int str_eq_dev_video2(const char *p) {           /* path ends with "/dev/video2" */
    /* find a "video2" with no trailing digit */
    const char *m = 0;
    for (const char *s = p; *s; s++)
        if (s[0]=='v'&&s[1]=='i'&&s[2]=='d'&&s[3]=='e'&&s[4]=='o'&&s[5]=='2'&&(s[6]<'0'||s[6]>'9')) m=s;
    return m != 0;
}
static int n2s(unsigned v, char *o){ char t[12]; int i=0,j=0; if(!v){o[0]='0';return 1;} while(v){t[i++]='0'+v%10;v/=10;} while(i)o[j++]=t[--i]; return j; }

/* ---- buffer tracking (per DQBUF index) ---- */
#define NB 8
static int    g_v2fd = -1;                 /* fd of /dev/video2 */
static unsigned long g_off[NB], g_len[NB], g_addr[NB];   /* per index: mmap offset, length, mapped addr */
static unsigned g_w, g_h, g_pixfmt, g_planesize;

static void write_fmt(void) {
    char b[96]; int n=0;
    n+=n2s(g_w,b+n); b[n++]='x'; n+=n2s(g_h,b+n);
    b[n++]=' '; b[n++]='p'; b[n++]='f'; b[n++]='=';
    b[n++]=g_pixfmt&0xff; b[n++]=(g_pixfmt>>8)&0xff; b[n++]=(g_pixfmt>>16)&0xff; b[n++]=(g_pixfmt>>24)&0xff;
    b[n++]=' '; b[n++]='s'; b[n++]='z'; b[n++]='='; n+=n2s(g_planesize,b+n); b[n++]='\n';
    int fd=(int)sys4(SYS_openat,AT_FDCWD,(long)"/tmp/cam_fmt.txt",0x241/*O_WRONLY|O_CREAT|O_TRUNC*/,0644);
    if(fd>=0){ sys3(SYS_write,fd,(long)b,n); sys1(SYS_close,fd); }
}
static void dump_frame(unsigned long addr, unsigned long len) {
    /* write to /tmp/.cam.tmp then rename to /tmp/cam_frame.raw (atomic latest) */
    int fd=(int)sys4(SYS_openat,AT_FDCWD,(long)"/tmp/.cam.tmp",0x241,0644);
    if(fd<0) return;
    unsigned long off=0; while(off<len){ long w=sys3(SYS_write,fd,(long)(addr+off),len-off); if(w<=0)break; off+=w; }
    sys1(SYS_close,fd);
    sys4(SYS_renameat,AT_FDCWD,(long)"/tmp/.cam.tmp",AT_FDCWD,(long)"/tmp/cam_frame.raw");
}
static int grab_requested(void){ return sys3(SYS_faccessat,AT_FDCWD,(long)"/tmp/cam_grab",0)==0; }
static void clear_grab(void){ sys3(SYS_unlinkat,AT_FDCWD,(long)"/tmp/cam_grab",0); }

/* ---- continuous stream egress: RAM-backed double-buffer shared with the cedar MJPEG server ---- */
#define SHM_SLOT  (1UL<<20)            /* 1 MiB per slot (cam frame ~0.5 MiB) */
#define SHM_HDR   4096UL              /* header page: [0]=latest [1]=seq [2]=w [3]=h [4]=size */
#define SHM_TOTAL (SHM_HDR + 2*SHM_SLOT)
static unsigned long g_shm = 0;        /* mapped base, 0 if not yet mapped */

static void fast_copy(unsigned char *d, const unsigned char *s, unsigned long n) {
    unsigned long i = 0;
    for (; i + 8 <= n; i += 8) *(unsigned long*)(d+i) = *(const unsigned long*)(s+i);
    for (; i < n; i++) d[i] = s[i];
}
static int stream_requested(void){ return sys3(SYS_faccessat,AT_FDCWD,(long)"/tmp/cam_stream",0)==0; }
static void stream_map(void) {
    int fd=(int)sys4(SYS_openat,AT_FDCWD,(long)"/tmp/cam_stream.buf",0x42/*O_RDWR|O_CREAT*/,0644);
    if(fd<0) return;
    sys3(SYS_ftruncate,fd,SHM_TOTAL,0);
    long r=sys6(SYS_mmap,0,SHM_TOTAL,3/*RW*/,1/*MAP_SHARED*/,fd,0);
    sys1(SYS_close,fd);
    if(r>0) g_shm=(unsigned long)r;
}
static void stream_frame(unsigned long addr, unsigned long n, unsigned w, unsigned h) {
    if(!g_shm){ stream_map(); if(!g_shm) return; }
    if(n > SHM_SLOT) n = SHM_SLOT;
    volatile unsigned int *hdr=(volatile unsigned int*)g_shm;
    unsigned cur = hdr[0] ? 0 : 1;                          /* write the inactive slot */
    fast_copy((unsigned char*)(g_shm+SHM_HDR+cur*SHM_SLOT), (const unsigned char*)addr, n);
    hdr[2]=w; hdr[3]=h; hdr[4]=(unsigned)n;
    hdr[0]=cur;                                             /* publish: flip latest */
    hdr[1]++;                                               /* seq bump */
}

/* ---- interposed entry points ---- */
long openat(int dirfd, const char *path, int flags, unsigned mode) {
    long fd = sys4(SYS_openat, dirfd, (long)path, flags, mode);
    if (fd >= 0 && path && str_eq_dev_video2(path)) g_v2fd = (int)fd;
    return fd;
}
long open(const char *path, int flags, unsigned mode) { return openat(AT_FDCWD, path, flags, mode); }

void *mmap(void *addr, unsigned long len, int prot, int flags, int fd, long off) {
    long r = sys6(SYS_mmap,(long)addr,len,prot,flags,fd,off);
    if (fd == g_v2fd && r > 0)
        for (int i=0;i<NB;i++) if (g_off[i]==(unsigned long)off && g_len[i]) { g_addr[i]=(unsigned long)r; break; }
    return (void*)r;
}

int ioctl(int fd, unsigned long req, void *arg) {
    long r = sys3(SYS_ioctl, fd, req, (long)arg);
    if (fd != g_v2fd || r != 0 || !arg) return (int)r;

    if (req == VIDIOC_QUERYBUF) {
        struct v4l2_buffer *b = arg;
        if (b->index < NB && b->length >= 1 && b->m.planes) {
            g_off[b->index] = b->m.planes[0].m.mem_offset;
            g_len[b->index] = b->m.planes[0].length;
        }
    } else if (req == VIDIOC_S_FMT || req == VIDIOC_G_FMT) {
        struct v4l2_format *f = arg;
        if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            g_w=f->fmt.pix_mp.width; g_h=f->fmt.pix_mp.height; g_pixfmt=f->fmt.pix_mp.pixelformat;
            g_planesize=f->fmt.pix_mp.plane_fmt[0].sizeimage; write_fmt();
        }
    } else if (req == VIDIOC_DQBUF) {
        struct v4l2_buffer *b = arg;
        if (b->index < NB && g_addr[b->index] && b->m.planes) {
            unsigned long n = b->m.planes[0].bytesused; if (!n) n = g_len[b->index];
            if (grab_requested()) { dump_frame(g_addr[b->index], n); clear_grab(); }
            if (stream_requested()) stream_frame(g_addr[b->index], n, g_w, g_h);
        }
    }
    return (int)r;
}

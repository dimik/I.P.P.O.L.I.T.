/* ============================================================================================
 * serialtap.c — LD_PRELOAD read-tap for AVA's sensor serial ports (Dreame D10s Pro).
 *
 * Taps BOTH AVA-owned serial links, read-only, by copying the bytes AVA already read:
 *   /dev/ttyS3  LiDAR LDS  (230400, `55 aa 03 08` frames)  -> /tmp/lds_ring.buf  -> /scan
 *   /dev/ttyS4  MCU        (`3c..3e` frames, IMU + odom)    -> /tmp/mcu_ring.buf  -> /imu,/odom
 *
 * (One shim for both because read()/readv() can only be interposed once — a second read-hooking
 *  .so would be shadowed. The historical single-port name is kept to avoid churn; see docs/ros.md.)
 *
 * WHY THIS IS AVA-SAFE (hard-won):
 *  - errno contract: a freestanding read() MUST return -1 and set errno on error, NOT the raw
 *    -errno — AVA's non-blocking read/select loops choke on a raw -errno (-11 vs -1/EAGAIN). This
 *    was the bug that broke the first tap (mcutap). Fixed via __errno_location (AVA's glibc 2.23).
 *  - shm ring, not sendto: memcpy into a RAM-backed ring (tmpfs), mmap'd ONCE — no syscall in
 *    AVA's hot read path (ttyS4 is AVA's real-time control loop, ~600 reads/s).
 *  - fd isolation: the streams are fragmented and other threads read other fds concurrently, so we
 *    tee ONLY the two target fds, found by scanning /proc/self/fd (robust to fd renumbering, no
 *    openat hook -> no clash with the camsiphon shim). The decoders re-sync on the framing markers.
 *
 * Exports read()/readv() only — not claimed by fanoff (write/writev) or camsiphon (open/openat/
 * mmap/ioctl). Freestanding (-nostdlib), built in the glibc-2.39 chroot. See build_ava_shims.sh.
 * ============================================================================================ */
#include <stddef.h>

#define SYS_read       63
#define SYS_readv      65
#define SYS_openat     56
#define SYS_close      57
#define SYS_ftruncate  46
#define SYS_mmap      222
#define SYS_readlinkat 78
#define AT_FDCWD      (-100)

struct iovec { void *iov_base; size_t iov_len; };

static long sys1(long n,long a){register long x8 asm("x8")=n,x0 asm("x0")=a;asm volatile("svc #0":"+r"(x0):"r"(x8):"memory","cc");return x0;}
static long sys3(long n,long a,long b,long c){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2):"memory","cc");return x0;}
static long sys4(long n,long a,long b,long c,long d){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c,x3 asm("x3")=d;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2),"r"(x3):"memory","cc");return x0;}
static long sys6(long n,long a,long b,long c,long d,long e,long f){register long x8 asm("x8")=n,x0 asm("x0")=a,x1 asm("x1")=b,x2 asm("x2")=c,x3 asm("x3")=d,x4 asm("x4")=e,x5 asm("x5")=f;asm volatile("svc #0":"+r"(x0):"r"(x8),"r"(x1),"r"(x2),"r"(x3),"r"(x4),"r"(x5):"memory","cc");return x0;}

/* glibc errno contract — return -1 + errno on error (NOT raw -errno). The bug that broke mcutap. */
extern int *__errno_location(void);
static long ret(long r){ if (r < 0) { *__errno_location() = (int)(-r); return -1; } return r; }

static int n2s(unsigned v, char *o){ char t[12]; int i=0,j=0; if(!v){o[0]='0';return 1;} while(v){t[i++]='0'+v%10;v/=10;} while(i)o[j++]=t[--i]; return j; }
static void fast_copy(unsigned char *d, const unsigned char *s, unsigned long n){
    unsigned long i=0; for(;i+8<=n;i+=8)*(unsigned long*)(d+i)=*(const unsigned long*)(s+i); for(;i<n;i++)d[i]=s[i];
}

/* ---- per-port tap: target device, ring file, resolved fd, mapped ring base ---- */
#define RING (256UL*1024)
#define HDR  64UL                               /* hdr[0]=write_pos hdr[1]=magic hdr[2]=ringsize */
#define MAGIC 0x0031534444530001UL
#define NTAP 2
struct tap { const char *dev; const char *ringpath; int fd; unsigned long ring; };
static struct tap g_tap[NTAP] = {
    { "/dev/ttyS3", "/tmp/lds_ring.buf", -1, 0 },   /* LiDAR LDS */
    { "/dev/ttyS4", "/tmp/mcu_ring.buf", -1, 0 },   /* MCU: IMU + odom */
};
static unsigned g_scan_ctr = 0;

static int dev_is(int fd, const char *want) {     /* readlink /proc/self/fd/<fd> == want (both 10 ch) */
    char path[24] = "/proc/self/fd/"; int p = 14; p += n2s((unsigned)fd, path + p); path[p] = 0;
    char buf[40];
    long len = sys4(SYS_readlinkat, AT_FDCWD, (long)path, (long)buf, sizeof buf);
    if (len != 10) return 0;
    for (int i = 0; i < 10; i++) if (buf[i] != want[i]) return 0;
    return 1;
}
static void find_fds(void) {                      /* resolve any not-yet-found tap fds */
    for (int t = 0; t < NTAP; t++) {
        if (g_tap[t].fd >= 0) continue;
        for (int fd = 3; fd < 64; fd++) if (dev_is(fd, g_tap[t].dev)) { g_tap[t].fd = fd; break; }
    }
}
static int all_found(void){ for(int t=0;t<NTAP;t++) if(g_tap[t].fd<0) return 0; return 1; }

static void ring_map(struct tap *tp){
    int fd=(int)sys4(SYS_openat,AT_FDCWD,(long)tp->ringpath,0x42/*O_RDWR|O_CREAT*/,0644);
    if(fd<0) return;
    sys3(SYS_ftruncate,fd,HDR+RING,0);
    long r=sys6(SYS_mmap,0,HDR+RING,3/*RW*/,1/*MAP_SHARED*/,fd,0);
    sys1(SYS_close,fd);
    if(r>0){ tp->ring=(unsigned long)r;
        volatile unsigned long *h=(volatile unsigned long*)tp->ring; h[1]=MAGIC; h[2]=RING; }
}
static void ring_put(struct tap *tp, const void *buf, long n){
    if(n<=0) return;
    if((unsigned long)n>RING) n=RING;
    if(!tp->ring){ ring_map(tp); if(!tp->ring) return; }
    volatile unsigned long *h=(volatile unsigned long*)tp->ring;
    unsigned char *data=(unsigned char*)(tp->ring+HDR);
    unsigned long wp=h[0], off=wp%RING; long first=n;
    if(off+(unsigned long)first>RING) first=(long)(RING-off);
    fast_copy(data+off,(const unsigned char*)buf,first);
    if(n>first) fast_copy(data,(const unsigned char*)buf+first,n-first);
    asm volatile("dmb ish":::"memory");           /* publish data before advancing write_pos */
    h[0]=wp+n;
}
static struct tap *tap_for(int fd){ for(int t=0;t<NTAP;t++) if(fd==g_tap[t].fd) return &g_tap[t]; return 0; }

/* ---- interposed entry points ---- */
long read(int fd, void *buf, size_t count) {
    long r = sys3(SYS_read, fd, (long)buf, (long)count);
    if (!all_found() && (g_scan_ctr++ & 0x3FF) == 0) find_fds();
    if (r > 0) { struct tap *tp = tap_for(fd); if (tp) ring_put(tp, buf, r); }
    return ret(r);
}

long readv(int fd, const struct iovec *iov, int iovcnt) {
    long r = sys3(SYS_readv, fd, (long)iov, (long)iovcnt);
    if (!all_found() && (g_scan_ctr++ & 0x3FF) == 0) find_fds();
    if (r > 0 && iov) {
        struct tap *tp = tap_for(fd);
        if (tp) { long left = r;
            for (int i = 0; i < iovcnt && left > 0; i++) {
                long m = (long)iov[i].iov_len; if (m > left) m = left;
                ring_put(tp, iov[i].iov_base, m); left -= m;
            } }
    }
    return ret(r);
}

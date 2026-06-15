/*
 * fanoff_shim.c — LD_PRELOAD interposer for AVA's MCU serial link (/dev/ttyS3).
 *
 * GOAL: guarantee the vacuum fan stays OFF in every mode (esp. manual navigation /
 * work_mode 17) by rewriting the fan-power byte of CLEANSET frames AVA writes to the MCU.
 * This is the single chokepoint downstream of all of AVA's CleanMode/fan_speed/only_mop
 * logic — see CLAUDE.md "MCU CLEANSET fan-byte filter".
 *
 * Two compile-time modes (see build_fanoff.sh):
 *   -DMODE_LOG    : passthrough; dump every 55 AA frame to /tmp/mcu_tx.log (Phase-1 capture)
 *   -DMODE_FILTER : rewrite CLEANSET fan byte -> 0, recompute checksum (production)
 *
 * FREESTANDING by design: AVA links glibc 2.23, but our build toolchain (Ubuntu 24.04
 * chroot) is glibc 2.39. A normally-linked .so would pull GLIBC_2.3x symbol deps and fail
 * to load under AVA. So we use NO libc: raw svc #0 syscalls + hand-rolled mem helpers.
 * Build: aarch64-linux-gnu-gcc-13 -nostdlib -ffreestanding -fPIC -shared.
 *
 * Frame detection keys off the 55 AA magic in the write buffer, NOT a hardcoded fd
 * (fd 26 -> /dev/ttyS3 today, but not guaranteed across boots).
 *
 * ============================ CONSTANTS TO FILL AFTER CAPTURE ============================
 * Run capture_cleanset.sh, then decode /tmp/mcu_tx.log to set the four constants below.
 * Until then MODE_FILTER is a no-op-safe passthrough (it only rewrites when it can both
 * match CLEANSET_CMD and the checksum it recomputes matches the trailing byte).
 */

#include <stddef.h>

/* ---- aarch64 syscall numbers (asm-generic) ---- */
#define SYS_openat 56
#define SYS_write  64
#define SYS_writev 66

#define AT_FDCWD   (-100)
#define O_WRONLY   1
#define O_CREAT    0100
#define O_APPEND   02000

struct iovec { void *iov_base; size_t iov_len; };

/* ---- raw syscalls via svc #0 (no libc) ---- */
static long sys3(long n, long a, long b, long c) {
    register long x8 asm("x8") = n;
    register long x0 asm("x0") = a;
    register long x1 asm("x1") = b;
    register long x2 asm("x2") = c;
    asm volatile("svc #0" : "+r"(x0) : "r"(x8), "r"(x1), "r"(x2) : "memory", "cc");
    return x0;
}
static long sys4(long n, long a, long b, long c, long d) {
    register long x8 asm("x8") = n;
    register long x0 asm("x0") = a;
    register long x1 asm("x1") = b;
    register long x2 asm("x2") = c;
    register long x3 asm("x3") = d;
    asm volatile("svc #0" : "+r"(x0) : "r"(x8), "r"(x1), "r"(x2), "r"(x3) : "memory", "cc");
    return x0;
}

static long raw_write(int fd, const void *buf, unsigned long n) {
    return sys3(SYS_write, fd, (long)buf, (long)n);
}

/* ---- hand-rolled helpers ---- */
static void mcpy(unsigned char *d, const unsigned char *s, unsigned long n) {
    for (unsigned long i = 0; i < n; i++) d[i] = s[i];
}

/* =================== FRAME SPEC — FILL FROM CAPTURE (Phase 2) =================== */
/* !!! 2026-06-15 CORRECTION: live strace proved the MCU protocol is NOT `55 AA`. !!!
 * The real channel is fd 24 / /dev/ttyS4 (ttyS3/fd26 is silent), and frames are
 * `3c <cmd> <payload...> <crc16> 3e`  ( '<' ... '>' ).  The constants below are STILL
 * the old 55 AA placeholders — they must be reworked for the 3c..3e protocol, and the
 * checksum is a 2-byte CRC16 (not a 1-byte sum). This cannot be finalized until we
 * capture a genuine fan-ON frame (blocked: manual control returns HTTP 400 while docked).
 * Until then MODE_FILTER is a safe no-op (no buffer starts with 0x55 0xAA on the bot). */
#define MAGIC0        0x55   /* TODO -> frame start 0x3c ('<')                           */
#define MAGIC1        0xAA   /* TODO -> (no fixed 2nd magic byte in 3c..3e; rework scan) */
#define CLEANSET_CMD  0xFF   /* TODO: fan-command id (2nd byte after 0x3c)               */
#define FAN_OFFSET    0xFF   /* TODO: byte offset of fan-power field within the frame    */
#define HDR_LEN_OFF   2      /* TODO: 3c..3e frames are end-delimited by 0x3e, not len   */

/* Recompute checksum over the frame. TODO: real protocol uses a 2-byte CRC16 trailer
 * before 0x3e — implement the actual CRC16 variant (poly TBD from capture), not this sum. */
static unsigned char cksum(const unsigned char *f, unsigned long framelen) {
    unsigned int s = 0;
    for (unsigned long i = 2; i + 1 < framelen; i++) s += f[i];
    return (unsigned char)(s & 0xFF);
}

/* Return total frame length given a pointer to 55 AA and remaining bytes, or 0 if unknown.
 * TODO: confirm whether <len> is payload-only, includes header, includes checksum, etc. */
static unsigned long framelen_of(const unsigned char *f, unsigned long avail) {
    if (avail < 4) return 0;
    unsigned long len = f[HDR_LEN_OFF];
    /* TODO: adjust per capture. Placeholder treats <len> as total-frame-minus-header(3). */
    unsigned long total = len + 3;
    if (total < 4 || total > avail) return 0;
    return total;
}

/* ---- logging (MODE_LOG only) ---- */
#ifdef MODE_LOG
static int log_fd = -2; /* -2 = uninitialized, -1 = failed */
static void log_open(void) {
    /* /tmp/mcu_tx.log */
    static const char path[] = "/tmp/mcu_tx.log";
    log_fd = (int)sys4(SYS_openat, AT_FDCWD, (long)path, O_WRONLY | O_CREAT | O_APPEND, 0644);
}
static void log_frame(const unsigned char *f, unsigned long n) {
    if (log_fd == -2) log_open();
    if (log_fd < 0) return;
    /* dump raw bytes; offline decoder hexdumps. A 0xFF 0xFF record separator precedes each. */
    unsigned char sep[2] = {0xFF, 0xFF};
    raw_write(log_fd, sep, 2);
    raw_write(log_fd, f, n);
}
#endif

/* Core: given a writable copy of the buffer, rewrite all CLEANSET frames in place.
 * Length-preserving (never changes byte count returned to AVA). */
static void process(unsigned char *b, unsigned long n) {
    unsigned long i = 0;
    while (i + 4 <= n) {
        if (b[i] == MAGIC0 && b[i + 1] == MAGIC1) {
            unsigned long fl = framelen_of(b + i, n - i);
            if (fl == 0) { i++; continue; }
#ifdef MODE_LOG
            log_frame(b + i, fl);
#endif
#ifdef MODE_FILTER
            if (b[i + 3] == CLEANSET_CMD && FAN_OFFSET < fl) {
                unsigned char want = cksum(b + i, fl);
                /* only rewrite if our checksum model matches reality (safety until verified) */
                if (b[i + fl - 1] == want) {
                    b[i + FAN_OFFSET] = 0;            /* fan power -> off */
                    b[i + fl - 1] = cksum(b + i, fl); /* fix checksum    */
                }
            }
#endif
            i += fl;
        } else {
            i++;
        }
    }
}

/* ---- interposed libc entry points ---- */
#define MAXFRAME 1024

long write(int fd, const void *buf, size_t count) {
    const unsigned char *p = (const unsigned char *)buf;
    /* Only MCU frames start with 55 AA; everything else passes through untouched. */
    if (count >= 4 && count <= MAXFRAME && p[0] == MAGIC0 && p[1] == MAGIC1) {
        unsigned char tmp[MAXFRAME];
        mcpy(tmp, p, count);
        process(tmp, count);
        return raw_write(fd, tmp, count);
    }
    return raw_write(fd, buf, count);
}

/* Hook writev too in case AVA uses it for the serial link (confirm via strace). */
long writev(int fd, const struct iovec *iov, int iovcnt) {
    /* If a single small 55 AA iovec, filter it; otherwise pass through verbatim. */
    if (iovcnt == 1 && iov && iov[0].iov_len >= 4 && iov[0].iov_len <= MAXFRAME) {
        const unsigned char *p = (const unsigned char *)iov[0].iov_base;
        if (p[0] == MAGIC0 && p[1] == MAGIC1) {
            unsigned char tmp[MAXFRAME];
            mcpy(tmp, p, iov[0].iov_len);
            process(tmp, iov[0].iov_len);
            return raw_write(fd, tmp, iov[0].iov_len);
        }
    }
    return sys3(SYS_writev, fd, (long)iov, (long)iovcnt);
}

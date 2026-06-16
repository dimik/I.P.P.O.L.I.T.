/*
 * fanoff_shim.c — LD_PRELOAD MCU-command filter for the Dreame D10s Pro (model r2250).
 * ============================================================================================
 *
 * PURPOSE
 *   AVA (the closed-source robot daemon) drives the MCU over a serial link. This shim is
 *   LD_PRELOAD-ed into AVA and transparently rewrites selected MCU command packets on their way
 *   out, so we can disable individual robot subsystems (e.g. the vacuum fan, the LiDAR turret)
 *   without touching AVA. Everything not matched by a rule passes through byte-for-byte.
 *
 * WHY A SHIM
 *   The MCU serial is opened exclusively by AVA; we cannot share it. Interposing libc's
 *   write()/writev() inside AVA is the least invasive, lowest-latency way to edit the stream.
 *
 * MCU WIRE PROTOCOL  (reverse-engineered; ref github.com/alufers/dreame_mcu_protocol)
 *   Channel : /dev/ttyS4 (the LiDAR/LDS link is a *separate* port, /dev/ttyS3 — not touched).
 *             The fd is not stable across boots, so frames are detected by their 0x3c start
 *             byte rather than by fd number.
 *   Framing : 3c <len> <type> <payload[len]> <crc_hi> <crc_lo> 3e
 *             0x3f ('?') escapes a literal 0x3c/0x3e/0x3f inside the body.
 *   CRC     : Modbus CRC16 over the unescaped [len, type, payload], stored big-endian.
 *
 * POLICY — see RULES[] below. Each rule matches a packet (type [+ first payload byte]) and,
 *   when its gate says so, rewrites the payload and recomputes the CRC. To add/adjust a
 *   subsystem you edit ONLY the RULES[] table; the codec and dispatch loop stay untouched.
 *
 * CURRENT RULES
 *   - vacuum fan  : SetCleaning(0x01) payload -> docked-idle "00 01 00 00 00", in EVERY mode.
 *   - LiDAR turret: _CtrlMcuCMD(0x14) subcmd 0x04 (LDS motor enable) -> 0, UNLESS
 *                   /tmp/lidar_allow exists. A separate gate daemon (fanoff_flag.sh) creates
 *                   that flag in active non-manual modes, so the turret runs for mapping/go-to
 *                   and is parked only during manual driving. Blocked-by-default means manual
 *                   nav never spins it up (no start-up blip / race).
 *
 * FREESTANDING BUILD
 *   AVA links glibc 2.23 but we compile in a glibc-2.39 chroot, so the shim uses NO libc:
 *   raw `svc #0` syscalls + hand-rolled helpers (-nostdlib -ffreestanding). It exports only
 *   write()/writev(). See build_ava_shims.sh.  Build modes: MODE_FILTER (apply rules) and
 *   MODE_LOG (passthrough + dump every frame to /tmp/mcu_tx.log).
 * ============================================================================================
 */

#include <stddef.h>

/* ============================================================================================
 * 1. Raw Linux syscalls (aarch64) — no libc.
 * ========================================================================================== */
#define SYS_openat    56
#define SYS_faccessat 48
#define SYS_write     64
#define SYS_writev    66
#define AT_FDCWD     (-100)

struct iovec { void *iov_base; size_t iov_len; };

static long sys3(long n, long a, long b, long c) {
    register long x8 asm("x8") = n, x0 asm("x0") = a, x1 asm("x1") = b, x2 asm("x2") = c;
    asm volatile("svc #0" : "+r"(x0) : "r"(x8), "r"(x1), "r"(x2) : "memory", "cc");
    return x0;
}
static long sys4(long n, long a, long b, long c, long d) {
    register long x8 asm("x8") = n, x0 asm("x0") = a, x1 asm("x1") = b, x2 asm("x2") = c, x3 asm("x3") = d;
    asm volatile("svc #0" : "+r"(x0) : "r"(x8), "r"(x1), "r"(x2), "r"(x3) : "memory", "cc");
    return x0;
}
static long raw_write(int fd, const void *buf, unsigned long n) { return sys3(SYS_write, fd, (long)buf, (long)n); }

/* faccessat(AT_FDCWD, path, F_OK) == 0  ->  file exists */
static int file_exists(const char *path) { return sys3(SYS_faccessat, AT_FDCWD, (long)path, 0) == 0; }

/* ============================================================================================
 * 2. Modbus CRC16 (ported from dreame_mcu_protocol/crc_util.py — reproduces every captured frame)
 * ========================================================================================== */
static const unsigned short CRC_TAB[256] = {
0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040,
};
static unsigned short crc16_modbus(const unsigned char *p, unsigned long n) {
    unsigned short crc = 0xFFFF;
    for (unsigned long i = 0; i < n; i++)
        crc = (crc >> 8) ^ CRC_TAB[(crc ^ p[i]) & 0xFF];
    return crc;
}

/* ============================================================================================
 * 3. MCU frame codec — framing (3c..3e) + 0x3f escaping.
 *
 *    A decoded frame's "content" is the unescaped [len, type, payload..., crc_hi, crc_lo];
 *    content[0]=len, content[1]=type, payload = content[2 .. 2+len-1], crc = last two bytes,
 *    and a well-formed frame has content_len == len + 4.
 * ========================================================================================== */
#define FRAME_START 0x3c
#define FRAME_END   0x3e
#define FRAME_ESC   0x3f
#define MAX_CONTENT 64          /* MCU command frames are small; this is ample */

static int is_reserved(unsigned char b) { return b == FRAME_START || b == FRAME_END || b == FRAME_ESC; }

/*
 * Decode one frame beginning at in[0] (== FRAME_START). Unescapes the body into `content`.
 * On success returns the number of input bytes consumed (including both delimiters) and sets
 * *content_len; returns 0 if the buffer does not hold a complete, well-terminated frame.
 */
static unsigned long frame_decode(const unsigned char *in, unsigned long n,
                                  unsigned char *content, unsigned long *content_len) {
    unsigned long i = 1, clen = 0;          /* skip FRAME_START */
    while (i < n && clen < MAX_CONTENT) {
        unsigned char c = in[i++];
        if (c == FRAME_END) { *content_len = clen; return i; }
        if (c == FRAME_ESC) { if (i < n) content[clen++] = in[i++]; }
        else                  content[clen++] = c;
    }
    return 0;                                /* no terminator within the buffer */
}

/* Emit `content` as a framed, escaped packet into `out`; returns bytes written. */
static unsigned long frame_encode(const unsigned char *content, unsigned long clen, unsigned char *out) {
    unsigned long o = 0;
    out[o++] = FRAME_START;
    for (unsigned long k = 0; k < clen; k++) {
        if (is_reserved(content[k])) out[o++] = FRAME_ESC;
        out[o++] = content[k];
    }
    out[o++] = FRAME_END;
    return o;
}

/* Recompute and store the trailing big-endian CRC for a content buffer of a `len`-payload frame. */
static void frame_fix_crc(unsigned char *content, unsigned int len) {
    unsigned short crc = crc16_modbus(content, 2 + len);     /* over [len, type, payload] */
    content[2 + len]     = (unsigned char)(crc >> 8);
    content[2 + len + 1] = (unsigned char)(crc & 0xFF);
}

/* ============================================================================================
 * 4. Filter policy — the rule table.  *** This is the only place to add/adjust a subsystem. ***
 * ========================================================================================== */

/* MCU packet types (frame content[1]) we act on. */
#define TYPE_SETCLEANING 0x01   /* fan/brush/pump power: payload f1..f5 */
#define TYPE_CTRLMCU     0x14   /* _CtrlMcuCMD: payload = <subcmd> <value> */
#define LDS_SUBCMD       0x04   /* _CtrlMcuCMD subcmd selecting the LiDAR turret motor (value 1=spin/0=off) */
#define MATCH_ANY        (-1)   /* rule.subcmd: match regardless of the first payload byte */

/* How a matched packet's payload is rewritten. */
typedef enum {
    REWRITE_SETCLEANING_IDLE,   /* force the whole SetCleaning payload to the docked-idle pattern */
    REWRITE_ZERO_BYTE,          /* set one payload byte (rule.arg) to 0 — e.g. an enable flag */
} RewriteAction;

/* When a rule is active. */
typedef enum {
    GATE_ALWAYS,                /* always filter (e.g. the vacuum fan: off in every mode) */
    GATE_UNLESS_FLAG,           /* filter UNLESS rule.allow_flag exists (e.g. LiDAR allowed in non-manual modes) */
} RuleGate;

typedef struct {
    const char   *name;         /* documentation / MODE_LOG label */
    unsigned char type;         /* MCU packet type to match (content[1]) */
    short         subcmd;       /* required first payload byte, or MATCH_ANY */
    RewriteAction action;
    unsigned char arg;          /* payload index for REWRITE_ZERO_BYTE */
    RuleGate      gate;
    const char   *allow_flag;   /* path tested for GATE_UNLESS_FLAG (NULL otherwise) */
} McuRule;

static const McuRule RULES[] = {
    /* Vacuum fan — silence in every mode. SetCleaning payload -> idle "00 01 00 00 00". */
    { "vacuum-fan",   TYPE_SETCLEANING, MATCH_ANY,  REWRITE_SETCLEANING_IDLE, 0, GATE_ALWAYS,      NULL },

    /* LiDAR turret — park during manual nav only. _CtrlMcuCMD subcmd 0x04 value -> 0, UNLESS the
     * gate daemon has flagged an active (non-manual) mode. arg=1 -> zero payload[1] (the value). */
    { "lidar-turret", TYPE_CTRLMCU,     LDS_SUBCMD, REWRITE_ZERO_BYTE,        1, GATE_UNLESS_FLAG, "/tmp/lidar_allow" },

    /* To disable another subsystem later, add a rule here — e.g. line-laser or front camera —
     * no other code changes are needed. (Do NOT gate the IMU: AVA needs it to drive.) */
};
#define NUM_RULES ((int)(sizeof(RULES) / sizeof(RULES[0])))

/* Does this rule match a decoded frame? `len` is the payload length (content[0]). */
static int rule_matches(const McuRule *r, const unsigned char *content, unsigned int len) {
    if (content[1] != r->type) return 0;
    if (r->subcmd != MATCH_ANY && (len < 1 || content[2] != (unsigned char)r->subcmd)) return 0;
    return 1;
}

/* Should this rule filter right now? (gate evaluation) */
static int rule_is_active(const McuRule *r) {
    switch (r->gate) {
        case GATE_ALWAYS:      return 1;
        case GATE_UNLESS_FLAG: return !file_exists(r->allow_flag);   /* filter while the allow-flag is absent */
    }
    return 0;
}

/* Apply a rule's rewrite to the payload, then fix the CRC. Returns 1 if anything changed. */
static int rule_apply(const McuRule *r, unsigned char *content, unsigned int len) {
    switch (r->action) {
        case REWRITE_SETCLEANING_IDLE:
            /* Idle pattern observed when docked: f1=0, f2=1, f3..=0. f1/f2 are the base
             * fan/brush/pump power; f3 is the fan boost tier. Zeroing only f3 left the fan at
             * base speed, so the whole payload is forced to idle. */
            if (len < 2) return 0;
            content[2] = 0x00;
            content[3] = 0x01;
            for (unsigned int k = 4; k < 2u + len; k++) content[k] = 0x00;
            break;
        case REWRITE_ZERO_BYTE:
            if (r->arg >= len) return 0;
            if (content[2 + r->arg] == 0) return 0;   /* already zero — nothing to do */
            content[2 + r->arg] = 0x00;
            break;
    }
    frame_fix_crc(content, len);
    return 1;
}

/* ============================================================================================
 * 5. Packet processing — scan a write buffer, filter matching frames, copy everything else.
 * ========================================================================================== */
#ifdef MODE_LOG
static int g_log_fd = -2;       /* -2 = unopened, -1 = open failed */
static void log_frame(const unsigned char *raw, unsigned long n) {
    if (g_log_fd == -2) {
        static const char path[] = "/tmp/mcu_tx.log";
        g_log_fd = (int)sys4(SYS_openat, AT_FDCWD, (long)path, 1 | 0100 | 02000, 0644); /* O_WRONLY|O_CREAT|O_APPEND */
    }
    if (g_log_fd >= 0) { unsigned char sep[2] = {0xFF, 0xFF}; raw_write(g_log_fd, sep, 2); raw_write(g_log_fd, raw, n); }
}
#endif

/* Process `in`[0..n) into `out`; returns the number of bytes written to `out`. */
static unsigned long process(const unsigned char *in, unsigned long n, unsigned char *out) {
    unsigned long i = 0, o = 0;
    while (i < n) {
        if (in[i] != FRAME_START) { out[o++] = in[i++]; continue; }

        unsigned char content[MAX_CONTENT];
        unsigned long clen = 0;
        unsigned long consumed = frame_decode(in + i, n - i, content, &clen);
        if (consumed == 0) {                       /* incomplete frame: pass the remainder through */
            while (i < n) out[o++] = in[i++];
            break;
        }
#ifdef MODE_LOG
        log_frame(in + i, consumed);
#endif
        unsigned int len = content[0];
        int rewritten = 0;
#ifdef MODE_FILTER
        if (clen == len + 4u) {                     /* well-formed: [len][type][payload][crc16] */
            for (int ri = 0; ri < NUM_RULES; ri++) {
                if (rule_matches(&RULES[ri], content, len) && rule_is_active(&RULES[ri])) {
                    if (rule_apply(&RULES[ri], content, len)) {
                        o += frame_encode(content, clen, out + o);
                        rewritten = 1;
                    }
                    break;                          /* first matching rule wins */
                }
            }
        }
#endif
        if (!rewritten) { for (unsigned long k = 0; k < consumed; k++) out[o++] = in[i + k]; }
        i += consumed;
    }
    return o;
}

/* ============================================================================================
 * 6. Interposed libc entry points. AVA's writes to the MCU come through write()/writev().
 *    We only touch buffers that begin with FRAME_START; we always report the original byte
 *    count back to AVA (the MCU resyncs on the 3c/3e delimiters, so a length change is fine).
 * ========================================================================================== */
#define MAX_IN 1024

long write(int fd, const void *buf, size_t count) {
    const unsigned char *p = (const unsigned char *)buf;
    if (count >= 4 && count <= MAX_IN && p[0] == FRAME_START) {
        unsigned char out[2 * MAX_IN];
        raw_write(fd, out, process(p, count, out));
        return (long)count;
    }
    return raw_write(fd, buf, count);
}

long writev(int fd, const struct iovec *iov, int iovcnt) {
    if (iovcnt == 1 && iov && iov[0].iov_len >= 4 && iov[0].iov_len <= MAX_IN) {
        const unsigned char *p = (const unsigned char *)iov[0].iov_base;
        if (p[0] == FRAME_START) {
            unsigned char out[2 * MAX_IN];
            raw_write(fd, out, process(p, iov[0].iov_len, out));
            return (long)iov[0].iov_len;
        }
    }
    return sys3(SYS_writev, fd, (long)iov, (long)iovcnt);
}

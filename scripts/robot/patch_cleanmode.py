#!/usr/bin/env python3
"""
patch_cleanmode.py - Binary patch WritePropIntProcess in node_porphyrion.so

PROBLEM: When Valetudo sends piid:13 (RemoteCtrl/manual drive enable), AVA calls
WritePropInt(type=0, value=0) which resets CleanMode to 0 (sweep=fan ON). This
happens atomically within a BT tick, so no polling daemon can win the race.

SOLUTION: Patch the instruction in WritePropIntProcess that loads msg->value just
before copying it into the outgoing pub/sub message. If type==0 AND value==0,
substitute value=1 (mop-only=fan OFF). This intercepts the message BEFORE it's
published to WritePropIntProcessSUB, so the property storage and CLEANSET motor
command both see CleanMode=1.

PATCH SITE (SO-relative): 0x2EB750 — LDR w2, [x21, #16]  (loads msg->value)
  At this point: x21=msg*, w3=msg->type (from LDR w3, [x21, #8] at SO+0x2EB72C)
  The result w2 is stored to outgoing msg at SO+0x2EB754 (STR w2, [x0, #16]).

CODE CAVE (SO-relative): 0x242380 (4088 zero bytes — alignment padding)

CAVE LAYOUT (6 instructions = 24 bytes):
  cave+0:  LDR w2, [x21, #16]    ; original load
  cave+4:  CBZ w3, →cave+12      ; if type==0, check value
  cave+8:  B AFTER               ; type!=0: return normally
  cave+12: CBNZ w2, →cave+20    ; if value!=0: return normally
  cave+16: MOVZ w2, #1           ; type==0 AND value==0: override to mop-only
  cave+20: B AFTER               ; return

PATCH at SO+0x2EB750:
  B cave                         ; branch to cave

Uses ptrace(PTRACE_POKETEXT) which triggers COW on r-xp pages and flushes icache.
ASLR: SO base is found dynamically from /proc/PID/maps at runtime.
"""
import os
import ctypes
import struct
import sys

# ─── SO-relative offsets (constant regardless of ASLR) ───────────────────────
SO_NAME       = b'node_porphyrion.so'
PATCH_SO_OFF  = 0x2EB750   # LDR w2, [x21, #16]
AFTER_SO_OFF  = 0x2EB754   # STR w2, [x0, #16]  (return target from cave)
CAVE_SO_OFF   = 0x242380   # 4088-byte zero block in .text

# ─── ptrace constants ─────────────────────────────────────────────────────────
PTRACE_PEEKTEXT = 1
PTRACE_POKETEXT = 4
PTRACE_ATTACH   = 16
PTRACE_DETACH   = 17

libc = ctypes.CDLL('libc.so.6', use_errno=True)
libc.ptrace.restype = ctypes.c_long   # MUST be c_long (8 bytes) on 64-bit Linux

def _ptrace(request, pid, addr, data=0):
    r = libc.ptrace(ctypes.c_int(request),
                    ctypes.c_int(pid),
                    ctypes.c_ulong(addr),
                    ctypes.c_ulong(data & 0xFFFFFFFFFFFFFFFF))
    err = ctypes.get_errno()
    if err:
        raise OSError(err, f"ptrace({request}) at 0x{addr:x}: {os.strerror(err)}")
    return r

def peek64(pid, addr):
    """Read 8 bytes from process address space via PTRACE_PEEKTEXT."""
    assert addr % 8 == 0, f"addr 0x{addr:x} must be 8-byte aligned"
    v = _ptrace(PTRACE_PEEKTEXT, pid, addr)
    return v & 0xFFFFFFFFFFFFFFFF

def poke64(pid, addr, val):
    """Write 8 bytes to process address space (triggers COW + icache flush)."""
    assert addr % 8 == 0, f"addr 0x{addr:x} must be 8-byte aligned"
    _ptrace(PTRACE_POKETEXT, pid, addr, val & 0xFFFFFFFFFFFFFFFF)

def make_B(from_addr, to_addr):
    """AArch64 B (unconditional branch, ±128MB)."""
    assert (to_addr - from_addr) % 4 == 0
    off_insns = (to_addr - from_addr) // 4
    imm26 = off_insns & 0x3FFFFFF
    return (0x14000000 | imm26) & 0xFFFFFFFF

def make_CBZ_W(Rn, from_addr, to_addr):
    """AArch64 CBZ Wn, label (branch if zero, ±1MB)."""
    off_insns = (to_addr - from_addr) // 4
    imm19 = off_insns & 0x7FFFF
    return (0x34000000 | (imm19 << 5) | (Rn & 0x1F)) & 0xFFFFFFFF

def make_CBNZ_W(Rn, from_addr, to_addr):
    """AArch64 CBNZ Wn, label (branch if non-zero, ±1MB)."""
    off_insns = (to_addr - from_addr) // 4
    imm19 = off_insns & 0x7FFFF
    return (0x35000000 | (imm19 << 5) | (Rn & 0x1F)) & 0xFFFFFFFF

LDR_W2_X21_16 = 0xb94012a2   # LDR w2, [x21, #16]
STR_W2_X0_16  = 0xb9001002   # STR w2, [x0, #16]
MOVZ_W2_1     = 0x52800022   # MOVZ w2, #1

def find_ava_pid():
    for pid_s in os.listdir('/proc'):
        if not pid_s.isdigit():
            continue
        try:
            cmd = open(f'/proc/{pid_s}/cmdline', 'rb').read().split(b'\x00')[0]
            if cmd in (b'ava', b'/ava/bin/ava') or cmd.endswith(b'/ava'):
                return int(pid_s)
        except:
            pass
    return None

def find_so_base(pid):
    """Find node_porphyrion.so load base from /proc/PID/maps."""
    with open(f'/proc/{pid}/maps') as f:
        for line in f:
            if SO_NAME.decode() in line and 'r-xp' in line and line.split()[2] == '00000000':
                return int(line.split('-')[0], 16)
    return None

def read_mem32(pid, addr):
    """Read 4 bytes via /proc/PID/mem (for verification only)."""
    with open(f'/proc/{pid}/mem', 'rb') as f:
        f.seek(addr)
        return struct.unpack('<I', f.read(4))[0]

def main():
    pid = find_ava_pid()
    if not pid:
        print('ERROR: AVA process not found')
        sys.exit(1)
    print(f'AVA PID: {pid}')

    so_base = find_so_base(pid)
    if not so_base:
        print(f'ERROR: {SO_NAME.decode()} not found in maps')
        sys.exit(1)
    print(f'node_porphyrion.so base: 0x{so_base:x}')

    PATCH_ADDR = so_base + PATCH_SO_OFF
    AFTER_ADDR = so_base + AFTER_SO_OFF
    CAVE_ADDR  = so_base + CAVE_SO_OFF
    print(f'Patch site: 0x{PATCH_ADDR:x}')
    print(f'Cave:       0x{CAVE_ADDR:x}')

    assert PATCH_ADDR % 8 == 0, 'patch addr not 8-byte aligned'
    assert CAVE_ADDR  % 8 == 0, 'cave addr not 8-byte aligned'

    # Check current state via /proc/mem (readable without ptrace)
    site_insn = read_mem32(pid, PATCH_ADDR)
    next_insn = read_mem32(pid, AFTER_ADDR)
    cave_insn = read_mem32(pid, CAVE_ADDR)
    branch_insn = make_B(PATCH_ADDR, CAVE_ADDR)

    print(f'Current at patch site: 0x{site_insn:08x}')
    print(f'Current at AFTER:      0x{next_insn:08x}')
    print(f'Current at cave:       0x{cave_insn:08x}')

    if site_insn == branch_insn and next_insn == STR_W2_X0_16:
        print('ALREADY PATCHED correctly. Nothing to do.')
        return
    if site_insn == branch_insn and next_insn != STR_W2_X0_16:
        print(f'WARNING: Branch is there but AFTER is wrong (0x{next_insn:08x}). Fixing...')
        _ptrace(PTRACE_ATTACH, pid, 0)
        os.waitpid(pid, 0)
        try:
            word = peek64(pid, PATCH_ADDR)
            new = branch_insn | (STR_W2_X0_16 << 32)
            poke64(pid, PATCH_ADDR, new)
            print(f'Fixed AFTER to 0x{STR_W2_X0_16:08x}')
        finally:
            _ptrace(PTRACE_DETACH, pid, 0)
        return

    if site_insn != LDR_W2_X21_16:
        print(f'ERROR: patch site has unexpected instruction 0x{site_insn:08x}')
        print(f'Expected LDR w2,[x21,#16] = 0x{LDR_W2_X21_16:08x}')
        sys.exit(1)
    if cave_insn != 0:
        print(f'WARNING: Cave not zero (0x{cave_insn:08x}). Might be already-patched differently.')

    print('\nBuilding code cave...')
    c0 = CAVE_ADDR
    cave_insns = [
        LDR_W2_X21_16,                              # +0: original load
        make_CBZ_W(3, c0+4, c0+12),                 # +4: CBZ w3, check_value
        make_B(c0+8, AFTER_ADDR),                   # +8: B return (type!=0)
        make_CBNZ_W(2, c0+12, c0+20),               # +12: CBNZ w2, return (value!=0)
        MOVZ_W2_1,                                   # +16: MOVZ w2, #1 (override)
        make_B(c0+20, AFTER_ADDR),                  # +20: B return
    ]
    for i, insn in enumerate(cave_insns):
        print(f'  cave+{i*4:2d} (0x{c0+i*4:x}): 0x{insn:08x}')

    print(f'\nBranch at patch site: 0x{branch_insn:08x}')

    print('\nAttaching to AVA (brief stop)...')
    _ptrace(PTRACE_ATTACH, pid, 0)
    os.waitpid(pid, 0)

    try:
        # Write cave (3 × 8-byte words)
        for i in range(0, 6, 2):
            word = cave_insns[i] | (cave_insns[i+1] << 32)
            poke64(pid, c0 + i*4, word)
        print('Cave written.')

        # Write patch: [branch_insn, STR_W2_X0_16] as one 8-byte write
        patch_word = branch_insn | (STR_W2_X0_16 << 32)
        poke64(pid, PATCH_ADDR, patch_word)
        print('Branch patched.')

    finally:
        _ptrace(PTRACE_DETACH, pid, 0)
        print('Detached — AVA resumed.')

    # Final verify via /proc/mem
    s = read_mem32(pid, PATCH_ADDR)
    a = read_mem32(pid, AFTER_ADDR)
    c = read_mem32(pid, CAVE_ADDR)
    ok = (s == branch_insn and a == STR_W2_X0_16 and c == LDR_W2_X21_16)
    print(f'\nVerification: patch=0x{s:08x} after=0x{a:08x} cave=0x{c:08x} → {"OK" if ok else "FAIL"}')
    if ok:
        print('WritePropInt(type=0, value=0) will now store value=1 (mop-only). Fan should stay off.')
    else:
        print('ERROR: Verification failed!')
        sys.exit(1)

if __name__ == '__main__':
    main()

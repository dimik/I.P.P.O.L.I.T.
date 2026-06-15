#!/usr/bin/env python3
"""
set_only_mop.py - Dual-mode fan suppressor for AVA.

Mode 1 (only_mop blackboard, 2s interval):
  Keeps AVA BT string key "only_mop"=1 so the behavior tree doesn't activate
  the fan in standard cleaning mode.

Mode 2 (CleanMode integer property, 50ms interval):
  Keeps the integer property type=0 (CleanMode) at 1 (mop-only).
  The property array is located by scanning for the pattern:
    type[1]=1 (CleanMop), type[17]=2 (CarpetPressState), type[23]=3 (StreamerSwitch).
  When piid:13 (remote control enable) is received, AVA calls WritePropInt(0,0)
  which resets CleanMode to 0 and then sends CLEANSET(fan=on) to the MCU.
  By patching CleanMode back to 1 within the BT tick boundary, we prevent the
  CLEANSET from using CleanMode=0 to activate the fan.
"""
import os
import struct
import time
import subprocess
import json
import sys

CLEANMODE_POLL_S = 0.05   # 50ms - within BT tick period
ONLYMOP_POLL_S  = 2.0

def log(msg):
    print(msg, flush=True)

def find_ava_pid():
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmdline = open(f"/proc/{pid}/cmdline", "rb").read()
            name = cmdline.split(b"\x00")[0].decode("utf-8", errors="replace")
            if name == "ava" or name.endswith("/ava"):
                return int(pid)
        except:
            pass
    return None

def find_only_mop_addr(pid):
    """Scan AVA heap for 'only_mop' blackboard entry, return value address."""
    heap_start = heap_end = None
    with open(f"/proc/{pid}/maps") as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2 and parts[0].split("-")[0] == "3293000":
                addr_range = parts[0]
                heap_start = int(addr_range.split("-")[0], 16)
                heap_end   = int(addr_range.split("-")[1], 16)
                break

    if heap_start is None:
        with open(f"/proc/{pid}/maps") as f:
            for line in f:
                if "[heap]" in line:
                    parts = line.split()
                    addr_range = parts[0]
                    heap_start = int(addr_range.split("-")[0], 16)
                    heap_end   = int(addr_range.split("-")[1], 16)
                    break

    if heap_start is None:
        log("Could not find heap")
        return None

    log(f"Heap: 0x{heap_start:08x}-0x{heap_end:08x} ({(heap_end-heap_start)//1024}KB)")

    with open(f"/proc/{pid}/mem", "rb") as mf:
        mf.seek(heap_start)
        heap = mf.read(heap_end - heap_start)

    log(f"Read {len(heap)} bytes")

    target = b"only_mop\x00"
    pool_pos = heap.find(target)
    if pool_pos < 0:
        log("Could not find 'only_mop' string in heap")
        return None
    only_mop_addr = heap_start + pool_pos
    log(f"String pool: 0x{only_mop_addr:08x}")

    target_ptr = struct.pack("<Q", only_mop_addr)
    ptr_pos = heap.find(target_ptr)
    if ptr_pos < 0:
        log("Could not find pointer to only_mop string")
        return None

    if ptr_pos + 16 > len(heap):
        return None
    key_len = struct.unpack_from("<Q", heap, ptr_pos + 8)[0]
    if key_len != 8:
        idx = ptr_pos + 1
        while True:
            ptr_pos = heap.find(target_ptr, idx)
            if ptr_pos < 0:
                log("No valid pointer found")
                return None
            if ptr_pos + 16 > len(heap):
                break
            key_len = struct.unpack_from("<Q", heap, ptr_pos + 8)[0]
            if key_len == 8:
                break
            idx = ptr_pos + 1

    val_offset = ptr_pos - 8
    if val_offset < 0:
        return None
    val_addr = heap_start + val_offset
    log(f"Blackboard entry val addr: 0x{val_addr:08x}")
    return val_addr

def find_cleanmode_addr(pid):
    """
    Find CleanMode integer property array by scanning for anchor values:
      type[1]=1 (CleanMop=1), type[17]=2 (CarpetPressState=2), type[23]=3 (StreamerSwitch=3).
    Returns address of type[0] (CleanMode), stride 4 bytes per type.
    """
    with open(f"/proc/{pid}/maps") as f:
        maps_text = f.read()

    regions = []
    for line in maps_text.split("\n"):
        parts = line.split()
        if len(parts) < 2 or "rw" not in parts[1]:
            continue
        a, b = parts[0].split("-")
        start, end = int(a, 16), int(b, 16)
        sz = end - start
        if sz < 4096 or sz > 100 * 1024 * 1024:
            continue
        if start > 0x7f0000000000:
            continue
        regions.append((start, end, sz))

    log(f"Scanning {len(regions)} regions for CleanMode array...")

    with open(f"/proc/{pid}/mem", "rb") as mf:
        for start, end, sz in regions:
            try:
                mf.seek(start)
                chunk = mf.read(sz)
            except:
                continue

            # type[17]=2 at i, type[23]=3 at i+24, type[1]=1 at i-64
            for i in range(17 * 4, len(chunk) - 24 * 4, 4):
                if struct.unpack_from("<I", chunk, i)[0] != 2:
                    continue
                if struct.unpack_from("<I", chunk, i + 24)[0] != 3:
                    continue
                base_off = i - 17 * 4
                if struct.unpack_from("<I", chunk, base_off + 4)[0] != 1:  # type[1]
                    continue
                # Verify all values are small ints (not pointers)
                plausible = True
                for j in range(40):
                    v = struct.unpack_from("<I", chunk, base_off + j * 4)[0]
                    if v > 100 and v != 0xFFFFFFFF and v != 0xFFFFFFFE:
                        plausible = False
                        break
                if not plausible:
                    continue

                base_addr = start + base_off
                vals = [struct.unpack_from("<I", chunk, base_off + j * 4)[0] for j in range(12)]
                log(f"CleanMode array found: base=0x{base_addr:08x} vals[0:12]={vals}")
                return base_addr

    log("CleanMode array not found in any region")
    return None

def read_byte(pid, addr):
    try:
        with open(f"/proc/{pid}/mem", "rb") as f:
            f.seek(addr)
            return f.read(1)[0]
    except:
        return None

def write_byte(pid, addr, val):
    try:
        with open(f"/proc/{pid}/mem", "r+b") as f:
            f.seek(addr)
            f.write(bytes([val]))
        return True
    except Exception as e:
        log(f"Write byte failed: {e}")
        return False

def read_int32(pid, addr):
    try:
        with open(f"/proc/{pid}/mem", "rb") as f:
            f.seek(addr)
            return struct.unpack("<I", f.read(4))[0]
    except:
        return None

def write_int32(pid, addr, val):
    try:
        with open(f"/proc/{pid}/mem", "r+b") as f:
            f.seek(addr)
            f.write(struct.pack("<I", val))
        return True
    except Exception as e:
        log(f"Write int32 failed: {e}")
        return False

def get_work_mode():
    try:
        cmd = '{"type":"msgCvt","cmd":"get_prop","prop":"work_mode"}'
        r = subprocess.run(["avacmd", "msg_cvt", cmd],
            capture_output=True, text=True, timeout=2)
        return json.loads(r.stdout.strip()).get("value", "?")
    except:
        return "?"

def main():
    log("set_only_mop v2: starting (dual-mode fan suppressor)")

    pid = None
    for _ in range(60):
        pid = find_ava_pid()
        if pid:
            break
        time.sleep(1)

    if not pid:
        log("ERROR: AVA not found after 60s")
        sys.exit(1)

    log(f"AVA PID: {pid}")
    time.sleep(3)

    # --- Mode 1: only_mop string blackboard ---
    om_addr = find_only_mop_addr(pid)
    if om_addr is None:
        log("WARNING: only_mop not found — blackboard patching disabled")
    else:
        cur = read_byte(pid, om_addr)
        log(f"Current only_mop=0x{cur:02x}")
        if cur != 1:
            write_byte(pid, om_addr, 1)
            log("Set only_mop=1")
        else:
            log("only_mop already true")

    # --- Mode 2: CleanMode integer property array ---
    cm_addr = find_cleanmode_addr(pid)
    if cm_addr is None:
        log("WARNING: CleanMode array not found — integer prop patching disabled")
    else:
        cur = read_int32(pid, cm_addr)
        log(f"CleanMode type[0] current value={cur}")
        if cur != 1:
            write_int32(pid, cm_addr, 1)
            log("Set CleanMode=1")
        else:
            log("CleanMode already mop-only")

    log("Monitoring loop: CleanMode@50ms, only_mop@2s...")

    check = 0
    last_pid = pid
    last_om_check = time.time()
    last_log = time.time()

    while True:
        time.sleep(CLEANMODE_POLL_S)
        check += 1
        now = time.time()

        # Detect AVA restart
        cur_pid = find_ava_pid()
        if cur_pid != last_pid:
            log(f"AVA restarted ({last_pid} -> {cur_pid}), re-scanning...")
            time.sleep(3)
            pid = cur_pid
            last_pid = cur_pid
            om_addr = find_only_mop_addr(pid)
            cm_addr = find_cleanmode_addr(pid)
            if om_addr:
                write_byte(pid, om_addr, 1)
            if cm_addr:
                write_int32(pid, cm_addr, 1)
            log(f"Re-found: om=0x{om_addr or 0:x} cm=0x{cm_addr or 0:x}")
            continue

        # Fast path: hold CleanMode=1 every 50ms
        if cm_addr is not None:
            v = read_int32(pid, cm_addr)
            if v is None:
                cm_addr = find_cleanmode_addr(pid)
            elif v != 1:
                write_int32(pid, cm_addr, 1)
                log(f"[{check}] CleanMode was {v} → reset to 1")

        # Slow path: hold only_mop=1 every 2s
        if now - last_om_check >= ONLYMOP_POLL_S:
            last_om_check = now
            if om_addr is not None:
                v = read_byte(pid, om_addr)
                if v is None:
                    om_addr = find_only_mop_addr(pid)
                elif v != 1:
                    write_byte(pid, om_addr, 1)
                    log(f"[{check}] only_mop was 0x{v:02x} → reset to 1")

        # Heartbeat every 5 minutes
        if now - last_log >= 300:
            last_log = now
            wm = get_work_mode()
            cm = read_int32(pid, cm_addr) if cm_addr else "?"
            om = read_byte(pid, om_addr) if om_addr else "?"
            log(f"[{check}] heartbeat: wm={wm} CleanMode={cm} only_mop={om}")

if __name__ == "__main__":
    main()

# USB gadget-Ethernet (robot ↔ companion over one USB cable)

**Authoritative, reproducible reference** for building the out-of-tree USB CDC-NCM/ECM
ethernet-gadget kernel modules for the Dreame D10s Pro, deploying them, and the link's
measured behaviour. If anything here drifts from reality, fix *this file*.

> **Status (2026-06): WORKING.** The gadget binds cleanly and passes IP traffic end-to-end
> (proven: ping + >1 GB transferred, 0 errors). Throughput **~11–12 MB/s** — a hard `sw_udc`
> (Allwinner UDC) ceiling, not a framing limit. Good for H.264/compressed video + ROS 2 topics;
> not raw uncompressed streams. **ECM is the preferred default over NCM** (same throughput, but
> 0.5 ms vs 2.7 ms latency — see §7). The adapter's "Micro USB VBUS" jumper is **not** required
> (link works with it open). **Boot-persistent + plug-and-play:** the gadget + a `usb0` DHCP server
> auto-start every boot (§6.1), so the companion just plugs in and gets `192.168.10.2` — no host
> config, no per-boot commands (reboot-verified).

---

## 1. The robot's kernel (the thing modules must match)

Query it directly on the robot:
```
uname -r                         # 4.9.191
cat /proc/version
#  Linux version 4.9.191 (sunfanqing@D-SUNFANQING)
#  (gcc version 6.4.1 (OpenWrt/Linaro GCC 6.4-2017.11 2017-11)) #3 SMP PREEMPT Sat May 11 2024
tr -d '\0' < /proc/device-tree/compatible    # allwinner,mr813 arm,sun50iw10p1
cat /proc/cpuinfo | grep 'CPU part'          # 0xd03  => Cortex-A53
```

| Fact | Value |
|------|-------|
| SoC | Allwinner **MR813** = platform **`sun50iw10`** (A100/A133 class) |
| CPU | quad **Cortex-A53** (`0xd03`), aarch64 — *not* Cortex-A7 |
| Kernel | **4.9.191**, `#3 SMP PREEMPT`, built 2024-05-11 with **gcc 6.4.1** by "sunfanqing" |
| Module ABI knobs | `CONFIG_MODVERSIONS` **not set** → only the **vermagic string** must match, not the compiler/CRC |
| Required vermagic | **`4.9.191 SMP preempt mod_unload aarch64`** (byte-exact) |
| Gadget core | `USB_GADGET/LIBCOMPOSITE/CONFIGFS=y`, `USB_SUNXI_UDC0=y` — built-in |
| Ethernet function | **not compiled** in the stock kernel → we supply it as modules |
| UDC name | `5100000.udc-controller` |

The robot's exact running `.config` is saved at **`kernel/config-4.9.191.txt`** (3840 lines,
header `Linux/arm64 4.9.191 Kernel Configuration`). This is the config used for the build.

Because `MODVERSIONS=n`, building with a *different* gcc (we used 7.3.0) is fine — the loader
only checks the vermagic string.

---

## 2. Why mainline-built modules crash (the core gotcha)

Building `usb_f_ncm`/`usb_f_ecm`/`u_ether` from **stock mainline 4.9.191** produces modules that
`insmod` fine but **panic the kernel the instant the gadget binds the UDC** (`echo <udc> > UDC`
→ watchdog reboot).

Root cause: the **Allwinner BSP patches the core gadget structs**, and the robot's *built-in*
composite framework was compiled *with* those extra fields. A mainline-built module has the
structs *without* them, so every field after the inserted ones sits at the wrong offset →
garbage on the first endpoint completion after bind → crash. It is **function-independent**
(ECM crashes identically to NCM) because the fault is in the shared struct ABI, not the
function code.

### The exact deltas (BSP vs mainline 4.9.191)

Discovered by diffing `HandsomeMod/linux-allwinner-4.9` (the sun50iw10 BSP) against mainline.
`f_ecm.c`, `f_ncm.c`, `u_ether.h` are **byte-identical** to mainline; only these differ:

**`include/linux/usb/gadget.h`** — extra field in `struct usb_request` (shifts `status`/`actual`):
```c
	void			*context;
	struct list_head	list;
+#if IS_ENABLED(CONFIG_USB_SUNXI_UDC0)
+	int			dma_flag;
+#endif
	int			status;
	unsigned		actual;
```
…and `gadget_is_dualspeed()` returns 1 unconditionally on SUNXI:
```c
 static inline int gadget_is_dualspeed(struct usb_gadget *g)
 {
+#if IS_ENABLED(CONFIG_USB_SUNXI_UDC0)
+	return 1;
+#else
 	return g->max_speed >= USB_SPEED_HIGH;
+#endif
 }
```

**`include/linux/usb/composite.h`** — extra field in `struct usb_function_instance`:
```c
 	struct list_head cfs_list;
 	struct usb_function_driver *fd;
+	struct usb_function *f;
 	int (*set_inst_name)(struct usb_function_instance *inst,
```

**`drivers/usb/gadget/function/u_ether.c`** — a 6-line spin-unlock reorder in `rx_submit()`
(the BSP version moves `spin_unlock_irqrestore` earlier). Functionally minor; we use the BSP
version for faithfulness.

> `drivers/usb/gadget/configfs.c` differs by ~376 lines in the BSP, **but it is built-in on the
> robot** — we do not compile it, and its struct ABI is defined by the headers above, so those
> BSP-internal changes don't affect our modules.

`CONFIG_USB_SUNXI_UDC0` is **on** on the robot, so both guarded fields are present in its
built-in framework. We force the same symbol during our build (see §4).

---

## 3. Source provenance (download once, keep)

Everything lives under **`kernel/build/`** (git-ignored — it's ~1.5 GB). To rebuild on a fresh
machine, recreate it:

| Item | Source | Local path |
|------|--------|------------|
| Kernel base | **mainline `linux-4.9.191`**, kernel.org: `https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.9.191.tar.xz` | `kernel/build/linux-4.9.191/` |
| BSP (delta source only) | GitHub **`HandsomeMod/linux-allwinner-4.9`** branch `main` (its Makefile = 4.9.118; ships `sun50iw10p1.dtsi`, `sun50iw10p1smp_defconfig`, `drivers/usb/sunxi_usb/`) | not vendored; fetch files as needed |
| Cross toolchain | **Bootlin aarch64 glibc stable 2018.11-1** (gcc 7.3.0): `https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--stable-2018.11-1.tar.bz2` | `kernel/build/aarch64--glibc--stable-2018.11-1/` |
| Robot `.config` | extracted from the robot (`/proc/config.gz`) | `kernel/config-4.9.191.txt` (committed) |

> We base on **mainline 4.9.191** (for the exact stable level / vermagic) and hand-apply the
> three BSP deltas above, rather than building the whole 4.9.118 BSP tree — this keeps the
> stable level identical to the robot while getting the struct ABI right.
>
> Note: `releases.linaro.org` is dead; use the Bootlin toolchain. The cross prefix is
> **`aarch64-linux-`** (not `aarch64-linux-gnu-`).

---

## 4. Build procedure (reproducible)

```sh
cd kernel/build/linux-4.9.191
export TC=$PWD/../aarch64--glibc--stable-2018.11-1/bin
export PATH=$TC:$PATH ARCH=arm64 CROSS_COMPILE=aarch64-linux-

# (a) start from the robot's exact config
cp ../../config-4.9.191.txt .config

# (b) apply the 3 BSP struct deltas to include/linux/usb/{gadget,composite}.h and
#     drivers/usb/gadget/function/u_ether.c  (see §2 for the exact hunks).
#     gadget.h/composite.h MUST be edited; u_ether.c can be taken from the BSP.

# (c) select the function modules. They are `select`-ed, not user-visible, so enable a parent:
#     - USB_ETH=m   -> selects USB_F_ECM=m + USB_U_ETHER=m  (gives usb_f_ecm.ko, u_ether.ko)
#     - USB_G_NCM=m -> selects USB_F_NCM=m + USB_U_ETHER=m  (gives usb_f_ncm.ko)
grep -q '^CONFIG_USB_ETH='   .config || echo CONFIG_USB_ETH=m   >> .config
grep -q '^CONFIG_USB_G_NCM=' .config || echo CONFIG_USB_G_NCM=m >> .config
make HOSTCFLAGS="-O2 -fcommon -w" olddefconfig    # propagates selects into auto.conf

# (d) build modules. KCFLAGS forces CONFIG_USB_SUNXI_UDC0 so the BSP struct fields compile in
#     (mainline has no such Kconfig symbol). HOSTCFLAGS -fcommon needed: modern host gcc
#     defaults to -fno-common which breaks the 4.9 dtc host tool. Build -j1 (modpost races).
make -j1 HOSTCFLAGS="-O2 -fcommon -w" KCFLAGS="-DCONFIG_USB_SUNXI_UDC0=1" \
     drivers/usb/gadget/function/u_ether.ko \
     drivers/usb/gadget/function/usb_f_ecm.ko \
     drivers/usb/gadget/function/usb_f_ncm.ko
```

### Verify before deploying
```sh
# vermagic must be byte-exact:
for m in u_ether usb_f_ecm usb_f_ncm; do
  strings drivers/usb/gadget/function/$m.ko | grep -m1 vermagic
done
# expect: vermagic=4.9.191 SMP preempt mod_unload aarch64

# confirm the SUNXI struct fields actually compiled in (the whole point):
GCCINC=$(aarch64-linux-gcc -print-file-name=include)
printf '#include <linux/usb/gadget.h>\n#include <linux/usb/composite.h>\nint a=sizeof(((struct usb_request*)0)->dma_flag);\nint b=sizeof(((struct usb_function_instance*)0)->f);\n' > /tmp/t.c
aarch64-linux-gcc -DCONFIG_USB_SUNXI_UDC0=1 -D__KERNEL__ -nostdinc -isystem "$GCCINC" \
  -I include -I arch/arm64/include -I arch/arm64/include/generated \
  -I include/uapi -I arch/arm64/include/uapi -I arch/arm64/include/generated/uapi -I include/generated/uapi \
  -include include/linux/kconfig.h -fsyntax-only /tmp/t.c && echo "OK: BSP fields present"
```

### NTB size (throughput tuning knob — currently 16K)
`drivers/usb/gadget/function/f_ncm.c`:
```c
#define NTB_DEFAULT_IN_SIZE	16384   /* tried 65536 -> NO throughput gain, +4ms latency */
#define NTB_OUT_SIZE		16384
#define TX_MAX_NUM_DPE		32      /* raise to 128 if NTB raised to 64K */
```
We shipped **16K** (see §7 — 64K made no difference because the cap is the UDC DMA).

---

## 5. Built artifacts (committed)

`kernel/modules/` (tracked in git; ~1.2 MB total):

| Module | sha256 | vermagic |
|--------|--------|----------|
| `u_ether.ko` | `238253495a7c…128e060` | `4.9.191 SMP preempt mod_unload aarch64` |
| `usb_f_ecm.ko` | `39ca1db3d07a…dc90a50` | same |
| `usb_f_ncm.ko` | `5c734e17f9a4…c2b5599e` (16K NTB) | same |

`kernel/modules/SHA256SUMS` holds the canonical hashes.

---

## 6. Deploy + load on the robot

Robot is reached over **WiFi** at `192.168.1.213`, key `~/.ssh/id_rsa_dreame`. busybox has **no
sftp**, so push files with `ssh 'cat > /tmp/x' < file` (scp fails).

```sh
KEY=~/.ssh/id_rsa_dreame
for f in u_ether.ko usb_f_ncm.ko usb_f_ecm.ko; do
  ssh -i $KEY root@192.168.1.213 "cat > /tmp/$f" < kernel/modules/$f
done
```

Then run **`scripts/robot/usb_ecm_gadget.sh`** (on the robot; the preferred default — see §7) — it
loads the modules, builds the ConfigFS gadget with **pinned MACs**, forces peripheral role, binds
the UDC, and sets the robot-side IP + static ARP. `usb_ecm_gadget.sh down` tears it all down. The
**`usb_ncm_gadget.sh`** variant is identical but uses the NCM function (only worth it for frame
aggregation, which doesn't help on this UDC).

Key robot-side details the script handles (and why):
- **Force peripheral role:** the sunxi OTG manager picks role by ID pin; force device mode by
  **reading** `/sys/devices/platform/soc/usbc0/usb_device` (this driver triggers the action on
  *read*, not write — reading `usb_host`/`usb_null` will flip it the wrong way).
- **Pin MACs** via `functions/ncm.usb0/{dev_addr,host_addr}` so the host interface name stays
  `enx<host_addr>` across re-enumerations (`46:bb:2c:4c:0d:4b` dev / `d6:7f:fa:3a:49:bd` host →
  host iface `enxd67ffa3a49bd`).
- **ConfigFS teardown order matters:** `rm -rf` fails ("Operation not permitted") on configfs.
  Must unlink the config symlink, then `rmdir` strings → configs → functions → strings → gadget.
- **Static ARP** (`arp -s`) — busybox `ip neigh` lacks `add`/`replace`. Needed for **NCM** (poor
  broadcast/ARP); **ECM resolves ARP normally and needs no static entry** (verified).

When run by hand, everything is **RAM-only** (modules in `/tmp`, ConfigFS volatile, IP runtime); a
reboot wipes it. For the permanent install, see §6.1.

### 6.1 Boot persistence + DHCP (the production setup) — PROVEN by reboot
The gadget is wired into the DustBuilder boot hook so it comes up automatically every boot, and a
dnsmasq makes the companion plug-and-play:

- **Persistent files on `/data`** (writable; survives reboot): `/data/usb-gadget/{u_ether,usb_f_ecm,
  usb_f_ncm}.ko` + `usb_ecm_gadget.sh` + `usb_ncm_gadget.sh`.
- **`/data/_root_postboot.sh`** (the late boot hook; no systemd — busybox `init` → `rc.sysinit` →
  `/data/_root*.sh`) brings up the ECM gadget with `MODDIR=/data/usb-gadget`, then starts a dnsmasq
  bound **only** to `usb0`:
  ```sh
  dnsmasq --conf-file=/dev/null --user=root --port=0 --interface=usb0 --bind-interfaces \
          --except-interface=lo --dhcp-authoritative \
          --dhcp-range=192.168.10.2,192.168.10.2,255.255.255.0 \
          --dhcp-leasefile=/tmp/dnsmasq-usb0.leases --pid-file=/tmp/dnsmasq-usb0.pid
  ```
  `--user=root` is required (no `nobody` user on busybox); `--port=0` = DHCP only (no DNS);
  `--bind-interfaces --interface=usb0` keeps it isolated from the WiFi-AP dnsmasq.
- **Reboot-verified (2026-06):** cold boot → in ~44 s `ecm.usb0` bound, `usb0`=192.168.10.1, dnsmasq
  serving; host got `192.168.10.2` via NM's default DHCP, ping 0.5 ms, no static config. The gadget
  block is defensive (`|| true`) so a failure can't abort the rest of postboot.
- **Caveat:** WiFi **AP mode** runs `killall -9 dnsmasq` (in `/usr/bin/wifi_start.sh`), which also
  kills this `usb0` instance — only matters if you enter AP/pairing mode.

---

## 7. Findings (measured, 2026-06)

### Throughput / latency (measured)
| Test | NCM 16K | NCM 64K | **ECM** |
|------|---------|---------|---------|
| host → robot | 11.0 MB/s | 11.3 MB/s | 11.9 MB/s |
| robot → host | 11.9 MB/s | ~same | 10.7 MB/s |
| **ping latency** | 2.7 ms | 6.7 ms | **0.51 ms** |
| 3× parallel | 10.4 MB/s (no headroom) | — | — |

**The ceiling is the Allwinner `sw_udc` DMA engine (~90 Mbit/s), not framing.** Evidence: larger
NTBs gave no gain, parallel streams gave no headroom, robot CPU ~45% idle during transfers. No
software lever moves it.

**→ ECM is the preferred default.** Throughput is a wash between ECM and NCM (all ~11–12 MB/s,
within noise), because the UDC caps *below* where NCM's aggregation would help — so NCM's only
theoretical advantage never materialises. ECM has **no NTB coalescing timer**, so its latency is
**5× lower (0.51 ms vs 2.7 ms)**, which matters for ROS control loops. NCM (16K) remains built &
available if you ever want aggregation; 64K NTB is pointless here (no gain, +4 ms latency).

### The "Micro USB VBUS" jumper — NOT required; it's the robot's host-presence sense
**The link works fine with this jumper left OPEN** — verified: multi-GB transfers + 60/60 pings
0% loss, and a full unplug/replug cycle that auto-recovered. What the jumper actually does:
connect the host's 5 V to the robot's **VBUS-sense** line, i.e. let the robot *detect* whether a
host is attached.

With it **open** (our setup), the robot has **no VBUS sense at all** — confirmed by the hot-plug
test: the UDC stays `configured` even after the cable is pulled (it never sees `not attached`). The
gadget still works because forced device-mode (§6 role-on-read) holds the **D+ pull-up always on**,
so the *host* drives enumeration whenever it's plugged in. Consequences:
- **`udc state` is NOT a "host connected?" indicator** — it reads `configured` with nothing plugged
  in. To know if a host is present, use the dnsmasq lease / ARP / a ping, not the UDC state.
- Hot-plug recovery is **host-driven** and automatic (see hot-plug note below).
- Bridging the jumper would give the robot real attach/detach detection — only worth it if some
  robot-side logic must react to host presence. Not needed for the link to function.
> Earlier notes wrongly said this jumper must be bridged (and blamed it for dropouts — those were
> NetworkManager, see below) and later that "the robot senses VBUS without it" (it doesn't —
> hot-plug test proved it). No software VBUS override exists on this firmware; none is needed.

### Hot-plug behavior (unplug/replug) — auto-recovers, host-driven (verified)
Pulling and reinserting the cable while both ends run: the robot doesn't notice (no VBUS sense,
see above), but the **host** detects its own carrier drop/return, and NetworkManager re-DHCPs —
the host gets `192.168.10.2` back and the link is up, **zero manual action**. Verified: dnsmasq
re-leased `.2`, ping 0.37 ms after replug. Combined with the reboot test (§6.1), the link survives
both robot reboots and cable yanks unattended.

### Host side — plug-and-play via DHCP (zero config; no per-boot sudo)
The robot runs a dnsmasq on `usb0` (see §6.1), so the host just needs its gadget interface under
**NetworkManager's default (managed) control** — then NM's built-in DHCP gets `192.168.10.2`
automatically on every plug-in/boot. **No static IP, no static ARP, no per-boot command.** On a
fresh Q6A this is automatic out of the box. **Verified:** robot lease file shows
`d6:7f:fa:3a:49:bd 192.168.10.2 <hostname>`, ping 0.5 ms with **no static ARP** — ECM resolves
ARP normally (the static-ARP workaround was **NCM-specific**; ECM doesn't need it).

Caveats / history:
- The earlier "dropouts" were **NetworkManager flushing a *manually-set static* IP** — an artifact
  of the pre-DHCP static approach. With DHCP, NM *is* the mechanism, so there's nothing to fight.
  (If you ever go back to static on a host, you'd `nmcli device set <if> managed no` + set it by
  hand — but with the robot serving DHCP there's no reason to.)
- If a host was previously forced to `managed no` for static testing, undo it **once** (not per
  boot): `sudo nmcli device set <if> managed yes`. NM then saves an autoconnect DHCP profile.
- *(NCM only)* host `cdc_ncm` `rx_max`/`tx_max` default 16384, clamped to the device NTB; only
  relevant if experimenting with larger NTB. (ECM has no such knobs.)

### Connection topology
Robot OTG = device/gadget; host (PC or Q6A) = USB host. Use the adapter's **micro-USB "FEL
USB"** port → host (the device-mode lines), **not** the gold "USB OTG" USB-A (that's host mode,
for plugging devices *into* the robot). USB 2.0 high-speed (480) negotiated; practical bus
ceiling ~280 Mbit/s — but we never get near it because the UDC caps first.

### Addressing
Robot `usb0` = static `192.168.10.1/24` (server). Host = `192.168.10.2/24` via **DHCP** from the
robot's `usb0` dnsmasq (§6.1). No static host config needed.

### Management over USB (SSH)
The robot's `sshd` listens on `0.0.0.0:22`, so `ssh root@192.168.10.1` works over this link — and
it's the **preferred** path when the cable's in: ~0.4 ms latency, faster/steadier bulk than WiFi,
and independent of WiFi (works even if it's down or being reconfigured). Local `~/.ssh/config`
alias convention: `dreame-usb` (192.168.10.1), `dreame-wifi` (192.168.1.213), `dreame` (AP
192.168.5.1). Note SSH's own throughput is A53-CPU-bound (it encrypts) — fine for shell/scp/ROS,
but for max bulk use a raw transport.

---

## 8. Reproduce-from-scratch checklist

1. Recreate `kernel/build/` per §3 (kernel tarball, Bootlin toolchain).
2. Build per §4; verify vermagic + BSP-field presence.
3. `cp` the three `.ko` to `kernel/modules/`, refresh `SHA256SUMS`.
4. **Permanent install:** copy the three `.ko` + both scripts to `/data/usb-gadget/`, and the
   updated `_root_postboot.sh` to `/data/` (§6.1). Reboot — gadget + `usb0` dnsmasq auto-start.
   *(Or, for a one-off RAM-only test: push to `/tmp` per §6 and run `usb_ecm_gadget.sh`.)*
5. Plug the adapter's **micro-USB (FEL)** → host (VBUS jumper can stay open).
6. Host: leave the interface under **NetworkManager (managed, default)** → it DHCPs `192.168.10.2`
   automatically. `ping 192.168.10.1`. (No static IP, no static ARP, no per-boot command.)
7. Expect ~11–12 MB/s; latency ~0.5 ms (ECM) / ~2.7 ms (NCM).

Related: `docs/hardware.md` (board/USB overview), memory `usb-gadget-ethernet-abi-fix`.

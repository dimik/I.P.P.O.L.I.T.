# USB gadget-Ethernet (robot ↔ companion over one USB cable)

**Authoritative, reproducible reference** for building the out-of-tree USB CDC-NCM/ECM
ethernet-gadget kernel modules for the Dreame D10s Pro, deploying them, and the link's
measured behaviour. If anything here drifts from reality, fix *this file*.

> **Status (2026-06): WORKING.** The gadget binds cleanly and passes IP traffic end-to-end
> (proven: ping + >1 GB transferred, 0 errors). Throughput **~11–12 MB/s** — a hard `sw_udc`
> (Allwinner UDC) ceiling, not a framing limit. Good for H.264/compressed video + ROS 2 topics;
> not raw uncompressed streams. **ECM is the preferred default over NCM** (same throughput, but
> 0.5 ms vs 2.7 ms latency — see §7). The adapter's "Micro USB VBUS" jumper is **not** required
> (link works with it open). The one host-side gotcha is **NetworkManager**, which must be told to
> leave the interface alone or it flushes the static IP (see §7).

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
- **Static ARP** (`arp -s`) — busybox `ip neigh` lacks `add`/`replace`; NCM is unreliable at
  broadcast/ARP so static entries both ends are required (see §7).

Everything is **RAM-only** (modules in `/tmp`, ConfigFS volatile, IP runtime). A reboot wipes it
all; nothing is written to eMMC. If a bind ever crashes, the watchdog reboots back to normal.

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

### The "Micro USB VBUS" jumper — NOT required (corrected)
**The link works fine with this jumper left OPEN** — verified: multi-GB transfers + 60/60 pings
0% loss over 30 s, jumper untouched. The robot senses host VBUS from the micro-USB connector
without it. (The jumper most likely gates feeding the host's 5 V to *power the board* — only
needed if running the robot off USB instead of its battery. Leave it open for normal data use.)
> Earlier notes here wrongly claimed the jumper must be bridged and blamed it for dropouts. That
> was a bad inference — see the host-side gotcha below for the actual cause. There is no software
> VBUS override on this firmware (no `vbus` param / force interface), but none is needed.

### Host-side gotchas (Linux host / Q6A) — *this* is the real reliability item
- **NetworkManager is what caused every "dropout."** It grabs the gadget interface and **flushes
  the static IP/ARP** (on each gadget re-enumeration, and on its own DHCP attempts), so the link
  goes dead while the robot still reads `configured` and the host `carrier=1`. **Fix:**
  `sudo nmcli device set <if> managed no` — after that the link is stable (proven). This is the
  single most important host step.
- **Static ARP** both ends (NCM/ECM are unreliable at broadcast/ARP):
  `sudo ip neigh replace 192.168.10.1 lladdr 46:bb:2c:4c:0d:4b dev <if>` (host) + `arp -s` (robot).
- A static IP that survives reboots: a systemd-networkd `.network` (or NM keyfile with
  `autoconnect`) for `enxd67ffa3a49bd` is cleaner than re-running `ip addr add` by hand.
- *(NCM only)* host `cdc_ncm` `rx_max`/`tx_max` default 16384, clamped to the device NTB; only
  relevant if experimenting with larger NTB. (ECM has no such knobs.)

### Connection topology
Robot OTG = device/gadget; host (PC or Q6A) = USB host. Use the adapter's **micro-USB "FEL
USB"** port → host (the device-mode lines), **not** the gold "USB OTG" USB-A (that's host mode,
for plugging devices *into* the robot). USB 2.0 high-speed (480) negotiated; practical bus
ceiling ~280 Mbit/s — but we never get near it because the UDC caps first.

### Static addressing
Robot `usb0` = `192.168.10.1/24`; host = `192.168.10.2/24`.

---

## 8. Reproduce-from-scratch checklist

1. Recreate `kernel/build/` per §3 (kernel tarball, Bootlin toolchain).
2. Build per §4; verify vermagic + BSP-field presence.
3. `cp` the three `.ko` to `kernel/modules/`, refresh `SHA256SUMS`.
4. Push to robot `/tmp` per §6.
5. Run `scripts/robot/usb_ecm_gadget.sh` on the robot (or `usb_ncm_gadget.sh` for NCM).
6. Plug the adapter's **micro-USB (FEL)** → host (VBUS jumper can stay open).
7. Host: **`nmcli device set <if> managed no`** (critical), set `192.168.10.2/24` + static ARP, `ping 192.168.10.1`.
8. Expect ~11–12 MB/s; latency ~0.5 ms (ECM) / ~2.7 ms (NCM).

Related: `docs/hardware.md` (board/USB overview), memory `usb-gadget-ethernet-abi-fix`.

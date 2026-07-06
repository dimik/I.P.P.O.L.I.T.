# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from).

---

## 2026-07-06 — Seqlock hardening on both shm channels (plan P0.1, P0.2)

**What:** Made the lock-free shared-memory handoff a *correct* seqlock on both directions.
- **Frame channel (streamer→detector):** `process()` now bumps `fseq` to **odd before** copying the RGB frame
  into shm and **even after** (was a single post-increment). Reader in `q6a_detector.py` already rejected an
  odd seq / a seq that changed mid-copy, so it now provably never consumes a torn frame.
- **Detection channel (detector→streamer):** `q6a_detector.py` now wraps the `dbuf`+`dcnt` write in the same
  odd/even `dseq` fence. Added `dseq` (offset 8) to the streamer's `DET` dict and rewrote `_read_dets()` as a
  guarded read: reject odd `dseq`, snapshot rows, re-check `dseq`; bounded 4-retry then fall back to the last
  good detection set (cosmetic overlay must never block the display path). Previously `_read_dets` read
  `dcnt`/`dbuf` with no fence and could overlay a half-written box list.

**Why:** Fable-5 finding — the frame handoff was described as a seqlock but the writer lacked the odd/even
fence, leaving a window where the reader could copy mid-write and see an unchanged seq on both sides
(torn frame). The return channel had no fence at all.

**Verify:** Deployed both files; restarted via the blessed launcher path (`setsid … &`, ssh returns
immediately — a trailing `sleep`/check in the *same* ssh session SIGHUPs the child). `curl /stream` →
**98 JPEG frames in ~6 s (~16 fps)**, no crash, zero `infer error`, board 58–65 °C. Both seqs settle even at
rest. **Note:** attaching to the shm from an ad-hoc Python probe *unlinked* the segment on exit
(Py3.12 `resource_tracker` leaked-object cleanup) — this is exactly the P0.4 hazard; the upcoming detector
step will attach with tracking disabled. Files: `q6a_camstream.py`, `q6a_detector.py`.

---

## 2026-07-06 — Fix wrong planning constants + stale model label (plan P1.6, P0.10)

**What:** (1) CLAUDE.md RAM `up to 16GB` → **12GB on this board** (11.5GB usable, avail ~2.8GB) with the
correct DDR figures (~22 GB/s theoretical / ~15 GB/s practical, not 40-50); fixed the LLM-section bandwidth
claim too. (2) `q6a_yolo.py` `QNNContext("yolov11_det", …)` → `"yolov8_det"` — the deployed model is YOLOv8
(v11 doesn't run on v68), so logs no longer mislabel it.

**Why:** Fable-5 review verified against `free` (11558 MB) and single-core memcpy — the 16GB/40-50GB/s figures
were wrong and would mis-size the LLM+Nav2+2nd-cam budgets. The `yolov11_det` string was a copy-paste label,
not the model.

**Verify:** `free` confirms 12GB; label is cosmetic (QNN context name), takes effect on next detector restart.
Files: `CLAUDE.md`, `scripts/companion/camera/q6a_yolo.py`.

---

## 2026-07-06 — Start the review-driven improvement batch; add this changelog

**What:** Established `CHANGELOG.md` and kicked off the architecture-improvement batch derived from the
Fable-5 review (`docs/q6a-pipeline-review-findings.md`) and plan (`docs/q6a-pipeline-improvement-plan.md`).

**Why:** The review was verified against the live board + source (every spot-checked code claim held), so the
plan's verified items are worth executing. Binding constraint is thermal + DDR bandwidth (~1.5 W passive
headroom), not compute — so the batch prioritizes (a) correctness/safety prerequisites and (b) changes that
*reduce* load (ISP-at-detector-cadence, w8a8 YOLO).

**Baseline (measured, this session):** `--gpu --bin --awb`, 1 client → ~16 fps publish, YOLO ~10 Hz (38–44 ms
infer incl. 5–15 ms float-I/O quantize), ~1.0 CPU core total, GPU pinned at 315 MHz, board 61 °C idle /
72–78 °C active (19 h uptime, bench-side). RAM **12 GB** (not 16), avail ~2.8 GB.

**Planned order (each = one commit):** constants+label → seqlock → MJPEG timeout → EAGAIN re-raise →
--fast/frame-assert → detector supervision → thermal governor → ISP-at-detector-cadence → w8a8 YOLO.

Files: `CHANGELOG.md` (new). Commit: _this_.

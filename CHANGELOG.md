# Changelog

Human-readable record of what changed and why. Newest first. Driving docs:
`docs/q6a-pipeline-improvement-plan.md` (the plan), `docs/q6a-pipeline-review-findings.md` (the Fable-5 review
it derives from).

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

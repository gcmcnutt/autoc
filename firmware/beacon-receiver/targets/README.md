# Printable bench targets — offline ground truth for the tracker

Regenerate with `./gen_targets.py` (stdlib only — no OpenCV, no PIL). The checked-in PNGs are what that
script emits; both were verified with a real detector (OpenCV 5.0.0): all four ArUco ids read back
correctly and `findChessboardCorners(7,10)` returns all 70 corners.

> ⚠️ **PRINT AT 100 % / ACTUAL SIZE.** "Fit to page" silently rescales, and every measurement downstream
> inherits the error. Print, then measure a feature with a ruler and confirm before trusting a run.

## What these are for

The tracker's own output cannot measure the tracker. `tools/oracle.py` re-derives beacon position from the
code, but that is a *static* matched filter and it fails in exactly the fast-motion case we most need to
measure (results/stage1-pan1-analysis.md: coherence dies at 1–3 °/s) — so ground truth depended on the very
algorithm it was meant to score. **Fiducials break that circle**: pose becomes geometric and per-frame,
independent of both the code and the tracker.

**They cannot corrupt the receiver, and that is checked in code, not assumed.** A constant-brightness object
is invisible to both detection paths:
- `corr.c` removes the per-chip-bin mean (`dev[k] -= mean_q8`), so a constant source has zero deviation in
  every bin ⇒ **exactly zero correlation at every phase**, not merely a small one.
- `acquire_pass()` is a temporal-difference blink detector ⇒ a constant source produces no diff at all.

They *will* show up in the `--field-map` viewfinder (a spatial-contrast map), which is a bonus: free aiming
references.

## The two targets do different jobs

### `aruco_4x4_50_id{0..3}_160mm.png` — per-frame pose (the main job)

Four `DICT_4X4_50` markers, 160 mm each, one per A4. Mount them rigidly around the emitter and measure the
spacing once; that geometry is the ground truth.

**Why ArUco rather than QR or a checkerboard.** QR is built for data density and wants the whole symbol; a
checkerboard must be *entirely* visible. During a pan, targets leave the field — that is the normal case
here, not an edge case. Four independently-identified tags keep pose from whichever remain, and one tag
alone still gives four corners, enough for a homography.

**Sizing is forced by the optics.** 0.152 °/native px = 2.65 mrad, so a feature of size S at range d spans
`px = S / (d × 0.00265)`. ArUco wants ≥ ~40 px across the black square ⇒ 21 cm at 2 m. On this ultra-wide
lens targets must be big or close; 160 mm on A4 is the practical compromise. At 1.5–2.5 m they read well.

### `checkerboard_8x11sq_20mm_inner7x10.png` — lens calibration (a different job)

**This one matters more than it looks.** Every °/s in this feature — including the measured 1–3 °/s
acquisition knee — assumes a **uniform 0.304 °/M2 px across a 97.3° field**. On a 1.56 mm ultra-wide lens
that is an approximation which degrades toward the edge, so a rate measured at the corner is not strictly
comparable to one on axis. One static calibration pass quantifies it. Spec §3.2 records this as owed before
the envelope campaign publishes absolute °/s.

The 7×10 inner-corner grid is deliberately **asymmetric** (odd × even): a symmetric grid leaves a 180°
orientation flip the calibrator cannot resolve.

## ⚠️ The gotcha that will bite first: exposure

The rig runs at **53 µs / gain 2.0**, tuned so a bright IR LED sits in band. **Paper under room light at
53 µs is black.** A fiducial pass needs exposure raised toward `exposure_max_us = 3000` (the frame period at
288 fps is 3474 µs, so ~3 ms is the ceiling anyway) *and* the sheets well lit. The beacon saturates at that
exposure, which is fine and already handled — spec §5's flat-top estimator engages, and the code still
decodes because lit/dark is binary.

## After the IR filter

Printed sheets are a **visible-band, short-term** fixture. Once the 1.56 mm lens and IR filter go on, paper
stops being illuminated in-band and these become useless. The replacement is the same idea in the right
band: **constant-on 850 nm LEDs** in a known geometry — same invisibility argument (constant ⇒ zero
correlation, zero temporal diff), no illumination problem, and no print-scale error.

## Second use worth taking

Fiducial-derived **ego-motion** is a bench stand-in for AHRS feed-forward. Spec §2.3 wants feed-forward
built last and measured, not assumed; markers let its value be measured **offline, with no flight hardware**,
before 043 commits to any of it.

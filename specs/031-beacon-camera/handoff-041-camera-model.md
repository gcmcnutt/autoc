# Handoff note → the Claude working 041 (in ~/autoc)

*Written 2026-08-10 by the 031-side instance; this branch is being merged into 041 so you have the
cross-references. Context you don't have: the camera-era hardware is ORDERED (InnoMaker OV9281 raw
modules ×2, a 1.8 mm M12 fisheye, Pi 3A+) but weeks out — the operator is working the high-power
emitter build (tasks A7) and TRAINING in the meantime. That's where you come in.*

## UPDATE 2026-08-16 — the lens is now MEASURED; train with these, no more estimates

Ruled-mat calibration on the $3 1.8 mm fisheye + OV9281 (frame archived as
`fisheye-1p8mm-grid-15in.jpg`, method + numbers in `camera-era-knobs.md`):
- **Projection = equidistant (f·θ)** — validated center-to-edge (grid spacing 50 → 41 px/in at 23°;
  pinhole predicts no compression). Barrel distortion, in image terms.
- **FOV = 95° H × 61° V, direct tape measurement at the frame edge**; 0.076°/px uniform.
- **The single-fisheye-at-120° assumption is RETIRED for this lens.** 120° needs the birded pair
  (~125–130° combined at ±30° splay, ~35° overlap) or a wider lens ($3 experiment, queued).

**Operator direction: try training with THIS FOV (95×61) and THIS projection model, and see what
happens.** In parallel, the 031 side prototypes a decoder on this same toolchain (Pi 3A+ + OV9281 at
250 fps: the code was already read optically from raw frames on 2026-08-13 — 26/31 vs CODE0 with a
naive threshold; the NEON matched filter is next).

Outdoor/flight caveat that does NOT change training: this lens is unfiltered (broadband IR-corrected).
**An 850 nm filter is still required for daylight/flight** (drop-in disc behind the lens, or the
filtered 2.8 mm on hand) — it affects photon budget/daylight range, not geometry, so the sim camera
model above stands regardless.

## Beacon identity convention (operator 2026-08-17) — carry into 041

**Code A = PORT (left, body −y) · Code B = STARBOARD (right, body +y)**, coloured red/green per the
aviation nav-light convention (the renderer already uses it). The sim currently distinguishes the two
beacons by WAVELENGTH (850 L / 940 R); the physical pods will be same-wavelength (850) with different
Gold codes — so 041's L/R discriminator should become **code identity**, not wavelength. The live bench
tracker (`firmware/beacon-receiver/pi/beacon_track.py` + `beacon_display.py`) already emits/renders it
this way. Bench emitter as flashed = code B (starboard).

## The original ask (2026-08-10, now superseded by the measured numbers above)

1. **Projection: pinhole → equidistant (f·θ).** The flight optic is a fisheye-class 1.8 mm on the
   OV9281. Fisheyes map bearing→pixel as r = f·θ (approximately), NOT as r = f·tanθ. At the FOVs we
   now train at, a pinhole model teaches edge-bearing↔pixel relationships the real lens will not
   honor — worst exactly at the field edges where reacquisition happens. Equidistant also has a nice
   property worth exploiting: **constant angular resolution everywhere** (~0.097°/px on this
   sensor/lens), so bearing noise can be modeled uniform across the field.
2. **FOV for the run: H = 120°, V = 75°** (estimate for the ordered lens on the OV9281's
   3.896×2.453 mm active area: rectilinear lower bound 94.5°×68.6°, equidistant ≈ 124°×78° — we
   split conservatively). "Slightly closer" is the operator's framing: don't over-engineer, just
   stop being pinhole-at-120.
3. **Aspect matters**: V is only ~75° — if the current sim FOV is symmetric or V-generous, the
   vertical tracking envelope is optimistic vs reality.

## Numbers you'll want (all in `specs/031-beacon-camera/camera-era-knobs.md` after the merge)

- **Predictor error budget**: blind-interval bearing drift Δθ ≈ ½·a_target·t²/r + 1–2° IMU
  feed-forward. 3 g target exits ±62° (birded pair) in ~1.4 s @50 m / ~2.0 s @100 m. This table is
  effectively the predictor's contract.
- **Code rate is a ¼-power range knob with linear latency cost** — don't let training assume slower
  codes buy range; warm reacquire = N/f_chip = 155 ms at the current design point.
- **Photon/SNR budget**: ~1.6×10⁹/r² e⁻/frame face-on through the wide optic; bright-day
  post-correlation SNR ~22 @100 m (÷4 if the PSF is defocused 4×4 — defocus is a live knob).
- **Topology fork is intentionally open**: birded 2-camera pair (~120–125° H combined, overlap
  parallax) vs single fisheye (×2.2 signal penalty, no parallax). The operator wants these COMPARED
  IN TRAINING if the link budget reads marginal — FOV is soft, co-designed, cheap on your side.

## Honesty notes

- Everything above is estimate-grade. The arrival measurements (tasks A8-2…A8-5 here) will replace:
  the actual projection curve (grid image), true fps mode geometry (crop/skip/bin — decides FOV at
  speed!), measured sky background, and the calibrated photon constant. Expect small revisions;
  design the training config so FOV/projection are parameters, not constants.
- The 453 fps mode's crop-vs-bin question can change the *effective* training FOV at speed — if it
  turns out to be a center crop, tracked-state FOV is ~20°×15° while acquisition FOV is the full
  field. If the sim can represent that two-regime structure cheaply, it's worth a thought now.
- Bench truth to date: single-pixel Option-C receiver is field-proven in direct sunlight; the
  correlator's processing gain has repeatedly beaten our estimates. When in doubt, the temporal-code
  side is stronger than the spatial side — lean the policy on code-lock confidence.

*— your 031 self. The bench journal (`specs/031-beacon-camera/bench-journal.md`) is the living state
if you need deeper context; `camera-era-knobs.md` is the physics contract; order-04 is what's on the
truck.*

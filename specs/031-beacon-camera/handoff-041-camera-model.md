# Handoff note → the Claude working 041 (in ~/autoc)

*Written 2026-08-10 by the 031-side instance; this branch is being merged into 041 so you have the
cross-references. Context you don't have: the camera-era hardware is ORDERED (InnoMaker OV9281 raw
modules ×2, a 1.8 mm M12 fisheye, Pi 3A+) but weeks out — the operator is working the high-power
emitter build (tasks A7) and TRAINING in the meantime. That's where you come in.*

## The ask: nudge the sim camera model closer to the real lens before the next long training run

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

# Camera-era knob framework — photon budget · code rate · reacquire · FOV

**Status**: seeded 2026-08-09 from the order-04 lens/architecture discussion; **destined for the 041 spec**
(predictor / reacquisition, developed in ~/autoc) — migrate these numbers there when that spec forms.
Everything here is *for-now assumptions + scaling laws*; the plan of record is to get empirical as the
Pi + OV9281 rig comes up (order-04).

## Architecture (operator 2026-08-09)

- **Two cameras oriented OUTWARD, bird-style**, with a central overlap zone: two 2.8 mm (~72° H each)
  splayed ~±26° → **~120–125° combined H field** with a ~20° central overlap band.
  - Overlap gives **built-in parallax ranging**: disparity = baseline/r. At b ≈ 0.2 m: 10 px @20 m
    (range to ~±10 %), 4 px @50 m (~±25 %), 2 px @100 m (coarse). Weak far, strong exactly where the
    endgame needs it. Overlap also doubles the photon budget and cross-validates code locks.
  - Hardware note: Pi Zero 2 W = one CSI port → two-camera flight = **two Zeros** (11 g each) or Pi 5
    (dual CSI).
- **The 16 mm 850 nm lens is NOT a flight camera** — it exists to approximate what the 1-pixel PD can do
  (link-budget instrument). Flight optics are the wide family.
- **Predictor (041) = the educated guess during occlusion against a maneuvering (offense/defense)
  target** — it points the reacquisition, the code flywheel + warm relock close it.

## For-now assumptions

| Assumption | Value | Source |
|---|---|---|
| Emitter | flight cube, 5× L1IZ-0850 @ **306 mA**, co-aimed | order-03 §C build (tasks A7) |
| Radiant intensity | **~0.53 W/sr face-on** (5 × ~105 mW/sr); worst single-die aspect ~0.11 W/sr | DS190 scaling from the measured bench curve |
| Camera | OV9281, 3 µm px, QE ≈ 0.38 @ 850 nm, read noise ~3 e⁻ | datasheet class |
| Frame rate / exposure | 453 fps / ≤2.2 ms | InnoMaker documented mode |
| Wide pupil | 2.8 mm F/2.2 → **1.27 mm** | lens drawing |
| Code | N=31 Gold @ ~189 Hz chips (453/2.4) | design point |

## Scaling laws (the knobs and their exponents)

**1. Photon budget (signal per frame on the beacon pixel):**
N_e ≈ I_emit · A_pupil · t_exp · (λ/hc) · QE / r² ≈ **2.4×10⁹ / r² electrons** (face-on cube, wide lens).

| r | e⁻/frame | vs 3 e⁻ read noise |
|---|---|---|
| 30 m | ~2700 | huge |
| 50 m | ~960 | huge |
| 100 m | ~240 | 80× |
| 150 m | ~110 | 36× |

The **small pixel is the camera's secret weapon**: per-pixel sky background ∝ (pixel IFOV)² — at
0.056°/px the sky patch behind the beacon is tiny, so even the 1.27 mm pupil gives workable daylight
contrast (this is the "spatial ambient rejection" the optics record claims, quantified). The real
daylight competitor is background shot noise per (binned) pixel through the 40 nm filter — **measure it,
first lensed-camera session** (it sets the true range floor, not read noise).
Range scaling: **r_max ∝ D_pupil · √I_emit** at fixed floor — pupil and emitter current are the strong
knobs (linear and square-root).

**2. Code rate ↔ reacquire time — the surprising weak lever:**
- Warm reacquire ≈ 1 word = N/f_chip; cold ≈ 2–4 words. At N=31: **200 Hz → 155 ms · 150 Hz → 207 ms ·
  100 Hz → 310 ms.**
- Slowing the code integrates longer per word, but correlation SNR grows only as √(frames) → detection
  range ∝ **(word time)^¼**. Halving the chip rate buys **×1.19 range** and costs **×2 reacquire
  latency**. **Conclusion: code rate is a ¼-power range knob with a linear latency price — keep it
  fast (189–200 Hz) and buy range with pupil/emitter/integration-across-words instead.**

**3. FOV ↔ percentage-in-field (the predictor's error budget):**
Blind-interval bearing growth Δθ ≈ ½·a_target·t²/r + ~1–2° IMU feed-forward error. Time for a 3 g
target to exit the half-field:

| half-field | r=50 m | r=100 m |
|---|---|---|
| ±7° (16 mm class) | ~0.5 s | ~0.7 s |
| ±36° (single wide) | ~1.1 s | ~1.5 s |
| ±62° (birded pair) | ~1.4 s | ~2.0 s |

Blind-time budget ∝ √(half-field) — the birded pair nearly **triples** the narrow camera's occlusion
tolerance, on top of removing most cold-acquisition pointing anxiety. Beyond the budget, the predictor's
job is exactly the operator's framing: an educated guess at which way the fight went; scan-acquire
(sweep ≤ ~FOV per code word ≈ 60–90°/s with coarse binning) is the backstop.

**4. Sampling-mode interaction** (from order-04 verify (c)): during search/scan use **bin** (gap-free,
photon-preserving); a skip mode makes a sub-pixel beacon blink as it crosses the sampling grid — only
usable with PSF deliberately defocused past the skip pitch. Slight defocus (2–4 px PSF) is desirable
anyway for centroid interpolation.

## Empirical backlog (what the Pi rig measures, in order)

1. Daylight sky background e⁻/px/frame through the 40 nm filter (bin and full-res) — sets the real floor.
2. Achieved fps + mode geometry (crop/skip/bin) of the 453 fps mode — registers, not the marketing page.
3. Beacon e⁻/frame vs range, bare cube — calibrates the 2.4×10⁹/r² constant.
4. Warm/cold reacquire wall-clock through the NEON correlator vs the N/f_chip prediction.
5. Parallax disparity noise vs range on the birded pair (b measured) — the endgame ranging curve.

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
N_e ≈ I_emit · A_pupil · t_exp · (λ/hc) · QE · **T_optics** / r² — with **T_optics ≈ 0.65** ≈
**1.6×10⁹ / r² electrons** (face-on cube, wide lens, 2.2 ms @ 453 fps).
**T_optics decomposition (reconciled 2026-08-10 — "0.88 on the lens page" vs 0.65 here are different
scopes)**: lens stack ≈ 0.85–0.88 (the datasheet number) × filter peak ≈ 0.86 × **spectral overlap of
the LED's 30 nm line inside the ~40 nm passband ≈ 0.85–0.9 (on no datasheet)** → chain ≈ 0.62–0.68.
Separately, the drawing's **Relative Illumination = 52 %** is a FOV-dependent derate: corner-of-field
beacons get ~half the center photons — on-axis budget × RI(θ) off-axis. Bird-pair bonus: the central
overlap is each camera's edge region, but BOTH see it → the pair sums back to ~parity at the seam.
The static range test measures the whole product (empirical #3) — these factors then stop being
estimates.

| r | e⁻/frame | dark/indoor per-frame SNR (shot + 3 e⁻ read) | bright-day per-frame SNR (sky ~4000 e⁻/px est. → σ≈63) | post-correlation ×√74, bright day |
|---|---|---|---|---|
| 30 m | ~1800 | ~42 | ~28 | ~240 |
| 50 m | ~640 | ~25 | ~10 | ~87 |
| 100 m | ~160 | ~12 | ~2.5 | ~22 |
| 150 m | ~71 | ~8 | ~1.1 | ~10 |

vs the ~13 dB (×4.5) lock threshold → **bright-day lock pencils to ~200 m-class even through the wide
pupil** — BUT the sky-background estimate is the least-trusted number here (±3×) and sits near the
OV9281's ~10 ke⁻ full well at 2.2 ms: **manual exposure is load-bearing** (bright day = shorter exposure
= √t SNR cost, well depth is the cap). Read noise is irrelevant inside ~300 m. Empirical item #1
calibrates this entire column with one frame of filtered daytime sky.

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

**4. Sampling-mode + defocus interaction** (from order-04 verify (c); DERATE ADDED 2026-08-09): during
search/scan use **bin** (gap-free, photon-preserving); skip modes make a sub-pixel beacon blink — only
usable with PSF defocused past the skip pitch. Defocus (desired for centroiding anyway) has an SNR
price: spreading the spot over k pixels leaves total signal intact but the spatial sum (or the k-bin)
takes √k more background noise → **background-limited SNR ÷√k. At the 16-px (4×4) defocus: ÷4** —
the bright-day post-correlation column drops 22→~5.5 @100 m (vs the ×4.5 threshold) → **bright-day
range with 4×4 defocus ≈ 100–110 m-class**, dark barely affected (shot-limited: ~12→9 @100 m).
Defocus is a knob: a 2×2-px PSF (÷2) may satisfy centroiding + skip-bridging at half the cost —
optimize live against corrB + centroid jitter.

**5. Filter FWHM (10 vs 40 nm) in the whole budget — the LED linewidth decides it:**
The LED line is **~30 nm FWHM**, so a 10 nm filter passes only ~30 % of OUR OWN signal (×0.3) vs
~85 % for 40 nm. Two regimes:
- **Exposure-time-limited** (the 453 fps case — sky ~40 % of well at 2.2 ms): SNR ∝ T_sig/√BW →
  **40 nm wins ×1.4**.
- **Well-depth-limited** (sky so bright that exposure must shrink to protect the well): t ∝ 1/BW →
  SNR ∝ T_sig/BW → **10 nm wins ×1.4**. Only reachable on very bright days or slower frame modes.
- **AoI disqualifier for wide optics**: CRA ~10° blue-shifts a dielectric stack ~4–5 nm — HALF a 10 nm
  passband — so an edge-of-field beacon slides off the filter. 10 nm is narrow-optics-only regardless.
- **Market reality (searched 2026-08-09)**: integrated-10 nm M12 lens SKUs effectively don't exist —
  10 nm FWHM is the laser/VCSEL isolation market, sold as loose discs (Amazon/SyronOptics); the
  LED-illumination lens market ships 30–60 nm ("narrow" board-lens filters spec ≈ 850±10 CWL, 30 nm
  HBW, T≈86 % — cctvopticallens/TowinLens class).
**Verdict: the wide pair stays 30–40 nm class (it is near-OPTIMAL — matched to the LED linewidth);
shelve 10 nm unless a well-depth-limited regime emerges, and then narrow-FOV optics only.**

**6. FOV is CO-DESIGNED with training (operator 2026-08-09; decision SOFT)**: the current sim training
assumes **FOV-H = 120°**, and the flight optics must realize whatever the policy is trained on — but
the coupling runs both ways: **training is the cheap side of the trade**, so if the link budget lands
on the fence at 120°, alternate topologies (narrower single camera + predictor-heavier policy, other
splay angles, asymmetric pairs) can be TRAINED AND COMPARED in sim before any glass is bought. For the
current 120° assumption: the birded pair (2×72° − overlap ≈ 120–125°) hits it while keeping each
camera's 1.27 mm pupil and 0.056°/px sky patch; the single-lens alternative (f ≈ 1.1 mm @ F/2.2 →
0.50 mm pupil) costs **×6.5 in signal** and coarsens the sky patch ×2.8 — the bird geometry is the
link-budget answer *to 120° specifically*. If static range testing says even the pair is marginal,
the FOV knob re-opens via retraining, not via optics heroics.

## Empirical backlog (what the Pi rig measures, in order)

1. Daylight sky background e⁻/px/frame through the 40 nm filter (bin and full-res) — sets the real floor.
2. Achieved fps + mode geometry (crop/skip/bin) of the 453 fps mode — registers, not the marketing page.
3. Beacon e⁻/frame vs range, bare cube — calibrates the 2.4×10⁹/r² constant.
4. Warm/cold reacquire wall-clock through the NEON correlator vs the N/f_chip prediction.
5. Parallax disparity noise vs range on the birded pair (b measured) — the endgame ranging curve.

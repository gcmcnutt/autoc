# Camera Considerations

> **MOVED 2026-06-22 from `specs/031-beacon-camera/`. Parked 040 camera-redo reference — predates the 031 1-bit phase and is likely stale; re-validate (20 Hz / 480 fps / 200 Hz / 75 ms baseline + 031 field findings) on 040 restart. See [`README.md`](README.md).**

Sensor selection and link-budget notes for the LED-beacon tracking system.

## Operating Assumptions

| Parameter | Value |
|---|---|
| Camera resolution | **320×240 (QVGA)** baseline |
| Frame rate | **240 fps** baseline (480 fps reserved for special cases) |
| Wavelength | 850 nm (60% sensor QE; faintly visible red glow) |
| Global shutter | Required — no rolling shutter |
| Total camera mass | 1–2 g (flight hardware) |
| Beacons on target | 4× wide-angle LEDs on wing tops, pyramidal arrangement |
| Detection range | Up to **100 m** |
| Position update rate | 10 Hz minimum, 20 Hz desired |
| Acquisition time | <100 ms preferred for cold acquisition (edge-of-FOV entry); slight tolerance allowed |

## Top Sensor Recommendations

### OmniVision OG0VA — the standout for this use case

Purpose-built for what we're doing.

- **640×480 @ 240 fps**, **320×240 @ 480 fps** — covers both operating modes
- 1/10-inch optical format — industry's smallest at release
- **QE: 40% @ 940 nm, 60% @ 850 nm** — excellent NIR response, lowers LED power and extends beacon battery life
- Global shutter
- **OC0VA CameraCubeChip variant**: sensor + ISP + optics in a **2.69 × 3.04 × 3.04 mm** wafer-level module — well under 1 g

**Catch:** OEM channel only. Need to go through a design house or buy an eval module.
This is what drone-swarm and AR-glasses people use.

### OmniVision OG01A — the easier-to-buy step up

- 1 MP @ 120 fps, **640×480 @ 240 fps**, can window down to 320×240 for higher rates
- 1/5-inch format, global shutter, BSI pixel
- **Same 40%/60% QE @ 940/850 nm** as OG0VA
- Modules from Arducam, Leopard, etc. typically come in at **3–8 g** with lens + PCB
- Strip down to bare sensor + flex + minimal lens to hit 1–2 g target

### SmartSens SC031GS — cheap and available

- 640×480 @ 240 fps (non-HDR), global shutter, monochrome
- HDR-capable at 120 fps
- Bare die is light; full modules **2–5 g**
- IR sensitivity decent but not class-leading like the OG-series

### Sony IMX287 — industrial-grade option

- 728×544, global shutter, 240+ fps at full res
- Good NIR response (~30% @ 850 nm)
- Pregius-S generation — best image quality of the bunch
- Typically appears in heavier industrial modules (**10+ g**) — custom carrier required

## Sensors to Skip

| Sensor | Why not |
|---|---|
| Sony IMX296 | Tops out ~226 fps, only at reduced ROI — too slow at full frame |
| OnSemi AR0144 | Typically 60 fps at full res, ~120 fps with ROI — not fast enough |
| Raspberry Pi GS Camera (IMX296) | Same speed limit, heavy lens mount |

## Mass Budget Reality Check

Almost nothing off-the-shelf hits 1–2 g with a lens. Realistic paths:

1. **Bare sensor on flex PCB** — OG0VA die is sub-100 mg; with a wafer-level lens still well under 1 g. **Most realistic path.**
2. **Stripped-down OG01A module** — remove C/CS mount, use M6/M8 lens or wafer-level optics, custom flex → achievable at 1.5–2 g
3. **OC0VA CameraCubeChip** at 2.69 × 3.04 × 3.04 mm — essentially a finished tiny camera, drop-in solution

## Optical System

### Lens choice

Aperture/focal-length tradeoff at 320×240 with 3 µm pixels:

| Aperture | Focal length | f/# | iFOV | FOV (320 px) | Pixels per m @ 100 m |
|---|---|---|---|---|---|
| 4 mm | 8 mm | f/2.0 | 375 µrad | 6.9° | 27 |
| 6 mm | 12 mm | f/2.0 | 250 µrad | 4.6° | 40 |
| **8 mm** | **16 mm** | **f/2.0** | **188 µrad** | **3.4°** | **53** |
| 12 mm | 24 mm | f/2.0 | 125 µrad | 2.3° | 80 |

**Recommended: 8 mm aperture / 16 mm focal length / f/2.0.** Balances detection range, FOV, and lens mass.
At 100 m, beacon is a sub-pixel point source spread by diffraction/defocus over ~3 pixels — ideal for centroid sub-pixel interpolation.

NIR-coated, no IR-cut filter — let the IR through. Mass under 0.5 g achievable with wafer-level or M6 lenses.

### LED wavelength

| Wavelength | QE (OmniVision) | Visibility | Best for |
|---|---|---|---|
| **850 nm** | **60%** | Faint red glow | Mass-constrained flight (every photon counts) |
| 940 nm | 40% | Fully invisible | Covert operations |

**Default recommendation: 850 nm.** ~1.5× QE advantage over 940 nm directly reduces LED power and beacon mass.

### Bandpass filter

- Narrow (~30 nm) bandpass over lens, centered on 850 nm
- Dramatically improves ambient rejection — critical for outdoor daytime tracking
- ~70% transmission typical
- Adds <0.1 g

### LED beacon assumption

- 1 W electrical → ~370 mW optical (single LED, 850 nm)
- 150° wide-angle radiation pattern (≈4.7 sr cone)
- 4 LEDs around pyramidal wing tops; typically 1–2 visible to chase camera at any given aspect

## Link Budget at 100 m (Daytime)

Photon budget with 8 mm f/2 lens, 30 nm bandpass, OG-series sensor, daytime sky background:

| Frame rate | Exposure | Per-frame SNR | Post-correlation SNR (N=15) | Margin vs lock threshold |
|---|---|---|---|---|
| 240 fps | 2.08 ms | ~44 dB | **~56 dB** | ~43 dB |
| 480 fps | 1.04 ms | ~41 dB | **~53 dB** | ~40 dB |

**100× derated** (haze + dirty optics + off-axis LED + partial occlusion):

| Frame rate | Post-correlation SNR (N=15) |
|---|---|
| 240 fps | ~52 dB |
| 480 fps | ~49 dB |

### Key insight: we are saturation-limited, not noise-limited

Inside ~30 m, the LED is bright enough to overfill the pixel well in any reasonable exposure. The system runs on "is the pixel hot?" not photometry. Throttle exposure or sync LED duty cycle to manage this.

Beyond ~30 m, 1/R² rolloff begins. At 100 m we have ~40 dB margin to lock threshold (~13 dB post-correlation SNR for clean detection). **Range is comfortable.**

### Likely real-world loss mechanisms (not in the photon budget)

The model doesn't capture, and we should plan ~30 dB of margin against:

- **Atmospheric scintillation/turbulence** — blob spreads across more pixels (still detectable, dimmer per pixel)
- **Direct sun in FOV** — saturates whole sensor; no filter helps with in-band sunlight
- **Beacon aspect angle** — 150° cone has steep edge falloff; off-axis can cost 20 dB easily
- **Airframe occlusion** during banking maneuvers
- **Window/canopy losses** if camera is behind glass

Even with all of these, 240 fps gives us ~10 dB of remaining margin at 100 m. **Confirms 240 fps as baseline.**

## Coding & Acquisition Strategy

### Base code: N = 15 Gold sequence

- **Acquisition time at 240 fps:** 15 × 4.17 ms = **63 ms** ✓ (under 100 ms)
- **Acquisition time at 480 fps:** 15 × 2.08 ms = **31 ms**
- Family of 17 codes available — plenty for 2 beacons (or future expansion)
- Cross-correlation bounded by 9 (vs peak 15) — clean separation between codes

### Acquisition error tolerance (cold lock)

Worst-case bit-error capacity vs code length:

| Code length | Margin @ 0 errors | Max correctable errors (worst-case) |
|---|---|---|
| N = 7 | 2 | **0** — single error can cause wrong-phase or wrong-ID lock |
| N = 15 | 8 | **1** |
| N = 31 | 22 | **5** |

**N = 7 is unsuitable for noisy environments.** N = 15 gives 1-bit margin in the worst case — workable given our huge SNR budget, but no headroom for surprises.

### Tracking error tolerance (maintained lock)

Once locked, the problem changes — we're testing one hypothesis (still the same code at the expected phase), not searching 30. Sidelobes and other codes drop out of the picture.

For N = 15, sliding-window correlation per frame:
- Useful lock threshold: peak > 0.4 × N = **6**
- **Tolerates ~4 bit flips or ~8 erasures out of 15** (over 50% erasure rate)
- Mixed: 2 flips + 4 erasures still maintains lock

With Kalman filter coasting through gaps, multi-frame dropouts under ~40 ms are invisible downstream. Sustained >50% chip loss over many code periods triggers fallback to re-acquisition.

### Erasure-aware correlation (recommended)

Bit flips cost **2** per chip from the correlation peak. Erasures (chip marked uncertain, treated as 0) cost only **1**.

The blob detector already knows when a chip read is unreliable: pixel saturation, partial occlusion, blob momentarily lost, motion-blur smear. **Pass these as erasures, not guesses.** Roughly doubles error tolerance for free.

### Soft thresholds (recommended)

- **Tentative lock:** peak crosses a moderate threshold — produce position estimate with low-confidence flag
- **Confirmed lock:** peak crosses high threshold — full-confidence position
- Tentative locks still feed the Kalman filter but with inflated covariance

### When to consider ECC

Skip explicit ECC for **beacon ID itself** — with only 2 beacons, the cross-correlation distance is large enough that parity bits would cost more than they gain.

Add ECC if/when a **data payload** is layered on top (timestamp, sequence number, telemetry from the target). The GPS playbook applies: short Gold code for acquisition/ID, longer Reed-Solomon or BCH-protected payload modulated on top.

## 240 fps vs 480 fps Tradeoff Summary

| | 240 fps | 480 fps |
|---|---|---|
| Acquisition (N=15) | 63 ms | 31 ms |
| Per-frame SNR | +3 dB advantage | −3 dB |
| Motion-blur immunity | Adequate | Better (helps at <20 m) |
| Sensor sourcing | Commodity | Narrower selection |
| Data rate / compute | 18 MB/s @ 320×240 | 37 MB/s @ 320×240 |
| Power | Lower | Higher |

**Decision: 240 fps baseline.** Reserve 480 fps as an option if close-in motion blur during terminal phase becomes a problem (can also be addressed with shorter exposure at 240 fps).

## Compute

At 240 fps × 320×240 mono → **~18 MB/s** raw data rate.

Suitable platforms:
- STM32H7
- NXP i.MX RT series
- Tiny FPGA (Lattice iCE40 / CrossLink)

Per-blob processing (not full-frame): extract centroids on-chip, send `(x, y, intensity, chip_value, confidence)` downstream per frame. Keeps system bandwidth and compute load tiny.

## Open Questions / TODOs

- [ ] Confirm OG0VA sourcing path (design house vs. eval module vs. direct OEM)
- [ ] LED current / drive circuit design — match exposure window, manage thermal
- [ ] Bandpass filter vendor + spec (center, FWHM, transmission, AR coatings)
- [ ] Compute platform decision: MCU vs FPGA vs hybrid
- [ ] Mechanical: lens mount + sensor flex + filter holder integration
- [ ] Pyramidal LED arrangement geometry — how many simultaneously visible across expected aspect range?
- [ ] Validate ~30 dB real-world loss budget with bench measurements
- [ ] Define "tentative" vs "confirmed" lock thresholds for the soft-decision pipeline
- [ ] Decide on erasure-marking criteria in the blob detector (saturation level, blob-area thresholds)

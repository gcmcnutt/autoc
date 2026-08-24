# 042 — camera / physics data (child of 031) — seed

**Created 2026-08-18.** Holds the camera-era measurement work descended from 031, sequenced between
041 (better M1, flown) and 043 (M2 tracking). **Not started.**

## Why it sits between 041 and 043

The constants it produces are **inputs to M2's sensor model**, and measuring them depends on neither
feature. So M2 waits on it, and it does not wait on M1.

## ⭐ NEW 2026-08-18 — a 1.56 mm lens is ORDERED, ≈120° FOV

Operator: *"042 has ordered a 1.56mm lens which is close to 120deg fov. So that part of m2 is dependent on
range and response findings anyway."*

⚠️ **This may REVERSE a conclusion 041 acted on.** 031's ruled-mat calibration measured the **1.8 mm**
fisheye at **95° H × 61° V** (0.076 °/px) and recorded that *"the single-fisheye-at-120° assumption is
RETIRED for this lens"* — 120° was said to need the birded pair or a wider lens. **The 1.56 mm is that
wider lens.** If it measures near 120°, single-camera-at-120° comes back on the table and the birded pair
loses its main justification.

**What that means for the numbers currently in the tree:**

| | value in the tree today | status |
|---|---|---|
| `CameraDegPerPixel` | 0.304 (4× bin of 0.076) | measured for the **1.8 mm** |
| grid | 320 × 200 | the sensor's dump; unchanged by lens |
| derived FOV | 97.3° × 60.8° | **1.8 mm only** — re-derive for the 1.56 mm |

⚠️ **Do NOT pre-emptively change the sim to 120°.** That was the pre-arrival *estimate* 031 retired on
measurement; replacing a measurement with a different estimate repeats the original error. Measure the
1.56 mm the same way (ruled mat, tape at the frame edge, projection curve center-to-edge) and let the
number decide.

## What 042 must produce

1. **Measured FOV + projection curve for the 1.56 mm** — is it equidistant to the edge like the 1.8 mm?
2. **Range and response findings** — the operator's framing: *M2's sensor scope depends on these anyway.*
   Detection range against the photon budget (`CameraDetectionRangeM = 100` is currently corroborated only
   by the 031 estimate), and temporal response / relock behaviour.
3. **The lens decision**: 1.56 mm single vs 1.8 mm single vs birded pair. Inputs are (a) measured FOV,
   (b) the photon budget at the wider aperture, (c) 043's blind-gap sensitivity.
   ⚠️ 041's T082 blind-gap distribution is a **lower bound** (measured at 120°×90°) — a genuinely 120° lens
   would move it back toward that measurement, which is the one case where the optimistic number becomes
   the right one.
4. **850 nm filter** for daylight — affects photon budget/range, not geometry. Still required for flight.

## Standing traps

- **Geometry is derived, never configured** (FR-003): field = grid × pitch. Change the pitch, not the field.
- The **grid is 320 × 200 because that is what the sensor dumps** — a 4× bin of 1280 × 800. Do not size it
  to make a derived angle match a tape reading (041 T041f tried; reverted).
- `CameraDegPerPixel` has **five definitions** in the tree (two struct defaults + three inis). Change all
  five or the modes diverge.

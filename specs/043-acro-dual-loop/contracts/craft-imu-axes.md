# Contract — the new IMU craft-variation axes

⭐ **Framing**: *"craft variations are many and we add a few more, to be closer to reality."* These append
to the **existing craft class** (spec FR-054) — not a new subsystem, not a new PRNG class.

## Axes

| field | units | σ | 2.5σ = limit | represents |
|---|---|---:|---:|---|
| `craftImuMisalignRoll/Pitch/Yaw` | deg | 2.0 | **±5°** | the IMU is foam-taped to the underside of the upper surface |
| `craftGyroScaleX/Y/Z` | fraction | 0.02 | **±5%** | sensor + calibration tolerance |
| `craftAccelScaleX/Y/Z` | fraction | 0.02 | **±5%** | same |
| `craftAccelBiasX/Y/Z` | g | from measurement | — | ⭐ anchored to the 041-t7 audit: `accel_y` **1.70σ** off sim, where sim sits at exactly 0.000 |

⛔ Sigmas are set so **2.5σ is the intended hard limit**, using the pipeline-wide `nextGaussian` truncation.
No bespoke clip constants — the convention `camera_variation.h` documents.

## Sibling axis added in the same break — `craftCmQ` (not an IMU axis)

| field | units | centre | clamp | represents |
|---|---|---:|---|---|
| `craftCmQ` | — | **−4.2** | [−5.0, −3.6] | pitch damping. ⭐ Range pre-derived in `hb1_streamer.xml`: *"Cm_q range −3.6 (no streamer) to −5.0 (25 ft streamer)"* |

Absolute physical value + clamp, like `craftServoSlew` — **not** a delta. ⛔ **Does not duplicate the
static side**: `craftCGDelta` already varies CG against the neutral point, which *is* static margin on the
mass side. This is the dynamic side only (spec FR-052b).

## Where the misalignment applies

⭐ **Both loops, consistently** — the same FC IMU feeds INAV's rate feedback and the policy's observation:

| | effect |
|---|---|
| zero-rate nulling | **none** — the rotation is invertible, so measured→0 implies true→0 |
| **commanded-rate axes** | ⛔ INAV's roll and pitch **are** the IMU's axes: ±5° ⇒ **~9% cross-axis coupling on every command** (first-order) |
| **the policy's self-view** | gyro, accel and the attitude estimate are all rotated ⇒ the geometry it computes to the target is biased. **This is the "error view of its goal" the axes exist to train against.** |

## Rules

1. ⛔ **σ = 0 is bit-identical** to the axes not existing (FR-053).
2. **Draw-and-discard** — the craft PRNG advances identically whether the axes are enabled (FR-054).
3. **Append at the bottom** of `generateCraftFromClassPRNG`, and last in `CraftDeltas` /
   `ScenarioMetadata` — draw order is frozen; appending preserves every existing draw's value.
4. ⛔ **Not ramped** — diversity, not difficulty (FR-055), same as the rest of craft.
5. ⛔ **Gross misconfiguration is out** (FR-052a): no upside-down AHRS, no swapped axis, no 90°/180° board
   alignment. Those are configuration faults the bench catches, not build tolerances.
6. ⛔ **FR-057 first** — this changes `ScenarioMetadata`'s wire format and orphans every existing dmp,
   including the `retain=keep` 041-t7 baseline. Extract before writing.

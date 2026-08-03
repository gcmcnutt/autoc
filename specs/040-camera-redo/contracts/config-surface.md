# Contract — Configuration Surface

**Every physical quantity is an externally configured value with a stated default and a recorded basis
(FR-034), classified measured / derived / assumed (FR-035).** This table *is* the classification record —
a later calibration pass reads it to know exactly what it may overwrite.

**M** = measured · **D** = derived · **A** = assumed (placeholder awaiting calibration)

> ✅ **AUDITED 2026-08-02 (T091) — regenerated against the SHIPPED `AUTOC_CONFIG_FIELDS`, not the plan.**
> The plan-time version had drifted badly: **32 keys it named were never implemented under those names**,
> and **~50 perception keys that did ship were absent from it**. A classification record that disagrees
> with the code is worse than none, because T092's rehearsal reads it to decide what may be overwritten.
> Every row below was checked against `include/autoc/util/config.h` and `autoc-tracker.ini`.
>
> **Planned keys that do NOT exist**, recorded so nobody goes hunting:
> `AcqChipRateHz` · `AcqCodeChips` · `AcqGoodThreshold` · `AcqWarmRelockMs` · `AcqColdAcquireMs` ·
> `AcqHoldMs` · `AcqCoastWindowSec` → collapsed into the four `Acquisition*Ms` keys, because the machine
> advances **analytically in milliseconds** and never needs a chip rate or chip count.
> `CameraEntrancePupilMm` · `CameraExposureMs` → never implemented: nothing reads them, since the
> propeller term ships as one measured `AirframePropAttenuation` rather than being derived from pupil
> geometry. `BeaconSeparationM` → separation IS `BeaconLeftMountY`/`BeaconRightMountY`, so it cannot
> disagree with the mounts. `BeaconEnclosureSizeM` · `BeaconDriveCurrentMa` → not read; the five-face
> enclosure is structural in `emissionGain`, and drive folds into `SignalFluxConstant`.
> `CameraVar*Clip*` → **deliberately not implemented**, see the envelope note under Camera variation.
> `AirframeWingSpanIn` / `ChordIn` / `LeStationIn` / `PropDiameterIn` / `PropStationIn` → superseded by
> explicit AABB corners in **metres** on the prop-axle datum, which removes a units conversion rather
> than configuring one. `CameraFOVHorizontalDeg` · `CameraFOVVerticalDeg` · `CameraFrameRateHz` ·
> `CameraLatencyMs` · `BeaconEmissionConeDeg` → retired as planned (T017/T029/T058).

---

## Sensor grid (FR-003 — field of view is DERIVED, never configured beside the grid)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `CameraCount` | — | 1 | — | structural |
| `CameraPixelsH` | px | 320 | **A** | assumed sensor format (OG0VA paper spec) |
| `CameraPixelsV` | px | 240 | **A** | 4:3 |
| `CameraDegPerPixel` | deg | 0.375 | **A** | **the single resolution knob** ⇒ 120° × 90° derived |
| `CameraMountOffsetX` | m | −0.150400 | **M** | wing LE, 2 mm proud of the LE face |
| `CameraMountOffsetY` | m | +0.203200 | **M** | 8″ outboard — **starboard**; clears the prop disc |
| `CameraMountOffsetZ` | m | −0.031750 | **M** | ~1¼″ above the thrust line (−z is up) |

*Field of view is deliberately absent.* `fovHDeg()`/`fovVDeg()` are accessors, so no setter exists that
could contradict the grid.

## Emitter

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `BeaconLeftMountY` / `BeaconRightMountY` | m | ∓0.386 | **M** | ⇒ **0.772 m** separation: 30″ span + 5 mm/tip. Replaced 0.9 m — a ~17% error in the sole range channel |
| `BeaconLeft/RightMountX`, `…Z` | m | 0 | **M** | wingtip, on the span line |
| `BeaconLeftWavelengthNm` / `BeaconRightWavelengthNm` | nm | 850 / 940 | **M** | distinct per beacon |
| `BeaconEmissionFlatDeg` | deg | 45.0 | **M** | Lumileds DS190 flat region |
| `BeaconEmissionHalfPowerDeg` | deg | 75.0 | **M** | DS190 half-power ⇒ zero at 105° |

## Signal budget (FR-014/FR-015)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `SignalFluxConstant` | µA·m² | 0.27 | **M/D** | 031 bench 1.35 (five **co-aimed**) ÷ 5 faces |
| `SignalOpticsGain` | × | 1.0 | **A** | no collection optics fitted |
| `SignalAmbientFloor` | µA | 2.16e-5 | **A** | shade / overcast |
| `SignalNoiseFloor` | µA | 0.54e-5 | **A** | fixed sensor term |
| `SignalAmbientKnee` | µA | 2.16e-3 | **A** | **the 031 field-test #4 term** — ambient at which signal *transfer* halves |
| `SignalCdmaPenaltyDb` | dB | 3.0 | **M** | 031 §4, ≈ one SNR tier |
| `SignalQFloorDb` | dB | 0.0 | **M** | 031 decode floor ⇒ q = 0 |
| `SignalQSaturationDb` | dB | 20.0 | **A** | where the AGC-normalised q tops out at 9 |

> ⚠️ **Two couplings invisible in the ini that break silently if edited alone.**
> **(1)** `SignalAmbientFloor + SignalNoiseFloor` is **back-solved** so 0 dB lands exactly at
> `CameraDetectionRangeM` on beam peak (`flux / range² = 0.27/100² = 2.7e-5`). Pinned by
> `SignalModel.NoiseFloorIsCoherentWithTheAssertedDetectionRange`.
> **(2)** `SignalAmbientKnee` sits 100× above `SignalAmbientFloor`, so the nominal case is essentially
> uncompressed (−0.09 dB) and the t1 bake's semantics hold. Move the floor without the knee and
> compression switches on silently.

## Envelopes (FR-033 — two envelopes, not one)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `CameraDetectionRangeM` | m | 100.0 | **A** | **ASSERTED, not emergent** (FR-033a) |
| `SeparationMinResolvablePx` | px | 5.0 | **A** | 0.772 m at 0.375°/px ⇒ 5 px at ≈23.6 m |
| `SharedElementPx` | px | 1.5 | **M** | the 031 single-detector rig is exactly this case |
| `CepGateThreshold` | — | 1.25 | **M** | equals `kCepSentinelThreshold`, so it is a VISIBILITY gate — dropping it below 1.0 would start suppressing tentative locks and break FR-017c |

## Acquisition (shipped gateware, hardware-measured at N=31)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `AcquisitionCodeWordMs` | ms | 154.0 | **M** | one N=31 word — the **warm** relock budget |
| `AcquisitionColdMs` | ms | 308.0 | **M** | rate stale, needs `MINLOCK` |
| `AcquisitionHoldMaxMs` | ms | 308.0 | **M** | `HOLDMAX` = 2 bad periods |
| `AcquisitionCoastWindowMs` | ms | 10000.0 | **M** | `COASTMAX=65` — **wallclock-driven, and the knob sim difficulty actually hinges on** |
| `QualityConfidentCep` | — | 0.02 | **D** | cep at q = 9 |
| `QualityTentativeCep` | — | 1.0 | **D** | cep at q = 0 |
| `QualityIdentityUncertainCep` | — | 0.75 | **A** | FR-017d floor for unresolved identity |

Source: `firmware/beacon-decoder-stepfpga/SIM-FEATURES.md` (recovery-counter measurements; N=63 would read
315/629 ms — a **value** change, which is why these are keys).

## Airframe obstruction (metres, prop-axle datum: +x fwd, +y right, +z down)

| Key | Default | Class | Basis |
|---|---|---|---|
| `AirframeObstructionEnabled` | 1 | — | **ON since T043** — the LE mount is what made it usable |
| `AirframeWingMin{X,Y,Z}` / `AirframeWingMax{X,Y,Z}` | see ini | **M** | measured wing slab |
| `AirframeNoseMin{X,Y,Z}` / `AirframeNoseMax{X,Y,Z}` | see ini | **A** | ⚠️ **assumed pending the pod measurement** (checklist A1b). It accounts for the whole residual 2.9% blockage, and is deliberately modelled to under-obstruct |
| `AirframePropPlaneX` / `AirframePropAxisY` / `AirframePropAxisZ` | 0 | **M** | the datum itself |
| `AirframePropRadius` | 0.069850 | **M** | 5.5″ two-blade |
| `AirframePropAttenuation` | 0.18 | **A** | blade-over-pupil duty; research R13 |

## Camera variation (US6)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `EnableCameraVariations` | — | 1 | — | master switch; draw-and-discard either way |
| `CameraBoresightSigmaDeg` | deg | 8.0 | **M** | ⇒ 2.5σ = 20° |
| `CameraRollSigmaDeg` | deg | 8.0 | **M** | ⇒ 2.5σ = 20°; **highest-impact — biases tilt 1:1** |
| `CameraMountTranslationSigmaM` | m | 0.002 | **M** | ⇒ 2.5σ = 5 mm (the 1 cm box); **obstruction path only** |
| `CameraWingThicknessSigmaM` | m | 0.0008 | **A** | folded foam board is variable |
| `CameraAmbientSigmaFrac` | — | **0.0** | **A** | **DEFERRED** — emitter stays perfect until the lens+filter tests pin `SignalAmbientKnee` |

> **There are no `CameraVar*Clip*` keys, deliberately.** `ClassPRNG::nextGaussian` already truncates every
> draw at `kGaussianSigmaClamp` (2.5σ), and the project convention is to express an envelope AS sigma —
> `entryConeSigma = 18.0 // 2.5sigma = 45 deg`. A separate clip constant would be a second mechanism that
> could silently disagree with the first.

---

## SC-010 — what this table is for

A calibration pass may overwrite **any A row** without touching code (FR-036); T092 rehearses exactly that.
**M rows are evidence, not tuning** — changing one asserts that a measurement was wrong, and should arrive
with the new measurement.

**The A rows most likely to move first**, in order:
1. `SignalAmbientKnee`, `SignalOpticsGain` — the lens + bandpass field tests (~week of 2026-08-03).
2. `AirframeNose*` — the pod measurement (checklist A1b).
3. `CameraPixelsH/V`, `CameraDegPerPixel` — the sensor decision recorded in [optics-record.md](../optics-record.md).

## Binding rules

1. **No in-class defaults on `WorkerInit`-sourced members** (Principle VII). The constitution cites the
   `cepGateThreshold` bug — in this exact code — as the motivating failure. ⚠️ **Known limit, hit twice
   during 040**: this buys a compile error only under *aggregate* initialisation. Under `Foo f;` plus
   field assignment — which every test fixture uses — a new field is **silent garbage**, caught by a test
   failure rather than by the compiler. `SignalConfig` and `ProjectionInput::obstruction_mount_chase_body`
   both behaved exactly this way. Fast, but not the guarantee the wording implies.
2. **Derived values are never settable.** FOV is computed from grid × deg-per-pixel (FR-003).
3. **Classification is maintained with the value.** Changing a default without updating its class breaks
   the calibration contract, which depends on knowing what is safe to overwrite.
4. **Field-count contract test.** `contract_config_tests.cc` asserts the `AUTOC_CONFIG_FIELDS` count and
   `contract_tracker_config_tests.cc` asserts key presence in all three tracker inis; both must be updated
   in the same change that adds a key.

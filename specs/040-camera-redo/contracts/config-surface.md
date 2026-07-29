# Contract — Configuration Surface

**Every physical quantity is an externally configured value with a stated default and a recorded basis
(FR-034), classified measured / derived / assumed (FR-035).** This table *is* the classification record —
a later calibration pass reads it to know exactly what it may overwrite.

**M** = measured · **D** = derived · **A** = assumed (placeholder awaiting calibration)

---

## Already wired (no work beyond retirement)

`src/autoc.cc:955-960` already reads these into `CameraConfig`:

| Key | Class | Note |
|---|---|---|
| `CameraFOVHorizontalDeg` | **D** | becomes derived from grid × deg/px — **retire as an independent input** |
| `CameraFOVVerticalDeg` | **D** | same |
| `CameraMountOffset{X,Y,Z}` | M | baseline mount |
| `CameraFrameRateHz` | — | **retire** — 30 Hz predates the 20 Hz decision |
| `CameraLatencyMs` | — | **retire** — latency now emerges from acquisition timing |
| `CepGateThreshold` | M | retained; semantics change with quality regimes |

---

## Sensor

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `CameraPixelsH` | px | 320 | A | assumed sensor format |
| `CameraPixelsV` | px | 240 | A | 4:3 |
| `CameraDegPerPixel` | deg | 0.375 | A | ⇒ 120°×90°; **the single resolution knob** |
| `CameraEntrancePupilMm` | mm | 8.0 | A | pre-031 doc; affects propeller attenuation only |
| `CameraExposureMs` | ms | 1.04 | A | 50% of a 2.08 ms frame; report as sensitivity |

## Emitter

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `BeaconSeparationM` | m | **0.772** | M | 30″ span + 5 mm/tip; **replaces 0.9** |
| `BeaconEnclosureSizeM` | m | 0.01 | M | 1 cm cube |
| `BeaconFlatHalfAngleDeg` | deg | 45 | M | operator-observed flat region |
| `BeaconHalfPowerHalfAngleDeg` | deg | 75 | M | 150° FWHM, Lumileds DS190 Table 1 |
| `BeaconDriveCurrentMa` | mA | 306 | M | field current (bench is 51) |

> **Retire** `BeaconEmissionConeDeg` (270°) — the hard angular cutoff is replaced by the flat-top profile.

## Signal budget

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `SignalFluxConstant` | µA·m² | 1.1 | M | 031 bench measured 1.1–1.6 — **record which is used** |
| `SignalAmbientFloorUa` | µA | per-scenario | A | overcast → direct sun |
| `SignalDetectionThresholdDb` | dB | 0 | M | 031 decode floor ≤10 nA |
| `SignalCdmaPenaltyDb` | dB | ~3 | M | ≈1 SNR tier, 031 §4 |
| `SignalOpticsGain` | × | 1.0 | A | collection optics if fitted |

## Acquisition

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `AcqCodePeriodMs` | ms | 75 | M | 15 chips @ 200 Hz = 1.5 ticks @ 20 Hz |
| `AcqPartialLockMs` | ms | 55 | M | 031 early-lock |
| `AcqHoldPeriods` | — | 2 | M | 031 measured hold |
| `AcqWarmRelockPeriods` | — | 1 | M | 031 measured |

## Obstruction

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `AirframeWingSpanIn` / `ChordIn` / `ThicknessIn` | in | 30 / 7 / 1 | M | sketch 2026-07-28 |
| `AirframeWingLeStationIn` | in | 6 | M | station stack |
| `PropDiameterIn` | in | 5.5 | M | photo (Master Airscrew / Windsor) |
| `PropStationIn` | in | 0 | M | datum |
| `PropAttenuation` | fraction | 0.18 | A | representative blade-over-aperture duty |
| `AirframeObstructionEnabled` | bool | **true** | — | was a compile-time `false` |

## Camera variation (PRNG slot 5)

| Key | Unit | Default | Class | Basis |
|---|---|---|---|---|
| `CameraVarBoresightSigmaDeg` | deg | 10 | M | operator |
| `CameraVarBoresightClipDeg` | deg | **20** | M | **hard clip**, not tail-sampled |
| `CameraVarRollSigmaDeg` | deg | 10 | M | operator |
| `CameraVarRollClipDeg` | deg | **20** | M | hard clip |
| `CameraVarTranslationBoxM` | m | 0.01 | M | 1 cm box; **obstruction consumer only** |
| `CameraVarWingThicknessSigmaIn` | in | TBD | A | folded foam board, "highly variable" |
| `CameraVarAmbientSigma` | µA | TBD | A | pairs with `SignalAmbientFloorUa` |

**All-zero sigmas MUST reproduce the no-variation baseline bit-identically (FR-023).**

---

## Binding rules

1. **No in-class defaults on `WorkerInit`-sourced members** (Principle VII). The constructor initializer
   list is the single assignment site. The constitution cites the `cepGateThreshold` bug — in this exact
   code — as the motivating failure: a hardcoded `1.25` fallback silently overrode operator configuration,
   producing correct-looking but semantically wrong results with no compile error, test failure, or warning.
2. **Derived values are never settable.** FOV is computed from grid × deg-per-pixel so the two cannot
   disagree (FR-003).
3. **Classification is maintained with the value.** Changing a default without updating its class breaks
   the calibration contract, which depends on knowing what is safe to overwrite.
4. **Field-count contract test.** `contract_tracker_config_tests.cc` asserts the ini key count; it must be
   updated in the same change that adds keys.

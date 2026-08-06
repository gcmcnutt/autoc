# Calibration Rehearsal — T092 (FR-036, SC-012)

**Run**: 2026-08-02 · **Verdict**: ✅ **PASS — no structural change required**

> **This is the feature's central claim under test.** 040 was built plumbing-first: every physical quantity
> is a configured value with a stated class, so that when real measurements arrive they land as *value
> changes*. If a calibration pass demanded a code change, that contract would be broken. This document is
> the test of it.

---

## Method

Copy the shipped `autoc-tracker.ini`, substitute a **plausible alternative** for every value classified
**A (assumed)** in [contracts/config-surface.md](contracts/config-surface.md), and confirm that

1. the alternative config **parses**,
2. the model's **results change** (a config that changes nothing proves nothing), and
3. **no source file or shipped ini is edited** to make it work.

The rehearsal ran against a *copy* so the shipped configuration was never disturbed.

## Substitutions — 15 assumed values, all moved

| Key | Shipped | Rehearsal | Why this alternative is plausible |
|---|---|---|---|
| `CameraPixelsH` / `CameraPixelsV` | 320 / 240 | **640 / 480** | a VGA sensor instead of QVGA |
| `CameraDegPerPixel` | 0.375 | **0.1875** | halved to hold the same field on the finer grid |
| `SignalOpticsGain` | 1.0 | **4.0** | the C-14 collection lens actually fitted |
| `SignalAmbientFloor` | 2.16e-5 | **2.16e-4** | 10× ambient — bright overcast rather than shade |
| `SignalNoiseFloor` | 0.54e-5 | **0.54e-4** | moved with the floor to keep the back-solve coherent |
| `SignalAmbientKnee` | 2.16e-3 | **5.0e-3** | a bandpass filter raising the compression knee |
| `SignalQSaturationDb` | 20.0 | **26.0** | a wider usable AGC span |
| `CameraDetectionRangeM` | 100.0 | **60.0** | a more conservative asserted envelope |
| `SeparationMinResolvablePx` | 5.0 | **3.0** | better centroiding on the finer grid |
| `AirframePropAttenuation` | 0.18 | **0.35** | a measured blade duty rather than the estimate |
| `QualityIdentityUncertainCep` | 0.75 | **0.6** | a softer identity-uncertainty floor |
| `AirframeNoseMaxY` / `MaxZ` | 0.0381 | **0.0508** | a fatter pod once actually measured |
| `CameraWingThicknessSigmaM` | 0.0008 | **0.0016** | more variable foam board |

## Result

**The configuration parsed and the model changed**, via `dmp-dump --obstruction-report -i <rehearsal.ini>`:

| | shipped | rehearsal |
|---|---:|---:|
| `blocked_frac` | 0.02925 | **0.04147** |
| `attenuated_frac` | 0.02201 | **0.01441** |
| `nominal_sr` | 2.98916 | 2.98920 |

Blockage rose because the pod box is fatter; the attenuated share fell because the finer grid samples the
prop disc differently. Both moved for the reasons the substitutions imply, which is the point — the config
is not merely accepted, it is *load-bearing*.

**And nothing was edited.** `git status` over `src/`, `include/`, `tools/`, `crrcsim/` and the shipped
`autoc-tracker.ini` was clean throughout. **No structural change was required. SC-012 holds.**

### A second property fell out, unplanned

The FOV stayed exactly **120° × 90°** across the substitution — 640 × 0.1875° = 120°, as 320 × 0.375° did.
That is FR-003 working: field of view is *derived* from the grid, so doubling resolution while halving
angular pitch holds the field constant **with no field key to keep in sync**. Under the retired
`CameraFOVHorizontalDeg` design this substitution would have silently produced a config whose declared
field and actual grid disagreed.

---

## Honest limits of what this exercised

Stated so the pass is not read as broader than it is.

- **End-to-end coverage is geometry/sensor.** `--obstruction-report` consumes the camera-grid, mount and
  airframe rows from the ini through the real code path. It does **not** touch the signal budget, so the
  `Signal*` and `Quality*` substitutions were proven to *parse and print* (`AUTOC_CONFIG_FIELDS` drives
  both, and `contract_config_tests` asserts parse+print agree) and are assigned field-by-field to
  `WorkerInit` in `src/autoc.cc` — but no report in this rehearsal reads them back out. The unit tests
  cover the same fields by varying them at runtime (`AmbientCompressionCollapsesTransferAsAmbientRises`,
  `NoiseFloorIsCoherentWithTheAssertedDetectionRange`), which is why the gap is small — but it is a gap.
- **A real calibration would need a retrain to be *validated*.** The rehearsal shows the plumbing accepts
  new values; it says nothing about whether the resulting controller is better.
- **The `M` rows were deliberately left alone.** They are evidence, not tuning: changing one asserts that
  a measurement was wrong and should arrive with the new measurement.

## What this predicts for the real thing

The lens + bandpass field tests (~week of 2026-08-03) are expected to pin `SignalAmbientKnee` and
`SignalOpticsGain`. On this evidence that lands as **an ini edit and a retrain** — no code, no schema, no
test changes beyond any that encode the old value as an expectation.

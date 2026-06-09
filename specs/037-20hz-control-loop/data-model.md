# Data Model: Faster Control Loop (037)

**Date**: 2026-06-09. Entities the cadence change touches. "Schema-affecting" entries are the
Principle-V surface (NN layout / dmp). Field-level detail references the grounded code sites.

## 1. CadenceConfig (control-rate single source of truth)

The cadence the NN control loop runs at. Today split across two constants; R6 unifies it.

| Field | Type | Today | Notes |
|---|---|---|---|
| `controlIntervalMsec` | int | 100 | the NN eval interval; replaces env `AUTOC_EVAL_INTERVAL_MSEC` |
| `gEvalUpdateIntervalMsec` | unsigned long | 100 | crrcsim mirror — must equal `controlIntervalMsec` |
| `SIM_TIME_STEP_MSEC` | macro/const | 100 | autoc-eval mirror — must equal `controlIntervalMsec` |
| `fdmDt` | double | 0.005 (200 Hz) | `autoc_config.xml`; raise for fast arms (≥500 Hz; 2 kHz matches INAV) |
| `outerFps` | int | 20 (50 ms) | `autoc_config.xml`; `cycleLengthMs` must divide `controlIntervalMsec` |

**Invariant (enforced at startup, already present)**: `controlIntervalMsec % cycleLengthMs == 0`;
`framesPerEval = controlIntervalMsec / cycleLengthMs`. **Validation rule (new)**: no in-class default for
`controlIntervalMsec` (Principle VII); read from `.ini`/CLI (R6).

**State transition**: changing `controlIntervalMsec` triggers the tick-rescale audit (research.md) — it
is NOT a standalone knob.

## 2. NN input layout — M1 / M2 (SCHEMA-AFFECTING, Principle V)

| Entity | Today | Site | After R5 (history time-basis) |
|---|---|---|---|
| M1 (pathgen) inputs | `NN_INPUT_COUNT = 33` | `nn_inputs.h:36-58`, topology `"33,32,16r,3"` (1923 w) | `NN_INPUT_COUNT` changes if slot count changes |
| M1 history | `target_{x,y,z}[6]`, `dist[6]` uniform 100 ms past-only | `evaluator.cc:312 HIST_PAST[]` | log/time-spaced lag set; slot count = new N |
| M2 (tracker) inputs | 54 | `nn_inputs.h:136-240`, topology `"54,32,16r,3"` (2595 w) | changes with slot count |
| M2 history | `left/right_{x,y,cep}[6]`, `span[6]` | `evaluator.h:112-127`, `tracker_stepper.cc:121-181` | same lag restructure |
| error ring | `HISTORY_SIZE=10` (1 s @ 10 Hz) | `aircraft_state.h:330` | resize on time basis |

**Validation/contract**: changing the layout changes the cereal `EvalResults`/NN-weight shape. Per
`feedback_no_cereal_versioning` do NOT bump the cereal version; per Principle V the reader MUST **fail
loud** on a layout/`NN_INPUT_COUNT` mismatch (no silent truncation). Old t6 dmps are not layout-compatible
— compare via recorded metrics (Q3), not by replaying old dmps through the new layout. Honest dmp
recording (`feedback_honest_dmp_recording`): the new layout must record all inputs + outputs.

## 3. Tick-denominated fitness terms (rescale targets)

See research.md "Tick-rescale audit". Entities: `stabilityAccum`, `energyAccum` (unnormalized Σ —
rescale), `closing_rate` (`/0.1f` → dt), `closure_rate`/`thrash_rate` (per-second; verify),
`streakStepsToMax` (auto ✅), engage-delay window (implement rate-independent). Types: `gp_fitness` for
accumulators, `gp_scalar` for NN-bound rates (Principle VI grep audit on touched files).

## 4. EntryVariationEnvelope (P1 prework)

| Field | Today | Target | Site |
|---|---|---|---|
| `entryConeSigma` | 30° | **18°** (⇒ 2.5σ = 45°) | `config.h:63` |
| `entrySpeedSigma` | 0.10 | **0.05–0.06** (⇒ 2.5σ ≈ 12.5–15%) | `config.h:65` |
| `kGaussianSigmaClamp` | 2.5 | unchanged | `scenario_prng.h:49` |
| (existing hard cap) | `kEntryConeMaxRad=80°` | moot at 18° sigma; leave/note | `variation_generator.h:297` |

**Rule**: lower sigmas only (clamp stays 2.5σ; do NOT add a new hard cap). Clean step, not mid-bake; t6
remains the baseline (Q3). Changes M1 difficulty distribution — flag in the outcome doc.

## 5. RateTier sources (Phase B)

| Tier | Signals | Source / rate | Bridge |
|---|---|---|---|
| Fast | accel + gyro | local IMU @ control rate | none (primary) |
| Intermediate | position, velocity, airspeed | INAV poll | dead-reckon between polls (good ~10s of ms) |
| Slow (resync) | attitude abs + yaw | INAV slow poll | local complementary propagates between |

**Future-swappable**: intermediate source = INAV-now / direct-GPS-later; slow attitude-resync falls away
post-INAV. Alignment: auto-calibrate local-IMU→body rotation from INAV (not manual board-alignment);
convention coherence via `inavQuatToAerospaceEB` is mandatory.

## 6. PackedLogFrame (Phase B prework P5, deferred)

Blackbox-style differential, packed binary; **variable frame type per tier** (fast IMU / intermediate
nav / slow attitude-resync / cross-check pairing local vs INAV), **variable frame rate per type**,
delta+varint encoding with periodic keyframes. Single authoritative writer/reader pair
(`project_log_format_shared_parser`); `join_flight_analysis.py` must decode it. Replaces the ~328-char
text dump. Tail-safe: pre-erased ring, async-DMA, no in-loop erase (fixes the `flash_logger.cpp` blocking
erase/write tail).

## 7. EvalCycleMeasurement (R4 harness output)

`{shape ∈ {M1,M2}, tanhImpl ∈ {expf, poly/LUT}, phase ∈ {macs, +tanh, +gather}} → {cycles, µs@64MHz}`,
on-target (`DWT->CYCCNT`) + off-target op-count. Feeds the defended eval budget and the Phase-C go/no-go.

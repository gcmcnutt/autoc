# Phase 1 Data Model: 038 Accurate M2

**Date**: 2026-06-27 | **Plan**: [plan.md](plan.md) | **Research**: [research.md](research.md)

Schema/entity deltas for 038. All changes are **greenfield, no cereal version bump, fail-loud on read**
(Constitution V practice). The format-breaking ones (history layout, NN topology, output count, EvalResults
config block) are bundled into **P0-D's single dmp break** — one determinism break, retrain from scratch.

Legend: ⬛ format-breaking (orphans old dmps/genomes) · ◻ additive/non-breaking · 🔬 ablation-gated (final
shape is a bake output) · ⏸ deferred (US5, recorded only).

---

## 1. History layout (US1) ⬛🔬

**File**: `include/autoc/nn/nn_inputs.h`

| Field | Current | 038 starting ablation |
|---|---|---|
| `kNNHistoryLagsMsec[]` | `{800,400,200,100,50,0}` (6 slots, 0.8 s) | `{1600,800,400,200,100,50,0}` (7 slots, 1.6 s) — 🔬 final spacing/depth is a bake output |
| `kNNHistoryLayoutVersion` | `3` | `4` (bump on any lag change) |

**Derived (auto-recompute from the constant)**:
- `HISTORY_SIZE` (`aircraft_state.h:391`) = `(kNNHistoryLagsMsec[0]/SIM_TIME_STEP_MSEC)+1`
- `TrackerObservationRing::kDepth` (`evaluator.h:143`) = same
- `HIST_PAST[]` (`evaluator.cc:314`) = per-slot `historyLagTicks()`

**Validation rules**:
- `historyLagsIntegral()` static_assert — every lag MUST be an integral multiple of `SIM_TIME_STEP_MSEC`
  (50 ms). At 20 Hz, sub-50 ms slots (e.g. 25 ms) are illegal → compile-time fail-loud.
- Layout version mismatch on dmp read → throw (fail-loud).

**Consumers to update**: tests (`tick_rescale_tests`, `nn_layout_tests`), xiao codegen (`HIST_PAST` is
compile-time), NN01 genome weight count (input fan-in unchanged for history depth, but slot count feeds
`TrackerInputs`/`NNInputs` beacon-history arrays → if slot count changes, struct size changes → ⬛).

> Note: increasing history *slot count* (6→7) changes the per-beacon history arrays in `TrackerInputs`
> (beacon_l_x[6]→[7], etc.) → `TrackerInput::COUNT` grows from 54 → 60 (6 beacon channels × +1 slot). That
> is an NN input-count change (see §2). Keeping slot count at 6 but changing *spacing* avoids the input-count
> change. The ablation choice (deeper window with more slots vs same-count denser) is a bake decision — both
> are bundled into the one P0-D break regardless.

---

## 2. NN topology (US2 recurrence, US3 predictor head) ⬛🔬

**File**: `include/autoc/nn/topology.h` (both pathgen + tracker sections)

| Constant | Current (tracker) | US2 slow-channel (start) | US3 predictor head (start) |
|---|---|---|---|
| `TRACKER_NN_TOPOLOGY` | `{54,32,16,3}` | `{54,32,(16+8)r,3}` | `{...,7}` (3 control + 4 aux optical) |
| `*_OUTPUT_COUNT` | `3` | `3` | `7` |
| `NN_RECURRENT` | `{f,f,t,f}` | hidden-2 fast+slow recurrent | unchanged (aux head feedforward off hidden-2) |
| `*_HIDDEN_STATE_COUNT` | `16` | `16 + 8` (slow) | unchanged by US3 |
| `*_WEIGHT_COUNT` | `2595` | recompute (slow W + W_hh_slow) | recompute (output fan-in 16→7) |

**NNGenome** (`evaluator.h:10`): `weights`/`topology`/`recurrent` vectors validated against compiled
constants at load → any mismatch orphans the genome (fail-loud).

**NN01 binary** (`serialization.cc`): topology-agnostic on the wire (variable-length topology + recurrent
arrays) but runtime weight-count validation rejects mismatched genomes. No format-version change needed in
NN01 itself; old genomes simply fail to load (retrain).

**Output convention (US3)**: outputs `[0..2]` = pitch/roll/throttle (control, as today); outputs `[3..6]` =
predicted next-tick optical state (scored, not actuated). xiao codegen sinks aux outputs (first-3-for-
control convention preserved).

**🔬 bake outputs**: slow-channel width + leak (US2), aux head shape/target/horizon (US3) — research.md §2/§3.

---

## 3. NNInputs / TrackerInputs (US1 slot-count, US3 if aux inputs) ⬛

**File**: `include/autoc/nn/nn_inputs.h`

- `TrackerInput::COUNT` (currently 54) changes ONLY if history slot count changes (§1) or US3 adds new
  *inputs* (the starting US3 plan reuses existing inputs → no input change; only output count grows).
- Field declaration order = on-disk byte order (cereal/data.dat/nn2cpp/sim_response). **No reordering, no
  padding, floats only** (Constitution VI byte-format note).
- `static_assert(sizeof(TrackerInputs)==COUNT*sizeof(float))` enforces layout.
- `kTrackerInputMeta[]` display-name table updated to match any new slots.

---

## 4. ScenarioScore — new lexicase axes (US4, US3) ◻ (additive to in-memory struct; ⬛ if recorded to dmp)

**File**: `include/autoc/eval/fitness_decomposition.h` (`struct ScenarioScore`, lines 71–93)

| New field | Type | Sign | Source | User story |
|---|---|---|---|---|
| `visibility_score` | `gp_fitness` | negated (lower=better) | per-tick CEP/in-FOV accumulate × `kCadenceTickScale`; **0 for M1/pathgen** | US4 |
| `prediction_score` | `gp_fitness` | negated | per-tick aux-output vs realized-next-tick optical error | US3 (ablation-gated) |

**Validation/invariants**:
- M1 (pathgen) branch leaves both zero (no `CameraViewSample`/aux target). The lexicase pool tolerates
  M1=0 on tracker-only axes (pathgen members score 0; tracker members score computed).
- `applyCrashPenalty` (`autoc.cc:181`) multiplies ONLY `score` — leaves `visibility_score`/`prediction_score`
  untouched (matches how it already leaves `energy_score`/`stability_score` alone). Verify on add.
- New axes are added to the lexicase pool in `selection.cc` as **separate per-scenario test cases**
  (`pool.push_back({s, &ScenarioScore::visibility_score, eps})`), **never scalar-composited into `score`**
  (sidesteps 033 Pareto collapse).

**Recording**: if the new axes are persisted to the dmp (for `dmp-dump`/analytics, honest-recording per
FR-031 / [feedback_honest_dmp_recording]), that is part of the P0-D break. The `TrackerDiag` block already carries `vis_frac`/`max_lost_sight_run`
as diagnostics; US4 promotes visibility from diagnostic to selection axis.

---

## 5. EvalResults — self-describing config block (P0-D-2) ⬛

**File**: `include/autoc/rpc/protocol.h` (`struct EvalResults`, lines 365–514)

**New embedded block** (the fitness/cadence config the dmp was previously reading from the live `.ini`):

| Field group | Members | Source |
|---|---|---|
| Fitness cone | `fitDistScaleBehind`, `fitDistScaleAhead`, `fitConeAngleDeg`, `fitStreakThreshold`, `fitStreakRampSec`, `fitStreakMultiplierMax` | `config.h:158–163` |
| Cadence | `simTimeStepMsec` (= `SIM_TIME_STEP_MSEC`), derived `kCadenceTickScale` | `aircraft_state.h:60,90` |
| Crash penalty | `enableHullCrashPenalty`, `hullCrashPenaltyFactor`, `oobCrashPenaltyWeight` | `config.h:136–138` |
| Visibility (US4) | `enableVisibilityReward` + params | new (`config.h` X-macro) |

**Read-side contract** (P0-B): `renderer.cc:2933` + `dmp_dump.cc:406` prefer the dmp-recorded block over
`ConfigManager::getConfig()`. Only remaining `.ini` dependence: S3 bucket/profile (bootstrap).

**Versioning**: no cereal bump; old v=2 dmps without the block fail loud on read (retrain).

---

## 6. AircraftState — simTimeMsec + wind_velocity (P0-D-1, P0-D-3) ⬛ (behavior) / ◻ (schema)

**File**: `include/autoc/eval/aircraft_state.h` (cereal serialize line ~556)

| Field | Current | 038 |
|---|---|---|
| `simTimeMsec` | truncated 200 Hz step clock (49/50/51 ms jitter) | step-count-derived exact 50 ms gaps (`SimStateHandler.cpp:392` fix) |
| `wind_velocity` (`gp_vec3`) | serialized but **never set** (zero everywhere) | populated from crrcsim FDM at record time (`inputdev_autoc.cpp` ~906) |

Both fields already exist in the schema (no new field), but the *values change* → determinism-affecting →
bundled into P0-D, retrain. Source-spacing check reverts to strict single-gap
(`crrcsim_tracker_helper.cpp:76`, `tracker_stepper.cc`).

---

## 7. Config knobs (X-macro) ◻

**File**: `include/autoc/util/config.h` (`AUTOC_CONFIG_FIELDS(X)`) + `src/util/config.cc` (auto-parse) +
`src/autoc.cc` (auto-print)

| New knob | Type | Default | Purpose |
|---|---|---|---|
| `enableVisibilityReward` | `int` | (no in-class default per VII; set at parse) | US4 gate |
| visibility-reward params | `double` | — | US4 reward shape (research.md §4) |
| `enablePredictorHead` | `int` | — | US3 ablation gate |

Add field to struct + one `X(type,field,"IniKey")` line → auto-parses + auto-prints (startup banner is
non-driftable). P0-F edits existing `fitStreakThreshold` value (0.3→0.5) in the three tracker inis — no new
field.

---

## 8. cameraSeed / cameraPRNG (US5) ⏸

**File**: `include/autoc/rpc/scenario_metadata.h:98` (reserved insertion point)

Recorded, NOT implemented in 038. When US5 unparks: append `uint32_t cameraSeed` after `craftSeed` (wire
order frozen — appended, not interleaved), add to the serialize walk, follow the craft pattern
(draw-and-discard when disabled to preserve cross-class PRNG determinism), `EnableCameraVariations` knob,
ramp via `applyVariationScale`. The 5th `ClassPRNG` camera slot already exists in the cascade
(`deriveClassSubSeeds`, append-only D5).

---

## State transitions / lifecycle

No new stateful entities. The architecture changes alter the NN's internal recurrent state shape (US2 slow
channel) and output vector (US3 aux head); `NNControllerBackend::reset()` zeros all recurrent state
(including the new slow channel) on span/engage start — verify the slow channel is included in the reset.

## Determinism invariants (FR-030, cross-cutting)

Every change above preserves: (a) absolute per-scenario determinism from `scenarioSeed`; (b) eval-vs-
training bitwise `ScenarioScore` parity (the regression gate); (c) eval-mode replay via
`gEvalVariationScaleOverride`. The new axes/inputs/outputs feed deterministic accumulation; the predictor
error and visibility term are computed from already-deterministic per-tick data.

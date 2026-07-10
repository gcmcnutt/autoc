# Contract: lexicase axes + the visibility/predictor rewards

**Feature**: 038 | **Status**: design | **Consumers**: `selection.cc`, `fitness_decomposition.cc`,
`autoc.cc` (`applyCrashPenalty`), analytics.

Defines how 038's new fitness signals (US4 visibility, US3 prediction) enter selection **without**
re-creating the 033 scalar-objective Pareto collapse.

## Invariant: separate axes, never scalar-composited

- New signals are **per-scenario lexicase test cases**, added to the pool in `selection.cc`:
  `pool.push_back({s, &ScenarioScore::visibility_score, eps})` /
  `pool.push_back({s, &ScenarioScore::prediction_score, eps})`.
- They MUST NOT be summed into `ScenarioScore::score`. Scalar compositing of multi-objectives caused the 033
  Pareto-corner collapse (`project_scalar_multiobjective_collapse`); lexicase keeps each objective an
  independent dimension.
- Sign convention: `gp_fitness`, **negated (lower = better)**, matching `score`/`energy_score`.

## US4 visibility-maintenance reward

- Accumulated per-tick in the tracker branch of `computeScenarioScores` from the EXISTING per-tick CEP /
  in-FOV data (`cv.beacon_left/right.cep < kCepSentinelThreshold`). Start: continuous in-FOV term (reward
  low CEP / tight framing), × `kCadenceTickScale`.
- **M1/pathgen invariant**: `visibility_score = 0` (no `CameraViewSample`). The pool tolerates M1=0 on
  tracker-only axes.
- Behind `EnableVisibilityReward` (X-macro knob, no in-class default per VII). NOT format-breaking (reward
  change only) — can iterate without a re-bake, but changes the fitness scale → compare with fixed-eval.

## US3 prediction-accuracy axis

- Per-tick: error between the aux outputs (`outputs[3..N]`, predicted next-tick optical state) and the
  realized next-tick optical state; accumulated negated. Separate axis as above.
- Ablation-gated behind `EnablePredictorHead`; format-breaking (output count) → bundled into P0-D.

## Crash penalty composition (carried, t14)

- `applyCrashPenalty` (`autoc.cc:181`) multiplies ONLY `ScenarioScore::score`. It MUST leave
  `visibility_score` and `prediction_score` untouched (as it already leaves `energy_score`/`stability_score`).
  Verify on add — the new axes carry their own selection pressure independent of the crash multiplier.

## Selection-stability check

- Mixing the new axes (whose per-scenario spread may differ in magnitude from `score`) MUST be validated to
  not destabilize lexicase selection (epsilon/aggregation). If it does, that is the trigger to promote the
  deferred **MAD-relative epsilon** (FR-032 / `project_lexicase_mad_epsilon`) behind its ini switch
  (constant-0.5 default for reproducibility).

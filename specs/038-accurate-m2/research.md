# Phase 0 Research: 038 Accurate M2 — Architecture for Tracking Depth

**Date**: 2026-06-27 | **Plan**: [plan.md](plan.md) | **Spec**: [spec.md](spec.md)

This document resolves the spec's intentional `[NEEDS RESEARCH]` markers into **concrete starting points
with decision criteria** for each ablation. It does NOT pre-commit final architecture forms — those are
research outputs of the bakes (spec Out-of-Scope). Each section gives Decision (the starting configuration
to bake), Rationale, and Alternatives considered. Grounded in the integration map (see the four
subsystem maps consolidated below).

---

## 0. Integration map (consolidated — the load-bearing facts)

**NN topology is compile-time `constexpr`** in `include/autoc/nn/topology.h`:
- Pathgen: `NN_TOPOLOGY={33,32,16,3}`, `NN_RECURRENT={false,false,true,false}`, 1923 weights, 16-dim hidden
  state (hidden-2 recurrent, hidden-1 feedforward).
- Tracker: `TRACKER_NN_TOPOLOGY={54,32,16,3}`, same recurrent pattern, 2595 weights.
- NN01 binary (`src/nn/serialization.cc`) encodes topology + recurrent flags + weights; it is
  **topology-agnostic on read** but the runtime validates weight-count → any topology change orphans old
  genomes (fail-loud).

**History** is `kNNHistoryLagsMsec[6]={800,400,200,100,50,0}` in `include/autoc/nn/nn_inputs.h:48`,
versioned by `kNNHistoryLayoutVersion=3` (line 60). Consumed via `historyLagTicks()` →
`HISTORY_SIZE` (`aircraft_state.h:391`), `TrackerObservationRing::kDepth=17` (`evaluator.h:143`), and
`HIST_PAST[]` (`evaluator.cc:314`). `static_assert` requires every lag be an integral tick multiple at the
current cadence.

**Fitness** flows: `FitnessComputer::decomposeStepScore(along,lateralDist)` (cone: `FitConeAngleDeg`,
`FitDistScaleAhead/Behind`, streak via `FitStreakThreshold`/`applyStreak`) → per-tick accumulate ×
`kCadenceTickScale` → `computeScenarioScores` (`fitness_decomposition.cc:29`) finalizes a `ScenarioScore`
(`score`, `energy_score`, `stability_score`, all `gp_fitness`, negated lower-better) →
`applyCrashPenalty` (`autoc.cc:181`, the t14 ramped `0.75^(K_hull·scale)·exp(-w·scale·K_oob/N)`) →
`lexicase_select` (`selection.cc`, pool = per-scenario `{score, energy_score}` axes, eps constant-0.5 or MAD).

**Per-tick visibility is already computed** but only as diagnostics: `cv.beacon_left/right.cep <
kCepSentinelThreshold` (1.25) → `TrackerDiag.vis_frac`, `max_lost_sight_run` (`fitness_decomposition.cc:196`).
The visibility *infrastructure exists*; US4 promotes it from diagnostic to a selection axis.

**Variation ramp**: `computeVariationScale()` (`autoc.cc:167`) ∈ [0,1]; `applyVariationScale()`
(`scenario_meta_apply.h:24`) scales entry/wind/position (NOT craft — diversity not difficulty); eval-mode
uses `gEvalVariationScaleOverride` for bitwise replay.

**dmp contract**: `EvalResults` (`protocol.h:365`) + `AircraftState` (`aircraft_state.h`) cereal. Both
currently `CEREAL_CLASS_VERSION(...,2)`. `simTimeMsec` truncates at `SimStateHandler.cpp:392`;
`wind_velocity` is serialized but **never set** (zero in every dmp). Renderer/`dmp_dump` read fitness
params from the live `.ini` via `ConfigManager::getConfig()` (renderer.cc:2933, dmp_dump.cc:406).

**PRNG cascade** (`scenario_prng.h`): `MasterPRNG → ScenarioRootPRNG(scenarioSeed) → ClassPRNG` ×5 (wind,
rabbit, entry, craft, camera-reserved); `deriveClassSubSeeds()` is pure/cross-process; contract tests
D1–D5 in `tests/scenario_prng_tests.cc`. `scenarioSeed` persisted per-scenario in `ScenarioMetadata:40`.

**xiao** firmware is pathgen-only (`#ifndef ARDUINO` gates tracker in `evaluator.cc:427`); `nn2cpp.cc`
codegen is topology-agnostic (reads NN01) but `HIST_PAST[]`/cadence are compile-time → a history/cadence/
topology change requires firmware regen. The tracker is not yet on xiao (deferred, BACKLOG).

---

## 1. US1 — Deeper / non-uniform history layout

**Decision (starting ablation)**: bake the M1 history ablation first (mixed-M1-first gate) with a **deeper
8-slot log/geometric layout reaching 1.6 s**: candidate `kNNHistoryLagsMsec[8] = {1600, 800, 400, 200, 100,
50, 25, 0}` — doubles the window, adds two near-now slots for finer derivative/closure. Bump
`kNNHistoryLayoutVersion` to 4. Compare against the t10 `{800,400,200,100,50,0}` baseline at matched seed.

**Constraint check**: at 20 Hz (50 ms tick) the 25 ms slot is NOT an integral tick multiple → it would trip
`historyLagsIntegral()` static_assert. So the 25 ms slot is dropped for the 20 Hz bake (the static_assert is
the fail-loud guard); the viable 20 Hz candidate is `{1600,800,400,200,100,50,0}` (7 slots, all integral).
Finer-than-tick lags are only available if cadence increases (out of scope here).

**Rationale**: `rnn_capacity` says the limiter is *structure not width* (eff-rank ~10/16); a deeper/denser
temporal context is the cheapest structural lever and the cleanest M1-provable test (M1's rabbit is always
visible → no FOV confound). M1 also tracks a target, so a better temporal buffer should lift M1 too — if it
doesn't move an M1 ceiling, that's a clean negative result.

**Alternatives considered**:
- *Fibonacci/exponential spacing* (`{1600,800,500,300,150,50,0}` etc.) — kept as a secondary ablation knob
  if the geometric layout helps but plateaus; spacing scheme is itself a research output.
- *More slots at the same 0.8 s window* (denser, not deeper) — cheaper format break but doesn't test the
  "longer horizon for maneuver prediction" hypothesis; deferred to a follow-up ablation.
- *M2-specific layout distinct from M1* — viable (the constant could be per-mode) but adds a second format
  axis; start shared, split only if M2 wants depth M1 doesn't.

**Format-break surface**: `nn_inputs.h` (lags + version), `aircraft_state.h` (`HISTORY_SIZE`),
`evaluator.h` (`kDepth`), `evaluator.cc` (`HIST_PAST[]`), `tests/tick_rescale_tests.cc`,
`tests/nn_layout_tests.cc`, xiao codegen regen. Retrain from scratch; bundled into P0-D's break.

**Success criterion (SC-001, M1-first gated)**: the deeper layout moves an M1 reward-invariant ceiling
(close-track %, median error) at matched seed; only then inherited by M2.

---

## 2. US2 — Two-timescale recurrence (structural slow channel)

**Decision (starting ablation)**: add a **parallel leaky-slow recurrent channel** by widening the recurrent
hidden-2 layer with a fixed-leak slow sub-block — start with the slow channel as a fixed time-constant
(leak α≈0.9 per tick) rather than evolving the time-constant, to keep the search space minimal for the
first pass. Concretely: hidden-2 stays 16-wide fast + an 8-wide slow channel (`h_slow_t = α·h_slow_{t-1} +
(1-α)·tanh(W·x)`), feeding the output layer. May go straight to M2 (mixed policy — M1-first not required).

**Rationale**: `rnn_capacity` SVD shows the 16-dim recurrent layer is NOT width-saturated (eff-rank ~10/16,
90 % of dynamics in 8 modes), so adding *idle width* won't help — but a *structurally distinct* slow
timescale gives the network a trajectory-memory channel the single fast timescale can't represent (the fast
state is being scaled harder, not restructured — `whh_xh_ratio` rising while `w_hh_cv` flat). A fixed leak
is the smallest structural change that introduces a second timescale.

**Alternatives considered**:
- *Evolved per-unit time-constants* (each slow unit learns its α) — more expressive, larger search; promote
  if fixed-leak helps but caps.
- *Second sequential recurrent layer* (`{54,32,16r,8r,3}`) — also a two-timescale structure but couples the
  timescales through depth; harder to attribute. Kept as alternative if the parallel channel underperforms.
- *Make hidden-1 (32-wide) recurrent too* — the 028-deeper-rnn lever; bigger search-space/basin-lottery
  cost. Deferred — start with the targeted slow channel.

**Format-break surface**: `topology.h` (both pathgen + tracker: `NN_TOPOLOGY`, `NN_NUM_LAYERS` if a layer is
added, `NN_RECURRENT`, `NN_WEIGHT_COUNT`, `NN_HIDDEN_STATE_COUNT`, topology string), `nn_xavier_init`
(auto via topology iteration), NN01 fixtures, `nn_layout_tests`/`nn_evaluator_tests`. `nn_forward_recurrent`
is topology-agnostic; `nn2cpp` codegen adapts. The hidden-state allocation in `NNControllerBackend` grows.

**Success criterion (SC-001)**: better tracking and/or healthier `rnn_capacity` (eff-rank using the new
modes) at equal-or-fewer total recurrent parameters vs the single-timescale baseline.

---

## 3. US3 — Auxiliary target-predictor head (the pivot)

**Decision (starting ablation)**: add an **auxiliary output head predicting next-tick target optical state**
— the predicted (beacon_L, beacon_R) NDC centroids (4 values) or the span+tilt (3 values) at +1 tick
(50 ms horizon). Output count grows from 3 (pitch/roll/throttle) to 3+4=7; only the first 3 drive control,
the aux outputs are scored by a **separate lexicase prediction-accuracy axis** (new `ScenarioScore` field),
**never scalar-composited** with the tracking axes. Start pure-evolve (no backprop) so it stays
evolution-native and xiao-deployable. May go straight to M2 (mixed policy).

**Rationale**: 037 found the dominant failure (overrun) is ~87 % prediction-driven (vis-loss + target-turn),
and "once close, track holds ~96 %" — the bottleneck is acquire + reacquire-through-blindness, i.e. the
network needs an explicit motion model. An aux predictor head trained by a prediction-accuracy lexicase
axis gives the network a gradient toward "model the target's motion," which is the substrate for
predict-through-blindness (deferred US5) and planning. It is the highest-leverage, highest-risk study.

**Critical design constraint**: the prediction-accuracy objective MUST be a **separate lexicase axis**
(`pool.push_back({s, &ScenarioScore::prediction_score, eps})`), NOT added into `score` — scalar compositing
of multi-objectives caused the 033 Pareto-corner collapse (`project_scalar_multiobjective_collapse`).
Lexicase keeps it as an independent test dimension.

**Alternatives considered**:
- *Gradient-pretrain-then-evolve hybrid* — could bootstrap the predictor faster, but adds a non-evolution
  training path and complicates determinism/xiao codegen. Deferred — try pure-evolve first; escalate if the
  predictor doesn't develop.
- *Predict target world-position* — rejected: a stale world bearing drifts with ego-rotation in optical
  flight (only a `time-since-seen` scalar survives, per spec). Predict OPTICAL state (NDC/span), unitless.
- *Longer horizon (multi-tick)* — start at +1 tick; lengthen if the 1-tick predictor is too myopic to aid
  reacquire.
- *Recurrent output layer* (give the predictor head memory) — possible but compounds the topology change;
  start feedforward head off the recurrent hidden-2.

**Format-break surface**: `topology.h` (`*_OUTPUT_COUNT` 3→7, weight count), `aircraft_state.h`
(`nnOutputs_[]` size + deserialize output-count check at line 580), `evaluator.h`/`evaluator.cc` (output
buffer, control vs aux split), `fitness_decomposition.h` (new `prediction_score` field + per-tick aux-vs-
realized error accumulation), `selection.cc` (new axis), data.dat columns, NN01 fixtures, xiao codegen
(first-3-outputs-for-control convention; aux outputs sunk). Retrain from scratch.

**Success criterion (SC-002, supporting)**: predicted optical state matches realized within a target error
AND tracking depth/reacquire improves (SC-001) vs control-only baseline.

---

## 4. US4 — Visibility-maintenance reward (FOV-specific, IN SCOPE)

**Decision (starting form)**: add a **new lexicase scenario axis** `ScenarioScore.visibility_score`
(`gp_fitness`, negated lower-better) accumulated per-tick from the EXISTING per-tick CEP/visibility data:
start with a **continuous in-FOV term** `vis_step = -(framed_quality)` where framed_quality rewards low CEP
(tight position quality) for the better-of the two beacons, accumulated × `kCadenceTickScale`. M1 (pathgen)
branch leaves it zero (no `CameraViewSample`). Behind an `EnableVisibilityReward` config knob (X-macro +
fail-loud-no-default per VII).

**Rationale**: directly attacks the ~30 % blind / 8–10 s reacquire ceiling by selecting for flight
geometries that *maintain perception*. The visibility data is already computed (`fitness_decomposition.cc`
:196) as diagnostics — US4 promotes it to a selection axis. As a separate lexicase axis (not scalar
composite) it composes safely with tracking/energy. Can proceed independently of the architecture ablations
(it's a reward-term change, not an NN-contract change) — so it is NOT format-breaking and can land/iterate
without a re-bake gate, though it changes the fitness scale (use fixed-eval comparator).

**Alternatives considered**:
- *Binary CEP-gate* (`vis_step = any_visible ? -1 : 0`) — simplest, but a flat gate gives no gradient to
  *improve* framing once minimally visible. Start continuous; fall back to gate if continuous destabilizes.
- *Fold into the tracking cone* (reshape `decomposeStepScore`) — rejected: that's reward-shaping (037 proved
  exhausted) and would entangle visibility with distance. Keep it a distinct axis.
- *Ramp the visibility reward* via `applyVariationScale` — optional; start un-ramped (full from gen 0, like
  craft variations are diversity) and add a ramp only if early-gen pressure hurts.

**Lexicase-stability check (FR-009 sibling)**: the new axis mixes with existing axes in `selection.cc`;
validate it doesn't destabilize selection (epsilon/aggregation) — especially if `visibility_score` spread
differs in magnitude from `score`/`energy_score` (candidate trigger for the deferred MAD-ε, US3-spec/FR-032).

**Success criterion (SC-003)**: when enabled, improves in-FOV fraction / shortens reacquire without
destabilizing selection or regressing the carried crash penalty.

---

## 5. P0-D — The one clean-slate dmp break

**Decision**: land THREE recording changes as ONE dmp break, no cereal version bump, fail-loud read:

1. **simTimeMsec stamping** — replace the truncating `(unsigned long)(sim_steps*Global::dt*1000)`
   (`SimStateHandler.cpp:392`) with a step-count-derived stamp (`sim_steps * 1000 / stepsPerSec` or round)
   so a 20 Hz run records exact 50 ms gaps. Then revert `crrcsim_tracker_helper.cpp:76` (and
   `tracker_stepper.cc`) from the average-gap test back to a **strict single-gap** test. crrcsim submodule
   change → pointer-bump-first per submodule merge order; determinism-affecting → retrain.
2. **Self-describing dmp** — serialize the fitness/cadence config block (`fitDistScaleBehind/Ahead`,
   `fitConeAngleDeg`, `fitStreakThreshold/RampSec/MultiplierMax`, the cadence `SIM_TIME_STEP_MSEC`/
   `kCadenceTickScale`, and the new crash-penalty + visibility knobs) into `EvalResults` (`protocol.h:365`).
3. **wind_velocity recording** — wire crrcsim's per-tick FDM wind into `AircraftState::setWindVelocity()`
   at the record path (`inputdev_autoc.cpp` ~906, currently never called → zeros). Then re-run
   `wind_study.py` against the real gusty wind.

**Rationale**: 038 re-bakes M1/M2 anyway (the architecture changes are format-breaking), so this is the
agreed single moment to land all deferred recording/determinism items together — one break, not several.

**No-version-bump tension (Principle V)**: V says fully-committed transitions "should bump the version field
directly." The project's no-cereal-versioning practice (greenfield, in-tree, no back-compat) instead keeps
the version field and relies on **fail-loud read** (the V safety net) — a reader hitting an old dmp errors
clearly rather than silently mis-parsing. This is the operator-sanctioned resolution
(`feedback_no_cereal_versioning`); 038 follows it: NO bump, readers fail loud, old dmps orphaned (retrain).

**P0-B (renderer reads dmp, not ini)**: once the config block is in the dmp (item 2), flip
`renderer.cc:2933` and `dmp_dump.cc:406` to prefer dmp-recorded config over `ConfigManager::getConfig()`,
so a drifted `.ini` can't misrender a pinned run. The only remaining ini dependence is the S3 bucket/profile.

---

## 6. P0-A — 033 PRNG cascade validation

**Decision**: validate (don't redesign) the cascade. Run the existing D1–D5 contract tests
(`tests/scenario_prng_tests.cc`) + the eval-mode replay test (`tests/eval_mode_replay_tests.cc`, D4
cross-process bitwise), and add a determinism check that regenerating a scenario from its persisted
`scenarioSeed` reproduces the exact class sub-seeds and per-scenario draws. Produce a **written verdict**
(clear / bug-found-and-fixed) per FR-P0A.

**Rationale**: the cascade is append-only and frozen (D5 guarantees adding the 6th camera class doesn't
perturb the first 5); the suspected single-SHA bug from BACKLOG is the thing to confirm-or-find before any
honest variation work. Validation is the deliverable, not a code change (unless a bug surfaces).

**Alternatives considered**: re-seeding crash-hull PRNG from windSeed — explicitly NOT re-opened (already
fixed in the 033 cleanup; seeds from the per-scenario rabbit-class sub-seed). Don't touch it.

---

## 7. Cross-cutting decisions

- **Ablation protocol (parallel, combine winners)**: bake US1/US2/US3 as independent variants off the SAME
  post-P0-D baseline (same seed, same source), compare on the SC-001 ceilings + fixed-eval comparator, then
  combine the winning structural changes into a single architecture for the final M2. Each ablation is a
  research spike (Constitution I exemption) until mainlined.
- **Mixed M1-first**: US1 (history) MUST show an M1 lift first (clean, no FOV confound). US2/US3 may go
  straight to M2 (M1 has less to gain from recurrence-memory/predictor of an always-visible rabbit).
- **Regression gate per format change**: each format-breaking change runs through `rebuild-perf.sh` +
  the eval-vs-training bitwise `ScenarioScore` gate (operator-driven) before a larger bake; the pre-run
  build gate (Constitution IX) is the cheap insurance.
- **Type-domain (VI)**: all new eval/nn scalars `gp_scalar`/`gp_fitness`; NN-byte buffers `// raw-ok:`;
  P0-E grep audit on touched `src/eval/ src/nn/` at each `/speckit.implement` close.
- **No silent fallback (VII)**: new config-supplied members (predictor params, visibility-reward knobs) get
  no in-class defaults — single assignment site.
- **Artifact naming (VIII)**: bakes named `autoc-038-tN-<details>` (lexicographic); pin only a milestone
  baseline and record its S3 prefix in the outcome doc.

---

## Open items explicitly NOT resolved here (research outputs of the bakes)

Per spec Out-of-Scope, the FINAL forms are bake outputs, not plan commitments: the exact winning history
layout, the winning recurrence structure, the predictor head/objective final shape, and whether MAD-ε
(FR-032) must be promoted once the new axes land. research.md gives each a concrete *starting* configuration
+ decision criteria so the bakes have a defined entry point.

# Tasks: NN Training Improvements

**Input**: Design documents from `specs/015-nn-training-improvements/`

---

## Status Summary (completed phases)

### Phase 1: Sigma Floor — DONE
Mutation sigma clamped to configurable floor. Prevents search freeze on long runs.

### Phase 2: Fitness Decomposition — DONE
Per-scenario × per-component scores (completion, distance RMSE, attitude, smoothness[3])
extracted from EvalResults. Logged per-generation for best individual.

### Phase 3: Lexicase Selection — DONE
Epsilon-lexicase replaces scalar tournament. Priority: completion_fraction → distance_rmse.
Minimax and sum modes retained for comparison. Config: `SelectionMode = lexicase`.

### Phase 4a: Sensor Expansion — DONE
- 29 inputs (up from 22): 6 temporal slots for dPhi/dTheta/dist, dDist/dt, quaternion, airspeed, alpha, beta, cmd feedback
- Topology: 29→16→8→3 = 616 weights. Clean break — old weights incompatible.
- Normalizers (NORM_DIST, NORM_VEL, NORM_ANGLE, NORM_RATE) removed — raw values
- att= now uses quaternion geodesic distance (2*acos(|q1·q2|)) — no gimbal lock, logging only
- Smoothness as lexicase dimension: **tried and reverted**. Pegged outputs (Δu=0) score
  perfectly smooth → reinforces spiral (pitch=max, throttle=max, roll-only). Observed gen 73:
  sm=0.00/0.41/0.00. Path-relative smoothness (normalize Δu by path curvature) is the
  correct formulation — deferred to Phase 7.
- WindScenarios: 9→25 for richer lexicase test-case diversity

**Observed outcome**: Sensors helped tracking (dist 30–60m → 2–12m), but spiral persists.
All 25 scenarios complete with throttle sm=0.00 (pegged). The NN found a fixed-throttle
strategy that works on all wind seeds. Selection (completion→distance) doesn't penalize it.

---

## Current situation: the spiral problem

The spiral exploit survives because:
1. The path is fixed geometry — a fixed spiral can stay near the rabbit
2. Lexicase only asks: *did you survive?* and *did you stay close?* — spiral answers both
3. No test case asks: *can you acquire the path from a cold start?*
4. No penalty for high throttle / high energy

The three levers to break it (in priority order):
- **Intercept/entry phase**: force path acquisition from varied position/heading/speed — a spiral can't intercept, only maintain
- **Energy pressure**: penalize high throttle — directly attacks the exploit
- **Path shape diversity**: varied geometry so no fixed strategy dominates all scenarios

---

## Phase 4b: Anti-Spiral Selection Pressure

**Goal**: Add energy as a lexicase dimension. Quick to implement, directly attacks
throttle saturation without requiring intercept or path changes.

**Hypothesis**: Among individuals with similar completion and distance, prefer lower
mean throttle. This should push the search toward efficient flight paths.

**Note**: mean throttle is already in smoothness[2] — no new computation needed.

- [ ] T100 Fix build: NORM_ANGLE/NORM_DIST/NORM_VEL/NORM_RATE removed from topology.h
  but still referenced in tests/nn_evaluator_tests.cc — remove stale test lines
- [ ] T101 Add mean throttle (energy) as 3rd lexicase dimension: completion → distance → energy
  in src/eval/lexicase_selection.cc. Use smoothness[2] (already computed). Epsilon same as distance.
- [ ] T102 Run 50-gen experiment, log throttle distribution — does mean throttle drop?
  Does spiral break or just become a more efficient spiral?

**Checkpoint**: Energy dimension tested. If throttle drops significantly → keep. If spiral
just adapts → confirms intercept phase is the required fix.

---

## Phase 4c: Intercept / Entry Phase — highest leverage

**Goal**: Force the aircraft to *acquire* the path from a cold-start offset, not just
maintain proximity. A spiral can only maintain — it cannot intercept.

**Why this is the real fix**: The spiral is a degenerate solution that only works when
already near the path. If every episode begins off-path (random position offset,
heading offset, approach speed), the NN must develop genuine path acquisition behavior.
Completion becomes hard again and creates real lexicase differentiation. This is also
the real-world mission requirement.

**Design questions to resolve before implementation**:
- What is the entry state distribution? (position offset range, heading offset, speed range)
- Does crrcsim support starting the aircraft at an arbitrary position/heading relative to the path?
- Is this a new eval phase (intercept + track) or just varied initial conditions?
- How does completion_fraction handle the intercept phase? (time to acquire vs time on path)

- [ ] T110 Audit current entry variation: what does `EntryRollSigma=22.5` actually do?
  Read crrcsim inputdev_autoc.cpp entry logic. Is position offset already supported?
- [ ] T111 Design intercept phase: define entry state distribution (position: ±Xm from path
  start, heading: ±Y°, speed: Z m/s range). Document in specs/015.../intercept-design.md
- [ ] T112 Implement varied entry states in crrcsim — randomize initial position/heading
  relative to path start using seed from scenario index
- [ ] T113 Update completion_fraction to include intercept: time-to-acquire counts against
  completion, or add intercept_success as a new lexicase dimension
- [ ] T114 Run 50-gen experiment with intercept — does spiral fail to complete? Does
  lexicase find genuine path-acquisition behavior?
- [ ] T115 Revisit lexicase ordering once intercept is working: completion → energy → distance
  may be better ordering at this point (survive, then be efficient, then be precise)

**Checkpoint**: Spiral fails to complete intercept phase. Lexicase differentiates on
acquisition ability. Throttle modulation emerges naturally.

---

## Phase 4d: Path Shape Diversity

**Goal**: 25 wind seeds on the same path geometry still lets a fixed spiral win.
Need varied *path shapes* so no single fixed strategy dominates all scenarios.

**Note**: This is a path generator problem, not config tuning. The current rabbit
path is a fixed circuit — more wind variations just perturb it, not change its topology.

- [ ] T120 Audit path generator: what parameters control path shape vs path speed?
  What's the range of achievable geometries from current config?
- [ ] T121 Add path shape variation per scenario: vary curvature (turn rate, segment length)
  across the 25 scenarios, not just wind. Goal: some scenarios have tight turns, some
  have reversals, some have altitude changes
- [ ] T122 Experiment: reduced rabbit speed (RabbitSpeedSigma=1.0) for 50 gens —
  does throttle modulation appear? (T049 from old plan)
- [ ] T123 Experiment: variable rabbit (sigma=3-5 m/s, aggressive speed cycles) —
  does closing rate signal (dDist/dt) become active? (T050 from old plan)

**Checkpoint**: Path shape diversity makes fixed-throttle spiral nonviable across
the scenario set. Lexicase finds scenario-specialist behavior.

---

## Phase 5: Path-Relative Smoothness — deferred

**Goal**: Correct formulation of smoothness: normalize Δu by the path curvature at
each step. Penalize control change *above what the path demands*.

**Why deferred**: Requires per-step curvature from path geometry (radiansFromStart or
per-segment curvature derivative). The path has this data but instrumenting it in the
fitness pipeline is non-trivial. Correct in principle — do after intercept works.

- [ ] T130 Instrument per-step path curvature in EvalResults (or derive from existing path data)
- [ ] T131 Compute path-normalized smoothness: |Δu(t)| / max(curvature(t), ε)
- [ ] T132 Add as lexicase dimension after completion and distance
- [ ] T133 Run experiment — does path-normalized smoothness reward coordinated turns
  while penalizing oscillation on straight segments?

---

## Phase 6: Research Spikes — after intercept validated

**Goal**: Explore alternative selection strategies once the fitness signal is trustworthy
(post-intercept, post-energy). Each on its own branch.

### NSGA-II Pareto Spike
- [ ] T140 Branch `015-spike-nsga2`: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- [ ] T141 Run 500-gen experiment, compare Pareto front evolution against lexicase

### Rank-Based Fitness Shaping Spike
- [ ] T142 Branch `015-spike-rank-shaping`: CMA-ES style rank-derived weights
- [ ] T143 Run 500-gen experiment, compare against lexicase baseline

### Analysis
- [ ] T144 Compare all spike results, document in research.md — which strategy broke the plateau?
- [ ] T145 Merge winner to 015-nn-training-improvements

---

## Phase 7: CMA-ES Optimizer — after fitness signal is trustworthy

**Goal**: Replace GA with sep-CMA-ES. Pop 5000→50. Only worth doing once fitness signal
gives real gradient — with a spiral exploit in place, CMA-ES will just optimize the spiral faster.

- [ ] T150 Branch `015-spike-cmaes`
- [ ] T151 Implement SepCMAES (ask/tell) in include/autoc/nn/sep_cmaes.h + src/nn/sep_cmaes.cc
- [ ] T152 Wire into evolution loop, run 500-gen experiment with pop=50
- [ ] T153 If validated, merge and remove GA code

---

## Phase 8: Polish

- [ ] T160 Verify all 3 repo builds: autoc, crrcsim, xiao
- [ ] T161 Run full test suite: `cd build && ctest --output-on-failure`
- [ ] T162 Verify export pipeline: nnextractor → nn2cpp → xiao build
- [ ] T163 GP legacy cleanup: scan for GP-era remnants (`GPrandDouble`, `gp_` prefixes,
  `#ifndef` guards → `#pragma once`). At minimum: include/autoc/util/gp_math_utils.h,
  include/autoc/util/types.h
- [ ] T164 Consider removing legacy scalar fitness path (aggregateScalarFitness) —
  only used for logging now, not selection. Replace with mean distance RMSE summary.

---

## Priority Order

1. **T100** — fix build (blocker)
2. **T101–T102** — energy lexicase (quick experiment, tests hypothesis cheaply)
3. **T110–T115** — intercept/entry phase (highest leverage, fundamental fix)
4. **T120–T123** — path shape diversity (complements intercept)
5. **T130–T133** — path-relative smoothness (correct formulation, after above works)
6. **T140–T145** — research spikes (after fitness signal is trustworthy)
7. **T150–T153** — CMA-ES (last, needs good gradient)
8. **T160–T164** — polish

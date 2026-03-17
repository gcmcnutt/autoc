# Tasks: NN Training Improvements

**Input**: Design documents from `specs/015-nn-training-improvements/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US9, US3)
- Include exact file paths in descriptions

---

## Phase 1: Sigma Floor (US1, P1) — MVP

**Goal**: Prevent search freeze by clamping mutation sigma to configurable minimum

**Independent Test**: Run evolution for 50+ gens with NNSigmaFloor=0.05, verify sigma never drops below 0.05 in console output

### Contract Tests

- [ ] T001 [P] [US1] Write test: sigma clamped to floor when it would decay below in tests/sigma_floor_tests.cc
- [ ] T002 [P] [US1] Write test: sigma floor = 0 produces identical behavior to unclamped in tests/sigma_floor_tests.cc
- [ ] T003 [P] [US1] Write test: warn if NNSigmaFloor > NNMutationSigma in tests/sigma_floor_tests.cc

### Implementation

- [ ] T004 [US1] Add NNSigmaFloor config key (default: 0) to config parser in src/util/config.cc and include/autoc/util/config.h
- [ ] T005 [US1] Implement sigma floor clamping in nn_gaussian_mutation() in src/nn/population.cc — clamp sigma after self-adaptive update, log clamping event
- [ ] T006 [US1] Add sigma floor validation: warn at startup if floor > initial sigma in src/autoc.cc
- [ ] T007 [US1] Run tests: `cd build && ctest -R sigma_floor --output-on-failure`
- [ ] T008 [US1] Integration test: run `./build/autoc` for 20 gens with NNSigmaFloor=0.05, verify sigma ≥ 0.05 in console output

**Checkpoint**: Sigma floor working. Can run long experiments without sigma collapse.

---

## Phase 2: Fitness Decomposition in autoc (US9, P1) — see the signal

**Goal**: Refactor computeNNFitness() to compute and retain per-scenario × per-component scores from existing EvalResults data. No RPC or worker change.

**Objective priority hierarchy** (from calm-air diagnostic + Pareto lessons):
1. **completion_fraction** (0.0-1.0) — did it survive? Crash = natural truncation, no separate penalty.
2. **attitude_error** — normalized by completion fraction. Don't tumble/spiral.
3. **distance_rmse** — tracking accuracy over completed timesteps.
4. **smoothness[3]** — computed but may not drive selection initially.

**Independent Test**: Run evolution, verify per-scenario breakdown logged for best individual. Verify aggregate from components matches legacy scalar exactly.

### Contract Tests

- [X] T010 [P] [US9] Write test: completion_fraction = 0.5 when crash at step 83 of 167 in tests/fitness_decomposition_tests.cc
- [X] T011 [P] [US9] Write test: per-scenario distance RMSE computation matches expected values for known aircraft states in tests/fitness_decomposition_tests.cc
- [X] T012 [P] [US9] Write test: per-scenario attitude error normalized by completion fraction in tests/fitness_decomposition_tests.cc
- [X] T013 [P] [US9] Write test: per-scenario smoothness (Σ|Δu(t)|) computation for bang-bang input ≈ 2.0, constant ≈ 0.0 in tests/fitness_decomposition_tests.cc
- [X] T014 [US9] Write test: aggregate of decomposed scores matches legacy computeNNFitness() output exactly in tests/fitness_decomposition_tests.cc

### Implementation

- [X] T015 [US9] Define ScenarioScore struct (completion_fraction, attitude_error, distance_rmse, smoothness[3]) in include/autoc/eval/fitness_decomposition.h — crash handling via truncation, no separate crash penalty (Pareto lesson)
- [X] T016 [US9] Add per-scenario score storage to NNGenome: vector<ScenarioScore> scenario_scores in include/autoc/nn/evaluator.h
- [X] T016 [US9] Extract per-scenario scoring from computeNNFitness() into computeScenarioScores() in src/eval/fitness_decomposition.cc — compute distance RMSE, attitude error, smoothness per scenario from existing EvalResults aircraft states
- [X] T017 [US9] Add backward-compat aggregate: aggregateScalarFitness() that recombines ScenarioScores into legacy scalar in src/eval/fitness_decomposition.cc
- [X] T018 [US9] Wire into evolution loop: call computeScenarioScores() then aggregateScalarFitness() in place of computeNNFitness() in src/autoc.cc:616
- [X] T019 [US9] Log per-scenario breakdown for best individual each generation (distance RMSE, attitude, smoothness per scenario) in src/autoc.cc
- [X] T020 [US9] Run tests: `cd build && ctest -R fitness_decomposition --output-on-failure`
- [X] T021 [US9] Integration test: run `./build/autoc` for 10 gens, verify per-scenario log output and that aggregate matches legacy behavior

**Checkpoint**: Per-scenario scores visible. Aggregate unchanged. Ready for multi-objective selection.

---

## Phase 3: Multi-Objective Selection (US3, P1) — core hypothesis test

**Goal**: Replace single-scalar selection with survival-first multi-objective selection.
The calm-air diagnostic showed the controller learns one turn but can't handle the reversal —
scalar fitness can't express "left loop = great, right loop = crash." Selection must let
the optimizer see *which part* needs improvement.

**Primary approach**: Lexicase selection (survival-first, per-scenario test cases)
**Comparison**: Minimax aggregation on worst-scenario completion fraction

**Constraints (from Pareto experiment)**: Max 2-3 objectives. Crash = natural truncation.
No weights during selection. Weights only at extraction.

**Independent Test**: Run calm-air single-scenario for 200 gens with lexicase vs sum. Does
the controller learn to complete both loops?

**Depends on**: Phase 2 (per-scenario component scores)

### Contract Tests

- [X] T025 [P] [US3] Write test: lexicase with 3 scenarios selects individual with best completion on randomly-ordered first scenario in tests/selection_tests.cc
- [X] T026 [P] [US3] Write test: epsilon-lexicase with continuous completion_fraction uses epsilon threshold in tests/selection_tests.cc
- [X] T027 [P] [US3] Write test: minimax on completion fractions {1.0, 1.0, 0.5} returns 0.5 in tests/selection_tests.cc
- [X] T028 [P] [US3] Write test: sum mode produces identical result to legacy aggregate in tests/selection_tests.cc

### Implementation

- [X] T029 [US3] Implement epsilon-lexicase selection in src/eval/lexicase_selection.h and src/eval/lexicase_selection.cc — shuffle scenarios, filter by epsilon-best on completion_fraction first, then distance_rmse, repeat until 1 remains
- [X] T030 [US3] Implement minimax aggregation on worst-scenario completion_fraction in src/eval/fitness_aggregator.h and src/eval/fitness_aggregator.cc
- [X] T031 [US3] Add FitnessAggregation config key (sum/minimax/lexicase) to src/util/config.cc and include/autoc/util/config.h
- [X] T032 [US3] Wire selection strategy into evolution loop: replace tournament-on-scalar with configurable selection in src/nn/population.cc and src/autoc.cc
- [X] T033 [US3] Log selection mode and per-generation statistics (completion fraction distribution, best/worst per scenario) in src/autoc.cc
- [X] T034 [US3] Run tests: `cd build && ctest -R selection --output-on-failure`
- [X] T035 [US3] Integration test: calm-air single-scenario, lexicase vs sum for 200 gens — does controller complete both loops?
- [X] T036 [US3] Integration test: 9-scenario with entry variations, lexicase vs sum — does worst-scenario completion improve?

**Checkpoint**: Lexicase selection working. Core hypothesis testable: does survival-first
selection break the plateau? Compare against sum and minimax baselines.

---

## Phase 4a: Enhanced NN Sensors (US13, P1) — anti-spiral, throttle signal

**Goal**: Give the NN direct closing-rate and lookahead signals so it can modulate throttle
instead of full-blast spiraling. Expand temporal history for curve-ahead estimation.

**Current problem**: NN has no direct closing-rate input. It sees distance at 4 time offsets
and must implicitly compute dDist/dt. Full-throttle spiral is an exploit — the NN can't
distinguish "need more speed" from "need to turn harder."

**Depends on**: Phase 3 (lexicase + smoothness working)

### Sensor Changes

New input layout (29 inputs, up from 22):
```
 0- 5: dPhi  [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]  (raw, no NORM)
 6-11: dTheta [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]  (raw, no NORM)
12-17: dist  [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]   (raw, no NORM)
   18: dDist/dt (closing rate, m/s)                        (raw)
19-22: quaternion (w, x, y, z)                             (already [-1,1])
   23: airspeed (m/s)                                      (raw, no NORM_VEL)
   24: alpha (rad)                                         (raw, no NORM_ANGLE)
   25: beta (rad)                                          (raw, no NORM_ANGLE)
26-28: rollCmd, pitchCmd, throttleCmd feedback              (already [-1,1])
```

Topology: 29→16→8→3 = 616 weights (up from 531). Clean break — all prior weights incompatible.

### Tasks

- [ ] T040 [US13] Update NN_INPUT_COUNT to 29 and topology to {29,16,8,3} in include/autoc/nn/evaluator.h
- [ ] T041 [US13] Remove NORM_ANGLE, NORM_DIST, NORM_VEL, NORM_RATE constants from include/autoc/nn/evaluator.h — use raw sensor values
- [ ] T042 [US13] Expand temporal history from 4 slots to 6 slots (add +0.1s, +0.5s lookahead) in nn_gather_inputs() — both autoc/src/autoc.cc and crrcsim inputdev_autoc.cpp
- [ ] T043 [US13] Add dDist/dt (closing rate) input: compute from dist temporal history in nn_gather_inputs()
- [ ] T044 [US13] Update nn_gather_inputs() input ordering to match new layout
- [ ] T045 [US13] Update xiao nn_program.h input layout to match (xiao/include/nn_program.h)
- [ ] T046 [US13] Update nn2cpp tool to generate correct input count
- [ ] T047 [US13] Update all tests: nn_evaluator_tests.cc, contract_evaluator_tests.cc — new input count, removed norms
- [ ] T048 [US13] Rebuild all 3 repos: autoc, crrcsim, xiao
- [ ] T049 [US13] Diagnostic run: lexicase + smoothness, 9 wind scenarios, constant rabbit — verify dist signal improves, throttle not pegged at max
- [ ] T050 [US13] Experiment: variable rabbit (sigma=3-5 m/s, aggressive cycles=[0.3, 2.0]) — verify throttle modulation and closing rate signal visible in per-scenario logs
- [ ] T051 [US13] Tune rabbit speed config: test wider speed range (sigma=5+), shorter cycles, verify scenarios create meaningful throttle diversity for lexicase

**Checkpoint**: NN has direct closing-rate signal. Throttle modulation emerges under variable rabbit. Full-throttle spiral strategy no longer viable.

---

## Phase 4b: Research Spikes (US12, P2) — branched experiments

**Goal**: Explore alternative selection strategies. Each on its own git branch, validated against Phase 3 baseline.

**Independent Test**: Each spike produces a 500-gen run with per-scenario fitness logs. Compare against lexicase baseline on same scenarios.

**Depends on**: Phase 4a (enhanced sensors)

### NSGA-II Pareto Spike

- [ ] T055 [US12] Create branch `015-spike-nsga2` from 015-nn-training-improvements
- [ ] T056 [P] [US12] Write test: non-dominated sort on known 2-objective vectors in tests/nsga2_tests.cc
- [ ] T057 [P] [US12] Write test: crowding distance computation in tests/nsga2_tests.cc
- [ ] T058 [US12] Implement non-dominated sort + crowding distance selection in src/eval/nsga2_selection.cc — objectives: tracking RMSE, smoothness, worst-case spread
- [ ] T059 [US12] Wire NSGA-II selection into evolution loop in src/nn/population.cc
- [ ] T060 [US12] Run 500-gen experiment, log Pareto front evolution, compare against lexicase

### Rank-Based Fitness Shaping Spike

- [ ] T061 [US12] Create branch `015-spike-rank-shaping` from 015-nn-training-improvements
- [ ] T062 [US12] Implement rank-based fitness transformation (CMA-ES style) in src/eval/fitness_aggregator.cc — replace raw fitness with rank-derived weights
- [ ] T063 [US12] Run 500-gen experiment, compare against lexicase baseline

### Research Analysis

- [ ] T064 [US12] Compare all spike results: plot per-scenario fitness trajectories, smoothness evolution, worst-case convergence
- [ ] T065 [US12] Document findings in specs/015-nn-training-improvements/research.md — which strategy broke the plateau?
- [ ] T066 [US12] Merge winning strategy to 015-nn-training-improvements, remove losing branches

**Checkpoint**: Best selection strategy identified and merged. Decision documented.

---

## Phase 5: Smoothness as Selection Pressure (US10, P2) — deferred

**Goal**: Add control smoothness as explicit selection pressure, informed by Phase 4 findings

**Depends on**: Phase 4 (research determines how to apply smoothness — objective vs constraint)

- [ ] T060 [US10] Based on Phase 4 findings, decide: smoothness as Pareto objective, lexicase dimension, or constraint threshold
- [ ] T061 [P] [US10] Write test: smoothness threshold rejects bang-bang individuals in tests/smoothness_tests.cc
- [ ] T062 [US10] Integrate smoothness into winning selection strategy in src/eval/ and src/nn/population.cc
- [ ] T063 [US10] Run 500-gen experiment, verify bang-bang suppression and tracking quality maintained
- [ ] T064 [US10] Update autoc.ini with smoothness-related config keys

**Checkpoint**: Controllers produce smooth outputs suitable for physical servos.

---

## Phase 6: Per-Segment Temporal Credit (US11, P2) — deferred

**Goal**: Reward error reduction on hard maneuvers, not just low absolute error

**Depends on**: Phase 4 (confirm temporal decomposition helps)

- [ ] T070 [P] [US11] Write test: segment boundaries detected at path curvature changes in tests/segment_scorer_tests.cc
- [ ] T071 [P] [US11] Write test: error_reduction = start_distance - end_distance for known segment in tests/segment_scorer_tests.cc
- [ ] T072 [P] [US11] Write test: difficulty_weight increases with turn rate and crosswind in tests/segment_scorer_tests.cc
- [ ] T073 [US11] Implement segment boundary detection from path geometry in src/eval/segment_scorer.cc
- [ ] T074 [US11] Implement segment scoring: error_reduction × difficulty_weight in src/eval/segment_scorer.cc
- [ ] T075 [US11] Wire segment scores into fitness aggregation pipeline in src/autoc.cc
- [ ] T076 [US11] Run tests: `cd build && ctest -R segment --output-on-failure`
- [ ] T077 [US11] Run 500-gen experiment, verify credit flows to hard segments

**Checkpoint**: Temporal credit assignment working. Controllers improve on hard maneuvers.

---

## Phase 7: sep-CMA-ES Optimizer (US4, P3) — branch experiment

**Goal**: Replace GA with sep-CMA-ES. Pop 5000→50. Explore after fitness signal is sorted.

**Depends on**: Phase 4 (need good fitness signal first)

- [ ] T080 [US4] Create branch `015-spike-cmaes` from 015-nn-training-improvements
- [ ] T081 [P] [US4] Write test: sep-CMA-ES converges on Rosenbrock (N=10) in tests/cmaes_tests.cc
- [ ] T082 [P] [US4] Write test: ask() generates lambda candidates in tests/cmaes_tests.cc
- [ ] T083 [P] [US4] Write test: tell() updates mean/sigma/diagonal_cov in tests/cmaes_tests.cc
- [ ] T084 [US4] Implement SepCMAES class (ask/tell) in include/autoc/nn/sep_cmaes.h and src/nn/sep_cmaes.cc — Eigen vectors, hyperparams from research.md
- [ ] T085 [US4] Wire into evolution loop: ask/tell replaces mutate/select in src/autoc.cc
- [ ] T086 [US4] Run 500-gen experiment with pop=50, compare against GA baseline on same fitness
- [ ] T087 [US4] If validated, merge to 015; remove GA code

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final verification after research converges

- [ ] T090 Verify all 3 repo builds: autoc (`scripts/rebuild.sh`), CRRCSim (`make`), xiao (`pio run`)
- [ ] T091 Run full test suite: `cd build && ctest --output-on-failure`
- [ ] T092 Verify export pipeline: nnextractor → nn2cpp → xiao build
- [ ] T093 Update autoc.ini with all new config keys and sensible defaults
- [ ] T094 Run quickstart.md validation: follow all steps and verify they work
- [ ] T095 GP legacy cleanup: scan all source files (src/, include/, tools/, xiao/, crrcsim/) for GP-era remnants — `GP[A-Z]` identifiers (GPrandDouble, GPConfiguration, etc.), `gp_` prefixed functions/variables, `#ifndef` include guards (convert to `#pragma once`), GP references in comments. Rename or remove all vestiges. Covers at least: include/autoc/util/gp_math_utils.h, include/autoc/util/types.h, and any other files found by case-insensitive grep for `\bgp[_A-Z]`
- [ ] T096 Consider removing legacy scalar fitness path: with lexicase selection, the single fitness number is only used for logging — not selection, not elitism ranking. Options: (a) remove aggregateScalarFitness() and power/norm constants entirely, (b) replace with a simple human-readable summary (mean distance RMSE across scenarios), (c) keep for A/B comparison with sum mode. Also consider: crashReasonToString() duplicated in minisim.cc and inputdev_autoc.cpp — should be a single definition in a shared header.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Sigma Floor)**: No dependencies — start immediately
- **Phase 2 (Fitness Decomposition)**: No dependencies — can parallel with Phase 1
- **Phase 3 (Multi-Objective Selection)**: Depends on Phase 2
- **Phase 4 (Research Spikes)**: Depends on Phase 3
- **Phase 5 (Smoothness)**: Depends on Phase 4 findings
- **Phase 6 (Segment Scoring)**: Depends on Phase 4 findings
- **Phase 7 (CMA-ES)**: Depends on Phase 4 (good fitness signal)
- **Phase 8 (Polish)**: After all active phases complete

### Within Each Phase

- Contract tests written FIRST, must FAIL before implementation
- Implementation follows test structure
- Integration test validates end-to-end
- Commit after each logical group

### Parallel Opportunities

- Phase 1 and Phase 2 are fully independent — can run in parallel
- Within Phase 2: T010, T011, T012 (scenario score tests) can run in parallel
- Within Phase 3: T025, T026, T027 (aggregator tests) can run in parallel
- Within Phase 4: Lexicase and NSGA-II spikes can run on separate branches in parallel
- Phase 5-7 depend on Phase 4 findings but are independent of each other

---

## Implementation Strategy

### MVP First (Phases 1-2)

1. Phase 1: Sigma floor — unblock running experiments
2. Phase 2: Fitness decomposition — see per-scenario signal
3. **STOP and VALIDATE**: Run long experiment, examine per-scenario logs

### Core Hypothesis Test (Phase 3-4)

1. Phase 3: Minimax selection
2. Phase 4: Research spikes
3. **STOP and DECIDE**: Which strategy breaks the plateau?

### Deferred (Phases 5-7)

Informed by Phase 4 findings. Order and scope TBD based on research results.

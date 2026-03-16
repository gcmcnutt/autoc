# Tasks: NN Training Improvements

**Input**: Migrated from 014 Phases 6-13
**Prerequisites**: 014 complete (standalone NN-only autoc, clean build)

## Format: `[ID] [P?] [Story] Description`

---

## Phase 1: Sigma Floor (US1, P1) — MVP

**Goal**: Prevent search freeze by clamping mutation sigma

### Contract Tests

- [ ] T001 [US1] Write test: sigma clamped to floor when it would decay below in tests/sigma_floor_tests.cc
- [ ] T002 [US1] Write test: sigma floor = 0 produces identical behavior to unclamped in tests/sigma_floor_tests.cc

### Implementation

- [ ] T003 [US1] Add NNSigmaFloor config key (default: 0) to config parser in src/util/config.cc
- [ ] T004 [US1] Implement sigma floor clamping in NNPopulation::mutate() in src/nn/nn_population.cc — clamp sigma after self-adaptive update, log clamping event
- [ ] T005 [US1] Add sigma floor edge case: warn if floor > initial sigma in src/nn/nn_population.cc
- [ ] T006 [US1] Run tests: `cd build && ctest -R sigma_floor --output-on-failure`
- [ ] T007 [US1] Integration test: run `./build/autoc` for 20 generations with NNSigmaFloor=0.05, verify sigma never drops below 0.05 in console output

---

## Phase 2: Curriculum Scenario Ramp (US2, P1)

**Goal**: Progressive scenario difficulty ramping for NN evolution

### Contract Tests

- [ ] T010 [US2] Write test: CurriculumSchedule parses "1:50,7:150,49:0" into 3 stages in tests/curriculum_tests.cc
- [ ] T011 [US2] Write test: stage transitions occur at correct generation boundaries in tests/curriculum_tests.cc
- [ ] T012 [US2] Write test: disabled curriculum uses all scenarios from gen 0 in tests/curriculum_tests.cc

### Implementation

- [ ] T013 [US2] Create CurriculumSchedule class in include/autoc/eval/curriculum.h and src/eval/curriculum.cc
- [ ] T014 [US2] Add CurriculumEnabled and CurriculumSchedule config keys to config parser in src/util/config.cc
- [ ] T015 [US2] Wire CurriculumSchedule into NN evolution loop in src/autoc.cc — query active scenario count per generation, log stage transitions
- [ ] T016 [US2] Run tests: `cd build && ctest -R curriculum --output-on-failure`
- [ ] T017 [US2] Integration test: run `./build/autoc` for 20 generations with CurriculumSchedule=1:5,7:15,49:0, verify stage transitions in console output

---

## Phase 3: Fitness Decomposition (US3, P2)

**Goal**: Minimax or percentile fitness aggregation across scenarios

### Contract Tests

- [ ] T020 [US3] Write test: minimax returns worst-case scenario score in tests/fitness_aggregator_tests.cc
- [ ] T021 [US3] Write test: percentile returns correct value at 95th percentile in tests/fitness_aggregator_tests.cc
- [ ] T022 [US3] Write test: sum mode matches current behavior in tests/fitness_aggregator_tests.cc

### Implementation

- [ ] T023 [US3] Create FitnessAggregator class in include/autoc/eval/fitness_aggregator.h and src/eval/fitness_aggregator.cc
- [ ] T024 [US3] Add FitnessAggregation and FitnessPercentile config keys to config parser in src/util/config.cc
- [ ] T025 [US3] Wire FitnessAggregator into fitness computation in src/autoc.cc — replace sum-over-scenarios with aggregator call
- [ ] T026 [US3] Run tests: `cd build && ctest -R fitness_aggregator --output-on-failure`
- [ ] T027 [US3] Integration test: run `./build/autoc` for 20 generations with FitnessAggregation=minimax, verify worst-case scenario fitness drives selection in console output

---

## Phase 4: sep-CMA-ES Optimizer (US4, P2)

**Goal**: Replace GA with sep-CMA-ES for 531-dim weight optimization

### Contract Tests

- [ ] T030 [US4] Write test: CMA-ES converges on Rosenbrock function (N=10) in tests/cmaes_tests.cc
- [ ] T031 [US4] Write test: ask() generates lambda candidates from distribution in tests/cmaes_tests.cc
- [ ] T032 [US4] Write test: tell() updates mean/sigma/covariance correctly in tests/cmaes_tests.cc
- [ ] T033 [US4] Write test: CMA-ES state serialization round-trips in tests/cmaes_tests.cc

### Implementation

- [ ] T034 [US4] Implement SepCMAES class in include/autoc/nn/sep_cmaes.h and src/nn/sep_cmaes.cc — ask/tell interface, Eigen vectors, hyperparams from research.md (lambda=50, mu=25, sep-scaled learning rates)
- [ ] T035 [US4] Add OptimizerType config key ("ga" or "sep-cma-es") to config parser in src/util/config.cc
- [ ] T036 [US4] Wire SepCMAES into evolution loop in src/autoc.cc — replace mutate/crossover/select with ask/tell when OptimizerType=sep-cma-es
- [ ] T037 [US4] Add CMA-ES logging: sigma, best fitness, mean fitness per generation in src/autoc.cc
- [ ] T038 [US4] Add CMA-ES state to S3 archive format in src/nn/nn_serialization.cc
- [ ] T039 [US4] Run tests: `cd build && ctest -R cmaes --output-on-failure`
- [ ] T040 [US4] Integration test: run `./build/autoc` for 20 generations with OptimizerType=sep-cma-es and PopulationSize=50, verify sigma/fitness logging and fitness improvement

---

## Phase 5: Per-Timestep Streaming (US7, P3)

**Goal**: Return per-timestep data from simulator (enables US5)

- [ ] T050 [US7] Define TimestepRecord struct in include/autoc/eval/timestep_record.h
- [ ] T051 [US7] Add per-timestep data collection in minisim evaluation loop in src/minisim.cc
- [ ] T052 [US7] Extend EvalResponse in rpc_protocol.h to include per-timestep data
- [ ] T053 [US7] Update RPC serialization in src/minisim.cc to send per-timestep data when enabled
- [ ] T054 [US7] Update CRRCSim minisim to match new response format in crrcsim/
- [ ] T055 [US7] Verify aggregate fitness from streamed data matches legacy scalar in tests/timestep_streaming_tests.cc
- [ ] T056 [US7] Run tests: `cd build && ctest -R timestep --output-on-failure`

---

## Phase 6: Segment Scoring (US5, P3)

**Depends on**: Phase 5 (per-timestep streaming)

- [ ] T060 [US5] Create TrajectorySegment struct and segment scoring function in include/autoc/eval/segment_scorer.h and src/eval/segment_scorer.cc
- [ ] T061 [US5] Implement difficulty weighting (turn rate, crosswind) in segment_scorer.cc
- [ ] T062 [US5] Wire segment scoring into fitness aggregation pipeline in src/autoc.cc
- [ ] T063 [US5] Write tests: segment scoring produces expected scores for known trajectories in tests/segment_scorer_tests.cc
- [ ] T064 [US5] Run tests: `cd build && ctest -R segment --output-on-failure`

---

## Phase 7: Checkpoint/Resume (US8, P3)

- [ ] T070 [US8] Define EvolutionCheckpoint struct with serialization in include/autoc/nn/checkpoint.h and src/nn/checkpoint.cc
- [ ] T071 [US8] Include CMAESState in checkpoint when optimizer_type=sep-cma-es (requires Phase 4)
- [ ] T072 [US8] Implement checkpoint writing at end of each generation in src/autoc.cc
- [ ] T073 [US8] Implement checkpoint loading and resume at startup in src/autoc.cc
- [ ] T074 [US8] Add checkpoint corruption detection (magic bytes + checksum) in src/nn/checkpoint.cc
- [ ] T075 [US8] Write test: checkpoint round-trip produces identical state in tests/checkpoint_tests.cc
- [ ] T076 [US8] Run tests: `cd build && ctest -R checkpoint --output-on-failure`
- [ ] T077 [US8] Integration test: run for 5 generations, kill, resume, verify gen 6 continues correctly

---

## Phase 8: Polish

- [ ] T080 Verify all 3 repo builds: autoc, CRRCSim, xiao-gp
- [ ] T081 Run full test suite: `cd build && ctest --output-on-failure`
- [ ] T082 Verify export pipeline: nnextractor → nn2cpp → xiao-gp build
- [ ] T083 Update autoc.ini with all new config keys and sensible defaults

---

## Dependencies

- Phases 1-4 are independent (can run in any order or parallel)
- Phase 5 must complete before Phase 6
- Phase 7 T071 depends on Phase 4 (CMAESState)
- Phase 8 after all others

## Cross-References

- 014 tasks (completed): T070-T131 (original numbering)
- 014 spec: acceptance scenarios and edge cases
- 014 research.md: CMA-ES hyperparameters, segment scoring formulas

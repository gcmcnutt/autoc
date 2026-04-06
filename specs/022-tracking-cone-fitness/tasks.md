# Tasks: 022 — Point-Accumulation Fitness

**Input**: Design documents from `/specs/022-tracking-cone-fitness/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: Required per constitution (Testing-First).

**Organization**: Single user story — replace fitness function. Linear dependency chain.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US1]**: All tasks belong to one story: replace fitness function

---

## Phase 1: Setup

**Purpose**: Add config parameters, no behavioral change yet

- [ ] T001 Add 6 Fit* fields to AutocConfig struct in include/autoc/util/config.h
- [ ] T002 Parse Fit* fields from autoc.ini in src/util/config.cc
- [ ] T003 Add Fit* parameters with defaults to autoc.ini (under new FITNESS section)

**Checkpoint**: Build succeeds, existing behavior unchanged, new params parsed but unused

---

## Phase 2: Foundational — New Fitness Computer

**Purpose**: Implement and test the new scoring surface + streak logic before wiring it in

**⚠️ CRITICAL**: Tests must pass before proceeding to integration

### Tests — Scoring Surface (tests/fitness_computer_tests.cc)

- [ ] T004 [P] [US1] Scoring surface unit tests in tests/fitness_computer_tests.cc:
  - **At rabbit**: computeStepScore(0, 0) == 1.0
  - **Behind on path**: computeStepScore(-3, 0) ≈ 0.917, (-5, 0) ≈ 0.80, (-10, 0) ≈ 0.50, (-20, 0) ≈ 0.20
  - **Ahead on path**: computeStepScore(+0.5, 0) ≈ 0.50, (+1, 0) ≈ 0.20, (+2, 0) ≈ 0.059
  - **Pure lateral**: computeStepScore(0, 5) ≈ 0.50, (0, 10) ≈ 0.20
  - **Combined**: computeStepScore(-5, 5) = 1/(1 + 0.25 + 1.0) ≈ 0.444
  - **Symmetry**: computeStepScore(0, 5) == computeStepScore(0, -5) (lateral sign doesn't matter — lateralDist is always positive)
  - **Asymmetry**: computeStepScore(+3, 0) << computeStepScore(-3, 0) (ahead much worse than behind)
  - **Far away signal**: computeStepScore(-50, 0) > 0 and computeStepScore(0, 50) > 0 (always positive)
  - **Different scales**: construct with behind=5, ahead=1, cross=3 and verify shape changes

### Tests — Streak Logic (tests/fitness_computer_tests.cc)

- [ ] T005 [P] [US1] Streak mechanism unit tests in tests/fitness_computer_tests.cc:
  - **No streak at start**: after resetStreak(), applyStreak(0.8) returns 0.8 * 1.16 (streakCount=1, mult=1.16)
  - **Streak builds**: 25 consecutive applyStreak(0.8) calls → last returns 0.8 * 5.0
  - **Streak caps**: 30 consecutive calls → still 5.0x (doesn't exceed max)
  - **Hard reset**: 10 good steps then applyStreak(0.1) → streakCount resets to 0, next good step starts at 1
  - **Threshold boundary**: applyStreak(0.50) maintains streak, applyStreak(0.499) resets
  - **Multiplier linearity**: at step 12 of 25, multiplier = 1.0 + 4.0 * 12/25 = 2.92
  - **Diagnostic: maxStreak**: 5 good, reset, 10 good → getMaxStreak() == 10
  - **Diagnostic: totalStreakSteps**: 5 good, reset, 10 good → getStreakSteps() == 15
  - **Diagnostic: maxMultiplier**: 10 good steps → getMaxMultiplier() == 1.0 + 4.0*10/25 = 2.6
  - **Rate independence**: construct with streakStepsToMax=50 (5s at 100ms) vs 25 (2.5s) → same multiplier at same fraction

### Implementation

- [ ] T006 [US1] Replace FitnessComputer class in include/autoc/eval/fitness_computer.h — new constructor (6 params), computeStepScore(along, lateralDist), applyStreak(stepPoints), resetStreak(), diagnostic getters. Remove computeStepPenalty, computeCrashPenalty, computeAttitudeScale, computeInterceptScale, computeInterceptBudget. Remove old #define constants.
- [ ] T007 [US1] Implement new FitnessComputer in src/eval/fitness_computer.cc — Lorentzian scoring surface with directional along-track scaling, integer streak counter with linear multiplier ramp, diagnostic tracking (maxStreak, totalStreakSteps, maxMultiplier)
- [ ] T008 [US1] Remove old fitness constants from include/autoc/autoc.h — DISTANCE_TARGET, DISTANCE_NORM, DISTANCE_POWER, ATTITUDE_NORM, ATTITUDE_POWER, CRASH_COMPLETION_WEIGHT, INTERCEPT_SCALE_FLOOR, INTERCEPT_SCALE_CEILING, INTERCEPT_BUDGET_MAX, INTERCEPT_TURN_RATE
- [ ] T009 [US1] Verify tests pass: `cd build && make -j8 && ./fitness_computer_tests`

**Checkpoint**: New FitnessComputer tested in isolation, old one removed

---

## Phase 3: User Story 1 — Wire New Fitness Into Evolution (Priority: P1) 🎯 MVP

**Goal**: Replace the inner fitness loop so training uses point-accumulation scoring

**Independent Test**: Start a training run, verify fitness is negative (negated score), verify per-scenario logging shows streak diagnostics, verify training converges

### Implementation

- [ ] T010 [US1] Simplify ScenarioScore in include/autoc/eval/fitness_decomposition.h — replace with single `double score` field (negated accumulated points). Keep `crashed` and `steps_completed` for logging. Remove completion_fraction, distance_rmse, attitude_error, smoothness, mean_throttle, all legacy_* fields.
- [ ] T011 [US1] Rewrite computeScenarioScores() in src/eval/fitness_decomposition.cc — for each scenario: extract 6 Fit* params from AutocConfig, compute streakStepsToMax from FitStreakRampSec/SIM_TIME_STEP_MSEC, construct FitnessComputer, resetStreak, loop steps computing path tangent (reuse previous at last waypoint) + along/cross decomposition + stepScore + applyStreak, store negated total. Remove intercept budget, attitude delta, distance RMSE computation.
- [ ] T012 [US1] Rewrite aggregateRawFitness() in src/eval/fitness_decomposition.cc — sum of per-scenario score (already negated, lower is better). Remove aggregateScalarFitness().
- [ ] T013 [US1] Simplify lexicase_select() in src/eval/selection.cc — single dimension per scenario: score (lower is better). Remove 3-phase cascade (completion → distance → throttle).
- [ ] T014 [US1] Update computeNNFitness() in src/autoc.cc (lines ~557-655) — replace inline fitness loop with call to computeScenarioScores() + aggregateRawFitness(). The structured path (computeScenarioScores in fitness_decomposition.cc) does the actual scoring; this function just aggregates. Remove inline intercept budget, distance/attitude accumulation, crash penalty.
- [ ] T015 [US1] Update per-scenario logging in src/autoc.cc — replace dist/att/thr/sm fields with score/maxStrk/strkPct/maxMult from FitnessComputer diagnostics
- [ ] T016 [US1] Update data.stc generation logging in src/autoc.cc — add bestScore, avgMaxStreak, pctInStreak to #NNGen line
### Tests — End-to-End Fitness (tests/fitness_decomposition_tests.cc)

- [ ] T017 [US1] Scoring with synthetic trajectories in tests/fitness_decomposition_tests.cc:
  - **Perfect tracking**: aircraft at rabbit position for all steps → score ≈ numSteps * 1.0 * (avg multiplier). With 100 steps and 25-step ramp: first 25 steps ramp 1→5x, remaining 75 at 5x → total ≈ 25*avg(1,5) + 75*5 = 25*3 + 375 = 450
  - **Behind 5m constant**: aircraft 5m behind rabbit on path for 100 steps → stepPoints=0.80 each, all in streak, total ≈ 0.80 * (ramp multiplier sum) ≈ 0.80 * 450/1.0 * 0.80 ... compute exact expected value
  - **Ahead 2m constant**: aircraft 2m ahead for 100 steps → stepPoints≈0.059, below streak threshold (0.5), all at 1x → total ≈ 5.9. Verify MUCH worse than 5m behind.
  - **Lateral 10m constant**: stepPoints=0.20, below threshold, no streak → total ≈ 20.0
  - **Crash at step 50**: 50 steps perfect tracking then stop → score ≈ half of full-flight perfect. No crash penalty added.
  - **Crash vs wandering**: 50 steps at rabbit then crash MUST score higher than 200 steps at 20m lateral (stepPoints=0.20, no streak)

- [ ] T017b [US1] Streak diagnostics with controlled trajectories in tests/fitness_decomposition_tests.cc:
  - **Three short streaks**: 5 steps close (streak), 3 steps far (reset), 5 steps close, 3 steps far, 5 steps close → maxStreak=5, totalStreakSteps=15, 3 separate streaks
  - **One long streak**: 90 steps at rabbit, 10 steps far → maxStreak=25 (capped), totalStreakSteps=90, maxMultiplier=5.0. Verify accumulated score: first 25 steps ramp 1→5x, remaining 65 at 5x, all at stepPoints=1.0
  - **Verify negation**: scenario score stored as negative (lower = better)
  - **Multi-scenario aggregate**: 3 scenarios with different scores → aggregateRawFitness() = sum of negated scores

- [ ] T017c [US1] Path tangent decomposition tests in tests/fitness_decomposition_tests.cc:
  - **Straight path**: rabbit on X axis, aircraft offset in Y → along=0, lateralDist=offset. Score matches computeStepScore(0, offset).
  - **Straight path behind**: rabbit at (10,0,0), aircraft at (5,0,0), tangent=(1,0,0) → along=-5, lateral=0. Score = computeStepScore(-5, 0).
  - **Diagonal path**: rabbit moving at 45° (tangent=(0.707,0.707,0)), aircraft offset perpendicular → verify along/cross decomposition is correct.
  - **Last waypoint**: at final path point, tangent reuses previous segment. Verify no crash/NaN.
  - **3D offset**: aircraft above rabbit by 5m (Z offset in NED) → treated as lateral, score = computeStepScore(0, 5).

- [ ] T018 [US1] Update selection_tests.cc — lexicase with single dimension:
  - Two individuals: one scores -100 (good), one scores -50 (worse) → lexicase selects the -100 individual
  - Tie-breaking: equal scores → random selection (verify both can be selected over multiple runs)
- [ ] T019 [US1] Build and run all tests: `bash scripts/rebuild.sh`

**Checkpoint**: Training uses new fitness. All tests pass. Ready for training run.

---

## Phase 4: Polish & Validation

**Purpose**: Verify training convergence and clean up

- [ ] T020 Start training run with new fitness: `nohup stdbuf -oL -eL build/autoc >logs/autoc-022-fit1.log 2>&1 &`
- [ ] T021 Monitor first 20 generations — verify fitness is negative, decreasing (improving), streak diagnostics appearing in log
- [ ] T022 Update data.dat per-step logging in src/autoc.cc logEvalResults():
  - Replace `attDq` column with `along` (signed along-track distance, - = behind)
  - Replace `intSc` column with `stpPt` (raw step score 0-1)
  - Add `mult` column (current streak multiplier 1.0-5.0)
  - Requires: compute path tangent + along/cross decomposition in logging loop (same math as fitness_decomposition.cc), create FitnessComputer instance, track streak state per scenario
  - Update header string and sprintf format (7 trailing diag columns instead of 6)
  - Remove attitude_delta computation from logging loop (no longer needed)
- [ ] T023 Remove dead code — any remaining references to old fitness constants, computeStepPenalty callers, DISTANCE_TARGET usage
- [ ] T024 Add streak threshold ramp:
  - Add FitStreakThresholdMin (0.1) and FitStreakThresholdMax (0.5) to config.h, config.cc, autoc.ini
  - In computeScenarioScores(), interpolate threshold: min + (max - min) * computeVariationScale()
  - FitnessComputer constructor takes the interpolated threshold
  - Update tests to verify threshold interpolation
- [ ] T025 Commit all changes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — config only
- **Phase 2 (Foundational)**: Depends on Phase 1 (needs config fields). Tests before implementation.
- **Phase 3 (Integration)**: Depends on Phase 2 (needs working FitnessComputer)
- **Phase 4 (Polish)**: Depends on Phase 3 (needs complete build)

### Within Phase 2

- T004 and T005 (tests) can run in parallel
- T006 and T007 must be sequential (header then implementation)
- T008 (remove old constants) after T006/T007

### Within Phase 3

- T010 → T011 → T012 (decomposition: header → impl → aggregate)
- T013 (selection) can run in parallel with T010-T012
- T014 (autoc.cc) depends on T011 and T013
- T015-T016 (logging) depend on T014
- T017-T018 (tests) depend on T010-T013
- T019 (build) depends on all above

---

## Implementation Strategy

### MVP (Phase 1-3)

1. Add config params (Phase 1)
2. Build + test new FitnessComputer in isolation (Phase 2)
3. Wire into evolution pipeline (Phase 3)
4. **VALIDATE**: Start training run, monitor convergence

### Key Risk

Local minimum where streak on partial course dominates. Monitor `pctInStreak` in diagnostics — should increase over generations, not plateau early.

---

## Notes

- Constitution requires tests — T004/T005 before T006/T007
- No CRRCSim, xiao, or NN topology changes
- Existing variation ramp (VariationRampStep) unchanged
- Path tangent at last waypoint: reuse previous segment tangent
- Fitness direction: negate score so lower = better (existing convention)

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

## Phase 3b: Coordinate Convention Cleanup

**Purpose**: Fix raw/virtual position inconsistency. See [coordinate-cleanup-research.md](coordinate-cleanup-research.md).

**Principle**: Convert raw→virtual at the producer boundary (CRRCSim/xiao), once.
`getPosition()` is always virtual. `getVirtualPosition()` removed.

**Ordering**: Internal frameworks + minisim first (prove the fix), then CRRCSim
(apply to real sim once minisim shows signal), then renderer (display correctly).
Xiao already stores virtual — no code changes needed.

**⚠️ TEST-FIRST**: Existing tests encode the bug (all positions at Z=-25 raw). Rewrite
tests to express the correct virtual-space contract BEFORE fixing the code. Tests should
FAIL against current code, then PASS after the fix.

---

### Phase 3b-i: Tests + Internal Framework + Minisim

**Goal**: Prove the fix works end-to-end with minisim training before touching CRRCSim.

#### Tests — Coordinate Contract (FIRST — before code changes)

- [ ] T029 [US1] Rewrite fitness_decomposition_tests.cc to express virtual coordinate contract:
  - All path positions at virtual origin: Z=0 (not Z=-25)
  - All aircraft positions in virtual space: Z≈0 (near path, not raw altitude)
  - No `setOriginOffset()` calls anywhere in tests
  - **New test: VirtualOriginPerfectTracking**: aircraft at (x,0,0), path at (x,0,0) → step score 1.0
  - **New test: RawPositionGivesWrongScore**: aircraft at (x,0,-25) with path at (x,0,0) → large Z offset
    penalizes score heavily. This test documents the bug — it PASSES now (proving the bug exists)
    and should STILL pass after the fix (the scenario genuinely has a 25m error).
  - **Update makeStraightPath helper**: paths at Z=0, aircraft at Z=0 + offsets. Remove Z=-25 everywhere.
  - Existing test logic (behind/ahead/lateral/crash/streaks) preserved, just in virtual space.
  - **These tests will FAIL against current code** (which feeds raw Z=-25 to fitness). This is expected
    and proves the bug. They pass after T030-T034 fix the coordinate pipeline.

#### Internal Framework Changes

- [ ] T030 [US1] Clean-cut remove ALL originOffset machinery from AircraftState (aircraft_state.h):
  - Remove `originOffset_` member (L395)
  - Remove `setOriginOffset()`, `getOriginOffset()`, `getVirtualPosition()` (L230-234)
  - No deprecated alias — Constitution III: no compatibility shims
  - `getPosition()` is now the sole position accessor — always virtual
  - Compile will break at ALL callers (CRRCSim included) → forces updating everything in one cut

- [ ] T031 [US1] Add `gp_vec3 originOffset` to ScenarioMetadata (protocol.h):
  - Add field with default `gp_vec3::Zero()`
  - Add to cereal `serialize()` method
  - This preserves the raw→virtual offset for renderer and logging

- [ ] T033 [US1] Update ALL getVirtualPosition() callers → getPosition() (one cut):
  - `fitness_decomposition.cc:43`
  - `sensor_math.cc:144, 157`
  - `evaluator.cc:213`
  - `inputdev_autoc.cpp:795, 972` (CRRCSim — updated now, not deferred)
  - `minisim.cc:174`

- [ ] T034 [US1] Fix autoc.cc logEvalResults() to use virtual positions:
  - Remove manual `originOffset = aircraftStates.at(0).getPosition()` (L573)
  - Change `stepState.getPosition() - originOffset` (L591) → `stepState.getPosition()`
  - X,Y,Z columns in data.dat (L678-680) now log virtual position automatically
  - Optionally log origin offset per-scenario as a comment line

#### Minisim Changes

- [ ] T033b [US1] Minisim: store virtual position, reconstruct raw for OOB (minisim.cc):
  - Start at virtual origin: `initialPosition = (0, 0, 0)` — remove SIM_INITIAL_ALTITUDE from initial position
  - Remove `setOriginOffset(initialPosition)` (L139)
  - Initial velocity: rotate body-forward through initial orientation (already correct, just verify
    that with virtual Z=0 start the velocity vector is horizontal)
  - After `minisimAdvanceState()` (which advances position in virtual space),
    reconstruct raw for OOB: `rawForOOB = getPosition() + vec3(0, 0, SIM_INITIAL_ALTITUDE)`
  - OOB check uses `rawForOOB` against existing SIM_MAX_ELEVATION/SIM_MIN_ELEVATION bounds
  - **Critical**: Without this, virtual Z=0 > SIM_MIN_ELEVATION(-7) → immediate OOB crash on first tick
  - Remove stale comment "Raw altitude matches CRRCSim config" (L131)
  - Store `SIM_INITIAL_ALTITUDE` as originOffset in ScenarioMetadata for downstream renderer use
  - `getVirtualPosition()` (L174) → `getPosition()`

#### Documentation (can run in parallel)

- [ ] T037 [P] [US1] Add "Virtual Frame" section to COORDINATE_CONVENTIONS.md:
  - Define virtual frame: origin at (0,0,0), path-relative coordinates
  - Document the boundary: CRRCSim (inputdev_autoc.cpp), minisim (minisim.cc), xiao (msplink.cpp)
  - State the invariant: `getPosition()` is always virtual downstream
  - Document origin offset metadata: per-scenario in ScenarioMetadata
  - Document renderer display convention: virtual + SIM_INITIAL_ALTITUDE

#### Validation — Minisim Signal

- [ ] T038a [US1] Verify minisim coordinate cleanup:
  - Build succeeds with minisim: `scripts/rebuild.sh`
  - Unit tests pass: `build/fitness_computer_tests`, `build/fitness_decomposition_tests`
  - Start minisim training run, verify data.dat X,Y,Z columns near (0,0,~0) not (~0,~0,~-25)
  - Verify minisim doesn't immediately OOB on first tick
  - **Acceptance criterion**: Aircraft Z distribution in minisim training centers on Z≈0 (path
    altitude), NOT at Z=-7 (hard deck). If minisim shows correct Z behavior, the core fix is validated.

**Checkpoint**: Internal framework correct. Minisim shows proper Z tracking. Tests pass.
All getVirtualPosition() callers updated (clean cut). CRRCSim compiles but still stores raw
in position — fixed in Phase 3b-ii (T032).

---

### Phase 3b-ii: CRRCSim

**Goal**: Apply virtual-at-boundary to CRRCSim. Run real training (test3).
CRRCSim callers already updated in T033 (clean cut). This phase changes the producer.

- [ ] T032 [US1] CRRCSim: store virtual position at boundary (inputdev_autoc.cpp):
  - Add module-level `gp_vec3 pathOriginOffset` (like xiao's `test_origin_offset`)
  - At path start (L511-540): `pathOriginOffset = initialPos`
  - Initial state: `initialState.setPosition(gp_vec3::Zero())` (virtual origin)
  - Each tick (L685): `aircraftState.setPosition(p - pathOriginOffset)` (virtual)
  - OOB check (L702-708): use local raw `p` directly, NOT `aircraftState.getPosition()`
  - Store `pathOriginOffset` in scenario metadata sent back to autoc
  - Remove all `setOriginOffset()` calls

- [ ] T038b [US1] Verify CRRCSim coordinate cleanup:
  - CRRCSim build succeeds: `cd crrcsim/build && make -j8`
  - Full rebuild: `scripts/rebuild.sh`
  - Start FRESH test3 training run with CRRCSim (new random population):
    `nohup stdbuf -oL -eL build/autoc >logs/autoc-022-test3.log 2>&1 &`
  - **Acceptance criterion A**: Aircraft Z distribution centers on path altitude (~Z=0 virtual),
    NOT hard deck (Z=-7 raw). The hard-deck-hugging behavior must disappear.
  - **Acceptance criterion B**: Fitness scores increase significantly vs test2 — confirms the
    scoring surface now sees true distances, not ~25m Z error.
  - **Throughput check**: Time 1 generation, verify no regression from ~16K sims/sec baseline.

**Checkpoint**: CRRCSim training produces correct coordinate behavior. test3 validates the fix.

---

### Phase 3b-iii: Renderer

**Goal**: Display virtual positions correctly. Xiao already stores virtual — verify unchanged.

- [ ] T035 [US1] Renderer: handle virtual aircraft positions from training eval results (renderer.cc):
  - `updateGenerationDisplay()` (L320-334): add `SIM_INITIAL_ALTITUDE` to aircraft
    positions (same as already done for paths) — both are now virtual at Z≈0
  - Update comment at L320-322 to reflect new convention
  - `renderFullScene()` all-flight mode: use originOffset from ScenarioMetadata
    to reconstruct raw positions for "all paths in raw" display
  - Verify single-arena training playback: paths and aircraft tape should overlap in Z

- [ ] T036 [US1] Renderer: verify xiao modes unchanged:
  - Xiao already stores virtual and adds SIM_INITIAL_ALTITUDE at render time
  - Xiao has zero references to originOffset/getVirtualPosition — no code changes needed
  - Verify single-span and all-flight modes still display correctly
  - Load existing xiao log, confirm rendering matches pre-cleanup behavior

---

## Phase 4: Polish & Validation

**Purpose**: Verify training convergence and clean up

- [ ] T020 Continue monitoring test3 run started in T038b — verify convergence beyond initial 20 generations
- [ ] T021 Monitor training metrics: fitness improving over generations, streak diagnostics (pctInStreak increasing), aircraft Z distribution stable at path altitude (~Z=0 virtual)
- [ ] T022 Update data.dat per-step logging in src/autoc.cc logEvalResults() (builds on T034 changes):
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
- **Phase 3b-i (Core + Minisim)**: Depends on Phase 3 (fitness_decomposition.cc exists). Tests first, then clean-cut all callers + minisim.
- **Phase 3b-ii (CRRCSim)**: Depends on Phase 3b-i (minisim shows signal). CRRCSim producer changes only (callers already updated).
- **Phase 3b-iii (Renderer)**: Depends on Phase 3b-ii (CRRCSim produces virtual positions). Xiao: verify only.
- **Phase 4 (Polish)**: Depends on Phase 3b-ii (test3 running with correct coordinates)

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

## Notes

- Constitution requires tests — T004/T005 before T006/T007
- Constitution III: No shims. All getVirtualPosition() callers updated in one cut (T030). No deprecated aliases.
- CRRCSim changes limited to coordinate boundary cleanup (Phase 3b) — no NN topology changes
- Xiao: no code changes needed (already stores virtual)
- Existing variation ramp (VariationRampStep) unchanged
- Path tangent at last waypoint: reuse previous segment tangent
- Fitness direction: negate score so lower = better (existing convention)
- Phase 3b is the critical correctness fix — fitness scoring has been using raw positions (~25m Z error)
- See plan.md for full MVP strategy and risk assessment

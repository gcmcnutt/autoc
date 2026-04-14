---

description: "Task list for feature 023: OOD Coverage, Engage Transient, Throttle Discipline"
---

# Tasks: 023 — OOD Coverage, Engage Transient, Throttle Discipline

**Input**: Design documents from `/specs/023-ood-and-engage-fixes/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/nn_interface.md, contracts/engage_delay.md, contracts/history_reset.md, contracts/evaluator.md, quickstart.md, train-eval-code-dedup.md

**Tests**: REQUIRED. Constitution I (Testing-First) mandates tests for all significant changes. Contracts define explicit test coverage matrices.

**Organization**: Tasks are grouped by user story. Setup and Foundational phases are shared infrastructure that unblocks all stories.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1–US6)
- Exact file paths included for every task

## User Stories

- **US1 (P1) — Wrap-Free Bearing Representation**: Replace dPhi/dTheta with 3-cosine direction vectors. Structurally eliminates atan2 wrapping. Includes NN topology bump to {33, 32, 16, 3}. **This is the primary technical lever of 023.**
- **US2 (P1) — Clean Engage Handoff**: History buffer reset on engage + CRRCSim engage delay window + INAV config-only delay reductions. The aircraft engages cleanly and the sim trains against realistic handoff conditions.
- **US3 (P2) — Throttle Discipline**: Throttle lexicase dimension + expanded training distribution (far-start intercept, wider sigmas).
- **US4 (P2) — Discontinuity-Forcing Training Paths**: Purpose-built paths that exercise target-behind / target-below / inverted regimes.
- **US5 (P3) — Authority-Limit Iteration**: Conditional iteration on NN output cap if Milestone A baseline shows saturation.
- **US6 (P3) — Craft Parameter Variations**: Robustness training with ± stddev variations on aircraft dynamics parameters.

## Validation Ladder (per Implementation Order in plan.md)

All tasks feed into a 3-rung validation ladder:
1. **Minisim smoke test** — plumbing gate, not tracking quality
2. **CRRCSim training milestones A (baseline) → B (layered) → C (robustness)** — observational layering, judgment-based
3. **Xiao field-test prep** — flash, bench, fly

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project state check; no new build infrastructure needed (023 reuses the existing unified CMake + PlatformIO setup from 022).

- [ ] T001 Verify clean build baseline at 023 branch HEAD: `bash scripts/rebuild.sh` succeeds, all existing tests pass. No code changes — just confirm the starting state.
- [ ] T002 Verify xiao build baseline: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` succeeds with current code (pre-refactor).
- [ ] T003 Create backup of current `xiao/inav-hb1.cfg` at `xiao/inav-hb1.cfg.pre-023.bak` for quick rollback during Change 1c testing.

**Checkpoint**: Clean baseline confirmed. Proceed to Foundational phase.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Phase 0a (type-safe NN sensor interface) + Phase 0b (train/eval dedup refactor). These MUST complete before any user story work begins. Both are large refactors with their own test suites.

**⚠️ CRITICAL**: No user story work can begin until Phase 2 completes.

### Phase 0a — Type-Safe NN Sensor Interface (Struct-of-Floats)

Landmarks: `include/autoc/nn/nn_inputs.h` (NEW), `tests/nn_inputs_tests.cc` (NEW), 27→33 input migration. See `contracts/nn_interface.md` for full 6-rule contract.

#### Phase 0a.1 — Refactor at 27 inputs (no behavior change)

- [ ] T004 Create `include/autoc/nn/nn_inputs.h` with `NNInputs` struct containing the CURRENT 27-field layout (dPhi[6], dTheta[6], dist[6], closing_rate, quat_w/x/y/z, airspeed, gyro_p/q/r). Include `static_assert(sizeof(NNInputs) == 27 * sizeof(float))` and `static_assert(alignof(NNInputs) == alignof(float))`. Add constitutional note header comment explaining field order is the serialization contract.
- [ ] T005 [P] Create `tests/nn_inputs_tests.cc` with initial tests: sizeof contract, alignment contract, topology match (`NN_TOPOLOGY[0] == NN_INPUT_COUNT`), unit-vector invariant placeholder (not needed yet at 27 inputs — that's Phase 0a.2).
- [ ] T006 Update `include/autoc/nn/topology.h` to derive `NN_INPUT_COUNT` from `sizeof(NNInputs) / sizeof(float)` instead of hand-maintaining the constant. Keep `{27, 16, 8, 3}` topology unchanged for this step.
- [ ] T007 Update `include/autoc/autoc.h` to remove duplicate `NN_INPUT_COUNT` defines (per BACKLOG type-safe-NN file list).
- [ ] T008 Update `include/autoc/eval/aircraft_state.h`: replace `std::array<float, NN_INPUT_COUNT> nnInputs_` with `NNInputs nnInputs_`. Update cereal `serialize()` member to round-trip the struct. Add `resetHistory(const Path& armedPath, const AircraftState& currentState)` method declaration (implementation is a later task — US2).
- [ ] T009 Update `src/eval/aircraft_state.cc` (or inline in header): implement `nnInputs_` serialization via cereal with field-by-field roundtrip or positional member iteration.
- [ ] T010 Update `src/nn/evaluator.cc` `nn_gather_inputs()`: populate `NNInputs` fields by name (not integer index). Use designated initializer or field-by-field assignment. Add matrix-multiply entry point that does `reinterpret_cast<const float*>(&inputs)` once at the start of `forward()`.
- [ ] T011 Update `src/autoc.cc` `data.dat` format: replace hand-maintained column list with `emitNNInputsColumnHeaders()` helper that derives names from struct fields (e.g., `dPhi_0, dPhi_1, ..., gyro_r`). Update the per-step row writer to use named field access. Keep `pitch`, `roll`, `throttle` output columns plus meta columns unchanged.
- [ ] T012 Update `tests/contract_evaluator_tests.cc`: topology assertions use field-name access, not integer indexing.
- [ ] T013 Update `tests/nn_evaluator_tests.cc`: input layout assertions use field-name access.
- [ ] T014 Update `specs/019-improved-crrcsim/sim_response.py`: parse `data.dat` by column NAME (use header row), not positional index. Fail loud on missing expected column names.
- [ ] T015 Update `tools/nn2cpp/` codegen templates: emit `inputs.dPhi[0]` style field access in generated xiao C++ instead of `inputs[0]`. Verify generated file still compiles.
- [ ] T016 Update `xiao/src/msplink.cpp`: populate `NNInputs` fields by name in the xiao-side input gathering code (currently around L402+ per msplink structure).
- [ ] T017 Update `tools/minisim.cc`: populate `NNInputs` fields by name (minisim is the simpler sensor gatherer).
- [ ] T018 Update `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`: populate `NNInputs` fields by name. This is the CRRCSim-side sensor gatherer.
- [ ] T019 [P] Add poison-value completeness test in `tests/nn_inputs_tests.cc`: construct `NNInputs` via `memset(&inputs, 0xAB, sizeof(inputs))`, call each producer's population routine, assert every field is NOT `0xABABABAB`. Covers P1 from contracts/nn_interface.md.
- [ ] T020 [P] Add field-name round-trip test in `tests/nn_inputs_tests.cc`: write a known `NNInputs` to a temp `data.dat`, parse via `sim_response.py` (invoked as subprocess or via a Python embedding), compare read-back values to written values within FP rounding.
- [ ] T021 Run `bash scripts/rebuild.sh`. All tests must pass. No behavior change expected.
- [ ] T022 **Pre/post diff verification**: run a fixed minisim scenario pre-refactor (from backup) and post-refactor at the same seed + weights. Compare `data.dat` line by line — column count matches (27+3+meta), per-tick values match within FP rounding. If any mismatch, Phase 0a.1 has a bug. Document the comparison command and outcome in commit message.
- [ ] T023 Verify xiao build still succeeds: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. xiao binary hash will change (struct field access vs integer indexing) but functional behavior must be identical.

**Checkpoint 0a.1**: `NNInputs` struct at 27 fields, all tests pass, `data.dat` output unchanged, xiao builds. Zero behavior change from main.

#### Phase 0a.2 — Grow to 33 inputs (3-cosine bearing representation)

- [ ] T024 Create `include/autoc/nn/nn_input_computation.h` with `computeTargetDir(vec3 target_body, float dist, vec3 rabbit_vel_dir_body)` helper declaration and `DIR_NUMERICAL_FLOOR = 1e-4f` constant. See `contracts/history_reset.md` §computeTargetDir() helper contract.
- [ ] T025 Implement `computeTargetDir()` in `src/nn/nn_input_computation.cc`: hard numerical floor check, fallback to rabbit_vel_dir_body at singularity, unit-vector output. No trig calls. Eigen vec3 or bare float triple per existing conventions.
- [ ] T026 [P] Add `computeTargetDir()` unit tests in `tests/nn_inputs_tests.cc`: normal case (target (3,4,0), dist 5 → (0.6,0.8,0)), singularity fallback (dist 1e-5 → rabbit_vel_dir_body returned verbatim), unit-vector invariant on random samples, NO trig calls in compiled assembly (optional check if binutils available).
- [ ] T027 Edit `include/autoc/nn/nn_inputs.h`: replace `dPhi[6]` + `dTheta[6]` fields with `target_x[6]`, `target_y[6]`, `target_z[6]` (see data-model.md §1 for exact layout). Update `static_assert` from 27 to 33.
- [ ] T028 Edit `include/autoc/nn/topology.h`: update hidden layer sizes `NN_HIDDEN1_SIZE = 32`, `NN_HIDDEN2_SIZE = 16`. Update `NN_TOPOLOGY_STRING = "33,32,16,3"`. Update `static_assert(NN_WEIGHT_COUNT == 1667)`.
- [ ] T029 Compile. Expect compile errors at every site that still references `inputs.dPhi[...]` or `inputs.dTheta[...]`. Fix each one by calling `computeTargetDir()` and populating the new `target_x/y/z` fields. Expected touch points: `src/nn/evaluator.cc` (nn_gather_inputs), `src/eval/sensor_math.cc`, `src/eval/fitness_decomposition.cc`, `tools/minisim.cc`, `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`, `xiao/src/msplink.cpp`. Continue until the whole tree compiles.
- [ ] T030 Update NN weight file deserialization: existing 611-weight NN01 files must produce a clear error message on load because the new topology is 1667 weights. Add version/size check in the NN loader that rejects old files loudly. **No compat shim** (Constitution III).
- [ ] T031 Update `tools/nn2cpp/` codegen: regenerate xiao-side inference code for the new {33, 32, 16, 3} topology. Update the xiao build to pick up the new generated files.
- [ ] T032 Add unit-vector invariant tests in `tests/nn_inputs_tests.cc`: after each producer (minisim, crrcsim, xiao stub) populates `NNInputs`, assert `target_x[i]² + target_y[i]² + target_z[i]² ∈ (0.99, 1.01)` for all 6 time samples.
- [ ] T033 [P] Update `src/eval/fitness_decomposition.cc` `computeScenarioScores()`: the scoring function reads target position from `AircraftState` and the path — verify it still works correctly with the new `NNInputs` layout. Path tangent in body frame (used by `computeTargetDir()` singularity fallback) must be derived from the path, not from `nnInputs_` history.
- [ ] T034 Run `bash scripts/rebuild.sh`. All tests pass. NN topology is now {33, 32, 16, 3}.
- [ ] T035 Run `tests/fitness_decomposition_tests.cc` explicitly to verify scoring surface tests still pass at the new topology.

**Checkpoint 0a.2**: `NNInputs` struct at 33 fields with unit-vector bearing representation. NN topology 1667 weights. Xiao builds. All tests pass.

#### Phase 0a.3 — Deliberate-break verification (scratch-branch, do not commit)

- [ ] T036 On a SCRATCH branch forked from Phase 0a.2 HEAD: add a throwaway field `float DELIBERATE_BREAK;` to `NNInputs`. Compile. Verify `static_assert(sizeof(NNInputs) == 33 * sizeof(float))` FIRES with a clear error message. Capture the compile error output. Paste into the Phase 0a.2 commit message (amended) or a dedicated commit note as "deliberate-break verification confirmed at nn_inputs.h:<line>". Revert the scratch branch — do NOT commit the throwaway field.

### Phase 0b — Train/Eval Scenario Construction De-Duplication

Landmarks: `include/autoc/eval/build_eval_data.h` (NEW), `tests/build_eval_data_contract_tests.cc` (NEW), `tests/eval_determinism_tests.cc` (NEW). See `contracts/evaluator.md` for full contract.

- [ ] T037 Create `include/autoc/eval/build_eval_data.h` with `EvalPurpose` enum (`Training`, `EliteReeval`, `StandaloneEval`), `EvalJob` struct, and `buildEvalData(const EvalJob&)` declaration. See `contracts/evaluator.md` §Interface.
- [ ] T038 Create `tests/build_eval_data_contract_tests.cc` with stub contract tests (Test A "every purpose populates every field", Test B "purpose-specific field values", Test C "cross-caller consistency", Test D "callin surface", Test E "eval iterates all scenarios"). Tests reference `buildEvalData()` which doesn't exist yet — they fail to link. This is the test-first step.
- [ ] T039 [P] Create `tests/eval_determinism_tests.cc` with stub end-to-end test: loads a known weight file (placeholder — will be `betterz2` gen 400 once wired), runs through training path and eval path, asserts fitness reproduces within FP rounding. Initially fails because eval path still has Bug 3.
- [ ] T040 Implement `buildEvalData()` stub in `src/autoc.cc` (or new `src/eval/build_eval_data.cc`) by COPYING the per-individual scenario-list construction code from `src/autoc.cc:932-975` verbatim. Route the per-individual loop through it. Run the full test suite — expect only the helper contract tests to start passing for `EvalPurpose::Training`.
- [ ] T041 Route training elite re-eval through the helper: delete `src/autoc.cc:1021-1060` (Copy B of the scenario populator), replace with `buildEvalData({scenario, nnBytes, EvalPurpose::EliteReeval})`. Test E passes for EliteReeval. Zero behavior change (Copy B was functionally identical to Copy A).
- [ ] T042 Route `runNNEvaluation()` through the helper: delete `src/autoc.cc:751-787` (Copy C), replace with `buildEvalData({scenario, nnBytes, EvalPurpose::StandaloneEval})`. **This is the commit that fixes Bug 3** (rabbitSpeedConfig now populated correctly). Eval fitness values will differ from pre-refactor because the bug masked the configured rabbit speed variation.
- [ ] T043 **Fix Bug 2** in eval path: in `runNNEvaluation()` around `src/autoc.cc:820`, add `genome.fitness = fitness; nn_serialize(genome, nnData); evalResults.gp.assign(nnData.begin(), nnData.end());` BEFORE the S3 upload. The eval-computed fitness now lands in the serialized NN01 bytes.
- [ ] T044 **Fix Bug 5** in eval caller loop: iterate over all scenarios in `runNNEvaluation()` instead of hard-coding `scenarioForIndex(0)`. Match training's aggregation pattern: `for (size_t s = 0; s < generationScenarios.size(); ++s) { ... }` and aggregate across all scenarios before reporting fitness. See `train-eval-code-dedup.md` §Migration order step 5.
- [ ] T045 **Fix Bug 4** in eval path: passing `EvalPurpose::StandaloneEval` already sets `isEliteReeval = true` via the helper (per contract). Verify the helper writes `meta.enableDeterministicLogging = true` for EliteReeval and StandaloneEval. Contract test B catches regressions here.
- [ ] T046 Add determinism check in eval mode: mirror training's `bitwiseEqual` check at `src/autoc.cc:1077-1087`. After computing eval fitness, log "SAME" or "DIVERGED" relative to the stored NN01 fitness. Helps catch future regressions visually in eval logs.
- [ ] T047 Wire up `eval_determinism_tests.cc`: create a committed test fixture under `tests/fixtures/` — a small synthetic NN weight file generated deterministically (e.g., all weights = 0.01, or Xavier-init with a fixed seed). This fixture is the PRIMARY gate for SC#7 (eval-mode determinism). The test loads the fixture, runs one scenario through the training path and the eval path, and asserts **exact** `ScenarioScore` match (`memcmp` — must be bitwise identical because both paths now call the same `buildEvalData()` code). This test is MANDATORY — not conditional on betterz2 availability.
- [ ] T048 Run the full test suite. All build_eval_data_contract_tests pass. eval_determinism_tests passes. `bash scripts/rebuild.sh` is green.
- [ ] T049 **Secondary acceptance check from train-eval-code-dedup.md**: if a `betterz2` gen-400 weight file is available from 022 S3 artifacts, run it through eval mode and confirm the reported fitness reproduces the training-time `-34771` within FP rounding. Document the result in the commit message. NOTE: the PRIMARY SC#7 gate is T047 (committed synthetic fixture, exact `memcmp` match). This T049 check is a real-world validation that the betterz2 weights — which were produced under the pre-dedup eval path with Bug 3 active (constant 16 m/s rabbit) — now produce DIFFERENT fitness under the fixed eval path (correct 13±2 m/s rabbit). The "different" result proves the bug fix took effect. If betterz2 weights aren't accessible, note "betterz2 real-world check deferred" — this does NOT block SC#7 acceptance because T047 already covers it.

### Phase 0b supplement — Config knob additions (new 023 settings)

- [ ] T050 [P] Update `include/autoc/util/config.h`: add `engageDelayMs` (int, default 750, for startup logging + CRRCSim launcher env var propagation — NOT transmitted via RPC) and `nnAuthorityLimit` (double, default 1.0) fields to the config struct. Validate `NNAuthorityLimit ∈ (0.0, 1.0]` with fatal error. `EngageDelayMs` validated CRRCSim-side.
- [ ] T051 [P] Update `src/util/config.cc`: parse the two new keys from `autoc.ini`. Apply validation with fatal error on out-of-range.
- [ ] T052 Update `autoc.ini` and `autoc-eval.ini`: add `EngageDelayMs = 750` and `NNAuthorityLimit = 1.0` with comments explaining each.
- [REMOVED — I3] ~~T053~~ `engage_delay_ms` is CRRCSim-local, not transmitted via RPC. CRRCSim reads from `AUTOC_ENGAGE_DELAY_MS` env var (same pattern as existing `AUTOC_ACRO_*` knobs). autoc still parses `EngageDelayMs` from config for logging purposes but does not put it on the wire.

**Checkpoint Phase 2 complete**: Type-safe NN interface at 33 inputs, train/eval dedup landed, config knobs in place. All foundational work done. **User stories can now proceed.**

---

## Phase 3: User Story 1 — Wrap-Free Bearing Representation (Priority: P1) 🎯 MVP

**Goal**: The NN observes direction cosines instead of atan2 angles. Wrapping becomes structurally impossible. This is the primary technical intervention of 023.

**Independent Test**: After US1 is complete, training on a single deterministic path (minisim or CRRCSim) produces a NN that learns SOMETHING (fitness decreases across gens). `data.dat` bearing columns satisfy unit-vector invariant on every tick. No `atan2` calls remain in the NN input pipeline.

Most of the heavy lifting was done in Phase 0a.2 (T024–T035). US1 adds the final pieces that tie the representation change to end-to-end training.

- [ ] T054 [P] [US1] Verify `computeTargetDir()` is called from ALL input producers at exactly one site each: `src/nn/evaluator.cc`, `src/eval/sensor_math.cc`, `tools/minisim.cc`, `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`, `xiao/src/msplink.cpp`. `grep -rn "atan2" src/ tools/ crrcsim/src/mod_inputdev/ xiao/src/` should return zero hits for `atan2` in the NN input pipeline (atan2 in unrelated code like pathgen/rendering is fine).
- [ ] T055 [P] [US1] Add a regression test in `tests/nn_inputs_tests.cc`: construct a synthetic aircraft state where the rabbit crosses behind the aircraft (body-X component of target vector goes negative), verify the bearing inputs evolve smoothly without any discontinuity larger than ~0.5 in any component. Previously with dPhi/dTheta this would produce a ~π jump; now it's a smooth rotation.
- [ ] T056 [US1] Update `src/eval/fitness_decomposition.cc` to derive path tangent in body frame (needed for `computeTargetDir()` singularity fallback). The path tangent for fitness scoring is already computed; add a helper to project it into body frame for the singularity case.
- [ ] T057 [US1] Run the Phase 2 (minisim smoke test) recipe from `quickstart.md`: 50-gen training on `autoc-smoke.ini` (1 path, zero variations, minisim backend). All 12 pass criteria from the quickstart must pass. Document results in commit message.
- [ ] T058 [US1] Verify zero bearing discontinuities in the smoke test's `data.dat`: `python -c "import pandas as pd; df = pd.read_csv('data.dat', sep=' '); max_dtx = df.target_x_3.diff().abs().max(); max_dty = df.target_y_3.diff().abs().max(); max_dtz = df.target_z_3.diff().abs().max(); print(f'max per-tick delta: x={max_dtx:.3f}, y={max_dty:.3f}, z={max_dtz:.3f}'); assert max(max_dtx, max_dty, max_dtz) < 0.5, 'bearing input discontinuity detected'"`. Should print per-tick deltas all well under 0.5.
- [ ] T059 [US1] Verify unit-vector invariant holds across every `data.dat` row: `python -c "import pandas as pd; df = pd.read_csv('data.dat', sep=' '); norm_sq = df.target_x_3**2 + df.target_y_3**2 + df.target_z_3**2; assert ((norm_sq - 1.0).abs() < 0.01).all(), 'unit vector invariant violated'"`.

**Checkpoint US1**: Minisim smoke test passes. Bearing representation is wrap-free, verified structurally (no atan2) and empirically (no large per-tick deltas). **MVP milestone complete — ready for CRRCSim promotion.**

---

## Phase 4: User Story 2 — Clean Engage Handoff (Priority: P1)

**Goal**: Change 1 (history reset) + Change 1b (engage delay window) + Change 1c (INAV config-only fixes). Aircraft engages cleanly; sim trains against realistic handoff conditions.

**Independent Test**: After US2, a minisim or CRRCSim run with the engage delay window active shows: (1) NN history buffer is flat at engage (closing_rate == 0 on first tick), (2) sim stick is centered for the first ~7 ticks, (3) NN runs every tick including the delay window, (4) NN outputs take effect after tick 8+.

### Change 1 — History reset implementation

- [ ] T060 [P] [US2] Implement `AircraftState::resetHistory(const Path& armedPath, const AircraftState& currentState)` in `src/eval/aircraft_state.cc`. See `contracts/history_reset.md` for full semantics. Uses `computeTargetDir()` to derive direction cosines; pre-fills all 6 time samples (past + lookahead) with identical current-tick values; sets `closing_rate = 0`; populates quat/airspeed/gyro from currentState.
- [ ] T061 [P] [US2] Create `tests/engage_reset_tests.cc` with the 6 tests from `contracts/history_reset.md` §Verification:
  - `AllHistorySlotsIdentical` — all target_x/y/z/dist time samples equal after reset
  - `ClosingRateZero` — closing_rate == 0.0 after reset
  - `UnitVectorPopulated` — direction vector is unit-norm at every time sample
  - `ScalarFieldsFromCurrentState` — quat, airspeed, gyro match input
  - `SingularityFallbackOnZeroDistance` — dist < 1e-4 returns path tangent direction
  - `FirstEngageAndReEngageAreIdentical` — two sequential resets with identical arguments produce identical outputs (no special-case first engage)
- [ ] T062 [US2] Call `resetHistory()` at engage transition in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (at scenario start).
- [ ] T063 [US2] Call `resetHistory()` at scenario start in `tools/minisim.cc`.
- [ ] T064 [US2] Call `resetHistory()` at autoc enable transition in `xiao/src/msplink.cpp` (currently around L402+). This replaces whatever ad-hoc history handling may be present.
- [ ] T065 [US2] Run `tests/engage_reset_tests.cc`. All 6 tests pass.

### Change 1b — CRRCSim engage delay window

- [ ] T066 [P] [US2] Create `tests/engage_delay_tests.cc` with the 3 unit tests from `contracts/engage_delay.md` §Verification:
  - `WindowSuppressesOutputs` — mock NN with non-zero outputs, verify sim receives {0,0,0} for ticks 1–N, receives NN outputs for tick N+1+, NN called every tick
  - `ZeroDelayDisablesWindow` — `EngageDelayMs = 0` → NN outputs take effect immediately
  - `WindowResetsOnScenarioBoundary` — back-to-back scenarios both re-enter the window
- [ ] T067 [US2] Implement `EngageDelayState` (file-local struct) and delay window logic in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`. Read `AUTOC_ENGAGE_DELAY_MS` env var (default 750 if not set, matching existing `AUTOC_ACRO_*` pattern in `inputdev_autoc.h`). At scenario start: `window_active = true`, `ticks_remaining = ceil(engage_delay_ms / SimTimeStepMs)`. Each tick: if `window_active`, apply `{0, 0, 0}` stick, decrement counter, transition to `window_active = false` when counter reaches zero. After window: apply `NN_outputs * NNAuthorityLimit`. NN inputs updated every tick regardless. Verify env var is read (not hardcoded) by running with `AUTOC_ENGAGE_DELAY_MS=500` and observing 5 ticks of suppressed output instead of 7.
- [REMOVED — I3] ~~T068~~ Minisim does NOT model the INAV engage delay — minisim is a kinematic plumbing test, not a flight-fidelity sim. The delay is a property of the INAV handoff that only CRRCSim models.
- [REMOVED — I3/D1] ~~T069~~ Absorbed: `engage_delay_ms` not on RPC. CRRCSim reads from `AUTOC_ENGAGE_DELAY_MS` env var. Verification that CRRCSim uses the env var (not hardcoded) is folded into T067.
- [ ] T070 [US2] Run `tests/engage_delay_tests.cc`. All 3 tests pass.

### Change 1c — INAV config-only delay reductions (bench work, no code)

**⚠️ These tasks modify `xiao/inav-hb1.cfg` and `xiao/inav-bench.cfg` and require bench verification with xiao blackbox logs. Apply one at a time. Document measurements in a Phase 1 summary note appended to `docs/inav-signal-path-audit.md`.**

- [ ] T071 [US2] Apply INAV config change #1: `set gyro_main_lpf_hz = 60` in `xiao/inav-hb1.cfg`. Reflash FC. Bench test: capture xiao blackbox log with autoc enabled, measure per-tick gyro input latency. Document before/after delta (~-4 ms expected).
- [ ] T072 [US2] Apply INAV config change #2: `set setpoint_kalman_enabled = OFF` in `xiao/inav-hb1.cfg`. Reflash. Bench test. Document before/after delta.
- [ ] T073 [US2] Apply INAV config change #3: `set rc_filter_lpf_hz = 0` in `xiao/inav-hb1.cfg`. Reflash. Bench test (verify servos still respond to xiao commands normally — this removes an unmodeled filter). Document delta (~-2 ms expected on servo command path).
- [ ] T074 [US2] Apply INAV config change #4 (conditional): `set dynamic_gyro_notch_enabled = OFF` in `xiao/inav-hb1.cfg`. Reflash. Bench test. Verify gyro signal quality is still acceptable for fixed-wing NN input (no new oscillation).
- [ ] T075 [US2] INAV config change #5 **BENCH TEST ONLY**: edit `xiao/inav-bench.cfg` (NOT `inav-hb1.cfg`) to `set failsafe_recovery_delay = 0`. Reboot FC. Measure MSPRCOVERRIDE engage delay from xiao blackbox. Expected: ~760 ms → ~260 ms. If hypothesis confirmed, document in `docs/inav-signal-path-audit.md` appendix. **Revert `inav-bench.cfg` after test. DO NOT apply to `inav-hb1.cfg`.**
- [ ] T076 [US2] Document cumulative latency improvement in a new "Change 1c Measurements" section of `docs/inav-signal-path-audit.md`: per-change before/after MSP round-trip latency, NN input path delay reduction, expected effect on NN training realism.

**Checkpoint US2**: Engage handoff is clean (history reset + delay window + reduced INAV-side latency). All tests pass. **Ready for CRRCSim Milestone A training** using `autoc.ini` with the new config knobs.

---

## Phase 5: User Story 3 — Throttle Discipline (Priority: P2)

**Goal**: Change 3 (throttle lexicase dimension) + Change 2 (expanded training distribution). The NN learns to modulate throttle and the training distribution matches real flight conditions.

**Independent Test**: After US3, a CRRCSim training run shows (a) throttle distribution is not saturated at 0.9+ on most individuals, (b) expanded `EntryPositionRadiusSigma` (30–50) is handled without catastrophic fitness collapse, (c) lexicase selection discriminates on throttle as well as score.

### Change 3 — Throttle lexicase dimension

- [ ] T077 [P] [US3] Update `include/autoc/eval/fitness_decomposition.h` `ScenarioScore` struct: add `double mean_throttle;` field. Document range `[-1, +1]`, lower = less energy used.
- [ ] T078 [US3] Update `src/eval/fitness_decomposition.cc` `computeScenarioScores()`: accumulate `mean_throttle` across the scenario's ticks. Compute from NN output values logged per-tick in the scenario response.
- [ ] T079 [US3] Update `src/eval/selection.cc` `lexicase_select()`: add a second lexicase dimension. Per scenario, apply `score` filter first (existing primary dimension, epsilon ~0.05), then `mean_throttle` filter (secondary, epsilon ~0.1). Evolution selects individuals that score well AND use less throttle.
- [ ] T080 [P] [US3] Update `tests/selection_tests.cc`: add tests for the two-dimension lexicase. One test with two individuals: A scores 100 mean_throttle 0.3, B scores 100 mean_throttle 0.9 → lexicase selects A. Another test: A scores 100 mean_throttle 0.9, B scores 95 mean_throttle 0.3 → score filter eliminates B first, A is selected.
- [ ] T081 [US3] Update logging in `src/autoc.cc`: per-scenario log line now includes `meanThr` column. `data.stc` per-generation line includes `bestMeanThrottle`.

### Change 2 — Training distribution expansion

- [ ] T082 [P] [US3] Update `autoc.ini` training parameters to match the Phase 4 (Layer Complexity) progression defined in spec.md §Parameter Schedules for CRRCSim Milestones → Milestone B. Initial values: `EntryPositionRadiusSigma = 30` (was 15), `EntrySpeedSigma = 0.25` (was 0.1), `EntryRollSigma = 60` (was 30), `SimNumPathsPerGeneration = 6` (was 5). These are starting points — actual values layered in one at a time during training.
- [ ] T083 [US3] Run `tests/selection_tests.cc` and verify the two-dim lexicase tests pass.
- [ ] T084 [US3] Run a short CRRCSim training run (50 gens, pop 500) with `EnableEntryVariations=1`, `EntryPositionRadiusSigma=15` (moderate value, not full 30 yet). Verify fitness converges without catastrophic collapse, throttle distribution shows some spread (not saturated at 0.9+ for every individual). This is a sanity check, not a quality gate.

**Checkpoint US3**: Throttle is now a lexicase dimension; expanded distribution produces learnable signal. CRRCSim Milestone B progression can layer in the other variations.

---

## Phase 6: User Story 4 — Discontinuity-Forcing Training Paths (Priority: P2)

**Goal**: Change 7 — purpose-built paths that exercise target-behind / target-below / inverted regimes. Complements US1's 3-cosine representation by giving the NN training examples in the hard regimes.

**Independent Test**: After US4, the 5 deterministic aeroStandard paths are joined by 4 new paths (chase-from-behind, overshoot recovery, inverted sustained, vertical descent). CRRCSim training with `SimNumPathsPerGeneration=9` (or a subset selected for the milestone) completes without fitness collapse on the new paths.

- [ ] T085 [P] [US4] Design the 4 new paths in `include/autoc/pathgen.h` (and `specs/019-improved-crrcsim/embedded_pathgen_selector.h` per the pathgen-portability BACKLOG note). Each path is a waypoint sequence:
  - **ChaseFromBehind**: rabbit starts 30m ahead of aircraft's initial heading. Aircraft must actively close.
  - **OvershootRecovery**: rabbit slows or stops at a waypoint. Aircraft must decelerate or turn to avoid flying past.
  - **InvertedSustained**: path segment that requires sustained inverted tracking (extended loop or split-s).
  - **VerticalDescent**: rabbit below aircraft at entry. Aircraft must pitch down and dive.
- [ ] T086 [P] [US4] Add the 4 new paths to the aeroStandard generator method. Update `pathgen.cc` switch/case or equivalent dispatch.
- [ ] T087 [P] [US4] Update `tests/pathgen_tests.cc` (or existing path generator tests) with smoke tests for each new path: verify they generate non-zero length waypoint sequences, all waypoints are reachable (distance from previous < some max), and the rabbit speed profile is sane.
- [ ] T088 [US4] Update `autoc.ini` for the Milestone B progression: include the new paths by selecting `SimNumPathsPerGeneration = 9` (5 original + 4 new) or configurably subset them. Document the intended progression in a comment.
- [ ] T089 [US4] Run a brief CRRCSim training run (100 gens, pop 1000) with the new paths enabled. Verify:
  - Training runs without crashes or NaN
  - Per-path fitness logging shows the new paths get scored (even if poorly at first)
  - `data.dat` bearing columns on the new paths still satisfy the unit-vector invariant (regression check on US1)
  - No new category of discontinuities appears in the scoring

**Checkpoint US4**: Discontinuity-forcing paths integrated. CRRCSim can train on them. The NN gets training examples in the hard regimes.

---

## Phase 7: User Story 5 — Authority-Limit Iteration (Priority: P3, Conditional)

**Goal**: Change 8 — conditional iteration on NN output cap if Milestone A baseline observation shows saturation.

**Independent Test**: After US5 (if triggered), a CRRCSim training run with `NNAuthorityLimit = 0.5` produces a NN whose effective command range is half of the pre-limit run. Both sim bridge and xiao apply the same limit.

**⚠️ Conditional**: US5 tasks are only executed if post-Milestone-A analysis shows the NN is saturating (mean command magnitude > some empirical threshold). The trigger criterion is determined during Milestone A observation per Q4 clarify decision. If the baseline looks fine, skip US5 entirely.

### Authority limit implementation (lands regardless of trigger)

- [ ] T090 [P] [US5] Implement authority limit application in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`: `output_scaled[i] = output_raw[i] * gNNAuthorityLimit;` applied AFTER the engage delay window check. Default 1.0 = no change.
- [ ] T091 [P] [US5] Apply the same authority limit in `tools/minisim.cc`.
- [ ] T092 [P] [US5] Apply the same authority limit in `xiao/src/msplink.cpp`: compile-time constant matching the training config value. Document that mismatched values between sim and xiao would be a sim-to-real gap.
- [ ] T093 [P] [US5] Add unit test `tests/authority_limit_tests.cc`: verify `NNAuthorityLimit = 0.5` produces output_scaled = output_raw * 0.5 on all 3 axes in the sim bridge. Verify `= 1.0` is a no-op.

### Conditional iteration (may or may not run)

- [ ] T094 [US5] **PLAN-PHASE DECISION** (after Milestone A baseline observation): determine whether authority limiting is warranted. If yes, set `NNAuthorityLimit = 0.5` in `autoc.ini`, retrain from scratch or warm-start from baseline weights, compare.
- [ ] T095 [US5] Document the Milestone A observation + trigger decision in a new "Authority Limit Iteration" section of `docs/inav-signal-path-audit.md` (or a dedicated 023 dev report file). Include: baseline throttle distribution, surface deflection distribution, `d²output²` chatter stats, reason for triggering (or skipping), post-iteration results if triggered.

**Checkpoint US5**: Authority limit mechanism in place. Iteration runs only if baseline observation demands it.

---

## Phase 8: User Story 6 — Craft Parameter Variations (Priority: P3)

**Goal**: Robustness training with ± stddev variations on aircraft dynamics parameters. Pulled from 015 T140–T143.

**Independent Test**: After US6, a CRRCSim training run with craft parameter variations active produces NNs that tolerate ±10–20% variation in control effectiveness, damping, and drag without catastrophic fitness collapse.

- [ ] T096 [P] [US6] Add craft parameter variation config knobs to `include/autoc/util/config.h` and `src/util/config.cc`: `CraftClDaSigma`, `CraftCmDeSigma`, `CraftClPSigma`, `CraftCmQSigma`, `CraftCDProfSigma`, `CraftMassSigma`, `CraftInertiaSigma`, `CraftCtrlPhaseDelaySigma`. Default all to 0.0 (disabled). Ranges per spec.md §Change 5 / §Phase 5 Robustness.
- [ ] T097 [US6] Update `include/autoc/rpc/protocol.h` `ScenarioMetadata` to carry per-scenario craft parameter deltas (sampled from Gaussians with the configured sigmas).
- [ ] T098 [US6] Update `src/eval/variation_generator.cc` (or equivalent) to populate per-scenario craft parameter deltas in `populateVariationOffsets()`. Seeded per-scenario RNG, same pattern as existing wind/entry variations.
- [ ] T099 [US6] Update `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`: apply the received craft parameter deltas to the FDM model BEFORE the scenario starts. Verify the deltas actually take effect (bench test with a known delta and observe altered control response).
- [ ] T100 [P] [US6] Add unit test `tests/craft_variation_tests.cc`: verify per-scenario sampling is deterministic given a fixed RNG seed, verify deltas are within configured sigma bounds, verify zero sigma produces zero delta.
- [ ] T101 [US6] Run a CRRCSim training run (200 gens, pop 1000) with `CraftClDaSigma=0.15` and `CraftCtrlPhaseDelaySigma=0.05` (narrow start per spec recommendation). Verify training still converges; compare fitness to a baseline run without variations.

**Checkpoint US6**: Craft parameter variations integrated and training-validated. Foundation for sim-to-real robustness.

---

## Phase 9a: RC Smoothing Filter Experiment — ABANDONED 2026-04-13

**Purpose**: Replicate INAV's RC smoothing filter (pt3) in CRRCSim to naturally smooth bang-bang control outputs. Enable the matching filter in the real INAV config. If training converges with smooth commands, 023 can go to flight test without waiting for 024 craft variations.

**Background**: test3 (gen 243) shows strong fitness (-18k+) but bang-bang control — NN outputs saturate at ±1 on 25% of ticks. INAV has two filter stages on the command path, both currently disabled in our hb1 config:

1. **RC smoothing** (`rc_filter_lpf_hz`) — pt3 filter (3rd-order cascade) on ALL 4 channels including throttle. Currently set to 250Hz (passthrough). Default is 50Hz.
2. **Servo LPF** (`servo_lpf_hz`) — biquad Butterworth on servo channels only (not throttle). Currently 0 (disabled). Default is 20Hz.

The RC smoothing filter is the better choice for CRRCSim because it covers all channels including throttle, and the pt3 has a gentler rolloff than the biquad.

**INAV pt3 filter** (from `inav/src/main/common/filter.c`):
```c
// Gain computation:
float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f/3.0f) - 1); // ≈ 1.9615
float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
float k = dT / (RC + dT);

// 3-stage cascade (runs every PID tick):
state1 += k * (input - state1);
state2 += k * (state1 - state2);
state  += k * (state2 - state);
return state;
```

**Real xiao signal path**: NN eval at 10Hz → MSP_SET_RAW_RC at 20Hz (keepalive repeats last command) → INAV PID at 2kHz → RC smoothing pt3 at 2kHz → servo LPF (if enabled) → PWM output.

**CRRCSim timing model**: `getInputData()` is called every FDM frame at 333Hz (`dt=0.003` in `autoc_config.xml`). NN eval at 10Hz. The pt3 filter runs at 333Hz, smoothing the 10Hz NN command steps — analogous to INAV's 2kHz pt3 smoothing 20Hz MSP updates. The filter sees held commands between NN evals and continues settling toward them.

**Timing constants for `inputdev_autoc.h`**:
```
NN_EVAL_INTERVAL_MSEC     = 100   // 10Hz NN evaluation (existing)
RC_FILTER_HZ_DEFAULT       = 20   // pt3 cutoff frequency (new)
```
Note: no `SERVO_UPDATE_INTERVAL_MSEC` — the 20Hz MSP keepalive is a real-hardware liveness mechanism with no behavioral role in the sim filter model (clarified 2026-04-12).

### Tasks

- [ ] T114 Implement pt3 RC smoothing filter in `inputdev_autoc.cpp`. Replicate INAV's `pt3FilterGain()` + `pt3FilterApply()` as file-local functions (same algorithm, same cutoff correction factor 1.9615). Initialize at startup with `k` computed from FDM dt (3ms from `autoc_config.xml`) and cutoff frequency from `AUTOC_RC_FILTER_HZ` env var (default 20, 0=disabled). Maintain 3 filter instances (pitch, roll, throttle). Apply every `getInputData()` call (333Hz) to the cached `pitchCommand`/`rollCommand`/`throttleCommand` BEFORE CRRCSim scale conversion. Reset filter state at scenario start (pre-fill with initial commands to avoid transient).
- [ ] T115 Add filtered command columns to `data.dat`. Log both raw NN output (`outPt`, `outRl`, `outTh` — unchanged) AND the filtered surface commands (`fltPt`, `fltRl`, `fltTh`). This lets the viz show the pt3's smoothing effect and phase delay. Update `logEvalResults()` in `src/autoc.cc` to emit the new columns from `AircraftState`.
- [ ] T116 Add filtered command fields to `AircraftState`. New fields `filteredPitchCommand_`, `filteredRollCommand_`, `filteredThrottleCommand_` set by CRRCSim's `getInputData()` each frame. These capture what actually reached the FDM surfaces (post-filter) vs what the NN requested (pre-filter). Serialize in the cereal round-trip so they propagate to `data.dat`.
- [ ] T117 Update `xiao/inav-hb1.cfg`: `set rc_filter_lpf_hz = 20` and `set rc_filter_auto = OFF`. This enables the real INAV RC smoothing filter to match what the sim now models. Leave `servo_lpf_hz = 0` (the RC filter already covers servos upstream). Document the change and rationale.
- [ ] T118 Determine optimal filter frequency for 10Hz NN + 20Hz servo update at 333Hz FDM rate. Test 15Hz, 20Hz, and 30Hz in short training runs (50 gens, pop 2000). Compare control smoothness (mean |du/dt|, saturation %, phase delay) and fitness. Pick the frequency that eliminates bang-bang without killing tracking.
- [ ] T119 Update `sim_polar_viz.py` to plot filtered surface output (`fltPt`, `fltRl`, `fltTh`) alongside raw NN output. Add a subplot or overlay showing the pt3 smoothing and phase delay.
- [ ] T120 Full training run (400 gens, pop 10k or reduced if T118 suggests pop reduction works) with RC filter enabled. Compare fitness curve and control profiles to test3 (no filter). If mean |filtered command| < 0.6 and tracking fitness within 80% of test3, this is flight-test ready.
- [ ] T121 Investigate autoc RSS memory (~40GB at gen 1). Profile heap to find what's accumulating. This blocks long training runs on constrained hardware.
- [ ] T122 Reduce `RabbitSpeedNominal` from 13.0 to 12.0 m/s in `autoc.ini`. test3 analysis shows the NN at full throttle 72% of ticks and still averaging only 1.7 m/s faster than the rabbit — no headroom for maneuvering. Dropping to 12 m/s gives the underpowered craft breathing room and should reduce throttle saturation. Adjust `RabbitSpeedMin` accordingly (9.0 m/s).

**ABANDONED**: pt3 filter stunts training convergence (test6: -2225 at gen 55 vs test4: -4410). Filter mechanically prevents the quick corrections the NN needs during early evolution. All pt3 code backed out. Smoothness must come from fitness-based incentives instead. See `specs/BACKLOG.md` for full findings.

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Flight prep, final validation, documentation, and follow-through on research items.

- [ ] T102 Run the complete minisim smoke test recipe from `quickstart.md` one final time. All 12 pass criteria must pass. This is the "is the tree in a shippable state" gate.
- [ ] T103 [P] Run CRRCSim Milestone A → Milestone B → Milestone C progression as an end-to-end validation. Document fitness, per-path brittleness, throttle saturation (must achieve mean < 0.5 per SC#3), `d²output²` chatter, and discontinuity counts at each milestone. Verify SC#2 (sim p99 distance > 40m, p99 dd/dt negative) at Milestone B or C. This is the training validation gate — produces the weights for the next flight test.
- [REMOVED — U2] ~~T104~~ Merged into T076 — the measurements are documented once, at the time of measurement, not twice.
- [ ] T105 [P] Update `docs/COORDINATE_CONVENTIONS.md` if Change 6 introduces any new coordinate conventions (unlikely — the body frame convention is unchanged — but verify).
- [ ] T106 [P] Update `CLAUDE.md` with any new technologies or conventions introduced by 023 (already done via update-agent-context.sh, verify still accurate).
- [ ] T107 Generate final NN weights via `nn2cpp` for xiao flash: codegen emits field-name access per T031. Verify the generated xiao-side C++ compiles and produces output bit-identical to sim forward pass on a test fixture (from BACKLOG "Cross-ISA NN output comparison").
- [ ] T108 Walk the pre-flight checklist from `~/.claude/projects/-home-gmcnutt-autoc/memory/project_preflight_checklist.md`. The failsafe verification section (NEW in 023) is a gating step — bench-test SBUS disconnect during autoc span, verify full failsafe chain works.
- [ ] T109 Flash final weights to xiao: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed -t upload`. Bench verify: TX power on, arm, verify CH6 override → MANUAL mode, servo actuation, xiao console autoc enable → NN running.
- [ ] T110 [P] Delete `specs/023-ood-and-engage-fixes/train-eval-code-dedup.md` — this is a one-off research doc that becomes obsolete once Phase 0b lands (per spec). Removing it closes the TODO. Keep `docs/inav-signal-path-audit.md` (durable).
- [ ] T111 [P] Optional follow-up (Phase 0b step 9 from `contracts/evaluator.md` §Future shape): refactor `buildEvalData(EvalJob)` to `class Evaluator` + `TrainingEvaluator` / `EliteReevalEvaluator` / `StandaloneEvalEvaluator` subclasses. Zero behavior change, better compile-time safety. Can land any time after T048 — does not block flight test.
- [ ] T112 Flight test: deploy the Phase 3-trained NN weights and fly. Per success criteria #6 in spec.md, blackbox analysis must show **zero projection discontinuities** in the bearing inputs. If any appear, representation change is incomplete somewhere in the pipeline — stop and diagnose before further flights.
- [ ] T113 Post-flight analysis: run the same diagnostic suite as the 2026-04-07 flight report. Verify SC#1 (|dd/dt| < 20 m/s on first engage sample), SC#4 (distance < 30m for > 5s on at least one autoc span), SC#6 (zero projection discontinuities in bearing inputs). Document input distribution vs sim, throttle saturation, AHRS quality. Results in a new `flight-results/flight-<date>/flight-report.md`. Track metrics across 023 flights to catch regressions.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — verify clean baseline. Trivially parallel with nothing.
- **Phase 2 (Foundational: 0a + 0b)**: Depends on Phase 1. **Blocks all user stories.** Must complete entirely.
  - Phase 0a.1 and Phase 0b can start in parallel (different files: 0a touches `nn_inputs.h` + input path; 0b touches `buildEvalData` + `src/autoc.cc:runNNEvaluation`). They should NOT be interleaved — complete 0a.1 fully before starting 0a.2, and complete 0b before touching 0a.2 (because 0b's acceptance test exercises the eval path).
  - Phase 0a.2 (27→33 migration) depends on 0a.1 landing first.
  - Phase 0a.3 is a scratch-branch verification; does not block anything else.
- **Phase 3 (US1 Wrap-Free Bearing)**: Depends on Phase 2. Primarily validates Phase 0a.2 work end-to-end.
- **Phase 4 (US2 Clean Engage)**: Depends on Phase 2. Can run in parallel with US1 — different files (history reset + CRRCSim engage delay + INAV config vs bearing representation). Actual parallelism depends on team capacity.
- **Phase 5 (US3 Throttle Discipline)**: Depends on Phase 2. Can run in parallel with US1, US2. Lexicase changes are in `src/eval/selection.cc`, disjoint from everything else.
- **Phase 6 (US4 Discontinuity Paths)**: Depends on US1 (needs the wrap-free representation to test against) + Phase 2. Path generator changes are self-contained.
- **Phase 7 (US5 Authority Limit)**: Depends on US1 + US2 + a Milestone A observational run. CONDITIONAL — may be skipped.
- **Phase 8 (US6 Craft Variations)**: Depends on Phase 2. Can run in parallel with US1-4. Independent.
- **Phase 9 (Polish)**: Depends on all desired user stories. Final validation, flight test prep.

### Within Phase 2 (critical path)

- **0a.1** (T004 → T023) is a linear sequence — mostly because each task depends on the previous refactor being in a consistent state. Exceptions: T005, T019, T020 are [P] — test additions that don't depend on each other.
- **0a.2** (T024 → T035) depends on 0a.1 being complete. T026 ([P]) can run in parallel with T025 (helper implementation). T033 is [P] with the other 0a.2 tasks once the struct has grown.
- **0b** (T037 → T049) can run in parallel with 0a.1 (different files). Must complete before flight test (T112). The migration steps T040 → T041 → T042 → T043 → T044 → T045 → T046 are sequential (each commit builds on the previous).
- **Config knob additions** T050 → T053 are mostly [P] among themselves and independent of 0a / 0b.

### Within each user story

- Tests BEFORE implementation where the contract mandates it (per Constitution I and the contracts' test matrices).
- Models / data structures before services / logic.
- Unit tests before integration tests.

### Parallel Opportunities

- **0a.1 and 0b**: different files, independent migrations. Can run truly in parallel.
- **US1 and US2 and US3 and US4 and US6**: largely independent once Phase 2 is done. US5 is conditional and depends on Milestone A observation.
- **Within US2**: Change 1 (history reset), Change 1b (engage delay), Change 1c (INAV bench work) all touch different files. Change 1c is bench work, not code changes.
- **Contracts tests marked [P]**: different test files, independent assertions.

---

## Parallel Example: Phase 2 startup

```bash
# Developer A starts Phase 0a.1 (type-safe NN struct refactor)
Task: T004 — Create include/autoc/nn/nn_inputs.h with 27-field struct
Task: T005 — [P] Create tests/nn_inputs_tests.cc with sizeof/alignment/topology tests

# Developer B starts Phase 0b in parallel (train/eval dedup)
Task: T037 — Create include/autoc/eval/build_eval_data.h with EvalPurpose enum
Task: T038 — Create tests/build_eval_data_contract_tests.cc with stub contract tests
Task: T039 — [P] Create tests/eval_determinism_tests.cc with stub end-to-end test
```

## Parallel Example: Phase 4 (US2) Change 1 + 1b

```bash
# Change 1 (history reset) tests and implementation
Task: T060 — [P] Implement resetHistory() in aircraft_state.cc
Task: T061 — [P] Create tests/engage_reset_tests.cc with 6 unit tests

# Change 1b (engage delay) tests can run in parallel
Task: T066 — [P] Create tests/engage_delay_tests.cc with 3 unit tests
```

---

## Implementation Strategy

### MVP Scope (US1 + US2)

**Minimum deliverable for a flight-ready 023 build**: US1 (wrap-free bearing representation) + US2 (clean engage handoff). Without US1, the NN still has wrap discontinuities. Without US2, the aircraft can't engage cleanly. Together they deliver the core 023 value.

1. Complete Phase 1 (Setup).
2. Complete Phase 2 (Foundational: 0a + 0b). Type-safe NN interface and train/eval dedup are both landed.
3. Complete Phase 3 (US1) — validates Phase 0a.2 end-to-end.
4. Complete Phase 4 (US2) — engage handoff clean.
5. Run quickstart.md minisim smoke test. Promote to CRRCSim Milestone A.
6. Flight test if CRRCSim results are flight-ready.

### Incremental Delivery (full 023)

1. MVP (US1 + US2) → flight test → post-flight diagnostic.
2. Add US3 (throttle lexicase) → retrain → compare fitness and throttle distribution.
3. Add US4 (discontinuity paths) → retrain → verify hard-regime coverage.
4. Run Milestone A observation: is US5 (authority limit) triggered? If yes, run it.
5. Add US6 (craft parameter variations) → retrain.
6. CRRCSim Milestone C complete → final flight test → production 023 NN weights.

### Parallel Team Strategy

With multiple developers:

1. Developer A: Phase 0a (T004–T036). Touches `nn_inputs.h`, all input-gathering code sites, tests.
2. Developer B: Phase 0b (T037–T049) + config knobs (T050–T053). Touches `src/autoc.cc runNNEvaluation`, `protocol.h`, `config.cc/h`, `autoc.ini`.
3. Developer C: pre-fetching US1 (T054–T059) paperwork while A+B finish Phase 2. US1 code tasks start once Phase 0a.2 is in main.
4. After Phase 2 complete: A takes US1+US2 follow-through (bearing + engage), B takes US3+US4 (lexicase + paths), C takes US6 (craft variations). US5 is conditional and handled by whoever's free after Milestone A observation.

### Notes on Constitution Compliance

- **I. Testing-First**: Every phase includes test tasks BEFORE implementation. Phase 0a.1 explicitly verifies the struct layout contract via deliberate-break in T036. Phase 0b gates the refactor on contract tests + end-to-end determinism.
- **II. Build Stability**: Each task is scoped so `bash scripts/rebuild.sh` succeeds after it lands. T022 is an explicit pre/post diff verification gate. T023 verifies xiao builds.
- **III. No Compatibility Shims**: T030 (weight file version check) rejects old NN01 files with a clear error — no auto-migration. Protocol change T053 requires joint rebuild. Clean cuts throughout.
- **IV. Unified Build**: No changes to top-level CMake structure. New tests slot into `tests/` directory automatically.

---

## Notes

- **All tasks include absolute file paths** per the checklist format.
- **[P] tasks**: different files, no dependencies on incomplete tasks within the same phase.
- **[Story] labels**: US1 through US6 map to user stories above; Setup, Foundational, and Polish phases have no story label.
- **Xiao changes**: tasks touching `xiao/src/*` require a `pio run -e xiaoblesense_arduinocore_mbed` rebuild after each change. Xiao-side changes are interleaved with autoc changes — don't leave xiao in a half-refactored state between commits.
- **CRRCSim changes**: tasks touching `crrcsim/src/*` require rebuilding the submodule (`cd crrcsim/build && make -j8`). Commit the submodule pointer bump in autoc AFTER verifying the CRRCSim change works, per the "submodule merge order" lesson in feedback memory.
- **Research docs** (`docs/inav-signal-path-audit.md`, `docs/failsafe-behavior-audit.md`): durable reference material. Update with Change 1c measurements (T076) and Milestone A observation (T095). Do NOT delete.
- **Feature-scoped doc** (`specs/023-ood-and-engage-fixes/train-eval-code-dedup.md`): one-off survey. Delete in T110 after Phase 0b lands.
- **Commit granularity**: one commit per logical task or tightly-coupled group. Each commit must leave main buildable and tests green.
- **Stop at any checkpoint** to validate the story independently. Especially: after Phase 2 before user stories, after US1+US2 before US3+, after each CRRCSim milestone before the next.

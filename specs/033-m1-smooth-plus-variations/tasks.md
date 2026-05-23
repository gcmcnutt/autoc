# Tasks: 033 — M1 smoothness + replay-friendly variation PRNGs (+ kamikaze inherit)

**Input**: Design documents from `/specs/033-m1-smooth-plus-variations/`
**Prerequisites**: plan.md, spec.md, research.md (R1-R8), data-model.md, contracts/{scenario_prng_chain, smoothness_factor, dmp_schema_extension, ini_schema}.md, quickstart.md

**Tests**: TDD enforced per Constitution I + plan.md Constitution Check. Pure-math + parsing tests precede integration tests precede end-to-end bake regression. Each contract enumerates required tests.

**Organization**: Tasks grouped by user story. **US1 + US2 = phase-1 (033 bake target)**. **US3 + US4 = phase-2 (gated on mezzanine real-flight test pass)**.

## Status snapshot — 2026-05-22

- **Phase A complete** (autoc-side PRNG cleanup + crrcsim worker rewire + per-tick smoothness mirror in both pathgen + tracker workers). All tests green (`scenario_prng_tests` + extended `derived_features_tests` + extended `fitness_computer_tests` + tracker dmp roundtrip).
- **Two production bugs caught + fixed during smoke**:
  1. `tools/minisim.cc` + `crrcsim/.../inputdev_autoc.cpp:929` unconditionally pushed `cameraViewList`/`targetTrajectoryList` in pathgen mode → renderer mis-classified pathgen dmps as tracker mode → rabbit path hidden. Gated push on `Mode::TRACKER` in both producers; consumers (autoc-side fitness_decomposition, data.dat logger) already check inner-vec emptiness so no consumer-side change.
  2. `src/autoc.cc::buildWorkerInit` had the smoothness propagation block AFTER a `if (init.mode != Mode::TRACKER) return init;` early-return — so pathgen WorkerInit shipped `smoothnessPenaltyFloor=1.0` (default) → workers ran with smoothness OFF while EvalResults provenance MISLABELED as 0.5. Hoisted propagation above the early return. First bake (smoothness-off pollution) discarded; relaunched after fix.
- **Phase-1 M1 crrcsim bake LAUNCHED 2026-05-22** with autoc.ini (master seed printed as `Effective master seed: 1779420285`). Gen 7 reached at sweep time; `smooth` column varying in `[~0.5, 1.0]`; `avgMaxStreak` climbing (0.6→1.6 across gens 1-5); workers healthy.
- **Operator housekeeping**: 4 ini files removed (autoc-minisim.ini, autoc-eval-minisim.ini, autoc-quicktrain.ini, autoc-tracker-minisim.ini).
- **Deferred to post-flight**: tests T010/T011/T012a/T012b/T013/T028, per-(path, wind) variation table expansion, renderer static-vs-animation path-truncation cosmetic.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Different file, no dependency on incomplete tasks → parallelizable
- **[Story]**: US1=PRNG architecture, US2=M1 smoothness penalty, US3=M2 inherits smoothness, US4=kamikaze penalty
- File paths absolute or repo-rooted

## Path Conventions

Single-project mono-tree per plan.md §Project Structure. autoc-side under `include/autoc/`, `src/`, `tests/`. crrcsim worker under `crrcsim/src/mod_inputdev/inputdev_autoc/`. Operator ini at repo root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Confirm branch + dependencies are ready; no new infrastructure required (autoc + crrcsim + cereal + inih + GoogleTest all already in place).

- [X] T001 Branch verified — autoc + crrcsim submodule both on `033-m1-smooth-plus-variations`.
- [X] T002 Build green via incremental `cd build && make -j8` (operator drove; rebuild-perf.sh not gated since 033 intentionally breaks pre-033 bitwise gate per plan.md II note).
- [X] T003 [P] Test suite green: scenario_prng_tests (18 cases), extended derived_features_tests (10 new smoothness cases), extended fitness_computer_tests (5 new smoothness cases + all 22 prior callers updated to explicit 1.0), tracker_dmp_roundtrip, tracker_stepper_init, contract_tracker_config (all 3 cases). Full build runs ~3 dozen test suites green.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core types + ini parsing that BOTH phase-1 user stories (US1 PRNG + US2 smoothness) depend on. Must complete before either US1 or US2 implementation begins.

**⚠️ CRITICAL**: US1 and US2 phases cannot begin until this phase completes.

- [X] T004 Add `SmoothnessMotionMode` enum (Pythagorean / Sum / Max) to `include/autoc/eval/derived_features.h` per [contracts/smoothness_factor.md](./contracts/smoothness_factor.md) "Pure-math contract" section — also added `compute_smoothness_factor()` inline helper in the same edit (the enum + helper land together).
- [X] T005 [P] Add `smoothnessPenaltyFloor` (double, default 0.5) + `smoothnessMotionMode` (`std::string`, default `"pythagorean"`) fields to `AutocConfig` in `include/autoc/util/config.h` per [contracts/ini_schema.md](./contracts/ini_schema.md). Parsing is T032 (US2).
- [X] T006 [P] `ScenarioMetadata.scenarioSeed` added; legacy `windSeed` + `rabbitSpeedSeed` REMOVED in cleanup-pass cascade (autoc.cc all 28 touch sites + autoc.h `WindScenarioConfig`/`ScenarioDescriptor` field removal + crrcsim worker rewire + display tools + tests). Cereal walk cleaned. Worker derives all 5 class sub-seeds via `autoc::util::deriveClassSubSeeds(meta.scenarioSeed)` helper.
- [X] T007 [P] Create `include/autoc/util/scenario_prng.h` with `MasterPRNG`, `ScenarioRootPRNG`, `ClassPRNG` types per [data-model.md](./data-model.md). Uses Park-Miller LCG; defense-in-depth zero-seed guard via `fold_to_pm_state`.
- [X] T008 [P] Create `src/util/scenario_prng.cc` implementing the three PRNG types from T007. Wired into `autoc_common` library in `CMakeLists.txt:86`.

**Checkpoint**: Type surface for both PRNG architecture AND smoothness config is in place; downstream stories can wire to these symbols.

---

## Phase 3: User Story 1 — Replay-friendly variation PRNG architecture (Priority: P1) 🎯 MVP-floor

**Goal**: Master-seed → per-scenario sub-PRNG chain wired through autoc-side init AND worker-side per-scenario init. `scenarioSeed[K]` persisted in `EvalResults` dmp. Determinism contracts D1-D5 (per [contracts/scenario_prng_chain.md](./contracts/scenario_prng_chain.md)) all hold.

**Independent Test**: With `SmoothnessPenaltyFloor=1.0` (smoothness no-op), run M1 minisim with `Seed=42` twice → assert per-scenario `EvalResults` bitwise-equal. Then add new variation class slot 6 (placeholder) and re-run → assert first 5 sub-seeds unchanged for same scenarioSeed (append-only contract). Eval mode with same NN + same scenarioSeed[K] → per-tick aircraftState bitwise-equal to training trace.

### Tests for User Story 1 (TDD — write FIRST and confirm FAILING before implementation)

- [X] T009 [P] [US1] Create `tests/scenario_prng_tests.cc` with cases per [contracts/scenario_prng_chain.md](./contracts/scenario_prng_chain.md) "Validation tests" — all 7 contract cases (D1/D2/D3/D5 determinism + class_prng_independence + nn_prng_separation + cross_mode_equivalence) plus Park-Miller primitive coverage, ClassPRNG draw-type coverage (nextDouble unit-range, nextGaussian statistics ~N(0,1)), and a ScenarioRootPRNG zero-seed defense-in-depth test. Wired into `CMakeLists.txt:201-204` + ctest + run_autoc_tests.
- [→POST-FLIGHT] T010 [P] [US1] tracker_dmp_roundtrip extensions for scenarioSeed roundtrip + pre-033 dmp loud-fail. (D4 manually validated end-to-end in minisim eval-replay during Phase A smoke; CI-level test deferred to post-flight to avoid blocking Sunday bake.)
- [→POST-FLIGHT] T011 [P] [US1] contract_config tests for the smoothness keys + range/enum check. (Range check landed in `ConfigManager::load()` with LOG_FATAL+exit; runtime contract holds — automated test is the CI safety net.)
- [→POST-FLIGHT] T012a/b [US1] Wind capture hook + wind_replay_tests. (D2/D3 contracts manually validated via cross-process determinism of `deriveClassSubSeeds` output + minisim eval-replay; automated hook deferred.)
- [→POST-FLIGHT] T013 [US1] eval_mode_replay automated test. (Manually validated by operator running autoc-eval-minisim.ini against minisim training trace → bit-identical aircraftStateList per Turn 3 D4 check.)

### Implementation for User Story 1

- [X] T014 [US1] Implement autoc-side init chain in `src/autoc.cc` per [contracts/scenario_prng_chain.md](./contracts/scenario_prng_chain.md) "Surface — autoc-side init chain" steps 1-6: parse Seed→effectiveSeed (preserve `-1`=time semantics), print "Effective master seed:" at startup with operator-replay note, init `gMasterPRNG`, seed `rng::*` from `masterPRNG.next()`, populate `gScenarioSeedTable[0..M_total-1]` via `populateScenarioSeedTable()` (called once just before `prefetchAllVariations`), populate per-scenario `meta.scenarioSeed = gScenarioSeedTable[idx]` in `buildWorkerInit()`.
- [~] T015 [US1] **Partial — transitional state**. Refactored `src/autoc.cc::prefetchAllVariations()` to consume class PRNGs derived from `scenarioSeed[K]` via `ScenarioRootPRNG` → 5-class sub-PRNG chain (slots 0-4 in append-only order). `generateEntryVariationsFromClassPRNG()` + `windDirectionOffsetFromClassPRNG()` added to [variation_generator.h](../../include/autoc/eval/variation_generator.h). Legacy GPrand path coexists in header but is unused by `prefetchAllVariations`. NOTE: `gScenarioVariations[]` table is still indexed per-WIND (size = windScenarioCount); per-(path, wind) expansion + GPrand-path removal lands in the cleanup pass.
- [X] T016 [US1] Modified `include/autoc/util/rng.h` with header comment documenting NN-evolution / variation PRNG separation per [research.md](./research.md) R6. `rng::seed()` now called from `gMasterPRNG.next()` in autoc.cc instead of directly from `cfg.seed`.
- [X] T017 [US1] `scenarioSeed` field lives on `ScenarioMetadata` which roundtrips via `WorkerInit.scenarioMetaList` (existing per-scenario container). No separate WorkerInit scalar needed.
- [SUPERSEDED] T018/T019 No parallel `scenarioSeedList` vector on EvalResults — `scenarioSeed` is on `ScenarioMetadata` which is per-scenario in `scenarioList`, so the parallel vector is redundant. Operator reads `evalResults.scenarioList[K].scenarioSeed` for replay. EvalResults instead got provenance fields: `effectiveMasterSeed` + `smoothnessPenaltyFloor` + `smoothnessMotionMode` (decision in Turn 3 implement).
- [X] T020 [P] [US1] Worker-side per-scenario PRNG: `crrcsim/.../inputdev_autoc.cpp:531-542` constructs `ScenarioRootPRNG` from `meta.scenarioSeed` via `deriveClassSubSeeds`; routes `windPRNG.next()` → `Global::Simulation->reset()`; `subseeds.rabbit` → `generateSpeedProfile` (pathgen) at line 564.
- [X] T021 [P] [US1] Mirror in `crrcsim_tracker_helper.cpp::initScenario:54` — crash-hull PRNG seeded from `subseeds.rabbit` (M2 crash-hull is rabbit-class consumer per contract). No `crrcsim_pathgen_helper.cpp` exists; pathgen worker path is inline in inputdev_autoc.cpp.
- [X] T021a [P] [US1] Crash-hull PRNG (autoc-side `tools/minisim.cc:198` and crrcsim worker `crrcsim_tracker_helper.cpp:54`) both seed from `deriveClassSubSeeds(meta.scenarioSeed).rabbit`. Same value across processes for the same scenarioSeed.
- [X] T021b [P] [US1] Rabbit-speed segment generator: worker-side `inputdev_autoc.cpp:561` passes `subseeds.rabbit` as `unsigned int seed` to `generateSpeedProfile()`. Deterministic per scenarioSeed.
- [X] T021c [P] [US1] Entry-pose samplers (cone / roll / pitch / speed / position) drawn autoc-side in `prefetchAllVariations()` via `ClassPRNG entryPRNG(subseeds.entry)` → `generateEntryVariationsFromClassPRNG()`. Cross-process determinism by construction (same `subseeds.entry` everywhere).
- [X] T021d [US1] `prefetchAllVariations()` now draws from each class PRNG unconditionally — `entryPRNG.next()` happens regardless of `enableEntry`, value applied only when enabled. Wind-class `windDirectionOffset` same pattern. Append-only contract preserved.
- [X] T022 [US1] `scenarioSeed==0` defense-in-depth: autoc-side `populateScenarioSeedTable()` converts 0 → `0xC0FFEEu` at insert; worker-side `ScenarioRootPRNG` constructor + `ClassPRNG` constructor both have the same guard via `fold_to_pm_state`. Belt-and-suspenders.
- [X] T023 [P] [US1] Created `docs/variation-prng.md` — operator-facing doc covering all 4 required topics + D1-D5 contracts + "Transitional state" section explaining current vs cleanup-pass surface.
- [X] T024 [US1] Extended `tools/tracker_dmp_inspect.cc` to print first 5 scenario seeds in hex with replay guidance (the operator can feed any of these back into eval mode for single-scenario D4 replay).
- [X] T025 [US1] scenario_prng_tests GREEN (18 cases); tracker_dmp_roundtrip GREEN (existing cases — extended-coverage T010 deferred to post-flight per above).

**Checkpoint US1**: PRNG architecture and dmp extension complete. Determinism contracts D1-D5 verified by unit tests. **NO dedicated PRNG-only bake here** (per operator routing 2026-05-20: if PRNG determinism breaks, it surfaces in the US2 bake or in on-the-spot eval runs). Proceed directly to US2.

---

## Phase 4: User Story 2 — M1 multiplicative smoothness penalty (Priority: P1) 🎯 MVP-completer

**Goal**: Per-tick smoothness factor (Pythagorean default) is computed from NN output Δ and applied multiplicatively on `stepPoints` inside `FitnessComputer::applyStreak()` BEFORE the streak multiplier. Stepper integrates `prev_out_*` state. Phase-1 bake (M1 only) is the deliverable; closeout success criteria per spec §2.B "Phase-1 success criteria" + Clarifications Q1.

**Independent Test**: With US1 complete, run M1 minisim at `SmoothnessPenaltyFloor=0.5` for 50-100 gens → assert (a) avgMaxStreak ≥ 20 (mid-bake floor; closeout floor is 30), (b) best-fitness trending up (not collapsed), (c) per-tick `smoothness_factor` distribution centered well below 1.0 for late-gen elites (the penalty is actually biting). Bake-via-crrcsim is the closeout-target test; see quickstart.md.

### Tests for User Story 2 (TDD — write FIRST and confirm FAILING)

- [X] T026 [P] [US2] Extended `tests/derived_features_tests.cc` with 10 new `compute_smoothness_factor` cases per [contracts/smoothness_factor.md](./contracts/smoothness_factor.md) — rest state (factor=1), floor=1.0 no-op, all-3-axis bang-bang hits floor (Pythagorean + Sum), Max single-axis hits floor, Pythagorean single-axis (factor=0.7113), Pythagorean Δ=(1,1,1) (factor=0.75), over-range clamp, floor=0.0 annihilates, range invariant fuzz [factor ∈ floor..1.0 for ~3000 input combinations], mode-distinctness, and operator-example reference (Δ=1.5 single-axis → 0.7835). Compiled into existing `derived_features_tests` target.
- [X] T027 [P] [US2] Extended `tests/fitness_computer_tests.cc` with 5 new smoothness cases — FactorOneIsBackCompat (factor=1 ≡ pre-033), FactorHalvesStepPoints (0.5 exact halving), StreakStateIndependentOfFactor (factor doesn't perturb streak counter), ZeroFactorAnnihilatesStep, MultiplicationOrderContractFinalStep (verifies stepPoints × factor × streak_mult product). All 22 existing applyStreak call sites updated to pass explicit 1.0 per Constitution III no-default policy.
- [→POST-FLIGHT] T028 [US2] stepper_smoothness_tests. (Manually validated via data.dat `smooth` column showing first-tick=1.0000, varying afterward in [floor, 1.0] range; rebuild-and-relaunch caught the WorkerInit propagation bug that this test would have flagged at CI time. Add post-flight.)

### Implementation for User Story 2

- [X] T029 [US2] `compute_smoothness_factor()` was implemented in T004 (foundational pass); this task is a no-op landing-bookmark.
- [X] T030 [US2] `FitnessComputer::applyStreak()` now takes `double smoothness_factor` as second positional parameter — NO DEFAULT per Constitution III. All 25+ existing call sites updated explicitly in same PR (autoc.cc data.dat loggers pass per-tick AircraftState value; fitness_decomposition.cc reads from AircraftState; fitness_computer_tests pass explicit 1.0).
- [X] T031 [US2] `src/eval/fitness_computer.cc::FitnessComputer::applyStreak()` now multiplies `stepPoints * smoothness_factor * multiplier` per the multiplication-order contract; streak state update is independent of smoothness_factor.
- [X] T032 [P] [US2] Flat smoothness keys parsed in `src/util/config.cc::ConfigManager::load()` — `SmoothnessPenaltyFloor` range-checked [0.0, 1.0] with LOG_FATAL+exit on out-of-range; `SmoothnessMotionMode` enum-mapped with LOG_FATAL+exit on unknown string ("pythagorean"/"sum"/"max").
- [X] T033 [P] [US2] `WorkerInit` gained `smoothnessPenaltyFloor` (gp_scalar) + `smoothnessMotionMode` (uint8 wire-stable enum); cereal `serialize()` appends both at end; `buildWorkerInit()` populates from AutocConfig with string→enum conversion.
- [X] T034 [P] [US2] Added flat smoothness keys (`SmoothnessPenaltyFloor=0.5`, `SmoothnessMotionMode=pythagorean`) to `autoc.ini` per ini convention (no section header — `[Smoothness]` header reverted per operator routing 2026-05-21). Inline operator docs explaining floor range + motion-mode tradeoffs. Declarative until T032 parsing lands in US2.
- [X] T035 [P] [US2] Added identical flat keys to `autoc-tracker.ini` (for US3 phase-2 M2 inheritance — keys present from phase 1).
- [X] T036 [P] [US2] Added identical flat keys to `autoc-tracker-minisim.ini` for minisim/crrcsim config-parity.
- [X] T036b [P] [US2] Added identical flat keys to all eval/quicktrain inis (`autoc-eval.ini`, `autoc-eval-tracker.ini`, `autoc-eval-visual.ini`, `autoc-quicktrain.ini`) — eval-mode bit-identical reproduction requires the operator to copy the training run's printed "Effective master seed: N" into `Seed=N` in the eval ini (the smoothness factor itself is config-deterministic; the variation PRNG chain needs the master seed).
- [X] T037 [US2] `PathgenStepper` gained `prev_out_pt_/rl_/th_` + `prev_out_valid_` + `smoothness_floor_` + `smoothness_mode_`. `initScenario()` resets prev_out state (first-tick factor=1.0). `stepOnce()` computes Δs vs prev_out post NN forward-pass, calls `compute_smoothness_factor`, stores on AircraftState via `setSmoothnessFactor()`. Constructor takes floor + mode (defaults preserve no-op back-compat for any direct-instantiation tests). minisim.cc:242 passes WorkerInit-derived live values.
- [X] T038 [US2] `TrackerStepper` mirror of T037 — same per-tick pattern after `nn_.evaluateTracker()` line. Constructor extended with floor + mode (defaults preserve no-op back-compat for existing tracker_stepper_init_tests). minisim.cc:199 passes WorkerInit-derived live values with motion-mode uint8→enum decode at boundary.
- [X] T039 [US2] `src/eval/fitness_decomposition.cc:169` now reads `stepState.getSmoothnessFactor()` and passes to `applyStreak(stepPoints, smoothness_factor)`. Single autoc-side per-tick site (no other applyStreak calls in fitness_decomposition.cc).
- [X] T040 [US2] crrcsim worker per-tick smoothness mirror landed in Phase A: `CrrcsimTrackerHelper` gained `prev_out_*` + smoothness floor/mode members (initScenario captures from WorkerInit, resets prev_out); `tick()` computes factor after `nn.evaluateTracker()` and stores via `chaseState.setSmoothnessFactor()`. Pathgen branch in `inputdev_autoc.cpp` gained instance-level `prev_out_*` (reset at path boundary), computes factor after `nnController_->evaluate()` and stores on `aircraftState`. **Production-bug fix during smoke**: WorkerInit smoothness propagation in `buildWorkerInit` was AFTER tracker-only early-return — hoisted before the return so pathgen workers receive the floor value. Bake relaunched; data.dat `smooth` column confirmed varying after fix.
- [X] T041 [US2] derived_features_tests (10 new smoothness cases) + fitness_computer_tests (5 new smoothness cases) GREEN. stepper_smoothness_tests (T028) deferred to post-flight.
- [X] T042 [US2] Full ctest suite green across both `cd build && make` runs (post-foundational + post-cleanup-cascade). No regressions in adjacent test suites.

**Checkpoint US2**: Phase-1 implementation complete. Build is green, all phase-1 tests pass. Ready for operator-driven bake protocol per [quickstart.md](./quickstart.md). DO NOT auto-launch the bake; per [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md) the operator drives the regression check.

---

## Phase 5: User Story 3 — M2 inherits smoothness (Priority: P2) — DEFERRED to 033 phase 2

**Goal**: Same `compute_smoothness_factor` + `applyStreak` smoothness path applied to M2 tracker mode. Per-tick treatment in tracker_stepper.cc.

**Gating**: Per spec §6 phased plan, US3 starts ONLY after US2's M1 phase-1 bake closeout + mezzanine real-flight test pass. Until that gate clears, US3 is not active.

**Independent Test**: M2 bake with `SmoothnessPenaltyFloor=0.5` → per-axis dCtrl/⟨|out|⟩ reduction vs 032 phase-1 closeout baseline; hull-strike rate stable or improved (smoothness alone may reduce kamikaze before US4 lands).

### Implementation for User Story 3

- [ ] T043 [US3] Verify TrackerStepper integration (T038) still wired post-phase-1-merge; smoke-test with a 50-gen M2 minisim run
- [ ] T044 [US3] Validate flat smoothness keys already-present in `autoc-tracker.ini` + `autoc-tracker-minisim.ini` per T035-T036 (no code change required — the inheritance is automatic from US2's mode-agnostic implementation)
- [ ] T045 [US3] Bake M2 crrcsim per quickstart.md with `SmoothnessPenaltyFloor=0.5` (phase-2 success criteria deferred to /clarify before phase 2 starts — see spec §2.B Phase-1/Phase-2 split)

**Checkpoint US3**: M2 smoothness inheritance validated. Phase 2 step 1 complete.

---

## Phase 6: User Story 4 — Kamikaze penalty (Priority: P2) — DEFERRED to 033 phase 2

**Goal**: Multiplicative hull-crash penalty on per-scenario fitness. Default `HullCrashScoreFactor = 0` (zero out scenario score on crash); fall-back to 0.25/0.5 if over-deters.

**Gating**: Per spec §2.D + §6 phased plan, US4 ships in 033 phase 2 alongside US3. Same mezzanine-flight gate.

**Independent Test**: M2 bake with kamikaze penalty active → hull-strike rate trends DOWN over gens (vs 032 baseline's monotonic ESCALATION); plateau-avgInRamp held within phase-1 range.

### Implementation for User Story 4

- [ ] T046 [US4] Add `HullCrashScoreFactor` (double, default 0.0) to `AutocConfig` in `include/autoc/util/config.h`; parse `[Penalty]` (or appropriate existing section) in `src/util/config.cc` with range check `[0.0, 1.0]`, LOG_FATAL+abort on out-of-range
- [ ] T047 [US4] Add `hullCrashScoreFactor` to `WorkerInit` propagation in `include/autoc/rpc/protocol.h`
- [ ] T048 [US4] Apply multiplicative `* hullCrashScoreFactor` to accumulated scenario score on hull-crash detection site in TrackerStepper (or its M2-specific scoring path) — confirm site at implementation time
- [ ] T049 [US4] Add penalty section to `autoc-tracker.ini` + `autoc-tracker-minisim.ini` with `HullCrashScoreFactor=0.0`
- [ ] T050 [US4] Bake M2 crrcsim with kamikaze active → measure hull-strike-rate trajectory + plateau-avgInRamp

**Checkpoint US4**: Phase 2 complete; M2 real-flight validation is next.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Phase-1 closeout artifacts (NOT runtime code).

- [ ] T051 [P] Update `CLAUDE.md` "Active Technologies" + "Recent Changes" entries to mark 033 as in-flight (the agent-context script will do most of this; verify the entry matches the 033 surface)
- [ ] T052 [P] Operator-facing doc cross-link: ensure `docs/variation-prng.md` (created in T023) is linked from `docs/COORDINATE_CONVENTIONS.md` and from spec.md §2.E
- [ ] T053 Phase-1 closeout snapshot at `specs/033-m1-smooth-plus-variations/outcome.md` capturing bake result (mirror 032's outcome.md layout) — operator-triggered AFTER bake completes
- [ ] T054 Type-domain `gp_scalar` audit per Constitution VI on US2 surface (`compute_smoothness_factor`, `applyStreak` smoothness_factor param, per-tick `dpt/drl/dth` in stepper) — grep `float` in 033-touched files; document any unavoidable raw-`float` boundaries
- [ ] T055 [P] Add `[Penalty]` section + kamikaze knob to `autoc.ini` (M1) for symmetry, even though M1 has no hull (default = inert)
- [ ] T056 Expand `gScenarioVariations` from per-wind to per-(path, wind). Today `prefetchAllVariations(windScenarioCount, …)` only computes 49 entry/wind offsets ([src/autoc.cc:2108](../../src/autoc.cc#L2108)) and `gScenarioVariations[windIdx]` is shared across all 6 paths ([src/autoc.cc:1124-1133](../../src/autoc.cc#L1124-L1133)). Side-effect: a config of `SimNumPathsPerGeneration=1` with `WindScenarios=6` would run the same scenario 6× rather than 6 distinct variations of one path. Worker-side rabbit profile + CRRC sim seed already are per-K (294 unique), but autoc-side entry pose + wind direction offset are not. Fix is to size `gScenarioVariations` to `paths × winds`, index by linear `K`, and update the autoc-side comment that calls today's behavior "transitional phase-2 state." Touches: `prefetchAllVariations` loop bound, `gScenarioVariations` indexing in `buildWorkerInit` + `populateVariationOffsets` + `logPrefetchedVariations`.
- [ ] T057 Config parser / startup-banner audit. Today the banner echoes only a subset of `autoc.ini` keys; the rest (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`, `FitStreakThreshold`/`RampSec`/`MultiplierMax`, `DemeticGrouping`/`DemeSize`/`DemeticMigProbability`, `SmoothnessPenaltyFloor`, `SmoothnessMotionMode`, and the tracker/beacon/camera block when in tracker mode) are read by `config.cc` but never logged, so an operator can't verify them by reading the log. Same gap existed in 029 — flagging now because the 033 debugging session ran into "we'd see it if it were logged" multiple times. Scope: extend the startup banner in `src/autoc.cc` (the block after "Sigmas: …") to print every key in the active `AutocConfig`, mode-gated where appropriate. Companion: make the banner the single source of truth for "what config did this bake actually use" so it can be diffed across runs.
- [ ] T058 Right-size seed widths to match actual entropy (32-bit, not 64). The whole cascade carries `uint64_t` for `effectiveMasterSeed`, `gScenarioSeedTable[K]`, `ScenarioMetadata.scenarioSeed`, and `MasterPRNG.next()`, but the actual entropy is only 31 bits (Park-Miller modulus 2^31-1; `fold_to_pm_state` XOR-folds and masks the top bit). Two sharp edges this creates: (a) `config.h:46` `int seed = -1` silently truncates if an operator pastes back a `Seed = N > 2^31-1` value the log told them to copy — current wall-clock seeds (~1.78×10⁹) are just under that ceiling, will overflow within the next ~16 years; (b) `MasterPRNG.next()` returns uint64 with bits 31 and 63 always 0, so the ceremonial width is misleading. Two cleanup directions, pick one:
  - **Option A (preferred per operator)**: narrow `scenarioSeed` + `effectiveMasterSeed` + `MasterPRNG.next()` to `uint32`. Drop the fold (input is already correctly sized), keep the zero-sentinel guard. Match wire format, log format (`%08x`), and config field width to actual entropy. Bonus: cereal payload shrinks 8 bytes per scenarioMeta × 294 × per-gen.
  - **Option B**: widen `cfg.seed` to `int64_t` and replace `fold_to_pm_state` with a wider-state PRNG (xoshiro256, splitmix64) that genuinely uses 64-bit entropy. Bigger change; no benefit unless we plan to need >2 billion distinct replay keys.

  Default to Option A. Touches: `config.h:46`, `scenario_metadata.h:40`, `scenario_prng.h` (MasterPRNG state + next() return type), `protocol.h:376` (effectiveMasterSeed), `autoc.cc` startup log format, replay docs (`docs/variation-prng.md` from T023).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately on branch checkout
- **Foundational (Phase 2)**: Depends on Setup — BLOCKS both US1 and US2
- **US1 (Phase 3)**: Depends on Foundational — can run in parallel with US2 once Foundational completes, but the integration is cleaner if US1 lands first (PRNG is the bake-reproducibility substrate). NO intermediate bake stop after US1 per operator routing 2026-05-20
- **US2 (Phase 4)**: Depends on Foundational — can technically parallelize with US1 (different files); joint closeout requires both; **operator-driven bake happens HERE, not between US1 and US2**
- **US3 + US4 (Phase 5 + 6)**: DEFERRED to 033 phase 2 (gated on US2 mezzanine real-flight test pass — see spec §6 phased plan and quickstart.md)
- **Polish (Phase 7)**: Depends on US1 + US2 complete for T053; other tasks independent

### Critical Phase-1 Path

T001 → T002 → T003 → (T004-T008 in parallel) → US1 implementation (T009-T025) AND US2 implementation (T026-T042) in parallel → T053 closeout

### Within Each User Story

- TDD: T009-T013 (US1 tests) and T026-T028 (US2 tests) MUST FAIL before any implementation tasks run
- Models/types before services: T004-T008 must complete before T014+ or T029+
- Within US1: T014-T024 are largely sequential because they all touch `src/autoc.cc` + protocol.h or are sequential by data flow; T020-T021 (crrcsim mirrors) are [P]
- Within US2: T029-T031 (pure-math + applyStreak) are sequential (same file or trivial chain); T032-T039 (config, ini, stepper integration) split across files and are mostly [P]

### Parallel Opportunities

- All [P] foundational tasks (T005, T006, T007 + T008 as the .cc impl)
- All [P] test files (T009, T010, T011 across different test files)
- All [P] ini-edit tasks (T034, T035, T036)
- US1 implementation tasks where different files (T020, T021 crrcsim mirrors)
- US2 implementation tasks across config (T032), protocol (T033), ini (T034-T036) all [P]
- Polish [P] doc tasks (T051, T052)

---

## Parallel Example: Foundational Phase

```bash
# After T001-T003 setup, launch in parallel:
Task: "T005 [P] Add smoothnessPenaltyFloor + smoothnessMotionMode to AutocConfig in include/autoc/util/config.h"
Task: "T006 [P] Add scenarioSeed (uint64_t) field to ScenarioMetadata in include/autoc/rpc/scenario_metadata.h"
Task: "T007 [P] Create include/autoc/util/scenario_prng.h with MasterPRNG/ScenarioRootPRNG/ClassPRNG types"
Task: "T008 [P] Create src/util/scenario_prng.cc implementing T007 types"
```

## Parallel Example: US1 Test-Writing (TDD red-phase)

```bash
Task: "T009 [P] [US1] Create tests/scenario_prng_tests.cc with cases per contracts/scenario_prng_chain.md Validation tests section"
Task: "T010 [P] [US1] Extend tests/tracker_dmp_roundtrip_tests.cc with cases per contracts/dmp_schema_extension.md Validation tests"
Task: "T011 [P] [US1] Extend tests/contract_config_tests.cc with [Smoothness] section cases per contracts/ini_schema.md Validation tests"
```

## Parallel Example: US2 Config + Protocol + Ini Wiring

```bash
Task: "T032 [P] [US2] Parse [Smoothness] section in src/util/config.cc ConfigManager::load()"
Task: "T033 [P] [US2] Add smoothnessPenaltyFloor + smoothnessMotionMode fields to WorkerInit in include/autoc/rpc/protocol.h"
Task: "T034 [P] [US2] Add [Smoothness] section to autoc.ini"
Task: "T035 [P] [US2] Add identical [Smoothness] section to autoc-tracker.ini"
Task: "T036 [P] [US2] Add identical [Smoothness] section to autoc-tracker-minisim.ini"
```

---

## Implementation Strategy

### Phase-1 bake target (US1 + US2 in one pass — no intermediate bake stop)

Per operator routing 2026-05-20: do NOT run a separate PRNG-only validation bake between US1 and US2. PRNG determinism leaks will surface in the US2 bake or in on-the-spot eval-mode reruns; unit tests T009-T013 are the within-code safety net.

1. Complete Phase 1 + Phase 2 (Setup + Foundational)
2. Complete Phase 3 (US1) — PRNG architecture + dmp extension — unit tests green, no bake
3. Complete Phase 4 (US2) — multiplicative smoothness penalty integrated
4. Run all 033 ctest suites → ALL GREEN (T041 + T042)
5. **STOP at end of US2** — operator-triggered bake per [quickstart.md](./quickstart.md) with `SmoothnessPenaltyFloor=0.5` (phase-1 YOLO start per spec §6 step 2). Agent does NOT auto-launch
6. Phase-1 closeout per spec §2.B "Phase-1 success criteria" + Clarifications Q1 (continued learning + streak% ≥ 2/3 baseline + best-fitness in same range + materially-better per-axis aggressiveness)
7. PRNG determinism cross-check happens organically via the bake (cross-run scenario reproducibility) and any on-the-spot eval-mode replay the operator chooses to run
8. **Mezzanine real-flight test** — qualifier to phase 2

### Phase 2 (US3 + US4) — POST-mezzanine-flight-pass

1. Complete Phase 5 (US3 M2 inheritance) — automatic from US2's mode-agnostic implementation
2. Complete Phase 6 (US4 kamikaze penalty)
3. M2 phase-2 bake + real-flight validation

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable; US1 (PRNG-only) is the cleanest independent slice
- Verify TDD red-phase: tests must FAIL before implementation
- Per [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md): operator drives bake + rebuild-perf gate; agent does NOT auto-launch
- Per [feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md): NO `CEREAL_CLASS_VERSION` bump; append-at-end is the policy
- Per [feedback_submodule_merge_order](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_submodule_merge_order.md): crrcsim PR merges FIRST, autoc PR merges SECOND
- Commit after each task or logical group; stop at any checkpoint to validate story independently
- Avoid: vague tasks, same-file conflicts in [P] tasks, cross-story dependencies that break independence

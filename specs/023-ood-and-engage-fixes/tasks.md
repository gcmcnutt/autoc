---
description: "Task list for feature 023: OOD Coverage, Engage Transient, Throttle Discipline"
---

# Tasks: 023 — Remaining Work

**Updated**: 2026-04-16 (post-hb1-adjust4 eval suite pass)

## Completed Phases

The following are implemented, tested via training + eval suite, and committed:

- **Phase 0a** — Type-safe NNInputs struct at 33 fields, {33,32,16,3} topology, computeTargetDir() helper. All desktop producers (autoc, crrcsim, minisim) updated.
- **Phase 0b** — Train/eval dedup via buildEvalData(). Bugs 2-5 fixed. Eval determinism verified (bitwise match).
- **US1** — Direction cosines replace dPhi/dTheta. No atan2 in NN input pipeline.
- **US2 (desktop)** — resetHistory() at engage, 750ms delay config, history pre-fill.
- **FDM tuning** — hb1-adjust1 through adjust4 (6 tuning passes from flight data).
- **Eval suite** — eval_suite.sh updated, all tiers passing including random path generalization.

## P1 — Xiao Code Sync (flight blocker)

All tasks in this section must complete before flight test.

- [ ] T-XIAO-1 Update `xiao/src/msplink.cpp` direction cosines: replace `executeGetDPhi()`/`executeGetDTheta()` atan2 code (~lines 257-268) with `computeTargetDir()` populating `target_x/y/z` fields. Update `recordErrorHistory()` call to new `(targetDir_3vec, dist, timeMs)` signature.
- [ ] T-XIAO-2 Replace `clearHistory()` with `resetHistory()` at engage transition in `xiao/src/msplink.cpp`.
- [ ] T-XIAO-3 Update `XIAO_RABBIT_SPEED_MPS` from 13.0f to 12.0f in `xiao/src/msplink.cpp`.
- [ ] T-XIAO-4 Regenerate `xiao/src/generated/nn_program_generated.cpp` from hb1-adjust4 best weights via `tools/nn2cpp/`. Verify topology {33,32,16,3} and weight count 1667.
- [ ] T-XIAO-5 Build verify: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.
- [ ] T-XIAO-6 Walk pre-flight checklist (memory: project_preflight_checklist.md). Bench-test failsafe chain.

## P2 — Effort Lexicase (deferred, decision gate after flight)

Design complete. Implementation tasks T123-T136 defined below. Gate: implement only if flight data shows bang-bang control is unacceptable.

- [ ] T123 Create `StreakAccumulator` struct in `include/autoc/eval/fitness_computer.h`.
- [ ] T124 Add effort config fields to config.h/config.cc: `EffortStreakThreshold`, `EffortStreakRampSec`, `EffortStreakMultiplierMax`, `EffortLexicaseEpsilon`.
- [ ] T125 Add `effortCost`, `maxGentleStreak`, `totalGentleTicks` to `ScenarioScore`.
- [ ] T126 Compute `effortCost` in `computeScenarioScores()`. Formula: `effortScore = 1.0 - sqrt(pitch^2 + roll^2 + throttle^2) / sqrt(3)`.
- [ ] T127 Extend `lexicase_select()` with effort cascade (tracking filters first, then effort).
- [ ] T128 Add 2-dim lexicase tests in `tests/selection_tests.cc`.
- [ ] T129 Add effort computation tests in `tests/fitness_decomposition_tests.cc`.
- [ ] T130 Add effort config knobs to `autoc.ini` and `autoc-eval.ini`.
- [ ] T131 Add `effort` and `eftMul` columns to `data.dat`.
- [ ] T132 Add effort stats to `data.stc`.
- [ ] T133 Wire effort epsilon from config to `lexicase_select()` call site.
- [ ] T134 Build and test: verify `NN_ELITE_SAME` determinism holds.
- [ ] T135 Training run with effort lexicase (400 gens, pop 3500). Success: effort < 0.5, fitness within 80% of baseline.
- [ ] T136 Update `sim_polar_viz.py` to plot effort column.

## P3 — Test Coverage (post-flight)

- [ ] T-TEST-1 `tests/engage_reset_tests.cc` — 6 contract tests for resetHistory().
- [ ] T-TEST-2 `tests/engage_delay_tests.cc` — 3 contract tests for delay window.
- [ ] T-TEST-3 `tests/nn_inputs_tests.cc` — unit-vector invariant, poison-value completeness, sizeof contract.
- [ ] T-TEST-4 Extend `tests/selection_tests.cc` with 2-dim lexicase tests (blocked on P2).

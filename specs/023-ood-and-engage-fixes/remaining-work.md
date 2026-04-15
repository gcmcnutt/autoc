# 023 Remaining Work — Post-test7 Plan

**Status**: test7 running (sanity check, post-pt3 revert). Eval determinism fixed (float variationScale).
**Date**: 2026-04-13

## Priority Order

### P0 — Done

**1. Eval determinism fix (DONE)**
- `computeVariationScale()` returns `float`. Verified in test4.

### P1 — Control effort lexicase (flight-test gate)

**2. Implement effort streak in `computeScenarioScores()` (NEW)**
- Per-tick: `effort = sqrt(pitch² + roll² + throttle²)` — command magnitude, not delta
- Streak: `du < threshold` → multiplier ramps; violent tick → reset
- Per-scenario: `effortCost = sum(du[t] / multiplier[t])`, lower = better
- Skip first tick after engage transition
- Config: `EffortStreakThreshold=0.3`, `EffortStreakRampSec=3.0`, `EffortStreakMultiplierMax=3.0`

**3. Add `effortCost` to `ScenarioScore`**
- New field in `include/autoc/eval/fitness_decomposition.h`
- Computed in `src/eval/fitness_decomposition.cc`

**4. Update lexicase selection for 2 dimensions**
- `src/eval/selection.cc`: filter on tracking score (eps ~0.05), then effort (eps ~0.1)
- Tests: `tests/selection_tests.cc` — add 2-dim lexicase tests

**5. Update data.dat with per-tick delta**
- Add `du` column (3D control delta) for visualization
- Update `logEvalResults()` header and format

**6. Update data.stc with effort stats**
- Per-gen: `bestEffort`, `avgEffort`

**7. Config knobs in autoc.ini**
- `EffortStreakThreshold`, `EffortStreakRampSec`, `EffortStreakMultiplierMax`
- Parse in `src/util/config.cc`

**8. Training run with effort lexicase**
- 400 gens, pop 3500
- Compare fitness AND smoothness vs test4/test7 baseline
- Success: mean |du| < 0.5, smooth streak > 10 ticks, fitness within 80% of baseline

**9. Reduce rabbit speed (T122)**
- `RabbitSpeedNominal` 13→12 m/s, `RabbitSpeedMin` 10→9 m/s
- Reduces throttle saturation

### P2 — Xiao code sync (flight blocker)

**10. Xiao direction cosines (T016, T029)**
- `xiao/src/msplink.cpp`: update from dPhi/dTheta atan2 to 33-input direction cosines
- Use `computeTargetDir()` — same as CRRCSim/minisim

**11. Xiao resetHistory (T064)**
- Replace `clearHistory()` with `resetHistory()` at engage transition

**12. Xiao rabbit speed**
- Update `XIAO_RABBIT_SPEED_MPS` constant if we drop to 12 m/s

**13. Xiao nn2cpp regen (T031, T107)**
- Regenerate `nn_program_generated.cpp` from final training weights
- Verify topology matches {33, 32, 16, 3}

**14. Xiao build verify (T023)**
- `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`

### P3 — Test coverage (code works, needs tests)

**15. US2 engage delay tests (T061, T066)**
- `tests/engage_reset_tests.cc` — 6 contract tests for resetHistory()
- `tests/engage_delay_tests.cc` — 3 contract tests for delay window

**16. US1 verification (T054-T059)**
- Bearing continuity regression test
- Unit-vector invariant formal test

### P4 — Investigation

**17. Memory bloat (T121)**
- ~4.3MB/individual, scales linearly with pop
- heaptrack on short run to identify leak

## Moved Out

| Item | Destination |
|------|-------------|
| US3 Throttle lexicase | 024 (or absorbed by Change 9 effort) |
| US4 Discontinuity-forcing paths | BACKLOG |
| US5 Authority limit | 024 |
| US6 Craft variations | 024 |
| US2 Change 1c INAV bench work | Pre-flight hardware |
| pt3 RC filter | ABANDONED (see BACKLOG.md) |

## Decision Gate

After effort lexicase training run:
- **If smooth + good fitness** → xiao code sync → flight prep (T107-T113)
- **If still bang-bang** → escalate to 024 craft variations + authority limit

## Precision Rule

All values that round-trip through serialization must be `float`/`gp_scalar`, never `double`.

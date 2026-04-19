# 023 Remaining Work — Post-hb1-adjust4

**Status**: hb1-adjust4 training complete. Eval determinism verified. Eval suite passes (tier0 repro, tier1 novel seed, tier2 generalization including random paths, tier3 stress/quiet).
**Date**: 2026-04-16

## Done

1. **Direction cosines / 33-input topology** — NNInputs struct at 33 fields, {33,32,16,3} topology, computeTargetDir() helper, all desktop producers updated (autoc, crrcsim, minisim).
2. **Train/eval dedup** — buildEvalData() single canonical path, Bugs 2-5 fixed.
3. **Engage delay + history reset** — resetHistory() in AircraftState, 750ms engage delay config, CRRCSim history pre-fill.
4. **Eval determinism** — float variationScale serialization, verified bitwise match.
5. **FDM tuning** — hb1-adjust1 through adjust4 (CD_prof, F, CG_arm, CL_drop, Cl_da, Cl_b, Cn_b, Cn_p, Cn_r).
6. **Rabbit speed** — 12 m/s nominal, 9 m/s min.
7. **Eval suite** — eval_suite.sh updated for current log format, tier0-3 all passing.

## P1 — Xiao Code Sync (flight blocker)

**8. Xiao direction cosines**
- `xiao/src/msplink.cpp`: still uses dPhi/dTheta atan2 code (lines 257-268)
- Must switch to `computeTargetDir()` and populate `target_x/y/z` fields
- Must update `recordErrorHistory()` call signature to `(targetDir_3vec, dist, timeMs)`

**9. Xiao resetHistory**
- Replace `clearHistory()` with `resetHistory()` at engage transition

**10. Xiao rabbit speed constant**
- `XIAO_RABBIT_SPEED_MPS` is 13.0f, needs update to 12.0f

**11. Xiao nn2cpp regen** (may already be current)
- Verify `nn_program_generated.cpp` matches current training weights (not just topology)
- Regenerate from hb1-adjust4 best weights

**12. Xiao build verify**
- `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`

## P2 — Effort Lexicase (moved to 024)

Design complete (T123-T136). Moved to 024 feature — implement after flight data confirms whether bang-bang control is a real problem in the air.

## P3 — Test Coverage (optional before flight)

- `tests/engage_reset_tests.cc` — 6 contract tests for resetHistory()
- `tests/engage_delay_tests.cc` — 3 contract tests for delay window
- `tests/nn_inputs_tests.cc` — unit-vector invariant, poison-value completeness
- `tests/selection_tests.cc` — 2-dim lexicase tests (blocked on P2)

## Moved Out

| Item | Destination |
|------|-------------|
| US3 Throttle lexicase | 024 |
| Effort lexicase (T123-T136) | 024 |
| US4 Discontinuity-forcing paths | BACKLOG |
| US5 Authority limit | 024 |
| US6 Craft variations | 024 |
| US2 Change 1c INAV bench work | Pre-flight hardware |
| pt3 RC filter | ABANDONED |

## Precision Rule

All values that round-trip through serialization must be `float`/`gp_scalar`, never `double`.

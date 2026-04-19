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

## P1 — Xiao Code Sync (flight blocker) — DONE

All committed in `1189782 feat(023): xiao direction cosines, resetHistory, eval suite fixes, nn2cpp NNInputs API` (2026-04-16) and flight-tested on 2026-04-17.

- [x] T-XIAO-1 Update `xiao/src/msplink.cpp` direction cosines: replace `executeGetDPhi()`/`executeGetDTheta()` atan2 code with `computeTargetDir()` populating `target_x/y/z` fields. Update `recordErrorHistory()` call to new `(targetDir_3vec, dist, timeMs)` signature.
- [x] T-XIAO-2 Replace `clearHistory()` with `resetHistory()` at engage transition in `xiao/src/msplink.cpp`.
- [x] T-XIAO-3 Update `XIAO_RABBIT_SPEED_MPS` from 13.0f to 12.0f in `xiao/src/msplink.cpp`.
- [x] T-XIAO-4 Regenerate `xiao/src/generated/nn_program_generated.cpp` from hb1-adjust4 best weights via `tools/nn2cpp/`. Verify topology {33,32,16,3} and weight count 1667.
- [x] T-XIAO-5 Build verify: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.
- [x] T-XIAO-6 Walk pre-flight checklist. Bench-test failsafe chain.

## Flight-20260417 Postflight — Diagnostic Work

Did not land as tasks at feature start; surfaced during/after flight-20260417. Committed scripts and findings:

- Command→response scatter (sim + flight, 3×3 and 8-row lag sweep). Sim script in `specs/023-ood-and-engage-fixes/cmd_response_scatter_sim_lagged.py` and `cmd_response_lagsweep_sim.py`. Flight script in `flight-results/flight-20260417/cmd_response_scatter_lagged.py` and `cmd_response_lagsweep.py`.
- Quat ordering audit across INAV → blackbox-tools → xiao → cereal → scripts → renderer: **clean**, scalar-first everywhere.
- NN output polarity audit (pitch / roll): **clean**, strong positive cmd→gyro correlation at proper lag in flight.
- Cadence finding: data.dat at 117 ms/step instead of 100 ms. Root-caused to CRRCSim physics tick vs strict-greater eval threshold.
- Flight AHRS roll anomaly: quat-derived roll rate correlation is negative at all lags 0–707 ms while gyro-derived is +0.76. Not a simple lag or sign flip.

All above **moved to 024** for resolution:
- Cadence fix → 024 WI1
- Compound-attitude bench → 024 WI3
- Full-sensor cmd→response audit → 024 WI4
- AHRS roll root cause → 024 WI5

## Moved Out

| Item | Destination |
|------|-------------|
| P2 Effort Lexicase (T123-T136) | 025 (Change 5 Option A — smoothness pressure) |
| P3 T-TEST-1/2/3 (engage_reset, engage_delay, nn_inputs) | 024 WI12 |
| P3 T-TEST-4 (2-dim lexicase) | 025 (with effort lexicase) |
| Cadence 117ms bug | 024 WI1 |
| Compound-attitude bench | 024 WI3 |
| Full-sensor cmd→response audit | 024 WI4 |
| Flight AHRS roll divergence | 024 WI5 |

## 023 Status: CLOSED

All in-scope work done. Remaining items have new owners (024 or 025). Flight-20260417 is the closing checkpoint; its findings seeded 024.

# 023 Remaining Work — Post-test3 Plan

**Status**: test3 complete (gen 260, best -21130). Eval determinism bug found and fixed.
**Date**: 2026-04-12

## Priority Order

### P0 — Must fix before next training run

**1. Eval determinism fix (DONE)**
- Root cause: `computeVariationScale()` returned `double` in training but was
  stored as `float` in NN01 weights. Eval read back the `float` value. The
  `double→float` truncation of 6/9 = 0.6667 produced a ~2e-8 delta that
  propagated through `rabbitSpeedConfig.sigma` and `populateVariationOffsets()`,
  changing rabbit speed profiles and entry variation offsets just enough to
  cause per-scenario FP divergence that cascaded over 100+ ticks.
- Fix: changed `computeVariationScale()` return type from `double` to `float`.
  All call sites now see float precision. Training and eval produce bit-identical
  variation scaling. Rule: **all values that round-trip through serialization
  (NN01 weights, RPC) must be float/gp_scalar, never double.**

**2. Memory bloat (T121)**
- Heap 27-42GB, slowly growing gen-over-gen
- **NOT arena bloat** — `MALLOC_ARENA_MAX=4` tested in test4, RSS still
  15GB at pop 3500 (test3 was ~40GB at pop 10k). Scales linearly with
  population = per-individual allocation not being freed.
  Expected ~2GB, actual ~4.3GB per 1k individuals = ~4.3MB per individual.
  `EvalData` is ~176KB, `scenario_scores` is ~14KB = 190KB per individual.
  4.3MB/190KB = ~22x overhead. Something per-individual is leaking.
- Suspect: `shared_ptr<EvalData>` lambda captures in thread pool, or
  cereal serialization buffers, or `EvalResults` copies not being released.
- Next step: heaptrack on a short run (10 gens, pop 1000) to identify
  the allocation call stack.

### P1 — RC smoothing filter experiment (flight-test gate)

**3. Implement pt3 filter in CRRCSim (T114)**
- INAV pt3 algorithm, runs at 333Hz FDM rate
- All 3 channels (pitch, roll, throttle)
- `AUTOC_RC_FILTER_HZ` env var, default 20Hz
- Reset filter state at scenario start, pre-fill with engage delay commands

**4. Add filtered command fields to AircraftState (T116)**
- `filteredPitchCommand_`, `filteredRollCommand_`, `filteredThrottleCommand_`
- Capture what FDM surfaces actually see (post-filter)

**5. Add filtered columns to data.dat (T115)**
- `fltPt`, `fltRl`, `fltTh` alongside existing `outPt`, `outRl`, `outTh`
- Update `logEvalResults()` header and format string

**6. Update sim_polar_viz.py (T119)**
- Plot filtered vs raw NN output
- Show phase delay and smoothing effect

**7. Update INAV config (T117)**
- `set rc_filter_lpf_hz = 20`
- `set rc_filter_auto = OFF`
- `set servo_lpf_hz = 0` (leave disabled)

**8. Reduce rabbit speed (T122)**
- `RabbitSpeedNominal` 13→12 m/s
- `RabbitSpeedMin` 10→9 m/s
- Reduces throttle saturation (currently 72% at full throttle)

**9. Frequency tuning runs (T118)**
- Short runs (50 gens, pop 3500) at 15Hz, 20Hz, 30Hz
- Compare smoothness vs fitness vs phase delay

**10. Full training run with filter (T120)**
- 400 gens, pop 3500 (~2x params, best practice for GA with lexicase + deterministic eval)
- `MALLOC_ARENA_MAX=4` to prevent glibc arena bloat
- Success criteria: mean |filtered command| < 0.6, fitness within 80% of test3
- pt3 filter deterministic by construction
- Verify eval determinism (NN_EVAL_SAME) with the float variation_scale fix

### P2 — Code sync (lock down engage delay)

**11. US2 engage delay tests (T061, T066)**
- Write `tests/engage_reset_tests.cc` — 6 contract tests for resetHistory()
- Write `tests/engage_delay_tests.cc` — 3 contract tests for delay window
- Code implemented and working in test3, just needs test coverage

### P3 — US1 verification (mostly done)

**12. US1 wrap-free bearing (T054-T059)**
- T054: atan2 grep — verified clean
- T055: bearing continuity regression test — write
- T058-T059: unit-vector invariant — verified in viz, write formal test

## Not in 023 (moved)

| Item | Destination |
|------|-------------|
| US3 Throttle lexicase | 024 |
| US4 Discontinuity-forcing paths | BACKLOG |
| US5 Authority limit | 024 (with craft variations) |
| US6 Craft variations | 024 |
| US2 Change 1c INAV bench work (T071-T076) | Pre-flight hardware work |

## Decision Gate

After T120 completes:
- **If smooth + good fitness** → proceed to flight prep (T107-T113)
- **If still bang-bang** → escalate to 024 craft variations + authority limit

## Precision Rule (from this debugging session)

**All values that participate in the training→eval round-trip must be `float`
(or `gp_scalar`), never `double`.** This includes:
- `computeVariationScale()` — returns float
- Any value stored in NN01 weights (variation_scale, mutation_sigma, fitness)
- Any value that feeds scenario construction on both training and eval paths
- The NN forward pass is already all-float (NNInputs, weights, outputs)

Double precision is fine for:
- Fitness aggregation (summing scores — intermediate precision, not serialized)
- FDM internal computation (CRRCSim uses double throughout)
- Logging and display

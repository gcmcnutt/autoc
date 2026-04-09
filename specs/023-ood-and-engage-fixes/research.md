# Research — 023 OOD and Engage Fixes

**Feature**: 023-ood-and-engage-fixes
**Phase**: 0 (Research, complete)
**Date**: 2026-04-08

Index document. Captures the decisions locked during `/speckit.clarify`, points
at the deep research artifacts, and summarizes what each found in enough detail
that downstream `/speckit.tasks` can work without re-reading the full documents.

## Decisions locked during clarify session (2026-04-08)

| # | Question | Decision | Rationale |
|---|---|---|---|
| Q1 | Engage transient fix approach | Pre-fill all history slots with current in-autoc geometry on every engage transition | First NN call sees a flat history matching reality, closing rate zero. No "first engage special case" — same reset every time. |
| Q2 | INAV engage delay modeling | Simulate in sim pipeline with centered stick during the ~0.75s window | Matches user's real-flight handoff technique. NN runs every tick and fills history naturally; only outputs are ignored. |
| Q3 | Type-safe NN sensor interface | Struct-of-floats with named fields + `static_assert` on layout | Strongest compile-time safety. `reinterpret_cast<const float*>` preserves inner-loop performance. Catches the 021 silent-corruption failure mode structurally. |
| Q4 | Change 8 authority-limit trigger criterion | Deferred to empirical observation during Phase 3b | The 3-cosine representation is the primary intervention; authority limit is a conditional follow-up whose threshold should come from observed baseline data, not guessed. |
| Q5 | Streak threshold ramp | Leave in BACKLOG; not in 023 | 022 verdict holds — betterz2 converged without it. Revisit only if Phase 3a shows an early-generation "no streak signal" problem under the new representation. |

All five decisions are integrated into spec.md Clarifications section and the
corresponding Change / Prerequisite sections.

## Research artifacts

### Deep research (written by background agents during plan phase)

| Artifact | Location | Purpose | Durability |
|---|---|---|---|
| INAV signal path audit | `docs/inav-signal-path-audit.md` | Every filter/delay/latency stage in the MANUAL-mode NN signal path; MSP2 latency measurement; 750 ms delay root cause investigation; escape-hatch architectures for 024+ | **Durable** — may inform 024+ architectural decisions beyond 023's scope |
| Train/eval code dedup survey | `specs/023-ood-and-engage-fixes/train-eval-code-dedup.md` | Side-by-side of `runNNEvaluation()` vs training path; 6 divergences found; refactor proposal | **One-off** — becomes obsolete once the Phase 0b refactor lands |

### Decision support (pre-existing)

- `specs/023-ood-and-engage-fixes/spec.md` — feature specification, Clarifications section
- `flight-results/flight-20260407/flight-report.md` — 2026-04-07 flight analysis (problem statement)
- `docs/COORDINATE_CONVENTIONS.md` — virtual/raw/body frame conventions (022 heritage)
- `docs/sensor-pipeline.md` — existing NN input pipeline documentation

## INAV signal path audit — key findings

Full audit at `docs/inav-signal-path-audit.md`. Distilled highlights:

### Delay sources in the NN signal path

**Gyro input (raw → xiao) — where the real-flight NN gets its body rates:**

| Stage | File | Current config | Delay | Recommended |
|---|---|---|---|---|
| Hardware anti-alias | `sensors/gyro.c:259-262` | Default | ~0 ms | unchanged |
| **`gyro_main_lpf_hz`** | `sensors/gyro.c:485` | **25 Hz PT1** | **~6.4 ms** | **60 Hz (default)** |
| **Gyro Kalman (`setpoint_kalman_enabled`)** | `sensors/gyro.c:506-510` | **ON, q=100** | state-dependent | **OFF** |
| Static notch | `sensors/gyro.c` | Default | ~0 ms | unchanged |
| Dynamic notch | `sensors/gyro.c` | ON | state-dependent | **OFF for fixed-wing** |
| MSP handler | `fc/fc_msp.c:709-711` | `MSP2_INAV_LOCAL_STATE` | 0 | unchanged |

**Key correction to prior spec language**: `MSP2_GYRO` does NOT exist as a
standalone endpoint. Gyro data is piggybacked on `MSP2_INAV_LOCAL_STATE`, the
same endpoint the xiao already fetches for attitude/position. This means the
xiao has been receiving gyro data through the full INAV filter stack, not raw.

**Servo output (xiao MSP → servo):**

| Stage | File | Current config | Delay | Recommended |
|---|---|---|---|---|
| UART RX + TASK_SERIAL | `fc/fc_tasks.c:488-493` | 100 Hz LOW priority | up to 10 ms ingress jitter | unchanged (architectural) |
| MSP handler | `fc/fc_msp.c:2068-2080` | Synchronous | 0 ms | unchanged |
| TASK_RX event-driven | `rx/msp_override.c:124-146` | Event-driven | <1 ms | unchanged |
| PID tick | `fc/fc_core.c:885-989` | 2 kHz | 500 µs | unchanged |
| `processPilotAndFailSafeActions` MANUAL | `fc/fc_core.c:389-436` | Pass-through | 0 ms | unchanged |
| **`rcInterpolationApply` (PT3 @ `rc_filter_lpf_hz`)** | `fc/rc_smoothing.c:143-145` | **250 Hz, NOT gated by MANUAL** | **~1.9 ms** | **0 (disabled)** |
| `mixTable` MANUAL passthrough | `flight/mixer.c:520-525` | Direct | 0 ms | unchanged |
| `servoMixer` MANUAL | `flight/servos.c:282-285` | Direct | 0 ms | unchanged |
| Rate limiter | `flight/servos.c:427` | `speed=0` (disabled) | 0 ms | unchanged |
| `filterServos` biquad | `flight/servos.c:230-251` | `servo_lpf_hz=0` (disabled) | 0 ms | unchanged ✓ |
| PWM output | `servo_pwm_rate=50 Hz` | — | up to 20 ms quantization | unchanged (unavoidable for analog servos) |

### Measured MSP2 round-trip latency

From `flight-results/flight-20260407/flight_log_2026-04-08T02-16-40.txt` and
`flight_log_2026-04-08T02-22-51.txt`:
- **Median**: ~23 ms (xiao-observed fetch+eval+send)
- **Max**: 42 ms
- **Min**: 15.5 ms
- Plus 0-15 ms INAV-internal post-receive latency (TASK_SERIAL → TASK_RX → TASK_PID scheduling)
- Plus up to 20 ms PWM output quantization (50 Hz servo rate)
- **End-to-end**: ~25 ms typical / ~55 ms worst case

This is the number that feeds success criterion validation and the
`EngageDelayMs` config default (though the engage delay is a separate
mechanism — see below).

### 750 ms engage delay — PROBABLE root cause

The flight report documents a ~750 ms delay between xiao enabling
`MSPRCOVERRIDE` and INAV reaching `MANUAL` mode. The audit traced two
candidate causes:

**Candidate A (PROBABLE, unconfirmed)**: `failsafe_recovery_delay = 5` ×
100 ms + `PERIOD_RXDATA_RECOVERY` in
`src/main/rx/msp_override.c:103-104, 201-211`. The failsafe state machine
requires `rxDataRecoveryPeriod` of valid frames before clearing the
failsafe flag. Math could plausibly add up to 750 ms. The audit agent did
NOT trace the `PERIOD_RXDATA_RECOVERY` constant through the source (time
budget), so this is marked PROBABLE but unconfirmed.

**Candidate B**: xiao-side delay between enabling `MSPRCOVERRIDE` AUX and
enabling `MANUAL` AUX. Xiao code was out of scope for the audit.

**Test to confirm/refute**: Phase 1 bench test — set
`failsafe_recovery_delay = 0`, re-measure engage delay on bench.
- If delay drops: confirmed, candidate A is the root cause, keep
  `failsafe_recovery_delay = 0` in production.
- If delay persists: candidate B, xiao-side; investigate xiao startup
  sequence in `xiao/src/msplink.cpp`.

This is a Phase 1 work item, NOT a spec clarification. Tracked in the
Implementation Order under Phase 1 bench work.

### INAV config-only recommendations (Change 1c in spec)

The audit concluded that all reducible delay can be eliminated via
config changes to `xiao/inav-hb1.cfg`, no INAV code changes required:

1. `gyro_main_lpf_hz`: 25 → 60 (restore default)
2. `setpoint_kalman_enabled`: ON → OFF
3. `rc_filter_lpf_hz`: 250 → 0
4. `dynamic_gyro_notch_enabled`: ON → OFF (for fixed-wing)
5. `failsafe_recovery_delay`: 5 → 0 (bench test; production change only if
   confirmed)

Applied one at a time with bench measurement between each step. Expected
cumulative improvement: ~6-10 ms removed from NN input path + ~2 ms from
servo output path + potentially ~750 ms engage delay gone.

### Escape-hatch architectures (NOT 023 scope)

The audit documented three 024+ candidate architectures in case 023's config
changes plus other improvements are insufficient:

- **Option A**: Direct xiao → servo PWM/PPM bypassing INAV's control loop
- **Option B**: Drop GPS from NN input path during autoc spans, dead-reckon
- **Option C**: Higher-rate NN on xiao with LSM6DS3 IMU-only inputs

Not deliverables for 023. Parked in the audit doc for future reference.

## Train/eval code dedup survey — key findings

Full survey at `specs/023-ood-and-engage-fixes/train-eval-code-dedup.md`.
Distilled highlights:

### Six divergences between `runNNEvaluation()` and the training path

| Bug | Status | Divergence | Fix |
|---|---|---|---|
| Bug 1 (metric drift) | Already fixed | Both paths call `computeScenarioScores` + `aggregateRawFitness` | No action |
| **Bug 2** (stale S3 fitness) | CONFIRMED | Eval copies original NN01 bytes into `evalResults.gp` at `src/autoc.cc:823-824` without rewriting `genome.fitness`. Renderer reads training-time fitness. | Assignment before `nn_serialize` |
| **Bug 3** (rabbit speed config) | CONFIRMED | `src/autoc.cc:751` never assigns `evalData.rabbitSpeedConfig`; inherits `{16.0, 0.0, ...}` default. Training sets it at `L939-940` and `L1028-1029`. | Fixed by `buildEvalData()` helper |
| **Bug 4 NEW** (`isEliteReeval` lies) | CONFIRMED | Eval hard-codes `isEliteReeval = false` and `enableDeterministicLogging = false`, but eval semantically IS an elite re-eval. Symptom: eval-mode `data.dat` may be missing deterministic logging fields. | Pass `purpose = EliteReeval` through helper |
| **Bug 5 NEW** (scenario asymmetry) | CONFIRMED | Eval hard-codes `scenarioForIndex(0)` at `L748`. In demetic mode, silently drops N-1 path variants. | Iterate all scenarios, match training aggregation |
| **Bug 6 NEW** (triple duplication) | CONFIRMED | Three near-identical copies of scenario-list populator: per-individual L932-975, elite re-eval L1021-1060, eval L751-787. Root cause of the whole bug class. | Extract `buildEvalData(EvalJob)`, delete all three copies |

### Proposed refactor: `buildEvalData(EvalJob)`

```cpp
enum class EvalPurpose {
    Training,        // Per-individual fitness during evolution
    EliteReeval,     // Best-of-generation deterministic re-run
    StandaloneEval,  // evaluateMode=1 one-shot weight evaluation
};

struct EvalJob {
    const ScenarioDescriptor& scenario;
    const std::vector<char>& nnBytes;
    EvalPurpose purpose;
};

EvalData buildEvalData(const EvalJob& job);
```

The helper reads `gRabbitSpeedConfig`, `computeVariationScale()`,
`globalScenarioCounter`, `gScenarioVariations`, and calls
`populateVariationOffsets` internally. No caller needs to remember them.

### Migration order (8 steps from survey)

1. Extract helper without behavior change (identical code path, 3 callers → 1 function)
2. Route training elite re-eval through helper (delete Copy B)
3. Route `runNNEvaluation()` through helper (delete Copy C, fix Bug 3 as side effect)
4. Fix Bug 2 inside eval path (assignment before serialize)
5. Fix Bug 5 (iterate all scenarios in eval mode)
6. Fix Bug 4 (set `purpose = EliteReeval` in eval)
7. Add determinism check in eval mode (mirror training's `bitwiseEqual`)
8. Verify end-to-end

### Acceptance criterion

The refactor is correct iff ALL of the following hold:

1. **Reproduces betterz2 gen-400 fitness.** Loading saved betterz2 weights
   (stored NN01 fitness = −34771) via eval mode with matching config
   reproduces the training-time fitness within FP rounding.
2. **Bitwise identical `ScenarioScore`** for a given (weight, scenario) pair
   when run through training vs eval path.
3. **No recurrence by construction.** Bugs 1-6 cannot recur because the
   divergent code no longer exists — one helper, one code path.
4. **Determinism check in eval mode** logs "SAME" when eval runs against a
   weight file saved during training with identical seed + config.

This is the first time eval fitness on saved weights will have been
correct since the codebase existed (Bug 3 alone — constant-16 rabbit
instead of configured 13±2 — has corrupted every eval-mode run).

### Biggest migration risk

The elite re-eval block has side effects the per-individual loop does NOT:
- Calls `logEvalResults(fout, bestResults)` into `data.dat`
- Uploads to S3 as `gen<N>.dmp`

If the `buildEvalData()` helper accidentally merges these into itself,
training will start writing `data.dat` once per individual (wrong) and
uploading to S3 once per individual (very wrong). **The helper's sole job
is `EvalData` construction. Logging and S3 upload stay in the caller.**

## Technical unknowns resolved

All Technical Context `NEEDS CLARIFICATION` items from plan.md are now
resolved:

- **INAV filter inventory** — resolved by audit (§NN-Output → Servo Path,
  §Raw Gyro → MSP2_INAV_LOCAL_STATE)
- **MSP2 round-trip latency bound** — resolved: median 23 ms, max 42 ms
- **Train/eval divergence set** — resolved: 6 divergences enumerated
- **Refactor shape for Phase 0b** — resolved: `buildEvalData(EvalJob)`
  helper, 8-step migration order
- **750 ms delay root cause** — PROBABLE (`failsafe_recovery_delay`),
  needs single bench test to confirm
- **Authority-limit trigger criterion** — DEFERRED to Phase 3b empirical
  observation per Q4 clarify decision

## Items still "unknown" (tracked in audit doc's §Sections marked "unknown")

1. Exact `PERIOD_RXDATA_RECOVERY` constant value (test via bench, not audit)
2. `FEATURE_THR_VBAT_COMP` active status in feature bitmask (negligible for 023)
3. Adaptive Kalman / dynamic notch exact group delay (state-dependent, irrelevant once they're disabled)
4. Exact INAV-internal post-receive delay from `MSP_SET_RAW_RC` → servo wire (bounded by audit, not directly measured)
5. Whether stage-1 anti-alias filter runs at 8 kHz or 2 kHz (depends on HW driver behavior, negligible for 023)

All five are below the action threshold for 023. They don't block any
implementation decision. If a later iteration cares, the audit doc has the
specific file:line references to trace them.

## Next steps

Proceed to `/speckit.tasks` to generate the actionable task breakdown from
this research + plan + spec + data-model + contracts. Tasks should map 1:1
to the migration orders in this research document (Phase 0b 8-step refactor,
Phase 0a 3-step type-safe refactor) plus the contract verification tests.

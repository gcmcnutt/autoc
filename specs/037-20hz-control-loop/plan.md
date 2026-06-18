# Implementation Plan: Faster Control Loop (10 Hz → harmonic ~20 Hz)

**Branch**: `037-20hz-control-loop` | **Date**: 2026-06-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/037-20hz-control-loop/spec.md`

## Summary

Raise the NN control-loop cadence from 10 Hz to a faster, harmonically-chosen rate (target
neighborhood ~20 Hz; exact rate decided by the Phase-0 clock/transport research) to reduce roll
bang-bang (a control-rate aliasing/dither artifact, not an objective or actuation problem) while
holding the tracking goal. Execution is phased so the **cheap sim decision comes first** and embedded
firmware only follows if justified:

- **Phase 0 (research, gating):** the **smoothing theory** (first-principles prediction of whether faster
  cadence smooths the command or just relocates bang-bang to the new Nyquist), the clock/frequency survey
  → **projected NN loop rate from candidate hardware**, camera frame-rate grid + AGC budget,
  serial-transport ceiling (115 kbaud is a soft limit), and an on-target NN-eval cycle-count harness.
  Co-resolved with 031's hardware shortlist. Emits the **projected rate + the derived training-config
  bundle + eval budget**.
- **Phase A (sim, the go/no-go):** retrain M1 at the **projected rate with the derived bundle** (cadence +
  latency/jitter model + history-bucket sizing + FDM rate — these move together, not independently);
  rescale all tick-denominated training math; settle the history time-basis; **confirm the smoothing
  theory's prediction** against the de-alias gate vs the historical t6 10 Hz baseline. The cadence is
  projected from hardware, NOT a round 20 Hz; a (B)-relay theory verdict is a cheap no-go before any bake.
- **Phase A M2 (US1b, gated):** if the M1 retrain shows a sensible higher-rate signal, retrain M2
  (tracker) at the same projected cadence on the M2 layout/rescale that landed with M1 — gated-in-scope,
  not committed. (The short-source fix is prework precisely because it gates this clean M2 bake.)
- **Phase B (embedded ~20 Hz):** local-IMU fast loop + INAV slow sync, three rate tiers, packed
  logging, INAV 20 Hz serve. Gated on Phase A clearing the gate.
- **Phase C (50 Hz stretch):** NN unroll + fast-tanh, real-time slot scheduling, tail-bounding. Gated on
  Phase A showing 20 Hz insufficient.

Per the operator decision (2026-06-09) this plan covers **all three phases**, with B/C planned at
decision-point granularity and explicitly conditioned on Phase A's result.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), C++ (xiao / PlatformIO arduino-mbed), Python 3.11 (analysis)
**Primary Dependencies**: Eigen (vec3/quat), cereal (NN + EvalResults + dmp), inih (ini), GoogleTest,
CRRCSim LaRCSim FDM; embedded: nRF52840 (Cortex-M4F), LSM6DS3 (to be brought up), INAV MSP over UART
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen aggregate), S3 `.dmp`
(cereal `EvalResults`, per-mode buckets `autoc-m1`/`autoc-m2`); `autoc.ini` / `autoc-tracker.ini`;
`autoc_config.xml` (FDM dt + video fps); xiao QSPI flash log
**Testing**: GoogleTest (unit/contract — cadence-invariance + tick-rescale tests), the FP-deterministic
bit-replay regression gate (`rebuild-perf.sh` + autoc-eval), on-target bench (DWT cycle-count, MSP
latency), sim↔real cross-check
**Target Platform**: Linux desktop (train: autoc + crrcsim); Seeed XIAO BLE Sense / nRF52840 (deploy);
INAV on MATEKF722MINI (FC)
**Project Type**: Multi-component embedded-control + evolutionary-training system (three components:
autoc evolution, crrcsim FDM, xiao firmware)
**Performance Goals**:
- Phase A go/no-go gate: roll lag-1 autocorr negative→≥0 AND sign-flip −≥20 pts (t6 56%→≤36%) at
  tracking within noise of historical t6 10 Hz (spec Clarifications Q1/Q3).
- Phase B: sustain a fresh ~20 Hz control tick on real hardware (read+write within the link budget,
  control-critical path bounded).
- Phase C (if reached): 50 Hz (20 ms tick) with eval ≤ a defended budget and tail-bounded jitter.
**Constraints**:
- Absolute FP determinism for the bit-replay gate (no `Date.now`/random in eval path).
- Sim↔real cadence parity is a prerequisite for a fair flight test.
- fp32 in the NN path for sim↔real parity (no fixed-point).
- 115 kbaud is the current (soft) link budget: read 12.6 ms avg / write 9.2 ms avg.
**Scale/Scope**: M1 = 33→32→16r→3 (1923 weights); M2 = 54→32→16r→3 (2595 weights); 294 scenario slots
(paths × winds); FDM 200 Hz today (raise for fast arms).

### Resolved cadence touchpoints (from Phase-0 grounding scan, 2026-06-09)

| Concern | Location | Behavior at rate change |
|---|---|---|
| Sim control interval | `gEvalUpdateIntervalMsec` (default 100) — `crrcsim/.../inputdev_autoc.h:53`, `.cpp:281,293-324,372`; env `AUTOC_EVAL_INTERVAL_MSEC` | drives `framesPerEval`; startup asserts `cycleLengthMs | evalIntervalMsec` |
| Autoc eval interval | `SIM_TIME_STEP_MSEC = 100` — `include/autoc/eval/aircraft_state.h:39-40` | master constant many terms key off; **must move in lockstep with `gEvalUpdateIntervalMsec`** |
| FDM dt / outer fps | `crrcsim/autoc_config.xml:157` (`dt=0.005`=200 Hz), `:3` (`fps=20`=50 ms) | raise dt for the 50 Hz arm (≥500 Hz; 2 kHz matches INAV) |
| Streak ramp | `streakStepsToMax = fitStreakRampSec / (SIM_TIME_STEP_MSEC/1000)` — `src/eval/fitness_decomposition.cc:33-34` | **auto-rescales** via `SIM_TIME_STEP_MSEC` ✅ |
| Stability/energy accum | `src/eval/fitness_decomposition.cc:179-180` (Σ per tick, unnormalized) | **scales ~linearly with tick count — needs normalization/rescale** ⚠️ |
| Closing/closure rate | `src/nn/evaluator.cc:310-334` (`/0.1f`), `fitness_decomposition.cc:206-223` (`dt=SIM_TIME_STEP_MSEC/1000`) | hardcoded `0.1f` is **wrong at new rate — must use dt** ⚠️ |
| Pathgen history | `include/autoc/nn/nn_inputs.h:36-58` (33 inputs, 6 slots), `evaluator.cc:312` (`HIST_PAST[]`) | tick-indexed → time-basis change alters `NN_INPUT_COUNT` |
| Tracker history | `include/autoc/nn/evaluator.h:112-127` (`span[6]`), `nn_inputs.h:136-240` (54 inputs), `tracker_stepper.cc:121-181` | tick-indexed; same |
| Error history ring | `aircraft_state.h:330` (`HISTORY_SIZE=10` = "1s @ 10Hz") | hardcoded slot count |
| Engage delay | spec `023/contracts/engage_delay.md` (`ceil(ms/step)`) — **not yet in code** | designed rate-independent; implement so |
| Entry sigmas | `include/autoc/util/config.h:59-66` (`entryConeSigma=30`, `entrySpeedSigma=0.1`), `scenario_prng.h:49` (`kGaussianSigmaClamp=2.5`); apply `variation_generator.h:286-318` (note existing hard cap `kEntryConeMaxRad=80°`) | P1 lowers sigmas to 18° / 0.05–0.06 |
| Xiao cadence | `xiao/include/main.h:24-25` (`MSP_LOOP_INTERVAL_MSEC=50`, `MSP_NN_EVAL_DIVISOR=2`) | already 20 Hz outer/send, 10 Hz NN → set divisor=1 for 20 Hz NN |
| Xiao link | `xiao/src/msplink.cpp:342` (`Serial1.begin(115200)`), `:276-308` (eval), `:364`/`:669-678` (fetch/send) | baud is a soft limit |
| Xiao IMU | none (no LSM6DS3/TWIM) — confirmed | Phase B brings it up |
| Xiao NN | `xiao/src/generated/nn_program_generated.cpp` → `nn_forward_recurrent` (library, looped) | Phase C unrolls |

**NEEDS CLARIFICATION (→ Phase 0 research, all have defined deliverables in spec).**
*Status 2026-06-10: 0/1/5/6 RESOLVED (research.md); 2/3/4 remain open and gate Phase B/C, not Phase A.*
0. **Will a faster cadence smooth or just relocate bang-bang?** First-principles theory (τ_roll vs T; raw
   pre-tanh drive; command-vs-motion) predicting (A) smooths or (B) relay-persists. → research.md RT.
   GATES the bake. **RESOLVED: GO, roll = case A (aliasing-dither).**
1. The **projected** control rate from candidate-hardware capability (camera-grid + transport-bounded +
   responsiveness), and its derived config **bundle**. → research.md R1/R2/R3. **RESOLVED: 20 Hz for the
   Phase-A sim arm (operator 2026-06-10); R2/R3 stay open for the Phase-B flown rate.**
2. Camera reachable fps grid + exposure/AGC budget. → R2. *(open — Phase B/031 co-resolve)*
3. Serial transport decision (baud bump vs SPI) + resulting read/write budget. → R3. *(open — Phase B)*
4. Measured unrolled-NN eval budget (cycles/µs). → R4. *(open — Phase C scope)*
5. History time-basis: log-spaced vs more-slots vs time-resampled, and the chosen N. → R5.
   **RESOLVED: ms-based log-spaced lags {1600,800,400,200,100,0}, slot count unchanged (33/54).**
6. Whether `gEvalUpdateIntervalMsec`/`SIM_TIME_STEP_MSEC` should be unified and moved from env-var to
   `.ini`/CLI per `feedback_cli_over_env_vars`. → R6. **RESOLVED: ControlIntervalMsec ini key (T001);
   in-struct default tracks SIM_TIME_STEP_MSEC (T017).**

## Constitution Check

*GATE: must pass before Phase 0. Re-checked after Phase 1.*

| Principle | Status | Notes |
|---|---|---|
| I. Testing-First | PASS (with required work) | New tests: cadence-invariance (per-tick metric semantics under dt), tick-rescale unit tests for stability/energy/closure, history-time-basis assembly. Hardware bring-up (Phase B/C) is research-spike-exempt until promoted. |
| II. Build Stability | PASS | autoc+crrcsim `rebuild.sh`; xiao `pio run`. |
| III. No Compatibility Shims | PASS | Cadence constant change is clean-cut; update all callers of `SIM_TIME_STEP_MSEC` / hardcoded `0.1f`. No dual-rate shims. |
| IV. Unified Build | PASS | No new top-level deps for Phase A. Phase B may add an LSM6DS3 driver under `xiao/` (PlatformIO lib, not top CMake). Any CMakeLists change → operator-driven `rebuild-perf.sh`. |
| V. Versioned Persistence | **ATTENTION** | If history rep changes (slot count/time basis), `NN_INPUT_COUNT` (M1) / tracker 54 and the dmp `EvalResults`/NN-layout change. Greenfield (no cereal version bump per `feedback_no_cereal_versioning`) but reader MUST fail loud on layout mismatch. Old t6 dmps become unreadable by the new layout — acceptable (compare via recorded metrics, per Q3). |
| VI. Type-Domain Discipline | PASS (with audit) | Rescale code touches `src/eval/`,`src/nn/` → run the grep audit; new dt-scaled terms are `gp_scalar`/`gp_fitness`. |
| VII. No Silent Fallback Defaults | PASS (with required work) | New config (control rate, history-basis, latency model) → no in-class defaults; M2-era no-fallback (update every call site). |
| VIII. Training-Artifact Lifecycle | PASS | New M1/M2 bakes tagged `retain=expire`; pin + record any milestone (flown M1) S3 prefix in the outcome doc. |
| IX. Detached Training Launch | PASS | All retrains via `scripts/train.sh`; never agent `run_in_background`; operator drives the regression gate. |

No unjustified violations → **Constitution gate PASS**. Principle V is the one design-shaping
constraint (history-layout schema change), tracked in data-model.md + contracts.

## Project Structure

### Documentation (this feature)

```text
specs/037-20hz-control-loop/
├── spec.md              # Feature spec (complete, clarified 2026-06-09)
├── plan.md              # This file
├── research.md          # Phase 0 — clock/camera/transport/eval-budget/history decisions
├── data-model.md        # Entities: cadence config, NN input layouts, history window, dmp schema, rate tiers, packed-log frames
├── quickstart.md        # How to run the Phase-A retrain + measure the de-alias gate; bench harnesses
├── contracts/
│   ├── cadence-config.md       # control-rate config keys + the cadence-triple invariant + tick-rescale contract
│   ├── nn-input-layout.md      # M1/M2 input layout + history time-basis change (Principle V)
│   ├── transport-link.md       # MSP read/write budget, baud/SPI options, per-tier poll contract
│   ├── eval-cycle-harness.md   # the on/off-target NN cycle-count program contract
│   └── packed-log-format.md    # Phase-B blackbox-style packed dual-stream log
└── tasks.md             # Phase 2 (/speckit.tasks — NOT created here)
```

### Source Code (repository root) — touched paths

```text
crrcsim/
├── autoc_config.xml                                  # FDM dt + video fps (rate arms)
└── src/mod_inputdev/inputdev_autoc/inputdev_autoc.{h,cpp}   # gEvalUpdateIntervalMsec, cadence triple
└── src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp           # FDM step (rate fidelity)

include/autoc/
├── eval/aircraft_state.h                             # SIM_TIME_STEP_MSEC, HISTORY_SIZE, error ring
├── eval/fitness_computer.h / fitness_decomposition.h # streak, stability/energy accum
├── nn/nn_inputs.h                                    # M1(33)/M2(54) layout + history slots
├── nn/evaluator.h                                    # TrackerHistoryWindow span[6]
├── nn/topology.h                                     # topology strings + weight counts
└── util/config.h, util/scenario_prng.h               # entry sigmas, gaussian clamp

src/
├── eval/fitness_decomposition.cc                     # tick-rescale: stability/energy/closure/thrash
├── eval/fitness_computer.cc                          # streak (already time-derived)
├── eval/tracker_stepper.cc                           # span history shift
├── nn/evaluator.cc                                   # gather_*_inputs, closing_rate /0.1f → dt
└── autoc.cc                                           # sigma load/log, engage-delay (new)

xiao/                                                  # Phase B/C
├── include/main.h                                    # MSP_LOOP_INTERVAL_MSEC, MSP_NN_EVAL_DIVISOR
├── src/msplink.cpp                                   # eval, fetch/send, Serial1 baud
├── src/flash_logger.cpp                              # blocking erase/write → async ring (Phase B)
├── src/generated/nn_program_generated.cpp            # NN forward (Phase C unroll)
└── src/imu_local.* (NEW, Phase B)                    # LSM6DS3 TWIM-DMA + complementary fusion

tests/                                                 # cadence-invariance + tick-rescale + layout
docs/aircraft_tracker_handoff.md                       # ↔031 rate/part co-resolve write-back
```

**Structure Decision**: Existing three-component layout (autoc / crrcsim / xiao) is retained. Phase A is
entirely autoc+crrcsim sim-side. Phase B adds one new embedded module (`xiao/src/imu_local.*`) and
evolves the logger. No new top-level project or build target for Phase A.

## Phasing & gating (execution order)

- **Phase 0 → A → (gate) → B or C.** Phase 0 research is a *hard predecessor* of Phase A (it names the
  rate). Phase A's de-alias gate is the *hard predecessor* of any firmware. Phase B is committed only on
  a clear Phase-A win at ~20 Hz; Phase C only if 20 Hz proves insufficient.
- **P3 short-source fix** gates any clean M2 bake (not M1) — land before the 035 M2 rerun / 037 M2.
- **P1 entry-envelope tighten** folds into the Phase-A retrain (clean step, not mid-bake; t6 stays the
  baseline per Q3 caveat).
- **P5 packed logging** deferred into Phase B (after the M1 signal), not an up-front prerequisite.
- **Phase A is a candidate to split into its own near-term feature** (noted; not done here).

## Complexity Tracking

| Item | Why needed | Simpler alternative rejected because |
|---|---|---|
| History-layout schema change (Principle V touch) | tick-indexed history breaks at a new cadence (window shrinks); time-basis fix is the point of Q4 | "just add slots" grows NN input linearly and is still tick-bound; doesn't fix the 15×-too-short window |
| Three rate tiers (Phase B) | local IMU gives attitude/rate but not position; position double-integration drifts in ~10s of ms | single-rate local loop has stale target between INAV syncs — fails the tracking geometry |
| New `xiao/src/imu_local.*` module | no onboard-IMU code exists; 50 Hz needs local attitude (removes 12.6 ms MSP fetch) | keeping attitude on MSP caps the rate at the serial budget |
| Actuator dynamics (servo lag+slew, thrust lag) inside `fdm_larcsim` at substep dt (2026-06-09; 3 craft variations) | tau_servo ~20 ms <= the outer-frame command interval, so a faithful lag needs the FDM substep loop; consistency with the other craft variations (`Global::craft*`); the gate must be measured against a realistic, not infinite-bandwidth, servo | command-path filter rejected: settles ~instantly at the once-per-frame rate so it would not model the lag, and would not benefit from the raised FDM rate at 20/50 Hz |

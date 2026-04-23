---

description: "Task list for 026 — INAV ACRO Delegation"
---

# Tasks: 026 — INAV ACRO Delegation (non-bang-bang control via rate-mode inner loop)

**Input**: Design documents from `/specs/026-nn-temporal-state/`
**Prerequisites**: [spec.md](./spec.md), [plan.md](./plan.md), [research.md](./research.md)

**Clarifications**: Session 2026-04-23 locked five decisions —
full-variation training parity with cadence7, organic go/no-go with
plan.md thresholds as anchors, passive yaw in sim (HB1 has no
rudder), `out=` log token kept unchanged, throttle bang-bang accepted
for 026 gate. See [spec.md Clarifications](./spec.md#clarifications).

**Tests**: Contract test coverage is light for 026 — the validation
path is a sim retrain + measurement loop, not unit tests. A single
rate-PID unit test verifies the PID math under known step inputs
(T004). Integration validation is Phase 2 training signal.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no unresolved dependencies).
- **[Story]**: `[US1]`..`[US5]` — the five user-story phases below.
- File paths are absolute from repo root.

## Path Conventions

- Autoc source: `src/`, `include/autoc/`, `tools/`
- CRRCSim (submodule): `crrcsim/src/`
- Xiao: `xiao/src/`, `xiao/include/`
- Tests: `tests/` (Google Test)
- Analysis scripts: `specs/024-sim-real-fidelity/` (training/flight
  analysis tooling, carrying forward from 024),
  `flight-results/flight-NNNNNNNN/` (flight-specific)
- INAV fork: `/home/gmcnutt/inav/`
- Docs: `docs/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Minimal. Confirm branch health on both `autoc` and
`crrcsim` submodule before code changes.

- [ ] T001 Verify `026-nn-temporal-state` branch builds clean on both
  sides: `scripts/rebuild-perf.sh` (autoc + crrcsim) and
  `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. Baseline
  must be green before Phase 2 code changes.

**Checkpoint**: Clean build baseline.

---

## Phase 2: User Story 1 — Sim infra (ACRO PID + filters + diagnostics) (Priority: P1) 🎯 MVP

**Goal**: A sim binary that treats NN `out*` as desired body rates
for pitch and roll, runs an ACRO-style PID on top, emulates INAV's
gyro LPF + dterm LPF, writes the new diagnostic fields to data.dat,
and downstream scripts parse the new fields. No flight hardware
touched.

**Independent Test**: Run a short sim scenario with NN output held
at rate = +1 on pitch → aircraft rotates at ~`ACRO_MAX_RATE_PITCH`
(300 deg/s); hold at rate = 0 → aircraft holds attitude against a
mild disturbance (wind gust). data.dat contains the new rate and
PID columns; `sensor_self_check_lib.py` parses them without error.

### Group 1.a — CRRCSim ACRO PID re-enable

- [ ] T010 [US1] Reinstate ACRO rate PID in
  `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`
  `getInputData()` (reference preserved at commit 9809dd6 in the
  crrcsim repo). Adapt to post-024 code state: cadence-fix already
  landed so timing is clean. Use existing `ACRO_MAX_RATE_*`,
  `ACRO_FF/P/I_*`, `ACRO_PID_SCALE` constants from
  `inputdev_autoc.h:63-90`. **Pitch and roll only** — yaw stays
  passive (HB1 has no rudder per spec clarifications). Integral
  anti-windup ±10 rad, reset on span start.
- [ ] T011 [US1] Wire existing `gAcroIntegralRoll / Pitch`, `gAcroLastTimeMsec`
  globals as live state. Reset on engage start alongside
  `engageCoastThrottle` init (around
  `inputdev_autoc.cpp` line 538).
- [ ] T012 [US1] Remove the "ACRO PID disabled" comment block at
  `inputdev_autoc.cpp:993-1005` (reverted in commit 07c4832 2026-04-02).
  Replace with a single brief comment noting the PID is now live and
  pointing at spec 026.
- [ ] T013 [P] [US1] Add 25 Hz single-pole LPF on body angular rate
  before PID error computation. Compile constant
  `ACRO_GYRO_LPF_HZ = 25` next to the ACRO_* block in
  `inputdev_autoc.h`. Per-axis filter state (`gyroLpfP`, `gyroLpfQ`)
  resets on span start.
- [ ] T014 [P] [US1] Add 10 Hz PT2 (2nd-order Butterworth) LPF on PID
  D-term output. Compile constant `ACRO_DTERM_LPF_HZ = 10`. Per-axis
  state, reset on span start.
- [ ] T015 [US1] Smoke-test helper: write `scripts/026_acro_smoke.sh`
  that launches a deterministic one-path eval, extracts per-tick
  commanded vs achieved rate from the new data.dat columns (T020),
  and asserts rate-tracking error < 10% after 40 ms settling on a
  step input. Exit code signals pass/fail.

### Group 1.b — data.dat diagnostics + serialization

- [ ] T020 [US1] Extend `data.dat` header + sprintf in
  `src/autoc.cc` (around line 636 / 658) with the new per-axis
  columns: `rateCmdP`, `rateCmdQ` (desired rate rad/s),
  `rateAchP`, `rateAchQ` (achieved rate from FDM `getOmegaBody`),
  `pidFF_P`, `pidFF_Q`, `pidP_P`, `pidP_Q`, `pidI_P`, `pidI_Q`,
  `pidIntP`, `pidIntQ` (integrator state), `pidSat` (bitmask:
  bit0=pitch saturation, bit1=roll saturation). Column format
  matches existing `%7.4f` convention.
- [ ] T021 [US1] Extend `AircraftState` in
  `include/autoc/eval/aircraft_state.h` with optional `PidInternals`
  struct (per-axis FF/P/I/integrator/saturation). Populate only
  during elite-reeval (gate on `gTraceIsEliteReeval` same as
  existing PhysicsTrace). Cereal schema bump — old binary
  serializations do not load; no backward compatibility (per 026
  spec).
- [ ] T022 [US1] Wire `PidInternals` population in
  `inputdev_autoc.cpp` at the PID-compute site (alongside T010).
  Store per-tick during elite reeval; serialize with the aircraft
  state cereal stream.
- [ ] T023 [P] [US1] Update the RPC protocol / EvalResults
  serialization (`include/autoc/rpc/protocol.h`) to carry the
  `PidInternals` through from crrcsim workers back to the main
  autoc process. Version bump, no backward compat.

### Group 1.c — downstream tools

- [ ] T030 [P] [US1] Update
  `flight-results/flight-20260417/sensor_self_check_lib.py`
  `read_sim_data_dat` reader to tolerate the new columns. Existing
  reader uses header-name indexing so should just work; verify and
  add a smoke-test assertion that the new columns are present in a
  post-T020 data.dat.
- [ ] T031 [P] [US1] Update
  `specs/024-sim-real-fidelity/plot_bangbang_flight.py` — verify
  header-name parsing works; add an optional panel that plots
  `rateCmd*` vs `rateAch*` time series if present (helpful for
  Phase 2 measurement).
- [ ] T032 [P] [US1] Update
  `specs/024-sim-real-fidelity/cmd_response_scatter.py` — header
  parsing should be robust; add a note in comments that `out*` is
  now rate-command under ACRO, not surface-deflection.
- [ ] T033 [P] [US1] Update
  `specs/023-ood-and-engage-fixes/sim_polar_viz.py` — header-parse
  robustness check.
- [ ] T034 [US1] Update `tools/renderer.cc` data.dat parser
  (`parseXiaoData` or equivalent site where `rabbitTimesMs` was added
  in commit 4f94d14) to read the new PID-internal columns and store
  into `SpanData` (add `pidInternals` field). No visualization
  panel yet — just absorb the data so future renderer work can
  display it without re-training.
- [ ] T035 [US1] Update `tools/minisim.cc` header comment: add a
  one-line note "minisim does not emulate ACRO; use for MANUAL-mode
  analysis only since 026." No code change.

### Group 1.d — docs + tests

- [ ] T040 [P] [US1] Update
  [`docs/COORDINATE_CONVENTIONS.md`](../../docs/COORDINATE_CONVENTIONS.md)
  "Control Command Polarity & Scaling" section: add an "under ACRO
  (026+)" column for pitch/roll command semantics. Note yaw stays
  passive (HB1 no rudder).
- [ ] T041 [P] [US1] Add a short comment block at
  `inputdev_autoc.cpp` top documenting ACRO delegation and the
  "rate command, not surface deflection" interpretation of `out*`
  for pitch/roll.
- [ ] T042 [P] [US1] Update `msplink.cpp` `convertPitchToMSPChannel`
  / `convertRollToMSPChannel` comments: note that under 026+ INAV
  interprets the converted MSP value as rate-command (ACRO mode),
  not as stick-position (MANUAL mode). Functional code unchanged.
- [ ] T050 [US1] Unit test for ACRO PID math in a new
  `tests/acro_pid_tests.cc`: given synthetic inputs (desired rate,
  measured rate, dt, prior integrator), assert PID output matches
  hand-computed expectation. 4-6 test cases: step response, steady
  zero, integral windup clamp, integrator reset on span start.
  GoogleTest.

### Group 1.e — rebuild + phase checkpoint

- [ ] T060 [US1] Full rebuild: `scripts/rebuild-perf.sh`. All
  targets (autoc, crrcsim, renderer, minisim) compile and link.
- [ ] T061 [US1] Run T015 smoke test (`scripts/026_acro_smoke.sh`)
  against the rebuilt binary. Must pass: rate-tracking error
  < 10 % on pitch and roll step inputs.

**Checkpoint**: US1 complete. Sim runs ACRO-delegated control;
data.dat has new columns; analysis tools parse them; smoke test
passes. No flight hardware touched.

---

## Phase 3: User Story 2 — Training + measurement on cadence8 (Priority: P1, depends on US1)

**Goal**: Train cadence8 (400 gens, ACRO sim, same fitness +
topology + variations as cadence7) and measure whether dCtrl/|out|
plateau moves meaningfully off cadence7's (1.0, 2.2).

**Independent Test**: `plot_control_aggressiveness.py` on cadence8
data.dat produces a dCtrl plateau measurably below 1.0 (target
< 0.8) or the organic go/no-go call indicates the run should
escalate.

**Clarification carry-ins**: variations identical to cadence7
(apples-to-apples); gate organic not threshold-strict; gate is
pitch/roll-dominant (throttle bang-bang may persist).

- [ ] T100 [US2] Short smoke training run — 50-100 gens with
  `scripts/training.sh`, log to
  `logs/autoc-026-smoke-cadence8.log`. Confirm fitness rises, no
  NaN, no early collapse.
- [ ] T101 [US2] Full training run — 400 gens. Log to
  `logs/autoc-026-cadence8.log`. Output `data.dat` + `data.stc` in
  the standard location.
- [ ] T102 [US2] Eval suite on cadence8 extracted weights:
  `scripts/eval-suite.sh`. All tier-0 and tier-1 must PASS; tier-2
  and tier-3 reported for information per 024 norm.
- [ ] T103 [US2] Capture fresh tier-0 bitwise-determinism reference
  against the 026 binary (the reference from 024 is invalid once
  the input→output function changes).
- [ ] T104 [P] [US2] `plot_fitness_ramp.py` with cadence8 as focus
  line, cadence7 + hb1-adjust4 + test7 as comparisons. PNG to
  `specs/026-nn-temporal-state/autoc-026-cadence8-fitness.png`.
- [ ] T105 [P] [US2] `plot_control_aggressiveness.py` on cadence8's
  data.dat. PNG to
  `specs/026-nn-temporal-state/control_aggressiveness_cadence8.png`.
  dCtrl plateau compared to cadence7's (1.0, 2.2).
- [ ] T106 [P] [US2] `plot_bangbang_flight.py` on a tier-1 eval
  data.dat from cadence8. Output histogram assessment — visual
  evidence of output-value distribution spreading off ±1 extremes.
- [ ] T110 [US2] **Go/no-go decision**: organic call based on
  dCtrl plateau (< 0.8 ideal), fitness curve shape (not catastrophic
  regression from cadence7's -35951), and histogram spread.
  Document decision in
  `specs/026-nn-temporal-state/cadence8_measurement.md` with the
  PNGs and the rationale.
  - If **GO**: proceed to US3 flight-deployment sprint.
  - If **NO-GO**: proceed to US4 escalation.

**Checkpoint**: US2 complete. Training + measurement done; go/no-go
decision recorded.

---

## Phase 4: User Story 3 — Flight deployment sprint (Priority: P1, depends on US2 GO)

**Goal**: Deploy cadence8 to flight FC under INAV ACRO mode. Fly.
Analyze.

**Scope gate**: Only executed if US2 T110 is GO. Otherwise skipped
until US4 delivers a GO candidate.

**Independent Test**: Flight test produces clean cmd→rate sign
correlation (bangbang_flight histograms spread from ±1), INAV
ACRO PID tracking is reasonable in rate scatter, and the flight
report shows pitch/roll bang-bang reduced relative to
flight-20260422.

- [ ] T200 [US3] INAV flight-config edit: `xiao/inav-hb1.cfg`
  profile 1 — `rc_expo = 0`, `rc_yaw_expo = 0`. Commit with
  message calling out the MANUAL → ACRO transition.
- [ ] T201 [US3] Xiao code edit: `xiao/src/msplink.cpp`
  `performMspSendLocked()` line ~676 — change
  `state.command_buffer.channel[5] = 1000;` (forces MANUAL) to
  `= 1500;` (falls in 1300-1700 band, INAV default = ACRO on
  fixed-wing). Update the adjacent comment.
- [ ] T202 [US3] Weights → nn2cpp: extract cadence8 final weights,
  regenerate `xiao/src/generated/nn_program_generated.cpp`. Verify
  topology + weight count match expectation.
- [ ] T203 [US3] Xiao rebuild: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.
  Clean build expected.
- [ ] T204 [US3] Apply INAV config (T200) to flight FC via CLI.
  Save config and reboot. Verify rc_expo = 0 via CLI `get`.
- [ ] T205 [US3] Flash flight FC with 026 xiao binary (T203).
  Boot banner should show expected schema version + weight count.
- [ ] T206 [US3] **New tool**:
  `specs/024-sim-real-fidelity/plot_rate_tracking_flight.py`. Per-axis
  scatter of commanded rate (from xiao NN log `out=` × `ACRO_MAX_RATE_*`)
  vs achieved rate (from blackbox gyroADC × unit-conversion). Will
  measure INAV PID tracking quality in flight. Write this before
  T210 so the post-flight analysis has the tool ready.
- [ ] T210 [US3] Bench rate-response characterization: force
  commanded-rate step inputs on pitch and roll (e.g., via a test
  script that temporarily sets NN outputs to +1 / -1 / 0), record
  gyro response, compare shape to sim T015 smoke-test output. If
  divergence > 30 % on rise-time, overshoot, or steady-state gain,
  tune sim `ACRO_FF/P/I` constants in `inputdev_autoc.h` and
  re-smoke — do NOT re-train from this. Goal is sim-to-real match
  on rate-response shape.
- [ ] T211 [US3] Bench preflight checklist walk: carry forward
  T114 items from 024 (polarity sanity, `ctl loop:` summary clean
  with zero overruns, compound-attitude holds). Extend with ACRO
  verification: commanded rate = 0 → aircraft holds attitude
  against slow tilt (gravity only, no wind).
- [ ] T212 [US3] Flight test. Short xiao engage spans, varied
  paths, prefer calmer wind to isolate the 026 change from
  envelope issues observed in flight-20260422.
- [ ] T213 [US3] Post-flight: run full analysis pipeline on new
  data:
  - `flight-results/flight-20260417/sensor_self_check.py` on the
    new blackbox CSV.
  - `specs/024-sim-real-fidelity/plot_bangbang_flight.py` on the
    new xiao log (compare histograms to flight-20260422's).
  - `specs/024-sim-real-fidelity/plot_gravity_check.py` — AHRS
    independence carries forward.
  - New `plot_rate_tracking_flight.py` (T206) — INAV ACRO PID
    tracking scatter.
- [ ] T214 [US3] Write `flight-results/flight-NNNNNNNN/FLIGHT_REPORT.md`
  matching the 2026-04-22 format. Cover: engage spans, sensor
  audit, rate-tracking assessment, bang-bang comparison to
  cadence7/flight-20260422, any sim-to-real gaps surfaced.

**Checkpoint**: US3 complete. Flight-verified cadence8 controller.

---

## Phase 5: User Story 4 — Escalation ladder (Priority: P2, depends on US2 NO-GO)

**Goal**: If cadence8 doesn't pass US2 T110 go/no-go, apply
escalation steps from research §5. Re-measure after each step;
proceed to US3 flight once a step passes.

**Scope gate**: Only executed if US2 T110 is NO-GO.

**Independent Test**: Each escalation variant's retrain measures
meaningfully off the prior plateau.

### Escalation A — previous-output feedback inputs

- [ ] T300 [US4] Extend `include/autoc/nn/nn_inputs.h` NNInputs
  struct with `prev_out[3]` (or `prev_out[3][N]` for HIST_PAST-style
  window). Increment `NN_INPUT_COUNT`.
- [ ] T301 [US4] Populate `prev_out*` in the evaluator:
  `src/nn/evaluator.cc` stores this tick's NN output, reads on
  next tick. Reset on span start.
- [ ] T302 [US4] Xiao mirror: `xiao/src/msplink.cpp` already has
  `cached_*_cmd` for MSP heartbeat — wire those back as NN inputs
  in `convertMSPStateToAircraftState`.
- [ ] T303 [US4] Retrain as **cadence9** at 400 gens.
- [ ] T304 [US4] Measure (same as T104-T106) and re-decide
  go/no-go. If GO → US3. If still NO-GO → escalation E.

### Escalation E — Pareto selection

- [ ] T310 [US4] Implement Pareto-front selection as a new
  `SelectionMode` value in `src/nn/selection.cc` (or wherever
  `lexicase` lives). Dominates on (tracking-fitness,
  sum-of-|Δout|-per-tick). Keep both dominated-by-neither
  individuals in the mating pool.
- [ ] T311 [US4] Add `SelectionMode = pareto` option in
  `autoc.ini`. Compile-time constants for the
  objective-pair if tuning is needed.
- [ ] T312 [US4] Retrain as **cadence10** at 400 gens with
  `SelectionMode = pareto`.
- [ ] T313 [US4] Measure and decide. If GO → US3. If still
  NO-GO → T320.

### Escalation fallback

- [ ] T320 [US4] If both A and E fall short: write an analysis
  doc `specs/026-nn-temporal-state/escalation_outcome.md`
  documenting what was tried, what each run's plateau looked
  like, and the learning. 025 remains blocked. This is a real
  outcome possibility, not a project failure — the data clarifies
  what architectural change the NN needs.

**Checkpoint**: US4 produces a GO candidate or a clear analysis
of why ACRO + simple feedback + Pareto aren't sufficient.

---

## Phase 6: User Story 5 — Feature close (Priority: P1, depends on US3 or US4)

- [ ] T400 [US5] Summary commit documenting 026's actual outcome:
  fitness trajectory, flight data reference, what worked / what
  didn't. On the `026-nn-temporal-state` branch before merging.
- [ ] T401 [US5] Update
  [`specs/025-craft-variations/spec.md`](../025-craft-variations/spec.md)
  Status section: UNBLOCKED if 026 succeeded, or REVISIT with
  new notes if 026 changed our understanding.
- [ ] T402 [US5] Merge `026-nn-temporal-state` → main (submodule
  pointer bump in crrcsim first, then autoc parent — per project
  memory on merge order).
- [ ] T403 [US5] Backlog cleanup: move any follow-ups surfaced
  during 026 (throttle airspeed PID, per-tick renderer PID panel,
  rename `out*` to `rate_cmd*`, etc.) to
  [`specs/BACKLOG.md`](../BACKLOG.md).

**Checkpoint**: 026 closed. Main branch carries the flight-verified
ACRO-delegated training and inference pipeline.

---

## Dependencies

```
Setup (T001)
   ↓
US1: Sim infra (MVP)
  Group 1.a CRRCSim ACRO PID    (T010-T015)
  Group 1.b data.dat + cereal   (T020-T023)    [sequential after 1.a]
  Group 1.c downstream tools    (T030-T035)    [after 1.b, parallel within]
  Group 1.d docs + tests        (T040-T050)    [after 1.a, parallel]
  Group 1.e rebuild + smoke     (T060-T061)
   ↓
US2: Train + measure cadence8 (T100-T110)
   ↓
   ├── GO → US3: Flight deployment (T200-T214)
   └── NO-GO → US4: Escalation (T300-T320)
                       ↓
                    GO → US3
   ↓
US5: Feature close (T400-T403)
   ↓
FEATURE CLOSED
```

### Parallel opportunities

- **Within US1**: Group 1.a (T013, T014) parallel with each other
  after T010-T012. Group 1.c (T030-T033) all [P] after T020.
  Group 1.d (T040-T042) [P] after T010.
- **Within US2**: T104, T105, T106 measurement plots run in parallel.
- **Between phases**: T206 (new `plot_rate_tracking_flight.py`) can
  be written during US1 or US2 to reduce US3 critical-path work.

### Critical path

US1 → US2 → US3 (assuming GO). T001 → T010 → T020 → T061 → T101 →
T110 → T212 → T214.

## Implementation strategy

**MVP scope** = US1 (Phase 2). Sim running ACRO-delegated control
with full diagnostics is the core new infrastructure. Everything
else is measurement, deployment, or escalation on that
infrastructure.

**Incremental checkpoints**:

- After US1: sim runs ACRO. Data flows through the pipeline.
  Training hasn't started. Fully rollback-able if something looks
  wrong in Phase 1 — just revert the branch.
- After US2: trained controller with ACRO-aware fitness profile.
  No hardware touched. Go/no-go gate determines next action.
- After US3 (GO path): flight-verified cadence8. Flight hardware
  runs new binary.
- After US4 (NO-GO path): escalation variants trained and
  measured. Eventually produces a GO or a clear understanding of
  why simple escalations don't work.
- After US5: 026 merged to main, 025 status updated, follow-ups
  in backlog.

**Guardrails**:

- No hardware flash until US2 gate is GO. Bench rate-response
  characterization (T210) is the last sim-matching sanity check
  before flight.
- Keep compile-time constants in `inputdev_autoc.h`. Tunable
  coefficients in ini files are out of project style.
- No backward compatibility on data.dat, cereal binary
  serialization, or xiao log format. Break cleanly.
- Semantic of `out*` shifts under ACRO but the literal token stays
  the same in all file formats. Communicate via docs + new
  diagnostic fields.

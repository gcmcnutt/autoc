# Tasks: 043 — ACRO dual-loop

**Input**: [spec.md](spec.md) (governs) · [plan.md](plan.md) · [research.md](research.md) ·
[data-model.md](data-model.md) · [contracts/](contracts/) · [quickstart.md](quickstart.md)

---

## ⛔ READ-FIRST — four things before you touch anything

1. **Read [spec.md](spec.md) § What ACRO is.** The whole feature is downstream of that definition. ACRO is
   **rate** control and therefore implicitly **not** ANGLE: no attitude feedback, no self-levelling, no
   recovery beyond the envelope. Building "zero command holds attitude" as a *goal* produces ANGLE mode
   and trains the policy on a safety net the aircraft does not have.
2. ⛔ **T004 IS IRREVERSIBLE-IF-SKIPPED.** Extracting from the pinned 041-t7 dmps MUST complete before any
   `ScenarioMetadata` change (T010+). The format break orphans every dmp ever written, including the
   `retain=keep` baseline every 043 comparison is measured against. Same failure mode as 041's T011a.
3. ⛔ **Phase order comes from spec.md § Execution order, NOT from story priority.** One bake carries
   everything (spec assumption 12), so every story is upstream of US4. US5 (P2) and US6 (P3) both land
   *before* US4 (P2). This is intentional; do not "fix" it.
4. **INAV's fixed-wing loop is NOT a PID** — see research.md R1/R1a. It is **feed-forward dominant** with P
   and D **attenuated by a Gaussian in the setpoint**. A textbook PID implementation is wrong by roughly an
   order of magnitude in the dominant term.

**Tests are REQUIRED** (Constitution I, and the contracts carry explicit acceptance tests).

## Format: `[ID] [P?] [Story] Description`

- **[P]** — parallelizable (different files, no dependency on an incomplete task)
- **[Story]** — US2…US6 (⚠️ US1 was DROPPED 2026-08-24; numbering preserved deliberately)

---

## Phase 1: Setup — baselines and facts to pin

**Purpose**: establish clean build baselines and resolve the two cheap unknowns that shape later work.

- [X] T001 Verify clean baseline build of autoc + crrcsim via `bash scripts/rebuild.sh` from repo root; record the test count in `specs/043-acro-dual-loop/baseline.md`. ⭐ **Measured 2026-08-25**: 47 suites ran / 501 tests / 0 failures, **plus** the 2 suites T021a wires in (7 more tests) = **508 passing**; one suite skipped by design (`source_dmp_s3_integration_tests`, needs `AUTOC_S3_TESTS=1`)
- [X] T002 [P] Verify xiao host compile via `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed` from `xiao/`; record result in `specs/043-acro-dual-loop/baseline.md`
- [X] T003 [P] Confirm the **as-run** FDM substep by logging `Global::dt` at scenario init in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`; record the value and the 333 Hz-vs-2 kHz phase justification (research.md addendum) in `specs/043-acro-dual-loop/baseline.md`

---

## Phase 2: Foundational — ⛔ BLOCKING, and one task is irreversible

**Purpose**: preserve the baseline before the format break. ⛔ **No task in Phase 3+ may start until T004
and T005 are complete.**

- [X] T004 ⛔ **IRREVERSIBLE-IF-SKIPPED** Extract everything later phases need from the pinned 041-t7 dmps at `s3://autoc-m1/autoc-9223370249590214474-2026-08-20T22:22:41.333Z/` into a format that survives the wire-format break (per-tick CSV), writing to `specs/043-acro-dual-loop/artifacts/t7-extract/` (FR-057)
- [X] T005 Verify the T004 extract is readable and complete **independently of any dmp loader** — row counts, per-axis rate statistics and the 3–5 Hz / 5–10 Hz band-power figures reproduce the values in `specs/041-m2-depth/outcome.md`; record in `specs/043-acro-dual-loop/artifacts/t7-extract/VERIFY.md`
- [X] T006 [P] Read the **dynamic gyro notch centre frequency** from the 041-t7 blackbox log and record it in `specs/043-acro-dual-loop/research.md` addendum D — this decides whether the notch is modelled at all (research.md addendum D; Q = 2.5, not 250)

**Checkpoint**: the baseline is preserved and independently readable. The format break is now safe.

---

## Phase 3 (US5 · exec order 2): Variations — inventory, then the new axes

**Goal**: the variation regime is legible and documented, and a few more craft axes land for IMU
imperfection and pitch damping.

**Independent test**: read the inventory against the code and confirm every class appears with the correct
ramp status; run with the new axes at σ=0 and confirm bit-identical results, then at σ>0 and confirm the
draws reach the FDM and replay from the recorded seed.

### Inventory (FR-050 / FR-051)

- [X] T007 [US5] Write the variation-class inventory — class, what it varies, magnitude, enable knob, per-scenario or not, **and whether the ramp applies** — for wind, rabbit, entry, craft, camera, in `specs/043-acro-dual-loop/variation-inventory.md` (FR-050, SC-008)
- [X] T008 [US5] ⛔ Fix the documentation contradiction: `autoc.ini` claims craft *"RAMPS with wind/entry (same VariationRampStep)"*; `include/autoc/eval/scenario_meta_apply.h` ramps **only** the environmental classes and is what runs. Correct the `autoc.ini` comment (FR-051, SC-008)

### The new craft axes (FR-052 / FR-052a / FR-052b)

- [X] T008a [US5] ⛔ Assert the scenario regiment is **unchanged at 294** (6 paths × 49 winds) — `WindScenarios = 49`, population not increased — and add the check to the pre-run gate so a silent bump cannot reach a bake, in `autoc.ini` and `specs/043-acro-dual-loop/variation-inventory.md` (FR-058)
- [X] T009 [US5] Add `CraftImuMisalignSigma`, `CraftGyroScaleSigma`, `CraftAccelScaleSigma`, `CraftAccelBiasSigma`, `CraftCmQSigma` to the config X-macro in `include/autoc/util/config.h` and to `autoc.ini` with the σ from contracts/craft-imu-axes.md (2.5σ = the intended limit; ⛔ no bespoke clip constants)
- [X] T010 [US5] Append the new fields to `CraftSigmas` and `CraftDeltas` in `include/autoc/eval/craft_variation.h`, and append their draws at the **bottom** of `generateCraftFromClassPRNG` so every existing draw keeps its value (FR-054)
- [X] T011 [US5] Append the matching fields to `ScenarioMetadata` in `include/autoc/rpc/scenario_metadata.h`, last, and add them to the `serialize()` walk in the same position — ⛔ requires T004/T005 complete (data-model.md §1)
- [X] T012 [US5] Implement `craftCmQ` as an **absolute physical value + clamp** (centre −4.2, clamp [−5.0, −3.6] per `crrcsim/models/hb1_streamer.xml`), following the `craftServoSlew` pattern — ⛔ **not** a delta (FR-052b)
- [X] T013 [US5] Wire the new draws through `populateScenarioSeedTable` / the variation prefetch in `src/autoc.cc`, honouring draw-and-discard so toggling cannot shift another class's draws (FR-054)
- [X] T014 [US5] Extend the prefetched-variations startup log in `src/autoc.cc` with the new columns, gated the same way the craft columns are
- [~] T015 [US5] Apply the new axes FDM-side in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` — IMU misalignment/scale/bias on the sensor gather path, `craftCmQ` onto the FDM pitch-damping coefficient. ⭐ **SPLIT (operator 2026-08-25)**: ✅ `craftCmQ → fdm_larcsim Cm_q` DONE (no-op verified: XML nominal = center −4.2) + Global carriers for all 13 fields set per-scenario in inputdev (`global.{h,cpp}`, inputdev `:602`). ⛔ **IMU observation-transform (gyro/accel/attitude→target-geometry the policy sees) DEFERRED to Phase 5** — `getGyroRates()` feeds BOTH NN inputs and `fitness_decomposition.cc`, so it needs a *sensed* copy distinct from truth; the inner loop (`Cntrl_InavFwRate`) is the other consumer and the T055 bench polarity check validates signs end-to-end. Land it with Phase 5.
- [X] T016 [US5] ⛔ Confirm `applyVariationScale` in `include/autoc/eval/scenario_meta_apply.h` leaves the new fields **untouched** — craft is not ramped (FR-055)

### Tests

- [X] T017 [P] [US5] Test: σ=0 on every new axis produces **bit-identical** results to the axes not existing, in `tests/` (FR-053, SC-009)
- [X] T018 [P] [US5] Test: σ>0 replays identically from the same `scenarioSeed`, and the five pre-existing class sub-seeds are unchanged, in `tests/` (SC-009)
- [X] T019 [P] [US5] Test: `craftCmQ` clamps to [−5.0, −3.6] at ±2.5σ and centres at −4.2 with σ=0, in `tests/`
- [X] T019a [P] [US5] ⛔ Test: a **pre-043 dmp fails LOUDLY** on load — a clear error naming the artifact/reader mismatch, ⛔ never a silent truncation or default-init. Use one object from the T004 extract's source prefix as the fixture, in `tests/` (Constitution V read-side contract)
- [X] T020 [P] [US5] Test: `craftCGDelta` and `craftCmQ` are **independent** draws and do not double-count the static/dynamic split, in `tests/` (FR-052b)

**Checkpoint**: SC-009 passes. ⛔ The wire format has changed — every pre-043 dmp is now unreadable, by design.

---

## Phase 4 (US6 · exec order 3): Housekeeping on the opened surfaces

**Goal**: items whose only cost is that someone already has the file open.

**Independent test**: each item verifiable alone; any item not done is recorded as deferred.

- [X] T021 [P] [US6] Make `crrcsim/src/mod_inputdev/CMakeLists.txt` link `autoc_common` instead of cherry-picking individual source files; remove the cherry-pick lines (FR-072)
- [X] T021a [US6] ⛔ Add `shared_input_block_tests` and `nn_input_scaling_tests` to the `run_autoc_tests` ALL-target `DEPENDS` list in `CMakeLists.txt`. ⚠️ **Found 2026-08-25 by the pre-implement `rebuild-perf.sh`**: both are registered via `add_test(NAME ...)` — so the script's gate self-check counts them — but neither is in the ALL target, so `make` never runs them. Gate expected **49** suites, **47** ran. Both pass when invoked by hand (4/4 and 3/3), so nothing was broken — the coverage was **invisible**, which is exactly the failure GUARD 3 exists to catch. ⭐ Directly relevant here: `nn_input_scaling_tests` covers the constants T023 changes and `shared_input_block_tests` covers the craft tail T024 touches (Constitution II/IV)
- [X] T022 [P] [US6] Resolve the `nnextractor -g` (FILE number) vs `dmp-dump --gen` (GENERATION) footgun — make them agree or make each state which it takes, in `tools/` (FR-071)
- [~] T023 [US6] Formal input normalization from **measured** statistics rather than hand-derived constants, in `include/autoc/nn/nn_inputs.h` and its consumers (FR-070) ⚠️ **DEFERRED (043 T026, 2026-08-25)** — bake-affecting NN input rescale; already filed in specs/BACKLOG.md (041 P2-8 follow-up). Not in 043's bake.
- [~] T024 [US6] Type-safe NN sensor interface — name input columns by enum at the call sites the new axes touch, in `include/autoc/nn/nn_inputs.h` (FR-073) ⚠️ **RESEQUENCED to Phase 5 (043 T026)** — its target sites are the deferred T015 observation-path; enum-naming lands with them.
- [~] T025 [US6] Simulator sampling-time variation (20 Hz tick dither) in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (FR-074) ⚠️ **DEFERRED to specs/BACKLOG.md (043 T026)** — new determinism-affecting tick-dither feature, not open-file housekeeping; cut-list item.
- [X] T026 [US6] Record any FR-07x item **not** done as deferred in `specs/043-acro-dual-loop/outcome.md`, and append it to `specs/BACKLOG.md` (FR-076, Constitution X)
- [X] T027 [US6] ⛔ Clean `bash scripts/rebuild-perf.sh` — REQUIRED after the T021/T021a `CMakeLists.txt` changes, not an incremental reconfigure. ⭐ **Verify the gate self-check now reports 49 of 49 suites** — it read 47 before T021a (Constitution IV). **Operator-driven; ask first.**

**Checkpoint**: build coherent, tests green, format break fully absorbed.

---

## Phase 5 (US2 · exec order 4): The ACRO inner-loop model

**Goal**: crrcsim's chase aircraft is driven by rate setpoints through a model of INAV's fixed-wing rate
controller.

**Independent test**: replay the 041-t7 command stream and compare rate response against the flight's
ACRO-flown segments; separately confirm a known-good genome still trains.

### Tests first (Constitution I)

- [X] T028 [P] [US2] Contract test: constant rate setpoint ⇒ achieved rate converges, rise time consistent with the gains, in `tests/` (contracts/inav-fw-rate-loop.md test 1)
- [X] T029 [P] [US2] Contract test: **zero command + non-zero `craftTrimDelta` ⇒ body rate settles to ZERO and stays**, in `tests/` (SC-012)
- [X] T030 [P] [US2] ⛔ Contract test: **no self-levelling** — displaced to a bank angle with zero command, bank is approximately held over ~1 s and then **drifts** (expected, on the order of seconds — there is no attitude reference). ⭐ The discriminator is **sign correlation, not stability**: run from **+30° and −30°**; ANGLE drives *both* toward zero (drift correlated with bank sign), ACRO's drift is uncorrelated. A sign-correlated restoring trend FAILS, in `tests/` (FR-019a, SC-012 converse)
- [X] T031 [P] [US2] Contract test: attenuation curve matches `exp(−r²/2σ²)` at r ∈ {0, σ, 2σ}, σ = 61.2 °/s roll and 20.4 °/s pitch, in `tests/`
- [X] T032 [P] [US2] Contract test: FF dominance — 88 °/s roll setpoint at zero error yields ≈142 of the ±500 budget, in `tests/`
- [X] T033 [P] [US2] Contract test: I-term lock freezes accumulation for ≤`lockTimeMaxMs` on a large setpoint step with large error, in `tests/`

### Implementation

- [X] T033a [P] [US2] ⭐ Test: the **cascade RATIOS** are right, not just the constants — inner PID cadence : servo command frame : outer control loop, and the gyro-filter corner relative to each. FR-011's own claim is that *"getting the ratios right matters more than getting any single constant exactly right"*, and a model with correct gains and one wrong rate oscillates where the aircraft does not. Assert the as-run ratios against contracts/inav-fw-rate-loop.md, in `tests/` (FR-011)
- [X] T034 [US2] Create `crrcsim/src/mod_cntrl/cntrl_inavfwrate/cntrl_inavfwrate.h` — per-axis state (integrator, prevGyroRate, dterm/pterm filter state, `targetOverThresholdTimeMs`); ⛔ **no attitude state of any kind** (FR-019a, data-model.md §4)
- [X] T035 [US2] Implement `crrcsim/src/mod_cntrl/cntrl_inavfwrate/cntrl_inavfwrate.cpp` exactly per contracts/inav-fw-rate-loop.md — FF + Gaussian-attenuated P/D + locked/clamped I, output clamped to ±500
- [X] T036 [US2] Register the controller in `crrcsim/src/mod_cntrl/controller.cpp::LoadList` (one `else if`) and add it to `crrcsim/src/mod_cntrl/CMakeLists.txt`
- [X] T037 [US2] Add the `<controllers>` node with every constant from contracts/inav-fw-rate-loop.md to `crrcsim/models/hb1_streamer.xml`, so they change **without a rebuild** (FR-014) ⭐ **UPDATED 2026-08-30 (operator)**: the node lives in `crrcsim/models/hb1_streamer.xml` `<config><controllers>` as the task originally said — `fdm_larcsim` gained the per-model load (fdm_mcopter01 pattern) so this now works. (It was briefly in the global `autoc_config.xml`, the only place that loaded controllers before that change.) Verified: hb1 → "model-local controllers loaded: 1"; stock model → "none". See outcome.md.
- [X] T037a [US2] ⛔ Clean `bash scripts/rebuild-perf.sh` — REQUIRED after T036's `crrcsim/src/mod_cntrl/CMakeLists.txt` change (new target + test registration), **not** an incremental reconfigure. ⚠️ This is a **second** mandatory clean rebuild; T027 covered the Phase-4 CMakeLists change only (Constitution IV). **Operator-driven; ask first**
- [X] T038 [US2] ⛔ Model only `gyro_main_lpf_hz` (25 Hz PT1) inside the loop; **`acc_lpf_hz` is the observation path and contributes NO phase to ACRO** (FR-013, corrected 2026-08-25)
- [X] T039 [US2] Model the deliberately-absent list as absent, each with its reason in a comment: TPA (`tpa_rate=0`), D-boost (identity), setpoint accel limit (`rate_accel_limit_roll_pitch=0`, FR-019b), anti-alias LPF (1.15° at 5 Hz), and the notch per the T006 measurement
- [X] T040 [US2] Convert NN outputs to rate setpoints in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` per contracts/action-space.md — one shared scaling definition. ⭐ This is what makes the chase aircraft rate-driven rather than surface-driven (FR-010, FR-016) ⭐ **Done in the adapter (Cntrl_InavFwRate), not a separate inputdev change**: ControllerCallback auto-routes pInputsFromUser→controller, so getInputData needs no change; the NN→rate scaling (×2 command recovery, ×maxRate, ÷pidSumLimit→surface) lives in the adapter/core per action-space.md.
- [X] T041 [US2] Keep throttle a **direct** command, not a rate, in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (FR-017); confirm yaw reaches no surface — no rudder (FR-018) ⭐ Done in the adapter: throttle passes through (CopyFrom), rudder forced 0 (no yaw surface).
- [X] T042 [US2] Document the per-axis **effective gain curve** (σ 61.2 roll vs 20.4 pitch — the two axes run materially different controllers) in `specs/043-acro-dual-loop/research.md` (R8)

### Gates

- [X] T043 [US2] ⛔ Run the **all-attitude zero-command sweep BEFORE autoc is connected** — in sim across the attitude sphere — so a hold failure is attributable to the model (FR-019, SC-014 part 1)
- [~] T044 [US2] ⛔ **Trainability gate (SC-004)**: seed a short run from a known-good genome and confirm the GA improves rather than stalling — the 023-Phase-9a guard. Launch per Constitution IX via `scripts/train.sh` ⭐ **Satisfied by evidence, not in the prescribed form (2026-08-31)**: the arm-A smoke climbed −182 → −1577 by gen 328 (16/16 scenarios, past the basic-m1 400-gen baseline of −1320), and the production bake itself is climbing strongly on arm C (gen 248: best −63,107, pctInStreak 43.7% vs 041-t7 FINAL 38.3%). ⚠️ No separate seeded-from-known-good short run was done, and the arm-A evidence predates the arm-C plant change (T050a).
- [X] T045 [US2] Verify determinism: identical seed + config reproduce identical trajectories, and the eval-vs-training bitwise gate holds (FR-015). **Operator-driven; ask first** ✅ **PASSED 2026-09-01**: eval-vs-training BITWISE match on the t2 bake\'s gen-554 genome — `NN Eval fitness: -87763.770153` == `Stored fitness: -87763.770153` (`NN_EVAL_SAME`), 294 scenarios, and the scenario cascade reproduced exactly (`first=0x528b9fe8256d2c31, last=0x4c49380b620ed285`, identical to the bake). Config `autoc-043-t2-eval.ini` mirrored `autoc.ini` with only the 5 eval knobs differing; EvalThreads kept at 20 so the aggregate sum order matches. ⚠️ **That file was DELETED 2026-09-04** — `autoc-eval.ini` now mirrors `autoc.ini` exactly and the same gate runs as `scripts/eval_suite.sh <nn01-file> 0 1788150478`; re-verified on the gen-800 genome (`NN_EVAL_SAME`, −88013.840878, 281/294). ⭐ This validates the whole changed 043 pipeline as replay-exact: the new craft IMU/CmQ wire fields, arm C (pitch maxRate 240) and `Cntrl_InavFwRate` in the substep loop.

**Checkpoint**: SC-012, SC-004 and SC-014 part 1 pass. The model is trustworthy enough to build on.

---

## Phase 6 (US2 · exec order 5): Pin the plant

**Goal**: the actuator term is measured rather than modelled, and the 037 constants are triaged.

⚠️ **STATUS 2026-09-04 — Phase 6 did NOT run before the bake.** T065 launched 2026-08-30 with every task
below still open, so **SC-010 is unmet** and the plant the 043-t2 genome trained against is the *modelled*
actuator, not a *measured* one. That is a knowing deviation from the execution order, not an oversight to
correct now: re-running it cannot change a finished bake. ⛔ The consequence is scope, not safety — it
becomes an attribution limit on Phase 10 (a sim↔flight divergence in T077 cannot be cleanly split between
"the ACRO model is wrong" and "the actuator model is wrong"). Route these to `specs/BACKLOG.md` under T082
unless the flight itself raises an actuator question. **T050a is therefore moot for this bake** — there was
no plant change to re-gate against.

- [ ] T046 [US2] Bench servo step-response on the flight article; place the real servo inside the `craftServoSlew` (16–32 units/s) and `craftServoPwmPhase` (0–20 ms) spread. Record in `specs/043-acro-dual-loop/actuator-pin.md` (FR-020, SC-010)
- [ ] T047 [US2] Targeted 037-constant review — mark each **"contradicted by 041-t7 and changed"** or **"checked and unchanged"**, ⭐ including the **static-margin / pitch-damping class**, in `specs/043-acro-dual-loop/actuator-pin.md` (FR-021, FR-022)
- [ ] T047a [US2] Complete the **FR-056 craft-realism review** at `n = 2` articles — AHRS alignment, control-surface trim/bias, and control response gains and rates — recording per axis whether its spread is **measured** or still **assumed**. ⚠️ Bounded by the operator's 2026-08-25 note: build repeatability is coming, so **characterise, do not chase**. Write to `specs/043-acro-dual-loop/variation-inventory.md` (FR-056)
- [ ] T048 [US2] Evaluate the FR-012a phase-delay candidates — `gyro_main_lpf_hz`, `dterm_lpf_hz`, dynamic notch Q, `servo_pwm_rate` — each with a **computed** phase contribution at the frequency the inner loop is trying to control. ⛔ `acc_lpf_hz` is excluded on inner-loop grounds (FR-013). Record in `specs/043-acro-dual-loop/phase-delay.md`
- [ ] T049 [US2] ⛔ For any INAV parameter changed: change it **identically in the sim**, bench-verify it, and fold it into the config of record before the bake. ⚠️ *"a param or two"* — every change costs attribution (FR-012a)
- [ ] T050 [US2] ⛔ Check the servos are digital before considering `servo_pwm_rate`; record the finding either way (FR-012a)

- [ ] T050a [US2] ⚠️ **If T046–T050 materially changed the plant** (actuator constants, or any FR-012a phase-delay parameter), **re-run the T044 trainability gate** against the changed model. ⛔ Otherwise SC-004 was measured on a plant the bake will not use. If nothing material changed, record that judgement and its basis instead of re-running (SC-004)

**Checkpoint**: SC-010 passes; every constant is marked changed-or-checked.

---

## Phase 7 (US3 · exec order 6): The flight stack commands ACRO

**Goal**: engage selects ACRO, disengage releases the mode, and the bench proves both before anything flies.

**Independent test**: on the bench with GPS disconnected and the bench target flashed first, engage and
disengage while watching INAV's mode flags and the servo response.

### ⭐ ACRO tuning of INAV — NEW 2026-08-30, from the A/B/C action-space experiment

- [X] T051a [US3] ✅ **`xiao/inav-hb1.cfg` UPDATED 2026-08-31 to `pitch_rate = 24`** (rateprofile 1; ⚠️ the file is CRLF, so anchored `sed` silently no-ops — use a binary replace). ⛔ Still to do: flash/CLI the actual FC + bench-verify. **DECIDED (operator): go with 24.** INAV ACRO rate tuning — `pitch_rate 12 → 24` in `xiao/inav-hb1.cfg` **and** on both FC targets, matching the sim's arm C (`models/hb1_streamer.xml` pitch `maxRate="240"`). ⭐ Rationale in [outcome.md](outcome.md) § A/B/C: `pitch_rate 12` sits **below INAV's own default of 20**, capping full-stick elevator at **54%** of the fixed ±500 `pidSumLimit` (FF = 120 × 2.258 = 271) *and* forcing every useful pitch rate to the P/D attenuation floor (aP 0.016 at 58 °/s). At 24 → **100% elevator and aP 0.363**. Operator flew it: *"feels about right … a full commanded pitch up doesn't do a loop in real either"*. ⚠️ This is a **deviation from the spec's "gains and rates stay as-is"** and follows FR-012a discipline (change identically in sim, bench-verify, fold into the config of record). Allowed range is 4–180, so 24 is well inside it. ✅ **BENCH-CONFIRMED**: `H rates:36,24,3`, axisRate[1] reaches 238 °/s. ✅ **FC CONFIRMED ON THE FLIGHT ARTICLE 2026-09-04** — `pitch_rate 24` set and saved, verified by a fresh CLI dump which is now `xiao/inav-hb1.cfg` itself. ⭐ The file graduates from *"Apr-2 dump with `pitch_rate` hand-edited"* to a **genuine flight-article capture** (firmware `38ff0d29e`, `control_profile 1`, rates **36 / 24 / 4**). The CRLF hand-edit hazard noted above is therefore retired. Accel recalibrated at the same time (`ins_gravity_cmss` 948.609 → **972.092**). Before/after preserved in [artifacts/MANIFEST.md](artifacts/MANIFEST.md) § 4b; the loose `INAV_8*.txt` dumps were deleted once folded in.
- [X] T051b [US3] ⛔ ⚠️ **Bench rates differ and that is FINE (operator 2026-08-31: the bench rig is defaults, it does not fly or really move)**: `inav-bench.cfg` rateprofile 1 is roll 18 / pitch 9 vs the flight 36/24. The bench verifies PLUMBING — mode entry/exit, polarity, MSPRCOVERRIDE floor — none of which depends on the rate setting. ⛔ But the arm-C signature (full pitch stick ⇒ ~100% elevator) is rate-dependent, so verify THAT on the **flight article** after flashing, not on the bench, where pitch_rate 9 would show ~41% and look like a failure. **SIM↔FC RATE-PARITY GATE (pre-flight, hard stop)**: assert the flown FC's `rates` equals the model XML's `maxRate/10` on every axis — today roll 36/360 and pitch **24/240**. ⚠️ **The 043-t2 bake is training against pitch 240 °/s; flying an FC still set to 120 °/s diverges precisely in the axis this feature exists to fix**, and would waste the bake. Record both sides in `specs/043-acro-dual-loop/bench-notes.md` and in the run MANIFEST (T068). ✅ **PARITY CONFIRMED 2026-09-01**: FC 36/24 == model maxRate 360/240.
- [ ] T051c [US3] Decide the **roll dead-band** question: full-stick roll FF = 360 × 1.613 = 581 against the fixed ±500 budget, so the top **14% of roll stick is clipped** (operator 2026-08-30: *"the roll is a bit hot"*). `roll_rate 31` would give exactly full surface at full stick with no dead range, costing a little damping (aP 0.30 → 0.20 at 95 °/s). ⚠️ If changed it is a **second** variable and must also pass T051b parity; if not changed, record the decision and why. ⛔ **PRE-FLIGHT FORCING (2026-09-04)**: the 043-t2 bake trained against roll `maxRate="360"` and the FC is set to `roll_rate 36`, so T051b parity holds **only while roll stays 36**. Changing it now invalidates the parity gate against the genome about to fly. **Decision for THIS flight: leave roll at 36 and record the 14% clip as accepted**; revisit for the next bake, where the sim side can move with it.
- [X] T051 [US3] INAV fork: fix the `mspOverrideInit` first-frame 200 ms floor in `~/inav/src/main/rx/msp_override.c` (FR-042) ⚠️ **NOT NEEDED / REVERTED 2026-09-01** — the 700 ms is history-buffer priming, not dead time. Stock INAV; no fork build.
- [X] T052 [US3] Build INAV for **both** targets — bench `MAMBAF722_2022A` first, then flight `MATEKF722MINI`. ⚠️ Disconnect the GPS before flashing (FR-043) ⚠️ **NOT NEEDED 2026-09-01** — no custom INAV firmware; CLI config only.
- [X] T053 [US3] ✅ **DONE 2026-08-31** (`channel[5]` 1000 → **1500**; bands confirmed on the bench with the INAV Configurator: RC6 >1200 and <1600 ⇒ ACRO. The old comment claiming 1000 'forces MANUAL' is corrected — MANUAL is a separate switch the xiao does NOT override, deliberately, so the pilot keeps a mid-engagement escape). xiao host compile passes. Change `performMspSendLocked` in `xiao/src/msplink.cpp` to select **ACRO** rather than forcing MANUAL — the aux-2 mid-band (1200–1800) selects ACRO on the config of record (FR-040) ✅ **BENCH-VERIFIED 2026-09-01** — see bench-notes.md.
- [X] T054 [US3] Stop forcing the mode on disengage in `xiao/src/msplink.cpp` so flight-mode selection returns to the pilot's switch; today the channel is forced on **every** frame the xiao sends (FR-041) ✅ **BENCH-VERIFIED 2026-09-01** (path-complete → pilot has control → servo reset → re-arm).
- [X] T055 [US3] ⛔ Bench-verify **polarity end to end**: NN sign → PWM about 1500 → `rcCommand` sign → commanded rate sign → achieved body-rate sign. A sign error here is invisible in every surviving artifact (FR-016, contracts/action-space.md) ✅ **BENCH-VERIFIED 2026-09-01** to the SURFACE (elevon common/diff +0.998/+1.000; NN→rate signs correct). ⛔ Final surface→achieved-rate link still needs flight (static bench).
- [X] T056 [US3] Bench-verify mode entry, mode exit, rate response at the surfaces, and that **no surface responds to the yaw axis** (FR-045, FR-018). ⭐ **Extended 2026-08-30, scoped 2026-08-31**: ⛔ on the **FLIGHT ARTICLE** (not the bench rig, which runs default rates and would show ~41%), confirm at the surfaces that **full pitch stick reaches ~100% elevator travel** (it reached only 54% at `pitch_rate 12`) — that is the physical signature of the arm-C change and the cheapest confirmation the FC actually took it. ✅ **BENCH-VERIFIED 2026-09-01**: axisF[1] 537 (was 270), servo[1] 1091–2000 hits the stop, yaw axisRate/axisF = 0.0.
- [X] T057 [US3] Bench-verify MSPRCOVERRIDE engages without the 200 ms floor at `failsafe_recovery_delay = 0` (FR-042) ✅ **MEASURED 2026-09-01: 783 ms** — and deliberately KEPT (it primes the NN history buffers). INAV fork change reverted; stock firmware.
- [X] T058 [US3] Measure the **achievable INAV telemetry rate** and record it in `specs/043-acro-dual-loop/bench-notes.md` — `blackbox_rate_denom = 32` gives 60 Hz today; either answer is a result (FR-044, R7) ✅ **MEASURED 2026-09-01: ~60 Hz** (1673 samples / 28.05 s).
- [X] T059 [US3] Regenerate the xiao firmware NN via `tools/nn2cpp.cc` against the current genome and confirm the host compile still passes; ⛔ **no xiao log-format change in 043** (US1 dropped, FR-005 cut) ⛔ **RE-OPENED 2026-09-04 — the working copy holds the WRONG NET.** `xiao/src/generated/nn_program_generated.cpp` on disk is dated **2026-07-12**, topology **37 → 32 → 16r → 3**, sourced from `autoc-m1/autoc-9223370253553029228-2026-07-06…/gen9200` — a pre-041 recurrent 37-input net. The current M1 is **45 → 32 → 16 → 3**. The directory is git-ignored (`.gitignore:24`), so the gen-554 regeneration recorded in commit `400b217` did **not** survive into this working copy and a branch checkout will never restore it. ⭐ Regenerate against **gen 800**, not gen 554 — the bake finished, and gen 800 is the genome T045 proved bitwise (−88,013.84 vs gen 554's −87,763.77). Correct form (⚠️ `-w` is the genome, `-i` is the config): `./build/nn2cpp -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp`, then confirm the header says `45 -> 32 -> 16 -> 3` and the S3 source line ends `…2026-08-31T04:27:58.060Z/gen9200.dmp.zst` before compiling. ✅ **DONE 2026-09-04 (this host)**: ⚠️ the "wrong net" finding above was true on the *other* host only — this working copy held the correct run at gen 554 (`gen9446`, −87,763.77), not the July 37-input net. Regenerated against **gen 800**: `nnextractor -k autoc-9223370248704297747-2026-08-31T04:27:58.060Z -o nn_weights.dat` (fitness **−88,013.840878**, sigma 0.051642) then `./build/nn2cpp -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp`. Header verified: source `…/gen9200.dmp.zst`, topology `45 -> 32 -> 16r -> 3`. Build gate GREEN: `pio run -e xiaoblesense_arduinocore_mbed` SUCCESS, RAM 53.5%, Flash 45.5%. ⚠️ Note `nnextractor` prints `Generation: 800` (filename-derived) while `autoc` eval mode prints `799` (`genome.generation`, 0-indexed) — same net; **fitness is the field to match on**.

**Checkpoint**: the aircraft commands ACRO, releases it, and the bench says so.

---

## Phase 8 (US2 · exec order 7): The arm's-length answer

**Goal**: decide whether the outer loop needs new visibility into the inner loop — **before** the bake,
because a "yes" changes the input vector.

- [~] T060 ⚠️ **DEFERRED to specs/BACKLOG.md (operator 2026-08-31)** — [US2] Train a short **45-input baseline** against the ACRO model, launched per Constitution IX via `scripts/train.sh` (FR-030). **Operator-driven; ask first**
- [~] T061 [US2] Evaluate the FR-030 candidates against that baseline — ⭐ **rate-tracking error is the leading one**: the Gaussian attenuation means loop authority *falls* as commanded rate rises, so a large setpoint is tracked worse, and nothing in the current 45 reports it (research.md R2)
- [~] T062 [US2] ⛔ Record the verdict with its evidence in `specs/043-acro-dual-loop/arms-length.md` — ⭐ **including if the answer is "nothing"**, which is a genuine and cheap outcome (SC-011). ⛔ No input is added ahead of this measurement (FR-031)
- [~] T063 [US2] If and only if T062 says add: extend `include/autoc/nn/nn_inputs.h` and every consumer, and regenerate the xiao forward pass

**Checkpoint**: SC-011 answered. The input vector is final; the bake can start.

---

## Phase 9 (US4 · exec order 8): The production bake

**Goal**: one M1 trained against the rate-commanded plant.

- [X] T064 [US4] ⛔ Constitution IX **pre-run build gate**: clean build + relevant tests pass before committing compute; ⚠️ re-confirm the T008a regiment check (294 scenarios, population unchanged). **Operator-driven; ask first** ✅ **DONE 2026-08-30**: clean `rebuild-perf.sh` 50/50 suites, 0 failures, immediately before launch; regiment re-confirmed at 294.
- [X] T065 [US4] Launch the production M1 bake via `bash scripts/train.sh autoc.ini <unique-logfile>` — ⛔ detached, never via a harness background task (FR-060, Constitution IX). **Operator-driven** ✅ **LAUNCHED 2026-08-30 21:27** detached via `scripts/train.sh autoc.ini logs/autoc-043-t2-m1-acro.log`. Run `autoc-9223370248704297747-2026-08-31T04:27:58.060Z`, master seed 1788150478, pop 5000 × 800 gens × 294 scenarios. ⚠️ Trains against **arm C** (pitch maxRate 240) — see T051b.
- [X] T066 [US4] ⛔ Do **not** rebuild autoc while the run is in progress — overwriting `build/bin/autoc` breaks worker re-execs ✅ **RELEASED 2026-09-04** — the bake reached gen 800 and is complete (800 objects under the run prefix). Rebuilding is safe again.
- [X] T067 [US4] Monitor via `scripts/generate_pngs.sh m1 <log>` for the per-gen report set (⛔ not by hand-calling the analytics scripts — that loses the incremental S3 cache) ✅ Report packages generated at gens 115 / 178 / 248 via the wrapper; committed + pushed to `specs/043-acro-dual-loop/`. Evolution chart overlays 041-t7 + 038-t5.
- [X] T068 [US4] Pin the run `retain=keep` and write `specs/043-acro-dual-loop/artifacts/MANIFEST.md` with the S3 prefix, master seed, commits, `autoc.ini.as-run`, and ⛔ the **input scale constants** — without them the genome loads clean and flies wrong (FR-061, Constitution VIII) ⭐ **HALF DONE 2026-09-04**: ✅ `retain=keep` **VERIFIED** on `s3://autoc-m1/autoc-9223370248704297747-2026-08-31T04:27:58.060Z/` — 800 objects, sampled gen9999 / gen9900 / gen9500 / gen9201 / gen9200, all tagged `keep`, so the 30-day `retain=expire` lifecycle will not reach it. ⛔ **MANIFEST.md still unwritten** — and it is the last thing standing between a flown genome and an unreproducible one. Facts it must carry: prefix above · master seed **1788150478** · pop 5000 × 800 gens × 294 scenarios · gen-800 fitness **−88,013.84** (281/294) · commits `2d4b27a` (final PNGs) and `400b217` (rates 36/24) · `autoc.ini.as-run` · the input scale constants (`AccelScaleG = 8.0` and the rest of `nn_inputs.h`) · ⭐ **arm C** (`hb1_streamer.xml` pitch `maxRate="240"`, FC `pitch_rate 24`). ✅ **MANIFEST WRITTEN 2026-09-04** — [`artifacts/MANIFEST.md`](artifacts/MANIFEST.md), with [`artifacts/autoc.ini.as-run`](artifacts/autoc.ini.as-run) snapshotted alongside (⭐ `autoc.ini` is unchanged since `82e45f6`, 2026-08-25, i.e. **before** the launch, so the working copy *is* the as-run ini — sha256 `2f34f0054a72fc84…`). Carries: prefix · seed 1788150478 · 5000 × 800 × 294 · gen 800 = **−88,013.840878** sigma 0.051642 (⚠️ `nnextractor` labels it 800, `autoc` labels it 799 — match on fitness) · commits `c8d00ab` (autoc at launch) + **`ac4d796`** (crrcsim pointer, the arm-C model) + `400b217` + `2d4b27a` · all 11 input scale constants + `kNNHistoryLayoutVersion 3` · arm C parity table · the Phase-6/crash-rate deviations.

**Checkpoint**: a pinned, manifested M1 exists.

---

## Phase 10 (US4 · exec order 9): Flight and outcome

**Goal**: prove it in the air, and quantify by how much.

⭐ **FLIGHT-PREP ORDER (2026-09-04)** — T068 → T059 → flash → T069 → T070. T068 goes first because it is the
only step that becomes *impossible* later: once `nn_weights.dat` is overwritten, which run and generation
flew is unrecoverable. T059 before flashing because the working copy's generated net is the wrong topology
(see the task). ⚠️ **Known and accepted going in**: the t2 genome's crash rate ran **4–7%** against 041-t7's
**0.7%** (`specs/BACKLOG.md` § streak-outbids-crash) — the operator ruled it FLYABLE 2026-09-04, but it is
the thing to watch for in the air, and it argues for altitude in hand on the first engagement.

---

### ⭐ t3 CYCLE (2026-09-12) — READY FOR FLIGHT

The prep chain above was re-run end-to-end for **043-t3**, the crash-cost bake. ⛔ The t2 entries for T065 /
T068 / T059 / T069 are left checked as the **t2 record**; this block is the t3 pass.

⭐ **The "known and accepted going in" caveat above is RETIRED for t3.** t2 flew with a 4–7% crash rate;
**t3's gen-800 genome is 0/294 — zero crashes, 294/294 rabbitComplete**, and tier1 reproduces zero on a
novel seed. Altitude in hand is still good airmanship, but it is no longer compensating for a known defect.

- [X] **T065-t3** Bake launched 2026-09-07 19:02 via `bash scripts/train.sh autoc.ini logs/autoc-043-t3-m1-crashcost.log`, completed 2026-09-11 17:08 (3 d 22 h, 1.18 B sims). Master seed **1788832952**. gen 800: fitness **−88026.619367**, `pctInStreak` **54.6%**, crashes **0/294**. ⚠️ A first launch attempt at 19:00 died ~1 s in because `Xvfb :2` had died (workers failed `SDL_SetVideoMode`; surfaced as `TcpSocket::read recv: Connection reset by peer`) — it drew a *different* master seed, 1788832839, and produced no S3 prefix. Log preserved as `logs/autoc-043-t3-m1-crashcost.log.failed-noxvfb`.
- [X] **T068-t3** MANIFEST written **before** extraction → [artifacts-t3/MANIFEST.md](artifacts-t3/MANIFEST.md), with `autoc.ini.as-run`. S3 prefix pinned `retain=keep` on all **800** objects, verified by sampling **48/48**. ⚠️ `nn_weights.dat` held t2's flown genome (`3af8e3ab787b75a5`) and is **git-ignored** — backed up to `artifacts/nn_weights-t2-3af8e3ab.dat` before overwriting.
- [X] **T059-t3** Regenerated: `nn2cpp -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp`. Header verified — source `…2026-09-08T02:02:32.439Z/gen9200.dmp.zst`, topology `45 -> 32 -> 16r -> 3`, fitness −88026.619367, **zero** residual t2 references. Build gate **SUCCESS**: RAM 53.5% (127204/237568), Flash 45.5% (369388/811008). ⭐ Identity: `weight_id` **`a5d097da18bf33ed`**, `firmware_id` **`1e8342d6a38d55f6`**.
- [X] **Eval suite** `./scripts/eval_suite.sh nn_weights.dat all 1788832952` → `eval-results/2026-09-12T02:12:54Z/`. **tier0 BITWISE EXACT** (−88026.619367 == stored) ✅ · tier1 **294/294, 0 crashes** on a novel seed ✅ · tier2-progressive 49/49 · tier2-long 49/49 · tier3-quiet 1/1. ⚠️ tier2-random / tier3-stress 122/144 (84.7%) fail a 95% bar — **not a regression**: t2 scores 34.0% on the identical tier. Root cause is entry RANGE (`GenerateRandom` is the only generator that does not anchor `path[0]` at the origin ⇒ median 38 m vs the trained 0.3 m), i.e. an untrained cold-acquisition task. Full analysis in `specs/BACKLOG.md` § patrol/intercept → UPDATE 2026-09-11.
- [X] **`scripts/eval_suite.sh` repaired** — tiers 2/3 never restated `ExpectedScenarioCount`, so the FR-058 guard (added in `82e45f6`) aborted every regiment-changing tier. Five values added (49/49/144/144/1). ⛔ Those tiers had been dead since that commit; only tiers 0/1 ever ran.
- [X] **`autoc-eval.ini` drift fixed** — `OobCrashPenaltyWeight` 0.0 → **10.0** and `HullCrashPenaltyFactor` 0.5 → **0.75** to mirror the bake. Only the four intended eval differences remain (`EvaluateMode`, `PopulationSize`, `NumberOfGenerations`, `S3Bucket`).
- [X] **No INAV work for t3** ⭐ `rc_expo` 20 → 0 and `setpoint_kalman_enabled = OFF` were captured in `025ea0d` at **2026-09-07 13:44**, *before* the bake launched at 19:02 — t3 was baked against the aircraft as it now stands. No retune, no reflash (T052: CLI config only). ✅ **Rate-parity gate re-checked**: model `maxRate` 360/240 == FC `rates` 36/24, `control_profile 1` active with `rc_expo = 0`.
- [X] **flash the xiao** — ✅ proven by the 09-13 xiao log header: `firmware_id=1e8342d6a38d55f6 weight_id=a5d097da18bf33ed`, an exact match to the t3 MANIFEST.
- [X] **T069-t3** ⚠️ **Verified POST-HOC from the 09-13 flight logs — no pre-flight bench record exists** in `bench-notes.md` (bench verification #3 is the 09-05 t2 re-fly). Identity ✅ (xiao header above). Rate parity ✅ blackbox `H rates:36,24,4` vs model 360/240. ACRO purity ✅ 1856 engaged frames pure `ARM|MSPRCOVERRIDE` (flight-analysis-20260913 §5.1). arm-C `axisF[1]` saturation not checked. Original gate text: Bench-verify ⭐ flash identity must read `weight_id=a5d097da18bf33ed` (t2 was `3af8e3ab787b75a5`); re-assert rate parity after flashing; ACRO purity; arm-C `axisF[1]` saturation. ⚠️ **Watch-item**: t2 railed nose-down at 93% through the whole bench engagement (OOD saturation from static airspeed ≈0.2 vs 13 m/s trained). t3 took the pitch rail 33.5% → **0.0%** on the *cold-start* OOD, so it may look better — ⛔ but that is a **different** OOD axis, so a clean bench does not confirm the static-airspeed hole is closed.
- [X] **T070** Flown **2026-09-13** — see [flight-analysis-20260913.md](flight-analysis-20260913.md) + ADDENDUM. Continues in the T070–T077 chain below.

⭐ **What the t3 flight is testing**: t2 and t3 fly the *same* aircraft configuration, so this is a clean read on whether the better-aligned sim (latency 30 → 10 ms against the flight-measured 9.9, `rc_expo` 20 → 0, gyro Kalman off) produces a *smoother* result in the air. ⛔ **D1 remains unfixed** — sim pitch peaks at 85 ms vs 133–166 ms real — so if pitch specifically is still not smooth, D1 is the first suspect, not the ACRO work.

- [X] T069 [US4] Bench-verify the deployed firmware and INAV build before flying (FR-063) ⭐ Re-assert the **T051b rate-parity gate** on the flight article as part of this — FC `rates 36,24,3` against the model's `maxRate` 360/240 — since flashing is what could silently move it. ✅ **DONE 2026-09-04** — full bench flight cycle on the **flight article** with the flown genome; record in [bench-notes.md](bench-notes.md) § *bench verification #2*. Flash identity `weight_id=3af8e3ab787b75a5` = gen 800 ✅ · **parity re-asserted**: blackbox `H rates:36,24,4` vs model `maxRate` 360/240 ✅ · ACRO 325/341 ticks pure `ARM|MSPRCOVERRIDE`, the 16 impure ones contiguous from tick 0 = the 800 ms T057 priming window ✅ · arm-C `axisF[1]` max **+541** saturates ±500 ✅ · yaw `axisRate[2]`/`axisF[2]` exactly 0.0 ✅ · release + recordings clean · 341 ticks, 0 overruns/resyncs/drops · clock-join method rehearsed (+508 ppm, 341/341 matched). ⛔ **Surfaced, not a gate failure**: the policy railed **nose-down at 93% of max for the whole engagement** — the known OOD-saturation mode from [BACKLOG.md](../BACKLOG.md), provoked by a static bench (airspeed ≈ 0.2 vs 13 m/s trained). See the flight watch-items in bench-notes.
- [X] T084 [US4] ⭐ **Charge M1 for leaving the arena** — promoted from `specs/BACKLOG.md` at operator request 2026-09-07 (*"that should be part of this run, not a separate feature"*). ✅ **APPLIED to `autoc.ini`**: `EnableHullCrashPenalty` **0 → 1**, `HullCrashPenaltyFactor` **0.5 → 0.75**, `OobCrashPenaltyWeight` **0.0 → 2.0**. ⛔ **The gotcha that makes this a two-line change, not one**: `applyCrashPenalty()` returns early on `if (!c.enableHullCrashPenalty ...)` (`src/autoc.cc:276`), and **that gate covers the OOB branch as well** — setting `OobCrashPenaltyWeight` alone would have been a **silent no-op**. ⭐ Values are **M2's validated t14 triple**, not new numbers, per [feedback_clear_objectives_not_tuning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_clear_objectives_not_tuning.md) — this **states the envelope constraint**, it does not tune the streak multiplier down (that is doing its job: pctInStreak 54.6% vs t7's 51.3%). Penalty is `exp(−w·scale·K_oob/N)`: smooth, curriculum-ramped, never clamps to 0; at t2's measured 4–7% OOB rate that is a **7.7–13.1%** fitness hit. ⚠️ t2 had `hullStrike = 0` for all 800 gens so the hull term costs nothing today; the flag is there to unlock OOB. ⛔ **This changes the OBJECTIVE, so the next run's raw fitness is NOT comparable to t2's −88,013.84** — compare on crash rate, pctInStreak and per-axis measures. Tests: 50/50 green.
- [X] T084a [US4] ⭐ **Split the crash gates, and price OOB GENTLY** — operator 2026-09-07: *"M1 OOB is not as bad as hull crash — so we should not use the same ruleset for m1 or frankly for OOB … basically there is no hull crash in m1. we want some soft impact across the whole population — but gentle."*
  - ⛔ **Gate split** (`src/autoc.cc`): `EnableHullCrashPenalty` used to short-circuit the WHOLE `applyCrashPenalty()`, so `OobCrashPenaltyWeight` alone was a **silent no-op**. Now `hullOn` and `oobOn` are independent — hull by its flag, OOB by its own weight being > 0. **No new config key**, no protocol change.
  - ⭐ **`OobCrashPenaltyWeight = 1.0`, and the value is DEFINED, not tuned.** Marginal global cost of one egress is `1 − exp(−w/N)`; at `w = 1.0, N = 294` that is **0.340% = exactly one average scenario**. The rule states itself: **"you lose the scenario you busted, plus one more."** ⛔ Deliberately **not** M2's 2.0 — M2 prices a tracker where OOB approaches a hull strike in cost; for M1 an arena egress is a soft failure and wants a soft price.
  - **Works at both ends, by construction**: the existing `variationScale` ramp is **0.0 through gen 40** (while everything crashes) and reaches 1.0 only at **gen 761**. Late-run: 5% OOB → **×0.951**, 1% → **×0.990**. Gentle and proportionate, never clamping to 0.
  - ⭐ **Zero crashes → ×1.000 exactly**, so a clean genome is unpenalised and its fitness *is* "fairly comparable" with t2 — the operator's point.
  - `HullCrashPenaltyFactor` stays **0.75** and hull stays **enabled**: a ground strike *should* be severe. It is inert today (t2 recorded `hullStrike = 0` for all 800 gens) but is now expressed as its own rule rather than as OOB's gate.
  - ⚠️ **Watch under lexicase**: the penalty multiplies *every* case of a genome uniformly, so its selective bite depends on whether the shift clears the per-case MAD epsilon. If crash rate does not move, check that before raising `w` — see [project_lexicase_mad_epsilon](../../.claude/projects/-home-gmcnutt-autoc/memory/project_lexicase_mad_epsilon.md). Tests 50/50 green.
- [X] T084b [US4] ⛔ **CORRECTION — hull is a TRACKER concept, and OOB gets the operator's price.** Operator 2026-09-07: *"hull is hitting the chase in m2… exit arena should be oob — and the bottom of the cyl is still a hard deck, not the ground."* T084/T084a had the semantics wrong.
  - **What the reasons actually mean** (`crash_reason.h`): `HullStrike` = *chase intersected the **target** hull, **tracker-mode only*** — M1 has no target, so it is **structurally impossible**, not merely unobserved. `Eval` = arena egress **including the hard deck** at the cylinder bottom. `Sim` = a real ground impact — **and this function does not price it at all**.
  - ⇒ `EnableHullCrashPenalty` **1 → 0** for M1. Enabling it was a category error, not a conservative default. `OobCrashPenaltyWeight` is now **the only crash term M1 uses**.
  - ⭐ **`OobCrashPenaltyWeight = 10.0`**, from the operator's own framing — *"maybe 10 [scenarios]? e.g. 29/30 PER crash"*. Those are the same number: `−ln(29/30)·294 = 9.97`. Per-crash cost `1 − exp(−w/N)` = **3.34% = 29/30 = ~10 scenarios**.
  - **Endgame at the 0/1/2-crash goal**: ×1.000 / ×0.967 / ×0.934. At t2's measured 12–21 crashes: ×0.66–×0.49. ⇒ **Soft at the target, decisive away from it** — which is what the objective should say.
  - ⭐ **The MAD answer** (operator asked to think about it): epsilon is MAD-relative per case, and the recorded spreads are ~**0.3%** on a converged case vs ~**100%** on a hard one. The per-crash shift ramps 0.00% (gen ≤40) → 0.54% (gen 141) → 3.34% (gen 761). ⇒ It is **absorbed on hard cases and bites only on converged ones, from ~gen 140**. That is a good schedule by construction: exploration is free while the population is still learning to fly, and the constraint tightens on the cases it has already solved.
  - ⚠️ **Known limit of the uniform form**: because the shift *exceeds* epsilon on converged cases, it behaves there as a **threshold, not a gradient** — 1 crash and 3 crashes both fall outside epsilon. The graded price only expresses where MAD is larger. ⇒ If the run over-corrects into timidity, the lexicase-native fix is to penalise **only the crashed scenario's case** rather than every case uniformly; that is a bigger change and is not being made blind. Tests 50/50.
- [X] T070 [US4] ✅ **t3 flown 2026-09-13** (3 spans, 618 ticks, 0 gaps/overruns/resyncs/drops; t2 flown 09-05/09-06/09-07). Fly the baked genome; capture the xiao log **and** the blackbox recording (⚠️ the clock-join is required again — US1 dropped)
- [ ] T071 [US4] ⚠️ **PARTIAL** — 09-13 aligned via xiao `INAV_CLOCK` anchors (+~375 ms) and `mspOverrideFlags == 2` (agrees with xiao ENGAGE/DISENGAGE to ~1 s), but NOT fitted and cross-validated to 041's standard. Perform and cross-validate the blackbox clock-join as 041 did (−970 ppm fit, 0.5% against `ARM|MANUAL|MSPRCOVERRIDE`), in `specs/043-acro-dual-loop/flight-analysis.md` (FR-062)
- [X] T072 [US4] ✅ **DONE** — flight-analysis-20260913 §5.1/§5.6: pitch peak 2.32 / 2.09 Hz, **56.2 / 74.6%** in 2–3 Hz, **unchanged from 041-t7** (2.21/2.09/2.21/1.98 Hz, 37–69%) in frequency, band share and amplitude. `pitch_spectrum.py` is the reproducible instrument. Compute engaged-segment roll/pitch-rate power spectra and compare against the 041-t7 reference (3–5 Hz 30.1%, 5–10 Hz 7.4%) — ⚠️ **judged subjectively at first, deliberately** (SC-001)
- [ ] T073 [US4] ⚠️ **RESULT IN HAND, outcome.md NOT YET WRITTEN** — the quantified answer for pitch is a **null**: the 2–3 Hz oscillation survived the MANUAL→ACRO architecture change intact (§5.1), and the ADDENDUM A1 shows why — it is a plant resonance (D1) the sim lacks, not the phase budget 043 targeted. A null result is valid under SC-001a. Quantify **how much** the offload helped, on the same measures, in `specs/043-acro-dual-loop/outcome.md` — a null or small result is valid; an unquantified *"seems better"* is not (SC-001a)
- [ ] T074 [US4] ⚠️ **CONFOUNDED on 09-13** — ~80% of spans 1/3 tracking error is a one-sided ~25 m DOWNWIND bias (§2) from untrained wind speed (ADDENDUM A2/A3), so a comparison against calm-air t7 would measure wind, not tracking. Needs a calm-air flight. Verify tracking occupancy is within noise of the 041-t7 baseline using the same definition on both sides — ⚠️ compound with SC-001 (SC-003)
- [ ] T075 [US4] ⚠️ **PARTIAL** — throttle has CHANGED CHARACTER rather than regressed cleanly: near-continuous +rail 85–90% vs 09-05/06 rail-switching (§4), partly honest headwind response. `specific_energy` / `dist_to_boundary` not yet compared. Verify no regression on the channels 041 validated — throttle, `specific_energy`, `dist_to_boundary`, rate amplitude — distance-standardized (SC-005)
- [X] T076 [US4] ✅ **DONE** — 09-13: 0 gaps, 0 overruns, 0 resyncs, 0 drops across 618 ticks. Verify loop health: zero fetch failures, overruns, resyncs, tick gaps (SC-006)
- [X] T077 [US4] ✅ **DONE — pitch diverges, roll does not.** Pitch: flight 56–81% of rate power in 2–3 Hz vs sim median **10.7%**, flight pitch-rate RMS ≈ **2× sim** (flight-analysis-20260913 ADDENDUM A1); cause is the sim's missing short period (`actuator-pin.md` §7 — sim settles by ~175 ms, real rings at ~3 Hz). Roll: sim within ~17% gain / ~10% rise (`actuator-pin.md` §7). Report SC-002's sim-vs-flight divergence ⛔ **per-axis, not pooled** — pitch is where the open-loop stability question lives (research.md Finding 1a consequence 3)

---

## Phase 10b: t4 corrected bake (wind + D1), and a parallel prop-balance flight (filed 2026-09-13, RESCOPED 2026-10-03)

**Why**: the 09-13 flight + ADDENDUM surfaced three defects — **wind**, **pitch oscillation vs sim**, **Z**.
⭐ ✅ **IMPLEMENTATION STATUS 2026-10-03 (evening)**: T087/T087a/T087b/T088/T100 implemented, built, 52/52 tests, smoke-verified end to end; T090 gates (a)+(b) passed; T094 done; T100a resolved by decision. **Open before launch**: ~~T102~~ (✅ implemented 2026-10-06) + T106 acceptance, T091 (t4 MANIFEST + expectations), T093/T095/T096 (D1 fit — needs the prop-balance flight's held doublets), T092 (trainability gate once T096 lands), and the operator's sign-off on the realized thermal strength (5.4 m/s max). **Operator rescoping 2026-10-03**: *"the primary brittleness was probably the wind which I thought was in
there — so we do need a significant distribution of wind/gust and direction and thermals — to fit Baylands
(everything from calm to 15 kts on summer days)"* … *"maybe we don't dither the position/heading much this
round"* … *"maybe we DO attempt to fix D1."* ⇒ **t4 = wind envelope + D1 + a second random course (T100). Entry dither deferred.** ✅ **Operator 2026-10-03: tasks below are implementation-ready for t4.** Shear with height: *"I don't think the model needs shear for now"* — not in t4.

⭐ **Operator framing to carry into the outcome**: *"we are in far greater fine-tuning control — we do
something and see the intended results more often than not."* t3 is the proof — one priced constraint moved
the crash rate to zero exactly as intended. t4 and t5 are a fresh big bake each; fitness **lands in a different
realm** and that is expected, not a regression.

### ⭐ Sanity check of what the sim ALREADY does (2026-10-03, from source + the t3 training-table dmp)

| axis | present today? | what it actually is | verdict for Baylands 0–15 kt |
|---|---|---|---|
| **direction** | ✅ varies | base **330°** (`autoc_config.xml:100`), σ **45°** drawn once per scenario, constant within (`Global::windDirectionOffset`, applied per tick `windfield.cpp:~569`) | ✅ base matches the Baylands NW sea breeze; spread is generous. Keep. |
| **speed** | ❌ **fixed** | `velocity="12"` **ft/s = 3.66 m/s = 7.1 kt**, a single global read at **9 sites** via `cfg->wind->getVelocity()`, never per scenario; `T_Wind::setVelocity` stores **ft/s with no conversion** (`config.cpp:180`) | ⛔ **The gap.** One point near the low-middle of 0–7.7 m/s; never calm, never 15 kt. |
| **gusts** | ✅ live | MIL-HDBK-1797 **Dryden**, computed every FDM step (`crrc_fdm.cpp:72` → `windfield.cpp:866`); noise `eta1..4` are `RandGauss` objects **reset per scenario** (`initialize_gust()` after `CRRC_Random::reset(windSeed)`) ⇒ deterministic and varied | ⚠️ σ = **0.1·V_wind**, so with V fixed: σ_u 0.57 / σ_w 0.37 m/s, TI **always 15.7%**; corners **0.010 / 0.075 Hz** (slow drift). Scales automatically once speed varies; spectrum shape stays slow (T088). `setTurbulence()` exists → TI can be varied per scenario cheaply. |
| **thermals** | ✅ **in there** | `arena_thermals enabled="1"`: **0–5** cells in a 300×300 m box, strength **2.0 ± 0.5 m/s**, radius **20 ± 5 m**, lifetime 180 ± 60 s, drift with the wind; **re-spawned per scenario** from the reseeded `CRRC_Random` (`SimStateHandler::reset(windSeed)` → `Init_mod_windfield()` → `spawnThermals()`), so placement varies deterministically | ⚠️ Exposure is **low** — only **~10%** of t3 scenarios met one (max 2.8 m/s vertical). Summer Baylands afternoons are thermic; raise count/strength or add a day-type mixture (T087b). |
| **shear with height** | ⚠️ none aloft | 1/7-power boundary layer only within **10 m** of terrain (`windfield.cpp:623`); the 25–105 m band flies `fact = 1.0` — which is why the dmp shows **3.66 on every tick** | Minor realism gap; real sea breeze strengthens with height above 10 m. Note, do not fix in t4. |
| **determinism contract** | ✅ | per-scenario `windSimSeed = windPRNG.next()` from `subseeds.wind`; `kDisabledWindSeed` when `EnableWindVariations=0` (`inputdev_autoc.cpp:~650`); worker receives `evalData.variationScale` per eval (`protocol.h:335`) | ✅ New draws must be **draw-and-discard** when disabled and **ramped by `variationScale`** like everything else. |
| **D1 model files** | ✅ T094 done | `hb1_streamer_steptest.xml` pitch aero **identical** to `hb1_streamer.xml`: `Cm_0=0.015 Cm_a=-0.55 Cm_q=-4.2 Cm_de=-0.36`; only diff is StepTest-controller-in / `InavFwRate`-out | ✅ The step test is a valid proxy for the trained plant. ⭐ History in the file: `Cm_q` −3.6 (orig) → −5.5 ("way too sluggish") → **−4.2**, tuned on **steady pitch-rate gain**, never on the transient — the ringing was never a fitting target. The real aircraft rings ⇒ the fix moves damping **down** / static margin & inertia re-fit, i.e. toward or past the original −3.6 (T096). |

⚠️ **Known limit carried into t4, not fixed**: the `AIRSPEED` input is **groundspeed on both sides**
(`msplink.cpp:1144` / `inputdev_autoc.cpp:897`). In a 0–8 m/s wind distribution the policy has **no airspeed
signal** — it must infer wind from `Es`/closure. This caps what t4 can learn about crabbing and is the
pitot's case (`BACKLOG.md` § Pitot tube). Record it in the t4 MANIFEST at launch.

### t4 — wind envelope (PRIMARY)

- [X] T087 [t4] ✅ **IMPLEMENTED 2026-10-03 (route B)** — `include/autoc/eval/wind_variation.h` (pure, tested) + `WorkerInit.windVariation` (RPC-only, appended) + `inputdev_autoc.cpp` applies `cfg->wind->setVelocity()` per scenario BEFORE `Simulation->reset()`; 4 draws appended after `drawnWindSeed`. **Smoke (t3 genome, t4 cfg, Seed 4242)**: per-scenario median wind min **0.04** / p10 0.68 / median 3.75 / p90 6.86 / max **7.93 m/s**, **252 distinct speeds** (t3: 3.66 on all 294); direction spread 0–300°; t3 genome still **293/294 OK**. ⭐ **Wind SPEED variation class — Baylands envelope 0 → ~7.7 m/s (0–15 kt), tail to ~9.** ⛔ **No wire change**: do NOT add to `ScenarioMetadata` (serialized into every dmp; orphans t2/t3 and breaks the MANIFEST extract commands). Two no-break routes, pick one:
  - **(B, recommended)** **crrcsim-side**: draw `windSpeedScale` from the scenario's **wind subseed** (one more `windPRNG.next()` / ClassPRNG draw next to `drawnWindSeed`, `inputdev_autoc.cpp:~650`), ramp it with `evalData.variationScale`, and apply **`cfg->wind->setVelocity(base_ftps × scale)` BEFORE `Global::Simulation->reset(seed)`** so thermal drift, scenery wind and Dryden σ all see the new speed through the existing 9 read sites — no per-site edits. Same precedent as the thermal/gust seeding (033/034). Cache `base_ftps` once; set absolutely every scenario (never accumulate). **Draw-and-discard** when `enableWindVariations=0` (force scale 1.0 — the `kDisabledWindSeed` path must not produce a random fixed scale).
  - **(A)** autoc-side draw shipped per scenario in the RPC-only `WorkerInit` (040 US6 `cameraVariations` precedent, `protocol.h:~170`).
  - Auditability survives either way: realized steady wind is in the dmp per tick (`wN/wE/wD`).
  - Distribution — ✅ **operator 2026-10-03: uniform 0–8 m/s.** A day-type mixture (calm-morning / sea-breeze-afternoon) is T087b.
  - **Primary heading** — operator: *"some variation is good, don't go crazy."* Today σ 45° about 330° (2.5σ clamp ⇒ ±112°, already most of the circle). Proposal: widen to **σ 60°** (2.5σ = 150°) and keep the 330° base so the Baylands NW prior survives; **uniform 0–360°** is the alternative if a wind-heading prior is unwanted. Direction stays constant within a scenario (slow veer is not needed once T088 shortens the gust timescale).
  - **Gust heading** — ✅ already isotropic in the horizontal: Dryden computes u/v/w in a wind-aligned frame with **σ_v = σ_u** (`windfield.cpp:~905`) and rotates to body, so crosswind gusts equal along-wind gusts by construction. Nothing to add; the weakness is **timescale**, not direction → T088.
  - ⛔ **Historical reproducibility**: every new draw must be gated by a **new ini key defaulting OFF** (e.g. `WindSpeedVariationMax = 0`) and must be consumed **after** the existing `drawnWindSeed` in the wind-subseed sequence, so that with the key off the thermal/gust seeds are unchanged and **t3's tier0 still reproduces bitwise** under the new binary. Same precedent as the lexicase-epsilon switch ([project_lexicase_mad_epsilon](../../.claude/projects/-home-gmcnutt-autoc/memory/project_lexicase_mad_epsilon.md): *keep an ini switch for historical reproducibility*).
- [X] T087a [t4] ✅ **IMPLEMENTED** (`WindTurbIntensity{Min,Max}` 0.5–2.0 → `setTurbulence()` per scenario). **Turbulence intensity per scenario** — second draw from the same subseed, `cfg->wind->setTurbulence(base × tiScale)`, proposal **0.5–2.0×** (TI 8–31%), ramped and draw-and-discard like T087. Cheap once T087 exists; gives gusty vs smooth days at the same mean wind.
- [X] T087b [t4] ✅ **IMPLEMENTED** as per-scenario `ThermalStrengthScale{Min,Max}` 0.5–2.0 + `ThermalCountMax` 8 applied in `initialize_arena_thermals` (XML re-read every reset ⇒ never accumulates; count ramped XML→target by `variationScale`). **Smoke**: **47%** of scenarios meet a thermal (t3 ~10%), max vertical **5.45 m/s** ✅ **Operator 2026-10-03: capped `ThermalStrengthScaleMax` 2.0 → 1.5** (both inis); ramp-contrast smoke max vertical 3.13 m/s. **Thermal distribution for summer Baylands.** Today ~10% exposure. Options: (i) config-only bump in `autoc_config.xml` `arena_thermals` — `count_max` 5 → 8 (`MAX_THERMALS` = 8), `strength_mean` 2.0 → ~2.5 with σ ~0.8 (1–4 m/s), keep radius 20 ± 5; (ii) per-scenario thermal-strength scale tied to the T087 day draw (calm/warm ⇒ stronger thermals). ⭐ Recommend (i) for t4; (ii) with T087b-mixture later. ⚠️ Thermals drift at wind speed — at 8 m/s a 20 m cell crosses the arena in ~18 s; lifetime 180 s is fine.
- [X] T088 [t4] ✅ **IMPLEMENTED** (`WindGustLengthScale{Min,Max}` 0.25–1.0 → `Global::gustLengthScale` × L_u/L_v/L_w in `calculate_gust`). **Gust timescale — per-scenario Dryden length-scale factor (light touch).** Corners today 0.010 Hz horizontal / 0.075 Hz vertical (L_u 211 m, L_w 28 m at 55 m AGL): at 13 m/s a horizontal gust is effectively a **constant offset over a 20–40 s scenario**, so within-scenario variation is weak regardless of heading. ⭐ Cheapest honest fix: a per-scenario factor **0.25–1.0×** on L_u/L_v/L_w in `calculate_gust` (`windfield.cpp:~896`), drawn from the wind subseed, ramped, gated OFF by default. 0.25× puts the horizontal corner at ~0.04 Hz (25 s) — gusts that *change* during a scenario, still far from "crazy". Discrete gust events stay out of t4.
- [X] T090 [t4] ✅ **PASSED 2026-10-03** — (a) `nnextractor -k …t3… -g 800` under the new binary ⇒ sha256 **`a5d097da18bf33ed`**, fitness −88026.619367 ✓; (b) t3 **as-run ini** (no new keys, `WindDirectionSigma 45`) through the new binary ⇒ **`NN_EVAL_SAME: fitness=-88026.619367`**, 294/294, slot 3 resolved `fortyFive` ✓. Full ctest **52/52** incl. new `wind_variation_tests` (7) and `aero_path3_tests` (4); `arena_path_fit_tests` 6/6. ⭐ **Re-run on the CLEAN PERF build** (`scripts/rebuild-perf.sh`, operator-requested, `PERFORMANCE_BUILD=ON`, 2026-10-03 20:18): ctest `100% tests passed, 0 tests failed out of 52`; t3 as-run repro ⇒ `NN_EVAL_SAME: fitness=-88026.619367` ✓. **Ramp-contrast TRAINING smoke** (pop 24, 2 gens, `VariationRampStep=1`, S3→autoc-eval, Seed 4242): **gen 1 rampSc 0.00** → wind median-speed 2.54–5.33 m/s (base 3.66 + thermal inflow), 60 distinct, thermals 13%; **gen 2 rampSc 1.00** → **0.04–8.31 m/s, 250 distinct**, thermals **25%** (max 3.13 m/s with the 1.5 cap), slot 3 at 19.8 m both gens. ⇒ the envelope is gated by the ramp exactly as every other class, and realized at full scale once the ramp reaches 1.0. ⚠️ **Footgun found and fixed**: inih strips only `;` inline comments — a trailing `# …` on a STRING key (`AeroStandardPath3`) became part of the value and the fail-loud parse aborted (correctly). Comments moved off string-key lines in both inis + a defensive trim in `autoc.cc`; numeric keys only tolerate `#` because `atof` stops early. **Schema-safety regression gate** after T087–T087b: `nnextractor -k autoc-9223370248021823368-2026-09-08T02:02:32.439Z -g 800` must reproduce sha256 `a5d097da18bf33ed`, and t2's prefix must still extract. Plus the determinism gate: same `scenarioSeed` ⇒ bit-identical wind/thermal/gust with the new draws on; all new keys OFF ⇒ **bit-identical to today**, which must be proven by re-running **t3's tier0 with the new binary and the t3 eval ini** → `−88026.619367` exactly. ⚠️ That also requires T100's path switch set to the **old** path 3 in t3's eval ini — the path set is code, and a changed path 3 would change t3's fitness without touching a single dmp.
- [ ] T091 [t4] **Record expectations BEFORE launch** (t4 MANIFEST). Raw fitness **will drop** — crosswind holding at up to 8 m/s against a 12 m/s rabbit, plus D1's re-fit plant. ⛔ Do NOT compare to t3's −88,026.62. Judge on: crash rate (t3: 0); `pctInStreak` on calm scenarios vs windy ones (**bin by realized wind from `wN/wE`**); downwind tracking bias (09-13: ~25 m one-sided); cold-start completion vs t3's 84.7%. ⚠️ **Ground truth for wind at the t4 flight**: INAV's estimate is heavily filtered — take a handheld anemometer reading at the field (pitot is backlog).

### t4 — tempered control: the grease (operator 2026-10-05; derivation in [tempered-control-first-principles.md](tempered-control-first-principles.md))

⭐ The vibration study reframed the 2–3 Hz pitch cycle as a **±7 g loading event** (vibration-analysis-20261004 § 6.4) and it is **commanded** (t3 § 5.3). First-principles finding: the dual loop offloaded rate *tracking* (works — delivered fraction 0.5–0.7, pitch rail 13% → 0.3%) but **nothing in the chain shapes the setpoint** (xiao → MSP unfiltered, sim mirrors, INAV `rate_dynamics`/rc-filter at pass-through, FF-dominant loop reproduces a 20 Hz staircase at `kFF`), and **nothing in the objective asks for a smooth command** — the only pitch/roll smoothness case has been OFF since 035 FR-008 and measured amplitude, not rate. Sim command stream is itself a near random walk (sign-flip 40–47%, |n_z| p99 4.3 g, un-priced); flight is ~2× rougher on top (D1). 023's pt3-filter experiment stunted training and concluded *"a fitness-based smoothness incentive is the better path"* — never executed.

- [X] T102 [t4] ✅ **IMPLEMENTED 2026-10-06** — `include/autoc/eval/excess_rotation.h` (pure accumulator, ‖ω_body‖·dt vs ∠(tangent_t, tangent_{t−1})), `ScenarioScore::{excess,craft,path}_rotation`, third co-equal lexicase case gated by `EnableExcessRotationAxis` (default 0; t4 = 1 in both inis), `#NNGen excessRot= rotRatio=` per gen, eval lines `exRot= rotRatio=`, `dmp-dump` run-summary `mean_excess_rot,rot_ratio` appended. **Evidence**: ctest **53/53** (5 new `excess_rotation_tests` incl. hard-turn-free / wandering-charged / pegged-not-smooth; 3 new `Selection043` incl. `RotationTradeoffBothSurvive`); **T090(b) on the new binary `NN_EVAL_SAME −88026.619367`** with the key absent. ⭐ **Baseline from the real code path** (t3 genome, t4 regiment, Seed 1788832952): excess median **47.8 rad**, **rotRatio median 5.10×** — Straight **6.09×** / Spiral 4.57× / Fig-8 4.03× / RandomA 5.10× / HighPerch 6.92× / RandomB 4.87×; 0 crashes. (Offline |p|+|q| estimate was 4.9×; the implemented ‖ω‖ form is the number of record.) Original scope — ⭐ **Excess-rotation lexicase axis — the craft's rotation over what the path demands** (REVISED 2026-10-05, operator: *"param tuning is something to avoid … a smooth line should fly smooth, an abrupt turn should turn hard … the measure is more about wandering around paths"*). `excess_rotation = Σ_t (|p|+|q|)·dt − Σ_t ∠(tangent_t, tangent_{t−1})` per scenario, lower = better; **no tunable parameter**. Third co-equal case (pool: score, energy, excess_rotation), MAD epsilon, present from gen 0, gate `EnableExcessRotationAxis` default **0** (t3 tier0 bitwise; t4 = 1). ⭐ Measured on t3's table: craft rotates **4.9×** the path demand (median; Straight 5.4× / Spiral 4.5× / Fig-8 3.9× / 45° loop 3.1× / HighPerch 6.7× / RandomB 5.5×) — the straight line is among the worst, the hard turn the best: wandering, exactly the signature; excess median 37.7 rad with **40% MAD**; corr with |n_z| p99 **+0.69**. Plumbing: inside the existing per-tick loop in `fitness_decomposition.cc` the path `tangent`/`prevTangent` is already derived (pathgen: consecutive path points; tracker: `target.velocity` — so **the same axis carries to M2**, and a bang-bang target just raises the demand), `getGyroRates()` is on the record; one accumulator, `ScenarioScore::excess_rotation` (post-hoc, not in the dmp — schema-safe), `selection.cc` `pool.push_back`, tests (`Selection027` pattern; accumulator unit test with a synthetic straight path ⇒ excess = craft rotation; OFF ⇒ pool unchanged). ⚠️ Not throttle-gameable, not Δ=0-gameable (a pegged elevator rotates). ⚠️ Baseline honesty: roll-in/roll-out means a perfect pilot is not 1.0× either — it is a relative measure, which is what lexicase wants. **Sibling**: excess load `Σ max(0,|n_z| − n_req)`, `n_req = √(1+(v²κ/g)²)` (corr +0.75), if loads stay high while rotation falls. ⚠️ Score the **t3 genome under the new pool once** and record it in the t4 MANIFEST as the baseline.
- [ ] T106 [t4] **"Tempered, not dull" acceptance, stated before launch** (doc § 8): sim — `pctInStreak`/target distance within noise of t3 on calm bins, crash ≤ t3, **`rotRatio` median < 5.10×** (t3 under the t4 regiment; and the straight path no longer the worst at 6.09×), |n_z| p99 < 4.30 g and ticks > 3 g < 12.6% as consequences, command flip % falling (watch), pitch 2–3 Hz share stays ~10%; flight — pitch 2–3 Hz share ≪ 56–81%, `|accSmooth[2]|` p99 < 5.2 g, flip % < 52%, flown `pctInStreak` ≥ 54.6%. Rate-fair signals only (037 caveat on per-tick dctrl).
- [ ] T103 [t5, only if T102 leaves a command floor] **Command-rate axis WITH its saturation companion** (the 2025-11 `CONTROL_RATE_PENALTY` + `CONTROL_SATURATION_PENALTY` pairing, now as lexicase cases) **or path-relative Δu** (015's revisit note; BACKLOG "Path-Relative Smoothness"). ⛔ Never the bare `Σ|Δu|` form — 015 exploit on record.
- [ ] T104 [t5, only if T102 leaves a floor] **Setpoint slew limit in the action space** — identical function on xiao and in `inputdev_autoc` (parity by construction), generous bound. ⛔ 023 Phase 9a pt3 LPF stunted training (−2225 vs −4410 at gen 55); a slew limit is milder but the warning stands — fitness first. INAV's `rate_dynamics` weight is the native analogue and is **not modelled in sim** (FR-012a).
- [X] T105 ⛔ **REJECTED 2026-10-05 (operator: "that's cheating")** — path lookahead for M1. 029's reason stands: the tracker has no oracle, so a future input is fictional and would teach M1 a crutch M2 cannot have. "Further ahead" comes from the recurrent state and the formal M2 predictor line, not an input. Recorded so it is not re-proposed.

### t4 — D1, folded in (operator 2026-10-03)

⭐ ADDENDUM A1: the 2.1 Hz oscillation is **absent in the sim** (median 10.7% vs flight 56–81% in 2–3 Hz) ⇒ not the RNN, not the tick. `actuator-pin.md` §7: the sim has **no pitch short period** — settles by ~175 ms where the real aircraft rings at 3.01 → 3.77 Hz. ⛔ **Missing a lightly damped resonance, not delay — do not add delay.**

- [ ] T093 [D1] **Tool gap**: per-cell mode on the sim side of `step_response.py` — pooled averaging mixes trim datums and polarities and returns nonsense for pitch (16.7 Hz).
- [X] T094 [D1] ✅ **DONE 2026-10-03** — `hb1_streamer_steptest.xml` pitch aero equals `hb1_streamer.xml` (see sanity table). The step test is a valid proxy for the trained plant.
- [ ] T095 [D1] **Real reference with controlled holds** — MANUAL held pitch steps / doublets at **500 Hz**. ⭐ **Fly these on the upcoming prop-balance flight** (T098): it is the "prior s/w", which is exactly what is wanted — the FDM fit needs the airframe, not the policy. Requires `blackbox_rate_denom 4` + `save` + header read-back (`H P interval:1/4`) — lost two flights running.
- [ ] T096 [D1] **Fit `Cm_q` / `Cm_alpha` / pitch inertia** so the sim rings at ~3 Hz, peaks **133–166 ms**, frequency **rising with airspeed**, overshoot ~2.5×. ⭐ `CraftCmQSigma = 0.32` (centre −4.2, clamp [−5.0, −3.6]) must be **recentred and re-clamped** on the fitted value. Validate with T093, per-cell. ⚠️ n = 1 article with a known-asymmetric wing — record that the fit is to *this* airframe.
- [X] T100 [t4] ✅ **IMPLEMENTED 2026-10-03** — `AeroStandardPath3 = randomA` (default `fortyFive`), `RandomPathSeedA = 12345`, slot-3 body duplicates the SeededRandomB generator with seed A (`pathgen.h`), storage in `pathgen.cc`, parsed fail-loud in `autoc.cc`; xiao mirror `EMBEDDED_PATH3_RANDOM_A 1` + `appendSeededRandom()` helper, pio gate SUCCESS (RAM 53.5% / Flash 45.5%). **Smoke**: slot 3 now starts **19.8 m** from the rabbit, **45.2 s** (was 7.7 s loop); slot 5 unchanged 37.6 m / 41 s. ⭐ **Second random course: replace path 3 `FortyFiveDegreeAngledLoop` with `SeededRandomA`.** Operator 2026-10-03: *"remove the right 45 loop and replace it with a second random … 1/3 of the courses have random entry — and the little right 45 was too short anyway."* ✅ **Confirmed too short**: median **7.7 s** (153 ticks) vs 16.6 / 21.2 / 21.2 / 25.6 s for paths 0/1/2/4 and **40.8 s** for `SeededRandomB` — a fifth of the random path, half the next-shortest. Mechanics: `SeededRandomA` + `RandomPathSeedA` existed (added `1e0da14` 2025-12-23, removed `3366afd` 2025-12-24, seeds then **hardcoded** 12345/67890 — do NOT restore that form). Restore as a proper ini key plumbed like `RandomPathSeedB`/`gPathSeed`, same `localRandomPointInCylinder` + Catmull-Rom generator, enum slot 3 so every path index above is unchanged. `ExpectedScenarioCount` stays **294**. ⚠️ **Weighting consequence, stated now**: under lexicase each scenario is one case, so random-entry *selection pressure* is 2/6 = 1/3 — but by **ticks** the two random paths are ~82 s of ~127 s per wind ≈ **65%**, so every per-tick aggregate (`pctInStreak`, energy, stability) becomes random-course-dominated. Bin reports by path. ⭐ **Make the path-3 choice an ini switch** (e.g. `AeroStandardPath3 = fortyFive | randomA`, default the historical value) so t3's eval still reproduces (T090).
- [X] T100a [t4] ✅ **RESOLVED BY DECISION 2026-10-03 — the flight MAY fly a different random path than the sim trained.** Finding: `xiao/src/msplink.cpp:808` generates paths from `EMBEDDED_PATH_SEED` = **67890** while `autoc.ini` has `RandomPathSeedB = 13337` (since ≥ 2026-05-31, never mirrored); bounds parity holds (shared `aircraft_state.h`) so the seed is the only difference, and it is sufficient — random slot 5 (and now slot 3) in the air is a different course than in training. **Operator**: *"it is ok that flight is a different seed — in fact one could argue at some point a unique path on every activation for the field."* ⇒ **Not a defect; a generalization test by design.** Consequences recorded: (1) any sim↔flight comparison on a random slot must regenerate the flown course from the *flown* seed, not assume parity; (2) the xiao mirror keeps the path **type** per slot aligned (slot 3 = random when `AeroStandardPath3 = randomA`) so indices mean the same thing on both sides, while seeds are free to differ; (3) ⭐ **BACKLOG candidate**: a fresh seed per activation in the field (e.g. from the xiao clock at engage), logged in the flight-log header so analysis can regenerate the course.
- [ ] T101 [t5+] **If t4 trains well on 2/6 random: go to 6 random courses, slightly shorter than today** (operator). The lever is the random generator's segment count / control-point count (today ~41 s vs 17–26 s for the structured paths); target ~25–30 s each. Decide after the t4 flight, not before.
- [ ] T092 [t4] ⛔ **MANDATORY once T096 lands**: re-run the **T044 trainability gate** against the re-fit plant before the bake (T050a discipline). A plant that rings may need the smoothness/rate terms re-checked.

### Deferred from t4 (operator 2026-10-03) — entry geometry

Operator: *"we do have true random entry on the random path — and the other shorter paths are more or less
tail entry — maybe we don't dither the position/heading much this round."* ⭐ Recorded precisely so the record
is honest: path 5 (`SeededRandomB`) gives a random **path geometry** and a random entry **attitude** (cone σ 18°,
roll σ 30°, speed σ 6%), but with `RandomPathSeedB = 13337` it is **one fixed path** and the aircraft always
starts **~38 m** from the rabbit (49/49 scenarios, rabbit-start spread ~0.09 m). Paths 0–4 are latched tail
entries at 0.1–1.1 m. That is **accepted coverage for t4**; the items below are kept for t5+.

- [ ] T085 [t5+] Entry POSITION dither (`EntryPositionRadiusSigma` / `EntryPositionAltSigma`, both 0.0) — plumbing exists end to end and is on the wire already; dormant since spec 005, **no test**; two generation sites (`variation_generator.h:113`, `:308`); `crrc_main.cpp:258` sign note; keep 2.5σ clear of the 25 m floor and inside the 70 m cylinder.
- [ ] T086 [t5+] Decouple entry HEADING from the pitch cone (today capped at 45° — can never present the tail-away entry that crashed 72% in cold-start eval); `entryHeadingOffset` already serialized. Full-sphere attitude needs quaternion init in the FDM — out of scope.
- [ ] T089 [t5+] More random paths (per-scenario `RandomPathSeedB`) needs a per-scenario path library — only if cold-start eval stays weak after entry dither.

### Parallel flight track — prior s/w (t3 genome) with a balanced prop

- [ ] T098 ⭐ **Balance the prop, then fly the t3 genome** (operator: *"another flight on the prior s/w with a better balanced prop"*). Re-run the `accVib`-vs-motor table as the check. ⭐ **Discriminating experiment**: flight-analysis §3 predicts the Z divergence disappears with vibration down; ADDENDUM A4 predicts it **persists** in ACRO-engaged flight. Either outcome settles it.
- [ ] T097 **Instrument the flights — SPLIT 2026-10-04 (operator: flash write bandwidth is tighter than the 28 s 09-07 logs suggest; a vibration-only sortie does not engage autoc, so log the minimum).**
  - ⭐ **Sortie A — prop balance, autoc OFF, minimal fields**: `blackbox_rate_denom 4`; turn OFF `NAV_POS`, `MAG`, `RC_DATA`, `QUAT`; keep `GYRO_RAW`, `ACC`, `MOTORS`; turn ON `PEAKS_R/P/Y`; `debug_mode = VIBE`; `save` + header read-back; `flash_info` + erase first. Answers **T098 prop balance** (`gyroPeak*` + `accVib` vs motor, before/after) and gives the vibration→`accWeightFactor` curve under pilot flying. **Optional D1 (T095)**: keep `RC_COMMAND` + `SERVOS` and fly MANUAL held pitch doublets. ⛔ Does NOT test the Z mechanism (needs NN-engaged flight).
  - **Sortie B — later, autoc ON**: add back `NAV_ACC`, `ATTI`/`QUAT`, keep `VIBE` — the Z-mechanism discriminator (`accWeightFactor` during engaged spans vs `|navPos[2] − baro|`), plus the >30 Hz question on engaged pitch.
  Full field rationale (2026-10-04, verified against the fork's `blackbox.c`; config of record is `xiao/inav-hb1.cfg`, there is no `xiao/hb1.cfg`):
  - `set blackbox_rate_denom = 4` → **500 Hz**. ✅ Empirically fine: the 09-07 logs at 1/4 sustained **481 Hz at 18.9 kB/s with 0.01% frames missing** (vs 59 Hz / 8.4 kB/s at 1/32) ⇒ ~**11 MB per 10 min**. `1/2` (1 kHz, ~38 kB/s) is affordable if flash allows and you want 500 Hz Nyquist on `gyroRaw` for prop harmonics. ⛔ Two flights have lost this to an unsaved CLI change — **`save`, power-cycle, then confirm `H P interval:1/4` in the header**.
  - `blackbox ATTI` (Euler `attitude[0..2]`, currently **off**; `QUAT` is already on — keep both), `blackbox NAV_ACC` (`navAcc[0..2]`, currently off), `blackbox PEAKS_R` / `PEAKS_P` / `PEAKS_Y` (dynamic-notch tracked peak frequency per axis, currently off — the notch runs at 2 kHz so this reads the prop's noise frequency independent of the logging rate: **the before/after prop-balance instrument**).
  - `set debug_mode = VIBE` → `debug[0..2]` per-axis vibe levels ×100, `debug[3]` accel clip count, **`debug[4]` `accWeightFactor`×1000, `debug[5]` `accWeightScaled`×1000** — the estimator's trust in the accelerometer, per tick (`imu.c:849`, `navigation_pos_estimator.c:367`). ⭐ This is the Z-mechanism discriminator (ADDENDUM A4 correction). Debug fields are only written when `debug_mode != NONE` (`blackbox.c:782`); this fork logs 8 slots.
  - Keep `GYRO_RAW` on (pre-`gyro_main_lpf` 25 Hz — `gyroADC` is useless above ~25 Hz for vibration). ⛔ Do **not** touch filters (`gyro_main_lpf_hz`, `acc_lpf_hz`, notch) for this flight — FR-012a: any filter change must land identically in the sim first.
  - Bench: `flash_info` (the MINI uses the auto-detecting `M25P16` driver — capacity is whatever it reports), **erase** before the sortie, and verify the first few seconds of a bench log decode with the new fields present.
  - Xiao side: nothing to change — v5 log already carries `quat_w..z`, `gyro_*`, `out_*` at 20 Hz.
  Original scope — (1) `blackbox_rate_denom 4` **and `save` on the bench**, header read back; (2) **log attitude and `navAcc`** — no attitude trace is what blocked the Z mechanism; (3) **MANUAL held pitch doublets = T095** (D1's damping ratio); (4) ACRO engaged spans for Z and the >30 Hz question. A calm day additionally rescues T074.
- [ ] T099 → **BACKLOG**: pitot tube (early-days hardware) and NN pathology handling (post-M2). Filed in `specs/BACKLOG.md` § 043 deferrals.

---

## Phase 11: Polish and cross-cutting

- [ ] T078 [P] Run the Constitution VI type-domain grep on the touched paths; annotate `// raw-ok:` or convert. ⛔ No milestone is done with unannotated raw `float`/`double` in its diff
- [ ] T079 [P] Complete the FR-059 **observability audit** — mark every variation axis observable, absorbed, or observable-only-at-limits, with the measurement behind it, in `specs/043-acro-dual-loop/variation-inventory.md`. ⚠️ Verdicts are **sim verdicts, provisional on the flight** (SC-013)
- [ ] T080 [P] Check FR-059a — does the regiment actually reach the aero/power limits where the buried axes reappear? ⭐ The mechanism is **hold-attitude-and-descend**, not rate saturation
- [ ] T081 Write `specs/043-acro-dual-loop/outcome.md` — the result, what caused it, what did not, and every deferral
- [ ] T082 Append deferrals to `specs/BACKLOG.md` in order (Constitution X)
- [ ] T083 Update `CLAUDE.md` § Active feature to point at whatever follows 043

---

## Dependencies

```
Phase 1 (setup)
   └─> Phase 2 (T004/T005 ⛔ IRREVERSIBLE GATE, T006)
          └─> Phase 3 (US5 variations)  ─┐
          └─> Phase 4 (US6 housekeeping) ┤ both inside the format-break window
                 └─> T027 clean rebuild ─┘
                        └─> Phase 5 (US2 model, ⛔ T037a 2nd clean rebuild)
                                 └─> Phase 6 (US2 plant pin, T050a re-gate)
                                                        └─> Phase 7 (US3 stack + bench)
                                                               └─> Phase 8 (US2 arm's length)
                                                                      └─> Phase 9 (bake)
                                                                             └─> Phase 10 (flight)
                                                                                    └─> Phase 11
```

⛔ **Hard ordering constraints**

| constraint | why |
|---|---|
| **T004/T005 before T011** | the format break orphans the pinned baseline. **Permanent if violated.** |
| **T027 after T021 + T021a** | both edit `CMakeLists.txt`; a clean `rebuild-perf.sh` is required, and T021a is what makes its 49-suite self-check pass (Constitution IV) |
| ⛔ **T037a after T036** | ⭐ a **SECOND** mandatory clean rebuild — T036 adds a target to `mod_cntrl/CMakeLists.txt`. T027 does not cover it (Constitution IV) |
| **T019a after T011** | the fail-loud test needs the format break to have happened |
| **T050a after T046–T050** | re-runs the trainability gate if the plant changed under it |
| **T043 before T044** | verify the model standing alone before letting a GA train against it |
| **T044 before T065** | ⛔ the trainability gate must fire *before* 27 h of compute |
| **T008a before T065** | the 294-scenario regiment check must hold at bake time (FR-058) |
| **T062 before T064** | a "yes" changes the input vector; the bake needs it final |
| **T006 before T039** | the notch measurement decides whether it is modelled |

## Parallel opportunities

- **Phase 1**: T002, T003 with T001
- **Phase 2**: T006 alongside T004/T005
- **Phase 3 tests**: T017–T020 (incl. T019a) together once T010–T016 land
- **Phase 4**: T021, T022 together; T021a after T021 (same file); T023/T024 after (both touch `nn_inputs.h`)
- **Phase 5 tests**: T028–T033a written together, before T034–T042
- **Phase 11**: T078, T079, T080 together

## Implementation strategy

⛔ **This feature has no MVP subset.** One bake carries everything (spec assumption 12), so the increment is
the whole thing — which is exactly why the *gates* are placed early: T005 (baseline preserved), T043 (model
verified alone), T044 (trainability), T062 (input vector final). Each is cheap and each prevents an
expensive or irreversible mistake.

⚠️ **If the critical path tightens**, cut in this order — and ⛔ **decide before T065, not during it**:
1. Phase 4 housekeeping (US6, P3) — T023/T024/T025 first
2. FR-052b `craftCmQ`, then the IMU axes (US5's optional half)
3. ⛔ Never Phase 2, and never T043/T044

⭐ **The stop signal for Phase 5–6**: if the work turns into tuning constants to close a gap at the
aerodynamic limit, **stop and record the divergence instead**. The bar is *a decent stab, then measure* —
the plant model cannot resolve that regime anyway, and chasing it is the 023-Phase-9a mistake in a new
costume.

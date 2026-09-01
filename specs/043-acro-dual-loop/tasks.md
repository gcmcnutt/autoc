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
- [ ] T045 [US2] Verify determinism: identical seed + config reproduce identical trajectories, and the eval-vs-training bitwise gate holds (FR-015). **Operator-driven; ask first**

**Checkpoint**: SC-012, SC-004 and SC-014 part 1 pass. The model is trustworthy enough to build on.

---

## Phase 6 (US2 · exec order 5): Pin the plant

**Goal**: the actuator term is measured rather than modelled, and the 037 constants are triaged.

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

- [X] T051a [US3] ✅ **`xiao/inav-hb1.cfg` UPDATED 2026-08-31 to `pitch_rate = 24`** (rateprofile 1; ⚠️ the file is CRLF, so anchored `sed` silently no-ops — use a binary replace). ⛔ Still to do: flash/CLI the actual FC + bench-verify. **DECIDED (operator): go with 24.** INAV ACRO rate tuning — `pitch_rate 12 → 24` in `xiao/inav-hb1.cfg` **and** on both FC targets, matching the sim's arm C (`models/hb1_streamer.xml` pitch `maxRate="240"`). ⭐ Rationale in [outcome.md](outcome.md) § A/B/C: `pitch_rate 12` sits **below INAV's own default of 20**, capping full-stick elevator at **54%** of the fixed ±500 `pidSumLimit` (FF = 120 × 2.258 = 271) *and* forcing every useful pitch rate to the P/D attenuation floor (aP 0.016 at 58 °/s). At 24 → **100% elevator and aP 0.363**. Operator flew it: *"feels about right … a full commanded pitch up doesn't do a loop in real either"*. ⚠️ This is a **deviation from the spec's "gains and rates stay as-is"** and follows FR-012a discipline (change identically in sim, bench-verify, fold into the config of record). Allowed range is 4–180, so 24 is well inside it.
- [ ] T051b [US3] ⛔ ⚠️ **BENCH RIG ALSO MISMATCHED (found 2026-08-31)**: `xiao/inav-bench.cfg` rateprofile 1 is `roll_rate 18` / `pitch_rate 9` (180/90 °/s) against the flight article's 36/24. At pitch 9 the bench tops out at 41% elevator, so T056's "full stick now reaches ~100%" check would FAIL misleadingly. Align the bench or interpret its numbers accordingly. **SIM↔FC RATE-PARITY GATE (pre-flight, hard stop)**: assert the flown FC's `rates` equals the model XML's `maxRate/10` on every axis — today roll 36/360 and pitch **24/240**. ⚠️ **The 043-t2 bake is training against pitch 240 °/s; flying an FC still set to 120 °/s diverges precisely in the axis this feature exists to fix**, and would waste the bake. Record both sides in `specs/043-acro-dual-loop/bench-notes.md` and in the run MANIFEST (T068).
- [ ] T051c [US3] Decide the **roll dead-band** question: full-stick roll FF = 360 × 1.613 = 581 against the fixed ±500 budget, so the top **14% of roll stick is clipped** (operator 2026-08-30: *"the roll is a bit hot"*). `roll_rate 31` would give exactly full surface at full stick with no dead range, costing a little damping (aP 0.30 → 0.20 at 95 °/s). ⚠️ If changed it is a **second** variable and must also pass T051b parity; if not changed, record the decision and why.
- [ ] T051 [US3] INAV fork: fix the `mspOverrideInit` first-frame 200 ms floor in `~/inav/src/main/rx/msp_override.c` (FR-042)
- [ ] T052 [US3] Build INAV for **both** targets — bench `MAMBAF722_2022A` first, then flight `MATEKF722MINI`. ⚠️ Disconnect the GPS before flashing (FR-043)
- [X] T053 [US3] ✅ **DONE 2026-08-31** (`channel[5]` 1000 → **1500**; bands confirmed on the bench with the INAV Configurator: RC6 >1200 and <1600 ⇒ ACRO. The old comment claiming 1000 'forces MANUAL' is corrected — MANUAL is a separate switch the xiao does NOT override, deliberately, so the pilot keeps a mid-engagement escape). xiao host compile passes. Change `performMspSendLocked` in `xiao/src/msplink.cpp` to select **ACRO** rather than forcing MANUAL — the aux-2 mid-band (1200–1800) selects ACRO on the config of record (FR-040)
- [ ] T054 [US3] Stop forcing the mode on disengage in `xiao/src/msplink.cpp` so flight-mode selection returns to the pilot's switch; today the channel is forced on **every** frame the xiao sends (FR-041)
- [ ] T055 [US3] ⛔ Bench-verify **polarity end to end**: NN sign → PWM about 1500 → `rcCommand` sign → commanded rate sign → achieved body-rate sign. A sign error here is invisible in every surviving artifact (FR-016, contracts/action-space.md)
- [ ] T056 [US3] Bench-verify mode entry, mode exit, rate response at the surfaces, and that **no surface responds to the yaw axis** (FR-045, FR-018). ⭐ **Extended 2026-08-30**: with T051a's `pitch_rate 24`, confirm at the surfaces that **full pitch stick now reaches ~100% elevator travel** (it reached only 54% at `pitch_rate 12`) — that is the physical signature of the arm-C change and the cheapest confirmation the FC actually took it.
- [ ] T057 [US3] Bench-verify MSPRCOVERRIDE engages without the 200 ms floor at `failsafe_recovery_delay = 0` (FR-042)
- [ ] T058 [US3] Measure the **achievable INAV telemetry rate** and record it in `specs/043-acro-dual-loop/bench-notes.md` — `blackbox_rate_denom = 32` gives 60 Hz today; either answer is a result (FR-044, R7)
- [ ] T059 [US3] Regenerate the xiao firmware NN via `tools/nn2cpp.cc` against the current genome and confirm the host compile still passes; ⛔ **no xiao log-format change in 043** (US1 dropped, FR-005 cut)

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
- [~] T066 [US4] ⛔ Do **not** rebuild autoc while the run is in progress — overwriting `build/bin/autoc` breaks worker re-execs ⏳ **IN FORCE** while the bake runs.
- [X] T067 [US4] Monitor via `scripts/generate_pngs.sh m1 <log>` for the per-gen report set (⛔ not by hand-calling the analytics scripts — that loses the incremental S3 cache) ✅ Report packages generated at gens 115 / 178 / 248 via the wrapper; committed + pushed to `specs/043-acro-dual-loop/`. Evolution chart overlays 041-t7 + 038-t5.
- [ ] T068 [US4] Pin the run `retain=keep` and write `specs/043-acro-dual-loop/artifacts/MANIFEST.md` with the S3 prefix, master seed, commits, `autoc.ini.as-run`, and ⛔ the **input scale constants** — without them the genome loads clean and flies wrong (FR-061, Constitution VIII)

**Checkpoint**: a pinned, manifested M1 exists.

---

## Phase 10 (US4 · exec order 9): Flight and outcome

**Goal**: prove it in the air, and quantify by how much.

- [ ] T069 [US4] Bench-verify the deployed firmware and INAV build before flying (FR-063)
- [ ] T070 [US4] Fly the baked genome; capture the xiao log **and** the blackbox recording (⚠️ the clock-join is required again — US1 dropped)
- [ ] T071 [US4] Perform and cross-validate the blackbox clock-join as 041 did (−970 ppm fit, 0.5% against `ARM|MANUAL|MSPRCOVERRIDE`), in `specs/043-acro-dual-loop/flight-analysis.md` (FR-062)
- [ ] T072 [US4] Compute engaged-segment roll/pitch-rate power spectra and compare against the 041-t7 reference (3–5 Hz 30.1%, 5–10 Hz 7.4%) — ⚠️ **judged subjectively at first, deliberately** (SC-001)
- [ ] T073 [US4] ⭐ Quantify **how much** the offload helped, on the same measures, in `specs/043-acro-dual-loop/outcome.md` — a null or small result is valid; an unquantified *"seems better"* is not (SC-001a)
- [ ] T074 [US4] Verify tracking occupancy is within noise of the 041-t7 baseline using the same definition on both sides — ⚠️ compound with SC-001 (SC-003)
- [ ] T075 [US4] Verify no regression on the channels 041 validated — throttle, `specific_energy`, `dist_to_boundary`, rate amplitude — distance-standardized (SC-005)
- [ ] T076 [US4] Verify loop health: zero fetch failures, overruns, resyncs, tick gaps (SC-006)
- [ ] T077 [US4] Report SC-002's sim-vs-flight divergence ⛔ **per-axis, not pooled** — pitch is where the open-loop stability question lives (research.md Finding 1a consequence 3)

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

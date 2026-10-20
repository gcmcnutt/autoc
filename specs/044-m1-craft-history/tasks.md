---
description: "Task list for 044 — M1 craft-sensor history + roll plant"
---

# Tasks: 044 — M1 craft-sensor history + roll plant

**Input**: `specs/044-m1-craft-history/` — spec.md (governs, clarified 2026-10-19), plan.md, research.md (R1–R14), data-model.md,
contracts/ (nn-input-layout-v4, flight-log-v6, nn01-genome, config-keys, scenario-table), quickstart.md.
**Tests**: required (Constitution I) — each story writes its tests first and sees them fail.
**Organization**: by user story; ⛔ **phase order follows the plan's gated execution order** (spec § Execution / plan § Execution order),
not the P1/P2 labels — US4 and US2 must land before US1's format break, and US3 last.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an incomplete task)
- **[Story]**: US1 history inputs + layout · US2 roll plant · US3 bake / pick / fly · US4 trustworthy + affordable runs
- ⛔ No rebuild while a smoke or the bake is training (`build/` is shared by the workers).

---

## Phase 1: Setup

- [ ] T001 Pin `retain=keep` on the 043 evaluation dumps the new build cannot read — the six T122 candidate evals (seed 20261018) and the
  t6 g800 eval-suite runs (`eval-results/2026-10-18T22:35:14Z/`, keys in each tier's `console.log` "S3 upload:") in bucket `autoc-eval`
  (`aws s3api put-object-tagging … retain=keep`, verify by sampling) and record every key in a new `specs/044-m1-craft-history/MANIFEST.md`
  § Preservation (FR-005)
- [ ] T002 [P] Create `specs/044-m1-craft-history/smoke.ini` from `autoc.ini`: `PopulationSize = 3000`, `SimNumPathsPerGeneration = 6`,
  `WindScenarios = 16`, `NumberOfGenerations = 200`, `VariationRampStep = 10`, a fixed `Seed` (recorded in the header comment), all other
  keys identical; string keys keep comments on their own line (research R9)
- [ ] T003 [P] Create `specs/044-m1-craft-history/profile.md` and `specs/044-m1-craft-history/honesty.md` skeletons (sections per research
  R10 / R11)
- [ ] T004 [P] Extend `src/analytics/pitch_cycle_progress.py` with roll rows — elite roll-rate RMS and 3–5 Hz roll share per sampled
  generation (it already logs roll cap and flips); bump its cache name to `_pitchcycle_v2` (research R13, FR-013)

## Phase 2: Foundational (blocking)

- [ ] T005 On the branch head (still 45 inputs, unchanged plant): clean `bash scripts/rebuild-perf.sh`, `cd build && ctest`,
  `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed -d xiao`, and the t6 g800 tier-0 replay
  (`./scripts/eval_suite.sh specs/043-acro-dual-loop/artifacts-t6/nn_weights-t6-g800.dat 0 1791951530` → `−93924.534457` bitwise);
  record in `specs/044-m1-craft-history/MANIFEST.md` § Starting point

---

## Phase 3: User Story 4 — trustworthy and affordable runs (P2, ⛔ exec order 1)

**Goal**: remove defects that would be frozen into the new baselines; find where training time went.
**Independent test**: each fix has its own unit test; the profile is recorded; any adopted speed-up leaves a tier-0 replay bitwise identical.

### Tests (write first, see them fail)

- [ ] T006 [P] [US4] In `tests/scenario_prng_tests.cc` add `ClassStreamsIndependent`: for 10 000 scenario seeds, no class's first 64
  `ClassPRNG` draws contain another class's sub-seed or first draw (fails today — the wind stream's first draw IS the rabbit sub-seed)
- [ ] T007 [P] [US4] In `tests/contract_config_tests.cc` add `EvalVariationScale` (default −1; −1 and 0…1 accepted; anything else a startup
  error) and `CraftClPSigma`; raise the contract key count by 2 (contracts/config-keys.md)
- [ ] T008 [P] [US4] In `tests/wind_variation_tests.cc` add a test that the autoc-side wind-draw helper returns exactly the worker's
  `drawWindVariation` values for a given scenario seed (after skipping the gust-seed draw)

### Implementation

- [ ] T009 [US4] Mix sub-seeds: in `include/autoc/util/scenario_prng.h` `deriveClassSubSeeds`, pass each root output through murmur3
  `fmix32`, then the zero guard (`kSeedZeroSentinel`); no other generator change (research R4, FR-009)
- [ ] T010 [US4] `EvalVariationScale`: X-macro entry in `include/autoc/util/config.h`; in `src/autoc.cc` eval path, when ≥ 0 override
  `gEvalVariationScaleOverride` after reading the weights file, log stored and used values, validate at startup; add the key (own line)
  to `autoc-eval.ini` with `−1` (research R6, FR-011)
- [ ] T011 [US4] Wind-draw columns: add a helper in `include/autoc/eval/wind_variation.h` (scenario seed → `WindDraws` via
  `deriveClassSubSeeds` → `ClassPRNG(wind)` → skip one → `drawWindVariation`) and append `windMps turbX gustLx thermX` (full-strength
  values) to the scenario table in `src/autoc.cc` (contracts/scenario-table.md, research R5, FR-010)
- [ ] T012 [US4] Per-tick cost A/B: `perf stat` a fixed tier-1 eval of t6 g800 with the stock model and with a copy of
  `crrcsim/models/hb1_streamer.xml` whose `<controllers>` node is removed (same build); count rate-loop calls per tick; record in
  `specs/044-m1-craft-history/profile.md` (research R10a, FR-015)
- [ ] T013 [US4] Parent profile: `perf record -g -p $(pgrep -x autoc)` across one generation of a short `smoke.ini` run; attribute time to
  deserialize / scoring / selection / dmp write; record in `profile.md`; adopt only changes that keep a tier-0 replay bitwise identical,
  file the rest in `specs/BACKLOG.md` (research R10b)
- [ ] T014 [US4] `cd build && make && ctest` (T006–T008 now pass); record that every scenario draw changed (expected) — new baselines are
  taken at T026

**Checkpoint**: streams independent, wind draws logged, eval strength settable, profile recorded.

---

## Phase 4: User Story 2 — the roll airframe is partly honest (P1, ⛔ exec order 2)

**Goal**: roll damping as a class centred on a 3 Hz corner with aileron authority scaled; pitch damping spanning the streamer; FC sensor
delays modelled — all on the 45-input build.
**Independent test**: t6 gen 752 on the new plant shows more 3–5 Hz roll than on the t6 plant (honesty ladder) before any training.

### Tests (write first, see them fail)

- [ ] T015 [P] [US2] In `tests/craft_variation_tests.cc` add `craftClP` draw tests (centre −0.36, clamp [−0.60, −0.18], σ from
  `CraftClPSigma`) and re-pin `craftCmQ` to centre −2.25, clamp [−3.5, −1.0] (via the named constants)
- [ ] T016 [P] [US2] In `tests/craft_variation_tests.cc` add a test of the roll-authority scale helper: for `craftClP` across the clamp,
  `Cl_da/|Cl_p|` equals the t6 reference ratio 0.0633/0.135 (steady roll rate per aileron unchanged)
- [ ] T017 [P] [US2] New `tests/sensor_lpf_tests.cc` (register in `CMakeLists.txt` → clean rebuild-perf at T026): `Pt1Filter` 25 Hz and
  `BiquadFilter` 15 Hz stepped at 5 ms give low-frequency group delays ≈ 6.4 ms and ≈ 21 ms; first sample seeds the state

### Implementation

- [ ] T018 [US2] `craftClP` axis: constants and draw in `include/autoc/eval/craft_variation.h` (`kCraftClPCenter/Min/Max`,
  `craftClPSigma`, roll-authority scale helper with named refs −0.135 / 0.0633); `CraftClPSigma` in `include/autoc/util/config.h`;
  append `craftClP` LAST in `include/autoc/rpc/scenario_metadata.h` (and its `serialize`); draw, copy to metadata, `clP` scenario-table
  column and startup validation in `src/autoc.cc`; `CraftClPSigma = 0.10` in `autoc.ini` and `autoc-eval.ini` (research R1, FR-006)
- [ ] T019 [US2] Re-centre pitch damping: `kCraftCmQCenter −2.25`, clamp [−3.5, −1.0] in `include/autoc/eval/craft_variation.h`;
  `CraftCmQSigma = 0.5` in `include/autoc/util/config.h`, `autoc.ini`, `autoc-eval.ini` (research R2, FR-007)
- [ ] T020 [US2] crrcsim roll: `Global::craftClP` (NaN = unset) in `crrcsim/src/global.h` / `global.cpp`; set per scenario next to
  `craftCmQ` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`; in `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.{h,cpp}`
  keep `nominal_Cl_p`, apply `Cl_p = craftClP` and `Cl_da = Cl_da_ref·(craftClP/Cl_p_ref)·(1 + craftRollEffDelta)` (research R1)
- [ ] T021 [US2] Sensor filters at the sub-step: in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` hold per-scenario
  `Pt1Filter` (gyro, 25 Hz) and `BiquadFilter` (specific force, 15 Hz) from `include/autoc/eval/sensor_lpf.h`, step them in
  `autoc_apply_pending_substep` from the FDM body rates / specific force, have the 20 Hz gather read the filtered values, reset per
  scenario (research R3, FR-008)
- [ ] T022 [US2] Honesty ladder 1 — step test: roll steps at 15 m/s with `crrcsim/models/hb1_streamer_steptest.xml` at `craftClP` centre
  and both clamps; record roll corner (target ≈ 3 / 5 / 1.5 Hz) and steady roll rate per aileron (equal to the t6 plant's) in
  `specs/044-m1-craft-history/honesty.md` (research R11)
- [ ] T023 [US2] Honesty ladder 2 — closed-loop roll FRF of t6 g752 (setpoint → roll rate) on the new plant against the 2026-10-18 flight
  blackbox: new script `specs/044-m1-craft-history/roll_frf_compare.py` (pattern of `specs/043-acro-dual-loop/frf_pitch_compare.py`,
  departed scenarios excluded); record in `honesty.md`
- [ ] T024 [US2] Honesty ladder 3 — equal-environment eval of t6 g752 (`EvalVariationScale = 1.0`, seed 20261018) on the t6 build vs the
  new plant; `specs/043-acro-dual-loop/candidate_compare.py` 3–5 Hz roll share (calm and all); **gate**: it rises toward the flight's 50 %;
  record in `honesty.md`
- [ ] T025 [US2] Plant change in `crrcsim` committed on `044-m1-craft-history` and the submodule pointer bumped in autoc (submodule first,
  parent second)
- [ ] T026 [US2] Clean `bash scripts/rebuild-perf.sh` (CMakeLists changed at T017), `ctest`, xiao build; take the **new tier-0 baselines**
  (t6 g752 and g800 replays on the new plant + PRNG) and the eval-vs-training bitwise gate; record in `MANIFEST.md`; then
  `git tag 044-last-45in` (local; operator pushes) (FR-005, research R12)
- [ ] T027 [US2] **Smoke A** (plant only, 45 inputs): `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini
  logs/autoc-044-smokeA-plant.log` (detached; Constitution IX); record seed and prefix in `MANIFEST.md`

**Checkpoint**: plant honest-ish in roll, streamer-spanning pitch damping, sensor delays in; new baselines; smoke A running or done.

---

## Phase 5: User Story 1 — the policy sees its own recent rotation (P1, ⛔ exec order 3)

**Goal**: craft-sensor history (gyro p/q/r + accel z at 100/50 ms), adjacent layout shared with M2, an index-free layout gate, one complete
format break including `nnextractor`, `nn2cpp` and the xiao.
**Independent test**: smoke B (plant + history) vs smoke A on the same seed and plant — SC-006 recorded; plus the layout tests and the gate.
⛔ Smoke A must have finished before T034 rebuilds `build/`.

### Gate and tests (write first, see them fail)

- [ ] T028 [US1] Index-free gate: new `scripts/check_input_layout_literals.py` scanning hand-written consumers (`src/`, `include/`,
  `tools/`, `xiao/src`, `xiao/include`, `xiao/web`, `src/analytics/`, ported 043 scripts; excluding `xiao/src/generated/` and historical
  `specs/0[2-3]*` scripts) for literal input indices into the input vector / flat view and literal input counts; register as ctest
  `input_layout_literals` in `CMakeLists.txt`; run it now — expected hits: `src/analytics/flightlog_decode.py` (`NUM_INPUTS = 45`, name
  list) and `xiao/web/flight_logger.html` (`FL_NUM_IN = 45`, offsets) (FR-016, research R14)
- [ ] T029 [P] [US1] `tests/nn_layout_tests.cc`: `NNInputs` 53 / `TrackerInputs` 74 floats; craft tail order per data-model.md and last in
  both enums; the two enums' craft tails identical; metadata table length = enum COUNT; `kNNInputLayoutSignature` stable for a fixed
  table and different for any reorder or rename (FR-003, FR-016)
- [ ] T030 [P] [US1] New `tests/craft_history_tests.cc` (register in `CMakeLists.txt`): `recordErrorHistory` with craft samples fills the
  ring; after a known sequence the 100 / 50 / 0 ms slots hold the expected values; `resetHistory` pre-fills every slot with the current
  reading; no slot reads zero after reset (FR-001, FR-002)
- [ ] T031 [P] [US1] `tests/nn_serialization_tests.cc`: NN01 v2 round trip carries `layout_signature`; a v1 file is refused with a version
  error; a v2 file with a different signature is refused naming both signatures (contracts/nn01-genome.md)
- [ ] T032 [P] [US1] `tests/flightlog_roundtrip_tests.cc`: v6 header carries the ordered name table and signature; 53 inputs round-trip by
  name; a v5 log still decodes on the v5 path (contracts/flight-log-v6.md)
- [ ] T033 [P] [US1] `tests/nn2cpp_unroll_tests.cc`: generated code contains and asserts `kGeneratedLayoutSignature ==
  kNNInputLayoutSignature`; `nn2cpp` refuses a genome with a mismatched signature or `topology[0] != NN_INPUT_COUNT`; the generated gather
  and the sim gather produce identical vectors (history included) from identical `AircraftState` (FR-002 acceptance 2)

### Implementation

- [ ] T034 [US1] Layout v4 in `include/autoc/nn/nn_inputs.h`: `CraftCommonInputs` 28 floats in data-model.md order; `PathgenInput` /
  `TrackerInput` craft tails; regenerate `CRAFT_COMMON_INPUT_META` (display `gyrP-2 … acZ`); `kNNCraftHistoryLagsMsec = {100, 50, 0}` and
  the craft signal list as single constants; layout marker v4; `constexpr` FNV-1a `kNNInputLayoutSignature` over the ordered display
  names + version; update static_asserts; `include/autoc/nn/topology.h` weight asserts 2563 / 3235 (FR-001, FR-003)
- [ ] T035 [US1] Recording: craft ring in `include/autoc/eval/aircraft_state.h`; `recordErrorHistory(targetDir, dist, timeMs,
  craftSample)` and pre-fill in `resetHistory`; fill history slots in `writeCraftCommonInputs` in `src/nn/evaluator.cc`; update every caller —
  `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`, `xiao/src/msplink.cpp`, the tracker paths
  (`include/autoc/eval/tracker_stepper.h`, `src/eval/tracker_stepper.cc`, crrcsim tracker helper) — values are the ones the gather uses
  (sim: post-filter from T021) (research R7, FR-002)
- [ ] T036 [US1] Versions: `EvalResults::kSchemaVersion` 5 and its static_assert in `include/autoc/rpc/protocol.h`; `NN_FORMAT_VERSION` 2
  and a `layout_signature` field in `include/autoc/nn/serialization.h`, `src/nn/serialization.cc`, `NNGenome` in
  `include/autoc/nn/evaluator.h` (stamped on save) (research R8, R14)
- [ ] T037 [US1] Genome tools: `tools/nnextractor.cc` refuses a genome whose signature ≠ the tree's or `topology[0] != NN_INPUT_COUNT`;
  `tools/nn2cpp.cc` emits `kGeneratedLayoutSignature` with a `static_assert` beside the count and scale-signature asserts and refuses
  mismatches (FR-004, FR-016)
- [ ] T038 [US1] Flight log v6 in `xiao/include/flight_log_format.h`: `kFormatVersion = 6`; header gains the ordered slot-name table and
  the layout signature (CRC-guarded with the scale table); 8 scale entries by name for the history slots (contracts/flight-log-v6.md)
- [ ] T039 [US1] Decoders read names / count / signature from the v6 header, keep the v5 path: `src/analytics/flightlog_decode.py`
  (remove `NUM_INPUTS` / literal `INPUT_NAMES` for v6), `xiao/web/flight_logger.html` (remove `FL_NUM_IN` / fixed offsets for v6);
  `src/analytics/input_investment.py` by name — clears T028's hits
- [ ] T040 [US1] Verify the enum-driven consumers compile and print the new columns: `tools/dmp_dump.cc`, data.dat header in
  `src/autoc.cc`, `tools/renderer.cc`, `include/autoc/nn/input_mask.h`, `src/nn/mode.cc`, `include/autoc/eval/craft_observations.h`,
  `include/autoc/eval/envelope_state.h`
- [ ] T041 [P] [US1] Port the 043 analysis scripts to read input columns by name (no `INPUT_COLS = 45` / `rbHhd` offsets):
  `specs/043-acro-dual-loop/{candidate_compare,pitch_input_attribution,nn_replay_probe,rate_history_availability,history_lag_study,sim_pitch_spectrum,windiest_scenarios,airframe_id}.py`
- [ ] T042 [US1] Update the remaining count / index tests in `tests/` (`shared_input_block_tests.cc`, `nn_sensor_interface_tests.cc`,
  `contract_evaluator_tests.cc`, `nn_evaluator_tests.cc`, `input_mask_tests.cc`, `gather_tracker_inputs_tests.cc`,
  `recording_fidelity_tests.cc`, `arena_tests.cc`, `predictor_head_tests.cc`)
- [ ] T043 [US1] Clean `bash scripts/rebuild-perf.sh` (CMakeLists changed at T028/T030), `ctest` all green including
  `input_layout_literals`; regenerate `xiao/src/generated/nn_program_generated.cpp` with `build/nn2cpp` from a 53-input genome (first smoke B
  elite or a test genome), `pio run`; bench flight log round-trips through both decoders; take new tier-0 baselines on the 53-input build
- [ ] T044 [US1] **Smoke B** (plant + history): `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini logs/autoc-044-smokeB-history.log`;
  when both finish, `scripts/generate_pngs.sh m1 logs/autoc-044-smokeB-history.log --compare smokeA:logs/autoc-044-smokeA-plant.log` and
  record **SC-006** (first gen at which B's elite ≤ A's best-of-run smoothness on pitch and roll with `pctInStreak` ≥ A's) in `MANIFEST.md`
  — the bake keeps history either way (clarification 1)

**Checkpoint**: 53-input stack green on every surface; the gate passes; SC-006 recorded.

---

## Phase 6: User Story 3 — one controller, chosen fairly, flown and judged (P1, ⛔ exec order 4)

**Goal**: one 800-gen bake, equal-environment pick, flight, verdict.
**Independent test**: the flight analysed with the reference scripts; SC-001…005 each reported with its number.

- [ ] T045 [US3] Write `specs/044-m1-craft-history/artifacts-t1/MANIFEST.md` before launch (pattern of `artifacts-t6/MANIFEST.md`): regiment,
  what changed vs t6, gates run, as-run ini snapshot, seed policy, early calls (gens 5 / 40 / 100) and stop-and-rethink rows; pre-run gate
  (clean build + ctest) per Constitution IX
- [ ] T046 [US3] Launch: `bash scripts/train.sh autoc.ini logs/autoc-044-t1-m1-history.log`; record pid, effective master seed, S3 prefix in
  the MANIFEST; monitor with `scripts/generate_pngs.sh m1 logs/autoc-044-t1-m1-history.log --compare 043-t6-plant:logs/autoc-043-t6-m1-plant.log`
- [ ] T047 [US3] Close-out at gen 800: `retain=keep` on every object (verify by sampling incl. gen 1 / 800), extract the final elite,
  eval suite with the run's seed (tier 0 bitwise), MANIFEST § Post-run
- [ ] T048 [US3] Candidate selection: finalists (final elite + smoothest / best-tracking elites from the run report) evaluated in one
  environment (tier 0, fixed never-trained seed, `EvalVariationScale = 1.0`); `candidate_compare.py` with the 10-18 flight as calibration;
  pick on smoothness at matched tracking; recorded in the MANIFEST (FR-014)
- [ ] T049 [US3] Flight build and bench: `build/nnextractor -k <prefix> -g <gen>` → `build/nn2cpp` → `xiao/src/generated/` → `pio run` /
  flash; bench cycle; diff the FC CLI dump against `xiao/inav-hb1.cfg` @ 91abd4d (must be identical); `flash_erase`
- [ ] T050 [US3] Fly with the 043 procedure (blackbox 1/8 with SERVOS + ATTI, streamer state per span, wind recorded, level boot); logs to
  `flight-results/flight-<YYYYMMDD>/`
- [ ] T051 [US3] Analyse in `specs/044-m1-craft-history/flight-analysis-<YYYYMMDD>.md`: `candidate_compare.py` (flight mode) and
  `airframe_id.py` vs the 10-18 reference; SC-001 (roll 3–5 Hz < 82 °/s), SC-002 (pitch 3–5 Hz < 48, 2–3 Hz ≤ 51 °/s), SC-003, SC-004 (roll
  corner inside the class), SC-005 (1.5–5 Hz load < 1.88 g); **a miss on SC-001 or SC-002 opens another bake / flight cycle (append tasks);
  045 waits** (clarification 2)

---

## Phase 7: Polish & cross-cutting

- [ ] T052 [P] Constitution VI audit on touched paths: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/
  | grep -v '// raw-ok:'` — annotate or convert every new hit
- [ ] T053 [P] File deferrals in `specs/BACKLOG.md` (Constitution X): profiling speed-ups that were not bitwise-neutral; the accel x/y
  history revisit trigger; anything the honesty ladder left open
- [ ] T054 When SC-001 and SC-002 pass: `specs/044-m1-craft-history/outcome.md`, final BACKLOG pass, `CLAUDE.md` § Active feature → 045,
  memory pointer updated

---

## Dependencies

```
Setup (T001–T004) → Foundational (T005)
  → US4 (T006–T014)            new streams, wind columns, eval scale, profile
    → US2 (T015–T027)          plant + honesty ladder → new baselines → tag 044-last-45in → smoke A
      → US1 (T028–T044)        gate → layout v4 + recording + versions + tools + xiao + decoders → smoke B → SC-006
        → US3 (T045–T051)      bake → pick → fly → verdict  (miss on SC-001/002 → new cycle)
          → Polish (T052–T054)
```

⛔ Hard constraints: T026 (new baselines + tag) before T034; smoke A (T027) finished before T043 rebuilds `build/`; submodule pointer
bump before the parent commit (T025); no rebuild during any training run; T028's gate must pass at T043.

## Parallel opportunities

- **Setup**: T002, T003, T004 together.
- **US4 tests**: T006, T007, T008 together; T012 (per-tick A/B) alongside T009–T011 coding (different files, no build overlap if run on
  the T005 build).
- **US2 tests**: T015, T016, T017 together; T022–T024 sequential (each uses the build from T020/T021).
- **US1 tests**: T029–T033 together after T028; T041 (script ports) in parallel with T034–T040; T042 alongside T039–T040.

## Implementation strategy

- **MVP = through US2's honesty ladder (T024)**: the plant correction is independently valuable — it would change the next bake even if
  history were dropped — and it is cheap to verify against the flight.
- **Then US1** for the format break (one change, gate green before and after), **then US3** for the flight.
- Each phase ends at a checkpoint with a green build and recorded baselines; commit per task or logical group, never across a training run.

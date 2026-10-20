---
description: "Task list for 044 — M1 craft-sensor history + roll plant"
---

# Tasks: 044 — M1 craft-sensor history + roll plant

**Input**: `specs/044-m1-craft-history/` — spec.md (governs, clarified 2026-10-19), plan.md, research.md (R1–R14), data-model.md,
contracts/ (nn-input-layout-v4, flight-log-v6, nn01-genome, config-keys, scenario-table), quickstart.md.
**Tests**: required (Constitution I) — each story writes its tests first and sees them fail.
**Organization**: by user story; ⛔ **phase order follows plan § Execution order**, not the P1/P2 labels — US4 lands first (the honesty
ladder needs `EvalVariationScale`), then US2, then US1's format break, and US3 last.
**Revised 2026-10-19** after `/speckit.analyze`: new tests live in already-registered test targets except the two Python ctests, which
are registered in one `CMakeLists.txt` edit (T028); the index-free gate goes green on the **45-input** layout before the layout changes
(T028–T038, then T039–T046); smoke A runs from its own worktree; a `043-final` tag keeps the 043 artifacts readable.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an incomplete task)
- **[Story]**: US1 history inputs + layout · US2 roll plant · US3 bake / pick / fly · US4 trustworthy + affordable runs
- ⛔ No rebuild of a tree while a run launched from that tree is training (`build/` is shared by the workers).

---

## Phase 1: Setup

- [X] T001 Preserve 043 (FR-005): (a) `git tag 043-final 8cd5a5d` — the 043 close code state (crrcsim `7f345c5`), the build that reads
  every 043 dmp, NN01 genome and v5 flight log; local tag, operator pushes. (b) Pin `retain=keep` in `autoc-eval` on the six T122
  candidate evals (seed 20261018), the t6 g800 eval-suite runs (`eval-results/2026-10-18T22:35:14Z/`, keys in each tier's
  `console.log` "S3 upload:") and every t5 gen 488 eval dump used in 043 (the flight-calibration evals behind `g488-lat-tick.csv`).
  (c) Verify `retain=keep` on the t5 training run in `autoc-m1` by sampling gen 1, gen 488 (`gen9512`) and the last gen (t6 is already
  verified, `artifacts-t6/MANIFEST.md` § 6). Record every key and the tag in a new `specs/044-m1-craft-history/MANIFEST.md`
  § Preservation
- [X] T002 [P] Create `specs/044-m1-craft-history/smoke.ini` from `autoc.ini`: `PopulationSize = 3000`, `WindScenarios = 16`,
  `NumberOfGenerations = 200`, `VariationRampStep = 10`, a fixed `Seed` (recorded in the header comment), all other keys identical —
  in particular `SimNumPathsPerGeneration = 6` and `RandomPathSeedB = 13337`, so both smoke arms fly the same courses and scenarios
  (the scenario seed table is drawn apart from the weight-init stream); string keys keep comments on their own line (research R9)
- [X] T003 [P] Create `specs/044-m1-craft-history/profile.md` and `specs/044-m1-craft-history/honesty.md` skeletons (sections per research
  R10 / R11, with the R11 pass criteria written in)
- [X] T004 [P] Extend `src/analytics/pitch_cycle_progress.py` with roll rows — elite roll-rate RMS and 3–5 Hz roll share per sampled
  generation (it already logs roll cap and flips); bump its cache name to `_pitchcycle_v2` (research R13, FR-013)

## Phase 2: Foundational (blocking)

- [X] T005 On the branch head (still 45 inputs, unchanged plant): clean `bash scripts/rebuild-perf.sh`, `cd build && ctest`,
  `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed -d xiao`, and the t6 g800 tier-0 replay
  (`./scripts/eval_suite.sh specs/043-acro-dual-loop/artifacts-t6/nn_weights-t6-g800.dat 0 1791951530` → `−93924.534457` bitwise);
  record in `specs/044-m1-craft-history/MANIFEST.md` § Starting point

---

## Phase 3: User Story 4 — trustworthy and affordable runs (P2, ⛔ exec order 1)

**Goal**: remove defects that would be frozen into the new baselines; find where training time went.
**Independent test**: each fix has its own unit test; the profile is recorded; any adopted speed-up leaves the T012 reference replays
bitwise identical.

### Tests (write first, see them fail)

- [X] T006 [P] [US4] In `tests/scenario_prng_tests.cc` add `ClassStreamsIndependent`: for 10 000 scenario seeds, no class's first 64
  `ClassPRNG` draws contain another class's sub-seed or first draw (fails today — the wind stream's first draw IS the rabbit sub-seed)
- [X] T007 [P] [US4] In `tests/contract_config_tests.cc` add `EvalVariationScale` (default −1; −1 and 0…1 accepted; anything else a startup
  error) and `CraftClPSigma`; raise the contract key count by 2 (contracts/config-keys.md)
- [X] T008 [P] [US4] In `tests/wind_variation_tests.cc` add a test that the autoc-side wind-draw helper returns exactly the worker's
  `drawWindVariation` values for a given scenario seed (after skipping the gust-seed draw)

### Implementation

- [X] T009 [US4] Mix sub-seeds: in `include/autoc/util/scenario_prng.h` `deriveClassSubSeeds`, pass each root output through murmur3
  `fmix32`, then the zero guard (`kSeedZeroSentinel`); no other generator change (research R4, FR-009)
- [X] T010 [US4] `EvalVariationScale`: X-macro entry in `include/autoc/util/config.h`; in `src/autoc.cc` eval path, when ≥ 0 override
  `gEvalVariationScaleOverride` after reading the weights file, log stored and used values, validate at startup; add the key (own line)
  to `autoc-eval.ini` with `−1` (research R6, FR-011)
- [X] T011 [US4] Wind-draw columns: add a helper in `include/autoc/eval/wind_variation.h` (scenario seed → `WindDraws` via
  `deriveClassSubSeeds` → `ClassPRNG(wind)` → skip one → `drawWindVariation`) and append `windMps turbX gustLx thermX` (full-strength
  values) to the scenario table in `src/autoc.cc` (contracts/scenario-table.md, research R5, FR-010)
- [X] T012 [US4] `cmake --build build && (cd build && ctest)` (T006–T008 now pass). Every scenario draw has changed (expected), so take
  the **post-fix reference replays** on this build — t6 g800 tier 0 seed 1791951530 and t6 g752 tier 0 seed 20261018 — and record them
  in `MANIFEST.md` § Post-stream-fix reference; T013/T014 judge speed-ups against these, not T005's
- [X] T013 [US4] Per-tick cost (research R10a, FR-015), on a fixed tier-1 eval of t6 g800: (a) `perf stat` with the stock model vs a copy
  of `crrcsim/models/hb1_streamer.xml` whose `<controllers>` node is removed (same build), counting rate-loop calls per tick; (b) a
  throwaway build with the per-tick `EvalTick` payload trimmed, built in a scratch worktree so `build/` is untouched; (c) if (a) + (b)
  do not account for the +47 % t2 → t3 per-tick cost, record the unexplained remainder and file it in `specs/BACKLOG.md` — the bake is
  not held for it. Record all in `specs/044-m1-craft-history/profile.md`
- [X] T014 [US4] Parent profile (research R10b): launch `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini
  logs/autoc-044-profile.log` (detached, Constitution IX), `perf record -g -p $(pgrep -x autoc)` across one generation, then stop the
  run; attribute time to deserialize / scoring / selection / dmp write in `profile.md`; adopt only changes that keep the T012 reference
  replays bitwise identical; file the rest in `specs/BACKLOG.md`

**Checkpoint**: streams independent, wind draws logged, eval strength settable, profile recorded, no run left training.

---

## Phase 4: User Story 2 — the roll airframe is partly honest (P1, ⛔ exec order 2)

**Goal**: roll damping as a class centred on a 3 Hz corner with aileron authority scaled; pitch damping spanning the streamer; FC sensor
delays modelled — all on the 45-input build. No `CMakeLists.txt` change in this phase.
**Independent test**: t6 gen 752 on the new plant shows more 3–5 Hz roll than on the t6 plant (honesty ladder) before any training.

### Tests (write first, see them fail)

- [X] T015 [P] [US2] In `tests/craft_variation_tests.cc` add `craftClP` draw tests (centre −0.36, clamp [−0.60, −0.18], σ from
  `CraftClPSigma`) and re-pin `craftCmQ` to centre −2.25, clamp [−3.5, −1.0] (via the named constants)
- [X] T016 [P] [US2] In `tests/craft_variation_tests.cc` test the roll-coefficient helper `craftRollCoefficients(craftClP,
  craftRollEffDelta)` → `{Cl_p, Cl_da}`: `Cl_p == craftClP`; `Cl_da / |Cl_p|` equals the t6 reference ratio 0.0633/0.135 × (1 + delta)
  across the clamp (steady roll rate per aileron unchanged); NaN `craftClP` returns the reference pair × (1 + delta). The FDM calls
  this helper (T020), so the test covers what the FDM applies
- [X] T017 [P] [US2] In `tests/tick_rescale_tests.cc` (existing target — no CMakeLists change) test `FcSensorFilterChain`: gyro PT1
  25 Hz ×3 and accel biquad 15 Hz ×3 stepped at 5 ms give low-frequency group delays ≈ 6.4 ms and ≈ 15 ms (not 21 ms — corrected at implementation); the first sample seeds the
  state; `reset` re-seeds; reading the chain every 4th sub-step (the 20 Hz gather) returns the filtered value at that sub-step, not
  the raw one

### Implementation

- [X] T018 [US2] `craftClP` axis: constants and draw in `include/autoc/eval/craft_variation.h` (`kCraftClPCenter/Min/Max`,
  `craftClPSigma`, `craftRollCoefficients` with named refs −0.135 / 0.0633); `CraftClPSigma` in `include/autoc/util/config.h`;
  append `craftClP` LAST in `include/autoc/rpc/scenario_metadata.h` (and its `serialize`); draw, copy to metadata, `clP` scenario-table
  column and startup validation in `src/autoc.cc`; `CraftClPSigma = 0.10` in `autoc.ini` and `autoc-eval.ini`. `ScenarioMetadata` is
  serialized in the dmp, so bump `EvalResults::kSchemaVersion` 4 → **5** and its static_assert in `include/autoc/rpc/protocol.h`, with
  the honest-recording audit at the bump (research R1, R8, FR-006)
- [X] T019 [US2] Re-centre pitch damping: `kCraftCmQCenter −2.25`, clamp [−3.5, −1.0] in `include/autoc/eval/craft_variation.h`;
  `CraftCmQSigma = 0.5` in `include/autoc/util/config.h`, `autoc.ini`, `autoc-eval.ini` (research R2, FR-007)
- [X] T020 [US2] crrcsim roll: `Global::craftClP` (NaN = unset) in `crrcsim/src/global.h` / `global.cpp`; set per scenario next to
  `craftCmQ` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`; in `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.{h,cpp}`
  add `nominal_Cl_p` beside `nominal_Cl_da` and set `Cl_p` / `Cl_da` from `craftRollCoefficients` — no inline formula (research R1)
- [X] T021 [US2] Sensor filters at the sub-step: `FcSensorFilterChain` in `include/autoc/eval/sensor_lpf.h` (built on its `Pt1Filter` /
  `BiquadFilter`); `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` holds one per scenario, steps it in
  `autoc_apply_pending_substep` from the FDM body rates / specific force, has the 20 Hz gather read its output, resets it per scenario
  (research R3, FR-008)
- [X] T022 [US2] Honesty ladder 1 — step test: roll steps at 15 m/s with `crrcsim/models/hb1_streamer_steptest.xml` at `craftClP` centre
  and both clamps. **Pass**: roll corner within ±20 % of 3 / 5 / 1.5 Hz; steady roll rate per aileron within ±5 % of the t6 plant's at all
  three; at the fast clamp the rate loop's roll output stays below saturation for the step sizes used (record max |servo|). Record in
  `specs/044-m1-craft-history/honesty.md` (research R11)
- [X] T023 [US2] Honesty ladder 2 — closed-loop roll FRF of t6 g752 (setpoint → roll rate) on the new plant against the 2026-10-18 flight
  blackbox: new script `specs/044-m1-craft-history/roll_frf_compare.py` (pattern of `specs/043-acro-dual-loop/frf_pitch_compare.py`,
  departed scenarios excluded); magnitude and phase 1–6 Hz, t6 plant vs new plant vs flight. **Recorded, not gated**, in `honesty.md`
- [X] T024 [US2] Honesty ladder 3 — equal-environment eval of t6 g752 (`EvalVariationScale = 1.0`, seed 20261018) on the t6 build
  (`043-final`) vs the new plant; `specs/043-acro-dual-loop/candidate_compare.py` 3–5 Hz roll share, calm and all. **Gate**: higher on
  the new plant in both subsets (toward the flight's 50 %). **If T022 or T024 fails**: stop before T025 — revise the class (centre, σ,
  `Cl_da` scaling) in research R1 and re-run the ladder; no baselines or smokes on a plant that failed it. Record in `honesty.md`
- [X] T025 [US2] Plant change in `crrcsim` committed on `044-m1-craft-history` and the submodule pointer bumped in autoc (submodule first,
  parent second)
- [ ] T026 [US2] Gate-confirming clean `bash scripts/rebuild-perf.sh`, `ctest`, xiao build; take the **new tier-0 baselines** (t6 g752
  and g800 replays on the new plant + PRNG) and the eval-vs-training bitwise gate; replay one quiet tier (variations off) twice →
  identical (edge case "variation off"); record in `MANIFEST.md`; then `git tag 044-last-45in` (local; operator pushes) (FR-005,
  research R12)
- [ ] T027 [US2] **Smoke A** (plant only, 45 inputs) from its own worktree, so the main tree's `build/` stays free for US1:
  `git worktree add ../autoc-044-smokeA 044-last-45in && git -C ../autoc-044-smokeA submodule update --init`, operator clean
  `bash scripts/rebuild-perf.sh` there, then from `../autoc-044-smokeA`: `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini
  logs/autoc-044-smokeA-plant.log` (detached; Constitution IX). Record seed, prefix and worktree path in `MANIFEST.md`; when it finishes,
  copy the log to the main tree's `logs/` and `git worktree remove ../autoc-044-smokeA`

**Checkpoint**: plant honest-ish in roll, streamer-spanning pitch damping, sensor delays in; new baselines; smoke A running in its worktree.

---

## Phase 5: User Story 1 — the policy sees its own recent rotation (P1, ⛔ exec order 3)

**Goal**: craft-sensor history (gyro p/q/r + accel z at 100/50 ms), adjacent layout shared with M2, an index-free layout gate, one complete
format break including `nnextractor`, `nn2cpp` and the xiao.
**Independent test**: the gate passes on the 45-input layout and again after the change; the layout change itself touches only the layout
header, its asserts and the recording sites; smoke B vs smoke A gives SC-006.

### 5a — index-free gate, on the 45-input layout (FR-016)

- [ ] T028 [US1] Write `scripts/check_input_layout_literals.py` — scans hand-written consumers (`src/`, `include/`, `tools/`, `xiao/src`,
  `xiao/include`, `xiao/web`, `src/analytics/`, the ported 043 scripts; excluding `xiao/src/generated/` and historical `specs/0[2-3]*`
  scripts) for literal input indices into the input vector / flat view and literal input counts. Then the **one `CMakeLists.txt` edit of
  044**: register ctest `input_layout_literals` (that script) and ctest `flightlog_decode_py` (`python3 src/analytics/flightlog_decode.py
  --check` on the v6 fixture written by `flightlog_roundtrip_tests` into `${CMAKE_BINARY_DIR}/fixtures`, via `FIXTURES_SETUP` /
  `FIXTURES_REQUIRED` and a compile definition for the path, plus the v5 xiao log in `flight-results/flight-20261018/`). Operator clean
  `bash scripts/rebuild-perf.sh` (Constitution IV). Both fail now: literal hits in `src/analytics/flightlog_decode.py` (`NUM_INPUTS = 45`,
  name list) and `xiao/web/flight_logger.html` (`FL_NUM_IN = 45`, offsets); no v6 fixture yet
- [ ] T029 [P] [US1] `tests/nn_layout_tests.cc`: `kNNInputLayoutSignature` stable for a fixed table, different for any reorder, rename or
  layout-version change; metadata table length = enum COUNT (on today's 45 / 66 layout)
- [ ] T030 [P] [US1] `tests/nn_serialization_tests.cc`: NN01 v2 round trip carries `layout_signature`; a v1 file is refused with a version
  error; `checkGenomeLayout()` — the one function `nnextractor` and `nn2cpp` both call — refuses a genome whose signature differs (naming
  both) or whose `topology[0] != NN_INPUT_COUNT` (contracts/nn01-genome.md)
- [ ] T031 [P] [US1] `tests/flightlog_roundtrip_tests.cc`: v6 header carries the ordered name table and signature; inputs round-trip by name;
  a v5 log still decodes on the v5 path; the test writes the v6 fixture for `flightlog_decode_py` (contracts/flight-log-v6.md)
- [ ] T032 [P] [US1] `tests/nn2cpp_unroll_tests.cc`: generated code contains and asserts `kGeneratedLayoutSignature ==
  kNNInputLayoutSignature`; `nn2cpp` refuses a genome with a mismatched signature or input count
- [ ] T033 [US1] Layout signature in `include/autoc/nn/nn_inputs.h`: `constexpr` FNV-1a 64 `kNNInputLayoutSignature` over the ordered display
  names of the metadata table + the layout marker (research R14)
- [ ] T034 [US1] NN01 v2: `NN_FORMAT_VERSION` 2 and a `layout_signature` field in `include/autoc/nn/serialization.h`,
  `src/nn/serialization.cc`, `NNGenome` in `include/autoc/nn/evaluator.h` (stamped on save); `checkGenomeLayout()` there;
  `tools/nnextractor.cc` and `tools/nn2cpp.cc` call it; `nn2cpp` emits `kGeneratedLayoutSignature` with a `static_assert` beside the count
  and scale-signature asserts (FR-004, FR-016)
- [ ] T035 [US1] Flight log v6 in `xiao/include/flight_log_format.h`: `kFormatVersion = 6`; header gains the ordered slot-name table and the
  layout signature (CRC-guarded with the scale table), filled from the metadata table (contracts/flight-log-v6.md)
- [ ] T036 [US1] Decoders read names / count / signature from the v6 header and keep the v5 path: `src/analytics/flightlog_decode.py`
  (no `NUM_INPUTS` / literal `INPUT_NAMES` for v6; add the `--check <file>…` mode the ctest uses), `xiao/web/flight_logger.html` (no
  `FL_NUM_IN` / fixed offsets for v6), `src/analytics/input_investment.py` by name
- [ ] T037 [P] [US1] Port the 043 analysis scripts to read input columns by name (no `INPUT_COLS = 45` / `rbHhd` offsets):
  `specs/043-acro-dual-loop/{candidate_compare,pitch_input_attribution,nn_replay_probe,rate_history_availability,history_lag_study,sim_pitch_spectrum,windiest_scenarios,airframe_id}.py`
- [ ] T038 [US1] **Gate green on 45 inputs**: incremental build, `ctest` all green including `input_layout_literals` and
  `flightlog_decode_py`; regenerate `xiao/src/generated/nn_program_generated.cpp` with `build/nn2cpp` from a v2 test genome, `pio run`;
  a bench v6 log decodes in the Python decoder and (by hand) the web decoder; commit

### 5b — the layout change (FR-001…003)

- [ ] T039 [P] [US1] `tests/nn_layout_tests.cc`: `NNInputs` 53 / `TrackerInputs` 74 floats; craft tail order per data-model.md and last in
  both enums; the two enums' craft tails identical; weight counts 2563 / 3235 (FR-003)
- [ ] T040 [P] [US1] `tests/tick_rescale_tests.cc` (existing target): `recordErrorHistory` with craft samples fills the ring; after a known
  sequence the 100 / 50 / 0 ms slots hold the expected values; `resetHistory` pre-fills every slot with the current reading; no slot reads
  zero after reset (FR-001, FR-002)
- [ ] T041 [P] [US1] `tests/nn2cpp_unroll_tests.cc`: the generated gather and the sim gather produce identical vectors, history included,
  from identical `AircraftState` (US1 acceptance 2)
- [ ] T042 [US1] Update the remaining count / index tests in `tests/` (`shared_input_block_tests.cc`, `nn_sensor_interface_tests.cc`,
  `contract_evaluator_tests.cc`, `nn_evaluator_tests.cc`, `input_mask_tests.cc`, `gather_tracker_inputs_tests.cc`,
  `recording_fidelity_tests.cc`, `arena_tests.cc`, `predictor_head_tests.cc`)
- [ ] T043 [US1] Layout v4 in `include/autoc/nn/nn_inputs.h`: `CraftCommonInputs` 28 floats in data-model.md order; `PathgenInput` /
  `TrackerInput` craft tails; regenerate `CRAFT_COMMON_INPUT_META` (display `gyrP-2 … acZ`); `kNNCraftHistoryLagsMsec = {100, 50, 0}` and
  the craft signal list as single constants; layout marker v4; update static_asserts; `include/autoc/nn/topology.h` weight asserts
  2563 / 3235 (FR-001, FR-003)
- [ ] T044 [US1] Recording: craft ring in `include/autoc/eval/aircraft_state.h`; `recordErrorHistory(targetDir, dist, timeMs,
  craftSample)` and pre-fill in `resetHistory`; fill history slots in `writeCraftCommonInputs` in `src/nn/evaluator.cc`; update every
  caller — `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`, `xiao/src/msplink.cpp`, the tracker paths
  (`include/autoc/eval/tracker_stepper.h`, `src/eval/tracker_stepper.cc`, crrcsim tracker helper) — values are the ones the gather uses
  (sim: `FcSensorFilterChain` output from T021); confirm `xiao/src/msplink.cpp` calls `resetHistory` at engage and after a companion
  reboot (research R7, FR-002)
- [ ] T045 [US1] Versions and readers: `EvalResults::kSchemaVersion` 5 → **6** and its static_assert in `include/autoc/rpc/protocol.h`
  (the input width changed; the existing count check already refuses a 45-wide dump, the bump names the cause); verify the enum-driven
  consumers compile and print the new columns — `tools/dmp_dump.cc`, data.dat header in `src/autoc.cc`, `tools/renderer.cc`,
  `include/autoc/nn/input_mask.h`, `src/nn/mode.cc`, `include/autoc/eval/craft_observations.h`, `include/autoc/eval/envelope_state.h`
- [ ] T046 [US1] Gate-confirming clean `bash scripts/rebuild-perf.sh`, `ctest` all green (the gate still passes; `flightlog_decode_py`
  decodes the 53-input v6 fixture and the v5 log); regenerate the xiao code from a 53-input v2 test genome, `pio run`; bench: a 53-input v6
  log decodes in both decoders, and an engage plus a companion reboot mid-log show history pre-filled (no zero rows); new tier-0 baselines
  on the 53-input build. **"Just works" check**: list the files T043–T045 changed beyond `nn_inputs.h`, `topology.h`, the recording sites
  and the tests — each one is a missed index, fixed and added to the gate's patterns; record in `MANIFEST.md`
- [ ] T047 [US1] **Smoke B** (plant + history) from the main tree: `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini
  logs/autoc-044-smokeB-history.log`; when both have finished, `scripts/generate_pngs.sh m1 logs/autoc-044-smokeB-history.log --compare
  smokeA:logs/autoc-044-smokeA-plant.log` and record **SC-006**: A's generation of best smoothness (pitch and roll), B's first generation
  at or below that on both axes with `pctInStreak` ≥ A's at the same generation; met if B's generation is earlier. Recorded either way —
  the bake keeps history (clarification 1)

**Checkpoint**: 53-input stack green on every surface; the gate passes; SC-006 recorded.

---

## Phase 6: User Story 3 — one controller, chosen fairly, flown and judged (P1, ⛔ exec order 4)

**Goal**: one 800-gen bake, equal-environment pick, flight, verdict.
**Independent test**: the flight analysed with the reference scripts; SC-001…005 each reported with its number.

- [ ] T048 [US3] Write `specs/044-m1-craft-history/artifacts-t1/MANIFEST.md` before launch (pattern of `artifacts-t6/MANIFEST.md`): regiment,
  what changed vs t6, gates run, as-run ini snapshot, seed policy, early calls (gens 5 / 40 / 100) and stop-and-rethink rows; pre-run gate
  (clean build + ctest) per Constitution IX; smoke B finished
- [ ] T049 [US3] Launch: `bash scripts/train.sh autoc.ini logs/autoc-044-t1-m1-history.log`; record pid, effective master seed, S3 prefix in
  the MANIFEST; monitor with `scripts/generate_pngs.sh m1 logs/autoc-044-t1-m1-history.log --compare 043-t6-plant:logs/autoc-043-t6-m1-plant.log`
- [ ] T050 [US3] Close-out at gen 800: `retain=keep` on every object (verify by sampling incl. gen 1 / 800), extract the final elite,
  eval suite with the run's seed (tier 0 bitwise), MANIFEST § Post-run
- [ ] T051 [US3] Candidate selection: finalists (final elite + smoothest / best-tracking elites from the run report) evaluated in one
  environment (tier 0, fixed never-trained seed, `EvalVariationScale = 1.0`); `candidate_compare.py` with the 10-18 flight as calibration;
  pick on smoothness at matched tracking; recorded in the MANIFEST (FR-014)
- [ ] T052 [US3] Flight build and bench: `build/nnextractor -k <prefix> -g <gen>` → `build/nn2cpp` → `xiao/src/generated/` → `pio run` /
  flash; bench cycle; diff the FC CLI dump against `xiao/inav-hb1.cfg` @ 91abd4d (must be identical); `flash_erase`
- [ ] T053 [US3] Fly with the 043 procedure (blackbox 1/8 with SERVOS + ATTI, streamer state per span, wind recorded, level boot); logs to
  `flight-results/flight-<YYYYMMDD>/`
- [ ] T054 [US3] Analyse in `specs/044-m1-craft-history/flight-analysis-<YYYYMMDD>.md`: `candidate_compare.py` (flight mode) and
  `airframe_id.py` vs the 10-18 reference; SC-001 (roll 3–5 Hz < 82 °/s), SC-002 (pitch 3–5 Hz < 48, 2–3 Hz ≤ 51 °/s), SC-003, SC-004 (roll
  corner inside the class), SC-005 (1.5–5 Hz load < 1.88 g); **a miss on SC-001 or SC-002 opens another bake / flight cycle (append tasks);
  045 waits** (clarification 2)

---

## Phase 7: Polish & cross-cutting

- [ ] T055 [P] Constitution VI audit on touched paths: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/
  | grep -v '// raw-ok:'` — annotate or convert every new hit
- [ ] T056 [P] File deferrals in `specs/BACKLOG.md` (Constitution X): profiling speed-ups that were not bitwise-neutral and any unexplained
  per-tick cost (T013); the accel x/y history revisit trigger; anything the honesty ladder left open
- [ ] T057 When SC-001 and SC-002 pass: `specs/044-m1-craft-history/outcome.md` citing every pinned S3 prefix (the bake, the flown genome,
  the 043 preservation keys from T001) and both tags (Constitution VIII.2), final BACKLOG pass, `CLAUDE.md` § Active feature → 045, memory
  pointer updated

---

## Dependencies

```
Setup (T001–T004) → Foundational (T005)
  → US4 (T006–T014)            new streams, wind columns, eval scale, post-fix reference, profile
    → US2 (T015–T027)          plant + honesty ladder (gate) → new baselines → tag 044-last-45in → smoke A (own worktree)
      → US1 5a (T028–T038)     one CMakeLists edit + clean rebuild → signature, NN01 v2, log v6, decoders → gate green on 45 inputs
        → US1 5b (T039–T047)   layout v4 + recording + dmp v6 → gate green again → smoke B → SC-006 (needs smoke A finished)
          → US3 (T048–T054)    bake → pick → fly → verdict  (miss on SC-001/002 → new cycle)
            → Polish (T055–T057)
```

⛔ Hard constraints: T001's `043-final` tag before anything changes the dmp (T018); T024's gate passes before T025; T026 (new baselines + tag)
before T027 and T028; smoke A runs from its worktree, so the main tree may rebuild during it, but T047 needs it finished; no main-tree
rebuild while T014's profile run, smoke B or the bake is training; submodule pointer bump after the crrcsim commit (T025); the gate passes
at T038 and again at T046.

## Parallel opportunities

- **Setup**: T002, T003, T004 together.
- **US4 tests**: T006, T007, T008 together; T013(a) alongside T009–T011 coding on the T005 build.
- **US2 tests**: T015, T016, T017 together; T022–T024 sequential (each uses the T020/T021 build).
- **US1 5a**: T029–T032 together after T028; T037 (script ports) in parallel with T033–T036.
- **US1 5b**: T039–T041 together; T042 alongside T043–T045.
- **Across stories**: US1 coding (T028 onward) runs while smoke A trains in its worktree.

## Implementation strategy

- **MVP = through US2's honesty ladder (T024)**: the plant correction is independently valuable — it would change the next bake even if
  history were dropped — and it is cheap to verify against the flight.
- **Then US1** in two steps: make every consumer layout-agnostic on the old layout (5a), then change the layout in one place (5b) — the
  file list at T046 is the evidence that it "just works".
- **Then US3** for the flight. Each phase ends at a checkpoint with a green build and recorded baselines; commit per task or logical
  group, never rebuild a tree that a run is training from.

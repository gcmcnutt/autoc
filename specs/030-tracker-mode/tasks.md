---

description: "030 — Tracker Mode — task list (parked, gated on 029-no-future-arch)"
---

# Tasks: 030 Tracker Mode (beacon-camera target tracking via flight playback)

**Input**: Design documents from [`specs/030-tracker-mode/`](.)
**Prerequisites**: [plan.md](./plan.md), [spec.md](./spec.md), [research.md](./research.md), [data-model.md](./data-model.md), [contracts/](./contracts/), [quickstart.md](./quickstart.md), AND [`specs/029-no-future-arch/`](../029-no-future-arch/) cleared (controller architecture trains effectively from past-only inputs)

**Status**: Parked. See [pivot note in spec.md](./spec.md).

> **PIVOT NOTE (2026-04-30)**: This task list was originally 029. The "Phase 2 — US1 past-only baseline experiment" (T006-T015) has moved out — that experiment is the *only* US in the new [`specs/029-no-future-arch/`](../029-no-future-arch/) and gates this whole list. When 030 unparks: drop Phase 2 from this list, renumber tasks starting from Phase 3 (US2 substrate work) as the new Phase 1, and adopt the 029-chosen NN architecture as the controller baseline.

**Tests**: Included per Constitution Principle I (Testing-First). Each contract gets a contract-test task before implementation. Determinism tests are explicit and load-bearing.

**Organization**: Tasks grouped by operator user story so each story is independently testable. **(Historical note: US1 past-only baseline was originally Phase 2 here; now lives in 029-no-future-arch.)** Type-safe sensor interface refactor remains as Phase 3.1 (US2 substrate prerequisite).

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Maps task to user story (US1–US6)
- File paths absolute or repo-rooted

## Path Conventions

- Single project rooted at repo root. C++ in `src/` and `crrcsim/src/`, headers in `include/autoc/` and `crrcsim/src/...`, tests in `tests/`, feature docs/scripts in `specs/029-tracker-mode/`.

---

## Phase 1: Setup

**Purpose**: Verify branch state and prerequisites before any 029 work.

- [ ] T001 Verify branch `029-tracker-mode` checked out, working tree clean (or with only the existing 029 spec/plan/research artifacts uncommitted): `git status` shows the expected state
- [ ] T002 Confirm 028 baseline is in tree: `git log --oneline | grep more-rnn3` finds the more-rnn3 commit
- [ ] T003 [P] Verify `bash scripts/rebuild.sh` is green at the 028 baseline before any 029 changes: clean rebuild + `ctest --output-on-failure` produces 12/12 tests passing
- [ ] T004 [P] Verify xiao build is green at the 028 baseline: `cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed` succeeds (Constitution II)
- [ ] T005 [P] Confirm a source pathgen run with S3 .dmp files is identified for use as the 029 library source. Record the source-run ID + chosen source gen in `specs/029-tracker-mode/source_run.txt` (e.g., `more-rnn3-2026-04-26T... gen=600`)

**Checkpoint**: 028 baseline verified green; library source identified.

---

## Phase 2: User Story 1 — Past-only input baseline experiment (Priority: P1) 🎯 First experiment, gates the rest

**Goal**: Validate that the recurrent NN trains effectively without future-lookahead inputs. **Runs on pristine 028 codebase** — no refactor prerequisites. Single-file change to time-offset constants. No 029-specific implementation begins until this passes.

**Critical: pristine 028.** The type-safe sensor interface refactor (Phase 3.1) does NOT land before this — US1's whole point is fail-fast architectural validation, and a behavior-preserving-but-still-meaningful refactor in the path would muddy the signal.

**Independent Test**: Run the experiment to gen 600 (matched compute with more-rnn3). Compare against more-rnn3 on (1) late-plateau fitness within ±10 % under fixed-difficulty eval, (2) per-axis aggressiveness shape (architecture-consistent — roll dominates), (3) streak quality emergence on schedule.

### Implementation for US1

- [X] T006 [US1] Update time-sample comment in [`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h): `// Time samples: [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]` → `// Time samples: [-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`. Per spec US1 distribution choice. **[Also updated mirror comments in include/autoc/nn/topology.h, src/nn/evaluator.cc, tests/nn_evaluator_tests.cc, src/autoc.cc data.dat header column labels.]**
- [X] T007 [US1] Update time-offset constants in [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc) (NOT `nn_input_computation.cc` — that file does not exist; the time-offset code lives in evaluator.cc:`nn_gather_inputs`). Replaced `HIST_PAST[] = {9, 3, 1, 0}` (4 past tick-offsets) with `{5, 4, 3, 2, 1, 0}` (6 past-only tick-offsets at 100 ms each). **Removed**: `FORECAST_OFFSETS[]`, `getPathTangentAtOffset()` static helper, the per-axis forecast-projection loop, and the per-axis forecast-distance loop — all dead under past-only. **Retained**: `PathProvider&` parameter on `nn_gather_inputs` (marked `[[maybe_unused]]`) for API stability across nn2cpp / xiao / minisim callers. Also bumped `tools/renderer.cc` `NOW` constant from index 3 → 5 to match the new "now" slot position
- [ ] T008 [US1] Build + verify: `bash scripts/rebuild.sh` green, `ctest --output-on-failure` 12/12 pass with the new time semantics. Topology unchanged (still 33 inputs); only field semantics differ
- [ ] T009 [US1] xiao build verification: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green (Constitution II)
- [ ] T010 [US1] Launch the past-only training run: `nohup ./build/autoc -c autoc.ini > logs/autoc-029-pastonly.log 2>&1 &` (Path A config: pop 5000 × 600 gens, recurrent NN, single seed). Run name: `more-rnn4-pastonly`
- [ ] T011 [P] [US1] Set up monitoring loop for 6-panel evolution plot every 50 gens — invocation per [quickstart.md Phase 1.4](./quickstart.md#14-monitor-every-50-gens) using existing `specs/028-deeper-rnn/plot_evolution_progress.py` (no script changes — `data.stc` schema unchanged). Comparison priors: more-rnn3, cadence7-redux
- [ ] T012 [P] [US1] Set up monitoring loop for per-axis aggressiveness PNG every 100 gens — invocation per [quickstart.md Phase 1.4](./quickstart.md#14-monitor-every-50-gens) using existing `specs/028-deeper-rnn/plot_per_axis_time_series.py`
- [ ] T013 [US1] Apply early-stop criteria during run: kill the run if (a) best-fitness above pid1's −27045 floor by gen 100 with no descent, OR (b) per-axis aggressiveness pattern deviates substantially from more-rnn3 (e.g., pitch becoming dominant — would suggest the input change broke architecture-consistency)
- [ ] T014 [US1] At run completion, capture late-plateau metrics: read final-gen `#NNGen` line, compute late-plateau dCtrl + ⟨|out|⟩ from existing aggressiveness tooling (last-50-gens window). Re-evaluate winning gen against fixed-difficulty eval per [project_late_run_fitness_interpretation.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md)
- [ ] T015 [US1] Write outcome doc `specs/029-tracker-mode/pastonly_outcome.md` with fitness comparison vs more-rnn3, per-axis pattern comparison, and **branch decision**: PASS (proceed to Phase 3 029 substrate work) or FAIL (investigate alternative time-offset distributions per plan §1.6 fallback list before declaring no-lookahead-doesn't-work)

**Checkpoint** ✅ US1 passes (or alternative-distribution follow-up gives a pass). Foundational architectural assumption for 029 is validated. Phase 3 029 substrate work begins.

**During the ~24-48h US1 training run, Phase 3.1 (type-safe sensor interface refactor) can develop and merge in parallel** — its only consumer is tracker-mode-specific work which doesn't begin until US1 passes anyway.

---

## Phase 3: User Story 2 — Operator launches tracker-mode training from a recorded pathgen run (Priority: P1) 🎯 029 substrate gateway

**Goal**: Land the 029 substrate end-to-end. Tracker-mode autoc launches against a recorded library, the second aircraft visibly replays trajectories, but the NN sees no useful beacon data yet (placeholder zero-input). This is the *plumbing* phase — Phase 4 connects the perception pipeline.

**Phase 3.1 (sensor interface refactor) lands first within Phase 3** because Phase 3.2+ adds new tracker-mode-specific named inputs that need the typed interface.

**Independent Test**: Run the dmp-to-playback converter against a source dmp; verify 245 .crrclog files produced with valid format. Launch tracker-mode autoc; verify second aircraft renders + scenarios run + fitness reports (uniformly bad, since NN sees no target signal). Determinism preserved (same seed → identical fitness across runs).

### Phase 3.1: Type-safe sensor interface refactor (US2 prerequisite)

Per FR-006, [contracts/nn_sensor_interface.md](./contracts/nn_sensor_interface.md), and [research.md R7](./research.md). ~270-330 LOC across 12 files. Behavior-preserving — pathgen mode behavior unchanged.

- [ ] T016 [P] [US2] Write contract test for sensor interface enum integrity: `tests/nn_sensor_interface_tests.cc` per [contracts/nn_sensor_interface.md §Test surface](./contracts/nn_sensor_interface.md). Tests: `EnumCount_MatchesPathgenLayout`, `RoundtripInputs_PathgenIdentity`, `MetadataLookup_NamesMatchEnum`, `RangeValidation_OutOfRangeAsserts`, `BackwardCompat_DataDatHeader_Stable`. Tests MUST FAIL initially (red).
- [ ] T017 [US2] Register `nn_sensor_interface_tests` in `CMakeLists.txt` test list
- [ ] T018 [US2] Create `include/autoc/nn/sensor_interface.h` with `PathgenSensorInput` enum (33 entries matching today's `NNInputs` field order), `SensorInputDescriptor` struct, `kPathgenSensorMeta[]` table, `SensorInputs<EnumT>` template per [contracts/nn_sensor_interface.md](./contracts/nn_sensor_interface.md)
- [ ] T019 [US2] Migrate `include/autoc/nn/nn_inputs.h` — replace direct field declarations with template-instance typedef `using NNInputs = SensorInputs<PathgenSensorInput>;` while preserving the on-disk byte layout (constitutional: serialization contract preserved)
- [ ] T020 [US2] Migrate `src/nn/nn_input_computation.cc` — replace field-name accesses with enum-indexed `inputs[QUAT_W]` / `inputs[GYRO_P]` / etc. Preserve all computation semantics. Note: T007 from US1 already updated time offsets; preserve that change
- [ ] T021 [US2] Migrate `src/autoc.cc` data.dat header + format string to auto-generate from the descriptor table (DRY — single source of truth)
- [ ] T022 [US2] Migrate `tests/contract_evaluator_tests.cc` and `tests/nn_evaluator_tests.cc` to use named constants from sensor_interface.h instead of magic numbers
- [ ] T023 [US2] Migrate `xiao/src/msplink.cpp` xiao-side input gathering to typed interface
- [ ] T024 [US2] Update `tools/nn2cpp.cc` to emit enum-aware C code (xiao generated NN program references typed interface)
- [ ] T025 [US2] Migrate `specs/019-improved-crrcsim/sim_response.py` data.dat parser to consume the auto-generated header (or pin to enum names)
- [ ] T026 [US2] Run T016 tests — all pass (green). Run full `ctest --output-on-failure` — 12/12 existing tests still pass with refactored interface
- [ ] T027 [US2] Run xiao build to confirm the typed interface migrated cleanly: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green (Constitution II)
- [ ] T028 [US2] Smoke training run on refactored 028+US1 baseline: `nohup ./build/autoc -c autoc.ini > logs/autoc-029-refactor-smoke.log 2>&1 &` for ~50 gens. Verify fitness shape matches more-rnn4-pastonly's first 50 gens (Constitution II: behavior preserved across the refactor)

**Checkpoint** Phase 3.1 ✅ Type-safe sensor interface lands. Pathgen behavior preserved. Tracker-mode-specific work (Phase 3.2+) can begin.

### Phase 3.2: crrcsim multi-aircraft accessor (US2 substrate)

- [ ] T029 [US2] Write contract test `tests/robot_programmable_tests.cc` per data-model.md §1 (playback file consumer side). Tests: `MultiAircraftInstantiation_Both`, `RobotStateQuery_RoundTrip`, `Determinism_SameSeedSameOutput`. Tests MUST FAIL initially (red — accessor doesn't exist yet)
- [ ] T030 [US2] Add `Robots::getRobotFDM(int idx) const` accessor to [`crrcsim/src/mod_robots/robots.h`](../../crrcsim/src/mod_robots/robots.h) and [`robots.cpp`](../../crrcsim/src/mod_robots/robots.cpp) — returns the `RobotBase*` for the indexed robot. ~5 LOC change per [research.md R4](./research.md)
- [ ] T031 [US2] Run T029 tests — green. `bash scripts/rebuild.sh` green, full ctest passes
- [ ] T032 [US2] Register `robot_programmable_tests` in `CMakeLists.txt` test list

### Phase 3.3: dmp-to-playback converter tool

- [ ] T033 [P] [US2] Write contract test `tests/dmp_to_playback_tests.cc` per [contracts/playback_file_format.md §Test surface](./contracts/playback_file_format.md). Tests: `Roundtrip_KnownDmp_PlaybackMatches`, `Determinism_SameDmpSameOutput`, `XmlRootValidation_HeaderRequired`, `EulerScaling_RoundTrip`, `EmptyTrajectory_HandledGracefully`. Tests MUST FAIL initially (red)
- [ ] T034 [US2] Create new tool source [`tools/dmp_to_playback.cc`](../../tools/dmp_to_playback.cc) — reads `.dmp` (cereal `EvalResults`), iterates per-scenario aircraft trajectories, emits one `.crrclog` per scenario per [contracts/playback_file_format.md](./contracts/playback_file_format.md). CLI: `--source-run <id> --source-gen <N> --output-dir <path>`. Pattern follows [`tools/nnextractor.cc:177-192`](../../tools/nnextractor.cc) for cereal deserialization
- [ ] T035 [US2] Register `dmp_to_playback` executable in [`CMakeLists.txt`](../../CMakeLists.txt) — links `autoc_common` per [research.md R2](./research.md), uses existing pattern alongside `autoc`, `minisim`, `renderer`, `nnextractor`, `nn2cpp`
- [ ] T036 [US2] Library metadata emission: tool writes `library_metadata.json` per [data-model.md §2.2](./data-model.md) summarizing source-run id, source-gen, conversion timestamp, per-scenario variation params. Validation script `specs/029-tracker-mode/scripts/validate_library.py` (NEW) checks scenario-count consistency
- [ ] T037 [US2] Register `dmp_to_playback_tests` in test list. Run tests — green
- [ ] T037a [US2] **FR-013 crashed-scenario handling — pin decision + emit metadata flag**: Per [contracts/playback_file_format.md §Open contract decisions item 1](./contracts/playback_file_format.md), commit to **truncate-and-flag** (NOT skip-and-renumber) so library scenario count stays at 245 and the operator has visibility into degraded entries. Modify the converter to: (a) detect terminal-crash scenarios in source `.dmp` (per-scenario `CrashReason != none`); (b) write the truncated trajectory as a valid `.crrclog`; (c) emit a `truncated_scenarios: [<idx>, ...]` field plus per-scenario `crash_reason` field in `library_metadata.json` per [data-model.md §2.2](./data-model.md). Add test `TruncatedScenario_FlaggedInMetadata` to `dmp_to_playback_tests.cc`. Add test `Tracker_HandlesTruncatedEntry_NoCrash` to `tracker_mode_integration_tests.cc` (covered by T047) — tracker mode running against a truncated library entry must not crash; scenario fitness reflects whatever ticks were available
- [ ] T038 [US2] Generate first library: `./build/dmp_to_playback --source-run <ID-from-T005> --source-gen <N> --output-dir libraries/<source-run-name>/` produces 245 .crrclog files + library_metadata.json

### Phase 3.4: tracker-mode EvalResults schema bump

- [ ] T039 [P] [US2] Write contract test `tests/tracker_mode_integration_tests.cc` (placeholder — full integration tests land in T046; this test only covers the schema roundtrip part). Tests: `SchemaRoundtrip_TrackerMode` (cereal-roundtrip a tracker-mode `EvalResults`, all fields preserved), `SchemaRoundtrip_PathgenMode` (cereal-roundtrip a pathgen-mode `EvalResults` under the new v2 schema with `tracker_mode = false` + empty `trackerScenarios`, all existing fields preserved), `UnifiedSchema_BothModesInOneBinary` (a single 029-built binary correctly reads both pathgen-mode and tracker-mode dumps produced under the v2 schema, dispatching by `tracker_mode` flag). Tests MUST FAIL initially (red). NOTE: there is intentionally NO test for "029 binary reads pre-029 dump" or "pre-029 binary reads 029 dump" — these are clean-cut unsupported per [no-cereal-versioning policy](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md)
- [ ] T040 [US2] Modify [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h) — extend `EvalResults` per [contracts/tracker_dmp_schema.md](./contracts/tracker_dmp_schema.md): add `bool tracker_mode = false`, `std::vector<TrackerScenarioState> trackerScenarios`. Bump `CEREAL_CLASS_VERSION(EvalResults, 1)` → 2 per project [no-cereal-versioning policy](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) (clean-cut, old dumps unloadable by 029-aware tools)
- [ ] T041 [US2] Add `TrackerScenarioState` and `TickCameraState` struct definitions to [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h) per [contracts/tracker_dmp_schema.md §Schema additions](./contracts/tracker_dmp_schema.md). Cereal serialize methods for both
- [ ] T042 [US2] Run T039 tests — green. `bash scripts/rebuild.sh` green. Schema bump is a **clean cut** per [no-cereal-versioning policy](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) and [contracts/tracker_dmp_schema.md §Backward compatibility — CLEAN CUT](./contracts/tracker_dmp_schema.md): pre-029 `.dmp` files are unloadable by 029-aware tools (no migration shim), and pre-029 binaries cannot read 029 dumps. Only assert roundtrip *within* 029 — `MixedFleetTooling_PathgenReadsNewSchema` test verifies pathgen-mode dumps produced by 029 tools still parse with 029 tools.

### Phase 3.5: RobotPathProvider + tracker-mode autoc.ini selector

- [ ] T043 [US2] Create [`crrcsim/src/mod_inputdev/inputdev_autoc/robot_path_provider.h`](../../crrcsim/src/mod_inputdev/inputdev_autoc/robot_path_provider.h) and `.cc` — implements the same interface as `VectorPathProvider` but pulls position from `Global::robots->getRobotFDM(targetRobotIdx)->getPos()`. ~50 LOC
- [ ] T044 [US2] Modify [`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp) — add `TrainingMode` enum, mode-selector reads from autoc.ini config. In tracker mode: instantiate `CRRC_AirplaneSim_Playback` per scenario from library file path; substitute `RobotPathProvider` for `VectorPathProvider`. ~50 LOC modifications
- [ ] T045 [US2] Create [`autoc-tracker.ini`](../../autoc-tracker.ini) (NEW config file at repo root) — separate from `autoc.ini`. Contains: `TrainingMode = TRACKER`, `LibraryDirectory = libraries/<source-run-name>/`, plus tracker-mode-specific knobs (camera config selectors, etc.). Inherits Path A pop/gens/etc. from autoc.ini conventions
- [ ] T046 [US2] Modify [`src/autoc.cc`](../../src/autoc.cc) — read `TrainingMode` from config; in tracker mode, delegate scenario construction to library loader (reads `library_metadata.json`, assigns one .crrclog file per scenario by index). 245 scenarios from the library map to 245 tracker-mode scenarios
- [ ] T047 [US2] Add full integration tests to `tests/tracker_mode_integration_tests.cc`: `EndToEnd_DeterministicEval` (single scenario, fixed seed, fixed library entry → identical fitness across runs), `MultiAircraftCoexists_PerScenario` (verify both aircraft instantiate per scenario, second replays target trajectory, training craft simulates physics)
- [ ] T048 [US2] Phase 3 substrate smoke: `nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-substrate-smoke.log 2>&1 &`. Run for 5 generations. Verify: tracker mode launches, second aircraft visibly replaying, fitness reported (uniformly bad — NN sees no beacons yet, this is expected; placeholder zero-input). Determinism check: re-run same seed, identical fitness

**Checkpoint** ✅ Tracker-mode autoc launches end-to-end. Library construction works. Multi-aircraft per scenario. Schema bump complete. Determinism preserved. NN gets placeholder beacon inputs (zeros) — Phase 4 connects the real perception.

---

## Phase 4: User Story 3 — Operator experiments with camera configurations (Priority: P1)

**Goal**: Connect the perception pipeline. NN now sees real beacon coordinates. Camera-config experimentation harness in place. Short training runs validate behavior before US4 commits to long-run training.

**Independent Test**: Run a 50–100 gen tracker-mode training at the v1 baseline camera config (planar pinhole, 120° FOV, single forward-mounted, 30 Hz, 0 ms latency). Verify sane fitness descent + visual lock signal. Sweep ≥3 camera-config variants (FOV, mount, frame rate); each variant trains successfully without code restructuring.

### Phase 4.1: Camera config + projection module

- [ ] T049 [P] [US3] Write contract test `tests/beacon_projection_tests.cc` per [contracts/beacon_projection_api.md §Test surface](./contracts/beacon_projection_api.md). Tests: `BeaconAtCameraOrigin_VisibleAtCenter`, `BeaconBehindCamera_NotVisible`, `BeaconOutsideFOV_NotVisible`, `BeaconAtFOVEdge_VisibleAtEdge`, `BeaconNearField_LargeAngularDisplacement`, `BeaconFarField_ScreenCoordsConverge`, `EmissionConeBackside_NotVisible`, `WrongWavelength_NotVisible`, `Determinism_RepeatCalls`, `NoAllocation_HotPath`. Tests MUST FAIL initially (red)
- [ ] T050 [US3] Create [`include/autoc/eval/camera_config.h`](../../include/autoc/eval/camera_config.h) per [data-model.md §3.1](./data-model.md): `CameraConfig` struct with compile-time-fixed and PRNG-varied parameter classes; v1 baseline `kDefaultCameraV1` constant (planar pinhole, 120° FOV, single forward-mounted, 30 Hz, 0 ms latency)
- [ ] T051 [US3] Create [`include/autoc/eval/beacon_projection.h`](../../include/autoc/eval/beacon_projection.h) per [contracts/beacon_projection_api.md](./contracts/beacon_projection_api.md): `BeaconProjectionResult` struct + `project_beacon()` function declaration
- [ ] T052 [US3] Create [`src/eval/beacon_projection.cc`](../../src/eval/beacon_projection.cc) — analytic planar pinhole projection in pure Eigen per [research.md R3](./research.md). Algorithm steps 1-7 from [contracts/beacon_projection_api.md](./contracts/beacon_projection_api.md). ~30-50 LOC. Aberration steps stubbed identity for v1
- [ ] T053 [US3] Add `src/eval/beacon_projection.cc` to `autoc_common` library in [`CMakeLists.txt`](../../CMakeLists.txt). Register `beacon_projection_tests` test executable
- [ ] T054 [US3] Run T049 tests — all 10 tests green. `bash scripts/rebuild.sh` green; full ctest passes

### Phase 4.2: Beacon model on target craft

- [ ] T055 [US3] Define beacon body-frame positions per [data-model.md §4.1](./data-model.md): `kBeaconLeftV1` (left wingtip, 850 nm IR), `kBeaconRightV1` (right wingtip, 940 nm IR). Compile-time constants in `camera_config.h`
- [ ] T056 [US3] Wire beacon world-position emission per tick into the playback target craft's pose: each NN tick, target's playback pose + body-frame beacon offset → world-frame beacon position. Lives in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` per-tick loop

### Phase 4.3: Frame buffer / history window

- [ ] T057 [US3] Create per-camera ring buffer infrastructure per [data-model.md §8.1](./data-model.md): `CameraFrameBuffer` struct with `std::deque` of last N frames (N = ceil(camera.frame_rate_hz / 10)). Sized for 30 Hz / 10 Hz NN tick = 3 frames minimum, with 16-frame buffer for the deepest -0.5s history offset
- [ ] T058 [US3] Sample selection per NN tick: pull frames at offsets `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` per [data-model.md §8.2](./data-model.md). Warm-up state when buffer is not yet full → sentinel `(0, 0, visible=0)` for not-yet-recorded slots
- [ ] T058a [US3] **De-scope FR-003e parameterization for v1**: v1 commits to FR-003e strategy (c) — "latest + raw past samples at fixed offsets," no derived velocity/accel and no sub-tick recurrent. The other strategies named in FR-003e (a) latest-only, (b) N-frame stack as parallel inputs, (d) NN runs at camera frame rate are deferred to a follow-up feature if v1 outcome motivates exploration. Document the de-scope rationale in `specs/029-tracker-mode/us3_outcome.md` (or create a stub doc to be filled at T067) so the FR-003e "must be parameterized" requirement is intentionally bounded to "future-work-friendly architectural shape" rather than v1 deliverable

### Phase 4.4: Type-safe sensor interface — beacon inputs (extends Phase 3.1's interface)

- [ ] T059 [US3] Add `TrackerSensorInput` enum to [`include/autoc/nn/sensor_interface.h`](../../include/autoc/nn/sensor_interface.h) — 44 entries per [contracts/nn_sensor_interface.md §Tracker-mode inputs](./contracts/nn_sensor_interface.md): `BEACON_L_X[t]`, `BEACON_L_Y[t]`, `BEACON_L_VISIBLE[t]` for each of 6 history offsets, mirrored for `BEACON_R_*`, plus 8 aircraft state inputs
- [ ] T060 [US3] Add `kTrackerSensorMeta[]` descriptor table per [contracts/nn_sensor_interface.md §Per-input metadata table](./contracts/nn_sensor_interface.md) — per-input names, units, ranges, categorical flags
- [ ] T061 [US3] Mode-conditional NN input layout: when `TrainingMode == TRACKER`, the NN input array is `SensorInputs<TrackerSensorInput>` (44-wide) instead of `SensorInputs<PathgenSensorInput>` (33-wide). Decision deferred to mode-selector — runtime polymorphism vs compile-time switch (pin in T061a)
- [ ] T061a [US3] Pin mode-selector mechanism: runtime mode dispatch (single binary handles both) vs compile-time switch (autoc-tracker built separately). Default recommendation: runtime — autoc binary handles both modes via mode-selector at startup. NN forward pass dispatches by mode
- [ ] T062 [US3] Tracker-mode input gathering: new `nn_input_computation_tracker.cc` (or extend existing) — populates `SensorInputs<TrackerSensorInput>` per tick from camera frame buffer + projection results + aircraft state

### Phase 4.5: US3 camera-config experimentation harness

- [ ] T063 [US3] Document the v1 sweep grid in `specs/029-tracker-mode/us3_sweep_grid.md`: variants to compare (FOV {60°, 90°, 120°}, mount {nose, canopy}, frame rate {30, 60})
- [ ] T064 [US3] Make `CameraConfig` selection in `autoc-tracker.ini`: option to either (a) name a preset compile-time config or (b) override individual fields. v1: preset selector by name (`CameraConfig = default_v1` etc.)
- [ ] T065 [US3] First short tracker-mode run at v1 baseline camera config: `nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-tracker-baseline-50gen.log 2>&1 &`. Target: 50-100 gens. Demonstrate sane fitness descent + visual lock signal (≥70% of ticks have at least one beacon visible)
- [ ] T066 [US3] Run camera-config sweep — at least 3 variants from the grid (e.g., FOV 60° / 120° / 180° fisheye). For each: edit camera config, rebuild, launch 50-gen run, compare per-axis aggressiveness. Document in `specs/029-tracker-mode/us3_camera_sweep.md`
- [ ] T067 [US3] Choose v1 baseline camera config for US4 long-run from the sweep results. Pin in `specs/029-tracker-mode/us3_outcome.md` with rationale

**Checkpoint** ✅ Camera + perception pipeline online. v1 baseline camera config chosen. ≥3 sweep variants validated. NN sees real beacons; short training run produces sane fitness shape.

---

## Phase 5: User Story 4 — Controller learns to track via beacon-camera signal alone (Priority: P1)

**Goal**: Produce a tracker-mode controller meeting SC-003 / SC-004 / SC-005 / SC-006 success criteria.

**Independent Test**: Long-run training at v1 baseline camera (chosen in T067), pop 5000 × 600 gens, recurrent NN, single seed. Demonstrate fitness descent shape qualitatively similar to pathgen mode. Late-run elite maintains visual lock ≥80% of ticks (SC-004); recovers from short FOV exits within bounded ticks (SC-005); per-axis aggressiveness in same range as pathgen-mode controllers (SC-006).

- [ ] T068 [US4] Update `autoc-tracker.ini` with v1 baseline camera config from T067. Confirm Path A training params (pop 5000 × 600 gens)
- [ ] T069 [US4] Launch long-run tracker training: `nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-tracker-base.log 2>&1 &`. Run name: `tracker-base`
- [ ] T070 [P] [US4] Set up monitoring loop — 6-panel evolution plot every 50 gens (using existing `plot_evolution_progress.py` — works unchanged because data.stc schema is unchanged for the per-gen log line), per-axis time series every 100 gens
- [ ] T071 [P] [US4] Apply early-stop criteria: kill if fitness flat above pid1's −27045 floor by gen 100, OR visual-lock fraction stuck below 30% by gen 200 (signals fundamental tracking failure)
- [ ] T072 [US4] At run completion (or early-stop), capture final metrics: late-plateau fitness (re-eval under fixed-difficulty), per-axis dCtrl + ⟨|out|⟩, visual-lock fraction (added per FR-008 secondary telemetry — pin landing location: extra field on `#NNGen` line OR sidecar log; see T072a). Compute SC-003/SC-004/SC-005/SC-006 evaluations
- [ ] T072a [US4] **FR-014 explicit verification — per-axis aggressiveness comparable to pathgen baseline**: Per FR-014 ("the same analysis tooling applies, unchanged in tracker mode"), run the existing `specs/028-deeper-rnn/plot_per_axis_time_series.py` against `tracker-base`'s `data.dat` and compare the dCtrl + ⟨|out|⟩ trajectories to more-rnn3's matched-gen trajectories. Pass criterion (SC-006): per-axis ranges within ~1.5× of pathgen-mode baseline (allowing modest aggressiveness increase from sparser visual signal). Document comparison in `specs/029-tracker-mode/us4_per_axis_comparison.md`. Also pin the visual-lock-fraction landing location (resolves the T070/T072 schema-change tension): if added to `#NNGen` line, update `plot_evolution_progress.py` to read the new field; if sidecar, ensure existing tooling still parses `data.stc` unchanged
- [ ] T073 [US4] Write outcome doc `specs/029-tracker-mode/tracker-base_outcome.md` — fitness vs pathgen-mode comparators, lock fraction, per-axis comparison (cite T072a), branch decision (US5/US6 close-out OR additional camera-config experiments needed)

**Checkpoint** ✅ tracker-base controller trained. SC-003 through SC-006 evaluated. Outcome documented.

---

## Phase 6: User Story 5 — Renderer dual-mode + per-tick scrub (Priority: P2)

**Goal**: Operator can inspect tracker-mode training in 3rd-person + 1st-person camera-POV views with per-tick scrub controls.

**Independent Test**: Open a recorded tracker-mode `.dmp` in the renderer. Switch between 3rd-person and 1st-person camera-POV views. Per-tick scrub controls work in both modes. Operator can identify per-tick beacon channel response, projection coordinates, and lock state within 30 seconds (SC-007); within 5 seconds for FOV/channel-state events (SC-012).

- [ ] T074 [P] [US5] Create [`tools/tracker_view_modes.h`](../../tools/tracker_view_modes.h) and `.cc` — view-mode state machine (3rd-person ↔ 1st-person) and shared rendering primitives
- [ ] T075 [US5] Modify [`tools/renderer.cc`](../../tools/renderer.cc) — detect tracker-mode dump (`tracker_mode == true` flag from FR-015 schema) and dispatch to dual-mode rendering. Existing pathgen-mode rendering unchanged for non-tracker dumps
- [ ] T076 [US5] Implement 3rd-person view per [spec FR-012](./spec.md#requirements-mandatory): both aircraft (training + target), beacons drawn on target wingtips (colored per channel), camera FOV cone from training craft, per-tick error metrics overlay
- [ ] T077 [US5] Implement 1st-person camera-POV view per [spec FR-012](./spec.md#requirements-mandatory): render *from training craft's camera*. Beacons appear as colored points at projected screen positions; FOV bounds at screen edges; aberration / rolling-shutter visible directly when enabled in camera config (v1: not enabled)
- [ ] T078 [US5] Per-tick scrub controls (FR-012a, rolled-in BACKLOG renderer-playback-enhancements): pause / step-forward / step-backward keyboard shortcuts work in both viewing modes. Updates camera-POV / 3rd-person renders synchronously with per-tick error overlay
- [ ] T079 [US5] Manual test: load a tracker-mode `.dmp` from Phase 5's training run. Verify (per US5 acceptance scenarios): both aircraft displayed, beacons colored, FOV cone drawn, error overlay updates with scrub, 1st-person view shows what controller sees, configuration changes propagate to FOV cone visualization (SC-010). Document validation in `specs/029-tracker-mode/us5_renderer_validation.md`

**Checkpoint** ✅ Renderer dual-mode + scrub controls operational. SC-007 / SC-012 / SC-010 validated.

---

## Phase 7: User Story 6 — Real-target-tracking bridge readiness (Priority: P3)

**Goal**: Confirm via architectural review that the perception-to-NN interface is structurally clean — same NN binary would work against real perception (cameras + FPGA centroids → coordinate output) without retraining.

**Independent Test**: Code review at end of Phase 5. Reviewer confirms: (a) NN forward pass takes typed sensor inputs (no raw pixel access), (b) projection module is a separable component (could be replaced with a real-perception module that produces equivalent `(x, y, visible)` tuples), (c) library entry format is replaceable (a real-perception module's per-tick output drops into the same TrackerSensorInput layout).

- [ ] T080 [US6] Architectural review: walk the codebase and document interface separation. Specifically inspect: `nn_input_computation_tracker.cc` (input gathering), `beacon_projection.h/.cc` (perception), `robot_path_provider.h/.cc` (target source), `sensor_interface.h` (NN-facing contract). Confirm none of these references any pixel-domain types or sim-only constructs that would prevent a real-perception swap
- [ ] T081 [US6] Document the architectural review in `specs/029-tracker-mode/us6_architecture_review.md` — "what would need to change to swap sim-perception for real-perception": likely just the projection module entry point + a per-tick coordinate ingestion endpoint. Confirm by sketching the hypothetical follow-on feature spec

**Checkpoint** ✅ Architectural review complete. Bridge to real-target-tracking is documented.

---

## Phase 8: Polish & Cross-Cutting Concerns

- [ ] T082 [P] Update [`specs/BACKLOG.md`](../BACKLOG.md) — mark `Type-Safe NN Sensor Interface` complete (rolled into 029 Phase 3.1). Mark `Renderer Playback Enhancements` (per-tick scrub + streak/multiplier overlay) complete (rolled into 029 Phase 6). Add cross-references to 029 outcome docs
- [ ] T083 [P] Generate findings doc `specs/029-tracker-mode/findings.md` — summarize US1 past-only outcome, US3 camera sweep results, US4 long-run outcome. List open questions and recommendations for the next-milestone (real-target tracking) feature
- [ ] T084 [P] Update memory entries: [`project_post_028_routing.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_post_028_routing.md) with 029 outcome + next-milestone routing. [`project_library_based_training.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md) with v1 implementation notes / learnings. [`reference_crrcsim_mod_robots.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md) updated to reflect that no `RobotProgrammable` subclass was needed (R4 finding)
- [ ] T085 Run [`quickstart.md`](./quickstart.md) walkthrough top-to-bottom on the final tree state. Confirm every step still works; update any commands that drifted during implementation
- [ ] T086 Final build + test verification: `bash scripts/rebuild.sh` clean, `ctest --output-on-failure` 100 % pass (all old + new tests), `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green
- [ ] T087 If win path (US4 outcome cleared SC-003/004/005/006): document next steps for flight test (xiao-port plan trigger). If bounded-no-go: write 029 close findings.md with carry-forward to 030 / next feature

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1 (Setup)
        ↓
Phase 2 (US1 past-only experiment) — runs FIRST on pristine 028
        ↓ (gates the rest of 029 — must PASS)
Phase 3 (US2 substrate) — Phase 3.1 sensor refactor can develop IN PARALLEL with Phase 2's training run, but doesn't merge until US1 passes
        ↓
Phase 4 (US3 camera + perception)
        ↓
Phase 5 (US4 long-run training)
        ↓
Phase 6 (US5 renderer)            Phase 7 (US6 review) ← can run in parallel with Phase 6
                                  ↓
Phase 8 (Polish & close)
```

### User Story Dependencies (linear with one parallel)

- **US1** (P1, MVP, FIRST): runs on pristine 028, depends only on Phase 1 setup. Gates everything else for 029 — pass/fail conditions all downstream work
- **US2** (P1, gateway): depends on US1 PASS. Phase 3.1 (sensor refactor) is the prerequisite for Phase 3.2+ within US2 — can develop in parallel with US1's training run, merges after US1 PASS
- **US3** (P1, camera config): depends on US2 substrate
- **US4** (P1, long-run): depends on US3 baseline camera config
- **US5** (P2, renderer): depends on US2 (loads tracker-mode dumps), can run in parallel with US3/US4
- **US6** (P3, review): runs at end of Phase 5, parallel with Phase 6

### Within Each Phase

- Tests (contract tests per [contracts/](./contracts/)) MUST be written and FAIL before implementation tasks (Constitution I)
- Models / data structures before services
- Services before integration
- Build verification gates each phase exit

### Parallel Opportunities

**Phase 2 (US1 training) ⊕ Phase 3.1 (sensor refactor) ⊕ Phase 3.3 (dmp converter)**:
- US1's 600-gen run takes ~24-48h calendar time. During that window:
  - Phase 3.1 sensor refactor can be implemented + tested + ready-to-merge (lands after US1 PASS confirms architecture)
  - Phase 3.3 dmp converter can be implemented + tested against fixture data (lands after Phase 3.1 + US1 PASS)
- Effectively: by the time US1's training completes, Phase 3 substrate is mostly built — just needs the integration step

**Within Phase 3.1** (sensor interface refactor):
- T016 (test) + T018 (header) + T023 (xiao migration) — different files, parallel-friendly

**Within Phase 3 substrate**:
- T029 (multi-aircraft test) + T033 (converter test) + T039 (schema test) — independent test files, parallel

**Within Phase 4** (camera + perception):
- T049 (projection test) + T055 (beacon model) + T057 (frame buffer) — independent

**Within Phase 8** (polish):
- T082 (BACKLOG update) + T083 (findings) + T084 (memory updates) — all documentation, parallel

### Cross-phase parallelism

- Phase 6 (renderer US5) and Phase 7 (architecture review US6) — different concerns, parallel
- Monitoring tasks during long-running training (T011, T012 during US1; T070, T071 during US4) run in parallel with the training itself

---

## Parallel Example: User Story 1 (running on pristine 028)

```bash
# US1 — pristine 028 codebase, two-line edit, then full training run:
Task: "T006 [US1] Update nn_inputs.h time-sample comment"
Task: "T007 [US1] Update nn_input_computation.cc time offsets"
# Build verification:
Task: "T008 [US1] bash scripts/rebuild.sh + ctest"
Task: "T009 [US1] xiao build verification"
# Launch + monitor (parallel with the training process):
Task: "T010 [US1] Launch the past-only training run (24-48h calendar)"
Task: "T011 [P] [US1] 6-panel plot regen every 50 gens"
Task: "T012 [P] [US1] Per-axis aggressiveness PNG every 100 gens"

# DURING the 24-48h US1 training, Phase 3.1 sensor refactor can develop in parallel
# (different files, doesn't merge until US1 PASS):
Task: "T016 [P] [US2] sensor_interface tests"
Task: "T018 [US2] sensor_interface.h header"
# ... etc through T028, ready to merge when US1 confirms PASS
```

---

## Implementation Strategy

### MVP — First Milestone (US1 only)

US1 is the user's stated "first experiment, gates the rest", running on pristine 028:

1. Complete Phase 1: Setup (T001–T005)
2. Complete Phase 2: US1 past-only experiment (T006–T015) — pristine 028, two-line code change, then 600-gen run
3. **STOP and DECIDE**: T015 outcome PASS or FAIL
   - PASS → MVP delivered; proceed to Phase 3 029 substrate
   - FAIL → 029 architectural rethink needed before Phase 3

This is the most consequential single experiment in 029. It runs cheap (~24-48 hours of compute) on pristine 028 (no refactor risk) and conditions whether the rest of 029 has a sound foundation.

### Incremental Delivery — 029 Substrate → Camera → Training

After US1 passes:

1. Phase 3 (US2 substrate) → starts with sensor refactor (Phase 3.1, ~13 tasks), then multi-aircraft + converter + RobotPathProvider + schema bump. End state: tracker mode launches but NN sees zeros → demonstrates plumbing works
2. Phase 4 (US3 camera) → NN now sees real beacons; short runs validate perception pipeline
3. Phase 5 (US4 long-run) → produces the actual tracker-mode controller → main research outcome
4. Phase 6 + 7 (US5 renderer + US6 review) → diagnostic / audit deliverables alongside US3/US4
5. Phase 8 (polish) → close 029, generate findings, hand off to next milestone

### Why This Order

- **US1 first on pristine 028** — cheapest experiment with the highest information density. Runs in parallel with 028 flight wait so no calendar cost. No refactor risk in the experimental signal
- **Phase 3.1 (sensor refactor) lands within US2 substrate, not before US1** — refactor pays off in US2+ when tracker-mode adds new beacon-named inputs; for US1 (which just tweaks time offsets in existing layout), it adds no value but adds risk
- **Phase 3 (US2 substrate) before Phase 4 (US3 camera)** — substrate provides "tracker mode runs end-to-end" guarantee; Phase 4 fills in the perception. If Phase 3 has bugs, they manifest cleanly without perception confounding
- **Phase 4 (US3 camera-config sweep) before Phase 5 (US4 long-run)** — camera-design lead time is on the project's critical path per US3. The long-run uses a *chosen* baseline; US3 picks it
- **Phase 6 (US5 renderer) and Phase 7 (US6 review) parallel** — different concerns; renderer is implementation, review is audit

---

## Notes

- `[P]` tasks = different files, no dependencies on incomplete tasks
- `[Story]` label maps task to user story for traceability
- Each user story is independently completable and testable per the Independent Test criteria
- Verify tests FAIL (red bar) before implementing (Constitution I)
- Commit after each logical group (e.g., after T015 — US1 outcome documented; after T028 — Phase 3.1 sensor refactor complete; after T048 — Phase 3 substrate online)
- **Stop at the US1 → US2 boundary** to confirm the architectural assumption holds before committing months of implementation effort
- Avoid: bypassing test-first ordering, skipping the determinism contract tests, conflating tracker-mode and pathgen-mode dumps in any single tool, attempting to implement perception before the substrate is verified, landing the sensor refactor before US1 (would muddy US1's experimental signal)
- Total estimated new C++ surface (per [research.md](./research.md)): ~600–800 LOC + ~270–330 LOC sensor interface refactor. Manageable single-feature scope.

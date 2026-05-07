---

description: "030 — Tracker Mode — task list (fresh rewrite 2026-05-04 against current spec/plan/research)"
---

# Tasks: 030 Tracker Mode

**Input**: [`specs/030-tracker-mode/`](.) — spec.md (29 FRs, 16 design notes, 5 clarifications session 2026-05-04), plan.md (M0–M11 milestone ramp, fresh 2026-05-04), research.md (R1–R12, fresh 2026-05-04), data-model.md, contracts/ (4 files), quickstart.md
**Constitution**: v1.1.0 — Principle I (Testing-First) requires tests for all significant changes; tests are inlined in each phase below, NOT optional.

**Organization**: Tasks grouped by user story per the speckit template, but phase ordering follows plan.md's smoke-test-first milestone ramp (M0 done → M1 → M2 → M3 → M5 → M6 → M7 → M8 → M9 → M10 → M11). The active user stories for v1 are **US2 (gateway / training launch)**, **US4 (signal-or-not — smoke test)**, **US5 (renderer inspection)**. US1 is already complete in 029-no-future-arch and is excluded from this task list. **US3 (camera-config experimentation)** and **US6 (real-target-tracking bridge)** are deferred from v1 per D13 / D15 — see [BACKLOG.md "030 spin-offs"](../BACKLOG.md) for the 031-candidate routing.

---

> ## ⏸ Session pause — 2026-05-06
>
> **Status**: M1 + M2 + M3 complete (12 commits on `030-tracker-mode` branch, latest is the M3-status doc commit). Eval-vs-training fitness bitwise `-55944.664164` under `rebuild-perf.sh` baseline `gen9200.dmp`. Operator independently re-verified post-M3: fresh nnextractor + nn2cpp regenerate cleanly + eval still bitwise; xiao pio still SUCCESS.
>
> **Next session pickup**: **M5 — Beacon projection module** (T025–T031). The first M3-loader consumer, and the first place `TrackerInput` enum slots become load-bearing (BEACON_L_X_NOW, BEACON_L_CEP_NOW, etc.). Resolves R6 (CEP encoding) + R7 (int8 quantization math) from research.md. v1 baseline per spec D10: planar pinhole, 120° FOV, 30 Hz, top-of-wing-chord mount; beacon emission cones 270° outward at wingtip body-frame positions per FR-004.
>
> **Suggested M5 sub-checkpoints**:
> - **M5a** — `camera_config.h` + `beacon_config.h` config structs (v1 fixed; PRNG-varied placeholders); test scaffolding.
> - **M5b** — Analytic pinhole projection + AirframeProxy ray-box self-occlusion (D10 first-order fidelity).
> - **M5c** — CEP encoding (R6) + int8 quantization round-trip (R7) + sentinel handling.
>
> Regression-tight gate at every M5 sub-checkpoint: rebuild-perf.sh + autoc-eval bitwise. M5 adds new code on a separate path, so regression invariant should hold trivially throughout.
>
> **Open principle locked in by M3a discovery** (per [memory: feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md)): at every dmp schema-bump boundary, audit `AircraftState::serialize()` against the full sensor inventory. The M8 v=2 schema bump (T048) carries this audit — gyroRates and any other transient-only fields land as serialized fields then.

---

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Maps to spec.md user stories (US2, US4, US5) — Setup / Foundational / Polish phases have no story label
- File paths are relative to repo root (`/home/gmcnutt/autoc/`)

## Path Conventions

Single-repo C++ tree (per plan.md Project Structure):
- `src/`, `include/autoc/`, `tests/`, `tools/`, `crrcsim/src/`, `xiao/src/` at repo root
- New eval-side modules in `src/eval/` and `include/autoc/eval/`
- Spec dir: `specs/030-tracker-mode/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prerequisite work to receive 030 changes. M0 (plan-research) already complete.

- [X] T001 Create branch `030-tracker-mode` from current `029-no-future-arch`; document branch creation in commit message referencing this plan _(verified — branch rooted at origin/main `0b12070` = Merge PR #3 from 029-no-future-arch; descends from 029 work)_
- [X] T002 [P] ~~Pull latest source M1 dmp from S3 to local fixture path~~ — **dropped 2026-05-06**. Approach revised: tests are hermetic via synthetic in-code fixtures (T020 generates an `EvalResults` programmatically with cereal, no aws-CLI dependency). Real-dmp testing happens in T020a integration test against the embedded S3 client (`ConfigManager::getS3Client()` per `src/util/config.cc:133`), gated on credential availability. Reference run for any post-implementation operator smoke: `autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp` (pastonly3 gen 800 = "latest by search-order default")

---

## Phase 2: Foundational (M1 + M2 — blocking prerequisites for all user stories)

**Purpose**: Build hygiene + type-safe scaffolding. **No user-story implementation can begin until this phase is complete.**

**⚠️ CRITICAL**: Per plan.md M1 ordering rationale, the mod_inputdev linkage fix MUST land before any new file in `src/nn/` or `src/eval/` is added — otherwise the next file addition silently breaks the crrcsim build at link.

### M1 — Build precondition + dmp version-field groundwork

- [X] T003 Replace cherry-picked source compilation in [`crrcsim/src/mod_inputdev/CMakeLists.txt:21-23`](../../crrcsim/src/mod_inputdev/CMakeLists.txt) with `target_link_libraries(mod_inputdev autoc_common)`; remove the three cherry-picked .cc lines; verify `bash scripts/rebuild.sh` clean _(edit applied 2026-05-06; build verification pending operator run)_
- [X] T004 Audit any duplicate-symbol issues exposed by T003's autoc_common link; reconcile by moving offending definitions out of headers if necessary _(2026-05-06: clean build, no duplicate-symbol or undefined-reference errors; static-archive linker stops transitive .o pull at evaluator/serialization/sensor_math + their direct deps, so AWS deps from autoc_common's config.cc never reach crrcsim)_
- [X] T005 [P] Confirm `CEREAL_CLASS_VERSION(EvalResults, 1)` already in [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h) (it is per R8 finding); add a contract test `tests/cereal_version_anchor_tests.cc` that fails loudly if the version constant changes without an explicit task to bump it (catches accidental version drift) _(2026-05-06: tests/cereal_version_anchor_tests.cc added; anchors EvalResults / ScenarioMetadata / EvalData all at version 1; wired into CMakeLists.txt + run_autoc_tests)_
- [X] T006 Verify `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` clean after T003 to ensure xiao build is unaffected _(2026-05-06: SUCCESS, 44.2% flash; only pre-existing third-party warnings)_

### M2 — Type-safe NN sensor interface (FR-006, FR-019 scaffolding)

**Per FR-019 mode-dispatch architecture, scaffolding has TWO enums (PathgenInput, TrackerInput) coexisting in one binary.**

- [X] T007 [P] Contract test `tests/nn_sensor_interface_tests.cc` — typed-name → enum → name round-trip identity for both `PathgenInput` and `TrackerInput`; assert `static_cast<size_t>(PathgenInput::COUNT) == 33` and `static_cast<size_t>(TrackerInput::COUNT) == 48` (per FR-006 + FR-016 arena-awareness) _(2026-05-06: 8 assertions green — anchor positions, meta-array well-formedness, display-name lock for M2b regression)_
- [ ] T008 [P] Contract test `tests/mode_dispatch_tests.cc` — load `autoc.ini` → mode = pathgen + active enum = `PathgenInput`; load `autoc-tracker.ini` → mode = tracker + active enum = `TrackerInput`; mutually-exclusive parameter rejection _(deferred to M6 when actual ModeStrategy + mode-selection-from-config wires up)_
- [X] T009 Create [`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h) with `enum class PathgenInput : uint16_t` (33 entries matching current pathgen `NNInputs` struct) + `kPathgenInputMeta` parallel `constexpr` array; `static_assert(static_cast<size_t>(PathgenInput::COUNT) == NN_INPUT_COUNT)` _(2026-05-06: enum + 33-entry meta added; static_asserts couple PathgenInput::COUNT == NN_INPUT_COUNT and meta length == enum count)_
- [X] T010 Add `enum class TrackerInput : uint16_t` to [`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h) (48 entries: 36 beacon — `BEACON_L_X/Y/CEP_TM5..NOW`, `BEACON_R_X/Y/CEP_TM5..NOW`; 8 aircraft state — `QUAT_W/X/Y/Z`, `AIRSPEED`, `GYRO_P/Q/R`; 4 arena-awareness — `HOME_X/Y/Z`, `HOME_DIST` per FR-016 clarification); `kTrackerInputMeta` parallel array _(2026-05-06: enum + 48-entry meta added with placeholder display names — actual data.dat tracker-mode columns wire up in M5)_
- [X] T011 [P] Update [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h) to derive input-layer width from active enum (compile-time mode select) per FR-019; consolidate the scattered `NN_INPUT_COUNT` magic-number defines _(M2a partial → M7a complete 2026-05-07: TRACKER_NN_TOPOLOGY {45,32,16,3} parallel constants in topology.h; ModeStrategy bundle in mode.h carries topology pointers; getActiveModeStrategy() / getModeStrategyByName() helpers; minisim genome validation + autoc.cc population init both mode-aware. Operator-confirmed pathgen bitwise on gen9200.dmp + tracker NN now 2307-weight 45-input genome (gpBytes=9288 vs pathgen 7752))_
- [X] T012 Update [`include/autoc/autoc.h`](../../include/autoc/autoc.h) to remove duplicate `DISTANCE_TARGET` etc. defines (now sourced from `nn_inputs.h` typed names) _(2026-05-06: no-op — grep confirmed no DISTANCE_TARGET-style magic-number defines exist as obstacles. Spec-implied cleanup not needed)_
- [X] T013 [P] Refactor [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc) `nn_gather_inputs()` into pluggable `gather_pathgen_inputs()` + `gather_tracker_inputs()` strategy (per FR-019); pathgen path is byte-identical to pre-M2 behavior _(M2b.1 / 0c1ce11: nn_gather_inputs → gather_pathgen_inputs rename, body byte-identical; ModeStrategy bundle in include/autoc/nn/mode.h + src/nn/mode.cc; gather_tracker_inputs is loud-fail stub until M5; eval-vs-training fitness bitwise match -55944.664164)_
- [X] T014 [P] Update [`src/autoc.cc`](../../src/autoc.cc) `data.dat` header emission to walk `kPathgenInputMeta` / `kTrackerInputMeta` (mode-aware) for canonical column names _(M2b.2 / 31e7b70: pathgen header walks kPathgenInputMeta with std::setw(header_width); contract test PathgenMetaWalkProducesExistingHeaderText asserts byte-identity vs literal pre-M2b string; tracker-mode header walk lands in M5)_
- [~] T015 [P] Update [`tests/contract_evaluator_tests.cc`](../../tests/contract_evaluator_tests.cc) and [`tests/nn_evaluator_tests.cc`](../../tests/nn_evaluator_tests.cc) to assert against typed names; existing pathgen-mode tests stay green (regression-tight invariant) _(2026-05-06: deferred — cosmetic. Existing tests use NN_INPUT_COUNT as buffer size; static_assert(PathgenInput::COUNT == NN_INPUT_COUNT) in nn_inputs.h already couples them at compile time. Adding typed-name EXPECT_EQ at runtime is verbose with no safety win. Will batch when downstream M5+ work introduces a typed-name assertion gap)_
- [~] T016 [P] Update [`specs/019-improved-crrcsim/sim_response.py`](../../specs/019-improved-crrcsim/sim_response.py) parser to key off enum-derived header names (no hardcoded column positions) _(2026-05-06: no-op for M2 — header is byte-identical to pre-M2 (PathgenMetaWalkProducesExistingHeaderText proves it), so the existing column-position-based parser still works. Will revisit if/when Python tooling actively benefits from typed-name introspection — likely M11a per-tick dmp extractor)_
- [~] T017 [P] Update [`xiao/src/msplink.cpp`](../../xiao/src/msplink.cpp) to use the typed enum mirror; preprocessor-select active mode per FR-019 compile-time selection (`-DAUTOC_MODE=PATHGEN` for v1 xiao deploy) _(M2b.1 partial: xiao builds clean post-rename — pio SUCCESS, 44.2% flash. xiao cherry-picks the same .cc files (now defining gather_pathgen_inputs) and the PathgenInput typed enum is in the shared header nn_inputs.h. Full -DAUTOC_MODE preprocessor compile-time-mode-select defers to M5+ when xiao starts coexisting with tracker code paths)_
- [~] T018 [P] Update [`include/autoc/eval/aircraft_state.h`](../../include/autoc/eval/aircraft_state.h) `nnInputs_` array typing + serialization to preserve typed names _(2026-05-06: deferred — cosmetic. nnInputs_ is already a typed NNInputs struct; cereal serialization uses NN_INPUT_COUNT as payload-size invariant which is statically coupled to PathgenInput::COUNT. Replacing the constant is verbose with no win. Will batch when downstream M5+ work introduces a typed-name serialization gap)_
- [X] T019 Regression-tight gate post-M2 — **two complementary checks**, both built with `bash scripts/rebuild-perf.sh` (the FP-deterministic build, per [memory: reference_perf_build_reproducibility](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_perf_build_reproducibility.md); `rebuild.sh` debug builds intentionally produce different FP values and are NOT regression baselines):
  1. **byte-identical data.dat**: ✓ contract test `PathgenMetaWalkProducesExistingHeaderText` (2026-05-06) asserts unit-test-time that the meta-walk header bytes match the literal pre-M2 string; data row format string at autoc.cc:677-690 is untouched; values come from `gather_pathgen_inputs` which is byte-identical to the renamed-from `nn_gather_inputs`. Byte-identity holds robustly.
  2. **eval-vs-training fitness equivalence**: ✓ `autoc -i autoc-eval.ini` on baseline `gen9999.dmp` extracted `nn_weights.dat` post-M2b shows `NN Eval fitness: -55944.664164 == Stored fitness: -55944.664164` (operator-confirmed 2026-05-06 after both M2b.1 rename and M2b.2 header-walk refactors).

**Checkpoint**: M1 + M2 complete. Build is clean; type-safe scaffolding is in place; pathgen mode unchanged. User-story implementation can begin.

---

## Phase 3: User Story 2 — Operator launches tracker-mode training (Priority: P1) 🎯 MVP gateway

**Goal**: Operator points `autoc-tracker.ini` at a source M1 dmp; autoc loads it, runs tracker-mode training; per-gen log shows fitness statistics + tracker-specific telemetry.

**Independent Test**: Per quickstart.md — launch `build/autoc -i autoc-tracker.ini`; expect log lines showing source dmp loaded, scenario slice (6 paths × 20 winds = 120 scenarios) distributed, gen 0 evaluations executing, gen 1+ fitness statistics improving over baseline.

**Maps to plan.md milestones**: M3 (source dmp loader) → M5 (projection — M4 deferred per FR-002 v1 deferral note + plan §M4) → M6 (autoc-tracker.ini + main loop) → M7 (trail rabbit + crash hull + arena fitness).

### M3 — Source M1 dmp loader (FR-001)

- [X] T020 [P] [US2] Contract test `tests/source_dmp_loading_tests.cc` — **hermetic via synthetic fixture**: in-test, build an `EvalResults` programmatically with 2-3 scenarios × ~10 ticks × known quat/pos values, serialize via cereal to a tmp path, then load it back through `loadSourceDmp(tmp_path)`. Assert exact-match against the synthetic content; assert scenario count, monotonic timestamps, quat magnitude in `[0.99, 1.01]`, position bounded < 10 km; reject truncated scenarios (< `MIN_SCENARIO_TICKS = 30`); validate S3 key parser round-trip on `autoc-storage/<run-id>/gen<N>.dmp` form. No aws-CLI / no S3 / no committed binary fixture _(M3a / b081e45: 7 hermetic assertions all green)_
- [X] T020a [US2] Integration test `tests/source_dmp_s3_integration_tests.cc` — gated on env var `AUTOC_S3_TESTS=1`; loads a real dmp by S3 key via the embedded `ConfigManager::getS3Client()` (per `src/util/config.cc:133`) following the `tools/nnextractor.cc:98-166` pattern. Skips silently when env var unset. Reference S3 key: `autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp`. This test exercises the production load path end-to-end _(M3b / 090f9ce: PASSED against real reference dmp 2026-05-06; loads 294 scenarios, validates first-tick quat-norm + position bounds)_
- [X] T021 [P] [US2] Define [`include/autoc/eval/source_trajectory.h`](../../include/autoc/eval/source_trajectory.h) — `SourceTickSample` struct (simTimeMsec, position, orientation, velocity, angularRate) + `SourceScenarioTrajectory` struct (scenarioIndex, variation, samples[]) _(M3a / b081e45: angularRate intentionally omitted per v1 cereal schema gap — see header note. Numerical-differentiation alternative or M8 schema-bump-to-v2 can revisit)_
- [X] T022 [US2] Implement `loadSourceDmp(s3_key_or_path)` in [`src/eval/source_dmp_loader.cc`](../../src/eval/source_dmp_loader.cc) following `tools/nnextractor.cc:177-192` cereal pattern — accepts an S3 key (production path) and streams the dmp directly via the existing S3 client; local file paths accepted as offline-test convenience but not the canonical input; throws on Constitution V version mismatch (loud-fail per FR-015a) _(M3a / b081e45: dual local-file/S3 resolution via std::filesystem::exists; cereal failures wrapped in std::runtime_error with Constitution V loud-fail message)_
- [X] T023 [P] [US2] Implement `filterByScenarioIndex()` for the path × wind subset selection per FR-011 clarification (cross-product subsetting) _(M3a / b081e45: empty subset on either axis = "all" identity)_
- [X] T023a [P] [US2] Implement `filterCrashedSourceScenarios()` per FR-013 — at source-dmp load time, filter out scenarios where the source run's aircraft crashed mid-scenario (detected via terminal-tick crash flag in source `EvalResults`). v1 default: **skip** (filter out entirely), simpler than truncate. Log the count of filtered scenarios. Wire ahead of T023's path × wind subsetting so the cross-product operates on clean scenarios only. Contract assertion: filtered scenarios are absent from the resulting `vector<SourceScenarioTrajectory>` _(M3a / b081e45: uses protocol.h::isCrash() — Boot/Sim/Eval drop, TimeLimit/RabbitComplete keep)_
- [X] T024 [US2] Add standalone CLI tool `tools/source_dmp_inspect.cc` that loads a source dmp **directly by S3 key** (e.g. `tools/source_dmp_inspect autoc-storage/<run-id>/gen391.dmp` — no local-cache step required) and prints per-scenario summary (count, tick count, sample target pose at tick 0 and middle); local file paths accepted as offline-test convenience; operator sanity-check before any tracker-mode run _(M3b / 090f9ce: scenario count + tick min/mean/max + first 5 scenarios' variation params + sample first/middle/last pose of scenario 0; operator-validated against reference dmp returning 294 scenarios)_

### M5 — Beacon projection module (FR-003 + FR-004 + FR-005 + FR-007 + FR-017)

- [X] T025 [P] [US2] Contract test `tests/beacon_projection_tests.cc` — known-geometry assertions: target dead-ahead → `(x≈0, y≈0, cep≈0)`; target left-edge → `screen_x≈-1, cep elevated`; target behind → sentinel; target occluded by airframe proxy → sentinel; target outside emission cone (tail-on aspect) → sentinel _(M5 / 2026-05-06: 16 assertions across geometry / sentinel / quantization / determinism — all green; rebuild-perf.sh + autoc-eval bitwise gate confirmed by operator)_
- [X] T026 [P] [US2] Contract test (continuation of T025) — int8 round-trip: `dequantize_xy(quantize_xy(x))` within `1/127` step for visible values; `dequantize_cep(quantize_cep(any_value >= 1.25)) == kCepSentinelFloat (1.5f)` exactly _(M5 / 2026-05-06: included in beacon_projection_tests.cc; XyRoundTripWithinOneStep + CepVisibleRoundTripWithinOneStep + CepSentinelExactRoundTrip + XyClampsOutOfRangeInputs all green)_
- [X] T027 [P] [US2] Define [`include/autoc/eval/camera_config.h`](../../include/autoc/eval/camera_config.h) — `CameraConfig` struct with v1 baseline values (planar pinhole, 120° FOV, 30 Hz, top-of-wing-chord mount); compile-time-fixed fields + PRNG-varied placeholders (sigmas at zero in v1) _(M5 / 2026-05-06: gp_scalar for FOV/frame-rate/latency, gp_vec3/gp_quat for mount; PRNG-varied placeholders in place at zero sigma)_
- [X] T028 [P] [US2] Define [`include/autoc/eval/beacon_config.h`](../../include/autoc/eval/beacon_config.h) — `BeaconConfig` struct with v1 baseline (270° outward emission cone, hb1 wingtip body-frame positions ±0.45 m on Y axis, distinct IR wavelengths) _(M5 / 2026-05-06)_
- [X] T029 [P] [US2] Define [`include/autoc/eval/camera_projection.h`](../../include/autoc/eval/camera_projection.h) — `BeaconObservation` struct (NN-facing fp32 + raw int8 raw_*_int8 fields); `ProjectionInput` struct; quantize / dequantize helpers + `kCepSentinelThreshold (1.25f)` / `kCepSentinelFloat (1.5f)` constants _(M5 / 2026-05-06)_
- [X] T030 [US2] Implement `projectBeacon(input)` in [`src/eval/camera_projection.cc`](../../src/eval/camera_projection.cc) per the contract math in `contracts/beacon_projection_api.md` — Eigen-based analytic pinhole projection, FOV / behind / emission-cone / airframe-proxy occlusion sentinel checks, CEP linear encoding [0, 1] with edge-factor, int8 quantize + dequantize round-trip _(M5 / 2026-05-06: NED body-frame convention adapted from contract's z-forward convention — body +x = optical axis; documented in header. autoc_common adds camera_projection.cc; rebuild-perf.sh + autoc-eval bitwise on baseline gen9200.dmp confirmed clean)_
- [X] T031 [US2] Implement `AirframeProxy` ray-box intersection in [`src/eval/camera_projection.cc`](../../src/eval/camera_projection.cc); v1 default proxy = hb1 fuselage + wing AABB; coarse but calibrated against operator's reference video as plan-research deliverable _(M5 / 2026-05-06: slab-method ray-AABB; defaultAirframeProxyHB1() helper for v1 default; per-test custom proxy override exercises hit / miss / segment-ends-before-box edge cases)_

### M6 — Tracker-mode autoc.ini + main-loop branch (FR-011 + FR-018 + FR-019)

- [ ] T032 [P] [US2] Contract test `tests/timing_model_tests.cc` — FR-018 + FR-009 determinism: same source dmp + same seed → bit-identical M2 trajectory across two invocations regardless of sim-clock speed; variable-rate source samples (synthetic 50 ms / 73 ms / 100 ms intervals) handled correctly without interpolation drift
- [X] T033 [P] [US2] Define `autoc-tracker.ini` template in [`autoc-tracker.ini.template`](../../autoc-tracker.ini.template) at repo root with v1 default values per quickstart.md (source key, scenario subset = 6 paths × 20 winds, trail = 3.048 m, crash hull = 1m sphere with curriculum p_crash, arena = 80m radius / 5m floor / 100m ceiling, camera + beacon configs) _(M6b / 2026-05-06: file at repo root; reference `TrackerSourceRun = autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp` matches M3 reference dmp; smoke slice = 6 paths × 20 winds = 120 scenarios per Q2)_
- [X] T034 [US2] Extend [`src/util/config.cc`](../../src/util/config.cc) inih-based parser to read tracker-specific keys (per data-model.md §2 — sections were illustrative; existing autoc.ini convention is no-section + descriptive prefix, kept consistent); add mutual-exclusion validation (`Mode = tracker` requires `TrackerSourceRun`; loud-fail on missing) _(M6b / 2026-05-06: AutocConfig extended with mode + 30 tracker fields (source, trail, crash hull, arena, camera, beacon); ConfigManager::initialize parses each + validates Mode ∈ {pathgen, tracker} and TrackerSourceRun present when tracker. Strict pathgen-only-rejection deferred to v2 — inih reader doesn't expose key-was-set vs default. Smoke-tested loud-fail message via build/autoc -i invalid.ini. tests/contract_tracker_config_tests.cc: 9 assertions cover Mode default + tracker keys + .ini.template parses-clean)_
- [X] T035 [US2] Add mode dispatch in [`src/autoc.cc`](../../src/autoc.cc) — config-file detection determines pathgen vs tracker; instantiates the right `gather_inputs()` strategy (FR-019), the right typed enum, and the right `ScenarioStepper` strategy (R5) _(M6c / 02d0acf: EvalData::mode field plumbs ConfigManager::config->mode through to minisim worker; minisim dispatches PathgenStepper vs TrackerStepper. Pathgen path byte-identical. End-to-end smoke run 2026-05-06 confirmed: source dmp loads (294 → 1 after subset filter), workers log mode=tracker, gen 0 evaluations execute, fitness numbers reported. Pathgen regression bitwise-confirmed including renderer m1 dmp read.)_
- [X] T036 [US2] Refactor existing pathgen per-tick loop in [`src/autoc.cc`](../../src/autoc.cc) into `PathgenStepper : public ScenarioStepper` (no behavior change; existing tests stay green — regression-tight) _(M6a / 1572998: actual extraction site is `tools/minisim.cc` not `src/autoc.cc` — task docstring path was wrong, real worker per-tick lives in minisim. include/autoc/eval/{scenario_stepper,pathgen_stepper}.h + src/eval/pathgen_stepper.cc; minisim inner-loop reduced from ~120 lines to ~12 delegating to PathgenStepper. Termination last-write-wins preserved (RabbitComplete > TimeLimit > Eval); push_back cadence preserved. rebuild-perf.sh + autoc-eval bitwise on baseline gen9200.dmp confirmed bitwise-identical 2026-05-06)_
- [X] T037 [US2] Implement `TrackerStepper : public ScenarioStepper` in [`src/eval/tracker_stepper.cc`](../../src/eval/tracker_stepper.cc) — drives per-tick loop off M1 source timestamps per FR-018; advances target state from `SourceScenarioTrajectory.sample(t_i)`; calls `projectBeacon()` for each (camera, beacon) pair; populates NN inputs via `gather_tracker_inputs()`; runs NN forward pass; advances chase-craft physics until `t_{i+1}` _(M6d / 7c48809 + M6e / 84d3942: TrackerStepper class symmetric with PathgenStepper, source-trajectory-driven loop, M5 projection per tick, 6-slot beacon history ring buffer, gather_tracker_inputs fills 48 floats, evaluateTracker forward pass. Topology mode-select deferred to M7+ — current 33-input pathgen topology consumes only beacon channels via reinterpret_cast, NN steers blind to its own quat/airspeed/gyro/home. M7 architecture decision pending. Smoke run 2026-05-06 confirmed loop closes structurally; fitness signal is meaningless until M7 wires trail-rabbit fitness)_
- [X] T038 [US2] Wire source dmp loading at autoc startup (T022) to populate `vector<SourceScenarioTrajectory>` available to the worker contract; per-scenario distribution to workers analogous to pathgen. Preserves FR-010 joint-PRNG variation model — source-scenario-index becomes one of the joint axes alongside wind/entry/craft (which are inherited from the source recording per FR-001) _(M6e / 84d3942: gSourceTrajectoryList loaded at autoc startup via M3 loadSourceDmp + filterByScenarioIndex with parseCsvIndices helper. EvalData carries vector<SourceScenarioTrajectory> + camera/beacon/airframe/home configs cereal-serialized. ScenarioMetadata factored into rpc/scenario_metadata.h to break circular include. filterCrashedSourceScenarios deferred — needs crashReasonList plumbing through loadSourceDmp; tracker stepper's source-exhaustion fallback handles short trajectories gracefully. Smoke run 2026-05-06 confirmed: 294 scenarios loaded, 1 after subset filter, scenarios distribute to workers, TrackerStepper fires per scenario)_

### M7 — Tracker-mode fitness (FR-008 + FR-008a + FR-008b + FR-016)

**Sub-checkpoint sequencing** (revised 2026-05-07 per Session 2026-05-07 Q1):

| Sub | Tasks | Scope |
|---|---|---|
| **M7a** | T044 + T041 + T011 finish-up + TrackerInputs reshape | `arena.h` (`FlightArena` + `distanceToBoundary` + `checkArenaBounds` + `ArenaEgressKind`); replaces M8b shim's `checkAircraftOOB` for tracker (pathgen keeps M1 hardcoded path); `TrackerInputs` slot rework — drop `HOME_X/Y/Z/HOME_DIST` (4 slots), add single `DIST_TO_BOUNDARY_ALONG_VEL` (1 slot); `TrackerInput::COUNT = 48 → 45`; topology mode-select 33→45; `gather_tracker_inputs` consumes `arena.h` for the new input. Arena-input math + per-tick check are the same function (single source of truth). |
| **M7b** | T042 + T039 | `trail_rabbit.h/.cc` — simplified `target_pos − velocity_unit × 3.048m` (FR-008). FR-008a nose-fallback **deferred to 031 backlog** per Session 2026-05-07; v1 fallback for `\|velocity\| < 1e-3` is `rabbit = target_pos`. |
| **M7c** | T043 + T040 | `crash_hull.h/.cc` — sphere intersection + curriculum-anneal `p_crash` (FR-008b + R3). Mode-gated. |
| **M7d** | T045 + T046 | `FitnessComputer` wire-up — cone-surface against trail-rabbit; crash-hull strike + arena egress as scenario terminators; per-scenario telemetry counters populated. |

**Bitwise risk** highest at M7d (FitnessComputer touch). M7a/b/c are mostly additive (+ TrackerInputs rename, which is tracker-mode-only).

- [X] T039 [P] [US2] Contract test `tests/trail_rabbit_tests.cc` — FR-008 math identity (`rabbit = target_pos - velocity_unit × 3.048`). ~~FR-008a degenerate-velocity fallback per R10 (hard-switch at 2 m/s with hysteresis ±0.5)~~ **Amended 2026-05-07 per Session 2026-05-07 Q1**: nose-fallback deferred; v1 degenerate fallback (`\|velocity\| < 1e-3` ⇒ `rabbit = target_pos`) is the only fallback case asserted. (M7b) _(M7b / 2026-05-07: 7 contract tests green — forward-flying / diagonal-velocity / configurable-distance / zero-velocity / near-zero-velocity-threshold / descending-target / pure-function determinism)_
- [X] T040 [P] [US2] Contract test `tests/crash_hull_tests.cc` — FR-008b sphere-intersection identity at boundary tangent / inside / outside; `p_crash` curriculum per the schedule resolved in [research.md R3](./research.md#r3--p_crash-v1-default) (curriculum-anneal 0.0 → 0.30 by gen 200 within deterministic seed); mode-gated (pathgen scenarios never invoke hull check). (M7c) _(M7c / 2026-05-07: 17 contract tests green — 6 isInsideHull (origin / boundary / outside / radius-configurable / reserved-shapes) + 5 didCrashFire (outside-no-fire / p=0 / p=1 / determinism / Bernoulli rate) + 6 pCrashForGen (gen0 / negative-gen / plateau / beyond / linear-interp / configurable))_
- [X] T041 [P] [US2] Contract test `tests/arena_tests.cc` — FR-016 boundary checks: chase outside radius → egress with `RADIUS` flag; below floor → `FLOOR`; above ceiling → `CEILING`; scenario-terminating per Q3 clarification; **`DIST_TO_BOUNDARY_ALONG_VEL` ray-projection identity** (cylinder quadratic + floor/ceiling linear; min positive t; sentinel when no intersection along velocity) — same function feeds NN input AND OOB termination per Session 2026-05-07 Q1. (M7a) _(M7a / 2026-05-07: 11 contract tests green — 5 checkArenaBounds (NONE / RADIUS / FLOOR / CEILING / RADIUS-priority) + 6 distanceToBoundary (cylinder hit / near-wall / heading-away / floor descent / ceiling climb / sentinel-on-zero-velocity))_
- [X] T042 [P] [US2] Define + implement [`include/autoc/eval/trail_rabbit.h`](../../include/autoc/eval/trail_rabbit.h) + [`src/eval/trail_rabbit.cc`](../../src/eval/trail_rabbit.cc) — `computeTrailRabbit()`: `target_pos − target.velocity / |target.velocity| × trail_distance` per data-model.md §6. **Simplified per Session 2026-05-07 Q1**: degenerate-velocity branch returns `target_pos` (no nose-direction rotation, no hysteresis). (M7b) _(M7b / 2026-05-07: trail_rabbit.h + trail_rabbit.cc landed; TrackerStepper.projectAndShiftHistory now writes real trail rabbit into last_target_sample_ for v=2 dmp output (was placeholder = target.position). Trail distance defaults to kDefaultTrailDistance (3.048m); M7d plumbs configurable cfg.trailDistance through EvalData)_
- [X] T043 [P] [US2] Define + implement [`include/autoc/eval/crash_hull.h`](../../include/autoc/eval/crash_hull.h) + [`src/eval/crash_hull.cc`](../../src/eval/crash_hull.cc) — `isInsideHull()` + `didCrashFire()` with curriculum-anneal `p_crash` per R3. (M7c) _(M7c / 2026-05-07: crash_hull.h + crash_hull.cc landed; SPHERE shape v1, AABB_HB1 + MESH_AIRFRAME reserved; Park-Miller LCG for deterministic Bernoulli draw with caller-managed PRNG state; pCrashForGen linear-ramp curriculum 0→0.30 across gens 0..200. TrackerStepper.projectAndShiftHistory now writes the geometric inside_crash_hull flag into v=2 dmp recording (was hardcoded false). Probabilistic firing + scenario termination are M7d.)_
- [X] T044 [P] [US2] Define + implement [`include/autoc/eval/arena.h`](../../include/autoc/eval/arena.h) + [`src/eval/arena.cc`](../../src/eval/arena.cc) — `FlightArena` struct + `checkArenaBounds()` returning `ArenaEgressKind` + **`distanceToBoundary(pos, vel_unit, arena)`** ray-projection scalar (Session 2026-05-07 Q1: same function feeds NN input AND egress check). Replaces M8b shim's `checkAircraftOOB` on the tracker side. (M7a) _(M7a / 2026-05-07: arena.h + arena.cc landed; FlightArena cereal-serialized in EvalData; CrashReason factored to crash_reason.h to break circular protocol.h ↔ arena.h; TrackerStepper.stepOnce calls checkArenaBounds; gather_tracker_inputs calls distanceToBoundary for slot 44; pathgen still uses M8b shim's checkAircraftOOB)_
- [ ] T045 [US2] Plan-phase research item resolution: choose between extending `ENTRY_SAFE_*` (pathgen entry-time clamp) into in-flight bounds, OR adding parallel `FLIGHT_ARENA_*` constants per R2 decision. Document choice + audit `src/autoc.cc:266-282` for any required refactoring; do NOT silently overload entry-time constants per D11. (M7d)
- [ ] T046 [US2] Plug trail-rabbit + crash-hull + arena into the tracker-mode `FitnessComputer` path: cone-surface fitness (existing pathgen `include/autoc/eval/fitness_computer.h` reused) with trail rabbit substituted for path point; crash-hull strike → scenario terminator (Q1); arena egress → scenario terminator (Q3); both record telemetry. (M7d)

**Checkpoint**: US2 complete. Operator can launch tracker-mode training, see fitness numbers per gen, and observe the loop close end-to-end.

---

## Phase 4: User Story 5 — Renderer inspection (Priority: P2)

**Goal**: Operator loads a tracker-mode dmp into the renderer and animates the result — 3rd-person view (both aircraft + beacons), camera-POV mode (1st-person), camera-POV mini-panel, per-tick scrub controls, CEP error-bar visualization.

**Independent Test**: Per quickstart.md step 5 — `build/renderer -i autoc-tracker.ini -k autoc-storage/<run-id>/genN.dmp`; visual confirmation of two aircraft + beacons + mini-panel + CEP error bars + scrub controls.

**Maps to plan.md milestones**: M8 (tracker-mode dmp output v2) → M9 (renderer tracker-mode playback).

### M8 — Tracker-mode dmp output (FR-015 + FR-015a)

- [X] T047 [P] [US5] Contract test `tests/tracker_dmp_roundtrip_tests.cc` — `EvalResults` v2 schema serialize/deserialize identity; v1 (pathgen) dmp loads with `cameraViewList` + `targetTrajectoryList` empty; future-version dmp throws cleanly; M2-dmp self-containedness (renderer-mock loads only the M2 dmp, no M1 source needed) _(M8a / 2026-05-06: 3 contract tests green — V2RoundTripIdentity (every-field bitwise on synthetic 3 scenarios × 5 ticks), PathgenStyleDmpHasEmptyTrackerFields (pathgen-style write/read leaves new fields empty), FutureVersionLoudFail_DocumentationOnly (Constitution V contract documented; byte-tampering hermetic reproducer deferred to follow-up). cereal_version_anchor_tests bumped to anchor EvalResults@2 + new AircraftStateAtVersion2 anchor)_
- [X] T048 [US5] Extend `EvalResults` schema in [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h) per data-model.md §8 — add `cameraViewList[scenario][tick]` + `targetTrajectoryList[scenario][tick]` + `arenaEgressCount[scenario]` + `hullStrikeCount[scenario]`; bump `CEREAL_CLASS_VERSION(EvalResults, 2)` (FR-015a M1 → M2 boundary per Q5 milestone-versioning principle). **Honest-recording audit** (per [memory: feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md), 2026-05-06): at the v2 boundary, audit `AircraftState::serialize()` against the full sensor inventory (PathgenInput / TrackerInput meta arrays + key derived fields like `gyroRates_`); add the omitted fields rather than inheriting v1's partial coverage; default to always-on serialization rather than `hasNNData_`-gated optional. The 030 M3a regression caught the gyroRates omission; this is the right boundary to fix it. _(M8a / 2026-05-06: EvalResults v=1 → v=2 with version-aware serialize (v=1 reads back-compat-clean). AircraftState v=1 → v=2: gyroRates_ field added (closes M3a-discovered gap), NN data block switched from hasNNData_-gated to always-on for v=2 reads. v=1 read path preserved bitwise — operator-confirmed three ways: rebuild-perf.sh autoc-eval bitwise on gen9200.dmp, fresh nnextractor + eval, renderer reads gen 800 m1 dmp cleanly)_
- [X] T049 [US5] Define `CameraViewSample` and `CopiedTargetSample` structs in [`include/autoc/rpc/protocol.h`](../../include/autoc/rpc/protocol.h) with cereal `serialize()` methods _(M8a / 2026-05-06: structs in protocol.h with cereal serialize; CameraViewSample has camera world pose + FOV + 2 BeaconObservation; CopiedTargetSample has target pose/orient/vel + trail_rabbit_position (M7 wires) + inside_crash_hull (M7 wires). BeaconObservation got its own cereal serialize method)_
- [ ] T050 [US5] Wire M2 dmp output: `TrackerStepper` (T037) records per-tick `CameraViewSample` (camera pose + 2 `BeaconObservation`) and per-tick `CopiedTargetSample` (copied from `SourceScenarioTrajectory.sample(t_i)`, including computed `trail_rabbit_position` from T042 and `inside_crash_hull` flag from T043) into the eval results. Output dmps written to S3 at `autoc-storage/<030-run-id>/gen<N>.dmp` — same bucket as source, separate run-id (per US2 Independent Test + spec D13)

### M9 — Renderer tracker-mode playback (FR-012 + FR-012a)

- [ ] T051 [P] [US5] Smoke test `tests/renderer_tracker_smoke_tests.cc` — renderer loads a fixture tracker-mode dmp; assertions on scene actor count (chase + target = 2), HUD overlay slot existence; existing pathgen-renderer tests stay green (regression invariant)
- [ ] T052 [US5] Add tracker-mode dmp loader path in [`tools/renderer.cc`](../../tools/renderer.cc) — version-field dispatch (v1 → existing pathgen renderer path; v2 → new tracker renderer path per Q5/FR-015a)
- [ ] T053 [US5] Implement 3rd-person view in [`tools/renderer.cc`](../../tools/renderer.cc) — chase craft (existing) + target craft (NEW VTK actor reading `targetTrajectoryList` per FR-015 self-containedness — NO crrcsim mod_robots dependency per FR-002 v1 deferral note + plan §M4); beacons rendered on target wingtips at hb1 body-frame positions; FOV cone drawn from chase camera mount
- [ ] T054 [US5] Implement 1st-person camera-POV view in [`tools/renderer.cc`](../../tools/renderer.cc) — render scene through chase camera using recorded camera pose + FOV from M2 dmp; beacons appear as colored points at projected `(screen_x, screen_y)` from `cameraViewList`
- [ ] T055 [US5] Implement camera-POV mini-panel as HUD overlay in [`tools/renderer.cc`](../../tools/renderer.cc) — small 2D rectangle near throttle/control-state; renders current-tick beacon positions; sentinel-CEP visually distinguishable (dimmed dot or absent); follows existing HUD-visibility logic (D5 — NOT always-on, slots into existing toggle system)
- [ ] T056 [US5] Implement CEP error-bar visualization in 1st-person + mini-panel — render CEP as ellipse spread around each projected beacon centroid (D15 v1 commit — committed for v1 because it's load-bearing for smoke-test signal-or-not assessment)
- [ ] T057 [US5] Implement per-tick scrub controls in [`tools/renderer.cc`](../../tools/renderer.cc) (rolled-in BACKLOG entry "Renderer Playback Enhancements") — pause / step-forward / step-backward; works in both 3rd-person and camera-POV modes
- [ ] T058 [P] [US5] Streak/multiplier overlay in [`tools/renderer.cc`](../../tools/renderer.cc) (rolled-in BACKLOG entry, optional plumbing path) — recompute path or schema-bump path per backlog entry; pick during M9 implementation

**Checkpoint**: US5 complete. Operator can visually inspect tracker-mode dmp output. This is the qualitative "do we believe the loop closes" eyeball test ahead of the formal smoke test.

---

## Phase 5: User Story 4 — Smoke test (Priority: P1) 🎯 v1 acceptance floor

**Goal**: End-to-end loop closes on a real M1 source dmp + real tracker-mode autoc run + real renderer playback. Fitness curve does *something* — descends, plateaus, or fails informatively. M10 IS the experimental answer to R10 (perception representation can-it-train) per research.md.

**Independent Test**: Operator-driven full execution of the four D13 deliverables (run from M1 file → single-config slice → save → renderer animates). No automated test gates here; gates were at M5/M6/M7/M8/M9 individually.

**Maps to plan.md milestone**: M10 — SMOKE TEST.

### M10 — Smoke test execution

- [ ] T059 [US4] Configure `autoc-tracker.ini` for smoke run per quickstart.md — pastonly3 source dmp, scenario slice = 6 paths × 20 winds = 120 scenarios per Q2 clarification, population = 5000, gens = 100, default trail / hull / arena, deterministic seed
- [ ] T060 [US4] Execute smoke run: `stdbuf -oL -eL build/autoc -i autoc-tracker.ini 2>&1 | tee logs/autoc-030-smoke-001.log`
- [ ] T061 [US4] Per quickstart.md step 4 — pull a recent gen's dmp from S3, sanity-check via per-tick dmp extractor (depends on T064 from M11a but can be cursory until then)
- [ ] T062 [US4] Per quickstart.md step 5 — load M2 dmp into renderer, verify all four D13 deliverables visually green (autoc loaded M1 ✓; single-slice scenario per ini ✓; results saved ✓; renderer animated M2 ✓)
- [ ] T063 [US4] Capture findings per quickstart.md step 6 — write `eval-results/030-smoke-<date>/SMOKE_REPORT.md` (sim-only smoke; `flight-results/` is reserved for real-flight artifacts) with: source dmp S3 key + scenario slice + 030-run-id; fitness curve shape (descending / plateau / pathological); renderer screenshots; sentinel events (arena egresses / hull strikes / NaN propagations / build issues)

**Checkpoint**: SMOKE TEST GREEN means 030 v1 floor is hit. Decision per D13: continue to Phase 6 analytics (the "030 done" ramp) OR debug failing milestone.

---

## Phase 6: Post-smoke analytics (M11a-M11c — the "030 done" ramp per Q4 ceiling)

**Goal**: Build/extend tools to assess what trained — the analytics needed to interpret smoke-test signal and inform R10/R11/R12 architectural responses.

**Maps to**: M11a (per-tick dmp extractor) + M11b (eval Bug 2 fix) + M11c (tracker-specific analytics).

### M11a — Per-tick dmp extractor (rolled-in BACKLOG entry)

- [ ] T064 [P] Implement `tools/aircraft_state_extractor.cc` — read tracker-mode dmps, emit CSV with new column set per data-model.md §8 (chase per-tick state + beacon `(x, y, CEP)` per camera + camera pose + target-craft pose + trail-rabbit position + arena-egress flag + hull-strike flag); version-field dispatch handles both v1 (pathgen) and v2 (tracker) sources
- [ ] T065 [P] Adapt `specs/029-no-future-arch/plot_per_axis_time_series.py` to consume the new column set; existing data.dat path stays usable for pathgen-mode runs

### M11b — Eval Fitness Bug 2 fix (rolled-in BACKLOG entry)

- [ ] T066 [P] Bug fix in [`src/autoc.cc`](../../src/autoc.cc) eval-mode dump path — update `genome.fitness` with eval result before serializing to `evalResults.gp` (or alternatively store eval fitness in a separate `evalResults` field); renderer's fitness display reflects the eval-mode tracker fitness, NOT the gen's training-time fitness
- [ ] T067 [P] Contract test `tests/eval_fitness_bug2_tests.cc` — eval-mode run produces a dmp where `genome.fitness` matches the eval-computed fitness, not the training-time pre-eval value; regression-locks the bug

### M11c — Tracker-specific analytics (the six instrumentation items from R11)

- [ ] T068 [P] Implement per-tick output saturation + per-axis aggressiveness analytics (FR-014) in `specs/030-tracker-mode/per_axis_tracker_analytics.py` — reads M2 dmp via T064, emits per-scenario output saturation rates and per-axis dCtrl + ⟨|out|⟩ statistics; pathgen-mode tooling applies unchanged per FR-014
- [ ] T069 [P] Implement CEP-sentinel-rate vs output-magnitude correlation analytics (the load-bearing R12 dead-reckoning diagnostic) in `specs/030-tracker-mode/cep_sentinel_analytics.py` — slices each scenario into visible / sentinel-burst / post-sentinel-recovery segments; emits tracking-error trajectory comparison
- [ ] T070 [P] Implement chase-quat-extreme-event flag + chase-rotation-vs-beacon-motion correlation in `specs/030-tracker-mode/chase_attitude_analytics.py` — detects if controller mis-attributes chase rotation to target motion
- [ ] T071 [P] Implement inter-beacon angle change rate histogram in `specs/030-tracker-mode/aliasing_analytics.py` — measures Trouble 8 roll-rate aliasing per R10 trouble list
- [ ] T072 [P] Implement mean-target-screen-distance trajectory + fitness-vs-gen plateau auto-flag in `specs/030-tracker-mode/convergence_analytics.py` — auto-flags when 50-gen rolling fitness improvement < 1%

**Checkpoint**: Phase 6 complete = "030 done" per Q4 ceiling decision (M10 + M11a + M11b + M11c). Beyond this, all candidates are 031-CANDIDATE BACKLOG entries.

---

## Phase 7: Polish & cross-cutting concerns

**Purpose**: Documentation, post-implementation review, decisions for next iteration.

- [ ] T073 [P] Update `CLAUDE.md` agent context with 030 v1 entry — note the smoke-test outcome + which R-question response was triggered (if any)
- [ ] T074 [P] Write `specs/030-tracker-mode/outcome.md` documenting smoke-test results, R10/R11/R12 diagnostic readings, and recommended next direction (more 030 work / unpark 031 / unpark 025 / re-fly converged pastonly3 with tracker mode)
- [ ] T075 Code review pass: ensure no temporary scaffolding left in tree (no `TODO` comments without ticket links; no commented-out blocks; no stale references to the obsolete `(x, y, visible)` interface or `RobotProgrammable`-in-v1 path)
- [ ] T076 Constitution compliance audit: Principle I (every new module has a contract test); Principle II (autoc + crrcsim + xiao all build clean); Principle III (no shims left); Principle IV (mod_inputdev linkage stays clean post-implementation); Principle V (CEREAL_CLASS_VERSION = 2 confirmed at milestone freeze); Principle VI (no unannotated raw `float`/`double` in 030's diff under `src/eval/`, `src/nn/`, `include/autoc/eval/`, `include/autoc/nn/` — see T078 for the codebase-wide backfill, separate scope)
- [ ] T077 [P] Document plan-research's "030 done" decision in [`specs/BACKLOG.md`](../BACKLOG.md) — confirm which 031-CANDIDATE entries should unpark next (perception-front-end / variable-rate / library curation / renderer exotic goodies); cross-reference any items that smoke-test signal pulled forward into v1 (per D15's "may extract cheap-and-load-bearing items" note)
- [ ] T078 **Type-domain audit (codebase-wide backfill — Principle VI)** — scan all of `src/eval/`, `src/nn/`, `include/autoc/eval/`, `include/autoc/nn/` for raw `float` / `double` declarations; for each hit, decide alias-vs-keep per the Principle VI domain table + whitelist (NN byte buffers, hardware-protocol fields, cereal byte-format members, host-only metadata, library-imposed signatures). Convert eligible sites to `gp_scalar` / `gp_fitness` / `gp_vec3` / `gp_quat`; annotate intentional raw-type sites with `// raw-ok: <reason>`. Run regression-tight gate (`bash scripts/rebuild-perf.sh` + `autoc -i autoc-eval.ini` bitwise on baseline `gen9200.dmp`) — type-alias edits SHOULD be no-ops, any FP drift = uncovered semantic difference and gets investigated before merge. Cross-reference [`project_scalar_type_audit_backlog.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_type_audit_backlog.md) memory; on completion, that backlog entry transitions to "Resolved". Audit grep one-liner per Principle VI:

  ```bash
  grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ \
    | grep -v -E '// raw-ok:'
  ```

---

## Dependencies

**Phase ordering** (sequential, with intra-phase parallelism):

1. **Phase 1 (Setup)** — T001 → T002 (T002 [P] with T001 since branch creation doesn't gate fixture pull)
2. **Phase 2 (Foundational)** — T003 (mod_inputdev linkage) MUST precede any new file in `src/nn/` or `src/eval/`. T003 → T004/T005/T006 → T007–T019 (T007–T018 mostly [P] across files; T019 final regression check is sequential)
3. **Phase 3 (US2)** — Foundational (Phase 2) MUST complete first. Within: M3 (T020-T024) → M5 (T025-T031) → M6 (T032-T038) → M7 (T039-T046). M5 needs M3's `SourceScenarioTrajectory` shape; M6 needs M3 + M5; M7 needs M6.
4. **Phase 4 (US5)** — depends on Phase 3 (US2) for M2 dmp output to exist. M8 (T047-T050) → M9 (T051-T058). T051-T058 mostly [P] within M9.
5. **Phase 5 (US4 — smoke test)** — depends on Phase 3 + Phase 4 fully complete.
6. **Phase 6 (Analytics)** — depends on Phase 5 for the smoke-test M2 dmps to analyze. T064-T072 mostly [P] across separate analytics scripts.
7. **Phase 7 (Polish)** — depends on Phase 6.

**Cross-story dependencies**:

- US2 unblocks US5: US5's renderer reads M2 dmps that US2 produces (M8 schema is part of US5 for grouping purposes but M8 work needs US2's tracker-mode runtime to actually produce dmps).
- US4 (smoke test) is the integration test of US2 + US5 working together end-to-end.
- All v1 user stories (US2 + US5 + US4) feed into Phase 6 (analytics) which is the "030 done" ramp.

**Out of scope for v1 (per spec D13/D15)**:

- US1 (past-only baseline): done in 029, not 030.
- US3 (camera-config experimentation): deferred per D13/D15 — see [BACKLOG.md "030 spin-offs"](../BACKLOG.md) for 031 routing.
- US6 (real-target-tracking bridge): post-v1, naturally falls out of US2's architecture once converged tracker-mode controllers exist.

---

## Parallel execution opportunities

**Phase 2 (Foundational)** — high parallelism within M2 type-safe scaffolding:

```text
After T003 (linkage fix):
  Concurrent: T005 (cereal anchor test) + T007/T008 (contract tests) +
              T011 (topology.h) + T013 (evaluator.cc) + T014 (autoc.cc data.dat) +
              T015 (existing tests update) + T016 (sim_response.py) +
              T017 (xiao msplink.cpp) + T018 (aircraft_state.h)
Sequential: T009 (PathgenInput enum) → T010 (TrackerInput enum) → T012 (autoc.h cleanup) → T019 (regression check)
```

**Phase 3 (US2)** — within M5 (projection):

```text
Concurrent: T025 (geometry test) + T026 (int8 test) + T027 (camera_config) +
            T028 (beacon_config) + T029 (camera_projection.h)
Sequential: T030 (projectBeacon impl) → T031 (AirframeProxy)
```

**Phase 3 (US2)** — within M7 (fitness):

```text
Concurrent: T039 (trail_rabbit test) + T040 (crash_hull test) + T041 (arena test) +
            T042 (trail_rabbit impl) + T043 (crash_hull impl) + T044 (arena impl)
Sequential: T045 (arena primitive choice) → T046 (FitnessComputer wire-up)
```

**Phase 4 (US5)** — within M9 (renderer):

```text
Concurrent: T051 (smoke test) + T053 (3rd-person view) + T054 (1st-person view) +
            T055 (mini-panel) + T056 (CEP error bars) + T058 (streak overlay)
Sequential: T052 (loader path) → all M9 view tasks; T057 (scrub controls) gates
            on at least one view existing
```

**Phase 6 (Analytics)** — fully [P] across separate Python files:

```text
Concurrent: T064 (extractor) + T065 (per-axis port) + T066/T067 (Bug 2) +
            T068 (saturation) + T069 (CEP correlation) + T070 (chase attitude) +
            T071 (aliasing) + T072 (convergence)
```

---

## Implementation strategy (MVP-first delivery)

**MVP scope** = US2 (Phase 3) + US4 (Phase 5) + US5 (Phase 4) — the smoke-test triangle. Per Q4 clarification, "030 v1 done" = MVP + Phase 6 analytics ramp.

**Incremental visible checkpoints** (per plan.md M0-M11 framing):

| Milestone | After tasks | Visible artifact |
|---|---|---|
| M1 done | T003-T006 | `bash scripts/rebuild.sh` clean; CEREAL_VERSION anchor test green |
| M2 done | T007-T019 | pathgen-mode regression run produces byte-identical `data.dat` to pre-M2 reference |
| M3 done | T020-T024 | `tools/source_dmp_inspect <s3-key>` prints scenario summary stats |
| M5 done | T025-T031 | Beacon projection contract tests green; can read each as documentation of perception output |
| M6 done | T032-T038 | Tracker mode launches, runs gen 0 evaluations, log shows source dmp loaded + per-scenario distribution |
| M7 done | T039-T046 | Tracker-mode short run produces sensible per-gen fitness numbers; extreme-parameter sanity checks break in expected directions |
| M8 done | T047-T050 | An actual tracker-mode `.dmp` exists in S3 from a real run; `cereal` round-trips it; v2 version field embedded |
| M9 done | T051-T058 | Renderer plays M2 dmp end-to-end with all 4 view modes (3rd-person + camera-POV + mini-panel + CEP error bars) and per-tick scrub |
| **M10 SMOKE GREEN** | T059-T063 | **All four D13 deliverables checked**; smoke-report written |
| M11a-c done | T064-T072 | Operator has tools to localize R10/R11/R12 trouble responses from smoke-test data |
| **030 v1 done** | T073-T077 | Polish complete; outcome documented; 031-CANDIDATE routing decided |

**If smoke red**: per plan.md "If smoke red specifically because of representation" / "specifically because of capacity" / "mechanically (loop doesn't close)" diagnostic ladder, the analytics from T068-T072 localize the trouble and inform whether the response is M5/M6/M7/M8/M9 fix (rebuild) OR R10/R11/R12 architectural response (out-of-scope for v1; opens a v2 spec).

---

## Validation summary

**Format compliance**: All 78 tasks follow the strict `- [ ] [TaskID] [P?] [Story?] Description with file path` format per the template. Setup / Foundational / Polish phases have no `[Story]` label; user-story phases (Phase 3 / 4 / 5) all carry `[US2]` / `[US5]` / `[US4]` labels.

**Test coverage**: Constitution Principle I satisfied — every new module has a contract or smoke test (T005, T007, T008, T020, T025, T026, T032, T039, T040, T041, T047, T051, T067 = 13 distinct test files). Tests inline with implementation per phase, NOT optional.

**Independent testability**:

- US2 (Phase 3) — can be exercised standalone via `tools/source_dmp_inspect` (T024) and short tracker-mode runs (T060 with reduced settings).
- US5 (Phase 4) — exercised standalone via `build/renderer -k <dmp>` once a tracker-mode dmp exists.
- US4 (Phase 5) — IS the integration test; smoke green = success.

**Parallel opportunities identified**: ~40 of 77 tasks (~52%) marked [P], dominated by independent-file changes within each milestone (different .h / .cc / Python script files) and intra-phase test-vs-implementation parallelism.

**Total task count**: 78 tasks across 7 phases.

**Task count per user story**: US2 = 27 tasks (Phase 3), US5 = 12 tasks (Phase 4), US4 = 5 tasks (Phase 5). Foundational + Setup + Analytics + Polish = 33 tasks.

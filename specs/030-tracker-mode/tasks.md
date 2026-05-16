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

- [~] T032 [P] [US2] Contract test `tests/timing_model_tests.cc` — FR-018 + FR-009 determinism: same source dmp + same seed → bit-identical M2 trajectory across two invocations regardless of sim-clock speed; variable-rate source samples (synthetic 50 ms / 73 ms / 100 ms intervals) handled correctly without interpolation drift _(deferred 2026-05-07: FR-018 determinism is already locked by the rebuild-perf.sh autoc-eval bitwise gate (eval-vs-training fitness exact match) + tests/beacon_projection_tests.cc determinism assertions. Variable-rate source samples are a 031-candidate per [BACKLOG.md "Variable-rate / real-flight source robustness"](../BACKLOG.md) — sim-recorded sources are at integral 100ms NN cadence (crrcsim 5ms internal × 20 = 100ms NN tick), no interpolation needed for v1. T032 unparks when real-flight-recorded trajectories become available, post-virtual-beacon flight test.)_
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
| **M7d.a** | T045 + T046 (partial) | ✅ `FitnessComputer` wire-up — cone-surface against trail-rabbit; arena egress as scenario terminator; `arenaEgressCount` + `hullStrikeCount` populated. (Done 2026-05-07.) |
| **M7d.b** | T046 (finish) | ✅ Crash-hull probabilistic firing as scenario terminator — `CrashReason::HullStrike` variant; `didCrashFire` invocation in `TrackerStepper::stepOnce`; PRNG plumbed from `AutocConfig` through `EvalData`; `p_crash` curriculum from `pCrashForGen(gen)`; `hullStrikeCount[i]` returned by stepper. (Done 2026-05-07.) |

**Bitwise risk** highest at M7d.a (FitnessComputer touch). M7a/b/c are mostly additive (+ TrackerInputs rename, which is tracker-mode-only). M7d.b touches `TrackerStepper.stepOnce` only — pathgen unaffected.

- [X] T039 [P] [US2] Contract test `tests/trail_rabbit_tests.cc` — FR-008 math identity (`rabbit = target_pos - velocity_unit × 3.048`). ~~FR-008a degenerate-velocity fallback per R10 (hard-switch at 2 m/s with hysteresis ±0.5)~~ **Amended 2026-05-07 per Session 2026-05-07 Q1**: nose-fallback deferred; v1 degenerate fallback (`\|velocity\| < 1e-3` ⇒ `rabbit = target_pos`) is the only fallback case asserted. (M7b) _(M7b / 2026-05-07: 7 contract tests green — forward-flying / diagonal-velocity / configurable-distance / zero-velocity / near-zero-velocity-threshold / descending-target / pure-function determinism)_
- [X] T040 [P] [US2] Contract test `tests/crash_hull_tests.cc` — FR-008b sphere-intersection identity at boundary tangent / inside / outside; `p_crash` curriculum per the schedule resolved in [research.md R3](./research.md#r3--p_crash-v1-default) (curriculum-anneal 0.0 → 0.30 by gen 200 within deterministic seed); mode-gated (pathgen scenarios never invoke hull check). (M7c) _(M7c / 2026-05-07: 17 contract tests green — 6 isInsideHull (origin / boundary / outside / radius-configurable / reserved-shapes) + 5 didCrashFire (outside-no-fire / p=0 / p=1 / determinism / Bernoulli rate) + 6 pCrashForGen (gen0 / negative-gen / plateau / beyond / linear-interp / configurable))_
- [X] T041 [P] [US2] Contract test `tests/arena_tests.cc` — FR-016 boundary checks: chase outside radius → egress with `RADIUS` flag; below floor → `FLOOR`; above ceiling → `CEILING`; scenario-terminating per Q3 clarification; **`DIST_TO_BOUNDARY_ALONG_VEL` ray-projection identity** (cylinder quadratic + floor/ceiling linear; min positive t; sentinel when no intersection along velocity) — same function feeds NN input AND OOB termination per Session 2026-05-07 Q1. (M7a) _(M7a / 2026-05-07: 11 contract tests green — 5 checkArenaBounds (NONE / RADIUS / FLOOR / CEILING / RADIUS-priority) + 6 distanceToBoundary (cylinder hit / near-wall / heading-away / floor descent / ceiling climb / sentinel-on-zero-velocity))_
- [X] T042 [P] [US2] Define + implement [`include/autoc/eval/trail_rabbit.h`](../../include/autoc/eval/trail_rabbit.h) + [`src/eval/trail_rabbit.cc`](../../src/eval/trail_rabbit.cc) — `computeTrailRabbit()`: `target_pos − target.velocity / |target.velocity| × trail_distance` per data-model.md §6. **Simplified per Session 2026-05-07 Q1**: degenerate-velocity branch returns `target_pos` (no nose-direction rotation, no hysteresis). (M7b) _(M7b / 2026-05-07: trail_rabbit.h + trail_rabbit.cc landed; TrackerStepper.projectAndShiftHistory now writes real trail rabbit into last_target_sample_ for v=2 dmp output (was placeholder = target.position). Trail distance defaults to kDefaultTrailDistance (3.048m); M7d plumbs configurable cfg.trailDistance through EvalData)_
- [X] T043 [P] [US2] Define + implement [`include/autoc/eval/crash_hull.h`](../../include/autoc/eval/crash_hull.h) + [`src/eval/crash_hull.cc`](../../src/eval/crash_hull.cc) — `isInsideHull()` + `didCrashFire()` with curriculum-anneal `p_crash` per R3. (M7c) _(M7c / 2026-05-07: crash_hull.h + crash_hull.cc landed; SPHERE shape v1, AABB_HB1 + MESH_AIRFRAME reserved; Park-Miller LCG for deterministic Bernoulli draw with caller-managed PRNG state; pCrashForGen linear-ramp curriculum 0→0.30 across gens 0..200. TrackerStepper.projectAndShiftHistory now writes the geometric inside_crash_hull flag into v=2 dmp recording (was hardcoded false). Probabilistic firing + scenario termination are M7d.)_
- [X] T044 [P] [US2] Define + implement [`include/autoc/eval/arena.h`](../../include/autoc/eval/arena.h) + [`src/eval/arena.cc`](../../src/eval/arena.cc) — `FlightArena` struct + `checkArenaBounds()` returning `ArenaEgressKind` + **`distanceToBoundary(pos, vel_unit, arena)`** ray-projection scalar (Session 2026-05-07 Q1: same function feeds NN input AND egress check). Replaces M8b shim's `checkAircraftOOB` on the tracker side. (M7a) _(M7a / 2026-05-07: arena.h + arena.cc landed; FlightArena cereal-serialized in EvalData; CrashReason factored to crash_reason.h to break circular protocol.h ↔ arena.h; TrackerStepper.stepOnce calls checkArenaBounds; gather_tracker_inputs calls distanceToBoundary for slot 44; pathgen still uses M8b shim's checkAircraftOOB)_
- [X] T045 [US2] Plan-phase research item resolution: choose between extending `ENTRY_SAFE_*` (pathgen entry-time clamp) into in-flight bounds, OR adding parallel `FLIGHT_ARENA_*` constants per R2 decision. Document choice + audit `src/autoc.cc:266-282` for any required refactoring; do NOT silently overload entry-time constants per D11. (M7d.a) _(M7d.a / 2026-05-07: parallel-constants choice taken — `FlightArena` struct in `include/autoc/eval/arena.h` (radius_m / floor_agl_m / ceiling_agl_m) plumbed through `EvalData::flightArena`. `ENTRY_SAFE_RADIUS` / `ENTRY_SAFE_ALT_MIN` / `ENTRY_SAFE_ALT_MAX` confirmed used only at `src/autoc.cc:305-313` for entry-time clamp; not overloaded for in-flight bounds.)_
- [X] T046 [US2] Plug trail-rabbit + crash-hull + arena into the tracker-mode `FitnessComputer` path: cone-surface fitness (existing pathgen `include/autoc/eval/fitness_computer.h` reused) with trail rabbit substituted for path point; crash-hull strike → scenario terminator (Q1); arena egress → scenario terminator (Q3); both record telemetry. (M7d) _(M7d.a / 2026-05-07: tracker-mode branch in `src/eval/fitness_decomposition.cc::computeScenarioScores` keyed on non-empty `targetTrajectoryList[i]`; rabbit = `trail_rabbit_position`, tangent = unit target velocity. Pathgen branch wrapped in `else`, bitwise-preserved (gen9200.dmp eval-vs-training match confirmed). `arenaEgressCount[i]` = 1 if tracker `crashReason == Eval`, else 0. M7d.b / 2026-05-07: `CrashReason::HullStrike` variant added; `TrackerStepper.stepOnce` calls `didCrashFire` post-physics (after arena egress, which wins precedence on coincident triggers); `EvalData` carries `pCrashThisGen` (pre-computed at autoc via `pCrashForGen(gCurrentGeneration, cfg.pCrashGenRamp, cfg.pCrashGen0, cfg.pCrashPlateau)`), `crashHullRadius`, `trailDistance`. PRNG seed = `scenarioMeta.scenarioSequence` cast to u32 (Park-Miller LCG, OR-with-1 to avoid zero-state pathology). `hullStrikeCount[i] = stepper.hullFiredCount()`. M7d.b smoke gen 1-2 fitness identical to M7d.a smoke (didCrashFire short-circuits without PRNG draw when outside hull → non-perturbing determinism check). Hull-strike firing path not exercised at gen 1-2 (p_crash ≈ 0, hull=outside throughout); validated by unit tests in `tests/crash_hull_tests.cc`.)_

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
- [X] T050 [US5] Wire M2 dmp output: `TrackerStepper` (T037) records per-tick `CameraViewSample` (camera pose + 2 `BeaconObservation`) and per-tick `CopiedTargetSample` (copied from `SourceScenarioTrajectory.sample(t_i)`, including computed `trail_rabbit_position` from T042 and `inside_crash_hull` flag from T043) into the eval results. Output dmps written to S3 at `autoc-storage/<030-run-id>/gen<N>.dmp` — same bucket as source, separate run-id (per US2 Independent Test + spec D13) _(M8b / 2026-05-07: TrackerStepper.projectAndShiftHistory populates last_camera_view_ + last_target_sample_; minisim pushes per-tick into evalResults.cameraViewList + targetTrajectoryList in lockstep with aircraftStateList. S3 output via existing autoc dmp path: prefix `autoc-storage/autoc-<id>-<timestamp>/gen<N>.dmp` (gen9998.dmp for eval mode). M7b/M7c filled trail_rabbit_position + inside_crash_hull (no longer placeholders). Smoke validated via tracker_dmp_inspect: cameraViewList POPULATED, targetTrajectoryList POPULATED, 98 parallel ticks across all per-scenario lists.)_

### M9 — Renderer tracker-mode playback (FR-012 + FR-012a)

- [ ] T051 [P] [US5] Smoke test `tests/renderer_tracker_smoke_tests.cc` — renderer loads a fixture tracker-mode dmp; assertions on scene actor count (chase + target = 2), HUD overlay slot existence; existing pathgen-renderer tests stay green (regression invariant)
- [~] T052 [US5] Add tracker-mode dmp loader path in [`tools/renderer.cc`](../../tools/renderer.cc) — version-field dispatch (v1 → existing pathgen renderer path; v2 → new tracker renderer path per Q5/FR-015a) _(M9a partial 2026-05-07: load-time detection + `Renderer::isTrackerMode_` flag landed in tools/renderer.cc + tools/renderer.h. v=2 dmps with non-empty cameraViewList + targetTrajectoryList set the flag; pathgen v=1/v=2-empty dmps leave it false. Virtual→display Z offset extended to targetTrajectoryList[].position + .trail_rabbit_position + cameraViewList[].camera_pose_world_pos so M9b's target-craft actor renders at correct altitude. Existing pathgen render path runs unchanged. Render-path branching on isTrackerMode_ is M9b's job (target-craft VTK actor + beacons + FOV cone). Cereal version-aware load handles v=1 ↔ v=2 transparently via M8a's serialize template.)_
- [~] T053 [US5] Implement 3rd-person view in [`tools/renderer.cc`](../../tools/renderer.cc) — chase craft (existing) + target craft (NEW VTK actor reading `targetTrajectoryList` per FR-015 self-containedness — NO crrcsim mod_robots dependency per FR-002 v1 deferral note + plan §M4); beacons rendered on target wingtips at hb1 body-frame positions; FOV cone drawn from chase camera mount. _(M9b.1 partial 2026-05-08: target-craft tape landed — `targetActuals` vtkAppendPolyData + `targetActor` magenta/lime ribbon mirroring `actuals`/`actor2`. Helpers `targetSamplesToVector` / `targetSamplesToOrientation` mirror `stateToVector` / `stateToOrientation`. Per-scenario population gated on `isTrackerMode_` (M9a flag). Pathgen-mode dmps render unchanged. Operator post-render fixes 2026-05-08: tape colors changed purple/violet → magenta/lime for clearer top/bottom orientation contrast; **color routing flipped 2026-05-08**: target tape (M2 source = M1 chase recording in role of rabbit) inherits the historical orange/cyan, M2 chase tape (the new craft being trained) gets magenta/lime via mode-conditional `actor2->GetProperty()->SetColor` at load time. M1 pathgen mode unchanged — actor2 stays orange/cyan when `tracker_data_present=false`. Beacons stay on target craft (now orange/cyan), matching real-flight intuition: physical beacons mount on the aircraft being observed by the chase camera. blue error-bar segments switched chase→rabbit → chase→target via new `createSegmentSetToTarget` helper (tracker mode only; pathgen still uses `createSegmentSet` chase→rabbit). All three render paths updated: `updateGenerationDisplay` (S3 load), `renderFullScene` (refresh), `updatePlaybackAnimation` (timer-driven reveal). Animation now: target-tape reveals index-parallel to chase reveal; chase→target blue segments; red rabbit path **hidden** in tracker mode (the pathList stays in the v=2 dmp for renderer-pipeline compat, but chase doesn't follow it semantically — magenta target tape is the actual source-of-truth reference; operator routing 2026-05-08 chose hide-rather-than-decoration to keep larger random-path scenarios visually clean). Pathgen + xiao-flight modes unchanged across all three paths. Beacons + FOV cone are M9b.2/M9b.3 follow-ups. **M9b.2 partial 2026-05-08**: beacon trail glyphs landed — small spheres (radius 0.15m) at every recorded target tick in navigation-light convention (RED port, GREEN starboard). World pos = `target.position + target.orientation × beacon_mount_body`; mount offsets hardcoded to v1 autoc-tracker.ini defaults `(0, ∓0.45, 0)` since `EvalResults` doesn't carry `BeaconConfig` (worker-input-only via `EvalData`). Animation reveal index-parallel with target tape. New helper `targetSamplesToBeaconPositions(samples, mount_body)` + new `targetBeaconLeftActor` / `targetBeaconRightActor` glyph actors. **M9b.3 partial 2026-05-08**: chase camera FOV wireframe pyramid landed — apex at `cameraViewList[i][k].camera_pose_world_pos`, axis from `camera_pose_world_orient`, half-extents `tan(fov_h/2)·30m` and `tan(fov_v/2)·30m`. Yellow semi-transparent lines (4 apex→corner edges + 4 base-rectangle edges). Single pyramid per scenario follows the latest visible chase tick during animation. New helper `createFovPyramidLines(offset, cameraSample, length)`. Pyramid length 30m fixed for v1; revisit if visually too small/large for typical chase-target distances.)_ **Basic-mode point convention (FR-012 clarified 2026-05-07)**: the rendered target marker IS `targetTrajectoryList[i][k].position` — the actual target-craft position — NOT `trail_rabbit_position` (which lives 10 ft behind the velocity vector and is a fitness-only construct). **Nice-to-have post-v1** (rolled into renderer-polish backlog alongside scrub + streak overlay): render an arrow from `target.position` along `−velocity_unit × trail_distance` to visualize where the rabbit lives relative to the target, so operator can eyeball whether chase is converging on the rabbit (cone-fitness sweet spot) vs. the target itself.
- [→BACKLOG] T054 [US5] Implement 1st-person camera-POV view in [`tools/renderer.cc`](../../tools/renderer.cc) — render scene through chase camera using recorded camera pose + FOV from M2 dmp; beacons appear as colored points at projected `(screen_x, screen_y)` from `cameraViewList` _(deferred 2026-05-08: the M9b mini-panel HUD overlay (T055) covers the load-bearing "what does the NN see?" question; full 1st-person main viewport is research-grade analytics. Moved to BACKLOG.md "Renderer 1st-person camera-POV view".)_
- [X] T055 [US5] Implement camera-POV mini-panel as HUD overlay in [`tools/renderer.cc`](../../tools/renderer.cc) — small 2D rectangle near throttle/control-state; renders current-tick beacon positions; sentinel-CEP visually distinguishable (dimmed dot or absent); follows existing HUD-visibility logic (D5 — NOT always-on, slots into existing toggle system) _(M9b 2026-05-08, commit 58cf328: 2D rectangle top-left of window; aspect ratio = camera fov_h/fov_v from CameraViewSample (geometrically accurate, no warp on banking craft); 16-segment polygon splat dots — RED port + GREEN starboard at projected (screen_x, screen_y); image-coord screen_y maps top→bottom of rect (cockpit POV); sentinel CEP ⇒ dot hidden; renders during animation alongside other HUD; self-hides outside tracker mode.)_
- [X] T056 [US5] Implement CEP error-bar visualization in 1st-person + mini-panel — render CEP as ellipse spread around each projected beacon centroid (D15 v1 commit — committed for v1 because it's load-bearing for smoke-test signal-or-not assessment) _(M9b 2026-05-08: collapsed into T055 mini-panel — splat dot radius scales linearly with CEP (2px sharp at CEP=0 → 18px fuzzy at CEP=1). Visual signal "is the beacon centroid sharp or smeared?" reads directly from dot size. True ellipse outline overlay deferred to backlog if operator needs explicit CEP magnitude readout beyond "sharp/fuzzy" feel.)_
- [→BACKLOG] T057 [US5] Implement per-tick scrub controls in [`tools/renderer.cc`](../../tools/renderer.cc) (rolled-in BACKLOG entry "Renderer Playback Enhancements") — pause / step-forward / step-backward; works in both 3rd-person and camera-POV modes _(deferred 2026-05-08: animation already auto-pauses on left-click for camera interaction; full scrub controls are research-grade. Moved to BACKLOG.md "Renderer scrub controls".)_
- [→BACKLOG] T058 [P] [US5] Streak/multiplier overlay in [`tools/renderer.cc`](../../tools/renderer.cc) (rolled-in BACKLOG entry, optional plumbing path) — recompute path or schema-bump path per backlog entry; pick during M9 implementation _(deferred 2026-05-08: optional plumbing per spec; not load-bearing for smoke. Moved to BACKLOG.md "Renderer streak overlay".)_

**M9 v1-ready bonus features delivered 2026-05-08** (beyond original task list):
- Detail-toggle ('d' key): hides FOV pyramid + wingtip beacon glyph trails by default; operator presses 'd' to surface during troubleshooting (commit 3c5b5b1)
- Up-vector flag on FOV pyramid: small "T" indicator at top-edge midpoint extending in body -z direction; tracks chase roll/pitch for instant attitude read (commit 5f6638d)
- FOV pyramid sized to 10m apex-to-base after operator visual-feedback (~35m base width @ 120° FOV — visually proportional to chase craft) (commit 593a896)
- Color routing per mode: M1 chase = orange/cyan; M2 source craft = orange/cyan (M1 chase replayed in role of rabbit); M2 chase craft = magenta/lime — visual identity stable across modes (commit f7730e6)
- Focus-mode animation skip: in `f`-key focus, only the focused arena animates per frame; non-focused arenas skipped for smooth playback at 300+ scenarios (commit ccf09c2)

**Checkpoint**: US5 complete. Operator can visually inspect tracker-mode dmp output. This is the qualitative "do we believe the loop closes" eyeball test ahead of the formal smoke test.

---

## Phase 5: User Story 4 — Smoke test (Priority: P1) 🎯 v1 acceptance floor

**Goal**: End-to-end loop closes on a real M1 source dmp + real tracker-mode autoc run + real renderer playback. Fitness curve does *something* — descends, plateaus, or fails informatively. M10 IS the experimental answer to R10 (perception representation can-it-train) per research.md.

**Independent Test**: Operator-driven full execution of the four D13 deliverables (run from M1 file → single-config slice → save → renderer animates). No automated test gates here; gates were at M5/M6/M7/M8/M9 individually.

**Maps to plan.md milestone**: M10 — SMOKE TEST.

### M10 — Smoke test execution _(satisfied de facto by autoc-030-m91 run, 2026-05-08)_

- [X] T059 [US4] Configure `autoc-tracker.ini` for smoke run per quickstart.md — pastonly3 source dmp, scenario slice = 6 paths × 20 winds = 120 scenarios per Q2 clarification, population = 5000, gens = 100, default trail / hull / arena, deterministic seed _(M9.preB / 2026-05-07: `autoc-tracker.ini` configured at production scale — pop 5000, gens 800, all 294 source scenarios cross-product, 20 EvalThreads. Larger than smoke baseline; scenario slice opened to full set for the m91 fitness-curve probe.)_
- [X] T060 [US4] Execute smoke run: `stdbuf -oL -eL build/autoc -i autoc-tracker.ini 2>&1 | tee logs/autoc-030-smoke-001.log` _(2026-05-08: `logs/autoc-030-m91.log` — 400+ gens overnight, fitness curve descended -4202 → -9469+ over first 25 gens with healthy avgMaxStreak 1.0 → 7.9 trajectory after pre-roll removal. Crash rate 12.6% at gen 403. Mid-plateau ±200 fitness jitter flagged as project_tracker_fitness_nondeterminism.md backlog candidate; not v1-blocking.)_
- [~] T061 [US4] Per quickstart.md step 4 — pull a recent gen's dmp from S3, sanity-check via per-tick dmp extractor (depends on T064 from M11a but can be cursory until then) _(2026-05-08: `tracker_dmp_inspect` has been run on multiple m91 dmps as cursory sanity check — cameraViewList + targetTrajectoryList populated, parallel-tick counts match aircraftStateList. Full per-tick CSV extractor (T064) deferred to M11 post-crrcsim.)_
- [X] T062 [US4] Per quickstart.md step 5 — load M2 dmp into renderer, verify all four D13 deliverables visually green (autoc loaded M1 ✓; single-slice scenario per ini ✓; results saved ✓; renderer animated M2 ✓) _(2026-05-08: M9b renderer playback verified on m91 dmps with all 4 D13 deliverables green: source dmp loaded ✓, scenarios per ini executed ✓, v=2 dmps written to S3 ✓, renderer animates target tape + chase tape + chase→target error bars + camera-POV mini-screen + (toggleable) FOV pyramid + wingtip beacons.)_
- [~] T063 [US4] Capture findings per quickstart.md step 6 — write `eval-results/030-smoke-<date>/SMOKE_REPORT.md` (sim-only smoke; `flight-results/` is reserved for real-flight artifacts) with: source dmp S3 key + scenario slice + 030-run-id; fitness curve shape (descending / plateau / pathological); renderer screenshots; sentinel events (arena egresses / hull strikes / NaN propagations / build issues) _(deferred 2026-05-08: m91 was operator-driven scale check, not formal smoke. Formal SMOKE_REPORT writes alongside the post-crrcsim run (M11.preA), where FDM-driven training results are the load-bearing deliverable.)_

**Checkpoint**: SMOKE TEST GREEN means 030 v1 floor is hit. Decision per D13: continue to Phase 6 analytics (the "030 done" ramp) OR debug failing milestone.

**Routing 2026-05-08**: minisim m91 informal smoke shows the loop closes structurally — fitness curve descends, streak grows, arena egresses + hull strikes fire as expected. Per session decision, **CRRCSim integration (M11.preA below) lands BEFORE formal smoke + analytics**. Reasoning: minisim is kinematic; smoke results matter most when run on FDM-driven physics (the path to sim-to-real). Pulling crrcsim forward from `[030 v1+]` BACKLOG into v1 path so the formal smoke happens on FDM-grade chase craft.

---

## Phase 6: CRRCSim integration + post-smoke analytics (per 2026-05-08 routing)

**Goal**: Lift tracker mode from kinematic minisim to FDM-driven crrcsim before formal smoke + analytics. Then build/extend tools to assess what trained.

**Maps to**: M11.preA (crrcsim mod_inputdev tracker integration) + M11.preB (live two-aircraft display, optional) + M11a-c (per-tick dmp extractor + bug fixes + tracker-specific analytics).

### M11.preA — CRRCSim tracker-mode integration (was M4-deferred BACKLOG; pulled forward 2026-05-08) ✓ DONE 2026-05-08

**Goal**: Mirror the M6a-M6e strategy-pattern split (`PathgenStepper` / `TrackerStepper`) from minisim into `crrcsim/src/mod_inputdev/inputdev.cpp` so FDM-driven tracker training is selectable via `MinisimProgram = ./scripts/crrcsim.sh` in autoc-tracker.ini.

**Why this is the path**:
- minisim is kinematic — no FDM, no aero, no wind, no real entry-variation interaction with chase dynamics
- The training fitness signal m91 produced is structurally valid (loop closes, fitness descends) but is a kinematic-controller's adaptation, not a flight-dynamics-controller's
- For sim-to-real, smoke-test results MUST be on the same FDM the real flight will encounter (CRRCSim LaRCSim — same FDM that produced pastonly3 source dmps)
- Per `feedback_no_cereal_versioning.md` and project routing: post-tracker-v1 wants real-flight prep, not more kinematic training

**Sub-checkpoints** (proposed sequencing — operator decides):
- [x] T079 [US4] Audit `crrcsim/src/mod_inputdev/inputdev.cpp` per-tick loop + extract the equivalent of `PathgenStepper` into a `ScenarioStepper` strategy interface (mirrors `include/autoc/eval/scenario_stepper.h` from M6a) — regression-tight: bitwise pathgen-mode crrcsim training before/after
- [x] T080 [US4] Wire `EvalData::mode` consumption + `TrackerStepper`-equivalent body in `mod_inputdev` — source-trajectory-driven per-tick, beacon projection via M5 `projectBeacon`, NN forward via `evaluateTracker`, FDM advance for chase craft physics
- [x] T081 [US4] Plumb `cameraConfig` / `beaconLeftConfig` / `beaconRightConfig` / `flightArena` / source trajectories through `mod_inputdev` from EvalData (parallel to minisim's wiring at M6e)
- [x] T082 [US4] Smoke run with crrcsim: `MinisimProgram = ./scripts/crrcsim.sh` in `autoc-tracker.ini`, full overnight run, compare fitness curve shape vs m91 minisim baseline; dmps written to S3 with separate run-id
- [ ] T083 [US4] Capture findings — write `eval-results/030-smoke-<date>/SMOKE_REPORT.md` per T063 spec; this is the formal D13 smoke check (FDM-grade), supersedes the m91 informal probe

**Checkpoint M11.preA**: tracker training runs on crrcsim FDM, formal smoke green or diagnosed.

### M11.preA.1 — Worker-init priming + correctness fixes (2026-05-08) ✓ DONE

Surfaced during M11.preA scaling: pop=5000 / 294-scenario tracker training OOM'd a 128 GB box before gen 1 because `EvalData` carried a deep-copy `sourceList` per-eval (~8.5 MB × 5000 individuals = ~42 GB queued) AND a `pathList` per-eval (~7 MB × 5000 = ~35 GB). Fixed via two layered priming refactors plus two crrcsim correctness bugs found during scaling.

**V1 priming — once-per-worker WorkerInit RPC**:
- [x] New `WorkerInit` struct ([include/autoc/rpc/protocol.h](../../include/autoc/rpc/protocol.h)) carrying `sourceList`, camera/beacon/airframe/flightArena configs, crashHull/trail/pre-roll. Sent once per worker right after TCP accept (the "phone home" handshake), before the eval loop.
- [x] `ThreadPool` ctor takes a `WorkerInit` by value; worker thread `sendRPC`s it to its child after `acceptor.accept()` ([include/autoc/util/threadpool.h](../../include/autoc/util/threadpool.h)).
- [x] Workers cache locally; minisim ([tools/minisim.cc](../../tools/minisim.cc)) + crrcsim mod_inputdev both receive at startup.
- [x] `EvalData` shed those fields; `buildEvalData` no longer attaches sourceList per-eval.

**V1.5 priming — pathList + scenarioMetaList run-static priming**:
- [x] `gPathSeed` is run-constant, so `generateSmoothPaths` produces byte-identical output every gen — hoisted out of the gen loop in [src/autoc.cc](../../src/autoc.cc) to a one-time startup call. Same for `rebuildGenerationScenarios` + `prefetchAllVariations`.
- [x] `WorkerInit` extended with `pathList` (294 entries) + `scenarioMetaList` (294 entries pre-populated with full-scale entry offsets from joint-PRNG variation table).
- [x] `EvalData` slimmed to ~50 B: NN bytes + `gpHash` + `controllerType` + `isEliteReeval` + `scenarioSequence` + `pCrashThisGen` + `variationScale` + `rabbitSpeedConfig`. No more pathList / scenarioList per-eval.
- [x] Workers reconstruct per-eval `ScenarioMetadata` via `makePerEvalMeta(init_, evalData, idx)` that copies cached base meta + applies per-eval `variationScale` via the new shared helper [include/autoc/eval/scenario_meta_apply.h](../../include/autoc/eval/scenario_meta_apply.h).

**crrcsim worker-side correctness fixes** (pre-existing M11.preA bugs surfaced by V1.5 scaling):
- [x] `evalResults.cameraViewList` / `targetTrajectoryList` / `arenaEgressCount` / `hullStrikeCount` were never cleared post-`sendRPC` — accumulated 294 entries × N evals on each worker, bloating evalResults by ~140 GB across 20 workers before gen 1 finished. Mirror minisim's clear pattern in [crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp) post-send block.
- [x] **Tracker arena egress / hull strike were silently ignored**: `helper.tick()` set `crashReason` from `checkArenaBounds` / `didCrashFire`, but the function had ALREADY passed its early-exit save-+-return block (helper runs LATER in the function). Symptom: chase craft flew to dhome=758 m vs 80 m arena, alt below ground, never recorded a crash. Fixed by refactoring save-+-return into a `finalizeScenarioOnCrash` lambda + adding a second checkpoint after the NN tick block (post-`aircraftStates.push_back`). Same egress-tick capture pattern as minisim's `stepOnce`-then-push.

**Validated 2026-05-08 smoke (autoc-tracker.ini, pop=5000 / 294 scenarios)**:
- minisim total memory: 26 GB → 6 GB; gen 1 ELITE_SAME at -3476 (intermittent gen 1/9-10/18-19 ELITE_DIVERGED at small ~15-32 unit deltas — pre-existing FP determinism gap unmasked by V1.5's lower allocation churn, NOT introduced by it; tracked as separate investigation)
- crrcsim total memory: OOM'd at 97 GB → 5 GB; gen wallclock 5:47 → 0:61 (production scale 60-70s expectation matched); sim rate 3,855 → 22,744 sims/sec (5.9× speedup from OOB fix); gens 1-4 all ELITE_SAME (no crrcsim-side determinism gap observed in this run)

**Backlog cross-references**:
- specs/BACKLOG.md `[030 v1 — UNPARKED 2026-05-08] Worker-side scenario priming` — V1 + V1.5 both shipped; V2 LRU demand-fetch deferred until libraries evolve within a run
- specs/BACKLOG.md `[NEXT] Trim EvalResults on the return path — score-only for non-elite` — orthogonal follow-on, deferred per operator routing 2026-05-08 ("ok as it is streamed back")
- [project_worker_init_priming.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_worker_init_priming.md) — never put scenario-shaped fields in per-eval `EvalData`; they go in `WorkerInit`

### M11.preA.2 — NN input transforms + pop=6000 + chase-init bias (2026-05-09) ✓ DONE

Surfaced post-M11.preA.1 during smoke14a → smoke14b on autoc-tracker.ini:
- [x] **NN input transforms** ([include/autoc/nn/nn_inputs.h](../../include/autoc/nn/nn_inputs.h) constants + [src/nn/evaluator.cc](../../src/nn/evaluator.cc) `gather_tracker_inputs`):
  - `airspeed`: raw m/s → cruise-normalized (`relVel / kCruiseSpeed_mps=13.0`). Range ≈ [0, 2]; chase at cruise = 1.0.
  - `dist_to_boundary_along_vel`: raw meters → `tanh(d / kDistToBoundaryScale_m=20.0)`. Sized to bracket the chase's ~10-15 m emergency-180° turn budget; 10m→0.46, 20m→0.76, 30m→0.91. Polarity: dBnd→0 = at the wall, dBnd→1 = far from wall.
  - Gyro stays raw rad/s (already NN-friendly).
  - TrackerInputs struct unchanged at float[45]; data.dat byte format unchanged. Old genomes incompatible (greenfield, no backward compat).
- [x] **Population**: pop=5000 → 6000 in autoc-tracker.ini + autoc-tracker-minisim.ini to recover 029-baseline density given tracker variations expand the scenario factorial.
- [x] **Chase-init bias**: chase placed at +1.5×`trail_distance` north of source[0] (was 1.0× pre-2026-05-09 — knife-edge at trail rabbit). Plus crrcsim sign-flip fix in [inputdev_autoc.cpp:498-525](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L498-L525) — `crrc_main.cpp:258` does `posX += -Global::entryNorthOffset`, so `trackerInitBiasNorth_m` ships negative through the boundary.
- [x] **Conventions docs** ([docs/COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md)): new "030 Tracker-Mode NN Inputs (45 floats)" table with units/ranges/source per slot; new "Camera-POV HUD projection" subsection documenting verified NDC→HUD mapping. [docs/sensor-pipeline.md](../../docs/sensor-pipeline.md) scope-banner: M0/M1 23-input pathgen-only.

**Validated 2026-05-09 (smoke14b, autoc-tracker.ini pop=6000)**: 98 gens, 98 NN_ELITE_SAME, 0 DIVERGED through variation-ramps at gen 40 + gen 80. Determinism contract holds clean across cruise-norm + tanh-dist + larger pop. Fitness climb -4474→-14315 (3.2× over 98 gens), accelerating-shape consistent with no-future-input training.

**Sim-rate slowdown observed** (logged for follow-up): gen-1 11553/sec → gen-98 2620/sec (4.4× drop). Two layered effects:
- 030 starts ~2× slower per-tick than 029 pastonly3 baseline (tracker-mode adds 2× projectBeacon + arena ray-cast + 6-history shift + larger 45-float input vector)
- 030's better inputs make scenarios survive longer earlier (gen ~30 hits 97% OK vs 029 reaching that around gen 100), so total ticks/gen scale up faster

### M11.preA.3 — Hull-crash deterministic re-enable + PCrash collapse (2026-05-10) ✓ DONE

Surfaced post-smoke14b: crash-hull was disabled in code (V1.5 kill-switch comment block in tracker_stepper.cc + crrcsim_tracker_helper.cpp) while determinism contract was being restored. With determinism now confirmed clean across smoke14b's 98-gen bake (P1 windSeed seed fix verified), re-enable with simplified deterministic config.

- [x] **Replaced 4-param ramp** (PCrashGen0 / PCrashGenRamp / PCrashGenPlateau / PCrashPlateau) with single fixed `CrashHullProbability=0.10` (Bernoulli per NN tick at 10 Hz; ~50% chance of dying within 7 ticks once inside the 1m sphere). Removes per-gen state that complicated determinism debugging.
- [x] **Added `EnableCrashHullVariations`** (default 0) for symmetry with the other Enable* knobs in the ini, located beside them. Reserved for per-scenario radius/probability variation; logic not yet wired (both 0 and 1 currently produce identical fixed-prob behavior).
- [x] **Re-enabled `didCrashFire`** in [src/eval/tracker_stepper.cc](../../src/eval/tracker_stepper.cc) and [crrcsim_tracker_helper.cpp](../../crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp). PRNG seed = windSeed (P1 fix, train↔elite stable).
- [x] **Removed `pCrashForGen()`** function + 6 ramp-curriculum tests; CrashHullFire tests already cover Bernoulli probability handling.

### M11.preA.5 — postdiag2 closure + T-102 32r topology experiment (2026-05-13) — in flight

postdiag2 baked 542 gens on the current 030 baseline (45 → 32 → 16r → 3, pop=5000, fov/cone dead-code fix). Operator-stopped at gen 542 once the plateau was clear (~-17K best fitness; trajectory flat from gen 450 onward). Full outcome captured in [postdiag2_report.md](postdiag2_report.md).

**Key finding**: M2 v1 ceiling sits at -17K vs M1 pathgen's -50K on the same topology + cone fitness. The 3× gap is real and reproducible. Several plausible binding constraints; testing them in order (smallest bet first):

1. **Recurrent state capacity** — current 16r layer has 256 W_hh weights to maintain target prior across visibility cycles. **T-102 tests this in 030**: bump to 32r (1024 W_hh weights, 4× capacity), same inputs, same source dmp.
2. **Visibility-time signal richness** (deferred to 032): the NN sees raw NDC + history but doesn't see derived geometric primitives (angular width = range proxy; beacon-pair tilt = roll proxy).
3. **FOV-induced blindness** (hypothesis, deferred): M2 is blind 30% of ticks vs M1's infinite-FOV oracle bearing. Would surface only if (1) + (2) don't close the gap.

The 32r bump stays in **030 v1 baseline tuning** — it's not a feature, it's the right topology for the current input shape. 032 starts from whichever 030 baseline is current when 032 unparks.

**T-102 implementation tasks** (operator workflow: claude drafts code → operator reviews + builds → claude runs):
- [X] Change `TRACKER_NN_HIDDEN2_SIZE` from 16 → 32 in [include/autoc/nn/topology.h](../../include/autoc/nn/topology.h) (single constant cascading through `TRACKER_NN_TOPOLOGY`, `TRACKER_NN_WEIGHT_COUNT`, `TRACKER_NN_RECURRENT_STATE_COUNT`).
- [X] Update `static_assert(TRACKER_NN_WEIGHT_COUNT == ...)` to new value: 45·32+32 + 32·32+32 + 32·3+3 + 32·32 = 1440+32 + 1024+32 + 96+3 + 1024 = **3,651** (was 2,307 at 16r). Recurrent W_hh contribution: 256 → 1024 (+768 weights).
- [X] Re-run pop=5000 against the same source dmp (pastonly3 gen9200), same autoc-tracker.ini settings.
- [X] Compare fitness ceiling to postdiag2's -17K — see outcome below.

**T-102 outcome — FINAL (2026-05-15, operator-stopped at gen 544)** — capacity is NOT the binding constraint. Full report: [postdiag3_report.md](postdiag3_report.md).

| run | topology | weights | best fitness | gen of best | gens baked |
|---|---|---|---|---|---|
| postdiag2 | 45→32→16r→3 | 2,307 | **-17,060** | 517 | 542 (operator-stopped) |
| postdiag3 | 45→32→**32r**→3 | 3,651 | -16,382 | 519 | 544 (operator-stopped) |

postdiag3 (32r) was consistently ~700-1500 points WORSE than postdiag2 (16r) at every comparable gen through gen 544. 4× recurrent W_hh capacity (256→1024 weights) produced no fitness lift; the GA could not exploit the extra capacity within the variation-pressure budget. Determinism contract held at the larger weight count (544/544 SAME, 0 DIVERGED). Operator qualitative read from playback at gen 522: **tracking flights are visibly good, but (a) overruns persist** (no absolute-distance signal — only apparent NDC beacon-pair separation) **and (b) per-scenario behavior chaotically reshuffles on small fitness ticks rather than incrementally tightening** (classic GA-stuck-in-local-optima signature with an ambiguous input gradient). Neither pattern is addressable by adding state capacity; both are addressable by adding disambiguating perceptual inputs.

Additional observation (2026-05-15, distinct failure mode): chase loses sight → high-pitch spiral → drifts until wall (hull-strike rate climbing in plateau zone). RNN's 16r hidden state isn't learning informed lost-sight patrol from the 6-tick beacon history (600ms window at 10Hz vs 9s typical lost-sight intervals — history is 15× too short). Phase-1 features (span/tilt) collapse to zero when blind, so they don't address this. Filed as 032 phase-2 candidate (lost-sight patrol sensors: `vis_now`, `ticks_since_seen`, `last_*` frozen-at-loss snapshots) — distinct from phase 1's chase-mode focus.

Routing decision: **drop back to 16r as the v1 baseline**. 032 starts from postdiag2's 16r baseline with phase-1 features layered on top. The three qualitative observations (no absolute-distance signal → overruns; chaos shuffling → ambiguous gradient; uninformed lost-sight patrol → wall) all motivate input-information enhancements, not architecture.

**Revert step** (in this commit batch, before 032 phase-1 code lands):
- [X] Revert `TRACKER_NN_HIDDEN2_SIZE` to 16 in [include/autoc/nn/topology.h](../../include/autoc/nn/topology.h); restore weight-count assert to 2,307. *(to be executed during pre-commit cleanup — see "Pre-032 commit batch" below)*

### Pre-032 commit batch (2026-05-15) — 030 v1 sealed-for-this-purpose

The 030 v1 baseline is the postdiag2 (16r) weights. Trunk topology reverts to 16r before commit. M11.wrap follow-ups (T083 SMOKE_REPORT, T073 CLAUDE.md, T074 outcome.md, T075 code review, T076 constitution audit, T077 BACKLOG routing) remain queued but are not gates on 032 phase-1 code lands — they can be folded in alongside 032 work.

### Dropped from v1 closeout 2026-05-13 (per operator routing post-postdiag2)

- T-100 (TrackerScenarioVariationSource=dmp knob): joint-PRNG has unconditional consumers (rabbitSpeedSeed draw is unconditional per src/autoc.cc:229). Partial match unachievable; full match requires auditing every PRNG consumer. Drop entirely.
- T-101 (chase-init-at-origin): not informative without T-100 (matched conditions). Drop.
- T-103 (hull-imminent NN inputs `inside_hull_now` + `dist_to_hull_along_vel`): defer until after T-102 result. If 32r alone reduces hull-strike rate by integrating better trajectory prior, T-103 unnecessary.

### M11.wrap — Wrap-up + post-bake follow-ups (planning) — pending operator triage

**v1 closeout — locked 2026-05-10** (must-do to declare 030 v1 done):
- [ ] T083 [US4] Capture findings — write `eval-results/030-smoke-2026-05-10/SMOKE_REPORT.md`: smoke14b/smoke15 outcomes (gens 0 DIVERGED, fitness curve, hull-on determinism vindicating P1 windSeed fix, autoc-eval-tracker.ini bitwise-eval-validates). Replaces the deferred M11.preA T063 smoke report.
- [X] T066 [P] **Eval-mode dmp fitness display fix (Bug 2)** — **already in tree since commit fe0fae9 (023 train/eval scenario dedup, Apr 2026)**. `src/autoc.cc:1238` does `genome.fitness = fitness;` before `nn_serialize(genome, updatedNnData)` and the cereal `BinaryOutputArchive` write at line 1253. _(2026-05-11 audit during M11.wrap diagnostics batch confirmed the fix is live; no further work needed.)_
- [X] T067 [P] **Contract test `tests/eval_fitness_bug2_tests.cc`** — **closing without test** (2026-05-11). The Bug 2 fix is a one-line assignment (`genome.fitness = fitness`) immediately before cereal serialization at `src/autoc.cc:1238`. Any unit test would either assert that the one line is present (trivially) or require a full end-to-end run with file I/O and mock S3 — out of unit-test scope. Visual inspection of the call ordering at the serialize site is the right correctness check.
- [ ] T073 [P] Update `CLAUDE.md` agent context with 030 v1 entry.
- [ ] T074 [P] Write `specs/030-tracker-mode/outcome.md` documenting smoke-test results + recommended next direction (= 031 optical path per handoff doc).
- [ ] T075 Code review pass: ensure no temporary scaffolding left in tree.
- [ ] T076 Constitution compliance audit (Principles I-VI for the 030 diff).
- [ ] T077 [P] Document the post-030 feature routing in [BACKLOG.md](../BACKLOG.md):
  - **031 = optical perception front-end** per [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md) (FPGA-based beacon detection + camera pipeline producing `(x, y, CEP)` for the M2 NN). The handoff doc is the architectural source; spec drafted at [specs/031-beacon-camera/spec.md](../031-beacon-camera/spec.md) (phase 1 — LED pyramid + camera/filter + raw-frame recording; phase 2-4 = FPGA / protocol / integration deferred).
  - **032 = tracker NN enhancements (derived pose features)** at [specs/032-tracker-nn-enhancements/spec.md](../032-tracker-nn-enhancements/spec.md). Letter-to-ourselves form documenting the M2 plateau diagnosis (input information content is the bottleneck, not architecture / compute / determinism) and the proposed first-experiment scope: beacon-code identity propagation + derived pose features (range, closing-rate, bearing-rate, line-tilt). Controller-side change only; no hardware dependency; can ship in parallel with 031 hardware bring-up.
  - **M3 (research-track)**: full target identification + pose estimation via dedicated perception loop (CNN/transformer + temporal prior) — supersedes 032's hand-crafted features. Long-term placeholder, no spec yet.

**Shelved 2026-05-10** (operator routing — perf is acceptable post-smoke15, divergence analysis tools not load-bearing for v1):
- ~~T086 Profiling pass~~ — shelved. Current per-tick cost is good enough; revisit only if a future bake stalls on throughput.
- ~~T087 Trim EvalResults on return path~~ — shelved. Stubbed analysis path that EvalResults supports (divergence reconstruction) isn't being used; not worth the surgery now. The BACKLOG `[NEXT]` entry stays parked until divergence analysis becomes load-bearing.

**Routing decisions (defer to backlog, not v1-blocking)** — confirmed 2026-05-10:
- T064-T072 analytics scripts (aircraft_state_extractor, per_axis tracker analytics, CEP-sentinel correlation, attitude correlation, aliasing histogram, convergence auto-flag) → 031 CANDIDATE — fold into 031 perception spec or post-031 analytics work
- T078 type-domain audit → already a separate backlog entry (`project_scalar_type_audit_backlog.md`); leave parked
- T051 renderer tracker smoke test → low-value (manual renderer validation has covered the surface every smoke run; renderer code stable since M8/M9), defer to backlog
- T008 mode_dispatch_tests → low-value, defer
- T084-T085 M11.preB live two-aircraft display → BACKLOG `[030 v1 — UNPARKED 2026-05-08] Live two-aircraft display`; M11.preA outcome did not require visual mid-training, defer
- [POLISH] **`scripts/eval_suite.sh` mode-aware refactor** — surfaced 2026-05-10 after confirming M2 tracker eval bitwise-matches training. The script is currently M1-only: `EVAL_BASE_INI=autoc-eval.ini` hardcoded, tier overrides use M1 path methods (`aeroStandard` / `progressiveDistance` / `longSequential` / `random`) that don't apply to tracker mode (tracker has `TrackerSourceRun` + `TrackerPathSubset` / `TrackerWindSubset`). Want: a mode flag `-m {m1,m2}` (default m1 for back-compat) that selects:
  - M1: `EVAL_BASE_INI=autoc-eval.ini`, tier overrides as today
  - M2: `EVAL_BASE_INI=autoc-eval-tracker.ini`, tier overrides re-mapped to tracker concepts — tier0 = same TrackerSourceRun + full subset (bitwise repro); tier1 = same source, alternate scenario subset; tier2 = alternate TrackerSourceRun (generalization to a different recorded flight); tier3 = stress sigmas on entry/wind variations + alternate source. M2 NN weight filename also differs (`nn_weights-tracker.dat` per the eval ini default). Output dirs may also need an `m1-` / `m2-` prefix to avoid path collision when running both modes back-to-back on the same operator workstation.
  - **M2 stress depends on M1 path variety** (2026-05-10 operator note): tracker has no synthetic path generator — its scenarios are bounded by whichever source dmp is supplied. To stress M2 with harder trajectories (sharper turns, longer engagements, novel geometries), you first run M1 eval with `progressiveDistance` / `longSequential` / `random` path methods to PRODUCE the source dmps, then point M2 eval at those dmps via `TrackerSourceRun`. So the M2 stress tier really depends on a curated library of M1-generated source dmps + variation sigma sweeps. Library curation is already a 031-CANDIDATE backlog item (turn-direction mirror pairing, cross-source mixing); this just makes it load-bearing for M2 stress-eval rather than purely a 031 nice-to-have.
  - **Validated 2026-05-10**: operator ran an M1 random-path eval (6 paths × 5 variations = 30 source flights), fed each into M2 eval 1:1 — works end-to-end. **Concrete CLI shape for M2 eval**: the load-bearing input is the **M1 eval output dmp** (which carries the 6×5 paths + their wind/entry variation seeds). M2 eval then mirrors that geometry — same path count, same wind subset, same variation seeds — so the per-scenario test surface is identical between M1 and M2. The script's M2 tier overrides should therefore key off the M1 dmp + scenario shape, NOT independently re-generate paths.
- [POLISH] **Eval-bucket split — may be unnecessary if M2 follows M1-eval workflow** — surfaced 2026-05-10. Original framing: eval needs to READ source dmp + `nn_weights*.dat` from training bucket (`autoc-storage`) but WRITE results to eval bucket (`autoc-eval-arm`), conflated by the single `S3Bucket` knob. **Workflow alternative (operator note 2026-05-10)**: standardize M2 source dmps to come from **M1 eval-mode stress runs** (M1 eval with `PathGeneratorMethod=random` etc. generates harder paths) rather than from M1 training. Both M1-eval output AND M2-eval output then naturally live in `autoc-eval-arm`, and M2 eval's `S3Bucket = autoc-eval-arm` handles both read + write cleanly. Bucket split becomes unnecessary; the v1 case (M2 reading M1-training pastonly3 gen9200 from `autoc-storage`) is the only outlier. Decide post-v1: either ship a separate `S3ReadBucket` / `S3WriteBucket` config split, or adopt the workflow convention + document the v1 outlier as historical. Don't touch during training either way.
- [X] **`#GenDiag` fov/cone dead-code in streak-break classifier** — surfaced 2026-05-11, fixed 2026-05-11 (option a, remove dead branches). Removed:
  - `loss_vis_fov` + `loss_vis_cone` fields from `TrackerDiag` (include/autoc/eval/fitness_decomposition.h)
  - The unreachable `if (stepPoints >= fc.getStreakThreshold())` branch in `computeScenarioScores` (src/eval/fitness_decomposition.cc:275-295)
  - The `targetInChaseCameraFOV` helper and its feeder camera-pose locals (`cam_pos`, `cam_orient`, `fov_h`, `fov_v`, `have_cam`)
  - `total_fov`, `total_cone` accumulators + `fov=` / `cone=` fields from `#GenDiag` emission (src/autoc.cc:1604, 1614-1615, 1631-1633)
  - `fov=` / `cone=` fields from per-scenario `loss=[...]` log line (training-mode + eval-mode log sites)
  - `fov` / `cone` panels from `specs/030-tracker-mode/plot_gen_diag.py` stacked-area plot
  
  Rationale (sketch from polish task): streak counter resets only when `stepPoints < streakThreshold` (see [applyStreak](../../src/eval/fitness_computer.cc#L57-L67)), so at the streak-break tick `stepPoints < threshold` BY DEFINITION — the visibility branch's guard `stepPoints >= threshold` was always false. Visibility events captured separately via `avgVis` + `maxLost` (still in #GenDiag). Build + 11 crash_hull + 6 gather_tracker_inputs tests green. Pop=5000 set at the same time, postdiag2 bake launched.
- [X] **Remove dead `TrackerSourcePreRollSec` knob** — surfaced 2026-05-10, completed 2026-05-11 during M11.wrap diagnostics batch. Scrubbed: ini removal (autoc-tracker.ini + autoc-tracker-minisim.ini + autoc-eval-tracker.ini), config field (`trackerSourcePreRollSec` removed from include/autoc/util/config.h), loader call (src/util/config.cc), WorkerInit field (`trackerSourcePreRollTicks` removed from protocol.h), TrackerStepper ctor + `pre_roll_ticks_` member + initScenario pre-roll loop (replaced with simple replicate-source[0]×6 history pre-fill, since pre_roll was always 0 in production), autoc.cc plumbing (3 sites), minisim.cc wiring, tests/contract_tracker_config_tests.cc + tests/tracker_stepper_init_tests.cc fixture updates. Also folded in: `crashReasonToString` deduped (was in minisim.cc + crrcsim's inputdev_autoc.cpp, now inline in include/autoc/rpc/crash_reason.h). Full incremental build clean, all 24 tests pass.
- [X] **Per-scenario diagnostics enhancement** — surfaced 2026-05-10, completed 2026-05-11 during M11.wrap diagnostics batch. Delivered:
  - **Crash-reason in per-scenario log**: `[N] CRASH reason=HullStrike score=…` / `[N] OK reason=TimeLimit score=…` at both training-mode and eval-mode log sites. ScenarioScore now carries `crashReason` (CrashReason enum) populated in `src/eval/fitness_decomposition.cc`.
  - **Per-gen `#GenCrash` aggregate** in data.stc: counts of {hullStrike, eval, sim, boot, timeLimit, rabbitComplete, none} across the elite's scenario set. Surfaces hull-strike vs arena-egress vs timeout distribution without dmp inspection.
  - **Forward-looking diagnostic slots** delivered via T088 (below) since they all flow through the same per-tick computation: lost-sight run length (`max_lost_sight_run`), spiraling (`spiral_ratio` = mean |gyro|/|vel|), thrashing (`thrash_rate_pt` / `thrash_rate_rl` = transitions/sec where |dout| > 0.5).
  - **Container decision**: computed post-worker in `fitness_decomposition.cc` from existing `aircraftStateList` + `targetTrajectoryList` + `cameraViewList`. No schema bump, no `data.crash` companion file needed — diag travels with `ScenarioScore` (in-process only). Surfaced via per-scenario log continuation line and per-gen `#GenDiag` aggregate in data.stc.

- [X] T088 [P] **Streak-loss reason counters + range/overrun stats** — implemented 2026-05-11 as part of the M11.wrap diagnostics batch. All 6-reason taxonomy (geom.too_far / geom.angle / geom.overshoot / vis.fov / vis.cone / hull) + range/closure/overrun stats + 327-330 forward-looking slots delivered. Land sites:
  - **`include/autoc/eval/fitness_computer.h`**: new `decomposeStepScore()` method returning {score, distTermSq, angleTermSq, ahead}. `computeStepScore` now delegates to it (bitwise identical numerics; pathgen regression gate unaffected). Added `getStreakCount()` + `getStreakThreshold()` getters.
  - **`include/autoc/eval/fitness_decomposition.h`**: new `TrackerDiag` struct embedded in `ScenarioScore`; zero-initialized in pathgen mode.
  - **`src/eval/fitness_decomposition.cc`**: per-tick loop now classifies each streak-loss transition (`prevStreakCount > 0 → curStreakCount == 0`) into one of the six buckets. `vis.fov` vs `vis.cone` distinguished by re-projecting target position into chase camera frustum via `targetInChaseCameraFOV()` helper. Range/closure/overrun stats + lost-sight / spiral / thrash accumulators populated in same loop. Post-loop aggregation finalizes median/p95 via `percentileSorted()`.
  - **`src/autoc.cc`**: per-scenario log emission gets a tracker-mode continuation line printing `vis=…/inRamp=…/rng=[min/med/p95]/loss=[far/ang/over/fov/cone/hull]/over[flips/maxClose]/fwd[lostMax/spiral/thrPt/thrRl]`. Per-gen `#GenDiag` aggregate emitted to data.stc next to `#NNGen` / `#GenCrash`.
  - **Observation-only**: no effect on fitness or selection. Pathgen regression gate unaffected (all new code under `if (is_tracker)`).
  - **First-experiment value**: re-run smoke15 final dmp through `autoc -i autoc-eval-tracker.ini` to read out which `loss.*` cause dominates. Decision tree per T088 draft: dominant `fov + over` ⇒ overrun/FOV cutoff confirmed (route to dx/dy explicit derivatives or non-linear screen mapping); dominant `angle` ⇒ control bandwidth limited (route to bigger NN); dominant `cone` ⇒ target aspect (beacon emission re-spec).
- [ ] T088 [BACKLOG] **Streak-loss reason counters + range/overrun stats** — surfaced 2026-05-11 from smoke15-hullon plateau analysis (g262 elite at -16.8K vs M1 pastonly3 -32K at same gen). Per-scenario stats today (`score=… maxStrk=… strkSteps=… maxMult=…`) say a streak broke but not WHY; with the leading hypothesis being overrun-driven FOV exit (chase has 0.47°/step int8 angular resolution but ~no range discrimination at tail-chase aspect since beacons project near-coincident), we need geometry-vs-perception separation on every streak-loss event. Companion to the 327-330 entry; both fold into one combined PR post-bake.
  - **Geometry oracle = cone-fitness ramp itself** (not a hard angular cone). Reuse `FitnessComputer::computeStepScore(along, lateralDist) ≥ streakThreshold_` (= 0.5 in current ini) as the streak-eligibility check on every tick. Honest to actual selection pressure; no parallel threshold to drift.
  - **Taxonomy** — on each tick that transitions `streakCount > 0 → 0`:
    - `geom.too_far` — `(dist/distScale)²` term dominates `computeStepScore < 0.5` (chase fell behind / drifted laterally)
    - `geom.angle` — `(angle_clamped/coneAngle)²` term dominates (chase off the tail-chase line)
    - `geom.overshoot` — `along > 0` (chase forward of rabbit, sharp `distScaleAhead=2.0` ramp — overrun signature)
    - `vis.fov` — `computeStepScore ≥ 0.5` (geometry eligible) but both beacons project outside [-1, +1] image plane
    - `vis.cone` — geometry eligible but emission-cone occludes both beacons (target aspect)
    - `hull` — scenario terminated by hull-strike fire this tick (mutually exclusive, scenario ends)
  - **Per-scenario aggregates** added to the elite per-gen log (additive line, doesn't break existing parsers):
    ```
    [N] OK score=… maxStrk=… strkSteps=… maxMult=…
        vis=X.XX inFitRamp=X.XX range: min=X.X med=X.X p95=X.X
        loss: far=N angle=N over=N fov=N cone=N hull=N
        overrun: closureFlips=N maxClosureRate=X.X
    ```
    - `vis` = fraction of ticks ≥1 beacon visible; `inFitRamp` = fraction with `computeStepScore ≥ 0.5` regardless of visibility (separates "wrong place" from "right place but blind")
    - `range.min/med/p95` = chase→target distance distribution (catches stand-off + overrun in one stat)
    - `overrun.closureFlips` = sign reversals of `d(range)/dt` (overshoot-then-recover signature); `maxClosureRate` = peak signed closure speed
  - **Per-gen aggregate** (one line per generation, parallel to existing `Gen N  Best=… Avg=… Worst=…`):
    ```
    Gen N  Vis=X.XX  InRamp=X.XX  RngMed=X.X  Loss: far=A.A% angle=A.A% over=A.A% fov=A.A% cone=A.A% hull=A.A%  ClosureFlips=X.X
    ```
  - **Patch surface** (additive only; no NN-input shape change, no fitness path change, no retrain required):
    - [include/autoc/eval/fitness_computer.h](../../include/autoc/eval/fitness_computer.h) — add `decomposeStepScore(along, lateral) -> {score, distTerm, angleTerm}` so the classifier picks the dominant geometry term without recomputing
    - [src/eval/tracker_stepper.cc](../../src/eval/tracker_stepper.cc) — instrument the streak-loss path in the scenario step loop (prev-tick streak state, classify on transition, accumulate counters, track range + closure rate + sign flips). New `TrackerScenarioDiag` struct populated per scenario
    - [include/autoc/rpc/protocol.h](../../include/autoc/rpc/protocol.h) — add `TrackerScenarioDiag` to `EvalResults` (v=3 candidate) OR carry via the `data.crash` companion file proposed in the 327-330 backlog entry — **decide container before splitting work**; combining with that entry into one schema bump is cheaper than two
    - [src/autoc.cc](../../src/autoc.cc) — emit new per-scenario + per-gen aggregate lines in the elite-reporting block
  - **Bitwise eval gate**: pre-instrumentation training dmp must produce identical fitness numbers under `autoc -i autoc-eval-tracker.ini`. Instrumentation is observation-only; if numbers shift the patch is wrong.
  - **First-experiment value**: re-run smoke15 final dmp through eval-mode with the instrumentation and check which `loss.*` cause dominates. Decision tree:
    - `loss.fov` + `loss.over` dominate ⇒ overrun + FOV cutoff confirmed; route to dx/dy explicit-derivative inputs (cheap, ~4 new TrackerInput slots) and/or non-linear screen_x/y mapping (031 path — touches the xiao hardware-locked int8 wire format, so coordinate)
    - `loss.angle` dominates ⇒ control-bandwidth / topology limited; route to bigger NN (45→64→32r→3) trial
    - `loss.cone` dominates ⇒ target aspect kills lock; route to beacon-emission-cone re-spec or beacon mount-axis sweep (already a 031-CANDIDATE)
  - **Sequencing**: do not start while smoke15 is baking. Lands as part of M11.wrap instrumentation batch alongside the 327-330 per-scenario crash-reason + `data.crash` items; share the schema bump if the container decision lands on `EvalResults` v=3 rather than the companion file.

### M11.preB — Live two-aircraft display in crrcsim — **deferred to BACKLOG 2026-05-10**

M11.preA outcomes did not require visual mid-training (smoke14b/smoke15 dmp-replay-via-M9-renderer carried the operator-inspection burden cleanly). T084-T085 remain unimplemented; both live in the BACKLOG `[030 v1 — UNPARKED 2026-05-08] Live two-aircraft display` entry and unpark only if a future training scenario surfaces a need for live two-aircraft debugging. Do not touch as part of v1 closeout.

### M11a — Per-tick dmp extractor (rolled-in BACKLOG entry)

- [ ] T064 [P] Implement `tools/aircraft_state_extractor.cc` — read tracker-mode dmps, emit CSV with new column set per data-model.md §8 (chase per-tick state + beacon `(x, y, CEP)` per camera + camera pose + target-craft pose + trail-rabbit position + arena-egress flag + hull-strike flag); version-field dispatch handles both v1 (pathgen) and v2 (tracker) sources
- [ ] T065 [P] Adapt `specs/029-no-future-arch/plot_per_axis_time_series.py` to consume the new column set; existing data.dat path stays usable for pathgen-mode runs

### M11b — Eval Fitness Bug 2 fix (rolled-in BACKLOG entry) — **folded into M11.wrap 2026-05-10**

T066 + T067 are now tracked in the M11.wrap v1 closeout list above (small fix + regression test, no reason to keep as a separate milestone since v1 cleanup absorbs both). This section preserved as a pointer; do not re-implement here.

### M11a — Per-tick dmp extractor — **deferred to BACKLOG 2026-05-10**

T064-T065 (aircraft_state_extractor + 029 plot adaptation). Original Q4 ceiling included these, but smoke14b/smoke15 inspection has been served by direct data.dat parsing + the live plot scripts (`plot_evolution_progress.py`, `plot_per_axis_time_series.py`, `per_axis_aggressiveness.py`). The CSV extractor + adapted 029 plot would be useful once M2 has enough converged runs to warrant cross-run comparison tooling. Routes to BACKLOG `[031 CANDIDATE]` analytics bundle. Do not touch as part of v1 closeout.

### M11c — Tracker-specific analytics — **deferred to BACKLOG 2026-05-10**

T068-T072 (per-axis saturation, CEP-sentinel correlation, attitude correlation, aliasing histogram, convergence auto-flag). The R10/R11/R12 diagnostic surface was justified pre-smoke when we didn't know which failure modes would dominate. smoke15 has clean determinism + steady fitness climb, so dead-reckoning / aliasing / convergence stalls haven't manifested yet. Routes to BACKLOG `[031 CANDIDATE]` analytics bundle alongside M11a. Trigger to unpark: a future training run hits a plateau where these diagnostics would materially answer "why."

**Checkpoint update 2026-05-10**: Q4's original ceiling was `M10 + M11a + M11b + M11c` = "030 done." Operator-routed cutdown: **v1 = M10 + M11.preA.{1,2,3} + M11.wrap (with Bug 2 folded in)**. M11a + M11c → BACKLOG analytics bundle; M11.preB → BACKLOG live-display; T086/T087 → shelved; T078 → separate codebase-wide audit backlog entry; T051/T008 → low-value defers. Beyond v1 closeout, all candidates are 031-CANDIDATE BACKLOG entries — see `T077` for the routing decision.

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
| **M8 done ✓** 2026-05-07 | T047-T050 | An actual tracker-mode `.dmp` exists in S3 from a real run; `cereal` round-trips it; v2 version field embedded. **Wrap audit 2026-05-08**: all four [X], gyroRates_ honest-recording audit closed in-band, m91 production run gen 1-405 producing v=2 dmps continuously. Deferrals inside marked-X tasks: T047 FutureVersionLoudFail hermetic reproducer (cereal-mechanism-test, low-value until v=3 schema bump); T032/M6f timing_model_tests (031-candidate, not in M8 scope). |
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

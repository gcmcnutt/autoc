---
description: "Task list for 034 — M1/M2 Cleanup + Craft Variations → Flight Test"
---

# Tasks: M1/M2 Cleanup + Craft Variations → Flight Test

**Input**: Design documents from `/specs/034-energy-objective-cleanup/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/craft-variation-contract.md

**Tests**: REQUIRED — Constitution Principle I (Testing-First) mandates tests for significant changes. Smoothness tests are *deleted* (not disabled); craft-variation invariants get new tests. Pure-removal stories (US1/US2) use operator-run eval-parity as the test-of-record.

**Organization**: By user story. Unlike a greenfield feature, 034's stories have a real sequence — **US1 (minisim teardown) must land first** so US2/US3/US4 transport edits are single-path. US2 should precede US4 (clean schema before adding craft fields). US5 is operator-run acceptance.

**Policy reminders baked into tasks**: greenfield cereal changes, **NO version revision** for transport or .dmp (M2-era policy); clean cuts, no back-compat aliases (Constitution III); `gp_scalar` for new eval math (VI); no in-class defaults on config-sourced members (VII); operator drives bakes/eval gates (`feedback_operator_runs_regression_gate`).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependency on incomplete tasks)
- **[Story]**: US1–US5
- **(operator-run)**: executed by the operator, not the implementing agent

---

## Phase 1: Setup (Shared)

- [X] T001 Confirm clean baseline at branch HEAD before any edits: `bash scripts/rebuild.sh` (autoc+crrcsim build + tests green) and `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green.
- [ ] T002 [P] (operator-run) Capture behavior-neutrality reference for SC-002: run a fixed-seed M1 eval (`./build/autoc -i autoc-eval.ini`) and archive the per-scenario fitness from `data.stc` as the pre-change baseline.

---

## Phase 2: Foundational (resolve research unknowns that gate the stories)

**⚠️ These cheap verifications de-risk US1/US2/US4 file decisions — do first.**

- [X] T003 [P] Verify `include/autoc/eval/derived_features.h` also holds the 032 derived perceptual features (beacon span / tilt) — confirms the file is RETAINED in US2 and only the `SmoothnessMotionMode` enum + `compute_smoothness_factor()` are stripped.
- [X] T004 [P] Determine whether `PathgenStepper` (`include/autoc/eval/pathgen_stepper.{h,cc}`) is consumed ONLY by `tools/minisim.cc` (crrcsim has inline pathgen). `grep -rn PathgenStepper src tools crrcsim include`. If minisim-only → it is deleted in US1; else retained.
- [X] T005 [P] Identify the roll-authority aero coefficient name in `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.{h,cpp}` (the aileron→roll-moment derivative, analog of `Cm_de`) — needed for US4 roll-effectiveness.

**Checkpoint**: file decisions resolved; stories can proceed.

---

## Phase 3: User Story 1 — Minisim teardown (P1) 🎯 first / force-multiplier

**Goal**: crrcsim is the sole worker path; one worker, not two.
**Independent Test**: `grep -rn minisim CMakeLists.txt tests/ src/ tools/ include/` returns no live refs; build green; a crrcsim eval reproduces pre-removal per-scenario fitness on a fixed seed.

- [X] T006 [US1] Delete `tools/minisim.cc` entirely.
- [X] T007 [US1] Remove the minisim build target from `CMakeLists.txt` (the `add_executable(minisim ...)` @ ~109 and `target_link_libraries(minisim ...)` @ ~117).
- [X] T008 [US1] Rename config field `minisimProgram`→`workerProgram` and `minisimPortOverride`→`workerPortOverride` in `include/autoc/util/config.h`; update the parser in `src/util/config.cc:54-55` (key `MinisimProgram`→`WorkerProgram`, `MinisimPortOverride`→`WorkerPortOverride`). **No back-compat alias** (Constitution III).
- [X] T009 [P] [US1] Rename `MinisimProgram`/`MinisimPortOverride` keys → `WorkerProgram`/`WorkerPortOverride` in `autoc.ini`, `autoc-tracker.ini`, `autoc-eval.ini`, `autoc-eval-tracker.ini`, `autoc-eval-visual.ini` (keep value `./scripts/crrcsim.sh`); delete the stale "minisim retiring" comment in `autoc-tracker.ini:10`.
- [X] T010 [US1] Per T004: if `PathgenStepper` was minisim-only, delete `include/autoc/eval/pathgen_stepper.{h,cc}` + remove from `CMakeLists.txt` and any test; otherwise retain and skip.
- [X] T011 [P] [US1] Update comment-only minisim references to be worker-agnostic: `include/autoc/util/threadpool.h:59,90`, `tests/rpc_transport_tests.cc:99`, `tests/tracker_stepper_init_tests.cc:12-14`, `README.md:10,62`, `docs/*.md`, `.specify/memory/constitution.md` ("minisim or crrcsim FDM" → "crrcsim FDM").
- [X] T012 [US1] Update any test referencing the renamed config field or removed `PathgenStepper`; ensure all build + pass.
- [ ] T013 [US1] (operator-run) Build green (`scripts/rebuild.sh`) + crrcsim eval parity: fixed-seed per-scenario fitness identical to pre-removal (FR-003/SC-001). This parity check is the test-of-record for this removal.

**Checkpoint**: minisim gone, crrcsim sole path, build+eval clean.

---

## Phase 4: User Story 2 — Smoothness removal (P1)

**Goal**: zero live smoothness references; codebase honest about what M1 optimizes.
**Independent Test**: smoothness grep returns zero live refs; build green; before/after fixed-seed M1 eval byte-identical (smoothness already bypassed → guaranteed neutral).

- [X] T014 [US2] **FIRST (silent-miss guard)**: refactor `specs/029-no-future-arch/audit_shift_register.py` from hard column offsets (`POS_OFF=44` etc.) to header-name lookup, so removing the `smooth` column can't silently shift its indices.
- [X] T015 [P] [US2] Delete dead/broken `specs/019-improved-crrcsim/sim_response.py`.
- [X] T016 [US2] Strip `SmoothnessMotionMode` enum + `compute_smoothness_factor()` from `include/autoc/eval/derived_features.h` (retain the 032 span/tilt features per T003).
- [X] T017 [US2] Remove the `smoothness_factor` parameter from `applyStreak` (`include/autoc/eval/fitness_computer.h:53-69`, `src/eval/fitness_computer.cc:57-84`) and the commented smoothness block in `src/eval/fitness_decomposition.cc:174-181`.
- [X] T018 [P] [US2] Remove smoothness state/compute from steppers: `include/autoc/eval/tracker_stepper.h` + `src/eval/tracker_stepper.cc` (ctor params, `prev_out_*`, `initScenario` reset, `stepOnce` compute); and `pathgen_stepper.{h,cc}` if retained per T010.
- [X] T019 [US2] Remove smoothness from `include/autoc/rpc/protocol.h`: `WorkerInit` fields+serialize (171-179, 203) AND the **duplicate** `ScenarioMetadata` fields+serialize+reset (371-378, 399, 423-424). NO version revision.
- [X] T020 [P] [US2] Remove the crrcsim smoothness mirror: `crrcsim/.../inputdev_autoc.{h,cpp}` pathgen mirror (members 138-145, per-tick 1064-1093, reset 583) and `crrcsim/.../crrcsim_tracker_helper.{h,cpp}` tracker mirror (members 105-115, capture 58-62, per-tick 195-226).
- [X] T021 [P] [US2] Remove `smoothnessFactor_` from `include/autoc/eval/aircraft_state.h` (member 492-494, getter/setter 323-332, serialize 582-587).
- [X] T022 [US2] Remove the `smooth` column from BOTH data.dat writers in `src/autoc.cc` (header label @ ~790 + ~976, `% 7.4f` format token @ ~841-842, value source ~770-772, dmp store ~1030). NOTE (verified): data.dat is the ONLY smoothness output site — there is no `smoothness=` gen-log/`data.stc` emit, so no other writer needs touching.
- [X] T023 [P] [US2] Remove `[Smoothness]` sections from all `.ini` files; remove `smoothnessPenaltyFloor`/`smoothnessMotionMode` from `include/autoc/util/config.h:159-177` + parse in `src/util/config.cc:170-191`.
- [X] T024 [US2] DELETE smoothness tests: `tests/derived_features_tests.cc:163-290` (8 `SmoothnessFactor.*`) and `tests/fitness_computer_tests.cc:304-396` (6 `FitnessComputer033Smoothness.*`); UPDATE `tests/tracker_stepper_init_tests.cc:104-110` to drop smoothness ctor args.
- [X] T025 [P] [US2] Update stale doc `specs/024-sim-real-fidelity/contracts/sim_data_dat_contract.md` (remove smooth column, mark superseded if simpler). (The 027–033 `plot_evolution_progress.py` `smoothness=` regexes are harmless optional-match back-compat — leave; out of scope per FR-008 note.)
- [X] T026 [US2] (operator-run) Build green + before/after fixed-seed M1 eval byte-identical vs T002 baseline (SC-002); confirm renderer reads a fresh dmp (FR-009; stale dmps orphaned by reset, no version ceremony). This byte-identical check is the test-of-record for this removal.

**Checkpoint**: smoothness fully gone, behavior-neutral, schema clean for US4.

---

## Phase 5: User Story 3 — Cross-cutting tech-debt fold-ins (P1)

**Goal**: config auto-print, right-sized seeds, **per-(path,wind) variation table**, S3 disambiguation, lighter return path. (FR-013/FR-014 dropped — already satisfied. **5 live items.**)
**Independent Test**: banner prints every key; seed paste-back round-trips with determinism intact; per-(path,wind) variations distinct (or no-gap proven); tracker runs emit `tracker-` prefix; non-elite evals return score-only with unchanged fitness.

- [X] T027 [US3] Introduce an `AUTOC_CONFIG_FIELDS(X)` X-macro single-source list and refactor `include/autoc/util/config.h` (decl), `src/util/config.cc` (parse), and the startup print block `src/autoc.cc:1937-1990` to generate from it (FR-010). Eliminates the three-place edit.
- [X] T028 [P] [US3] Test: a bake's startup banner prints every active config key (including future craft knobs) — `tests/` config-dump assertion (SC-006).
- [X] T029 [US3] Right-size the seed cascade types in `src/util/config.cc:67` (`int seed`), `src/autoc.cc:1890-1910` (`int`→`long`→`uint64_t`), and `include/autoc/util/scenario_prng.h:78-88` (uint32_t Park-Miller state); guard the `seed=-1` time-sentinel paste-back/overflow edge (FR-011). Must NOT alter the PRNG sequence for in-range seeds.
- [X] T030 [P] [US3] Test: a logged master seed pasted back reproduces the run (round-trip); the bit-exact M1→M1 replay gate still passes (SC-005).
- [X] T031 [US3] Resolve FR-012 (per-(path,wind) variation table): audit `gScenarioVariations` indexing in `src/autoc.cc:197-360` — the vector is sized "paths × windScenarioCount" (`:230`) but a `:247` comment references a `[windIdx]` lookup. Determine whether generation + lookup is genuinely per-(path,wind) or collapses to per-wind; **fix to per-(path,wind) if collapsed, else close as "no real gap" with an explicit code comment + the test below**.
- [X] T032 [P] [US3] Test: two scenarios sharing a wind index but differing in path index receive distinct variation draws (per-(path,wind)) — OR encode the verified no-gap finding in the test name/assertion (FR-012 evidence).
- [X] T033 [US3] Prepend run-id prefix by mode at the call site (`src/autoc.cc` ~2143, where `cfg.mode` is known): `tracker-` when `cfg.mode=="tracker"`, else `autoc-`. Keep the bare timestamp in `generate_iso8601_timestamp()` (don't bury mode there). (FR-015)
- [ ] T034 [P] [US3] Test: tracker-mode run-id carries the `tracker-` prefix; pathgen carries `autoc-`.
- [~] T035 [US3] DEFERRED → project_fitness_to_worker_backlog (requires fitness-to-worker refactor, not a gate; memory-opt only). Original: gate non-elite evals to a score-only return: in `crrcsim/.../inputdev_autoc.cpp:920-1000`, suppress per-tick `aircraftStateList` (and camera/target lists) population unless `evalData.isEliteReeval`; verify `src/eval/fitness_decomposition.cc:29-100` `computeScenarioScores` only needs the score-level data. (FR-016)
- [~] T036 [US3] DEFERRED with T035. Original: test non-elite eval returns score-only (no per-tick state); elite-reeval still returns full `EvalResults`; aggregated fitness identical to pre-change for a fixed seed.

**Checkpoint**: 5 fold-ins landed; config recoverable from log; variation table resolved; lighter wire.

---

## Phase 6: User Story 4 — Craft variations (P1)

**Goal**: deterministic per-scenario airframe diversity (CG, drag, trim, thrust-scale, pitch-eff, roll-eff) on the crrcsim chase craft in both modes; **ramped via shared `applyVariationScale()`** (eval replays saved `genome.variation_scale`); no-op at σ=0; craft seed in `ScenarioMetadata`. Decision 2026-05-31: ramping rather than non-ramping (consistency with wind/entry, curriculum benefit, zero new wiring — eval replay already plumbed via `gEvalVariationScaleOverride`).
**Independent Test**: same `scenarioSeed` → bit-identical *draw* (full-magnitude); σ=0 → identical to nominal; craft seed round-trips dmp; eval mode replays saved `genome.variation_scale` exactly (not 100%, not recomputed); replay gate intact.

- [X] T037 [US4] Add craft fields to `ScenarioMetadata` (`include/autoc/rpc/scenario_metadata.h`): `craftCGDelta, craftDragDelta, craftTrimDelta, craftThrustScale, craftPitchEffDelta, craftRollEffDelta` (`gp_scalar`) + `uint32_t craftSeed`; append to `serialize()`; NO version revision. Documented insertion point for a future `cameraSeed` (FR-020). (Unit doc fix 2026-05-31: craftCGDelta is dimensionless CRRCSim MAC units, not meters — hb1_streamer nominal CG_arm = 0.28.)
- [X] T038 [US4] Add craft σ knobs (`craftCGSigma, craftDragSigma, craftTrimSigma, craftThrustSigma, craftPitchEffSigma, craftRollEffSigma`) to the `AUTOC_CONFIG_FIELDS` X-macro and a `[Craft]` block in `autoc.ini`/`autoc-tracker.ini`/eval inis. Decision 2026-05-31: code defaults are SENSIBLE non-zero values (mirror entry/wind pattern: ConeSigma=30°, RollSigma=22.5° etc.) so a missing-from-ini key still produces meaningful diversity. Added `EnableCraftVariations` macro-disable knob (parallels EnableEntry/Wind/RabbitSpeed) — when 0, the craft-class PRNG still advances (draw-and-discard) but deltas are zeroed before write. ContractConfig.ConfigFieldsMacroCount expectation: 80 → 86 (6 sigmas) → 87 (enable flag).
- [X] T039 [US4] Create `include/autoc/eval/craft_variation.h` (header-only): `CraftSigmas` + `CraftDeltas` structs + `generateCraftFromClassPRNG(craftPRNG, sigmas) → CraftDeltas` (fractional-σ Gaussian draws at full magnitude, deterministic). `gp_scalar` throughout. Wired into `prefetchAllVariations` in `src/autoc.cc` (draws after wind, writes to `gScenarioVariations[K].craftDeltas` + `craftSeed`); per-eval meta construction copies the 6 deltas + `craftSeed` into `ScenarioMetadata`. Extended `applyVariationScale()` in `include/autoc/eval/scenario_meta_apply.h` to scale the craft fields the same way it scales entry/wind (additive deltas `*= scale`; `craftThrustScale` interpolated toward 1.0 like `entrySpeedFactor`).
- [X] T039b [US4] Extend the startup "Pre-fetched Scenario Variations" table (`logPrefetchedVariations` in `src/autoc.cc`) with 6 craft draw columns + craftSeed when `EnableCraftVariations=1 AND any σ > 0`. Gated so the common no-op case leaves the table byte-identical to pre-US4. Values are DRAWN (full magnitude); operator can compute applied = drawn × rampSc using the per-gen rampSc already emitted in data.dat/log lines. Pre-T052 telemetry decision: no new columns in data.dat or data.stc (both going away in T052–T055 + 035 prereqs).
- [X] T040 [P] [US4] Add `Global::craftCGDelta/DragDelta/TrimDelta/ThrustScale/PitchEffDelta/RollEffDelta` carriers to `crrcsim/src/global.{h,cpp}`, mirroring the existing `Global::entry*Offset` pattern. Defaults at semantic nominal (0.0 / 1.0 thrust) so a worker without craft data fed-through is byte-identical to pre-US4.
- [X] T041 [US4] Populate the `Global::craft*` carriers at the per-scenario reset hook in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (right after the entry/wind block). Values come from `meta.craft*` AFTER worker-side `applyVariationScale()` has scaled them — same dataflow as `meta.entry*Offset`.
- [X] T042 [US4] Apply craft params in the FDM (`crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.{h,cpp}`): added `nominal_X` cache members for Cm_0, Cm_de, CL_de, CD_prof, CG_arm, Cl_da set at LOAD time (after XML parse); `initAirplaneState` applies `CG_arm = nominal_CG_arm + craftCGDelta`, `CD_prof = nominal_CD_prof * (1+craftDragDelta)`, `Cm_0 = nominal_Cm_0 + craftTrimDelta`, pitch authority `Cm_de/CL_de *= (1+craftPitchEffDelta)`, roll authority `Cl_da *= (1+craftRollEffDelta)` (T005 coeff confirmed = Cl_da, the aileron→roll-moment derivative); `engine()` scales propeller v_F + v_M by `Global::craftThrustScale` after `power->step()`. Nominal cache means a re-init with all craft zeroed falls back exactly to pre-US4 LOAD-only behavior.
- [X] T043 [P] [US4] `tests/craft_variation_tests.cc` (8 tests covering contracts/craft-variation-contract.md invariants): (a) σ=0 no-op → all-zero deltas + 1.0 thrust scale; (b) same `scenarioSeed` → identical drawn deltas bit-for-bit (via direct `ClassPRNG` and via the `deriveClassSubSeeds` cascade); (c) ramp invariants: scale=0 collapses to nominal, scale=1 leaves draw unchanged, scale=0.5 halves additives + thrust toward 1.0, draw is independent of any scale; (d) `craftSeed` + all 6 craft deltas round-trip through cereal in `ScenarioMetadata` without loss. Wired into `CMakeLists.txt` (target + `add_test` + `run_autoc_tests` dependency + `--gtest_brief` invocation).
- [X] T044 [US4] (operator-run) Build green + bit-exact M1→M1 replay gate passes after the craft-seed cascade (SC-005). **Signed off 2026-05-31**: live grab of an in-progress NN through the updated `autoc-eval.ini` reproduced training fitness — autoc training, nnextractor, autoc-eval, renderer all coherent. M1→M1 replay survives the US4 cereal-schema growth (ScenarioMetadata gained 6 craft deltas + craftSeed + the EnableCraftVariations master-disable knob) under the no-cereal-versioning policy. Type-domain audit grep deferred to T048.

**Checkpoint**: craft variations deterministic, no-op at zero, replay intact — implementation code-complete (US1–US4).

---

## Phase 7: User Story 5 — Craft-variation confirmation experiment (P1, operator-run acceptance)

**Reframed 2026-05-31:** 034 will NOT produce a flight-tested controller. With the M1 basin-lottery worse than originally thought (see [stuck-basin-bisect.md](stuck-basin-bisect.md) + [project_m1_basin_lottery_actual_rate](../../.claude/projects/-home-gmcnutt-autoc/memory/project_m1_basin_lottery_actual_rate.md)) and the smoothness rework deferred to a future feature, 034 closes when craft variations are **demonstrably working in the proper direction** — controller bias histograms correlate with the per-scenario craft draws. Flight test moves to a future feature once we have good runs to fly. Next sequence: 034 wrap → smoothness rework (likely new feature dir) → 035 lexicase-energy → 036 demetic islands.

**Goal**: prove craft variations evolved a controller whose per-scenario behavior shows traceable compensation for the per-scenario airframe — biases in pitch / roll / throttle correlate with the relevant craft draws.

**Isolation design**: turn off every variation class EXCEPT craft (no wind, no entry-pose, no rabbit-speed), pick ONE path, let `WindScenarios` define how many distinct airframes the population sees per individual. With everything but craft pinned, the only scenario-to-scenario diversity is the airframe — clean signal for the histogram analysis.

**Independent Test**: regression of per-scenario mean(out_pitch) vs scenario.craftTrimDelta should show negative slope (controller pushes pitch UP to compensate for nose-down trim); mean(out_throttle) vs scenario.craftThrustScale should show negative slope (more thrust → less throttle command); mean(|out_roll|) magnitudes should correlate with |craftRollEffDelta|. Quantitative thresholds are soft (the GA may compensate partially or via different control axes than the simple physical intuition predicts) — what matters is the direction of the bias.

- [ ] T045 [US5] (operator-run) Build `autoc-craft-only.ini` from `autoc.ini` with `EnableWindVariations=0`, `EnableEntryVariations=0`, `EnableRabbitSpeedVariations=0`, `EnableCraftVariations=1`, single-path generator (`PathGeneratorMethod=longSequential` or similar), `WindScenarios` chosen to give enough airframe samples (suggest 36+ for statistical reach on the regression). Keep the σ values at autoc.ini defaults (CG=0.02, drag/thrust/pitch-eff/roll-eff=0.05, trim=0.02 rad).
- [ ] T046 [US5] (operator-run) Short M1 bake on the craft-only ini, gens chosen by convergence (likely fewer than full 800 needed since the basin is narrower with diversity-locked-to-craft only). Capture data.dat + dmps.
- [ ] T047 [US5] (operator-run) Per-scenario control-bias analysis: extend `specs/034-energy-objective-cleanup/per_axis_aggressiveness.py` (or new script) to (a) read per-scenario craft draws from the startup-table log line, (b) compute per-scenario mean(out_pt), mean(out_rl), mean(out_th) from data.dat at the converged gen, (c) regress / scatter-plot each (control axis × craft axis) pair, (d) report the sign + rough magnitude of each correlation. **Success**: at least the trim → pitch-bias regression has the physically-expected sign with non-trivial R². At least one other correlation (thrust, pitch-eff, or roll-eff) also reads in the expected direction.

**Checkpoint**: 034 closes as "craft variations land in code AND show the proper-direction control bias in a controlled experiment." Future feature picks up smoothness rework. Flight test deferred until post-smoothness runs are flyable.

---

## Phase 8: Polish & Cross-Cutting

- [ ] T048 [P] Run the Constitution VI type-domain audit grep on touched paths: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v '// raw-ok:'` — annotate or convert each hit.
- [ ] T049 [P] Annotate the BACKLOG.md entries pulled into 034 (minisim retire, smoothness, the 5 fold-ins, craft-variation forward-loop) with `→ 034` / DONE; note FR-013/FR-014 were already-satisfied.
- [ ] T050 [P] Update `CLAUDE.md` Recent Changes for 034 (already auto-updated; verify accuracy).
- [ ] T051 Run `specs/034-energy-objective-cleanup/quickstart.md` verification gates end-to-end (grep gates, no-op check) before the US5 bakes.

### Data.stc retirement (single-source telemetry)

**Goal**: kill the `data.stc` breadcrumb file. Per-gen telemetry currently lives in two places (`data.stc` for offline plot scripts, stdout `.log` for live observation); fold the six gen-tagged lines into the logger so the `.log` is the sole source. No C++ consumer reads `data.stc` (renderer doesn't either — verified); only the Python plot scripts use it. Logger adds timestamp+level prefix which actually *helps* (wall-clock correlation), and existing plot-script regexes use `re.search` so the prefix doesn't break parsing.

**Why land before US5**: the US5 bakes produce the artifacts that the post-bake outcome doc parses; the cleanup must be live before then to avoid retro-fitting outcome scripts.

- [ ] T052 [Cleanup] Migrate the six `bout << "#…"` write sites in `src/autoc.cc` to `*logger.info() << "#…"` (keep the `#NNEval` / `#NNGen` / `#GenCrash` / `#GenDiag` / `#GenSimStats` / `#SimRuns` marker tokens unchanged for grep stability): lines 1359, 1621, 1653, 1693, 2040, 2106. Delete the `bout` ofstream declaration (`src/autoc.cc:379`), the `strStatFile` open/close (`:2002-2007`), and the `bout.flush()` calls (`:1712, 2046, 2107`). Constitution III: clean cut, no back-compat dual-write.
- [ ] T053 [P] [Cleanup] Update `specs/034-energy-objective-cleanup/plot_evolution_progress.py` `--focus` / `--compare` defaults + docstrings to point at `.log` files instead of `data.stc`; remove the .log NN_ELITE fallback path added 2026-05-30 (no longer needed once `#NNGen` lives in the .log). Same updates to `per_axis_aggressiveness.py` + `plot_per_axis_time_series.py` only where they reference .stc (they primarily read `data.dat`, so likely no-op besides docstrings).
- [ ] T054 [P] [Cleanup] Strip `.stc` from: `include/autoc/eval/eval_logger.h:7` header comment, the eval-pipeline `eval-data.stc` paste-back baseline reference in `specs/034-energy-objective-cleanup/quickstart.md:33`, and `.gitignore` `/*.stc` rule (now dead — no .stc files will be produced).
- [ ] T055 [Cleanup] (operator-run) Run a short bake (≥10 gens) and confirm: (a) no `data.stc` / `eval-data.stc` produced in workspace; (b) the 6 marker lines appear in `logs/autoc-<run>.log` at expected cadence; (c) `plot_evolution_progress.py` against the new .log produces the same panels as the .stc-based version did.

**Checkpoint**: single source of telemetry, no orphaned breadcrumb file, plot scripts read the same .log as live tail.

---

## Dependencies & Execution Order

### Phase / story order

- **Setup (P1)** → **Foundational (P2, verifications)** → **US1** → **US2** → **US3** / **US4** → **US5** → **Polish**.
- **US1 is first and blocking**: the single-path collapse is a precondition for US2's smoothness-mirror removal and US4's craft-apply (both edit the now-sole crrcsim worker).
- **US2 before US4**: clean the cereal structs (remove smoothness) before appending craft fields, so the schema touch is one coherent pass.
- **US3 is largely independent of US2/US4** but T027 (X-macro) is a prerequisite for T038 (craft knobs use the X-macro) — do T027 before T038. Otherwise US3 can interleave with US4.
- **US5 (operator-run)** depends on US1–US4 code-complete.

### Within stories

- Tests written/with implementation per Constitution I; the σ=0 no-op + determinism tests (T043) gate US4 acceptance.
- US2 T014 (audit script refactor) MUST precede T022 (column removal) — the silent-miss guard.

### Parallel opportunities

- Foundational T003/T004/T005 all [P].
- US1: T009, T011 [P] (different files) after the core deletes.
- US2: T015, T018, T020, T021, T023, T025 [P] (distinct files); T014 first, T022/T024 touch autoc.cc/tests serially.
- US3: T028/T030/T032/T034 tests [P]; the five fold-ins (T027/T029/T031/T033/T035) — T031 (variation-table) is independent of the others; T027/T029/T033/T035 touch overlapping files (autoc.cc, config) so mostly serial.
- US4: T040 [P], T043 [P]; T037→T039 and T041→T042 are serial chains.

---

## Implementation Strategy

### MVP path (cleanup-first)

1. Setup + Foundational verifications.
2. **US1 (minisim)** → build+eval parity → the single-path foundation.
3. **US2 (smoothness)** → byte-identical eval → honest schema.
4. **US3 (fold-ins)** + **US4 (craft variations)** → the feature payload.
5. **STOP & VALIDATE**: quickstart gates green, σ=0 no-op, replay gate intact → implementation code-complete.
6. **US5 (operator)**: bakes + flight test → feature closure.

### Notes

- [P] = different files, no incomplete-task dependency.
- No dmp/transport version revision anywhere (M2-era policy) — smoothness fields deleted, craft fields appended, that's all.
- Commit per task or logical group; keep build green (Constitution II) at every checkpoint.
- FR-013 / FR-014 intentionally absent — verified already satisfied in-tree (D2). US3 = **5** live items (config X-macro, seed-width, variation-table, S3 prefix, EvalResults trim).

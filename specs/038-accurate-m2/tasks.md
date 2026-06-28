# Tasks: 038 Accurate M2 — Architecture for Tracking Depth

**Input**: Design documents from `/specs/038-accurate-m2/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Execution model (operator 2026-06-27)**: tasks are a series of **gated milestones with distinct
acceptance**, NOT a linear build list. Phase 0 (P0-A…P0-G) is deterministic prework. US1/US2/US3 are
**empirical ablations** — each is a bake → judge → keep/escape loop. **Escape to `specs/BACKLOG.md` or a
future feature is a first-class outcome at every 🚦 gate**: a lever that doesn't clear its gate closes as
"ruled out / deferred" and does NOT block the feature. A real decision node (Phase 6) combines winners.

**Tests**: included where load-bearing (P0-A is a determinism-test deliverable; format changes regen
fixtures; US4 needs a fitness/selection test). Ablation bakes are research spikes (Constitution I exemption)
until the winner is mainlined in Phase 6, which carries the test regen.

**Bakes are operator-driven** (Constitution IX): launch via `scripts/train.sh` (never `run_in_background`);
the operator runs the eval-vs-training bitwise gate. Tasks marked **[OP]** are operator-gated actions.

## Format: `[ID] [P?] [Story?] Description`
- **[P]**: parallelizable (different files, no incomplete-dep)
- **[Story]**: US1/US2/US3/US4 (Phase 0 + Polish have no story label)
- **[OP]**: operator-gated (bake launch / regression gate / pin)

---

## Phase 1: Setup — Phase 0 deterministic prework (independent, mostly parallel)

**Purpose**: front-matter tech-debt that de-risks the bakes; none format-breaking except P0-D (Phase 2).

- [ ] T001 P0-A: run the PRNG determinism contract suite and write a verdict (clear / bug-found-and-fixed) into `specs/038-accurate-m2/outcome.md`: `cd build && make run_autoc_tests && ./tests/run_autoc_tests --gtest_filter='*scenario_prng*:*eval_mode_replay*'` (validates the 033 cascade in `include/autoc/util/scenario_prng.h` + `tests/scenario_prng_tests.cc`, `tests/eval_mode_replay_tests.cc`)
- [ ] T002 [P] P0-F: revert `FitStreakThreshold` 0.3→0.5 in `autoc-tracker.ini:258`, `autoc-eval-tracker.ini:215`, `autoc-eval-tracker-visual.ini:219` (M1 inis already 0.5)
- [ ] T003 [P] P0-E: remove the `svTau` dead path — `specs/037-20hz-control-loop/verify_037_metrics.py:19,23,35` and any residual `svTau`/servoTau-tau references in `include/autoc/eval/craft_variation.h`
- [ ] T004 [P] P0-E: time-denominate rate-dependent reports — surface streak / avgMaxStreak in seconds (or pctInStreak) consistently across `src/analytics/` (confirm `--tick-sec` in `scripts/generate_pngs.sh:66`, `src/analytics/m2_compare.py:52`)
- [ ] T005 [P] P0-E: run the type-domain grep audit on the 037-touched paths and annotate/convert hits: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -E '// raw-ok:'`
- [ ] T006 [P] P0-C: finalize the standardized report wrapper `scripts/generate_pngs.sh m2 <log>` and add `src/analytics/pyproject.toml` alongside `src/analytics/requirements.txt` (numpy, matplotlib) — maintained analytics home

---

## Phase 2: Foundational — the ONE dmp break + baseline re-bake (BLOCKING)

**Purpose**: the single P0-D contract break (operator-sanctioned, no cereal version bump, fail-loud read) +
the fresh source/baseline every ablation branches from. **⚠️ No ablation (Phase 3+) may begin until the
baseline (T013) exists.** crrcsim submodule changes land pointer-bump-first (submodule merge order).

- [ ] T007 P0-D-1: replace the truncating stamp in `crrcsim/src/SimStateHandler.cpp:392` (`getSimulationTimeSinceReset`) with a step-count-derived stamp (exact 50 ms gaps), then revert the source-spacing check to strict single-gap in `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp:76` and `src/eval/tracker_stepper.cc`
- [ ] T008 P0-D-3: wire crrcsim FDM wind into the record path — call `AircraftState::setWindVelocity()` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (~906, currently never called → zeros)
- [ ] T009 P0-D-2: add the self-describing fitness/cadence config block to `EvalResults` in `include/autoc/rpc/protocol.h:365` (fit cone params, `SIM_TIME_STEP_MSEC`/`kCadenceTickScale`, crash-penalty knobs) — no `CEREAL_CLASS_VERSION` bump; readers fail loud on old dmps
- [ ] T010 P0-B: flip `tools/renderer.cc:2933` and `tools/dmp_dump.cc:406` to prefer the dmp-recorded config block over `ConfigManager::getConfig()` (only remaining ini dep: S3 bucket/profile)
- [ ] T011 regenerate affected test fixtures and confirm green: `tests/scenario_prng_tests`, `tests/eval_mode_replay_tests`, `tests/tick_rescale_tests` (per [contracts/nn-and-dmp-format.md](contracts/nn-and-dmp-format.md))
- [ ] T012 [OP] pre-run build gate + eval-vs-training bitwise `ScenarioScore` gate: `bash scripts/rebuild-perf.sh` (single-threaded FP-deterministic) — must pass before any bake
- [ ] T013 [OP] P0-G: re-bake a fresh M1 source `scripts/train.sh autoc.ini logs/autoc-038-t1-m1-rebake.log`, pin the winner in `autoc-m1`, record the S3 prefix in `outcome.md`; then re-bake the M2 baseline `scripts/train.sh autoc-tracker.ini logs/autoc-038-t2-m2-baseline.log`

**🚦 Checkpoint**: post-P0-D baseline (M1 source + M2 baseline) exists → ablations may begin in parallel.

---

## Phase 3: US1 — Deeper / non-uniform history (Priority: parallel ablation, M1-FIRST GATE) 🎯

**Goal**: richer temporal context (deeper/non-uniform history) lifts a reward-invariant ceiling — proven on
**M1 first** (no FOV confound), then inherited by M2.
**Independent Test**: M1 bake with the alt layout vs t10-matched layout at matched seed moves close-track %
or median error; determinism + bitwise replay preserved.

- [ ] T014 [US1] implement the deeper layout in `include/autoc/nn/nn_inputs.h:48` (start `{1600,800,400,200,100,50,0}`, 7 integral slots at 50 ms) + bump `kNNHistoryLayoutVersion`→4 (`nn_inputs.h:60`); update `kTrackerInputMeta`/`kPathgenInputMeta` if slot count changes (per [data-model.md](data-model.md) §1)
- [ ] T015 [US1] propagate derived sizes: `HISTORY_SIZE` (`include/autoc/eval/aircraft_state.h:391`), `TrackerObservationRing::kDepth` (`include/autoc/nn/evaluator.h:143`), `HIST_PAST[]` (`src/nn/evaluator.cc:314`); regen `tests/nn_layout_tests.cc`, `tests/nn_evaluator_tests.cc`, `tests/tick_rescale_tests.cc`; regen xiao codegen per [contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md)
- [ ] T016 [US1][OP] pre-run build gate + bitwise gate (`rebuild-perf.sh`)
- [ ] T017 [US1][OP] bake M1 ablation `scripts/train.sh autoc.ini logs/autoc-038-t3-m1-hist1p6s.log`; judge with `scripts/generate_pngs.sh m1 <log> --compare baseline:<rebake-log>` against SC-001 (M1 ceiling, fixed-eval comparator)

**🚦 US1 gate**: if the deeper layout moves an M1 ceiling → **T018 (inherit to M2)**. If not → **escape**:
record the negative result in `outcome.md`, file the alternative spacing/depth options to `specs/BACKLOG.md`,
close US1 as ruled-out. Does NOT block US2/US3.

- [ ] T018 [US1][OP] (gate-pass only) inherit the layout to M2 and bake `scripts/train.sh autoc-tracker.ini logs/autoc-038-t4-m2-hist.log`; judge vs SC-001 (M2)

---

## Phase 4: US2 — Two-timescale recurrence (Priority: parallel ablation, M2-direct)

**Goal**: a structural leaky-slow recurrent channel adds trajectory memory without widening W_hh.
**Independent Test**: M2 bake with the slow channel vs single-timescale baseline at matched seed shows
better tracking and/or healthier `rnn_capacity` eff-rank at equal-or-fewer recurrent params.

- [ ] T019 [US2] implement the slow channel in `include/autoc/nn/topology.h` (both pathgen + tracker sections): start hidden-2 16-wide fast + 8-wide fixed-leak slow (α≈0.9); update `NN_TOPOLOGY`/`TRACKER_NN_TOPOLOGY`, `NN_RECURRENT`, `*_WEIGHT_COUNT`, `*_HIDDEN_STATE_COUNT`, topology string (per [data-model.md](data-model.md) §2, [research.md](research.md) §2)
- [ ] T020 [US2] implement the leaky-slow update + ensure `NNControllerBackend::reset()` zeros the slow channel (`src/nn/evaluator.cc` recurrent path, `include/autoc/nn/evaluator.h`); regen `tests/nn_layout_tests.cc`, `tests/nn_serialization_tests.cc`, xiao codegen
- [ ] T021 [US2][OP] pre-run build gate + bitwise gate
- [ ] T022 [US2][OP] bake M2 ablation `scripts/train.sh autoc-tracker.ini logs/autoc-038-t5-m2-slowchan.log`; judge vs baseline with `generate_pngs.sh m2 … --compare` (SC-001 + `rnn_capacity` eff-rank)

**🚦 US2 gate**: clears if it moves a ceiling and/or eff-rank uses the new modes. Else **escape**: record,
file evolved-time-constant / second-recurrent-layer alternatives to BACKLOG, close US2 ruled-out.

---

## Phase 5: US3 — Auxiliary target-predictor head (Priority: parallel ablation, M2-direct, the pivot)

**Goal**: an aux head predicting next-tick optical state, scored by a SEPARATE lexicase axis, gives the
network an explicit motion model.
**Independent Test**: M2 bake with predictor vs control-only at matched seed; predicted optical state matches
realized within a target error AND tracking depth/reacquire improves; determinism + replay preserved.

- [ ] T023 [US3] grow output head 3→7 in `include/autoc/nn/topology.h` (`*_OUTPUT_COUNT`, `*_WEIGHT_COUNT`); resize `nnOutputs_[]` + deserialize output-count check in `include/autoc/eval/aircraft_state.h:580`; output buffers in `include/autoc/nn/evaluator.h` + `src/nn/evaluator.cc` (outputs[0..2]=control, [3..6]=predicted optical; aux not actuated)
- [ ] T024 [US3] add `prediction_score` (`gp_fitness`, negated) to `ScenarioScore` in `include/autoc/eval/fitness_decomposition.h:71`; accumulate per-tick aux-vs-realized-next-tick optical error in `src/eval/fitness_decomposition.cc`; add it as a SEPARATE per-scenario axis in `src/eval/selection.cc` (NOT scalar-composited — per [contracts/lexicase-axis-and-reward.md](contracts/lexicase-axis-and-reward.md))
- [ ] T025 [US3] add `EnablePredictorHead` knob to `AUTOC_CONFIG_FIELDS` in `include/autoc/util/config.h` (no in-class default per VII); verify `applyCrashPenalty` (`src/autoc.cc:181`) leaves `prediction_score` untouched; regen `tests/nn_*`, xiao codegen; add a prediction-axis selection unit test
- [ ] T026 [US3][OP] pre-run build gate + bitwise gate
- [ ] T027 [US3][OP] bake M2 ablation `scripts/train.sh autoc-tracker.ini logs/autoc-038-t6-m2-predhead.log`; judge vs SC-001 (depth/reacquire) + SC-002 (prediction accuracy)

**🚦 US3 gate**: clears if predictor develops AND lifts a ceiling. Else **escape**: record, file
pretrain-then-evolve / longer-horizon / recurrent-head alternatives to BACKLOG, close US3 ruled-out.

---

## Phase 6: Decision node — judge + combine winners (BLOCKING for US4 final)

**Purpose**: the branch point that can't be pre-sequenced — decide which ablation winners combine.

- [ ] T028 [OP] judge US1/US2/US3 on the SC-001 ceilings (fixed-eval comparator, not raw late-run fitness per `project_late_run_fitness_interpretation`); record verdicts + every escape in `outcome.md`
- [ ] T029 combine the winning structural levers into one architecture (merge the kept `topology.h`/`nn_inputs.h` changes); regenerate ALL NN test fixtures + xiao codegen; type-domain grep audit on the merged diff
- [ ] T030 [OP] pre-run build gate + bitwise gate, then bake the combined M2 `scripts/train.sh autoc-tracker.ini logs/autoc-038-t7-m2-combined.log`

**🚦 Combine gate**: if NO ablation cleared its gate, the architecture thesis is unsupported at this depth →
record the finding, route the depth problem to a future feature, and 038 still delivers Phase 0 + (if it
landed) US4. Skip to Phase 8.

---

## Phase 7: US4 — Visibility-maintenance reward (Priority: P2, FOV-specific, reward-only)

**Goal**: reward keeping the target framed so the chase flies perception-maintaining geometries.
**Independent Test**: M2 bake with `EnableVisibilityReward=1` improves in-FOV fraction / shortens reacquire
without destabilizing selection or regressing the carried crash penalty.
**Note**: reward-only — NOT format-breaking; can iterate without a re-bake (but changes fitness scale → use
fixed-eval comparator). Can run against the combined architecture or the baseline.

- [ ] T031 [US4] add `visibility_score` (`gp_fitness`, negated) to `ScenarioScore` (`include/autoc/eval/fitness_decomposition.h:71`); accumulate per-tick from existing CEP/in-FOV data (`src/eval/fitness_decomposition.cc:196`, continuous in-FOV term × `kCadenceTickScale`); **M1/pathgen leaves it 0** (per [data-model.md](data-model.md) §4)
- [ ] T032 [US4] add `visibility_score` as a SEPARATE lexicase axis in `src/eval/selection.cc`; add `EnableVisibilityReward` + reward-shape params to `AUTOC_CONFIG_FIELDS` (`include/autoc/util/config.h`, no in-class default per VII); verify `applyCrashPenalty` leaves `visibility_score` untouched
- [ ] T033 [US4] add a fitness unit test (visibility term + M1=0 invariant) and a selection-stability check (mixing the new axis); if it destabilizes, promote MAD-ε behind its ini switch (FR-032) — else record the constant-0.5 baseline holds
- [ ] T034 [US4][OP] pre-run build gate + bitwise gate, then bake `scripts/train.sh autoc-tracker.ini logs/autoc-038-t8-m2-vis.log`; judge vs SC-003 (in-FOV/reacquire) + SC-004 (crash penalty not regressed)

**🚦 US4 gate**: clears if it lifts in-FOV/reacquire without destabilizing selection. Else **escape**:
record, file the binary-gate / ramped variant to BACKLOG, close US4 ruled-out.

---

## Phase 8: Polish & cross-cutting

- [ ] T035 [P] final type-domain grep audit across ALL 038-touched `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` (Constitution VI per-milestone close); annotate/convert remaining hits
- [ ] T036 [P] record the xiao firmware NN-contract update (FR-051) in `outcome.md` per [contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md) (input/output/topology/cadence sync points for the deferred tracker port)
- [ ] T037 write `specs/038-accurate-m2/outcome.md`: per-SC verdicts (SC-000…SC-005), the pinned milestone S3 prefix (Constitution VIII), and a consolidated list of all escapes routed to `specs/BACKLOG.md`
- [ ] T038 [OP] confirm SC-000 (Phase 0 done) + SC-005 (bitwise gate green) + that US5 stays deferred in `specs/BACKLOG.md`

---

## Dependencies & execution order

- **Phase 1 (Setup)**: T001–T006 all independent — fully parallel.
- **Phase 2 (Foundational, BLOCKING)**: T007/T008 (crrcsim) ∥ T009 (protocol) → T010 (readers) → T011 (fixtures) → T012 (gate) → T013 (baseline). T013 blocks ALL of Phase 3+.
- **Phases 3/4/5 (ablations)**: independent of each other — US1, US2, US3 run **in parallel** off the T013 baseline (separate bakes; the operator paces concurrency). Each self-gates with escape.
- **Phase 6 (decision)**: depends on whichever ablations completed (T017/T022/T027); T028→T029→T030.
- **Phase 7 (US4)**: reward-only; can run any time after T013 (independent of the ablations), but final bake (T034) is best against the combined architecture (T030) if it exists.
- **Phase 8 (Polish)**: after the milestones it documents.

## Parallel opportunities

- Phase 1: T002–T006 in one batch (distinct files).
- Phase 2: T007/T008/T009 concurrent (crrcsim ∥ protocol), then serialize T010→T013.
- Phases 3–5: the three ablation bakes are the primary parallelism (operator-paced, separate logs/buckets).
- US4 (Phase 7) can overlap the ablations (different files: fitness/selection vs topology).

## Implementation strategy (MVP + increments)

- **MVP = Phase 0 + one cleared ablation gate.** SC-001 ("any one ceiling moves") is satisfied by the first
  ablation that clears — that is the minimal architecture win.
- **Increment 1**: Phase 0 baseline (SC-000) — the clean re-baked, self-describing-dmp foundation; valuable
  even if every ablation escapes.
- **Increment 2**: the first ablation that clears its 🚦 gate → SC-001.
- **Increment 3**: combine winners (Phase 6) + US4 (SC-003) → the deeper, framed M2.
- **Escapes are expected outcomes**, not failures: a ruled-out lever routes to BACKLOG and the feature
  proceeds on the levers that worked.

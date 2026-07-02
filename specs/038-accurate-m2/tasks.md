# Tasks: 038 Accurate M2 — Architecture for Tracking Depth

**Input**: Design documents from `/specs/038-accurate-m2/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Execution model (operator 2026-06-27, refined 2026-06-30)**: tasks are a series of **gated milestones with
distinct acceptance**, NOT a linear build list. Phase 0 (P0-A…P0-G) is deterministic prework. The generic
architecture studies are **empirical ablations** — each a bake → judge → keep/escape loop. **Initial wave =
US1 (deeper history) + US3 (predictor head)**, the "add state" levers; **US2 (two-timescale recurrence,
Phase 4) is deferred to `specs/BACKLOG.md`** as a follow-on lever and is NOT launched in the initial wave
(its tasks stay recorded for unpark). **Escape to `specs/BACKLOG.md` or a future feature is a first-class
outcome at every 🚦 gate**: a lever that doesn't clear its gate closes as "ruled out / deferred" and does
NOT block the feature. A real decision node (Phase 6) combines the winners of the initial wave.

**Tests**: included where load-bearing (P0-A is a determinism-test deliverable; format changes regen
fixtures; US4 needs a fitness/selection test). Ablation bakes are research spikes (Constitution I exemption)
until the winner is mainlined in Phase 6, which carries the test regen.

**Bakes are operator-driven** (Constitution IX): launch via `scripts/train.sh` (never `run_in_background`);
the operator runs the eval-vs-training bitwise gate. Tasks marked **[OP]** are operator-gated actions.

## Format: `[ID] [P?] [Story?] Description`
- **[P]**: parallelizable (different files, no incomplete-dep)
- **[Story]**: US1/US2/US3/US4 (Phase 0 + Polish have no story label)
- **[OP]**: operator-gated (bake launch / regression gate / pin)

> **"Phase" disambiguation** — the word is overloaded across the docs. This file's execution-phases
> (Phase 1 Setup … Phase 8 Polish) map onto the **spec's program-phases** as: tasks Phase 1–2 = spec
> **Phase 0** (prework / the one dmp break); tasks Phase 3–6 = spec **Phase 1** (RNN architecture studies);
> tasks Phase 7 = spec **Phase 2** (M2 prototyping / US4). The plan's "Phase 0/1" are the speckit
> research/design phases (already complete). When a gate or SC cites a "phase," resolve via this key.

---

## Phase 1: Setup — Phase 0 deterministic prework (independent, mostly parallel)

**Purpose**: front-matter tech-debt that de-risks the bakes; none format-breaking except P0-D (Phase 2).

- [X] T001 P0-A: run the PRNG determinism contract suite and write a verdict into `specs/038-accurate-m2/outcome.md` — **CLEAR 2026-07-01**. Pattern: `make -C build run_autoc_tests` (the gtest suites run inline as part of the build — no standalone `tests/run_autoc_tests` binary). All suites PASS incl. `scenario_prng_tests` (cascade D1–D5) + `eval_mode_replay_tests` (cross-process bitwise); validates the 033 cascade in `include/autoc/util/scenario_prng.h`. No bug found.
- [X] T002 [P] P0-F: revert `FitStreakThreshold` 0.3→0.5 in `autoc-tracker.ini:258`, `autoc-eval-tracker.ini:215`, `autoc-eval-tracker-visual.ini:219` (M1 inis already 0.5) — DONE 2026-07-01
- [X] T003 [P] P0-E: `svTau` dead path — **already removed 2026-06-12** (`craft_variation.h:150-151` "servo first-order tau draw removed … draw order shifts … fine cross-build"; `scenario_metadata.h:118`; no residual draw-order placeholder). Only residue is the historical `verify_037_metrics.py` — left untouched per [feedback_historical_scripts_immutable]. No code change needed.
- [X] T004 [P] P0-E: time-denominate rate-dependent reports — **DONE 2026-07-01** (convert-to-seconds). `avgMaxStreak`→seconds in `plot_evolution_progress.py` (new `--tick-sec`, axis relabelled) + `maxLost`→seconds in `mode_progress.py` (new `--tick-sec`, panel "worst blind streak (s)"); `generate_pngs.sh` now passes `--tick-sec "$TICK_SEC"` to both (it already fed `intercept_analysis`). py_compile + `bash -n` green.
- [X] T005 [P] P0-E: type-domain grep audit — **AUDITED 2026-07-01** (~430 unannotated hits tree-wide = the codebase-wide backfill Constitution VI defers to a separate spec; 038 hasn't touched eval/nn code yet → no 038 drift; conversions are determinism-affecting → ride `rebuild-perf.sh`, not blind Phase-1 edits; per-milestone VI audit on 038-touched code runs at T035). `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -E '// raw-ok:'`
- [X] T006 [P] P0-C: **DONE 2026-07-01** — added `src/analytics/pyproject.toml` (PEP 621, deps numpy+matplotlib, requires-python ≥3.11, valid TOML) alongside the existing `requirements.txt`; `generate_pngs.sh m2 <log>` wrapper confirmed functional (`bash -n` green, tick-sec passthrough fixed in T004). Full 11-report S3-fed run is [OP] (needs a real dmp+log).

---

## Phase 2: Foundational — the ONE dmp break + baseline re-bake (BLOCKING)

**Purpose**: the single P0-D contract break (operator-sanctioned, no cereal version bump, fail-loud read) +
the fresh source/baseline every ablation branches from. **⚠️ No ablation (Phase 3+) may begin until the
baseline (T013) exists.** crrcsim submodule changes land pointer-bump-first (submodule merge order).

- [X] T007 P0-D-1: **DONE 2026-07-01** — `getSimulationTimeSinceReset` now `std::llround(sim_steps*dt*1000)` (was truncating → 49/50/51 ms jitter; round snaps to exact 50 ms gaps) + `#include <cmath>` in `crrcsim/src/SimStateHandler.cpp`; strict single-gap source-spacing restored in `crrcsim_tracker_helper.cpp` + `src/eval/tracker_stepper.cc`. ✅ [OP] **SETTLED 2026-07-02** — the basic-m1 (t2) eval bit-matched training (`autoc-basic-m1-eval.ini`, elite extracted from a live gen), i.e. second-order determinism holds with the enriched pipeline; the identical-state/different-simTimeMsec divergence is empirically resolved.
- [X] T008 P0-D-3: **DONE 2026-07-01** — record steady wind (NED, m/s) via `eom01->getLastLocalAirmass()` (ft/s → m/s) → `aircraftState.setWindVelocity()` in `inputdev_autoc.cpp` (both modes). ✅ [OP] **CONFIRMED 2026-07-02** — `wN/wE/wD` non-zero across all 6690 ticks of a basic-m1 (t2) gen dmp (wN −4.0→0.1, wE −3.1→4.8; wD ~0 except 156 ticks = steady horizontal wind, physically correct). Getter populates each tick.
- [X] T009 P0-D-2: **DONE 2026-07-01** — added `RecordedRunConfig` struct (fit cone ×6, `simTimeStepMsec`/`cadenceTickScale`, crash-penalty ×3) to `EvalResults` in `protocol.h`, appended after `effectiveMasterSeed` in the v≥2 serialize block (no `CEREAL_CLASS_VERSION` bump; old dmps orphaned by retrain). Populated in `stampEvalResultsProvenance()` (`src/autoc.cc`) from `ConfigManager::getConfig()` + compiled cadence constants.
- [X] T009b [FR-P0H] P0-D situational-awareness input enrichment (baseline, NOT an ablation lever) — **DONE 2026-07-01** (part 1 layout `3cd2bc9` + part 2 population; clean rebuild-perf GREEN; see outcome.md). **(A) exit cue implemented as held NDC-centroid bearing sin/cos** (not raw image-velocity). Follow-ons NOT in T009b: T009c unit test (unwritten), xiao firmware regen (FR-033). — `include/autoc/nn/nn_inputs.h`: **`TrackerInputs` gets (A)+(B); `NNInputs` (M1/pathgen) gets (B) only** (Clarifications 2026-07-01). **(A)** `time_since_seen` + last-seen exit-side/image-velocity (compute in `src/eval/tracker_stepper.cc` from CEP-sentinel-crossing transitions; held/decaying, no stored world position; tracker-only); **(B)** `inward_body` unit vector — 3 body-frame direction cosines `chase_quat.conjugate() * normalize(arena_center − pos)` — + the existing `dist_to_boundary` tanh (generalize it from along-velocity to true radial distance). **All-attitude body-frame direction, NOT planar `sin/cos`** (operator 2026-07-01: a heading angle assumes a reference "up" and drops the out-of-plane component inverted/knife-edge); rotation/translation-relative, NOT raw x,y; **both M1 + M2**. Reuse the arena center/radius from where `dist_to_boundary` is currently computed. Update `TRACKER_NN_TOPOLOGY[0]` **and `NN_TOPOLOGY[0]`** (pathgen also grows by (B)) + `*_WEIGHT_COUNT` + `kTrackerInputMeta`/`kPathgenInputMeta` + the `sizeof(TrackerInputs)`/`sizeof(NNInputs)` static_asserts; regen `tests/nn_layout_tests.cc` + xiao codegen (pathgen firmware input count changes); surface the new columns in `tools/dmp_dump.cc` (honest recording). **The (A) cues are stateful** — `time_since_seen`/exit-side MUST reset per scenario/engage (mirror `NNControllerBackend::reset()`; un-reset state leaks across scenarios → breaks the FR-030 bitwise gate). **No new reward tuning** — existing penalty stack arbitrates. Per spec "Situational-awareness input enrichment" + FR-P0H
- [X] T009c [FR-P0H] behavioral + determinism unit test for the situational-awareness inputs (mainlined baseline, Constitution I — not a spike) — **DONE 2026-07-01**: `tests/situational_inputs_tests.cc` (registered in CMakeLists, green via `run_autoc_tests` in clean rebuild-perf). Covers time_since_seen increment/reset on CEP-sentinel crossing + at reset; held exit-bearing sign + freeze-through-blindness; inward_body unit-vec incl. inverted knife-edge out-of-plane case + on-axis Zero; gather_pathgen (B)-only vs gather_tracker (A)+(B); same-sequence determinism (memcmp). `time_since_seen` increment + reset on CEP-sentinel crossing and at scenario/engage start; last-seen exit-side sign; `inward_body` unit-vector correctness vs a known attitude+position geometry (incl. an inverted-attitude case — proves the body-frame rotation, not a planar angle); **M1 gets (B) not (A)**; and a per-scenario-reset determinism assertion (same scenario twice ⇒ identical input trace). New `tests/` case (e.g. `situational_inputs_tests.cc`) registered in the top-level CMakeLists (Constitution IV)
- [X] T010 P0-B: **DONE 2026-07-01** — `replayScore` (renderer) + the CSV-dump path (dmp_dump) now read fitness cone / streak / cadence from `evalResults.runConfig` (`RecordedRunConfig`), NOT `ConfigManager::getConfig()`. **No fallback** (M2 greenfield — a pre-038 zeroed runConfig is reproduced by checking out matching code, operator 2026-07-01). Renderer also swaps compiled `SIM_TIME_STEP_MSEC`/`kCadenceTickScale` for the recorded `simTimeStepMsec`/`cadenceTickScale`. **`-cfg`/ConfigManager retained ONLY for S3 bucket (m1/m2) + creds/profile** (renderer `s3Bucket` reads; dmp_dump bucket from the URI + `-i` for creds). dmp_dump `-i` help text corrected. Clean incremental build green (renderer + dmp-dump + run_autoc_tests + crrcsim).
- [X] T011 **DONE 2026-07-01** — affected fixtures regenerated to the 038 layout (33→37 / 54→60 counts, weights 2051/2787, pathgen meta-walk header, all `gather_*`/backend-ctor call sites) + new `situational_inputs_tests`; clean `rebuild-perf.sh` green, incl. `scenario_prng_tests` / `eval_mode_replay_tests` / `tick_rescale_tests`.
- [X] T012 [OP] **DONE 2026-07-01** — operator ran the pre-run build gate + eval-vs-training bitwise `ScenarioScore` gate (`scripts/rebuild-perf.sh`); passed. Binary current for the bake.
- [ ] T013 [OP] P0-G: re-bake a fresh **enriched** M1 source (post-P0-D format incl. T009b situational-awareness inputs) `scripts/train.sh autoc.ini logs/autoc-038-t1-m1-rebake.log`, pin the winner in `autoc-m1`, record the S3 prefix in `outcome.md`; then re-bake the M2 baseline `scripts/train.sh autoc-tracker.ini logs/autoc-038-t2-m2-baseline.log`. **Measure the enriched baseline vs the old (037 t11/t14) M1+M2 baseline** (FR-P0H): did overrun drop / boundary behavior improve? Record in `outcome.md` — this is the situational-awareness payoff, separate from the architecture ablations. **Unconditional** (Clarifications 2026-07-01): the inputs stay regardless — a null result is recorded, NOT reverted (no escape gate; reverting would force a second break).

**🚦 Checkpoint**: post-P0-D **enriched** baseline (M1 source + M2 baseline) exists → ablations may begin in parallel.

---

## Phase 3: US1 — Deeper / non-uniform history (Priority: parallel ablation, M1-FIRST GATE) 🎯

**Goal**: richer temporal context (deeper/non-uniform history) lifts a reward-invariant ceiling — proven on
**M1 first** (no FOV confound), then inherited by M2.
**Independent Test**: M1 bake with the alt layout vs t10-matched layout at matched seed moves close-track %
or median error; determinism + bitwise replay preserved.

- [ ] T014 [US1] implement the deeper layout in `include/autoc/nn/nn_inputs.h:48` (start `{1600,800,400,200,100,50,0}`, 7 integral slots at 50 ms) + bump `kNNHistoryLayoutVersion`→4 (`nn_inputs.h:60`); update `kTrackerInputMeta`/`kPathgenInputMeta` if slot count changes (per [data-model.md](data-model.md) §1)
- [ ] T015 [US1] propagate derived sizes: `HISTORY_SIZE` (`include/autoc/eval/aircraft_state.h:391`), `TrackerObservationRing::kDepth` (`include/autoc/nn/evaluator.h:143`), `HIST_PAST[]` (`src/nn/evaluator.cc:314`); **the +1 slot grows the NN input vector by 6 beacon channels** (`TrackerInput::COUNT` += 6, `NNInput::COUNT` for pathgen). **NB: US1 branches from the FR-P0H-enriched baseline (T013), so the base COUNT is already > 54** — compute from the enriched baseline, not the pre-038 `{54,32,16,3}` (the raw "54→60" no longer holds; it is `enriched_COUNT + 6`). Update `TRACKER_NN_TOPOLOGY[0]`/`NN_TOPOLOGY[0]` to the new COUNT in `include/autoc/nn/topology.h` and recompute `*_WEIGHT_COUNT`/`*_HIDDEN_STATE_COUNT` (input fan-in changes → weights change); confirm the `static_assert(sizeof(TrackerInputs)==COUNT*sizeof(float))` (per [data-model.md](data-model.md) §1 note + §2); regen `tests/nn_layout_tests.cc`, `tests/nn_evaluator_tests.cc`, `tests/tick_rescale_tests.cc`; regen xiao codegen per [contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md)
- [ ] T016 [US1][OP] pre-run build gate + bitwise gate (`rebuild-perf.sh`)
- [ ] T017 [US1][OP] bake M1 ablation `scripts/train.sh autoc.ini logs/autoc-038-t<N>-m1-hist1p6s.log` (t<N> = next free experiment number; t1/t2/t3 already consumed by rebake/basic-m1/baseline); judge with `scripts/generate_pngs.sh m1 <log> --compare baseline:logs/autoc-038-t3-m1-baseline.log` against SC-001 (M1 ceiling, fixed-eval comparator) **+ control-quality regression gate (co-equal, [project_038_regression_gate])**: no aggressiveness rise (roll ac / flip% / thr sat, per-axis ⟨|out|⟩) or bang-bang axis migration vs the t3 baseline — read `per_axis_aggressiveness` + `per_axis_time_series` from the same `--compare`

**🚦 US1 gate**: if the deeper layout moves an M1 ceiling **AND holds control quality** (no aggressiveness rise / bang-bang axis migration vs t3, [project_038_regression_gate]) → **T018 (inherit to M2)**. Ceiling lift WITH a control-quality regression does NOT clear. If neither → **escape**:
record the negative result in `outcome.md`, file the alternative spacing/depth options to `specs/BACKLOG.md`,
close US1 as ruled-out. Does NOT block US2/US3.

- [ ] T018 [US1][OP] (gate-pass only) inherit the layout to M2 and bake `scripts/train.sh autoc-tracker.ini logs/autoc-038-t4-m2-hist.log`; judge vs SC-001 (M2)

---

## Phase 4: US2 — Two-timescale recurrence (DEFERRED-TO-START → `specs/BACKLOG.md`; not in the initial wave)

> **Deferred out of the initial ablation wave (operator 2026-06-30).** US2 overlaps US1 (both add temporal
> memory); the initial wave is US1+US3. T019–T022 below stay recorded as the launch-ready plan so the arm
> can bake off the same post-P0-D baseline (T013) if unparked — pick up if US1+US3 don't move a ceiling, or
> if "not enough state" persists. File the unpark trigger in `specs/BACKLOG.md` (T037). **Do not launch
> T021/T022 in the initial wave.**

**Goal**: a structural leaky-slow recurrent channel adds trajectory memory without widening W_hh.
**Independent Test**: M2 bake with the slow channel vs single-timescale baseline at matched seed shows
better tracking and/or healthier `rnn_capacity` eff-rank at equal-or-fewer recurrent params.

- [ ] T019 [US2] implement the slow channel in `include/autoc/nn/topology.h` (both pathgen + tracker sections): start hidden-2 16-wide fast + 8-wide fixed-leak slow (α≈0.9); update `NN_TOPOLOGY`/`TRACKER_NN_TOPOLOGY`, `NN_RECURRENT`, `*_WEIGHT_COUNT`, `*_HIDDEN_STATE_COUNT`, topology string (per [data-model.md](data-model.md) §2, [research.md](research.md) §2)
- [ ] T020 [US2] implement the leaky-slow update + ensure `NNControllerBackend::reset()` zeros the slow channel (`src/nn/evaluator.cc` recurrent path, `include/autoc/nn/evaluator.h`); regen `tests/nn_layout_tests.cc`, `tests/nn_serialization_tests.cc`, xiao codegen
- [ ] T021 [US2][OP] pre-run build gate + bitwise gate
- [ ] T022 [US2][OP] bake M2 ablation `scripts/train.sh autoc-tracker.ini logs/autoc-038-t<N>-m2-slowchan.log`; judge vs baseline with `generate_pngs.sh m2 … --compare` (SC-001 + `rnn_capacity` eff-rank) **+ control-quality regression gate ([project_038_regression_gate])**: no aggressiveness rise or bang-bang axis migration vs the M2 baseline (`per_axis_aggressiveness`/`per_axis_time_series`)

**🚦 US2 gate**: clears if it moves a ceiling and/or eff-rank uses the new modes. Else **escape**: record,
file evolved-time-constant / second-recurrent-layer alternatives to BACKLOG, close US2 ruled-out.

---

## Phase 5: US3 — Auxiliary target-predictor head (Priority: parallel ablation, M2-direct, the pivot)

**Goal**: an aux head predicting next-tick optical state, scored by a SEPARATE lexicase axis, gives the
network an explicit motion model.
**Independent Test**: M2 bake with predictor vs control-only at matched seed; predicted optical state matches
realized within a target error AND tracking depth/reacquire improves; determinism + replay preserved.

- [ ] T023 [US3] grow output head 3→7 in `include/autoc/nn/topology.h` (`*_OUTPUT_COUNT`, `*_WEIGHT_COUNT`); resize `nnOutputs_[]` + deserialize output-count check in `include/autoc/eval/aircraft_state.h:580`; output buffers in `include/autoc/nn/evaluator.h` + `src/nn/evaluator.cc` (outputs[0..2]=control, [3..6]=predicted optical; aux not actuated). **The dmp is the per-tick trace** (data.dat retired in 035 FR-P05); the resized `nnOutputs_` records the aux outputs honestly via `AircraftState` cereal, but the CSV dump tool has a hardcoded 3-output header — extend `out_pt,out_rl,out_th` in `tools/dmp_dump.cc:68` to surface the 4 aux columns so they're inspectable (honest-recording audit, [feedback_honest_dmp_recording])
- [ ] T024 [US3] add `prediction_score` (`gp_fitness`, negated) to `ScenarioScore` in `include/autoc/eval/fitness_decomposition.h:71`; accumulate per-tick aux-vs-realized-next-tick optical error in `src/eval/fitness_decomposition.cc`; add it as a SEPARATE per-scenario axis in `src/eval/selection.cc` (NOT scalar-composited — per [contracts/lexicase-axis-and-reward.md](contracts/lexicase-axis-and-reward.md))
- [ ] T025 [US3] add `EnablePredictorHead` knob to `AUTOC_CONFIG_FIELDS` in `include/autoc/util/config.h` (no in-class default per VII); verify `applyCrashPenalty` (`src/autoc.cc:181`) leaves `prediction_score` untouched; regen `tests/nn_*`, xiao codegen; add a prediction-axis selection unit test
- [ ] T026 [US3][OP] pre-run build gate + bitwise gate
- [ ] T027 [US3][OP] bake M2 ablation `scripts/train.sh autoc-tracker.ini logs/autoc-038-t<N>-m2-predhead.log`; judge vs SC-001 (depth/reacquire) + SC-002 (prediction accuracy) **+ control-quality regression gate ([project_038_regression_gate])**: no aggressiveness rise or bang-bang axis migration vs the M2 baseline (`per_axis_aggressiveness`/`per_axis_time_series`)

**🚦 US3 gate**: clears if predictor develops AND lifts a ceiling **AND holds control quality** (no aggressiveness rise / bang-bang axis migration vs baseline, [project_038_regression_gate]). Else **escape**: record, file
pretrain-then-evolve / longer-horizon / recurrent-head alternatives to BACKLOG, close US3 ruled-out.

---

## Phase 6: Decision node — judge + combine winners (BLOCKING for US4 final)

**Purpose**: the branch point that can't be pre-sequenced — decide which ablation winners combine.

- [ ] T028 [OP] judge the initial-wave ablations **US1 + US3** on the SC-001 ceilings (fixed-eval comparator, not raw late-run fitness per `project_late_run_fitness_interpretation`) **AND a co-equal control-quality regression gate ([project_038_regression_gate])** — a winner must lift its ceiling WITHOUT raising aggressiveness (roll ac / flip% / thr sat, per-axis ⟨|out|⟩) or migrating the bang-bang axis vs the t3 baseline (`per_axis_aggressiveness`/`per_axis_time_series`); a ceiling lift shipped with a control-quality regression is a net loss under "accurate-m2" and does NOT clear. Record verdicts + every escape in `outcome.md` (note the US2 deferral + its unpark trigger)
- [ ] T029 combine the winning structural levers into one architecture (merge the kept `topology.h`/`nn_inputs.h` changes — up to a 2-way US1+US3 merge for the initial wave). **Reconcile the compounded counts**: final input dim = FR-P0H-enriched baseline + US1 history slots (if kept); final output dim = 3 + US3 aux (if kept); recompute `*_WEIGHT_COUNT`/`*_HIDDEN_STATE_COUNT` off both. Regenerate ALL NN test fixtures + xiao codegen; type-domain grep audit on the merged diff
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
- [ ] T034 [US4][OP] pre-run build gate + bitwise gate, then bake `scripts/train.sh autoc-tracker.ini logs/autoc-038-t<N>-m2-vis.log`; judge vs SC-003 (in-FOV/reacquire) + SC-004 (crash penalty not regressed) **+ control-quality regression gate ([project_038_regression_gate])**: the visibility reward must not raise aggressiveness or migrate the bang-bang axis vs baseline (`per_axis_aggressiveness`/`per_axis_time_series`)

**🚦 US4 gate**: clears if it lifts in-FOV/reacquire without destabilizing selection. Else **escape**:
record, file the binary-gate / ramped variant to BACKLOG, close US4 ruled-out.

---

## Phase 8: Polish & cross-cutting

- [ ] T035 [P] final type-domain grep audit across ALL 038-touched `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` (Constitution VI per-milestone close); annotate/convert remaining hits
- [ ] T036 [P] record the xiao firmware NN-contract update (FR-033) in `outcome.md` per [contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md). **Two impacts**: (1) the FR-P0H (B) arena input lands on **live pathgen firmware** — a real `NNInputs` count change requiring codegen regen + `pio run` rebuild/reflash *in this feature* (not deferred); (2) the tracker-side input/output/topology/cadence sync points stay recorded for the still-deferred tracker port
- [ ] T037 write `specs/038-accurate-m2/outcome.md`: per-SC verdicts (SC-000…SC-005), the pinned milestone S3 prefix (Constitution VIII), and a consolidated list of all escapes routed to `specs/BACKLOG.md`
- [ ] T038 [OP] confirm SC-000 (Phase 0 done) + SC-005 (bitwise gate green) + that US5 stays deferred in `specs/BACKLOG.md`

---

## Dependencies & execution order

- **Phase 1 (Setup)**: T001–T006 all independent — fully parallel.
- **Phase 2 (Foundational, BLOCKING)**: T007/T008 (crrcsim) ∥ T009 (protocol) ∥ T009b (situational-awareness inputs) → T009c (its unit test) → T010 (readers) → T011 (fixtures) → T012 (gate) → T013 (enriched baseline). T009b feeds the same P0-D break/format as T009; T009c must be green before the T012 gate; T013 blocks ALL of Phase 3+.
- **Phases 3 & 5 (initial-wave ablations)**: independent of each other — **US1 and US3 run in parallel** off the T013 baseline (separate bakes; the operator paces concurrency). Each self-gates with escape. **Phase 4 (US2) is deferred** — not launched in the initial wave (T021/T022 held for unpark).
- **Phase 6 (decision)**: depends on the initial-wave ablations completed (T017/T027); T028→T029→T030.
- **Phase 7 (US4)**: reward-only; can run any time after T013 (independent of the ablations), but final bake (T034) is best against the combined architecture (T030) if it exists.
- **Phase 8 (Polish)**: after the milestones it documents.

## Parallel opportunities

- Phase 1: T002–T006 in one batch (distinct files).
- Phase 2: T007/T008/T009/T009b concurrent (crrcsim ∥ protocol ∥ situational-awareness inputs), then serialize T010→T013.
- Phases 3 & 5: the two initial-wave ablation bakes (US1 + US3) are the primary parallelism (operator-paced, separate logs/buckets). US2 (Phase 4) is deferred — not part of the initial parallelism.
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

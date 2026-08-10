# Tasks: 041 — M2 Depth (observation-side objectives)

**Input**: Design documents from `/specs/041-m2-depth/`

## 📖 READ THIS FIRST — order, and which document wins

A fresh context should read in this order. The ordering is not cosmetic: two documents disagree with each
other on purpose, and one of them is superseded.

| # | document | why, and how much |
|---|---|---|
| 1 | **this file** | the execution list. Each task carries its file paths and its own warnings — you can work from it directly |
| 2 | [plan.md](plan.md) | stack, constitution gates, phase sequencing |
| 3 | [spec.md](spec.md) **§ Clarifications** | ⭐ **GOVERNS.** On any conflict between documents, this wins. 11 decisions across two sessions live here |
| 4 | [contracts/](contracts/) + [data-model.md](data-model.md) | binding interface detail — read the relevant one per task, not all up front |
| 5 | [research.md](research.md) | the *why* per decision, R1–R14. Read when a task's rationale is unclear |
| 6 | [hypothesis.md](hypothesis.md) | ⚠️ **SUPERSEDED WHERE IT CONFLICTS.** The derivation of record and the only place the *retractions* are written down — genuinely worth reading for **why**, but **not for what**. It carries a supersession table at the top; trust that over its body. Optional |

**If you read only one thing beyond this file, read spec.md § Clarifications.** Several decisions reversed
earlier ones, and the reversals are the load-bearing part: no load axis in 041; the in-envelope *flag* (not the
duration accumulator) is the primary input; the predictor is a horizon-free current-bearing estimator with its
error fed back, not a fixed-horizon forecast.

**Three traps that cost real money if missed** — each is called out at its task, listed here because they are
the ones you cannot recover from:

1. **T011a before T044** — extract the pre-break comparator CSVs, or the prior-M1 baseline (SC-007a) and the
   blind-gap distribution (FR-024b) are gone permanently.
2. **T036** — bit-identical fitness across moving the step score into the tick loop. Without it, a rounding
   difference is a silent objective change.
3. **T040(a)** — steady level flight must read ≈1 g on the normal accel channel, not ≈0. The only guard
   between specific force and kinematic acceleration.

**Tests**: REQUIRED, not optional — Constitution I, and FR-003 mandates the zero-answer pattern for every
paired-series fitness term. Test tasks are interleaved with implementation per Constitution I's TDD ordering.

**Organization**: grouped by user story. US1–US4 are P1, US5–US6 are P2.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable (different files, no dependency on incomplete work)
- **[Story]**: US1–US6 per spec.md
- **[OP]**: operator-driven — an assistant MUST NOT run this (clean rebuilds, bakes, S3 mutation, flight)

## ⛔ Hard ordering constraints (violating these costs a run)

1. **One contract-break commit** (FR-005). US1's grouped record and all of US2 land **together**, before any
   bake. T045 is that commit. *(One narrow exception: **FR-005a** permits the M2-only tracker innovation
   channels at T094, because N is unknown until T088 and the T023 serialize split makes it M1-safe.)*
2. **Never rebuild while a bake is live** — workers re-exec `build/autoc`.
3. **Nothing fitness-affecting after the bundle** until the M1 bake completes.
4. **Submodule pointer bump before parent merge** (crrcsim tick stamping).
5. ⛔ **T011a MUST precede T044.** Studies A and B both read the **pre-break** pinned comparators, and T044's
   version bump + fail-loud read makes those dmps unreadable to a 041 binary. Extract the CSVs first or lose
   the prior-M1 baseline (SC-007a) and the blind-gap distribution (FR-024b) permanently.
6. **T046 and T047 MUST precede T045.** T045's gate builds xiao, which compiles the generated forward pass —
   a stale `nn_program_generated.cpp` against the new input count fails or, worse, misleads.
7. All training via `scripts/train.sh` (Constitution IX). Never a background task.

---

## Phase 1: Setup

**Purpose**: confirm the build surfaces and establish artifact homes.

> ⚠️ **Standing hardware procedure — applies to EVERY INAV flash in this feature (T001, T072, T078, T080)**:
> **remove the GPS before flashing the INAV controller.** Known quirk, not up for debate. Forgetting it costs
> a debugging session, so it belongs in the runbook rather than in somebody's memory.

- [ ] T001 Confirm the INAV baseline builds in `~/inav` for **both** established targets — **bench = `MAMBAF722_2022A`** (STM32F722; `xiao/inav-bench.cfg`) and **flight = `MATEKF722MINI`** (`xiao/inav-hb1.cfg`), both currently at `63cffaf4`. Routine and precedented: **021 T041 already did exactly this** ("INAV builds for bench (MAMBAF722_2022A) and flight (MATEKF722MINI)", closed), and the commands are recorded in `specs/020-pre-flight-pipeline/plan.md`: `cd ~/inav && mkdir -p build && cd build && cmake .. && make MAMBAF722_2022A`. **Bench first.**
- [ ] T001a Record the two-variant build/deploy sequence in `specs/041-m2-depth/artifacts/README.md`, pointing at `specs/020-pre-flight-pipeline/plan.md` for the commands rather than restating them. Every later INAV task (T072, T078, T080) builds and flashes **both** targets, bench first — a change validated only on the bench target is not validated for flight.
- [ ] T002 [P] [OP] Confirm the xiao baseline builds unchanged: `cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed`, and record the byte size of `xiao/src/generated/nn_program_generated.cpp` as the pre-change reference.
- [ ] T003 [P] Create `specs/041-m2-depth/artifacts/` with a `README.md` stating what belongs there (final M1/M2 `data.dat` snapshots per FR-022, archived `nn_weights*.dat` per FR-010) and what does not (anything re-derivable from S3).
- [ ] T004 [P] Record the pinned comparator prefixes and their verified retention state in `specs/041-m2-depth/artifacts/README.md`: `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` and `autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/`, both 800/800 `retain=keep` verified 2026-08-07 (Constitution VIII.3 — provenance lives in the repo).

---

## Phase 2: Foundational (blocking prerequisites)

**Purpose**: config-surface and reporting fixes that would otherwise fail for reasons unrelated to correctness,
plus the physics reader that unblocks Study A on existing data.

- [ ] T005 Raise or remove the `INI_MAX_LINE` cap in `src/util/config.cc` so a legal `key = value  # long comment` never fails on length. This trap already aborted startup once on `EnablePredictorHead = 1` plus its comment (216 bytes) with no diagnostic, costing a bisect — and 041 adds several commented knobs.
- [ ] T006 Surface `ini_parse`'s error line in `src/util/config.cc`: replace the bare `FATAL ERROR: Cannot parse configuration file '<f>'` with `Cannot parse '<file>': line <N>: <line text>` plus a hint for the common causes (over-length line, missing `=`, stray `[`).
- [ ] T007 [P] Add a test in `tests/contract_config_tests.cc` that a deliberately malformed temp ini fails with the offending line number in the message (fixture-owned ini, never the production file).
- [ ] T008 Strip the mutable-production-value pins from `tests/contract_tracker_config_tests.cc` (`FitStreakThreshold == 0.5`, `FlightArenaRadius == 80`, `CepGateThreshold == 1.25`, `BeaconEmissionConeDeg == 270`, `BeaconLeftMountY == -0.45`). Keep at most a structural guard: production ini parses clean and required keys (`Mode`, `TrackerSourceRun`) are present. 041 changes streak config, so these pins would fail for a reason unrelated to correctness.
- [ ] T009 [P] Time-denominate the streak metrics in `src/analytics/` so `pctInStreak` / `avgMaxStreak` are surfaced in seconds consistently (037 P-O11). These are 041's primary progress signal and raw tick-denominated counts read 2× at 20 Hz — fix before they are used to judge a bake.
- [ ] T010 Add physics columns to `tools/dmp_dump.cc`: per-tick `acc[3]`, `omegaDotBody[3]`, `alpha`, `vRelWind` from `PhysicsTraceEntry`, plus derived body-frame normal acceleration from `acc[]` + `quat[]`. ⚠️ This data is **already recorded** for every elite reeval (`inputdev_autoc.cpp:1047`) and has **no consumer anywhere** — this is a reader, not a recording change, and it works on **current** dmps.
- [ ] T011 [P] Add a test in `tests/dmp_dump_tests.cc` (or the nearest existing suite) that the derived normal acceleration equals 1 g for a synthetic steady-level-flight tick and the documented sign for a synthetic pull-up.
- [ ] T011a ⛔ **HARD PREREQUISITE OF PHASE 4 — extract the comparator data while it is still readable.** Run `dmp-dump --physics` (and the standard per-tick CSV) over **both pinned comparators** — the prior M1 `autoc-m1/…2026-07-06T01:35:46.579Z/` and 040-t4 `autoc-m2/…2026-08-04T03:43:24.586Z/` — and archive the CSVs under `specs/041-m2-depth/artifacts/pre-break/`.
  **Why this cannot wait**: T044 bumps the `EvalResults` version and makes reads **fail loud** on older artifacts, so from that commit onward a 041 binary **cannot read either comparator**. Two things become permanently unobtainable if this is skipped:
  1. **The prior M1's per-regime profile** (SC-007a, FR-011c) — its 37-input genome cannot be loaded by a 041 binary either, so the recorded dmp is the *only* route to that baseline. Lose the read and the baseline is gone for good.
  2. **The blind-gap distribution** (FR-024b) — which *defines* the predictor go/no-go bins and feeds the lens-purchase decision.
  Extracting to CSV (rather than keeping a pre-break binary around) means the data outlives any build.

**Checkpoint**: config edits no longer fail spuriously, streak metrics are honest, and load is readable from
existing pinned runs. Study A (Phase 5) is now unblocked without touching the schema.

---

## Phase 3: US1 — Retire the index-coupled failure class (P1)

**Goal**: make the parallel-index bug class unrepresentable rather than tested-for.
**Independent test**: the inventory exists and every entry is fixed, structurally eliminated, or covered by a
zero-answer test; no known instance remains asserted only by a comment (SC-001).

### A0 — the scan

- [ ] T012 [US1] Sweep for collection pairs indexed by a shared loop variable across a producer/consumer boundary: `grep -rn "List\.at(\|List\[" src/eval/ src/nn/ tools/ crrcsim/src/mod_inputdev/`, then **read each hit** rather than trusting the pattern. Record findings in a new `specs/041-m2-depth/index-coupling-inventory.md`.
- [ ] T013 [P] [US1] Sweep for structs serving two lifetimes (RPC-only vs persisted) across `include/autoc/rpc/` and `include/autoc/eval/`; `ScenarioMetadata` in both roles already cost a launch on 2026-08-02. Add to the inventory.
- [ ] T014 [P] [US1] Sweep for values duplicated across two definitions (the `CameraConfig` default vs `hb1AirframeObstruction()` pattern — that one HAS a test and is the model to copy). Add to the inventory.
- [ ] T015 [P] [US1] Sweep for "compiled-in default vs recorded config" reads; note which are resolved by the US2 config block and which are not. Add to the inventory.
- [ ] T016 [US1] Complete `index-coupling-inventory.md`: every entry marked **fixed**, **structurally eliminated**, or **covered by a zero-answer test**, with the grouped-record migration list as an appendix. This is A0's deliverable and the gate on everything downstream.

### The zero-answer test pattern (write these BEFORE the refactor — they must pass identically after)

- [ ] T017 [US1] Add a zero-answer test for the M2 objective in `tests/fitness_decomposition_tests.cc`: construct data whose correct score is **exactly 0**, assert exactly 0. Add the companion assertion that a deliberately one-tick-shifted input scores **visibly worse** — a test that passes either way would be worse than none here.
- [ ] T018 [P] [US1] Add the same zero-answer + shifted-worse pair for `vis_frac` in `tests/fitness_decomposition_tests.cc`.
- [ ] T019 [P] [US1] Add the same pair for `prediction_score` in `tests/fitness_decomposition_tests.cc`. ⚠️ **Timing exception to this block's "write before the refactor" rule**: `prediction_score`'s pairing is *currently wrong*, so a zero-answer test cannot pass until T022 lands the grouped record. Write it here, expect RED, and confirm it goes green at T022 — that transition is the evidence the pairing was actually fixed. ⚠️ Two fixture traps make these silently vacuous, both already paid for: an empty `pathList` makes `computeScenarioScores` **skip the scenario** so every variant scores 0 and the comparison looks passed without running; and a bare `TEST()` misses the `ConfigManager` fixture so the run proceeds on defaults. Verify each new test **fails** when the fix is reverted.

### The structural fix

- [ ] T020 [US1] Define the grouped per-tick record in `include/autoc/rpc/protocol.h`: `tickList[i][k] = { state, cameraView, targetSample }`, with the pre-loop initial state as a **separate named field** beside the list (research.md R5) — **not** `tickList[0]` with sentinel members, which would recreate the hazard as "slot 0 is special". Tracker-only members absent (not zero-filled) in pathgen records.
- [ ] T021 [US1] Update the push sites in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` to emit grouped records, and store the initial state into its named field once before the loop.
- [ ] T022 [US1] Migrate `src/eval/fitness_decomposition.cc` to the grouped record. **Delete** the `stepIndex - 1` clamp rather than relocating it — if any consumer still needs an offset, the grouping is wrong. This subsumes FR-004: the prediction-score pairing becomes correct by construction rather than by a fix.
- [ ] T023 [P] [US1] Migrate `tools/dmp_dump.cc` to the grouped record (coordinate with T010's physics columns).
- [ ] T024 [P] [US1] Migrate `tools/renderer.cc` to the grouped record.
- [ ] T025 [P] [US1] Migrate `src/eval/source_dmp_loader.cc` to the grouped record.
- [ ] T026 [P] [US1] Migrate `src/eval/tracker_stepper.cc` and `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` to the grouped record.
- [ ] T027 [P] [US1] Migrate any `src/analytics/` reader that consumes per-tick arrays.
- [ ] T028 [US1] Add a grouped-record round-trip test in `tests/serialization_tests.cc` (or nearest): serialize → deserialize preserves every member and the tick count, and the initial-state field survives without being confused for tick 0.

**Checkpoint**: the objective is correct by construction, the tests have teeth, and the inventory says what
else shares the shape. ⚠️ Do **not** commit yet — this lands with Phase 4 as one commit (T045).

---

## Phase 4: US2 — One clean-slate contract break (P1)

**Goal**: every model- and schema-incompatible change in a single commit, so exactly one M1 rebake is owed.
**Independent test**: a fresh build records a self-describing dmp with realized wind and exact tick stamps,
read by every consumer, with input-count assertions and metadata tables in agreement (SC-002, SC-003).

### NN input slots

- [ ] T029 [US2] Add the five new slots to `include/autoc/nn/nn_inputs.h` in **both** `PathgenInput`/`NNInputs` and `TrackerInput`/`TrackerInputs`, in the order given by [contracts/nn-input-layout.md](contracts/nn-input-layout.md): `IN_ENVELOPE`, `ENVELOPE_SECS`, `ACCEL_X`, `ACCEL_Y`, `ACCEL_Z`. Place `ACCEL_*` **adjacent to `GYRO_*`** so the 6-DOF inertial block is visible. Annotate each `// raw-ok: NN-byte-format buffer`.
- [ ] T030 [US2] Add the matching rows to `kPathgenInputMeta` and `kTrackerInputMeta` (name, short name, width). The existing `static_assert` on table length vs `COUNT` enforces this — do not weaken it.
- [ ] T031 [US2] Update counts and recompute weight-count `static_assert`s in `include/autoc/nn/topology.h`: `NN_INPUT_COUNT` 37→42, `TRACKER_NN_INPUT_COUNT` 58→63, `TRACKER_NN_TOPOLOGY_STRING` → `"63,32,16r,<out>"`. **Recompute**, never relax.
- [ ] T032 [P] [US2] Add `kAccelScale_g` alongside `kCruiseSpeed_mps` / `kDistToBoundaryScale_m` in `include/autoc/nn/nn_inputs.h`, with the rationale comment (±11 g observed must land in a tanh-friendly range).
- [ ] T033 [US2] Add config knobs via the `AUTOC_CONFIG_FIELDS(X)` X-macro in `include/autoc/util/config.h`: `EnableEnvelopeInputs`, `EnableAccelInputs`, `AccelScaleG`, and the reserved M2 estimator knobs `EnvelopeSpanLo` / `EnvelopeSpanHi` / `EnvelopeCentroidRadius`. Per Constitution VII, no in-class default initializers for constructor-supplied members.
- [ ] T034 [P] [US2] Update the field-count assertion in `tests/contract_config_tests.cc` for the new knobs.

### The step score moves into the tick loop (FR-018a — the single-source-of-truth refactor)

- [ ] T035 [US2] Move the per-tick step-score / streak computation out of post-hoc `computeScenarioScores` in `src/eval/fitness_decomposition.cc` into the eval tick path, and record the per-tick result into the tick record. One computation feeds **both** the NN input gather and the fitness accumulation.
- [ ] T036 [US2] ⚠️ **Prove numerical equivalence** in `tests/fitness_decomposition_tests.cc`: fitness on a fixed genome is **bit-identical** before and after the move. Post-hoc read serialized state; inline reads live state — if those differ by a rounding step the objective changed silently, and this test is the only thing that would catch it.
- [ ] T037 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `src/nn/evaluator.cc` `gather_inputs` from the tick-loop step score. Accumulator is an **external counter** in the stepper, resets **on envelope exit only** (not on regime change), millisecond-based against `FitStreakRampSec`, **linear** normalization — no log, no tanh.
- [ ] T038 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `gather_tracker_inputs` from the M2 direct-perception estimator: both beacons CEP-visible AND separation within `[EnvelopeSpanLo, EnvelopeSpanHi]` AND pair centroid within `EnvelopeCentroidRadius`. Same accumulator mechanics as M1 — only the flag's source differs.
- [ ] T039 [US2] Populate `ACCEL_*` in both gather functions as **body-frame specific force including gravity**: `R(quat)ᵀ · (a_world − g_world) / kAccelScale_g`. ⚠️ **Not FDM kinematic acceleration** — that would put a constant ~1 g offset in the most load-relevant axis, invisible in sim and wrong in flight.
- [ ] T040 [US2] Add input-semantics tests in `tests/nn_evaluator_tests.cc`: (a) **steady level flight → normal channel ≈1 g, not ≈0** (the test that catches the kinematic-vs-specific error — and **empirically confirmed on hardware**: the bench table in `docs/COORDINATE_CONVENTIONS.md` reads `+2050` on the normal axis in level attitude — blackbox `accSmooth` counts, `acc_1G ≈ 2048`, so `≈ +1.0 g` — meaning INAV already reports specific force including gravity, exactly the sim semantics required); (b) documented sign on a pull-up; (c) `IN_ENVELOPE` ∈ {0,1}; (d) `ENVELOPE_SECS` monotone within a streak, 0 immediately after exit, saturates at 1; (e) `ENVELOPE_SECS` identical for a given wall-clock duration at two cadences; (f) M1 `IN_ENVELOPE` agrees tick-for-tick with the objective's own threshold decision.
- [ ] T041 [P] [US2] Update layout assertions in `tests/contract_evaluator_tests.cc` for both modes' new counts.

### Camera model (M2-only, fidelity to ordered hardware)

- [ ] T041a [US2] Set `CameraPixelsV = 240 → 200` in `autoc-tracker.ini`, `autoc-eval-tracker.ini`, and `autoc-eval-tracker-visual.ini`, giving V = 200 × 0.375 = **75°** (H unchanged at 120°). Leave `CameraDegPerPixel` alone so `radPerPx`, per-pixel quantisation and CEP are untouched. Rationale in the ini comment: the ordered 1.8 mm fisheye on OV9281 estimates ~124°×78° equidistant, 120×75 is the conservative split, and 320:200 = 1.6 matches the real 1280×800 aspect (the prior 240 px was a 4:3 invention, optimistic by 15° vertically). ⚠️ **Fitness-affecting** → A1 bundle only.
- [ ] T041b [P] [US2] Update the derived-FOV assertion comment in `include/autoc/eval/camera_projection.h` (currently cites ±0.785 rad / 45° for V) so the documented half-angles match the grid, and add/extend a test asserting derived V = 75° and derived H = 120° from the configured grid — the FR-003 "field and resolution cannot disagree" property.
- [ ] T041c [P] [US2] Record in `specs/041-m2-depth/artifacts/README.md` that the projection is **already equidistant** (`camera_projection.cc:158-184`, since 040 T031) and that `CameraDetectionRangeM = 100.0` is now **independently corroborated** by the 031 photon budget (bright-day post-correlation SNR ≈22 @100 m, ÷4 at 4×4 defocus → ≈5.5 vs ×4.5 threshold). Neither is a change; both are facts a later reader will otherwise re-derive.

### Recording changes

- [ ] T042 [US2] Set `AircraftState::wind_velocity` at record time in `crrcsim/.../inputdev_autoc.cpp` from the FDM's applied local airmass (getter/setter and `dmp_dump` columns already exist; the field has been serialized-but-never-set, zero in every dmp). Add a test that a run in non-zero wind records non-zero wind.
- [ ] T043 [US2] Serialize the fitness/cadence config block into `EvalResults` (`include/autoc/rpc/protocol.h`) and flip `tools/dmp_dump.cc` + `tools/renderer.cc` to **prefer the dmp-recorded config over `ConfigManager`**. Add a test that a reader with a deliberately-drifted ini still replays the recorded config's numbers.
- [ ] T044 [US2] Bump the `EvalResults` version field and implement **fail-loud** reads naming both the artifact and reader versions (Constitution V; research.md R6). No migration path, no shim. Add a test that a prior-version artifact errors with both numbers and does **not** crash in the allocator — the 038 baseline currently dies as `vector::_M_default_append`, which is exactly the diagnosis this prevents.
- [ ] T044a [US2] [OP] **crrcsim submodule**: fix `simTimeMsec` stamping in `crrcsim/src/SimStateHandler.cpp` (round, or derive from the integer step count) so a 20 Hz run records exact 50 ms gaps. Add a test asserting exact gaps. ⚠️ Determinism-affecting; **pointer-bump the submodule first**, parent merge second.

### Land it

- [ ] T045 [US2] [OP] **THE SINGLE COMMIT** — everything from Phase 3 and Phase 4 together (FR-005).
  ⛔ **Preconditions**: **T046** (`nn2cpp` regenerated) and **T047** (`sim_response.py` parser) must be done *first* — this gate builds xiao, which compiles the generated forward pass, so a stale generated file against the new input count either fails to build or builds something wrong. Also **T011a** must be done (see Phase 2), since this commit makes the pre-break comparators unreadable.
  Gates, all of which must pass: `bash scripts/rebuild.sh` green; `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green; Constitution VI audit clean on touched paths (`grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'`). Nothing lands after this until the M1 bake completes.
- [ ] T046 [US2] Regenerate `xiao/src/generated/nn_program_generated.cpp` via `tools/nn2cpp.cc` for the new layout, and add a parity test that the generated forward pass matches the desktop forward pass on a fixed input vector.
- [ ] T047 [P] [US2] Update the `data.dat` parser in `specs/019-improved-crrcsim/sim_response.py` for the new column set.
- [ ] T048 [US2] [OP] If `CMakeLists.txt` was touched by this phase, run a clean `bash scripts/rebuild-perf.sh` (Constitution IV — an incremental reconfigure can leave stale link state and miss test registration).

**Checkpoint**: one commit, one owed rebake. All three build targets green.

---

## Phase 5: US3 — Instruments that can answer the questions (P1)

**Goal**: build the measurement tools before spending compute.
**Independent test**: empty-mask ablation reproduces baseline fitness exactly; Study A reports per-regime load
from a pinned run with no recording change (SC-004, SC-005).

### The ablation tool

- [ ] T049 [US3] Create `tools/nn_ablate.cc` per [contracts/ablation-cli.md](contracts/ablation-cli.md): `-i <ini> --genome <dmp-key|weights-file> [--zero-input NAME[,...]] [--out csv]`. Slot names resolve against `kPathgenInputMeta` / `kTrackerInputMeta` — **no new naming infrastructure needed**, it already exists. Masked columns forced to 0.0 every tick after gathering, before the forward pass. Unrecognised slot name is a **hard error** listing the valid set, never a silent no-op.
- [ ] T050 [US3] Register the `nn_ablate` target and its test in `CMakeLists.txt`.
- [ ] T051 [US3] Report fields per the contract: Δfitness; per-axis Δ`dCtrl` / Δ`⟨|u|⟩`; Δ`pctInStreak` / Δ`avgMaxStreak`; Δ peak and mean normal load; per-scenario Δ distribution; and **per-regime breakdown** (`{tracking, intercept, patrol}`) — required, not optional (FR-011a), because the hypothesis predicts a signal in *one* regime and pooling would hide it.
- [ ] T052 [US3] Add `tests/nn_ablate_tests.cc`: **empty-mask identity** — with no mask, reproduces the source run's fitness **exactly** (SC-004; a tool that quietly perturbs the eval path makes every finding worthless); unknown slot name errors with the valid list; masking a known-critical input degrades measurably; two identical invocations are bit-identical.
- [ ] T053 [US3] [OP] Clean `bash scripts/rebuild-perf.sh` for the `CMakeLists.txt` touch (Constitution IV).

### Study A — regime and load (report-only; no load axis in 041)

- [ ] T054 [US3] Create `src/analytics/regime_load_study.py`: classify every tick into `{tracking, intercept, patrol}` using the **existing** rule (`stpPt ≥ 0.5`; below that, smoothed `d(dist)/dt < 0` is intercept, else patrol) — reuse `dynamics_progress.py:74-80`'s definition rather than writing a new one, so numbers stay comparable with every prior report.
- [ ] T055 [US3] Report per regime, per axis: `dCtrl`, `⟨|u|⟩`, and load distribution **plus peak** (peak is the damage-relevant statistic; a mean hides ±11 g excursions entirely). Emit machine-readable CSV alongside any plot. State sample sizes and any excluded ticks.
- [ ] T056 [US3] Add the H2 test to the study: within each regime, does pitch/roll `dCtrl` predict throttle level and load? Report correlation with a stated confidence, not a scatter plot alone.
- [ ] T057 [US3] Add normal-load **autocorrelation at the history lags** to the study — this is the cheap evidence that decides whether the accel channels ever need temporal depth (research.md R1 fallback ladder). Strong autocorrelation at 50–100 ms ⇒ one instantaneous sample suffices and neither fallback is warranted.
- [ ] T058 [US3] Run Study A on the **pre-break CSVs archived at T011a** (`artifacts/pre-break/`), covering the pinned prior M1 and 040-t4, producing `specs/041-m2-depth/study-a/`. ⚠️ Do **not** plan to re-extract from S3 — after T044 the comparators are unreadable by a 041 binary. ⚠️ Uses **current** dmps — no schema dependency, so this can run before or after the break. This also produces the **prior M1's per-regime profile**, which is the only obtainable form of that baseline (FR-011c: the 37-input genome cannot be loaded by a 041 binary, so it can never be re-evaluated or ablated).

**Checkpoint**: both instruments exist and are validated. Study A's findings are recorded as input to the
**follow-on** aggressiveness feature — 041 builds no load axis.

---

## Phase 6: US4 — Give the controller the tracking state it is paid for (P1)

**Goal**: bake a new M1 carrying the envelope inputs; answer whether the policy uses them.
**Independent test**: the bake clears non-regression, and ablation says whether the learned policy depends on
the new inputs (SC-006, SC-007, SC-007a).

### Smoke — plumbing and ballpark only, NOT comparison arms

- [ ] T059 [US4] [OP] Pre-run build gate, then M1 smoke: `scripts/train.sh autoc-basic-m1.ini logs/autoc-041-smoke-m1.log` (pop 3000, fast). Check: builds, runs, climbs at all, new inputs carry sane values. **No delta is being measured.**
- [ ] T060 [US4] [OP] M2-mode smoke — the topology change is **generic**, so prove it runs in both modes: `scripts/train.sh autoc-tracker.ini logs/autoc-041-smoke-m2.log`, killed once ticking cleanly.
- [ ] T061 [US4] Inspect the smoke runs' `data.dat` / dmp for the new columns: `IN_ENVELOPE` toggling plausibly, `ENVELOPE_SECS` ramping and resetting, `ACCEL_Z` ≈1 g in level flight. A diagnostic that does not vary is telling you something (040's trap 3).

### Production bake

- [ ] T062 [US4] [OP] Declare the retry budget and abort criterion **in writing before starting** (research.md R8 proposes 3 attempts): abort on the stuck-basin signature — throttle amplitude exactly 1.000 with **σ = 0.000** and `dCtrl` 0.000, plus `avgMaxStreak` frozen and best-sigma not annealing past ~0.14 by gen 200. Note that throttle *saturation* (mean ≈0.85, σ>0) is **not** the tell; climbers pass through it.
- [ ] T063 [US4] [OP] Pre-run build gate, then `scripts/train.sh autoc.ini logs/autoc-041-t1-m1-envelope.log` (pop 8000 / 49 winds). Judge climb on `pctInStreak` / `avgMaxStreak`, **not** completions.
- [ ] T064 [US4] [OP] Repeat as a lottery re-draw if the abort criterion fires, within the declared budget, incrementing the artifact index (`t2`, `t3`) per `autoc-<feature>-t<N>-<details>`.
- [ ] T065 [US4] [OP] Immediately on completion: snapshot `data.dat` to `specs/041-m2-depth/artifacts/` (FR-022 — the next launch overwrites it), tag all dmp objects `retain=keep`, and archive `nn_weights*.dat` beside the dmp (FR-010 — the dmp preserves numbers, only NN01 preserves a controller you can re-fly). Record the S3 prefix in the outcome doc.

### Reads

- [ ] T066 [US4] Generate the report set: `scripts/generate_pngs.sh m1 logs/autoc-041-t1-m1-envelope.log`.
- [ ] T067 [US4] Assess non-regression against the historical band (research.md R7): `pctInStreak` / `avgMaxStreak` within noise of or above the band; crash/OOB counts not worse; per-axis `dCtrl` / `⟨|u|⟩` per regime; peak load not higher. ⚠️ Absolute fitness sums are **not** comparable across runs with different scenario counts — use per-scenario or per-step rates.
- [ ] T068 [US4] Run the **ablation matrix** on the new elite: flag alone (`--zero-input IN_ENVELOPE`), duration alone (`ENVELOPE_SECS`), both, and the accel channels — per FR-014b. Expectation is that the **flag** carries the effect; a duration-only effect would be distinct and more surprising.
- [ ] T069 [US4] Run the **control-input ablations** for calibration (FR-011b): `DIST_NOW` (known-critical end), `GYRO_P,GYRO_Q,GYRO_R`, `INWARD_BODY_X,INWARD_BODY_Y,INWARD_BODY_Z` (plausibly marginal). State the envelope verdict as a **position on this spectrum**, never against an assumed absolute threshold.
- [ ] T070 [US4] Record the H1a verdict as **pass** (fitness drop **and** behavioural shift, beyond the marginal end of the control spectrum), **partial** (one but not both), or **fail** (neither) — per regime. All three close the hypothesis; only an unclassifiable result fails SC-007.
- [ ] T071 [US4] Run Study A on the new elite and produce the **per-regime intent comparison** against the prior M1's profile from T058 (SC-007a), including per-axis aggressiveness and **peak load per regime** (SC-008). Report as a **ballpark read**, not an attributable effect size — it is a cross-run profile comparison, not a controlled delta.

### Hardware deployment (conditional on the M1 result)

- [ ] T072 [US4] Extend `MSP2_AUTOC_STATE` in `~/inav/src/main/fc/fc_msp.c` (the `MSP2_INAV_LOCAL_STATE` case) to carry accel in the **same single round trip**. Copy the shape of fork commit `63cffaf4f` ("extend MSP2_AUTOC_STATE with filtered gyro rates"): append at payload end, fixed integer scale stated at the write site, and document the axis/sign convention for the consumer. ⚠️ **Source `acc.accADCf` — the TRANSFORMED field, never a raw sensor read.** `acceleration.c:563-568` applies `applySensorAlignment` then `applyBoardAlignment` then divides by `acc.dev.acc_1G`, so `acc.accADCf` is board-alignment-corrected and **already in g units**. This mirrors `gyro.gyroADCf` (`gyro.c:438-442`, same two alignment calls), which is exactly why the existing gyro extension is correct. **Do NOT use** the file-static `accADC` (`acceleration.c:73`) or `acc.dev.ADCRaw` — those are pre-alignment and would bake in each board's misalignment differently (bench roll = −16 vs flight). Wire encoding: milli-g `int16` = `lrintf(acc.accADCf[axis] * 1000.0f)`, giving ±32 g against ±11 g observed. Build **both** targets, bench first; disconnect GPS before flashing.
- [ ] T073 [US4] Pin the accel axis and sign convention against the **already-measured bench table** in `docs/COORDINATE_CONVENTIONS.md` ("Ground Verification Results, bench 2026-03-30", `MAMBAF722_2022A`, board alignment roll = −16): level → `[~0, ~0, +2050]`; right wing down 90° → `[~0, +2060, ~0]`; nose up 90° → `[+2050, ~0, ~0]`. ⚠️ **Units**: those are **blackbox `accSmooth` counts** (`acc_1G ≈ 2048`), *not* the runtime `acc.accADCf`, which is the same vector already divided by `acc_1G` — i.e. `+2050 counts` ⇔ `+1.0 g`. Convert before comparing. The **axes and signs** transfer directly; only the scale differs. T073 is therefore *match the table*, not derive the convention — add a test asserting all three attitudes. ⚠️ Board alignment differs between bench (roll = −16) and flight, so verify on **both** targets; do not assume one target's result transfers.
- [ ] T074 [US4] Consume the new accel fields in `xiao/src/msplink.cpp` and feed `ACCEL_*` into the input vector with the same specific-force semantics as sim.
- [ ] T075 [US4] Implement `IN_ENVELOPE` / `ENVELOPE_SECS` on-target in `xiao/src/`: the step-score cone geometry (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`) thresholded at `FitStreakThreshold`, plus the duration accumulator with a **reset on engage** as well as on envelope exit (FR-022a). This is firmware work, not codegen.
- [ ] T076 [US4] Verify the added payload does not push the MSP cycle past its loop budget. 039 measured zero overruns at 115200 with the prior payload; if headroom is marginal, the unexercised 460800 baud-raise lever is the documented next step rather than dropping the field.
- [ ] T077 [US4] Decide explicitly whether the queued `mspOverrideInit` first-frame patch (backlog C1 — MSPRCOVERRIDE engage pays a spurious 200 ms floor) rides along, since INAV is being built and flashed anyway. Record the decision either way — this is the same "now is the time" logic as the format break, and the window closes when the flash does.
- [ ] T078 [US4] [OP] Bench parity before flight (SC-011a): generated forward pass reproduces the desktop forward pass on a fixed input vector, and the new inputs read sane values on-target against a known geometry.
- [ ] T079 [US4] [OP] Flight-test go/no-go per FR-022d: proceed **unless** the M1 result is an utter fail/reject (non-regression failed, or H1a a clear fail with no behavioural change). **Record the decision and its reason either way** — silence on this point is not an acceptable outcome (SC-011b).
- [ ] T080 [US4] [OP] If go: fly, and produce a flight report in `flight-results/flight-<date>/` with per-flight clock-anchor fit (standing practice). Watch the standing load trend — +11.2 g / −8.4 g is the current record and loads have crept up flight-over-flight.

**Checkpoint**: M1 rebaked and pinned; the central hypothesis has a verdict; the new M1 is the M2 source.

---

## Phase 7: US5 — Decide the predictor's fate on evidence (P2)

**Goal**: settle offline whether the head can produce usable signal, before committing a bake.
**Independent test**: the study returns a verdict against a no-information baseline, binned by gap age
(SC-009).

- [ ] T081 [US5] Extend `src/analytics/regime_load_study.py` (or a sibling in the same package) to extract per-tick target-bearing truth and visibility from the **T011a pre-break tracker CSVs** (not from S3 — see constraint 5), and to compute the **hold-last-seen** baseline (dead-reckon at zero rate).
- [ ] T082 [US5] Produce the **blind-gap distribution** — frequency, duration histogram, exit→re-entry bearing offset. ⚠️ **This must come first**, because it *defines* the go/no-go criterion (FR-024b), and it is a deliverable in its own right regardless of the predictor verdict — **it is an input to the lens purchase** (1.8 mm vs 2.x mm depends on how useful the predictor turns out to be), and nobody has measured it.
  ⚠️ **Measured at V = 90°, applied at V = 75°.** The recorded runs available (040-t4) predate FR-029, so their gap distribution is **optimistic** — narrowing V shifts mass toward longer gaps. This is conservative in the right direction (bins qualified against t4 are a harder bar than reality), so use it, but say so. Predictability itself (T083) is largely FOV-independent and needs no caveat.
  Cross-check against the physics: [camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md) §3 predicts a 3 g target exits a ±36° half-field in ~1.1 s @50 m / ~1.5 s @100 m — so expect the relevant timescale to be **order 1 s, not sub-second**. A measured distribution far from that wants explaining before it is trusted.
- [ ] T083 [US5] Fit an offline regressor for **target (a) — the continuous current-bearing estimate**: from the perception history window, predict the target's bearing *now*, including through blind ticks. Score as **r² against hold-last-seen**, **binned by seconds since last truth**, with per-bin sample counts. ⚠️ A pooled number is misleading by construction — on visible ticks the truth is an input, so both head and baseline are near-perfect.
- [ ] T084 [P] [US5] Fit target **(b) — Δspan at 50/100/150 ms** as the *control* that confirms why the old head failed. Confounded until the pairing is correct; T022 satisfies that structurally.
- [ ] T085 [P] [US5] Fit target **(c) — discounted future `stepPoints`** vs a constant-mean baseline (the value-head fallback).
- [ ] T086 [US5] Apply the go/no-go rule (FR-024a): the head must beat hold-last-seen **in the gap-age bins where real excursions occur**, per T082's measured distribution, by a margin stated in advance, with adequate per-bin samples. Bound the claim by two physical facts (FR-030, FR-031): the drift budget Δθ ≈ ½·a·t²/r + 1–2° IMU error sets what hold-last-seen's error *should* look like as a gap ages, and **warm code relock has a ≈155 ms floor** — so the predictor's value is in **pointing, not latency**, and any claimed reduction in time-to-reacquire below 155 ms is unphysical. ⚠️ The verdict feeds a hardware decision (which lens gets bought), so record it with its evidence rather than as a verdict alone.
- [ ] T087 [US5] ⚠️ **DECISION NEEDED FROM OPERATOR before running**: is E1 (`EnablePredictorHead` 0 vs 1, short tracker runs) still worth a run, given that T086 decides the head's fate anyway? If the head is being retired or re-targeted regardless, E1's question ("is the dead head taxing the search") is moot and this task should be **dropped**. Flagged outstanding at the second clarify pass; resolving it may remove a tracker run from the plan.
- [ ] T088 [US5] Record the **C3 decision** in `specs/041-m2-depth/predictor-decision.md`: re-targeted continuous-estimate head, value-head fallback, or **retire**. Retirement is an accepted outcome (FR-027) and shrinks the output topology 7→3, reclaiming 119 output weights and a third of the lexicase pool.

**Checkpoint**: the predictor question is answered before any M2 compute is spent.

---

## Phase 8: US6 — One M2 bake, scoped to the predictor question (P2)

**Goal**: report whether prediction produced signal.
**Independent test**: the predictor verdict is reportable independent of any aggressiveness outcome (SC-010).

*Tasks T089–T094 apply only if T088 decided to build a head. If it decided to retire, do T089 and T093–T096 only.*

- [ ] T089 [US6] Apply the T088 decision to `include/autoc/nn/topology.h`: output count 7 (head retained) or 3 (retired), with `TRACKER_NN_TOPOLOGY_STRING` and weight-count `static_assert`s recomputed.
- [ ] T090 [US6] Implement the continuous current-bearing estimate outputs in `src/nn/evaluator.cc`, scaled into the target quantity's domain (FR-026) so the usable range is not a small fraction of one output unit — the failure that turned 040-t1's error curve into a saturation readout.
- [ ] T091 [US6] Implement scoring in `src/eval/fitness_decomposition.cc`: score against truth wherever truth exists — every visible tick **plus the reacquisition tick** — and **do not** exclude blind ticks by visibility gating (FR-025a). The current objective CEP-gates both endpoints, scoring only where prediction is information-free; that exclusion is the defect being corrected.
- [ ] T092 [US6] Implement the **innovation feedback** (FR-025c–f): compute `error = truth − estimate` in the stepper when truth arrives, write it into the *next* tick's tracker input vector, signed per axis. **Hold the last value during blindness** — zeroing would falsely assert "my model is correct". No new staleness slot: the existing `TIME_SINCE_SEEN` covers it. No output-layer recurrence required.
- [ ] T093 [US6] Add tests in `tests/fitness_decomposition_tests.cc` and `tests/gather_tracker_inputs_tests.cc`: a perfect estimator scores exactly zero error; blind ticks are included in scoring; the innovation input holds its value through a synthetic blind gap and updates on reacquisition.
- [ ] T094 [US6] Update `include/autoc/nn/topology.h` tracker input count `63 → 63 + N` for the innovation channels (N = estimate dimensionality, fixed at T088) and add the metadata rows. ⚠️ This is the **one permitted post-A1 layout change** (FR-005a) — legal because `TrackerInputs` is a separate struct with a separate genome, and 040's T023 `AircraftState::serialize` split means **no M1 source rebake** is needed, so the M1 source dmp stays readable. Recompute the weight-count `static_assert`s; do not relax them.
- [ ] T095 [US6] [OP] Repoint `autoc-tracker.ini` at the **new pinned M1 source** from T065. ⚠️ Check the scenario **shape** explicitly (`SimNumPathsPerGeneration`, `WindScenarios`), not just the run id — reproducing a run once needed four hand-aligned fields, and the 1:1 seed-table guard catches count mismatches only. A config differing in sigmas or enables produces a plausible wrong number.
- [ ] T096 [US6] [OP] Pre-run build gate, then `scripts/train.sh autoc-tracker.ini logs/autoc-041-t<N>-m2-predictor.log`.
- [ ] T097 [US6] Report in priority order: (1) **predictor signal or its absence** — the deliverable, judged as variance explained against the no-information baseline, binned by gap age; (2) whether the M1 aggressiveness change **carried through** — a free observation, not a designed experiment (FR-028); (3) **novel-geometry generalization** on the pinned 038-t10 source plus the training set, against the T085 baseline (13.03 m median, 15.3% inside 5 m trained; 15.00 m, 8.4% novel). ⚠️ State the single-pinned-set limitation: the second novel source was deliberately left to expire, so a difference cannot be cross-checked against an independent sample.
- [ ] T098 [US6] [OP] Pin the M2 run `retain=keep`, archive its weights, and snapshot its `data.dat` (FR-010, FR-022).

---

## Phase 9: Polish & wrap

- [ ] T099 Run the Constitution VI type-domain audit on every touched path and either annotate `// raw-ok: <reason>` or convert: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'`. No milestone is done with unannotated raw-type hits in its diff.
- [ ] T100 [P] Write `specs/041-m2-depth/outcome.md`: every hypothesis (H1a, H1b, H2, H3, predictor) recorded as supported or refuted **with its evidence** (SC-012 — a refuted hypothesis is a successful outcome), plus all pinned S3 prefixes (Constitution VIII.3).
- [ ] T101 [P] Return deferred items to `specs/BACKLOG.md` in order (Constitution X): accelerometer noise/bias modelling; the accel temporal-depth fallback ladder (differentiation before history); the load/aggressiveness feature seeded by Study A's findings; anything Study A removed from scope. The visibility-filter entry is already filed.
- [ ] T102 [P] Add the INAV build section to `docs/toolchains.md` from T001's recorded sequence, if not already done there.
- [ ] T103 [P] Update `~/.claude/.../feedback_no_cereal_versioning.md` to reflect the resolved practice — bump the version field, maintain no compatibility — since it currently reads as an unqualified "never bump" and the constitution outranks it.
- [ ] T104 Update `specs/BACKLOG.md` routing header with the 041 outcome and the next feature's pointer.

---

## Dependencies & story order

```text
Phase 1 (setup, INAV bring-up)
   │
Phase 2 (foundational: config surface, physics reader, ⛔ T011a pre-break CSV extraction)
   │
   ├─────────────► Phase 5 partial: T054–T058 (Study A on the T011a CSVs — must be extracted
   │                                            BEFORE T044's version bump, see constraint 5)
   │
Phase 3 (US1 structural) ──┐
                           ├──► T045 THE SINGLE COMMIT ──► Phase 5 rest (instruments) ──► Phase 6 (US4 bake)
Phase 4 (US2 bundle) ──────┘                                                                   │
                                                                                               ▼
                                                              Phase 7 (US5 predictor decision, offline)
                                                                                               │
                                                                                               ▼
                                                                          Phase 8 (US6 M2 bake) ──► Phase 9
```

**Story independence**:

- **US1** delivers value alone (a bug class retired) even if 041 stops there.
- **US2** is coupled to US1 by the one-commit rule — they are separate work but a single landing.
- **US3** is independent and its Study A half runs on existing data before any change.
- **US4** depends on the commit; its hardware sub-phase is conditional on its own result.
- **US5** is independent of US4's outcome and could run in parallel with the bake (it is offline).
- **US6** depends on US4 (source) and US5 (design decision).

**Parallel opportunities**:

- Phase 1: T002, T003, T004 together.
- Phase 2: T007, T009, T011 alongside T005–T006, T010.
- Phase 3: T013–T015 together; T018–T019 together; T023–T027 together (distinct consumers).
- Phase 5: Study A (T054–T058) fully parallel with the ablation tool (T049–T053).
- Phase 7: T084, T085 alongside T083.
- **Phase 7 can run entirely during the Phase 6 bake** — it is offline and touches no build state. ⚠️ But it must not *rebuild* while the bake is live; analysis only.
- Phase 9: T100–T103 together.

---

## Implementation strategy

**MVP = US1 + US2.** A retired bug class, a correct objective, and one clean contract break is a complete,
defensible increment even if no bake ever runs.

**Incremental delivery**:

1. **Phase 1–2** retires the INAV unknown and stops config edits failing spuriously.
2. **Study A early** (T054–T058, on current dmps) — cheapest possible information, and it can *remove* work
   from the feature by showing no load problem exists.
3. **US1 + US2 → T045** the single commit.
4. **US3 instruments**, validated before they are trusted.
5. **US4** the bake and its verdict; hardware only if the result earns it.
6. **US5 offline during the bake**, then **US6** if the predictor earns a head.

**What can cheaply kill work — by design**:

| gate | can remove |
|---|---|
| T058 Study A | the entire load/aggressiveness thread from the follow-on's scope |
| T057 load autocorrelation | the accel temporal-depth fallbacks |
| T070 H1a verdict | the whole envelope hypothesis, for the cost of an eval |
| T086 predictor go/no-go | Phase 8's head work, before a 27 h bake |
| T087 E1 decision | one tracker run |

**Total**: 110 tasks. US1 17 · US2 24 · US3 10 · US4 22 · US5 8 · US6 10 · setup/foundational/polish 19.
*(US2 grew by 3 at the 2026-08-10 camera-model pass: T041a–T041c. T011a added at the same pass's ordering
review — it is the one task whose omission is unrecoverable.)*

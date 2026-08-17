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
2. **T036** — moving the step score into the tick loop must not silently change the objective. *(Reframed
   2026-08-10: the gate is **determinism + "materially the same or better"**, not literal bit equality —
   see spec.md § Clarifications. Silence is the failure mode, not difference.)*
3. **T040(a)** — steady level flight must read ≈1 g on the normal accel channel, not ≈0. The only guard
   between specific force and kinematic acceleration.

**Tests**: REQUIRED, not optional — Constitution I, and FR-003 mandates the zero-answer pattern for every
paired-series fitness term. Test tasks are interleaved with implementation per Constitution I's TDD ordering.

**Organization**: grouped by user story. US1–US4 are P1, US5–US6 are P2.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable (different files, no dependency on incomplete work)
- **[Story]**: US1–US6 per spec.md
- **[OP]**: operator-driven — an assistant MUST NOT run this (clean rebuilds, bakes, S3 mutation, flight)

## ▶️ RESUME HERE (state as of 2026-08-11)

**Progress: 47/111 marked.** Phases 1–3 complete; Phase 4 (US2) through T041c. Build clean on both
surfaces (autoc + crrcsim), **43/43 ctest suites pass** (+1: `envelope_accel_inputs_tests`, the T040
semantics suite). ⚠️ All Phase 3/4 work is **checkpointed WIP, not landed** — it squashes into T045.
Task count is 111, not 110: **T074a** added 2026-08-11 (bench-observable NN inputs, operator ask).

### What landed 2026-08-11

- **The accel sign datum is SETTLED** (see the next section) — and the answer was "no flip", so nothing
  was flipped. `docs/COORDINATE_CONVENTIONS.md` now carries the derivation and the evidence.
- **T037–T040 complete.** The step score now computes **pre-eval in both modes**; `IN_ENVELOPE` /
  `ENVELOPE_SECS` / `ACCEL_*` reach both gathers; 8 new semantics tests.
- New shared header **`include/autoc/eval/envelope_state.h`** — `EnvelopeState` (accumulator mechanics,
  identical in both modes) + `perceivedInEnvelope` (the M2 direct-perception estimator). `EnvelopeState`
  joins `resetPerceptionState()`, so both tracker paths reset it without either being edited.
- `CrrcsimTrackerHelper::peekTargetGeometry()` — reads `samples[cursor_]` **without advancing**, which is
  what let the tracker score pre-eval. The clamp at exhaustion is not padding: it reproduces the old
  `lastTargetSample()` value exactly, which is what keeps the move a relocation.

### State after the 2026-08-11 evening pass

**Phase 4 is code complete except T046**, which is deliberately parked (below). Landed this pass:
T042 + T044a tests (`tests/recording_fidelity_tests.cc`), T043's real remainder (041 knobs recorded,
`renderer.cc:4651` un-half-flipped, tests), and T044 (version 2→3 with a **real** fail-loud check).
**44/44 ctest suites pass**; autoc, crrcsim, dmp-dump and renderer all build.

⚠️ **T044's headline correction**: the old comment claimed *"v=3+ dmps fail loudly via cereal's
class-version mechanism"*. **Cereal does no such thing** — it forwards the stored version to `serialize()`
and rejects nothing, which is exactly why mismatches died in the allocator as `vector::_M_default_append`
instead of naming the artifact. The check is now explicit in `EvalResults::serialize`, keyed to
`EvalResults::kSchemaVersion`, static_asserted against `CEREAL_CLASS_VERSION`, and covered by three
hermetic tests (older / newer / current) that replaced a `SUCCEED()` placeholder asserting nothing.

**T046 is PARKED** — operator 2026-08-11: *"that part of validate needs to wait."* It is the sole
remaining T045 precondition, so **T045 cannot land until T046a is done**. See T046 for the split and for
the silent 37-vs-42 layout hazard that must be made loud as part of it.

### ⚠️ Next, and the one thing to be careful about

**T042 → T043 → T044 → T046 → T045.** Ordering constraint 6 reduced to T046-only (T047 is inverted), so
the remaining gate order is: T046 regenerates `nn_program_generated.cpp` for 42 inputs, **then** T045
commits and builds xiao. T011a is already done, so T044's version bump is safe to take.

⚠️ **The T037 move is unverified by anything an assistant can run.** Be precise about which "T036" is meant,
because the two are not interchangeable:

| | what it is | catches the T037 move? |
|---|---|---|
| **T036 unit test** (`fitness_decomposition_tests.cc`) | in ctest, no rebuild, passing | ❌ **No.** Its fixtures fill `stepScore` via `fillStepScores()` — the post-hoc reference — so the worker is never invoked. The test says so itself at `:895-900`. It proves the objective READS the recorded series, deterministically. That is orthogonal to where the worker computes it. |
| **eval-vs-training bitwise gate** [OP] | clean `rebuild-perf.sh` + a run, fitness compared exactly | ✅ **Yes** — it is the only thing that compares a worker-produced dmp against the objective end to end. |

So the argument for the move being value-preserving (the NN eval writes only pitch/roll/throttle; position,
attitude and the target lookup are all fixed before it runs; `peekTargetGeometry`'s exhaustion clamp
reproduces the old `lastTargetSample()` exactly) is **an argument, not a measurement** — and "silence is the
failure mode" is exactly T036's point. **The operator's bitwise gate is what closes it, before T045.**
Re-running the unit suite is not a substitute and should not be mistaken for one.

### ✅ SETTLED 2026-08-11 — the accel sign needs NO flip; do not re-open it

The open datum is **resolved against `~/inav` @ `63cffaf4`**, and the answer is that
`include/autoc/eval/specific_force.h` and its five tests in `tests/dmp_dump_tests.cc` are **already
correct**. Full derivation and evidence: `docs/COORDINATE_CONVENTIONS.md` → "Accelerometer as an INTERFACE
quantity (041)" → *RESOLVED 2026-08-11*.

- **INAV's body frame is FLU** (x fwd, y **left**, z **up**) and `acc.accADCf` is plain **proper
  acceleration**, which at rest points UP. Under that reading **all three** bench rows fit — including
  nose-up. Nose-up was the only *discriminating* row because FLU→FRD flips y and z but **shares x**, so on
  level and RWD the frame flip and the sign flip cancel and both hypotheses agree.
- **The msplink converter is the same y/z flip already used for the quat and gyro**:
  `accel_FRD = (accADCf[0], −accADCf[1], −accADCf[2])` (T074).
- **In FRD, level flight reads `ACCEL_Z = −1`.** That matches spec.md § Clarifications (which GOVERNS —
  session 2026-08-10, "the sim's `ACCEL_Z` in steady level flight is **−1** in body axes") and matches the
  header as written.
- ⚠️ **The earlier "flip it" instruction is WITHDRAWN.** It was derived against the unresolved datum and
  asserted the opposite convention. Flipping the header would have put the NN's load axis backwards
  relative to flight — invisible in sim, wrong in the air.
- **All three attitudes are now safe to assert** at T073, in FRD: level `[0,0,−1]`, nose up `[+1,0,0]`,
  right wing down `[0,−1,0]`. The bench table is in INAV FLU **counts** (`acc_1G ≈ 2048`); divide *and*
  flip y/z before comparing.

### Then T037–T040, which are now fully specified

- **Ordering is SOLVED** (spec.md Clarifications, "the tracker target is a PRELOADED PATH"): the tracker
  target is `source_->samples[cursor_]` (`crrcsim_tracker_helper.cpp:208`) — a preloaded trajectory
  indexed by a cursor, structurally identical to pathgen's `path[pathIndex]`. So the step score is a
  LOOKUP in both modes, knowable before the NN acts ⇒ compute **pre-eval in both**. No tick k−1 fallback.
  Currently the worker scores AFTER the eval (`inputdev_autoc.cpp`, "SCORE THE TICK") — move it up.
- **T039 plumbing is DECIDED**: body-frame specific force in g stored on `AircraftState` **beside
  `gyroRates_`**, computed worker-side via `specific_force.h`; both gathers only COPY. Criterion was
  "closest to what INAV presents to real hardware NN inputs" — on hardware the value arrives finished and
  the gather does no transformation, so the sim must match that shape.
- **T040** is wiring, not derivation: the shared math and its five tests already exist from T010/T011.

### ✅ Filed 2026-08-16 — no longer unfiled

The **M2 flight architecture** (operator 2026-08-10) is now in `specs/BACKLOG.md` → *Post-041 direction*,
where it belongs as future firmware scope. It was extended there by the operator on 2026-08-16: M2 is
**two-fold** — phase 1 a virtual M1 target on the flight hardware with a synthetic camera (beacon
projection + CEP on the nRF52840), phase 2 chasing a real craft with real beacons — demonstrating
all-attitude flight control and a generalized controller. **M3 then forks**, deliberately undecided:
target-stops-broadcasting (vision-only) vs strategy (offense/defense).

Still not 041 — 041's flight test is M1 (T079/T080).

### Decisions taken 2026-08-10 that changed the plan

| | outcome |
|---|---|
| T062 | **Reframed, closed.** No retry budget, no lottery framing — "a good bake is good enough". Early failures are **diagnostics** (missed assumptions / non-determinism), not bad luck. T064 folds into T063 |
| T067 / R7 | Judge against the **pinned prior M1 alone**, loose bar, "materially worse" — one comparator cannot make a band |
| T087 | **DROPPED** — one tracker run removed |
| "calibration" | = re-establish the **comparator set**. Closes with no new work (T065 / FR-010 / T097) |
| Camera pitch | ~~**0.375 stays** (120°×75°). Revisit from measurement, not a second estimate~~ → **SUPERSEDED 2026-08-16 (T041d)**: the measurement arrived. **0.304** ⇒ 97.3°×60.8°. The reserved trigger fired; see spec.md § Clarifications |
| crrcsim builds | **Build freely; gates stay with the operator.** `crrcsim/CLAUDE.md` updated |
| T036 | Gate is **determinism + materially-same**, not literal bit equality |
| T047 | **INVERTED** — `data.dat` retired at 035 FR-P05; its parser must NOT be updated |

---

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
6. **T046 MUST precede T045.** T045's gate builds xiao, which compiles the generated forward pass —
   a stale `nn_program_generated.cpp` against the new input count fails or, worse, misleads.
   *(T047 was the second half of this constraint and is now INVERTED — `data.dat` is no longer generated, so
   its parser must NOT be updated. See T047.)*
7. All training via `scripts/train.sh` (Constitution IX). Never a background task.

---

## Phase 1: Setup

**Purpose**: confirm the build surfaces and establish artifact homes.

> ↪ **INAV bring-up (T001, T001a) is deferred to Phase 6** (operator decision 2026-08-10): it is hardware
> work that wants the operator near the gear, and nothing in Phases 2–5 depends on it. It is now the first
> item of Phase 6's hardware sub-phase, triggered once the M1 read (T067) looks decent. The IDs are
> unchanged so cross-references (T102) still resolve. Phase 1 is therefore desktop-only.

- [ ] T002 [P] [OP] Confirm the xiao baseline builds unchanged: `cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed`, and record the byte size of `xiao/src/generated/nn_program_generated.cpp` as the pre-change reference. **Host compile — no board attached.** It stays in Phase 1 (operator decision 2026-08-10, when INAV moved out) for two reasons: the byte size is only a *pre-change* reference if taken before T046 overwrites the file, and T045's gate compiles this same generated forward pass (ordering constraint 6). The xiao tasks that need the board — T074, T075, T078 — are in Phase 6 with INAV.
- [X] T003 [P] Create `specs/041-m2-depth/artifacts/` with a `README.md` stating what belongs there (final M1/M2 `data.dat` snapshots per FR-022, archived `nn_weights*.dat` per FR-010) and what does not (anything re-derivable from S3).
- [X] T004 [P] Record the pinned comparator prefixes and their verified retention state in `specs/041-m2-depth/artifacts/README.md`: `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` and `autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/`, both 800/800 `retain=keep` verified 2026-08-07 (Constitution VIII.3 — provenance lives in the repo).

---

## Phase 2: Foundational (blocking prerequisites)

**Purpose**: config-surface and reporting fixes that would otherwise fail for reasons unrelated to correctness,
plus the physics reader that unblocks Study A on existing data.

- [X] T005 Raise or remove the `INI_MAX_LINE` cap in `src/util/config.cc` so a legal `key = value  # long comment` never fails on length. This trap already aborted startup once on `EnablePredictorHead = 1` plus its comment (216 bytes) with no diagnostic, costing a bisect — and 041 adds several commented knobs.
- [X] T006 Surface `ini_parse`'s error line in `src/util/config.cc`: replace the bare `FATAL ERROR: Cannot parse configuration file '<f>'` with `Cannot parse '<file>': line <N>: <line text>` plus a hint for the common causes (over-length line, missing `=`, stray `[`).
- [X] T007 [P] Add a test in `tests/contract_config_tests.cc` that a deliberately malformed temp ini fails with the offending line number in the message (fixture-owned ini, never the production file).
- [X] T008 Strip the mutable-production-value pins from `tests/contract_tracker_config_tests.cc` (`FitStreakThreshold == 0.5`, `FlightArenaRadius == 80`, `CepGateThreshold == 1.25`, `BeaconEmissionConeDeg == 270`, `BeaconLeftMountY == -0.45`). Keep at most a structural guard: production ini parses clean and required keys (`Mode`, `TrackerSourceRun`) are present. 041 changes streak config, so these pins would fail for a reason unrelated to correctness.
- [X] T009 [P] Time-denominate the streak metrics in `src/analytics/` so `pctInStreak` / `avgMaxStreak` are surfaced in seconds consistently (037 P-O11). These are 041's primary progress signal and raw tick-denominated counts read 2× at 20 Hz — fix before they are used to judge a bake.
- [X] T010 Add physics columns to `tools/dmp_dump.cc`: per-tick `acc[3]`, `omegaDotBody[3]`, `alpha`, `vRelWind` from `PhysicsTraceEntry`, plus derived body-frame normal acceleration from `acc[]` + `quat[]`. ⚠️ This data is **already recorded** for every elite reeval (`inputdev_autoc.cpp:1047`) and has **no consumer anywhere** — this is a reader, not a recording change, and it works on **current** dmps.
- [X] T011 [P] Add a test in `tests/dmp_dump_tests.cc` (or the nearest existing suite) that the derived normal acceleration equals 1 g for a synthetic steady-level-flight tick and the documented sign for a synthetic pull-up.
- [X] T011a ⛔ **HARD PREREQUISITE OF PHASE 4 — extract the comparator data while it is still readable.** Run `dmp-dump --physics` (and the standard per-tick CSV) over **both pinned comparators** — the prior M1 `autoc-m1/…2026-07-06T01:35:46.579Z/` and 040-t4 `autoc-m2/…2026-08-04T03:43:24.586Z/` — and archive the CSVs under `specs/041-m2-depth/artifacts/pre-break/`.
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

- [X] T012 [US1] Sweep for collection pairs indexed by a shared loop variable across a producer/consumer boundary: `grep -rn "List\.at(\|List\[" src/eval/ src/nn/ tools/ crrcsim/src/mod_inputdev/`, then **read each hit** rather than trusting the pattern. Record findings in a new `specs/041-m2-depth/index-coupling-inventory.md`.
- [X] T013 [P] [US1] Sweep for structs serving two lifetimes (RPC-only vs persisted) across `include/autoc/rpc/` and `include/autoc/eval/`; `ScenarioMetadata` in both roles already cost a launch on 2026-08-02. Add to the inventory.
- [X] T014 [P] [US1] Sweep for values duplicated across two definitions (the `CameraConfig` default vs `hb1AirframeObstruction()` pattern — that one HAS a test and is the model to copy). Add to the inventory.
- [X] T015 [P] [US1] Sweep for "compiled-in default vs recorded config" reads; note which are resolved by the US2 config block and which are not. Add to the inventory.
- [X] T016 [US1] Complete `index-coupling-inventory.md`: every entry marked **fixed**, **structurally eliminated**, or **covered by a zero-answer test**, with the grouped-record migration list as an appendix. This is A0's deliverable and the gate on everything downstream.

### The zero-answer test pattern (write these BEFORE the refactor — they must pass identically after)

- [X] T017 [US1] Add a zero-answer test for the M2 objective in `tests/fitness_decomposition_tests.cc`: construct data whose correct score is **exactly 0**, assert exactly 0. Add the companion assertion that a deliberately one-tick-shifted input scores **visibly worse** — a test that passes either way would be worse than none here.
- [X] T018 [P] [US1] Add the same zero-answer + shifted-worse pair for `vis_frac` in `tests/fitness_decomposition_tests.cc`.
- [X] T019 [P] [US1] Add the same pair for `prediction_score` in `tests/fitness_decomposition_tests.cc`. ⚠️ **Timing exception to this block's "write before the refactor" rule**: `prediction_score`'s pairing is *currently wrong*, so a zero-answer test cannot pass until T022 lands the grouped record. Write it here, expect RED, and confirm it goes green at T022 — that transition is the evidence the pairing was actually fixed. ⚠️ Two fixture traps make these silently vacuous, both already paid for: an empty `pathList` makes `computeScenarioScores` **skip the scenario** so every variant scores 0 and the comparison looks passed without running; and a bare `TEST()` misses the `ConfigManager` fixture so the run proceeds on defaults. Verify each new test **fails** when the fix is reverted.

### The structural fix

- [X] T020 [US1] Define the grouped per-tick record in `include/autoc/rpc/protocol.h`: `tickList[i][k] = { state, cameraView, targetSample }`, with the pre-loop initial state as a **separate named field** beside the list (research.md R5) — **not** `tickList[0]` with sentinel members, which would recreate the hazard as "slot 0 is special". Tracker-only members absent (not zero-filled) in pathgen records.
- [X] T021 [US1] Update the push sites in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` to emit grouped records, and store the initial state into its named field once before the loop.
- [X] T022 [US1] Migrate `src/eval/fitness_decomposition.cc` to the grouped record. **Delete** the `stepIndex - 1` clamp rather than relocating it — if any consumer still needs an offset, the grouping is wrong. This subsumes FR-004: the prediction-score pairing becomes correct by construction rather than by a fix.
- [X] T023 [P] [US1] Migrate `tools/dmp_dump.cc` to the grouped record (coordinate with T010's physics columns). ⚠️ **Also `tools/tracker_dmp_inspect.cc`** — 11 parallel-index hits, found by the T012 sweep and absent from the original task list; it breaks at T045 if skipped (index-coupling-inventory.md appendix).
- [X] T024 [P] [US1] Migrate `tools/renderer.cc` to the grouped record.
- [X] T025 [P] [US1] Migrate `src/eval/source_dmp_loader.cc` to the grouped record.
- [X] T026 [P] [US1] Migrate `src/eval/tracker_stepper.cc` and `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` to the grouped record.
- [X] T027 [P] [US1] Migrate any `src/analytics/` reader that consumes per-tick arrays.
- [X] T028 [US1] Add a grouped-record round-trip test in `tests/serialization_tests.cc` (or nearest): serialize → deserialize preserves every member and the tick count, and the initial-state field survives without being confused for tick 0.

**Checkpoint**: the objective is correct by construction, the tests have teeth, and the inventory says what
else shares the shape. ⚠️ Do **not** commit yet — this lands with Phase 4 as one commit (T045).

---

## Phase 4: US2 — One clean-slate contract break (P1)

**Goal**: every model- and schema-incompatible change in a single commit, so exactly one M1 rebake is owed.
**Independent test**: a fresh build records a self-describing dmp with realized wind and exact tick stamps,
read by every consumer, with input-count assertions and metadata tables in agreement (SC-002, SC-003).

### NN input slots

- [X] T029 [US2] Add the five new slots to `include/autoc/nn/nn_inputs.h` in **both** `PathgenInput`/`NNInputs` and `TrackerInput`/`TrackerInputs`, in the order given by [contracts/nn-input-layout.md](contracts/nn-input-layout.md): `IN_ENVELOPE`, `ENVELOPE_SECS`, `ACCEL_X`, `ACCEL_Y`, `ACCEL_Z`. Place `ACCEL_*` **adjacent to `GYRO_*`** so the 6-DOF inertial block is visible. Annotate each `// raw-ok: NN-byte-format buffer`.
- [X] T030 [US2] Add the matching rows to `kPathgenInputMeta` and `kTrackerInputMeta` (name, short name, width). The existing `static_assert` on table length vs `COUNT` enforces this — do not weaken it.
- [X] T031 [US2] Update counts and recompute weight-count `static_assert`s in `include/autoc/nn/topology.h`: `NN_INPUT_COUNT` 37→42, `TRACKER_NN_INPUT_COUNT` 58→63, `TRACKER_NN_TOPOLOGY_STRING` → `"63,32,16r,<out>"`. **Recompute**, never relax.
- [X] T032 [P] [US2] Add `kAccelScale_g` alongside `kCruiseSpeed_mps` / `kDistToBoundaryScale_m` in `include/autoc/nn/nn_inputs.h`, with the rationale comment (±11 g observed must land in a tanh-friendly range).
- [X] T033 [US2] Add config knobs via the `AUTOC_CONFIG_FIELDS(X)` X-macro in `include/autoc/util/config.h`: `EnableEnvelopeInputs`, `EnableAccelInputs`, `AccelScaleG`, and the reserved M2 estimator knobs `EnvelopeSpanLo` / `EnvelopeSpanHi` / `EnvelopeCentroidRadius`. Per Constitution VII, no in-class default initializers for constructor-supplied members.
- [X] T034 [P] [US2] Update the field-count assertion in `tests/contract_config_tests.cc` for the new knobs.

### The step score moves into the tick loop (FR-018a — the single-source-of-truth refactor)

- [X] T035 [US2] Move the per-tick step-score / streak computation out of post-hoc `computeScenarioScores` in `src/eval/fitness_decomposition.cc` into the eval tick path, and record the per-tick result into the tick record. One computation feeds **both** the NN input gather and the fitness accumulation.
- [X] T036 [US2] ⚠️ **Prove DETERMINISM and characterise the delta** in `tests/fitness_decomposition_tests.cc`. *(Reframed by the operator 2026-08-10 — see spec.md § Clarifications "what bit-identical actually has to mean". The original wording said **bit-identical**; the real gate is determinism plus "materially the same or better".)*
  Three obligations, in priority order:
  1. **Determinism** — the same genome and scenario scored twice give exactly the same number, and the move introduces no worker-order or FP-associativity sensitivity. This one IS an equality assertion.
  2. **Materially the same or better** — the inline result is compared against the post-hoc one and the delta is *measured and reported*, not asserted to be zero. Post-hoc reads float-rounded `AircraftState` (`gp_scalar` is `float`); inline reads live state, so a last-bits difference is expected and acceptable. A changed *objective* is not.
  3. **Silence is the failure mode, not difference.** What this task prevents is a rounding change nobody notices. Measuring and reporting satisfies that; asserting `==` and then loosening the tolerance when it fails does not.
  ⚠️ Unchanged: Constitution IV, the pre-run build gate, and the eval-vs-training bitwise regression gate.
- [X] T037 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `src/nn/evaluator.cc` `gather_inputs` from the tick-loop step score. Accumulator is an **external counter** in the stepper, resets **on envelope exit only** (not on regime change), millisecond-based against `FitStreakRampSec`, **linear** normalization — no log, no tanh.
- [X] T038 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `gather_tracker_inputs` from the M2 direct-perception estimator: both beacons CEP-visible AND separation within `[EnvelopeSpanLo, EnvelopeSpanHi]` AND pair centroid within `EnvelopeCentroidRadius`. Same accumulator mechanics as M1 — only the flag's source differs.
- [X] T039 [US2] ✅ **Sign convention SETTLED 2026-08-11 — `specific_force.h` is correct as written, NO flip (see the RESUME block). FRD level flight ⇒ `ACCEL_Z = −1`.** Plumbing is decided — store on `AircraftState` beside `gyroRates_`, computed worker-side; both gathers only COPY. Populate `ACCEL_*` in both gather functions as **body-frame specific force including gravity**: `R(quat)ᵀ · (a_world − g_world) / kAccelScale_g`. ⚠️ **Not FDM kinematic acceleration** — that would put a constant ~1 g offset in the most load-relevant axis, invisible in sim and wrong in flight.
- [X] T040 [US2] Add input-semantics tests in `tests/nn_evaluator_tests.cc`: (a) **steady level flight → normal channel ≈1 g, not ≈0** (the test that catches the kinematic-vs-specific error — and **empirically confirmed on hardware**: the bench table in `docs/COORDINATE_CONVENTIONS.md` reads `+2050` on the normal axis in level attitude — blackbox `accSmooth` counts, `acc_1G ≈ 2048`, so `≈ +1.0 g` — meaning INAV already reports specific force including gravity, exactly the sim semantics required); (b) documented sign on a pull-up; (c) `IN_ENVELOPE` ∈ {0,1}; (d) `ENVELOPE_SECS` monotone within a streak, 0 immediately after exit, saturates at 1; (e) `ENVELOPE_SECS` identical for a given wall-clock duration at two cadences; (f) M1 `IN_ENVELOPE` agrees tick-for-tick with the objective's own threshold decision.
- [X] T041 [P] [US2] Update layout assertions in `tests/contract_evaluator_tests.cc` for both modes' new counts.

### Camera model (M2-only, fidelity to ordered hardware)

- [X] T041a [US2] Set `CameraPixelsV = 240 → 200` in `autoc-tracker.ini`, `autoc-eval-tracker.ini`, and `autoc-eval-tracker-visual.ini`, giving V = 200 × 0.375 = **75°** (H unchanged at 120°). Leave `CameraDegPerPixel` alone so `radPerPx`, per-pixel quantisation and CEP are untouched. Rationale in the ini comment: the ordered 1.8 mm fisheye on OV9281 estimates ~124°×78° equidistant, 120×75 is the conservative split, and 320:200 = 1.6 matches the real 1280×800 aspect (the prior 240 px was a 4:3 invention, optimistic by 15° vertically). ⚠️ **Fitness-affecting** → A1 bundle only.
- [X] T041d [US2] ✅ **DONE 2026-08-16 — supersedes the pitch half of T041a.** Set `CameraDegPerPixel = 0.375 → 0.304` in `autoc-tracker.ini`, `autoc-eval-tracker.ini` and `autoc-eval-tracker-visual.ini`, plus the two struct defaults (`CameraConfig::deg_per_px`, `AutocConfig::cameraDegPerPixel`) — five definitions of one value, the E1 hazard in `index-coupling-inventory.md`. Grid unchanged at 320 × 200 ⇒ **97.3° H × 60.8° V**. **Trigger**: T041a explicitly reserved this knob for measurement (*"revisit from MEASUREMENT (the real grid image, an 031 deliverable when hardware lands), not from a second estimate"*), and 031's ruled-mat calibration of the 1.8 mm fisheye on the OV9281 landed on the `031-beacon-camera` merge — equidistant **confirmed** (no action; the sim has been f·θ since 038 t9), native pitch **0.076°/px**, FOV **95° H × 61° V** by tape. 4 × 0.076 = 0.304 makes the sim grid a true 4× bin of the real 1280 × 800. ⚠️ The estimate was wrong in the **narrow** direction, ~19% on both axes — the single-fisheye-at-120° assumption is retired for this lens. Consequences: CEP/quantisation ~19% **finer** (separation crossover 27.75 → 34.25 m); field ~19% narrower, which is the **fitness-affecting** half → A1 bundle with the rest, never mid-comparison. Comment fallout fixed at the same time in `camera_projection.h`, `derived_features.h`, `signal_model.cc`, `camera_projection.cc` and the ini `SeparationMinResolvablePx` note.
- [X] T041f [US2] ↩️ **PROPOSED then REVERTED 2026-08-16 — the grid stays 320×200.** T041f briefly set `CameraPixelsH = 320 → 312` so the DERIVED field landed on 031's tape-measured 95° H instead of the f·θ extrapolation's 97.3°. **Operator reversed it, and the reasoning is better**: *"the actual dump from camera will be 320x200 — we know that — so seems we really should go with this and assume some slight measurement errors."* The sensor dumps 320×200 (a 4× bin of 1280×800); the pixel count is the one quantity here known **exactly**, and bending it to make a derived angle match a tape reading models a sensor that does not exist. The residual ~2.4° is absorbed as **tape measurement error** — the calibration itself only claims agreement within 3°.
  **Kept from the exercise** (the reversal did not undo these):
  1. The **pod-nose obstruction sweep**, now recorded in the test: onset between 46.2° and 46.8° half-H, then 0.214% @ 47.42°, **0.628% @ 48.64° (shipped)**, 2.57% @ 54.72°. The blockage sits at the OUTER EDGE of the horizontal field, so it is steeply field-dependent — one degree moves it ~2×.
  2. ⚠️ **`EffectiveFieldDiffersFromNominalByAJustifiedAmount` passes by a HAIR** at the shipped grid: 0.628% against a 0.5% floor. That floor and its "~3% hard-blocked" comment both date from the retired 120° era. The comment now says so, and says to set any future bound from the sweep rather than by nudging the floor.
  3. `CameraPixelsH/V`, `CameraDegPerPixel`, `CameraDetectionRangeM` added to the train/eval ini-agreement guard — M2-only keys, skipped on M1 pairs, but a tracker mismatch would otherwise evaluate a policy against a different optic than it trained on.
  **Net config change: none.** 320 × 200 @ 0.304 °/px ⇒ 97.3° H × 60.8° V, as T041d left it. 46/46 suites green.
- [X] T041e [US2] ✅ **DONE 2026-08-16.** Re-derive the projection tests against the measured field — **46/46 suites green**. Four broke, each a literal that had silently encoded the retired 60° half-field, and all four are now **computed from the config** so they follow the one knob (FR-003) rather than needing a human to re-evaluate a tangent: the ±FOV-edge target positions (were `±17.32` = 10·tan 60°), `SeparationIsInvariant`'s outer sweep sample (was a literal 40°, whose ψ/2 pushed the pair to 50° — outside the new field, so the pair gated and the teeth-guard silently disarmed), and `BearingIsQuantisedToPixelCentres`' ±50° sweep. ⚠️ One finding worth keeping: `FieldOfViewIsDerivedFromGridAndPixelPitch` used `EXPECT_DOUBLE_EQ` between the accessor's **float** multiply and the test's **double** one. That passed exactly only because 0.375 = 3/8 and 320 × 0.375 = 120 are both exactly representable in binary — luck, not a property. 0.304 is a repeating binary fraction, so the two now differ by ~2e-6; the check compares at float precision. Also widened the separation-crossover sweep 40 → 60 m: the crossover scales as 1/`deg_per_px`, and at 34.25 m the old start had under 6 m of headroom, past which the loop would have reported **its own start range** as the answer instead of failing.
- [X] T041b [P] [US2] Update the derived-FOV assertion comment in `include/autoc/eval/camera_projection.h` (currently cites ±0.785 rad / 45° for V) so the documented half-angles match the grid, and add/extend a test asserting derived V = 75° and derived H = 120° from the configured grid — the FR-003 "field and resolution cannot disagree" property.
- [X] T041c [P] [US2] Record in `specs/041-m2-depth/artifacts/README.md` that the projection is **already equidistant** (`camera_projection.cc:158-184`, since 040 T031) and that `CameraDetectionRangeM = 100.0` is now **independently corroborated** by the 031 photon budget (bright-day post-correlation SNR ≈22 @100 m, ÷4 at 4×4 defocus → ≈5.5 vs ×4.5 threshold). Neither is a change; both are facts a later reader will otherwise re-derive.

### Recording changes

- [X] T042 [US2] ⚠️ **IMPLEMENTATION ALREADY DONE — verified 2026-08-11. The task premise is STALE.** It says the field is "serialized-but-never-set, zero in every dmp"; that was true when 041 was specced but was fixed at **038 P0-D-3** (`e6108cc`), and `inputdev_autoc.cpp` has set it from `eom01->getLastLocalAirmass()` (NED ft/s → m/s, NaN-guarded) ever since. **Do not re-implement it.**
  Outstanding: **only the test** — that a non-zero wind survives to the dmp. Note the honest limit of a unit test here: recording is a worker-side effect, so unit level can prove the field round-trips through serialization, and the end-to-end "a run in wind records wind" claim is closed by the **T061 smoke inspection**, not by ctest.
- [X] T043 [US2] ⚠️ **MOSTLY DONE ALREADY — verified 2026-08-11. Scope is much smaller than this text implies.** The 038 P0-D-2 work landed `RecordedRunConfig` (`protocol.h:432`), serialized it into `EvalResults` (`:583`), and flipped `dmp_dump.cc` to read it with **no ConfigManager fallback** (`:655-660`). Do not redo any of that. What actually remains:
  1. **`tools/renderer.cc` is only half-flipped.** It reads `runConfig` at `:3313` but still reads the **live ini** at `:4651` (`ConfigManager::getConfig().fitStreakMultiplierMax`, the HUD streak-colour scale). Display-only, but it is precisely the "reader with a drifted ini" case this task exists to close, and a half-flipped reader is worse than an unflipped one because the inconsistency is invisible.
  2. **Add 041's knobs to `RecordedRunConfig`** — `enableEnvelopeInputs`, `enableAccelInputs`, `accelScaleG`, `envelopeSpanLo/Hi`, `envelopeCentroidRadius`. Without them a dmp cannot say whether its `ACCEL_*` columns were populated or ablated, which is exactly the question the T068 matrix asks of it.
  3. **The test**: a reader with a deliberately-drifted ini still replays the recorded numbers.
- [X] T044 [US2] Bump the `EvalResults` version field and implement **fail-loud** reads naming both the artifact and reader versions (Constitution V; research.md R6). No migration path, no shim. Add a test that a prior-version artifact errors with both numbers and does **not** crash in the allocator — the 038 baseline currently dies as `vector::_M_default_append`, which is exactly the diagnosis this prevents.
- [X] T044a [US2] ⚠️ **IMPLEMENTATION ALREADY DONE — verified 2026-08-11. The task premise is STALE.** `SimStateHandler::getSimulationTimeSinceReset()` already returns `llround(sim_steps * Global::dt * 1000.0)` (**038 P0-D-1**, `e6108cc` — the same commit as T042), replacing the truncation that produced 49/50/51 ms jitter. **Do not re-implement it.**
  **Two consequences, both worth stating because they change the plan:**
  1. ⛔ **The `[OP]` marking and the "determinism-affecting / pointer-bump the submodule first" warning NO LONGER APPLY to this task.** There is no determinism-affecting change owed here. An earlier 2026-08-11 note claimed T044a was still pending and would land *between* two runs of the bitwise gate — that was wrong, and it made the gate look more fragile than it is.
  2. The consumer-side invariant is already enforced **fail-loud at runtime**: `crrcsim_tracker_helper.cpp::initScenario` throws when the first source gap ≠ `SIM_TIME_STEP_MSEC`, and `tracker_stepper.cc` mirrors it.
  Outstanding: **only the test** asserting exact gaps.

### Land it

- [ ] T045 [US2] [OP] **THE SINGLE COMMIT** — everything from Phase 3 and Phase 4 together (FR-005).
  ⛔ **Preconditions**: **T046** (`nn2cpp` regenerated) and **T047** (`sim_response.py` parser) must be done *first* — this gate builds xiao, which compiles the generated forward pass, so a stale generated file against the new input count either fails to build or builds something wrong. Also **T011a** must be done (see Phase 2), since this commit makes the pre-break comparators unreadable.
  Gates, all of which must pass: `bash scripts/rebuild.sh` green; `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green; Constitution VI audit clean on touched paths (`grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'`). Nothing lands after this until the M1 bake completes.
- [ ] T046 [US2] Regenerate `xiao/src/generated/nn_program_generated.cpp` via `tools/nn2cpp.cc` for the new layout, and add a parity test that the generated forward pass matches the desktop forward pass on a fixed input vector.
  ⛔ **ORDERING TRAP found 2026-08-11 — read before starting.** `nn2cpp -i FILE` requires an **NN01 weight file**, i.e. a *trained genome*. The current generated file is baked from the **pinned prior M1** (`generatedNNProgramSource` = `…2026-07-06T01:35:46.579Z/gen9200.dmp.zst`, `generatedNNInputCount = 37`). That genome is 37-input and **cannot be loaded by a 041 binary** (FR-011c), and **no 42-input genome exists until the new M1 bake lands at T065.** So T046 as literally written cannot be done before T045.
  **Resolution — T046 splits in two:**
  1. **T046a, before T045**: regenerate from a **placeholder 42-input genome** at the new topology (Xavier-init or zeroed — `nn_xavier_init` exists). Its only job is to make the xiao compile and the parity test run against the *new layout*. It is **not flyable** and must say so at the top of the generated file.
  2. **T046b, after T065**: regenerate from the **real new M1 weights** archived at T065. This is the one that flies, and it gates T078/T080.
  ⚠️ **Second, worse gap — the layout mismatch is currently SILENT.** The generated forward pass hardcodes `for (i = 0; i < 37; i++)` and fixed `nn_weights + offset` slices, while the gather now fills **42** floats. That combination **compiles cleanly** and quietly reads the first 37 slots against weights trained for a different layout — precisely the "builds something wrong" case T045's precondition warns about. `generatedNNInputCount` is exported but **never checked against `PathgenInput::COUNT`**. Add that assertion as part of this task (compile-time in the generated file, or a boot-assert in firmware): it converts an invisible wrong-flight into a loud failure, which is the whole point of US1.
- [X] T047 [P] [US2] ⛔ **INVERTED — do NOT update `specs/019-improved-crrcsim/sim_response.py`. Verified 2026-08-10.** The task assumed `data.dat` is a live output. It is not: **035 FR-P05 retired the per-step writer** (`src/autoc.cc:1969` "data.dat output file retired; the dmp (S3) is the single training trace"), and nothing in the tree writes one. Operator 2026-08-10: *"data.dat is gone from being generated, but the content in s3 remains."*
  So `sim_response.py` is a **reader of historical artifacts only**, and its documented field map is frozen at the **021-era** layout (`F0..F49`, 33-input era — pre-038's four arena inputs, let alone 041's five). Teaching it the 041 column set would make it mis-parse every file it can still be pointed at, which is the exact opposite of the intent. It is also a pre-035 script under [[feedback_historical_scripts_immutable]].
  **Consequence for ordering**: hard constraint 6 reduces to **T046 only**.
- [ ] T048 [US2] [OP] If `CMakeLists.txt` was touched by this phase, run a clean `bash scripts/rebuild-perf.sh` (Constitution IV — an incremental reconfigure can leave stale link state and miss test registration).

**Checkpoint**: one commit, one owed rebake. All three build targets green.

---

## Phase 5: US3 — Instruments that can answer the questions (P1)

**Goal**: build the measurement tools before spending compute.
**Independent test**: empty-mask ablation reproduces baseline fitness exactly; Study A reports per-regime load
from a pinned run with no recording change (SC-004, SC-005).

### The ablation tool

- [ ] T049 [US3] ⚠️ **PARTIALLY DONE 2026-08-13 — the MASKING ENGINE ships, the standalone binary does not.** What exists and is validated (`specs/041-m2-depth/ablation/instrument-validation.md`): the mask is plumbed `WorkerInit.nnInputMask` → worker → `NNControllerBackend::setInputMask`, applied after the gather and immediately before the forward pass in BOTH modes; slot-name resolution lives in `include/autoc/nn/input_mask.h` against the existing metadata tables, with a hard error listing valid names; `autoc --zero-input NAMES` drives it, and `scripts/train.sh` forwards extra args so an ablation runs through the Constitution IX detached path. Empty-mask identity, determinism, degrade-on-known-critical, wrong-length and off-by-one are all tested (11 tests, `tests/input_mask_tests.cc`). **Still owed**: the `tools/nn_ablate.cc` front-end that runs baseline+ablated and emits the T051 report. Original text: Create `tools/nn_ablate.cc` per [contracts/ablation-cli.md](contracts/ablation-cli.md): `-i <ini> --genome <dmp-key|weights-file> [--zero-input NAME[,...]] [--out csv]`. Slot names resolve against `kPathgenInputMeta` / `kTrackerInputMeta` — **no new naming infrastructure needed**, it already exists. Masked columns forced to 0.0 every tick after gathering, before the forward pass. Unrecognised slot name is a **hard error** listing the valid set, never a silent no-op.
- [ ] T050 [US3] Register the `nn_ablate` target and its test in `CMakeLists.txt`.
- [ ] T051 [US3] Report fields per the contract: Δfitness; per-axis Δ`dCtrl` / Δ`⟨|u|⟩`; Δ`pctInStreak` / Δ`avgMaxStreak`; Δ peak and mean normal load; per-scenario Δ distribution; and **per-regime breakdown** (`{tracking, intercept, patrol}`) — required, not optional (FR-011a), because the hypothesis predicts a signal in *one* regime and pooling would hide it.
- [X] T052 [US3] ✅ **DONE as `tests/input_mask_tests.cc` + the end-to-end run** (empty-mask identity verified against the real eval, not a stub: `-1045.136851` both sides). Original text: Add `tests/nn_ablate_tests.cc`: **empty-mask identity** — with no mask, reproduces the source run's fitness **exactly** (SC-004; a tool that quietly perturbs the eval path makes every finding worthless); unknown slot name errors with the valid list; masking a known-critical input degrades measurably; two identical invocations are bit-identical.
- [ ] T053 [US3] [OP] Clean `bash scripts/rebuild-perf.sh` for the `CMakeLists.txt` touch (Constitution IV).

### Study A — regime and load (report-only; no load axis in 041)

- [X] T054 [US3] Create `src/analytics/regime_load_study.py`: classify every tick into `{tracking, intercept, patrol}` using the **existing** rule (`stpPt ≥ 0.5`; below that, smoothed `d(dist)/dt < 0` is intercept, else patrol) — reuse `dynamics_progress.py:74-80`'s definition rather than writing a new one, so numbers stay comparable with every prior report.
- [X] T055 [US3] Report per regime, per axis: `dCtrl`, `⟨|u|⟩`, and load distribution **plus peak** (peak is the damage-relevant statistic; a mean hides ±11 g excursions entirely). Emit machine-readable CSV alongside any plot. State sample sizes and any excluded ticks.
  ⚠️ **Load comes from the recorded NN input column `ACCEL_Z`, NOT from `physicsTrace`** (spec.md Clarifications, session 2026-08-10). `nz_g = −ACCEL_Z × kAccelScale_g`, the same line flight uses. The physics trace is capped at 175 ms per scenario (0.89% of ticks) and is staying that way. Consequence: **load exists only for runs baked after T039** — the two pre-break comparators have no usable load at all, so T058 reports their CONTROL half only.
- [X] T056 [US3] Add the H2 test to the study: within each regime, does pitch/roll `dCtrl` predict throttle level and load? Report correlation with a stated confidence, not a scatter plot alone.
- [ ] T057 [US3] ⏳ **IMPLEMENTED 2026-08-13, NOT YET RUN** — the code is in `regime_load_study.py` and emits `<label>-autocorr.csv`, but it needs `ACCEL_*` and so cannot execute until the new M1 (T071). Emitted empty for both pre-break comparators, correctly. Add normal-load **autocorrelation at the history lags** to the study — ⚠️ runs on the **new M1** (T071), not on the pre-break CSVs, which carry no load. Sampling is 20 Hz by decision, so the shortest observable lag is 50 ms — this is the cheap evidence that decides whether the accel channels ever need temporal depth (research.md R1 fallback ladder). Strong autocorrelation at 50–100 ms ⇒ one instantaneous sample suffices and neither fallback is warranted.
- [X] T058 [US3] Run Study A's **control half** on the **pre-break CSVs already archived at T011a** (`artifacts/pre-break/*.csv.gz`, extracted 2026-08-10), covering the pinned prior M1 and 040-t4, producing `specs/041-m2-depth/study-a/`. Per-regime `dCtrl` / `⟨|u|⟩` and the regime classification: 100% tick coverage, fully available. This is the **prior M1's per-regime profile**, the only obtainable form of that baseline (FR-011c: its 37-input genome cannot be loaded by a 041 binary, so it can never be re-evaluated or ablated).
  ⚠️ **No load half here.** The prior M1 predates `ACCEL_*` and its physics trace covers 0.89% of ticks, so old-vs-new load comparison is impossible — accepted by the operator 2026-08-10 ("old M1 is what it is"). Every 041 load number is a single-run profile of the NEW M1 (T071), never a delta. ⚠️ Do **not** plan to re-extract from S3: after T044 these comparators are unreadable by a 041 binary.

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

- [X] T062 [US4] [OP] ⚠️ **REFRAMED by the operator 2026-08-10 — this is not a retry-budget task.** Asked how many attempts to allow before re-drawing, the answer was: *"a good bake is good enough — forget the random as we aren't quite at the brittle edge of NN capacity. Often the early runs uncover missed assumptions or non determinism — that's the key search."*
  So: **no fixed attempt count, and no lottery framing.** Run until a bake is good; a good bake is good enough. The important correction is what an early failure MEANS — it is a **diagnostic**, not bad luck. The first runs are the cheapest place to discover a missed assumption or a non-determinism, and treating a bad one as "re-draw and hope" throws away the signal we are actually there to collect. R8's 3-attempt proposal is superseded.
  The **abort signature** below still stands, but as an *investigate* trigger rather than a *re-draw* trigger: abort on the stuck-basin signature — throttle amplitude exactly 1.000 with **σ = 0.000** and `dCtrl` 0.000, plus `avgMaxStreak` frozen and best-sigma not annealing past ~0.14 by gen 200. Note that throttle *saturation* (mean ≈0.85, σ>0) is **not** the tell; climbers pass through it.
- [ ] T063 [US4] [OP] Pre-run build gate, then `scripts/train.sh autoc.ini logs/autoc-041-t1-m1-envelope.log` (pop 8000 / 49 winds). Judge climb on `pctInStreak` / `avgMaxStreak`, **not** completions.
- [ ] T064 [US4] [OP] Repeat as a lottery re-draw if the abort criterion fires, within the declared budget, incrementing the artifact index (`t2`, `t3`) per `autoc-<feature>-t<N>-<details>`.
- [ ] T065 [US4] [OP] Immediately on completion: tag all dmp objects `retain=keep`, and archive `nn_weights*.dat` beside the dmp (FR-010 — the dmp preserves numbers, only NN01 preserves a controller you can re-fly). Record the S3 prefix in the outcome doc. ⚠️ **The `data.dat` snapshot FR-022 asks for is MOOT** — 035 FR-P05 retired the writer, so there is no per-tick file for "the next launch" to overwrite. The dmp IS the trace, and `retain=keep` is what preserves it. FR-022 is satisfied by the pinning, not by a file copy.

### Reads

- [ ] T066 [US4] Generate the report set: `scripts/generate_pngs.sh m1 logs/autoc-041-t1-m1-envelope.log`.
- [ ] T067 [US4] Assess non-regression against **the pinned prior M1 alone** (`autoc-m1/…2026-07-06T01:35:46.579Z/`), with a **loose bar**: the trigger is *materially worse*, not a numeric band. ⚠️ **Operator decision 2026-08-10, superseding research.md R7's "historical band".** Reasons, both worth keeping: we have **one** pinned comparator, so a "band" would be a spread invented from a single sample; and per the T062 reframing the early runs exist to surface missed assumptions, which a noise-triggered bar actively works against. Same spirit as T036 — measure, report, and judge materially, rather than assert a threshold that gets loosened when it fires. Compare on: `pctInStreak` / `avgMaxStreak` within noise of or above the band; crash/OOB counts not worse; per-axis `dCtrl` / `⟨|u|⟩` per regime; peak load not higher. ⚠️ Absolute fitness sums are **not** comparable across runs with different scenario counts — use per-scenario or per-step rates.
- [ ] T068 [US4] Run the **ablation matrix** on the new elite: flag alone (`--zero-input IN_ENVELOPE`), duration alone (`ENVELOPE_SECS`), both, and the accel channels — per FR-014b. Expectation is that the **flag** carries the effect; a duration-only effect would be distinct and more surprising.
- [ ] T069 [US4] Run the **control-input ablations** for calibration (FR-011b): `DIST_NOW` (known-critical end), `GYRO_P,GYRO_Q,GYRO_R`, `INWARD_BODY_X,INWARD_BODY_Y,INWARD_BODY_Z` (plausibly marginal). State the envelope verdict as a **position on this spectrum**, never against an assumed absolute threshold.
- [ ] T070 [US4] Record the H1a verdict as **pass** (fitness drop **and** behavioural shift, beyond the marginal end of the control spectrum), **partial** (one but not both), or **fail** (neither) — per regime. All three close the hypothesis; only an unclassifiable result fails SC-007.
- [ ] T071 [US4] Run Study A on the new elite and produce the **per-regime intent comparison** against the prior M1's profile from T058 (SC-007a). Per-axis aggressiveness is comparable across the two runs; **peak load per regime** (SC-008) is **new-M1-only** — there is no prior-M1 load to compare against, and peak is bounded by the 20 Hz sampling decision (a between-tick structural peak is not observed). Report as a **ballpark read**, not an attributable effect size — it is a cross-run profile comparison, not a controlled delta.

### Hardware deployment (conditional on the M1 result)

> ⚠️ **Standing hardware procedure — applies to EVERY INAV flash in this feature (T001, T072, T078, T080)**:
> **remove the GPS before flashing the INAV controller.** Known quirk, not up for debate. Forgetting it costs
> a debugging session, so it belongs in the runbook rather than in somebody's memory.

**This sub-phase is where every gear-attached task lives** — both INAV targets *and* the xiao board work
(T074, T075, T078). Nothing before this point requires hardware; the xiao tasks outside this phase (T002,
T046, T045's gate) are host compiles only.

**INAV bring-up** (moved here from Phase 1, 2026-08-10 — operator is near the gear at this point). Trigger:
the T067 non-regression read looks decent. Do this **before** T072, so the baseline is known-good before it
is modified.

- [ ] T001 [US4] Confirm the INAV baseline builds in `~/inav` for **both** established targets — **bench = `MAMBAF722_2022A`** (STM32F722; `xiao/inav-bench.cfg`) and **flight = `MATEKF722MINI`** (`xiao/inav-hb1.cfg`), both currently at `63cffaf4`. Routine and precedented: **021 T041 already did exactly this** ("INAV builds for bench (MAMBAF722_2022A) and flight (MATEKF722MINI)", closed), and the commands are recorded in `specs/020-pre-flight-pipeline/plan.md`: `cd ~/inav && mkdir -p build && cd build && cmake .. && make MAMBAF722_2022A`. **Bench first.**
- [ ] T001a [US4] Record the two-variant build/deploy sequence in `specs/041-m2-depth/artifacts/README.md`, pointing at `specs/020-pre-flight-pipeline/plan.md` for the commands rather than restating them. Every later INAV task (T072, T078, T080) builds and flashes **both** targets, bench first — a change validated only on the bench target is not validated for flight.

- [ ] T072 [US4] Extend `MSP2_AUTOC_STATE` in `~/inav/src/main/fc/fc_msp.c` (the `MSP2_INAV_LOCAL_STATE` case) to carry accel in the **same single round trip**. Copy the shape of fork commit `63cffaf4f` ("extend MSP2_AUTOC_STATE with filtered gyro rates"): append at payload end, fixed integer scale stated at the write site, and document the axis/sign convention for the consumer. ⚠️ **Source `acc.accADCf` — the TRANSFORMED field, never a raw sensor read.** `acceleration.c:563-568` applies `applySensorAlignment` then `applyBoardAlignment` then divides by `acc.dev.acc_1G`, so `acc.accADCf` is board-alignment-corrected and **already in g units**. This mirrors `gyro.gyroADCf` (`gyro.c:438-442`, same two alignment calls), which is exactly why the existing gyro extension is correct. **Do NOT use** the file-static `accADC` (`acceleration.c:73`) or `acc.dev.ADCRaw` — those are pre-alignment and would bake in each board's misalignment differently (bench roll = −16 vs flight). Wire encoding: milli-g `int16` = `lrintf(acc.accADCf[axis] * 1000.0f)`, giving ±32 g against ±11 g observed. ⚠️ **The wire carries INAV's native FLU, unflipped** — exactly as the quat and gyro already do. The FLU→FRD y/z flip belongs at T074's msplink boundary, once, beside the other two (settled 2026-08-11). Build **both** targets, bench first; disconnect GPS before flashing.
- [ ] T073 [US4] ✅ **The bench table is fully resolved (2026-08-11) — all three rows are consistent and all three are safe to assert.** INAV's frame is FLU and `accADCf` is proper acceleration; nose-up `+2050` on X is correct, and it was the discriminating row precisely because FLU→FRD shares the x axis. Assert in **FRD** (post-msplink): level `[0,0,−1]`, nose up `[+1,0,0]`, right wing down `[0,−1,0]` — see `docs/COORDINATE_CONVENTIONS.md` → "Accelerometer as an INTERFACE quantity (041)" for the `~/inav` evidence. ⚠️ Board alignment DIFFERS bench vs flight (different mounting), so verify on **both** targets — a bench-only check is not a flight check. Pin the accel axis and sign convention against the **already-measured bench table** in `docs/COORDINATE_CONVENTIONS.md` ("Ground Verification Results, bench 2026-03-30", `MAMBAF722_2022A`, board alignment roll = −16): level → `[~0, ~0, +2050]`; right wing down 90° → `[~0, +2060, ~0]`; nose up 90° → `[+2050, ~0, ~0]`. ⚠️ **Units**: those are **blackbox `accSmooth` counts** (`acc_1G ≈ 2048`), *not* the runtime `acc.accADCf`, which is the same vector already divided by `acc_1G` — i.e. `+2050 counts` ⇔ `+1.0 g`. Convert before comparing. The **axes and signs** transfer directly; only the scale differs. T073 is therefore *match the table*, not derive the convention — add a test asserting all three attitudes. ⚠️ Board alignment differs between bench (roll = −16) and flight, so verify on **both** targets; do not assume one target's result transfers.
- [ ] T074 [US4] Consume the new accel fields in `xiao/src/msplink.cpp` and feed `ACCEL_*` into the input vector with the same specific-force semantics as sim. **The conversion is `accel_FRD = (a[0], −a[1], −a[2])`** — the identical y/z flip already applied to the quat (`inavQuatToAerospaceEB`) and the gyro (`msplink.cpp:964-966`), because INAV's frame is FLU. Put it beside them so the three cannot drift. Level flight must land on `ACCEL_Z ≈ −1`, matching sim.
- [ ] T074a [US4] ⭐ **Bench-observable NN inputs — operator requirement 2026-08-11: "we will definitely want the NN inputs logged in xiao, and in the sim as usual, to help troubleshoot; bench where we move the craft in various directions to confirm hypothesis."** The engaged path already satisfies this and needs nothing: `TickRecord.inputs[kNumInputs]` is *"post-gather values ACTUALLY fed to the NN (honest)"*, so growing `kNumInputs` 37→42 logs `ACCEL_*` + envelope per tick for free.
  **The gap is the bench posture itself.** Moving the craft by hand happens **armed-but-not-engaged**, and in that state only `FlightStateRecord` is written — pos, vel, quat, and **no accel**. So the one test that confirms the convention is the one state that cannot see it. Close it:
  1. Add `int16_t accel_frd[3]` to `FlightStateRecord` (`xiao/include/flight_log_format.h`) — **post-flip aerospace FRD**, the value the NN would receive, same convention as its `quat[4]` (already `q_EB`, post-`neuQuaternionToNed`).
  2. Add `int16_t accel_inav[3]` alongside it — the **raw INAV FLU** value, pre-flip. ⚠️ Not redundant: with only the post-flip number a wrong reading cannot be attributed to the sensor, the wire, or the flip. With both, the bench says *which step* is wrong. The operator's standing constraint is that this must be right on the **first** flight, and one extra `int16[3]` is the cheapest possible way to make the failure legible.
  3. **Bump `kFormatVersion` 3 → 4** and update the decoder (Constitution V — every decoder loud-fails on unknown version).
  **Bench acceptance** (the hypothesis under test, settled 2026-08-11): hold the craft level → `accel_frd ≈ [0, 0, −1]`; nose up 90° → `[+1, 0, 0]`; right wing down 90° → `[0, −1, 0]`; and `accel_inav` shows the un-flipped FLU counterparts `[0,0,+1]`, `[+1,0,0]`, `[0,+1,0]`. Verify on **both** targets — board alignment differs (bench roll = −16, flight roll = 1700 / yaw = 900).
  **Sim side needs no work**: `AircraftState::nnInputs_` already records the full input vector per tick, so `ACCEL_*` and the envelope slots reach the dmp as a consequence of being inputs, and `dmp-dump` labels them from the metadata short names. Confirm the columns appear at T061 rather than building anything.
- [ ] T075 [US4] Implement `IN_ENVELOPE` / `ENVELOPE_SECS` on-target in `xiao/src/`: the step-score cone geometry (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`) thresholded at `FitStreakThreshold`, plus the duration accumulator with a **reset on engage** as well as on envelope exit (FR-022a). This is firmware work, not codegen.
- [ ] T076 [US4] Verify the added payload does not push the MSP cycle past its loop budget. 039 measured zero overruns at 115200 with the prior payload; if headroom is marginal, the unexercised 460800 baud-raise lever is the documented next step rather than dropping the field.
- [ ] T077 [US4] Decide explicitly whether the queued `mspOverrideInit` first-frame patch (backlog C1 — MSPRCOVERRIDE engage pays a spurious 200 ms floor) rides along, since INAV is being built and flashed anyway. Record the decision either way — this is the same "now is the time" logic as the format break, and the window closes when the flash does.
- [ ] T078 [US4] [OP] Bench parity before flight (SC-011a): generated forward pass reproduces the desktop forward pass on a fixed input vector, and the new inputs read sane values on-target against a known geometry. **Includes the T074a attitude sweep** — move the craft through level / nose-up / right-wing-down and read `accel_frd` + `accel_inav` back out of the log, on **both** targets. This is the empirical confirmation of the 2026-08-11 convention finding; do not treat the desk derivation as sufficient on its own.
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
  Cross-check against the physics: [camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md) §3 predicts a 3 g target exits a ±36° half-field in ~1.1 s @50 m / ~1.5 s @100 m — so expect the relevant timescale to be **order 1 s, not sub-second**. A measured distribution far from that wants explaining before it is trusted. ⚠️ **OPERATOR DECISION 2026-08-16 — report this as a STATED LOWER BOUND.** The pre-break CSVs were captured at **120° × 90°**; the measured flight lens is **95° × 61°** (T041d), roughly HALF the solid angle. Every gap frequency and duration here is therefore **optimistic** versus the real optic — the predictor's measured value is a floor, not an estimate. Say so in the deliverable, because it feeds a hardware purchase and a floor argues for the lens differently than a point estimate does. Chosen over re-measuring at the new FOV, which would have blocked the lens decision behind a full run. Refreshing it against the A1 run later is cheap and stays available.
- [ ] T083 [US5] Fit an offline regressor for **target (a) — the continuous current-bearing estimate**: from the perception history window, predict the target's bearing *now*, including through blind ticks. Score as **r² against hold-last-seen**, **binned by seconds since last truth**, with per-bin sample counts. ⚠️ A pooled number is misleading by construction — on visible ticks the truth is an input, so both head and baseline are near-perfect.
- [ ] T084 [P] [US5] Fit target **(b) — Δspan at 50/100/150 ms** as the *control* that confirms why the old head failed. Confounded until the pairing is correct; T022 satisfies that structurally.
- [ ] T085 [P] [US5] Fit target **(c) — discounted future `stepPoints`** vs a constant-mean baseline (the value-head fallback).
- [ ] T086 [US5] Apply the go/no-go rule (FR-024a): the head must beat hold-last-seen **in the gap-age bins where real excursions occur**, per T082's measured distribution, by a margin stated in advance, with adequate per-bin samples. Bound the claim by two physical facts (FR-030, FR-031): the drift budget Δθ ≈ ½·a·t²/r + 1–2° IMU error sets what hold-last-seen's error *should* look like as a gap ages, and **warm code relock has a ≈155 ms floor** — so the predictor's value is in **pointing, not latency**, and any claimed reduction in time-to-reacquire below 155 ms is unphysical. ⚠️ The verdict feeds a hardware decision (which lens gets bought), so record it with its evidence rather than as a verdict alone.
- [X] T087 [US5] ⛔ **DROPPED — operator decision 2026-08-10. Do not run E1.** T086 settles the head's fate on offline evidence, and T088 then re-targets or retires it; either way the "dead head" configuration E1 exists to measure **stops existing**, so its answer cannot change anything downstream. Removes one tracker run from the plan. *(Original task text kept below for provenance.)* ~~is E1 (`EnablePredictorHead` 0 vs 1, short tracker runs) still worth a run, given that T086 decides the head's fate anyway? If the head is being retired or re-targeted regardless, E1's question ("is the dead head taxing the search") is moot and this task should be **dropped**. Flagged outstanding at the second clarify pass; resolving it may remove a tracker run from the plan.~~
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
- [ ] T098 [US6] [OP] Pin the M2 run `retain=keep` and archive its weights (FR-010). ⚠️ No `data.dat` snapshot — retired at 035 FR-P05; see T065.

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
Phase 1 (setup — desktop only; INAV bring-up deferred to Phase 6)
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

1. **Phase 1–2** stops config edits failing spuriously and makes load readable from existing runs. The INAV
   unknown is deliberately **not** retired here — it moved to Phase 6, where the operator is at the bench
   anyway, and it blocks nothing before then.
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
| T087 E1 decision | one tracker run — **exercised 2026-08-10: dropped, run removed** |

**Total**: 110 tasks. US1 17 · US2 24 · US3 10 · US4 24 · US5 8 · US6 10 · setup/foundational/polish 17.
*(US2 grew by 3 at the 2026-08-10 camera-model pass: T041a–T041c. T011a added at the same pass's ordering
review — it is the one task whose omission is unrecoverable. T001/T001a moved from setup into US4's hardware
sub-phase the same day — same IDs, later position; the count is unchanged.)*

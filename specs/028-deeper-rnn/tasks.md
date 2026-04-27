---

description: "028 — Deeper RNN — task list"
---

# Tasks: 028 Deeper RNN (D-alone diagnostic + telemetry-gated escalation)

**Input**: Design documents from [`specs/028-deeper-rnn/`](.)
**Prerequisites**: [plan.md](./plan.md) (required), [spec.md](./spec.md) (required for stories),
[research.md](./research.md), [research_rlayer_placement.md](./research_rlayer_placement.md),
[data-model.md](./data-model.md), [contracts/](./contracts/), [quickstart.md](./quickstart.md)

**Tests**: Included per Constitution Principle I (Testing-First) — telemetry-signal computation
is testable and gets unit tests; existing test files re-enabled or updated as part of marker flips.

**Organization**: Tasks grouped by operator user story so each story is independently testable
and the "first milestone — get something to train on" can land before any training.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Maps task to user story (US1, US2, US3, US4)
- File paths are absolute or repo-rooted

## Path Conventions

- Single project rooted at repo root. C++ sources in `src/`, headers in `include/autoc/`,
  tests in `tests/`, feature docs/scripts in `specs/028-deeper-rnn/`.

---

## Phase 1: Setup

**Purpose**: Verify baseline state before any 028 changes.

- [X] T001 Verify branch `028-deeper-rnn` checked out, working tree clean: `git status` shows no uncommitted changes (the existing `crrcsim` submodule pointer change from session start is acceptable if intentional)
- [X] T002 Run baseline build on cadence7-redux state: `bash scripts/rebuild.sh` → green; `./build/contract_evaluator_tests` passes at 1667 weights (pre-marker-flip baseline) — *(actual test path is `./build/contract_evaluator_tests`, not `./build/tests/...`; doc nit)*
- [X] T003 [P] Create `logs/` directory if missing: `mkdir -p logs`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Test scaffolding that all telemetry tasks need before they can be implemented test-first.

**⚠️ CRITICAL**: No US1 implementation tasks can begin until this phase is complete.

- [X] T004 Create empty test file `tests/nn_telemetry_tests.cc` with GoogleTest boilerplate — *implemented directly with full test bodies (T007–T012), test-first phase combined with scaffolding*
- [X] T005 Register `nn_telemetry_tests` in `CMakeLists.txt` (top-level, not `tests/CMakeLists.txt` — the project has a single CMake file). Also added `src/nn/telemetry.cc` to the `autoc_common` library
- [X] T006 Verify scaffold builds: `make -j8 nn_telemetry_tests` → green; binary runs and reports the six tests passing

**Checkpoint**: Test scaffolding ready. US1 implementation can proceed test-first.

---

## Phase 3: User Story 1 — Telemetry-instrumented D-alone binary ready (Priority: P1) 🎯 MVP

**Goal**: As an operator, I have a built-and-tested binary with two new telemetry signals
(W_hh/W_xh activation ratio + W_hh population CV) wired into the per-generation log, and
recurrence re-enabled at 16-wide layer 2, so I can launch the D-alone diagnostic training run.

**Independent Test**: Run `./build/autoc -c autoc.ini` for ~5 generations on the new binary;
inspect the log for `#NNGen` lines containing the four new fields (`whh_xh_ratio`, `w_xh0_cv`,
`w_xh1_cv`, `w_hh_cv`); render the 6-panel evolution plot and verify it produces a valid PNG.
Per-generation wall-clock within ~5 % of cadence7-redux baseline.

### Tests for User Story 1 (Test-First)

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation. They drive the
> activation-ratio calibration thresholds in [data-model.md §2.3](./data-model.md#23-test-surface).**

- [X] T007 [P] [US1] `WhhXhRatio_ZeroWhh_ReturnsZero` — green
- [X] T008 [P] [US1] `WhhXhRatio_ActiveWhh_ReturnsPositiveRatio` (renamed from "IdentityWhh" — uses W_hh=0.1·I rather than literal identity matrix per recurrent forward-pass conventions). Calibration data point: ratio settles in [0.01, 10] band with equal-scale W_xh and W_hh. Plot threshold 0.10 is a reasonable initial midpoint
- [X] T009 [P] [US1] `WhhXhRatio_NoRecurrentLayer_Sentinel` — uses recurrent={0,0,0,0} flags; ratio == 0.0 sentinel
- [X] T010 [P] [US1] `WhhCv_IdenticalPopulation_Zero` — green
- [X] T011 [P] [US1] `WhhCv_TwoMagnitudes_AnalyticMatch` (renamed from "BimodalPopulation" — bimodal with ±sign-only collapses to identical |w|=1.0; using two magnitudes 0.5 and 1.5 gives non-zero CV that matches analytic prediction CV=0.5)
- [X] T012 [P] [US1] `WhhCv_NoRecurrentLayer_Sentinel` — sentinel decision: **NaN** for w_hh_cv when no recurrent layer (matches data-model.md §1.2 default; plot script omits red trace from legend in this case)

**Verify all six tests FAIL** (red bar) before proceeding to implementation.

### Implementation for User Story 1

- [X] T013 [US1] Optional `RecurrentTelemetry*` parameter added to `nn_forward_recurrent` (default null = no overhead; matches pre-028 behavior). Also added `enableTelemetryCapture()` / `disableTelemetryCapture()` methods + value-held `telemetry_` member to `NNControllerBackend` in [`include/autoc/nn/evaluator.h`](../../include/autoc/nn/evaluator.h) and [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc)
- [X] T014 [US1] Signal-1 accumulation hooked into the recurrent inner loop ([`src/nn/evaluator.cc:198-230`](../../src/nn/evaluator.cc)): `xh_part` and `hh_part` are computed separately; when `telemetry != nullptr`, `|xh_part|` and `|hh_part|` are accumulated for recurrent neurons only. `RecurrentTelemetry::activation_ratio()` returns hh_mean/xh_mean (or 0.0 sentinel)
- [X] T015 [US1] `compute_population_block_stats` in new [`src/nn/telemetry.cc`](../../src/nn/telemetry.cc); per-block CV across population using per-individual aggregate magnitude (per data-model §3.1 rationale). Sentinel = NaN when block absent
- [X] T016 [US1] Synthetic per-best-of-gen ratio capture via `compute_synthetic_activation_ratio` in [`src/nn/telemetry.cc`](../../src/nn/telemetry.cc) — deterministic 10-tick replay with zeroed state and ±0.5 alternating inputs. Captures structural fingerprint of W_hh engagement without RPC plumbing through the eval worker. Called from `autoc.cc` once per generation immediately before the `#NNGen` log emission
- [X] T017 [US1] `#NNGen` log line in [`src/autoc.cc:~1206`](../../src/autoc.cc) extended with four new fields: `whh_xh_ratio`, `w_xh0_cv`, `w_xh1_cv`, `w_hh_cv` — appended after `energy=...`, format `%.4f`
- [X] T018 [US1] `bash scripts/rebuild.sh && ctest` — 12/12 tests pass at 1923 weights including the six new telemetry tests
- [ ] T019 [US1] **OPERATOR-LAUNCHED**: Telemetry-only smoke run. NOTE: with markers 1, 2, 4 flipped per T022–T024, the binary now has `NN_RECURRENT[2]=true` — running it directly is no longer the "no-marker-flip" smoke described originally. Three options for the operator: (a) launch a short ~30-gen run on the *current* state and observe the new fields with `whh_xh_ratio > 0` and non-NaN `w_hh_cv` (different smoke than originally specified, but verifies telemetry on the recurrent build); (b) temporarily revert markers, run the FF smoke, re-flip; (c) skip and proceed directly to T027 D-alone diagnostic. Recommended: option (a) — same telemetry validation, less churn

### Plot extension

- [X] T020 [P] [US1] Created `specs/028-deeper-rnn/plot_evolution_progress.py` — 7-row gridspec layout (5 original panels + ratio + CV split as bottom two rows of "panel 6"). Sentinel handling: whh_xh_ratio all-zero → "no recurrent layer" annotation; w_hh_cv all-NaN → omitted from CV legend. Supports older log lines (017-format pre-028 fields all optional in regex)
- [ ] T021 [P] [US1] **OPERATOR-LAUNCHED**: Validate plot script against the smoke log produced in T019

### Marker flips — re-enable D-simple at 16-wide layer 2

- [X] T022 [US1] Marker 1 flipped: `NN_RECURRENT = {false, false, true, false}` in [`include/autoc/nn/topology.h:52-57`](../../include/autoc/nn/topology.h)
- [X] T023 [US1] Marker 2 flipped: `static_assert(NN_WEIGHT_COUNT == 1923, ...)` in [`include/autoc/nn/topology.h:69`](../../include/autoc/nn/topology.h)
- [X] T024 [US1] Marker 4 flipped: `tests/contract_evaluator_tests.cc:14` now expects 1923 weights, `NN_RECURRENT[2]==true`, `NN_HIDDEN_STATE_COUNT == NN_HIDDEN2_SIZE`
- [X] T025 [US1] Markers 3 + 5 deliberately untouched (D-alone is tracking-only; markers flip in pattern 2 5a if needed)
- [X] T026 [US1] `make -j8 && ctest` — full build green at 1923 weights, **12/12 tests pass** including all six new telemetry tests
- [X] T026b [US1] xiao build green via `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed` — Constitution Principle II satisfied. `nn_program_generated.cpp` stale-snapshot pattern works as designed (xiao internally consistent, no regen during 028 sim work)

**Checkpoint** ✅ **First milestone — something to train on.** Binary is built, telemetry signals
emit, recurrence is re-enabled at 16-wide layer 2, all tests green. Operator can now launch the
D-alone diagnostic.

---

## Phase 4: User Story 2 — D-alone diagnostic run + outcome assessment (Priority: P1)

**Goal**: As an operator, I have run the D-alone diagnostic (recurrent ON, lexicase OFF,
single-seed) and assessed which of four outcome branches the result indicates, per the
[D-alone outcome table](./spec.md#required-prework).

**Independent Test**: A populated `logs/autoc-028-dalone.log` with 400 generations of `#NNGen`
data, regenerated 6-panel evolution plot, regenerated control-aggressiveness plot, and a
documented branch decision in `specs/028-deeper-rnn/dalone_outcome.md`.

### Implementation for User Story 2

- [ ] T027 [US2] Launch D-alone diagnostic training run: `nohup ./build/autoc -c autoc.ini > logs/autoc-028-dalone.log 2>&1 &` (pop 3500, gens 400, single seed)
- [ ] T028 [US2] Set up monitoring loop (separate terminal): regenerate 6-panel plot every 50 gens — `python3 specs/028-deeper-rnn/plot_evolution_progress.py logs/autoc-028-dalone.log --out specs/028-deeper-rnn/dalone_evolution.png`
- [ ] T029 [P] [US2] Set up monitoring for control-aggressiveness PNG every 50 gens — invocation per [quickstart.md Phase 3](./quickstart.md#phase-3--d-alone-diagnostic-run); existing tool from 027
- [ ] T030 [US2] Apply early-stop criteria during run per [plan §Phase 3](./plan.md#phase-3--d-alone-diagnostic-single-seed-recurrent-on-lexicase-off): kill run if best-fitness > pid1's −27045 floor by gen 100 with no descent OR `whh_xh_ratio` flat near zero across gens 50–150 (sustained ≥ 30 gens)
- [ ] T031 [US2] At run completion (or early-stop), capture the late-plateau metrics: read final-gen `#NNGen` line, compute late-plateau `dCtrl` and `<|out|>` from the control-aggressiveness tool's last-50-gens window
- [ ] T032 [US2] Write `specs/028-deeper-rnn/dalone_outcome.md` with: final fitness, dCtrl, `<|out|>`, `whh_xh_ratio` final value, `w_hh_cv` trajectory summary, comparison against [validation gate](./spec.md#validation-gate-carries-from-027-plan), and **branch decision** (Phase 6 win, Phase 5a, Phase 5b, or Phase 5c per [quickstart.md Phase 4 table](./quickstart.md#phase-4--branch-on-d-alone-outcome))

**Checkpoint**: D-alone outcome assessed. One of four next-action paths is documented and ready
to execute as US3 — *or* US2 itself triggers the Phase 6 win path directly.

---

## Phase 5: User Story 3 — Pattern 2 follow-on (Priority: P2, conditional)

**Goal**: As an operator, given the D-alone outcome from US2, I run the appropriate pattern-2
follow-on attempt (5a stability-only lexicase, 5b orthogonal init, or 5c larger budget) per the
revised escalation ladder in [plan §Phase 5](./plan.md#phase-5--pattern-2-only-if-phase-4-indicates).

**⚠️ CONDITIONAL**: This phase ONLY executes if US2's `dalone_outcome.md` does **not** trigger
the Phase 6 win path. Pick the sub-phase (5a / 5b / 5c) the outcome indicates; do **not** run
all three.

**Independent Test**: A populated `logs/autoc-028-pattern2-<5a|5b|5c>.log`, regenerated plots,
and a `specs/028-deeper-rnn/pattern2_outcome.md` with branch decision (Phase 6 win, US4 next
attempt within budget, or 028 close as bounded no-go).

### Pattern 2 — Sub-phase 5a (stability-only lexicase) — only if US2 indicates

- [ ] T033 [US3] Flip CADENCE7-REDUX marker 3 in [`src/eval/selection.cc:66-69`](../../src/eval/selection.cc): uncomment ONLY the `pool.push_back(stability_score)` call; leave the energy push commented (single-axis stability, not 3-axis)
- [ ] T034 [US3] Flip CADENCE7-REDUX marker 5 in [`tests/selection_tests.cc:152+`](../../tests/selection_tests.cc): rename `DISABLED_Selection027v4*` tests back to `Selection027v4*` (stability-axis tests); leave `DISABLED_Selection027*` energy variants disabled
- [ ] T035 [US3] Rebuild + verify: `bash scripts/rebuild.sh && ./build/tests/selection_tests` — `Selection027v4*` tests pass; `Selection027*` energy tests remain disabled
- [ ] T035a [US3] **ε-floor recalibration check** (per [research.md §8.4](./research.md#84-stability-axis-027-v4-score-distribution-mining)): mine stability-score distributions from rnn3's late-gen population (`logs/autoc-027-rnn3*` or wherever 027 logs live). Plot percentiles of per-individual stability-score deltas. If 027's inherited ε=0.5 falls outside the [P25, P75] band of those deltas, derive a re-scaled ε for the stability axis before T036 launches; record the chosen ε and its derivation in `specs/028-deeper-rnn/pattern2_eps_calibration.md`. If the inherited ε is in-band, document that and proceed with ε=0.5.
- [ ] T036 [US3] Launch 5a run: `nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5a.log 2>&1 &` (same pop/gens/seed as D-alone; ε per T035a)
- [ ] T037 [US3] Monitor + plot regen on the same cadence as US2 (T028, T029)

### Pattern 2 — Sub-phase 5b (orthogonal W_hh init) — only if US2 indicates

- [ ] T038 [US3] Replace Xavier init for W_hh in [`src/nn/population.cc`](../../src/nn/population.cc) `nn_xavier_init` with orthogonal init (Saxe et al. 2013) per [research_rlayer_placement.md §4](./research_rlayer_placement.md); keep W_xh and W_out blocks on Xavier
- [ ] T039 [US3] Add post-init sanity assert: spectral radius of W_hh lands in `[0.7, 1.05]` per research §4; assertion in `nn_xavier_init` (or unit test) using Eigen's `EigenSolver`
- [ ] T040 [US3] Add `OrthogonalInit_SpectralRadiusInBand` test in `tests/nn_telemetry_tests.cc` (or `tests/nn_evaluator_tests.cc` if more appropriate): generate N=10 fresh W_hh inits, assert all spectral radii in band
- [ ] T041 [US3] Rebuild + run telemetry tests + spectral-radius test: `bash scripts/rebuild.sh && ./build/tests/nn_telemetry_tests` — all green
- [ ] T042 [US3] Launch 5b run: `nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5b.log 2>&1 &` (D-alone config: recurrent ON, lexicase OFF; markers 3+5 stay in their D-alone state)
- [ ] T043 [US3] Monitor + plot regen on the same cadence as US2

### Pattern 2 — Sub-phase 5c (larger budget) — only if US2 indicates

- [ ] T044 [US3] Edit `autoc.ini`: bump `nMembers` 3500 → 5000 OR `nGenerations` 400 → 600 (pick one based on per-gen wall-clock estimate; envelope cap from [spec §Q3](./spec.md#clarifications))
- [ ] T045 [US3] Launch 5c run: `nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5c.log 2>&1 &` (no code changes, just config bump)
- [ ] T046 [US3] Monitor + plot regen; **revert** `autoc.ini` change after the run completes so future attempts use the standard envelope (commit revert as a separate atomic edit)

### Pattern 2 — outcome assessment

- [ ] T047 [US3] Write `specs/028-deeper-rnn/pattern2_outcome.md` with same structure as `dalone_outcome.md` (T032): final metrics, gate comparison, branch decision (Phase 6 win, second-attempt-in-budget, or 028 close as bounded no-go)

**Checkpoint**: Pattern 2 attempt assessed. Operator decides: ship to flight (US4 win path),
run a third pattern-2 sub-phase if budget remains and another telemetry signal points at it, or
close 028 as bounded no-go (US4 no-go path).

---

## Phase 6: User Story 4 — Close 028 (Priority: P3)

**Goal**: As an operator, I close 028 — either by shipping a flight-validated winner or by
documenting a bounded no-go and handing off to 029.

**Independent Test**: One of: (a) a successful flight on a 028-architecture build is logged in
`flight-results/flight-<date>/FLIGHT_REPORT.md` AND a `findings.md` documents the win, OR (b) a
`findings.md` documents the no-go with telemetry-evidence summary and 029 handoff notes.

### Win path — sim gate + flight

- [ ] T048 [US4] Run robustness check on the winning attempt's log: compute `pop_spread` per [data-model.md §4](./data-model.md#4-robustness-metric--pop_spread-close-of-attempt-only). If `pop_spread < 0.05`, run a second-seed attempt before flight (extends the bounded envelope by one); document in `findings.md` either way.
- [ ] T049 [US4] Extract winning NN weights: `./build/tools/nnextractor <S3-or-local-dmp-path> nn_weights.dat` for the gen that hit the gate
- [ ] T050 [US4] Trigger separate xiao port plan (out of 028 scope): per [spec §Resolved Q5](./spec.md#resolved-pre-clarify-questions), wire recurrent forward pass into `xiao/src/generated/nn_program_generated.cpp` via `nn2cpp` regeneration; covered in a new feature branch, not in this tasks file
- [ ] T051 [US4] Conduct flight test on the winning architecture; record outcome in `flight-results/flight-<YYYYMMDD>/FLIGHT_REPORT.md` per [project pre-flight checklist](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/project_preflight_checklist.md)
- [ ] T052 [US4] Write `specs/028-deeper-rnn/findings.md` win-path version: which sub-phase landed the win, telemetry signal trajectories, sim gate metrics, flight outcome, what carries forward to 025 (craft variations)

### No-go path — bounded effort exhausted

- [ ] T053 [US4] Write `specs/028-deeper-rnn/findings.md` no-go-path version: telemetry signal evolution across all attempts (D-alone + 1–2 pattern 2 runs), failure-mode hypothesis the data best supports per [027 findings.md §Plausible failure modes](../027-recurrent-nn/findings.md#plausible-failure-modes-not-yet-disambiguated), what carries forward to 029 (which markers stay in which state, what scenario-physics observations from [spec §Q4 adjacent](./spec.md#clarifications) need 029 attention)

**Checkpoint**: 028 closes. PR merged with findings.md as the durable artifact regardless of
outcome.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Updates that span multiple US, applied at close.

- [ ] T054 [P] Update [`specs/BACKLOG.md`](../BACKLOG.md) entry for CADENCE7-REDUX markers: reflect their current state at 028 close (which are tripped, which remain) for 029's start-state
- [ ] T055 [P] Run `quickstart.md` validation: walk the recipe top-to-bottom, confirm every step still works given the final tree state; update any commands that drifted during implementation
- [ ] T056 [P] Update [memory: project_model_convergence](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/project_model_convergence.md) with 028's win/no-go outcome and current model-tuning state
- [ ] T057 If win path: update [memory: project_xiao_imu_crosscheck](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/project_xiao_imu_crosscheck.md) and other active-backlog memory entries to reflect 028 close

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — start immediately
- **Foundational (Phase 2)**: Depends on Setup — blocks US1 implementation tasks
- **US1 (Phase 3)**: Depends on Foundational. **First milestone target.** Once complete, the
  binary is ready for training.
- **US2 (Phase 4)**: Depends on US1 (binary must be built and tested before training launch)
- **US3 (Phase 5)**: Depends on US2 (need D-alone outcome before picking pattern 2 sub-phase)
- **US4 (Phase 6)**: Depends on US2 (win path) OR US3 (win path or bounded no-go)
- **Polish (Phase 7)**: Depends on US4

### User Story Dependencies (linear, with conditional skips)

```text
Setup → Foundational → US1 (telemetry + marker flips, "first milestone")
                            ↓
                          US2 (D-alone diagnostic)
                            ↓
                  ┌─────────┴─────────┐
                  │                   │
              gate hit             gate miss
                  │                   │
                US4 win             US3 (pattern 2: 5a OR 5b OR 5c)
                                      ↓
                              ┌───────┴───────┐
                              │               │
                          gate hit       budget exhausted
                              │               │
                          US4 win        US4 no-go
                              ↓               ↓
                        Polish (Phase 7)
```

### Within Each User Story

- US1 tests (T007–T012) MUST be written and FAIL before T013–T018 implementation
- T013–T017 must complete before T018 (build/test verification)
- T018 must pass before T019 (smoke run)
- T020–T021 (plot script) parallelizable with T013–T017 (different files)
- T022–T026 (marker flips) sequential with T013–T018 (telemetry must work on cadence7-redux first)
- US2 monitoring tasks (T028, T029) parallelizable while T027 run is in progress

### Parallel Opportunities

**Within US1 (Phase 3) — high parallelism in test phase**:
- T007, T008, T009, T010, T011, T012: all six tests in same file but independent test functions — write in one editor session, parallel-friendly conceptually
- T020 (plot script) parallel with T013–T017 (different file)

**Across phases**: Limited. The plan is mostly linear because:
- The diagnostic run output is the input to branch-decision tasks
- Pattern 2 sub-phase selection depends on US2 outcome
- Marker flips for US1 must precede US2 launch

---

## Parallel Example: User Story 1

```bash
# Test-first phase — write all six telemetry tests in one editor session.
# Files: tests/nn_telemetry_tests.cc (single file, multiple TEST() functions)
Task: "T007 [US1] Implement WhhXhRatio_ZeroWhh_ReturnsZero test"
Task: "T008 [US1] Implement WhhXhRatio_IdentityWhh_ReturnsExpectedMagnitude test"
Task: "T009 [US1] Implement WhhXhRatio_NoRecurrentLayer_Sentinel test"
Task: "T010 [US1] Implement WhhCv_IdenticalPopulation_Zero test"
Task: "T011 [US1] Implement WhhCv_BimodalPopulation_AnalyticMatch test"
Task: "T012 [US1] Implement WhhCv_NoRecurrentLayer_Sentinel test"

# Implementation phase — different files, parallelizable:
Task: "T020 [P] [US1] Create plot_evolution_progress.py (Python, separate file)"
# while:
Task: "T013–T017 [US1] Telemetry hooks in src/nn/evaluator.cc + src/autoc.cc (C++)"
```

---

## Implementation Strategy

### MVP — First Milestone (US1 only)

This is the user's stated "first milestone — something to train on":

1. Complete Phase 1: Setup (T001–T003)
2. Complete Phase 2: Foundational (T004–T006)
3. Complete Phase 3: US1 (T007–T026)
4. **STOP and VALIDATE**: T026 green build at 1923 weights, T019 smoke run shows telemetry
   fields populated. Binary is ready to launch D-alone diagnostic.

### Incremental Delivery — Train and Iterate

1. MVP delivered → US1 binary ready → launch US2 D-alone diagnostic
2. US2 outcome assessed → branch:
   - Gate hit → US4 win path (sim gate + flight)
   - Gate miss → US3 pattern 2 (one of 5a/5b/5c per telemetry indication)
3. US3 outcome assessed → branch (same gate-hit / gate-miss / second-attempt logic)
4. Maximum: D-alone + 2 pattern-2 attempts per [spec §Q3 envelope](./spec.md#clarifications)
5. US4 closes 028 either way (win or bounded no-go)

### Why This Order

- **US1 first** — telemetry must land *before* recurrence is enabled, otherwise the first
  D-alone run produces no diagnostic data and any failure has to be repeated.
- **Markers 3 + 5 left untouched in US1** — D-alone is tracking-only by design; pattern 2 is
  conditional. Flipping markers 3 + 5 only when needed avoids running mis-configured experiments.
- **Test-first in US1** — telemetry signal correctness is the load-bearing assumption for
  every downstream decision. Constitution Principle I makes this non-negotiable.

---

## Notes

- `[P]` tasks = different files, no dependencies on incomplete tasks.
- `[Story]` label maps task to user story for traceability and parallel-team handoff.
- Each user story is independently completable and testable per the Independent Test criteria.
- Verify tests fail (red) before implementation per Constitution Principle I.
- Commit after each logical group (e.g., after T012 — all telemetry tests written; after T018
  — all tests passing; after T026 — markers flipped + green build).
- **Stop at any checkpoint** to validate before proceeding — especially the US1 → US2 boundary
  (binary built, smoke run clean) before launching a 400-gen training run.
- Avoid: bypassing test-first ordering, flipping markers 3 + 5 before pattern 2 is selected,
  running 5d (32-wide / layer-1) within the 3-attempt envelope.

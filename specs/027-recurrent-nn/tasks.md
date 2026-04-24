---

description: "Task list for 027 — Recurrent NN + lexicase smoothness"
---

# Tasks: 027 — Recurrent NN + C2-via-lexicase-smoothness

**Input**: Design documents from `/specs/027-recurrent-nn/`
**Prerequisites**: [spec.md](./spec.md), [plan.md](./plan.md), [research.md](./research.md)

**Clarifications (Session 2026-04-24 — five decisions locked)**:
- C2 incentive in scope as lexicase test-case extension (NOT Pareto).
- MVP scope is steps #0 + #3 only; A2-alone / A2+C2 / D-alone reserved.
- D-simple 16-wide (layer2 recurrent, +256 weights, total 1923).
- Hidden state `h_t` zero on span start; advance only on NN tick (~10 Hz).
- Xiao deployment deferred behind sim gate; nn2cpp emits recurrent
  code but xiao side not built/flashed in this feature.
See [spec.md Clarifications](./spec.md#clarifications).

**Tests**: Required for the contract/math changes (weight count,
recurrent forward, reset semantics, lexicase smoothness). Integration
validation is Phase 2 training signal against the go/no-go gate in
[plan.md](./plan.md).

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no unresolved dependencies).
- **[Story]**: `[US1]`..`[US5]` — user-story phases below.
- File paths are absolute from repo root.

## Path Conventions

- Autoc source: `src/`, `include/autoc/`, `tools/`
- CRRCSim (submodule): `crrcsim/src/`
- Xiao (deferred this feature): `xiao/src/`
- Tests: `tests/` (Google Test)
- Analysis: `specs/024-sim-real-fidelity/` (reused plotting scripts),
  `specs/027-recurrent-nn/` (feature-specific docs + results)

---

## Phase 1: Setup

**Purpose**: Baseline build check before any code changes.

- [ ] T001 Verify `027-recurrent-nn` branch builds clean on both
  autoc and crrcsim submodule with the PID-nullified code (commits
  3cd193e autoc / 74ba0b8 crrcsim): `cd build && make -j8`. All
  existing tests pass. Baseline must be green.

**Checkpoint**: Clean build baseline with PID nullified.

---

## Phase 2: Foundational (blocking prerequisites for all user stories)

**Purpose**: Topology + weight-count plumbing. Every consumer in US1
depends on these.

- [ ] T010 Extend [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h):
  add `constexpr bool NN_RECURRENT[NN_NUM_LAYERS] = {false, false, true, false}`
  alongside existing `NN_TOPOLOGY`. Layer 2 (16-wide) is recurrent per
  clarify Q3.
- [ ] T011 Update `NN_WEIGHT_COUNT` math in the same header to add
  `NN_TOPOLOGY[i] * NN_TOPOLOGY[i]` for each layer `i` where
  `NN_RECURRENT[i]`. Target: 1667 + 256 = **1923**. Keep existing
  sum-form readable; don't over-abstract.
- [ ] T012 Update `nn_weight_count(topology)` helper in
  [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc) or wherever it
  lives to compute the same value from a runtime topology + recurrent-flag
  vector. Signature may grow a `const std::vector<bool>& recurrent`
  parameter; callers updated in US1.
- [ ] T013 [P] Contract test in
  [`tests/contract_evaluator_tests.cc`](../../tests/contract_evaluator_tests.cc):
  assert `NN_WEIGHT_COUNT == 1923`, `NN_RECURRENT[2] == true`, all
  others false. Replace the existing `EXPECT_EQ(NN_WEIGHT_COUNT, 1667)`.

**Checkpoint**: Topology headers carry the recurrent flag; math
compiles; contract test pins the numeric values.

---

## Phase 3: User Story 1 — Sim infra (D-simple + C2 compile-clean) (Priority: P1) 🎯 MVP

**Goal**: Autoc + crrcsim + minisim + renderer + nn2cpp + all tests
build cleanly with the new 1923-weight recurrent topology and C2
lexicase plumbing. Training can launch. Nothing flies.

**Independent Test**: `cd build && make -j8` succeeds; all unit tests
pass; a 5-gen smoke run of `build/autoc` produces a data.dat with the
expected columns and no crash.

### Group 1.a — NN evaluator recurrent forward pass

- [ ] T020 [US1] Extend `NNControllerBackend` in
  [`include/autoc/nn/evaluator.h`](../../include/autoc/nn/evaluator.h):
  add `std::vector<float> hidden_state_` (one entry per recurrent
  layer unit, sized from topology); add `void reset()` that zeros
  it. Add comment: callers must construct once per span, call
  `evaluate()` per NN tick, and `reset()` on span start.
- [ ] T021 [US1] Implement `nn_forward_recurrent()` in
  [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc). Forward pass
  walks topology layer-by-layer; at a recurrent layer, compute
  `h_t = tanh(W_xh · x_t + W_hh · h_{t-1} + b_h)` using
  `hidden_state_` as `h_{t-1}`, then store the new `h_t` back. All
  other layers unchanged. Weight layout: `[W_xh, b_h, W_hh]` for
  recurrent layers; plain `[W, b]` for feedforward.
- [ ] T022 [US1] Update `nn_xavier_init()` in
  [`src/nn/population.cc`](../../src/nn/population.cc) to emit the
  extra W_hh block for each recurrent layer. Use Xavier fan-in =
  (input_size + layer_size) for the recurrent block. Standard
  Xavier for the rest; crossover/mutation flat-vector code
  unchanged.
- [ ] T023 [US1] Update
  [`src/nn/serialization.cc`](../../src/nn/serialization.cc): the
  serialized topology header must include the recurrent-flag array
  so a loaded `NNGenome` knows which layers carry state. No
  backward compatibility — old `.dmp` files become unloadable
  (per project policy, no cereal version bump).
- [ ] T024 [P] [US1] Unit tests in
  [`tests/nn_evaluator_tests.cc`](../../tests/nn_evaluator_tests.cc):
  (a) forward pass produces deterministic outputs for a fixed
  seeded weight vector, (b) `reset()` zeros hidden state, (c) two
  sequential evaluates produce different outputs when input is
  identical (proving state advancement), (d) reset between them
  yields same output as the first eval.
- [ ] T025 [P] [US1] Unit test in
  [`tests/nn_serialization_tests.cc`](../../tests/nn_serialization_tests.cc):
  serialize NNGenome with recurrent topology, deserialize,
  recurrent-flag array preserved, weight count matches 1923.

### Group 1.b — downstream consumers (compile + compile-only for xiao)

- [ ] T030 [P] [US1] Update [`tools/nn2cpp.cc`](../../tools/nn2cpp.cc)
  to emit: `static const bool nn_recurrent[] = {...}` array,
  `static float nn_hidden_state[N] = {0}` for the recurrent layer,
  `static void nn_reset() { ... }` that zeros `nn_hidden_state`,
  and updated forward-pass emission that does the W_hh mat-vec
  add for recurrent layers. Generated C must syntactically compile
  — verify by running `nn2cpp` on a test NNGenome and checking the
  output compiles with a minimal harness (no xiao link required).
- [ ] T031 [P] [US1] Update [`tools/nnextractor.cc`](../../tools/nnextractor.cc)
  if needed — it reads NNGenome from S3 `.dmp`; confirm the new
  header format round-trips. Likely zero code change (reads size
  from header); verify and note.
- [ ] T032 [US1] Update [`tools/minisim.cc`](../../tools/minisim.cc)
  to the construct-once-per-span pattern: move
  `NNControllerBackend nnBackend(nnGenome)` outside the tick loop
  to span-start, add `nnBackend.reset()` at span begin, use the
  same instance across ticks. Matches the crrcsim pattern added
  in T051.
- [ ] T033 [P] [US1] Verify [`tools/renderer.cc`](../../tools/renderer.cc)
  compiles against the new topology constants. Add a comment at
  the playback loop noting that recurrent NN state is not
  reconstructed during scrubbing (documented limitation — low
  priority per plan §5).

### Group 1.c — C2 smoothness in lexicase selection

- [ ] T040 [US1] Extend `ScenarioScore` in
  [`include/autoc/eval/fitness_decomposition.h`](../../include/autoc/eval/fitness_decomposition.h):
  add `double smoothness_score` field (lower = better). Init to 0
  in constructor.
- [ ] T041 [US1] Compute smoothness per scenario in
  [`src/autoc.cc`](../../src/autoc.cc) where `aircraftStates` is
  walked for data.dat writing (around line 590+). For each pair
  of consecutive ticks `i, i+1`, accumulate
  `|out_pt[i+1] - out_pt[i]| + |out_rl[i+1] - out_rl[i]| + |out_th[i+1] - out_th[i]|`.
  Normalize by `steps_completed - 1` (per-tick mean). Write to
  `ScenarioScore::smoothness_score` in the score aggregation path
  (likely `computeScenarioScores()` in
  [`src/eval/fitness_decomposition.cc`](../../src/eval/fitness_decomposition.cc)).
- [ ] T042 [US1] Extend `lexicase_select()` in
  [`src/eval/selection.cc`](../../src/eval/selection.cc): build
  the test-case pool as `(scenario_idx, kind, score)` triples
  where `kind ∈ {TRACKING, SMOOTHNESS}`. Pool size = 2 × scenarios.
  Each selection event shuffles over the full pool and applies
  ε-filter per kind. Epsilon scale per kind: tracking uses the
  existing 0.05 relative; smoothness uses a per-kind epsilon
  calibrated to the smoothness-score range (start 0.05, tune
  in T043 tests).
- [ ] T043 [P] [US1] Smoothness lexicase tests in
  [`tests/selection_tests.cc`](../../tests/selection_tests.cc):
  (a) equal tracking, different smoothness → smoother survives,
  (b) tradeoff case: A tracks better / jitters, B tracks worse /
  smooth — both survive over many selections with test-case
  shuffling, (c) smoothness-only equal case: best tracker still
  wins.
- [ ] T044 [US1] Update
  [`tests/contract_config_tests.cc`](../../tests/contract_config_tests.cc)
  if it has a SelectionMode test — lexicase stays as enum name,
  internal pool doubled. Verify config parsing still accepts
  `SelectionMode = lexicase`.

### Group 1.d — Sim integration (crrcsim reset hook)

- [ ] T050 [US1] In
  [`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h):
  add `NNControllerBackend nnController_` as a member of
  `T_TX_InterfaceAUTOC`. Construct with current `nnGenome` on
  weight-ready (same place `nnGenome` is populated).
- [ ] T051 [US1] In
  [`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp):
  (a) at span-start reset block (near line 520 where
  `aircraftStates.clear()` is), call `nnController_.reset()`;
  (b) replace per-tick `NNControllerBackend nnBackend(nnGenome);
  nnBackend.evaluate(...)` (around line 907) with
  `nnController_.evaluate(aircraftState, pathProvider)` on the
  member instance.
- [ ] T052 [US1] Add a one-line comment at the `shouldEval` gate
  in the same file noting that NN hidden state advances only on
  NN-eval ticks (clarify Q4 contract). No code change.

### Group 1.e — Build + verify

- [ ] T060 [US1] Full rebuild: `cd build && make -j8`. All targets
  (autoc, crrcsim, minisim, renderer, nn2cpp, nnextractor, all
  unit tests) compile. Phase 2 contract test and Phase 3 US1
  tests all pass. User runs.
- [ ] T061 [US1] Smoke training run: 5 generations via
  `./build/autoc -c autoc.ini` with `NumberOfGenerations = 5`
  (or override). Verify data.dat has the expected columns
  including the 13 PID-observational columns; no NaN, no
  crashes; fitness prints sane values. User runs.

**Checkpoint**: US1 complete. Binary runs end-to-end with recurrent
D-simple + C2 lexicase extension. No hardware touched.

---

## Phase 4: User Story 2 — Primary training run cadence9 (Priority: P1, depends on US1)

**Goal**: Execute the full 400-gen training run of the primary bet
(#3: D-simple + C2). Monitor during the run. Collect the data needed
for the go/no-go decision.

**Independent Test**: A completed `data.dat` + `data.stc` + training
log at 400 generations; fitness reaches the plateau; plot scripts
produce the measurement PNGs without error.

- [ ] T100 [US2] Launch full training:
  `nohup ./build/autoc -c autoc.ini > logs/autoc-027-cadence9.log 2>&1 &`.
  Default config parity with cadence7/pid1 (pop 3500, 400 gens,
  longSequential, all variations on).
- [ ] T101 [P] [US2] Every ~50 gens during the run, regenerate
  fitness ramp PNG: `python3 specs/024-sim-real-fidelity/plot_fitness_ramp.py
  --focus cadence9:logs/autoc-027-cadence9.log
  --compare cadence7:logs/autoc-024-cadence7.log
  --compare pid1:logs/autoc-026-pid1.log --total-gens 400
  --out specs/027-recurrent-nn/autoc-027-cadence9-fitness.png`.
- [ ] T102 [P] [US2] Every ~50 gens during the run, regenerate
  aggressiveness PNG:
  `python3 specs/024-sim-real-fidelity/plot_control_aggressiveness.py
  data.dat specs/027-recurrent-nn/control_aggressiveness_cadence9.png "autoc-027-cadence9 (D-simple + C2)"`.
- [ ] T103 [US2] Early-stop check at gen 100: if dCtrl mean-of-paths
  exceeds pid1's 1.60, pause training, capture diagnostic PNGs,
  flag for plan-level decision (likely escalation to US4 without
  waiting for full 400 gens).

**Checkpoint**: US2 complete. cadence9 trained to plateau OR
early-stopped with clear failure mode.

---

## Phase 5: User Story 3 — Analyze + decide (Priority: P1, depends on US2)

**Goal**: Apply the go/no-go gate from plan.md against cadence9's
result. Record the decision and the evidence.

**Independent Test**: A `cadence9_measurement.md` committed with
fitness ramp + aggressiveness PNGs + explicit GO or NO-GO call with
quantitative justification against the gate.

- [ ] T200 [US3] Final fitness ramp PNG (same command as T101, final
  log state). Capture cadence9's final fitness number.
- [ ] T201 [US3] Final aggressiveness PNG (same command as T102,
  final data.dat state). Capture mean dCtrl + mean |out| across
  paths at plateau.
- [ ] T202 [P] [US3] Output-histogram panel via
  `python3 specs/024-sim-real-fidelity/plot_bangbang_flight.py`
  on a tier-1 eval data.dat (needs tier-1 eval run first via
  `scripts/eval-suite.sh` if not already). Verify histogram
  spreads toward 0 vs pid1's more-bimodal state.
- [ ] T203 [US3] Write
  [`specs/027-recurrent-nn/cadence9_measurement.md`](./cadence9_measurement.md)
  with: the three PNGs, table of metrics vs plan.md gate, GO or
  NO-GO call with rationale.
  - **GO** if all three quantitative bars hit (dCtrl ≤ 0.80,
    |out| ≤ 2.00, fitness ≥ −30000 at gen 400).
  - **NO-GO** if any misses. Document which bar missed and by
    how much.
- [ ] T204 [US3] Update [`spec.md`](./spec.md) status header with
  the decision. If GO → "GO — primary bet landed, see cadence9_measurement.md".
  If NO-GO → "NO-GO #3 — escalating per US4".

**Checkpoint**: US3 complete. Decision recorded with evidence.
Branch between US4 (NO-GO escalation) and US5 (close, either
outcome).

---

## Phase 6: User Story 4 — Escalation ladder (Priority: P2, depends on US3 NO-GO)

**Goal**: If #3 doesn't clear the gate, apply the right escalation
based on *how* it failed. Each escalation retrains and re-decides.
Scope-gated: only executed if US3 calls NO-GO.

### Escalation branches (pick based on failure shape)

- [ ] T300 [US4] **If aggressiveness improved but fitness regressed**
  (memory + incentive cost outweighed the benefit): escalate to
  32-wide recurrent. Edit
  [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h)
  `NN_RECURRENT[1] = true` (layer 1, 32-wide); update
  `NN_WEIGHT_COUNT` to 1667 + 1024 = 2691. Contract test T013
  refresh. Retrain as cadence10.
- [ ] T310 [US4] **If neither aggressiveness nor fitness improved**
  (D-simple not the right architecture): run #4 diagnostic —
  D-simple alone, no C2. Disable the smoothness lexicase pool
  (revert T042 to tracking-only) without reverting D. Retrain as
  cadence10-diag. If D-alone also flatlines, D-simple is not the
  answer; escalate to D-ESN (new tasks, separate plan).
- [ ] T320 [US4] **If fitness fine but aggressiveness unchanged**
  (C2 signal not biting): either (a) add C1 α-penalty on top as
  additional pressure, or (b) tune smoothness-kind epsilon in
  T042 down to make lexicase filtering tighter. Retrain as
  cadence10-c1c2.
- [ ] T330 [US4] Retrain + measure per selected escalation
  (T100–T203 equivalents). Update
  [`specs/027-recurrent-nn/escalation_outcome.md`](./escalation_outcome.md)
  with what was tried and what it showed. Loop to US3 decision
  or decide feature-close.

**Checkpoint**: Either a GO candidate from escalation, or a clear
analysis of why the 027 approaches can't land.

---

## Phase 7: User Story 5 — Feature close (Priority: P1, depends on US3 GO or US4 final)

**Goal**: Close 027 either as a go-with-cadence9 (or cadenceN from
escalation) or as a no-go-with-learnings. Queue follow-up work.

- [ ] T400 [US5] Summary commit on `027-recurrent-nn` branch
  documenting the outcome: fitness trajectory, aggressiveness,
  which cell of the 2×2 worked, what we learned about the
  incentive question from research.md §2.
- [ ] T401 [US5] If GO: draft xiao-port plan in
  [`specs/027a-xiao-recurrent/`](../027a-xiao-recurrent/) (or
  similar) per clarify Q5 — port the recurrent forward pass to
  `nn_program_generated.cpp` runtime, regenerate weights, rebuild
  xiao, bench rate-response, flight prep checklist. Not executed
  in 027; sets up the next feature.
- [ ] T402 [US5] Update
  [`specs/025-craft-variations/spec.md`](../025-craft-variations/spec.md)
  status: UNBLOCKED if 027 produced a non-bang-bang controller,
  or REVISIT-with-different-framing if 027 closed NO-GO across
  all escalations.
- [ ] T403 [US5] Merge `027-recurrent-nn` → main: crrcsim submodule
  pointer first, then autoc parent (per project memory on merge
  order).
- [ ] T404 [US5] Backlog cleanup: move any follow-ups surfaced
  during 027 (32-wide retry, D-ESN exploration, C1 augmentation,
  renderer scrub-with-state, Pareto selection if we ever want it)
  to [`specs/BACKLOG.md`](../BACKLOG.md).

**Checkpoint**: 027 closed. Main branch carries the result.

---

## Dependencies

```
Setup (T001)
   ↓
Foundational (T010-T013)
   ↓
US1: Sim infra (MVP)
  Group 1.a NN evaluator          (T020-T025)
  Group 1.b Downstream compile    (T030-T033)    [after 1.a]
  Group 1.c C2 lexicase           (T040-T044)    [parallel to 1.a]
  Group 1.d Sim integration       (T050-T052)    [after 1.a]
  Group 1.e Build + verify        (T060-T061)    [after all above]
   ↓
US2: Primary training cadence9   (T100-T103)
   ↓
US3: Analyze + decide             (T200-T204)
   ↓
   ├── GO  → US5
   └── NO-GO → US4 escalation     (T300-T330)
                    ↓
               (GO or final) → US5
   ↓
US5: Close                         (T400-T404)
   ↓
FEATURE CLOSED
```

### Parallel opportunities

- **Within Foundational**: T013 parallel to T010-T012 (different file).
- **Within US1**:
  - Group 1.a can mostly proceed sequentially (T020→T021→T022→T023),
    but T024 and T025 tests are parallel to each other.
  - Group 1.b T030/T031/T033 all [P] after T020 lands the new
    evaluator API. T032 has a light dependency on T020's new
    `reset()` signature.
  - Group 1.c (T040-T044) is independent of 1.a/1.b and can be
    developed fully in parallel by a different contributor/thread.
  - Group 1.d (T050-T052) depends only on T020.
- **Within US2**: T101 and T102 run in parallel during the training
  (independent monitoring scripts).
- **Within US3**: T200, T201, T202 parallel (all read from same
  final data.dat / log).

### Critical path

T001 → T010 → T011 → T020 → T021 → T050 → T060 → T061 → T100 → T101
→ T203 → (decision) → (US4 or US5) → T403 → FEATURE CLOSED.

Realistic estimate assuming T100 = 400 gens × ~110 sec/gen on
current hardware ≈ 12 hr single-thread: US1 code work is ~2 days,
US2 training is ~1 day wall clock, US3 analysis ~2 hours, US5 close
~1 hour. Total roughly 4 days of elapsed calendar time minimum,
assuming US4 escalation not triggered.

## Implementation strategy

**MVP scope** = US1 + US2 + US3. Get the binary running the primary
bet, train, decide. Everything else (US4 escalation, US5 close) is
conditional downstream.

**Incremental checkpoints**:

- After Foundational: weight-count contract landed, no runtime
  changes yet. Fully revertable (revert 2-3 commits).
- After US1 Group 1.a: evaluator has recurrent capability but
  nothing calls it yet; existing non-recurrent topology still
  works because `NN_RECURRENT[]` is read by consumers.
- After US1 Group 1.b: downstream tools compile. Still no runtime
  change if nobody constructs with recurrent layers yet.
- After US1 Group 1.c: C2 lexicase pool active in selection. If
  topology still feedforward, runtime change is only selection
  pressure (smoothness awarded zero penalty since outputs
  reasonable? — verify early).
- After US1 Group 1.d: crrcsim runtime has NN hidden state in its
  span loop. Recurrent behavior now reachable.
- After US1 Group 1.e: full binary builds and runs a smoke. This
  is the "can we train" milestone the user asked for.
- After US2: 400 gens of data against the primary bet.
- After US3: decision recorded, path forward clear.

**Guardrails**:

- No hardware flash this feature. Xiao side compiles (nn2cpp
  output is valid C++) but isn't built into a flashable binary.
- No backward compatibility on `.dmp` / `.stc` / data.dat —
  rebuild everything from current tree.
- Compile-time constants only. No new ini knobs.
- If training collapses catastrophically in the first ~30 gens
  (fitness worse than −10000, sigma pinned at floor), pause,
  diagnose, don't let it burn 400 gens of compute.

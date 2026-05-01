---
description: "029 — No-Future-Input NN Architecture — task list"
---

# Tasks: 029 — No-Future-Input NN Architecture

**Input**: Design documents from [`specs/029-no-future-arch/`](.)
**Prerequisites**: [plan.md](./plan.md), [spec.md](./spec.md)

**Note**: 029 was pivoted 2026-04-30 from "tracker mode" to "no-future-input architecture exploration." Tracker mode moved to [`specs/030-tracker-mode/`](../030-tracker-mode/). The branch name `029-tracker-mode` is historical.

## Format: `[ID] [P?] [Story?] Description`

---

## Phase 1: US1 — Past-only baseline (RUNNING — most tasks complete)

- [X] T001 Branch state confirmed clean before pivot. Working on `029-tracker-mode` branch
- [X] T002 028 baseline (more-rnn3 commit) confirmed in tree
- [X] T003 Build verified green at 028 baseline (user-driven, incremental)
- [X] T006 Update time-sample comment in [`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h) to past-only distribution `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`. Mirror comments updated in topology.h, evaluator.cc, tests/nn_evaluator_tests.cc, autoc.cc data.dat header column labels
- [X] T007 Update time-offset constants in [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc) — `HIST_PAST = {5,4,3,2,1,0}`. Forecast-projection loops removed. PathProvider parameter retained `[[maybe_unused]]` for caller-API stability. Renderer NOW constant 3→5
- [X] T008 Build green incremental, ctest passes
- [ ] T009 xiao build verification (deferred — defer until run completes or before any flight test of the resulting controller)
- [X] T010 Training run launched: `nohup stdbuf -oL -eL build/autoc -i autoc.ini > logs/autoc-029-pastonly1.log 2>&1 &`. Run name: `more-rnn4-pastonly1`
- [X] T011 [P] Monitoring set up: 6-panel evolution plot at gen 134; comparators more-rnn3 + cadence7-redux. PNG at [pastonly1_evolution.png](./pastonly1_evolution.png)
- [X] T012 [P] Per-axis aggressiveness time-series at gen 134; PNG at [pastonly1_per_axis.png](./pastonly1_per_axis.png). Result: roll-dominant dCtrl pattern matches RNN-architecture expectation
- [ ] T013 Apply early-stop criteria during run: kill if (a) best-fitness above pid1's −27045 floor by gen 100 with no descent (already past gen 100; criterion not triggered), OR (b) per-axis pattern deviates substantially from more-rnn3 (also not triggered — roll-dominant matches)
- [ ] T014 At run completion (or stall), capture late-plateau metrics: read final-gen `#NNGen` line, late-plateau dCtrl + ⟨|out|⟩ (last-50-gens window), re-evaluate winning gen against fixed-difficulty eval
- [ ] T015 Write outcome doc `specs/029-no-future-arch/pastonly1_outcome.md` — fitness comparison vs more-rnn3, per-axis pattern comparison, **branch decision** (PASS / PARTIAL / FAIL) per [plan.md §1.5](./plan.md)

**Checkpoint**: PASS → close 029, proceed to 030. PARTIAL/FAIL → trigger US2.

---

## Phase 2: US2 — Architectural variants (CONDITIONAL on US1 outcome)

**Status**: Not started. Fires only if US1 lands PARTIAL or FAIL.

Each variant: separate experimental branch off the more-rnn4-pastonly1 baseline, identical Path A config (pop 5000 × 600+ gens), fresh training run. Per-variant outcome doc + evolution PNG.

### Phase 2 launch sequencing

V1 first (cheapest); fall back to V2/V4 only if V1 doesn't close the gap. V3 as parallel wildcard if compute available.

### Phase 2.1: V1 — Wider recurrent state

- [ ] T020 [US2-V1] Update [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h): NN_TOPOLOGY second hidden layer 16 → 32. Update NN_WEIGHT_COUNT (~+1.5x). Compile-time topology constants update
- [ ] T021 [US2-V1] Update tests for topology change. ctest green
- [ ] T022 [US2-V1] Launch training: `more-rnn5-pastonly-wide16x32`. Path A config, single seed, 600+ gens
- [ ] T023 [P] [US2-V1] Monitor / outcome at gen 600+: `specs/029-no-future-arch/v1_wider_outcome.md`

### Phase 2.2: V2 — Deeper recurrent stack

- [ ] T030 [US2-V2] Update topology: {33, 32, 16r, 16r, 3} — two stacked recurrent blocks. Update NN_RECURRENT array, NN_WEIGHT_COUNT
- [ ] T031 [US2-V2] Verify recurrent forward-pass correctly handles two recurrent layers (existing infrastructure should — confirm in evaluator.cc)
- [ ] T032 [US2-V2] Launch training: `more-rnn6-pastonly-deep`. Outcome → `v2_deeper_outcome.md`

### Phase 2.3: V3 — Wider input encoding (parallel wildcard)

- [ ] T040 [US2-V3] Extend HIST_PAST to 12 past samples × 4 fields = 48 inputs. Adjust NNInputs struct, topology first layer 33 → 48
- [ ] T041 [US2-V3] Launch training: `more-rnn7-pastonly-wide-input`. Outcome → `v3_wider_input_outcome.md`

### Phase 2.4: V4 — Hybrid predictor head

- [ ] T050 [US2-V4] Design: predictor sub-net (extrapolates target +0.1/+0.5 from past) + control sub-net (consumes predicted future + current state). Network structure documented in `specs/029-no-future-arch/v4_predictor_design.md`
- [ ] T051 [US2-V4] Implement two-headed forward pass; predictor head trained jointly with control head against the existing fitness signal (pure end-to-end learning) OR with a secondary supervised signal from path lookahead during pathgen-mode training
- [ ] T052 [US2-V4] Launch training: `more-rnn8-pastonly-predictor`. Outcome → `v4_predictor_outcome.md`. **This is the most direct test of the "evolve a predictor" hypothesis from the pivot rationale.**

### Phase 2.5: US2 cross-comparison

- [ ] T060 [US2] Cross-comparison summary `specs/029-no-future-arch/us2_summary.md` — fitness, per-axis aggressiveness, weight count, training-time cost across V1-V4. Choose architecture for 030 readiness criterion (SC-004)

---

## Phase 3: US3 — Attention / qualitative jump (CONDITIONAL on US2 not closing the gap)

**Status**: Speculative. Likely warrants its own feature spec (029.5 or 031) if reached. Listed here for awareness only.

- [ ] T070 [US3] If reached: write fresh feature spec `specs/029.5-attention-arch/spec.md` (or similar). Out of scope for this 029.

---

## Phase 4: Close-out

- [ ] T080 Update memory entries: [`project_post_028_routing.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_post_028_routing.md) with 029 outcome + routing decision (proceed to 030 OR back to architecture work). [`project_library_based_training.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md) with the no-future architecture finding
- [ ] T081 Update [`specs/030-tracker-mode/`](../030-tracker-mode/) plan / spec with the chosen 029 architecture as the prerequisite baseline
- [ ] T082 Final outcome doc `specs/029-no-future-arch/findings.md` summarizing US1 (and US2 if fired) results + the architecture chosen as 030's foundation

---

## Notes

- Original 029-tracker-mode tasks T016-T087 (sensor refactor, multi-aircraft, dmp converter, beacon projection, etc.) moved to [`specs/030-tracker-mode/tasks.md`](../030-tracker-mode/tasks.md)
- US1 has been running since 2026-04-30 — no decision needed until late-plateau forms (~gen 400-600)
- The roll-dominant per-axis aggressiveness signature at gen 134 is *encouraging* — architecture is engaging memory consistently with more-rnn3. Final fitness gap is the open question

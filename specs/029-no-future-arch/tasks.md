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

### Phase 2.6: Prediction-aiding alternatives (CONDITIONAL — if US2 V1-V4 don't close the gap, OR as parallel exploration)

Brainstormed 2026-04-30 after pastonly1 gen-220 read showed a real fitness gap to more-rnn3. These are alternative levers — orthogonal to US2 V1-V4 capacity tweaks — to *help the NN develop a predictor inside its trunk*. Ordered by structural commitment (cheapest first):

**Pose-estimation framing (added 2026-04-30)**: any prediction-aiding lever should be designed with **pose estimation** as the structural target — not generic future-position extrapolation. Reason: the eventual 030 tracker-mode inputs are two beacon dots per camera tuple `(screen_x, screen_y, visible)` per beacon × 6 history slots. From two dots over time, the *only* physically grounded predictor is one that recovers target **pose** (bearing, bank, range, attitude rates) and then propagates pose forward via banked-aircraft kinematics. A naive position-extrapolator is *fundamentally* wrong on a banking target — the target follows an arc, the extrapolator predicts a straight line. So D1-D5 below are framed in pose-aware terms.

- [ ] T061 [US2-D1] **Pose-derived features (cheapest, ~10-15 LOC).** Compute pose-aware geometric features from the past samples and append as additional NN inputs. In pathgen mode (rabbit = single point), the closest analog is target-direction-vector + first/second derivatives. In 030 tracker mode (two beacons), the natural set is: inter-beacon screen-distance (range proxy), inter-beacon screen-angle (target bank proxy), inter-beacon midpoint (bearing proxy), plus the rates of change of all three. ~8 extra inputs in tracker mode. **Tests whether the bottleneck is "NN can't extract pose from raw points" vs "even with pose, can't predict."** If derived features close the gap on 029 pathgen mode, the technique transfers directly to 030 with minor sharpening. Variant name: `more-rnn4-pastonly-derived`
- [ ] T062 [US2-D2] **Auxiliary supervised pose head (moderate, infrastructure lift).** Two-headed NN: control head (pitch/roll/throttle, GA-evolved against fitness as today) + **pose head** (estimated target bearing / bank / range / attitude rates, supervised against ground-truth pose during pathgen training — the rabbit's pose is known exactly). Shared trunk's representation shaped by both gradient descent (pose) and GA (control). **Critical design choice**: head outputs **pose** (~5-6 floats: bearing-x, bearing-y, range, bank-angle, plus rate fields), NOT "future position." Pose is small, physically meaningful, has clean ground truth, and **the head structure transfers to 030 unchanged** — pose is pose, whether derived from sim beacons or real beacon detection. **Most informative answer to "can a ~2000-weight NN encode a usable pose estimator when explicitly incentivized to?"** Variant name: `more-rnn4-pastonly-posehead`
- [ ] T063 [US2-D3] **Two-stage pose → predict → control (= sharpened V4).** Pose sub-net outputs current pose from history → predictor sub-net extrapolates pose forward using **explicit banked-aircraft kinematics** (e.g., bank-angle × airspeed → turn radius → next-tick pose) → control sub-net consumes predicted future pose + current state. Most prediction-correct architecture on banking targets; biggest structural commit. Cross-references T050-T052 (US2-V4)
- [ ] T064 [US2-D4] **Attention over history (= US3 territory).** Soft attention over the 6 history slots — lets the NN learn which past samples matter for the current decision. Different parameterization in different regimes. Pairs naturally with T062 (the attention layer feeds the pose head). Bigger commitment, likely needs GPU-eval. Cross-references T070 (US3)
- [ ] T065 [US2-D5] **Dithered-future curriculum (= existing BACKLOG item).** Replace the exact future-lookahead with noise-corrupted samples — "fuzzy cone of probable next steps" — to teach the NN uncertainty tolerance. Anneal noise sigma from 0 (exact future) to large (no information) over training. Orthogonal to D1-D4 — pairs with any of them as a robustness curriculum. Best combined with D1 or D2 to teach uncertainty-tolerance simultaneously with prediction-learning

**Recommended sequencing if US1 lands PARTIAL**: D1 first (cheap test of "is it features or capacity?"), then D2 if D1 insufficient (the meatier "can it learn to estimate pose?" question — and the most direct 030-readiness signal since the pose head transfers). D3/D4 are larger commitments — only fire if D1+D2 results point that direction. D5 is a curriculum modifier, not a standalone variant.

**Cross-reference**: 030 tracker mode's perception interface (per [`specs/030-tracker-mode/spec.md`](../030-tracker-mode/spec.md) FR-005) emits `(screen_x, screen_y, visible)` per beacon — a deliberately minimal interface that mirrors the deployed FPGA centroid extractor. The pose-head architecture (T062) consumes this directly; pose estimation is the bridge from "two dots in screen space" to "physical target state usable by a controller." This is why T062's head transfer cleanly to 030 is the highest-leverage 029-stage commitment.

---

## Phase 3: US3 — Attention / qualitative jump (CONDITIONAL on US2 not closing the gap)

**Status**: Speculative. Likely warrants its own feature spec (029.5 or 031) if reached. Listed here for awareness only.

- [ ] T070 [US3] If reached: write fresh feature spec `specs/029.5-attention-arch/spec.md` (or similar). Out of scope for this 029.

---

## Phase 4: Close-out

- [ ] T080 Update memory entries: [`project_post_028_routing.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_post_028_routing.md) with 029 outcome + routing decision (proceed to 030 OR back to architecture work). [`project_library_based_training.md`](../../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md) with the no-future architecture finding
- [ ] T081 Update [`specs/030-tracker-mode/`](../030-tracker-mode/) plan / spec with the chosen 029 architecture as the prerequisite baseline
- [ ] T082 Final outcome doc `specs/029-no-future-arch/findings.md` summarizing US1 (and US2 if fired) results + the architecture chosen as 030's foundation

## Phase 5: Polish & cross-cutting

- [ ] T090 **Mirror data.stc telemetry into the log** — make the per-gen training log a *superset* of `data.stc`, not a sibling. Today: [`src/autoc.cc:1150-1155`](../../src/autoc.cc#L1150-L1155) writes the legacy `Gen N Best=... Avg=... Worst=... Sigma=...` line via `*logger.info()` (→ log file), while [`src/autoc.cc:1201-1215`](../../src/autoc.cc#L1201-L1215) writes the richer structured `#NNGen gen=N best=... avg=... worst=... bestSigma=... avgMaxStreak=... pctInStreak=... stability=... energy=... whh_xh_ratio=... w_xh0_cv=... w_xh1_cv=... w_hh_cv=...` line via `bout` (→ `data.stc` only). Result: when `data.stc` gets clobbered (e.g., by a successor training run launched in the same working dir), the recurrent telemetry is irrecoverable from the log.
   - **Fix**: emit the `#NNGen ...` line to BOTH `*logger.info()` AND `bout`. Update `plot_evolution_progress.py`'s `load_log()` (already auto-detects `#NNGen` first) — works unchanged since the auto-detect path was added. The legacy `Gen N Best=...` line can stay (human-readable) or be dropped (parser only needs `#NNGen`).
   - **Why**: motivated by 029 pastonly1 (gen 220, single seed) — `data.stc` was overwritten by pastonly2's launch on the same working dir; recurrent telemetry was lost. Fitness trajectory was recoverable from the log via legacy `Gen N Best=...`, but `whh_xh_ratio` / `w_*_cv` / `pctInStreak` / `stability` / `energy` were gone.
   - **Scope**: ~5 LOC change in `src/autoc.cc`. No schema break — `#NNGen` lines are already the canonical structured emit format. Verify on a smoke run that `plot_evolution_progress.py` parses identically against either the log or `data.stc` afterwards.

---

## Notes

- Original 029-tracker-mode tasks T016-T087 (sensor refactor, multi-aircraft, dmp converter, beacon projection, etc.) moved to [`specs/030-tracker-mode/tasks.md`](../030-tracker-mode/tasks.md)
- US1 has been running since 2026-04-30 — no decision needed until late-plateau forms (~gen 400-600)
- The roll-dominant per-axis aggressiveness signature at gen 134 is *encouraging* — architecture is engaging memory consistently with more-rnn3. Final fitness gap is the open question

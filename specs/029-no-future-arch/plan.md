# 029 — Implementation Plan (No-Future-Input NN Architecture)

**Branch**: `029-tracker-mode` (working branch — historical name) | **Date**: 2026-04-30 (post-pivot) | **Spec**: [spec.md](./spec.md)

## Summary

029 explores whether a controller NN architecture can train effectively from **past-only sensor inputs** — no future-lookahead samples. This is the prerequisite for tracker mode (030), which has no parametric path to look ahead on.

**Three concentric scopes**, gated on outcome:

1. **US1 — Past-only baseline** (running): existing D-simple recurrent NN with the input time-distribution redistributed past-only. Already deployed; training active.
2. **US2 — Architectural variants** (conditional on US1 PARTIAL/FAIL): widen / deepen / hybrid-predictor variants of the existing recurrent NN.
3. **US3 — Non-recurrent / attention** (conditional on US2 not closing the gap): qualitative architecture jump.

Most likely outcome: US1 gives an answer (PASS / PARTIAL / FAIL) within the current 800-gen run. US2/US3 only fire if US1 results warrant them.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis)
**Primary Dependencies**: Eigen (vec3/quat math), cereal (NN serialization), inih (autoc.ini), GoogleTest, CRRCSim LaRCSim FDM
**New Dependencies**: none for US1; US2 may add nothing (just topology / weight-count tweaks); US3 (if reached) may need new linear-algebra primitives or GPU-native eval — separate decision
**Storage**: file-based (`data.dat`, `data.stc`, S3 `.dmp`) — no schema changes for US1/US2
**Testing**: existing GoogleTest harness + per-axis aggressiveness + per-gen evolution plot; no new contracts
**Target Platform**: Linux x86-64 (training & analysis); xiao deferred to flight cycle decoupled from 029
**Project Type**: Architecture-research feature; deliverable is an outcome document + chosen architecture + evolution telemetry
**Performance Goals**: per-gen eval cost should not regress (US1 — uses same forward pass as 028)
**Constraints**:
- Determinism preserved (per project_variation_design_principles.md)
- Joint-PRNG variation preserved (per same)
- gp_fitness type throughout
- US2 topology variants must roundtrip cleanly through cereal NNGenome serialize without backwards-compat shims (per feedback_no_cereal_versioning.md — clean cuts only)
**Scale/Scope**: US1 is ~6 lines of code already done. US2 (if needed) is small topology / config changes. US3 (if needed) is its own bigger feature — likely deferred to a 029-followup spec.

## Constitution Check

| Principle | Status | Notes |
|---|---|---|
| **I. Testing-First** | PASS | US1 reuses existing 028 test infrastructure unchanged (33 inputs, topology preserved). US2 variants would need topology-coverage tests; deferred until US2 fires. |
| **II. Build Stability** | PASS | US1 code change is trivial and behind no flag — pathgen mode coexists with no incremental work. xiao build: pending re-verification post-edit. |
| **III. No Compatibility Shims** | PASS | Past-only redistribution is a clean swap of the time-offset constants; no shim. Old `data.dat` files (with the +future columns) are unloadable by post-029 tools — clean cut. |
| **IV. Unified Build** | PASS | No new build infrastructure. |

**Gate result**: PASS.

## Phase 1 — US1: Past-only baseline (RUNNING)

### 1.1 Code change ✅ DONE

`HIST_PAST` array in [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc) changed from `{9, 3, 1, 0}` (4 past tick-offsets) to `{5, 4, 3, 2, 1, 0}` (6 past-only). Forecast-projection loops removed. PathProvider parameter retained `[[maybe_unused]]` for caller-API stability.

Mirror updates landed in: [`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h), [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h), [`tests/nn_evaluator_tests.cc`](../../tests/nn_evaluator_tests.cc), [`src/autoc.cc`](../../src/autoc.cc) data.dat header column labels (`tgX-9..+5` → `tgX-5..0`), [`tools/renderer.cc`](../../tools/renderer.cc) `NOW` constant 3 → 5.

Committed `ccbd837` and pushed.

### 1.2 Build + test ✅ DONE (incremental)

User ran incremental perf build; build green. xiao build verification — pending (defer until run completes; any xiao impact would manifest later).

### 1.3 Training run launched ✅ DONE

```bash
nohup stdbuf -oL -eL build/autoc -i autoc.ini > logs/autoc-029-pastonly1.log 2>&1 &
```

Run name: `more-rnn4-pastonly1`. Path A config (pop 5000 × 800 gens, recurrent NN, single seed). Live at gen 134 as of 2026-04-30 evening refresh.

### 1.4 Monitor cadence (live)

- 6-panel evolution plot every 50 gens via [`plot_evolution_progress.py`](./plot_evolution_progress.py) — focus `pastonly1:data.stc`, comparators `more-rnn3:more-rnn3-data.stc` + `cadence7-redux:logs/autoc-027-cadence7redux.log`
- Per-axis time-series every 100 gens via [`plot_per_axis_time_series.py`](./plot_per_axis_time_series.py)

Snapshots in [`pastonly1_evolution.png`](./pastonly1_evolution.png) and [`pastonly1_per_axis.png`](./pastonly1_per_axis.png) — refresh as gens accumulate.

### 1.5 Decide PASS / PARTIAL / FAIL at gen 600+ or stall

Pass criterion (per spec):
- Late-plateau fitness within ±10 % of more-rnn3 under fixed-difficulty eval
- Per-axis pattern architecture-consistent (roll-dominant)
- pctInStreak emergence on schedule

Outcome → `pastonly1_outcome.md`. Decision tree:

| Outcome | Branch |
|---|---|
| PASS | Close 029, proceed to 030 tracker-mode |
| PARTIAL | Trigger US2 (architectural variants) |
| FAIL | Trigger US2 + US3 architectural rethink |

## Phase 2 — US2: Architectural variants (CONDITIONAL)

Triggered if US1 lands PARTIAL or FAIL. Each variant is a single-axis change off the more-rnn4-pastonly1 baseline, identical Path A config, fresh training run.

| Variant | Change | Cost (compute / weight count) | Hypothesis |
|---|---|---|---|
| V1: Wider recurrent | Layer 2: 16 → 32 (NN_RECURRENT[2]) | +1.5x weights, +modest compute | Hidden-state capacity bottleneck |
| V2: Deeper recurrent stack | {33, 32, 16r, 16r, 3} two stacked recurrent blocks | +1.4x weights, +moderate compute | Temporal depth bottleneck |
| V3: Wider input encoding | 12 history slots × 4 fields = 48 inputs (vs current 33). First-layer: 48 → 32. | +0.5x weights (first layer), no compute change | Explicit temporal richness > recurrent capacity |
| V4: Hybrid predictor head | Split network: small predictor sub-net extrapolates target +0.1/+0.5 from past, control sub-net consumes both. | +0.3-0.5x weights, structurally different | Direct test of "evolve a predictor" hypothesis |

**Sequencing**: V1 first (cheapest, smallest change). If V1 doesn't close the gap, V2; if V2 doesn't, V4 (V3 is a wildcard — try in parallel with V4 if compute allows).

Each variant gets its own outcome doc (`v1_outcome.md`, `v2_outcome.md`, etc.) and an evolution PNG. Cross-comparison summary in `us2_summary.md` at end of phase.

## Phase 3 — US3: Attention / qualitative architecture jump (CONDITIONAL on US2)

Only if US2 variants don't close the gap. Likely scope-bumps to its own feature spec (029.5 or 031). Listed here only to mark it as the next-step option.

## Open implementation decisions

1. **US2 variant trigger criterion**: at what gen of pastonly1 does the gap warrant US2 commit? Provisional: if at gen 400, fitness gap to more-rnn3 (matched gen) > 30 %, fire V1 variant in parallel with continued pastonly1. Otherwise let pastonly1 finish first.
2. **Multi-seed re-run**: pastonly1 is single-seed for fast turnaround. If results are ambiguous (PARTIAL borderline), commit a 3-seed re-run before US2.
3. **xiao build verification**: defer to phase close or before any flight test against the past-only controller.

## What this plan does NOT cover

- Tracker mode (030)
- Real-flight test of the controller
- GPU-native eval (only relevant under US3)

## Complexity Tracking

No constitution violations. Section intentionally empty.

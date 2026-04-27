# 028 — Implementation Plan (Deeper RNN — D-alone diagnostic, telemetry-gated escalation)

**Branch**: `028-deeper-rnn` | **Date**: 2026-04-26 | **Spec**: [spec.md](./spec.md)
**Input**: [`spec.md`](./spec.md) (5 locked clarifications), [`research.md`](./research.md)
(027-inherited literature priors), [`research_rlayer_placement.md`](./research_rlayer_placement.md)
(028 plan-phase research), [027 findings.md](../027-recurrent-nn/findings.md) (failure-mode evidence base).

## Summary

028 is the iteration that runs the experiments 027 prepared the
ground for. Infrastructure (D-simple recurrent forward pass, 3-axis
ScenarioScore, lexicase pool, span-start reset hook, `nn2cpp` recurrent
emit, crrcsim integration) is already in tree, gated off behind five
`CADENCE7-REDUX` markers. 028 adds **two telemetry signals** (W_hh/W_xh
activation ratio + W_hh population CV), flips the markers to re-enable
recurrence, and runs a **D-alone diagnostic** (recurrent ON, lexicase
OFF, single-seed) as the first experiment. The diagnostic outcome
drives one of four branches per [spec §Clarifications](./spec.md#clarifications):
ship-and-fly, pattern 2 stability-only lexicase, larger budget, or
hidden-state init alternatives. Bounded envelope: D-alone + ≤2 follow-on
patterns at ≤ 5000 pop × ≤ 600 gens each. Either a flight-validated
winner ships, or 028 closes with a documented bounded no-go and
hands off to 029.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis & plotting)
**Primary Dependencies**: Eigen (vec3/dot, mat-vec), cereal (NN serialization), inih (autoc.ini),
  GoogleTest, CRRCSim LaRCSim FDM
**Storage**: File-based — `data.dat` (training), evolution log lines (per-gen telemetry),
  `data.stc` (eval results), `*.dmp` (NNGenome S3 checkpoints), evolution PNGs
**Testing**: GoogleTest (C++), pytest-style scripts (Python analysis)
**Target Platform**: Linux x86-64 (training & analysis); xiao deferred until 028 sim+flight gate
**Project Type**: Research/training feature within an evolved-NN control system. No user-facing UI;
  deliverables are training-run telemetry, evolution PNGs, findings.md, and (on win) a flight-test plan.
**Performance Goals**: Per-generation eval time should not regress vs cadence7-redux baseline by
  more than ~5 % from telemetry instrumentation (signals 1–2 are O(N_weights) and O(N_pop · N_weights)
  respectively — both already-incurred costs in the existing eval/log loop).
**Constraints**: Bounded compute envelope from spec — pop ≤ ~5000, gens ≤ ~600 per attempt,
  ≤ 3 attempts total. Single-seed per attempt (within-run population stats stand in for
  cross-seed replication).
**Scale/Scope**: Three architectural attempts maximum within 028. Two telemetry signals delivered
  ahead of any post-D-alone retraining. One plot extension (6th panel in
  `plot_evolution_progress.py`). Five `CADENCE7-REDUX` marker flips to re-enable recurrence.

## Constitution Check

*Reference: [`.specify/memory/constitution.md`](../../.specify/memory/constitution.md) v1.0.0*

| Principle | Status | Notes |
|---|---|---|
| **I. Testing-First** | PASS | Telemetry-signal computation is testable: deterministic given a known weight/state input. Plan adds unit tests for activation-ratio + W_hh-CV math (`tests/nn_telemetry_tests.cc`). The marker-flip itself is exercised by re-enabling the existing `Selection027*` tests (currently `DISABLED_`). |
| **II. Build Stability** | PASS | All marker flips and telemetry hooks are atomic edits; no half-finished code lands. `bash scripts/rebuild.sh` must remain green at every commit. xiao build untouched (recurrent C-emit code already lands; `nn_program_generated.cpp` regen happens only on win, not in 028). |
| **III. No Compatibility Shims** | PASS | No legacy code paths preserved. `data.dat` schema gains two new columns directly (no opt-in flag). Old `*.dmp` files from rnn1/2/3 remain readable since cereal versioning is intact (per [feedback memory](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md), but no schema bump needed — telemetry signals are evolution-log only, not in NNGenome). |
| **IV. Unified Build** | PASS | No new external deps. Telemetry math uses Eigen (already present); plot extension is Python (already in tree). |

**Gate result**: PASS. No violations require justification in §Complexity Tracking.

## Project Structure

### Documentation (this feature)

```text
specs/028-deeper-rnn/
├── spec.md                         # Locked clarifications (5 Q&A)
├── plan.md                         # This file
├── research.md                     # 027-inherited literature (unchanged) + §9 plan-phase items
├── research_rlayer_placement.md    # Plan-phase research (R-layer placement & sizing first principles)
├── data-model.md                   # Telemetry signal data model + log schema
├── contracts/
│   ├── evolution_log_columns.md    # Per-generation log column contract
│   └── plot_panel_api.md           # plot_evolution_progress.py 6th-panel contract
├── quickstart.md                   # Operator walkthrough: telemetry land → D-alone → branch decisions
└── tasks.md                        # Phase 2 output (NOT created by /speckit.plan)
```

### Source Code (repository root)

```text
include/autoc/nn/
├── topology.h               # Flip NN_RECURRENT[2]=true; static_assert == 1923 (CADENCE7-REDUX marker)
└── evaluator.h              # Existing recurrent forward pass — unchanged

src/nn/
├── evaluator.cc             # Hook signal 1 (W_hh/W_xh activation ratio) into recurrent forward pass
├── population.cc            # Hook signal 2 (W_hh CV) into per-gen population stats
└── serialization.cc         # Unchanged (recurrent flag already serialized)

src/eval/
└── selection.cc             # Uncomment stability + energy pool.push_back (CADENCE7-REDUX marker) —
                             # but kept commented for D-alone (pattern 0); uncommented only for
                             # pattern 2 if D-alone descends but smoothness misses

src/
└── autoc.cc                 # Extend logGenerationStats() with two new log columns

tests/
├── contract_evaluator_tests.cc    # Flip 1667→1923 expectation (CADENCE7-REDUX marker)
├── selection_tests.cc             # Un-DISABLED Selection027* tests (CADENCE7-REDUX marker)
└── nn_telemetry_tests.cc          # NEW — unit tests for signal 1 + signal 2 math

specs/028-deeper-rnn/
└── plot_evolution_progress.py     # NEW — extends 027's plot script with 6th panel for telemetry signals
                                   # (or imports + extends 027's; decide in tasks)
```

**Structure Decision**: Single-project, native to the existing autoc tree. No new top-level
directories. Per [feedback memory](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/feedback_scripts_dir_scope.md),
the new plot variant lives in `specs/028-deeper-rnn/` (feature-scoped one-off), not `scripts/`.

## Ordering principle

**Telemetry before retraining; diagnostic before escalation; evidence before patterns.**

028's prework lists telemetry signals 1 and 2 as **deliverables before
the next post-D-alone experiment runs**. That gates the order:

1. Land telemetry first, on the existing cadence7-redux binary (recurrent OFF) — proves the
   signals compute correctly when there's no recurrent block to measure (signal 1 ≡ 0,
   signal 2 — N/A for non-recurrent layers).
2. Flip the five `CADENCE7-REDUX` markers to re-enable recurrence at 16-wide layer 2.
3. Run the **D-alone diagnostic** (recurrent ON, lexicase OFF, single-seed). This is the
   information-cheapest experiment — bounds what the rest of the plan needs to cover.
4. Branch on D-alone outcome (4 paths per spec).
5. Run pattern 2 (only if needed). Run pattern 3 (only if needed AND budget remains).
6. Close 028 — win (sim gate + flight) or bounded no-go (3 attempts exhausted).

## Phase 1 — Telemetry instrumentation (PRE-D-ALONE)

**Goal**: Two cheap per-generation signals land in the evolution log + plot, on the
existing cadence7-redux binary, *before* recurrence is re-enabled.

### 1.1 Signal 1 — W_hh / W_xh activation ratio (best individual, per generation)

**What**: For the best individual at end-of-generation, accumulate over a representative scenario
replay (or piggyback on the existing best-of-gen eval pass): `mean_j max_t |W_hh[j] · h_{t-1}[j]|`
vs `mean_j max_t |W_xh[j] · x_t[j]|` per recurrent neuron, ratio = numerator / denominator.

**Hook**: `src/nn/evaluator.cc` recurrent forward-pass loop (around lines 198–211 per the
spec's clarify §Q2 reference). Add a magnitude-accumulator pass conditional on a
`telemetry_capture` flag set by `autoc.cc` for best-of-gen evaluation only (avoid per-individual
cost during selection).

**Output**: One float per generation, written as a new column in the per-generation evolution
log (currently `#NNGen` line in stdout/log file). Column name: `whh_xh_ratio`.

**Sentinel value**: When recurrent layer is disabled (`NN_RECURRENT[2]=false`), emit `0.0` —
allows the cadence7-redux smoke run in §1.4 to verify the signal pipeline without the
recurrent layer interfering.

### 1.2 Signal 2 — W_hh population coefficient of variation (CV)

**What**: For each weight block separately (`W_xh[layer 0]`, `W_xh[layer 1]`, `W_hh[layer 2]`),
compute population-level standard deviation / mean(|weight|) across all individuals — i.e., a
coefficient of variation per block.

**Hook**: `src/nn/population.cc` (or `src/autoc.cc` `logGenerationStats(gen)` — pick the
location that already iterates the population for stats; minimize new iteration).

**Output**: Three floats per generation: `w_xh0_cv`, `w_xh1_cv`, `w_hh_cv`.

### 1.3 Plot 6th-panel extension

`specs/028-deeper-rnn/plot_evolution_progress.py` extends 027's 5-panel plot
(fitness | streak | stability | energy | sigma) with a 6th panel:
- Top half: signal 1 (activation ratio) over generations, with a horizontal threshold line
  (calibrated from §1.5 below).
- Bottom half: signal 2 (three CV traces overlaid) over generations.

If the script can import 027's plot rather than fork it, prefer that. Decision deferred to
`/speckit.tasks`.

### 1.4 Cadence7-redux smoke (telemetry-only, no architecture change)

Short run (~30–50 gens) on the unchanged cadence7-redux binary with telemetry merged in.
Verify:
- `whh_xh_ratio == 0.0` for all generations (no recurrent layer present).
- `w_xh0_cv` and `w_xh1_cv` produce sane values (decreasing trend as population converges,
  not NaN/inf).
- `w_hh_cv` emits a sentinel (zero or NaN — decide in tasks) when no recurrent layer exists.
- 6th-panel plot renders cleanly.
- Per-generation eval time hasn't regressed by more than ~5 %.

**Gate**: All four checks pass before §Phase 2 marker flips.

### 1.5 Threshold calibration (informs §1.3 plot threshold line)

Reference values for the activation ratio:
- **Lower bound** ("block dead"): hand-zero W_hh, run forward pass, ratio should be exactly 0.
- **Upper bound** ("block engaged"): hand-set W_hh to identity (or a known-active matrix),
  measure ratio. Establishes order-of-magnitude for "actively used."

Two ad-hoc unit tests in `tests/nn_telemetry_tests.cc` cover both. Threshold for the plot
horizontal line: midpoint of the two bounds, or 10 % of the engaged value — pick in tasks.

## Phase 2 — Re-enable D-simple (CADENCE7-REDUX marker flips)

Flip the five markers documented in [spec §Carry-forward inventory](./spec.md#carry-forward-inventory):

| File | Change |
|---|---|
| `include/autoc/nn/topology.h:52-57` | `NN_RECURRENT[2] = true` |
| `include/autoc/nn/topology.h:69` | `static_assert == 1923` |
| `src/eval/selection.cc:66-69` | **LEAVE COMMENTED for D-alone** — only uncomment if pattern 2 runs |
| `tests/contract_evaluator_tests.cc:14` | Expect 1923 / `NN_RECURRENT[2]=true` |
| `tests/selection_tests.cc:152+` | Rename `DISABLED_Selection027*` back **only if pattern 2 runs** |

**Note**: Marker 3 (selection.cc lexicase pool) and marker 5 (selection_tests) stay commented
for D-alone. They flip on if-and-only-if pattern 2 (stability-only lexicase) runs.

**Build verification**: `bash scripts/rebuild.sh` green; all tests pass; weight count = 1923.

## Phase 3 — D-alone diagnostic (single-seed, recurrent ON, lexicase OFF)

```bash
nohup ./build/autoc -c autoc.ini > logs/autoc-028-dalone.log 2>&1 &
```

Budget: 400 generations (matched-compute with cadence7-redux baseline), pop 3500 (current
cadence). Single seed.

**During run** — every 50 gens, regenerate:
- 6-panel evolution PNG (fitness | streak | stability | energy | sigma | telemetry).
- Control aggressiveness PNG (existing tool).

**Early-stop conditions** (only if clearly diverged from any productive path):
- Fitness above pid1's −27045 floor by gen 100 with no descent trend.
- `whh_xh_ratio` flat-lined near zero across gens 50–150 (recurrent block never engages
  — tells us we're in failure-mode H1/H4 territory and bigger budget at this topology won't
  help).

## Phase 4 — Branch on D-alone outcome

Per [spec §Required prework](./spec.md#required-prework) outcome table:

| D-alone outcome | Spec branch | Plan phase to execute |
|---|---|---|
| Descends ≤ −30000 + smoothness gates hit (dCtrl ≤ 0.80, ⟨\|out\|⟩ ≤ 2.00) | Ship + flight | Phase 6 |
| Descends ≤ −30000 but smoothness gates miss | Pattern 2: stability-only lexicase | Phase 5a |
| Stalls similar to rnn1/2/3 | Larger budget at 16-wide layer 2 | Phase 5b |
| Descends partway then stalls | Hidden-state init alternatives | Phase 5c (telemetry-gated) |

The telemetry signals from §Phase 1 inform which 5b-5c sub-path applies if both stall paths
are open. If `w_hh_cv` is much lower than `w_xh*_cv`, GA isn't searching W_hh effectively →
5b (more budget). If `whh_xh_ratio` saturates near zero → 5c (state-init).

## Phase 5 — Pattern 2 (only if Phase 4 indicates)

**Escalation ordering (revised by [`research_rlayer_placement.md`](./research_rlayer_placement.md) §5):**

The inherited 027 escalation ladder went *larger budget → 32-wide → state-init*. Plan-phase
research promotes **orthogonal W_hh init ahead of topology changes** because it's a cheaper,
stronger lever for tanh-RNN under evolutionary search and directly addresses failure-mode H4.
Revised ladder for 028:

| Sub-phase | Trigger from §Phase 4 telemetry | Lever |
|---|---|---|
| 5a | Descends + smoothness misses | Stability-only single-axis lexicase (incentive change, not architecture) |
| 5b | Stalls; `whh_xh_ratio` flat near zero | **Orthogonal W_hh init** at 16-wide layer 2 (state-init change, ~1 file) |
| 5c | Stalls; `w_hh_cv` low vs `w_xh_cv` healthy | Larger search budget at 16-wide layer 2 (no code change, autoc.ini bump) |
| 5d | Stalls AND 5b/5c exhausted | 32-wide layer 2 OR layer-1 placement — only if budget remains |

Each sub-phase is a separate attempt against the spec's 3-attempt envelope. Pattern 2 is
whichever the telemetry indicates first; pattern 3 is the second-best signal.

### 5a — Stability-only single-axis lexicase

- Flip CADENCE7-REDUX markers 3 + 5 (selection.cc + tests/selection_tests.cc).
- **Modify** marker 3: only uncomment the *stability* push, leave energy commented.
  (027's 3-axis pool was tracking + stability + energy; pattern 2 here is tracking + stability,
  2-axis.)
- Re-run with same pop/gens/seed as D-alone. Log: `logs/autoc-028-pattern2-5a.log`.

### 5b — Orthogonal W_hh init (PROMOTED ahead of topology changes)

- `src/nn/population.cc` Xavier init for W_hh → orthogonal init (Saxe et al. 2013;
  per [`research_rlayer_placement.md`](./research_rlayer_placement.md) §4).
- W_xh, W_out blocks unchanged.
- Optional: post-init sanity assert that W_hh spectral radius lands in [0.7, 1.05] (research §4).
- Re-run with D-alone config (recurrent ON, lexicase OFF). Log: `logs/autoc-028-pattern2-5b.log`.

### 5c — Larger budget at 16-wide layer 2

- Bump `nMembers` and/or `nGenerations` in autoc.ini to 5000 / 600 (per spec envelope cap).
- No code changes. Single seed. Log: `logs/autoc-028-pattern2-5c.log`.

### 5d — 32-wide layer 2 OR layer-1 placement (last-resort, only if budget remains)

- Per [`research_rlayer_placement.md`](./research_rlayer_placement.md) §1+§2: prefer
  24-wide intermediate before jumping to 32-wide. Hidden dim 24 → W_hh = 576, total weights
  ≈ 2243. 32-wide is W_hh = 1024, total 2691.
- Escalation outside the bounded envelope unless 5a/5b/5c freed up significant budget; if the
  3-attempt envelope is exhausted, 5d is **explicitly out of scope** and 028 closes no-go
  per Phase 6.

## Phase 6 — Close (win or bounded no-go)

### Win path: sim gate + flight

1. Confirm a candidate winner hit the [spec validation gate](./spec.md#validation-gate-carries-from-027-plan)
   triple-bar. Population late-gen fitness spread sane (per spec Q5 mitigation —
   non-degenerate; metric defined in [`data-model.md`](./data-model.md)).
2. Generate `nnextractor` weights → `nn2cpp` → xiao firmware (separate xiao port plan,
   triggered by this win).
3. Flight test on the winning architecture. Successful flight = 028 closes, ships winner.

### Bounded no-go path: 3 attempts exhausted

1. Produce `specs/028-deeper-rnn/findings.md` summarizing: telemetry signal evolution
   across all attempts, the failure-mode hypothesis the data best supports, what
   carry-forward to 029.
2. Hand off to 029 with code in tree, telemetry signals as durable infrastructure.

## What this plan intentionally does NOT cover

- **Xiao port** — deferred behind sim gate. Triggers a separate xiao plan on win.
- **Flight test plan** — same.
- **Pattern 4+** — out of envelope. If 3 attempts don't land it, 028 closes no-go.
- **Selection regime alternatives (NSGA-II, Pareto)** — per spec, single-axis stability
  doesn't engage the equal-pressure failure mode that motivated revisiting selection.
- **Scenario-physics changes** — the throttle saturation observation from spec §Q4
  is captured as 029 carry-forward, explicitly out of 028 scope.
- **Hypothesis-3 ε-floor recalibration as a primary experiment** — 028's pattern 2
  is single-axis, where ε-scaling tradeoffs are bounded; full per-axis ε re-derivation
  is in [`research_rlayer_placement.md`](./research_rlayer_placement.md)'s adjacent
  research items but only triggers if pattern 2 itself stalls.

## Open implementation decisions (resolve in tasks)

1. **Telemetry capture cadence** — every gen, or every Nth gen for cost savings? Default: every gen.
2. **Plot panel layout** — extend 027's `plot_evolution_progress.py` directly, or fork to
   `specs/028-deeper-rnn/plot_evolution_progress.py`?
3. **w_hh_cv sentinel value when no recurrent layer present** — zero or NaN?
4. **Activation ratio aggregation** — mean across recurrent neurons, max, or both?
5. **Population spread metric for Q5 robustness check** — see [`data-model.md`](./data-model.md)
   for proposed definition; final picked in tasks.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified.**

No violations. Section intentionally empty.

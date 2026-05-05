# Feature Specification: 029 — No-Future-Input NN Architecture

**Feature Branch**: `029-tracker-mode` (working branch retains the original name; renamed scope inside the branch — see [pivot rationale](#pivot-rationale-2026-04-30) below)
**Created**: 2026-04-29 (original) — pivoted 2026-04-30
**Status**: Active (US1 training run live: `more-rnn4-pastonly1`)
**Input**: Develop a controller NN architecture that trains effectively from **past-only sensor inputs** (no future lookahead). This is a prerequisite for 030 tracker-mode, which has no parametric path to look ahead on.

## Overview

The current pathgen-mode controller (more-rnn3 baseline) trains with a 6-slot input pattern that includes **future** target-direction samples at +0.1s and +0.5s — synthesized by looking ahead on the known parametric rabbit path. This works because pathgen has a known curve; the NN gets the rabbit's near-future as an explicit input.

**Tracker mode (030) has no such oracle.** A target craft's future trajectory is not known to the controller, so any future-lookahead input is fictional.

To unlock 030, the controller architecture must be able to train *without* future inputs and still achieve fitness comparable to the more-rnn3 (with-future) baseline. **029's job is to find that architecture.** Initial experiment: redistribute the 6 input slots to past-only at uniform 100 ms spacing and observe how the existing recurrent NN (D-simple, 16-wide layer 2, 1923 weights) responds.

## Pivot rationale (2026-04-30)

The original 029-tracker-mode spec attempted the whole tracker-mode feature in one shot — 6 user stories, 22 functional requirements, multi-aircraft sim, beacon projection, library tooling, renderer, etc. After the US1 past-only experiment kicked off and started producing data, an insight surfaced:

> *Past-only inputs without an oracle for future motion mean the controller can only **react**, not **predict**. For tracker mode (030), where the target is a real-moving aircraft, that's the deployment reality. The current 1923-weight D-simple recurrent NN may not have enough capacity to evolve a predictor inside its hidden state — and if so, the next architectural step (more memory, attention, hybrid predictor head, etc.) is its own feature-class question.*

So 029 was pivoted to focus on **just** that question: develop an NN architecture that trains effectively from past-only inputs. The tracker-mode feature is moved out to [`specs/030-tracker-mode/`](../030-tracker-mode/) where it waits for 029 to clear.

**Branch naming note**: this work continues on the existing branch `029-tracker-mode` despite the pivoted scope. Branch will be renamed (or a fresh `029-no-future-arch` branch cut) at some appropriate later date; for now the branch label is historical.

## User Stories

### User Story 1 — Past-only baseline experiment (Priority: P1) 🎯 RUNNING

**Status**: Active. Run name `more-rnn4-pastonly1`. Launched 2026-04-30, gen 134 at last refresh, training continues.

**The experiment**: existing 028 codebase + recurrent NN architecture (D-simple, 1923 weights, NN_RECURRENT[2]=true), with the input time-offset distribution changed from past+future `[-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]` to **past-only `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`** (uniform 100 ms grid). Path A `autoc.ini` config (pop 5000 × 800 gens, recurrent NN, single seed). Everything else identical to more-rnn3.

**Pass criterion** (per [project_late_run_fitness_interpretation.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md)):
- Late-plateau fitness within ±10 % of more-rnn3 under fixed-difficulty eval
- Per-axis aggressiveness (dCtrl, ⟨|out|⟩) shape architecture-consistent with more-rnn3 (roll dominates per [project_bangbang_axis_migration.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md))
- pctInStreak emergence on schedule

**Branch decision** (recorded in `pastonly1_outcome.md` when the run completes or stalls):

| Outcome | Implication | Next |
|---|---|---|
| **PASS** (within ±10 %, smooth descent) | The existing D-simple recurrent NN has enough capacity to evolve a predictor inside its hidden state from past-only inputs. | Proceed to 030 tracker-mode — the architecture is ready. |
| **PARTIAL** (gap is 10-30 %, descent shape similar but fitness floor higher) | Architecture trains but is operating at a meaningful disadvantage vs. with-future. Worth trying capacity / arch tweaks before commits to 030. | Trigger US2 (architectural variants). |
| **FAIL** (>30 % regression OR descent stalls / per-axis pattern broken) | Past-only is not just a fitness-tax issue — the architecture isn't capable. | Re-architect (US2/US3). 030 blocked until this resolves. |

### User Story 2 — Architectural variants for capacity (Priority: P2 — conditional on US1 PARTIAL/FAIL)

If US1 lands PASS, US2 is skipped. If PARTIAL or FAIL, the question is *which architectural lever closes the gap*. Candidate variants, in rough cost order:

1. **More recurrent state**: widen layer 2 from 16 → 32 (or add a second recurrent block). Doubles `W_hh` weight count; moderate compute increase. Tests whether hidden-state capacity is the bottleneck.
2. **Deeper recurrent stack**: e.g., {33, 32, 16r, 16r, 3} — two stacked recurrent blocks. Tests temporal-depth.
3. **Wider input encoding**: stack more history slots (e.g., 12 past instead of 6) and let the first dense layer learn temporal features. Trades genome size for explicit temporal richness.
4. **Hybrid predictor head**: split the network into a *predictor* sub-network (extrapolates target +0.1s / +0.5s from past) and a *control* sub-network (consumes the predicted future plus current state). Mirrors the with-future baseline architecture but generates the future internally. Bigger structural change but most directly answers the "can it learn to predict" question.

Each variant is a separate experimental branch off the same Path A config, single-seed, 600+ gens. Compare fitness + per-axis shape vs more-rnn3 + pastonly1.

### User Story 3 — Attention / non-recurrent architectural variants (Priority: P3 — conditional on US2 not closing the gap)

If recurrent-capacity tweaks (US2) don't suffice, the next jump is qualitatively different architectures:

- **Attention over history**: a small attention block that learns to weight past inputs by relevance. Trades recurrent state for explicit attention parameters.
- **Transformer-style block**: small N-token transformer over the history window. Higher weight count + compute.
- **Temporal-convolution stack**: 1D conv over the history axis instead of recurrence. Lighter than transformer, denser than dense.

These are bigger commitments — likely require GPU-native eval. Out of scope unless US1+US2 evidence strongly points here.

## Functional Requirements

- **FR-001**: System MUST train the existing recurrent NN with the past-only input distribution `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]`. *(US1 — done, training active.)*
- **FR-002**: System MUST log per-gen fitness telemetry comparable to more-rnn3 — same `#NNGen` schema with `whh_xh_ratio` / `w_*_cv` recurrent telemetry. *(Already preserved by the 028 telemetry plumbing.)*
- **FR-003**: System MUST produce a comparable per-axis aggressiveness signature (the existing `plot_per_axis_time_series.py` works unchanged — verified at gen 134, roll-dominant pattern matches RNN-architecture expectation).
- **FR-004**: System MUST emit a fixed-difficulty eval for the late-run elite so cross-run fitness comparison is variation-stable. *(Existing `--eval` mode, no changes.)*
- **FR-005** (conditional): If US1 result triggers US2, the system MUST support architectural variants without requiring NN_TOPOLOGY-format breakage in serialized weights. Variant-A topology must coexist with the existing topology in the same tree (separate branch / config).
- **FR-006** (out-of-scope guard): System MUST NOT introduce tracker-mode infrastructure (multi-aircraft sim, beacon projection, library, perception interface, renderer dual-mode, etc.) — those are 030's scope.

## Success Criteria

- **SC-001** (US1 gate): pastonly1 late-plateau fitness within ±10 % of more-rnn3 under fixed-difficulty eval. *Measured 2026-05-XX when the run completes.*
- **SC-002** (US1 gate): Per-axis aggressiveness pattern on US1 is RNN-architecture-consistent (roll dominates dCtrl, throttle saturates) — confirmed at gen 134, watch for regression at late-plateau.
- **SC-003** (option-pinning): If US1 fails its gates, the architectural-variant table in US2 narrows to ≤2 candidates worth committing compute to. Outcome doc names the chosen path.
- **SC-004** (030 readiness): At 029 close, the chosen architecture's recurrent-block telemetry (`whh_xh_ratio` ≥ 0.3 sustained) demonstrates the hidden state is being used — the architecture is engaging memory, not just function-fitting on the dense slots.

## Out of Scope

- Tracker-mode infrastructure (030)
- Beacon-camera perception, library construction, multi-aircraft sim, renderer dual-mode — all 030
- Real-flight test of the no-future controller — gates on next flight cycle, separate concern
- GPU-native eval — only relevant if US3 architectural variants land

## Dependencies and connections

- **Inherits from 028**: D-simple recurrent (NN_RECURRENT[2]=true), telemetry signals (`whh_xh_ratio`, `w_*_cv`), Path A config
- **Inherits from prior runs**: more-rnn3 as fitness comparator (`more-rnn3-data.stc`); cadence7-redux for legacy-format reference
- **Blocks**: 030 tracker-mode (waits for 029 to clear)
- **Sibling concerns**: pending flight-test cycle on more-rnn3 (decoupled — flight test outcome doesn't gate 029)

## Memory references

- [project_library_based_training.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_library_based_training.md) — strategic arc 030 belongs to (this is the prereq)
- [project_post_028_routing.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_post_028_routing.md) — routing context that produced this pivot
- [project_bangbang_axis_migration.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md) — RNN-architecture per-axis signature predictor
- [project_late_run_fitness_interpretation.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md) — variation-ramp-aware fitness comparison

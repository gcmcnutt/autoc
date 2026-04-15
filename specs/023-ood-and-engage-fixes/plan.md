# Implementation Plan: 023 — Change 9: Control Chatter Streak

**Branch**: `023-ood-and-engage-fixes` | **Date**: 2026-04-13 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/023-ood-and-engage-fixes/spec.md` Change 9
**Context**: Phase 9a (pt3 filter) ABANDONED. This plan covers the fitness-based alternative.

## Summary

Add a second lexicase selection dimension measuring control smoothness via a
streak-based chatter metric. Per-tick 3D command delta (`sqrt(du_pitch² +
du_roll² + du_throttle²)`) is accumulated with a streak multiplier that
rewards sustained gentle control. Lexicase selects individuals with good
tracking AND low chatter — no tuning of relative weights needed.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), Python 3.11 (analysis)
**Primary Dependencies**: Eigen (math), cereal (serialization), GoogleTest
**Storage**: File-based — data.dat, data.stc, NN01 weights
**Testing**: GoogleTest (selection_tests.cc, fitness_decomposition_tests.cc)
**Target Platform**: Linux aarch64 (training), Xiao BLE Sense (deploy)
**Project Type**: Neuroevolution training pipeline + embedded controller
**Performance Goals**: ~16k sims/sec training throughput, deterministic eval
**Constraints**: All serialized values must be float (not double). NN eval is single-precision.
**Scale/Scope**: Pop 3500, 245 scenarios/individual, 400 gens, ~5h wall time

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Testing-First | PASS | Tests for 2-dim lexicase selection, chatter computation, streak logic |
| II. Build Stability | PASS | Incremental changes, each commit builds clean |
| III. No Compatibility Shims | PASS | Clean addition — new field in ScenarioScore, new lexicase dimension |
| IV. Unified Build | PASS | No new build targets or dependencies |

## Project Structure

### Documentation (this feature)

```text
specs/023-ood-and-engage-fixes/
├── plan.md              # This file
├── research.md          # Phase 0: chatter metric design decisions
├── data-model.md        # Existing — ScenarioScore extension
├── contracts/           # Existing — evaluator contract extension
├── remaining-work.md    # Updated priority list
├── spec.md              # Change 9 added
└── tasks.md             # Phase 2 output (existing, to be updated)
```

### Source Code (repository root)

```text
include/autoc/eval/
├── fitness_computer.h       # Streak logic (reuse or generalize)
├── fitness_decomposition.h  # ScenarioScore: add chatterCost field
└── selection.h              # lexicase_select: 2-dim signature

src/eval/
├── fitness_computer.cc      # Streak implementation
├── fitness_decomposition.cc # Compute chatterCost per scenario
└── selection.cc             # 2-dim lexicase filter

include/autoc/util/
└── config.h                 # ChatterStreak* config fields

src/util/
└── config.cc                # Parse ChatterStreak* from autoc.ini

src/
└── autoc.cc                 # data.dat: add du column; data.stc: chatter stats

tests/
├── selection_tests.cc       # 2-dim lexicase tests
└── fitness_decomposition_tests.cc  # chatter computation tests
```

## Complexity Tracking

No constitution violations. Clean addition to existing architecture.

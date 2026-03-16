# Feature Specification: NN Training Improvements

**Feature Branch**: `015-nn-training-improvements`
**Created**: 2026-03-16
**Status**: Draft (migrated from 014 Phases 6-13)
**Predecessor**: 014-nn-training-signal (replumbing complete, NN-only standalone autoc)

## Context

Feature 014 achieved standalone NN-only autoc: GP removed, Boost removed, source reorganized,
crrcsim/xiao-gp integrated as submodules. This feature picks up the training improvements
that 014 scoped but did not implement.

The immediate problem: nn13 hit a wall at fitness 3.99M (sigma collapsed 0.20→0.024).
These features address the root cause and add richer training signal.

## User Stories (from 014 spec, unchanged)

### US1 — Sigma Floor (P1, MVP)
Prevent search freeze by clamping mutation sigma to configurable minimum.
Quick fix for nn13 stall. Minimal code change.

### US2 — Curriculum Scenario Ramp (P1)
Progressive difficulty: start with few scenarios, ramp to full 49-scenario suite.
Leverages existing VariationRampStep infrastructure.

### US3 — Per-Scenario Fitness Decomposition (P2)
Minimax/percentile aggregation instead of sum. Forces robust controllers.

### US4 — sep-CMA-ES Optimizer (P2)
Replace GA with sep-CMA-ES for 531-dim weight space. Population drops from 5000→~50.

### US7 — Per-Timestep Fitness Streaming (P3)
Return per-timestep data from simulator. Infrastructure for US5 and future work.

### US5 — Per-Segment Credit Assignment (P3)
Score trajectory segments by error reduction. Depends on US7.

### US8 — Checkpoint/Resume (P3)
Save full evolution state per generation for crash recovery.

## Implementation Order

1. US1 (Sigma Floor) — independent, quick win
2. US2 (Curriculum) — independent, can parallel with US1
3. US3 (Fitness Decomposition) — independent
4. US4 (sep-CMA-ES) — independent
5. US7 (Timestep Streaming) — enables US5
6. US5 (Segment Scoring) — depends on US7
7. US8 (Checkpoint/Resume) — T119 depends on US4

## Success Criteria (from 014)

- SC-001: Fitness improvement beyond gen 500, ≥20% below 3.99M plateau
- SC-002: Curriculum achieves equivalent fitness in fewer evaluations
- SC-003: Minimax worst-case ≥30% better than sum aggregation
- SC-004: sep-CMA-ES equivalent fitness in ≤1/10th evaluations
- SC-006: Checkpoint/resume produces bit-identical trajectories

## References

- Full acceptance scenarios: [014 spec](../014-nn-training-signal/spec.md) (User Stories section)
- Research notes: [014 research](../014-nn-training-signal/research.md)
- Data model: [014 data-model](../014-nn-training-signal/data-model.md)
- Contracts: [014 contracts](../014-nn-training-signal/contracts/)

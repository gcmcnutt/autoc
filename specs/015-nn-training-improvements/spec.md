# Feature Specification: NN Training Improvements

**Feature Branch**: `015-nn-training-improvements`
**Created**: 2026-03-16
**Status**: Draft
**Predecessor**: 014-nn-training-signal (replumbing complete, NN-only standalone autoc)

## Problem Statement

The NN controller (22 inputs → 16 → 8 → 3 outputs, 531 weights) tracks an all-attitude 3D
aerobatic path under 49 deterministic scenarios (wind direction/speed variations, entry
point/heading/speed variations, craft performance variations). Each scenario runs 600 timesteps
at 10Hz (60 seconds).

**Current fitness**: Sum of (distance-to-path + attitude penalties) across all 600 timesteps ×
49 scenarios, collapsed to a single scalar. This is fundamentally broken:

1. **Signal masking**: 29,400 data points → 1 number. A controller that's excellent on 48
   scenarios but crashes on 1 scores poorly. A controller that tracks perfectly for 550 steps
   but diverges in the last 50 also scores poorly. The optimizer can't distinguish these
   failure modes from general incompetence.

2. **Bang-bang control**: The fitness function only measures distance/attitude error. There's
   no pressure toward smooth control. Small NNs naturally produce bang-bang outputs (saturated
   ±1) because that's what minimizes tracking error in the short term. This is unsuitable
   for a physical aircraft.

3. **Brittleness under mutation**: A small NN's weight space is tightly coupled. Standard
   GA mutation (isotropic Gaussian per-weight) is destructive — a single weight change can
   collapse all 49 scenarios. Crossover is even worse. The search gets stuck in local optima
   where sigma collapses (0.20→0.024 in nn13).

4. **No temporal credit**: A controller that makes a good correction during a hard turn gets
   no more credit than one that was already on-path during a straight segment. Error
   *reduction* is invisible in cumulative error.

## Domain Constraints

- **Deterministic simulator**: Given identical NN weights + scenario, output is bit-identical.
  This means the optimizer can see arbitrarily fine signal — if we decompose fitness properly.
- **Short-term planning only**: The controller can only see ~2-3 path points ahead (via
  GETDIST, GETDPHI, GETDTHETA sensors). It's a reactive tracker, not a planner. Upper-level
  strategy (resource management, intercept vs track mode) is future work.
- **Physical deployment**: The NN runs on a Xiao BLE Sense (ARM Cortex-M4). Control must be
  smooth enough for real servos. Bang-bang is not acceptable.
- **Multi-objective reality**: Even at this low level, the controller balances competing
  objectives: tracking accuracy, attitude stability, control smoothness, and robustness
  across scenarios. A single weighted sum conflates these.

## Approach: Decomposed Fitness + Multi-Objective Selection

### Fitness Decomposition

Workers (minisim/crrcsim) compute and return structured score vectors instead of a single
scalar. The return format is per-scenario × per-component:

```
EvalResponse {
    scenario_scores[49] {
        distance_rmse      // path tracking accuracy
        attitude_error     // orientation penalty
        control_smoothness // Σ|Δu(t)| or jerk measure
        segment_scores[]   // per-segment credit (optional, Phase 4)
    }
}
```

Heavy math stays on the worker side (scaleout). Autoc decides aggregation strategy.

### Multi-Objective Selection

Instead of a weighted sum, maintain separate objectives and use selection pressure that
respects the multi-objective structure:

- **Objective 1 — Tracking accuracy**: Per-scenario distance RMSE (or CTE). Aggregated
  via minimax (worst-case) or percentile to force robustness.
- **Objective 2 — Control smoothness**: Mean |Δu(t)| across all scenarios. Directly
  penalizes bang-bang. Separate objective, not mixed into distance.
- **Objective 3 — Worst-case robustness**: Variance or max-min spread across scenarios.
  Low variance = consistent controller.

Selection strategies to explore (research spikes):
- **NSGA-II style Pareto ranking**: Non-dominated sorting + crowding distance.
- **Lexicase selection**: Each scenario is a test case, random ordering per selection event.
  Naturally maintains specialists and generalists without explicit diversity pressure.
- **Weighted aggregation** (baseline): Traditional weighted sum as control comparison.

### Temporal Credit Assignment

Per-segment scoring rewards error *reduction*, not just low absolute error:
- Divide each 600-step flight into segments (e.g., by path geometry: straights, turns,
  Immelmans).
- Score each segment by: did the controller *improve* tracking during this segment?
- Weight segments by difficulty (turn rate, crosswind component).
- A controller that nails a hard turn but drifts on a straight is more valuable than one
  that does the opposite — the hard turn is the bottleneck.

## User Stories (revised)

### US1 — Sigma Floor (P1, quick fix)
Prevent search freeze by clamping mutation sigma to configurable minimum.
Unblocks experimentation while deeper fitness changes are developed.

### US9 — Structured Fitness Return (P1, infrastructure)
Workers return per-scenario × per-component score vectors instead of single scalar.
Enables all downstream selection strategies without re-running simulations.
This is the foundational change — everything else builds on it.

### US10 — Control Smoothness Objective (P1)
Add control smoothness (Σ|Δu(t)|) as a separate computed metric per scenario.
Workers compute it alongside distance/attitude. Directly addresses bang-bang.

### US3 — Multi-Objective Selection (P1)
Replace single-scalar selection with multi-objective aggregation. Start with minimax
on worst-scenario tracking + smoothness threshold. Research Pareto/lexicase as spikes.

### US11 — Per-Segment Temporal Credit (P2)
Divide flights into path-geometry segments. Score by error reduction per segment,
weighted by segment difficulty. Workers compute segment boundaries and scores.

### US12 — Selection Strategy Research Spikes (P2)
Branch-based experiments comparing:
- NSGA-II Pareto ranking
- Lexicase selection (one scenario = one test case)
- MAP-Elites / novelty search (behavioral diversity in trajectory space)
- Rank-based fitness shaping (removes scale issues, like CMA-ES internally)

### US4 — sep-CMA-ES Optimizer (P3, branch experiment)
Replace GA with sep-CMA-ES. Explore on branch after fitness signal quality improves.
Population drops from 5000→~50. Transitional: replaces GA if validated.

### US2 — Curriculum Scenario Ramp (P3, deferred)
Progressive difficulty. Revisit after fitness decomposition is understood.

### US8 — Checkpoint/Resume (P3, deferred)
Save full evolution state per generation for crash recovery.

## Implementation Order

1. **US1** (Sigma Floor) — quick fix, unblocks running experiments now
2. **US9** (Fitness Decomposition in autoc) — refactor computeNNFitness() to retain
   per-scenario × per-component scores. No RPC change — autoc already has the data.
3. **US3** (Multi-Objective Selection) — minimax/percentile on decomposed scores
4. **US12** (Research Spikes) — branched experiments: lexicase, NSGA-II, MAP-Elites,
   rank-based shaping. This is the core investigation.
5. **US10** (Smoothness) — deferred until research shows how to apply it
6. **US11** (Per-Segment Credit) — deferred until temporal signal is needed
7. **US4** (sep-CMA-ES) — optimizer experiment on branch, after fitness is sorted
8. **US2** (Curriculum) — deferred
9. **US8** (Checkpoint) — deferred

Steps 2-4 form the core hypothesis test: does decomposed multi-objective fitness
break through the nn13 plateau? Keep the experiment loop tight — no RPC or worker
changes during research.

## Success Criteria

**Core hypothesis**: Decomposed fitness with multi-objective selection produces controllers
that are (a) not bang-bang, (b) robust across scenarios, and (c) continue improving past
where scalar fitness plateaus.

- **SC-001**: Worst-scenario tracking RMSE improves monotonically through gen 500+
  (no plateau). Measured per-scenario, not aggregate.
- **SC-002**: Control smoothness (mean |Δu(t)|) below threshold TBD — no sustained
  bang-bang on any axis for any scenario.
- **SC-003**: Minimax worst-case tracking ≥30% better than sum-aggregation baseline.
- **SC-004**: Variance across 49 scenarios decreases over generations (robustness improves).
- **SC-005**: Per-segment scores show credit flowing to hard segments (turns, crosswind).
- **SC-006**: Checkpoint/resume produces bit-identical trajectories.

## Clarifications

### Session 2026-03-16
- Q: When US9 changes the RPC response format, must old CRRCSim binaries still work? → A: No — both sides update together, single protocol version.
- Q: Is OptimizerType a permanent config switch or transitional? → A: Transitional — validate CMA-ES on branch, replace GA if it wins, remove old code.
- Q: Is single-scalar fitness the right metric? → A: No — collapsing 29,400 data points into one number masks training signal. Fitness decomposition is the primary investigation.
- Q: Should fitness decomposition move ahead of curriculum? → A: Yes — US1 first (quick fix), then structured fitness return + multi-objective selection as primary investigation. Curriculum deferred.
- Q: What level of detail should workers return? → A: Partial reductions — workers compute per-scenario × per-component score vectors. Heavy math on worker side, autoc handles aggregation strategy.

## Constraints

- **RPC protocol**: No backward compatibility required. autoc and crrcsim are co-versioned via submodule and always deployed as a pair.
- **Optimizer strategy**: New optimizers developed on branches, validated against baseline. Winner replaces loser — no permanent config switches.
- **Fitness compute distribution**: Workers compute per-scenario × per-component scores. RPC returns structured vectors. Autoc decides aggregation. Scales with worker count.
- **Embedded target**: Controller must produce smooth outputs suitable for physical servos on ARM Cortex-M4.

## References

- Full acceptance scenarios: [014 spec](../014-nn-training-signal/spec.md)
- Research notes: [014 research](../014-nn-training-signal/research.md)
- Data model: [014 data-model](../014-nn-training-signal/data-model.md)
- Contracts: [014 contracts](../014-nn-training-signal/contracts/)
- Coordinate conventions: [docs/COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md)

### Literature & Techniques to Investigate
- **NSGA-II**: Deb et al., multi-objective evolutionary optimization with Pareto ranking
- **Lexicase selection**: Helmuth et al., per-test-case selection without aggregation
- **MAP-Elites**: Mouret & Clune, quality-diversity via behavioral feature space
- **Novelty search**: Lehman & Stanley, diversity pressure to escape local optima
- **CMA-ES**: Hansen & Ostermeier, covariance matrix adaptation for continuous optimization
- **Fitness shaping**: Rank-based transformation (used internally by CMA-ES)
- **DTW (Dynamic Time Warping)**: Temporal alignment for comparing trajectories with speed variation

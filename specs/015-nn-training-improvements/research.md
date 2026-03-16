# Research: NN Training Improvements

**Feature**: 015-nn-training-improvements
**Date**: 2026-03-16

## R1: Fitness Signal Analysis — Why Scalar Aggregation Fails

### Problem

Current fitness (`computeNNFitness()` in autoc.cc:614-709) computes:

```
fitness = Σ_scenarios Σ_timesteps (distance_penalty + attitude_penalty * attitude_scale) * intercept_scale
        + crash_penalty
```

This collapses 49 scenarios × 600 timesteps = 29,400 data points into one double.

### Failure Modes

1. **Scenario masking**: A controller excellent on 48/49 scenarios but crashing on 1 gets a
   catastrophic crash penalty (~1e6) that dominates. The optimizer can't distinguish "almost
   robust" from "generally bad."

2. **Temporal masking**: A controller that tracks perfectly for 550 steps then diverges scores
   similarly to one that's mediocre throughout. The cumulative sum hides *when* errors occur.

3. **Objective conflation**: Distance tracking, attitude stability, and control smoothness are
   competing objectives packed into one weighted sum. The optimizer can't trade them off —
   it finds whatever local minimum the weight ratios create.

4. **Bang-bang emergence**: No smoothness term exists. The fitness function only measures
   tracking error. Small NNs with tanh outputs naturally saturate to ±1 when that minimizes
   the error integral. This is unsuitable for physical servos.

### Current Fitness Components (autoc.h:13-56)

| Component | Formula | Constants |
|-----------|---------|-----------|
| Distance | `pow(abs(dist - 7.5) / 5.0, 1.5)` | DISTANCE_TARGET=7.5m, DISTANCE_NORM=5.0, DISTANCE_POWER=1.5 |
| Attitude | `pow(abs(dphi + dtheta) / 0.349, 1.5) * attitude_scale` | ATTITUDE_NORM=0.349 rad, ATTITUDE_POWER=1.5 |
| Attitude scale | `path_distance / max(path_turn_rad, 2π)` | Reduces attitude weight on straight segments |
| Intercept budget | Quadratic ramp 0.1→1.0 over estimated intercept time | Based on displacement, heading offset, speeds |
| Crash | `(1 - fraction_completed) * 1e6 + quality` | Soft lexicographic |

### Decision: Decompose into structured per-scenario × per-component scores

**Rationale**: The deterministic simulator can provide arbitrarily fine signal. The aggregation
strategy should be a *selection-time* decision, not baked into the fitness computation. Workers
compute component scores; autoc decides how to combine them for selection.

### Alternatives Considered

- **Keep scalar, tune weights**: Doesn't solve scenario masking or temporal credit.
- **Full per-timestep return**: ~1.2MB per individual × 5000 pop = 6GB/gen. Excessive for
  POSIX sockets. Better to reduce on worker side.
- **Rank-based fitness shaping**: Helps scale issues but doesn't decompose objectives.

---

## R2: Multi-Objective Selection Strategies

### Strategy 1: Minimax (Worst-Case) Selection

- **How**: For each individual, fitness = max(scenario_scores) (worst scenario).
- **Pro**: Forces robustness — can't hide behind good average.
- **Con**: May over-focus on one pathological scenario. Noisy if worst scenario changes.
- **Implementation**: Trivial — just take max of scenario score vector.

### Strategy 2: NSGA-II Pareto Ranking

- **How**: Maintain multiple objectives (tracking, smoothness, robustness). Non-dominated
  sorting + crowding distance for selection.
- **Pro**: Explores full Pareto front. No manual weight tuning.
- **Con**: 531-dim space with 3+ objectives — Pareto front may be too large for pop=5000.
  Selection pressure diluted.
- **Implementation**: ~200 LOC for non-dominated sort + crowding. Well-documented algorithm.
- **Reference**: Deb et al. (2002) "A Fast and Elitist Multiobjective Genetic Algorithm: NSGA-II"

### Strategy 3: Lexicase Selection

- **How**: Each selection event: shuffle the 49 scenarios, filter population by best-on-first,
  then best-on-second among survivors, etc. until 1 remains.
- **Pro**: Naturally maintains specialists *and* generalists without explicit diversity.
  Each scenario is a "test case." No aggregation at all.
- **Con**: O(pop × scenarios) per selection event. With 5000 × 49, this is ~245K comparisons
  per parent selection. May need epsilon-lexicase for continuous fitness.
- **Reference**: Helmuth et al. (2015) "Solving Uncompromising Problems with Lexicase Selection"
- **Note**: Particularly compelling for this domain — 49 scenarios ARE 49 test cases.

### Strategy 4: MAP-Elites / Quality-Diversity

- **How**: Define 2D behavioral feature space (e.g., mean tracking error × control smoothness).
  Maintain grid of best-in-cell. New individuals replace cell occupant if better.
- **Pro**: Explores diverse strategies. Finds stepping stones to good solutions.
- **Con**: Requires defining meaningful behavioral dimensions. Grid resolution tradeoffs.
- **Reference**: Mouret & Clune (2015) "Illuminating search spaces by mapping elites"

### Decision: Start with minimax (simplest), then research lexicase

**Rationale**: Minimax is a 10-line change once structured scores exist. It directly tests the
hypothesis that worst-case selection improves robustness. Lexicase is the most theoretically
compelling but requires more implementation work — do as a research spike on a branch.

---

## R3: Control Smoothness as Explicit Objective

### Problem

Current fitness has zero pressure toward smooth control. The NN produces bang-bang (±1 saturated)
outputs because that minimizes tracking error in the short term.

### Measure: Control Effort Rate (Jerk-like)

```
smoothness_score = (1/T) * Σ_t |u(t) - u(t-1)| for each control axis
```

Where u(t) is the control command (pitch, roll, throttle) at timestep t.

- **Bang-bang**: u alternates between +1 and -1 → smoothness ≈ 2.0 per axis
- **Smooth**: u changes gradually → smoothness ≈ 0.01-0.1 per axis
- **Threshold for "not bang-bang"**: smoothness < 0.3 per axis (TBD, needs empirical calibration)

### Alternative Measures

- **L2 jerk** (Σ|Δ²u|²): More sensitive to isolated large changes. Harder to threshold.
- **Frequency domain**: FFT of command signal, penalize high-frequency content. Elegant but
  complex to implement and interpret.
- **Actuator model**: Low-pass filter the commands, measure divergence from filtered version.
  Most realistic but adds simulator complexity.

### Decision: L1 rate-of-change (Σ|Δu|) per axis, computed by worker

**Rationale**: Simplest measure that directly penalizes bang-bang. L1 is more interpretable
than L2 for threshold-setting. Per-axis allows identifying which channel is problematic.
Computed by worker as part of structured response — zero cost to autoc.

---

## R4: Per-Segment Temporal Credit Assignment

### Problem

Cumulative error over 600 timesteps gives no credit for error *reduction*. A controller that
makes a brilliant correction during a hard turn gets the same credit as one that was already
on-path during a straight.

### Approach: Segment-Based Scoring

1. **Segment boundaries**: Defined by path geometry — straight segments, turns, Immelmans,
   etc. The path generator already knows these boundaries.
2. **Segment score**: `error_reduction = start_distance - end_distance` (positive = improved).
3. **Difficulty weight**: Based on turn rate and crosswind component during segment.
   `difficulty = max(1.0, turn_rate / base_turn_rate) * (1 + |crosswind| / wind_speed)`
4. **Weighted score**: `segment_fitness = error_reduction * difficulty_weight`

### Key Insight

A controller that *reduces* error during hard segments is more valuable than one with low
absolute error on easy segments. This is invisible in cumulative scoring.

### Implementation Location

Workers compute segment boundaries from path geometry (already available in simulation).
Workers return `vector<SegmentScore>` per scenario. Autoc can weight/aggregate at selection time.

### Decision: Workers compute per-segment scores, return as vector

**Rationale**: Path geometry is only available inside the simulation. Segment scoring must
happen on the worker side. Returning the vector lets autoc experiment with weighting strategies
without re-running simulations.

---

## R5: Existing Evolution Infrastructure (from codebase analysis)

### Current RPC Protocol (rpc/protocol.h)

- Binary cereal serialization with 4-byte length prefix
- `EvalData`: NN weights + scenario list + paths
- `EvalResults`: crash reasons + aircraft states + debug samples
- Already returns per-scenario crash status via `crashReasonList`
- `DebugSample` (protocol.h:155-213) already has per-timestep data — but only for elite re-eval

### Current NN Population (nn/population.cc)

- Self-adaptive sigma: `sigma' = sigma * exp(tau * N(0,1))` where tau = 1/sqrt(2n)
- Sigma clamped to [1e-6, 5.0] — no configurable floor
- BLX-α crossover, tournament selection, generational replacement
- Elitism: top N preserved unchanged

### Current Scenario System (autoc.cc:381-489)

- `rebuildGenerationScenarios()`: builds path × wind variation matrix
- Pre-computed variations in `gScenarioVariations` vector
- Variation ramping: `gVariationScale` from 0→1 over generations
- 49 scenarios = combinations of path variants × wind variants

### Current Fitness Aggregation (autoc.cc:614-709)

- `computeNNFitness()` returns single double
- Iterates over all paths in EvalResults
- No per-scenario breakdown returned to evolution loop

### What Needs to Change

1. **EvalResults** must include per-scenario score vectors (not just aggregate)
2. **computeNNFitness()** splits into worker-side component scoring + autoc-side aggregation
3. **Smoothness metric** added to worker-side computation
4. **Segment boundaries** computed from path geometry in worker
5. **Selection** in `nn_evolve_generation()` must support multi-objective

---

## R6: sep-CMA-ES (from 014 research, preserved)

### Algorithm: Ros & Hansen 2008

Separable CMA-ES replaces N×N covariance with diagonal vector: O(N) vs O(N³).

### Hyperparameters for N=531

| Parameter | Value | Formula |
|-----------|-------|---------|
| lambda (pop) | 50 | 4 + floor(3*ln(531)) = 22 min, 50 chosen |
| mu (parents) | 25 | lambda/2 |
| mu_eff | ~13.2 | (Σw_i)² / Σw_i² |
| c_sigma | 0.028 | (mu_eff + 2) / (N + mu_eff + 5) |
| d_sigma | 1.03 | 1 + 2*max(0, sqrt((mu_eff-1)/(N+1)) - 1) + c_sigma |
| c_c | 0.0075 | (4 + mu_eff/N) / (N + 4 + 2*mu_eff/N) |
| c_1 (sep) | 0.00125 | 2 / ((N+1.3)² + mu_eff) * (N+2)/3 |
| c_mu (sep) | 0.014 | min(1-c_1, 2*(mu_eff-2+1/mu_eff) / ((N+2)² + mu_eff)) * (N+2)/3 |
| sigma_0 | 0.3-0.5 | Initial step size |

**Key**: Learning rates scaled by (N+2)/3 = 177.7 for diagonal approximation.

### Decision: Defer to branch experiment after fitness signal improves

**Rationale**: CMA-ES with a broken fitness function will just converge faster to the wrong
answer. Fix the signal first, then experiment with better optimizers.

---

## R7: Literature References

| Topic | Reference | Relevance |
|-------|-----------|-----------|
| NSGA-II | Deb et al. (2002) | Multi-objective selection with Pareto ranking |
| Lexicase | Helmuth et al. (2015) | Per-test-case selection, no aggregation |
| MAP-Elites | Mouret & Clune (2015) | Quality-diversity, behavioral feature space |
| Novelty Search | Lehman & Stanley (2011) | Diversity pressure to escape local optima |
| sep-CMA-ES | Ros & Hansen (2008) | Diagonal CMA for high-dim continuous optimization |
| Fitness Shaping | Wierstra et al. (2014) | Rank-based transformation, scale-invariant |
| DTW | Sakoe & Chiba (1978) | Temporal alignment for trajectory comparison |
| Self-Adaptive ES | Schwefel (1981) | Per-individual sigma adaptation (current impl) |
| Epsilon-Lexicase | La Cava et al. (2016) | Lexicase for continuous domains |
| Control Smoothness | Flash & Hogan (1985) | Minimum-jerk trajectories in biological movement |

---

## R8: Archived Analysis from Prior Conversations

### nn13 Stall Analysis (from 014 development)

- **Run**: nn13, 22→16→8→3 topology, pop=5000, 49 scenarios
- **Plateau**: Fitness stalled at 3.99M around gen 500
- **Sigma collapse**: 0.20 → 0.024 over ~1000 generations
- **Diagnosis**: Self-adaptive sigma (Schwefel 1981) has no floor. Once sigma is small,
  mutations are too small to escape local optimum. Population converges to near-identical
  individuals.
- **Band-aid**: Sigma floor at 0.05 prevents collapse. But doesn't fix the underlying
  problem of poor fitness signal.

### Fitness Multiplier Tuning Attempts

- Adjusted DISTANCE_NORM, ATTITUDE_NORM, DISTANCE_POWER ratios
- **Result**: Different local optima, same plateau behavior
- **Conclusion**: The problem isn't the component weights — it's the aggregation strategy.
  Tuning weights on a fundamentally noisy signal just moves the noise around.

### NN Topology Experiments

- Tried 22→8→3 (smaller), 22→32→16→3 (deeper)
- **Result**: Smaller NNs converge faster but to worse optima. Deeper NNs harder to train.
- **Conclusion**: 22→16→8→3 is reasonable for the task. The bottleneck is fitness signal,
  not capacity.

### Bang-Bang Observation

- Evolved controllers consistently produce saturated ±1 outputs on pitch and roll
- Throttle somewhat smoother (less dynamic range needed)
- **Cause**: No smoothness pressure in fitness. Minimizing tracking error at each timestep
  independently favors maximum correction regardless of rate-of-change.
- **Physical consequence**: Cannot deploy on real aircraft. Servos can't follow bang-bang.

### Variation Robustness Challenge

- With 49 scenarios (7 wind × 7 entry variations), evolved controllers tend to specialize
  on a subset of scenarios that dominate the fitness sum
- Rare hard scenarios (strong crosswind + off-heading entry) contribute little to total
  because they're 1/49 of the sum
- **Need**: Selection that explicitly pressures worst-case performance

### Path Following Architecture Notes

- Controller is reactive (short-term planning only): sees GETDIST, GETDPHI, GETDTHETA
  for ~2-3 path points ahead
- Cannot anticipate upcoming maneuvers beyond sensor horizon
- Upper-level strategy (intercept director, resource management) is future work
- Current task: follow the presented path segment as smoothly and accurately as possible
  under environmental variation

### Deterministic Simulator Advantage

- Given identical weights + scenario, output is bit-identical across runs
- This means evolution can detect *arbitrarily small* fitness differences
- The signal isn't noisy — it's *masked* by aggregation
- Proper decomposition should unlock much finer-grained selection pressure

# Contract: Configuration Keys (015 additions)

**Feature**: 015-nn-training-improvements

## New Keys

All keys are flat (no sections), parsed by inih, with compiled defaults.

```ini
# Sigma floor — minimum mutation sigma (US1)
# 0 = disabled (backward compatible), >0 = clamp sigma after self-adaptive update
NNSigmaFloor = 0

# Fitness aggregation mode (US3)
# "sum" = legacy scalar sum (backward compatible)
# "minimax" = worst-scenario fitness drives selection
# "percentile" = Nth percentile scenario drives selection
FitnessAggregation = sum
FitnessPercentile = 0.95

# Per-segment temporal credit (US11) — Phase 6, not yet implemented
# 0 = disabled, 1 = autoc computes segment scores from path geometry
SegmentScoringEnabled = 0
```

## Unchanged Keys (reference)

These existing keys interact with new features but are not modified:

```ini
PopulationSize = 5000        # GA; would be ~50 for CMA-ES experiment
NNMutationSigma = 0.3       # Initial sigma; sigma floor should be < this
NumberOfGenerations = 500
EvalThreads = 4
```

## Validation Rules

1. `NNSigmaFloor >= 0` (negative is error)
2. If `NNSigmaFloor > NNMutationSigma`: warn at startup (floor > initial sigma)
3. `FitnessAggregation ∈ {"sum", "minimax", "percentile"}` (unknown = error)
4. `FitnessPercentile ∈ (0.0, 1.0]` (only validated when mode = percentile)
5. Unknown keys silently ignored (forward compat)

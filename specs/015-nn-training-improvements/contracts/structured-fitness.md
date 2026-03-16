# Contract: Structured Fitness Response

**Feature**: 015-nn-training-improvements
**Parties**: autoc (consumer) ↔ minisim/crrcsim (producer)

## Summary

Workers return per-scenario × per-component score vectors instead of contributing to a
single aggregate fitness. Autoc decides aggregation strategy at selection time.

## Wire Format Change

### Current (014 baseline)

```
EvalResults {
    crashReasonList[scenario_count]     // per-scenario crash status
    aircraftStateList[scenario_count][] // per-scenario per-timestep states
    debugSamples[scenario_count][]      // elite re-eval only
    physicsTrace[scenario_count][]      // elite re-eval only
}
```

Fitness computed by autoc after receiving results (`computeNNFitness()`).

### New (015)

```
EvalResults {
    // ... existing fields preserved for elite re-eval/logging ...
    scenario_scores[scenario_count] {   // NEW: always populated
        distance_rmse: double
        attitude_error: double
        smoothness: double[3]           // pitch, roll, throttle
        crashed: uint8
        crash_fraction: double          // 0.0-1.0
        segment_count: uint32           // 0 if segment scoring disabled
        segments[segment_count] {
            error_reduction: double
            difficulty_weight: double
            weighted_score: double
        }
    }
}
```

## Computation Rules (worker side)

### distance_rmse
```
sqrt( (1/T) * Σ_t (distance_to_target(t) - DISTANCE_TARGET)² )
```
Where DISTANCE_TARGET = 7.5m, T = timestep count.

### attitude_error
```
Σ_t |dphi(t)| + |dtheta(t)|
```
Unnormalized sum — autoc applies normalization at selection time if needed.

### smoothness[axis]
```
(1/T) * Σ_t |command[axis](t) - command[axis](t-1)|
```
Per-axis L1 rate-of-change. Range: 0.0 (constant) to 2.0 (full bang-bang every step).

### crash_fraction
```
last_valid_timestep / total_timesteps
```
0.0 = crashed immediately, 1.0 = completed full flight.

### segment scoring (when enabled)
- Boundaries from path geometry: segment changes when path curvature exceeds threshold
- error_reduction = distance_at_segment_start - distance_at_segment_end
- difficulty_weight = max(1.0, turn_rate/base_rate) × (1 + |crosswind|/wind_speed)
- weighted_score = error_reduction × difficulty_weight

## Invariants

1. `scenario_scores.size() == scenario_count` (always)
2. For non-crashed scenarios: `crash_fraction == 1.0`
3. `smoothness[axis] ∈ [0.0, 2.0]` (bounded by ±1 command range)
4. Segment scores may be empty even when enabled (e.g., very short flights)
5. `distance_rmse >= 0`, `attitude_error >= 0`

## Migration

No backward compatibility. Both sides update simultaneously per clarification Q1.
`computeNNFitness()` in autoc.cc is split: component scoring moves to worker,
aggregation remains in autoc as configurable strategy.

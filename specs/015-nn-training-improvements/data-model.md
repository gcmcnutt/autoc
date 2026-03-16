# Data Model: NN Training Improvements

**Feature**: 015-nn-training-improvements
**Date**: 2026-03-16

## Modified Entities

### EvalResults (rpc/protocol.h — MODIFIED)

Current: Returns per-scenario crash status + aircraft states + single aggregate fitness.
New: Add structured per-scenario × per-component score vectors.

```cpp
struct ScenarioScore {
    double distance_rmse;        // Path tracking accuracy (lower = better)
    double attitude_error;       // Orientation penalty sum
    double smoothness[3];        // |Δu(t)| per axis: pitch, roll, throttle
    bool crashed;                // Whether this scenario crashed
    double crash_fraction;       // 0.0-1.0, fraction completed before crash
    std::vector<SegmentScore> segments;  // Per-segment temporal credit (empty if disabled)

    template<class Archive>
    void serialize(Archive& ar) { ar(distance_rmse, attitude_error, smoothness,
                                     crashed, crash_fraction, segments); }
};

struct SegmentScore {
    double error_reduction;      // start_distance - end_distance (positive = improved)
    double difficulty_weight;    // turn_rate × crosswind factor
    double weighted_score;       // error_reduction * difficulty_weight

    template<class Archive>
    void serialize(Archive& ar) { ar(error_reduction, difficulty_weight, weighted_score); }
};
```

### EvalResults.scenario_scores (NEW FIELD)

```cpp
struct EvalResults {
    // ... existing fields ...
    std::vector<ScenarioScore> scenario_scores;  // NEW: one per scenario
};
```

### NNGenome (nn/evaluator.h — MODIFIED)

Add per-scenario fitness storage for multi-objective selection:

```cpp
struct NNGenome {
    // ... existing fields ...
    std::vector<double> scenario_fitnesses;  // NEW: per-scenario aggregate
    double smoothness_score;                  // NEW: mean smoothness across scenarios
    double worst_scenario_fitness;            // NEW: max(scenario_fitnesses)
};
```

## New Entities

### FitnessAggregator

Stateless function object that combines ScenarioScore vectors into selection criteria.

```cpp
enum class AggregationMode { SUM, MINIMAX, PERCENTILE, LEXICASE };

struct AggregationResult {
    double primary_fitness;           // Used for logging and basic comparisons
    std::vector<double> objectives;   // For multi-objective selection
};
```

### SegmentBoundary (computed by worker)

```cpp
struct SegmentBoundary {
    int start_timestep;
    int end_timestep;
    double turn_rate;           // Average turn rate during segment (rad/s)
    double crosswind_component; // Perpendicular wind component (m/s)
};
```

Derived from path geometry already available in minisim/crrcsim during simulation.

## State Flow (revised)

```
INIT → EVALUATE → [worker: compute component scores] → RECEIVE_SCORES → AGGREGATE → SELECT → REPRODUCE → EVALUATE
                                                                            ↑
                                                              aggregation strategy
                                                              (sum/minimax/lexicase)
```

Key change: aggregation is now a *selection-time* decision, not a *computation-time* decision.
Workers always return full decomposed scores. Autoc chooses how to use them.

## Backward Compatibility

None required (clarification Q1). Both autoc and crrcsim/minisim update simultaneously.
Old EvalResults format is replaced, not extended.

## Config Keys (new)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| NNSigmaFloor | double | 0 | Minimum mutation sigma (0 = disabled) |
| FitnessAggregation | string | "sum" | "sum", "minimax", "percentile" |
| FitnessPercentile | double | 0.95 | Percentile value (only with percentile mode) |
| SmoothnessWeight | double | 0 | Weight for smoothness in selection (0 = separate objective) |
| SegmentScoringEnabled | int | 0 | Enable per-segment temporal credit |

#pragma once

#include <vector>
#include "autoc/eval/fitness_computer.h"

// Forward declaration — full definition in rpc/protocol.h
struct EvalResults;

// Per-scenario fitness score (022 + 027).
// - `score`: accumulated tracking fitness (negated points, lower = better).
// - `smoothness_score`: mean-of-ticks control-rate magnitude
//   Σ_i (|Δoutpt| + |Δoutrl| + |Δoutth|) / (steps-1), lower = better
//   (spec 027 C2). Fed into lexicase as an independent test case.
struct ScenarioScore {
    double score;           // Tracking (negated accumulated points, lower = better)
    double smoothness_score; // Mean per-tick control-rate magnitude (spec 027, lower = better)
    bool crashed;           // Whether this scenario crashed
    int steps_completed;    // Number of simulation steps completed
    int steps_total;        // Total steps expected (path length)

    // Streak diagnostics
    int maxStreak;          // Longest consecutive streak in this scenario
    int totalStreakSteps;    // Total steps with stepPoints >= threshold
    double maxMultiplier;   // Highest multiplier reached

    ScenarioScore()
        : score(0.0), smoothness_score(0.0),
          crashed(false), steps_completed(0), steps_total(0),
          maxStreak(0), totalStreakSteps(0), maxMultiplier(1.0) {}
};

// Compute per-scenario scores from EvalResults using point-accumulation fitness.
// variationScale: 0.0 (no variations) to 1.0 (full variations), used for streak threshold ramp.
// Returns one ScenarioScore per scenario (path×wind combination).
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults);

// Aggregate: sum of per-scenario scores (already negated, lower is better).
double aggregateRawFitness(const std::vector<ScenarioScore>& scores);

#pragma once

#include <vector>
#include <cmath>
#include "autoc/eval/fitness_computer.h"

// Forward declaration — full definition in rpc/protocol.h
struct EvalResults;

// Per-scenario decomposed fitness scores.
// Objective priority: completion > attitude > distance > smoothness
// Crash handling: natural truncation — no separate crash penalty (Pareto lesson).
struct ScenarioScore {
    double completion_fraction;  // 0.0 (crashed immediately) to 1.0 (completed full flight)
    double attitude_error;       // Sum of |dphi| + |dtheta| per step, normalized by steps completed
    double distance_rmse;        // RMSE of distance-to-target over completed steps
    double smoothness[3];        // Mean |Δu(t)| per axis: pitch, roll, throttle (0=constant, 2=bang-bang)
    bool crashed;                // Whether this scenario crashed
    int steps_completed;         // Number of simulation steps completed
    int steps_total;             // Total steps expected (path length)

    // Legacy aggregate components (for backward-compat scalar reconstruction)
    double legacy_distance_sum;  // Raw distance penalty sum (with intercept scale, power)
    double legacy_attitude_sum;  // Raw attitude penalty sum (with intercept scale, power)
    double legacy_attitude_scale; // Path geometry attitude scaling factor
    double legacy_crash_penalty; // Crash penalty (1-completion)*1e6

    ScenarioScore()
        : completion_fraction(0.0), attitude_error(0.0), distance_rmse(0.0),
          crashed(false), steps_completed(0), steps_total(0),
          legacy_distance_sum(0.0), legacy_attitude_sum(0.0),
          legacy_attitude_scale(1.0), legacy_crash_penalty(0.0) {
        smoothness[0] = smoothness[1] = smoothness[2] = 0.0;
    }
};

// Compute per-scenario decomposed scores from EvalResults.
// Returns one ScenarioScore per scenario (path×wind combination).
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults);

// Reconstruct legacy scalar fitness from decomposed scores.
// Must produce identical output to the original computeNNFitness().
double aggregateScalarFitness(const std::vector<ScenarioScore>& scores);

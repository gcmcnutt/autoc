#pragma once

#include <vector>
#include "autoc/eval/fitness_computer.h"

// Forward declaration — full definition in rpc/protocol.h
struct EvalResults;

// Per-scenario fitness score (022: point-accumulation).
// Single score dimension: negated accumulated points (lower = better).
struct ScenarioScore {
    double score;           // Negated accumulated points (lower = better)
    bool crashed;           // Whether this scenario crashed
    int steps_completed;    // Number of simulation steps completed
    int steps_total;        // Total steps expected (path length)

    // Streak diagnostics
    int maxStreak;          // Longest consecutive streak in this scenario
    int totalStreakSteps;    // Total steps with stepPoints >= threshold
    double maxMultiplier;   // Highest multiplier reached

    ScenarioScore()
        : score(0.0), crashed(false), steps_completed(0), steps_total(0),
          maxStreak(0), totalStreakSteps(0), maxMultiplier(1.0) {}
};

// Compute per-scenario scores from EvalResults using point-accumulation fitness.
// variationScale: 0.0 (no variations) to 1.0 (full variations), used for streak threshold ramp.
// Returns one ScenarioScore per scenario (path×wind combination).
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults);

// Aggregate: sum of per-scenario scores (already negated, lower is better).
double aggregateRawFitness(const std::vector<ScenarioScore>& scores);

#pragma once

#include <vector>
#include "autoc/types.h"
#include "autoc/eval/fitness_computer.h"

// Forward declaration — full definition in rpc/protocol.h
struct EvalResults;

// Per-scenario fitness score (022 + 027 v4).
//
// All score fields use `gp_fitness` (= double) deliberately. Per-tick
// contributions are small (~-1) but accumulate across 245 scenarios ×
// hundreds of ticks; double precision avoids any sum-of-tiny-floats drift
// concerns, even when later summed into population-level aggregates.
//
// Three physics-grounded dimensions, all "lower = better":
// - `score`: accumulated tracking fitness (negated points). Streak multiplied.
// - `stability_score` (027 v4): time pitch/roll surfaces spend off-center.
//   Σ_t (|out_pt_t| - 1) + (|out_rl_t| - 1) over completed ticks. Each tick
//   contributes -2 (both surfaces centered, ideal) to 0 (both saturated, worst).
// - `energy_score` (027 v3): time throttle is on.
//   Σ_t (out_th_t - 1) / 2 over completed ticks. Each tick contributes
//   -1 (no throttle) to 0 (full throttle).
//
// All three additive per tick AND per scenario — no normalization,
// no streak amplification on the physics terms. The "all-attitude smooth
// tracking" objective: be on the path with surfaces near center and
// throttle at minimum. Lexicase uses each dimension as its own test case
// per scenario; no weighting, no fine-tuning, multi-objective.
struct ScenarioScore {
    gp_fitness score;            // Tracking (negated accumulated points, lower = better)
    gp_fitness stability_score;  // Per-tick (|out_pt|-1) + (|out_rl|-1) summed; lower = better
    gp_fitness energy_score;     // Per-tick (out_th - 1) / 2 summed; lower = better
    bool crashed;
    int steps_completed;
    int steps_total;

    // Streak diagnostics (tracking-only, for diagnostics not selection)
    int maxStreak;
    int totalStreakSteps;
    gp_fitness maxMultiplier;

    ScenarioScore()
        : score(0.0), stability_score(0.0), energy_score(0.0),
          crashed(false), steps_completed(0), steps_total(0),
          maxStreak(0), totalStreakSteps(0), maxMultiplier(1.0) {}
};

// Compute per-scenario scores from EvalResults using point-accumulation fitness.
// variationScale: 0.0 (no variations) to 1.0 (full variations), used for streak threshold ramp.
// Returns one ScenarioScore per scenario (path×wind combination).
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults);

// Aggregate: sum of per-scenario scores (already negated, lower is better).
double aggregateRawFitness(const std::vector<ScenarioScore>& scores);

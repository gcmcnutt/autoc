#include "autoc/eval/fitness_decomposition.h"
#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include "autoc/util/config.h"
#include "autoc/eval/aircraft_state.h"
#include <algorithm>
#include <cmath>

std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults) {
    std::vector<ScenarioScore> scores;

    const AutocConfig& cfg = ConfigManager::getConfig();
    int streakStepsToMax = static_cast<int>(cfg.fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0));
    if (streakStepsToMax < 1) streakStepsToMax = 1;

    for (size_t i = 0; i < evalResults.pathList.size(); i++) {
        ScenarioScore result;
        std::vector<Path>& path = evalResults.pathList.at(i);
        std::vector<AircraftState>& aircraftStates = evalResults.aircraftStateList.at(i);
        CrashReason& crashReason = evalResults.crashReasonList.at(i);

        if (path.empty() || aircraftStates.empty()) {
            scores.push_back(result);
            continue;
        }

        FitnessComputer fc(cfg.fitDistScaleBehind, cfg.fitDistScaleAhead, cfg.fitConeAngleDeg,
                           cfg.fitStreakThreshold, streakStepsToMax, cfg.fitStreakMultiplierMax);
        fc.resetStreak();

        gp_fitness accumulatedScore = 0.0;
        int simulation_steps = 0;

        // Spec 027 C2 v4: stability + energy accumulators.
        //
        // Stability per tick: (|out_pt|-1) + (|out_rl|-1) ∈ [-2, 0].
        //   Both surfaces centered → -2 (best, "trim flying")
        //   Either saturated → contributes 0 to that side → less negative
        //   Both saturated → 0 (worst, "fighting the controls")
        // Captures "time pitch/roll surfaces spend away from center" — the
        // PID-style cost of deflection. Throttle excluded (handled by energy).
        //
        // Energy per tick: (out_th - 1) / 2 ∈ [-1, 0].
        //   No throttle → -1 (best, gliding)
        //   Full throttle → 0 (worst, max propulsion)
        // FDM handles induced drag implicitly via airspeed loss → throttle
        // compensation, but only if NN actually compensates. Stability dim
        // (above) directly catches NN strategies that hide their drag in
        // pinned pitch/roll.
        //
        // Both additive across ticks AND scenarios (no division by ticks).
        // No streak multiplier — these are raw physical costs, not behavioral
        // rewards.
        gp_fitness stabilityAccum = 0.0;
        gp_fitness energyAccum = 0.0;

        // Previous tangent for last-waypoint fallback
        gp_vec3 prevTangent = gp_vec3::UnitX();

        int stepIndex = 0;
        while (++stepIndex < static_cast<int>(aircraftStates.size())) {
            auto& stepState = aircraftStates.at(stepIndex);
            int pathIndex = std::clamp(stepState.getThisPathIndex(), 0,
                                       static_cast<int>(path.size()) - 1);

            gp_vec3 aircraftPosition = stepState.getPosition();
            gp_vec3 rabbitPosition = path.at(pathIndex).start;

            // Path tangent: direction rabbit is traveling
            gp_vec3 tangent;
            if (pathIndex + 1 < static_cast<int>(path.size())) {
                tangent = (path.at(pathIndex + 1).start - path.at(pathIndex).start);
                double tn = tangent.norm();
                if (tn > 0.01) {
                    tangent = tangent / tn;
                    prevTangent = tangent;
                } else {
                    tangent = prevTangent;
                }
            } else {
                tangent = prevTangent;  // Last waypoint: reuse previous
            }

            // Decompose aircraft-rabbit offset into along-track and cross-track
            gp_vec3 offset = aircraftPosition - rabbitPosition;
            double along = offset.dot(tangent);
            gp_vec3 lateral = offset - along * tangent;
            double lateralDist = lateral.norm();

            // Score this step
            double stepPoints = fc.computeStepScore(along, lateralDist);
            double multipliedScore = fc.applyStreak(stepPoints);
            accumulatedScore += multipliedScore;

            simulation_steps++;

            // Accumulate stability + energy: requires NN data on this state.
            if (stepState.hasNNData()) {
                const float* out = stepState.getNNOutputs();
                const gp_fitness abs_pt = std::abs(static_cast<gp_fitness>(out[0]));
                const gp_fitness abs_rl = std::abs(static_cast<gp_fitness>(out[1]));
                stabilityAccum += (abs_pt - 1.0) + (abs_rl - 1.0);
                energyAccum    += (static_cast<gp_fitness>(out[2]) - 1.0) / 2.0;
            }
        }

        // Store result
        result.score = -accumulatedScore;       // Negate: lower = better
        result.stability_score = stabilityAccum; // Already negative; lower = better
        result.energy_score = energyAccum;       // Already negative; lower = better
        result.crashed = isCrash(crashReason);
        result.steps_completed = simulation_steps;
        result.steps_total = static_cast<int>(path.size()) - 1;
        result.maxStreak = fc.getMaxStreak();
        result.totalStreakSteps = fc.getStreakSteps();
        result.maxMultiplier = fc.getMaxMultiplier();

        scores.push_back(result);
    }

    return scores;
}

double aggregateRawFitness(const std::vector<ScenarioScore>& scores) {
    double total = 0.0;
    for (const auto& s : scores) {
        total += s.score;  // Already negated (lower = better)
    }
    return total;
}

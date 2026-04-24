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

        double accumulatedScore = 0.0;
        int simulation_steps = 0;

        // Spec 027 C2: smoothness accumulator — per-tick mean of
        // |Δoutpt| + |Δoutrl| + |Δoutth|. Lower = smoother.
        double smoothnessAccum = 0.0;
        int smoothnessPairs = 0;
        float prev_out[3] = {0.0f, 0.0f, 0.0f};
        bool havePrevOut = false;

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

            // Accumulate smoothness: requires NN data captured on this state.
            if (stepState.hasNNData()) {
                const float* out = stepState.getNNOutputs();
                if (havePrevOut) {
                    smoothnessAccum += std::abs(out[0] - prev_out[0])
                                     + std::abs(out[1] - prev_out[1])
                                     + std::abs(out[2] - prev_out[2]);
                    smoothnessPairs++;
                }
                prev_out[0] = out[0];
                prev_out[1] = out[1];
                prev_out[2] = out[2];
                havePrevOut = true;
            }
        }

        // Store result
        result.score = -accumulatedScore;  // Negate: lower = better
        result.smoothness_score = (smoothnessPairs > 0)
            ? (smoothnessAccum / smoothnessPairs)
            : 0.0;
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

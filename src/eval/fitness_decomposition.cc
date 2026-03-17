#include "autoc/eval/fitness_decomposition.h"
#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include <algorithm>
#include <cmath>

// Faithfully decomposed from computeNNFitness() in autoc.cc.
// Computes the same values but retains per-scenario component scores.
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults) {
    std::vector<ScenarioScore> scores;
    FitnessComputer fc;

    for (size_t i = 0; i < evalResults.pathList.size(); i++) {
        ScenarioScore score;
        std::vector<Path>& path = evalResults.pathList.at(i);
        std::vector<AircraftState>& aircraftStates = evalResults.aircraftStateList.at(i);
        CrashReason& crashReason = evalResults.crashReasonList.at(i);

        if (path.empty() || aircraftStates.empty()) {
            scores.push_back(score);
            continue;
        }

        // Intercept-budget scaling (same as computeNNFitness)
        double interceptBudget = 0.0;
        {
            gp_vec3 initialPos = aircraftStates.at(0).getPosition();
            gp_vec3 pathStart = path.at(0).start;
            gp_vec3 offset3d = initialPos - pathStart;
            double displacement = sqrt(offset3d[0] * offset3d[0] + offset3d[1] * offset3d[1]);
            double headingOffset = 0.0;
            if (i < evalResults.scenarioList.size()) {
                headingOffset = evalResults.scenarioList.at(i).entryHeadingOffset;
            }
            interceptBudget = FitnessComputer::computeInterceptBudget(
                displacement, headingOffset, SIM_INITIAL_VELOCITY, 16.0);
        }

        // Per-step accumulation
        double distance_sum_legacy = 0.0;
        double attitude_sum_legacy = 0.0;
        double distance_sq_sum = 0.0;  // For RMSE
        double attitude_delta_sum = 0.0;  // For normalized attitude error
        double prev_roll = 0.0, prev_pitch = 0.0;
        bool first_attitude_sample = true;
        int stepIndex = 0;
        int simulation_steps = 0;

        // Previous commands for smoothness (pitch, roll, throttle)
        double prev_cmd[3] = {0.0, 0.0, 0.0};
        double smoothness_sum[3] = {0.0, 0.0, 0.0};
        bool first_cmd = true;

        while (++stepIndex < static_cast<int>(aircraftStates.size())) {
            auto& stepState = aircraftStates.at(stepIndex);
            int pathIndex = std::clamp(stepState.getThisPathIndex(), 0,
                                       static_cast<int>(path.size()) - 1);

            gp_vec3 aircraftPosition = stepState.getPosition();
            gp_quat craftOrientation = stepState.getOrientation();

            // Attitude delta (same as computeNNFitness)
            double qw = craftOrientation.w(), qx = craftOrientation.x();
            double qy = craftOrientation.y(), qz = craftOrientation.z();
            double roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
            double pitch = asin(sinp);

            double attitude_delta = 0.0;
            if (first_attitude_sample) {
                prev_roll = roll;
                prev_pitch = pitch;
                first_attitude_sample = false;
            } else {
                attitude_delta = fabs(roll - prev_roll) + fabs(pitch - prev_pitch);
                prev_roll = roll;
                prev_pitch = pitch;
            }

            // Distance to rabbit
            gp_vec3 craftOffset = aircraftPosition - path.at(pathIndex).start;
            double distance = craftOffset.norm();

            // Legacy accumulation (identical to computeNNFitness)
            double stepTimeSec = stepState.getSimTimeMsec() / 1000.0;
            double interceptScale = FitnessComputer::computeInterceptScale(stepTimeSec, interceptBudget);
            double distDelta = fabs(distance - DISTANCE_TARGET);
            distance_sum_legacy += pow(interceptScale * distDelta / DISTANCE_NORM, DISTANCE_POWER);
            attitude_sum_legacy += pow(interceptScale * attitude_delta / ATTITUDE_NORM, ATTITUDE_POWER);

            // Decomposed metrics
            double dist_error = distance - DISTANCE_TARGET;
            distance_sq_sum += dist_error * dist_error;
            attitude_delta_sum += attitude_delta;

            // Smoothness: |Δu(t)| per axis from commands sent to aircraft
            double cmd[3] = {
                static_cast<double>(stepState.getPitchCommand()),
                static_cast<double>(stepState.getRollCommand()),
                static_cast<double>(stepState.getThrottleCommand())
            };
            if (first_cmd) {
                first_cmd = false;
            } else {
                for (int a = 0; a < 3; a++) {
                    smoothness_sum[a] += fabs(cmd[a] - prev_cmd[a]);
                }
            }
            for (int a = 0; a < 3; a++) prev_cmd[a] = cmd[a];

            simulation_steps++;
        }

        // Path geometry for attitude scale
        double path_distance = path.back().distanceFromStart;
        double path_turn_rad = path.back().radiansFromStart;
        double attitude_scale = fc.computeAttitudeScale(path_distance, path_turn_rad);

        // Completion fraction
        score.steps_completed = simulation_steps;
        score.steps_total = static_cast<int>(path.size()) - 1;  // path has one extra start point
        score.crashed = isCrash(crashReason);

        if (score.crashed) {
            double total_path_distance = path.back().distanceFromStart;
            double fraction_completed =
                static_cast<double>(path.at(aircraftStates.back().getThisPathIndex()).distanceFromStart) / total_path_distance;
            fraction_completed = std::max(fraction_completed, 0.001);
            score.completion_fraction = fraction_completed;
            score.legacy_crash_penalty = (1.0 - fraction_completed) * CRASH_COMPLETION_WEIGHT;
        } else {
            score.completion_fraction = 1.0;
            score.legacy_crash_penalty = 0.0;
        }

        // Decomposed scores
        if (simulation_steps > 0) {
            score.distance_rmse = sqrt(distance_sq_sum / simulation_steps);
            score.attitude_error = attitude_delta_sum / simulation_steps;
            for (int a = 0; a < 3; a++) {
                score.smoothness[a] = smoothness_sum[a] / std::max(1, simulation_steps - 1);
            }
        }

        // Legacy components for scalar reconstruction
        score.legacy_distance_sum = distance_sum_legacy;
        score.legacy_attitude_sum = attitude_sum_legacy;
        score.legacy_attitude_scale = attitude_scale;

        scores.push_back(score);
    }

    return scores;
}

// Reconstruct the exact legacy scalar fitness from decomposed scores.
double aggregateScalarFitness(const std::vector<ScenarioScore>& scores) {
    double totalFitness = 0.0;
    for (const auto& s : scores) {
        double localFitness = s.legacy_distance_sum + s.legacy_attitude_sum * s.legacy_attitude_scale;
        if (s.crashed) {
            localFitness = s.legacy_crash_penalty + localFitness;
        }
        totalFitness += localFitness;
    }
    return totalFitness;
}

// Raw aggregate: sum of distance RMSE + attitude error + crash penalty.
// No power functions, no norm scaling. Matches what lexicase filters on.
double aggregateRawFitness(const std::vector<ScenarioScore>& scores) {
    double total = 0.0;
    for (const auto& s : scores) {
        if (s.crashed) {
            total += (1.0 - s.completion_fraction) * CRASH_COMPLETION_WEIGHT;
        }
        total += s.distance_rmse + s.attitude_error;
    }
    return total;
}

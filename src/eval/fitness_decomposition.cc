#include "autoc/eval/fitness_decomposition.h"
#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include "autoc/util/config.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/camera_projection.h"
#include <algorithm>
#include <cmath>

namespace {

// targetInChaseCameraFOV removed 2026-05-11 — its only caller was the
// vis-fov/vis-cone classifier branch in computeScenarioScores, which was
// unreachable dead code (see TrackerDiag header comment).

double percentileSorted(const std::vector<double>& sorted, double pct) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted.front();
    double idx = pct * (sorted.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));
    if (hi >= sorted.size()) hi = sorted.size() - 1;
    double frac = idx - lo;
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

}  // namespace

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

        // 030 M7d.a — tracker scenarios populate targetTrajectoryList in
        // minisim; pathgen scenarios leave it empty. Branch on data presence
        // to avoid a separate mode field. Pathgen path below is unchanged
        // (bitwise-preserved against the M1 regression gate).
        const bool is_tracker =
            (i < evalResults.targetTrajectoryList.size()
             && !evalResults.targetTrajectoryList.at(i).empty());

        // 030 M11.wrap T088 + 327-330 — tracker-mode diagnostics state. All
        // zero-initialized; bookkeeping happens only inside the is_tracker branch.
        int prevStreakCount = 0;
        std::vector<double> rangeSamples;
        rangeSamples.reserve(aircraftStates.size());
        double prevRange = 0.0;
        double prevDRange = 0.0;
        bool haveDRange = false;
        int closure_flips = 0;
        double max_closure_rate = 0.0;
        int vis_count = 0;
        int in_ramp_count = 0;
        int lost_sight_run = 0;
        int max_lost_sight_run = 0;
        double spiral_ratio_accum = 0.0;
        int spiral_ratio_count = 0;
        float prev_out_pt = 0.0f, prev_out_rl = 0.0f;
        bool have_prev_out = false;
        int thrash_pt_count = 0, thrash_rl_count = 0;
        const double THRASH_THRESHOLD = 0.5;

        int stepIndex = 0;
        while (++stepIndex < static_cast<int>(aircraftStates.size())) {
            auto& stepState = aircraftStates.at(stepIndex);

            gp_vec3 aircraftPosition = stepState.getPosition();
            gp_vec3 rabbitPosition;
            gp_vec3 tangent;
            gp_vec3 targetPosition = gp_vec3::Zero();  // tracker-only, for diag

            if (is_tracker) {
                // Tracker: rabbit = trail-rabbit position trailing target's
                // velocity vector (M7b); tangent = target velocity unit.
                // targetTrajectoryList[i] is parallel-indexed with
                // aircraftStateList[i] (both pushed in lockstep by minisim).
                const std::vector<CopiedTargetSample>& targets =
                    evalResults.targetTrajectoryList.at(i);
                int targetIndex = std::clamp(stepIndex, 0,
                                             static_cast<int>(targets.size()) - 1);
                const CopiedTargetSample& target = targets.at(targetIndex);
                rabbitPosition = target.trail_rabbit_position;
                targetPosition = target.position;
                gp_vec3 vel = target.velocity;
                double vn = vel.norm();
                if (vn > 0.01) {
                    tangent = vel / vn;
                    prevTangent = tangent;
                } else {
                    tangent = prevTangent;  // degenerate target velocity
                }
            } else {
                int pathIndex = std::clamp(stepState.getThisPathIndex(), 0,
                                           static_cast<int>(path.size()) - 1);
                rabbitPosition = path.at(pathIndex).start;

                // Path tangent: direction rabbit is traveling
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
            }

            // Decompose aircraft-rabbit offset into along-track and cross-track
            gp_vec3 offset = aircraftPosition - rabbitPosition;
            double along = offset.dot(tangent);
            gp_vec3 lateral = offset - along * tangent;
            double lateralDist = lateral.norm();

            // Score this step. decomposeStepScore preserves computeStepScore's
            // numerics bitwise (computeStepScore now delegates to it), so
            // pathgen-mode regression-gate is unaffected.
            auto terms = fc.decomposeStepScore(along, lateralDist);
            double stepPoints = terms.score;
            // 033 §2.B — per-tick smoothness factor is computed by the
            // stepper (worker-side per-tick) and stored on AircraftState.
            // Read it here for fitness application; the per-tick value is
            // a pure function of the NN-output Δs so a re-derivation at
            // analysis time would produce the same number.
            const double smoothness_factor =
                static_cast<double>(stepState.getSmoothnessFactor());
            double multipliedScore = fc.applyStreak(stepPoints, smoothness_factor);
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

            // 030 M11.wrap T088 + 327-330 — tracker-mode diagnostic
            // accumulation. Observation-only; no effect on fitness or selection.
            if (is_tracker) {
                // Visibility from chase camera observations (per tick).
                bool left_vis = false, right_vis = false;
                if (i < evalResults.cameraViewList.size()
                    && stepIndex < static_cast<int>(evalResults.cameraViewList.at(i).size())) {
                    const CameraViewSample& cv = evalResults.cameraViewList.at(i).at(stepIndex);
                    left_vis  = (cv.beacon_left.cep  < autoc::eval::kCepSentinelThreshold);
                    right_vis = (cv.beacon_right.cep < autoc::eval::kCepSentinelThreshold);
                }
                const bool any_visible = left_vis || right_vis;
                if (any_visible) vis_count++;
                if (stepPoints >= fc.getStreakThreshold()) in_ramp_count++;

                // Lost-sight run (327-330 forward-looking).
                if (!any_visible) {
                    lost_sight_run++;
                    if (lost_sight_run > max_lost_sight_run) max_lost_sight_run = lost_sight_run;
                } else {
                    lost_sight_run = 0;
                }

                // Range + closure (chase→target).
                const double range = (aircraftPosition - targetPosition).norm();
                rangeSamples.push_back(range);
                if (stepIndex > 1) {  // need a previous range
                    double dRange = range - prevRange;
                    const double dt = SIM_TIME_STEP_MSEC / 1000.0;
                    double closureRate = -dRange / dt;  // positive = closing
                    if (std::abs(closureRate) > std::abs(max_closure_rate)) max_closure_rate = closureRate;
                    if (haveDRange) {
                        // Sign-flip detection (ignore exact zeros).
                        if ((dRange > 0 && prevDRange < 0) || (dRange < 0 && prevDRange > 0)) {
                            closure_flips++;
                        }
                    }
                    prevDRange = dRange;
                    haveDRange = true;
                }
                prevRange = range;

                // Spiral ratio: |gyro| / max(|vel|, 1). Mean across ticks.
                const gp_vec3 gyro = stepState.getGyroRates();
                const gp_vec3 vel = stepState.getVelocity();
                const double gnorm = static_cast<double>(gyro.norm());
                const double vnorm = std::max(static_cast<double>(vel.norm()), 1.0);
                spiral_ratio_accum += gnorm / vnorm;
                spiral_ratio_count++;

                // Thrashing: per-axis transition count where |dout| > THRASH_THRESHOLD.
                if (stepState.hasNNData()) {
                    const float* out = stepState.getNNOutputs();
                    if (have_prev_out) {
                        if (std::abs(out[0] - prev_out_pt) > THRASH_THRESHOLD) thrash_pt_count++;
                        if (std::abs(out[1] - prev_out_rl) > THRASH_THRESHOLD) thrash_rl_count++;
                    }
                    prev_out_pt = out[0];
                    prev_out_rl = out[1];
                    have_prev_out = true;
                }

                // T088 streak-loss reason classification — GEOMETRIC ONLY.
                // Detect transition: streak was > 0 last tick, now 0.
                //
                // 2026-05-11 polish: removed the visibility-classification branch
                // (loss_vis_fov / loss_vis_cone). It was unreachable dead code:
                // the streak counter is reset only when `stepPoints <
                // streakThreshold` (see FitnessComputer::applyStreak), so at the
                // streak-break tick `stepPoints < threshold` BY DEFINITION,
                // and the visibility branch's guard `stepPoints >= threshold`
                // was always false. Visibility events are still captured via
                // `avgVis` (frac ticks target-visible) and `maxLost` (longest
                // consecutive lost-of-sight run) in the same #GenDiag line.
                const int curStreakCount = fc.getStreakCount();
                if (prevStreakCount > 0 && curStreakCount == 0) {
                    // Geometry below ramp — classify by dominant term.
                    if (terms.ahead) {
                        result.tracker_diag.loss_geom_overshoot++;
                    } else if (terms.distTermSq >= terms.angleTermSq) {
                        result.tracker_diag.loss_geom_too_far++;
                    } else {
                        result.tracker_diag.loss_geom_angle++;
                    }
                }
                prevStreakCount = curStreakCount;
            }
        }

        // Store result
        result.score = -accumulatedScore;       // Negate: lower = better
        result.stability_score = stabilityAccum; // Already negative; lower = better
        result.energy_score = energyAccum;       // Already negative; lower = better
        result.crashed = isCrash(crashReason);
        result.crashReason = crashReason;
        result.steps_completed = simulation_steps;
        result.steps_total = static_cast<int>(path.size()) - 1;
        result.maxStreak = fc.getMaxStreak();
        result.totalStreakSteps = fc.getStreakSteps();
        result.maxMultiplier = fc.getMaxMultiplier();

        // 030 M11.wrap — finalize tracker-mode diagnostics. Aggregates from
        // per-tick accumulators; pathgen scenarios skip this and leave the
        // diag struct zero-initialized.
        if (is_tracker) {
            const int N = simulation_steps;
            // Hull strike from CrashReason (mutually exclusive scenario terminator;
            // streak-loss classifier records other reasons over the tick stream).
            if (crashReason == CrashReason::HullStrike) {
                result.tracker_diag.loss_hull = 1;
            }
            if (N > 0) {
                result.tracker_diag.vis_frac = static_cast<float>(vis_count) / N;
                result.tracker_diag.in_fit_ramp_frac = static_cast<float>(in_ramp_count) / N;
                if (spiral_ratio_count > 0) {
                    result.tracker_diag.spiral_ratio =
                        static_cast<float>(spiral_ratio_accum / spiral_ratio_count);
                }
                const double duration_sec = N * (SIM_TIME_STEP_MSEC / 1000.0);
                if (duration_sec > 0.0) {
                    result.tracker_diag.thrash_rate_pt = static_cast<float>(thrash_pt_count / duration_sec);
                    result.tracker_diag.thrash_rate_rl = static_cast<float>(thrash_rl_count / duration_sec);
                }
            }
            if (!rangeSamples.empty()) {
                auto sorted_ranges = rangeSamples;
                std::sort(sorted_ranges.begin(), sorted_ranges.end());
                result.tracker_diag.range_min = static_cast<float>(sorted_ranges.front());
                result.tracker_diag.range_med = static_cast<float>(percentileSorted(sorted_ranges, 0.5));
                result.tracker_diag.range_p95 = static_cast<float>(percentileSorted(sorted_ranges, 0.95));
            }
            result.tracker_diag.closure_flips = closure_flips;
            result.tracker_diag.max_closure_rate = static_cast<float>(max_closure_rate);
            result.tracker_diag.max_lost_sight_run = max_lost_sight_run;
        }

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

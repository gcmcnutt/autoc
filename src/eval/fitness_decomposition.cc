#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/energy_state.h"   // 041 P2-5 — Ps, one definition shared with the input and the reader
#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include "autoc/util/config.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/derived_features.h"   // 038 US3 — compute_pair_span
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

// 038 US3 — mean aux span/closure-prediction error over CEP-visible
// (t, t+horizon) pairs. Predicted spans are the tracker aux outputs
// (nnOutputs[3..5] at the kSpanPredictHorizonsMsec lookaheads); nnOutputs[6] is
// the predicted closure rate (span-rate, NDC/s). Realized span is the CEP-gated
// beacon-pair span from the camera views (same gating rule as the spn0 input).
// Returns 0 when there are no valid pairs (all-blind or pathgen). Cadence-
// invariant: tick offsets derive from the fixed ms horizons.
static_assert(kNumSpanPredictHorizons == 3, "update the horizon integral-check below if this changes");
static_assert(kSpanPredictHorizonsMsec[0] % SIM_TIME_STEP_MSEC == 0 &&
              kSpanPredictHorizonsMsec[1] % SIM_TIME_STEP_MSEC == 0 &&
              kSpanPredictHorizonsMsec[2] % SIM_TIME_STEP_MSEC == 0,
              "kSpanPredictHorizonsMsec must be integral multiples of SIM_TIME_STEP_MSEC (cadence-invariant)");

}  // namespace

// NOT in the anonymous namespace: declared in the header so the tick-pairing
// invariant can be asserted directly (see the header comment). It stays a plain
// free function otherwise.
gp_fitness computeSpanPredictionError(const std::vector<EvalTick>& ticks) {
    // 041 T022 — THE OFFSET IS GONE. This function used to take two parallel
    // vectors and reconcile them with `const int c = t - 1;  // <-- the offset`,
    // because the state list carried a pre-loop initial state and the camera
    // list did not. Pairing them 1:1 (038 US3 .. 2026-08-02) scored every
    // prediction ONE TICK LATE on a live lexicase axis, and nothing asserted the
    // pairing, which is why it survived two features.
    //
    // Now there is one series. `ticks[t]` carries the state, its NN outputs, and
    // the camera view FROM THE SAME TICK, so `span[t]` means "the span at tick
    // t" by construction and the horizon arithmetic reads directly. The pre-loop
    // initial state lives in ScenarioTicks::initialState and is not addressable
    // here at all.
    const int N = static_cast<int>(ticks.size());
    if (N < 2) return 0.0;
    // realized span + CEP visibility per TICK — one index, no reconciliation.
    std::vector<float> span(N, 0.0f);
    std::vector<char> vis(N, 0);
    for (int t = 0; t < N; ++t) {
        if (!ticks[t].cameraView.has_value()) continue;   // pathgen, or no view
        const auto& bl = ticks[t].cameraView->beacon_left;
        const auto& br = ticks[t].cameraView->beacon_right;
        const bool gated = (bl.cep >= autoc::eval::kCepSentinelThreshold) ||
                           (br.cep >= autoc::eval::kCepSentinelThreshold);
        if (!gated) {
            span[t] = static_cast<float>(autoc::eval::compute_pair_span(
                bl.bearing_x_rad, bl.bearing_y_rad,
                br.bearing_x_rad, br.bearing_y_rad));
            vis[t] = 1;
        }
    }
    const double dt = SIM_TIME_STEP_MSEC / 1000.0;
    double err = 0.0;
    long pairs = 0;
    for (int t = 0; t < N; ++t) {
        if (!vis[t] || !ticks[t].state.hasNNData()) continue;
        const float* out = ticks[t].state.getNNOutputs();
        for (int h = 0; h < kNumSpanPredictHorizons; ++h) {
            const int ta = t + (kSpanPredictHorizonsMsec[h] / SIM_TIME_STEP_MSEC);
            if (ta >= N || !vis[ta]) continue;
            const float predicted = out[TRACKER_NN_CONTROL_OUTPUT_COUNT + h];
            err += std::abs(static_cast<double>(predicted) - static_cast<double>(span[ta]));
            ++pairs;
        }
        // closure-rate aux output vs realized (span[t+1] - span[t]) / dt.
        if (t + 1 < N && vis[t + 1]) {
            const float predRate = out[TRACKER_NN_CONTROL_OUTPUT_COUNT + kNumSpanPredictHorizons];
            const double realRate = (static_cast<double>(span[t + 1]) - static_cast<double>(span[t])) / dt;
            err += std::abs(static_cast<double>(predRate) - realRate);
            ++pairs;
        }
    }
    return pairs > 0 ? static_cast<gp_fitness>(err / static_cast<double>(pairs)) : gp_fitness(0.0);
}

namespace {
}  // namespace

std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults) {
    std::vector<ScenarioScore> scores;

    const AutocConfig& cfg = ConfigManager::getConfig();
    int streakStepsToMax = static_cast<int>(cfg.fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0));
    if (streakStepsToMax < 1) streakStepsToMax = 1;

    for (size_t i = 0; i < evalResults.pathList.size(); i++) {
        ScenarioScore result;
        std::vector<Path>& path = evalResults.pathList.at(i);
        std::vector<EvalTick>& ticks = evalResults.tickList.at(i).ticks;
        CrashReason& crashReason = evalResults.crashReasonList.at(i);

        if (path.empty() || ticks.empty()) {
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
        // 041 P2-5 — per-scenario Ps state for the energy axis (below).
        gp_fitness prevEs = 0.0;
        double prevEsTimeMsec = 0.0;
        bool haveEs = false;

        // Previous tangent for last-waypoint fallback
        gp_vec3 prevTangent = gp_vec3::UnitX();

        // 030 M7d.a — tracker scenarios populate targetTrajectoryList in
        // the worker; pathgen scenarios leave it empty. Branch on data presence
        // to avoid a separate mode field. Pathgen path below is unchanged
        // (bitwise-preserved against the M1 regression gate).
        // 041 T022 — mode is now a property of the tick record itself: a
        // tracker tick carries a target sample, a pathgen tick does not (absent,
        // not zero-filled). Same "branch on data presence" as before, but the
        // presence is per-record rather than a separate list's length.
        const bool is_tracker = ticks.front().targetSample.has_value();

        // 030 M11.wrap T088 + 327-330 — tracker-mode diagnostics state. All
        // zero-initialized; bookkeeping happens only inside the is_tracker branch.
        int prevStreakCount = 0;
        std::vector<double> rangeSamples;
        rangeSamples.reserve(ticks.size());
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

        // 041 T022 — `tickIndex` is 0-BASED over the stepped ticks. The old
        // `stepIndex` was 1-based into a state array whose slot 0 was the
        // pre-loop initial state; every "- 1" below existed to undo that. The
        // initial state now has its own name (ScenarioTicks::initialState) and
        // is not in this loop at all.
        for (int tickIndex = 0; tickIndex < static_cast<int>(ticks.size()); ++tickIndex) {
            EvalTick& tickRecord = ticks.at(tickIndex);
            auto& stepState = tickRecord.state;

            gp_vec3 aircraftPosition = stepState.getPosition();
            gp_vec3 rabbitPosition;
            gp_vec3 tangent;
            gp_vec3 targetPosition = gp_vec3::Zero();  // tracker-only, for diag

            if (is_tracker) {
                // Tracker: rabbit = trail-rabbit position trailing target's
                // velocity vector (M7b); tangent = target velocity unit.
                //
                // 041 T022 — the clamp that used to live here is DELETED, not
                // relocated:
                //
                //     int targetIndex = std::clamp(stepIndex - 1, 0, size - 1);
                //
                // It existed because the state list carried a pre-loop initial
                // state and the target list did not, so tick k's target sat at
                // targets[k-1]. Pairing 1:1 scored the chase at tick k against
                // where the target was at tick k+1 — ~0.85 m at cruise against a
                // 3.048 m intended trail, a 28% error in the DEFINITION of the
                // task, live since 030 and invisible in the data because
                // CopiedTargetSample carries no timestamp.
                //
                // The target now travels IN the tick record, so there is no
                // index to get wrong and nothing to clamp. If a future reader
                // finds themselves needing an offset here, the grouping is
                // wrong — fix the grouping, do not re-add the offset.
                const CopiedTargetSample& target = *tickRecord.targetSample;
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

            // 041 T035 (FR-018a) — THE SCORE IS READ, NOT RE-DERIVED.
            //
            // `tickRecord.stepScore` was computed in the eval tick path, from
            // this same AircraftState before it was serialized, so the number
            // the policy is rewarded by and the number recorded for it to
            // observe are one value rather than two implementations agreeing.
            // The local geometry above is still computed because the
            // ATTRIBUTION buckets below need the decomposed terms (distTermSq
            // vs angleTermSq) — those are observation-only, carry no selection
            // pressure, and are not what gets scored.
            auto terms = fc.decomposeStepScore(along, lateralDist);
            double stepPoints = static_cast<double>(tickRecord.stepScore);
            double multipliedScore = fc.applyStreak(stepPoints);
            // 037 T018 — ×kCadenceTickScale: per-tick samples of the
            // instantaneous geometry accumulate in 100 ms-tick-equivalent
            // units, so the total is cadence-invariant (and the fixed
            // SIM_CRASH_PENALTY keeps its relative weight at 20 Hz).
            // ×1.0 bitwise no-op at 10 Hz. Streak threshold/ramp operate
            // on UNSCALED stepPoints above — geometry, not cadence.
            accumulatedScore += multipliedScore * kCadenceTickScale;

            simulation_steps++;

            // Accumulate stability + energy: requires NN data on this state.
            if (stepState.hasNNData()) {
                const float* out = stepState.getNNOutputs();
                const gp_fitness abs_pt = std::abs(static_cast<gp_fitness>(out[0]));
                const gp_fitness abs_rl = std::abs(static_cast<gp_fitness>(out[1]));
                // 037 T018 — ×kCadenceTickScale (see accumulatedScore note):
                // stability/energy totals stay cadence-invariant on the
                // historical 10 Hz scale; ×1.0 bitwise no-op at 10 Hz.
                stabilityAccum += ((abs_pt - 1.0) + (abs_rl - 1.0)) * kCadenceTickScale;
            }

            // ================================================================
            // 041 P2-5 — THE ENERGY AXIS, rebuilt on Ps.
            // ----------------------------------------------------------------
            // Was: a convex integral of the THROTTLE COMMAND (035 FR-001b).
            // That penalised an ABSOLUTE quantity, and full power is genuinely
            // correct when far behind, in a sustained spiral, and under
            // pitch-induced drag — so uniform pressure could only quiet
            // everything. 035 muted the whole regiment rather than trimming
            // waste, and 033 showed the same shape for smoothness.
            //
            // Now: metres of specific energy DESTROYED, Σ max(0, −Ps)·dt.
            // Three properties, each load-bearing:
            //
            //   1. ⭐ IT DOES NOT PENALISE CLIMBING. Gaining energy has Ps > 0
            //      and contributes exactly zero. A policy that climbs to follow
            //      a climbing target pays nothing for the climb — which is the
            //      muting failure, stated as a property. energy_metric_tests
            //      asserts it directly.
            //   2. ⭐ IT DOES NOT PENALISE TRADING. Es = h + v²/2g, so a dive
            //      that converts height into speed is Es-NEUTRAL and costs
            //      nothing. Only drag losses and deliberately destroyed energy
            //      appear. That is "waste", as opposed to "activity".
            //   3. It is NOT telescoping. Σ Ps·dt would collapse to
            //      (Es_end − Es_start) — gameable by simply finishing high, and
            //      blind to everything in between. Clipping at zero breaks the
            //      cancellation, so a climb-dive cycle is charged for the dive.
            //
            // ⛔ NEVER a scalar penalty term, and never without SPECIFIC_ENERGY
            // in the input vector: an axis for an unobservable is exactly what
            // muted 035 (TA03 — the policy had the v² half and never the h
            // half). It is a lexicase axis, and corr(Ps, closure rate) = −0.048
            // measured, so it is orthogonal to tracking — the ideal case for
            // lexicase and the worst case for scalar aggregation.
            //
            // Units: metres of Es. Multiplied by the ACTUAL elapsed seconds, so
            // it is cadence-invariant in physical units with no tick-scale
            // fudge — the same discipline energy_state.h imposes on Ps itself.
            {
                const gp_fitness esNow = static_cast<gp_fitness>(stepState.getSpecificEnergy());
                const double tNow = static_cast<double>(stepState.getSimTimeMsec());
                if (haveEs) {
                    const double dt = (tNow - prevEsTimeMsec) / 1000.0;
                    if (dt > 0.0) {
                        const gp_fitness ps = autoc::eval::specificExcessPower(
                            esNow, prevEs, static_cast<gp_fitness>(dt));
                        if (ps < 0.0) energyAccum += -ps * static_cast<gp_fitness>(dt);
                    }
                }
                prevEs = esNow;
                prevEsTimeMsec = tNow;
                haveEs = true;
            }

            // 030 M11.wrap T088 + 327-330 — tracker-mode diagnostic
            // accumulation. Observation-only; no effect on fitness or selection.
            if (is_tracker) {
                // Visibility from chase camera observations (per tick).
                bool left_vis = false, right_vis = false;
                // 041 T022 — was `cams[stepIndex - 1]` with a bounds test, the
                // sibling of the target offset above. Same fix: the view rides
                // in the record, so there is no pairing to state and none to
                // get wrong. Absent (pathgen, or a tick with no view recorded)
                // reads as not-visible, which is what absence means here.
                if (tickRecord.cameraView.has_value()) {
                    const CameraViewSample& cv = *tickRecord.cameraView;
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
                if (tickIndex > 0) {  // need a previous range
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
        result.energy_score = energyAccum;       // 041 P2-5: metres of Es destroyed; >= 0, lower = better
        result.crashed = isCrash(crashReason);
        result.crashReason = crashReason;

        // 041 P2-4 — attribute an egress to the bound it actually crossed.
        //
        // Re-checked from the TERMINAL recorded position against the RECORDED
        // arena, which is why the arena had to become part of RecordedRunConfig:
        // classifying against today's compiled default would silently re-label
        // every historical run each time the arena moves, and it moved three
        // times in one day.
        //
        // Only for CrashReason::Eval — that is the terminator arena egress maps
        // to. A HullStrike or a TimeLimit is not an egress and must not be
        // counted as one just because the final position happens to sit outside.
        if (crashReason == CrashReason::Eval && !ticks.empty()) {
            // ⚠️ NEAREST bound, not a re-run of checkArenaBounds.
            //
            // The first version called checkArenaBounds on the terminal state
            // and reported NONE for every single egress — which read as
            // "egFloor=0 egCeil=0 egRadius=0" in the per-gen line while the run
            // was in fact dying on the deck 16 times out of 16. The recorded
            // terminal state is the last SAMPLED one and sits marginally INSIDE
            // the bound that tripped (measured 0.01–0.51 m above a 25 m deck),
            // because the check and the state push happen at different points
            // in the tick. A strict re-check is therefore the wrong question.
            //
            // ⛔ A diagnostic that silently reports "none of the above" is worse
            // than no diagnostic: it looks like an answer.
            const auto& fa = evalResults.runConfig.flightArena;
            const gp_vec3 pos = ticks.back().state.getPosition();
            const gp_scalar alt_agl = -(pos.z() + SIM_INITIAL_ALTITUDE);
            const gp_scalar d_floor = alt_agl - fa.floor_agl_m;
            const gp_scalar d_ceil = fa.ceiling_agl_m - alt_agl;
            const gp_scalar d_radius =
                fa.radius_m - std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
            if (d_floor <= d_ceil && d_floor <= d_radius) {
                result.egress_kind = autoc::eval::ArenaEgressKind::FLOOR;
            } else if (d_ceil <= d_radius) {
                result.egress_kind = autoc::eval::ArenaEgressKind::CEILING;
            } else {
                result.egress_kind = autoc::eval::ArenaEgressKind::RADIUS;
            }
        }
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

            // 038 US3 — aux span/closure-prediction error (lexicase axis, gated
            // on EnablePredictorHead in selection). Computed always for tracker
            // so it is recorded even when the axis is off; 0 if the camera trace
            // is missing.
            result.prediction_score = computeSpanPredictionError(ticks);
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

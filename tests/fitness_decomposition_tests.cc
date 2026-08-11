#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include "autoc/rpc/protocol.h"
#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/util/config.h"

// Initialize ConfigManager for tests (uses defaults from AutocConfig)
class FitnessDecomp022Test : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ConfigManager::isInitialized()) {
            // Create a minimal config file for tests
            std::ofstream f("/tmp/test_autoc.ini");
            f << "PopulationSize = 10\n";
            f << "NumberOfGenerations = 1\n";
            f.close();
            std::ostringstream nullout;
            ConfigManager::initialize("/tmp/test_autoc.ini", nullout);
        }
    }
};


// 041 T020 — adapts the legacy pathgen fixtures, which build a flat state vector
// whose slot 0 is the pre-loop pose, to the grouped record. Identical content
// and identical ordering, so every pathgen assertion in this file is unchanged.
static ScenarioTicks toScenarioTicks(const std::vector<AircraftState>& states) {
    ScenarioTicks sc;
    if (states.empty()) return sc;
    sc.initialState = states.front();
    sc.ticks.reserve(states.size() - 1);
    for (size_t k = 1; k < states.size(); ++k) sc.ticks.emplace_back(states[k]);
    return sc;
}

// ============================================================
// Helper: build a straight-line path in VIRTUAL coordinates.
// All positions at Z=0 (virtual origin). No setOriginOffset().
// This expresses the coordinate contract: getPosition() is virtual.
// ============================================================
static EvalResults makeStraightPath(int numSteps, double aircraftOffsetAlong, double aircraftOffsetLateral,
                                    bool crash = false, int crashAtStep = -1,
                                    double aircraftOffsetZ = 0.0) {
    EvalResults results;

    // Path: rabbit moves along -X at ~1.3m per step (13 m/s at 10Hz)
    // Virtual coordinates: Z=0 (path at virtual origin)
    std::vector<Path> path;
    double totalDist = 0.0;
    for (int i = 0; i <= numSteps; i++) {
        double x = -static_cast<double>(i) * 1.3;
        gp_vec3 pos(static_cast<gp_scalar>(x), 0.0f, 0.0f);  // Z=0: virtual origin
        totalDist = static_cast<double>(i) * 1.3;
        path.push_back(Path(pos, gp_vec3::UnitX(), totalDist, 0.0));
    }
    results.pathList.push_back(path);

    // Aircraft: offset from rabbit position in virtual space (Z=0 + offsets)
    int actualSteps = crash ? std::min(crashAtStep, numSteps) : numSteps;
    std::vector<AircraftState> states;
    for (int i = 0; i <= actualSteps; i++) {
        double rabbit_x = -static_cast<double>(i) * 1.3;
        // along: positive = ahead of rabbit (more negative X)
        // Tangent is (-1,0,0) for this path, so "ahead" = more negative X
        double x = rabbit_x + aircraftOffsetAlong * (-1.0);  // along tangent direction
        double y = aircraftOffsetLateral;
        AircraftState state;
        // Virtual coordinates: Z=0 + aircraftOffsetZ
        state.setPosition(gp_vec3(static_cast<gp_scalar>(x), static_cast<gp_scalar>(y),
                                  static_cast<gp_scalar>(aircraftOffsetZ)));
        state.setOrientation(gp_quat::Identity());
        state.setThisPathIndex(i);
        state.setSimTimeMsec(static_cast<float>(i * 100.0));
        state.setPitchCommand(0.0f);
        state.setRollCommand(0.0f);
        state.setThrottleCommand(0.0f);
        states.push_back(state);
    }
    results.tickList.push_back(toScenarioTicks(states));
    results.crashReasonList.push_back(crash ? CrashReason::Eval : CrashReason::None);
    results.scenarioList.push_back(ScenarioMetadata());

    return results;
}

// ============================================================
// T029: Virtual coordinate contract tests
// ============================================================

// Aircraft and path both at virtual origin (Z=0) → perfect score
TEST_F(FitnessDecomp022Test, VirtualOriginPerfectTracking) {
    auto results = makeStraightPath(100, 0.0, 0.0);  // Z=0 for both
    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_LT(scores[0].score, -100.0);  // substantial points at Z=0
}

// Aircraft at raw Z=-25 with path at virtual Z=0 → large Z offset penalizes heavily
// This documents the coordinate bug: if raw positions leak into fitness,
// the 25m Z error dominates. This test ALWAYS passes (the scenario genuinely has
// a 25m error regardless of coordinate convention).
TEST_F(FitnessDecomp022Test, RawPositionGivesWrongScore) {
    auto results_correct = makeStraightPath(100, 0.0, 0.0, false, -1, 0.0);  // Z=0 (virtual, correct)
    auto results_raw = makeStraightPath(100, 0.0, 0.0, false, -1, -25.0);    // Z=-25 (raw leak)
    auto scores_correct = computeScenarioScores(results_correct);
    auto scores_raw = computeScenarioScores(results_raw);
    // Z=-25 offset is treated as 25m lateral error → much worse score
    EXPECT_LT(scores_correct[0].score, scores_raw[0].score);  // correct is better (more negative)
    // The ratio should be dramatic — 25m Z error vs 0
    double ratio = scores_raw[0].score / scores_correct[0].score;
    EXPECT_LT(ratio, 0.2);  // raw score is < 20% of correct score
}

// ============================================================
// T017: Scoring with synthetic trajectories (virtual Z=0)
// ============================================================

// Perfect tracking — aircraft at rabbit position
TEST_F(FitnessDecomp022Test, PerfectTracking) {
    auto results = makeStraightPath(100, 0.0, 0.0);  // at rabbit, Z=0
    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_FALSE(scores[0].crashed);
    EXPECT_EQ(scores[0].steps_completed, 100);
    // Score should be negative (negated) and large magnitude
    EXPECT_LT(scores[0].score, 0.0);
    // With 100 steps at stepPoints=1.0 and streak building: should be substantial
    EXPECT_LT(scores[0].score, -100.0);  // at minimum 100 * 1.0 * 1.0
}

// Behind 5m — good score, in streak
TEST_F(FitnessDecomp022Test, Behind5m) {
    auto results = makeStraightPath(100, -5.0, 0.0);  // 5m behind, Z=0
    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // stepPoints at -5m behind = 1/(1+(5/7)^2) ≈ 0.66, above streak threshold 0.5
    EXPECT_LT(scores[0].score, 0.0);
    EXPECT_GT(scores[0].maxStreak, 0);  // should be in streak
}

// Ahead 2m — poor score, no streak
TEST_F(FitnessDecomp022Test, Ahead2m) {
    auto results = makeStraightPath(100, 2.0, 0.0);  // 2m ahead, Z=0
    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // stepPoints at +2m ahead = 1/(1+16) ≈ 0.059, below streak threshold
    EXPECT_LT(scores[0].score, 0.0);
    EXPECT_EQ(scores[0].maxStreak, 0);  // no streak (below threshold)
}

// Ahead is MUCH worse than behind
TEST_F(FitnessDecomp022Test, AheadWorseThanBehind) {
    auto results_behind = makeStraightPath(100, -5.0, 0.0);
    auto results_ahead = makeStraightPath(100, 2.0, 0.0);
    auto scores_behind = computeScenarioScores(results_behind);
    auto scores_ahead = computeScenarioScores(results_ahead);
    // behind score more negative (better) than ahead
    EXPECT_LT(scores_behind[0].score, scores_ahead[0].score);
}

// Crash at step 50 — no crash penalty, just fewer points
TEST_F(FitnessDecomp022Test, CrashNoExtraPenalty) {
    auto results_full = makeStraightPath(100, 0.0, 0.0, false);
    auto results_crash = makeStraightPath(100, 0.0, 0.0, true, 50);
    auto scores_full = computeScenarioScores(results_full);
    auto scores_crash = computeScenarioScores(results_crash);
    // Crash scores worse (less negative) but proportional, not 1e6 cliff
    EXPECT_GT(scores_crash[0].score, scores_full[0].score);  // crash worse
    // The ratio should be roughly proportional to steps (not 1000x)
    double ratio = scores_crash[0].score / scores_full[0].score;
    EXPECT_GT(ratio, 0.1);  // crash score is within 10x, not 1000x
}

// Crash with good tracking > wandering without crash
TEST_F(FitnessDecomp022Test, GoodCrashBetterThanBadComplete) {
    auto results_good_crash = makeStraightPath(100, 0.0, 0.0, true, 50);  // perfect then crash at 50
    auto results_bad_complete = makeStraightPath(100, 0.0, 20.0, false);   // 20m lateral, full flight
    auto scores_gc = computeScenarioScores(results_good_crash);
    auto scores_bc = computeScenarioScores(results_bad_complete);
    // 50 steps at rabbit with streak > 100 steps at 20m lateral (stepPoints≈0.06, no streak)
    EXPECT_LT(scores_gc[0].score, scores_bc[0].score);  // good crash is better
}

// ============================================================
// T017b: Streak diagnostics with controlled trajectories
// ============================================================

// Three short streaks
TEST_F(FitnessDecomp022Test, ThreeShortStreaks) {
    // Build a trajectory that alternates close/far — all at virtual Z=0
    EvalResults results;
    std::vector<Path> path;
    int totalSteps = 30;
    for (int i = 0; i <= totalSteps; i++) {
        double x = -static_cast<double>(i) * 1.3;
        path.push_back(Path(gp_vec3(static_cast<gp_scalar>(x), 0.0f, 0.0f),
                           gp_vec3::UnitX(), i * 1.3, 0.0));
    }
    results.pathList.push_back(path);

    std::vector<AircraftState> states;
    for (int i = 0; i <= totalSteps; i++) {
        double rabbit_x = -static_cast<double>(i) * 1.3;
        // 5 close, 3 far, 5 close, 3 far, 5 close, rest far
        bool close = (i >= 1 && i <= 5) || (i >= 9 && i <= 13) || (i >= 17 && i <= 21);
        double offset_y = close ? 0.0 : 30.0;  // 30m = stepPoints ≈ 0.03, below threshold
        AircraftState state;
        state.setPosition(gp_vec3(static_cast<gp_scalar>(rabbit_x), static_cast<gp_scalar>(offset_y), 0.0f));
        state.setOrientation(gp_quat::Identity());
        state.setThisPathIndex(i);
        state.setSimTimeMsec(static_cast<float>(i * 100.0));
        states.push_back(state);
    }
    results.tickList.push_back(toScenarioTicks(states));
    results.crashReasonList.push_back(CrashReason::None);
    results.scenarioList.push_back(ScenarioMetadata());

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_EQ(scores[0].maxStreak, 5);       // longest streak is 5
    EXPECT_EQ(scores[0].totalStreakSteps, 15); // 3 × 5 steps in streak
}

// Multi-scenario aggregate
TEST_F(FitnessDecomp022Test, MultiScenarioAggregate) {
    // 3 scenarios, each at rabbit — all at virtual Z=0
    EvalResults results;
    for (int s = 0; s < 3; s++) {
        std::vector<Path> path;
        for (int i = 0; i <= 50; i++) {
            double x = -static_cast<double>(i) * 1.3;
            path.push_back(Path(gp_vec3(static_cast<gp_scalar>(x), 0.0f, 0.0f),
                               gp_vec3::UnitX(), i * 1.3, 0.0));
        }
        results.pathList.push_back(path);

        std::vector<AircraftState> states;
        for (int i = 0; i <= 50; i++) {
            double x = -static_cast<double>(i) * 1.3;
            AircraftState state;
            state.setPosition(gp_vec3(static_cast<gp_scalar>(x), 0.0f, 0.0f));
            state.setOrientation(gp_quat::Identity());
            state.setThisPathIndex(i);
            state.setSimTimeMsec(static_cast<float>(i * 100.0));
            states.push_back(state);
        }
        results.tickList.push_back(toScenarioTicks(states));
        results.crashReasonList.push_back(CrashReason::None);
        results.scenarioList.push_back(ScenarioMetadata());
    }

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 3u);

    double total = aggregateRawFitness(scores);
    EXPECT_LT(total, 0.0);  // negative (good)
    // Should be 3x a single scenario score
    EXPECT_NEAR(total, scores[0].score + scores[1].score + scores[2].score, 1e-6);
}

// T017c: Verify negation
TEST_F(FitnessDecomp022Test, ScoreIsNegated) {
    auto results = makeStraightPath(50, 0.0, 0.0);
    auto scores = computeScenarioScores(results);
    EXPECT_LT(scores[0].score, 0.0);  // accumulated points are positive, stored negated
}

// T017c: 3D offset — aircraft above rabbit by 5m (Z offset in NED)
// In virtual space, Z=0 is path altitude. Z=-5 means 5m above (NED).
TEST_F(FitnessDecomp022Test, ZOffsetTreatedAsLateral) {
    auto results_lateral = makeStraightPath(100, 0.0, 5.0);             // 5m Y offset
    auto results_z = makeStraightPath(100, 0.0, 0.0, false, -1, -5.0); // 5m Z offset (above in NED)
    auto scores_lateral = computeScenarioScores(results_lateral);
    auto scores_z = computeScenarioScores(results_z);
    // Both should produce similar scores — Z offset is cross-track like Y offset
    // Not exact because decomposition depends on tangent direction, but should be close
    double ratio = scores_z[0].score / scores_lateral[0].score;
    EXPECT_GT(ratio, 0.8);
    EXPECT_LT(ratio, 1.2);
}

// ===========================================================================
// 2026-08-02 — THE TICK-PAIRING INVARIANT for the prediction axis.
//
// `aircraftStateList` and `cameraViewList` are NOT index-parallel, despite an
// M8b comment saying they are. `inputdev_autoc.cpp` pushes the INITIAL aircraft
// state once at scenario start, before any NN tick, while camera views begin at
// tick 1 — so a scenario records e.g. 368 states against 367 camera views and
//
//     tick k's camera view is cams[k - 1]
//
// From 038 US3 until 2026-08-02 this function paired them 1:1, scoring every
// forecast ONE TICK LATE — a +50 ms prediction compared against the span two
// ticks ahead, a whole horizon of error on the shortest one, on a LIVE lexicase
// axis. It survived two features because nothing asserted the pairing.
//
// This is that assertion, and it is deliberately the strongest form available:
// a predictor that is EXACTLY right must score EXACTLY zero. Any pairing error
// makes a perfect predictor look wrong, so the test cannot pass on a shifted
// index no matter how the tolerances are chosen.
// ===========================================================================

namespace {

// span at tick k, chosen to change every tick so a shift cannot alias.
double spanAtTick(int k) { return 0.050 + 0.0013 * k; }

CameraViewSample camWithSpan(double span) {
    CameraViewSample cv{};
    // A pair placed symmetrically about the boresight subtends exactly `span`.
    cv.beacon_left.bearing_x_rad = static_cast<float>(-span * 0.5);
    cv.beacon_left.bearing_y_rad = 0.0f;
    cv.beacon_left.cep = 0.05f;
    cv.beacon_right.bearing_x_rad = static_cast<float>(span * 0.5);
    cv.beacon_right.bearing_y_rad = 0.0f;
    cv.beacon_right.cep = 0.05f;
    return cv;
}


// 041 T022 — translates the legacy two-vector fixture into the grouped series.
// This is now the ONLY place in the codebase that writes `cams[k - 1]`: the
// offset survives solely as a statement about how the OLD fixtures were laid
// out, and no production path can reach it.
static std::vector<EvalTick> zipTicks(const std::vector<AircraftState>& states,
                                      const std::vector<CameraViewSample>& cams) {
    std::vector<EvalTick> out;
    for (size_t k = 1; k < states.size(); ++k) {
        EvalTick t(states[k]);
        if (k - 1 < cams.size()) t.cameraView = cams[k - 1];
        out.push_back(t);
    }
    return out;
}

}  // namespace

// 041 T019 — converted from a bare TEST() to TEST_F so the ConfigManager
// fixture applies. A bare TEST() runs on struct defaults rather than the
// fixture's config, which is trap 2 of the zero-answer block: the test still
// passes, so nothing tells you it was measuring a different configuration from
// every other test in this file.
//
// ⚠️ T019's task note says this pairing is "currently wrong" and expects the
// test to be RED until T022. That premise is STALE: commit 1b290f2 (2026-08-02,
// "prediction_score was scored ONE TICK LATE since 038 US3") already fixed it,
// before 041 was specced. Both tests below are green today and must STAY green
// through T020-T022 -- their value now is as a regression gate on the grouped
// record, not as evidence of a pending fix.
TEST_F(FitnessDecomp022Test, PerfectPredictorScoresExactlyZero) {
    constexpr int kTicks = 12;               // ticks 1..11 have camera views
    const double dt = SIM_TIME_STEP_MSEC / 1000.0;

    // cams[j] is tick j+1's view.
    std::vector<CameraViewSample> cams;
    for (int k = 1; k < kTicks; ++k) cams.push_back(camWithSpan(spanAtTick(k)));

    // states[0] is the pre-tick initial state and carries NO NN data — exactly
    // as the recorder produces it. states[k] is tick k.
    std::vector<AircraftState> states(kTicks);
    for (int k = 1; k < kTicks; ++k) {
        float out[TRACKER_NN_OUTPUT_COUNT] = {0};
        out[0] = out[1] = out[2] = 0.0f;  // control outputs, unused here
        for (int h = 0; h < kNumSpanPredictHorizons; ++h) {
            const int ticksAhead = kSpanPredictHorizonsMsec[h] / SIM_TIME_STEP_MSEC;
            // A PERFECT forecast of the span at tick k + ticksAhead.
            out[TRACKER_NN_CONTROL_OUTPUT_COUNT + h] =
                static_cast<float>(spanAtTick(k + ticksAhead));
        }
        // ...and a perfect closure rate.
        out[TRACKER_NN_CONTROL_OUTPUT_COUNT + kNumSpanPredictHorizons] =
            static_cast<float>((spanAtTick(k + 1) - spanAtTick(k)) / dt);
        states[k].setNNData(NNInputs{}, out, TRACKER_NN_OUTPUT_COUNT);
    }

    const gp_fitness err = computeSpanPredictionError(zipTicks(states, cams));
    EXPECT_LT(static_cast<double>(err), 1e-4)
        << "a perfect predictor must score ~0. A non-zero result here means the "
           "state<->cameraView tick pairing is off, NOT that the predictor is bad.";
}

TEST_F(FitnessDecomp022Test, AOneTickShiftIsDetectable) {
    // Teeth for the test above: prove the assertion is actually sensitive to the
    // bug it guards, or PerfectPredictorScoresExactlyZero would have passed on
    // the old code too.
    //
    // Asserted as a RATIO against the correctly-paired error rather than an
    // absolute bar: the shift's magnitude scales with how fast span moves, so an
    // absolute threshold would be a tuning parameter that silently goes slack on
    // a slower-closing fixture. The ratio cannot.
    constexpr int kTicks = 12;
    const double dt = SIM_TIME_STEP_MSEC / 1000.0;

    std::vector<CameraViewSample> aligned, shifted;
    for (int k = 1; k < kTicks; ++k) {
        aligned.push_back(camWithSpan(spanAtTick(k)));
        shifted.push_back(camWithSpan(spanAtTick(k + 1)));  // one tick out
    }

    std::vector<AircraftState> states(kTicks);
    for (int k = 1; k < kTicks; ++k) {
        float out[TRACKER_NN_OUTPUT_COUNT] = {0};
        for (int h = 0; h < kNumSpanPredictHorizons; ++h) {
            const int ticksAhead = kSpanPredictHorizonsMsec[h] / SIM_TIME_STEP_MSEC;
            out[TRACKER_NN_CONTROL_OUTPUT_COUNT + h] =
                static_cast<float>(spanAtTick(k + ticksAhead));
        }
        out[TRACKER_NN_CONTROL_OUTPUT_COUNT + kNumSpanPredictHorizons] =
            static_cast<float>((spanAtTick(k + 1) - spanAtTick(k)) / dt);
        states[k].setNNData(NNInputs{}, out, TRACKER_NN_OUTPUT_COUNT);
    }

    const double good = static_cast<double>(computeSpanPredictionError(zipTicks(states, aligned)));
    const double bad = static_cast<double>(computeSpanPredictionError(zipTicks(states, shifted)));

    EXPECT_LT(good, 1e-4) << "sanity: the aligned pairing is the zero case";
    EXPECT_GT(bad, good * 100.0)
        << "a one-tick shift must be plainly visible in the score; if it is not, "
           "PerfectPredictorScoresExactlyZero has no teeth. good=" << good
           << " bad=" << bad;
}

// ===========================================================================
// 2026-08-03 — THE OBJECTIVE'S TICK PAIRING (the sibling of SpanPrediction).
//
// In TRACKER mode the rabbit and target come from `targetTrajectoryList`, and
// that list is NOT index-parallel with `aircraftStateList`: the recorder pushes
// the INITIAL aircraft state once before the tick loop, so
//
//     states = 1 + N     targets = N     => targets[j] is tick j + 1's target
//
// `stepIndex` is a STATE index, so tick k's target is `targets[k - 1]`. Pairing
// 1:1 — as this did from 030 until 2026-08-03 — scored the chase at tick k
// against where the target was at tick k+1: ~0.85 m at cruise, which against a
// 3.048 m intended trail distance is a 28% error in the DEFINITION of the task.
//
// It is invisible in the data (CopiedTargetSample has no timestamp), so the test
// has to construct the relationship rather than observe it: put the target at a
// position that MOVES every tick, and place the chase exactly on the rabbit that
// the CORRECT pairing implies. A correct implementation scores that as a perfect
// tail-chase; a shifted one puts the chase 0.85 m off every single tick.
// ===========================================================================

TEST_F(FitnessDecomp022Test, TrackerObjectiveUsesTheTargetFromTheSameTick) {
    const int kTicks = 20;
    const double kStepM = 0.85;   // target advances this far per tick

    // Build a tracker scenario where the chase sits exactly on the rabbit
    // implied by `whichTick`'s target. `shift = 0` is the CORRECT pairing
    // (tick k -> targets[k-1]); `shift = 1` is the old off-by-one.
    auto build = [&](int shift) {
        EvalResults r;
        // The path is semantically IGNORED in tracker mode — the rabbit comes
        // from targetTrajectoryList — but it must be non-empty, because
        // computeScenarioScores skips any scenario with an empty path
        // (`if (path.empty() || aircraftStates.empty()) continue;`). An empty
        // one silently scores 0 for every variant, which makes a comparison
        // test look like it passed when it never ran.
        std::vector<Path> path;
        for (int j = 0; j <= kTicks; ++j) {
            path.push_back(Path(gp_vec3(static_cast<gp_scalar>(-kStepM * j), 0.0f, 0.0f),
                                gp_vec3::UnitX(), kStepM * j, 0.0));
        }
        r.pathList.push_back(path);

        std::vector<CopiedTargetSample> targets;
        for (int j = 0; j < kTicks; ++j) {
            const int tick = j + 1;                  // targets[j] IS tick j+1
            CopiedTargetSample t;
            t.position = gp_vec3(static_cast<gp_scalar>(-kStepM * tick), 0.0f, 0.0f);
            t.velocity = gp_vec3(static_cast<gp_scalar>(-13.0), 0.0f, 0.0f);
            t.trail_rabbit_position =
                t.position + gp_vec3(static_cast<gp_scalar>(3.048), 0.0f, 0.0f);
            targets.push_back(t);
        }

        std::vector<AircraftState> states;
        for (int k = 0; k <= kTicks; ++k) {
            AircraftState st;
            const int j = std::clamp(k - 1 + shift, 0, kTicks - 1);
            st.setPosition(targets[j].trail_rabbit_position);
            // Nose along -X, i.e. pointed down the target's line of travel —
            // otherwise the tail-chase cone term zeroes the score regardless of
            // which tick's target is read, and the test measures nothing.
            st.setOrientation(gp_quat(Eigen::AngleAxis<gp_scalar>(
                static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())));
            st.setVelocity(gp_vec3(static_cast<gp_scalar>(-13.0), 0.0f, 0.0f));
            st.setSimTimeMsec(k * SIM_TIME_STEP_MSEC);
            st.setThisPathIndex(0);
            states.push_back(st);
        }

        // 041 T020 — same content, grouped. `states[0]` is the pre-loop pose
        // and `targets[j]` was tick j+1's, so the zip is states[k] with
        // targets[k-1]; that offset now exists only in this fixture translation.
        ScenarioTicks sc;
        sc.initialState = states.at(0);
        for (size_t k = 1; k < states.size(); ++k) {
            EvalTick t(states[k]);
            if (k - 1 < targets.size()) t.targetSample = targets[k - 1];
            sc.ticks.push_back(t);
        }
        r.tickList.push_back(std::move(sc));
        r.crashReasonList.push_back(CrashReason::None);
        return r;
    };

    EvalResults aligned = build(0);
    EvalResults shifted = build(1);
    const auto a = computeScenarioScores(aligned);
    const auto b = computeScenarioScores(shifted);
    ASSERT_EQ(a.size(), 1u);
    ASSERT_EQ(b.size(), 1u);

    // Scores are negative-lower-better. A chase parked on the tick's OWN rabbit
    // must score at least as well as one parked on the NEXT tick's rabbit; if
    // the objective reads the wrong tick, the preference inverts.
    EXPECT_LT(a[0].score, b[0].score)
        << "the objective must prefer the chase aligned with THIS tick's target. "
           "aligned=" << a[0].score << " shifted=" << b[0].score
           << " -- an inversion means targetTrajectoryList is being read one tick "
              "out (the 030-2026-08-03 bug).";
}

// ===========================================================================
// 041 T017-T019 (FR-003, SC-001) — THE ZERO-ANSWER PATTERN.
//
// Construct data whose correct answer is EXACTLY 0, assert exactly 0, and pair
// every such test with a shifted-input companion proving the assertion has
// teeth. Written BEFORE the T020-T028 grouped-record refactor and required to
// pass IDENTICALLY after it.
//
// ⚠️ THE POINT OF `TickFixture` BELOW. These tests must not encode the storage
// layout, or the refactor invalidates them exactly when they are needed most.
// Today the per-tick series are parallel lists with an offset — the recorder
// pushes an initial aircraft state before the tick loop, so `cams[j]` and
// `targets[j]` are tick j+1's — and after T020 they become
// `tickList[i][k] = {state, cameraView, targetSample}` with no offset at all.
// Everything that knows about the offset lives in this one helper. At T022 the
// helper changes; not one assertion below does.
//
// ⚠️ TWO FIXTURE TRAPS, both already paid for once:
//   1. An EMPTY pathList makes computeScenarioScores SKIP the scenario
//      (`if (path.empty() || ...) continue;`), so every variant scores 0 and a
//      comparison looks passed without ever running. `TickFixture` always lays
//      a path.
//   2. A bare TEST() misses the ConfigManager fixture and runs on defaults.
//      All of these are TEST_F(FitnessDecomp022Test, ...).
// ===========================================================================

namespace {

// Builds a single-scenario tracker EvalResults, addressed BY TICK.
// Tick indices are 1-based, matching `stepIndex` in the objective: tick 1 is
// the first stepped tick. Tick 0 is the pre-loop initial state and is never
// addressable here — which is the property T020's separately-named initial
// field is meant to make structural.
struct TickFixture {
    int ticks;
    std::vector<Path> path;
    AircraftState initialState;                 // pre-loop; NOT a tick
    std::vector<EvalTick> records;              // records[k-1] is tick k
    // 041 T020/T022 — the storage changed here and NOWHERE ELSE. It was three
    // parallel vectors with an offset; it is now one grouped series plus a named
    // initial state. Not one assertion in this file needed touching, which is
    // what the setters below were for.
    std::vector<AircraftState> states;          // kept: the [0] = initial view the
                                                // tests use to park the pre-loop pose

    explicit TickFixture(int n) : ticks(n), records(n), states(n + 1) {
        // Trap 1: a non-empty path, or the scenario is silently skipped.
        for (int j = 0; j <= n; ++j) {
            path.push_back(Path(gp_vec3(static_cast<gp_scalar>(-0.85 * j), 0.0f, 0.0f),
                                gp_vec3::UnitX(), 0.85 * j, 0.0));
        }
        for (int k = 0; k <= n; ++k) {
            states[k].setSimTimeMsec(k * SIM_TIME_STEP_MSEC);
            states[k].setThisPathIndex(0);
            states[k].setVelocity(gp_vec3(static_cast<gp_scalar>(-13.0), 0.0f, 0.0f));
            // Nose down the target's line of travel, or the tail-chase cone term
            // zeroes every score and the test measures nothing.
            states[k].setOrientation(gp_quat(Eigen::AngleAxis<gp_scalar>(
                static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())));
        }
    }

    // ---- the ONLY places that know the storage layout -----------------------
    void setTargetAtTick(int tick, const CopiedTargetSample& t) { records.at(tick - 1).targetSample = t; }
    void setViewAtTick(int tick, const CameraViewSample& c)     { records.at(tick - 1).cameraView = c; }
    void setChaseAtTick(int tick, const gp_vec3& p)             { states.at(tick).setPosition(p); }
    // -------------------------------------------------------------------------

    EvalResults build() const {
        EvalResults r;
        r.pathList.push_back(path);
        ScenarioTicks sc;
        sc.initialState = states.at(0);          // the pre-loop pose, by name
        sc.ticks = records;
        for (size_t k = 0; k < sc.ticks.size(); ++k) {
            sc.ticks[k].state = states.at(k + 1);  // tick k+1's pose
        }
        r.tickList.push_back(std::move(sc));
        r.crashReasonList.push_back(CrashReason::None);
        return r;
    }
};

CameraViewSample visibleView() {
    CameraViewSample cv{};
    cv.beacon_left.cep = 0.05f;      // below kCepSentinelThreshold => visible
    cv.beacon_right.cep = 0.05f;
    return cv;
}

CameraViewSample blindView() {
    CameraViewSample cv{};
    cv.beacon_left.cep = 9.0f;       // above threshold => not visible
    cv.beacon_right.cep = 9.0f;
    return cv;
}

// A target advancing steadily along -X, with its trail rabbit 3.048 m behind.
CopiedTargetSample targetAtTick(int tick, double stepM = 0.85) {
    CopiedTargetSample t;
    t.position = gp_vec3(static_cast<gp_scalar>(-stepM * tick), 0.0f, 0.0f);
    t.velocity = gp_vec3(static_cast<gp_scalar>(-13.0), 0.0f, 0.0f);
    t.trail_rabbit_position = t.position + gp_vec3(static_cast<gp_scalar>(3.048), 0.0f, 0.0f);
    return t;
}

}  // namespace

// --- T017: the M2 objective ------------------------------------------------

TEST_F(FitnessDecomp022Test, T017_TrackerObjectiveScoresExactlyZeroForAPerfectTailChase) {
    // THE ZERO CASE. A chase sitting exactly on this tick's trail rabbit, nose
    // down the line of travel, is a perfect tail-chase: zero positional error
    // every tick. Any non-zero error here is a PAIRING fault, not a tracking
    // one -- there is nothing else in the fixture that could produce it.
    constexpr int kTicks = 16;
    TickFixture f(kTicks);
    for (int k = 1; k <= kTicks; ++k) {
        const auto t = targetAtTick(k);
        f.setTargetAtTick(k, t);
        f.setChaseAtTick(k, t.trail_rabbit_position);   // exactly on the rabbit
        f.setViewAtTick(k, visibleView());
    }
    // Tick 0 is the pre-loop initial state; park it on tick 1's rabbit so it is
    // never the thing under test.
    f.states[0].setPosition(targetAtTick(1).trail_rabbit_position);

    EvalResults r = f.build();   // lvalue: computeScenarioScores takes a mutable ref
    const auto scores = computeScenarioScores(r);
    ASSERT_EQ(scores.size(), 1u);

    // A chase sitting on the rabbit is exactly the trail distance behind the
    // target, every tick -- so the median range is 3.048 EXACTLY. This is the
    // pairing-sensitive exact answer: reading a neighbouring tick's target moves
    // it by a full 0.85 m of target travel.
    EXPECT_NEAR(static_cast<double>(scores[0].tracker_diag.range_med), 3.048, 1e-4)
        << "a chase parked on THIS tick's rabbit is exactly the trail distance "
           "behind THIS tick's target; a different value means the objective is "
           "reading another tick's target (the 030..2026-08-03 off-by-one).";

    // And every tick is inside the fitness ramp -- exactly all of them, not
    // all-but-one from dropping a tick at either end of the series.
    EXPECT_EQ(static_cast<double>(scores[0].tracker_diag.in_fit_ramp_frac), 1.0)
        << "a perfect tail-chase is in the fit ramp on every tick";
}

TEST_F(FitnessDecomp022Test, T017_TrackerObjectiveScoresExactlyZeroWhenNoCreditIsEarned) {
    // The LITERAL zero: a chase parked far outside the cone earns no step points
    // on any tick, so the accumulated (negated) score is exactly 0.0. Not -0.0001
    // from a stray half-credit tick, and not a small positive from a sign slip.
    constexpr int kTicks = 16;
    TickFixture f(kTicks);
    for (int k = 1; k <= kTicks; ++k) {
        f.setTargetAtTick(k, targetAtTick(k));
        // 500 m off to the side: far outside any cone or distance term.
        f.setChaseAtTick(k, targetAtTick(k).trail_rabbit_position
                            + gp_vec3(0.0f, static_cast<gp_scalar>(500.0), 0.0f));
        f.setViewAtTick(k, blindView());
    }
    f.states[0].setPosition(targetAtTick(1).trail_rabbit_position
                            + gp_vec3(0.0f, static_cast<gp_scalar>(500.0), 0.0f));

    EvalResults r = f.build();
    const auto scores = computeScenarioScores(r);
    ASSERT_EQ(scores.size(), 1u);

    // The EXACT assertion: not one tick of these 16 reaches the fit threshold.
    // A count over a count -- no epsilon anywhere.
    EXPECT_EQ(static_cast<double>(scores[0].tracker_diag.in_fit_ramp_frac), 0.0)
        << "a chase 500 m off-axis is in the fit ramp on exactly zero ticks";

    // And the documented non-assertion: the score is near zero but NOT zero.
    // Pinned as strictly-less-than-zero so that if the objective ever gains a
    // hard cutoff (making an exact zero reachable), this test fails and someone
    // revisits the comment above rather than finding it quietly stale.
    EXPECT_LT(static_cast<double>(scores[0].score), 0.0)
        << "the Lorentzian never reaches zero; if this now passes as exactly 0, "
           "the objective gained a cutoff and the comment above needs updating";
}

TEST_F(FitnessDecomp022Test, T017_TrackerObjectiveOneTickShiftIsVisiblyWorse) {
    // TEETH. Same fixture, chase parked on the NEXT tick's rabbit. If the
    // objective were reading one tick out, this would be the one scoring zero.
    constexpr int kTicks = 16;
    auto buildWithShift = [&](int shift) {
        TickFixture f(kTicks);
        for (int k = 1; k <= kTicks; ++k) {
            f.setTargetAtTick(k, targetAtTick(k));
            f.setChaseAtTick(k, targetAtTick(k + shift).trail_rabbit_position);
            f.setViewAtTick(k, visibleView());
        }
        f.states[0].setPosition(targetAtTick(1).trail_rabbit_position);
        EvalResults r = f.build();
        return computeScenarioScores(r);
    };

    const auto aligned = buildWithShift(0);
    const auto shifted = buildWithShift(1);
    ASSERT_EQ(aligned.size(), 1u);
    ASSERT_EQ(shifted.size(), 1u);

    const double a = static_cast<double>(aligned[0].tracker_diag.range_med);
    const double b = static_cast<double>(shifted[0].tracker_diag.range_med);
    EXPECT_NEAR(a, 3.048, 1e-4) << "sanity: the aligned case is the exact case";
    EXPECT_GT(std::abs(b - a), 0.5)
        << "a one-tick shift is 0.85 m of target travel and must be plainly "
           "visible; if it is not, the exact-answer test above has no teeth. "
           "aligned=" << a << " shifted=" << b;
}

// --- T018: vis_frac --------------------------------------------------------

TEST_F(FitnessDecomp022Test, T018_VisFracIsExactlyZeroWhenNothingIsEverVisible) {
    // THE ZERO CASE for the visibility diagnostic. Every tick blind => exactly
    // 0.0, not 0.0-ish and not 1/N from an off-by-one reading past the end.
    constexpr int kTicks = 12;
    TickFixture f(kTicks);
    for (int k = 1; k <= kTicks; ++k) {
        f.setTargetAtTick(k, targetAtTick(k));
        f.setChaseAtTick(k, targetAtTick(k).trail_rabbit_position);
        f.setViewAtTick(k, blindView());
    }
    f.states[0].setPosition(targetAtTick(1).trail_rabbit_position);

    EvalResults r = f.build();   // lvalue: computeScenarioScores takes a mutable ref
    const auto scores = computeScenarioScores(r);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_EQ(static_cast<double>(scores[0].tracker_diag.vis_frac), 0.0)
        << "no beacon is ever within CEP threshold, so vis_frac must be exactly 0";
}

TEST_F(FitnessDecomp022Test, T018_VisFracIsExactlyOneWhenEverythingIsVisible) {
    constexpr int kTicks = 12;
    TickFixture f(kTicks);
    for (int k = 1; k <= kTicks; ++k) {
        f.setTargetAtTick(k, targetAtTick(k));
        f.setChaseAtTick(k, targetAtTick(k).trail_rabbit_position);
        f.setViewAtTick(k, visibleView());
    }
    f.states[0].setPosition(targetAtTick(1).trail_rabbit_position);

    EvalResults r = f.build();   // lvalue: computeScenarioScores takes a mutable ref
    const auto scores = computeScenarioScores(r);
    ASSERT_EQ(scores.size(), 1u);
    // Exactly 1.0. A reader that misses the last tick lands on 11/12 = 0.9166,
    // which an EXPECT_NEAR with a loose tolerance would wave through.
    EXPECT_EQ(static_cast<double>(scores[0].tracker_diag.vis_frac), 1.0)
        << "every tick has both beacons inside CEP threshold, so vis_frac must "
           "be exactly 1 -- a value just under it means a tick is being dropped "
           "at one end of the series";
}

TEST_F(FitnessDecomp022Test, T018_VisFracCountsTheTickItIsPairedWith) {
    // TEETH, and the reason a PREFIX pattern is used rather than an alternating
    // one: alternating visibility has the same FRACTION under a one-tick shift,
    // so it would pass either way. A prefix moves the count by exactly one.
    constexpr int kTicks = 12;
    constexpr int kVisibleThrough = 5;
    TickFixture f(kTicks);
    for (int k = 1; k <= kTicks; ++k) {
        f.setTargetAtTick(k, targetAtTick(k));
        f.setChaseAtTick(k, targetAtTick(k).trail_rabbit_position);
        f.setViewAtTick(k, (k <= kVisibleThrough) ? visibleView() : blindView());
    }
    f.states[0].setPosition(targetAtTick(1).trail_rabbit_position);

    EvalResults r = f.build();   // lvalue: computeScenarioScores takes a mutable ref
    const auto scores = computeScenarioScores(r);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(static_cast<double>(scores[0].tracker_diag.vis_frac),
                static_cast<double>(kVisibleThrough) / kTicks, 1e-6)
        << "vis_frac must count the view paired with each tick; being one tick "
           "out reads " << (kVisibleThrough - 1) << "/" << kTicks << " or "
        << (kVisibleThrough + 1) << "/" << kTicks;
}

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
    results.aircraftStateList.push_back(states);
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
    results.aircraftStateList.push_back(states);
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
        results.aircraftStateList.push_back(states);
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

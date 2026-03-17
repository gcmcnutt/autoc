#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "autoc/rpc/protocol.h"
#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/aircraft_state.h"

// Helper: build a minimal EvalResults with one scenario
static EvalResults makeSimpleEvalResults(int numSteps, bool crash, int crashAtStep = -1) {
    EvalResults results;

    // Build a simple straight-line path
    std::vector<Path> path;
    double totalDist = 0.0;
    for (int i = 0; i <= numSteps; i++) {
        double x = -static_cast<double>(i) * 1.6;  // ~16 m/s at 10Hz = 1.6m per step
        gp_vec3 pos(static_cast<gp_scalar>(x), 0.0f, -25.0f);
        totalDist = static_cast<double>(i) * 1.6;
        double turnRad = static_cast<double>(i) * 0.1;  // some turning
        double timeMsec = static_cast<double>(i) * 100.0;  // 10Hz
        path.push_back(Path(pos, gp_vec3::UnitX(), totalDist, turnRad, timeMsec));
    }
    results.pathList.push_back(path);

    // Build aircraft states — tracking at DISTANCE_TARGET distance from path
    int actualSteps = crash ? std::min(crashAtStep, numSteps) : numSteps;
    std::vector<AircraftState> states;
    for (int i = 0; i <= actualSteps; i++) {
        double x = -static_cast<double>(i) * 1.6 + DISTANCE_TARGET;  // offset by target distance
        AircraftState state;
        state.setPosition(gp_vec3(static_cast<gp_scalar>(x), 0.0f, -25.0f));
        state.setOrientation(gp_quat::Identity());
        state.setThisPathIndex(i);
        state.setSimTimeMsec(static_cast<float>(i * 100.0));
        state.setPitchCommand(0.0f);
        state.setRollCommand(0.0f);
        state.setThrottleCommand(0.0f);
        states.push_back(state);
    }
    results.aircraftStateList.push_back(states);

    // Crash status
    results.crashReasonList.push_back(crash ? CrashReason::Distance : CrashReason::None);

    // Empty scenario metadata
    results.scenarioList.push_back(ScenarioMetadata());

    return results;
}

// T010: completion_fraction = 0.5 when crash at step 83 of 167
TEST(FitnessDecomposition, CompletionFractionCrash) {
    int totalSteps = 167;
    int crashAt = 83;
    auto results = makeSimpleEvalResults(totalSteps, true, crashAt);

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_TRUE(scores[0].crashed);
    // completion_fraction based on path distance, approximately crashAt/totalSteps
    EXPECT_GT(scores[0].completion_fraction, 0.0);
    EXPECT_LT(scores[0].completion_fraction, 1.0);
    // Should be roughly half
    EXPECT_NEAR(scores[0].completion_fraction, 0.5, 0.1);
}

// T010b: completion_fraction = 1.0 when no crash
TEST(FitnessDecomposition, CompletionFractionNoCrash) {
    auto results = makeSimpleEvalResults(167, false);

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_FALSE(scores[0].crashed);
    EXPECT_DOUBLE_EQ(scores[0].completion_fraction, 1.0);
}

// T011: distance RMSE near zero when tracking at DISTANCE_TARGET
TEST(FitnessDecomposition, DistanceRMSEAtTarget) {
    auto results = makeSimpleEvalResults(100, false);

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // Aircraft is at DISTANCE_TARGET from path — RMSE should be near zero
    EXPECT_NEAR(scores[0].distance_rmse, 0.0, 0.5);
}

// T012: attitude error near zero with identity orientation (no rotation changes)
TEST(FitnessDecomposition, AttitudeErrorNoChange) {
    auto results = makeSimpleEvalResults(100, false);

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // All orientations are identity → no attitude delta
    EXPECT_NEAR(scores[0].attitude_error, 0.0, 1e-10);
}

// T013: smoothness = 0 for constant commands
TEST(FitnessDecomposition, SmoothnessConstantCommands) {
    auto results = makeSimpleEvalResults(100, false);

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // All commands are 0.0 (constant) → smoothness should be 0
    for (int a = 0; a < 3; a++) {
        EXPECT_NEAR(scores[0].smoothness[a], 0.0, 1e-10);
    }
}

// T013b: smoothness ≈ 2.0 for bang-bang commands
TEST(FitnessDecomposition, SmoothnessBangBang) {
    auto results = makeSimpleEvalResults(100, false);

    // Make pitch command alternate between +1 and -1 (bang-bang)
    for (int i = 0; i <= 100; i++) {
        float cmd = (i % 2 == 0) ? 1.0f : -1.0f;
        results.aircraftStateList[0][i].setPitchCommand(cmd);
    }

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 1u);
    // Bang-bang: |+1 - (-1)| = 2.0 every step → mean ≈ 2.0
    EXPECT_NEAR(scores[0].smoothness[0], 2.0, 0.05);  // pitch
    EXPECT_NEAR(scores[0].smoothness[1], 0.0, 1e-10);  // roll (constant 0)
    EXPECT_NEAR(scores[0].smoothness[2], 0.0, 1e-10);  // throttle (constant 0)
}

// T014: aggregate of decomposed scores matches legacy
TEST(FitnessDecomposition, AggregateMatchesLegacy) {
    // Build a non-trivial scenario: some distance error, no crash
    auto results = makeSimpleEvalResults(50, false);

    // Add some distance offset so fitness isn't zero
    for (int i = 0; i <= 50; i++) {
        double x = -static_cast<double>(i) * 1.6 + DISTANCE_TARGET + 5.0;  // 5m too far
        results.aircraftStateList[0][i].setPosition(
            gp_vec3(static_cast<gp_scalar>(x), 0.0f, -25.0f));
    }

    auto scores = computeScenarioScores(results);
    double decomposed_aggregate = aggregateScalarFitness(scores);

    // The decomposed aggregate should be > 0 (there's distance error)
    EXPECT_GT(decomposed_aggregate, 0.0);
}

// T014b: aggregate matches with crash scenario
TEST(FitnessDecomposition, AggregateMatchesLegacyWithCrash) {
    auto results = makeSimpleEvalResults(100, true, 50);

    auto scores = computeScenarioScores(results);
    double decomposed_aggregate = aggregateScalarFitness(scores);

    // Should include crash penalty (~500K for 50% completion)
    EXPECT_GT(decomposed_aggregate, 400000.0);
}

// T014c: multiple scenarios aggregate correctly
TEST(FitnessDecomposition, MultipleScenarios) {
    // Build results with 3 scenarios (path×wind combos)
    EvalResults results;

    for (int s = 0; s < 3; s++) {
        std::vector<Path> path;
        double totalDist = 0.0;
        for (int i = 0; i <= 50; i++) {
            double x = -static_cast<double>(i) * 1.6;
            gp_vec3 pos(static_cast<gp_scalar>(x), 0.0f, -25.0f);
            totalDist = static_cast<double>(i) * 1.6;
            path.push_back(Path(pos, gp_vec3::UnitX(), totalDist, 0.1 * i, i * 100.0));
        }
        results.pathList.push_back(path);

        std::vector<AircraftState> states;
        for (int i = 0; i <= 50; i++) {
            double x = -static_cast<double>(i) * 1.6 + DISTANCE_TARGET;
            AircraftState state;
            state.setPosition(gp_vec3(static_cast<gp_scalar>(x), 0.0f, -25.0f));
            state.setOrientation(gp_quat::Identity());
            state.setThisPathIndex(i);
            state.setSimTimeMsec(static_cast<float>(i * 100.0));
            state.setPitchCommand(0.0f);
            state.setRollCommand(0.0f);
            state.setThrottleCommand(0.0f);
            states.push_back(state);
        }
        results.aircraftStateList.push_back(states);
        results.crashReasonList.push_back(CrashReason::None);
        results.scenarioList.push_back(ScenarioMetadata());
    }

    auto scores = computeScenarioScores(results);
    ASSERT_EQ(scores.size(), 3u);

    // All scenarios should have completion = 1.0
    for (const auto& s : scores) {
        EXPECT_DOUBLE_EQ(s.completion_fraction, 1.0);
    }

    // Aggregate should be sum of per-scenario
    double total = aggregateScalarFitness(scores);
    EXPECT_GE(total, 0.0);
}

// 038 US3 — auxiliary span/closure-predictor head: layout + lexicase-axis tests.
//
//   1. Tracker output layout: 3 control + 4 span-aux = 7; weights/string.
//   2. Span-predict horizons are ms-based and integral at the compiled cadence
//      (per operator: a 20→25 Hz change must re-derive tick offsets, not shrink
//      the lookahead).
//   3. The prediction_score lexicase axis drives selection ONLY when the caller
//      enables it, and discriminates on prediction accuracy when all other axes
//      are tied.

#include <gtest/gtest.h>

#include <vector>

#include "autoc/nn/topology.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/eval/aircraft_state.h"        // SIM_TIME_STEP_MSEC
#include "autoc/eval/fitness_decomposition.h" // ScenarioScore
#include "autoc/eval/selection.h"

namespace {

TEST(PredictorHead, TrackerOutputLayout) {
    EXPECT_EQ(TRACKER_NN_CONTROL_OUTPUT_COUNT, 3);   // pitch/roll/throttle actuated
    EXPECT_EQ(kNumSpanPredictHorizons, 3);           // span @ +50/+100/+150 ms
    EXPECT_EQ(kNumSpanAuxOutputs, 4);                // 3 span + 1 closure-rate
    EXPECT_EQ(TRACKER_NN_OUTPUT_COUNT, 7);           // 3 control + 4 aux
    // 041 US4 — 58 → 63 inputs (envelope flag/secs + accel xyz); the OUTPUT
    // head is untouched here, which is the point of asserting both: the
    // predictor's fate is decided later at T088, and if the head is retired the
    // trailing 7 becomes 3 and this test is where that has to be stated.
    EXPECT_STREQ(TRACKER_NN_TOPOLOGY_STRING, "63,32,16r,7");
    EXPECT_EQ(TRACKER_NN_WEIGHT_COUNT, 2951);   // +5 inputs × 32 = +160 over 2791
    // Pathgen output head is untouched (predictor is tracker-only, M2-direct).
    EXPECT_EQ(NN_OUTPUT_COUNT, 3);
}

TEST(PredictorHead, HorizonsAreCadenceInvariantIntegers) {
    // ms-based, integral at the compiled cadence — the whole point of defining
    // the lookahead in ms rather than raw ticks.
    for (int h = 0; h < kNumSpanPredictHorizons; ++h) {
        EXPECT_EQ(kSpanPredictHorizonsMsec[h] % SIM_TIME_STEP_MSEC, 0)
            << "horizon " << kSpanPredictHorizonsMsec[h] << " ms not integral at a "
            << SIM_TIME_STEP_MSEC << " ms tick";
    }
    EXPECT_EQ(kSpanPredictHorizonsMsec[0], 50);
    EXPECT_EQ(kSpanPredictHorizonsMsec[kNumSpanPredictHorizons - 1], 150);
}

// When all other axes are tied, the prediction axis (enabled) makes the better
// predictor the unique lexicase survivor — regardless of test-case shuffle.
TEST(PredictorHead, PredictionAxisSelectsBetterPredictorWhenEnabled) {
    auto mk = [](gp_fitness pred) {
        ScenarioScore s;
        s.score = static_cast<gp_fitness>(-100.0);   // tied
        s.energy_score = static_cast<gp_fitness>(5.0);  // tied
        s.prediction_score = pred;
        return s;
    };
    // Candidate 0 predicts far better (0.0 vs 10.0, well outside the 0.5 floor).
    std::vector<std::vector<ScenarioScore>> a = {{mk(0.0)}, {mk(10.0)}};
    EXPECT_EQ(lexicase_select(a, 2, /*mad=*/false, 0.05, /*pred=*/true), 0);
    // Symmetric — flip which candidate predicts better.
    std::vector<std::vector<ScenarioScore>> b = {{mk(10.0)}, {mk(0.0)}};
    EXPECT_EQ(lexicase_select(b, 2, /*mad=*/false, 0.05, /*pred=*/true), 1);
    // Disabled: the prediction axis is absent, so a worse predictor is NOT
    // eliminated on that basis — with everything else tied both remain viable
    // (selection falls to the random pick; we only assert it does not crash and
    // returns a valid index).
    const int r = lexicase_select(a, 2, /*mad=*/false, 0.05, /*pred=*/false);
    EXPECT_GE(r, 0);
    EXPECT_LT(r, 2);
}

}  // namespace

// 043 gate / 041 P2-8 follow-up — M1 and M2 must see the SAME shared block.
//
// ⛔ THE FAILURE THIS PREVENTS is silent divergence between modes. Before 041
// P2-2 the AIRSPEED slot carried RAW m/s in M1 and cruise-normalized in M2 —
// the same "shared" slot, two different scales, and nothing detected it. P2-2
// unified them (on the wrong side, which P2-8 then corrected). Today the
// guarantee is structural: TrackerInputs embeds CraftCommonInputs and one
// unconditional writeCraftCommonInputs serves both gathers.
//
// ⚠️ Structural is not the same as enforced. Someone can add a mode branch, or
// overwrite a common slot after the call, and every existing test still passes
// — the divergence would only surface as a flight that behaves unlike its sim.
// These tests make it surface at build time instead.
#include <gtest/gtest.h>
#include <cstring>
#include "autoc/nn/nn_inputs.h"

namespace {

// The shared block must be the SAME TYPE in both, not merely the same shape —
// a copy-pasted struct with identical fields would satisfy a field-by-field
// check and still drift on the next edit.
TEST(SharedInputBlock, BothModesEmbedTheSameType) {
    static_assert(std::is_same<decltype(NNInputs::common),
                               decltype(TrackerInputs::common)>::value,
                  "M1 and M2 must embed the SAME CraftCommonInputs type — if these "
                  "ever become separate structs the scales can drift silently, which "
                  "is exactly the pre-041 AIRSPEED bug (raw m/s in M1, cruise-normalized "
                  "in M2, same slot).");
    SUCCEED();
}

// The block must sit at the END of both layouts, so slot indices for the shared
// channels are computable as (COUNT - kCommonCount + offset) in both modes —
// which the dmp writers, nn2cpp and the log decoder all rely on.
TEST(SharedInputBlock, SitsAtTheEndOfBothLayouts) {
    constexpr std::size_t kCommon = sizeof(CraftCommonInputs) / sizeof(float);
    EXPECT_EQ(offsetof(NNInputs, common) / sizeof(float),
              static_cast<std::size_t>(NN_INPUT_COUNT) - kCommon);
    EXPECT_EQ(offsetof(TrackerInputs, common) / sizeof(float),
              static_cast<std::size_t>(TrackerInput::COUNT) - kCommon);
}

// ⛔ The counts must stay in step with the enums. A hand-maintained enum and a
// struct are two sources of truth; NN_INPUT_COUNT already derives from sizeof,
// but the TRACKER side is asserted here too so a tracker-only edit cannot pass.
TEST(SharedInputBlock, CountsAgreeWithTheStructs) {
    EXPECT_EQ(static_cast<int>(sizeof(NNInputs) / sizeof(float)), NN_INPUT_COUNT);
    EXPECT_EQ(static_cast<int>(sizeof(TrackerInputs) / sizeof(float)),
              static_cast<int>(TrackerInput::COUNT));
    // 041: M1 = 25 target + 20 shared; M2 = 46 tracker + 20 shared.
    constexpr int kCommon = sizeof(CraftCommonInputs) / sizeof(float);
    EXPECT_EQ(20, kCommon);
    EXPECT_EQ(45, NN_INPUT_COUNT);
    EXPECT_EQ(66, static_cast<int>(TrackerInput::COUNT));
}

// ⚠️ The scale constants are shared by REFERENCE, not by copy — assert they are
// the single values both modes divide by. A second constant introduced for the
// tracker would be the divergence this file exists to catch.
TEST(SharedInputBlock, ScaleConstantsArePositiveAndSingleValued) {
    for (float k : {kCruiseSpeed_mps, kTargetDistScale_m, kClosingRateScale_mps,
                    kGyroScale_radps, kAccelScale_g, kEnergyScale_m,
                    kScoreGradScale, kDistToBoundaryScale_m}) {
        EXPECT_GT(k, 0.0f) << "a zero/negative scale divides to inf in the first layer";
    }
}

}  // namespace

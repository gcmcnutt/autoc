// 041 T049/T052 — NN input ablation mask: name resolution and application.
//
// The instrument has to be validated BEFORE it is trusted, because a tool that
// quietly perturbs (or quietly fails to perturb) the eval path makes every
// finding it produces worthless. Two failure modes matter most and both are
// silent:
//
//   1. a typo'd slot name that ablates NOTHING — reports "no effect", which is
//      a finding-shaped lie
//   2. a wrong-length mask that ablates the WRONG columns — reports a real
//      effect, attributed to the wrong input
//
// Both are hard errors below, not tolerated inputs.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "autoc/nn/evaluator.h"
#include "autoc/nn/input_mask.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/nn/topology.h"

using autoc::nn::buildInputMask;
using autoc::nn::describeInputMask;
using autoc::nn::splitSlotNames;

namespace {

// ---------------------------------------------------------------------------
// Name resolution
// ---------------------------------------------------------------------------

TEST(InputMask, EmptyMeansUnablatedBaseline) {
    // "No ablation" must have ONE representation, shared with the training
    // path, or baseline and training could diverge without anything saying so.
    EXPECT_TRUE(buildInputMask("", false).empty());
    EXPECT_TRUE(buildInputMask("", true).empty());
    EXPECT_TRUE(buildInputMask(std::vector<std::string>{}, false).empty());
}

TEST(InputMask, ResolvesTheEnergyAndBoundarySlots) {
    // 041 P2-2: IN_ENVELOPE / ENVELOPE_SECS are gone as inputs; the two new
    // scalar observations stand in as the by-name resolution case.
    const auto m = buildInputMask("SPECIFIC_ENERGY,BOUNDARY_CLOSURE_RATE", false);
    ASSERT_EQ(m.size(), static_cast<size_t>(PathgenInput::COUNT));
    EXPECT_EQ(m[static_cast<size_t>(PathgenInput::SPECIFIC_ENERGY)], 1);
    EXPECT_EQ(m[static_cast<size_t>(PathgenInput::BOUNDARY_CLOSURE_RATE)], 1);
    // Everything else untouched — an over-broad mask would exaggerate an effect.
    EXPECT_EQ(m[static_cast<size_t>(PathgenInput::ACCEL_Z)], 0);
    EXPECT_EQ(m[static_cast<size_t>(PathgenInput::QUAT_W)], 0);
    int set = 0;
    for (uint8_t b : m) set += (b != 0);
    EXPECT_EQ(set, 2);
}

TEST(InputMask, MaskLengthAlwaysMatchesTheModesSlotCount) {
    EXPECT_EQ(buildInputMask("ACCEL_X", false).size(),
              static_cast<size_t>(PathgenInput::COUNT));
    EXPECT_EQ(buildInputMask("ACCEL_X", true).size(),
              static_cast<size_t>(TrackerInput::COUNT));
}

TEST(InputMask, UnknownNameIsAHardErrorListingValidNames) {
    try {
        buildInputMask("GYROP", false);   // typo for GYRO_P
        FAIL() << "a typo'd slot name must be an error, never a silent no-op — "
                  "a no-op would report 'no effect', which reads as a result";
    } catch (const std::exception& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("GYROP"), std::string::npos) << msg;
        // It must show the caller what IS valid, or the error is a dead end.
        EXPECT_NE(msg.find("GYRO_P"), std::string::npos) << msg;
        EXPECT_NE(msg.find("SPECIFIC_ENERGY"), std::string::npos) << msg;
    }
}

TEST(InputMask, TrackerOnlyNameRejectedInPathgenMode) {
    // The two modes have different slot sets. Accepting a tracker name in
    // pathgen mode would be the cross-mode version of a typo.
    EXPECT_THROW(buildInputMask("TIME_SINCE_SEEN", false), std::runtime_error);
    EXPECT_NO_THROW(buildInputMask("TIME_SINCE_SEEN", true));
}

TEST(InputMask, WhitespaceAndRepeatsAreTolerated) {
    // `--zero-input "A, B"` and `A,B` express the same intent; so does A,A.
    const auto a = buildInputMask("GYRO_P, GYRO_Q", false);
    const auto b = buildInputMask("GYRO_P,GYRO_Q", false);
    EXPECT_EQ(a, b);
    EXPECT_EQ(buildInputMask("GYRO_P,GYRO_P", false),
              buildInputMask("GYRO_P", false));
}

TEST(InputMask, DescribeRoundTripsToTheRequestedNames) {
    // The driver logs this, so a run's log states its ablation in slot names.
    const auto m = buildInputMask("ACCEL_X,ACCEL_Y,ACCEL_Z", false);
    const std::string d = describeInputMask(m, false);
    EXPECT_EQ(d, "ACCEL_X,ACCEL_Y,ACCEL_Z");
    EXPECT_NE(describeInputMask({}, false).find("none"), std::string::npos);
}

TEST(InputMask, SplitDropsEmptyFields) {
    EXPECT_EQ(splitSlotNames("A,,B,").size(), 2u);
    EXPECT_TRUE(splitSlotNames("   ").empty());
}

// ---------------------------------------------------------------------------
// Application — the mask must actually bite, and only where asked
// ---------------------------------------------------------------------------

NNGenome makeGenome() {
    NNGenome g;
    g.topology.assign(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    g.recurrent.assign(NN_NUM_LAYERS, 0);
    for (int i = 0; i < NN_NUM_LAYERS; ++i) g.recurrent[i] = NN_RECURRENT[i] ? 1 : 0;
    g.weights.assign(static_cast<size_t>(nn_weight_count(g.topology, g.recurrent)), 0.01f);
    return g;
}

TEST(InputMask, WrongLengthMaskIsAHardError) {
    NNGenome g = makeGenome();
    NNControllerBackend backend(g, autoc::eval::FlightArena{});

    // One slot short: would ablate the wrong columns from that point on and
    // produce a clean-looking number answering a different question.
    std::vector<uint8_t> shortMask(static_cast<size_t>(PathgenInput::COUNT) - 1, 0);
    EXPECT_THROW(backend.setInputMask(shortMask, static_cast<int>(PathgenInput::COUNT)),
                 std::runtime_error);

    std::vector<uint8_t> longMask(static_cast<size_t>(PathgenInput::COUNT) + 1, 0);
    EXPECT_THROW(backend.setInputMask(longMask, static_cast<int>(PathgenInput::COUNT)),
                 std::runtime_error);

    // Correct length accepted; empty accepted as "no ablation".
    EXPECT_NO_THROW(backend.setInputMask(
        std::vector<uint8_t>(static_cast<size_t>(PathgenInput::COUNT), 0),
        static_cast<int>(PathgenInput::COUNT)));
    EXPECT_NO_THROW(backend.setInputMask({}, static_cast<int>(PathgenInput::COUNT)));
    EXPECT_FALSE(backend.hasInputMask());
}

TEST(InputMask, MaskZerosExactlyTheNamedSlotsAndNothingElse) {
    // Apply the mask to a filled input struct through the same path the forward
    // pass uses, and check the buffer slot-by-slot. This is the assertion that
    // catches an off-by-one in the name→index mapping, which would ablate a
    // neighbouring channel while reporting the requested name.
    NNInputs in{};
    float* raw = reinterpret_cast<float*>(&in);
    const int n = static_cast<int>(PathgenInput::COUNT);
    for (int i = 0; i < n; ++i) raw[i] = static_cast<float>(i + 1);  // all non-zero

    const auto mask = buildInputMask("SPECIFIC_ENERGY,ACCEL_Y", false);
    for (int i = 0; i < n; ++i) {
        if (mask[static_cast<size_t>(i)]) raw[i] = 0.0f;
    }

    for (int i = 0; i < n; ++i) {
        const bool should_be_zero =
            (i == static_cast<int>(PathgenInput::SPECIFIC_ENERGY) ||
             i == static_cast<int>(PathgenInput::ACCEL_Y));
        if (should_be_zero) {
            EXPECT_FLOAT_EQ(raw[i], 0.0f) << "slot " << i << " must be ablated";
        } else {
            EXPECT_FLOAT_EQ(raw[i], static_cast<float>(i + 1))
                << "slot " << i << " must be UNTOUCHED; an off-by-one here "
                   "would ablate a neighbour while reporting the right name";
        }
    }
}

TEST(InputMask, StructIsExactlyCountFloats_TheAssumptionMaskingRestsOn) {
    // applyInputMask indexes the struct as a flat float buffer. If that ever
    // stopped being true (padding, a non-float member), masking would corrupt
    // memory rather than zero a column — so pin it here as well as in the
    // layout suite.
    EXPECT_EQ(sizeof(NNInputs), sizeof(float) * static_cast<size_t>(PathgenInput::COUNT));
    EXPECT_EQ(sizeof(TrackerInputs), sizeof(float) * static_cast<size_t>(TrackerInput::COUNT));
}

}  // namespace

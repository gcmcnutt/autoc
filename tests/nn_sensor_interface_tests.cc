// 030 M2 — Type-safe NN sensor interface (FR-006 + FR-019).
//
// Anchors PathgenInput / TrackerInput enum slot counts and meta-array
// consistency. Fires loud if either enum drifts without a corresponding
// meta-array update — catches the silent-corruption class of bugs that
// magic-number indexing has historically produced.

#include <gtest/gtest.h>
#include <cstring>

#include "autoc/nn/nn_inputs.h"

namespace {

constexpr size_t pathgen_meta_size() {
    return sizeof(kPathgenInputMeta) / sizeof(SensorInputMeta);
}
constexpr size_t tracker_meta_size() {
    return sizeof(kTrackerInputMeta) / sizeof(SensorInputMeta);
}

}  // namespace

TEST(NNSensorInterface, PathgenInputCountMatchesNNInputs) {
    EXPECT_EQ(static_cast<int>(PathgenInput::COUNT), 33);
    EXPECT_EQ(static_cast<int>(PathgenInput::COUNT), NN_INPUT_COUNT);
    EXPECT_EQ(pathgen_meta_size(), 33u);
}

TEST(NNSensorInterface, TrackerInputCountIs48) {
    EXPECT_EQ(static_cast<int>(TrackerInput::COUNT), 48);
    EXPECT_EQ(tracker_meta_size(), 48u);
}

TEST(NNSensorInterface, PathgenMetaWellFormed) {
    for (size_t i = 0; i < pathgen_meta_size(); ++i) {
        const auto& m = kPathgenInputMeta[i];
        ASSERT_NE(m.name, nullptr);
        ASSERT_NE(m.display_name, nullptr);
        EXPECT_GT(std::strlen(m.name), 0u) << "PathgenInput meta[" << i << "] has empty name";
        EXPECT_GT(std::strlen(m.display_name), 0u)
            << "PathgenInput meta[" << i << "] has empty display_name (data.dat header)";
    }
}

TEST(NNSensorInterface, TrackerMetaWellFormed) {
    for (size_t i = 0; i < tracker_meta_size(); ++i) {
        const auto& m = kTrackerInputMeta[i];
        ASSERT_NE(m.name, nullptr);
        ASSERT_NE(m.display_name, nullptr);
        EXPECT_GT(std::strlen(m.name), 0u) << "TrackerInput meta[" << i << "] has empty name";
        EXPECT_GT(std::strlen(m.display_name), 0u)
            << "TrackerInput meta[" << i << "] has empty display_name";
    }
}

TEST(NNSensorInterface, PathgenAnchorPositions) {
    // Anchor a few enum positions against the NNInputs struct field layout
    // (= cereal byte order). If reordering is needed, this test forces an
    // explicit, traceable update — not a silent index shift.
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_X_TM5), 0);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_X_NOW), 5);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_Y_TM5), 6);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_Z_TM5), 12);
    EXPECT_EQ(static_cast<int>(PathgenInput::DIST_TM5), 18);
    EXPECT_EQ(static_cast<int>(PathgenInput::CLOSING_RATE), 24);
    EXPECT_EQ(static_cast<int>(PathgenInput::QUAT_W), 25);
    EXPECT_EQ(static_cast<int>(PathgenInput::AIRSPEED), 29);
    EXPECT_EQ(static_cast<int>(PathgenInput::GYRO_P), 30);
    EXPECT_EQ(static_cast<int>(PathgenInput::GYRO_R), 32);
}

TEST(NNSensorInterface, TrackerAnchorPositions) {
    // Anchor positions per FR-006 + FR-016: 36 beacon + 8 state + 4 arena.
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_X_TM5), 0);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_CEP_NOW), 17);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_X_TM5), 18);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_CEP_NOW), 35);
    EXPECT_EQ(static_cast<int>(TrackerInput::QUAT_W), 36);
    EXPECT_EQ(static_cast<int>(TrackerInput::AIRSPEED), 40);
    EXPECT_EQ(static_cast<int>(TrackerInput::GYRO_P), 41);
    EXPECT_EQ(static_cast<int>(TrackerInput::HOME_X), 44);
    EXPECT_EQ(static_cast<int>(TrackerInput::HOME_DIST), 47);
}

TEST(NNSensorInterface, PathgenMetaNamesMatchAnchors) {
    // Spot-check that the meta-array `name` field at known indices matches
    // the enum identifier. Enforces the parallel-array invariant.
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::TARGET_X_TM5)].name,
                 "TARGET_X_TM5");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::CLOSING_RATE)].name,
                 "CLOSING_RATE");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::GYRO_R)].name, "GYRO_R");
}

TEST(NNSensorInterface, PathgenDisplayNamesMatchExistingHeader) {
    // The data.dat header in src/autoc.cc:644-651 today emits these labels.
    // M2b walks kPathgenInputMeta to derive them; this test locks the
    // labels in so M2b's refactor produces a byte-identical header.
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::TARGET_X_TM5)].display_name,
                 "tgX-5");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::TARGET_X_NOW)].display_name,
                 "tgX0");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::DIST_TM5)].display_name,
                 "ds-5");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::CLOSING_RATE)].display_name,
                 "dd/dt");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::QUAT_W)].display_name, "qw");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::AIRSPEED)].display_name, "vel");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::GYRO_P)].display_name, "gyrP");
    EXPECT_STREQ(kPathgenInputMeta[static_cast<int>(PathgenInput::GYRO_R)].display_name, "gyrR");
}

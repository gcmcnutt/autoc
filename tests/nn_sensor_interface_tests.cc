// 030 M2 — Type-safe NN sensor interface (FR-006 + FR-019).
//
// Anchors PathgenInput / TrackerInput enum slot counts and meta-array
// consistency. Fires loud if either enum drifts without a corresponding
// meta-array update — catches the silent-corruption class of bugs that
// magic-number indexing has historically produced.

#include <gtest/gtest.h>
#include <cstring>
#include <iomanip>
#include <sstream>

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

TEST(NNSensorInterface, TrackerInputCountIs54) {
    // 030 M7a Session 2026-05-07 Q1: was 48 with HOME_X/Y/Z/HOME_DIST,
    // then 45 with single DIST_TO_BOUNDARY_ALONG_VEL.
    // 032 phase 1: 45 + 9 derived (span[6] + span_rate + tilt sin/cos) = 54.
    EXPECT_EQ(static_cast<int>(TrackerInput::COUNT), 54);
    EXPECT_EQ(tracker_meta_size(), 54u);
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
    // Anchor positions per FR-006 + FR-016 + Session 2026-05-07 Q1 + 032 phase 1:
    // 36 beacon + 8 state + 1 arena + 9 derived = 54 inputs.
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_X_TM5), 0);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_CEP_NOW), 17);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_X_TM5), 18);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_CEP_NOW), 35);
    EXPECT_EQ(static_cast<int>(TrackerInput::QUAT_W), 36);
    EXPECT_EQ(static_cast<int>(TrackerInput::AIRSPEED), 40);
    EXPECT_EQ(static_cast<int>(TrackerInput::GYRO_P), 41);
    EXPECT_EQ(static_cast<int>(TrackerInput::DIST_TO_BOUNDARY_ALONG_VEL), 44);
    // 032 phase 1 — derived perceptual features at slots 45..53
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_TM5), 45);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_NOW), 50);
    EXPECT_EQ(static_cast<int>(TrackerInput::SPAN_RATE), 51);
    EXPECT_EQ(static_cast<int>(TrackerInput::TARGET_TILT_SIN), 52);
    EXPECT_EQ(static_cast<int>(TrackerInput::TARGET_TILT_COS), 53);
    EXPECT_EQ(static_cast<int>(TrackerInput::COUNT), 54);
}

TEST(NNSensorInterface, TrackerDerivedFeatureNamesCanonical) {
    // 032 phase 1 — name round-trip for the 9 new slots. Catches drift
    // between the enum and kTrackerInputMeta name strings.
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_TM5)].name,
                 "BEACON_PAIR_SPAN_TM5");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_NOW)].name,
                 "BEACON_PAIR_SPAN_NOW");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::SPAN_RATE)].name,
                 "SPAN_RATE");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::TARGET_TILT_SIN)].name,
                 "TARGET_TILT_SIN");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::TARGET_TILT_COS)].name,
                 "TARGET_TILT_COS");
}

TEST(NNSensorInterface, TrackerDerivedFeatureDisplayNamesCanonical) {
    // 032 phase 1 — data.dat column header labels for the 9 new slots.
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_TM5)].display_name,
                 "spn-5");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_NOW)].display_name,
                 "spn0");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::SPAN_RATE)].display_name,
                 "dspn");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::TARGET_TILT_SIN)].display_name,
                 "tltS");
    EXPECT_STREQ(kTrackerInputMeta[static_cast<int>(TrackerInput::TARGET_TILT_COS)].display_name,
                 "tltC");
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

TEST(NNSensorInterface, PathgenHeaderWidthsMatchFormatString) {
    // header_width values must match the corresponding format spec in
    // src/autoc.cc per-tick line emitter so meta-walk header lines up
    // with %N.Mf data columns. Pathgen layout per format string at
    // src/autoc.cc:677-683:
    //   target_x/y/z[6]: " % 6.3f" → 7-wide
    //   dist[6]:         " % 6.1f" → 7-wide
    //   closing_rate:    " % 6.1f" → 7-wide
    //   quat[4]:         " % 7.4f" → 8-wide
    //   airspeed:        " % 7.4f" → 8-wide
    //   gyro_p/q/r:      " % 6.3f" → 7-wide
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::TARGET_X_TM5)].header_width, 7);
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::DIST_NOW)].header_width, 7);
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::CLOSING_RATE)].header_width, 7);
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::QUAT_W)].header_width, 8);
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::AIRSPEED)].header_width, 8);
    EXPECT_EQ(kPathgenInputMeta[static_cast<int>(PathgenInput::GYRO_P)].header_width, 7);
}

TEST(NNSensorInterface, PathgenMetaWalkProducesExistingHeaderText) {
    // Byte-identity gate: meta-walk concat must equal the literal header
    // strings the pre-M2b autoc.cc emitted. Future drift in either the
    // meta array OR the autoc.cc emitter is caught here.
    std::ostringstream walk;
    for (size_t i = 0; i < sizeof(kPathgenInputMeta) / sizeof(SensorInputMeta); ++i) {
        walk << std::setw(kPathgenInputMeta[i].header_width)
             << kPathgenInputMeta[i].display_name;
    }
    const std::string expected =
        "  tgX-5  tgX-4  tgX-3  tgX-2  tgX-1   tgX0"
        "  tgY-5  tgY-4  tgY-3  tgY-2  tgY-1   tgY0"
        "  tgZ-5  tgZ-4  tgZ-3  tgZ-2  tgZ-1   tgZ0"
        "   ds-5   ds-4   ds-3   ds-2   ds-1    ds0"
        "  dd/dt"
        "      qw      qx      qy      qz"
        "     vel   gyrP   gyrQ   gyrR";
    EXPECT_EQ(walk.str(), expected);
}

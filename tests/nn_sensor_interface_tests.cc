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
    // 038 FR-P0H (B): 33 → 37 (dist_to_boundary + inward_body xyz appended).
    // 041 US4: 37 → 42 (in_envelope + envelope_secs + accel xyz appended).
    // 041 P2-2: 42 → 45 — a NET +3, not an append: IN_ENVELOPE and
    // ENVELOPE_SECS are REMOVED (ablation: zeroing IN_ENVELOPE *improves*
    // path-5 score by 0.3%, ENVELOPE_SECS costs 0.2%, both inside noise) and
    // SPECIFIC_ENERGY, BOUNDARY_CLOSURE_RATE and SCORE_GRAD_X/Y/Z are added.
    EXPECT_EQ(static_cast<int>(PathgenInput::COUNT), 45);
    EXPECT_EQ(static_cast<int>(PathgenInput::COUNT), NN_INPUT_COUNT);
    EXPECT_EQ(pathgen_meta_size(), 45u);
}

TEST(NNSensorInterface, TrackerInputCountIs66) {
    // 030 M7a Session 2026-05-07 Q1: was 48 with HOME_X/Y/Z/HOME_DIST,
    // then 45 with single DIST_TO_BOUNDARY_ALONG_VEL.
    // 032 phase 1: 45 + 9 derived (span[6] + span_rate + tilt sin/cos) = 54.
    // 038 FR-P0H: 54 + 4 situational-awareness (time_since_seen + inward_body
    // xyz) = 58. (exit_dir sin/cos removed 038 US3 2026-07-05.)
    // 041 US4: 58 + 5 (in_envelope + envelope_secs + accel xyz) = 63.
    // 041 P2-2: 63 → 66, the SAME net +3 as pathgen — which is the point. The
    // craft block is one struct now, so the modes cannot take different slot
    // changes; 46 M2 target representation + 20 CraftCommonInputs = 66.
    // ⚠️ Moves once more to 66 + N after T088 (FR-005a).
    EXPECT_EQ(static_cast<int>(TrackerInput::COUNT), 66);
    EXPECT_EQ(tracker_meta_size(), 66u);
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
    //
    // 041 P2-1/P2-2: 25 M1 target representation + 20 CraftCommonInputs = 45.
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_X_TM5), 0);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_X_NOW), 5);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_Y_TM5), 6);
    EXPECT_EQ(static_cast<int>(PathgenInput::TARGET_Z_TM5), 12);
    EXPECT_EQ(static_cast<int>(PathgenInput::DIST_TM5), 18);
    EXPECT_EQ(static_cast<int>(PathgenInput::CLOSING_RATE), 24);
    // ----- CraftCommonInputs, slots 25..44 -----
    EXPECT_EQ(static_cast<int>(PathgenInput::QUAT_W), 25);
    EXPECT_EQ(static_cast<int>(PathgenInput::AIRSPEED), 29);
    EXPECT_EQ(static_cast<int>(PathgenInput::GYRO_P), 30);
    EXPECT_EQ(static_cast<int>(PathgenInput::GYRO_R), 32);
    EXPECT_EQ(static_cast<int>(PathgenInput::ACCEL_X), 33);
    EXPECT_EQ(static_cast<int>(PathgenInput::ACCEL_Z), 35);
    EXPECT_EQ(static_cast<int>(PathgenInput::SPECIFIC_ENERGY), 36);
    EXPECT_EQ(static_cast<int>(PathgenInput::BOUNDARY_CLOSURE_RATE), 37);
    EXPECT_EQ(static_cast<int>(PathgenInput::DIST_TO_BOUNDARY), 38);
    EXPECT_EQ(static_cast<int>(PathgenInput::INWARD_BODY_X), 39);
    EXPECT_EQ(static_cast<int>(PathgenInput::INWARD_BODY_Z), 41);
    EXPECT_EQ(static_cast<int>(PathgenInput::SCORE_GRAD_X), 42);
    EXPECT_EQ(static_cast<int>(PathgenInput::SCORE_GRAD_Y), 43);
    EXPECT_EQ(static_cast<int>(PathgenInput::SCORE_GRAD_Z), 44);
    EXPECT_EQ(static_cast<int>(PathgenInput::COUNT), 45);
}

TEST(NNSensorInterface, TrackerAnchorPositions) {
    // 041 P2-1: 46 M2 target representation (36 beacon + 9 derived + 1
    // time-since-seen) + 20 CraftCommonInputs = 66.
    //
    // ⚠️ This enum was REORDERED at 041 P2-1. Pre-041 the craft slots were split
    // by the target block (quat at 36, then span/tilt, then inward at 55). A
    // shared sub-struct cannot be non-contiguous, so they are gathered into one
    // tail — and the tail now starts at 46 in both modes' terms.
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_X_TM5), 0);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_L_CEP_NOW), 17);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_X_TM5), 18);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_R_CEP_NOW), 35);
    // 032 phase 1 — derived perceptual features at 36..44
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_TM5), 36);
    EXPECT_EQ(static_cast<int>(TrackerInput::BEACON_PAIR_SPAN_NOW), 41);
    EXPECT_EQ(static_cast<int>(TrackerInput::SPAN_RATE), 42);
    EXPECT_EQ(static_cast<int>(TrackerInput::TARGET_TILT_SIN), 43);
    EXPECT_EQ(static_cast<int>(TrackerInput::TARGET_TILT_COS), 44);
    // 038 FR-P0H (A) — the target-lost cue, tracker-only, so it belongs to the
    // TARGET block and not to the shared one.
    EXPECT_EQ(static_cast<int>(TrackerInput::TIME_SINCE_SEEN), 45);
    // ----- CraftCommonInputs, slots 46..65 -----
    EXPECT_EQ(static_cast<int>(TrackerInput::QUAT_W), 46);
    EXPECT_EQ(static_cast<int>(TrackerInput::AIRSPEED), 50);
    EXPECT_EQ(static_cast<int>(TrackerInput::GYRO_P), 51);
    EXPECT_EQ(static_cast<int>(TrackerInput::ACCEL_X), 54);
    EXPECT_EQ(static_cast<int>(TrackerInput::SPECIFIC_ENERGY), 57);
    EXPECT_EQ(static_cast<int>(TrackerInput::BOUNDARY_CLOSURE_RATE), 58);
    EXPECT_EQ(static_cast<int>(TrackerInput::DIST_TO_BOUNDARY), 59);
    EXPECT_EQ(static_cast<int>(TrackerInput::INWARD_BODY_X), 60);
    EXPECT_EQ(static_cast<int>(TrackerInput::SCORE_GRAD_X), 63);
    EXPECT_EQ(static_cast<int>(TrackerInput::SCORE_GRAD_Z), 65);
    EXPECT_EQ(static_cast<int>(TrackerInput::COUNT), 66);
}

// 041 P2-1 — the assertion the refactor exists to make possible.
//
// The two structs share ONE definition of the craft block, but the two ENUMS
// are still written out twice (C++ enums do not compose). Nothing but this
// test stops the copies drifting: if they do, each mode reads the other's slot
// meanings and every resulting value is entirely plausible.
TEST(NNSensorInterface, CraftCommonBlockIsIdenticalInBothModes) {
    const int pgBase = static_cast<int>(PathgenInput::QUAT_W);
    const int trBase = static_cast<int>(TrackerInput::QUAT_W);

    // The block is the LAST NN_COMMON_INPUT_COUNT slots of each mode.
    EXPECT_EQ(pgBase, static_cast<int>(PathgenInput::COUNT) - NN_COMMON_INPUT_COUNT);
    EXPECT_EQ(trBase, static_cast<int>(TrackerInput::COUNT) - NN_COMMON_INPUT_COUNT);

    // Same names, same short names, same widths, same relative order.
    for (int i = 0; i < NN_COMMON_INPUT_COUNT; ++i) {
        const auto& pg = kPathgenInputMeta[pgBase + i];
        const auto& tr = kTrackerInputMeta[trBase + i];
        EXPECT_STREQ(pg.name, tr.name) << "common slot " << i << " name differs";
        EXPECT_STREQ(pg.display_name, tr.display_name)
            << "common slot " << i << " display_name differs";
        EXPECT_EQ(pg.header_width, tr.header_width)
            << "common slot " << i << " header_width differs";
    }

    // And the struct really is embedded at the documented offset in both.
    EXPECT_EQ(offsetof(NNInputs, common), static_cast<size_t>(pgBase) * sizeof(float));
    EXPECT_EQ(offsetof(TrackerInputs, common), static_cast<size_t>(trBase) * sizeof(float));
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
        // ----- CraftCommonInputs, slots 25..44 (041 P2-1) -----
        // ⚠️ This walk no longer describes a LIVE output format: the per-step
        // data.dat writer was retired at 035 FR-P05 and the dmp is the training
        // trace. The test still earns its keep as a drift guard between the enum
        // and kPathgenInputMeta — a slot added to one and not the other shows up
        // here — but do NOT propagate these columns into
        // specs/019-improved-crrcsim/sim_response.py, which reads HISTORICAL
        // data.dat files frozen at the 021-era layout (see T047).
        "      qw      qx      qy      qz"
        "     vel   gyrP   gyrQ   gyrR"
        "    acX    acY    acZ"
        "      Es    bClR    dBnd"
        "    inX    inY    inZ"
        "    sgX    sgY    sgZ";
    EXPECT_EQ(walk.str(), expected);
}

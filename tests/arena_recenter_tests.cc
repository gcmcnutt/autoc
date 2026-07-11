// 039 T006/T007 — engage-scoped arena re-centering (FR-001, D5).
//
// Pure ±K rule (2026-07-10 simplification — NO min-elevation clamp):
//   K         = (ceiling_agl − floor_agl) / 2 of the TEMPLATE geometry
//   floor_Z   = z_engage + K     (raw NED, down-positive)
//   ceiling_Z = z_engage − K
// Horizontal: the arena cylinder re-centers on the engage x/y.
//
// The xiao re-zeros its virtual frame at engage (test_origin_offset,
// msplink.cpp), so the resolver also produces the FlightArena expressed in
// that engage-zeroed virtual frame for the existing consumers
// (gather_pathgen_inputs / distanceToBoundary / checkArenaBounds, whose AGL
// convention runs through SIM_INITIAL_ALTITUDE). At engage the craft must
// read CENTER of the band both vertically and horizontally
// (contracts/bench-validation.md §FR-002 item 4).
//
// Coordinate conventions per docs/COORDINATE_CONVENTIONS.md (NED: z is
// down-positive; ceiling is more-negative than floor).

#include <gtest/gtest.h>

#include "autoc/eval/arena.h"

namespace {

using autoc::eval::ArenaEgressKind;
using autoc::eval::checkArenaBounds;
using autoc::eval::distanceToBoundary;
using autoc::eval::EngageArena;
using autoc::eval::FlightArena;
using autoc::eval::resolveEngageArena;

// The 80/5/100 m training arena template (autoc.ini M1 / nn2cpp -a default).
FlightArena trainingTemplate() { return FlightArena{80.0f, 5.0f, 100.0f}; }

constexpr gp_scalar kExpectedK = 47.5f;  // (100 − 5) / 2

TEST(ArenaRecenter, DerivesHalfBandFromTemplate) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3::Zero());
    EXPECT_FLOAT_EQ(e.half_band_m, kExpectedK);
    EXPECT_FLOAT_EQ(e.virtual_arena.radius_m, 80.0f);
}

TEST(ArenaRecenter, BenchEngageAtRawZeroGivesPlusMinusK) {
    // Stationary bench: engage at raw NED ≈ (0, 0, 0). Band = ±47.5 centered
    // on the bench; floor is BELOW the craft in NED (+47.5), ceiling above
    // (−47.5). The below-ground floor is the PASS condition on the bench —
    // limits are safety-only this phase.
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(e.floor_z_ned, 47.5f);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -47.5f);
    EXPECT_FLOAT_EQ(e.origin_ned.x(), 0.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.y(), 0.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.z(), 0.0f);
}

TEST(ArenaRecenter, HighEngageShiftsBandWithCraft) {
    // Engage at 80 m altitude (raw NED z = −80): floor −32.5, ceiling −127.5.
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, -80.0f));
    EXPECT_FLOAT_EQ(e.floor_z_ned, -32.5f);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -127.5f);
}

TEST(ArenaRecenter, HorizontalRecenterAtEngageXY) {
    const gp_vec3 engage(12.0f, -7.0f, -80.0f);
    const EngageArena e = resolveEngageArena(trainingTemplate(), engage);
    EXPECT_FLOAT_EQ(e.origin_ned.x(), 12.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.y(), -7.0f);
    // Vertical rule unaffected by the horizontal offset.
    EXPECT_FLOAT_EQ(e.floor_z_ned, -32.5f);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -127.5f);
}

// --- Virtual-frame arena: what gather/checkArenaBounds actually consume ---

TEST(ArenaRecenter, VirtualArenaCentersBandOnEngage) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(3.0f, 4.0f, -60.0f));

    // In the engage-zeroed virtual frame the craft sits at (0,0,0); its AGL
    // per the arena convention is −SIM_INITIAL_ALTITUDE. Band must be ±K
    // around exactly that (independent of raw engage altitude).
    const gp_scalar engage_agl = -SIM_INITIAL_ALTITUDE;
    EXPECT_FLOAT_EQ(e.virtual_arena.floor_agl_m, engage_agl - kExpectedK);
    EXPECT_FLOAT_EQ(e.virtual_arena.ceiling_agl_m, engage_agl + kExpectedK);

    // Craft at the virtual origin is inside the arena (band center).
    AircraftState st;
    st.setPosition(gp_vec3::Zero());
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::NONE);
}

TEST(ArenaRecenter, VirtualArenaSymmetricVerticalMargins) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, -10.0f));

    // Ray straight up and straight down from the engage point both hit a
    // bound at exactly K meters — center-of-band reading (FR-002 item 4).
    const gp_vec3 origin = gp_vec3::Zero();  // virtual frame
    const gp_vec3 up(0.0f, 0.0f, -1.0f);     // NED: up = −z
    const gp_vec3 down(0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(distanceToBoundary(origin, up, e.virtual_arena), kExpectedK, 1e-3f);
    EXPECT_NEAR(distanceToBoundary(origin, down, e.virtual_arena), kExpectedK, 1e-3f);

    // Horizontal ray from center: the radius (80 m) away in every direction.
    EXPECT_NEAR(distanceToBoundary(origin, gp_vec3(1.0f, 0.0f, 0.0f), e.virtual_arena),
                80.0f, 1e-3f);
    EXPECT_NEAR(distanceToBoundary(origin, gp_vec3(0.0f, -1.0f, 0.0f), e.virtual_arena),
                80.0f, 1e-3f);
}

TEST(ArenaRecenter, VirtualArenaEgressJustBeyondBand) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, 0.0f));

    AircraftState st;
    st.setPosition(gp_vec3(0.0f, 0.0f, kExpectedK + 0.5f));  // 0.5 m below the floor
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::FLOOR);

    st.setPosition(gp_vec3(0.0f, 0.0f, -(kExpectedK + 0.5f)));  // above the ceiling
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::CEILING);

    st.setPosition(gp_vec3(80.5f, 0.0f, 0.0f));  // outside the cylinder
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::RADIUS);
}

}  // namespace

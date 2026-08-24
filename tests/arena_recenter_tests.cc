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

// 041 P2-3 — the REAL training template, not a restated literal. Deriving it
// from the shipped defaults means this suite cannot drift from the arena the
// runs actually use, which a hard-coded 80/5/100 (its previous value) had
// already done twice over.
FlightArena trainingTemplate() { return FlightArena{}; }

// The band is ASYMMETRIC about the arm point as of 041 P2-3: the rabbit climbs
// 34.98 m above the origin and descends 2.74 m below it, so a symmetric band
// spent half its height on airspace nothing enters. Operator 2026-08-18:
// *"60m up and 10 down"*.
// ⚠️ Sized to the CHASE, not the rabbit. At +60/−10 the t2 smoke died on the
// deck 16 times out of 16 (terminal AGL 25.01–25.51 against a 25 m deck), mean
// survival 4.9 s, elite frozen 11 generations — while the ceiling was never
// approached. +50/−30 covers the chase's measured 17.9 m descent.
constexpr gp_scalar kExpectedUp = 50.0f;
constexpr gp_scalar kExpectedDown = 30.0f;

TEST(ArenaRecenter, DerivesAsymmetricExtentsFromTemplate) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3::Zero());
    EXPECT_FLOAT_EQ(e.up_m, kExpectedUp);
    EXPECT_FLOAT_EQ(e.down_m, kExpectedDown);
    EXPECT_FLOAT_EQ(e.virtual_arena.radius_m, trainingTemplate().radius_m);
}

// ⛔ THE ONE THIS SUITE EXISTS FOR. `resolveEngageArena` used to compute a single
// half-band K = (ceiling − floor)/2 and place the band at ±K, which is correct
// ONLY when the arm point is the vertical centre of the template. With +60/−10
// that would have handed the aircraft ±35 m: 25 m LESS room above than it
// trained with and 25 m MORE below, in flight, with every number looking
// plausible. The asymmetry is what makes this test load-bearing rather than
// arithmetic.
TEST(ArenaRecenter, PlacementIsNotHalfBandSymmetric) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3::Zero());
    const gp_scalar half_band =
        (trainingTemplate().ceiling_agl_m - trainingTemplate().floor_agl_m) / 2.0f;
    EXPECT_NE(e.up_m, half_band)
        << "the template is symmetric, so this suite is no longer testing what "
           "it was written to test — either restore the asymmetry or delete this";
    EXPECT_GT(e.up_m, e.down_m);
}

TEST(ArenaRecenter, BenchEngageAtRawZeroPlacesTheAsymmetricBand) {
    // Stationary bench: engage at raw NED ≈ (0, 0, 0). Floor is BELOW the craft
    // in NED (+down), ceiling above (−up). The below-ground floor is the PASS
    // condition on the bench — limits are safety-only this phase.
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(e.floor_z_ned, kExpectedDown);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -kExpectedUp);
    EXPECT_FLOAT_EQ(e.origin_ned.x(), 0.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.y(), 0.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.z(), 0.0f);
}

TEST(ArenaRecenter, HighEngageShiftsBandWithCraft) {
    // Engage at 80 m altitude (raw NED z = −80).
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, -80.0f));
    EXPECT_FLOAT_EQ(e.floor_z_ned, -80.0f + kExpectedDown);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -80.0f - kExpectedUp);
}

TEST(ArenaRecenter, HorizontalRecenterAtEngageXY) {
    const gp_vec3 engage(12.0f, -7.0f, -80.0f);
    const EngageArena e = resolveEngageArena(trainingTemplate(), engage);
    EXPECT_FLOAT_EQ(e.origin_ned.x(), 12.0f);
    EXPECT_FLOAT_EQ(e.origin_ned.y(), -7.0f);
    // Vertical rule unaffected by the horizontal offset.
    EXPECT_FLOAT_EQ(e.floor_z_ned, -80.0f + kExpectedDown);
    EXPECT_FLOAT_EQ(e.ceiling_z_ned, -80.0f - kExpectedUp);
}

// --- Virtual-frame arena: what gather/checkArenaBounds actually consume ---

// ⭐ The identity. Whatever altitude the aircraft armed at, the band it flies is
// the band it trained in — same struct, not merely the same size.
TEST(ArenaRecenter, VirtualArenaIsAnExactIdentityOnTheTemplate) {
    const FlightArena tmpl = trainingTemplate();
    for (gp_scalar z : {0.0f, -10.0f, -60.0f, -250.0f}) {
        const EngageArena e = resolveEngageArena(tmpl, gp_vec3(3.0f, 4.0f, z));
        EXPECT_FLOAT_EQ(e.virtual_arena.radius_m, tmpl.radius_m) << "engage z=" << z;
        EXPECT_FLOAT_EQ(e.virtual_arena.floor_agl_m, tmpl.floor_agl_m) << "engage z=" << z;
        EXPECT_FLOAT_EQ(e.virtual_arena.ceiling_agl_m, tmpl.ceiling_agl_m) << "engage z=" << z;

        // Craft at the virtual origin is inside the arena, by construction.
        AircraftState st;
        st.setPosition(gp_vec3::Zero());
        EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::NONE);
    }
}

TEST(ArenaRecenter, VirtualArenaVerticalMarginsMatchTheExtents) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, -10.0f));

    // Rays straight up and straight down from the arm point hit their bounds at
    // the respective extents — NOT at a common K.
    const gp_vec3 origin = gp_vec3::Zero();  // virtual frame
    const gp_vec3 up(0.0f, 0.0f, -1.0f);     // NED: up = −z
    const gp_vec3 down(0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(distanceToBoundary(origin, up, e.virtual_arena), kExpectedUp, 1e-3f);
    EXPECT_NEAR(distanceToBoundary(origin, down, e.virtual_arena), kExpectedDown, 1e-3f);

    // Horizontal ray from the axis: the radius away in every direction.
    const gp_scalar R = trainingTemplate().radius_m;
    EXPECT_NEAR(distanceToBoundary(origin, gp_vec3(1.0f, 0.0f, 0.0f), e.virtual_arena), R, 1e-3f);
    EXPECT_NEAR(distanceToBoundary(origin, gp_vec3(0.0f, -1.0f, 0.0f), e.virtual_arena), R, 1e-3f);
}

TEST(ArenaRecenter, VirtualArenaEgressJustBeyondBand) {
    const EngageArena e = resolveEngageArena(trainingTemplate(), gp_vec3(0.0f, 0.0f, 0.0f));
    const gp_scalar R = trainingTemplate().radius_m;

    AircraftState st;
    st.setPosition(gp_vec3(0.0f, 0.0f, kExpectedDown + 0.5f));  // 0.5 m below the deck
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::FLOOR);

    st.setPosition(gp_vec3(0.0f, 0.0f, -(kExpectedUp + 0.5f)));  // above the ceiling
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::CEILING);

    st.setPosition(gp_vec3(R + 0.5f, 0.0f, 0.0f));  // outside the cylinder
    EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::RADIUS);
}

}  // namespace

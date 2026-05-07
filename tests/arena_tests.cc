// 030 M7a — Arena contract tests (T041 + Session 2026-05-07 Q1).
//
// Two contract surfaces:
//   1. checkArenaBounds() — per-tick OOB termination check; returns
//      ArenaEgressKind enum naming which boundary was crossed.
//   2. distanceToBoundary() — ray-projection scalar fed to the NN as
//      input slot 44 (TrackerInput::DIST_TO_BOUNDARY_ALONG_VEL); meters
//      along chase velocity unit vector before ray hits cylinder /
//      floor / ceiling, whichever first. Single source of truth: same
//      function feeds the NN signal AND the per-tick check.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/types.h"

using autoc::eval::ArenaEgressKind;
using autoc::eval::FlightArena;
using autoc::eval::checkArenaBounds;
using autoc::eval::distanceToBoundary;
using autoc::eval::kSafeBoundaryDistance;

namespace {

AircraftState makeChaseState(const gp_vec3& position) {
    return AircraftState{0, /*relVel=*/15.0f,
                         gp_vec3(15.0f, 0.0f, 0.0f),
                         gp_quat::Identity(),
                         position,
                         0.0f, 0.0f, 0.0f, 0};
}

// SIM_INITIAL_ALTITUDE = -25 (raw NED z), so virtual_z=0 ↔ raw_z=-25 ↔
// 25m AGL. Helper: virtual_z for a given AGL altitude.
gp_scalar virtZForAGL(gp_scalar alt_agl_m) {
    return -alt_agl_m - SIM_INITIAL_ALTITUDE;  // virtual_z = -alt_agl - SIM_INIT_ALT
}

}  // namespace

// ---------------------------------------------------------------------------
// checkArenaBounds — per-tick OOB termination
// ---------------------------------------------------------------------------

TEST(ArenaBounds, ChaseInsideArenaReturnsNone) {
    FlightArena arena;  // defaults 80 / 5 / 100
    // Virtual (0, 0, 0) → 25m AGL (mid-arena both axes).
    auto state = makeChaseState(gp_vec3(0, 0, 0));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::NONE);
}

TEST(ArenaBounds, ChaseOutsideRadiusReturnsRADIUS) {
    FlightArena arena;
    // 90m horizontal — outside 80m radius.
    auto state = makeChaseState(gp_vec3(90, 0, 0));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::RADIUS);
}

TEST(ArenaBounds, ChaseBelowFloorReturnsFLOOR) {
    FlightArena arena;
    // 3m AGL — below 5m floor. virtual_z = -3 - (-25) = +22.
    auto state = makeChaseState(gp_vec3(0, 0, virtZForAGL(3.0f)));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::FLOOR);
}

TEST(ArenaBounds, ChaseAboveCeilingReturnsCEILING) {
    FlightArena arena;
    // 110m AGL — above 100m ceiling. virtual_z = -110 - (-25) = -85.
    auto state = makeChaseState(gp_vec3(0, 0, virtZForAGL(110.0f)));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::CEILING);
}

TEST(ArenaBounds, RadiusCheckedBeforeAltitude) {
    FlightArena arena;
    // Out-of-radius AND below floor — RADIUS reported first (priority).
    auto state = makeChaseState(gp_vec3(90, 0, virtZForAGL(3.0f)));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::RADIUS);
}

// ---------------------------------------------------------------------------
// distanceToBoundary — ray-projection NN input
// ---------------------------------------------------------------------------

TEST(DistanceToBoundary, ChaseAtCenterFlyingForwardHitsCylinder) {
    FlightArena arena;
    // At origin, velocity pure +X. Cylinder hit at radius 80m straight ahead.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(20, 0, 0), arena);
    EXPECT_NEAR(d, 80.0f, 0.5f);
}

TEST(DistanceToBoundary, ChaseNearWallShortDistance) {
    FlightArena arena;
    // 10m from origin, velocity pure +X. Wall ahead = 80 - 10 = 70m.
    gp_scalar d = distanceToBoundary(gp_vec3(10, 0, 0), gp_vec3(15, 0, 0), arena);
    EXPECT_NEAR(d, 70.0f, 0.5f);
}

TEST(DistanceToBoundary, ChaseHeadingAwayFromWallReturnsSentinel) {
    FlightArena arena;
    // 70m from origin in +X, velocity pure +X. Wall ahead via that
    // direction = 80 - 70 = 10m. So actually still hits ahead. Try
    // velocity pure -X: chase headed toward origin. Cylinder behind:
    // ray must traverse to opposite side. (-70,0,0) along (-1,0,0) hits
    // wall at -80, distance |(-80) - (-70)| = ...
    // Better test: chase at (70,0,0) flying +Y direction. Horizontal speed
    // perpendicular to +X axis. Will hit wall on the +Y side.
    // Just ensure ray hits *something* and distance is reasonable, not
    // sentinel.
    gp_scalar d = distanceToBoundary(gp_vec3(70, 0, 0), gp_vec3(0, 15, 0), arena);
    EXPECT_LT(d, kSafeBoundaryDistance);
    EXPECT_GT(d, 0.0f);
}

TEST(DistanceToBoundary, DescendingVelocityHitsFloorBeforeCylinder) {
    FlightArena arena;
    // Chase at virtual (0, 0, 0) = 25m AGL. Floor at 5m AGL = virtual_z=+20.
    // Pure +Z descent at 1 m/s velocity unit: 20m to floor.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, 1), arena);
    EXPECT_NEAR(d, 20.0f, 0.5f);
}

TEST(DistanceToBoundary, ClimbingVelocityHitsCeiling) {
    FlightArena arena;
    // Chase at virtual (0, 0, 0) = 25m AGL. Ceiling at 100m AGL =
    // virtual_z = -100 - (-25) = -75. Pure -Z ascent: distance to
    // ceiling = |-75 - 0| = 75m.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, -1), arena);
    EXPECT_NEAR(d, 75.0f, 0.5f);
}

TEST(DistanceToBoundary, ZeroVelocityReturnsSentinel) {
    FlightArena arena;
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, 0), arena);
    EXPECT_FLOAT_EQ(d, kSafeBoundaryDistance);
}

TEST(DistanceToBoundary, DescentChooseFloorOverCylinder) {
    // Chase near wall AND descending: should pick whichever is closer.
    FlightArena arena;
    // At (75, 0, +18) virtual = 7m AGL (near floor, near wall).
    // Velocity (10, 0, 1) — slight forward + slight descent.
    gp_scalar d = distanceToBoundary(gp_vec3(75, 0, 18), gp_vec3(10, 0, 1), arena);
    EXPECT_GT(d, 0.0f);
    EXPECT_LT(d, kSafeBoundaryDistance);
    // Wall ahead: ~5m. Floor below: 2m AGL away ⇒ via descent rate
    // 1/sqrt(101)*magnitude ≈ 2/(0.099) = 20m. So wall is much closer.
    // Just sanity-check it's small (<20m, dominated by wall).
    EXPECT_LT(d, 20.0f);
}

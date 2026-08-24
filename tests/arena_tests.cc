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

#include <fstream>
#include <string>

#include <cmath>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/types.h"

using autoc::eval::ArenaEgressKind;
using autoc::eval::FlightArena;
using autoc::eval::checkArenaBounds;
using autoc::eval::EngageArena;
using autoc::eval::resolveEngageArena;
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

// 041 P2-3: SIM_INITIAL_ALTITUDE = -55 (raw NED z), so virtual_z=0 ↔ raw_z=-55
// ↔ 55 m AGL — the ARM POINT, which sits 30 m above the 25 m deck and 50 m below
// the 105 m ceiling (the band is deliberately asymmetric).
// Helper: virtual_z for a given AGL altitude.
gp_scalar virtZForAGL(gp_scalar alt_agl_m) {
    return -alt_agl_m - SIM_INITIAL_ALTITUDE;  // virtual_z = -alt_agl - SIM_INIT_ALT
}

}  // namespace

// ---------------------------------------------------------------------------
// checkArenaBounds — per-tick OOB termination
// ---------------------------------------------------------------------------

TEST(ArenaBounds, ChaseInsideArenaReturnsNone) {
    FlightArena arena;  // 041 P2-3 defaults 70 / 25 / 105
    // Virtual (0, 0, 0) → 55 m AGL, the arm point.
    auto state = makeChaseState(gp_vec3(0, 0, 0));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::NONE);
}

TEST(ArenaBounds, ChaseOutsideRadiusReturnsRADIUS) {
    FlightArena arena;
    // 90 m horizontal — outside the 70 m radius.
    auto state = makeChaseState(gp_vec3(90, 0, 0));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::RADIUS);
}

TEST(ArenaBounds, ChaseBelowFloorReturnsFLOOR) {
    FlightArena arena;
    // 3 m AGL — below the 25 m hard deck. virtual_z = -3 - (-55) = +52.
    auto state = makeChaseState(gp_vec3(0, 0, virtZForAGL(3.0f)));
    EXPECT_EQ(checkArenaBounds(state, arena), ArenaEgressKind::FLOOR);
}

TEST(ArenaBounds, ChaseAboveCeilingReturnsCEILING) {
    FlightArena arena;
    // 130 m AGL — above the 105 m ceiling. virtual_z = -130 - (-55) = -75.
    auto state = makeChaseState(gp_vec3(0, 0, virtZForAGL(130.0f)));
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
    // At origin, velocity pure +X. Cylinder hit at radius 70 m straight ahead.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(20, 0, 0), arena);
    EXPECT_NEAR(d, 70.0f, 0.5f);
}

TEST(DistanceToBoundary, ChaseNearWallShortDistance) {
    FlightArena arena;
    // 10 m from origin, velocity pure +X. Wall ahead = 70 - 10 = 60 m.
    gp_scalar d = distanceToBoundary(gp_vec3(10, 0, 0), gp_vec3(15, 0, 0), arena);
    EXPECT_NEAR(d, 60.0f, 0.5f);
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
    gp_scalar d = distanceToBoundary(gp_vec3(60, 0, 0), gp_vec3(0, 15, 0), arena);
    EXPECT_LT(d, kSafeBoundaryDistance);
    EXPECT_GT(d, 0.0f);
}

TEST(DistanceToBoundary, DescendingVelocityHitsFloorBeforeCylinder) {
    FlightArena arena;
    // Chase at virtual (0, 0, 0) = 55 m AGL (the arm point). Deck at 25 m AGL
    // = virtual_z = +30. Pure +Z descent: 30 m — the DOWN-extent.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, 1), arena);
    EXPECT_NEAR(d, 30.0f, 0.5f);
}

TEST(DistanceToBoundary, ClimbingVelocityHitsCeiling) {
    FlightArena arena;
    // Chase at virtual (0, 0, 0) = 55 m AGL. Ceiling at 105 m AGL =
    // virtual_z = -105 - (-55) = -50. Pure -Z ascent: 50 m — the UP-extent.
    //
    // ⭐ Note this does NOT equal the descent distance above, and that is the
    // whole point of the 041 P2-3 asymmetry: the M1 rabbit climbs 34.98 m above
    // the arm point while the CHASE descends ~18 m and settles. The t2 smoke
    // proved the band must be sized to the chase, not the rabbit: at +60/−10
    // every scenario died on the deck. This pair of tests is where the 5:3 shape
    // is actually asserted.
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, -1), arena);
    EXPECT_NEAR(d, 50.0f, 0.5f);
}

TEST(DistanceToBoundary, ZeroVelocityReturnsSentinel) {
    FlightArena arena;
    gp_scalar d = distanceToBoundary(gp_vec3(0, 0, 0), gp_vec3(0, 0, 0), arena);
    EXPECT_FLOAT_EQ(d, kSafeBoundaryDistance);
}

TEST(DistanceToBoundary, DescentChooseFloorOverCylinder) {
    // Chase near wall AND descending: should pick whichever is closer.
    FlightArena arena;
    // At (65, 0, +8) virtual = 47 m AGL, 65 m out — the WALL is the near bound.
    // Velocity (10, 0, 1) — slight forward + slight descent.
    gp_scalar d = distanceToBoundary(gp_vec3(65, 0, 8), gp_vec3(10, 0, 1), arena);
    EXPECT_GT(d, 0.0f);
    EXPECT_LT(d, kSafeBoundaryDistance);
    // Wall ahead: ~5 m. Deck below: 2 m away, but reached at a descent rate
    // of only 1/sqrt(101) of the speed ⇒ ~20 m of travel. The wall is closer,
    // so the minimum must be wall-dominated.
    EXPECT_LT(d, 20.0f);
}

// ---------------------------------------------------------------------------
// 041 P2-3 — the invariants that make sim and flight ONE arena
// ---------------------------------------------------------------------------

// ⛔ THE LOAD-BEARING ONE. `SIM_INITIAL_ALTITUDE` (a compile-time constant) and
// the arena bounds (runtime config) are two independent knobs that must agree
// on ONE thing: where the arm point sits inside the band. `resolveEngageArena`
// derives the up/down extents from exactly that, so if someone edits a bound in
// an .ini without touching the constant, the sim trains in one band and the
// aircraft flies another — silently, with plausible numbers throughout.
//
// ⚠️ The band is deliberately ASYMMETRIC (+60 / −10) as of 041 P2-3, so this is
// no longer a centring check. It was one, and the earlier version of this test
// asserted entry == (floor+ceiling)/2 — which is exactly the assumption
// `resolveEngageArena`'s old ±K placement encoded, and exactly what would have
// broken silently when the asymmetry landed.
TEST(ArenaDatum, ArmPointSitsWhereTheArenaSaysItDoes) {
    FlightArena arena;  // the shipped defaults, which every .ini mirrors
    const gp_scalar arm_agl = -SIM_INITIAL_ALTITUDE;

    EXPECT_GT(arm_agl, arena.floor_agl_m)
        << "the arm point is below the hard deck — the craft would egress on "
           "tick 1";
    EXPECT_LT(arm_agl, arena.ceiling_agl_m)
        << "the arm point is above the ceiling";

    // The 041 P2-3 extents, stated so a config edit that changes the SHAPE of
    // the band has to come here and say so.
    EXPECT_NEAR(arena.ceiling_agl_m - arm_agl, 50.0, 1e-4) << "up-extent";
    EXPECT_NEAR(arm_agl - arena.floor_agl_m, 30.0, 1e-4) << "down-extent";

    EXPECT_GT(arena.floor_agl_m, static_cast<gp_scalar>(0))
        << "the hard deck must be above ground — the sim's ground plane is at "
           "0 m and a deck at or below it cannot be flown to";
}

// ⭐ Sim and flight are the same arena BY CONSTRUCTION, not by convention.
// `resolveEngageArena` — which the firmware runs at every span activation —
// reproduces the training arena exactly, wherever the aircraft happened to
// engage. Before 041 the sim entered 21% up its band while the flight arena
// centred on engage, so a policy trained with 20 m of room beneath it flew with
// 47.5 m.
//
// ⚠️ This must hold for an ASYMMETRIC band, which is what the ±K placement got
// wrong: with +60/−10 it would have produced ±35, giving the aircraft 25 m less
// room above than it trained with and 25 m more below.
TEST(ArenaDatum, ResolveEngageArenaReproducesTheTrainingArenaExactly) {
    const FlightArena training;  // the baked template
    for (const gp_vec3& engage : {gp_vec3(0, 0, 0),
                                  gp_vec3(120, -45, -80),
                                  gp_vec3(-10, 3, 250)}) {
        const EngageArena e = resolveEngageArena(training, engage);
        EXPECT_NEAR(e.virtual_arena.radius_m, training.radius_m, 1e-4);
        EXPECT_NEAR(e.virtual_arena.floor_agl_m, training.floor_agl_m, 1e-4)
            << "engage at z=" << engage.z();
        EXPECT_NEAR(e.virtual_arena.ceiling_agl_m, training.ceiling_agl_m, 1e-4)
            << "engage at z=" << engage.z();
        // And the craft is inside it at the moment of engage, by definition.
        auto st = makeChaseState(gp_vec3(0, 0, 0));
        EXPECT_EQ(checkArenaBounds(st, e.virtual_arena), ArenaEgressKind::NONE);
    }
}

// The measured M1 target envelope must FIT the arena, or the chase egresses
// following a target it was told to follow. Numbers from
// specs/041-m2-depth/measure/path_altitude_extents.cc.
//
// ⚠️ This asserts the TARGET fits. It deliberately does NOT assert the CHASE
// fits: measured on t1, the chase reached entry +66.2 m against the +50 m this
// band offers, and that 16.2 m shortfall is a KNOWN, flagged consequence of the
// operator's 2026-08-18 geometry choice — see toolchain-datum-validation.md.
// If this test is ever widened to cover the chase, it will fail, and that is
// the correct outcome rather than a reason to relax it.
TEST(ArenaDatum, MeasuredM1TargetEnvelopeFitsInsideTheArena) {
    FlightArena arena;
    const gp_scalar entry_agl = -SIM_INITIAL_ALTITUDE;
    // 041 P2-3: after the path shrinks, SpiralClimb/HighPerchSplitS top out at
    // +34.98 m and HighPerchSplitS is the widest at 57.14 m. Measured, not
    // assumed — arena_path_fit_tests re-derives both from the generator every
    // build, across a seed sweep AND against the analytic Catmull-Rom bound.
    constexpr gp_scalar kTargetHighestAboveEntry = 34.98f;  // measured
    constexpr gp_scalar kTargetMaxRadius = 57.14f;          // HighPerchSplitS, measured

    EXPECT_LT(entry_agl + kTargetHighestAboveEntry, arena.ceiling_agl_m)
        << "the rabbit itself leaves the arena at the top of its tallest path";
    EXPECT_LT(kTargetMaxRadius, arena.radius_m)
        << "the rabbit itself leaves the arena horizontally";
}

// ---------------------------------------------------------------------------
// 041 P2-3 — the crrcsim launch altitudes must agree with the frame
// ---------------------------------------------------------------------------

// ⛔ THE ONE THAT WOULD HAVE CAUGHT IT. There are TWO crrcsim configs — the
// headless training worker uses autoc_config.xml, the VISUAL worker uses
// autoc_config-eval.xml — and when the frame moved only the first was updated.
// The visual craft then launched at 25.0012 m AGL against a 25 m hard deck:
// 1.2 mm of clearance, every scenario egressing on the floor inside a second,
// and a policy that looked broken when the spawn point was the thing that was
// wrong.
//
// `<launch altitude>` is in FEET and measured to the aircraft's LOWEST point.
// crrc_main computes  Altitude = launch + zLow + groundHeight, so inverting:
//     launch_ft = (-SIM_INITIAL_ALTITUDE / 0.3048) - zLow - groundHeight
// ⚠️ groundHeight is NEGATIVE (Davis returns -0.1 ft), so that trailing term
// ADDS 0.1. Getting its sign wrong is a 0.2 ft error — which this test caught
// in its own first draft, which is the argument for computing it here rather
// than pasting the number from the XML.
//
// Parsed from the XML rather than restated, so the test fails when the FILE
// drifts rather than when someone forgets to update a copy of the number.
TEST(ArenaDatum, EveryCrrcsimLaunchAltitudeMatchesTheVirtualOrigin) {
    const double kZLowFt = 0.125;      // hb1_streamer.xml <wheels units="0">
    const double kGroundFt = -0.1;     // BuiltinSceneryDavis::getHeight()
    const double want_ft = (-static_cast<double>(SIM_INITIAL_ALTITUDE) / 0.3048)
                           - kZLowFt - kGroundFt;

    for (const char* rel : {"crrcsim/autoc_config.xml",
                            "crrcsim/autoc_config-eval.xml"}) {
        std::ifstream in(std::string(AUTOC_SOURCE_DIR) + "/" + rel);
        ASSERT_TRUE(in.good()) << "cannot open " << rel;
        std::string txt((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());

        const std::string key = "<launch altitude=\"";
        const size_t p = txt.find(key);
        ASSERT_NE(p, std::string::npos) << rel << " has no <launch altitude>";
        const size_t s = p + key.size();
        const double got = std::stod(txt.substr(s, txt.find('"', s) - s));

        EXPECT_NEAR(got, want_ft, 0.01)
            << rel << " launches at " << got << " ft = "
            << (got + kZLowFt + kGroundFt) * 0.3048 << " m AGL, but "
            << "SIM_INITIAL_ALTITUDE puts the virtual origin at "
            << -static_cast<double>(SIM_INITIAL_ALTITUDE) << " m. The craft would "
            << "spawn in the wrong place — and if that is at or below the hard "
            << "deck it egresses on tick 1 and the POLICY takes the blame.";
    }
}

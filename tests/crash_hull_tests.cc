// 030 M7c — Crash hull contract tests (T040 + FR-008b + R3).
//
// Three contract surfaces:
//   1. isInsideHull — sphere intersection at boundary tangent / inside /
//      outside.
//   2. didCrashFire — gating: outside hull never fires; inside + p=0
//      never fires; inside + p=1 always fires; inside + 0 < p < 1 fires
//      probabilistically (deterministic per seed).
//   3. pCrashForGen — curriculum-anneal schedule: ramp gen 0 → plateau
//      across plateau_gen (R3 v1 default 0 → 0.30 across gens 0..200).

#include <gtest/gtest.h>

#include <cstdint>

#include "autoc/eval/crash_hull.h"

using autoc::eval::CrashHull;
using autoc::eval::CrashHullShape;
using autoc::eval::didCrashFire;
using autoc::eval::isInsideHull;

// ---------------------------------------------------------------------------
// isInsideHull — sphere intersection identity
// ---------------------------------------------------------------------------

TEST(CrashHullInside, ChaseAtTargetIsInside) {
    CrashHull hull;  // SPHERE, radius 1.0
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0)));
}

TEST(CrashHullInside, ChaseAtBoundaryTangentIsInside) {
    // |chase - target| == radius — boundary inclusive (≤ in
    // implementation), so reports inside.
    CrashHull hull;
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(1, 0, 0), gp_vec3(0, 0, 0)));
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(0, 0, 0), gp_vec3(0, 1, 0)));
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, -1)));
}

TEST(CrashHullInside, ChaseJustOutsideRadiusIsOutside) {
    CrashHull hull;
    EXPECT_FALSE(isInsideHull(hull, gp_vec3(1.001f, 0, 0), gp_vec3(0, 0, 0)));
}

TEST(CrashHullInside, ChaseFarFromTargetIsOutside) {
    CrashHull hull;
    EXPECT_FALSE(isInsideHull(hull, gp_vec3(50, 0, 0), gp_vec3(0, 0, 0)));
}

TEST(CrashHullInside, RadiusConfigurable) {
    CrashHull hull;
    hull.sphere_radius_m = 5.0f;
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(4, 0, 0), gp_vec3(0, 0, 0)));
    EXPECT_TRUE(isInsideHull(hull, gp_vec3(5, 0, 0), gp_vec3(0, 0, 0)));  // tangent
    EXPECT_FALSE(isInsideHull(hull, gp_vec3(5.001f, 0, 0), gp_vec3(0, 0, 0)));
}

TEST(CrashHullInside, ReservedShapesReturnFalse) {
    // AABB_HB1 / MESH_AIRFRAME reserved enums in v1 — no crash
    // until they're implemented.
    CrashHull hull;
    hull.shape = CrashHullShape::AABB_HB1;
    EXPECT_FALSE(isInsideHull(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0)));

    hull.shape = CrashHullShape::MESH_AIRFRAME;
    EXPECT_FALSE(isInsideHull(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0)));
}

// ---------------------------------------------------------------------------
// didCrashFire — probabilistic gating
// ---------------------------------------------------------------------------

TEST(CrashHullFire, OutsideHullNeverFires) {
    CrashHull hull;
    uint32_t prng = 12345;
    // Even with p_crash = 1.0, outside-hull means no crash.
    EXPECT_FALSE(didCrashFire(hull, gp_vec3(50, 0, 0), gp_vec3(0, 0, 0),
                               1.0f, prng));
    // PRNG state should be unchanged (we don't burn a draw when geometric
    // gate fails).
    EXPECT_EQ(prng, 12345u);
}

TEST(CrashHullFire, InsideWithZeroProbabilityNeverFires) {
    CrashHull hull;
    uint32_t prng = 12345;
    EXPECT_FALSE(didCrashFire(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0),
                               0.0f, prng));
}

TEST(CrashHullFire, InsideWithFullProbabilityAlwaysFires) {
    CrashHull hull;
    uint32_t prng = 12345;
    EXPECT_TRUE(didCrashFire(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0),
                              1.0f, prng));
}

TEST(CrashHullFire, InsideWithPartialProbabilityIsDeterministicPerSeed) {
    CrashHull hull;
    // Same seed → same draw → same fire outcome across two invocations.
    uint32_t prng_a = 42;
    uint32_t prng_b = 42;
    bool fired_a = didCrashFire(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0),
                                  0.5f, prng_a);
    bool fired_b = didCrashFire(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0),
                                  0.5f, prng_b);
    EXPECT_EQ(fired_a, fired_b);
    EXPECT_EQ(prng_a, prng_b);  // PRNG state advanced identically
}

TEST(CrashHullFire, PartialProbabilityBernoulliRateMatches) {
    // 1000 draws at p=0.30 should give roughly 30% fire rate (allow
    // wide tolerance for a small sample).
    CrashHull hull;
    uint32_t prng = 0xCAFEBABE;
    int fired_count = 0;
    constexpr int N = 1000;
    for (int i = 0; i < N; ++i) {
        if (didCrashFire(hull, gp_vec3(0, 0, 0), gp_vec3(0, 0, 0),
                          0.30f, prng)) {
            ++fired_count;
        }
    }
    // Expected ~300 ± 50 (≈ 1.5σ). Coarse but catches obvious bugs.
    EXPECT_GT(fired_count, 200);
    EXPECT_LT(fired_count, 400);
}

// 030 M11.preA.3 (2026-05-10): pCrashForGen() removed in favor of a single
// fixed CrashHullProbability config knob (set in autoc.cc → EvalData →
// didCrashFire). The curriculum ramp tests below are intentionally gone;
// didCrashFire's probability handling is already covered by the
// CrashHullDidFire tests above (uses the constant p_crash_this_tick).

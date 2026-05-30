// 032 PHASE 1 — Pure-math tests for derived perceptual features.
//
// Tests autoc::eval::compute_pair_span and autoc::eval::compute_tilt in
// isolation (no AircraftState / FlightArena dependency). These are the
// primitives used by gather_tracker_inputs (src/nn/evaluator.cc) and
// projectAndShiftHistory (src/eval/tracker_stepper.cc + crrcsim mirror).
//
// CEP-gating is tested at the integration layer (gather_tracker_inputs_tests).
// Here we verify only that the math is correct given valid NDC inputs.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/derived_features.h"
#include "autoc/types.h"

using autoc::eval::compute_pair_span;
using autoc::eval::compute_tilt;
using autoc::eval::kTiltDegenerateEpsilon;
using autoc::eval::TiltSinCos;

namespace {

constexpr gp_scalar kEps = static_cast<gp_scalar>(1e-5);

}  // namespace

// ============================================================================
// span
// ============================================================================

TEST(DerivedFeaturesSpan, IdenticalPointsHaveZeroSpan) {
    const gp_scalar s = compute_pair_span(0.1, 0.2, 0.1, 0.2);
    EXPECT_NEAR(s, 0.0, kEps);
}

TEST(DerivedFeaturesSpan, UnitHorizontalSeparation) {
    // Port at (-0.5, 0), starboard at (+0.5, 0) → span = 1.0
    const gp_scalar s = compute_pair_span(-0.5, 0.0, 0.5, 0.0);
    EXPECT_NEAR(s, 1.0, kEps);
}

TEST(DerivedFeaturesSpan, DiagonalPair) {
    // Port at (0, 0), starboard at (3, 4) → span = 5 (3-4-5 triangle)
    const gp_scalar s = compute_pair_span(0.0, 0.0, 3.0, 4.0);
    EXPECT_NEAR(s, 5.0, kEps);
}

TEST(DerivedFeaturesSpan, NDCFullScreenMax) {
    // Largest geometrically possible NDC distance: opposite corners
    // of the [-1, +1]^2 NDC box → distance = 2√2 ≈ 2.828
    const gp_scalar s = compute_pair_span(-1.0, -1.0, 1.0, 1.0);
    EXPECT_NEAR(s, 2.0 * std::sqrt(2.0), kEps);
}

TEST(DerivedFeaturesSpan, SymmetricUnderSwap) {
    // span is order-invariant: ||r - l|| == ||l - r||
    const gp_scalar s1 = compute_pair_span(0.2, 0.3, -0.4, 0.7);
    const gp_scalar s2 = compute_pair_span(-0.4, 0.7, 0.2, 0.3);
    EXPECT_NEAR(s1, s2, kEps);
}

// ============================================================================
// tilt
// ============================================================================

TEST(DerivedFeaturesTilt, WingsLevelHorizontalPair) {
    // Port on image-left, starboard on image-right, same y → θ = 0
    // → (sin, cos) = (0, 1)
    const TiltSinCos t = compute_tilt(-0.5, 0.0, 0.5, 0.0);
    EXPECT_NEAR(t.sin, 0.0, kEps);
    EXPECT_NEAR(t.cos, 1.0, kEps);
}

TEST(DerivedFeaturesTilt, NinetyDegreeCCW) {
    // Port below, starboard above (dy positive, dx = 0) → θ = +π/2
    // → (sin, cos) = (1, 0)
    const TiltSinCos t = compute_tilt(0.0, -0.5, 0.0, 0.5);
    EXPECT_NEAR(t.sin, 1.0, kEps);
    EXPECT_NEAR(t.cos, 0.0, kEps);
}

TEST(DerivedFeaturesTilt, MinusNinetyDegreesCW) {
    // Port above, starboard below (dy negative, dx = 0) → θ = -π/2
    // → (sin, cos) = (-1, 0)
    const TiltSinCos t = compute_tilt(0.0, 0.5, 0.0, -0.5);
    EXPECT_NEAR(t.sin, -1.0, kEps);
    EXPECT_NEAR(t.cos, 0.0, kEps);
}

TEST(DerivedFeaturesTilt, OneEightyDegrees) {
    // Port on image-right, starboard on image-left (dx negative, dy = 0)
    // → θ = π → (sin, cos) = (0, -1)
    const TiltSinCos t = compute_tilt(0.5, 0.0, -0.5, 0.0);
    EXPECT_NEAR(t.sin, 0.0, kEps);
    EXPECT_NEAR(t.cos, -1.0, kEps);
}

TEST(DerivedFeaturesTilt, FortyFiveDegrees) {
    // Port at origin, starboard at (1, 1) → θ = +π/4
    // → (sin, cos) = (√2/2, √2/2)
    const TiltSinCos t = compute_tilt(0.0, 0.0, 1.0, 1.0);
    const gp_scalar half_root2 = std::sqrt(static_cast<gp_scalar>(0.5));
    EXPECT_NEAR(t.sin, half_root2, kEps);
    EXPECT_NEAR(t.cos, half_root2, kEps);
}

TEST(DerivedFeaturesTilt, IdentityPairIsDegenerateNeutral) {
    // Coincident beacons → degenerate; substitute (0, 1)
    const TiltSinCos t = compute_tilt(0.3, 0.4, 0.3, 0.4);
    EXPECT_NEAR(t.sin, 0.0, kEps);
    EXPECT_NEAR(t.cos, 1.0, kEps);
}

TEST(DerivedFeaturesTilt, BelowEpsilonIsDegenerateNeutral) {
    // Pair distance just under the epsilon → degenerate neutral.
    const gp_scalar tiny = static_cast<gp_scalar>(0.5) * kTiltDegenerateEpsilon;
    const TiltSinCos t = compute_tilt(0.0, 0.0, tiny, tiny);
    EXPECT_NEAR(t.sin, 0.0, kEps);
    EXPECT_NEAR(t.cos, 1.0, kEps);
}

TEST(DerivedFeaturesTilt, AboveEpsilonIsRealTilt) {
    // Pair distance just above the epsilon → real atan2 result.
    const gp_scalar above = static_cast<gp_scalar>(2.0) * kTiltDegenerateEpsilon;
    const TiltSinCos t = compute_tilt(0.0, 0.0, above, 0.0);
    // Should be (0, 1) — horizontal pair, same y → θ = 0
    EXPECT_NEAR(t.sin, 0.0, kEps);
    EXPECT_NEAR(t.cos, 1.0, kEps);
    // And NOT a degenerate-neutral substitution — verify by perturbing y
    // slightly: the result should differ from (0, 1).
    const TiltSinCos t2 = compute_tilt(0.0, 0.0, above, above);
    EXPECT_GT(std::abs(t2.sin), kEps);
}

TEST(DerivedFeaturesTilt, SinCosIdentityHolds) {
    // sin² + cos² ≈ 1.0 for all non-degenerate inputs.
    const struct { gp_scalar lx, ly, rx, ry; } cases[] = {
        {0.1, 0.2, 0.7, -0.3},
        {-0.5, 0.4, 0.6, 0.8},
        {0.0, 0.0, 0.001, 0.001},  // near epsilon
        {-0.99, -0.99, 0.99, 0.99},  // near full-screen
    };
    for (const auto& c : cases) {
        const TiltSinCos t = compute_tilt(c.lx, c.ly, c.rx, c.ry);
        const gp_scalar norm_sq = t.sin * t.sin + t.cos * t.cos;
        EXPECT_NEAR(norm_sq, 1.0, kEps)
            << "Failed for pair ("
            << c.lx << ", " << c.ly << ") -> ("
            << c.rx << ", " << c.ry << ")";
    }
}

// ============================================================================
// span_rate is mechanical subtraction at the call site; no helper needed.
// The integration layer (gather_tracker_inputs_tests) covers it end-to-end.
// ============================================================================


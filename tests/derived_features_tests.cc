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
using autoc::eval::compute_smoothness_factor;
using autoc::eval::SmoothnessMotionMode;

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

// ============================================================================
// 033 §2.B — compute_smoothness_factor pure-math tests.
//
// Per contracts/smoothness_factor.md "Validation tests". Pythagorean mode
// is the operator-preferred default; Sum + Max also covered.
// ============================================================================

// Rest state (no Δ since last tick) → factor = 1.0 regardless of floor or mode.
TEST(SmoothnessFactor, RestStateNoPenalty) {
    EXPECT_NEAR(compute_smoothness_factor(0, 0, 0, 0.5f, SmoothnessMotionMode::Pythagorean),
                1.0f, kEps);
    EXPECT_NEAR(compute_smoothness_factor(0, 0, 0, 0.0f, SmoothnessMotionMode::Pythagorean),
                1.0f, kEps);
    EXPECT_NEAR(compute_smoothness_factor(0, 0, 0, 0.5f, SmoothnessMotionMode::Sum),
                1.0f, kEps);
    EXPECT_NEAR(compute_smoothness_factor(0, 0, 0, 0.5f, SmoothnessMotionMode::Max),
                1.0f, kEps);
}

// Floor = 1.0 → factor = 1.0 always (back-compat / regression-test mode).
TEST(SmoothnessFactor, FloorOneIsNoOp) {
    // Any motion, any mode — factor should be 1.0.
    EXPECT_NEAR(compute_smoothness_factor(2, 2, 2, 1.0f, SmoothnessMotionMode::Pythagorean),
                1.0f, kEps);
    EXPECT_NEAR(compute_smoothness_factor(1, 0, 0, 1.0f, SmoothnessMotionMode::Sum),
                1.0f, kEps);
    EXPECT_NEAR(compute_smoothness_factor(2, 0, 0, 1.0f, SmoothnessMotionMode::Max),
                1.0f, kEps);
}

// All-3-axis bang-bang (Δ=2 each) → factor = floor (max penalty).
TEST(SmoothnessFactor, PythagoreanAll3AxesBangBangHitsFloor) {
    const gp_scalar f = compute_smoothness_factor(2, 2, 2, 0.5f, SmoothnessMotionMode::Pythagorean);
    EXPECT_NEAR(f, 0.5f, kEps);
}

TEST(SmoothnessFactor, SumAll3AxesBangBangHitsFloor) {
    const gp_scalar f = compute_smoothness_factor(2, 2, 2, 0.5f, SmoothnessMotionMode::Sum);
    EXPECT_NEAR(f, 0.5f, kEps);
}

TEST(SmoothnessFactor, MaxSingleAxisBangBangHitsFloor) {
    // Max mode: single-axis bang-bang saturates motion_max (=2.0).
    const gp_scalar f = compute_smoothness_factor(2, 0, 0, 0.5f, SmoothnessMotionMode::Max);
    EXPECT_NEAR(f, 0.5f, kEps);
}

// Pythagorean single-axis bang-bang (Δ=2 on one axis): factor ≈ 0.7113
// = 1 - 0.5 × (2/sqrt(12)) = 1 - 0.5 × 0.577 = 0.7113.
TEST(SmoothnessFactor, PythagoreanSingleAxisBangBang) {
    const gp_scalar f = compute_smoothness_factor(2, 0, 0, 0.5f, SmoothnessMotionMode::Pythagorean);
    const gp_scalar expected = 1.0f - 0.5f * (2.0f / std::sqrt(12.0f));
    EXPECT_NEAR(f, expected, kEps);
    EXPECT_GT(f, 0.5f);  // above floor
    EXPECT_LT(f, 1.0f);  // below no-penalty
}

// Pythagorean Δ=(1,1,1): motion = sqrt(3), motion_max = sqrt(12), ratio = 0.5,
// factor = 1 - 0.5×0.5 = 0.75.
TEST(SmoothnessFactor, PythagoreanModerateMotionFactorThreeQuarters) {
    const gp_scalar f = compute_smoothness_factor(1, 1, 1, 0.5f, SmoothnessMotionMode::Pythagorean);
    EXPECT_NEAR(f, 0.75f, kEps);
}

// Out-of-range Δs (theoretically impossible since outputs are tanh-saturated)
// should clamp to motion_max and produce floor — defensive guarantee.
TEST(SmoothnessFactor, OverRangeDeltasClampToFloor) {
    const gp_scalar f = compute_smoothness_factor(3, 3, 3, 0.5f, SmoothnessMotionMode::Pythagorean);
    EXPECT_NEAR(f, 0.5f, kEps);
}

// Floor = 0.0 with max motion → factor = 0.0 (extreme operator setting).
TEST(SmoothnessFactor, FloorZeroWithMaxMotionAnnihilates) {
    const gp_scalar f = compute_smoothness_factor(2, 2, 2, 0.0f, SmoothnessMotionMode::Pythagorean);
    EXPECT_NEAR(f, 0.0f, kEps);
}

// Range invariant: factor ∈ [floor, 1.0] for ALL inputs, floors, modes
// (fuzz Δs in [-3, +3], all floors in {0, 0.3, 0.5, 0.7, 1.0}, all modes).
TEST(SmoothnessFactor, PropertyFactorInRangeFloorToOne) {
    const gp_scalar floors[] = {0.0f, 0.3f, 0.5f, 0.7f, 1.0f};
    const SmoothnessMotionMode modes[] = {
        SmoothnessMotionMode::Pythagorean,
        SmoothnessMotionMode::Sum,
        SmoothnessMotionMode::Max,
    };
    // Coarse grid sweep over Δs covering the operator-relevant + out-of-range zones.
    for (gp_scalar dpt = -3.0f; dpt <= 3.0f; dpt += 0.5f) {
        for (gp_scalar drl = -3.0f; drl <= 3.0f; drl += 0.5f) {
            for (gp_scalar dth = -3.0f; dth <= 3.0f; dth += 0.5f) {
                for (gp_scalar floor : floors) {
                    for (SmoothnessMotionMode mode : modes) {
                        const gp_scalar f = compute_smoothness_factor(dpt, drl, dth, floor, mode);
                        EXPECT_GE(f, floor - kEps)
                            << "floor invariant failed: f=" << f << " floor=" << floor
                            << " Δ=(" << dpt << "," << drl << "," << dth << ")";
                        EXPECT_LE(f, 1.0f + kEps)
                            << "upper invariant failed: f=" << f
                            << " Δ=(" << dpt << "," << drl << "," << dth << ")";
                    }
                }
            }
        }
    }
}

// Mode comparison: same Δs, different modes → different factors (modes are
// genuinely distinct, not aliased).
TEST(SmoothnessFactor, ModesProduceDistinctFactors) {
    // Use a non-symmetric Δ where Pythagorean and Sum diverge.
    const gp_scalar f_pyth = compute_smoothness_factor(
        1.5f, 0.5f, 0.0f, 0.5f, SmoothnessMotionMode::Pythagorean);
    const gp_scalar f_sum = compute_smoothness_factor(
        1.5f, 0.5f, 0.0f, 0.5f, SmoothnessMotionMode::Sum);
    const gp_scalar f_max = compute_smoothness_factor(
        1.5f, 0.5f, 0.0f, 0.5f, SmoothnessMotionMode::Max);
    EXPECT_NE(f_pyth, f_sum);
    EXPECT_NE(f_pyth, f_max);
    EXPECT_NE(f_sum, f_max);
}

// Operator-facing example from spec confirmation:
// floor=0.5, Pythagorean, motion=1.5 → factor ≈ 0.7835
// (per "Reference points" table; penalty 21.6%)
TEST(SmoothnessFactor, OperatorExampleFromConfirmation) {
    // Motion of 1.5 via one axis at sqrt(1.5²) = 1.5 (single-axis case).
    // Pythagorean: motion = sqrt(1.5² + 0 + 0) = 1.5.
    const gp_scalar f = compute_smoothness_factor(1.5f, 0.0f, 0.0f, 0.5f,
                                                  SmoothnessMotionMode::Pythagorean);
    // motion_max = sqrt(12), ratio = 1.5/sqrt(12) ≈ 0.4330127
    // factor = 1 - 0.5 × 0.4330127 = 0.7834937
    const gp_scalar expected = 1.0f - 0.5f * (1.5f / std::sqrt(12.0f));
    EXPECT_NEAR(f, expected, kEps);
}

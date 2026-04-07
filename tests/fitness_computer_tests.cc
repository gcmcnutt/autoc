#include <gtest/gtest.h>
#include <cmath>
#include "autoc/eval/fitness_computer.h"

// Default test parameters matching autoc.ini defaults (V4 conical surface)
static const double DIST_BEHIND = 7.0;      // m (-7m behind = 0.5 threshold)
static const double DIST_AHEAD = 2.0;       // m
static const double CONE_ANGLE_DEG = 45.0;  // degrees
static const double STREAK_THRESH = 0.5;
static const int STREAK_STEPS = 50;          // 5.0s at 100ms
static const double STREAK_MULT_MAX = 5.0;

static FitnessComputer makeDefault() {
    return FitnessComputer(DIST_BEHIND, DIST_AHEAD, CONE_ANGLE_DEG,
                            STREAK_THRESH, STREAK_STEPS, STREAK_MULT_MAX);
}

// Reference implementation in test code (Python-validated values)
static double refScore(double along, double lat,
                       double distBehind = DIST_BEHIND,
                       double distAhead = DIST_AHEAD,
                       double coneDeg = CONE_ANGLE_DEG) {
    double dist = std::sqrt(along*along + lat*lat);
    if (dist < 1e-6) return 1.0;
    double cosA = -along / dist;
    if (cosA > 1.0) cosA = 1.0;
    if (cosA < -1.0) cosA = -1.0;
    double angle = std::acos(cosA);
    constexpr double HALF_PI = M_PI / 2.0;
    double angleClamped = (angle < HALF_PI) ? angle : HALF_PI;
    double distScale = (along <= 0.0) ? distBehind : distAhead;
    double effDist  = dist / distScale;
    double effAngle = angleClamped / (coneDeg * M_PI / 180.0);
    return 1.0 / (1.0 + effDist*effDist + effAngle*effAngle);
}

// ========================================================================
// V4 conical scoring surface unit tests
// ========================================================================

TEST(FitnessComputer022, ScoreAtRabbit) {
    auto fc = makeDefault();
    EXPECT_DOUBLE_EQ(fc.computeStepScore(0.0, 0.0), 1.0);
}

TEST(FitnessComputer022, ScoreBehindOnAxis) {
    auto fc = makeDefault();
    // Directly behind: angle = 0, only distance term contributes.
    // score = 1 / (1 + (d/7)²)
    EXPECT_NEAR(fc.computeStepScore(-3.0, 0.0),  refScore(-3, 0),  1e-10);
    EXPECT_NEAR(fc.computeStepScore(-5.0, 0.0),  refScore(-5, 0),  1e-10);
    EXPECT_NEAR(fc.computeStepScore(-7.0, 0.0),  0.5, 1e-10);    // (7/7)² = 1.0 → exact threshold
    EXPECT_NEAR(fc.computeStepScore(-10.0, 0.0), refScore(-10, 0), 1e-10);
    EXPECT_NEAR(fc.computeStepScore(-14.0, 0.0), 0.2, 1e-10);    // (14/7)² = 4
}

TEST(FitnessComputer022, ScoreAheadHasGradient) {
    auto fc = makeDefault();
    // V4's key fix: ahead positions have a real distance gradient via the
    // small distScaleAhead. Each meter ahead is materially worse than the last.
    double s_0_5  = fc.computeStepScore(0.5, 0.0);
    double s_1    = fc.computeStepScore(1.0, 0.0);
    double s_2    = fc.computeStepScore(2.0, 0.0);
    double s_5    = fc.computeStepScore(5.0, 0.0);
    double s_10   = fc.computeStepScore(10.0, 0.0);

    // Monotonic decrease as distance ahead grows
    EXPECT_GT(s_0_5, s_1);
    EXPECT_GT(s_1,   s_2);
    EXPECT_GT(s_2,   s_5);
    EXPECT_GT(s_5,   s_10);

    // Reference values from formula
    EXPECT_NEAR(s_0_5, refScore(0.5, 0), 1e-10);
    EXPECT_NEAR(s_1,   refScore(1.0, 0), 1e-10);
    EXPECT_NEAR(s_5,   refScore(5.0, 0), 1e-10);

    // Far ahead is catastrophic, near ahead has recovery signal
    EXPECT_GT(s_0_5, 0.15);  // recoverable
    EXPECT_LT(s_5,   0.10);  // very bad
    EXPECT_LT(s_10,  0.05);  // catastrophic
}

TEST(FitnessComputer022, ScoreAheadGradientExceedsV1) {
    auto fc = makeDefault();
    // Check the V4 fix specifically: V1 had ~constant ~0.058 for 0.5m to 5m
    // ahead. V4 must show meaningful difference.
    double s_0_5 = fc.computeStepScore(0.5, 0.0);
    double s_5   = fc.computeStepScore(5.0, 0.0);
    // V4: 0.198 vs 0.089 — roughly 2.2x difference
    EXPECT_GT(s_0_5 / s_5, 1.5);
}

TEST(FitnessComputer022, ScorePureLateralPenalized) {
    auto fc = makeDefault();
    // Pure lateral at along=0: angle = π/2 (clamped exactly), so eff_angle² = 4
    // (with cone=45°). plus small distance term. score ≈ 0.19.
    // This is the GRAVITY FIX: 5m off-axis at along=0 used to score 0.5,
    // now scores < 0.20 — clearly outside the streak threshold.
    EXPECT_NEAR(fc.computeStepScore(0.0, 5.0), refScore(0, 5), 1e-10);
    EXPECT_LT(fc.computeStepScore(0.0, 5.0), 0.25);
    EXPECT_LT(fc.computeStepScore(0.0, 5.0), STREAK_THRESH);

    EXPECT_NEAR(fc.computeStepScore(0.0, 10.0), refScore(0, 10), 1e-10);
    EXPECT_LT(fc.computeStepScore(0.0, 10.0), 0.20);
}

TEST(FitnessComputer022, ScoreCombinedBehindAndLateral) {
    auto fc = makeDefault();
    // -7 back, 7 lateral: distance ≈ 9.9, angle = 45° (cone edge)
    // eff_dist = 9.9/7 ≈ 1.414, eff_angle = 1.0
    // score = 1/(1 + 2 + 1) = 0.25 exactly
    EXPECT_NEAR(fc.computeStepScore(-7.0, 7.0), 0.25, 1e-10);

    // -5 back, 5 lateral: also 45° cone-edge, smaller distance → better score
    EXPECT_NEAR(fc.computeStepScore(-5.0, 5.0), refScore(-5, 5), 1e-10);

    // -10 back, 5 lateral: angle ≈ 26.57°
    EXPECT_NEAR(fc.computeStepScore(-10.0, 5.0), refScore(-10, 5), 1e-10);
}

TEST(FitnessComputer022, ScoreAsymmetryAheadVsBehind) {
    auto fc = makeDefault();
    // Same magnitude — ahead is dramatically worse than behind.
    // Behind 3m: 0.917 (good)
    // Ahead 3m: ~0.13 (bad — small dist scale ahead)
    double behind3 = fc.computeStepScore(-3.0, 0.0);
    double ahead3  = fc.computeStepScore(3.0, 0.0);
    EXPECT_GT(behind3, 0.8);
    EXPECT_LT(ahead3, 0.20);
    EXPECT_GT(behind3, 5.0 * ahead3);  // dramatic asymmetry
}

TEST(FitnessComputer022, ScoreLateralDistAlwaysPositive) {
    auto fc = makeDefault();
    // lateralDist is a magnitude (always >= 0). 5m left == 5m right.
    EXPECT_DOUBLE_EQ(fc.computeStepScore(0.0, 5.0), fc.computeStepScore(0.0, 5.0));
}

TEST(FitnessComputer022, ScoreFarAwayStillPositive) {
    auto fc = makeDefault();
    // Always strictly positive — gradient signal everywhere.
    EXPECT_GT(fc.computeStepScore(-50.0, 0.0), 0.0);
    EXPECT_GT(fc.computeStepScore(0.0, 50.0), 0.0);
    EXPECT_GT(fc.computeStepScore(50.0, 50.0), 0.0);
}

TEST(FitnessComputer022, ScoreGravityFix) {
    auto fc = makeDefault();
    // The key claim: 5m off-axis at along=0 (left/right/above/below)
    // is now firmly OUTSIDE the streak threshold (was exactly at 0.5).
    // This eliminates the "happy hovering" gradient-free zone.
    double off = fc.computeStepScore(0.0, 5.0);
    EXPECT_LT(off, STREAK_THRESH);
    EXPECT_LT(off, 0.25);

    // Pure tail-chase 5m behind: well above threshold, dramatically better than off-axis.
    // tail ≈ 0.66, off ≈ 0.18 → ratio ~3.6x
    double tail = fc.computeStepScore(-5.0, 0.0);
    EXPECT_GT(tail, STREAK_THRESH);
    EXPECT_GT(tail, 3.0 * off);  // very different scores
}

TEST(FitnessComputer022, ScoreAheadIsAlwaysWorseThanLateralAtSameDistance) {
    auto fc = makeDefault();
    // Side-by-side comparison at distance 5:
    //   5m to side (along=0): dist_scale = behind (7m),  eff_d = 0.71, eff_a = 2 → 0.18
    //   5m ahead (along=5):   dist_scale = ahead  (2m),  eff_d = 2.5,  eff_a = 2 → 0.089
    // The small dist_scale_ahead makes ahead-5 dramatically worse than side-5.
    double side5  = fc.computeStepScore(0.0, 5.0);
    double ahead5 = fc.computeStepScore(5.0, 0.0);
    EXPECT_GT(side5, ahead5);
}

TEST(FitnessComputer022, ScoreDifferentScales) {
    // Tighter cone (20°) and smaller distance scales
    FitnessComputer fc(5.0, 1.0, 20.0, 0.5, 50, 5.0);
    // 5m directly behind: eff_dist = 1, angle = 0 → score = 1/(1+1) = 0.5
    EXPECT_NEAR(fc.computeStepScore(-5.0, 0.0), 0.5, 1e-10);
    // 1m directly ahead: eff_dist = 1 (with ahead_scale=1), angle clamped to π/2,
    // eff_angle = (π/2) / (20°) = (π/2) / (π/9) = 4.5, eff_angle² = 20.25
    // score = 1/(1 + 1 + 20.25) ≈ 0.045
    EXPECT_LT(fc.computeStepScore(1.0, 0.0), 0.10);
}

// ========================================================================
// Streak mechanism unit tests (unchanged behavior)
// ========================================================================

TEST(FitnessComputer022, StreakFirstStep) {
    auto fc = makeDefault();
    fc.resetStreak();
    double result = fc.applyStreak(0.8);
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 1.0 / STREAK_STEPS), 1e-10);
}

TEST(FitnessComputer022, StreakBuildsToMax) {
    auto fc = makeDefault();
    fc.resetStreak();
    double result = 0.0;
    for (int i = 0; i < STREAK_STEPS; i++) {
        result = fc.applyStreak(0.8);
    }
    EXPECT_NEAR(result, 0.8 * 5.0, 1e-10);
}

TEST(FitnessComputer022, StreakCapsAtMax) {
    auto fc = makeDefault();
    fc.resetStreak();
    double result = 0.0;
    for (int i = 0; i < STREAK_STEPS + 10; i++) {
        result = fc.applyStreak(0.8);
    }
    EXPECT_NEAR(result, 0.8 * 5.0, 1e-10);
}

TEST(FitnessComputer022, StreakHardReset) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 10; i++) {
        fc.applyStreak(0.8);
    }
    fc.applyStreak(0.1);  // below threshold → reset
    double result = fc.applyStreak(0.8);
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 1.0 / STREAK_STEPS), 1e-10);
}

TEST(FitnessComputer022, StreakThresholdBoundary) {
    auto fc = makeDefault();
    fc.resetStreak();
    fc.applyStreak(0.50);
    double at_threshold = fc.applyStreak(0.50);
    EXPECT_GT(at_threshold, 0.50);

    fc.resetStreak();
    fc.applyStreak(0.499);
    double below = fc.applyStreak(0.499);
    EXPECT_NEAR(below, 0.499, 1e-10);
}

TEST(FitnessComputer022, StreakMultiplierLinearity) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 12; i++) {
        fc.applyStreak(0.8);
    }
    double result = fc.applyStreak(0.8);
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 13.0 / STREAK_STEPS), 1e-10);
}

TEST(FitnessComputer022, DiagnosticMaxStreak) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 5; i++) fc.applyStreak(0.8);
    fc.applyStreak(0.1);
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_EQ(fc.getMaxStreak(), 10);
}

TEST(FitnessComputer022, DiagnosticTotalStreakSteps) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 5; i++) fc.applyStreak(0.8);
    fc.applyStreak(0.1);
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_EQ(fc.getStreakSteps(), 15);
}

TEST(FitnessComputer022, DiagnosticMaxMultiplier) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_NEAR(fc.getMaxMultiplier(), 1.0 + 4.0 * 10.0 / STREAK_STEPS, 1e-10);
}

TEST(FitnessComputer022, StreakRateIndependence) {
    FitnessComputer fc50(DIST_BEHIND, DIST_AHEAD, CONE_ANGLE_DEG, STREAK_THRESH, 50, STREAK_MULT_MAX);
    FitnessComputer fc25(DIST_BEHIND, DIST_AHEAD, CONE_ANGLE_DEG, STREAK_THRESH, 25, STREAK_MULT_MAX);

    fc50.resetStreak();
    fc25.resetStreak();

    for (int i = 0; i < 50; i++) fc50.applyStreak(0.8);
    for (int i = 0; i < 25; i++) fc25.applyStreak(0.8);
    EXPECT_NEAR(fc50.getMaxMultiplier(), 5.0, 1e-10);
    EXPECT_NEAR(fc25.getMaxMultiplier(), 5.0, 1e-10);
}

TEST(FitnessComputer022, ResetClearsDiagnostics) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_EQ(fc.getMaxStreak(), 10);
    EXPECT_EQ(fc.getStreakSteps(), 10);
    EXPECT_GT(fc.getMaxMultiplier(), 1.0);

    fc.resetStreak();
    EXPECT_EQ(fc.getMaxStreak(), 0);
    EXPECT_EQ(fc.getStreakSteps(), 0);
    EXPECT_NEAR(fc.getMaxMultiplier(), 1.0, 1e-10);
}

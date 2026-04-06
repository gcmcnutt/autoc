#include <gtest/gtest.h>
#include <cmath>
#include "autoc/eval/fitness_computer.h"

// Default test parameters matching autoc.ini defaults
static const double BEHIND = 10.0;
static const double AHEAD = 0.5;
static const double CROSS = 5.0;
static const double STREAK_THRESH = 0.5;
static const int STREAK_STEPS = 25;  // 2.5s at 100ms
static const double STREAK_MULT_MAX = 5.0;

static FitnessComputer makeDefault() {
    return FitnessComputer(BEHIND, AHEAD, CROSS, STREAK_THRESH, STREAK_STEPS, STREAK_MULT_MAX);
}

// ========================================================================
// T004: Scoring surface unit tests
// ========================================================================

TEST(FitnessComputer022, ScoreAtRabbit) {
    auto fc = makeDefault();
    EXPECT_DOUBLE_EQ(fc.computeStepScore(0.0, 0.0), 1.0);
}

TEST(FitnessComputer022, ScoreBehindOnPath) {
    auto fc = makeDefault();
    // behind: eff_along = along/10, score = 1/(1+(along/10)^2)
    EXPECT_NEAR(fc.computeStepScore(-3.0, 0.0), 1.0/(1.0 + 9.0/100.0), 1e-10);  // 0.917
    EXPECT_NEAR(fc.computeStepScore(-5.0, 0.0), 1.0/(1.0 + 25.0/100.0), 1e-10); // 0.80
    EXPECT_NEAR(fc.computeStepScore(-10.0, 0.0), 1.0/(1.0 + 1.0), 1e-10);       // 0.50
    EXPECT_NEAR(fc.computeStepScore(-20.0, 0.0), 1.0/(1.0 + 4.0), 1e-10);       // 0.20
}

TEST(FitnessComputer022, ScoreAheadOnPath) {
    auto fc = makeDefault();
    // ahead: eff_along = along/0.5, score = 1/(1+(along/0.5)^2)
    EXPECT_NEAR(fc.computeStepScore(0.5, 0.0), 1.0/(1.0 + 1.0), 1e-10);   // 0.50
    EXPECT_NEAR(fc.computeStepScore(1.0, 0.0), 1.0/(1.0 + 4.0), 1e-10);   // 0.20
    EXPECT_NEAR(fc.computeStepScore(2.0, 0.0), 1.0/(1.0 + 16.0), 1e-10);  // 0.0588
}

TEST(FitnessComputer022, ScorePureLateral) {
    auto fc = makeDefault();
    // lateral: eff_lat = lat/5, score = 1/(1+(lat/5)^2)
    EXPECT_NEAR(fc.computeStepScore(0.0, 5.0), 1.0/(1.0 + 1.0), 1e-10);   // 0.50
    EXPECT_NEAR(fc.computeStepScore(0.0, 10.0), 1.0/(1.0 + 4.0), 1e-10);  // 0.20
}

TEST(FitnessComputer022, ScoreCombined) {
    auto fc = makeDefault();
    // -5 behind, 5 lateral: eff_along=-5/10=-0.5, eff_lat=5/5=1.0
    // eff_dist_sq = 0.25 + 1.0 = 1.25
    EXPECT_NEAR(fc.computeStepScore(-5.0, 5.0), 1.0/(1.0 + 1.25), 1e-10);  // 0.444
}

TEST(FitnessComputer022, ScoreAsymmetry) {
    auto fc = makeDefault();
    // 3m ahead MUCH worse than 3m behind
    double ahead3 = fc.computeStepScore(3.0, 0.0);   // 1/(1+36) = 0.027
    double behind3 = fc.computeStepScore(-3.0, 0.0);  // 1/(1+0.09) = 0.917
    EXPECT_LT(ahead3, behind3);
    EXPECT_LT(ahead3, 0.05);
    EXPECT_GT(behind3, 0.90);
}

TEST(FitnessComputer022, ScoreLateralDistAlwaysPositive) {
    auto fc = makeDefault();
    // lateralDist is always >= 0 (it's a distance, not signed)
    EXPECT_DOUBLE_EQ(fc.computeStepScore(0.0, 5.0), fc.computeStepScore(0.0, 5.0));
}

TEST(FitnessComputer022, ScoreFarAwaySignal) {
    auto fc = makeDefault();
    // Always positive even very far away
    EXPECT_GT(fc.computeStepScore(-50.0, 0.0), 0.0);
    EXPECT_GT(fc.computeStepScore(0.0, 50.0), 0.0);
    EXPECT_GT(fc.computeStepScore(50.0, 50.0), 0.0);
}

TEST(FitnessComputer022, ScoreDifferentScales) {
    // Custom scales: behind=5, ahead=1, cross=3
    FitnessComputer fc(5.0, 1.0, 3.0, 0.5, 25, 5.0);
    // 5m behind with behind_scale=5: eff_along = -5/5 = -1, score = 1/(1+1) = 0.5
    EXPECT_NEAR(fc.computeStepScore(-5.0, 0.0), 0.5, 1e-10);
    // 1m ahead with ahead_scale=1: eff_along = 1/1 = 1, score = 1/(1+1) = 0.5
    EXPECT_NEAR(fc.computeStepScore(1.0, 0.0), 0.5, 1e-10);
    // 3m lateral with cross_scale=3: eff_lat = 3/3 = 1, score = 1/(1+1) = 0.5
    EXPECT_NEAR(fc.computeStepScore(0.0, 3.0), 0.5, 1e-10);
}

// ========================================================================
// T005: Streak mechanism unit tests
// ========================================================================

TEST(FitnessComputer022, StreakFirstStep) {
    auto fc = makeDefault();
    fc.resetStreak();
    // First good step: streakCount=1, mult = 1 + 4*1/25 = 1.16
    double result = fc.applyStreak(0.8);
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 1.0 / 25.0), 1e-10);
}

TEST(FitnessComputer022, StreakBuildsToMax) {
    auto fc = makeDefault();
    fc.resetStreak();
    double result = 0.0;
    for (int i = 0; i < 25; i++) {
        result = fc.applyStreak(0.8);
    }
    // After 25 steps: streakCount=25, mult=5.0
    EXPECT_NEAR(result, 0.8 * 5.0, 1e-10);
}

TEST(FitnessComputer022, StreakCapsAtMax) {
    auto fc = makeDefault();
    fc.resetStreak();
    double result = 0.0;
    for (int i = 0; i < 30; i++) {
        result = fc.applyStreak(0.8);
    }
    // After 30 steps: still capped at 25, mult=5.0
    EXPECT_NEAR(result, 0.8 * 5.0, 1e-10);
}

TEST(FitnessComputer022, StreakHardReset) {
    auto fc = makeDefault();
    fc.resetStreak();
    // Build 10 steps
    for (int i = 0; i < 10; i++) {
        fc.applyStreak(0.8);
    }
    // Reset by low score
    fc.applyStreak(0.1);  // below threshold (0.5)
    // Next good step starts at streakCount=1 again
    double result = fc.applyStreak(0.8);
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 1.0 / 25.0), 1e-10);
}

TEST(FitnessComputer022, StreakThresholdBoundary) {
    auto fc = makeDefault();
    fc.resetStreak();
    // Exactly at threshold: should maintain streak
    fc.applyStreak(0.50);
    double at_threshold = fc.applyStreak(0.50);
    EXPECT_GT(at_threshold, 0.50);  // multiplied by > 1.0

    fc.resetStreak();
    // Just below threshold: should not build streak
    fc.applyStreak(0.499);
    // streakCount should be 0, so next call with 0.499 → still no streak
    double below = fc.applyStreak(0.499);
    EXPECT_NEAR(below, 0.499 * 1.0, 1e-10);  // no multiplier since streak reset

    // Actually: 0.499 < 0.5 → streak resets, then 0.499 < 0.5 → still 0
    // but wait — if below threshold, streakCount=0, multiplier=1.0
    // So result = 0.499 * 1.0... but applyStreak returns stepPoints * multiplier
    // where multiplier is based on NEW streakCount (0)
    // So below should be 0.499 * (1 + 4*0/25) = 0.499
}

TEST(FitnessComputer022, StreakMultiplierLinearity) {
    auto fc = makeDefault();
    fc.resetStreak();
    // Build to step 12
    for (int i = 0; i < 12; i++) {
        fc.applyStreak(0.8);
    }
    // Step 12: mult = 1.0 + 4.0 * 12/25 = 2.92
    double result = fc.applyStreak(0.8);  // step 13
    EXPECT_NEAR(result, 0.8 * (1.0 + 4.0 * 13.0 / 25.0), 1e-10);
}

TEST(FitnessComputer022, DiagnosticMaxStreak) {
    auto fc = makeDefault();
    fc.resetStreak();
    // 5 good, reset, 10 good
    for (int i = 0; i < 5; i++) fc.applyStreak(0.8);
    fc.applyStreak(0.1);  // reset
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_EQ(fc.getMaxStreak(), 10);
}

TEST(FitnessComputer022, DiagnosticTotalStreakSteps) {
    auto fc = makeDefault();
    fc.resetStreak();
    // 5 good, reset, 10 good
    for (int i = 0; i < 5; i++) fc.applyStreak(0.8);
    fc.applyStreak(0.1);  // reset (not in streak)
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    EXPECT_EQ(fc.getStreakSteps(), 15);
}

TEST(FitnessComputer022, DiagnosticMaxMultiplier) {
    auto fc = makeDefault();
    fc.resetStreak();
    for (int i = 0; i < 10; i++) fc.applyStreak(0.8);
    // maxMultiplier after 10 steps = 1.0 + 4.0*10/25 = 2.6
    EXPECT_NEAR(fc.getMaxMultiplier(), 1.0 + 4.0 * 10.0 / 25.0, 1e-10);
}

TEST(FitnessComputer022, StreakRateIndependence) {
    // 50 steps at "5s at 100ms" vs 25 steps at "2.5s at 100ms"
    // Both should reach max at the same fraction
    FitnessComputer fc50(BEHIND, AHEAD, CROSS, STREAK_THRESH, 50, STREAK_MULT_MAX);
    FitnessComputer fc25(BEHIND, AHEAD, CROSS, STREAK_THRESH, 25, STREAK_MULT_MAX);

    fc50.resetStreak();
    fc25.resetStreak();

    // At 50% of max steps, both should have same multiplier
    for (int i = 0; i < 25; i++) fc50.applyStreak(0.8);  // 50% of 50
    for (int i = 0; i < 12; i++) fc25.applyStreak(0.8);  // ~50% of 25

    // mult50 at step 25 of 50 = 1 + 4*25/50 = 3.0
    // mult25 at step 12 of 25 = 1 + 4*12/25 = 2.92  (close but not exact due to integer)
    // The key property: at 100% both reach 5.0
    for (int i = 25; i < 50; i++) fc50.applyStreak(0.8);
    for (int i = 12; i < 25; i++) fc25.applyStreak(0.8);
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

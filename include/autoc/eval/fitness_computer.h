#ifndef FITNESS_COMPUTER_H
#define FITNESS_COMPUTER_H

// Point-accumulation fitness (022): conical scoring surface + streak multiplier.
//
// Polar form with directional distance scaling and clamped angle:
//
//   distance     = sqrt(along^2 + lateral^2)
//   angle        = acos(-along / distance)        // 0 = directly behind, π = ahead
//   angle_clamp  = min(angle, π/2)                // ahead saturates at "sideways"
//   dist_scale   = distScaleBehind  if along <= 0
//                  distScaleAhead   if along > 0  // small => sharp ahead penalty
//
//   eff_dist  = distance / dist_scale
//   eff_angle = angle_clamp / coneAngle
//   score     = 1 / (1 + eff_dist^2 + eff_angle^2)
//
// Why this shape:
//   - Tail-chase rewarded — pure-behind on-axis is the global maximum line
//   - Cone-shaped good zone — angular term pinches lateral offset
//   - Gravity problem fixed — 5m to the side at along=0 is firmly < threshold
//   - Real ahead gradient — small distScaleAhead makes the distance term carry
//     the gradient ahead. The angle clamp at π/2 prevents the angle term from
//     saturating and erasing the distance gradient (V1's mistake).
//   - Adversarial intercept geometry — matches tail-chase missile semantics
//
// See specs/022-tracking-cone-fitness/development-report.md for the full
// V1 → V3 → V4 design discussion.
class FitnessComputer {
public:
    FitnessComputer(double distScaleBehind, double distScaleAhead, double coneAngleDeg,
                    double streakThreshold, int streakStepsToMax, double streakMultMax);

    // Compute step score from along-track and cross-track distances.
    // along: positive = ahead of rabbit, negative = behind (preferred)
    // lateralDist: perpendicular distance (always >= 0)
    // Returns: stepPoints in (0, 1], 1.0 at rabbit position.
    double computeStepScore(double along, double lateralDist) const;

    // Update streak state and return stepPoints * multiplier.
    // Streak increments if stepPoints >= threshold, hard-resets otherwise.
    double applyStreak(double stepPoints);

    // Reset streak and diagnostics (call at start of each scenario).
    void resetStreak();

    // Diagnostics
    int getMaxStreak() const { return maxStreak_; }
    int getStreakSteps() const { return totalStreakSteps_; }
    double getMaxMultiplier() const { return maxMultiplier_; }

private:
    double distScaleBehind_;  // m, distance half-decay when behind rabbit (forgiving)
    double distScaleAhead_;   // m, distance half-decay when ahead of rabbit (sharp)
    double coneAngleRad_;     // radians, angular half-decay (cone half-width)
    double streakThreshold_;
    int streakStepsToMax_;
    double streakMultMax_;

    // Per-scenario state
    int streakCount_ = 0;
    int maxStreak_ = 0;
    int totalStreakSteps_ = 0;
    double maxMultiplier_ = 1.0;
};

#endif

#include "autoc/eval/fitness_computer.h"
#include <algorithm>
#include <cmath>

FitnessComputer::FitnessComputer(double distScaleBehind, double distScaleAhead, double coneAngleDeg,
                                 double streakThreshold, int streakStepsToMax, double streakMultMax)
    : distScaleBehind_(distScaleBehind),
      distScaleAhead_(distScaleAhead),
      coneAngleRad_(coneAngleDeg * M_PI / 180.0),
      streakThreshold_(streakThreshold),
      streakStepsToMax_(streakStepsToMax),
      streakMultMax_(streakMultMax) {}

double FitnessComputer::computeStepScore(double along, double lateralDist) const {
    // Polar (conical) form with directional distance scaling and clamped angle.
    //   distance     = sqrt(along² + lateralDist²)
    //   angle        = acos(-along / distance)   [0 = behind, π = ahead]
    //   angle_clamp  = min(angle, π/2)           [ahead saturates at "sideways"]
    //   dist_scale   = behind or ahead based on along sign
    //   score        = 1 / (1 + (dist/dist_scale)² + (angle_clamp/cone)²)
    //
    // The clamp at π/2 prevents the angle term from saturating ahead, so the
    // (small) ahead distance scale carries the gradient when overshooting.
    double distance = std::sqrt(along * along + lateralDist * lateralDist);
    if (distance < 1e-6) {
        return 1.0;  // At the rabbit position
    }
    // Angle from "directly behind" direction (-tangent).
    double cosAngle = -along / distance;
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    double angle = std::acos(cosAngle);  // 0 = directly behind, π = directly ahead

    // Clamp angle at π/2 so ahead positions don't saturate the angle term —
    // the directional distance scale carries the ahead gradient.
    constexpr double HALF_PI = M_PI / 2.0;
    double angleClamped = (angle < HALF_PI) ? angle : HALF_PI;

    // Directional distance scale: forgiving behind, sharp ahead.
    double distScale = (along <= 0.0) ? distScaleBehind_ : distScaleAhead_;

    double effDist  = distance / distScale;
    double effAngle = angleClamped / coneAngleRad_;
    double effTotalSq = effDist * effDist + effAngle * effAngle;
    return 1.0 / (1.0 + effTotalSq);
}

double FitnessComputer::applyStreak(double stepPoints) {
    if (stepPoints >= streakThreshold_) {
        streakCount_ = std::min(streakCount_ + 1, streakStepsToMax_);
        totalStreakSteps_++;
    } else {
        streakCount_ = 0;
    }
    maxStreak_ = std::max(maxStreak_, streakCount_);
    double multiplier = 1.0 + (streakMultMax_ - 1.0) * static_cast<double>(streakCount_) / static_cast<double>(streakStepsToMax_);
    maxMultiplier_ = std::max(maxMultiplier_, multiplier);
    return stepPoints * multiplier;
}

void FitnessComputer::resetStreak() {
    streakCount_ = 0;
    maxStreak_ = 0;
    totalStreakSteps_ = 0;
    maxMultiplier_ = 1.0;
}

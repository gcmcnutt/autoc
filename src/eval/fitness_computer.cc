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
    return decomposeStepScore(along, lateralDist).score;
}

FitnessComputer::ScoreTerms FitnessComputer::decomposeStepScore(double along, double lateralDist) const {
    // Polar (conical) form with directional distance scaling and clamped angle.
    //   distance     = sqrt(along² + lateralDist²)
    //   angle        = acos(-along / distance)   [0 = behind, π = ahead]
    //   angle_clamp  = min(angle, π/2)           [ahead saturates at "sideways"]
    //   dist_scale   = behind or ahead based on along sign
    //   score        = 1 / (1 + (dist/dist_scale)² + (angle_clamp/cone)²)
    //
    // The clamp at π/2 prevents the angle term from saturating ahead, so the
    // (small) ahead distance scale carries the gradient when overshooting.
    ScoreTerms t;
    t.ahead = (along > 0.0);
    double distance = std::sqrt(along * along + lateralDist * lateralDist);
    if (distance < 1e-6) {
        t.score = 1.0; t.distTermSq = 0.0; t.angleTermSq = 0.0;
        return t;  // At the rabbit position
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
    t.distTermSq  = effDist * effDist;
    t.angleTermSq = effAngle * effAngle;
    t.score = 1.0 / (1.0 + t.distTermSq + t.angleTermSq);
    return t;
}

double FitnessComputer::applyStreak(double stepPoints) {
    // Streak state update is THRESHOLD-based on stepPoints. Operator-visible
    // intent: a sluggish-but-on-rabbit controller still earns streak credit.
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

// 041 P2-2 (FR-039) — see the derivation in fitness_computer.h.
gp_vec3 FitnessComputer::scoreGradientWorld(const gp_vec3& offset,
                                            const gp_vec3& tangent) const {
    const double d = static_cast<double>(offset.norm());
    if (d < 1e-6) return gp_vec3::Zero();  // at the rabbit: score is maximal

    const double along = static_cast<double>(offset.dot(tangent));
    double cosAngle = -along / d;
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    const double angle = std::acos(cosAngle);

    constexpr double HALF_PI = M_PI / 2.0;
    const bool clamped = (angle >= HALF_PI);
    const double angleClamped = clamped ? HALF_PI : angle;

    const double distScale = (along <= 0.0) ? distScaleBehind_ : distScaleAhead_;
    const double effDist = d / distScale;
    const double effAngle = angleClamped / coneAngleRad_;
    const double D = 1.0 + effDist * effDist + effAngle * effAngle;

    // Radial part: (2d/S²)·û
    const gp_vec3 u = offset / static_cast<gp_scalar>(d);
    gp_vec3 grad = u * static_cast<gp_scalar>(2.0 * d / (distScale * distScale));

    // Tangential part: (2θc/C²)·∇θ — zero where the angle is clamped, because
    // the objective's own angle term is flat there.
    if (!clamped) {
        const double lateral = std::sqrt(std::max(0.0, d * d - along * along));
        if (lateral > 1e-9) {
            const gp_vec3 gradTheta =
                tangent / static_cast<gp_scalar>(lateral) -
                u * static_cast<gp_scalar>(along / (d * lateral));
            grad += gradTheta * static_cast<gp_scalar>(
                        2.0 * angleClamped / (coneAngleRad_ * coneAngleRad_));
        }
    }

    // score = 1/D ⇒ ∇score = −(1/D²)·∇D, and `grad` above is ∇D.
    return grad * static_cast<gp_scalar>(-1.0 / (D * D));
}

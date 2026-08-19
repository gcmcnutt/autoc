#ifndef FITNESS_COMPUTER_H
#define FITNESS_COMPUTER_H

#include "autoc/types.h"

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

    // 030 M11.wrap diagnostics — decompose stepScore into its terms so callers
    // can attribute "why score is low" (geom.too_far vs geom.angle vs ahead).
    // distTermSq = (distance / distScale)^2  (uses distScaleAhead when along > 0)
    // angleTermSq = (angle_clamped / coneAngle)^2
    // score is the same value computeStepScore returns.
    struct ScoreTerms {
        double score;       // 1 / (1 + distTermSq + angleTermSq)
        double distTermSq;  // (dist / distScale)^2
        double angleTermSq; // (angle_clamped / coneAngle)^2
        bool ahead;         // along > 0 (chase forward of rabbit)
    };
    ScoreTerms decomposeStepScore(double along, double lateralDist) const;

    // 041 P2-2 (FR-039) — ∂score/∂position, WORLD frame, per metre.
    //
    // Lives here, next to the surface it differentiates, for the same reason
    // specific_force.h is one header: the gradient and the score must be
    // derived from the same constants. Split across files they would agree
    // today and disagree the first time a cone knob moves.
    //
    // The derivation, because the closed form is not obvious and a wrong one is
    // undetectable in flight. With `u = offset/|offset|` (rabbit→chase unit
    // vector) and `t` the path tangent:
    //
    //     score = 1/D,  D = 1 + (d/S)² + (θc/C)²
    //     ∇d     = u
    //     ∇θ     = t/L − (along/(d·L))·u,   with |∇θ| = 1/d
    //     ∇score = −(1/D²)·[ (2d/S²)·∇d + (2θc/C²)·∇θ ]
    //
    // The two parts are ORTHOGONAL (∇θ·u = (along/d)/L − along/(d·L) = 0),
    // which is the sanity check worth keeping: an angle changes only for motion
    // ACROSS the radius, never along it.
    //
    // ⚠️ The angle term VANISHES where θ is clamped at π/2 — ahead and off to
    // the side, `decomposeStepScore` saturates the angle deliberately so the
    // sharp ahead distance scale carries the gradient. Differentiating the
    // unclamped angle there would invent a gradient the objective does not have.
    //
    // ⚠️ The result is UNBOUNDED as `d → 0`, because |∇θ| = 1/d. That is real,
    // not a defect: measured on 131,127 t1 ticks, every sample above 2.0 had
    // d < 1.52 m. The CALLER bounds it — see kScoreGradScale's tanh-of-norm.
    // Returns zero for a degenerate offset (d < 1e-6), matching
    // decomposeStepScore's "at the rabbit position" branch, where the score is
    // at its maximum and there is no uphill direction.
    gp_vec3 scoreGradientWorld(const gp_vec3& offset, const gp_vec3& tangent) const;

    // Update streak state and return stepPoints × streak_multiplier.
    //
    // Streak: increments if stepPoints >= threshold, hard-resets otherwise.
    double applyStreak(double stepPoints);

    // Reset streak and diagnostics (call at start of each scenario).
    void resetStreak();

    // Diagnostics
    int getMaxStreak() const { return maxStreak_; }
    int getStreakSteps() const { return totalStreakSteps_; }
    int getStreakCount() const { return streakCount_; }
    double getMaxMultiplier() const { return maxMultiplier_; }
    double getStreakThreshold() const { return streakThreshold_; }

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

// 030 M7b — Trail rabbit implementation (FR-008 + Session 2026-05-07 Q1).
//
// Single-line computation per the simplified spec: target position minus
// trail_distance along velocity unit vector. Degenerate case (|velocity|
// near zero) collapses to target position.

#include "autoc/eval/trail_rabbit.h"

namespace autoc::eval {

gp_vec3 computeTrailRabbit(const SourceTickSample& target,
                            gp_scalar trail_distance) {
    const gp_scalar speed = target.velocity.norm();
    if (speed < kDegenerateVelocityThreshold) {
        // Degenerate case: target stationary. Rabbit collapses to
        // target position itself; chase fitness gradient still points
        // toward target (no separate nose-direction fallback in v1 —
        // deferred to 031 backlog per Session 2026-05-07 Q1).
        return target.position;
    }
    const gp_vec3 velocity_unit = target.velocity / speed;
    return target.position - velocity_unit * trail_distance;
}

}  // namespace autoc::eval

// 030 M7c — Crash hull implementation (FR-008b + R3).

#include "autoc/eval/crash_hull.h"

#include <algorithm>
#include <cstdint>

namespace autoc::eval {

bool isInsideHull(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const gp_vec3& target_pos) {
    switch (hull.shape) {
        case CrashHullShape::SPHERE: {
            const gp_vec3 delta = chase_pos - target_pos;
            const gp_scalar dist_sq = delta.squaredNorm();
            const gp_scalar radius_sq = hull.sphere_radius_m * hull.sphere_radius_m;
            return dist_sq <= radius_sq;
        }
        case CrashHullShape::AABB_HB1:
        case CrashHullShape::MESH_AIRFRAME:
        default:
            // Reserved shapes; v1 returns false (no hull strike). Future
            // milestones can add real implementations without breaking
            // the API.
            return false;
    }
}

namespace {

// Park-Miller LCG step. Deterministic, portable, no allocation; takes
// caller's prng_state by reference so each scenario's seed evolves
// across calls without contaminating other scenarios' PRNG streams.
// Returns a uniform float in [0, 1).
inline gp_scalar lcgUniform01(uint32_t& state) {
    // Park-Miller minimal standard: state = state * 48271 mod (2^31 - 1).
    constexpr uint64_t kModulus = 2147483647ULL;  // 2^31 - 1
    constexpr uint64_t kMultiplier = 48271ULL;
    state = static_cast<uint32_t>(
        (static_cast<uint64_t>(state) * kMultiplier) % kModulus);
    // Map to [0, 1).
    return static_cast<gp_scalar>(state) / static_cast<gp_scalar>(kModulus);
}

}  // namespace

bool didCrashFire(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const gp_vec3& target_pos,
                  gp_scalar p_crash_this_tick,
                  uint32_t& prng_state) {
    if (!isInsideHull(hull, chase_pos, target_pos)) {
        // Outside hull: no crash regardless of p_crash. Don't burn a
        // PRNG draw (preserves determinism across scenarios that hit
        // hull at different ticks).
        return false;
    }
    // Inside hull: draw Bernoulli(p_crash).
    if (p_crash_this_tick <= static_cast<gp_scalar>(0)) return false;
    if (p_crash_this_tick >= static_cast<gp_scalar>(1)) return true;
    const gp_scalar u = lcgUniform01(prng_state);
    return u < p_crash_this_tick;
}

}  // namespace autoc::eval

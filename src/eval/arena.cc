// 030 M7a — Arena ray-projection (FR-016 + Session 2026-05-07 Q1).
//
// `distanceToBoundary` solves three intersection problems and returns
// the minimum positive `t`:
//   - cylinder wall: quadratic in t (horizontal ray-circle intersection)
//   - floor: linear in t (z(t) = floor raw NED)
//   - ceiling: linear in t (z(t) = ceiling raw NED)
//
// All in the virtual frame (matches AircraftState::position). Floor /
// ceiling AGL converted to virtual z via SIM_INITIAL_ALTITUDE.

#include "autoc/eval/arena.h"

#include <algorithm>
#include <cmath>

namespace autoc::eval {

namespace {

// Solve quadratic at² + bt + c = 0 for smallest positive root, or
// `kSafeBoundaryDistance` if no positive real root.
gp_scalar smallestPositiveRoot(gp_scalar a, gp_scalar b, gp_scalar c) {
    if (std::abs(a) < static_cast<gp_scalar>(1e-12)) {
        // Degenerate to linear: bt + c = 0
        if (std::abs(b) < static_cast<gp_scalar>(1e-12)) {
            return kSafeBoundaryDistance;
        }
        const gp_scalar t = -c / b;
        return (t > 0) ? t : kSafeBoundaryDistance;
    }
    const gp_scalar disc = b * b - 4 * a * c;
    if (disc < 0) return kSafeBoundaryDistance;
    const gp_scalar sqrtDisc = std::sqrt(disc);
    const gp_scalar t1 = (-b - sqrtDisc) / (2 * a);
    const gp_scalar t2 = (-b + sqrtDisc) / (2 * a);
    // Smallest positive root.
    gp_scalar best = kSafeBoundaryDistance;
    if (t1 > 0 && t1 < best) best = t1;
    if (t2 > 0 && t2 < best) best = t2;
    return best;
}

}  // namespace

gp_scalar distanceToBoundary(const gp_vec3& chase_pos,
                             const gp_vec3& chase_velocity,
                             const FlightArena& arena) {
    const gp_scalar speed = chase_velocity.norm();
    if (speed < static_cast<gp_scalar>(1e-6)) {
        // Velocity ~zero — ray-projection undefined. Stationary chase
        // can't approach any boundary at any rate. Sentinel.
        return kSafeBoundaryDistance;
    }
    const gp_vec3 d = chase_velocity / speed;  // unit velocity vector

    // Cylinder wall: solve |horizontal_pos(t)|² = radius²
    //   (px + dx·t)² + (py + dy·t)² = R²
    //   t² · (dx² + dy²) + 2t · (px·dx + py·dy) + (px² + py² − R²) = 0
    const gp_scalar a_cyl = d.x() * d.x() + d.y() * d.y();
    const gp_scalar b_cyl = 2 * (chase_pos.x() * d.x() + chase_pos.y() * d.y());
    const gp_scalar c_cyl =
        chase_pos.x() * chase_pos.x() + chase_pos.y() * chase_pos.y()
        - arena.radius_m * arena.radius_m;
    gp_scalar t_wall = smallestPositiveRoot(a_cyl, b_cyl, c_cyl);

    // Floor / ceiling: linear ray. virtual_z + SIM_INITIAL_ALTITUDE = raw_z.
    // floor at raw_z = -floor_agl_m  → virtual_z_floor   = -floor_agl_m   - SIM_INITIAL_ALTITUDE
    // ceiling at raw_z = -ceiling_agl_m → virtual_z_ceil = -ceiling_agl_m - SIM_INITIAL_ALTITUDE
    const gp_scalar virt_z_floor   = -arena.floor_agl_m   - SIM_INITIAL_ALTITUDE;
    const gp_scalar virt_z_ceiling = -arena.ceiling_agl_m - SIM_INITIAL_ALTITUDE;
    // (floor virtual_z > ceiling virtual_z because +z = down; floor is "lower altitude".)

    auto rayHitsZ = [&](gp_scalar target_virt_z) -> gp_scalar {
        // dz·t + (pz - target) = 0 → t = (target - pz) / dz
        if (std::abs(d.z()) < static_cast<gp_scalar>(1e-12)) {
            return kSafeBoundaryDistance;  // ray parallel to z=const plane
        }
        const gp_scalar t = (target_virt_z - chase_pos.z()) / d.z();
        return (t > 0) ? t : kSafeBoundaryDistance;
    };

    const gp_scalar t_floor = rayHitsZ(virt_z_floor);
    const gp_scalar t_ceil = rayHitsZ(virt_z_ceiling);

    return std::min({t_wall, t_floor, t_ceil, kSafeBoundaryDistance});
}

gp_vec3 inwardBodyDirection(const gp_vec3& chase_pos,
                            const gp_quat& chase_orientation) {
    // World-frame radial-inward points from the chase's horizontal position
    // toward the axis (0, 0); the arena axis is vertical so z stays zero.
    const gp_scalar horiz = std::sqrt(chase_pos.x() * chase_pos.x() +
                                      chase_pos.y() * chase_pos.y());
    if (horiz < static_cast<gp_scalar>(1e-6)) {
        return gp_vec3::Zero();  // on-axis — inward direction undefined
    }
    const gp_vec3 inward_world(-chase_pos.x() / horiz,
                               -chase_pos.y() / horiz,
                               static_cast<gp_scalar>(0));
    // world→body rotation (see header — .inverse() convention).
    return chase_orientation.inverse() * inward_world;
}

}  // namespace autoc::eval

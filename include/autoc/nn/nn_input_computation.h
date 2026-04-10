#pragma once
#include "autoc/types.h"  // for gp_vec3, gp_scalar

constexpr float DIR_NUMERICAL_FLOOR = 1e-4f;  // meters

// Compute unit target direction vector in body frame.
// Normal case: target_body / dist.
// Singularity (dist < DIR_NUMERICAL_FLOOR): returns rabbit_vel_dir_body.
// No trig calls — eliminates atan2 without introducing cos/sin.
inline gp_vec3 computeTargetDir(const gp_vec3& target_body, float dist,
                                 const gp_vec3& rabbit_vel_dir_body) {
    if (dist < DIR_NUMERICAL_FLOOR) {
        return rabbit_vel_dir_body;
    }
    return target_body * (1.0f / dist);
}

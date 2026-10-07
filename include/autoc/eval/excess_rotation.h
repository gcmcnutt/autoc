// 043 T102 — EXCESS ROTATION over what the path demands.
//
// The measure the operator asked for (2026-10-05): "a smooth line in any attitude
// should fly smooth, an abrupt turn should turn hard … the measure is more about
// wandering around paths." So the baseline is the PATH, not a fixed threshold:
//
//     craft_rotation = Σ_t ‖ω_body(t)‖ · dt           (rad the airframe rotated)
//     path_rotation  = Σ_t ∠(tangent_t, tangent_{t−1})  (rad the path's direction turned)
//     excess         = craft_rotation − path_rotation   (lower = better)
//
// Properties that matter:
//  * No tunable parameter. A straight path demands ~0 and any rotation is
//    excess; a hard turn demands a lot and is free.
//  * In tracker mode the scorer takes `tangent` from the TARGET's velocity, so a
//    bang-bang target simply raises the demand — the chase is charged only for
//    what it adds. The same axis carries from M1 to M2 unchanged.
//  * Not Δ=0-gameable (015's `Σ|Δu|` exploit: a pegged elevator still rotates
//    the airframe) and not throttle-gameable (041's Es-destroyed exploit).
//  * ‖ω‖ uses all three body axes so a coordinated turn's rotation is compared
//    like-for-like with the path's direction change; roll-in/roll-out remains
//    an honest, unavoidable residual — this is a RELATIVE measure for lexicase.
//
// Measured on 043-t3's training table before any selection pressure: the
// craft rotated 4.9× the path's demand (straight-and-level 5.4×, 45° loop 3.1×).
#pragma once

#include <algorithm>
#include <cmath>

#include "autoc/eval/aircraft_state.h"  // gp_vec3, gp_scalar

namespace autoc {
namespace eval {

struct ExcessRotationAccum {
  double craft_rotation = 0.0;  // rad
  double path_rotation = 0.0;   // rad
  bool have_prev = false;
  gp_vec3 prev_tangent = gp_vec3::Zero();

  // One recorded tick. `tangent` must be unit length (the scorer normalises it
  // before calling); `gyro_body` is rad/s; `dt` is the tick spacing in seconds.
  void step(const gp_vec3& tangent, const gp_vec3& gyro_body, double dt) {
    if (have_prev) {
      const double c = std::clamp(static_cast<double>(tangent.dot(prev_tangent)), -1.0, 1.0);
      path_rotation += std::acos(c);
    }
    prev_tangent = tangent;
    have_prev = true;
    craft_rotation += static_cast<double>(gyro_body.norm()) * dt;
  }

  double excess() const { return craft_rotation - path_rotation; }
};

}  // namespace eval
}  // namespace autoc

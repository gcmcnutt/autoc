#ifndef AUTOC_EVAL_SPECIFIC_FORCE_H
#define AUTOC_EVAL_SPECIFIC_FORCE_H

// 041 T010/T039 — body-frame specific force, in ONE place.
//
// WHY A SHARED HEADER. Two consumers need this quantity and they must not be
// able to disagree: `tools/dmp_dump.cc --physics` reports it (T010) and
// `src/nn/evaluator.cc` feeds it to the network as ACCEL_X/Y/Z (T039). If the
// reader and the input drifted apart, Study A would be measuring a different
// quantity from the one the policy sees, and nothing would say so.
//
// ⚠️ SPECIFIC FORCE, NOT KINEMATIC ACCELERATION. A real accelerometer measures
// (a - g); the FDM's `acc` is a alone. Feeding the latter puts a constant ~1 g
// error into the most load-relevant axis — invisible in sim, wrong in flight.
// This is called out in contracts/nn-input-layout.md as the single most
// error-prone line in the feature, and is pinned by a test at both consumers.
//
// Frames (docs/COORDINATE_CONVENTIONS.md):
//   world  NED — +z is DOWN, so gravity is (0, 0, +g)
//   body   x forward, y right, z down
//   quat   q_EB, earth->body; world->body is `q.inverse() * v`

#include "autoc/types.h"

namespace autoc {
namespace eval {

struct BodySpecificForce {
    // Body-frame specific force in g units: (a_world - g_world) rotated into
    // body and divided by g. Steady level flight reads (0, 0, -1) because body
    // +z points DOWN while the measured reaction points UP.
    gp_vec3 g_units;

    // Aviation load factor: +1 in steady level flight, >1 in a pull-up,
    // negative in a push-over. This is `-g_units.z()`, and the negation exists
    // for ONE reason: so the number reads the way loads are quoted in the
    // flight reports ("+11.2 g / -8.4 g").
    //
    // ⚠️ This is a HUMAN-FACING quantity. The NN input ACCEL_Z is `g_units.z()`
    // — un-negated, so level flight feeds the network -1, not +1. Do not
    // "correct" that against INAV's bench table (level reads +1 g on its normal
    // axis): INAV's frame is FLU, ours is FRD, and msplink flips y/z at the
    // boundary exactly as it does for the quat and the gyro. Same physical
    // fact, two frames. Settled 2026-08-11 — see docs/COORDINATE_CONVENTIONS.md
    // -> "Accelerometer as an INTERFACE quantity (041)".
    gp_scalar load_factor_nz;
};

// `gravity` is the magnitude in the SAME units as `acc_world`. The caller
// passes the recorded value rather than a literal 9.81 because the FDM trace is
// in LaRCSim-native (ft-based) units — dividing by the recorded magnitude makes
// the result unit-free either way. A non-positive gravity returns zeros: there
// is no defensible answer, and guessing one is the failure mode this header
// exists to prevent.
inline BodySpecificForce bodySpecificForce(const gp_vec3& acc_world,
                                           const gp_quat& q_eb,
                                           gp_scalar gravity) {
    BodySpecificForce out;
    out.g_units = gp_vec3::Zero();
    out.load_factor_nz = static_cast<gp_scalar>(0);
    if (!(gravity > static_cast<gp_scalar>(0))) return out;

    const gp_vec3 g_world(static_cast<gp_scalar>(0),
                          static_cast<gp_scalar>(0),
                          gravity);
    const gp_vec3 sf_body = q_eb.inverse() * (acc_world - g_world);
    out.g_units = sf_body / gravity;
    out.load_factor_nz = -out.g_units.z();
    return out;
}

}  // namespace eval
}  // namespace autoc

#endif  // AUTOC_EVAL_SPECIFIC_FORCE_H

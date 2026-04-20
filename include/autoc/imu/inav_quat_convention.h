// INAV quaternion → aerospace q_EB convention conversion (static lookup only).
//
// Background (see docs/COORDINATE_CONVENTIONS.md):
//
// INAV's AHRS publishes an orientation quaternion that represents airframe
// attitude in INAV's own conventions — specifically, pitch is nose-down-
// positive and yaw is nose-left-positive (N→E negative). This matches
// INAV's Euler `attitude[]` and raw `gyroADCf[]` streams (pitch rate
// positive = nose down, yaw rate positive = N→W). Board alignment
// (`align_board_roll/pitch/yaw`) is already applied inside INAV before
// these values leave the FC, so the numbers arriving at xiao are
// airframe-body-frame — not board-frame.
//
// Our documented convention (docs/COORDINATE_CONVENTIONS.md) is aerospace
// NED / FRD / RHR: pitch positive = nose UP, roll positive = right-wing
// DOWN, yaw positive = nose RIGHT (N→E). To bring the INAV quat into
// conformance we must flip the components whose rotation axes have
// opposite sign in the two conventions:
//   qx (roll axis): same sign  → pass through unchanged
//   qy (pitch axis): opposite  → flip
//   qz (yaw axis):  opposite  → flip
//
// Bench verification on this FC (2026-04-19, post board-alignment 1700/0/900):
//   Nose up 30°:        raw qy = −0.2  → we want +0.26 (aerospace nose-up-pos)
//   Nose down 30°:      raw qy = +0.25 → we want −0.26
//   Right wing down 30°: raw qx = +0.2  → we want +0.26 (already correct)
//   Nose east 90°:      raw qz = −0.7  → we want +0.71 (aerospace N→E-pos)
//
// WARNING — not kinematically valid. This transform mixes sign flips on
// two different components, which is a reflection when viewed from the
// rotation-group side. The result is correct for STATIC attitude lookup
// (Euler extraction, direction-cosine computation, body-axis-in-world
// rotation) but NOT for dq/dt = 0.5·q·ω_body integration. Do not feed
// the output into Kalman/Mahony/Madgwick filters or any consumer that
// also integrates body rates to update attitude. Gyros must receive the
// matching pitch/yaw sign flips downstream (convertMSPStateToAircraftState
// handles that for NN consumption).

#pragma once

#include <autoc/types.h>

namespace autoc {
namespace imu {

// Convert an INAV-sent quaternion (body→earth in INAV's pitch-nose-down-
// positive / yaw-N→W-positive airframe convention) into an aerospace
// q_EB (earth→body, NED world, RHR) suitable for STATIC lookup.
//
// Input: INAV wire format q[0..3] = (w, x, y, z), scalar-first.
// Output: aerospace q_EB as gp_quat.
//
// See file header: output is NOT kinematically valid.
inline gp_quat inavQuatToAerospaceEB(const float q[4]) {
  // Flip qy (pitch axis) and qz (yaw axis). Keep qw/qx.
  gp_quat out(q[0], q[1], -q[2], -q[3]);
  if (out.norm() == 0.0f) {
    return gp_quat::Identity();
  }
  out.normalize();
  return out;
}

}  // namespace imu
}  // namespace autoc

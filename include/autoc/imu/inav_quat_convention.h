// INAV quaternion → aerospace q_EB convention conversion (static lookup only).
//
// Background (see docs/COORDINATE_CONVENTIONS.md "INAV NEU ↔ aerospace NED
// reflection" for the full rationale):
//
// INAV stores its orientation quaternion in its internal NEU world frame.
// Our aerospace stack uses NED. NEU→NED is a REFLECTION of the world-Z axis,
// NOT a rotation — and quaternions cannot represent reflections.
//
// There is therefore no single static transform that is both:
//   (a) correct for static attitude look-ups (Euler extraction, direction
//       cosine computation, body-axis-in-world rotation)
//   (b) kinematically valid (dq/dt = 0.5·q·ω_body preserved)
//
// This function delivers (a) — suitable for feeding the NN static inputs
// (direction cosines, quat components as features) and for rendering /
// display. DO NOT use the result in a Kalman/Mahony/Madgwick filter or any
// integration that consumes body rates to update attitude. For those, use
// the raw INAV quat + raw INAV gyro together, and apply this flip only at
// the consumer boundary.
//
// Empirical derivation (2026-04-19, flight-20260417 analysis):
// - Accel at rest (pre-takeoff): measured gravity in body gives physical
//   pitch=-7.2°, roll=-3.3°.
// - Raw INAV quat (no transform) Euler extraction (using aerospace q_EB
//   formulas): pitch=-8.1°, roll=-5.5°. Matches in sign within ~2° (accel
//   noise tolerance). Confirms raw INAV qx and qy match aerospace sign.
// - Mid-flight yaw-from-raw-quat consistently 180° off from ground-track
//   direction. Flipping qz brings agreement within ~10° (declination+wind).
// - Bench compound-attitude verification is the final definitive test
//   (024 WI5 T100-T104).

#pragma once

#include <autoc/types.h>

namespace autoc {
namespace imu {

// Convert an INAV-sent quaternion (body→earth, NEU world, with INAV's
// internal conventions) into an aerospace q_EB (earth→body, NED world,
// right-hand-rule) suitable for STATIC lookup.
//
// Input: INAV wire format q[0..3] = (w, x, y, z), scalar-first.
// Output: aerospace q_EB as gp_quat.
//
// See file header: output is NOT kinematically valid.
inline gp_quat inavQuatToAerospaceEB(const float q[4]) {
  // Flip qz only (NEU↔NED world-Z reflection). Keep qw/qx/qy.
  // Pitch/roll axes are horizontal — unaffected by the reflection.
  gp_quat out(q[0], q[1], q[2], -q[3]);
  if (out.norm() == 0.0f) {
    return gp_quat::Identity();
  }
  out.normalize();
  return out;
}

}  // namespace imu
}  // namespace autoc

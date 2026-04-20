// T040/T042 — bench-derived harness for INAV quat → aerospace q_EB
// conversion used at the msplink boundary.
//
// Evidence (bench 2026-04-19, this FC, post-alignment-by-INAV):
//   Nose up 30°        : raw INAV qy = -0.2   → aerospace expects +sin(15°) = +0.26
//   Nose down 30°      : raw INAV qy = +0.25  → aerospace expects -0.26
//   Right wing down 30°: raw INAV qx = +0.2   → aerospace expects +0.26  (match)
//   Nose east 90°      : raw INAV qz = -0.7   → aerospace expects +sin(45°) = +0.71
//   Compound (nose up 30°, RWD 30°): raw (0.92, +0.26, -0.3, 0)
//                                    → aerospace (0.93, +0.25, +0.25, -0.07)
//
// Conclusion: raw INAV airframe-frame quat has qy and qz sign-inverted vs
// aerospace (qx and qw are already aerospace-compatible). The boundary
// transform is therefore (w, x, -y, -z).

#include <gtest/gtest.h>
#include <cmath>

#include "autoc/imu/inav_quat_convention.h"
#include "autoc/types.h"

namespace {

constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

// Return the rotation matrix element that projects world-Z (gravity vector
// in NED (0,0,+1)) into body-frame Z component. If attitude is mostly level
// upright, this should be close to +1 (belly points down).
float bodyGravityZFromQuat(const gp_quat& q_EB) {
  const gp_vec3 world_gravity(0.0f, 0.0f, 1.0f);  // NED: Z down
  const gp_vec3 body_g = q_EB * world_gravity;  // q_EB rotates world -> body (Hamilton via Eigen)
  return body_g.z();
}

// Extract aerospace pitch (nose-up positive) from q_EB.
float pitchDeg(const gp_quat& q_EB) {
  const float w = q_EB.w(), x = q_EB.x(), y = q_EB.y(), z = q_EB.z();
  float s = 2.0f * (w * y - x * z);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return std::asin(s) / kDegToRad;
}

// Extract aerospace roll (right-wing-down positive) from q_EB.
float rollDeg(const gp_quat& q_EB) {
  const float w = q_EB.w(), x = q_EB.x(), y = q_EB.y(), z = q_EB.z();
  return std::atan2(2.0f * (y * z + w * x),
                    w * w - x * x - y * y + z * z) / kDegToRad;
}

// Extract aerospace yaw (nose-right positive) from q_EB.
float yawDeg(const gp_quat& q_EB) {
  const float w = q_EB.w(), x = q_EB.x(), y = q_EB.y(), z = q_EB.z();
  return std::atan2(2.0f * (x * y + w * z),
                    w * w + x * x - y * y - z * z) / kDegToRad;
}

}  // namespace

// ----------------------------------------------------------------------------
// Contract tests — empirically-grounded expectations
// ----------------------------------------------------------------------------

TEST(InavQuatConvention, IdentityIsIdentity) {
  // Level attitude, nose north: INAV sends identity → we return identity.
  const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  gp_quat out = autoc::imu::inavQuatToAerospaceEB(q);
  EXPECT_NEAR(out.w(), 1.0f, 1e-6f);
  EXPECT_NEAR(out.x(), 0.0f, 1e-6f);
  EXPECT_NEAR(out.y(), 0.0f, 1e-6f);
  EXPECT_NEAR(out.z(), 0.0f, 1e-6f);
}

TEST(InavQuatConvention, PureNoseUpBenchSample) {
  // Bench 2026-04-19: physically hold nose up 30°. Raw INAV airframe quat
  // reads (1, 0, -0.2, 0) (qy negative in INAV's nose-down-positive pitch
  // convention). After our (w, x, -y, -z) boundary transform, qy flips to
  // positive and aerospace Euler extraction gives nose-up.
  const float s = std::sin(15.0f * kDegToRad);
  const float c = std::cos(15.0f * kDegToRad);
  const float q_raw[4] = {c, 0.0f, -s, 0.0f};  // INAV: nose-up with qy negative
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(pitchDeg(q_eb), +30.0f, 1.0f)
      << "nose-up 30° should extract aerospace pitch +30° after qy flip";
  EXPECT_NEAR(rollDeg(q_eb), 0.0f, 1.0f);
  EXPECT_NEAR(yawDeg(q_eb), 0.0f, 1.0f);
}

TEST(InavQuatConvention, QyAndQzAreFlippedFromRaw) {
  // Transform spec: flip qy AND qz; pass qw and qx through. Verify
  // component-wise with a unit-normalized input.
  const float n = std::sqrt(0.64f + 0.01f + 0.04f + 0.09f);  // ≈ 0.883
  const float q_raw[4] = {0.8f / n, 0.1f / n, 0.2f / n, 0.3f / n};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(q_eb.w(),  0.8f / n, 1e-4f);
  EXPECT_NEAR(q_eb.x(),  0.1f / n, 1e-4f);
  EXPECT_NEAR(q_eb.y(), -0.2f / n, 1e-4f);
  EXPECT_NEAR(q_eb.z(), -0.3f / n, 1e-4f);
}

TEST(InavQuatConvention, PureYawEastFlipsYawSign) {
  // Raw INAV yaw-east 90° has qz=-sin(45°) (yaw-N→W-positive in INAV).
  // After the boundary flip qz becomes +sin(45°), aerospace yaw +90°.
  const float s = std::sin(45.0f * kDegToRad);
  const float c = std::cos(45.0f * kDegToRad);
  const float q_raw[4] = {c, 0.0f, 0.0f, -s};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(yawDeg(q_eb), +90.0f, 1.0f)
      << "nose-east should extract aerospace yaw +90° after qz flip";
  EXPECT_NEAR(pitchDeg(q_eb), 0.0f, 1.0f);
  EXPECT_NEAR(rollDeg(q_eb), 0.0f, 1.0f);
}

TEST(InavQuatConvention, PureRollRightWingDownPassesThrough) {
  // Raw INAV right-wing-down 30° has qx = +sin(15°), same sign as aerospace
  // (roll axis is nominally the same in both). Verify the boundary does
  // not touch qx.
  const float s = std::sin(15.0f * kDegToRad);
  const float c = std::cos(15.0f * kDegToRad);
  const float q_raw[4] = {c, s, 0.0f, 0.0f};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(rollDeg(q_eb), +30.0f, 1.0f)
      << "right-wing-down 30° should extract aerospace roll +30°";
  EXPECT_NEAR(pitchDeg(q_eb), 0.0f, 1.0f);
  EXPECT_NEAR(yawDeg(q_eb), 0.0f, 1.0f);
}

TEST(InavQuatConvention, CompoundNoseUpRightWingDownBenchSample) {
  // Bench 2026-04-19: physically pitch up 30° + right wing down 30° (yaw 0).
  // Raw INAV quat observed ≈ (0.92, +0.26, -0.3, 0).
  // After (w, x, -y, -z): (0.92, +0.26, +0.3, 0).
  // Expected aerospace q_EB ZYX (pitch +30, roll +30, yaw 0):
  //   w = cos(15°)·cos(15°)                    = 0.933
  //   x = cos(15°)·sin(15°)                    = 0.250
  //   y = sin(15°)·cos(15°)                    = 0.250
  //   z = -sin(15°)·sin(15°)                   = -0.067
  const float c = std::cos(15.0f * kDegToRad);
  const float s = std::sin(15.0f * kDegToRad);
  const float w_aero =  c * c;
  const float x_aero =  c * s;
  const float y_aero =  s * c;
  const float z_aero = -s * s;

  // Raw INAV for same physical attitude: qy and qz flipped vs aerospace.
  const float q_raw[4] = {w_aero, x_aero, -y_aero, -z_aero};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(pitchDeg(q_eb), +30.0f, 1.0f);
  EXPECT_NEAR(rollDeg(q_eb),  +30.0f, 1.0f);
  EXPECT_NEAR(yawDeg(q_eb),     0.0f, 1.0f);

  EXPECT_NEAR(q_eb.w(), w_aero, 1e-4f);
  EXPECT_NEAR(q_eb.x(), x_aero, 1e-4f);
  EXPECT_NEAR(q_eb.y(), y_aero, 1e-4f);
  EXPECT_NEAR(q_eb.z(), z_aero, 1e-4f);
}

TEST(InavQuatConvention, ZeroInputDoesNotNaN) {
  // Degenerate zero quat (shouldn't happen in practice but the function
  // handles it gracefully).
  const float q_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);
  // Should fall back to identity.
  EXPECT_NEAR(q_eb.w(), 1.0f, 1e-6f);
  EXPECT_NEAR(q_eb.x(), 0.0f, 1e-6f);
  EXPECT_NEAR(q_eb.y(), 0.0f, 1e-6f);
  EXPECT_NEAR(q_eb.z(), 0.0f, 1e-6f);
}

TEST(InavQuatConvention, OutputIsUnitNormalized) {
  // Non-unit input should be normalized on output.
  const float q_raw[4] = {10.0f, 0.0f, 0.0f, 0.0f};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);
  EXPECT_NEAR(q_eb.norm(), 1.0f, 1e-5f);
}

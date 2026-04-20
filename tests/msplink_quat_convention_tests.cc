// T040 — bench-derived harness for the INAV quat → aerospace q_EB conversion
// used at the msplink boundary.
//
// These tests encode the aerospace convention we expect after applying the
// boundary fix (autoc::imu::inavQuatToAerospaceEB). Evidence driving these
// expectations:
//
// 1. At-rest accelerometer (flight-20260417 row 2, pre-takeoff):
//    accel body ≈ (+0.12, -0.056, +0.95) g with acc_1G=2050. Tilt:
//    pitch=-7.2° (nose-down), roll=-3.3° (right-wing-up). Using raw INAV
//    quat q=(.9706, -.0303, -.0798, +.2252), the aerospace-formula Euler
//    extraction gives pitch=-8.1°, roll=-5.5° — matches accel in sign
//    within accel noise. Confirms raw qx and qy match aerospace signs.
//
// 2. Mid-flight yaw vs ground-track: raw extracted yaw is consistently
//    180°-ish off from track direction. After (w, x, y, -z) transform,
//    yaw matches track within ~10° (declination + wind). Confirms qz
//    needs flip.
//
// 3. Compound-attitude bench (WI5 T100-T104) will close the loop with
//    known physical poses vs extracted Euler/cosines. These unit tests
//    pre-encode those expectations so the msplink boundary implementation
//    can be CI-gated against the convention.

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

TEST(InavQuatConvention, AtRestSampleMatchesAccelAttitude) {
  // flight-20260417 row 2 raw INAV quat (pre-takeoff on ground).
  // Accel measured gravity in body = (+0.12, -0.056, +0.95) g
  //   → pitch ≈ -7.2° (nose down), roll ≈ -3.3° (right wing up).
  // Our conversion should extract matching signs within a few degrees.
  const float q_raw[4] = {0.9706f, -0.0303f, -0.0798f, 0.2252f};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  // Pitch: expect roughly -7.2° (nose down). Allow ±3° for accel noise +
  // extraction+float rounding.
  EXPECT_NEAR(pitchDeg(q_eb), -7.2f, 3.0f)
      << "pitch from at-rest sample should match accel (-7.2°)";

  // Roll: expect roughly -3.3° (right wing up).
  EXPECT_NEAR(rollDeg(q_eb), -3.3f, 3.0f)
      << "roll from at-rest sample should match accel (-3.3°)";

  // Body-frame gravity Z should be close to +1 (upright).
  EXPECT_GT(bodyGravityZFromQuat(q_eb), 0.95f)
      << "belly should point mostly down in world Z (upright aircraft)";
}

TEST(InavQuatConvention, QzIsFlippedFromRaw) {
  // The core empirical finding: qz must be flipped (NEU→NED world-Z
  // reflection). Verify the conversion does exactly that and nothing else.
  // Use a unit-normalized input so no additional scaling happens.
  const float n = std::sqrt(0.64f + 0.01f + 0.04f + 0.09f);  // ≈ 0.883
  const float q_raw[4] = {0.8f / n, 0.1f / n, 0.2f / n, 0.3f / n};
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  EXPECT_NEAR(q_eb.w(),  0.8f / n, 1e-4f);
  EXPECT_NEAR(q_eb.x(),  0.1f / n, 1e-4f);
  EXPECT_NEAR(q_eb.y(),  0.2f / n, 1e-4f);
  EXPECT_NEAR(q_eb.z(), -0.3f / n, 1e-4f);
}

TEST(InavQuatConvention, PureYawEastFlipsYawSign) {
  // If raw INAV represents the aircraft heading east (in NEU's frame),
  // after our fix the yaw extraction should be aerospace +90° (nose east).
  //
  // INAV NEU sends a quat where pure +90° yaw (nose rotating from N to E
  // counter-clockwise from above in NEU) has qz=-sin(45°). After flip, qz
  // becomes +sin(45°), matching aerospace q_EB for +90° aerospace yaw.
  const float s = std::sin(45.0f * kDegToRad);
  const float c = std::cos(45.0f * kDegToRad);
  const float q_raw[4] = {c, 0.0f, 0.0f, -s};  // INAV: "yaw east" with qz negative
  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  // After aerospace extraction we should get +90° yaw.
  EXPECT_NEAR(yawDeg(q_eb), +90.0f, 1.0f)
      << "nose-east should extract aerospace yaw +90° after qz flip";
  // Pitch/roll should be zero.
  EXPECT_NEAR(pitchDeg(q_eb), 0.0f, 1.0f);
  EXPECT_NEAR(rollDeg(q_eb), 0.0f, 1.0f);
}

TEST(InavQuatConvention, CompoundAttitudeExtractsAerospaceEuler) {
  // Compound physical attitude: pitch +45° nose up, roll +30° right-wing-down,
  // yaw 0° nose-north. Aerospace q_EB (ZYX intrinsic):
  //   e_0 = cos(0)·cos(22.5°)·cos(15°) + sin(0)·sin(22.5°)·sin(15°)
  //   e_1 = cos(0)·cos(22.5°)·sin(15°) - sin(0)·sin(22.5°)·cos(15°)
  //   e_2 = cos(0)·sin(22.5°)·cos(15°) + sin(0)·cos(22.5°)·sin(15°)
  //   e_3 = -cos(0)·sin(22.5°)·sin(15°) + sin(0)·cos(22.5°)·cos(15°)
  // For yaw=0, e_3 = -sin(22.5°)·sin(15°) = -0.0991.
  //
  // INAV's NEU-world quat for the same physical attitude differs ONLY in
  // the sign of qz (world-Z axis flip). So raw INAV quat has qz=+0.0991.
  // Applying inavQuatToAerospaceEB (which flips qz) should yield the
  // aerospace q_EB above.
  const float theta = 45.0f * kDegToRad;
  const float phi = 30.0f * kDegToRad;
  const float ct = std::cos(theta * 0.5f), st = std::sin(theta * 0.5f);
  const float cp = std::cos(phi * 0.5f),   sp = std::sin(phi * 0.5f);
  // Aerospace q_EB for yaw=0:
  const float w_aero = ct * cp;
  const float x_aero = ct * sp;
  const float y_aero = st * cp;
  const float z_aero = -st * sp;
  // Raw INAV quat: aerospace with qz flipped.
  const float q_raw[4] = {w_aero, x_aero, y_aero, -z_aero};

  gp_quat q_eb = autoc::imu::inavQuatToAerospaceEB(q_raw);

  // After conversion, Euler extraction should give the physical attitude.
  EXPECT_NEAR(pitchDeg(q_eb), 45.0f, 1.0f);
  EXPECT_NEAR(rollDeg(q_eb),  30.0f, 1.0f);

  // Output should match aerospace q_EB components we constructed.
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

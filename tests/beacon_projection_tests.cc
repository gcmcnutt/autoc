// 030 M5 — Beacon projection contract tests (T025 + T026).
//
// Geometry: target dead-ahead → (x≈0, y≈0, cep≈0); target at FOV edge →
// screen_x near ±1, cep elevated; target behind → sentinel; target
// occluded by airframe proxy → sentinel; target outside emission cone
// (tail-on aspect) → sentinel.
//
// Int8 round-trip: dequantize_xy(quantize_xy(x)) within 1/127 of x for
// visible values; dequantize_cep(quantize_cep(any ≥ 1.25)) exactly equals
// kCepSentinelFloat (1.5f); dequantize_cep maps INT8_MIN to sentinel.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/types.h"

using autoc::eval::AirframeProxy;
using autoc::eval::BeaconConfig;
using autoc::eval::BeaconObservation;
using autoc::eval::CameraConfig;
using autoc::eval::ProjectionInput;
using autoc::eval::dequantize_cep;
using autoc::eval::dequantize_xy;
using autoc::eval::kCepSentinelFloat;
using autoc::eval::kCepSentinelThreshold;
using autoc::eval::projectBeacon;
using autoc::eval::quantize_cep;
using autoc::eval::quantize_xy;
using autoc::eval::rayHitsProxy;

namespace {

// Test scaffolding — chase at world origin, identity orientation; left-
// wingtip beacon mounted at (0, -0.45, 0) on a target whose orientation
// the test sets. The default airframe proxy is loose enough that beacons
// out in front of the chase clear it.
ProjectionInput makeBaselineInput() {
    ProjectionInput in;
    in.chase_position_world = gp_vec3(0.0f, 0.0f, 0.0f);
    in.chase_orientation_world = gp_quat::Identity();
    in.target_position_world = gp_vec3(10.0f, 0.0f, 0.0f);
    in.target_orientation_world = gp_quat::Identity();
    // Left wingtip beacon, outward emission.
    in.beacon_mount_target_body = gp_vec3(0.0f, -0.45f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(0.0f, -1.0f, 0.0f);
    in.camera_mount_chase_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.camera_orientation_chase_body = gp_quat::Identity();
    in.camera = CameraConfig{};
    in.beacon = BeaconConfig{};
    in.beacon.mount_body = in.beacon_mount_target_body;
    in.beacon.emission_axis_body = in.beacon_emission_axis_target_body;
    // Placeholder proxy parked far behind the camera so default test rays
    // (forward, +x) never trip self-occlusion. Specific occlusion tests
    // override `in.chase_airframe` directly.
    in.chase_airframe = AirframeProxy{
        gp_vec3(-100.0f, -1.0f, -1.0f),
        gp_vec3(-99.0f,  +1.0f, +1.0f)
    };
    return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// T025 — Geometry: target dead-ahead.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, TargetDeadAheadProducesCenteredObservation) {
    ProjectionInput in = makeBaselineInput();
    // Move beacon to target body origin so projection lands exactly on the
    // optical axis (no -0.45 m offset to bias screen_x).
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);  // toward chase

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    EXPECT_NEAR(obs.screen_x, 0.0f, 1.0f / 127.0f);
    EXPECT_NEAR(obs.screen_y, 0.0f, 1.0f / 127.0f);
    EXPECT_LT(obs.cep, 0.05f);
    EXPECT_NE(obs.raw_cep_int8, INT8_MIN);
}

// ---------------------------------------------------------------------------
// T025 — Geometry: target near FOV edge → screen_x approaches ±1, cep up.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, TargetAtRightFovEdgeApproachesPlusOne) {
    ProjectionInput in = makeBaselineInput();
    // 120° H FOV ⇒ tan(60°) ≈ 1.732. Target at body (10, 17.32, 0) puts
    // the beacon centroid at u = 17.32/10 = 1.732, exactly at the right
    // edge. Set beacon mount to target origin to remove the wingtip bias.
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, 17.32f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    EXPECT_NEAR(obs.screen_x, 1.0f, 0.01f);
    EXPECT_GT(obs.cep, 0.20f);  // edge_factor near 1 ⇒ cep near 0.3
}

TEST(BeaconProjectionGeometry, TargetAtLeftFovEdgeApproachesMinusOne) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, -17.32f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    EXPECT_NEAR(obs.screen_x, -1.0f, 0.01f);
    EXPECT_GT(obs.cep, 0.20f);
}

// ---------------------------------------------------------------------------
// T025 — Sentinel: target behind chase.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionSentinel, TargetBehindChaseEmitsSentinel) {
    ProjectionInput in = makeBaselineInput();
    in.target_position_world = gp_vec3(-10.0f, 0.0f, 0.0f);
    // Beacon at target origin so we isolate "behind camera" check.
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_FLOAT_EQ(obs.cep, kCepSentinelFloat);
    EXPECT_EQ(obs.raw_cep_int8, INT8_MIN);
}

// ---------------------------------------------------------------------------
// T025 — Sentinel: target outside FOV (just past the right edge).
// ---------------------------------------------------------------------------

TEST(BeaconProjectionSentinel, TargetOutsideHorizontalFovEmitsSentinel) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    // u = 25/10 = 2.5 > tan(60°) = 1.732 ⇒ outside FOV.
    in.target_position_world = gp_vec3(10.0f, 25.0f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_FLOAT_EQ(obs.cep, kCepSentinelFloat);
}

// ---------------------------------------------------------------------------
// T025 — Sentinel: airframe proxy occlusion.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionSentinel, AirframeProxyOccludesEmitsSentinel) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    // Target dead ahead; proxy box straddles the optical axis between
    // camera and target.
    in.target_position_world = gp_vec3(10.0f, 0.0f, 0.0f);
    in.chase_airframe = AirframeProxy{
        gp_vec3(2.0f, -0.5f, -0.5f),
        gp_vec3(4.0f, +0.5f, +0.5f)
    };

    BeaconObservation obs = projectBeacon(in);

    EXPECT_FLOAT_EQ(obs.cep, kCepSentinelFloat);
}

TEST(BeaconProjectionSentinel, AirframeProxyMissesDoesNotOcclude) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, 0.0f, 0.0f);
    // Proxy way off to the side — ray to (10, 0, 0) doesn't pass through.
    in.chase_airframe = AirframeProxy{
        gp_vec3(2.0f, +5.0f, +5.0f),
        gp_vec3(4.0f, +6.0f, +6.0f)
    };

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
}

// ---------------------------------------------------------------------------
// T025 — Sentinel: target outside emission cone (tail-on aspect).
// ---------------------------------------------------------------------------

TEST(BeaconProjectionSentinel, TargetOutsideEmissionConeEmitsSentinel) {
    // Left-wingtip beacon emits along target body -y. Place chase directly
    // off the target's right wing (target body +y direction). The chase is
    // 180° from the emission axis ⇒ outside the 270° cone (half-angle 135°).
    ProjectionInput in = makeBaselineInput();
    in.target_position_world = gp_vec3(0.0f, 0.0f, 0.0f);
    in.target_orientation_world = gp_quat::Identity();
    in.chase_position_world = gp_vec3(0.0f, 10.0f, 0.0f);
    // Chase looks back toward target (-y direction). Rotate chase 90° CCW
    // about body +z so chase body +x = world -y.
    {
        Eigen::AngleAxisf rot(M_PI / 2.0f, gp_vec3::UnitZ());
        in.chase_orientation_world = gp_quat(rot);
    }
    in.beacon_mount_target_body = gp_vec3(0.0f, -0.45f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(0.0f, -1.0f, 0.0f);
    in.beacon.mount_body = in.beacon_mount_target_body;
    in.beacon.emission_axis_body = in.beacon_emission_axis_target_body;

    BeaconObservation obs = projectBeacon(in);

    EXPECT_FLOAT_EQ(obs.cep, kCepSentinelFloat);
}

// ---------------------------------------------------------------------------
// T026 — int8 round-trip determinism.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionQuant, XyRoundTripWithinOneStep) {
    constexpr float kStep = 1.0f / 127.0f;                        // raw-ok: test scaffolding for NN-byte-format primitive
    const float samples[] = {-1.0f, -0.99f, -0.5f, -0.123f, 0.0f, // raw-ok: test scaffolding for NN-byte-format primitive
                             0.001f, 0.314f, 0.732f, 0.999f, 1.0f};
    for (float x : samples) {                                     // raw-ok: test scaffolding for NN-byte-format primitive
        const int8_t q = quantize_xy(x);
        const float r = dequantize_xy(q);                         // raw-ok: test scaffolding for NN-byte-format primitive
        EXPECT_NEAR(r, x, kStep) << "input " << x;
    }
}

TEST(BeaconProjectionQuant, XyClampsOutOfRangeInputs) {
    EXPECT_EQ(quantize_xy(2.5f), 127);
    EXPECT_EQ(quantize_xy(-2.5f), -127);
    EXPECT_FLOAT_EQ(dequantize_xy(127), 1.0f);
    EXPECT_FLOAT_EQ(dequantize_xy(-127), -1.0f);
}

TEST(BeaconProjectionQuant, CepVisibleRoundTripWithinOneStep) {
    constexpr float kStep = 1.0f / 127.0f;                        // raw-ok: test scaffolding for NN-byte-format primitive
    const float samples[] = {0.0f, 0.001f, 0.123f, 0.3f, 0.5f, 0.999f, 1.0f}; // raw-ok: test scaffolding for NN-byte-format primitive
    for (float c : samples) {                                     // raw-ok: test scaffolding for NN-byte-format primitive
        const int8_t q = quantize_cep(c);
        EXPECT_NE(q, INT8_MIN) << "visible cep should not produce sentinel";
        const float r = dequantize_cep(q);                        // raw-ok: test scaffolding for NN-byte-format primitive
        EXPECT_NEAR(r, c, kStep) << "input " << c;
    }
}

TEST(BeaconProjectionQuant, CepSentinelExactRoundTrip) {
    EXPECT_EQ(quantize_cep(1.25f), INT8_MIN);
    EXPECT_EQ(quantize_cep(1.5f), INT8_MIN);
    EXPECT_EQ(quantize_cep(99.0f), INT8_MIN);
    EXPECT_FLOAT_EQ(dequantize_cep(INT8_MIN), kCepSentinelFloat);
    // Threshold boundary: anything strictly below 1.25 quantizes as visible.
    EXPECT_NE(quantize_cep(1.249f), INT8_MIN);
}

// ---------------------------------------------------------------------------
// rayHitsProxy direct unit test — covers parallel-axis edge cases.
// ---------------------------------------------------------------------------

TEST(RayHitsProxy, ForwardSegmentThroughBoxHits) {
    AirframeProxy box{gp_vec3(2.0f, -0.5f, -0.5f), gp_vec3(4.0f, 0.5f, 0.5f)};
    EXPECT_TRUE(rayHitsProxy(gp_vec3(0, 0, 0), gp_vec3(10, 0, 0), box));
}

TEST(RayHitsProxy, OffAxisSegmentMisses) {
    AirframeProxy box{gp_vec3(2.0f, -0.5f, -0.5f), gp_vec3(4.0f, 0.5f, 0.5f)};
    EXPECT_FALSE(rayHitsProxy(gp_vec3(0, 5, 0), gp_vec3(10, 5, 0), box));
}

TEST(RayHitsProxy, SegmentEndsBeforeBoxMisses) {
    AirframeProxy box{gp_vec3(2.0f, -0.5f, -0.5f), gp_vec3(4.0f, 0.5f, 0.5f)};
    // Segment from origin to (1, 0, 0); box starts at x=2 → no hit.
    EXPECT_FALSE(rayHitsProxy(gp_vec3(0, 0, 0), gp_vec3(1, 0, 0), box));
}

// ---------------------------------------------------------------------------
// Determinism — repeated invocation with identical input ⇒ identical bits.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionDeterminism, IdenticalInputProducesIdenticalOutput) {
    ProjectionInput in = makeBaselineInput();
    BeaconObservation a = projectBeacon(in);
    BeaconObservation b = projectBeacon(in);
    EXPECT_EQ(a.raw_x_int8, b.raw_x_int8);
    EXPECT_EQ(a.raw_y_int8, b.raw_y_int8);
    EXPECT_EQ(a.raw_cep_int8, b.raw_cep_int8);
    EXPECT_FLOAT_EQ(a.screen_x, b.screen_x);
    EXPECT_FLOAT_EQ(a.screen_y, b.screen_y);
    EXPECT_FLOAT_EQ(a.cep, b.cep);
}

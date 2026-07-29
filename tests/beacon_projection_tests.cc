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

using autoc::eval::AirframeObstruction;
using autoc::eval::rayHitsBox;
using autoc::eval::testObstruction;
using autoc::eval::ObstructionResult;
using autoc::eval::rayCrossesPropDisc;
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

namespace {

// Test scaffolding — chase at world origin, identity orientation; left-
// wingtip beacon mounted at (0, -0.45, 0) on a target whose orientation
// the test sets. The default airframe proxy is loose enough that beacons
// out in front of the chase clear it.
// 040 T014 — legacy box-occlusion tests below were written against the single
// AABB. They now exercise the WING slab, with the nose and prop parked clear,
// so what they assert (a box in the line of sight gates the beacon) still holds.
AirframeObstruction obstructionWithWingBox(const gp_vec3& lo, const gp_vec3& hi) {
    AirframeObstruction a =
        autoc::eval::hb1AirframeObstruction();
    a.enabled = true;
    a.wing_min = lo;
    a.wing_max = hi;
    // Park the nose and the prop disc far behind the camera so only the wing
    // box under test can produce a hit.
    a.nose_min = gp_vec3(-100.0f, -1.0f, -1.0f);
    a.nose_max = gp_vec3(-99.0f, 1.0f, 1.0f);
    a.prop_plane_x = -100.0f;
    a.prop_radius = 0.0f;
    return a;
}

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
    in.chase_airframe = obstructionWithWingBox(gp_vec3(-100.0f, -1.0f, -1.0f), gp_vec3(-99.0f,  +1.0f, +1.0f));
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
    // 120° H FOV ⇒ half-angle 60°. Target at body (10, 17.32, 0) puts the
    // beacon ray at atan(17.32/10) = 60° off-axis — exactly the right edge
    // (±1 at the FOV edge holds under BOTH the pre-t9 rectilinear and the
    // 038 t9 equidistant projection, by construction). Set beacon mount to
    // target origin to remove the wingtip bias.
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
// 038 t9 — Equidistant (f-theta) projection properties. NDC ∝ angle off the
// optical axis (not tan), so a mid-field position reads as its angular
// fraction of the half-FOV, and a fixed angular gap spans the same NDC
// anywhere in frame (the rectilinear tan-stretch this replaced would read
// an off-axis pair wider than a centered one).
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, EquidistantMidAngleIsAngularRatio) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    // 30° right of the optical axis at 10 m: (10·cos30°, 10·sin30°, 0).
    in.target_position_world = gp_vec3(8.6603f, 5.0f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    // Equidistant: 30°/60° = 0.5 (rectilinear tan30/tan60 read 0.333 here).
    EXPECT_NEAR(obs.screen_x, 0.5f, 0.01f);
    EXPECT_NEAR(obs.screen_y, 0.0f, 1.0f / 127.0f);
}

TEST(BeaconProjectionGeometry, EquidistantSpanIsPositionInvariant) {
    // Same 6° angular gap, pure horizontal: pair centered on-axis vs 40°
    // off-axis must span ≈ the same NDC (rectilinear read the off-axis
    // pair ~70% wider — the ego-pointing contamination t9 removes).
    auto obsAtDeg = [](float deg) {
        ProjectionInput in = makeBaselineInput();
        in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
        in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
        const float rad = deg * 3.14159265358979323846f / 180.0f;
        in.target_position_world =
            gp_vec3(10.0f * std::cos(rad), 10.0f * std::sin(rad), 0.0f);
        return projectBeacon(in);
    };
    const float span_center = obsAtDeg(3.0f).screen_x - obsAtDeg(-3.0f).screen_x;
    const float span_off = obsAtDeg(43.0f).screen_x - obsAtDeg(37.0f).screen_x;
    // 6°/60° = 0.1 NDC; allow int8 quantization slack (steps of 1/127).
    EXPECT_NEAR(span_center, 6.0f / 60.0f, 2.5f / 127.0f);
    EXPECT_NEAR(span_off, span_center, 2.5f / 127.0f);
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

TEST(BeaconProjectionSentinel, AirframeObstructionOccludesEmitsSentinel) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    // Target dead ahead; proxy box straddles the optical axis between
    // camera and target.
    in.target_position_world = gp_vec3(10.0f, 0.0f, 0.0f);
    in.chase_airframe = obstructionWithWingBox(gp_vec3(2.0f, -0.5f, -0.5f), gp_vec3(4.0f, +0.5f, +0.5f));

    BeaconObservation obs = projectBeacon(in);

    EXPECT_FLOAT_EQ(obs.cep, kCepSentinelFloat);
}

TEST(BeaconProjectionSentinel, AirframeObstructionMissesDoesNotOcclude) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, 0.0f, 0.0f);
    // Proxy way off to the side — ray to (10, 0, 0) doesn't pass through.
    in.chase_airframe = obstructionWithWingBox(gp_vec3(2.0f, +5.0f, +5.0f), gp_vec3(4.0f, +6.0f, +6.0f));

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
// rayHitsBox direct unit test — covers parallel-axis edge cases. (040 T014:
// the slab primitive moved to airframe_occlusion.h and now takes explicit
// bounds rather than a proxy struct.)
// ---------------------------------------------------------------------------

TEST(RayHitsBox, ForwardSegmentThroughBoxHits) {
    const gp_vec3 box_lo(2.0f, -0.5f, -0.5f);
    const gp_vec3 box_hi(4.0f, 0.5f, 0.5f);
    EXPECT_TRUE(rayHitsBox(gp_vec3(0, 0, 0), gp_vec3(10, 0, 0), box_lo, box_hi));
}

TEST(RayHitsBox, OffAxisSegmentMisses) {
    const gp_vec3 box_lo(2.0f, -0.5f, -0.5f);
    const gp_vec3 box_hi(4.0f, 0.5f, 0.5f);
    EXPECT_FALSE(rayHitsBox(gp_vec3(0, 5, 0), gp_vec3(10, 5, 0), box_lo, box_hi));
}

TEST(RayHitsBox, SegmentEndsBeforeBoxMisses) {
    const gp_vec3 box_lo(2.0f, -0.5f, -0.5f);
    const gp_vec3 box_hi(4.0f, 0.5f, 0.5f);
    // Segment from origin to (1, 0, 0); box starts at x=2 → no hit.
    EXPECT_FALSE(rayHitsBox(gp_vec3(0, 0, 0), gp_vec3(1, 0, 0), box_lo, box_hi));
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

// ---------------------------------------------------------------------------
// 040 T008 — the camera must never sit ON an obstruction boundary.
//
// WHY THIS EXISTS. The superseded single-AABB proxy was degenerate: it spanned
// z ∈ [-0.05, +0.20] while the default camera mount sat at z = -0.05, exactly
// on box_min_z. A surface touch counts as a hit, so enabling occlusion made
// essentially EVERY forward ray report blocked and the target vanish.
//
// That was harmless only because occlusion shipped disabled. 040 replaces the
// proxy with three primitives anchored to the camera mount, which removes the
// coincidence by construction. These tests pin that it stays removed.
// ---------------------------------------------------------------------------

TEST(AirframeObstruction, CameraMountIsNotInsideAnyPrimitive) {
    const gp_vec3 mount = CameraConfig{}.mount_offset_body;
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    a.enabled = true;

    // A degenerate mount shows up as a zero-length ray "hitting" a primitive.
    // Probe with a tiny forward segment: if the origin were on or inside a
    // face this would report blocked.
    const gp_vec3 just_ahead = mount + gp_vec3(1e-4f, 0.0f, 0.0f);
    EXPECT_FALSE(rayHitsBox(mount, just_ahead, a.wing_min, a.wing_max))
        << "camera mount lies on/inside the wing slab — the degeneracy is back";
    EXPECT_FALSE(rayHitsBox(mount, just_ahead, a.nose_min, a.nose_max))
        << "camera mount lies on/inside the nose box — the degeneracy is back";
}

TEST(AirframeObstruction, ForwardLevelRayIsNotSelfOccluded) {
    // The contract the replacement must satisfy, and precisely what fails with
    // the superseded proxy: a level forward ray from the configured mount
    // reaches a distant target without the aircraft blocking itself.
    const gp_vec3 mount = CameraConfig{}.mount_offset_body;
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    a.enabled = true;

    const gp_vec3 target = mount + gp_vec3(10.0f, 0.0f, 0.0f);
    const ObstructionResult r = testObstruction(mount, target, a);
    EXPECT_FALSE(r.blocked)
        << "a forward-level ray must not be gated by the chase's own airframe";
}

TEST(AirframeObstruction, LeadingEdgeMountClearsThePropDisc) {
    // THE DESIGN PROPERTY. The camera sits 8" outboard of the thrust axis, so
    // its radial distance (~8") vastly exceeds the 2.75" tip radius and a
    // forward ray never enters the disc. This is what makes the leading-edge
    // mount worth having, and it is why the 040 propeller model needs no blade
    // phase: at this mount the disc sits ~41-61 deg inboard, nowhere near the
    // boresight where a tail-chased target lives.
    // The LEADING-EDGE mount, not the legacy one. The legacy mount sits 5 cm
    // above the axle — inside the 6.99 cm prop radius, i.e. squarely behind
    // the disc — which is precisely why Stage D moves it.
    const gp_vec3 mount = autoc::eval::hb1LeadingEdgeCameraMount();
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    a.enabled = true;

    const gp_vec3 target = mount + gp_vec3(10.0f, 0.0f, 0.0f);
    EXPECT_FALSE(rayCrossesPropDisc(mount, target, a))
        << "an 8-inch outboard mount must clear the prop disc on boresight";

    const ObstructionResult r = testObstruction(mount, target, a);
    EXPECT_FALSE(r.blocked);
    EXPECT_FLOAT_EQ(r.attenuation, 1.0f) << "clear of the disc ⇒ no attenuation";

    // The contrast that motivates the mount change: the legacy centreline
    // position DOES sit behind the disc.
    const gp_vec3 legacy = CameraConfig{}.mount_offset_body;
    EXPECT_TRUE(rayCrossesPropDisc(legacy, legacy + gp_vec3(10.0f, 0.0f, 0.0f), a))
        << "expected the legacy mount to sit behind the disc";
}

TEST(AirframeObstruction, PropDiscAttenuatesButNeverGates) {
    // FR-009: where the disc IS crossed it attenuates and never gates. Put the
    // thrust axis on the boresight explicitly rather than relying on the
    // baseline geometry, which deliberately clears it (see the test above).
    const gp_vec3 mount = CameraConfig{}.mount_offset_body;
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    a.enabled = true;
    a.prop_axis_y = mount.y();
    a.prop_axis_z = mount.z();

    const gp_vec3 target = mount + gp_vec3(10.0f, 0.0f, 0.0f);
    ASSERT_TRUE(rayCrossesPropDisc(mount, target, a));

    const ObstructionResult r = testObstruction(mount, target, a);
    EXPECT_FALSE(r.blocked) << "the propeller must never gate (FR-009)";
    EXPECT_LT(r.attenuation, 1.0f) << "crossing the disc must attenuate";
    EXPECT_GT(r.attenuation, 0.0f) << "attenuation must be partial, not total";
}

TEST(AirframeObstruction, DisabledObstructionNeverOccludes) {
    // Guards the ship-safe default: with obstruction disabled, geometry is
    // irrelevant and nothing may be reported blocked or attenuated.
    const gp_vec3 mount = CameraConfig{}.mount_offset_body;
    const AirframeObstruction a =
        autoc::eval::hb1AirframeObstruction();
    ASSERT_FALSE(a.enabled)
        << "obstruction is expected to ship disabled until Stage D (T043)";

    const gp_vec3 target = mount + gp_vec3(10.0f, 0.0f, 0.0f);
    const ObstructionResult r = testObstruction(mount, target, a);
    EXPECT_FALSE(r.blocked);
    EXPECT_FLOAT_EQ(r.attenuation, 1.0f);
}

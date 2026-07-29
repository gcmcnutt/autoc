// 030 M5 — Beacon projection contract tests (T025 + T026), carried forward to
// the 040 US2 bearing representation.
//
// Geometry: target dead-ahead → bearing ≈ 0, cep ≈ 0; target at the derived
// FOV edge → bearing near the per-axis half-field, cep elevated; target behind
// → sentinel; target occluded by the airframe → sentinel; target outside the
// emission cone (tail-on aspect) → sentinel.
//
// 040 T031/T032 — bearings are ANGLES IN RADIANS quantised on the sensor's
// pixel grid. The assertions below were written against the retired ±1 NDC
// encoding and are restated in the angle domain; the geometric facts they pin
// are unchanged. The int8 bearing round-trip tests are GONE with the encoding
// (T032) — the grid is the resolution model now, and CameraGridGeometry.* at
// the end of this file is what replaces them.
//
// CEP round-trip survives: dequantize_cep(quantize_cep(any ≥ 1.25)) exactly
// equals kCepSentinelFloat (1.5f), and dequantize_cep maps INT8_MIN to
// sentinel. US4 (FR-014) replaces CEP with a signal-derived quality.

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
using autoc::eval::kCepSentinelFloat;
using autoc::eval::kCepSentinelThreshold;
using autoc::eval::projectBeacon;
using autoc::eval::quantize_cep;

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

// ---------------------------------------------------------------------------
// 040 US2 scaffolding — place a point source at a chosen ray direction.
//
// Principle VI: the raw `double` below is deliberate and stays raw-ok
// throughout this block. These helpers compute the REFERENCE geometry a test
// compares against, so they must be more precise than the code under test —
// carrying them in `gp_scalar` would fold the test's own rounding into
// tolerances measured in fractions of a pixel, and a 2% invariance claim would
// then be partly measuring the test.
// ---------------------------------------------------------------------------

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;  // raw-ok: test-reference geometry, see block note above
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// A point whose ray from the camera sits `theta_deg` off the boresight,
// rotated `phi_deg` about it in the image plane (0° = image right / body +y,
// 90° = image down / body +z), at `range_m`.
//
// Chase sits at the world origin with identity orientation and the camera is
// mounted at the body origin, so camera frame == world frame here and the
// returned vector is usable directly as a world position.
gp_vec3 rayPoint(double theta_deg, double phi_deg, double range_m) {
    const double t = theta_deg * kDegToRad;
    const double p = phi_deg * kDegToRad;
    return gp_vec3(static_cast<gp_scalar>(range_m * std::cos(t)),
                   static_cast<gp_scalar>(range_m * std::sin(t) * std::cos(p)),
                   static_cast<gp_scalar>(range_m * std::sin(t) * std::sin(p)));
}

// Project a lone point source along that ray. The emitter is aimed straight
// back at the chase so the emission cone can never gate and the test isolates
// the projection stage.
BeaconObservation obsAtRay(double theta_deg, double phi_deg,
                           double range_m = 10.0) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.target_position_world = rayPoint(theta_deg, phi_deg, range_m);
    const gp_vec3 back_at_chase = -in.target_position_world.normalized();
    in.beacon_emission_axis_target_body = back_at_chase;
    in.beacon.mount_body = in.beacon_mount_target_body;
    in.beacon.emission_axis_body = back_at_chase;
    return projectBeacon(in);
}

// Separation between two reported bearings, in degrees, measured as Euclidean
// distance in the (θx, θy) bearing plane — exactly what compute_pair_span does.
double sepDeg(const BeaconObservation& a, const BeaconObservation& b) {
    const double dx = static_cast<double>(a.bearing_x_rad - b.bearing_x_rad);
    const double dy = static_cast<double>(a.bearing_y_rad - b.bearing_y_rad);
    return std::sqrt(dx * dx + dy * dy) * kRadToDeg;
}

// Exact bearing of pixel `i`'s CENTRE on an `n`-pixel axis, in degrees.
// Centres sit at (i − (n−1)/2)·deg_per_px, so the sensor's outer edges land at
// ±n/2·deg_per_px and the derived FOV is exactly n × deg_per_px (FR-003).
// Tests that need zero quantisation error place rays on these values.
double pixelCentreDeg(int i, int n, double deg_per_px) {
    return (static_cast<double>(i) - (static_cast<double>(n) - 1.0) / 2.0) *
           deg_per_px;
}

}  // namespace

// ---------------------------------------------------------------------------
// T025 — Geometry: target dead-ahead.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, TargetDeadAheadProducesCenteredObservation) {
    ProjectionInput in = makeBaselineInput();
    // Move beacon to target body origin so projection lands exactly on the
    // optical axis (no wingtip offset to bias the bearing).
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);  // toward chase

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    // With an even pixel count the boresight falls on a pixel BOUNDARY, so a
    // perfectly centred beacon reports half a pixel, never exactly zero. That
    // is the honest behaviour of an even-width sensor (T031).
    const CameraConfig cam{};
    const float half_px = static_cast<float>(cam.radPerPx()) * 0.5f;
    EXPECT_NEAR(obs.bearing_x_rad, 0.0f, half_px + 1e-6f);
    EXPECT_NEAR(obs.bearing_y_rad, 0.0f, half_px + 1e-6f);
    EXPECT_LT(obs.cep, 0.05f);
    EXPECT_NE(obs.raw_cep_int8, INT8_MIN);
}

// ---------------------------------------------------------------------------
// T025 — Geometry: target near the derived FOV edge → bearing approaches the
// per-axis half-field, cep up.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, TargetAtRightFovEdgeApproachesPlusOne) {
    ProjectionInput in = makeBaselineInput();
    // The derived field is 320 px × 0.375° = 120° H ⇒ half-angle 60°. Target
    // at body (10, 17.32, 0) puts the beacon ray at atan(17.32/10) = 60°
    // off-axis — exactly the right edge. Set beacon mount to target origin to
    // remove the wingtip bias.
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, 17.32f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    EXPECT_NEAR(obs.bearing_x_rad, static_cast<float>(CameraConfig{}.halfFovHRad()),
                static_cast<float>(CameraConfig{}.radPerPx()));
    EXPECT_GT(obs.cep, 0.20f);  // edge_factor near 1 ⇒ cep near 0.3
}

TEST(BeaconProjectionGeometry, TargetAtLeftFovEdgeApproachesMinusOne) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    in.target_position_world = gp_vec3(10.0f, -17.32f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    EXPECT_NEAR(obs.bearing_x_rad, -static_cast<float>(CameraConfig{}.halfFovHRad()),
                static_cast<float>(CameraConfig{}.radPerPx()));
    EXPECT_GT(obs.cep, 0.20f);
}

// ---------------------------------------------------------------------------
// 038 t9 — Equidistant (f-theta) projection properties. Bearing ∝ angle off
// the optical axis (not tan), so a mid-field position reads as its true angle
// and a fixed angular gap spans the same amount anywhere in frame (the
// rectilinear tan-stretch this replaced read an off-axis pair wider than a
// centred one). 040 T031 restates these in radians — under the retired ±1 NDC
// encoding the same facts were expressed as fractions of each half-FOV.
// ---------------------------------------------------------------------------

TEST(BeaconProjectionGeometry, EquidistantMidAngleIsAngularRatio) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.beacon_emission_axis_target_body = gp_vec3(-1.0f, 0.0f, 0.0f);
    // 30° right of the optical axis at 10 m: (10·cos30°, 10·sin30°, 0).
    in.target_position_world = gp_vec3(8.6603f, 5.0f, 0.0f);

    BeaconObservation obs = projectBeacon(in);

    EXPECT_NE(obs.cep, kCepSentinelFloat);
    // Equidistant: the bearing IS the 30° angle (a rectilinear mapping would
    // read atan-compressed here, 0.333 of the half-field rather than 0.5).
    const CameraConfig cam{};
    EXPECT_NEAR(obs.bearing_x_rad, static_cast<float>(30.0 * kDegToRad),
                static_cast<float>(cam.radPerPx()));
    EXPECT_NEAR(obs.bearing_y_rad, 0.0f,
                static_cast<float>(cam.radPerPx()));
}

TEST(BeaconProjectionGeometry, EquidistantSpanIsPositionInvariant) {
    // Same 6° angular gap, pure horizontal: pair centred on-axis vs 40°
    // off-axis must span ≈ the same angle (rectilinear read the off-axis
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
    const float span_center =
        obsAtDeg(3.0f).bearing_x_rad - obsAtDeg(-3.0f).bearing_x_rad;
    const float span_off =
        obsAtDeg(43.0f).bearing_x_rad - obsAtDeg(37.0f).bearing_x_rad;
    // 6° in radians; allow two pixels of grid slack (one per endpoint).
    const float slack = 2.0f * static_cast<float>(CameraConfig{}.radPerPx());
    EXPECT_NEAR(span_center, static_cast<float>(6.0 * kDegToRad), slack);
    EXPECT_NEAR(span_off, span_center, slack);
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
// T026 — CEP round-trip determinism.
//
// 040 T032 — the int8 BEARING round-trip tests that lived here are deleted
// with the encoding they guarded. They were not wrong, they were guarding the
// wrong thing: a second resolution model, independent of and disagreeing with
// the sensor grid (0.472°/LSB horizontally against a 0.375° pixel). The grid
// is the resolution model now, and CameraGridGeometry.BearingIsQuantised* /
// .BearingResolvesNoFinerAndNoCoarserThanOnePixel replace them.
// ---------------------------------------------------------------------------

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
    EXPECT_EQ(a.raw_px_x, b.raw_px_x);
    EXPECT_EQ(a.raw_px_y, b.raw_px_y);
    EXPECT_EQ(a.raw_cep_int8, b.raw_cep_int8);
    EXPECT_FLOAT_EQ(a.bearing_x_rad, b.bearing_x_rad);
    EXPECT_FLOAT_EQ(a.bearing_y_rad, b.bearing_y_rad);
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

// ===========================================================================
// 040 US2 (T023-T028) — honest camera geometry.
//
// Bearing is quantised on a real pixel grid and reported as ISOTROPIC angles
// in radians. What changes versus the ±1 per-axis NDC encoding this replaces:
//
//   * the two axes shared no scale. Normalising each by its own half-FOV made
//     a horizontal separation read 60/45 = 1.333× its vertical twin — a 33%
//     orientation error sitting directly in the sole range channel.
//   * the int8 encoding was a SECOND, unrelated resolution model: 0.472°/LSB
//     horizontally against a 0.375° pixel, 26% coarser than the sensor it
//     claimed to represent, and 6% finer vertically.
//
// SCOPE OF THE INVARIANCE CLAIM (research R2, contract §6). Under equidistant
// mapping, Euclidean distance in (θx, θy) equals the great-circle angle
// EXACTLY along a radius and over-reads TANGENTIALLY by θ/sin θ — up to +21%
// at the frame corner. That residual is accepted and documented, not fixed;
// it is what makes a separate ray-angle span computation unnecessary.
//
// So SC-001's "within 2% ... at any orientation" holds radially and holds for
// the orientation flip that FR-002 is actually about, but NOT tangentially at
// large field angles. RadialSeparationIsPositionInvariant asserts the property
// delivered; TangentialSeparationOverReadsByThetaOverSinTheta pins the limit,
// so the residual is a tested contract rather than a surprise later.
// ===========================================================================

// ---------------------------------------------------------------------------
// T023 (SC-001) — a fixed angular separation reads the same anywhere in frame.
// ---------------------------------------------------------------------------

TEST(CameraGridGeometry, RadialSeparationIsPositionInvariant) {
    const CameraConfig cam{};
    const double dpp = static_cast<double>(cam.deg_per_px);

    // Both pairs are 40 pixels apart on the horizontal axis and both lie ON
    // pixel centres, so quantisation contributes EXACTLY zero and the 2%
    // tolerance measures the projection alone rather than the grid.
    const double near_lo = pixelCentreDeg(140, cam.pixels_h, dpp);   // −7.3125°
    const double near_hi = pixelCentreDeg(180, cam.pixels_h, dpp);   // +7.6875°
    const double far_lo  = pixelCentreDeg(260, cam.pixels_h, dpp);   // +37.6875°
    const double far_hi  = pixelCentreDeg(300, cam.pixels_h, dpp);   // +52.6875°

    // φ = 180° puts the ray on the −y side; φ = 0° on the +y side.
    const double sep_centre =
        sepDeg(obsAtRay(-near_lo, 180.0), obsAtRay(near_hi, 0.0));
    const double sep_edge =
        sepDeg(obsAtRay(far_lo, 0.0), obsAtRay(far_hi, 0.0));

    const double expected = 40.0 * dpp;  // 15.0°
    EXPECT_NEAR(sep_centre, expected, 0.02 * expected);
    EXPECT_NEAR(sep_edge, expected, 0.02 * expected);
    EXPECT_NEAR(sep_edge, sep_centre, 0.02 * sep_centre)
        << "a fixed angular separation must read the same at frame centre and "
           "out at 50° off-axis (SC-001)";
}

TEST(CameraGridGeometry, TangentialSeparationOverReadsByThetaOverSinTheta) {
    // The documented residual (research R2, contract §6). A pair straddling a
    // constant field angle θ₀ tangentially reads θ₀/sin θ₀ too wide. Pinned so
    // the accepted limit cannot drift silently — and so that anyone tempted to
    // read SC-001 as unconditional finds the exception under test.
    const double theta0 = 40.0;
    const double dphi = 20.0;

    const double measured =
        sepDeg(obsAtRay(theta0, -dphi / 2.0), obsAtRay(theta0, +dphi / 2.0));

    // True great-circle angle between the two rays.
    const double t = theta0 * kDegToRad;
    const double cos_psi = std::cos(t) * std::cos(t) +
                           std::sin(t) * std::sin(t) * std::cos(dphi * kDegToRad);
    const double truth = std::acos(cos_psi) * kRadToDeg;

    const double over_read = measured / truth;
    const double predicted = t / std::sin(t);   // 1.086 at 40°
    EXPECT_NEAR(over_read, predicted, 0.02)
        << "tangential over-read must match θ/sin θ — measured " << measured
        << "° vs true " << truth << "°";
    EXPECT_GT(over_read, 1.02)
        << "this case is deliberately OUTSIDE the 2% band; if it ever lands "
           "inside, the projection changed and contract §6 needs revisiting";
}

// ---------------------------------------------------------------------------
// T024 (FR-002) — one angular scale for both axes.
// ---------------------------------------------------------------------------

TEST(CameraGridGeometry, SeparationIsUnchangedWhenRotatedHorizontalToVertical) {
    const CameraConfig cam{};
    const double dpp = static_cast<double>(cam.deg_per_px);

    // Identical angular offsets either side of the boresight, measured once
    // across the image and once down it. Both axes' pixel centres sit at the
    // same offsets (both counts are even), so this is exact on both.
    const double lo = -pixelCentreDeg(140, cam.pixels_h, dpp);  // 7.3125°
    const double hi = pixelCentreDeg(180, cam.pixels_h, dpp);   // 7.6875°

    const double horizontal = sepDeg(obsAtRay(lo, 180.0), obsAtRay(hi, 0.0));
    const double vertical   = sepDeg(obsAtRay(lo, 270.0), obsAtRay(hi, 90.0));

    EXPECT_NEAR(horizontal, vertical, 1e-4)
        << "the same angular separation must read identically horizontally and "
           "vertically (FR-002). The retired ±1 NDC encoding normalised each "
           "axis by its own half-FOV and read these 33% apart";
    EXPECT_NEAR(horizontal, lo + hi, 1e-4);
}

// ---------------------------------------------------------------------------
// T025 (FR-001) — bearing resolves exactly as finely as the grid, no more.
// ---------------------------------------------------------------------------

TEST(CameraGridGeometry, BearingIsQuantisedToPixelCentres) {
    const CameraConfig cam{};
    const double dpp = static_cast<double>(cam.deg_per_px);

    // Every reported bearing must land on a pixel centre — an exact multiple
    // of the pixel pitch away from the axis origin.
    for (double deg = -50.0; deg <= 50.0; deg += 0.11) {
        const BeaconObservation obs = obsAtRay(std::abs(deg), deg < 0 ? 180.0 : 0.0);
        ASSERT_NE(obs.cep, kCepSentinelFloat) << "at " << deg << "°";
        const double reported = static_cast<double>(obs.bearing_x_rad) * kRadToDeg;
        const double in_pixels = reported / dpp - 0.5;
        EXPECT_NEAR(in_pixels, std::round(in_pixels), 1e-3)
            << "bearing " << reported << "° is not on a pixel centre (input "
            << deg << "°)";
    }
}

TEST(CameraGridGeometry, BearingResolvesNoFinerAndNoCoarserThanOnePixel) {
    const CameraConfig cam{};
    const double dpp = static_cast<double>(cam.deg_per_px);
    const double centre = pixelCentreDeg(200, cam.pixels_h, dpp);  // +15.1875°

    // NO FINER: two truths inside the same pixel are indistinguishable.
    const BeaconObservation a = obsAtRay(centre - 0.3 * dpp, 0.0);
    const BeaconObservation b = obsAtRay(centre + 0.3 * dpp, 0.0);
    EXPECT_EQ(a.raw_px_x, b.raw_px_x);
    EXPECT_FLOAT_EQ(a.bearing_x_rad, b.bearing_x_rad)
        << "sub-pixel detail must not survive — reported precision has to "
           "match sensor capability (FR-001)";

    // NO COARSER: one pixel apart resolves as exactly one pixel.
    const BeaconObservation c = obsAtRay(centre + dpp, 0.0);
    EXPECT_EQ(c.raw_px_x, a.raw_px_x + 1);
    EXPECT_NEAR(static_cast<double>(c.bearing_x_rad - a.bearing_x_rad) * kRadToDeg,
                dpp, 1e-4);
}

// ---------------------------------------------------------------------------
// T026 (SC-002) — range inferred from separation, and the 0.772 m correction.
// ---------------------------------------------------------------------------

TEST(CameraGridGeometry, RangeFromSeparationMatchesTruthWithoutSystematicBias) {
    const CameraConfig cam{};
    const BeaconConfig beacons{};
    // The measured article: a 30″ span plus the enclosure offset at each tip.
    const double separation_m =
        2.0 * std::abs(static_cast<double>(beacons.mount_body.y()));
    ASSERT_NEAR(separation_m, 0.772, 1e-6)
        << "range inference reads the physical separation straight off the "
           "beacon mounts; 0.9 m put a systematic ~17% into every range";

    const double rad_per_px = static_cast<double>(cam.deg_per_px) * kDegToRad;

    double bias_sum = 0.0;
    int n = 0;
    for (double truth_m = 5.0; truth_m <= 25.0001; truth_m += 0.25) {
        // Pair perpendicular to the line of sight, straddling the boresight,
        // so the reading is radial and the tangential residual does not enter.
        const double half = std::atan((separation_m / 2.0) / truth_m) * kRadToDeg;
        const BeaconObservation left  = obsAtRay(half, 180.0, truth_m);
        const BeaconObservation right = obsAtRay(half, 0.0, truth_m);
        ASSERT_NE(left.cep, kCepSentinelFloat);
        ASSERT_NE(right.cep, kCepSentinelFloat);

        const double span_rad = sepDeg(left, right) * kDegToRad;
        ASSERT_GT(span_rad, 0.0);
        const double inferred_m = separation_m / span_rad;

        // What the grid permits: a one-pixel error in the span propagates
        // proportionally into range.
        const double grid_bound = rad_per_px / span_rad;
        const double rel_err = (inferred_m - truth_m) / truth_m;
        EXPECT_LT(std::abs(rel_err), grid_bound + 0.01)
            << "range " << truth_m << " m inferred as " << inferred_m
            << " m — worse than the sensor grid permits";

        bias_sum += rel_err;
        ++n;
    }

    // NO SYSTEMATIC BIAS. This is the assertion that catches a wrong physical
    // separation: a wrong constant shifts every range the same direction, by
    // the same ratio. At 0.9 m against a true 0.772 m every estimate would sit
    // ~16.6% high and this would fail by a wide margin.
    //
    // The ~1% that legitimately remains is GRID CONVEXITY, not calibration:
    // range is separation/span, and E[1/span] > 1/E[span] for a span carrying
    // symmetric quantisation error (Jensen). At 25 m the pair subtends only
    // ~4.7 px, so ±1 px of grid error is enough to bend the mean upward by
    // about a percent. That is a genuine property of inferring range from a
    // discrete sensor, and 040 does not pretend otherwise — the bound below is
    // set to sit above it and far below any plausible constant error.
    const double mean_bias = bias_sum / n;
    EXPECT_LT(std::abs(mean_bias), 0.03)
        << "mean relative range error " << (mean_bias * 100.0)
        << "% — an offset this large is a wrong physical separation, not the "
           "~1% grid-convexity residual";
}

// ---------------------------------------------------------------------------
// T028 (FR-003) — the field of view is derived, never independently set.
// ---------------------------------------------------------------------------

TEST(CameraGridGeometry, FieldOfViewIsDerivedFromGridAndPixelPitch) {
    CameraConfig cam{};
    EXPECT_DOUBLE_EQ(static_cast<double>(cam.fovHDeg()),
                     cam.pixels_h * static_cast<double>(cam.deg_per_px));
    EXPECT_DOUBLE_EQ(static_cast<double>(cam.fovVDeg()),
                     cam.pixels_v * static_cast<double>(cam.deg_per_px));
    EXPECT_NEAR(static_cast<double>(cam.fovHDeg()), 120.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(cam.fovVDeg()), 90.0, 1e-9);

    // Resolution and field cannot disagree, because there is only one knob:
    // halving the pixel pitch halves both fields, and no setter exists that
    // could contradict it.
    cam.deg_per_px = static_cast<gp_scalar>(0.1875);
    EXPECT_NEAR(static_cast<double>(cam.fovHDeg()), 60.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(cam.fovVDeg()), 45.0, 1e-9);
}

TEST(CameraGridGeometry, FovEdgeFollowsTheDerivedField) {
    // The derived field is the real visibility boundary, not a separate knob
    // that could drift away from the grid.
    const CameraConfig cam{};
    const double half_h = static_cast<double>(cam.fovHDeg()) / 2.0;

    EXPECT_NE(obsAtRay(half_h - 0.5, 0.0).cep, kCepSentinelFloat)
        << "just inside the derived field must be visible";
    EXPECT_FLOAT_EQ(obsAtRay(half_h + 0.5, 0.0).cep, kCepSentinelFloat)
        << "just outside the derived field must be invisible";
}

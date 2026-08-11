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
#include "autoc/eval/acquisition_state.h"
#include "autoc/eval/signal_model.h"
#include "autoc/eval/tracker_tick_rule.h"
#include "autoc/eval/derived_features.h"  // 040 T033a — span is measured here
#include "autoc/types.h"

using autoc::eval::AirframeObstruction;
using autoc::eval::rayHitsBox;
using autoc::eval::testObstruction;
using autoc::eval::ObstructionResult;
using autoc::eval::rayCrossesPropDisc;
using autoc::eval::BeaconConfig;
using autoc::eval::BeaconObservation;
using autoc::eval::CameraConfig;
using autoc::eval::hb1AcquisitionConfig;
using autoc::eval::hb1SignalConfig;
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
    // 040 T074 — the obstruction ray origin is a SEPARATE field so US6 can drift
    // it from the bearing mount. Like SignalConfig it has no in-class default
    // (Constitution VII), so leaving it unset is silent garbage rather than a
    // compile error — set it explicitly, as both production paths do.
    in.obstruction_mount_chase_body = in.camera_mount_chase_body;
    in.camera = CameraConfig{};
    in.beacon = BeaconConfig{};
    in.beacon.mount_body = in.beacon_mount_target_body;
    in.beacon.emission_axis_body = in.beacon_emission_axis_target_body;
    // Placeholder proxy parked far behind the camera so default test rays
    // (forward, +x) never trip self-occlusion. Specific occlusion tests
    // override `in.chase_airframe` directly.
    in.chase_airframe = obstructionWithWingBox(gp_vec3(-100.0f, -1.0f, -1.0f), gp_vec3(-99.0f,  +1.0f, +1.0f));
    // 040 US4 — the link budget. SignalConfig carries NO in-class defaults
    // (Constitution VII), so leaving this unset does not fail to compile — it
    // silently yields garbage, and every ray reads as dark. Supply the shipped
    // values explicitly, exactly as the production tick rule does.
    in.signal = hb1SignalConfig();
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

// As obsAtRay, but aimed along an arbitrary direction rather than a (θ, φ)
// pair. Needed where the pair geometry is built in 3D so that the true angular
// separation is exact by construction rather than routed through the very
// mapping under test.
BeaconObservation obsAlong(const gp_vec3& dir, double range_m = 10.0) {
    ProjectionInput in = makeBaselineInput();
    in.beacon_mount_target_body = gp_vec3(0.0f, 0.0f, 0.0f);
    in.target_position_world =
        dir.normalized() * static_cast<gp_scalar>(range_m);
    const gp_vec3 back_at_chase = -in.target_position_world.normalized();
    in.beacon_emission_axis_target_body = back_at_chase;
    in.beacon.mount_body = in.beacon_mount_target_body;
    in.beacon.emission_axis_body = back_at_chase;
    return projectBeacon(in);
}

// Separation between two reported bearings, in degrees. Measured THROUGH the
// production function rather than reimplemented here: a duplicate metric in the
// test is how a metric change passes its own tests (040 T033a moved this from a
// planar distance to the great-circle angle, and a local copy would have gone
// on agreeing with the retired version).
double sepDeg(const BeaconObservation& a, const BeaconObservation& b) {
    return static_cast<double>(autoc::eval::compute_pair_span(
               a.bearing_x_rad, a.bearing_y_rad,
               b.bearing_x_rad, b.bearing_y_rad)) * kRadToDeg;
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
    // 040 T061 — the old assertion here was `cep > 0.20` "edge_factor near 1 ⇒
    // cep near 0.3". That encoded the RETIRED position-only placeholder, in
    // which sitting near the frame edge was itself the definition of low
    // confidence. Quality is now signal-derived (FR-014), and frame position is
    // not a signal term, so the edge no longer degrades it. Asserting the
    // REPLACEMENT property instead: an edge beacon is still fully detected.
    EXPECT_LT(obs.cep, 0.5f)
        << "frame position must no longer drive quality — that was the placeholder";
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
    EXPECT_LT(obs.cep, 0.5f);  // see the note on the right-edge twin above
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

    // The contrast that motivates the mount change: the RETIRED centreline
    // position sat behind the disc. Written out literally rather than read from
    // CameraConfig, because T043 made the leading-edge mount the default — so
    // reading the config here would compare the new mount against itself and
    // the contrast would silently evaporate.
    const gp_vec3 retired(0.0f, 0.0f, -0.05f);
    EXPECT_TRUE(rayCrossesPropDisc(retired, retired + gp_vec3(10.0f, 0.0f, 0.0f), a))
        << "expected the retired centreline mount to sit behind the disc";
}

TEST(AirframeObstruction, ConfigDefaultMountMatchesTheMeasuredLeadingEdgeMount) {
    // T043 — two places name the baseline mount: the CameraConfig default (what
    // the code ships) and hb1LeadingEdgeCameraMount() (what the sketch measured).
    // They cannot be single-sourced without camera_config.h depending on the
    // obstruction header, so they are pinned equal instead.
    const gp_vec3 cfg = CameraConfig{}.mount_offset_body;
    const gp_vec3 measured = autoc::eval::hb1LeadingEdgeCameraMount();
    EXPECT_FLOAT_EQ(cfg.x(), measured.x());
    EXPECT_FLOAT_EQ(cfg.y(), measured.y());
    EXPECT_FLOAT_EQ(cfg.z(), measured.z())
        << "the shipped camera mount has drifted from the measured geometry";
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

TEST(AirframeObstruction, ObstructionShipsEnabledAtTheBaselineMount) {
    // T043 inverted this. Obstruction shipped DISABLED while the camera sat on
    // the centreline inside the prop radius; with the mount at the leading edge
    // it is on, and that is the switch US3 exists to justify.
    const AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    EXPECT_TRUE(a.enabled)
        << "T043 turns obstruction on; if this is false the baseline mount and "
           "the obstruction switch have come apart";
}

TEST(AirframeObstruction, DisabledObstructionNeverOccludes) {
    // The kill switch still has to work: with obstruction off, geometry is
    // irrelevant and nothing may be reported blocked or attenuated. Disabled
    // explicitly now rather than relying on the shipped default (T043 flipped
    // it), so this tests the switch instead of the config.
    const gp_vec3 mount = CameraConfig{}.mount_offset_body;
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    a.enabled = false;
    // Put the disc dead ahead so the test would certainly fire if enabled.
    a.prop_axis_y = mount.y();
    a.prop_axis_z = mount.z();

    const gp_vec3 target = mount + gp_vec3(10.0f, 0.0f, 0.0f);
    const ObstructionResult r = testObstruction(mount, target, a);
    EXPECT_FALSE(r.blocked);
    EXPECT_FLOAT_EQ(r.attenuation, 1.0f);
}

// ===========================================================================
// 040 US3 (T036-T040) — prove the mount clears the obstructions.
//
// The point of this block is that obstruction is a DESIGN CHOICE being
// validated, not a defect being modelled faithfully. So the assertions are
// about the mount: the wing contributes nothing, the propeller shadow lands
// where geometry says it should and far from the boresight, and an alternative
// mount can be priced without touching code.
// ===========================================================================

TEST(AirframeObstructionField, WingContributesNoObstructionAtBaselineMount) {
    // T036 (acceptance scenario 1). The camera sits 2 mm FORWARD of the wing
    // leading-edge plane, so for any forward ray (dx > 0, true everywhere in a
    // ±75° field) the wing is strictly behind the aperture and cannot be hit.
    // Swept over the whole field rather than argued, because the claim is what
    // makes the leading-edge mount worth its build cost.
    const CameraConfig cam{};
    const gp_vec3 mount = cam.mount_offset_body;
    AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    ASSERT_TRUE(a.enabled);

    int wing_hits = 0;
    int samples = 0;
    for (int iy = 0; iy < cam.pixels_v; iy += 4) {
        for (int ix = 0; ix < cam.pixels_h; ix += 4) {
            const double bx =
                pixelCentreDeg(ix, cam.pixels_h, cam.deg_per_px) * kDegToRad;
            const double by =
                pixelCentreDeg(iy, cam.pixels_v, cam.deg_per_px) * kDegToRad;
            const double th = std::sqrt(bx * bx + by * by);
            const double s = (th < 1e-12) ? 1.0 : std::sin(th) / th;
            const gp_vec3 dir(static_cast<gp_scalar>(std::cos(th)),
                              static_cast<gp_scalar>(s * bx),
                              static_cast<gp_scalar>(s * by));
            const gp_vec3 target = mount + dir * 10.0f;
            if (rayHitsBox(mount, target, a.wing_min, a.wing_max)) ++wing_hits;
            ++samples;
        }
    }
    ASSERT_GT(samples, 1000) << "sweep did not actually cover the field";
    EXPECT_EQ(wing_hits, 0)
        << wing_hits << " of " << samples
        << " field rays hit the wing slab — the leading-edge mount is supposed "
           "to eliminate wing obstruction entirely (FR-007a)";
}

TEST(AirframeObstructionField, PropShadowOnsetMatchesGeometricPrediction) {
    // T037 (acceptance scenario 1, second half). Closed form: the mount sits at
    // radial distance r_cam from the thrust axis and the disc reaches r_prop, so
    // a ray must deflect inboard by atan((r_cam − r_prop)/Δx) before it can
    // cross, where Δx is the axial distance from aperture to disc plane.
    const CameraConfig cam{};
    const gp_vec3 mount = cam.mount_offset_body;
    const AirframeObstruction a = autoc::eval::hb1AirframeObstruction();

    const double dy = static_cast<double>(a.prop_axis_y - mount.y());
    const double dz = static_cast<double>(a.prop_axis_z - mount.z());
    const double r_cam = std::sqrt(dy * dy + dz * dz);
    const double dx = static_cast<double>(a.prop_plane_x - mount.x());
    const double predicted_deg =
        std::atan((r_cam - static_cast<double>(a.prop_radius)) / dx) * kRadToDeg;

    const double onset = static_cast<double>(autoc::eval::obstructionOnsetDeg(
        cam, a, mount, 0.0f));

    // The sweep steps at 0.05°, so it can only ever land on or just past truth.
    EXPECT_NEAR(onset, predicted_deg, 0.2)
        << "onset " << onset << "° vs closed-form " << predicted_deg << "°";

    // THE DESIGN CLAIM, stated as a number: the shadow is far enough off
    // boresight that a tail-chased target never sits in it. Scoping put this at
    // ~41°; anything much smaller means the mount stopped being worth it.
    EXPECT_GT(onset, 35.0)
        << "prop shadow has crept toward the boresight — at " << onset
        << "° a chased target would sit in it";
}

TEST(AirframeObstructionField, MisalignedMountBringsTheShadowInboard) {
    // T037 (acceptance scenario 2 support). The mounting-error envelope is the
    // reason US3 tests at the extremes, not at nominal: a camera glued 20°
    // inboard does not move the airframe, it rotates the boresight, so the same
    // shadow arrives ~20° closer to the middle of the frame.
    const CameraConfig cam{};
    const gp_vec3 mount = cam.mount_offset_body;
    const AirframeObstruction a = autoc::eval::hb1AirframeObstruction();

    const double nominal =
        static_cast<double>(autoc::eval::obstructionOnsetDeg(cam, a, mount, 0.0f));
    const double clipped =
        static_cast<double>(autoc::eval::obstructionOnsetDeg(cam, a, mount, 20.0f));

    EXPECT_NEAR(clipped, nominal - 20.0, 0.3)
        << "a 20° inboard glue error must move onset in by 20°: nominal "
        << nominal << "° vs misaligned " << clipped << "°";
    // The spec's stated worst case (~21° from its own boresight). Pinned so the
    // number the mount decision rests on cannot drift unnoticed.
    EXPECT_GT(clipped, 15.0);
    EXPECT_LT(clipped, 27.0);
}

TEST(AirframeObstructionField, AlternativeMountReportsDifferentObstruction) {
    // T038 (acceptance scenario 3, FR-011a). A wing-TOP mount is priced by
    // changing configuration only — no code path here is mount-specific.
    const CameraConfig cam{};
    const AirframeObstruction a = autoc::eval::hb1AirframeObstruction();

    const autoc::eval::EffectiveField baseline =
        autoc::eval::computeEffectiveField(cam, a, cam.mount_offset_body);

    // Wing top, mid-chord, on the centreline: 1 cm above the wing's upper
    // surface, half way between LE and TE. Nothing but config changed.
    const gp_vec3 wing_top(
        static_cast<gp_scalar>(0.5 * (-0.330200 + -0.152400)),
        0.0f,
        static_cast<gp_scalar>(-0.044450 - 0.010));
    const autoc::eval::EffectiveField alternative =
        autoc::eval::computeEffectiveField(cam, a, wing_top);

    EXPECT_NE(baseline.blockedFraction(), alternative.blockedFraction())
        << "an alternative mount must report different obstruction without a "
           "code change, or mount options cannot be compared (FR-011a)";
    // Direction of the difference is itself the finding: the wing-top mount has
    // the wing ahead of and below it, and the nose box ahead of it.
    EXPECT_GT(alternative.blockedFraction(), baseline.blockedFraction())
        << "expected the wing-top mount to be obstructed MORE than the "
           "leading-edge mount — if not, the case for the LE mount is weaker "
           "than the scoping analysis claimed";
}

TEST(AirframeObstructionField, EffectiveFieldDiffersFromNominalByAJustifiedAmount) {
    // T040 (FR-012, SC-003). The effective field is the nominal rectangle minus
    // what the airframe takes. Two things must both hold: the loss is real (the
    // report is not silently zero), and it is SMALL — because at this mount the
    // only contributor is a propeller disc that attenuates rather than blocks.
    const CameraConfig cam{};
    const AirframeObstruction a = autoc::eval::hb1AirframeObstruction();
    const autoc::eval::EffectiveField f =
        autoc::eval::computeEffectiveField(cam, a, cam.mount_offset_body);

    // Nominal solid angle, closed form: ∫∫ (sin θ/θ) dθx dθy over the
    // rectangle. Checked loosely — the point is that the integrator is
    // integrating the sphere and not the flat rectangle, whose area would be
    // 2.094 × 1.571 = 3.29 sr.
    const double flat = static_cast<double>(cam.fovHDeg() * kDegToRad) *
                        static_cast<double>(cam.fovVDeg() * kDegToRad);
    EXPECT_LT(static_cast<double>(f.nominal_sr), flat)
        << "the sin θ/θ Jacobian must pull the true solid angle BELOW the flat "
           "rectangle's area; got " << f.nominal_sr << " vs flat " << flat;
    EXPECT_GT(static_cast<double>(f.nominal_sr), 0.5 * flat);

    // THE LOSS IS REAL AND IT IS THE POD NOSE, not the wing. The scoping
    // analysis said so ("wing occlusion is eliminated at any LE offset; the pod
    // nose shadows the same inboard region and merges into the same patch") and
    // the sweep agrees: ~3% of the field is hard-blocked, entering around 48°
    // inboard where the ray finally descends into the nose box before clearing
    // its forward face. The wing contributes exactly zero — see
    // WingContributesNoObstructionAtBaselineMount.
    EXPECT_GT(f.blockedFraction(), 0.005f)
        << "the pod nose reaches the inboard field at this mount; a zero here "
           "means the sweep or the nose geometry is wrong, not that the mount "
           "is perfect";
    EXPECT_LT(f.blockedFraction(), 0.10f)
        << "blocked " << f.blockedFraction() * 100.0f
        << "% of the field — more than the inboard nose patch can account for";

    // ATTRIBUTION, so the number above cannot be quietly ascribed to the wrong
    // primitive: collapse the nose box to nothing and the blockage must vanish
    // entirely. This is also the only assertion here that would catch the wing
    // starting to contribute.
    AirframeObstruction no_nose = a;
    no_nose.nose_min = gp_vec3(0.0f, 0.0f, 0.0f);
    no_nose.nose_max = gp_vec3(0.0f, 0.0f, 0.0f);
    const autoc::eval::EffectiveField without_nose =
        autoc::eval::computeEffectiveField(cam, no_nose, cam.mount_offset_body);
    EXPECT_LT(without_nose.blockedFraction(), 1e-6f)
        << "with the nose box collapsed, nothing opaque should block the field "
           "— so the blockage above is the nose and nothing else";

    // The propeller reaches the field too, and ITS loss is attenuation rather
    // than blockage (FR-009) — the distinction the report must preserve.
    EXPECT_GT(f.attenuatedFraction(), 0.0f)
        << "the prop disc is inside the 120°×90° field at this mount";
    EXPECT_LT(f.attenuatedFraction(), 0.25f)
        << "prop shadow covering more than a quarter of the field would "
           "contradict the ~42-61° inboard patch the geometry predicts";

    // Attenuated field is still usable, so it counts as clear. NEAR not FLOAT_EQ:
    // both sides are sums over ~4800 samples and differ in the last few bits.
    EXPECT_NEAR(f.clearFraction(), 1.0f - f.blockedFraction(), 1e-5f);
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
// SCOPE OF THE INVARIANCE CLAIM (040 T033a; research R2 as amended). Bearing is
// still an equidistant mapping, but the pair is no longer MEASURED in that
// plane. Planar Euclidean distance in (θx, θy) equals the great-circle angle
// exactly along a radius and over-reads TANGENTIALLY by θ/sin θ — +8.6% at 40°,
// +35% at the 75° frame diagonal. R2 originally accepted that residual as
// documented-not-fixed, on the grounds that it "makes a separate ray-angle span
// computation unnecessary", which put a position-dependent error in the sole
// range channel and contradicted SC-001's "at any orientation".
//
// compute_pair_span now reconstructs both unit rays and returns the angle
// between them, so SC-001 holds LITERALLY: invariant at any position in frame
// and any pair orientation, to within what the pixel grid permits. There is no
// residual left to pin, so the test that pinned it is gone — replaced by
// SeparationIsInvariantAtAnyPositionAndOrientation, which sweeps the cases the
// old metric failed.
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

TEST(CameraGridGeometry, SeparationIsInvariantAtAnyPositionAndOrientation) {
    // SC-001 as literally worded, which 040 T033a made achievable. A pair of a
    // FIXED true angular separation is walked out from the boresight to 40° off
    // axis and rotated through every orientation from radial to tangential; all
    // readings must agree with the truth and with each other.
    //
    // Construction is in 3D rather than in bearings, so the truth is exact by
    // construction rather than derived through the mapping under test: take a
    // central ray u₀ at field angle θ₀, build the orthonormal tangent basis
    // there (e_rad along increasing θ, e_tan across it), and place the pair at
    //     u± = cos(ψ/2)·u₀ ± sin(ψ/2)·(cos α·e_rad + sin α·e_tan)
    // which subtends exactly ψ, centred on u₀, oriented α from radial.
    const CameraConfig cam{};
    const double rad_per_px = static_cast<double>(cam.deg_per_px) * kDegToRad;
    const double psi = 20.0 * kDegToRad;

    // TWO-PART BOUND, and both parts are needed. 2% is the SC-001 projection
    // claim. The grid term is separate and unavoidable: each bearing quantises
    // independently on each axis, so up to 0.5 px per axis per beacon, and a
    // pair can accumulate ~1.4 px between them. 1.5 px is that worst case with
    // a little room, and it is stated in radians so it scales if the grid does.
    const double bound = 0.02 * psi + 1.5 * rad_per_px;

    double min_read = 1e9;
    double max_read = -1e9;
    double worst_tangential_planar = 0.0;
    double worst_tangential_truth = 0.0;

    for (const double theta0_deg : {0.0, 15.0, 30.0, 40.0}) {
        const double t0 = theta0_deg * kDegToRad;
        const gp_vec3 u0(static_cast<gp_scalar>(std::cos(t0)),
                         static_cast<gp_scalar>(std::sin(t0)),
                         0.0f);
        const gp_vec3 e_rad(static_cast<gp_scalar>(-std::sin(t0)),
                            static_cast<gp_scalar>(std::cos(t0)),
                            0.0f);
        const gp_vec3 e_tan(0.0f, 0.0f, 1.0f);

        for (const double alpha_deg : {0.0, 30.0, 45.0, 60.0, 90.0}) {
            const double a = alpha_deg * kDegToRad;
            const gp_vec3 off = static_cast<gp_scalar>(std::cos(a)) * e_rad +
                                static_cast<gp_scalar>(std::sin(a)) * e_tan;
            const gp_scalar c = static_cast<gp_scalar>(std::cos(psi / 2.0));
            const gp_scalar s = static_cast<gp_scalar>(std::sin(psi / 2.0));

            const BeaconObservation lo = obsAlong(c * u0 - s * off);
            const BeaconObservation hi = obsAlong(c * u0 + s * off);
            ASSERT_NE(lo.cep, kCepSentinelFloat)
                << "θ₀=" << theta0_deg << "° α=" << alpha_deg
                << "° gated — the sweep must stay inside the field and clear "
                   "the airframe, or it is measuring the wrong thing";
            ASSERT_NE(hi.cep, kCepSentinelFloat)
                << "θ₀=" << theta0_deg << "° α=" << alpha_deg << "° gated";

            const double read = sepDeg(lo, hi) * kDegToRad;
            EXPECT_NEAR(read, psi, bound)
                << "θ₀=" << theta0_deg << "° α=" << alpha_deg
                << "° read " << read * kRadToDeg << "° for a true "
                << psi * kRadToDeg << "° (SC-001)";

            min_read = std::min(min_read, read);
            max_read = std::max(max_read, read);

            // Keep the most tangential, most off-axis case's planar distance so
            // the assertion below can prove this sweep has teeth.
            if (alpha_deg == 90.0 && theta0_deg == 40.0) {
                worst_tangential_planar =
                    std::hypot(static_cast<double>(hi.bearing_x_rad - lo.bearing_x_rad),
                               static_cast<double>(hi.bearing_y_rad - lo.bearing_y_rad));
                worst_tangential_truth = read;
            }
        }
    }

    // The invariance claim itself: not merely "each is close to truth" but
    // "they all agree", which is what a range channel actually needs.
    EXPECT_LT(max_read - min_read, 2.0 * bound)
        << "spread across position and orientation was "
        << (max_read - min_read) * kRadToDeg << "°";

    // TEETH. The retired planar metric read this case θ/sin θ = 1.086× wide,
    // i.e. ~1.7° on a 20° pair, which is outside the bound above. If this ever
    // stops holding, the sweep has gone slack and would pass on either metric.
    ASSERT_GT(worst_tangential_truth, 0.0) << "worst-case config never ran";
    EXPECT_GT(worst_tangential_planar - worst_tangential_truth, bound)
        << "the planar distance must land OUTSIDE the bound at 40° tangential, "
           "or this test no longer discriminates between the two metrics";
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
        // EXACT inverse of the pair geometry, not the small-angle L/ψ. Now that
        // span is a true great-circle angle, an isoceles pair of baseline L at
        // range R subtends ψ = 2·atan((L/2)/R), so R = (L/2)/tan(ψ/2) inverts it
        // with no approximation left in the chain.
        const double inferred_m =
            (separation_m / 2.0) / std::tan(span_rad / 2.0);

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
    // The ~1% that legitimately remains is GRID CONVEXITY, not calibration, and
    // it survived the T033a metric fix unchanged because it was never a metric
    // error: range is a convex decreasing function of span, so E[R(span)] >
    // R(E[span]) for a span carrying symmetric quantisation error (Jensen). At
    // 25 m the pair subtends only ~4.7 px, so ±1 px of grid error is enough to
    // bend the mean upward by about a percent. That is a genuine property of
    // inferring range from a discrete sensor, and 040 does not pretend
    // otherwise — the bound below sits above it and far below any plausible
    // constant error.
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
    // 041 T041b (FR-029) — the field the ordered hardware actually gives:
    // 120° H × 75° V from a 320×200 grid at 0.375°/px. V was 90° and optimistic
    // by 15°. H is unchanged. Asserting the DERIVED value is the FR-003
    // property — field and resolution cannot disagree, because there is one knob.
    EXPECT_NEAR(static_cast<double>(cam.fovHDeg()), 120.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(cam.fovVDeg()), 75.0, 1e-9);
    // The half-angles the projection actually clips against (camera_projection.h
    // documents ≈±1.047 / ±0.654 rad); pinned so that comment cannot go stale.
    // Tolerance 1e-6, not 1e-9: gp_scalar is float, so ~1e-7 is the type's own
    // resolution here. Still four orders of magnitude below the 0.13 rad the
    // 90° → 75° change moves it, which is what this needs to catch.
    EXPECT_NEAR(static_cast<double>(cam.halfFovHRad()), 1.0471975511965976, 1e-6);
    EXPECT_NEAR(static_cast<double>(cam.halfFovVRad()), 0.6544984694978736, 1e-6);

    // Resolution and field cannot disagree, because there is only one knob:
    // halving the pixel pitch halves both fields, and no setter exists that
    // could contradict it.
    cam.deg_per_px = static_cast<gp_scalar>(0.1875);
    EXPECT_NEAR(static_cast<double>(cam.fovHDeg()), 60.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(cam.fovVDeg()), 37.5, 1e-9);
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

// ===========================================================================
// 040 T056 (FR-033, SC-011) — TWO ENVELOPES, NOT ONE.
//
// Detection and range-inference have different reach, and the whole point is
// that NEITHER may be reported as usable outside its own:
//
//   bearing                    → to the ASSERTED detection range (~100 m)
//   separation-derived range   → dies at ~24 m, when the pair stops resolving
//
// At 120° over 320 px the 0.772 m pair subtends ≈1 px at 100 m. Reporting a
// range from that would be reporting a quantisation artefact as a measurement —
// which is exactly what the code did through 039, where separation was computed
// from whatever the two (identical) pixels gave.
//
// The CONSEQUENCE is intended and is behaviour M2 has never trained against:
// the controller experiences a genuine perceptual REGIME CHANGE as it closes —
// bearing-only at range, bearing-plus-range inside.
// ===========================================================================

namespace {

// Chase at the origin looking down +x; target dead ahead at `range_m`, wings
// level, so the beacon pair straddles the boresight horizontally.
// `double range_m` here is raw-ok per the test-scaffolding block note above:
// the sweep positions the target more precisely than the code under test
// resolves, which is what lets a crossover be located to a quarter metre.
autoc::eval::PerceptionTickResult tickAtRange(double range_m,  // raw-ok: test-reference geometry
                                              autoc::eval::PerceptionCarryState& carry) {
    AircraftState chase;
    chase.setPosition(gp_vec3(0.0f, 0.0f, 0.0f));
    chase.setOrientation(gp_quat::Identity());

    SourceTickSample target{};
    target.position = gp_vec3(static_cast<gp_scalar>(range_m), 0.0f, 0.0f);
    target.orientation = gp_quat::Identity();
    target.velocity = gp_vec3(0.0f, 0.0f, 0.0f);

    autoc::eval::TickRuleConfig cfg;
    cfg.camera = CameraConfig{};
    cfg.beacon_left = BeaconConfig{};
    cfg.beacon_left.mount_body = gp_vec3(0.0f, -autoc::eval::kBeaconMountY, 0.0f);
    cfg.beacon_left.emission_axis_body = gp_vec3(0.0f, -1.0f, 0.0f);
    cfg.beacon_right = BeaconConfig{};
    cfg.beacon_right.mount_body = gp_vec3(0.0f, +autoc::eval::kBeaconMountY, 0.0f);
    cfg.beacon_right.emission_axis_body = gp_vec3(0.0f, +1.0f, 0.0f);
    // Airframe parked far behind so obstruction plays no part here.
    cfg.airframe = obstructionWithWingBox(gp_vec3(-100.0f, -1.0f, -1.0f),
                                          gp_vec3(-99.0f, +1.0f, +1.0f));
    cfg.cep_gate_threshold = static_cast<gp_scalar>(1.25);
    cfg.obstruction_mount_offset = cfg.camera.mount_offset_body;
    cfg.signal = hb1SignalConfig();
    cfg.acquisition = hb1AcquisitionConfig();
    cfg.control_interval_ms = static_cast<gp_scalar>(50);

    return autoc::eval::projectPerceptionTick(chase, target, cfg, carry);
}

// Run enough ticks at a fixed range to get past acquisition into TRACKING, then
// return the settled tick. Acquisition timing is T051's business, not this
// test's — here it is setup.
autoc::eval::PerceptionTickResult settledAtRange(double range_m) {
    autoc::eval::PerceptionCarryState carry;
    carry.reset();
    autoc::eval::PerceptionTickResult r = tickAtRange(range_m, carry);
    for (int i = 0; i < 12; ++i) r = tickAtRange(range_m, carry);
    return r;
}

}  // namespace

TEST(TwoEnvelopes, BearingSurvivesToTheAssertedDetectionRange) {
    // FR-033a — the detection envelope is ASSERTED, so bearing must still be
    // reported at 95 m even though the link budget there is at its floor.
    const autoc::eval::PerceptionTickResult far = settledAtRange(95.0);

    EXPECT_LT(far.left.cep, autoc::eval::kCepSentinelThreshold)
        << "bearing must reach the asserted detection range";
    EXPECT_LT(far.right.cep, autoc::eval::kCepSentinelThreshold);
    EXPECT_NE(far.left.raw_px_x, autoc::eval::kPixelSentinel);
}

TEST(TwoEnvelopes, BeyondTheDetectionRangeNothingIsReported) {
    // The complement — without it the test above would pass on a model with no
    // detection envelope at all.
    const autoc::eval::PerceptionTickResult beyond = settledAtRange(140.0);

    EXPECT_GE(beyond.left.cep, autoc::eval::kCepSentinelThreshold);
    EXPECT_GE(beyond.right.cep, autoc::eval::kCepSentinelThreshold);
    EXPECT_EQ(beyond.left.raw_px_x, autoc::eval::kPixelSentinel);
}

TEST(TwoEnvelopes, SeparationRangeDiesLongBeforeBearingDoes) {
    // THE ASSERTION THAT MATTERS. Inside the resolving limit span is a real
    // measurement; outside it, span reads exactly neutral 0 while the BEARINGS
    // are still perfectly good. That asymmetry IS the two-envelope rule.
    const autoc::eval::PerceptionTickResult close = settledAtRange(15.0);
    const autoc::eval::PerceptionTickResult far = settledAtRange(60.0);

    EXPECT_GT(close.record.span, 0.0f) << "inside ~24 m, range from separation is usable";

    EXPECT_EQ(far.record.span, 0.0f)
        << "past the resolving limit, separation-derived range must be "
           "UNAVAILABLE, not merely imprecise";
    // ...and yet the beacons are plainly still seen. This is the pair of facts
    // that has to hold simultaneously.
    EXPECT_LT(far.left.cep, autoc::eval::kCepSentinelThreshold);
    EXPECT_LT(far.right.cep, autoc::eval::kCepSentinelThreshold);
}

TEST(TwoEnvelopes, TheSeparationCrossoverIsSetByTheQUANTISEDPixelGap) {
    // The crossover must be a property of the GRID, not a tuned constant. The
    // obvious way to assert that — predict a range from continuous geometry —
    // is WRONG, and instructively so: 5 px at 0.375°/px is 1.875°, which a
    // 0.772 m pair subtends at ≈23.6 m, but the measured crossover sits near
    // 27.75 m. Two real effects the continuous number ignores:
    //
    //   1. QUANTISATION ROUNDS OUTWARD for a boresight-straddling pair. With an
    //      even pixel count the boresight falls on a pixel BOUNDARY (T031), so
    //      two beacons at ±2.1 px land on centres ±2.5 px apart — a true 4.25 px
    //      gap reads as exactly 5. The gate sees pixels, because the sensor only
    //      has pixels.
    //   2. THE CAMERA IS NOT ON THE CENTRELINE. It sits 8″ outboard and ~1¼″
    //      above the thrust line, so the pair does not straddle the boresight
    //      symmetrically. This is the same parallax that makes the aim point sit
    //      a constant ~26.6% of the target's apparent wingspan off image centre
    //      at EVERY range.
    //
    // So the test asserts the INVARIANT rather than a derived range: wherever
    // the crossover falls, it falls exactly where the quantised gap reaches the
    // configured limit, and one step further out it has not.
    const gp_scalar limit_px = hb1SignalConfig().separation_min_px;

    auto gapPx = [](const autoc::eval::PerceptionTickResult& r) -> double {
        if (r.left.raw_px_x == autoc::eval::kPixelSentinel ||
            r.right.raw_px_x == autoc::eval::kPixelSentinel) {
            return -1.0;
        }
        const double dx = static_cast<double>(r.left.raw_px_x - r.right.raw_px_x);
        const double dy = static_cast<double>(r.left.raw_px_y - r.right.raw_px_y);
        return std::sqrt(dx * dx + dy * dy);
    };

    double crossover = -1.0;
    for (double r = 40.0; r >= 8.0; r -= 0.25) {
        if (settledAtRange(r).record.span > 0.0f) {
            crossover = r;
            break;
        }
    }
    ASSERT_GT(crossover, 0.0) << "span must become available somewhere inside 40 m";

    // At the crossover the quantised gap has just reached the limit...
    EXPECT_GE(gapPx(settledAtRange(crossover)), static_cast<double>(limit_px));
    // ...and one sweep step further out it had not. This is what ties the
    // envelope to the grid: move separation_min_px or deg_per_px and the
    // crossover moves with them, with nothing else to update.
    EXPECT_LT(gapPx(settledAtRange(crossover + 0.25)), static_cast<double>(limit_px));

    // Sanity band, deliberately loose: it must be in the tens of metres, not at
    // the detection edge and not in the weeds.
    EXPECT_GT(crossover, 15.0);
    EXPECT_LT(crossover, 40.0);
}

TEST(TwoEnvelopes, MergedBlobsStillDetectButCarryIdentityUncertainty) {
    // FR-016 + FR-017d together. At long range the pair shares a detector
    // element: 031 proves both codes still decode, so detection MUST survive —
    // but there is one blob and no way to assign two identities to two
    // positions, so quality must carry that.
    const autoc::eval::PerceptionTickResult merged = settledAtRange(95.0);

    ASSERT_LT(merged.left.cep, autoc::eval::kCepSentinelThreshold)
        << "a shared detector element is FIELD-PROVEN, not a failure mode";
    EXPECT_GT(merged.left.cep, 0.5f)
        << "unresolved identity must inflate quality (FR-017d)";
    EXPECT_EQ(merged.record.span, 0.0f)
        << "what merging removes is spatial separation, hence range";
}

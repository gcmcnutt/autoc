#pragma once

// 030 M5 — Beacon projection module (FR-005 + FR-007 + FR-017).
//
// Analytic EQUIDISTANT (f-theta / spherical) projection of a wingtip beacon
// through the chase-craft camera, collapsed to (x, y, CEP) per the
// perception interface contract. 038 t9: NDC ∝ angle off the optical axis
// (was rectilinear tan(angle) pre-t9), so the beacon-pair span is a clean,
// position-invariant angular quantity; swap in the real lens's projection
// function here once camera hardware is chosen (BACKLOG 040 Q3).
//
// PHYSICAL-CAMERA ASSUMPTION (operator 2026-07-09): the hardware model is
// still a CLASSIC PLANAR (rectilinear-lens) camera. A planar sensor reads
// pixels ∝ tan(angle); the perception front-end (040: pixels → x,y,CEP)
// applies the known intrinsics to remap pixel centroids to ANGLES — a
// deterministic, information-free calibration step. The sim simply skips
// the pixel stage and emits the angle domain directly. Angular NDC here is
// a REPRESENTATION choice for the NN interface, not a fisheye-lens claim.
// See `specs/030-tracker-mode/contracts/beacon_projection_api.md` and
// `data-model.md` §4 for the source contract.
//
// 040 T031 — bearings are now ANGLES IN RADIANS quantised on the sensor's
// pixel grid, replacing the ±1 per-axis NDC encoding. The retired encoding
// normalised each axis by its OWN half-FOV, so the two axes carried different
// angular scales and a horizontal separation read 60/45 = 1.333× its vertical
// twin — a 33% orientation error sitting in the sole range channel (FR-002).
//
// RESIDUAL, documented not corrected (research R2, contract §6): under
// equidistant mapping, Euclidean distance in (θx, θy) equals the great-circle
// angle EXACTLY along a radius and over-reads TANGENTIALLY by θ/sin θ — +21%
// at the frame corner. This is the only remaining position dependence, and it
// is what makes a separate ray-angle span computation unnecessary. Pinned by
// tests/beacon_projection_tests.cc CameraGridGeometry.Tangential*.
//
// Coordinate convention (chase body frame, NED): +x forward, +y right,
// +z down. Camera optical axis = body +x by default; bearing_x_rad runs
// left→right (image right positive), bearing_y_rad runs top→bottom
// (pixel-coord convention: image down positive). Each is bounded by the
// DERIVED per-axis half-FOV (CameraConfig::halfFov*Rad()).
//
// Determinism contract (FR-009): same `ProjectionInput` ⇒ bit-identical
// `BeaconObservation` across invocations. No PRNG, no clock, no thread-
// dependent state.

#include <cstdint>

#include "autoc/types.h"
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/airframe_occlusion.h"
#include "autoc/eval/camera_config.h"

namespace autoc::eval {

// CEP encoding sentinels (R6 + R7). The fp32 sentinel marker (1.5f) is
// distinguishable from any in-range CEP value (which sits in [0, 1.0]);
// the int8 sentinel (INT8_MIN = -128) is reserved separately from the
// valid quantized range [0, +127].
constexpr float kCepSentinelThreshold = 1.25f;  // raw-ok: NN-byte-format perception interface (fp32 contract). any cep ≥ this ⇒ invisible
constexpr float kCepSentinelFloat = 1.5f;       // raw-ok: NN-byte-format perception interface (fp32 contract). dequantized sentinel marker

// 040 T032 — pixel-index sentinel. Distinguishable from every valid index,
// which is [0, pixels−1] and therefore never negative.
constexpr int16_t kPixelSentinel = INT16_MIN;

// Per-tick perception output for one (camera, beacon) pair: BEARING plus
// quality, per contracts/perception-interface.md §1.
//
// 040 T032 — the int8 bearing encoding is GONE. It was a second, independent
// resolution model layered on top of the sensor grid, and the two disagreed:
// 2/254 of a 120° field is 0.472°/LSB horizontally against a 0.375° pixel —
// 26% COARSER than the sensor it claimed to model — while being 6% FINER
// vertically, over a 90° field. The pixel grid IS the resolution model, so the
// int8 layer was redundant where it agreed and wrong where it did not.
//
// Bearings are now angles in RADIANS, isotropic across both axes, quantised
// on the pixel grid. `raw_px_*` carries the pixel index the angle came from,
// for the dmp and the renderer.
struct BeaconObservation {
    // NN-facing values (fp32 at the input boundary). Raw fp32, not
    // `gp_scalar`, because they participate in cereal byte-format for the dmp
    // schema; flipping `gp_scalar` to fp64 would change the on-disk layout.
    //
    // Range is ±halfFovHRad() / ±halfFovVRad() — ≈±1.047 / ±0.785 at the
    // 320×240 @ 0.375°/px baseline. Both axes carry the SAME angular scale, so
    // a given angular separation reads identically at any orientation (FR-002).
    float bearing_x_rad;  // raw-ok: cereal byte-format member (dmp). right positive
    float bearing_y_rad;  // raw-ok: cereal byte-format member (dmp). down positive (pixel convention)
    float cep;            // raw-ok: cereal byte-format member (dmp). [0, 1.0] visible, == kCepSentinelFloat invisible

    // For dmp / renderer: the sensor-grid indices the bearings quantised to.
    // kPixelSentinel ⇔ invisible.
    int16_t raw_px_x;
    int16_t raw_px_y;
    int8_t raw_cep_int8;  // INT8_MIN ⇔ invisible

    // 030 M8a — cereal serialize for dmp output (cameraViewList).
    template <class Archive>
    void serialize(Archive& ar) {
        ar(bearing_x_rad, bearing_y_rad, cep, raw_px_x, raw_px_y, raw_cep_int8);
    }
};

// Full input to the projection module.
struct ProjectionInput {
    // Chase craft pose (world frame, NED meters / body→world quat).
    gp_vec3 chase_position_world;
    gp_quat chase_orientation_world;

    // Target craft pose (world frame).
    gp_vec3 target_position_world;
    gp_quat target_orientation_world;

    // Beacon mount + emission axis in target body frame.
    gp_vec3 beacon_mount_target_body;
    gp_vec3 beacon_emission_axis_target_body;

    // Camera mount + orientation in chase body frame.
    gp_vec3 camera_mount_chase_body;
    gp_quat camera_orientation_chase_body;

    // Configs.
    CameraConfig camera;
    BeaconConfig beacon;

    // 040 T014 — chase airframe obstruction (body frame). Replaces the
    // single-AABB AirframeProxy, which modelled a thin wing as a solid
    // brick AND placed the default camera mount exactly on a box face.
    AirframeObstruction chase_airframe;
};

// Project one beacon. Sentinel cases (behind camera, occluded by airframe,
// outside emission cone, outside the derived FOV) all set
// `cep = kCepSentinelFloat`, `raw_cep_int8 = INT8_MIN` and
// `raw_px_* = kPixelSentinel`; bearings are zero in sentinel.
BeaconObservation projectBeacon(const ProjectionInput& input);

// ---------------------------------------------------------------------------
// 040 T031 — sensor-grid quantisation (FR-001).
//
// Pixel CENTRES sit at (i − (n−1)/2)·rad_per_px for i ∈ [0, n−1], so the outer
// edges land at ±n/2·rad_per_px and the derived FOV is exactly n × deg_per_px.
// With an even pixel count the boresight falls on a pixel BOUNDARY, not a
// centre — so a perfectly centred beacon reports ±half a pixel, never exactly
// zero. That is the physically correct behaviour of an even-width sensor, and
// it is preserved rather than papered over.
// ---------------------------------------------------------------------------

// Nearest pixel index to `bearing_rad`, clamped to [0, pixels−1]. Callers must
// reject out-of-field bearings BEFORE calling; the clamp is a safety net for
// floating-point edge cases, not a visibility test.
int16_t bearingToPixel(gp_scalar bearing_rad, int pixels, gp_scalar rad_per_px);

// Exact centre bearing of pixel `px`. Inverse of the above up to the
// quantisation the grid imposes by design.
gp_scalar pixelToBearing(int16_t px, int pixels, gp_scalar rad_per_px);

// CEP quantisation (R7). `quantize_cep` reserves INT8_MIN for the sentinel and
// otherwise clamps to [0, 1] before rounding to [0, +127]; the negative range
// [-127, -1] is unused. `float` is `// raw-ok:` per Principle VI — these are
// int8 ↔ fp32 NN-byte-format primitives and the byte format is
// xiao-firmware-locked at fp32, so the `gp_scalar` alias does not apply.
//
// US4 replaces CEP with a signal-derived quality value (FR-014); this encoding
// survives only until then.
int8_t quantize_cep(float cep_in_zero_one_or_sentinel); // raw-ok: NN-byte-format conversion primitive
float dequantize_cep(int8_t q);                        // raw-ok: NN-byte-format conversion primitive

}  // namespace autoc::eval

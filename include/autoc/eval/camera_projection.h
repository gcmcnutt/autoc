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
// (Those 60/45 half-angles were the pre-041 grid; the field is now 120°×75°.
// The ratio is historical — it describes the bug that was removed, not today's
// geometry, and is left as written because it explains the retired encoding.)
//
// 040 T033a — SEPARATION IS MEASURED ON THE SPHERE. Under equidistant mapping,
// Euclidean distance in (θx, θy) equals the great-circle angle EXACTLY along a
// radius but over-reads TANGENTIALLY by θ/sin θ (+35% at the 75° diagonal).
// R2 originally accepted that residual; it is now CORRECTED in
// `compute_pair_span`, which reconstructs both unit rays and returns the angle
// between them — see include/autoc/eval/derived_features.h. Nothing in the
// projection changed: bearing is still equidistant (θx, θy). Pinned by
// tests/beacon_projection_tests.cc
// CameraGridGeometry.SeparationIsInvariantAtAnyPositionAndOrientation.
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
#include "autoc/eval/signal_model.h"

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
    // Range is ±halfFovHRad() / ±halfFovVRad() — ≈±1.047 / ±0.654 at the
    // 320×200 @ 0.375°/px baseline (120° H × 75° V). Both axes carry the SAME
    // angular scale, so a given angular separation reads identically at any
    // orientation (FR-002).
    // 041 T041b — was ±0.785 (90° V) at the retired 320×240 grid. The field is
    // DERIVED from the grid (FR-003), so this comment is the only place the two
    // could disagree; a test pins the derivation rather than trusting it.
    float bearing_x_rad;  // raw-ok: cereal byte-format member (dmp). right positive
    float bearing_y_rad;  // raw-ok: cereal byte-format member (dmp). down positive (pixel convention)
    float cep;            // raw-ok: cereal byte-format member (dmp). [0, 1.0] visible, == kCepSentinelFloat invisible

    // For dmp / renderer: the sensor-grid indices the bearings quantised to.
    // kPixelSentinel ⇔ invisible.
    int16_t raw_px_x;
    int16_t raw_px_y;
    int8_t raw_cep_int8;  // INT8_MIN ⇔ invisible

    // ----- 040 T089 (FR-028) — DIAGNOSTIC ONLY, never controller inputs -------
    //
    // These exist because without them the renderer cannot show the thing you
    // most want to look at: whether a dropout coasted warm and snapped back, or
    // expired cold. FR-017b keeps the state machine internal to PERCEPTION —
    // "internal" means it never becomes an NN input, not that it may not be
    // recorded. The input vector stays at 58 (FR-006).
    float raw_margin;   // raw-ok: cereal byte-format member (dmp). per-chip SNR in dB, cdma penalty applied
    int8_t lock_state;  // LockState as int8; values pinned, see acquisition_state.h

    // 030 M8a — cereal serialize for dmp output (cameraViewList).
    // 040: appended at the end per the in-code convention — no
    // CEREAL_CLASS_VERSION bump, old dmps orphaned (greenfield policy).
    template <class Archive>
    void serialize(Archive& ar) {
        ar(bearing_x_rad, bearing_y_rad, cep, raw_px_x, raw_px_y, raw_cep_int8,
           raw_margin, lock_state);
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

    // 040 T074 — the mount the OBSTRUCTION test rays from, which US6 lets drift
    // from the bearing mount by up to ±5 mm. Deliberately separate: ±5 mm is
    // 0.03° at 10 m (nothing for bearing) but swings propeller clearance ~15%,
    // because the clear cone is atan((h − r_tip)/d) with a small numerator.
    gp_vec3 obstruction_mount_chase_body;

    // Configs.
    CameraConfig camera;
    BeaconConfig beacon;

    // 040 T057 — the link budget (FR-014/FR-015). Range and emission aspect now
    // enter perception; through 039 neither did.
    SignalConfig signal;

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

// ---------------------------------------------------------------------------
// 040 T044 (FR-012, SC-003) — the EFFECTIVE field of view.
//
// The nominal field is a rectangle the grid defines. The effective field is
// what is left after the aircraft's own wing, nose and propeller take their
// share, and publishing the difference is the point of US3: obstruction here is
// a design choice to VALIDATE, not a defect to model faithfully.
//
// SOLID-ANGLE WEIGHTED, deliberately. Under an equidistant mapping equal areas
// in (θx, θy) do NOT subtend equal solid angle — the Jacobian is sin θ/θ — so
// counting samples uniformly would over-weight the frame corners, which is
// exactly where obstruction lives. An unweighted count would flatter or damn
// the mount for the wrong reason.
// ---------------------------------------------------------------------------

struct EffectiveField {
    // Solid angle of the nominal rectangle, steradians.
    gp_scalar nominal_sr;
    // Solid angle still fully usable: neither blocked nor attenuated.
    gp_scalar clear_sr;
    // Blocked by an opaque primitive (wing or nose) — beacon simply not seen.
    gp_scalar blocked_sr;
    // Crossed by the propeller disc: visible but attenuated (FR-009), so it is
    // NOT lost field and is reported apart from `blocked_sr` rather than summed
    // into it.
    gp_scalar attenuated_sr;

    gp_scalar blockedFraction() const { return blocked_sr / nominal_sr; }
    gp_scalar attenuatedFraction() const { return attenuated_sr / nominal_sr; }
    gp_scalar clearFraction() const { return clear_sr / nominal_sr; }
};

// Sweep the nominal field from `camera_mount_body` and integrate what the
// airframe takes. `stride_px` subsamples the grid (1 = every pixel); the
// integral is insensitive to it well before 1, and the default keeps the sweep
// cheap enough for a report. `probe_range_m` is how far out the test target
// sits — obstruction is near-field geometry, so any range past the airframe
// gives the same answer.
EffectiveField computeEffectiveField(const CameraConfig& camera,
                                     const AirframeObstruction& airframe,
                                     const gp_vec3& camera_mount_body,
                                     int stride_px = 4,
                                     gp_scalar probe_range_m =
                                         static_cast<gp_scalar>(10));

// ---------------------------------------------------------------------------
// 040 T045 (FR-007b, SC-007) — obstruction ONSET.
//
// The angle from the camera's own boresight at which the airframe first takes
// signal, sweeping inboard — toward the thrust axis, which is where every
// primitive lives. Reported per mount so the distribution across the
// mounting-error envelope can be assembled by the caller.
//
// Returns a negative value if the sweep finds no obstruction anywhere in the
// field, which is a meaningful answer and not an error: it is what a mount that
// fully clears the airframe looks like.
// ---------------------------------------------------------------------------

gp_scalar obstructionOnsetDeg(const CameraConfig& camera,
                              const AirframeObstruction& airframe,
                              const gp_vec3& camera_mount_body,
                              gp_scalar boresight_tilt_inboard_deg =
                                  static_cast<gp_scalar>(0));

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

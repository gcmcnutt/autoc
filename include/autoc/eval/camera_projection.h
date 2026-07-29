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
// Coordinate convention (chase body frame, NED): +x forward, +y right,
// +z down. Camera optical axis = body +x by default; screen_x ∈ [-1, +1]
// runs left→right (image right is positive); screen_y ∈ [-1, +1] runs
// top→bottom (pixel-coord convention: image down is positive); ±1 is the
// per-axis FOV edge in ANGLE (screen = θ·dir / (fov/2)).
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

// Per-tick perception output for one (camera, beacon) pair. `screen_x`,
// `screen_y`, and `cep` are the NN-facing dequantized fp32 values; the
// `raw_*_int8` fields preserve the on-wire quantized state for the M2 dmp.
struct BeaconObservation {
    // NN-facing dequantized values (fp32 at the input boundary). All three
    // fields are raw fp32, not `gp_scalar`, because they participate in
    // cereal byte-format for the M8 v=2 dmp schema; flipping `gp_scalar`
    // to fp64 would change the on-disk byte layout.
    float screen_x;       // raw-ok: cereal byte-format member (M8 dmp). [-1, +1] uncalibrated
    float screen_y;       // raw-ok: cereal byte-format member (M8 dmp). [-1, +1]
    float cep;            // raw-ok: cereal byte-format member (M8 dmp). [0, 1.0] visible, == kCepSentinelFloat invisible

    // For dmp / renderer:
    int8_t raw_x_int8;
    int8_t raw_y_int8;
    int8_t raw_cep_int8;  // INT8_MIN ⇔ invisible

    // 030 M8a — cereal serialize for v=2 dmp output (cameraViewList).
    // Wire-format note: the int8 raw_* fields are the canonical state;
    // the fp32 dequantized values are derived from them at render-time.
    // Both are serialized for renderer convenience (no dequantize cost
    // in the playback path).
    template <class Archive>
    void serialize(Archive& ar) {
        ar(screen_x, screen_y, cep, raw_x_int8, raw_y_int8, raw_cep_int8);
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

// Project one beacon. Returns dequantized fp32 values + raw int8 storage.
// Sentinel cases (behind camera, occluded by airframe, outside emission
// cone, outside FOV) all set `cep = kCepSentinelFloat` and
// `raw_cep_int8 = INT8_MIN`; `screen_x` / `screen_y` are zero in sentinel.
BeaconObservation projectBeacon(const ProjectionInput& input);

// Quantization round-trip primitives (R7). `quantize_xy` clamps to [-1, +1]
// before rounding to int8 ∈ [-127, +127]; `quantize_cep` reserves INT8_MIN
// for the sentinel and otherwise clamps to [0, 1] before rounding to
// [0, +127]. The negative int8 range [-127, -1] is unused for CEP.
//
// All `float` here is `// raw-ok:` per Principle VI: these are the int8 ↔
// fp32 NN-byte-format conversion primitives — the byte format is
// xiao-firmware-locked at fp32, so the alias `gp_scalar` doesn't apply.
int8_t quantize_xy(float v_in_minus_one_plus_one);    // raw-ok: NN-byte-format conversion primitive
int8_t quantize_cep(float cep_in_zero_one_or_sentinel); // raw-ok: NN-byte-format conversion primitive
float dequantize_xy(int8_t q);                        // raw-ok: NN-byte-format conversion primitive
float dequantize_cep(int8_t q);                       // raw-ok: NN-byte-format conversion primitive

}  // namespace autoc::eval

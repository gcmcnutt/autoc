// 030 M5 — Beacon projection module (FR-005 + FR-007 + FR-017).
//
// Analytic EQUIDISTANT (f-theta / spherical) projection (038 t9 — was
// rectilinear/pinhole pre-t9), sentinel handling, int8 quantization round-
// trip, and airframe obstruction (wing slab / pod nose / prop disc). Uses the NED
// body-frame convention (+x forward, +y right, +z down) directly; see
// `include/autoc/eval/camera_projection.h` for the convention contract.

#include "autoc/eval/camera_projection.h"

#include "autoc/eval/airframe_occlusion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autoc::eval {

namespace {

constexpr gp_scalar kPi = static_cast<gp_scalar>(3.14159265358979323846);

// Convert full FOV (degrees) to the half-angle in radians. Equidistant
// (f-theta) normalization: `screen = θ·dir / (fov/2)` lands exactly at ±1
// when the beacon ray sits at the per-axis FOV edge.
inline gp_scalar fovHalfRad(gp_scalar fov_full_deg) {
    return fov_full_deg * (kPi / static_cast<gp_scalar>(360));
}

// Sentinel-emitting BeaconObservation: cep = kCepSentinelFloat,
// raw_cep_int8 = INT8_MIN; screen coords zeroed (FR-007 — screen values
// arbitrary when invisible; choose zero for renderer hygiene).
inline BeaconObservation sentinelObservation() {
    BeaconObservation obs;
    obs.screen_x = 0.0f;
    obs.screen_y = 0.0f;
    obs.cep = kCepSentinelFloat;
    obs.raw_x_int8 = 0;
    obs.raw_y_int8 = 0;
    obs.raw_cep_int8 = INT8_MIN;
    return obs;
}

}  // namespace

// ---------------------------------------------------------------------------
// Quantization round-trip primitives (R7). All `float` here is // raw-ok: per
// Principle VI — int8 ↔ fp32 NN-byte-format primitives, byte format is
// xiao-firmware-locked at fp32.
// ---------------------------------------------------------------------------

int8_t quantize_xy(float v_in_minus_one_plus_one) {              // raw-ok: NN-byte-format primitive
    const float clamped = std::clamp(v_in_minus_one_plus_one, -1.0f, 1.0f);  // raw-ok: NN-byte-format primitive
    return static_cast<int8_t>(std::lround(clamped * 127.0f));
}

int8_t quantize_cep(float cep_in_zero_one_or_sentinel) {         // raw-ok: NN-byte-format primitive
    if (cep_in_zero_one_or_sentinel >= kCepSentinelThreshold) return INT8_MIN;
    const float clamped = std::clamp(cep_in_zero_one_or_sentinel, 0.0f, 1.0f); // raw-ok: NN-byte-format primitive
    return static_cast<int8_t>(std::lround(clamped * 127.0f));
}

float dequantize_xy(int8_t q) {                                  // raw-ok: NN-byte-format primitive
    return static_cast<float>(q) / 127.0f;                       // raw-ok: NN-byte-format primitive
}

float dequantize_cep(int8_t q) {                                 // raw-ok: NN-byte-format primitive
    return (q == INT8_MIN) ? kCepSentinelFloat : static_cast<float>(q) / 127.0f; // raw-ok: NN-byte-format primitive
}

// ---------------------------------------------------------------------------
// projectBeacon — main projection entry point.
// ---------------------------------------------------------------------------

BeaconObservation projectBeacon(const ProjectionInput& input) {
    // Step 1 — beacon position in world frame.
    const gp_vec3 beacon_offset_world =
        input.target_orientation_world * input.beacon_mount_target_body;
    const gp_vec3 beacon_world =
        input.target_position_world + beacon_offset_world;

    // Step 2 — beacon position in chase body frame.
    const gp_vec3 beacon_in_chase_body =
        input.chase_orientation_world.inverse() *
        (beacon_world - input.chase_position_world);

    // Step 3 — beacon position in camera frame. With the v1 default
    // (camera_orientation = identity), camera frame == body frame.
    const gp_vec3 beacon_in_camera =
        input.camera_orientation_chase_body.inverse() *
        (beacon_in_chase_body - input.camera_mount_chase_body);

    // Step 4a — behind camera. Camera optical axis = camera +x; if the
    // beacon's x component (forward) is non-positive, it's behind us.
    if (beacon_in_camera.x() <= static_cast<gp_scalar>(0)) {
        return sentinelObservation();
    }

    // Step 4b — emission cone. Vector from beacon to chase position must
    // fall within the beacon's emission half-angle of the emission axis.
    {
        const gp_vec3 emission_axis_world =
            input.target_orientation_world * input.beacon_emission_axis_target_body;
        const gp_vec3 from_beacon_to_chase =
            input.chase_position_world - beacon_world;
        const gp_scalar to_chase_norm = from_beacon_to_chase.norm();
        const gp_scalar emission_axis_norm = emission_axis_world.norm();
        constexpr gp_scalar kNormEps = static_cast<gp_scalar>(1e-9);
        if (to_chase_norm > kNormEps && emission_axis_norm > kNormEps) {
            const gp_scalar cos_angle =
                emission_axis_world.dot(from_beacon_to_chase) /
                (emission_axis_norm * to_chase_norm);
            const gp_scalar cos_half_cone =
                std::cos(input.beacon.emission_cone_deg *
                         (kPi / static_cast<gp_scalar>(360)));
            if (cos_angle < cos_half_cone) {
                return sentinelObservation();
            }
        }
    }

    // Step 4c — airframe obstruction (040 T014). Opaque primitives (wing
    // slab, pod nose) gate the beacon; the propeller disc attenuates rather
    // than gating (FR-009). Skipped entirely when disabled.
    const ObstructionResult obstruction = testObstruction(
        input.camera_mount_chase_body, beacon_in_chase_body,
        input.chase_airframe);
    if (obstruction.blocked) {
        return sentinelObservation();
    }
    // obstruction.attenuation is computed and available but not yet consumed:
    // it feeds the Stage-E signal budget (FR-014/015), which does not exist
    // yet. Deliberately not folded into the placeholder CEP below — that would
    // mix a real physical term into a geometric stand-in and make the Stage-E
    // replacement harder to attribute.

    // Step 4d — outside FOV. Equidistant (f-theta / spherical) projection,
    // 038 t9: NDC is proportional to ANGLE off the optical axis, not
    // tan(angle). A fixed angular separation (the beacon-pair span) then
    // reads ≈ the same anywhere in frame (radial scale exactly 1;
    // tangential ≤ θ/sinθ, +21% worst-case at 60°) — removing the
    // rectilinear tan-stretch that inflated span toward the frame edge
    // (ego-pointing contamination; see BACKLOG "spherical/equidistant
    // projection" + outcome.md US3 design rationale).
    // Camera frame: +x forward, +y right, +z down. Convention:
    //   θ        = angle between the beacon ray and the optical axis (+x)
    //   dir      = image-plane direction (y, z) / ‖(y, z)‖
    //   screen_x = θ·dir_y / (fov_h/2)   (right positive)
    //   screen_y = θ·dir_z / (fov_v/2)   (down positive, pixel-coord)
    // Per-axis normalization keeps ±1 == the per-axis FOV edge; span picks
    // up a FIXED h/v anisotropy (fov_h/fov_v) — a known constant coupled to
    // tilt, unlike the position-dependent stretch removed here.
    const gp_scalar fov_limit_h = fovHalfRad(input.camera.fov_h_deg);
    const gp_scalar fov_limit_v = fovHalfRad(input.camera.fov_v_deg);

    const gp_scalar ryz = std::sqrt(
        beacon_in_camera.y() * beacon_in_camera.y() +
        beacon_in_camera.z() * beacon_in_camera.z());
    gp_scalar screen_x = static_cast<gp_scalar>(0);
    gp_scalar screen_y = static_cast<gp_scalar>(0);
    if (ryz > static_cast<gp_scalar>(1e-12)) {
        // x > 0 guaranteed by the Step 4a early-out ⇒ θ ∈ (0, π/2).
        const gp_scalar theta = std::atan2(ryz, beacon_in_camera.x());
        screen_x = theta * (beacon_in_camera.y() / ryz) / fov_limit_h;
        screen_y = theta * (beacon_in_camera.z() / ryz) / fov_limit_v;
    }

    if (std::abs(screen_x) > static_cast<gp_scalar>(1) ||
        std::abs(screen_y) > static_cast<gp_scalar>(1)) {
        return sentinelObservation();
    }

    // Step 5 — CEP (R6 v1 linear). Edge factor grows from 0 (frame center)
    // to 1 (frame edge); the 0.3 coefficient matches the data-model.md §4
    // suggested edge baseline.
    const gp_scalar edge_factor =
        std::max(std::abs(screen_x), std::abs(screen_y));
    const gp_scalar cep_raw = std::clamp(
        edge_factor * static_cast<gp_scalar>(0.3),
        static_cast<gp_scalar>(0),
        static_cast<gp_scalar>(1));

    // Step 6 — quantize → int8 → dequantize for NN-facing fp32. Cast at
    // the boundary; quantize_xy / quantize_cep work in NN-byte space
    // which is fp32-typed by contract.
    BeaconObservation obs;
    obs.raw_x_int8 = quantize_xy(static_cast<float>(screen_x));   // raw-ok: gp_scalar→NN-byte-format boundary
    obs.raw_y_int8 = quantize_xy(static_cast<float>(screen_y));   // raw-ok: gp_scalar→NN-byte-format boundary
    obs.raw_cep_int8 = quantize_cep(static_cast<float>(cep_raw)); // raw-ok: gp_scalar→NN-byte-format boundary
    obs.screen_x = dequantize_xy(obs.raw_x_int8);
    obs.screen_y = dequantize_xy(obs.raw_y_int8);
    obs.cep = dequantize_cep(obs.raw_cep_int8);
    return obs;
}

}  // namespace autoc::eval

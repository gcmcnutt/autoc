// 030 M5 — Beacon projection module (FR-005 + FR-007 + FR-017).
//
// Analytic pinhole projection, sentinel handling, int8 quantization round-
// trip, and AirframeProxy ray–box self-occlusion. Uses the codebase's NED
// body-frame convention (+x forward, +y right, +z down) directly; see
// `include/autoc/eval/camera_projection.h` for the convention contract.

#include "autoc/eval/camera_projection.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autoc::eval {

namespace {

constexpr gp_scalar kPi = static_cast<gp_scalar>(3.14159265358979323846);

// Convert full FOV (degrees) to tan(half-angle in radians). Used as the
// projective half-extent so `screen_x = (camera.y / camera.x) / fov_limit`
// lands exactly at ±1 when the beacon sits at the FOV edge.
inline gp_scalar fovHalfTan(gp_scalar fov_full_deg) {
    return std::tan(fov_full_deg * (kPi / static_cast<gp_scalar>(360)));
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
// Ray–box (AABB slab method). Tests the segment from origin → target.
// ---------------------------------------------------------------------------

bool rayHitsProxy(const gp_vec3& ray_origin_chase_body,
                  const gp_vec3& ray_target_chase_body,
                  const AirframeProxy& proxy) {
    const gp_vec3 d = ray_target_chase_body - ray_origin_chase_body;
    gp_scalar t_enter = static_cast<gp_scalar>(0);
    gp_scalar t_exit = static_cast<gp_scalar>(1);

    for (int i = 0; i < 3; ++i) {
        const gp_scalar o = ray_origin_chase_body[i];
        const gp_scalar di = d[i];
        const gp_scalar bmin = proxy.box_min_chase_body[i];
        const gp_scalar bmax = proxy.box_max_chase_body[i];

        if (std::abs(di) < static_cast<gp_scalar>(1e-12)) {
            // Ray parallel to slab; hit only possible if origin already
            // inside the slab on this axis.
            if (o < bmin || o > bmax) return false;
            continue;
        }

        gp_scalar t1 = (bmin - o) / di;
        gp_scalar t2 = (bmax - o) / di;
        if (t1 > t2) std::swap(t1, t2);

        t_enter = std::max(t_enter, t1);
        t_exit = std::min(t_exit, t2);
        if (t_enter > t_exit) return false;
    }

    return true;
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

    // Step 4c — self-occlusion via airframe proxy. Ray from camera mount
    // to beacon-in-body intersects the AABB ⇒ occluded.
    if (rayHitsProxy(input.camera_mount_chase_body,
                     beacon_in_chase_body,
                     input.chase_airframe)) {
        return sentinelObservation();
    }

    // Step 4d — outside FOV. Compute normalized screen coords and test.
    // Camera frame: +x forward, +y right, +z down. Convention:
    //   screen_x  = (camera.y / camera.x) / tan(fov_h/2)   (right positive)
    //   screen_y  = (camera.z / camera.x) / tan(fov_v/2)   (down positive,
    //                                                       pixel-coord)
    const gp_scalar fov_limit_h = fovHalfTan(input.camera.fov_h_deg);
    const gp_scalar fov_limit_v = fovHalfTan(input.camera.fov_v_deg);

    const gp_scalar u = beacon_in_camera.y() / beacon_in_camera.x();
    const gp_scalar v = beacon_in_camera.z() / beacon_in_camera.x();

    if (std::abs(u) > fov_limit_h || std::abs(v) > fov_limit_v) {
        return sentinelObservation();
    }

    const gp_scalar screen_x = u / fov_limit_h;
    const gp_scalar screen_y = v / fov_limit_v;

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

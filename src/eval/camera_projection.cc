// 030 M5 — Beacon projection module (FR-005 + FR-007 + FR-017).
//
// Analytic EQUIDISTANT (f-theta / spherical) projection (038 t9 — was
// rectilinear/pinhole pre-t9), sentinel handling, sensor-grid quantisation
// (040 T031 — was an int8 NDC round-trip), and airframe obstruction (wing slab
// / pod nose / prop disc). Uses the NED body-frame convention (+x forward,
// +y right, +z down) directly; see `include/autoc/eval/camera_projection.h`
// for the convention contract.

#include "autoc/eval/camera_projection.h"

#include "autoc/eval/acquisition_state.h"
#include "autoc/eval/airframe_occlusion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autoc::eval {

namespace {

constexpr gp_scalar kPi = static_cast<gp_scalar>(3.14159265358979323846);

// Sentinel-emitting BeaconObservation: cep = kCepSentinelFloat,
// raw_cep_int8 = INT8_MIN, pixel indices = kPixelSentinel; bearings zeroed
// (FR-007 — bearing is arbitrary when invisible; choose zero for renderer
// hygiene).
inline BeaconObservation sentinelObservation() {
    BeaconObservation obs;
    obs.bearing_x_rad = 0.0f;
    obs.bearing_y_rad = 0.0f;
    obs.cep = kCepSentinelFloat;
    obs.raw_px_x = kPixelSentinel;
    obs.raw_px_y = kPixelSentinel;
    obs.raw_cep_int8 = INT8_MIN;
    obs.raw_margin = -999.0f;  // raw-ok: cereal byte-format member. no signal ⇒ no SNR
    obs.lock_state = static_cast<int8_t>(LockState::SEARCHING);
    return obs;
}

}  // namespace

// ---------------------------------------------------------------------------
// 040 T031 — sensor-grid quantisation (FR-001). Pixel centres at
// (i − (n−1)/2)·rad_per_px; see the header for why the boresight lands on a
// boundary rather than a centre.
// ---------------------------------------------------------------------------

int16_t bearingToPixel(gp_scalar bearing_rad, int pixels, gp_scalar rad_per_px) {
    const gp_scalar centre_offset =
        (static_cast<gp_scalar>(pixels) - static_cast<gp_scalar>(1)) /
        static_cast<gp_scalar>(2);
    const gp_scalar idx = std::round(bearing_rad / rad_per_px + centre_offset);
    const gp_scalar clamped = std::clamp(
        idx, static_cast<gp_scalar>(0), static_cast<gp_scalar>(pixels - 1));
    return static_cast<int16_t>(clamped);
}

gp_scalar pixelToBearing(int16_t px, int pixels, gp_scalar rad_per_px) {
    const gp_scalar centre_offset =
        (static_cast<gp_scalar>(pixels) - static_cast<gp_scalar>(1)) /
        static_cast<gp_scalar>(2);
    return (static_cast<gp_scalar>(px) - centre_offset) * rad_per_px;
}

// ---------------------------------------------------------------------------
// CEP quantization (R7). All `float` here is // raw-ok: per Principle VI —
// int8 ↔ fp32 NN-byte-format primitives, byte format is xiao-firmware-locked
// at fp32. The bearing int8 primitives that lived here were deleted at 040
// T032; the pixel grid is the resolution model now.
// ---------------------------------------------------------------------------

int8_t quantize_cep(float cep_in_zero_one_or_sentinel) {         // raw-ok: NN-byte-format primitive
    if (cep_in_zero_one_or_sentinel >= kCepSentinelThreshold) return INT8_MIN;
    const float clamped = std::clamp(cep_in_zero_one_or_sentinel, 0.0f, 1.0f); // raw-ok: NN-byte-format primitive
    return static_cast<int8_t>(std::lround(clamped * 127.0f));
}

float dequantize_cep(int8_t q) {                              // raw-ok: NN-byte-format primitive
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

    // Step 4b — emission (FR-019). The hard 270° cone is GONE: it modelled a
    // single outboard emitter with a cliff edge, when the enclosure is a cube
    // minus its base emitting on five faces with a flat-top-and-shoulders
    // pattern on each. The replacement is a continuous gain, and the only thing
    // that gates is genuine darkness — no face still illuminating this
    // direction, which for the shipped profile means past ~105° from every one
    // of the five axes. That is physics, not an arbitrary cone.
    //
    // The aspect must be taken in the TARGET body frame, since that is where the
    // enclosure's faces are fixed.
    const gp_vec3 to_chase_target_body =
        input.target_orientation_world.inverse() *
        (input.chase_position_world - beacon_world);
    const gp_scalar range_m = to_chase_target_body.norm();
    const gp_scalar emission_gain = emissionGain(
        to_chase_target_body, input.beacon_emission_axis_target_body, input.signal);
    if (emission_gain <= static_cast<gp_scalar>(0)) {
        return sentinelObservation();
    }

    // Step 4b2 — the DETECTION envelope (FR-033a). ASSERTED, not emergent: the
    // sensor is taken as good to the configured range, and the link budget
    // shapes quality WITHIN it rather than cutting visibility short. The budget
    // is not calibrated well enough to be trusted as a *limit* — the degradation
    // modes that would set a real one (sun angle, glint, dust, ambient level,
    // sensor variation) are deliberately out of scope — but it is good enough to
    // shape a *gradient*.
    if (range_m > input.signal.detection_range_m) {
        return sentinelObservation();
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

    // Step 4d — bearing, in the equidistant (f-theta) angular plane. 038 t9
    // established the angular mapping (NDC ∝ ANGLE off the optical axis, not
    // tan(angle)), which removed the rectilinear tan-stretch that inflated
    // span toward the frame edge — ego-pointing contamination.
    //
    // 040 T031 keeps that mapping and drops the per-axis normalisation.
    // Camera frame: +x forward, +y right, +z down. Convention:
    //   θ    = angle between the beacon ray and the optical axis (+x)
    //   dir  = image-plane direction (y, z) / ‖(y, z)‖
    //   θx   = θ·dir_y   (right positive)
    //   θy   = θ·dir_z   (down positive, pixel-coord)
    //
    // Both axes now carry ONE angular scale — radians — so a given angular
    // separation reads identically at any orientation (FR-002). The retired
    // per-axis normalisation divided each axis by its own half-FOV, making a
    // horizontal separation read fov_h/fov_v = 1.333× its vertical twin.
    const gp_scalar ryz = std::sqrt(
        beacon_in_camera.y() * beacon_in_camera.y() +
        beacon_in_camera.z() * beacon_in_camera.z());
    gp_scalar bearing_x = static_cast<gp_scalar>(0);
    gp_scalar bearing_y = static_cast<gp_scalar>(0);
    if (ryz > static_cast<gp_scalar>(1e-12)) {
        // x > 0 guaranteed by the Step 4a early-out ⇒ θ ∈ (0, π/2).
        const gp_scalar theta = std::atan2(ryz, beacon_in_camera.x());
        bearing_x = theta * (beacon_in_camera.y() / ryz);
        bearing_y = theta * (beacon_in_camera.z() / ryz);
    }

    // Step 4e — outside the DERIVED field (FR-003). The bound comes from the
    // grid, so field and resolution cannot disagree.
    const gp_scalar half_h = input.camera.halfFovHRad();
    const gp_scalar half_v = input.camera.halfFovVRad();
    if (std::abs(bearing_x) > half_h || std::abs(bearing_y) > half_v) {
        return sentinelObservation();
    }

    // Step 5 — quantise onto the sensor grid (FR-001). Reported precision now
    // matches sensor capability: never finer than a pixel, never coarser.
    const gp_scalar rad_per_px = input.camera.radPerPx();
    const int16_t px_x = bearingToPixel(bearing_x, input.camera.pixels_h, rad_per_px);
    const int16_t px_y = bearingToPixel(bearing_y, input.camera.pixels_v, rad_per_px);
    const gp_scalar q_bearing_x = pixelToBearing(px_x, input.camera.pixels_h, rad_per_px);
    const gp_scalar q_bearing_y = pixelToBearing(px_y, input.camera.pixels_v, rad_per_px);

    // Step 6 — QUALITY (FR-014). The position-only placeholder is gone. It was
    // `0.3 × max(|x|, |y|)` — purely where the beacon sat in frame, so a beacon
    // at 5 m and one at 500 m on the same pixel were indistinguishable. Quality
    // is now derived from the link budget: range, emission aspect and
    // obstruction attenuation all reach it.
    //
    // THE VALUE WRITTEN HERE IS SIGNAL-ONLY — what a permanently-locked receiver
    // would report. It is the correct standalone answer for a caller holding no
    // carried state (which is what the geometry tests are), but the production
    // path refines it: `projectPerceptionTick` applies the shared-detector
    // penalty, which needs BOTH beacons, and then the acquisition state machine,
    // which needs history. Neither is knowable from one beacon on one tick.
    const SignalResult signal = computeSignal(range_m, emission_gain,
                                              obstruction.attenuation,
                                              /*shares_detector_element=*/false,
                                              input.signal);
    const gp_scalar cep_raw = std::clamp(
        static_cast<gp_scalar>(1) - signal.q / static_cast<gp_scalar>(9),
        static_cast<gp_scalar>(0),
        static_cast<gp_scalar>(1));

    BeaconObservation obs;
    obs.raw_px_x = px_x;
    obs.raw_px_y = px_y;
    obs.bearing_x_rad = static_cast<float>(q_bearing_x);          // raw-ok: gp_scalar→NN-byte-format boundary
    obs.bearing_y_rad = static_cast<float>(q_bearing_y);          // raw-ok: gp_scalar→NN-byte-format boundary
    obs.raw_cep_int8 = quantize_cep(static_cast<float>(cep_raw)); // raw-ok: gp_scalar→NN-byte-format boundary
    obs.cep = dequantize_cep(obs.raw_cep_int8);
    obs.raw_margin = static_cast<float>(signal.snr_db);           // raw-ok: gp_scalar→cereal byte-format boundary
    obs.lock_state = static_cast<int8_t>(LockState::TRACKING);
    return obs;
}

// ---------------------------------------------------------------------------
// 040 T044 / T045 — effective field and obstruction onset.
//
// Both are pure geometry over the same primitives the per-tick path uses, so
// they cannot drift from what the simulator actually does: they call
// testObstruction, not a parallel reimplementation of it.
// ---------------------------------------------------------------------------

namespace {

// Unit ray in camera frame for a bearing, matching the projection's forward
// mapping (see derived_features.h bearing_to_unit_ray — same construction; kept
// local rather than shared because this file must not depend on the NN-facing
// derived-feature header).
gp_vec3 rayForBearing(gp_scalar bx, gp_scalar by) {
    const gp_scalar theta = std::sqrt(bx * bx + by * by);
    if (theta < static_cast<gp_scalar>(1e-9)) {
        return gp_vec3(static_cast<gp_scalar>(1), static_cast<gp_scalar>(0),
                       static_cast<gp_scalar>(0));
    }
    const gp_scalar s = std::sin(theta) / theta;
    return gp_vec3(std::cos(theta), s * bx, s * by);
}

}  // namespace

EffectiveField computeEffectiveField(const CameraConfig& camera,
                                     const AirframeObstruction& airframe,
                                     const gp_vec3& camera_mount_body,
                                     int stride_px,
                                     gp_scalar probe_range_m) {
    EffectiveField out;
    out.nominal_sr = static_cast<gp_scalar>(0);
    out.clear_sr = static_cast<gp_scalar>(0);
    out.blocked_sr = static_cast<gp_scalar>(0);
    out.attenuated_sr = static_cast<gp_scalar>(0);

    const int stride = std::max(1, stride_px);
    const gp_scalar rad_per_px = camera.radPerPx();
    // Solid angle a single sampled cell would subtend if the mapping were
    // area-true; the sin θ/θ Jacobian below corrects it to the real sphere.
    const gp_scalar cell =
        rad_per_px * rad_per_px * static_cast<gp_scalar>(stride) *
        static_cast<gp_scalar>(stride);

    for (int iy = 0; iy < camera.pixels_v; iy += stride) {
        const gp_scalar by = pixelToBearing(static_cast<int16_t>(iy),
                                            camera.pixels_v, rad_per_px);
        for (int ix = 0; ix < camera.pixels_h; ix += stride) {
            const gp_scalar bx = pixelToBearing(static_cast<int16_t>(ix),
                                                camera.pixels_h, rad_per_px);

            // Jacobian of the equidistant mapping: dΩ = (sin θ / θ) dθx dθy.
            const gp_scalar theta = std::sqrt(bx * bx + by * by);
            const gp_scalar jac =
                (theta < static_cast<gp_scalar>(1e-9))
                    ? static_cast<gp_scalar>(1)
                    : std::sin(theta) / theta;
            const gp_scalar d_omega = cell * jac;
            out.nominal_sr += d_omega;

            const gp_vec3 target =
                camera_mount_body + rayForBearing(bx, by) * probe_range_m;
            const ObstructionResult r =
                testObstruction(camera_mount_body, target, airframe);

            if (r.blocked) {
                out.blocked_sr += d_omega;
            } else if (r.attenuation < static_cast<gp_scalar>(1)) {
                out.attenuated_sr += d_omega;
                // Attenuated field is still usable field, so it also counts as
                // clear for the purpose of "what did the airframe take".
                out.clear_sr += d_omega;
            } else {
                out.clear_sr += d_omega;
            }
        }
    }

    return out;
}

gp_scalar obstructionOnsetDeg(const CameraConfig& camera,
                              const AirframeObstruction& airframe,
                              const gp_vec3& camera_mount_body,
                              gp_scalar boresight_tilt_inboard_deg) {
    // Sweep INBOARD — toward the thrust axis — because that is where every
    // primitive sits. "Inboard" is the direction from the mount's (y, z) to the
    // prop axis, which for the baseline mount is mostly −y with a little +z.
    const gp_scalar dy = airframe.prop_axis_y - camera_mount_body.y();
    const gp_scalar dz = airframe.prop_axis_z - camera_mount_body.z();
    const gp_scalar r = std::sqrt(dy * dy + dz * dz);
    if (r < static_cast<gp_scalar>(1e-9)) {
        // Mount is ON the thrust axis: there is no inboard direction, and the
        // disc is dead ahead. Onset is zero by inspection.
        return static_cast<gp_scalar>(0);
    }
    const gp_scalar uy = dy / r;
    const gp_scalar uz = dz / r;

    constexpr gp_scalar kDeg = static_cast<gp_scalar>(3.14159265358979323846 / 180.0);

    // A misaligned mount does not move the airframe — it rotates the boresight.
    // So the sweep starts at the tilt angle in BODY terms and the reported onset
    // is measured from the camera's OWN boresight, which is what a controller
    // trained on this camera would experience.
    const gp_scalar half_field_deg =
        std::max(camera.fovHDeg(), camera.fovVDeg()) /
        static_cast<gp_scalar>(2);

    // 0.05° steps: an order finer than a 0.375° pixel, so the reported onset is
    // limited by the geometry rather than by the sweep.
    const gp_scalar step = static_cast<gp_scalar>(0.05);
    for (gp_scalar a = static_cast<gp_scalar>(0); a <= half_field_deg;
         a += step) {
        const gp_scalar body_deg = a + boresight_tilt_inboard_deg;
        const gp_scalar t = body_deg * kDeg;
        const gp_vec3 dir(std::cos(t), std::sin(t) * uy, std::sin(t) * uz);
        const gp_vec3 target =
            camera_mount_body + dir * static_cast<gp_scalar>(10);
        const ObstructionResult res =
            testObstruction(camera_mount_body, target, airframe);
        if (res.blocked || res.attenuation < static_cast<gp_scalar>(1)) {
            return a;
        }
    }

    return static_cast<gp_scalar>(-1);  // nothing obstructs anywhere in field
}

}  // namespace autoc::eval

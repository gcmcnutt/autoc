#pragma once

// 030 M5 — Camera configuration (FR-003 + plan-phase R6/R7).
//
// Mount + projection parameters for the on-board chase-craft camera that
// produces the perception input for tracker-mode NN training. v1 baseline:
// single forward-pointing camera, planar pinhole, 120° H × 90° V FOV, 30 Hz,
// top-of-wing-chord mount per spec D10.
//
// Coordinate convention (chase body frame, NED): +x forward, +y right,
// +z down. The camera's optical axis is aligned with body +x by default
// (mount_orientation_body = identity quat). Tests + the projection module
// (`camera_projection.cc`) use this convention directly — body-frame x is
// forward, body-frame y is "screen right", body-frame z is "screen down"
// (pixel-coord convention: +screen_y points downward in the image).
//
// Per spec D13 + data-model.md §3: PRNG-varied dimension architectural in
// v1 (sigmas at zero); the placeholders below mark where per-scenario
// jitter slots in for v2+.

#include "autoc/types.h"

namespace autoc::eval {

struct CameraConfig {
    // ----- Compile-time-fixed (target-hardware spec) ------------------------
    enum class Projection { PLANAR_PINHOLE, SPHERICAL_FISHEYE };
    Projection projection = Projection::PLANAR_PINHOLE;

    // FOV is the FULL angular extent (not half-angle). Tests and projection
    // math convert to half-angle (fov_*_deg / 2) where geometry needs it.
    gp_scalar fov_h_deg = 120.0f;
    gp_scalar fov_v_deg = 90.0f;

    gp_scalar frame_rate_hz = 30.0f;
    gp_scalar latency_ms = 0.0f;    // v1 baseline: zero pipeline latency

    // ----- PRNG-varied per scenario (v1 sigmas at zero) ---------------------
    // Mount offset in chase body frame. v1 default: top-of-wing-chord, 5 cm
    // above body origin (NED: +z down → -z = up).
    gp_vec3 mount_offset_body{0.0f, 0.0f, -0.05f};

    // Mount orientation: rotation that takes camera-frame coordinates into
    // chase-body coordinates. Identity ⇒ camera frame == body frame, i.e.
    // optical axis is body +x (forward).
    gp_quat mount_orientation_body{1.0f, 0.0f, 0.0f, 0.0f};

    // 030 M6e — cereal serialize for EvalData wire-protocol carry. Enum
    // serialized as int (cereal doesn't handle enum class directly).
    template <class Archive>
    void serialize(Archive& ar) {
        int proj = static_cast<int>(projection);
        ar(proj, fov_h_deg, fov_v_deg, frame_rate_hz, latency_ms,
           mount_offset_body, mount_orientation_body);
        projection = static_cast<Projection>(proj);
    }
};

}  // namespace autoc::eval

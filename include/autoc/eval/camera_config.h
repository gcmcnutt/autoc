#pragma once

// 030 M5 — Camera configuration (FR-003 + plan-phase R6/R7).
//
// Mount + projection parameters for the on-board chase-craft camera that
// produces the perception input for tracker-mode NN training. v1 baseline:
// single forward-pointing camera, 120° H × 90° V FOV, top-of-wing-chord
// mount per spec D10.
//
// 040 T017 — three fields retired here, each because the thing it configured
// no longer exists:
//   `frame_rate_hz`  the camera does not have an independent rate. Sensor
//                    cadence follows the control loop, fixed at 20 Hz by 037
//                    (WorkerInit.controlIntervalMsec); the 30 Hz here predated
//                    that decision and was never read.
//   `latency_ms`     latency is no longer a configured constant. It emerges
//                    from the acquisition state machine's chip-credit timing
//                    (US4), so a scalar knob would contradict the model.
//   `Projection`     a two-value enum with one value ever constructed. The
//                    projection stage is rewritten in US2 (T031) to quantise
//                    on a real pixel grid; there is no second mode to select.
// Deleted outright rather than deprecated, per Constitution III. WorkerInit is
// RPC-only and never persisted, so the serialize change is same-rebuild safe.
//
// 040 T029 — `fov_h_deg` / `fov_v_deg` follow them, for a different reason:
// they were not dead, they were REDUNDANT. Field of view and resolution are
// not independent facts about a camera, and configuring them separately let
// them disagree. They are now derived from the grid (FR-003).
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
    // ----- The sensor grid — the ONLY resolution inputs (040 T029, FR-003) --
    //
    // Field of view is DERIVED from these, never configured beside them. That
    // is the whole point of FR-003: with independent `fov_h_deg` / `fov_v_deg`
    // fields (retired here) a config could declare a 120° field over a grid
    // whose pitch implied 90°, and nothing would notice. One knob, one answer.
    //
    // Defaults are the assumed sensor format (config-surface.md classifies all
    // three ASSUMED, pending the lens/sensor decision recorded in US7):
    // 320 × 240 at 0.375°/px ⇒ exactly 120° × 90°.
    int pixels_h = 320;
    int pixels_v = 240;
    gp_scalar deg_per_px = static_cast<gp_scalar>(0.375);

    // ----- PRNG-varied per scenario (v1 sigmas at zero) ---------------------
    // Mount offset in chase body frame. v1 default: top-of-wing-chord, 5 cm
    // above body origin (NED: +z down → -z = up).
    gp_vec3 mount_offset_body{0.0f, 0.0f, -0.05f};

    // Mount orientation: rotation that takes camera-frame coordinates into
    // chase-body coordinates. Identity ⇒ camera frame == body frame, i.e.
    // optical axis is body +x (forward).
    gp_quat mount_orientation_body{1.0f, 0.0f, 0.0f, 0.0f};

    // ----- Derived field of view (FR-003) -----------------------------------
    //
    // FULL angular extent, edge to edge. Pixel CENTRES sit at
    // (i − (n−1)/2)·deg_per_px, so the outer edges land at ±n/2·deg_per_px and
    // the field is exactly n × deg_per_px. Accessors, not fields, so there is
    // no setter that could contradict the grid.
    gp_scalar fovHDeg() const {
        return static_cast<gp_scalar>(pixels_h) * deg_per_px;
    }
    gp_scalar fovVDeg() const {
        return static_cast<gp_scalar>(pixels_v) * deg_per_px;
    }

    gp_scalar radPerPx() const { return deg_per_px * kDegToRad; }
    gp_scalar halfFovHRad() const {
        return static_cast<gp_scalar>(pixels_h) * radPerPx() /
               static_cast<gp_scalar>(2);
    }
    gp_scalar halfFovVRad() const {
        return static_cast<gp_scalar>(pixels_v) * radPerPx() /
               static_cast<gp_scalar>(2);
    }

    // 030 M6e — cereal serialize for the WorkerInit RPC carry.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(pixels_h, pixels_v, deg_per_px,
           mount_offset_body, mount_orientation_body);
    }

private:
    static constexpr gp_scalar kDegToRad =
        static_cast<gp_scalar>(3.14159265358979323846 / 180.0);
};

}  // namespace autoc::eval

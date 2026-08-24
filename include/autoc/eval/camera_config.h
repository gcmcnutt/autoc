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
    // Defaults are the MEASURED sensor format (they were ASSUMED through 040;
    // 031's ruled-mat calibration of the flight lens closed that — see below).
    // 320 × 200 at 0.304°/px ⇒ 97.3° H × 60.8° V.
    // 041 T041f (REVERTED 2026-08-16) — 320, the grid the SENSOR DELIVERS.
    //
    // T041f briefly set 312 to land the derived field on 031's tape-measured
    // 95° H rather than the f·θ extrapolation's 97.3°. Operator reversed it,
    // and the reasoning is better: the camera dumps 320×200 (a 4× bin of the
    // real 1280×800). Sizing the grid to make a DERIVED angle match a tape
    // reading means modelling a sensor that does not exist, and it silently
    // reattributes ~2.4° of tape/extrapolation disagreement to the pixel count
    // — the one quantity here we actually know exactly.
    //
    // The residual 95 vs 97.3 is measurement error in the tape (the two agree
    // within the 3° the calibration itself quotes). Absorb it there, not in the
    // grid. 320 × 0.304 = 97.3° H, 200 × 0.304 = 60.8° V.
    int pixels_h = 320;
    // 041 T041a — 240 → 200: matching the OV9281's 1280×800 (1.6) aspect rather
    // than the 4:3 invention 240 px was. Must move together with
    // AutocConfig::cameraPixelsV and the three tracker inis — three definitions
    // of one value, the E1 hazard in index-coupling-inventory.md. The ini is
    // authoritative at runtime; these defaults exist so a missing key is not
    // silently a DIFFERENT camera.
    int pixels_v = 200;
    // 041 T041d (FR-029) — 0.375 → 0.304, from MEASUREMENT, which is the exact
    // trigger the T041a note reserved ("revisit from measurement, not from a
    // second estimate"). 031 calibrated the flight optic on 2026-08-16 — the
    // $3 1.8 mm M12 fisheye on the OV9281, ruled mat at h = 15″, frame archived
    // as specs/031-beacon-camera/fisheye-1p8mm-grid-15in.jpg:
    //   * PROJECTION = equidistant (f·θ), confirmed to the field edge. The sim
    //     has been analytic equidistant since 038 t9, so nothing moves here —
    //     the measurement VALIDATES a model already in use.
    //   * NATIVE PITCH = 0.076°/px, uniform (the equidistant silver lining).
    //   * FOV = 95° H × 61° V by direct tape measurement at the frame edge.
    // The sim grid is a 4× bin of the real 1280×800 sensor, so the sim pitch is
    // 4 × 0.076 = 0.304°/px, and 320 × 200 at that pitch is 97.3° × 60.8° —
    // the measured lens inside its own spread (tape 95×61, f·θ extrapolation
    // 98×61). Grid and pitch now BOTH trace to hardware.
    // ⚠️ The prior 120° × 75° was the conservative split of a pre-arrival
    // ESTIMATE (~124° × 78°), and the estimate was wrong in the direction
    // nobody guarded against — the real field is NARROWER, ~19% on both axes.
    // The single-fisheye-at-120° assumption is RETIRED for this lens; 120° H
    // needs the birded pair or a wider lens (specs/031-beacon-camera/
    // camera-era-knobs.md §6).
    // Consequence to expect: per-pixel CEP and quantisation get ~19% FINER,
    // which is not a side effect to be sorry about — the real lens resolves
    // better than we assumed. The field narrowing is the fitness-affecting
    // half, and it lands in the A1 bundle with the rest.
    gp_scalar deg_per_px = static_cast<gp_scalar>(0.304);

    // ----- PRNG-varied per scenario (v1 sigmas at zero) ---------------------
    // Mount offset in chase body frame, metres, on the prop-axle datum
    // (station 0 = axle; +x forward, +y right wing, +z down).
    //
    // 040 T043 — THE BASELINE MOUNT: wing leading edge, 8″ outboard, ~1¼″ above
    // the thrust line. The retired default was (0, 0, −0.05) — on the
    // centreline, 5 cm above the axle, which is INSIDE the 6.985 cm prop radius
    // and therefore squarely behind the disc. Obstruction had to ship disabled
    // to keep that mount usable; this position is what lets it be switched on,
    // and it is why US3 treats obstruction as a design choice to validate
    // rather than a defect to model.
    //
    // Kept numerically identical to hb1LeadingEdgeCameraMount() — see the note
    // there on why x stands 2 mm proud of the leading-edge face rather than on
    // it (a surface touch counts as a hit).
    gp_vec3 mount_offset_body{-0.150400f, 0.203200f, -0.031750f};

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

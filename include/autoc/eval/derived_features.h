#pragma once

// 032 PHASE 1 — Derived perceptual features (beacon_pair_span, target_tilt).
//
// Pure, header-only math primitives consumed by:
//   - src/eval/tracker_stepper.cc       (autoc tracker_stepper projectAndShiftHistory)
//   - src/nn/evaluator.cc               (gather_tracker_inputs)
//   - crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp (mirror)
//
// CEP-gating is the CALLER's responsibility — these helpers are pure math
// over bearing pairs. The caller checks each beacon's CEP at the current tick
// and substitutes neutral values (span = 0, tilt = (0, 1)) when EITHER
// beacon's CEP exceeds the configured threshold.
//
// 040 T033 — UNITS CHANGED, MATH DID NOT. The inputs are now BEARINGS IN
// RADIANS quantised on the sensor grid, not ±1 per-axis NDC. Nothing here
// needed rewriting, because these were always pure planar geometry over
// whatever the caller passed — but what they MEAN changed substantially:
//
//   * span is now a true ANGLE. Range follows directly as
//     separation_m / span_rad, with no per-axis scale factor to unwind.
//   * span is now ORIENTATION-INVARIANT. Under the retired encoding each axis
//     was normalised by its own half-FOV, so the same physical pair read
//     60/45 = 1.333x wider measured horizontally than vertically — a 33%
//     error that moved with target roll, in the sole range channel.
//   * tilt is unchanged in form and stays (sin, cos) per FR-005, but its angle
//     is now measured in a plane with one consistent scale, so it no longer
//     carries the same anisotropic skew.
//
// See specs/032-tracker-nn-enhancements/contracts/gather_tracker_inputs_v54.md
// and spec.md Clarifications Q4 (CEP-gating) + research.md R4 (tilt math).

#include <algorithm>
#include <cmath>

#include "autoc/types.h"

namespace autoc {
namespace eval {

// Degenerate-pair epsilon, in RADIANS as of 040 T033. At the 0.375°/px
// baseline one pixel is 6.54e-3 rad, so 1e-4 sits at ~1.5% of a pixel —
// comfortably below anything the grid can resolve, which is what this guard
// wants. Grid quantisation makes the degenerate case sharper than it used to
// be: two beacons landing in the SAME pixel now produce bit-identical bearings
// and a distance of exactly zero. When the pair's angular distance is below
// this, tilt is geometrically undefined; substitute the neutral (sin = 0,
// cos = 1). Belt-and-suspenders to CEP-gating: a coincident pair with passing
// CEP still gets a defined output.
constexpr gp_scalar kTiltDegenerateEpsilon = static_cast<gp_scalar>(1e-4);

// Raw Euclidean distance between two beacon bearings, in radians.
// No scaling, no normalization, no clipping — exactly `sqrt(dx² + dy²)`.
// Pure: no CEP-gating, no validation. Caller substitutes neutral 0.0 when
// CEP-gated.
inline gp_scalar compute_pair_span(gp_scalar lx, gp_scalar ly,
                                   gp_scalar rx, gp_scalar ry) {
    const gp_scalar dx = rx - lx;
    const gp_scalar dy = ry - ly;
    return std::sqrt(dx * dx + dy * dy);
}

// (sin θ, cos θ) pair for θ = atan2(ry − ly, rx − lx). Convention: θ = 0
// when the port→starboard line projects horizontally with port on
// image-plane-left (chase + target wings level relative to each other).
//
// Returns the neutral {0, 1} pair if the beacon pair is geometrically
// degenerate (angular distance < kTiltDegenerateEpsilon). Per spec Q4 + R4.
struct TiltSinCos {
    gp_scalar sin;
    gp_scalar cos;
};

inline TiltSinCos compute_tilt(gp_scalar lx, gp_scalar ly,
                               gp_scalar rx, gp_scalar ry) {
    const gp_scalar dx = rx - lx;
    const gp_scalar dy = ry - ly;
    const gp_scalar pair_dist = std::sqrt(dx * dx + dy * dy);
    if (pair_dist < kTiltDegenerateEpsilon) {
        return TiltSinCos{static_cast<gp_scalar>(0.0),
                          static_cast<gp_scalar>(1.0)};
    }
    const gp_scalar theta = std::atan2(dy, dx);
    return TiltSinCos{std::sin(theta), std::cos(theta)};
}

}  // namespace eval
}  // namespace autoc

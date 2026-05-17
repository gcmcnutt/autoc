#pragma once

// 032 PHASE 1 — Derived perceptual features (beacon_pair_span, target_tilt).
//
// Pure, header-only math primitives consumed by:
//   - src/eval/tracker_stepper.cc       (autoc minisim projectAndShiftHistory)
//   - src/nn/evaluator.cc               (gather_tracker_inputs)
//   - crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp (mirror)
//
// CEP-gating is the CALLER's responsibility — these helpers are pure math
// over NDC pairs. The caller checks each beacon's CEP at the current tick
// and substitutes neutral values (span = 0, tilt = (0, 1)) when EITHER
// beacon's CEP exceeds the configured threshold.
//
// See specs/032-tracker-nn-enhancements/contracts/gather_tracker_inputs_v54.md
// and spec.md Clarifications Q4 (CEP-gating) + research.md R4 (tilt math).

#include <cmath>

#include "autoc/types.h"

namespace autoc {
namespace eval {

// Degenerate-pair epsilon in raw NDC (x, y) units (1e-4 is well below the
// smallest tilt the NN could meaningfully consume — at typical camera
// resolutions it's <1 pixel). When the beacon pair's NDC distance is
// below this, tilt is geometrically undefined; substitute the neutral
// (sin = 0, cos = 1). Belt-and-suspenders to CEP-gating: a near-coincident
// pair with passing CEP (rare sim-edge case) still gets a defined output.
constexpr gp_scalar kTiltDegenerateEpsilon = static_cast<gp_scalar>(1e-4);

// Raw Euclidean distance between two beacon centroids in NDC (x, y) units.
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
// degenerate (NDC distance < kTiltDegenerateEpsilon). Per spec Q4 + R4.
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

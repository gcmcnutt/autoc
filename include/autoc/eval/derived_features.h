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

#include <algorithm>
#include <cmath>

#include "autoc/types.h"

namespace autoc {
namespace eval {

// 033 PHASE 1 — multiplicative per-tick smoothness penalty.
//
// Per spec.md Clarifications Q4 (Pythagorean default) and operator routing
// 2026-05-20 (worker-side per-step placement). The per-tick smoothness
// factor is multiplied onto stepPoints BEFORE the streak multiplier inside
// FitnessComputer::applyStreak() — see contracts/smoothness_factor.md.
enum class SmoothnessMotionMode {
    Pythagorean,  // sqrt(dpt² + drl² + dth²); motion_max = sqrt(12) ≈ 3.464
    Sum,          // |dpt| + |drl| + |dth|;     motion_max = 6.0
    Max,          // max(|dpt|, |drl|, |dth|);  motion_max = 2.0
};

// Per-tick smoothness factor in [floor, 1.0]:
//   1.0    ⇔ motion == 0 (no Δ since last tick — controller at rest)
//   floor  ⇔ motion >= motion_max (maximum bang-bang)
//   linear interpolation between for partial motion
//
// Inputs (dpt/drl/dth) are per-tick deltas of NN-tanh-saturated outputs in
// [-1, +1], so each |Δaxis| ∈ [0, 2]. floor MUST be in [0, 1] (caller
// range-checks at ini-load time; this function does not re-validate).
// On the first tick of a scenario the caller passes 0/0/0 → factor = 1.0
// (no false-positive penalty on scenario start).
//
// See contracts/smoothness_factor.md for full edge-case enumeration and
// stepper integration pattern (PathgenStepper / TrackerStepper).
inline gp_scalar compute_smoothness_factor(
        gp_scalar dpt, gp_scalar drl, gp_scalar dth,
        gp_scalar floor,
        SmoothnessMotionMode mode = SmoothnessMotionMode::Pythagorean) {
    gp_scalar motion;
    gp_scalar motion_max;
    switch (mode) {
        case SmoothnessMotionMode::Pythagorean:
            motion = std::sqrt(dpt * dpt + drl * drl + dth * dth);
            motion_max = std::sqrt(static_cast<gp_scalar>(12.0));
            break;
        case SmoothnessMotionMode::Sum:
            motion = std::abs(dpt) + std::abs(drl) + std::abs(dth);
            motion_max = static_cast<gp_scalar>(6.0);
            break;
        case SmoothnessMotionMode::Max:
            motion = std::max({std::abs(dpt), std::abs(drl), std::abs(dth)});
            motion_max = static_cast<gp_scalar>(2.0);
            break;
    }
    const gp_scalar ratio = std::min(motion / motion_max,
                                     static_cast<gp_scalar>(1.0));
    return static_cast<gp_scalar>(1.0) -
           (static_cast<gp_scalar>(1.0) - floor) * ratio;
}

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

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
// RADIANS quantised on the sensor grid, not ±1 per-axis NDC. What they MEAN
// changed substantially:
//
//   * span is now a true ANGLE, so range follows from the pair geometry
//     directly, with no per-axis scale factor to unwind.
//   * span is now ORIENTATION-INVARIANT. Under the retired encoding each axis
//     was normalised by its own half-FOV, so the same physical pair read
//     60/45 = 1.333x wider measured horizontally than vertically — a 33%
//     error that moved with target roll, in the sole range channel.
//
// 040 T033a — SPAN IS MEASURED ON THE SPHERE, NOT IN THE BEARING PLANE.
// Bearing is an equidistant (f-theta) mapping: |(θx, θy)| is the true angle
// off the optical axis, and direction in that plane is true azimuth about the
// axis. Planar Euclidean distance in such a mapping equals the great-circle
// angle EXACTLY when the pair lies along a radius, and over-reads TANGENTIALLY
// by θ/sin θ — +8.6% at 40° off-axis, +35% at the 75° frame diagonal. That is
// the flat-metric-on-a-curved-surface error every map projection has; it is not
// a property of the encoding, and it sat in the ONE channel carrying range.
//
// So the pair is now measured by reconstructing both unit rays and taking the
// angle between them. This is exactly position- AND orientation-invariant by
// construction (it is a function of the ray pair alone, with no reference to
// where in frame they landed), which is what SC-001 asks for literally.
// Reconstruction is the exact inverse of the projection's forward mapping in
// src/eval/camera_projection.cc, so no information is added or lost — only the
// metric changed.
//
// TILT DELIBERATELY STAYS PLANAR. On a sphere "the angle of the pair" needs a
// reference direction, and there is no global horizontal; defining one would
// mean a parallel-transported tangent basis at the pair midpoint. Tilt's job is
// conveying target roll/aspect, so its residual is a DIRECTION error, which is
// far more benign than a MAGNITUDE error in the range channel. Recorded as a
// known approximation rather than plumbed.
//
// See specs/032-tracker-nn-enhancements/contracts/gather_tracker_inputs_v54.md
// and spec.md Clarifications Q4 (CEP-gating) + research.md R2 (span metric) and
// R4 (tilt math).

#include <algorithm>
#include <cmath>

#include "autoc/types.h"

namespace autoc {
namespace eval {

// Degenerate-pair epsilon, in RADIANS as of 040 T033. At the 0.304°/px
// baseline one pixel is 5.31e-3 rad, so 1e-4 sits at ~1.9% of a pixel —
// comfortably below anything the grid can resolve, which is what this guard
// wants. Grid quantisation makes the degenerate case sharper than it used to
// be: two beacons landing in the SAME pixel now produce bit-identical bearings
// and a distance of exactly zero. When the pair's angular distance is below
// this, tilt is geometrically undefined; substitute the neutral (sin = 0,
// cos = 1). Belt-and-suspenders to CEP-gating: a coincident pair with passing
// CEP still gets a defined output.
constexpr gp_scalar kTiltDegenerateEpsilon = static_cast<gp_scalar>(1e-4);

// Below this bearing magnitude, sin(θ)/θ is taken as 1. The neglected term is
// θ²/6 < 2e-13 — far under float epsilon, and the guard exists only to keep
// 0/0 out of the boresight case.
constexpr gp_scalar kSincSmallAngle = static_cast<gp_scalar>(1e-6);

// Unit ray for a bearing, in the camera frame (+x forward, +y right, +z down).
// Exact inverse of the projection's forward mapping: θ = |(θx, θy)| is the
// angle off the optical axis and (θx, θy)/θ is the image-plane direction, so
//
//     u = (cos θ, sin θ · dir_y, sin θ · dir_z)
//
// and since dir = (θx, θy)/θ, the two transverse components collapse to
// sinc(θ)·(θx, θy) — one sqrt and one sincos, no atan2, no division by a
// possibly-zero radius.
struct UnitRay {
    gp_scalar x;
    gp_scalar y;
    gp_scalar z;
};

inline UnitRay bearing_to_unit_ray(gp_scalar bx, gp_scalar by) {
    const gp_scalar theta = std::sqrt(bx * bx + by * by);
    const gp_scalar sinc = (theta < kSincSmallAngle)
                               ? static_cast<gp_scalar>(1)
                               : std::sin(theta) / theta;
    return UnitRay{std::cos(theta), sinc * bx, sinc * by};
}

// True angular separation between two beacon bearings, in radians — the
// great-circle angle between their rays, exact at any position in frame and any
// pair orientation (040 T033a; see the header note on why this is not the
// planar distance it replaced).
//
// Computed as 2·asin(chord/2) rather than acos(u_L · u_R) for a float-precision
// reason that bites in exactly our operating regime: at 25 m the pair subtends
// ~0.031 rad, so the dot product sits at 0.99952 where acos loses roughly half
// its significant digits. The chord is formed from components that differ by an
// O(span) amount with no cancellation, and asin is well-conditioned near zero.
//
// Pure: no CEP-gating, no validation. Caller substitutes neutral 0.0 when
// CEP-gated. An identical pair still returns exactly zero.
inline gp_scalar compute_pair_span(gp_scalar lx, gp_scalar ly,
                                   gp_scalar rx, gp_scalar ry) {
    const UnitRay ul = bearing_to_unit_ray(lx, ly);
    const UnitRay ur = bearing_to_unit_ray(rx, ry);
    const gp_scalar dx = ur.x - ul.x;
    const gp_scalar dy = ur.y - ul.y;
    const gp_scalar dz = ur.z - ul.z;
    const gp_scalar half_chord =
        std::sqrt(dx * dx + dy * dy + dz * dz) * static_cast<gp_scalar>(0.5);
    // Clamp guards float fuzz at the antipodal limit; unreachable in-field.
    return static_cast<gp_scalar>(2) *
           std::asin(std::min(half_chord, static_cast<gp_scalar>(1)));
}

// (sin θ, cos θ) pair for θ = atan2(ry − ly, rx − lx). Convention: θ = 0
// when the port→starboard line projects horizontally with port on
// image-plane-left (chase + target wings level relative to each other).
//
// Measured IN THE BEARING PLANE, deliberately — see the header note. The
// degenerate check below therefore uses the planar distance while span uses the
// spherical one; the two agree to O(θ²) at any separation small enough to trip
// the guard, so the guard fires on the same pairs either way.
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

#pragma once

// 030 M7b — Trail rabbit (FR-008 + Session 2026-05-07 Q1 simplification).
//
// The trail rabbit is the per-tick fitness target: a point N meters
// behind the source craft along its current velocity vector. Chase
// craft fitness pulls toward the rabbit (cone-surface fitness reuses
// the pathgen FitnessComputer with the rabbit substituted for the
// path-progress point).
//
// `rabbit_world = target_pos − (target_velocity / |target_velocity|) × trail_distance`
//
// v1 simplification (Session 2026-05-07 Q1): no nose-direction fallback
// (deferred to 031 backlog). Degenerate case (|target_velocity| < 1e-3)
// collapses rabbit to target position itself — fitness gradient still
// points toward target, opportunity cost of forfeited fitness is the
// only "penalty" for being on top of a stationary target.

#include "autoc/eval/source_trajectory.h"  // SourceTickSample
#include "autoc/types.h"

namespace autoc::eval {

constexpr gp_scalar kDefaultTrailDistance = static_cast<gp_scalar>(3.048);  // 10 ft (FR-008)
constexpr gp_scalar kDegenerateVelocityThreshold = static_cast<gp_scalar>(1e-3);

// Compute the trail-rabbit world position from a source-tick sample.
// `trail_distance` configurable via autoc-tracker.ini (TrailDistance).
//
// Pure function: same input → same output, no side effects, no PRNG,
// no history (Session 2026-05-07 Q1 simplification — was per FR-008a
// supposed to use prior-tick state for the velocity-vs-nose hysteresis,
// dropped for v1).
gp_vec3 computeTrailRabbit(const SourceTickSample& target,
                            gp_scalar trail_distance = kDefaultTrailDistance);

}  // namespace autoc::eval

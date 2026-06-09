#pragma once

// 030 M7c — Crash hull (FR-008b + R3 + Session 2026-05-04 Q1).
//
// A geometric "kill zone" around the target craft. When the chase craft's
// position enters the hull AND `p_crash` fires (curriculum-anneal random
// draw), the scenario terminates immediately with `CrashReason::Sim`
// (parallel to arena egress's CrashReason::Eval). The "penalty" is
// the opportunity cost of forfeited remaining-tick fitness; no separate
// negative-magnitude penalty added (per Session 2026-05-04 Q1).
//
// v1 hull = SPHERE of 1m radius around target CG (per FR-008b committed
// 2026-05-04). hb1-sized chase + hb1-sized target → 1m sphere is a
// coarse-but-honest first approximation; airframe-shape fidelity is a
// 031-candidate refinement.
//
// Mode gating: only invoked from tracker-mode scenarios. Pathgen
// scenarios never call into this module — there is no "target craft"
// in pathgen mode.

#include <cstdint>

#include "autoc/types.h"

namespace autoc::eval {

// v1 only SPHERE; AABB_HB1 / MESH_AIRFRAME reserved for future
// fidelity bumps without needing to renumber.
enum class CrashHullShape : int {
    SPHERE = 0,
    AABB_HB1 = 1,        // reserved
    MESH_AIRFRAME = 2,   // reserved
};

struct CrashHull {
    CrashHullShape shape = CrashHullShape::SPHERE;
    gp_scalar sphere_radius_m = static_cast<gp_scalar>(1.0);  // FR-008b v1 default
};

// Geometric: is chase position inside the hull around the target?
// SPHERE: |chase_pos - target_pos| <= sphere_radius_m.
// Other shapes return false in v1 (reserved enum values).
bool isInsideHull(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const gp_vec3& target_pos);

// Probabilistic firing: returns true iff chase is inside the hull AND
// a Bernoulli(p_crash) draw fires. Uses the caller's PRNG seed to
// preserve scenario-level determinism (FR-009).
//
// `prng_state` is updated in place; caller manages PRNG lifecycle. The
// implementation uses an LCG step for portability + zero allocation.
bool didCrashFire(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const gp_vec3& target_pos,
                  gp_scalar p_crash_this_tick,
                  uint32_t& prng_state);

// 030 M11.preA.3 (2026-05-10) — pCrashForGen() removed. Replaced by a
// fixed Bernoulli probability per NN tick (config.crashHullProbability,
// wired in autoc.cc → EvalData.pCrashThisGen → didCrashFire). The
// curriculum ramp added per-gen state that complicated determinism
// debugging; fixed prob is deterministic per (scenario, scenarioSeed-derived rabbit subseed).

}  // namespace autoc::eval

#ifndef AUTOC_EVAL_CRAFT_OBSERVATIONS_H
#define AUTOC_EVAL_CRAFT_OBSERVATIONS_H

// 041 P2-2 — the producer-side writer for the craft observations that BOTH
// modes' gathers then only copy.
//
// WHY THIS EXISTS RATHER THAN TWO CALL SITES. `CraftCommonInputs` gave the two
// modes one definition of the SLOTS; this gives them one definition of how the
// slots are FILLED. Without it, M1 (the crrcsim bridge) and M2 (the tracker
// stepper and its crrcsim helper) would each compute Es and the boundary
// closure rate from the same state in their own code — three copies of a
// sign-sensitive formula, which is the parallel-definition hazard moved one
// layer up rather than removed.
//
// What is NOT here: the score gradient. It is genuinely mode-specific in its
// SOURCE — M1 and M2-phase-1 have an exact closed form from the virtual
// target's geometry, and M2 phase 2 must proxy it from camera range estimation
// (a 043 study). Its producer therefore stays at each mode's tick path, where
// the target geometry is in scope.

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/eval/energy_state.h"
#include "autoc/types.h"

namespace autoc {
namespace eval {

// Outward radial velocity toward the cylinder wall, m/s. POSITIVE = closing.
//
// ⚠️ HORIZONTAL radius only, matching `inwardBodyDirection` and the RADIUS
// egress test. The floor and the ceiling are bounds too, but they are already
// covered — `DIST_TO_BOUNDARY` ray-casts against all three, and `Es` is
// literally height above the floor. This slot is the wall's missing rate term.
//
// ⚠️ Cadence-invariant by construction: an instantaneous dot product of
// position and velocity, with no dt and therefore no one-tick assumption to
// re-derive at another rate.
//
// Returns 0 on the cylinder axis, where "outward" has no direction — the same
// degenerate case `inwardBodyDirection` returns Zero for, answered the same way.
inline gp_scalar boundaryClosureRate(const gp_vec3& pos, const gp_vec3& vel) {
    const gp_scalar r = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
    if (r < static_cast<gp_scalar>(1e-6)) return static_cast<gp_scalar>(0);
    return (pos.x() * vel.x() + pos.y() * vel.y()) / r;
}

// Fill the two mode-agnostic craft observations on the state, before the NN
// runs. Both are stored UNSCALED (metres, m/s); the gather applies the NN
// normalization at the slot write and nowhere else.
inline void writeCraftObservations(AircraftState& state,
                                   const FlightArena& arena) {
    const gp_scalar h_hd = heightAboveDeck(state.getPosition().z(),
                                           SIM_INITIAL_ALTITUDE,
                                           arena.floor_agl_m);
    state.setSpecificEnergy(specificEnergy(h_hd, state.getRelVel()));
    state.setBoundaryClosureRate(
        boundaryClosureRate(state.getPosition(), state.getVelocity()));
}

}  // namespace eval
}  // namespace autoc

#endif  // AUTOC_EVAL_CRAFT_OBSERVATIONS_H

#ifndef AUTOC_EVAL_ENERGY_STATE_H
#define AUTOC_EVAL_ENERGY_STATE_H

// 041 P2-2 — specific energy `Es` and specific excess power `Ps`, in ONE place.
//
// WHY A SHARED HEADER, exactly as for specific_force.h: three consumers need
// these and must not be able to disagree —
//   * `src/nn/evaluator.cc` feeds Es to the network as SPECIFIC_ENERGY,
//   * `src/eval/fitness_decomposition.cc` scores Ps as a lexicase axis,
//   * `tools/dmp_dump.cc` reports both per tick.
// An objective and an observation that drift apart is precisely how 035 failed:
// it selected on an energy quantity the policy had no way to observe.
//
// ⛔ THE ORDER OF THOSE TWO MATTERS. Never add the Ps axis without the Es
// input. TA03 found the input vector carried AIRSPEED and no height term at
// all, so `Es = h + v²/2g` was UNOBSERVABLE — the policy had the v² half and
// never the h half. A selection axis for an unobservable can only mute the
// whole regiment, which is what happened.
//
// THE DATUM: height above the arena FLOOR (the hard deck), never AGL and never
// the virtual origin.
//   * AGL is NOT reproducible in flight. In sim `checkArenaBounds` derives
//     altitude from a ground plane the sim knows; in flight `resolveEngageArena`
//     builds the band from the engage point, because the aircraft has no ground
//     reference. An AGL input would have been a sim-only quantity — the same
//     class of error the datum work exists to remove.
//   * Both sides carry an explicit floor, so `h_hd` is defined identically in
//     each frame's own terms.
//   * It is also the operationally meaningful quantity: energy above the deck
//     is energy you can spend; below it is a crash. That unifies containment
//     with energy management rather than leaving them separate concerns, and it
//     is what "hard deck" means in the E-M framing the rest of this borrows.
//   * ⚠️ Negative Es is MEANINGFUL (below the deck), not an error to clamp.
//
// Frames: world NED, +z DOWN. `h_hd` is therefore a SUBTRACTION of z values,
// and the sign is the thing to get wrong — see heightAboveDeck below.

#include "autoc/types.h"

namespace autoc {
namespace eval {

// Standard gravity, m/s². Es is reported in METRES of energy height, so this is
// a real physical constant and not a tuning knob.
constexpr gp_scalar kStandardGravity_mps2 = static_cast<gp_scalar>(9.80665);

// Height above the hard deck, in metres, from a virtual-frame z and the arena.
//
// ⚠️ Every term here is down-positive NED, which is why this is a named
// function rather than an inline subtraction at three call sites. `floor_agl_m`
// is an ALTITUDE (up-positive), `virtual_z` is a DEPTH (down-positive), and
// `sim_initial_altitude` is the negative offset between the virtual origin and
// the ground datum. Getting one sign wrong yields a plausible number.
//
//   altitude_agl = -(virtual_z + sim_initial_altitude)
//   h_hd         = altitude_agl - floor_agl_m
inline gp_scalar heightAboveDeck(gp_scalar virtual_z,
                                 gp_scalar sim_initial_altitude,
                                 gp_scalar floor_agl_m) {
    return -(virtual_z + sim_initial_altitude) - floor_agl_m;
}

// Specific energy: Es = h_hd + v²/2g, in METRES.
//
// Total mechanical energy per unit weight — "the altitude I could reach if I
// traded all my speed for height, measured from the deck". Given to the network
// as the STATE, not only as the rate: Es is the integral, and making the
// recurrent layer accumulate it would spend capacity that is already unfilled
// (measured W_hh effective rank 11.3 of 16).
inline gp_scalar specificEnergy(gp_scalar height_above_deck_m,
                                gp_scalar speed_mps) {
    return height_above_deck_m +
           (speed_mps * speed_mps) /
               (static_cast<gp_scalar>(2) * kStandardGravity_mps2);
}

// Specific excess power: Ps = dEs/dt, in METRES PER SECOND.
//
// The rate at which the aircraft is gaining or losing total energy — the Boyd
// E-M quantity. Positive means it is buying energy (climbing without slowing,
// or accelerating without descending); negative means it is spending it.
//
// ⚠️ CADENCE-INVARIANT BY CONSTRUCTION: the caller passes the actual elapsed
// seconds, never an assumed one-tick interval. That is the same discipline
// CLOSING_RATE and SPAN_RATE were given at 037 T019/T022, and for the same
// reason — a hidden one-tick assumption silently rescales when the cadence
// moves and nothing reports it. A non-positive dt returns 0: there is no
// defensible rate over no time.
//
// ⚠️ Ps is available to the OBJECTIVE without being an input. TA03 measured
// corr(Ps, closure rate) = −0.048 — the energy axis is orthogonal to the
// tracking axis, which is the ideal case for lexicase (complementary pressure)
// and precisely the wrong case for scalar aggregation (033's Pareto-corner
// collapse). Never a scalar penalty term.
inline gp_scalar specificExcessPower(gp_scalar es_now_m,
                                     gp_scalar es_prev_m,
                                     gp_scalar dt_sec) {
    if (!(dt_sec > static_cast<gp_scalar>(0))) return static_cast<gp_scalar>(0);
    return (es_now_m - es_prev_m) / dt_sec;
}

}  // namespace eval
}  // namespace autoc

#endif  // AUTOC_EVAL_ENERGY_STATE_H

#pragma once

// 030 M7a — Arena geometry (FR-016 + Session 2026-05-07 Q1).
//
// Cylindrical operating volume with hard floor + ceiling AGL bounds.
// Single source of truth for two consumers:
//
//   1. Per-tick OOB termination check (TrackerStepper::stepOnce) —
//      replaces the M8b shim's shared `checkAircraftOOB` for tracker mode.
//      Pathgen mode keeps its M1 hardcoded check unchanged (regression
//      invariant).
//
//   2. NN sensor input `DIST_TO_BOUNDARY_ALONG_VEL` (TrackerInputs
//      slot 44, fed via gather_tracker_inputs) — meters of safe forward
//      flight along chase velocity until ray intersects cylinder wall,
//      floor, or ceiling. Same ray-projection math feeds both consumers
//      so what the controller sees and what the egress detector enforces
//      are computed from the same code path.
//
// 041 P2-3 — ONE arena, for both modes and both sides: radius 70 m, and a
// VERTICALLY ASYMMETRIC band of **+50 m up / −30 m down** about the arm origin.
// In sim that lands at 25 / 105 m AGL with the arm point at 55.
//
// ⭐ THE ARENA IS RELATIVE, NOT ABSOLUTE. Radius R about the arm origin, ceiling
// `up` above it and floor `down` below it — that is the whole definition. The
// AGL numbers are only where the band sits IN SIM, where the ground is known;
// `resolveEngageArena` places the identical band around wherever the aircraft
// actually armed.
//
// ⭐ WHY ASYMMETRIC, AND WHY THESE NUMBERS. The band used to be ±K with the arm
// at the centre; the max-extent measurement showed how badly that fit, since the
// M1 rabbit climbs **34.98 m** above the origin and descends **2.74 m** below it.
//
// ⛔ But sizing to the RABBIT was the second mistake, and the t2 smoke found it:
// at +60/−10 **16 of 16 scenarios died on the deck** (terminal AGL 25.01–25.51),
// mean survival 4.9 s, elite fitness frozen for 11 generations — while the
// ceiling was never approached (48 m of 95). The rabbit flies flat and never
// descends; the CHASE manoeuvres, bleeds energy and settles. It is the chase
// that is contained, so it is the chase that sizes the band.
//
// +50 / −30 covers the t1 chase's measured 17.9 m of descent (itself
// floor-truncated) and still clears the rabbit's climb by 15 m.
//
// ⛔ Keeping the band above terrain and inside the site's 400 ft working
// envelope are BOTH operator responsibilities exercised at arm time (operator
// 2026-08-18: *"Not the models problem. Is operator."*). Nothing here enforces
// an absolute altitude, and nothing should. (For reference: at a 35 m arm the
// ceiling sits at 105 m AGL = 344 ft.)
// Defaults match every .ini; `AutocConfig` populates from .ini at startup and
// plumbs through `EvalData::flightArena` to the worker.
//
// ⭐ The entry / arm point is at the band's exact vertical centre
// (73 m AGL = −SIM_INITIAL_ALTITUDE), and that is load-bearing rather than
// tidy: it makes `resolveEngageArena(...).virtual_arena` come out EQUAL to this
// struct, so sim and flight stop agreeing by convention and start agreeing by
// construction. arena_tests.cc asserts that equality.

// Note: no cereal include here — FlightArena::serialize is a template
// that instantiates only in the protocol.h cereal context (which already
// has cereal). Keeping this header cereal-free lets xiao firmware
// transitively include arena.h (via evaluator.cc) without needing
// cereal in the embedded build path.

#include "autoc/eval/aircraft_state.h"  // SIM_INITIAL_ALTITUDE
#include "autoc/rpc/crash_reason.h"      // CrashReason (factored out 030 M7a)
#include "autoc/types.h"

namespace autoc::eval {

// Cylindrical arena with axis aligned to world Z (NED), centered on the
// virtual frame origin (0, 0, 0). Floor + ceiling expressed as AGL
// altitudes; raw NED z conversion uses SIM_INITIAL_ALTITUDE so the arena
// follows the operator's "set origin here" frame reset (see FR-016 +
// Session 2026-05-07 Q1).
struct FlightArena {
    gp_scalar radius_m = static_cast<gp_scalar>(70.0);
    gp_scalar floor_agl_m = static_cast<gp_scalar>(25.0);
    gp_scalar ceiling_agl_m = static_cast<gp_scalar>(105.0);

    template <class Archive>
    void serialize(Archive& ar) {
        ar(radius_m, floor_agl_m, ceiling_agl_m);
    }
};

// Egress-kind enum recorded in M2 dmp telemetry (per-scenario count of
// each kind, populated by FitnessComputer at scenario terminate).
enum class ArenaEgressKind : int {
    NONE = 0,        // chase still inside arena
    RADIUS = 1,      // horizontal distance > radius_m
    FLOOR = 2,       // altitude AGL < floor_agl_m
    CEILING = 3,     // altitude AGL > ceiling_agl_m
};

// Per-tick egress check. Returns NONE if chase still inside arena;
// otherwise the kind of bound that was crossed. Maps `NONE` → keep going,
// any other value → scenario terminator (CrashReason::Eval, parallel to
// PathgenStepper's M1 OOB termination semantics).
inline ArenaEgressKind checkArenaBounds(const AircraftState& state,
                                        const FlightArena& arena) {
    const gp_vec3 pos = state.getPosition();  // virtual frame
    const gp_scalar horiz_dist = std::sqrt(pos.x() * pos.x() + pos.y() * pos.y());
    if (horiz_dist > arena.radius_m) return ArenaEgressKind::RADIUS;

    // virtual_z + SIM_INITIAL_ALTITUDE = raw NED z (altitude reads
    // negative; -raw_z = altitude AGL when ground is at raw_z=0).
    const gp_scalar alt_agl = -(pos.z() + SIM_INITIAL_ALTITUDE);
    if (alt_agl < arena.floor_agl_m) return ArenaEgressKind::FLOOR;
    if (alt_agl > arena.ceiling_agl_m) return ArenaEgressKind::CEILING;
    return ArenaEgressKind::NONE;
}

// Convenience: ArenaEgressKind → CrashReason for stepper integration.
inline CrashReason arenaEgressToCrashReason(ArenaEgressKind k) {
    return (k == ArenaEgressKind::NONE) ? CrashReason::None : CrashReason::Eval;
}

// Per-tick NN-input scalar: meters of safe forward flight along chase
// velocity vector before ray intersects cylinder wall, floor, or ceiling.
// Returns the minimum positive `t` over all three constraints (ray
// `r(t) = chase_pos + velocity_unit * t`).
//
// Returns `kSafeBoundaryDistance` (large sentinel) when:
//   - velocity is degenerate (|velocity| < 1e-6), OR
//   - ray heading away from all boundaries (cylinder discriminant < 0
//     AND vz pointing toward neither floor nor ceiling)
//
// NN learns "small number = trouble approaching; let body attitude /
// quat / gyro figure out which way to turn" — Session 2026-05-07 Q1.
constexpr gp_scalar kSafeBoundaryDistance = static_cast<gp_scalar>(1000.0);

gp_scalar distanceToBoundary(const gp_vec3& chase_pos,
                             const gp_vec3& chase_velocity,
                             const FlightArena& arena);

// 038 P0-D FR-P0H (B) — arena-inward body-frame unit vector. Radial direction
// toward the cylinder axis (the virtual-frame origin) at the chase's horizontal
// position, rotated into the chase body frame. All-attitude and smooth (works
// inverted / knife-edge), unlike a planar heading angle that assumes a
// reference "up" and drops the out-of-plane component. Returns Zero when the
// chase is on the axis (hypot(x, y) < 1e-6 — no defined inward direction).
//
// Convention: world→body = chase_orientation.inverse() (matches
// camera_projection.cc:117 and aircraft_state.h:440). The arena axis is
// vertical (world Z), so the world-frame inward vector has zero z-component;
// the body-frame result gains all three cosines through the rotation.
gp_vec3 inwardBodyDirection(const gp_vec3& chase_pos,
                            const gp_quat& chase_orientation);

// ============================================================================
// 039 FR-001 / D5 — engage-scoped arena resolution.
//
// The firmware re-centers the arena at every span activation: the template
// geometry (nn2cpp -a literal) contributes its radius and its vertical
// EXTENTS; the placement comes from the engage point.
//
//   raw NED (down-positive):  floor_Z = z_engage + down,  ceiling_Z = z_engage − up
//   horizontal:               cylinder axis at engage x/y
//
// ⚠️ 041 P2-3 — `up` AND `down` ARE DERIVED SEPARATELY. This used to compute a
// single half-band `K = (ceiling_agl − floor_agl)/2` and place the band at ±K,
// which is only correct when the arm point is the vertical CENTRE of the
// template. The 041 band is deliberately asymmetric (+60 / −10), so a ±K
// placement would silently hand the aircraft a ±35 m band: 25 m less room above
// than it trained with and 25 m more below. Every number would have looked
// plausible.
//
// ⭐ Derived this way, `virtual_arena` is an EXACT IDENTITY on the template, for
// ANY engage altitude and ANY asymmetry — which is the property that makes sim
// and flight the same arena by construction rather than by convention.
// `arena_tests.cc` asserts it.
//
// The xiao expresses AircraftState in an engage-zeroed virtual frame
// (msplink.cpp test_origin_offset), so `virtual_arena` is the same band
// translated into that frame for the existing consumers (gather_pathgen_inputs
// → distanceToBoundary / inwardBodyDirection, checkArenaBounds).
//
// Constitution VII: EngageArena has NO defaults — it exists only as the
// result of resolveEngageArena at span activation.
// ============================================================================
struct EngageArena {
    gp_vec3 origin_ned;        // raw NED engage point (EngageHeader provenance)
    gp_scalar up_m;            // template extent ABOVE the arm point
    gp_scalar down_m;          // template extent BELOW the arm point
    gp_scalar floor_z_ned;     // raw NED: z_engage + down
    gp_scalar ceiling_z_ned;   // raw NED: z_engage − up
    FlightArena virtual_arena; // engage-zeroed virtual frame (gather/bounds)
};

inline EngageArena resolveEngageArena(const FlightArena& geometry,
                                      const gp_vec3& engage_pos_ned) {
    // The arm point's own altitude within the template band. In sim this is the
    // virtual origin; in flight it is where the aircraft armed.
    const gp_scalar arm_agl = -SIM_INITIAL_ALTITUDE;
    const gp_scalar up = geometry.ceiling_agl_m - arm_agl;
    const gp_scalar down = arm_agl - geometry.floor_agl_m;
    return EngageArena{
        engage_pos_ned,
        up,
        down,
        engage_pos_ned.z() + down,
        engage_pos_ned.z() - up,
        FlightArena{geometry.radius_m, arm_agl - down, arm_agl + up},
    };
}

}  // namespace autoc::eval

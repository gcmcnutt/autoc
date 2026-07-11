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
// Default values match `autoc-tracker.ini` (FlightArenaRadius / FloorAGL
// / CeilingAGL, 80 / 5 / 100 m); `AutocConfig` populates from .ini at
// startup, plumbs through `EvalData::flightArena` to the worker.

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
    gp_scalar radius_m = static_cast<gp_scalar>(80.0);
    gp_scalar floor_agl_m = static_cast<gp_scalar>(5.0);
    gp_scalar ceiling_agl_m = static_cast<gp_scalar>(100.0);

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
// 039 FR-001 / D5 — engage-scoped arena resolution (pure ±K rule, no
// min-elevation clamp per the 2026-07-10 simplification).
//
// The firmware re-centers the arena at every span activation: the template
// geometry (nn2cpp -a literal, e.g. the 80/5/100 training arena) contributes
// only its radius and its vertical EXTENT K = (ceiling_agl − floor_agl)/2;
// the placement comes from the engage point:
//
//   raw NED (down-positive):  floor_Z = z_engage + K,  ceiling_Z = z_engage − K
//   horizontal:               cylinder axis at engage x/y
//
// The xiao expresses AircraftState in an engage-zeroed virtual frame
// (msplink.cpp test_origin_offset), so `virtual_arena` is the same band
// translated into that frame for the EXISTING consumers
// (gather_pathgen_inputs → distanceToBoundary / inwardBodyDirection,
// checkArenaBounds): the craft sits at virtual (0,0,0) = AGL
// −SIM_INITIAL_ALTITUDE at engage, hence floor/ceiling AGL = that ± K.
// At engage the craft reads CENTER of the band by construction
// (bench contract FR-002 item 4). Limits are safety-only this phase.
//
// Constitution VII: EngageArena has NO defaults — it exists only as the
// result of resolveEngageArena at span activation.
// ============================================================================
struct EngageArena {
    gp_vec3 origin_ned;        // raw NED engage point (EngageHeader provenance)
    gp_scalar half_band_m;     // K, from the template geometry
    gp_scalar floor_z_ned;     // raw NED: z_engage + K
    gp_scalar ceiling_z_ned;   // raw NED: z_engage − K
    FlightArena virtual_arena; // engage-zeroed virtual frame (gather/bounds)
};

inline EngageArena resolveEngageArena(const FlightArena& geometry,
                                      const gp_vec3& engage_pos_ned) {
    const gp_scalar k = (geometry.ceiling_agl_m - geometry.floor_agl_m)
                        / static_cast<gp_scalar>(2.0);
    const gp_scalar engage_agl = -SIM_INITIAL_ALTITUDE;  // virtual (0,0,0) in AGL terms
    return EngageArena{
        engage_pos_ned,
        k,
        engage_pos_ned.z() + k,
        engage_pos_ned.z() - k,
        FlightArena{geometry.radius_m, engage_agl - k, engage_agl + k},
    };
}

}  // namespace autoc::eval

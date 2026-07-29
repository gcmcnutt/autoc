#pragma once

// 040 T013 (FR-007, FR-008, FR-009) — chase-airframe obstruction model.
//
// Replaces the single AABB `AirframeProxy` this supersedes, which was unusable
// for two independent reasons:
//
//   1. It modelled a 30″ × 7″ × 1″ wing as a SOLID BRICK spanning
//      y ∈ [-0.6, +0.6], z ∈ [-0.05, +0.20] — grossly over-obstructing.
//   2. It was DEGENERATE: the default camera mount sat at z = -0.05, exactly
//      on box_min_z, and a surface touch counts as a hit — so enabling
//      occlusion obstructed essentially every forward ray. Pinned by
//      tests/beacon_projection_tests.cc AirframeObstruction.*
//
// Three primitives instead, because for a forward-looking camera the boom,
// dowels and vertical fin are all AFT and contribute nothing:
//
//   wing  — a THIN SLAB (thin in z: a plate, not a brick)
//   nose  — the pod box forward of the wing leading edge
//   prop  — a STATIC angular disc region
//
// PROPELLER MODELLING (FR-009). The disc is a static region applying PARTIAL
// ATTENUATION — a blade covering part of the entrance pupil — NOT a binary
// cutoff, and with NO engine-speed or blade-phase dependence. At the 040
// baseline mount (leading edge, 8″ outboard) the disc sits 41–61° inboard, so
// phase fidelity buys nothing. Engine-speed-resolved modelling is backlogged
// with an explicit trigger: a mount that puts the camera behind the disc.
//
// DETERMINISM (FR-020): pure geometry. No clock, no PRNG, no carried state.

#include "autoc/types.h"

namespace autoc::eval {

// Outcome of testing one line of sight against the airframe.
struct ObstructionResult {
    // An opaque solid (wing or nose) blocks the ray. Hard gate.
    bool blocked;
    // Multiplier in [0, 1] from crossing the propeller disc; 1.0 = clear.
    // Partial by construction — see the FR-009 note above.
    gp_scalar attenuation;
};

// Chase airframe geometry, body frame (NED: +x forward, +y right, +z down).
//
// No in-class default initializers on the geometry (Constitution VII): every
// field is supplied by the caller, so adding a primitive becomes a compile
// error at the construction site rather than a silent zero-sized box that
// obstructs nothing and looks like it works.
struct AirframeObstruction {
    bool enabled;

    // Wing — thin slab. `wing_max.z - wing_min.z` is the true airfoil
    // thickness; that thinness is the whole point of replacing the brick.
    gp_vec3 wing_min;
    gp_vec3 wing_max;

    // Pod nose — box forward of the wing leading edge.
    gp_vec3 nose_min;
    gp_vec3 nose_max;

    // Propeller disc: plane at body x = prop_plane_x, centred on the thrust
    // axis at (prop_axis_y, prop_axis_z), out to prop_radius.
    gp_scalar prop_plane_x;
    gp_scalar prop_axis_y;
    gp_scalar prop_axis_z;
    gp_scalar prop_radius;
    // Fraction of signal REMOVED when the ray crosses the disc (so the
    // multiplier is 1 - prop_attenuation). Representative of blade-over-pupil
    // duty; classified ASSUMED pending measurement.
    gp_scalar prop_attenuation;

    // 040 — cereal serialize for the WorkerInit RPC carry. Not a persisted
    // artifact: WorkerInit travels autoc → crrcsim worker, both always built
    // together, and never enters the dmp.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(enabled, wing_min, wing_max, nose_min, nose_max,
           prop_plane_x, prop_axis_y, prop_axis_z, prop_radius,
           prop_attenuation);
    }
};

// Ray–box intersection over the SEGMENT a→b (slab method). Touching a face
// counts as a hit — which is exactly why a camera must never be positioned
// on a primitive boundary; see the degeneracy note above.
bool rayHitsBox(const gp_vec3& a, const gp_vec3& b,
                const gp_vec3& lo, const gp_vec3& hi);

// Does the segment camera→target cross the propeller disc?
bool rayCrossesPropDisc(const gp_vec3& camera, const gp_vec3& target,
                        const AirframeObstruction& airframe);

// Test one line of sight. `blocked` is set by the opaque primitives;
// `attenuation` by the propeller. A disabled airframe returns
// {false, 1.0} — geometry is irrelevant when obstruction is off.
ObstructionResult testObstruction(const gp_vec3& camera,
                                  const gp_vec3& target,
                                  const AirframeObstruction& airframe);

// Measured airframe dimensions, all externally configured (FR-034). Units are
// INCHES because that is how the airframe was measured; conversion to metres
// happens once inside the builder rather than being scattered through the ini.
//
// No in-class defaults (Constitution VII): every value arrives from config, so
// a missing key is a loud failure rather than a silent stale geometry that
// looks plausible and obstructs the wrong things.
struct AirframeDimensions {
    // Station stack, measured AFT from the propeller disc (station 0).
    gp_scalar camera_station_in;      // camera at the wing leading edge
    gp_scalar wing_le_station_in;
    gp_scalar wing_chord_in;
    gp_scalar wing_span_in;
    gp_scalar wing_thickness_in;

    // Vertical stack, measured UP from the thrust line (up = -z in body NED).
    gp_scalar wing_bottom_above_thrust_in;
    gp_scalar camera_above_wing_bottom_in;

    // Lateral: camera outboard of the thrust axis.
    gp_scalar camera_outboard_in;

    // Propeller.
    gp_scalar prop_diameter_in;
    gp_scalar prop_attenuation;

    bool enabled;
};

// Build the obstruction set in body frame.
//
// ANCHORING (040 T013): the station stack is RELATIVE, and the thrust line's
// body-frame position is not yet measured (input-data-checklist A1b). The one
// station whose body-frame position IS known is the camera, from config — so
// the geometry is anchored to the camera mount and every other primitive is
// placed by measured offset from it. When A1b lands this can anchor to the
// thrust line directly and the camera dependency disappears.
AirframeObstruction buildAirframeObstruction(const gp_vec3& camera_mount_body,
                                             const AirframeDimensions& dims);

// The measured HB1 values, for tests and as the ini defaults' source of truth.
AirframeDimensions hb1AirframeDimensions();

}  // namespace autoc::eval

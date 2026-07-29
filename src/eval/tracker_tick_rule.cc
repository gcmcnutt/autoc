// 040 T010 (FR-031) — single-sourced per-tick tracker perception rule.
//
// Extracted VERBATIM from the two prior copies (crrcsim_tracker_helper.cpp and
// tracker_stepper.cc). Stage B is behaviour-preserving by contract: the
// bit-identity gate (T021) either reproduces the pinned elite's fitness exactly
// or there is a bug. No improvement rides along with this extraction — that is
// what makes the gate meaningful, and it is the last objective pass/fail in the
// feature, since every later stage deliberately changes outputs.

#include "autoc/eval/tracker_tick_rule.h"

namespace autoc::eval {

PerceptionTickResult projectPerceptionTick(const AircraftState& chase,
                                           const SourceTickSample& target,
                                           const TickRuleConfig& config) {
    ProjectionInput proj;
    proj.chase_position_world = chase.getPosition();
    proj.chase_orientation_world = chase.getOrientation();
    proj.target_position_world = target.position;
    proj.target_orientation_world = target.orientation;
    proj.camera_mount_chase_body = config.camera.mount_offset_body;
    proj.camera_orientation_chase_body = config.camera.mount_orientation_body;
    proj.camera = config.camera;
    proj.chase_airframe = config.airframe;

    // Left beacon.
    proj.beacon_mount_target_body = config.beacon_left.mount_body;
    proj.beacon_emission_axis_target_body = config.beacon_left.emission_axis_body;
    proj.beacon = config.beacon_left;
    const BeaconObservation left = projectBeacon(proj);

    // Right beacon.
    proj.beacon_mount_target_body = config.beacon_right.mount_body;
    proj.beacon_emission_axis_target_body = config.beacon_right.emission_axis_body;
    proj.beacon = config.beacon_right;
    const BeaconObservation right = projectBeacon(proj);

    PerceptionTickResult out;
    out.left = left;
    out.right = right;

    // 040 T031 — these carry BEARINGS IN RADIANS now, not ±1 NDC. Everything
    // downstream (span, span_rate, tilt, the aux predictor's realized target)
    // inherits the new scale.
    out.record.left_x = left.bearing_x_rad;    // raw-ok: NN-byte-format primitive
    out.record.left_y = left.bearing_y_rad;    // raw-ok: NN-byte-format primitive
    out.record.left_cep = left.cep;            // raw-ok: NN-byte-format primitive
    out.record.right_x = right.bearing_x_rad;  // raw-ok: NN-byte-format primitive
    out.record.right_y = right.bearing_y_rad;  // raw-ok: NN-byte-format primitive
    out.record.right_cep = right.cep;          // raw-ok: NN-byte-format primitive

    // 032 PHASE 1 — beacon-pair separation, CEP-gated: if EITHER beacon's CEP
    // is at or above the threshold, substitute neutral 0.0 rather than compute
    // from an untrusted endpoint. Threshold is compared in float on both paths
    // (TrackerStepper held a gp_scalar member, the crrcsim worker a double from
    // WorkerInit; both cast to float at the comparison) — preserved exactly.
    const float cep_gate = static_cast<float>(config.cep_gate_threshold);  // raw-ok: NN-byte-format comparison boundary
    const bool cep_gated =
        left.cep >= cep_gate ||
        right.cep >= cep_gate;
    if (cep_gated) {
        out.record.span = 0.0f;
    } else {
        out.record.span = static_cast<float>(  // raw-ok: NN-byte-format slot write
            compute_pair_span(
                static_cast<gp_scalar>(left.bearing_x_rad),
                static_cast<gp_scalar>(left.bearing_y_rad),
                static_cast<gp_scalar>(right.bearing_x_rad),
                static_cast<gp_scalar>(right.bearing_y_rad)));
    }

    return out;
}

void advanceSituationalAwareness(const TrackerHistoryWindow& history,
                                 SituationalAwarenessState& sa) {
    // Visibility uses the SENTINEL threshold, not the configured CEP gate —
    // matching fitness_decomposition.cc. The two thresholds are deliberately
    // different: the gate decides whether derived quantities are trustworthy,
    // the sentinel decides whether the beacon was seen at all.
    sa.update(history.left_cep[5], history.right_cep[5], kCepSentinelThreshold);
}

void resetPerceptionState(TrackerObservationRing& ring,
                          SituationalAwarenessState& sa) {
    ring.reset();
    // FR-020a: un-reset blind-tick / exit-bearing state leaks across scenarios
    // and breaks the bitwise gate. The 040 acquisition state machine's reset
    // belongs here too, so both execution paths inherit it without either
    // being edited.
    sa.reset();
}

}  // namespace autoc::eval

// 040 T010 (FR-031) — single-sourced per-tick tracker perception rule.
//
// Extracted VERBATIM from the two prior copies (crrcsim_tracker_helper.cpp and
// tracker_stepper.cc). Stage B is behaviour-preserving by contract: the
// bit-identity gate (T021) either reproduces the pinned elite's fitness exactly
// or there is a bug. No improvement rides along with this extraction — that is
// what makes the gate meaningful, and it is the last objective pass/fail in the
// feature, since every later stage deliberately changes outputs.

#include "autoc/eval/tracker_tick_rule.h"

#include <cmath>

namespace autoc::eval {

PerceptionTickResult projectPerceptionTick(const AircraftState& chase,
                                           const SourceTickSample& target,
                                           const TickRuleConfig& config,
                                           PerceptionCarryState& carry) {
    ProjectionInput proj;
    proj.chase_position_world = chase.getPosition();
    proj.chase_orientation_world = chase.getOrientation();
    proj.target_position_world = target.position;
    proj.target_orientation_world = target.orientation;
    proj.camera_mount_chase_body = config.camera.mount_offset_body;
    proj.obstruction_mount_chase_body = config.obstruction_mount_offset;
    proj.camera_orientation_chase_body = config.camera.mount_orientation_body;
    proj.camera = config.camera;
    proj.chase_airframe = config.airframe;
    proj.signal = config.signal;

    // Left beacon.
    proj.beacon_mount_target_body = config.beacon_left.mount_body;
    proj.beacon_emission_axis_target_body = config.beacon_left.emission_axis_body;
    proj.beacon = config.beacon_left;
    BeaconObservation left = projectBeacon(proj);

    // Right beacon.
    proj.beacon_mount_target_body = config.beacon_right.mount_body;
    proj.beacon_emission_axis_target_body = config.beacon_right.emission_axis_body;
    proj.beacon = config.beacon_right;
    BeaconObservation right = projectBeacon(proj);

    // ---------------------------------------------------------------------
    // 040 US4 — the pair-level stage. Everything below needs BOTH beacons or
    // needs history, so none of it can live inside projectBeacon.
    // ---------------------------------------------------------------------

    const bool left_visible = left.raw_px_x != kPixelSentinel;
    const bool right_visible = right.raw_px_x != kPixelSentinel;

    // Pixel gap between the two blobs. This one number drives BOTH envelopes.
    gp_scalar px_gap = static_cast<gp_scalar>(0);
    if (left_visible && right_visible) {
        const gp_scalar dx = static_cast<gp_scalar>(left.raw_px_x - right.raw_px_x);
        const gp_scalar dy = static_cast<gp_scalar>(left.raw_px_y - right.raw_px_y);
        px_gap = std::sqrt(dx * dx + dy * dy);
    }

    // FR-016 — a SHARED DETECTOR ELEMENT. Field-proven, not a failure mode: the
    // 031 single-detector rig is exactly this case and decodes both codes
    // reliably. It costs the measured interference penalty and it costs SPATIAL
    // SEPARATION; it does not cost detection.
    const bool shares_element =
        left_visible && right_visible && px_gap <= config.signal.shared_element_px;

    // FR-033 — the SEPARATION envelope, which is NOT the detection envelope.
    // Bearing reaches the asserted detection range; separation-derived range
    // dies far sooner, once the pair no longer resolves as two blobs. At 120°
    // over 320 px the 0.772 m pair subtends ≈1 px at 100 m — reporting a range
    // from that would be reporting a quantisation artefact as a measurement.
    const bool separation_resolvable =
        left_visible && right_visible && px_gap >= config.signal.separation_min_px;

    // Advance the acquisition machines. `signal_present` is GEOMETRIC visibility
    // only — FR-033a asserts the detection envelope rather than deriving it, so
    // SNR shapes quality within the envelope and never cuts visibility short.
    const gp_scalar cdma =
        shares_element ? config.signal.cdma_penalty_db : static_cast<gp_scalar>(0);

    auto refine = [&](BeaconObservation& obs, AcquisitionState& acq, bool visible) {
        const gp_scalar snr_db =
            visible ? static_cast<gp_scalar>(obs.raw_margin) - cdma
                    : static_cast<gp_scalar>(-999);
        const gp_scalar signal_q =
            visible ? qFromSnrDb(snr_db, config.signal) : static_cast<gp_scalar>(0);

        advanceAcquisition(acq, visible, signal_q, config.control_interval_ms,
                           config.acquisition);

        // FR-017d — identity uncertainty inflates quality. Two sources:
        //   (a) pre-decode, i.e. any state short of TRACKING. The blob centroid
        //       exists before the code resolves which beacon it is, and tilt
        //       SIGN rides on that identity while separation does not — a
        //       swapped pair flips tilt 180°, and tilt drives the roll command.
        //   (b) a shared detector element, where there is one blob and no way to
        //       assign two identities to two distinct positions.
        // Quality is the interface's ONLY confidence channel (the vector is
        // fixed at 58), so this is where it has to land.
        const bool identity_uncertain =
            (acq.state != LockState::TRACKING) || shares_element;

        obs.cep = static_cast<float>(  // raw-ok: gp_scalar→NN-byte-format boundary
            qualityFromAcquisition(acq, identity_uncertain, config.acquisition));
        obs.raw_cep_int8 = quantize_cep(obs.cep);
        obs.raw_margin = static_cast<float>(snr_db);  // raw-ok: gp_scalar→cereal byte-format boundary
        obs.lock_state = static_cast<int8_t>(acq.state);

        // A beacon whose quality came back sentinel reports no bearing at all —
        // keep the sentinel form whole rather than leaving a live bearing beside
        // a dead quality value.
        if (obs.cep >= kCepSentinelThreshold) {
            obs.bearing_x_rad = 0.0f;
            obs.bearing_y_rad = 0.0f;
            obs.raw_px_x = kPixelSentinel;
            obs.raw_px_y = kPixelSentinel;
        }
    };

    refine(left, carry.left, left_visible);
    refine(right, carry.right, right_visible);

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
    // 040 T063 (FR-033) — and the resolving limit. Below it the pair is one
    // blob, or close enough that the ±0.5 px quantisation swamps the gap, so
    // separation-derived range is UNAVAILABLE rather than merely imprecise.
    // Neither quantity may be reported as usable outside its own envelope.
    if (cep_gated || !separation_resolvable) {
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

void applyCameraVariation(TickRuleConfig& cfg, const CameraDeltas& draw) {
    // Alignment error is a rotation of the camera about its own axes, applied
    // ON TOP of the nominal mount orientation. Order is roll (about the optical
    // axis, body +x) then pitch then yaw — an intrinsic sequence, so a mount
    // that is both cocked and rolled composes the way a physically misglued
    // bracket would rather than in some arbitrary global order.
    constexpr gp_scalar kDeg = static_cast<gp_scalar>(3.14159265358979323846 / 180.0);
    const gp_quat roll(Eigen::AngleAxis<gp_scalar>(draw.rollDeg * kDeg,
                                                   gp_vec3::UnitX()));
    const gp_quat pitch(Eigen::AngleAxis<gp_scalar>(draw.boresightPitchDeg * kDeg,
                                                    gp_vec3::UnitY()));
    const gp_quat yaw(Eigen::AngleAxis<gp_scalar>(draw.boresightYawDeg * kDeg,
                                                  gp_vec3::UnitZ()));
    cfg.camera.mount_orientation_body =
        (cfg.camera.mount_orientation_body * yaw * pitch * roll).normalized();

    // OBSTRUCTION ONLY (research R6) — bearing keeps the nominal mount.
    // Free to wander: a mount that lands inside a primitive no longer blinds the
    // camera, because `testObstruction` treats a primitive containing the
    // aperture as cut away for it. See airframe_occlusion.cc.
    cfg.obstruction_mount_offset = cfg.camera.mount_offset_body + draw.mountTranslation;

    // Wing slab thickness. The slab runs from wing top (min z, "up" is −z) to
    // underside (max z); thickening it grows the underside downward, which is
    // the direction that can actually intrude on a leading-edge mount's view.
    cfg.airframe.wing_max.z() += draw.wingThicknessDelta;

    // Inert at the shipped 1.0 — see the header note on the emitter staying
    // perfect until the lens+filter field tests land.
    cfg.signal.ambient_floor *= draw.ambientScale;
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
                          SituationalAwarenessState& sa,
                          PerceptionCarryState& carry,
                          EnvelopeState& envelope) {
    ring.reset();
    // FR-020a: un-reset blind-tick / exit-bearing state leaks across scenarios
    // and breaks the bitwise gate.
    sa.reset();
    // 040 T064 — and the acquisition machines, which have identical exposure:
    // a leaked coast window would let a fresh scenario acquire WARM off the
    // previous scenario's lock, which is both wrong and non-reproducible.
    // Landing it here is what makes both execution paths inherit it without
    // either being edited.
    carry.reset();
    // 041 T038 — the envelope accumulator has exactly the same exposure: a
    // leaked accumulation would open a fresh scenario already claiming seconds
    // of envelope occupancy it never had, which is both wrong and
    // non-reproducible.
    envelope.reset();
}

}  // namespace autoc::eval

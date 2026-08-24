#pragma once

// 040 T010 (FR-031) — the single source of truth for the per-tick tracker
// perception rule.
//
// WHY THIS EXISTS. The rule below was duplicated verbatim in two places:
//   - crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp
//     (PRODUCTION — crrcsim FDM has been the sole worker since 034)
//   - src/eval/tracker_stepper.cc
//     (TEST-ONLY reference; the minisim was retired at 034, and TrackerStepper
//      now appears only in tests/tracker_stepper_init_tests.cc)
//
// The test-only twin matters precisely because it is the behavioural contract:
// a reference implementation carrying stale semantics would silently certify
// wrong behaviour. 040 introduces a per-beacon acquisition state machine
// (FR-014..FR-020a) which MUST land here once rather than twice, or the two
// copies diverge the moment either is touched.
//
// DETERMINISM (FR-020 / FR-020a). Nothing here holds a clock, a PRNG, or
// thread-dependent state. All carried per-scenario state is owned by the
// caller and reset through resetPerceptionState() at every scenario boundary —
// unreset state leaks between scenarios and breaks the bitwise gate. The
// existing situational-awareness state already carries that warning; the
// acquisition state added at 040 M4/Stage E has identical exposure.

#include "autoc/eval/acquisition_state.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_variation.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/derived_features.h"
#include "autoc/eval/envelope_state.h"
#include "autoc/eval/source_trajectory.h"
#include "autoc/nn/evaluator.h"  // TrackerObservationRing, SituationalAwarenessState
#include "autoc/nn/nn_inputs.h"
#include "autoc/types.h"

namespace autoc::eval {



// Everything the per-tick rule needs that is not carried state. Both call
// sites already hold these values, but from different homes — TrackerStepper
// keeps them as members, the crrcsim worker reads them from WorkerInit — so
// they are passed explicitly rather than reached for.
//
// No in-class default initializers (Constitution VII): every field is supplied
// by the caller, so a future field addition is a compile error at the call
// site rather than a silent stale default. The `cepGateThreshold` bug the
// constitution cites as its motivating example lives in exactly this code.
struct TickRuleConfig {
    CameraConfig camera;
    BeaconConfig beacon_left;
    BeaconConfig beacon_right;
    AirframeObstruction airframe;
    // 032 phase 1. NOTE at 040: the shipped value (1.25) is EXACTLY
    // kCepSentinelThreshold, so this is already a VISIBILITY gate in practice
    // and remains one under 040's quality semantics — visible quality spans
    // [0, 1] and never reaches it. That matters because FR-017c forbids
    // suppressing separation for a merely TENTATIVE lock: the blob centroid
    // exists before the code decodes, so a bearing is available pre-lock and the
    // controller is expected to learn the discount rather than be handed a gap.
    // Lowering this below 1.0 would start gating tentative locks and break
    // FR-017c — the resolving limit below is the physically meaningful gate.
    gp_scalar cep_gate_threshold;

    // 040 US4 — the link budget and the acquisition machine.
    SignalConfig signal;
    AcquisitionConfig acquisition;
    // Controller tick length. The acquisition machine advances ANALYTICALLY once
    // per tick rather than sub-stepping the 480 fps camera, so it needs to know
    // how much time one call represents.
    gp_scalar control_interval_ms;

    // 040 T074 — the mount the OBSTRUCTION test uses, which US6 lets drift from
    // the bearing mount by up to ±5 mm. Separate field rather than one shared
    // mount because the split is the whole point (research R6): translation is
    // negligible for bearing and material for propeller clearance.
    gp_vec3 obstruction_mount_offset;
};

// 040 T073-T075 (US6) — bake a scenario's camera draw into the tick config.
//
// ONE function, called once per scenario by BOTH execution paths, so the two
// cannot apply variation differently. Perturbs, in place:
//
//   * mount_orientation_body — boresight yaw/pitch + roll. Roll matters most:
//     it rotates the image plane and so biases the port→starboard tilt cue
//     degree-for-degree, and tilt drives the roll command.
//   * obstruction_mount_offset — mount translation, OBSTRUCTION PATH ONLY
//     (research R6). ±5 mm is 0.03° at 10 m, nothing for bearing, but ~15% of
//     propeller clearance. Bearing keeps the nominal mount.
//   * airframe wing slab thickness — folded foam board is "highly variable".
//
// `ambientScale` is applied to signal.ambient_floor. It ships at 1.0 (emitter
// stays perfect, operator 2026-08-02), so this is inert until the lens+filter
// tests pin SignalAmbientKnee — wired now so switching it on is a sigma change.
void applyCameraVariation(TickRuleConfig& cfg, const CameraDeltas& draw);

// Per-beacon state carried ACROSS ticks within a scenario (FR-020a).
//
// Owned by the caller, not by this module, because the two call sites own their
// own ring and dmp recording already. Reset through resetPerceptionState() at
// every scenario boundary — unreset state leaks between scenarios and breaks the
// bitwise gate.
struct PerceptionCarryState {
    AcquisitionState left;
    AcquisitionState right;

    void reset() {
        left.reset();
        right.reset();
    }
};

// One tick of perception: both beacon observations plus the ring record built
// from them. Returned rather than written through pointers so the caller keeps
// ownership of the ring and of dmp recording, which differ between the two
// call sites.
struct PerceptionTickResult {
    BeaconObservation left;
    BeaconObservation right;
    TrackerObservationRing::Record record;
};

// Project both beacons against the current target sample and build the ring
// record, including the CEP-gated pair separation.
//
// Gate semantics (032 phase 1, preserved verbatim at extraction): if EITHER
// beacon's CEP is at or above the threshold, separation is substituted with
// neutral 0.0 rather than computed from an untrusted endpoint.
PerceptionTickResult projectPerceptionTick(const AircraftState& chase,
                                           const SourceTickSample& target,
                                           const TickRuleConfig& config,
                                           PerceptionCarryState& carry);

// Advance the situational-awareness state from the freshly materialized "now"
// slot. Visibility uses the sentinel threshold, matching
// fitness_decomposition.cc.
//
// Call ONLY on real ticks — never during the history pre-fill, or the blind
// -tick and exit-bearing state would be advanced by scenario setup and break
// determinism.
void advanceSituationalAwareness(const TrackerHistoryWindow& history,
                                 SituationalAwarenessState& sa);

// Reset every piece of carried per-scenario perception state (FR-020a).
//
// This is the single place that knows what "carried perception state" means.
// When the 040 acquisition state machine lands, its reset belongs here — so
// that both execution paths pick it up without either being edited.
void resetPerceptionState(TrackerObservationRing& ring,
                          SituationalAwarenessState& sa,
                          PerceptionCarryState& carry,
                          EnvelopeState& envelope);

}  // namespace autoc::eval

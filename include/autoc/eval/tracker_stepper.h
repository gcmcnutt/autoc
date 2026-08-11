#pragma once

// 030 M6d — TrackerStepper (FR-018 + FR-019).
//
// Tracker-mode per-tick logic. Drives off recorded source-craft trajectory
// samples instead of a generated path. Per source tick:
//   1. Look up target state at simTime t_i from SourceScenarioTrajectory.
//   2. Project both wingtip beacons through the chase camera (M5 projection
//      module) → BeaconObservation × 2.
//   3. Push observations into the 6-slot history ring buffer.
//   4. Build TrackerInputs (48 floats) from history + chase state + home.
//   5. NN forward pass via NNControllerBackend::evaluateTracker.
//   6. Advance chase-craft physics until next source tick.
//
// Symmetric with PathgenStepper (M6a). The strategy-pattern split means
// the worker selects one or the other based on EvalData::mode.
//
// Source trajectory + camera/beacon/home configuration are passed at
// construction. M6d uses compile-time-default CameraConfig / BeaconConfig
// values; M6e wires .ini-driven overrides through EvalData.

#include <vector>

#include <cstdint>
#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"             // FlightArena, checkArenaBounds
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/crash_hull.h"        // M7d.b — CrashHull config + didCrashFire
#include "autoc/eval/derived_features.h"  // 032 — compute_pair_span / compute_tilt
#include "autoc/eval/scenario_stepper.h"
#include "autoc/eval/source_trajectory.h"
#include "autoc/eval/tracker_tick_rule.h"  // 040 — PerceptionCarryState, TickRuleConfig
#include "autoc/nn/evaluator.h"          // NNControllerBackend, TrackerHistoryWindow
#include "autoc/rpc/protocol.h"          // CrashReason, ScenarioMetadata, CameraViewSample, CopiedTargetSample
#include "autoc/types.h"

namespace autoc::eval {

class TrackerStepper : public ScenarioStepper {
public:
    // No parameter defaults per Constitution III + M2-era no-fallback
    // policy: every caller passes every argument explicitly. Call sites:
    // tests/tracker_stepper_init_tests.cc; the crrcsim worker mirrors this
    // contract in CrrcsimTrackerHelper.
    TrackerStepper(NNControllerBackend& nn,
                   AircraftState& state,
                   const SourceScenarioTrajectory& source,
                   const ScenarioMetadata& scenario_meta,
                   const CameraConfig& camera,
                   const BeaconConfig& beacon_left,
                   const BeaconConfig& beacon_right,
                   const AirframeObstruction& airframe,
                   const FlightArena& arena,
                   const CrashHull& crash_hull,
                   gp_scalar p_crash_this_gen,
                   uint32_t prng_seed,
                   gp_scalar trail_distance,
                   gp_scalar cep_gate_threshold);

    // 040 US6 — set the scenario's camera draw. The production path takes this
    // from WorkerInit; the test-only reference takes it directly so the
    // recorded-pose invariant can be asserted.
    void setCameraVariation(const CameraDeltas& d) { camera_variation_ = d; }

    void initScenario() override;
    CrashReason stepOnce() override;
    unsigned long durationMsec() const override { return duration_msec_; }

    // 030 M8b — Per-tick recorded outputs for the v=2 dmp output stream.
    // Populated by projectAndShiftHistory each step; the worker reads
    // after each successful step and push_backs into evalResults's
    // EvalTick::cameraView / EvalTick::targetSample (041 T020 — these were
    // the separate cameraViewList / targetTrajectoryList before grouping).
    const CameraViewSample& lastCameraView() const { return last_camera_view_; }
    const CopiedTargetSample& lastTargetSample() const { return last_target_sample_; }

    // 030 M7d.b — Crash-hull strike count for this scenario. 0 normally;
    // 1 if didCrashFire returned true (scenario terminates on first fire,
    // so the count is bounded). Returned to the worker for hullStrikeCount[i].
    int hullFiredCount() const { return hull_fired_count_; }

private:
    // Project both beacons against the current target sample, push into
    // history, write the most-recent slot. Also populates
    // last_camera_view_ + last_target_sample_ for M8b dmp output.
    void projectAndShiftHistory(const SourceTickSample& target);

    NNControllerBackend& nn_;
    AircraftState& state_;
    const SourceScenarioTrajectory& source_;
    ScenarioMetadata scenario_meta_;

    CameraConfig camera_;
    BeaconConfig beacon_left_;
    BeaconConfig beacon_right_;
    AirframeObstruction airframe_;
    FlightArena arena_;

    // 030 M7d.b — Crash hull (FR-008b). Geometry inside-hull check + per-tick
    // Bernoulli draw against p_crash_this_gen using a per-scenario PRNG
    // seeded at construction time (deterministic across runs). On fire:
    // CrashReason::HullStrike, scenario terminates, hull_fired_count_++.
    CrashHull crash_hull_;
    gp_scalar p_crash_this_gen_;
    uint32_t prng_state_;
    int hull_fired_count_;
    gp_scalar trail_distance_;
    gp_scalar cep_gate_threshold_;

    // 037 T022 — deep per-tick observation ring (depth = deepest R5 lag in
    // ticks + 1). history_ remains the 6-slot gather VIEW, materialized from
    // the ring at the kNNHistoryLagsMsec offsets each tick.
    TrackerObservationRing obs_ring_;
    TrackerHistoryWindow history_;

    // 040 T064 (FR-020a) — per-beacon acquisition state, carried across ticks
    // within a scenario and reset through resetPerceptionState() at every
    // scenario boundary. Mirrors CrrcsimTrackerHelper exactly; the two must
    // reset identically or the test-only reference certifies wrong behaviour.
    PerceptionCarryState perception_carry_;

    // 040 US6 — camera draw for the current scenario. Defaults to the NOMINAL
    // camera; the test-only reference has no ScenarioMetadata, and its job is to
    // pin the RULE, not to replay a bake.
    CameraDeltas camera_variation_{};

    // 038 P0-D FR-P0H (A) — situational-awareness state (time_since_seen +
    // held exit-bearing). Reset in initScenario, advanced each stepOnce from
    // the "now" beacon observation. Shared update rule with CrrcsimTrackerHelper.
    SituationalAwarenessState sa_state_;

    // Source-tick cursor. Each stepOnce consumes source_.samples[cursor_],
    // advances physics until the next sample's simTimeMsec, then increments.
    // initScenario sets cursor_ = 0; chase-init geometry (2026-05-07
    // Session Q1) replaced the original pre-roll warm-start.
    size_t cursor_ = 0;
    unsigned long duration_msec_ = 0;

    // 030 M8b — Per-tick recorded outputs (M2 dmp v=2 schema).
    CameraViewSample last_camera_view_{};
    CopiedTargetSample last_target_sample_{};
};

}  // namespace autoc::eval

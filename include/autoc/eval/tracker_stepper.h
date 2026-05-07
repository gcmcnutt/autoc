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
// the worker (minisim) selects one or the other based on EvalData::mode.
//
// Source trajectory + camera/beacon/home configuration are passed at
// construction. M6d uses compile-time-default CameraConfig / BeaconConfig
// values; M6e wires .ini-driven overrides through EvalData.

#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/scenario_stepper.h"
#include "autoc/eval/source_trajectory.h"
#include "autoc/nn/evaluator.h"          // NNControllerBackend, TrackerHistoryWindow
#include "autoc/rpc/protocol.h"          // CrashReason, ScenarioMetadata
#include "autoc/types.h"

namespace autoc::eval {

class TrackerStepper : public ScenarioStepper {
public:
    TrackerStepper(NNControllerBackend& nn,
                   AircraftState& state,
                   const SourceScenarioTrajectory& source,
                   const ScenarioMetadata& scenario_meta,
                   const CameraConfig& camera = CameraConfig{},
                   const BeaconConfig& beacon_left = BeaconConfig{},
                   const BeaconConfig& beacon_right = BeaconConfig{},
                   const AirframeProxy& airframe = defaultAirframeProxyHB1(),
                   const gp_vec3& home_world = gp_vec3::Zero());

    void initScenario() override;
    CrashReason stepOnce() override;
    unsigned long durationMsec() const override { return duration_msec_; }

private:
    // Project both beacons against the current target sample, push into
    // history, write the most-recent slot.
    void projectAndShiftHistory(const SourceTickSample& target);

    NNControllerBackend& nn_;
    AircraftState& state_;
    const SourceScenarioTrajectory& source_;
    ScenarioMetadata scenario_meta_;

    CameraConfig camera_;
    BeaconConfig beacon_left_;
    BeaconConfig beacon_right_;
    AirframeProxy airframe_;
    gp_vec3 home_world_;

    // 6-slot beacon history per channel. Index 0 is oldest, index 5 is
    // "now". Shift-left on each push (cheap — 6 floats × 6 channels).
    TrackerHistoryWindow history_;

    // Source-tick cursor. Each stepOnce consumes source_.samples[cursor_],
    // advances physics until the next sample's simTimeMsec, then increments.
    size_t cursor_ = 0;
    unsigned long duration_msec_ = 0;
};

}  // namespace autoc::eval

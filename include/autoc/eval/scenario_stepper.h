#pragma once

// 030 M6a — ScenarioStepper strategy interface (FR-019 mode dispatch + R5).
//
// One worker binary supports both pathgen and tracker training modes. The
// split happens at the per-tick stepping level: shared scaffolding (RPC
// receive, NN deserialization, scenario loop, EvalResults recording) lives
// in the worker; per-tick logic (input gathering, fitness target lookup,
// physics-advance cadence) is mode-specific and lives behind this strategy
// interface.
//
// PathgenStepper (M6a) wraps the existing path-following per-tick loop.
// TrackerStepper (M6d) drives off `SourceScenarioTrajectory` samples and
// uses the M5 projection module. Mode is selected at autoc startup (FR-011)
// and conveyed via EvalData (M6c).

#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/rpc/protocol.h"  // CrashReason, ScenarioMetadata

namespace autoc::eval {

class ScenarioStepper {
public:
    virtual ~ScenarioStepper() = default;

    // Initialize per-scenario state — initial aircraft pose, history
    // buffers, rabbit / path-tracking state. Called once before any
    // `stepOnce()`. Implementations write to the worker's `aircraftState`
    // reference (passed at construction).
    virtual void initScenario() = 0;

    // Run one tick: gather NN inputs, forward-pass NN, advance physics,
    // update tracking / time. Returns `CrashReason::None` to continue;
    // any other value terminates the scenario with that reason.
    virtual CrashReason stepOnce() = 0;

    // Total simulated duration for the scenario so far (msec). Used by
    // the worker to populate `AircraftState::simTimeMsec` for the
    // recording buffer.
    virtual unsigned long durationMsec() const = 0;
};

}  // namespace autoc::eval

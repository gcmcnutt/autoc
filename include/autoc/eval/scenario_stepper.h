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

#include <cmath>
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

// 030 M8b — Shared arena out-of-bounds check, used by both PathgenStepper
// and TrackerStepper (per operator routing 2026-05-07: M2 reuses M1's
// proven envelope rather than diverging). Returns `CrashReason::Eval`
// when chase has exited the M1 envelope:
//   - too high: rawZ < SIM_MAX_ELEVATION (-120 m NED, ⇒ alt > 120 m AGL)
//   - too low : rawZ > SIM_MIN_ELEVATION (-7 m NED,   ⇒ alt < 7 m AGL)
//   - too far : sqrt(x² + y²) > SIM_PATH_RADIUS_LIMIT (70 m horizontal)
// Otherwise returns `CrashReason::None`.
//
// Body extracted byte-for-byte from the pre-extraction PathgenStepper
// inline check; `inline` keyword hints the compiler to fold back into
// the call site at -O3 so pathgen-mode bitwise regression survives.
// Verified post-extraction via rebuild-perf.sh + autoc-eval bitwise on
// gen9200.dmp baseline.
//
// FR-016 supersedes this with a config-driven arena.h at M7 (cylinder
// radius / floor AGL / ceiling AGL from autoc-tracker.ini, plus
// per-scenario egress-kind telemetry counters). For now both modes use
// the same hardcoded M1 envelope.
inline CrashReason checkAircraftOOB(const AircraftState& state) {
    gp_vec3 rawForOOB =
        state.getPosition() + gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
    gp_scalar distanceFromOrigin =
        std::sqrt(rawForOOB[0] * rawForOOB[0] + rawForOOB[1] * rawForOOB[1]);
    if (rawForOOB[2] < (SIM_MAX_ELEVATION) ||
        rawForOOB[2] > (SIM_MIN_ELEVATION) ||
        distanceFromOrigin > SIM_PATH_RADIUS_LIMIT) {
        return CrashReason::Eval;
    }
    return CrashReason::None;
}

}  // namespace autoc::eval

#pragma once

// 030 M3 — Source M1 dmp loader (FR-001).
//
// In-memory representation of source-scenario target trajectories loaded
// from a prior pathgen-mode training run's S3 dmp. Per FR-001 + R1 + R8:
// 030 v1 does NOT introduce a new on-disk file format — the source dmp's
// EvalResults IS the trajectory bundle, loaded once at autoc startup and
// indexed per scenario.
//
// See data-model.md §1 + plan.md M3.

#include <cstdint>
#include <vector>

#include "autoc/types.h"
#include "autoc/rpc/protocol.h"  // ScenarioMetadata

// Minimum tick count for a usable source scenario (3 seconds at 10 Hz NN
// tick). Source scenarios shorter than this are typically truncated by
// terminal crashes and contribute too few samples for meaningful tracker-
// mode evaluation. Per data-model.md §1 validation rule.
constexpr int MIN_SCENARIO_TICKS = 30;

// Per-tick sample extracted from the source dmp's aircraftStateList.
// Carries only what the M2 tracker-mode loop actually needs: target pose +
// kinematics. Excludes NN-input/output state (irrelevant — the source NN's
// behavior is provenance-only per spec D13).
//
// NOTE — angularRate intentionally absent: the v1 dmp schema
// (CEREAL_CLASS_VERSION(AircraftState, 1)) does NOT serialize gyroRates_;
// it's a transient field set per-tick from physics but not persisted
// across cereal round-trips. M5+ analytics that actually need target body
// rates can numerically differentiate orientation across consecutive
// samples; an explicit per-tick gyroRates serialization would land at the
// M8 schema-bump-to-v2 boundary.
struct SourceTickSample {
    double simTimeMsec;       // M1 source timestamp (10 Hz nominal)
    gp_vec3 position;         // target world position (NED, meters)
    gp_quat orientation;      // target body→world quat
    gp_vec3 velocity;         // target velocity (NED, m/s)
};

// One source scenario worth of target trajectory + the joint-PRNG variation
// parameters that conditioned its recording. Per FR-001 + spec D13: each
// tracker-mode scenario inherits its source-scenario's wind/entry/craft
// variation so the trainee craft sees the same environmental conditions
// the target flew through.
struct SourceScenarioTrajectory {
    int sourceScenarioIndex;            // index into source dmp's scenarioList
    ScenarioMetadata variation;         // copied from source dmp scenarioList[i]
    std::vector<SourceTickSample> samples;  // copied from aircraftStateList[i]
};

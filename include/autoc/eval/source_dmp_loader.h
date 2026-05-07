#pragma once

// 030 M3 — Source M1 dmp loader (FR-001).
//
// Loads a prior pathgen-mode training run's S3 dmp directly into autoc
// memory at startup; per-scenario target trajectories are then distributed
// to workers analogous to how pathgen distributes path geometry today.
// No separate playback-library file format — the source dmp IS the library
// (revision 2026-05-04). See plan.md M3 + data-model.md §1.

#include <string>
#include <vector>

#include "autoc/eval/source_trajectory.h"
#include "autoc/rpc/protocol.h"  // CrashReason

// Load a source M1 dmp from a path-or-S3-key.
//
// Path resolution:
//   1. If `path_or_key` exists as a local file (std::filesystem::exists),
//      load it as a local cereal binary archive. Used for offline-test
//      fixtures and operator dev iteration.
//   2. Otherwise, treat `path_or_key` as an S3 key relative to the bucket
//      configured in `autoc.ini` (S3Bucket / S3Profile per src/util/
//      config.cc). Production tracker-mode runs always go through this
//      path; the FR-011 canonical form is `autoc-storage/<run-id>/gen<N>.dmp`.
//
// Throws std::runtime_error on:
//   - cereal version mismatch (loud-fail per Constitution V + FR-015a)
//   - missing / unreadable / unparseable dmp
//
// Each scenario's per-tick AircraftState is converted into a
// SourceTickSample carrying simTimeMsec + position + orientation +
// velocity + gyroRates. ScenarioMetadata is copied verbatim so the joint-
// PRNG variation parameters that conditioned the source recording flow
// forward to the M2 tracker-mode run (FR-001 + FR-010).
//
// Validation gates per data-model.md §1:
//   - non-empty samples (>= MIN_SCENARIO_TICKS) — short scenarios still
//     load but downstream filterCrashedSourceScenarios drops the truncated-
//     by-crash ones; the count gate is enforced in the test contract
//     (per-scenario, not at load).
//   - simTimeMsec monotonic (asserted by contract test).
//   - quat magnitude in [0.99, 1.01] (sanity check).
//   - position bounded < 10 km from origin (sanity check).
std::vector<SourceScenarioTrajectory> loadSourceDmp(
    const std::string& path_or_key);

// Filter out source scenarios where the source aircraft terminally
// crashed mid-scenario (CrashReason::Boot/Sim/Eval per protocol.h
// isCrash()). Per FR-013 v1 default — skip rather than truncate.
//
// `crashReasons` parallels `trajectories`: crashReasons[i] applies to
// the source scenario at the same index. Source-side TimeLimit and
// RabbitComplete are NOT crashes — those scenarios are preserved.
//
// Returns a new vector with crashed entries omitted; original vector
// is not modified.
std::vector<SourceScenarioTrajectory> filterCrashedSourceScenarios(
    const std::vector<SourceScenarioTrajectory>& trajectories,
    const std::vector<CrashReason>& crashReasons);

// Cross-product subset filter per FR-011 (autoc-tracker.ini
// TrackerPathSubset × TrackerWindSubset). Keeps scenarios where
// variation.pathVariantIndex ∈ pathSubset AND
// variation.windVariantIndex ∈ windSubset.
//
// An empty subset on either axis means "all" — i.e., no filter on that
// axis. Both empty = identity. Default smoke slice
// (pathSubset = {0..5}, windSubset = {0..19}) yields 120 scenarios from
// the 245-294-scenario source dmps.
std::vector<SourceScenarioTrajectory> filterByScenarioIndex(
    const std::vector<SourceScenarioTrajectory>& trajectories,
    const std::vector<int>& pathSubset,
    const std::vector<int>& windSubset);

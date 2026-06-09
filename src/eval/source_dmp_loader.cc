// 030 M3 — Source M1 dmp loader (FR-001).
//
// Implements loadSourceDmp() + filterCrashedSourceScenarios() +
// filterByScenarioIndex() per data-model.md §1. See header for contract.

#include "autoc/eval/source_dmp_loader.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cereal/archives/binary.hpp>

#include <aws/s3/S3Client.h>

#include "autoc/util/s3_run_selector.h"  // 035 FR-P09 — shared S3 dmp I/O

#include "autoc/eval/aircraft_state.h"
#include "autoc/rpc/protocol.h"
#include "autoc/util/config.h"

namespace {

// Pull cereal binary into an EvalResults from any std::istream.
EvalResults readEvalResultsFromStream(std::istream& in) {
    EvalResults results;
    cereal::BinaryInputArchive ia(in);
    ia(results);
    return results;
}

// Local-file load path — used for offline-deterministic test fixtures
// and operator dev iteration. Cereal version mismatch surfaces as a
// std::exception from cereal; we wrap it in std::runtime_error for the
// loud-fail contract.
EvalResults loadFromLocalFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error(
            "loadSourceDmp: could not open local file: " + path);
    }
    try {
        return readEvalResultsFromStream(in);
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "loadSourceDmp: cereal load failed for local file " + path +
            ": " + e.what() +
            "\n  (Constitution V: a version-mismatch error is the prompt for an"
            " explicit operator decision — re-run training to produce an"
            " upgraded artifact, or port the migration path forward.)");
    }
}

// S3 load path — production. Pattern follows tools/nnextractor.cc:165-191.
// Caller must have already invoked Aws::InitAPI() and ConfigManager::initialize().
EvalResults loadFromS3(const std::string& s3_key) {
    auto s3_client = ConfigManager::getS3Client();
    if (!s3_client) {
        throw std::runtime_error(
            "loadSourceDmp: no S3 client available. ConfigManager not"
            " initialized? Aws::InitAPI() not called?");
    }
    // 035: source reads from trackerSourceBucket (e.g. autoc-m1), NOT s3Bucket
    // (autoc-m2 = M2 output). Validated non-empty in tracker mode by ConfigManager.
    const std::string bucket = ConfigManager::getConfig().trackerSourceBucket;

    // FR-P09 — fetch + inflate via the shared S3 dmp I/O (fail-loud on S3 error).
    std::string body = autoc::s3GetDmpBlob(*s3_client, bucket, s3_key);

    try {
        std::istringstream iss(body, std::ios::binary);
        return readEvalResultsFromStream(iss);
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "loadSourceDmp: cereal load failed for s3://" + bucket + "/" +
            s3_key + ": " + e.what() +
            "\n  (Constitution V loud-fail: see local-file equivalent for the"
            " operator-decision prompt.)");
    }
}

// Convert per-tick AircraftState → SourceTickSample. Extracts only the
// pose + kinematic fields the M2 tracker-mode loop needs; NN inputs /
// outputs / fitness state are intentionally dropped (those are the source
// run's provenance, not load-bearing for tracker training per spec D13).
// gyroRates_ is intentionally NOT extracted — see source_trajectory.h note.
SourceTickSample fromAircraftState(const AircraftState& s) {
    SourceTickSample t;
    t.simTimeMsec = static_cast<double>(s.getSimTimeMsec());
    t.position = s.getPosition();
    t.orientation = s.getOrientation();
    t.velocity = s.getVelocity();
    return t;
}

bool contains(const std::vector<int>& xs, int v) {
    return std::find(xs.begin(), xs.end(), v) != xs.end();
}

}  // namespace

EvalResults loadEvalResultsDmp(const std::string& path_or_key) {
    // Path resolution: prefer local file if it exists (offline-test path);
    // otherwise treat as S3 key (production path per FR-011).
    return std::filesystem::exists(path_or_key)
               ? loadFromLocalFile(path_or_key)
               : loadFromS3(path_or_key);
}

std::vector<SourceScenarioTrajectory> loadSourceDmp(
    const std::string& path_or_key) {

    EvalResults source = loadEvalResultsDmp(path_or_key);

    const size_t scenarioCount = source.aircraftStateList.size();
    if (scenarioCount == 0) {
        throw std::runtime_error(
            "loadSourceDmp: source dmp has empty aircraftStateList — no"
            " scenarios to load. path_or_key=" + path_or_key);
    }
    if (source.scenarioList.size() != scenarioCount) {
        throw std::runtime_error(
            "loadSourceDmp: source dmp inconsistent — aircraftStateList has " +
            std::to_string(scenarioCount) +
            " entries but scenarioList has " +
            std::to_string(source.scenarioList.size()));
    }

    std::vector<SourceScenarioTrajectory> out;
    out.reserve(scenarioCount);
    for (size_t i = 0; i < scenarioCount; ++i) {
        SourceScenarioTrajectory traj;
        traj.sourceScenarioIndex = static_cast<int>(i);
        traj.variation = source.scenarioList[i];
        traj.samples.reserve(source.aircraftStateList[i].size());
        for (const auto& s : source.aircraftStateList[i]) {
            traj.samples.push_back(fromAircraftState(s));
        }
        out.push_back(std::move(traj));
    }
    return out;
}

std::vector<SourceScenarioTrajectory> filterCrashedSourceScenarios(
    const std::vector<SourceScenarioTrajectory>& trajectories,
    const std::vector<CrashReason>& crashReasons) {

    // Tolerate a missing/short crashReasonList (older dmps, partial
    // captures) — index out-of-range is treated as "not crashed", matching
    // the optimistic interpretation: only filter what we can prove crashed.
    std::vector<SourceScenarioTrajectory> kept;
    kept.reserve(trajectories.size());
    for (const auto& traj : trajectories) {
        const int idx = traj.sourceScenarioIndex;
        const bool indexInRange =
            idx >= 0 && static_cast<size_t>(idx) < crashReasons.size();
        if (indexInRange && isCrash(crashReasons[idx])) {
            continue;  // skip Boot/Sim/Eval-crashed source scenarios
        }
        kept.push_back(traj);
    }
    return kept;
}

std::vector<SourceScenarioTrajectory> filterByScenarioIndex(
    const std::vector<SourceScenarioTrajectory>& trajectories,
    const std::vector<int>& pathSubset,
    const std::vector<int>& windSubset) {

    const bool allPaths = pathSubset.empty();
    const bool allWinds = windSubset.empty();
    if (allPaths && allWinds) return trajectories;  // identity case

    std::vector<SourceScenarioTrajectory> kept;
    kept.reserve(trajectories.size());
    for (const auto& traj : trajectories) {
        const int p = traj.variation.pathVariantIndex;
        const int w = traj.variation.windVariantIndex;
        const bool pathOK = allPaths || contains(pathSubset, p);
        const bool windOK = allWinds || contains(windSubset, w);
        if (pathOK && windOK) kept.push_back(traj);
    }
    return kept;
}

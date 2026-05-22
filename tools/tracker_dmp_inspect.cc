// 030 M8b — Tracker-mode dmp (M2, schema v=2) inspector.
//
// Loads a tracker-mode output dmp from S3 key or local file, prints
// per-scenario summary covering both legacy fields (aircraftStateList,
// crashReasonList) and the v=2 additions (cameraViewList,
// targetTrajectoryList, arenaEgressCount, hullStrikeCount). Operator
// sanity-check before M9 renderer or per-tick analytics tools land.
//
// Sibling to source_dmp_inspect (M3): same loader infrastructure, but
// reports the full EvalResults rather than the SourceScenarioTrajectory
// projection. Use this on dmps autoc writes; use source_dmp_inspect on
// pastonly3-style dmps you intend to feed back as tracker-mode source.
//
// Usage:
//   build/tracker_dmp_inspect <path_or_s3_key> [-i autoc.ini]
//
// Convention: `-i <ini>` is the project-wide config-file flag (matches
// `autoc -i …` and source_dmp_inspect). `--config` stays as a long-form
// alias.
//
// Examples:
//   build/tracker_dmp_inspect <030-run-id>/gen0.dmp
//   build/tracker_dmp_inspect /tmp/local-test-output.dmp -i autoc-tracker.ini

#include <getopt.h>

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>

#include <aws/core/Aws.h>

#include "autoc/eval/camera_projection.h"  // BeaconObservation, kCepSentinelFloat
#include "autoc/eval/source_dmp_loader.h"  // loadEvalResultsDmp
#include "autoc/util/config.h"

namespace {

void printUsage(const char* prog) {
    std::cerr <<
        "Usage: " << prog << " <path_or_s3_key> [-i autoc.ini]\n"
        "  Loads a tracker-mode (v=2) M2 dmp and prints scenario-summary\n"
        "  stats covering both legacy fields and the v=2 additions\n"
        "  (cameraViewList, targetTrajectoryList, arenaEgressCount,\n"
        "  hullStrikeCount). Pathgen-mode dmps work too — v=2 fields\n"
        "  display empty per the back-compat contract.\n"
        "  -i <ini>      Config file (default autoc.ini). Provides S3\n"
        "                bucket + profile. `--config` is a long-form alias.\n";
}

const char* crashReasonName(CrashReason r) {
    switch (r) {
        case CrashReason::None:           return "None";
        case CrashReason::Boot:           return "Boot";
        case CrashReason::Sim:            return "Sim";
        case CrashReason::Eval:           return "Eval";
        case CrashReason::TimeLimit:      return "TimeLimit";
        case CrashReason::RabbitComplete: return "RabbitComplete";
        case CrashReason::HullStrike:     return "HullStrike";
    }
    return "?";
}

void printBeacon(const char* label, const autoc::eval::BeaconObservation& b) {
    const bool sentinel = (b.cep >= autoc::eval::kCepSentinelThreshold);
    std::cout << "    " << std::left << std::setw(8) << label;
    if (sentinel) {
        std::cout << "INVISIBLE (cep=" << std::fixed << std::setprecision(2)
                  << b.cep << ", raw_cep_int8=" << static_cast<int>(b.raw_cep_int8) << ")";
    } else {
        std::cout << std::fixed << std::setprecision(3)
                  << "x=" << std::setw(7) << b.screen_x
                  << " y=" << std::setw(7) << b.screen_y
                  << " cep=" << std::setw(5) << b.cep
                  << "  raw=("
                  << static_cast<int>(b.raw_x_int8) << ", "
                  << static_cast<int>(b.raw_y_int8) << ", "
                  << static_cast<int>(b.raw_cep_int8) << ")";
    }
    std::cout << "\n";
}

void printCameraView(const char* label, const CameraViewSample& v) {
    std::cout << "  " << label << " camera-pose: pos=(" << std::fixed
              << std::setprecision(2)
              << v.camera_pose_world_pos.x() << ", "
              << v.camera_pose_world_pos.y() << ", "
              << v.camera_pose_world_pos.z() << ")  fov="
              << std::setprecision(1) << v.camera_fov_h_deg << "°h × "
              << v.camera_fov_v_deg << "°v\n";
    printBeacon("left",  v.beacon_left);
    printBeacon("right", v.beacon_right);
}

void printTarget(const char* label, const CopiedTargetSample& t) {
    std::cout << "  " << label << " target: pos=("
              << std::fixed << std::setprecision(2)
              << t.position.x() << ", " << t.position.y() << ", " << t.position.z()
              << ")  vel=("
              << t.velocity.x() << ", " << t.velocity.y() << ", " << t.velocity.z()
              << ")  rabbit=("
              << t.trail_rabbit_position.x() << ", "
              << t.trail_rabbit_position.y() << ", "
              << t.trail_rabbit_position.z() << ")"
              << "  hull=" << (t.inside_crash_hull ? "INSIDE" : "outside") << "\n";
}

// 032 phase 1 — print the 9 derived perceptual feature slots from the per-tick
// TrackerInputs (serialized into AircraftState at v=2). Useful for the
// minisim/crrcsim smoke verification: confirm slots 45..53 carry non-trivial
// values across ticks before kicking off the production bake.
void printDerivedFeatures(const char* label, const TrackerInputs& inputs) {
    std::cout << "  " << label << " derived: span[6]=[";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(4) << inputs.beacon_pair_span[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]  span_rate=" << std::fixed << std::setprecision(4) << inputs.span_rate
              << "  tilt=(sin " << std::fixed << std::setprecision(4) << inputs.target_tilt_sin
              << ", cos " << std::fixed << std::setprecision(4) << inputs.target_tilt_cos
              << ")\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string pathOrKey;
    std::string configFile = "autoc.ini";

    static struct option long_options[] = {
        {"config", required_argument, 0, 'i'},
        {"help",   no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int c;
    while ((c = getopt_long(argc, argv, "i:h", long_options, &option_index)) != -1) {
        switch (c) {
            case 'i': configFile = optarg; break;
            case 'h': printUsage(argv[0]); return 0;
            case '?': printUsage(argv[0]); return 1;
            default: break;
        }
    }
    if (optind < argc) {
        pathOrKey = argv[optind];
    }
    if (pathOrKey.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    ConfigManager::initialize(configFile);
    Aws::SDKOptions options;
    Aws::InitAPI(options);

    int exitCode = 0;
    try {
        std::cout << "Loading: " << pathOrKey << "\n";
        EvalResults r = loadEvalResultsDmp(pathOrKey);

        // Top-level stats.
        std::cout << "EvalResults metadata:\n"
                  << "  workerId=" << r.workerId
                  << "  workerPid=" << r.workerPid
                  << "  workerEvalCounter=" << r.workerEvalCounter << "\n"
                  << "  gpHash=0x" << std::hex << std::setw(16)
                  << std::setfill('0') << r.gpHash << std::dec
                  << std::setfill(' ') << "  gpBytes=" << r.gp.size() << "\n";

        const size_t scenarioCount = r.aircraftStateList.size();
        std::cout << "Scenarios: " << scenarioCount << "\n";
        if (scenarioCount == 0) {
            std::cout << "  (empty — nothing to inspect)\n";
            Aws::ShutdownAPI(options);
            return 0;
        }

        // Per-scenario tick counts + crash reasons.
        std::vector<size_t> tickCounts;
        for (const auto& states : r.aircraftStateList) tickCounts.push_back(states.size());
        const size_t totalT = std::accumulate(tickCounts.begin(), tickCounts.end(), size_t{0});
        const size_t minT = *std::min_element(tickCounts.begin(), tickCounts.end());
        const size_t maxT = *std::max_element(tickCounts.begin(), tickCounts.end());
        const double meanT = static_cast<double>(totalT) / scenarioCount;
        std::cout << "Ticks/scenario: min=" << minT
                  << " mean=" << std::fixed << std::setprecision(1) << meanT
                  << " max=" << maxT << "\n";

        // 033 §2.A — per-scenario master-derived seed for replay. Each
        // scenarioSeed is sufficient to reconstruct all 5 class sub-PRNGs
        // (wind/rabbit/entry/craft/camera) deterministically via the chain
        // in include/autoc/util/scenario_prng.h. Hex output so operator can
        // feed a specific seed back into eval-mode replay.
        if (!r.scenarioList.empty()) {
            const size_t numToShow = std::min<size_t>(r.scenarioList.size(), 5);
            std::cout << "scenarioSeed (033 §2.A) per scenario (showing first "
                      << numToShow << " of " << r.scenarioList.size() << "):\n";
            for (size_t k = 0; k < numToShow; ++k) {
                std::cout << "  [" << std::setw(3) << k << "] 0x" << std::hex
                          << std::setw(16) << std::setfill('0')
                          << r.scenarioList[k].scenarioSeed
                          << std::dec << std::setfill(' ') << "\n";
            }
            if (r.scenarioList.size() > numToShow) {
                std::cout << "  ... (" << (r.scenarioList.size() - numToShow)
                          << " more)\n";
            }
        } else {
            std::cout << "scenarioList: empty (pre-033 dmp would fail-loud "
                         "on cereal length mismatch before this point)\n";
        }

        // v=2 schema-additions presence + parallel-indexing audit.
        const bool hasCameraView = !r.cameraViewList.empty();
        const bool hasTargetTraj = !r.targetTrajectoryList.empty();
        std::cout << "v=2 fields:\n"
                  << "  cameraViewList: " << (hasCameraView ? "POPULATED" : "empty (pathgen-mode dmp or pre-M8 tracker-mode)") << "\n"
                  << "  targetTrajectoryList: " << (hasTargetTraj ? "POPULATED" : "empty") << "\n"
                  << "  arenaEgressCount.size(): " << r.arenaEgressCount.size() << "\n"
                  << "  hullStrikeCount.size(): " << r.hullStrikeCount.size() << "\n";

        if (hasCameraView && r.cameraViewList.size() != scenarioCount) {
            std::cout << "  WARNING: cameraViewList.size()=" << r.cameraViewList.size()
                      << " does not match scenarioCount=" << scenarioCount << "\n";
        }

        // Per-scenario summary (cap at 10 lines for compactness).
        std::cout << "\nPer-scenario summary:\n"
                  << "  idx  ticks  crash         cameraViewTicks  targetTrajTicks  arena  hull\n";
        const size_t showN = std::min(scenarioCount, size_t{10});
        for (size_t i = 0; i < showN; ++i) {
            const size_t cvN = (i < r.cameraViewList.size()) ? r.cameraViewList[i].size() : 0;
            const size_t ttN = (i < r.targetTrajectoryList.size()) ? r.targetTrajectoryList[i].size() : 0;
            const int arena = (i < r.arenaEgressCount.size()) ? r.arenaEgressCount[i] : 0;
            const int hull = (i < r.hullStrikeCount.size()) ? r.hullStrikeCount[i] : 0;
            const CrashReason cr =
                (i < r.crashReasonList.size()) ? r.crashReasonList[i] : CrashReason::None;
            std::cout << "  " << std::setw(3) << i
                      << "  " << std::setw(5) << r.aircraftStateList[i].size()
                      << "  " << std::left << std::setw(14) << crashReasonName(cr) << std::right
                      << std::setw(15) << cvN
                      << std::setw(17) << ttN
                      << std::setw(7) << arena
                      << std::setw(6) << hull << "\n";
        }
        if (scenarioCount > showN) {
            std::cout << "  (... " << (scenarioCount - showN)
                      << " more scenarios omitted)\n";
        }

        // v=2 sample dump from scenario 0 — first / middle / last tick of
        // both cameraView and target trajectory streams.
        if (hasCameraView && !r.cameraViewList[0].empty()) {
            const auto& cv0 = r.cameraViewList[0];
            std::cout << "\nScenario 0 cameraView samples (" << cv0.size() << " ticks):\n";
            printCameraView("first ", cv0.front());
            if (cv0.size() >= 3) printCameraView("middle", cv0[cv0.size() / 2]);
            printCameraView("last  ", cv0.back());
        }

        if (hasTargetTraj && !r.targetTrajectoryList[0].empty()) {
            const auto& tt0 = r.targetTrajectoryList[0];
            std::cout << "\nScenario 0 target trajectory samples (" << tt0.size() << " ticks):\n";
            printTarget("first ", tt0.front());
            if (tt0.size() >= 3) printTarget("middle", tt0[tt0.size() / 2]);
            printTarget("last  ", tt0.back());
        }

        // 032 phase 1 — per-tick derived perceptual features (slots 45..53
        // of TrackerInputs). Serialized into AircraftState at dmp v=2.
        // Smoke verification: confirm non-trivial values across ticks.
        if (!r.aircraftStateList.empty() && !r.aircraftStateList[0].empty()) {
            const auto& as0 = r.aircraftStateList[0];
            std::cout << "\nScenario 0 derived perceptual features (032 phase 1 — slots 45..53):\n";
            printDerivedFeatures("first ", as0.front().getTrackerInputs());
            if (as0.size() >= 3) printDerivedFeatures("middle", as0[as0.size() / 2].getTrackerInputs());
            printDerivedFeatures("last  ", as0.back().getTrackerInputs());
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        exitCode = 1;
    }

    Aws::ShutdownAPI(options);
    return exitCode;
}

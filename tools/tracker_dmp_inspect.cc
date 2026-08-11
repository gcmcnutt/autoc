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
        "  (per-tick cameraView / targetSample, arenaEgressCount,\n"
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
        // 040 T031 — bearings in radians (was ±1 NDC); raw is the sensor-grid
        // pixel index the angle quantised to (was the retired int8 encoding).
        std::cout << std::fixed << std::setprecision(4)
                  << "x=" << std::setw(8) << b.bearing_x_rad << " rad"
                  << " y=" << std::setw(8) << b.bearing_y_rad << " rad"
                  << " cep=" << std::setw(5) << b.cep
                  << "  px=("
                  << b.raw_px_x << ", " << b.raw_px_y << ")"
                  << " raw_cep_int8=" << static_cast<int>(b.raw_cep_int8);
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
// crrcsim smoke verification: confirm slots 45..53 carry non-trivial
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

        const size_t scenarioCount = r.tickList.size();
        std::cout << "Scenarios: " << scenarioCount << "\n";
        if (scenarioCount == 0) {
            std::cout << "  (empty — nothing to inspect)\n";
            Aws::ShutdownAPI(options);
            return 0;
        }

        // Per-scenario tick counts + crash reasons.
        std::vector<size_t> tickCounts;
        for (const auto& st : r.tickList) tickCounts.push_back(st.ticks.size());
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

        // 041 T023 — the PARALLEL-INDEXING AUDIT THAT USED TO LIVE HERE IS GONE,
        // because what it audited is now unrepresentable: camera view and target
        // sample ride inside each EvalTick, so they cannot differ in length
        // from the states or start at a different tick. The old
        // "cameraViewList.size() != scenarioCount" warning had no failure mode
        // left to detect.
        //
        // What IS still worth reporting is per-tick PRESENCE: tracker members
        // are std::optional, and a scenario where only some ticks carry a view
        // means the recorder dropped samples — a real fault the old shape could
        // not distinguish from a short list.
        size_t withCam = 0, withTgt = 0, totalTicks = 0;
        for (const auto& st : r.tickList) {
            for (const auto& tk : st.ticks) {
                ++totalTicks;
                if (tk.cameraView.has_value()) ++withCam;
                if (tk.targetSample.has_value()) ++withTgt;
            }
        }
        const bool hasCameraView = withCam > 0;
        const bool hasTargetTraj = withTgt > 0;
        std::cout << "tracker per-tick members:\n"
                  << "  ticks with cameraView:   " << withCam << " / " << totalTicks
                  << (hasCameraView ? "" : "   (pathgen-mode dmp)") << "\n"
                  << "  ticks with targetSample: " << withTgt << " / " << totalTicks << "\n"
                  << "  arenaEgressCount.size(): " << r.arenaEgressCount.size() << "\n"
                  << "  hullStrikeCount.size(): " << r.hullStrikeCount.size() << "\n";
        if (hasCameraView && withCam != totalTicks) {
            std::cout << "  WARNING: " << (totalTicks - withCam)
                      << " tick(s) carry no cameraView in a tracker dmp — the"
                         " recorder dropped samples.\n";
        }

        // Per-scenario summary (cap at 10 lines for compactness).
        std::cout << "\nPer-scenario summary:\n"
                  << "  idx  ticks  crash         cameraViewTicks  targetTrajTicks  arena  hull\n";
        const size_t showN = std::min(scenarioCount, size_t{10});
        for (size_t i = 0; i < showN; ++i) {
            size_t cvN = 0, ttN = 0;
            for (const auto& tk : r.tickList[i].ticks) {
                if (tk.cameraView.has_value()) ++cvN;
                if (tk.targetSample.has_value()) ++ttN;
            }
            const int arena = (i < r.arenaEgressCount.size()) ? r.arenaEgressCount[i] : 0;
            const int hull = (i < r.hullStrikeCount.size()) ? r.hullStrikeCount[i] : 0;
            const CrashReason cr =
                (i < r.crashReasonList.size()) ? r.crashReasonList[i] : CrashReason::None;
            std::cout << "  " << std::setw(3) << i
                      << "  " << std::setw(5) << r.tickList[i].ticks.size()
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
        const auto& t0 = r.tickList[0].ticks;
        if (hasCameraView && !t0.empty() && t0.front().cameraView.has_value()) {
            std::cout << "\nScenario 0 cameraView samples (" << t0.size() << " ticks):\n";
            printCameraView("first ", *t0.front().cameraView);
            if (t0.size() >= 3 && t0[t0.size() / 2].cameraView.has_value())
                printCameraView("middle", *t0[t0.size() / 2].cameraView);
            if (t0.back().cameraView.has_value())
                printCameraView("last  ", *t0.back().cameraView);
        }

        if (hasTargetTraj && !t0.empty() && t0.front().targetSample.has_value()) {
            std::cout << "\nScenario 0 target trajectory samples (" << t0.size() << " ticks):\n";
            printTarget("first ", *t0.front().targetSample);
            if (t0.size() >= 3 && t0[t0.size() / 2].targetSample.has_value())
                printTarget("middle", *t0[t0.size() / 2].targetSample);
            if (t0.back().targetSample.has_value())
                printTarget("last  ", *t0.back().targetSample);
        }

        // 032 phase 1 — per-tick derived perceptual features (slots 45..53
        // of TrackerInputs). Serialized into AircraftState at dmp v=2.
        // Smoke verification: confirm non-trivial values across ticks.
        if (!t0.empty()) {
            std::cout << "\nScenario 0 derived perceptual features (032 phase 1 — slots 45..53):\n";
            printDerivedFeatures("first ", t0.front().state.getTrackerInputs());
            if (t0.size() >= 3)
                printDerivedFeatures("middle", t0[t0.size() / 2].state.getTrackerInputs());
            printDerivedFeatures("last  ", t0.back().state.getTrackerInputs());
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        exitCode = 1;
    }

    Aws::ShutdownAPI(options);
    return exitCode;
}

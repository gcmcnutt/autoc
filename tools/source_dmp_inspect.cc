// 030 M3b — Source M1 dmp inspector (T024).
//
// Loads a source dmp by S3 key (production canonical form) or local file
// path (offline-test convenience) via loadSourceDmp(), then prints
// per-scenario summary stats: scenario count, mean tick count, joint-PRNG
// variation params, sample target-craft pose at first / middle / last tick
// of scenario 0. Operator sanity-check before any tracker-mode run.
//
// Usage:
//   build/source_dmp_inspect <path_or_s3_key> [-i autoc.ini]
//
// Convention: `-i <ini>` is the project-wide config-file flag (matches
// `autoc -i …` and the other tools). `--config` stays as a long-form
// alias for back-compat but `-i` is the canonical surface in docs and
// new help text.
//
// Examples:
//   build/source_dmp_inspect <run-id>/gen9999.dmp
//   build/source_dmp_inspect tests/fixtures/local.dmp
//   build/source_dmp_inspect <key> -i autoc-tracker.ini

#include <getopt.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <map>
#include <numeric>
#include <string>

#include <aws/core/Aws.h>

#include "autoc/eval/source_dmp_loader.h"
#include "autoc/util/config.h"

namespace {

void printUsage(const char* prog) {
    std::cerr <<
        "Usage: " << prog << " <path_or_s3_key> [-i autoc.ini]\n"
        "  Loads a source M1 dmp and prints scenario-summary stats.\n"
        "  S3 key form (canonical): <run-id>/gen<N>.dmp (bucket from .ini)\n"
        "  Local-file form (offline-test): any path that exists on disk.\n"
        "  -i <ini>      Config file (default autoc.ini). `--config` is\n"
        "                kept as a long-form alias.\n";
}

// Print per-tick sample on a single line — keeps output compact for CLI scan.
void printTick(const char* label, const SourceTickSample& s) {
    std::cout << "  " << std::left << std::setw(10) << label
              << " t=" << std::right << std::setw(8) << std::fixed
              << std::setprecision(1) << s.simTimeMsec << " ms"
              << "  pos=(" << std::setw(7) << std::setprecision(2) << s.position.x()
              << ", " << std::setw(7) << s.position.y()
              << ", " << std::setw(7) << s.position.z() << ")"
              << "  vel=(" << std::setw(6) << s.velocity.x()
              << ", " << std::setw(6) << s.velocity.y()
              << ", " << std::setw(6) << s.velocity.z() << ")"
              << "  |q|=" << std::setprecision(4) << s.orientation.norm()
              << "\n";
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
        auto trajectories = loadSourceDmp(pathOrKey);

        std::cout << "Scenarios: " << trajectories.size() << "\n";
        if (trajectories.empty()) {
            std::cout << "  (empty — nothing to inspect)\n";
            Aws::ShutdownAPI(options);
            return 0;
        }

        // Per-scenario tick counts and mean.
        std::vector<size_t> tickCounts;
        tickCounts.reserve(trajectories.size());
        for (const auto& traj : trajectories) tickCounts.push_back(traj.samples.size());
        const size_t total =
            std::accumulate(tickCounts.begin(), tickCounts.end(), size_t{0});
        const size_t minT = *std::min_element(tickCounts.begin(), tickCounts.end());
        const size_t maxT = *std::max_element(tickCounts.begin(), tickCounts.end());
        const double meanT = static_cast<double>(total) / trajectories.size();
        std::cout << "Ticks/scenario: min=" << minT
                  << " mean=" << std::fixed << std::setprecision(1) << meanT
                  << " max=" << maxT << "\n";

        // Path × wind variation distribution (033: per-scenario master-derived
        // scenarioSeed replaces the pre-033 joint-PRNG windSeed).
        std::cout << "Variation axes (033 per-scenario master-derived seeds):\n";
        for (size_t i = 0; i < std::min(trajectories.size(), size_t{5}); ++i) {
            const auto& v = trajectories[i].variation;
            std::cout << "  scenario " << std::setw(3) << i
                      << ": pathVar=" << std::setw(2) << v.pathVariantIndex
                      << " windVar=" << std::setw(2) << v.windVariantIndex
                      << " scenarioSeed=0x" << std::hex << std::setw(16)
                      << std::setfill('0') << v.scenarioSeed
                      << std::dec << std::setfill(' ') << "\n";
        }
        if (trajectories.size() > 5) {
            std::cout << "  (... " << (trajectories.size() - 5)
                      << " more scenarios omitted)\n";
        }

        // Sample target-craft pose: first / middle / last tick of scenario 0.
        const auto& s0 = trajectories[0];
        if (!s0.samples.empty()) {
            std::cout << "Scenario 0 target pose samples:\n";
            printTick("first", s0.samples.front());
            printTick("middle", s0.samples[s0.samples.size() / 2]);
            printTick("last", s0.samples.back());
            std::cout << "  raw simTimeMsec[0..15]:";
            for (size_t i = 0; i < std::min(s0.samples.size(), size_t{16}); ++i)
                std::cout << " " << std::setprecision(1) << s0.samples[i].simTimeMsec;
            std::cout << "\n";
        }

        // 037 M2 source-spacing diagnostic. The tracker fail-loud
        // (crrcsim_tracker_helper.cpp / tracker_stepper.cc) compares ONLY
        // samples[1]-samples[0] per scenario against SIM_TIME_STEP_MSEC. Report
        // the real pattern: per-tick gap histogram across ALL ticks of ALL
        // scenarios, plus the first-gap-per-scenario the check actually uses —
        // so a relax decision is data-driven (systematic offset vs jitter).
        {
            std::map<long, size_t> gapHist;       // lround(gap ms) -> count (all gaps)
            std::map<long, size_t> firstGapHist;  // lround(first gap) -> #scenarios
            double gMin = 1e18, gMax = -1e18, gSum = 0.0; size_t gN = 0;
            for (const auto& traj : trajectories) {
                const auto& s = traj.samples;
                if (s.size() >= 2)
                    firstGapHist[std::lround(s[1].simTimeMsec - s[0].simTimeMsec)]++;
                for (size_t i = 1; i < s.size(); ++i) {
                    double g = s[i].simTimeMsec - s[i - 1].simTimeMsec;
                    gapHist[std::lround(g)]++;
                    gMin = std::min(gMin, g); gMax = std::max(gMax, g);
                    gSum += g; ++gN;
                }
            }
            std::cout << "\nTick-gap analysis (" << gN << " gaps across "
                      << trajectories.size() << " scenarios):\n"
                      << "  raw gap ms: min=" << std::setprecision(3) << gMin
                      << " mean=" << (gN ? gSum / gN : 0.0)
                      << " max=" << gMax << "\n"
                      << "  rounded-gap histogram (ms : count):\n";
            for (const auto& kv : gapHist)
                std::cout << "    " << std::setw(4) << kv.first << " : " << kv.second << "\n";
            std::cout << "  FIRST-gap per scenario (what the fail-loud checks) (ms : #scen):\n";
            for (const auto& kv : firstGapHist)
                std::cout << "    " << std::setw(4) << kv.first << " : " << kv.second << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        exitCode = 1;
    }

    Aws::ShutdownAPI(options);
    return exitCode;
}

// 030 M3b — S3 integration test for the source M1 dmp loader (T020a).
//
// Gated on env var AUTOC_S3_TESTS=1 — skips silently otherwise. When enabled,
// initializes AWS + ConfigManager (autoc.ini default) and exercises the
// production load path end-to-end against the reference baseline dmp from
// the 2026-05-06 perf-build regression baseline. Validates non-empty result,
// scenario count > 0, and quat-norm sanity on the first sample.
//
// This is the "production load path actually works" gate. The hermetic
// contract test (source_dmp_loading_tests) covers the synthetic case.

#include <gtest/gtest.h>

#include <cstdlib>  // getenv
#include <stdexcept>
#include <string>

#include <aws/core/Aws.h>

#include "autoc/eval/source_dmp_loader.h"
#include "autoc/util/config.h"

namespace {

// Reference baseline — gen9999 of pastonly3, 2026-05-06 perf-build
// regression baseline (memory: reference_perf_build_reproducibility).
// Eval-vs-training fitness for this dmp's NN was -55944.664164 under
// rebuild-perf.sh as of M2b.2.
constexpr const char* kReferenceS3Key =
    "autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp";

bool s3TestsEnabled() {
    const char* env = std::getenv("AUTOC_S3_TESTS");
    return env != nullptr && std::string(env) == "1";
}

}  // namespace

TEST(SourceDmpS3Integration, LoadReferenceBaselineDmp) {
    if (!s3TestsEnabled()) {
        GTEST_SKIP() << "AUTOC_S3_TESTS env var not set to 1; skipping. "
                        "Set AUTOC_S3_TESTS=1 to exercise the production "
                        "S3 load path end-to-end.";
    }

    ConfigManager::initialize("autoc.ini");
    Aws::SDKOptions options;
    Aws::InitAPI(options);

    std::vector<SourceScenarioTrajectory> trajectories;
    try {
        trajectories = loadSourceDmp(kReferenceS3Key);
    } catch (const std::exception& e) {
        Aws::ShutdownAPI(options);
        FAIL() << "loadSourceDmp threw on reference baseline S3 key: "
               << e.what();
    }

    EXPECT_GT(trajectories.size(), 0u)
        << "Reference dmp returned empty scenario list — production load "
           "path is broken or the reference key is stale.";
    if (!trajectories.empty()) {
        const auto& traj0 = trajectories.front();
        EXPECT_GT(traj0.samples.size(), 0u);
        if (!traj0.samples.empty()) {
            const auto& first = traj0.samples.front();
            EXPECT_NEAR(first.orientation.norm(), 1.0f, 1e-3f)
                << "First-tick quat magnitude out of unit-norm range — "
                   "source data integrity check failed.";
            EXPECT_LT(first.position.norm(), 10000.0f)
                << "First-tick position > 10 km from origin — source data "
                   "integrity check failed.";
        }
    }

    Aws::ShutdownAPI(options);
}

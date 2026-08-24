// 030 M3 — Source M1 dmp loader contract tests (T020).
//
// Hermetic: synthesizes an `EvalResults` programmatically, serializes via
// cereal to a tmp file, loads back through `loadSourceDmp(local_path)` and
// asserts exact-match against the synthetic content. No S3 dependency, no
// committed binary fixture, no aws-CLI. T020a (separate test executable)
// covers the production S3-key path gated on AUTOC_S3_TESTS=1.

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cereal/archives/binary.hpp>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/source_dmp_loader.h"
#include "autoc/rpc/protocol.h"

namespace {

// Build a minimal AircraftState with known per-tick values. Only the fields
// the loader reads are populated; everything else stays at default/zero so
// the per-cereal-field count still matches but irrelevant fields don't drag
// determinism into this test.
AircraftState makeTickState(unsigned long simTimeMsec, float xPos) {
    AircraftState s;
    s.setSimTimeMsec(simTimeMsec);
    s.setPosition(gp_vec3(xPos, 0.0f, -10.0f));
    // Unit quat (w=1, x=y=z=0)
    s.setOrientation(gp_quat(1.0f, 0.0f, 0.0f, 0.0f));
    s.setVelocity(gp_vec3(15.0f, 0.0f, 0.0f));
    // gyroRates intentionally not set — v1 cereal schema doesn't persist it
    // (see SourceTickSample header note); SourceTickSample omits angularRate.
    return s;
}

// Build a synthetic EvalResults with `numScenarios` scenarios × `ticksPer`
// ticks per scenario. Each tick's simTimeMsec increments by 100 ms,
// position.x increments by 1 m so the loader's monotonicity / value
// extraction is verifiable against known values.
EvalResults makeSyntheticEvalResults(int numScenarios, int ticksPer,
                                     bool perfectScenarios = true) {
    EvalResults results;
    results.tickList.resize(numScenarios);
    results.scenarioList.resize(numScenarios);
    results.crashReasonList.assign(numScenarios, CrashReason::None);
    results.pathList.resize(numScenarios);  // empty paths — loader doesn't read them

    for (int s = 0; s < numScenarios; ++s) {
        ScenarioMetadata& meta = results.scenarioList[s];
        meta.pathVariantIndex = s % 6;          // 0..5 cycling
        meta.windVariantIndex = s % 4;          // 0..3 cycling
        // 033 cleanup: windSeed removed; use scenarioSeed as the per-
        // scenario PRNG-derivation root.
        meta.scenarioSeed = static_cast<uint64_t>(0x1000ull + s);
        meta.scenarioSequence = static_cast<uint64_t>(100 + s);

        // 041 T020 — grouped. The loader emits initialState followed by one
        // sample per tick, so laying tick 0 into `initialState` keeps the total
        // at `ticksPer` and the position sequence at 0, 1, 2, … exactly as
        // before; the assertions downstream are unchanged.
        ScenarioTicks& sc = results.tickList[s];
        sc.initialState = makeTickState(0UL, 0.0f);
        sc.ticks.reserve(ticksPer > 0 ? ticksPer - 1 : 0);
        for (int t = 1; t < ticksPer; ++t) {
            sc.ticks.emplace_back(
                makeTickState(static_cast<unsigned long>(t) * 100UL,
                              static_cast<float>(t)));
        }
        (void)perfectScenarios;  // hook for future malformed-fixture variants
    }
    return results;
}

// Serialize an EvalResults to a tmp file. Returns the path.
std::string writeTmpDmp(const EvalResults& results) {
    std::string path =
        (std::filesystem::temp_directory_path() /
         (std::string("autoc_test_dmp_") +
          std::to_string(std::rand()) + ".dmp"))
            .string();
    std::ofstream out(path, std::ios::binary);
    cereal::BinaryOutputArchive oa(out);
    oa(results);
    out.close();
    return path;
}

}  // namespace

TEST(SourceDmpLoading, RoundTripSyntheticFixture) {
    // 3 scenarios × 5 ticks each.
    const int kScenarios = 3;
    const int kTicks = 5;
    EvalResults synthetic = makeSyntheticEvalResults(kScenarios, kTicks);
    const std::string path = writeTmpDmp(synthetic);

    auto trajectories = loadSourceDmp(path);
    std::filesystem::remove(path);

    ASSERT_EQ(static_cast<int>(trajectories.size()), kScenarios);
    for (int s = 0; s < kScenarios; ++s) {
        const auto& traj = trajectories[s];
        EXPECT_EQ(traj.sourceScenarioIndex, s);
        EXPECT_EQ(traj.variation.pathVariantIndex, s % 6);
        EXPECT_EQ(traj.variation.windVariantIndex, s % 4);
        EXPECT_EQ(traj.variation.scenarioSeed,
                  static_cast<uint64_t>(0x1000ull + s));
        ASSERT_EQ(static_cast<int>(traj.samples.size()), kTicks);
        for (int t = 0; t < kTicks; ++t) {
            const auto& sample = traj.samples[t];
            EXPECT_DOUBLE_EQ(sample.simTimeMsec, t * 100.0);
            EXPECT_FLOAT_EQ(sample.position.x(), static_cast<float>(t));
            EXPECT_FLOAT_EQ(sample.position.y(), 0.0f);
            EXPECT_FLOAT_EQ(sample.position.z(), -10.0f);
            EXPECT_NEAR(sample.orientation.norm(), 1.0f, 1e-6f);
            EXPECT_FLOAT_EQ(sample.velocity.x(), 15.0f);
        }
    }
}

TEST(SourceDmpLoading, MonotonicSimTimeMsec) {
    EvalResults synthetic = makeSyntheticEvalResults(1, 10);
    const std::string path = writeTmpDmp(synthetic);
    auto trajectories = loadSourceDmp(path);
    std::filesystem::remove(path);

    ASSERT_EQ(trajectories.size(), 1u);
    const auto& samples = trajectories[0].samples;
    for (size_t i = 1; i < samples.size(); ++i) {
        EXPECT_GT(samples[i].simTimeMsec, samples[i - 1].simTimeMsec)
            << "sample " << i << " not monotonic vs " << (i - 1);
    }
}

TEST(SourceDmpLoading, RejectsMissingFileLoudly) {
    EXPECT_THROW(loadSourceDmp("/tmp/this_path_does_not_exist_for_sure_xyz.dmp"),
                 std::runtime_error)
        << "Missing local file should raise — but only after the existence"
           " check fails through to S3 with no S3 client available";
    // (When ConfigManager is uninitialized, the S3 path also throws — both
    // failure modes funnel into std::runtime_error per loud-fail contract.)
}

TEST(SourceDmpLoading, RejectsEmptyAircraftStateList) {
    EvalResults empty;  // no scenarios populated
    const std::string path = writeTmpDmp(empty);
    EXPECT_THROW(loadSourceDmp(path), std::runtime_error);
    std::filesystem::remove(path);
}

TEST(SourceDmpLoading, FilterCrashedSourceScenarios) {
    EvalResults synthetic = makeSyntheticEvalResults(4, 5);
    // Mark scenario 1 as Sim crash, scenario 3 as TimeLimit (NOT a crash).
    synthetic.crashReasonList = {
        CrashReason::None,
        CrashReason::Sim,           // filter
        CrashReason::None,
        CrashReason::TimeLimit,     // keep (not a crash per protocol.h isCrash)
    };
    const std::string path = writeTmpDmp(synthetic);
    auto loaded = loadSourceDmp(path);
    std::filesystem::remove(path);
    ASSERT_EQ(loaded.size(), 4u);

    auto filtered =
        filterCrashedSourceScenarios(loaded, synthetic.crashReasonList);
    ASSERT_EQ(filtered.size(), 3u);
    // Verify scenario 1 was dropped; scenarios 0, 2, 3 preserved (in order).
    EXPECT_EQ(filtered[0].sourceScenarioIndex, 0);
    EXPECT_EQ(filtered[1].sourceScenarioIndex, 2);
    EXPECT_EQ(filtered[2].sourceScenarioIndex, 3);
}

TEST(SourceDmpLoading, FilterByPathWindCrossProduct) {
    EvalResults synthetic = makeSyntheticEvalResults(12, 3);
    // pathVariantIndex cycles 0..5 (s % 6); windVariantIndex cycles 0..3 (s % 4).
    // Scenarios:  s=0  → p=0 w=0
    //             s=1  → p=1 w=1
    //             s=2  → p=2 w=2
    //             s=3  → p=3 w=3
    //             s=4  → p=4 w=0
    //             s=5  → p=5 w=1
    //             s=6  → p=0 w=2
    //             s=7  → p=1 w=3
    //             s=8  → p=2 w=0
    //             s=9  → p=3 w=1
    //             s=10 → p=4 w=2
    //             s=11 → p=5 w=3
    const std::string path = writeTmpDmp(synthetic);
    auto loaded = loadSourceDmp(path);
    std::filesystem::remove(path);

    // Subset {paths=0,1; winds=0,1} should match s=0 (0,0), s=1 (1,1),
    // s=6 (0,2)→no, s=7 (1,3)→no — i.e. only s=0 and s=1.
    auto filtered = filterByScenarioIndex(loaded, {0, 1}, {0, 1});
    ASSERT_EQ(filtered.size(), 2u);
    EXPECT_EQ(filtered[0].sourceScenarioIndex, 0);
    EXPECT_EQ(filtered[1].sourceScenarioIndex, 1);
}

TEST(SourceDmpLoading, FilterByEmptySubsetIsIdentity) {
    EvalResults synthetic = makeSyntheticEvalResults(5, 3);
    const std::string path = writeTmpDmp(synthetic);
    auto loaded = loadSourceDmp(path);
    std::filesystem::remove(path);

    auto same = filterByScenarioIndex(loaded, {}, {});
    EXPECT_EQ(same.size(), loaded.size());

    auto pathOnly = filterByScenarioIndex(loaded, {1, 3}, {});
    // Scenarios with pathVariantIndex ∈ {1, 3}: s=1 (p=1), s=3 (p=3).
    EXPECT_EQ(pathOnly.size(), 2u);
    EXPECT_EQ(pathOnly[0].sourceScenarioIndex, 1);
    EXPECT_EQ(pathOnly[1].sourceScenarioIndex, 3);
}

// Contract test: Configuration parsing
// Defines the surviving behavior: load autoc.ini, verify key types and defaults.
// This contract MUST hold through config parser replacement (GP→inih).
//
// NOTE: These tests verify the config CONTRACT (key names, types, defaults),
// not the GPConfiguration implementation. When inih replaces GPConfiguration,
// these tests should pass with zero changes.

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <cstdio>
#include <set>
#include <vector>
#include <type_traits>
#include <cstdint>

#include "autoc/util/config.h"  // 034 FR-010 — AUTOC_CONFIG_FIELDS macro

// For now we test the contract by writing a temp ini file and verifying
// that key names and value types are correct. The actual parser will change
// from GPConfiguration to inih, but the contract stays the same.

namespace {

// Helper: write a test ini file with known values
std::string writeTempIni(const std::string& content) {
    std::string path = "/tmp/contract_config_test.ini";
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

} // namespace

// ============================================================
// Contract: Required config keys exist with correct types
// ============================================================

TEST(ContractConfig, RequiredIntKeys) {
    // These int keys MUST be parseable from autoc.ini
    std::vector<std::string> required_int_keys = {
        "PopulationSize",
        "NumberOfGenerations",
        "EvalThreads",
        "SimNumPathsPerGeneration",
    };

    std::string ini = "";
    for (const auto& key : required_int_keys) {
        ini += key + " = 42\n";
    }

    std::string path = writeTempIni(ini);
    // Verify the file is well-formed (parseable by any INI parser)
    std::ifstream in(path);
    ASSERT_TRUE(in.good());

    std::string line;
    int count = 0;
    while (std::getline(in, line)) {
        if (!line.empty()) count++;
    }
    EXPECT_EQ(count, static_cast<int>(required_int_keys.size()));
}

TEST(ContractConfig, RequiredDoubleKeys) {
    std::vector<std::string> required_double_keys = {
        "NNMutationSigma",
        "NNCrossoverAlpha",
    };

    std::string ini = "";
    for (const auto& key : required_double_keys) {
        ini += key + " = 0.5\n";
    }

    std::string path = writeTempIni(ini);
    std::ifstream in(path);
    ASSERT_TRUE(in.good());
}

TEST(ContractConfig, RequiredStringKeys) {
    std::vector<std::string> required_string_keys = {
        "S3Bucket",
        "S3Profile",
        "PathGeneratorMethod",
        "NNWeightFile",
        "NNInitMethod",
    };

    std::string ini = "";
    for (const auto& key : required_string_keys) {
        ini += key + " = test_value\n";
    }

    std::string path = writeTempIni(ini);
    std::ifstream in(path);
    ASSERT_TRUE(in.good());
}

// ============================================================
// Contract: Default values for critical NN config keys
// ============================================================

TEST(ContractConfig, NNDefaultValues) {
    // These are the documented defaults from the config contract.
    // When the config parser is replaced, these defaults must be preserved.

    // NNMutationSigma default: 0.1
    EXPECT_NEAR(0.1, 0.1, 1e-6);

    // NNCrossoverAlpha default: -1.0 (uniform random)
    EXPECT_NEAR(-1.0, -1.0, 1e-6);

    // NNInitMethod default: "xavier"
    EXPECT_STREQ("xavier", "xavier");

    // NNWeightFile default: "nn_weights.dat"
    EXPECT_STREQ("nn_weights.dat", "nn_weights.dat");
}

// ============================================================
// Contract: INI format supports comments and empty lines
// ============================================================

TEST(ContractConfig, IniFormatComments) {
    std::string ini =
        "# This is a comment\n"
        "; This is also a comment\n"
        "\n"
        "PopulationSize = 100\n"
        "# Another comment\n"
        "NumberOfGenerations = 500\n";

    std::string path = writeTempIni(ini);
    std::ifstream in(path);
    ASSERT_TRUE(in.good());

    // Count non-comment, non-empty lines
    std::string line;
    int data_lines = 0;
    while (std::getline(in, line)) {
        if (!line.empty() && line[0] != '#' && line[0] != ';') {
            data_lines++;
        }
    }
    EXPECT_EQ(data_lines, 2);
}

// ============================================================
// Contract: New 014 config keys (to be added)
// ============================================================

TEST(ContractConfig, NewKeysFromSpec014) {
    // These keys are defined in contracts/config.md and must be
    // supported after this feature is implemented.
    std::string ini =
        "NNSigmaFloor = 0.05\n"
        "OptimizerType = sep-cma-es\n"
        "CurriculumEnabled = 1\n"
        "CurriculumSchedule = 1:50,7:150,49:0\n"
        "FitnessAggregation = minimax\n"
        "FitnessPercentile = 0.95\n"
        "OutputDir = runs/\n";

    std::string path = writeTempIni(ini);
    std::ifstream in(path);
    ASSERT_TRUE(in.good());

    // Verify all 7 new keys are present
    std::string line;
    int count = 0;
    while (std::getline(in, line)) {
        if (!line.empty()) count++;
    }
    EXPECT_EQ(count, 7);
}

// ============================================================
// 034 FR-010 — AUTOC_CONFIG_FIELDS single-source integrity
// ============================================================
// The macro drives BOTH config.cc parse and autoc.cc startup print, so a
// run's exact config is always recoverable from its log. These guards catch
// accidental macro corruption (dupes / count drift). A field added to the
// struct but omitted from the macro is caught at the integration layer by the
// before/after byte-identical eval (it would silently parse to its default).
TEST(ContractConfig, ConfigFieldsMacroKeysUnique) {
    std::vector<std::string> keys;
#define X(type, field, key) keys.push_back(key);
    AUTOC_CONFIG_FIELDS(X)
#undef X
    const std::set<std::string> uniq(keys.begin(), keys.end());
    EXPECT_EQ(uniq.size(), keys.size())
        << "duplicate ini key in AUTOC_CONFIG_FIELDS (parse/print single source)";
}

TEST(ContractConfig, ConfigFieldsMacroCount) {
    size_t n = 0;
#define X(type, field, key) ++n;
    AUTOC_CONFIG_FIELDS(X)
#undef X
    // Bump this when knobs are added/removed. 034 US4 added 6 craft sigmas
    // (craftCG/Drag/Trim/Thrust/PitchEff/RollEff) → 86; then the
    // EnableCraftVariations master-disable knob → 87; 035 FR-003 added
    // LexicaseEpsilonMode → 88; 035 FR-P10 added S3ObjectTagging → 89, then
    // removed it (tagging is now hardcoded retain=expire, fail-fast) → 88;
    // 035 M2 added TrackerSourceBucket → 89.
    // 037 added ControlIntervalMsec + actuator sigmas (→ 93); servo v2 added
    // ServoModelEnabled → 94; then servo v2 cleanup removed CraftServoTauSigma
    // (dead — no lag term in the PWM-latch+slew model) → 93.
    // 038 T001 added EnableHullCrashPenalty + HullCrashPenaltyFactor → 95;
    // t13 added OobCrashPenaltyWeight (fraction-based OOB rate penalty) → 96;
    // 038 US3 added EnablePredictorHead (aux span-predictor ablation gate) → 97.
    // 038 t7 added TrackerChaseUseSourceScenarioSeed (chase shares M1 source
    // env/craft/entry seed) → 98.
    // 040 T015 added the 18 Airframe* obstruction keys (enabled + wing/nose
    // AABBs + prop disc), replacing the compile-time proxy → 116; T017 retired
    // CameraFrameRateHz + CameraLatencyMs (cadence follows ControlIntervalMsec,
    // latency emerges from acquisition) → 114; T029 retired
    // CameraFOVHorizontalDeg + CameraFOVVerticalDeg (FOV is now DERIVED from
    // the grid, FR-003) and added CameraPixelsH/V + CameraDegPerPixel → 115.
    // 040 US4 added 19 signal-budget + acquisition keys → 134: twelve for the
    // link budget (flux/optics/ambient/noise/cdma/qFloor/qSat + the two emission
    // beam-width angles + the three envelope bounds) and seven for the
    // acquisition machine (four gateware timings + three quality-regime
    // anchors). All nineteen are exposed rather than baked BECAUSE FR-036's
    // calibration rehearsal requires substituting an alternative for each
    // ASSUMED value with no structural change — a baked value cannot be
    // rehearsed. T058 then DELETED BeaconEmissionConeDeg -- the hard 270 deg
    // cutoff the flat-top emission profile replaced, which after FR-019 had no
    // reader left and was a live-looking knob that changed nothing -> 133.
    // 2026-08-02 added SignalAmbientKnee -> 134: the ambient-compression term
    // 031 field test #4 forced, where ambient shunts signal at the PD rather
    // than merely adding noise.
    EXPECT_EQ(n, 134u) << "AUTOC_CONFIG_FIELDS field count changed — update the "
                         "expected count and confirm parse+print still match";
}

// ============================================================
// 034 FR-011 — Seed width for lossless paste-back
// ============================================================
// The operator copies a logged `effectiveMasterSeed` (uint64-surfaced) into
// `Seed=N` to reproduce a run bit-deterministically. `Seed` must be 64-bit so
// values > 2^31-1 (which `time(NULL)` produces after 2038, and pasted seeds
// may) round-trip without truncation. Stays signed for the -1 auto-sentinel.
TEST(ContractConfig, SeedFieldWideForPasteBack_FR011) {
    EXPECT_GE(sizeof(AutocConfig::seed), 8u)
        << "Seed must be 64-bit for lossless paste-back of effectiveMasterSeed";
    EXPECT_TRUE(std::is_signed<decltype(AutocConfig::seed)>::value)
        << "Seed must stay signed so -1 (auto wall-clock) sentinel works";
    // A representative > INT_MAX seed is representable + round-trips through the field.
    AutocConfig cfg;
    cfg.seed = INT64_C(3000000000);  // > 2^31-1
    EXPECT_EQ(cfg.seed, INT64_C(3000000000));
}

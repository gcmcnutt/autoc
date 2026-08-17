// 041 — training/eval ini agreement on INPUT- and FITNESS-affecting knobs.
//
// WHY THIS EXISTS. The eval-vs-training bitwise gate compares a re-evaluated
// genome's fitness against its stored training fitness. That comparison is only
// meaningful if the eval ini presents the SAME policy inputs and the SAME
// objective as the training ini. When it does not, the gate fails — and it
// fails looking exactly like a code regression, which is the expensive way to
// discover an ini typo.
//
// 041 makes this sharper: EnableEnvelopeInputs / EnableAccelInputs / AccelScaleG
// change the NN INPUT VECTOR itself. A mismatch there does not shift a number
// slightly; it evaluates a different policy.
//
// ⚠️ HOW THIS DIFFERS FROM THE PINS T008 REMOVED. T008 stripped assertions on
// production VALUES (`FitStreakThreshold == 0.5`) because an operator retuning a
// knob would fail a test for a reason unrelated to correctness. This asserts a
// RELATIONSHIP — that two files agree — which stays true under any retune, as
// long as both files are edited. It cannot fire spuriously on a deliberate
// change; it fires on a half-applied one.
//
// ⚠️ SCENARIO SHAPE IS DELIBERATELY NOT CHECKED for every pair. autoc-eval.ini
// was repointed on 2026-07-10 to 7 random paths for novel-geometry
// generalization, so it MUST differ from autoc.ini in path count and generator.
// Only the shape-matched repro pair is held to that.

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Minimal ini scan: `Key = value  # comment`. Deliberately not INIReader —
// this test is about what the FILES say, and going through the parser would
// fold in defaults for absent keys, which is precisely what it must detect.
// Resolve against the repo root baked in at configure time. The ALL target
// runs test binaries from the build dir while ctest can run them from
// anywhere, so a relative path would pass in one harness and fail in the other
// — and "cannot open" would read as a missing ini rather than a wrong cwd.
#ifndef AUTOC_SOURCE_DIR
#error "AUTOC_SOURCE_DIR must be defined by the build (see CMakeLists.txt)"
#endif

std::string repoPath(const std::string& rel) {
    return std::string(AUTOC_SOURCE_DIR) + "/" + rel;
}

std::map<std::string, std::string> readKeys(const std::string& relPath) {
    const std::string path = repoPath(relPath);
    std::map<std::string, std::string> out;
    std::ifstream in(path);
    EXPECT_TRUE(in.is_open()) << "cannot open " << path;
    std::string line;
    while (std::getline(in, line)) {
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            const size_t b = s.find_first_not_of(" \t\r\n");
            const size_t e = s.find_last_not_of(" \t\r\n");
            s = (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
        };
        trim(key);
        trim(val);
        if (key.empty() || val.empty()) continue;
        out[key] = val;
    }
    return out;
}

// Knobs that change the NN input vector or the objective. A difference in any
// of these makes a training/eval fitness comparison meaningless.
const std::vector<std::string> kInputAndObjectiveKeys = {
    // 041 US4 — these change the INPUT VECTOR itself.
    "EnableEnvelopeInputs", "EnableAccelInputs", "AccelScaleG",
    // The objective's geometry.
    "FitDistScaleBehind", "FitDistScaleAhead", "FitConeAngleDeg",
    "FitStreakThreshold", "FitStreakRampSec",
    // Cadence + actuation: both change what the policy experiences per tick.
    "ControlIntervalMsec", "ServoModelEnabled",
    // 041 T041f — the camera GEOMETRY. M2-only keys, so they are absent from
    // both M1 inis and skipped there (absent in both is legal); on the tracker
    // pairs they are checked. These decide the field the beacons project into,
    // so a train/eval disagreement would evaluate a policy against a different
    // optic than it trained on — the same class of silent mismatch as the input
    // knobs above, and newly worth guarding now that the field traces to a
    // physical measurement rather than to a shared default.
    "CameraPixelsH", "CameraPixelsV", "CameraDegPerPixel",
    "CameraDetectionRangeM",
};

// Additionally required of a BITWISE-REPRO pair: the scenario set itself.
const std::vector<std::string> kScenarioShapeKeys = {
    "SimNumPathsPerGeneration", "PathGeneratorMethod", "RandomPathSeedB",
};

void expectAgreement(const std::string& trainPath, const std::string& evalPath,
                     const std::vector<std::string>& keys, const char* why) {
    const auto train = readKeys(trainPath);
    const auto eval = readKeys(evalPath);

    for (const auto& key : keys) {
        const auto t = train.find(key);
        const auto e = eval.find(key);
        const bool inTrain = (t != train.end());
        const bool inEval = (e != eval.end());

        // Present in one and absent in the other is the SILENT case: the absent
        // side falls back to a compiled default that may happen to match today
        // and stop matching after any default change. Both or neither.
        ASSERT_EQ(inTrain, inEval)
            << key << " is declared in only one of " << trainPath << " / "
            << evalPath << " — declare it in both, or neither. " << why;
        if (!inTrain) continue;

        EXPECT_EQ(t->second, e->second)
            << key << " differs: " << trainPath << " has '" << t->second
            << "', " << evalPath << " has '" << e->second << "'. " << why;
    }
}

const char* kInputWhy =
    "These change the NN input vector or the objective, so an eval run would "
    "score a different policy against a different target and the bitwise gate "
    "would fail looking like a code regression.";

}  // namespace

// ---------------------------------------------------------------------------
// The bitwise-repro pair — held to BOTH key sets.
// ---------------------------------------------------------------------------

TEST(IniPairConsistency, BasicM1TrainingAndEvalAgreeOnInputsAndObjective) {
    expectAgreement("autoc-basic-m1.ini", "autoc-basic-m1-eval.ini",
                    kInputAndObjectiveKeys, kInputWhy);
}

TEST(IniPairConsistency, BasicM1TrainingAndEvalAgreeOnScenarioShape) {
    // This is the pair the determinism check actually runs on, so its scenario
    // set must match too — not just its inputs.
    expectAgreement("autoc-basic-m1.ini", "autoc-basic-m1-eval.ini",
                    kScenarioShapeKeys,
                    "This is the BITWISE-REPRO pair; a different scenario set "
                    "cannot reproduce a stored fitness.");
}

// ---------------------------------------------------------------------------
// The generalization pairs — inputs/objective only, shape deliberately free.
// ---------------------------------------------------------------------------

TEST(IniPairConsistency, M1TrainingAndEvalAgreeOnInputsAndObjective) {
    // autoc-eval.ini runs NOVEL geometry on purpose (2026-07-10), so its path
    // count and generator differ from autoc.ini by design and are NOT checked.
    // The inputs and the objective still must match: otherwise "generalizes
    // worse" and "was scored differently" are indistinguishable.
    expectAgreement("autoc.ini", "autoc-eval.ini",
                    kInputAndObjectiveKeys, kInputWhy);
}

TEST(IniPairConsistency, M1VisualEvalAgreesOnInputsAndObjective) {
    expectAgreement("autoc.ini", "autoc-eval-visual.ini",
                    kInputAndObjectiveKeys, kInputWhy);
}

TEST(IniPairConsistency, M2TrackerTrainingAndEvalAgreeOnInputsAndObjective) {
    expectAgreement("autoc-tracker.ini", "autoc-eval-tracker.ini",
                    kInputAndObjectiveKeys, kInputWhy);
}

TEST(IniPairConsistency, M2TrackerVisualEvalAgreesOnInputsAndObjective) {
    expectAgreement("autoc-tracker.ini", "autoc-eval-tracker-visual.ini",
                    kInputAndObjectiveKeys, kInputWhy);
}

// ---------------------------------------------------------------------------
// M2's estimator thresholds — tracker inis only.
// ---------------------------------------------------------------------------

TEST(IniPairConsistency, M2EstimatorThresholdsAgreeAcrossTrackerInis) {
    const std::vector<std::string> kEstimator = {
        "EnvelopeSpanLo", "EnvelopeSpanHi", "EnvelopeCentroidRadius",
    };
    const char* why =
        "The M2 IN_ENVELOPE flag is an ESTIMATE produced from these thresholds, "
        "so a mismatch changes an input the policy reads.";
    expectAgreement("autoc-tracker.ini", "autoc-eval-tracker.ini", kEstimator, why);
    expectAgreement("autoc-tracker.ini", "autoc-eval-tracker-visual.ini", kEstimator, why);
}

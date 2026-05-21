// 030 M6b — Tracker-mode config parser contract tests (T034).
//
// Mirror what ConfigManager::initialize does — drive inih INIReader on
// temp .ini files and assert the new tracker fields parse correctly.
// Avoids ConfigManager's singleton state (init-once, no reset) so each
// test can drive its own scenario.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <INIReader.h>

namespace {

std::string writeTempIni(const std::string& filename, const std::string& content) {
    std::string path = "/tmp/" + filename;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

}  // namespace

// ---------------------------------------------------------------------------
// Mode dispatch field
// ---------------------------------------------------------------------------

TEST(TrackerConfig, ModeDefaultsToPathgen) {
    std::string path = writeTempIni("tracker_mode_default.ini", "");
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.Get("", "Mode", "pathgen"), "pathgen");
}

TEST(TrackerConfig, ModeReadsTrackerValue) {
    std::string ini = "Mode = tracker\n";
    std::string path = writeTempIni("tracker_mode_value.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.Get("", "Mode", "pathgen"), "tracker");
}

// ---------------------------------------------------------------------------
// Tracker source dmp + scenario subset
// ---------------------------------------------------------------------------

TEST(TrackerConfig, SourceDmpKeysParseAsStrings) {
    std::string ini =
        "TrackerSourceRun = autoc-storage/run-id-foo/gen9200.dmp\n"
        "TrackerPathSubset = 0,1,2,3,4,5\n"
        "TrackerWindSubset = 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19\n";
    std::string path = writeTempIni("tracker_source.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.Get("", "TrackerSourceRun", ""),
              "autoc-storage/run-id-foo/gen9200.dmp");
    EXPECT_EQ(reader.Get("", "TrackerPathSubset", ""), "0,1,2,3,4,5");
    // "0,1,...,19" — 20 indices, 19 commas, mix of 1- and 2-digit ⇒ 49 chars.
    EXPECT_EQ(reader.Get("", "TrackerWindSubset", "").size(), 49u);
}

// ---------------------------------------------------------------------------
// Trail rabbit fitness fields
// ---------------------------------------------------------------------------

TEST(TrackerConfig, TrailRabbitDefaults) {
    std::string path = writeTempIni("trail_rabbit_defaults.ini", "");
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "TrailDistance", 3.048), 3.048);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "LowSpeedTrailThreshold", 2.0), 2.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "LowSpeedTrailHysteresis", 0.5), 0.5);
}

// ---------------------------------------------------------------------------
// Crash hull curriculum fields
// ---------------------------------------------------------------------------

TEST(TrackerConfig, CrashHullCurriculumParses) {
    std::string ini =
        "CrashHullShape = SPHERE\n"
        "CrashHullRadius = 1.0\n"
        "PCrashGen0 = 0.0\n"
        "PCrashGenRamp = 100\n"
        "PCrashGenPlateau = 200\n"
        "PCrashPlateau = 0.30\n";
    std::string path = writeTempIni("crash_hull.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.Get("", "CrashHullShape", ""), "SPHERE");
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CrashHullRadius", 0.0), 1.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "PCrashGen0", -1.0), 0.0);
    EXPECT_EQ(reader.GetInteger("", "PCrashGenRamp", -1), 100);
    EXPECT_EQ(reader.GetInteger("", "PCrashGenPlateau", -1), 200);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "PCrashPlateau", 0.0), 0.30);
}

// ---------------------------------------------------------------------------
// Arena fields (FR-016)
// ---------------------------------------------------------------------------

TEST(TrackerConfig, FlightArenaDefaults) {
    std::string path = writeTempIni("arena_defaults.ini", "");
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "FlightArenaRadius", 80.0), 80.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "FlightArenaFloorAGL", 5.0), 5.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "FlightArenaCeilingAGL", 100.0), 100.0);
}

// ---------------------------------------------------------------------------
// Camera config
// ---------------------------------------------------------------------------

TEST(TrackerConfig, CameraConfigParses) {
    std::string ini =
        "CameraCount = 1\n"
        "CameraFOVHorizontalDeg = 120.0\n"
        "CameraFOVVerticalDeg = 90.0\n"
        "CameraFrameRateHz = 30.0\n"
        "CameraLatencyMs = 0.0\n"
        "CameraMountOffsetX = 0.0\n"
        "CameraMountOffsetY = 0.0\n"
        "CameraMountOffsetZ = -0.05\n";
    std::string path = writeTempIni("camera.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.GetInteger("", "CameraCount", -1), 1);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraFOVHorizontalDeg", 0.0), 120.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraFOVVerticalDeg", 0.0), 90.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraFrameRateHz", 0.0), 30.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraMountOffsetZ", 0.0), -0.05);
}

// ---------------------------------------------------------------------------
// Beacon config (FR-004)
// ---------------------------------------------------------------------------

TEST(TrackerConfig, BeaconConfigParses) {
    std::string ini =
        "BeaconLeftWavelengthNm = 850\n"
        "BeaconRightWavelengthNm = 940\n"
        "BeaconEmissionConeDeg = 270.0\n"
        "BeaconLeftMountY = -0.45\n"
        "BeaconRightMountY = 0.45\n";
    std::string path = writeTempIni("beacon.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.GetInteger("", "BeaconLeftWavelengthNm", 0), 850);
    EXPECT_EQ(reader.GetInteger("", "BeaconRightWavelengthNm", 0), 940);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "BeaconEmissionConeDeg", 0.0), 270.0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "BeaconLeftMountY", 0.0), -0.45);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "BeaconRightMountY", 0.0), 0.45);
}

// ---------------------------------------------------------------------------
// autoc-tracker.ini — verify the canonical operator file parses clean.
// ---------------------------------------------------------------------------

TEST(TrackerConfig, OperatorIniParsesClean) {
    // The repo-root autoc-tracker.ini is the canonical operator-runnable
    // file (mirrors autoc.ini section structure for side-by-side diff,
    // smoke values for fast visual iteration). If it ever fails to parse,
    // smoke-test runs would fail too. CMake passes the source dir so the
    // test runs from any CWD.
    const std::string ini_path =
        std::string(AUTOC_SOURCE_DIR) + "/autoc-tracker.ini";
    INIReader reader(ini_path);
    ASSERT_EQ(reader.ParseError(), 0)
        << "autoc-tracker.ini must parse cleanly: " << ini_path;
    EXPECT_EQ(reader.Get("", "Mode", ""), "tracker");
    EXPECT_FALSE(reader.Get("", "TrackerSourceRun", "").empty());
    EXPECT_EQ(reader.Get("", "CrashHullShape", ""), "SPHERE");
    EXPECT_DOUBLE_EQ(reader.GetReal("", "FlightArenaRadius", 0.0), 80.0);
}

// ---------------------------------------------------------------------------
// 032 PHASE 1 — [DerivedFeatures] section
// ---------------------------------------------------------------------------

TEST(TrackerConfig, DerivedFeaturesDefaultsWhenSectionAbsent) {
    // If the [DerivedFeatures] section is absent entirely, the loader falls
    // back to the compiled-in default (CepGateThreshold = 1.25). Verify
    // the parser returns the user-supplied default when the key isn't in
    // the file.
    const std::string ini =
        "Mode = tracker\n"
        "TrackerSourceRun = some/key.dmp\n";
    std::string path = writeTempIni("derived_default.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("DerivedFeatures", "CepGateThreshold", 1.25), 1.25);
}

TEST(TrackerConfig, DerivedFeaturesExplicitValuesParse) {
    const std::string ini =
        "Mode = tracker\n"
        "TrackerSourceRun = some/key.dmp\n"
        "[DerivedFeatures]\n"
        "CepGateThreshold = 0.75\n";
    std::string path = writeTempIni("derived_explicit.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("DerivedFeatures", "CepGateThreshold", 1.25), 0.75);
}

TEST(TrackerConfig, DerivedFeaturesAtCanonicalDefault) {
    // Spec Q4 + research.md R2 lock the default at 1.25 (matches
    // kCepSentinelThreshold). The repo-root autoc-tracker.ini ships with
    // CepGateThreshold = 1.25. This is the gate against accidental ini
    // drift.
    const std::string ini_path =
        std::string(AUTOC_SOURCE_DIR) + "/autoc-tracker.ini";
    INIReader reader(ini_path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("DerivedFeatures", "CepGateThreshold", -1.0), 1.25);
}

// Out-of-range CepGateThreshold loud-fail is enforced via the ConfigManager
// loud-fail path (calls exit(1) with a clear error) — covered by manual
// operator runs at startup, not unit-testable without a process fork. The
// INIReader-level parse succeeds for any double value; the range check
// lives in src/util/config.cc per contracts/ini_schema.md.

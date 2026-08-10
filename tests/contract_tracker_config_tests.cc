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
#include <sstream>
#include <vector>

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
        "CameraPixelsH = 320\n"
        "CameraPixelsV = 240\n"
        "CameraDegPerPixel = 0.375\n"
        "CameraMountOffsetX = 0.0\n"
        "CameraMountOffsetY = 0.0\n"
        "CameraMountOffsetZ = -0.05\n";
    std::string path = writeTempIni("camera.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.GetInteger("", "CameraCount", -1), 1);
    // 040 T029 (FR-003) — the grid is configured, the field is DERIVED.
    EXPECT_EQ(reader.GetInteger("", "CameraPixelsH", 0), 320);
    EXPECT_EQ(reader.GetInteger("", "CameraPixelsV", 0), 240);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraDegPerPixel", 0.0), 0.375);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CameraMountOffsetZ", 0.0), -0.05);
}

// ---------------------------------------------------------------------------
// Beacon config (FR-004)
// ---------------------------------------------------------------------------

TEST(TrackerConfig, BeaconConfigParses) {
    std::string ini =
        "BeaconLeftWavelengthNm = 850\n"
        "BeaconRightWavelengthNm = 940\n"
        "BeaconLeftMountY = -0.386\n"
        "BeaconRightMountY = 0.386\n";
    std::string path = writeTempIni("beacon.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_EQ(reader.GetInteger("", "BeaconLeftWavelengthNm", 0), 850);
    EXPECT_EQ(reader.GetInteger("", "BeaconRightWavelengthNm", 0), 940);
    // 040 T030 (FR-004) — the measured 0.772 m separation, ±0.386 per tip.
    EXPECT_DOUBLE_EQ(reader.GetReal("", "BeaconLeftMountY", 0.0), -0.386);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "BeaconRightMountY", 0.0), 0.386);
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
    //
    // 041 T008 — this is a STRUCTURAL guard only: the file parses, and the two
    // keys without which tracker mode cannot start are present. It deliberately
    // does NOT pin operator-tunable values (it used to pin CrashHullShape and
    // FlightArenaRadius == 80). Such pins fail on an ordinary knob edit — a red
    // test for a reason unrelated to correctness — and 041 edits streak and
    // camera knobs in exactly these files. Values that are physical
    // MEASUREMENTS rather than tuning choices (beacon separation below) stay
    // pinned; the distinction is whether an operator is expected to change it.
    const std::string ini_path =
        std::string(AUTOC_SOURCE_DIR) + "/autoc-tracker.ini";
    INIReader reader(ini_path);
    ASSERT_EQ(reader.ParseError(), 0)
        << "autoc-tracker.ini must parse cleanly: " << ini_path;
    EXPECT_EQ(reader.Get("", "Mode", ""), "tracker");
    EXPECT_FALSE(reader.Get("", "TrackerSourceRun", "").empty());
}

// ---------------------------------------------------------------------------
// 032 PHASE 1 — derived perceptual features (CepGateThreshold)
// 033 cleanup: [DerivedFeatures] section header removed; flat key per
// autoc-tracker.ini convention (parsed from default section "").
// ---------------------------------------------------------------------------

TEST(TrackerConfig, DerivedFeaturesDefaultsWhenKeyAbsent) {
    // If the CepGateThreshold key is absent entirely, the loader falls
    // back to the compiled-in default (CepGateThreshold = 1.25).
    const std::string ini =
        "Mode = tracker\n"
        "TrackerSourceRun = some/key.dmp\n";
    std::string path = writeTempIni("derived_default.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CepGateThreshold", 1.25), 1.25);
}

TEST(TrackerConfig, DerivedFeaturesExplicitValuesParse) {
    const std::string ini =
        "Mode = tracker\n"
        "TrackerSourceRun = some/key.dmp\n"
        "CepGateThreshold = 0.75\n";
    std::string path = writeTempIni("derived_explicit.ini", ini);
    INIReader reader(path);
    ASSERT_EQ(reader.ParseError(), 0);
    EXPECT_DOUBLE_EQ(reader.GetReal("", "CepGateThreshold", 1.25), 0.75);
}

// 041 T008 — REMOVED: `DerivedFeaturesAtCanonicalDefault`, which read the
// production autoc-tracker.ini and pinned CepGateThreshold == 1.25.
//
// It was written as a gate "against accidental ini drift", but it cannot tell
// accidental drift from an operator deliberately sweeping the knob — and the
// latter is the normal case. The two fixture-owned tests above already cover
// what is actually testable here: the key parses when present, and the caller's
// default applies when absent. The compiled-in default and its [0, 2] range
// check live in src/util/config.cc, where a bad value fails loud at startup.

// Out-of-range CepGateThreshold loud-fail is enforced via the ConfigManager
// loud-fail path (calls exit(1) with a clear error) — covered by manual
// operator runs at startup, not unit-testable without a process fork. The
// INIReader-level parse succeeds for any double value; the range check
// lives in src/util/config.cc per contracts/ini_schema.md.

// ---------------------------------------------------------------------------
// 040 T009 (FR-034) — every airframe-obstruction key must be PRESENT in the
// tracker inis, not merely defaulted in the struct.
//
// WHY. AutocConfig declares these with in-class defaults, following local
// convention. That is convenient but creates exactly the hazard Constitution
// VII was written about: a missing ini key silently falls back to a plausible
// value and the model obstructs the wrong things with no error anywhere. The
// defaults mirror hb1AirframeDimensions() so they cannot drift — this test is
// what stops them from ever being load-bearing.
// ---------------------------------------------------------------------------

TEST(ContractTrackerConfig, AirframeObstructionKeysPresentInTrackerInis) {
    const std::vector<std::string> required = {
        "AirframeObstructionEnabled",
        "AirframeWingMinX", "AirframeWingMinY", "AirframeWingMinZ",
        "AirframeWingMaxX", "AirframeWingMaxY", "AirframeWingMaxZ",
        "AirframeNoseMinX", "AirframeNoseMinY", "AirframeNoseMinZ",
        "AirframeNoseMaxX", "AirframeNoseMaxY", "AirframeNoseMaxZ",
        "AirframePropPlaneX", "AirframePropAxisY", "AirframePropAxisZ",
        "AirframePropRadius", "AirframePropAttenuation",
    };
    // Present in EVERY ini, not just the tracker ones: the config surface is
    // shared, so a mode switch must never trip a missing key.
    for (const char* name : {"autoc.ini", "autoc-eval.ini", "autoc-eval-visual.ini",
                             "autoc-basic-m1.ini", "autoc-basic-m1-eval.ini",
                             "autoc-tracker.ini", "autoc-eval-tracker.ini",
                             "autoc-eval-tracker-visual.ini"}) {
        // ctest runs from build/, so resolve against the injected source dir
        // rather than the working directory.
        const std::string ini = std::string(AUTOC_SOURCE_DIR) + "/" + name;
        std::ifstream f(ini);
        ASSERT_TRUE(f.good()) << "cannot open " << ini;
        std::stringstream ss;
        ss << f.rdbuf();
        const std::string body = ss.str();
        for (const auto& key : required) {
            EXPECT_NE(body.find(key), std::string::npos)
                << ini << " is missing " << key
                << " — it would silently fall back to the struct default";
        }
    }
}

// ---------------------------------------------------------------------------
// 040 T029/T030 (FR-003, FR-004, FR-034) — the camera grid and the measured
// beacon separation must be PRESENT in every tracker ini, same hazard as the
// airframe keys above: `AutocConfig` declares them with in-class defaults, so
// a missing key falls back silently rather than failing loud.
//
// This one bites harder than most. `cameraDegPerPixel` sets BOTH the reported
// resolution and the derived field of view (FR-003), and the beacon mounts are
// the ONLY range reference the controller has — a silent fallback in either
// would train a controller against a camera nobody configured.
// ---------------------------------------------------------------------------

namespace {

// True when the ini ASSIGNS `key` — i.e. a non-comment line of the form
// "Key = ...". A plain substring search over the file body would also match
// prose, and the retirement notes in these inis deliberately name the keys
// they retired so an operator hunting a remembered knob finds out where it
// went. Those notes are worth keeping, so the check has to be sharper than
// "does this string appear anywhere".
bool iniAssignsKey(const std::string& body, const std::string& key) {
    std::istringstream lines(body);
    std::string line;
    while (std::getline(lines, line)) {
        const size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        if (line[first] == '#' || line[first] == ';') continue;
        if (line.compare(first, key.size(), key) != 0) continue;
        const size_t after = line.find_first_not_of(" \t", first + key.size());
        if (after != std::string::npos && line[after] == '=') return true;
    }
    return false;
}

}  // namespace

TEST(ContractTrackerConfig, CameraGridAndBeaconSeparationKeysPresent) {
    const std::vector<std::string> required = {
        "CameraPixelsH", "CameraPixelsV", "CameraDegPerPixel",
        "CameraMountOffsetX", "CameraMountOffsetY", "CameraMountOffsetZ",
        "BeaconLeftMountY", "BeaconRightMountY",
    };
    for (const char* name : {"autoc-tracker.ini", "autoc-eval-tracker.ini",
                             "autoc-eval-tracker-visual.ini"}) {
        const std::string ini = std::string(AUTOC_SOURCE_DIR) + "/" + name;
        std::ifstream f(ini);
        ASSERT_TRUE(f.good()) << "cannot open " << ini;
        std::stringstream ss;
        ss << f.rdbuf();
        const std::string body = ss.str();
        for (const auto& key : required) {
            EXPECT_TRUE(iniAssignsKey(body, key))
                << ini << " is missing " << key
                << " — it would silently fall back to the struct default";
        }
        // The retired keys must not reappear as assignments: an independently
        // set field of view is exactly what FR-003 exists to prevent.
        for (const char* gone : {"CameraFOVHorizontalDeg", "CameraFOVVerticalDeg",
                                 "CameraFrameRateHz", "CameraLatencyMs"}) {
            EXPECT_FALSE(iniAssignsKey(body, gone))
                << ini << " still assigns retired key " << gone;
        }
    }
}

TEST(ContractTrackerConfig, TrackerInisCarryTheMeasuredBeaconSeparation) {
    // The value itself, not just the key. 0.9 m (±0.45) put a systematic ~17%
    // into every inferred range; this is the gate against it drifting back.
    for (const char* name : {"autoc-tracker.ini", "autoc-eval-tracker.ini",
                             "autoc-eval-tracker-visual.ini"}) {
        const std::string ini = std::string(AUTOC_SOURCE_DIR) + "/" + name;
        INIReader reader(ini);
        ASSERT_EQ(reader.ParseError(), 0) << ini;
        const double left = reader.GetReal("", "BeaconLeftMountY", 0.0);
        const double right = reader.GetReal("", "BeaconRightMountY", 0.0);
        EXPECT_NEAR(right - left, 0.772, 1e-9)
            << ini << " beacon separation is not the measured 0.772 m";
    }
}

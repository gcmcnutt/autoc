#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <iostream>
#include <memory>

// Forward declarations
namespace Aws { namespace S3 { class S3Client; } }

// Unified configuration parsed from autoc.ini via inih INIReader.
struct AutocConfig {
    // --- Evolution ---
    int populationSize = 500;
    int numberOfGenerations = 50;
    double crossoverProbability = 90.0;
    double creationProbability = 5.0;
    int tournamentSize = 7;
    double swapMutationProbability = 5.0;
    int addBestToNewPopulation = 1;

    // --- NN-specific ---
    double nnMutationSigma = 0.1;
    double nnCrossoverAlpha = -1.0;
    double nnSigmaFloor = 0.0;       // Minimum mutation sigma (0 = disabled)
    std::string nnWeightFile = "nn_weights.dat";
    std::string nnInitMethod = "xavier";

    // --- Simulation ---
    int simNumPathsPerGen = 1;
    std::string generatorMethod = "classic";
    int evalThreads = 1;
    std::string minisimProgram = "./build/minisim";
    unsigned short minisimPortOverride = 0;

    // --- S3 ---
    std::string s3Bucket = "autoc-storage";
    std::string s3Profile = "default";

    // --- Eval mode ---
    int evaluateMode = 0;

    // --- Scenarios ---
    int windScenarioCount = 1;
    int randomPathSeedB = 67890;
    int seed = -1;

    // --- Demetic grouping (scenario assignment) ---
    int demeticGrouping = 0;
    int demeSize = 100;
    double demeticMigProbability = 0.0;

    // --- Entry and wind direction variations ---
    int enableEntryVariations = 0;
    int enableWindVariations = 0;
    int enableRabbitSpeedVariations = 0;
    double entryConeSigma = 30.0;     // degrees: half-angle of nose direction cone
    double entryRollSigma = 22.5;     // degrees: roll around body axis
    double entrySpeedSigma = 0.1;
    double windDirectionSigma = 45.0;

    // --- Entry position variations ---
    double entryPositionRadiusSigma = 0.0;
    double entryPositionAltSigma = 0.0;

    // --- Variation landscape ramp ---
    int variationRampStep = 0;

    // --- Selection strategy (015) ---
    std::string selectionMode = "sum";  // "sum", "minimax", "lexicase"

    // --- Fitness: point-accumulation scoring (022, conical surface V4) ---
    double fitDistScaleBehind = 7.0;     // Distance half-decay when behind rabbit (m)
    double fitDistScaleAhead = 2.0;      // Distance half-decay when ahead of rabbit (m, sharp)
    double fitConeAngleDeg = 45.0;       // Angular half-decay (degrees from "directly behind")
    double fitStreakThreshold = 0.5;     // Min stepPoints to maintain streak
    double fitStreakRampSec = 5.0;       // Seconds to reach max multiplier
    double fitStreakMultiplierMax = 5.0; // Maximum streak multiplier

    // --- Variable rabbit speed ---
    double rabbitSpeedNominal = 16.0;
    double rabbitSpeedSigma = 0.0;
    double rabbitSpeedMin = 8.0;
    double rabbitSpeedMax = 25.0;
    double rabbitSpeedCycleMin = 0.5;
    double rabbitSpeedCycleMax = 5.0;

    // === 030 TRACKER MODE (FR-011 + FR-019) =============================
    // Mode dispatch — "pathgen" (default, original autoc.ini behavior) or
    // "tracker" (this file launches tracker-mode training against a
    // recorded source dmp). All `tracker*` / `crashHull*` / `arena*` /
    // `camera*` / `beacon*` fields below are inert when mode == "pathgen".

    std::string mode = "pathgen";

    // --- Tracker source dmp + scenario subset (FR-001 + FR-011) ---
    // S3 key form: "<run-id>/gen<N>.dmp" (resolved against `s3Bucket`),
    // or absolute / relative local file path for offline-test convenience.
    std::string trackerSourceRun;
    // Comma-separated path-variant indices (e.g. "0,1,2,3,4,5"). Empty = all.
    std::string trackerPathSubset;
    // Comma-separated wind-variant indices (e.g. "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19").
    // Empty = all. Cross-product with TrackerPathSubset per FR-011.
    std::string trackerWindSubset;

    // Source pre-roll: source craft plays forward this many seconds before
    // chase starts evolving — gives source a head start so chase has
    // something in its forward FOV at NN tick 0. Solves the "chase init
    // velocity > source init velocity" overtake problem and the "both at
    // origin" degenerate case at tick 0. Chase entry stays M1-style
    // (origin + 180° yaw + entry variations); only the source-tick cursor
    // is offset. NN warm-start history is filled with the last 6 source
    // ticks of the pre-roll window so the recurrent state has real
    // beacon-observation context on its first eval.
    double trackerSourcePreRollSec = 0.5;  // 100ms NN cadence ⇒ 5 ticks default

    // --- Tracker fitness: trail rabbit (FR-008 + FR-008a + R10) ---
    double trailDistance = 3.048;          // m; 10 ft per FR-008
    double lowSpeedTrailThreshold = 2.0;   // m/s; below ⇒ nose-trail per R10
    double lowSpeedTrailHysteresis = 0.5;  // m/s; ±band around threshold

    // --- Tracker fitness: crash hull (FR-008b + R3) ---
    // Shape: "SPHERE" only in v1; "AABB_HB1"/"MESH_AIRFRAME" reserved.
    std::string crashHullShape = "SPHERE";
    double crashHullRadius = 1.0;           // m
    // 030 M11.preA.3 (2026-05-10): replaced 4-param ramp curriculum with a
    // single fixed Bernoulli probability per NN tick (10Hz). The ramp added
    // determinism risk (per-gen state) and made debugging awkward. Fixed
    // probability ⇒ deterministic per (scenario, gen) given windSeed PRNG.
    // 0.10 default = ~"50% chance of dying within 7 ticks (~0.7s) inside
    // hull" → strong incentive to keep target outside the 1m sphere.
    double crashHullProbability = 0.10;
    // Reserved for per-scenario hull-radius / probability variation.
    // Symmetry knob with EnableEntryVariations / EnableWindVariations /
    // EnableRabbitSpeedVariations. When the variation logic ships,
    // setting this to 1 will let the joint PRNG perturb radius /
    // probability per scenario; until then both 0 and 1 produce the
    // fixed crashHullProbability + crashHullRadius above.
    bool enableCrashHullVariations = false;

    // --- Tracker arena (FR-016) ---
    double flightArenaRadius = 80.0;        // m horizontal
    double flightArenaFloorAGL = 5.0;       // m AGL hard floor
    double flightArenaCeilingAGL = 100.0;   // m AGL ceiling

    // --- Camera config (FR-003) ---
    int cameraCount = 1;
    double cameraFOVHorizontalDeg = 120.0;
    double cameraFOVVerticalDeg = 90.0;
    double cameraFrameRateHz = 30.0;
    double cameraLatencyMs = 0.0;
    double cameraMountOffsetX = 0.0;
    double cameraMountOffsetY = 0.0;
    double cameraMountOffsetZ = -0.05;      // m above wing surface (NED, +Z down)

    // --- Beacon config (FR-004) ---
    int beaconLeftWavelengthNm = 850;
    int beaconRightWavelengthNm = 940;
    double beaconEmissionConeDeg = 270.0;
    double beaconLeftMountX = 0.0;
    double beaconLeftMountY = -0.45;        // left wingtip (body -y)
    double beaconLeftMountZ = 0.0;
    double beaconRightMountX = 0.0;
    double beaconRightMountY = +0.45;       // right wingtip (body +y)
    double beaconRightMountZ = 0.0;
};

class ConfigManager {
public:
    static void initialize(const std::string& filename = "autoc.ini", std::ostream& out = std::cout);
    static AutocConfig& getConfig();
    static bool isInitialized();
    static std::shared_ptr<Aws::S3::S3Client> getS3Client();

private:
    static AutocConfig* config;
    static bool initialized;
};

#endif // CONFIG_MANAGER_H

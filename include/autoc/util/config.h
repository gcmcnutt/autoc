#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <iostream>
#include <memory>
#include <cstdint>

#include "autoc/eval/aircraft_state.h"  // SIM_TIME_STEP_MSEC (cadence master)

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
    // 037 T001 -- control-loop cadence (ms). Single source of truth for the
    // NN/sensor interval, read from .ini (NOT the AUTOC_EVAL_INTERVAL_MSEC env
    // var per feedback_cli_over_env_vars). Defaults like every other knob -- a
    // missing key is NOT a fail-loud here: the value is auto-printed at
    // startup (AUTOC_CONFIG_FIELDS), so a defaulted cadence is auditable in the
    // log, not silent. 037 T017: the default tracks the compile-time master
    // SIM_TIME_STEP_MSEC (was a literal 100, which broke every defaulted
    // config at the 20 Hz flip). config.cc DOES fail-loud if an explicit .ini
    // value != SIM_TIME_STEP_MSEC -- the two must move together when
    // retraining at a new rate. Threaded to the crrcsim worker via WorkerInit
    // priming (workers have no ConfigManager). Per contracts/cadence-config.md
    // + research.md R6.
    int controlIntervalMsec = SIM_TIME_STEP_MSEC;
    int simNumPathsPerGen = 1;
    std::string generatorMethod = "classic";
    int evalThreads = 1;
    std::string workerProgram = "./scripts/crrcsim.sh";
    unsigned short workerPortOverride = 0;

    // --- S3 ---
    std::string s3Bucket = "autoc-storage";
    std::string s3Profile = "default";

    // --- Eval mode ---
    int evaluateMode = 0;

    // --- Scenarios ---
    int windScenarioCount = 1;
    int randomPathSeedB = 67890;
    // 034 FR-011 — int64_t (not int) so the operator can paste a logged
    // `effectiveMasterSeed` (uint64-surfaced, may exceed 2^31-1; `time(NULL)`
    // does after 2038) into `Seed=N` and reproduce the run without truncation.
    // -1 = wall-clock auto-seed. Widening preserves the value (and thus the
    // PRNG sequence) for every in-range seed.
    int64_t seed = -1;

    // --- Demetic grouping (scenario assignment) ---
    int demeticGrouping = 0;
    int demeSize = 100;
    double demeticMigProbability = 0.0;

    // --- Entry and wind direction variations ---
    int enableEntryVariations = 0;
    int enableWindVariations = 0;
    // 037 servo v2 — in-FDM servo model master switch (PWM latch + slew).
    // 0/1 like every enable knob; lets the t8 (off, 10 Hz confirm) / t9 (on,
    // servo-v2) A/B pair differ by ini only, no rebuild. The per-scenario
    // craft draws (incl. PWM phase) run regardless so toggling never shifts
    // PRNG draw order.
    int servoModelEnabled = 0;
    int enableRabbitSpeedVariations = 0;
    // 037 T006 -- entry-envelope tighten. The Gaussian draw is clamped to
    // +/-kGaussianSigmaClamp (2.5) sigma (scenario_prng.h), so these sigmas set
    // the HARD limit at 2.5 sigma: cone 18 deg -> 45 deg max half-angle;
    // speed 0.06 -> 15% max delta. Enforces the 45 deg / 15% envelope we agreed
    // (was 30 deg / 10%). Folds into the Phase-A retrain (T026); t6 baseline
    // keeps the old 30/0.1 in its eval inis.
    double entryConeSigma = 18.0;     // degrees: half-angle of nose direction cone (2.5sigma = 45 deg)
    double entryRollSigma = 22.5;     // degrees: roll around body axis
    double entrySpeedSigma = 0.06;    // fraction (2.5sigma = 15% speed delta)
    double windDirectionSigma = 45.0;

    // --- Entry position variations ---
    double entryPositionRadiusSigma = 0.0;
    double entryPositionAltSigma = 0.0;

    // --- 034 US4 craft variations (per-scenario airframe parameter draws) ---
    // EnableCraftVariations is the macro-level disable (parallels
    // EnableEntry/Wind/RabbitSpeed). When 0, the craft-class PRNG is still
    // advanced (draw-and-discard semantics) but the drawn deltas are zeroed
    // before being written into ScenarioMetadata — same dataflow as
    // enableEntry=0. Sigma=0 is a finer-grained no-op on a single axis.
    //
    // Defaults mirror the entry/wind pattern: enable flag = 0 (off until
    // operator opts in via ini) but sigma values are SENSIBLE non-zero
    // numbers so that a missing-from-ini key still produces meaningful
    // diversity instead of a silent no-op. Per-class units:
    //   craftCGSigma         dimensionless (chord-fraction / MAC units —
    //                        not meters: CRRCSim's CG_arm in XML is
    //                        dimensionless, e.g. hb1_streamer = 0.28)
    //   craftDragSigma       fractional multiplier on CD_prof (0.05 = ±5%)
    //   craftTrimSigma       rad — Cm_0 trim Gaussian sigma
    //   craftThrustSigma     fractional multiplier on engine thrust
    //   craftPitchEffSigma   fractional multiplier on Cm_de/CL_de
    //   craftRollEffSigma    fractional multiplier on Cl_da
    //
    // 037 actuator-dynamics craft axes (DYNAMICS, not fractional multipliers
    // like the six above -- each is a nominal physical CENTER plus a Gaussian
    // delta, then clamped to a positive physical range at the draw site so the
    // time constants never go non-positive). These set the per-scenario
    // slew_servo / tau_thrust used by the inputdev_autoc actuator filter
    // (servo slew, thrust lag) instead of the nominal constants:
    //   craftServoSlewSigma  /second  -- servo slew-rate-limit sigma (autoc
    //                        [-1,1] command units/s); center ≈24.2, clamp [16,32]
    //   craftThrustTauSigma  seconds  -- thrust first-order lag time constant
    //                        sigma; center 0.150 s, clamped [0.050, 0.300]
    // (servo first-order tau removed 2026-06-12 — v2 has no lag term.)
    int enableCraftVariations = 0;
    // 038 T001 — member-level hull-crash penalty (M2 safety experiment). When on,
    // a member's per-scenario tracking `score` is multiplied by factor^(#hull-strike
    // scenarios) before fitness/selection, so a crasher's tracking advantage
    // collapses (score is negative-lower-better; ×<1 → toward 0 = worse). Penalizes
    // the score axis only; energy untouched. Default off ⇒ bit-identical to pre-038.
    int enableHullCrashPenalty = 0;
    double hullCrashPenaltyFactor = 0.5;   // rare hull strikes: factor^(K_hull·scale), scale=variationScale ramp (t14 tracker = 0.75)
    double oobCrashPenaltyWeight = 0.0;    // common OOB: smooth decay exp(−weight·scale·K_oob/N), curriculum-ramped, never clamps; 0 = off
    double craftCGSigma = 0.02;        // ~±7% MAC at hb1_streamer CG_arm=0.28
    double craftDragSigma = 0.05;      // ±5% CD_prof
    double craftTrimSigma = 0.02;      // ±1.15° pitch trim
    double craftThrustSigma = 0.05;    // ±5% maxThrust
    double craftPitchEffSigma = 0.05;  // ±5% pitch authority
    double craftRollEffSigma = 0.05;   // ±5% roll authority
    double craftServoSlewSigma = 4.0;    // /s, servo slew sigma (autoc [-1,1] units, center ≈24.2)
    double craftThrustTauSigma = 0.060;  // s, thrust lag tau sigma (center 0.150s)

    // --- Variation landscape ramp ---
    int variationRampStep = 0;

    // --- Selection strategy (015) ---
    std::string selectionMode = "sum";  // "sum", "minimax", "lexicase"
    // 035 FR-003 — lexicase epsilon scaling: "constant" (legacy fixed 0.5) or
    // "mad" (MAD-relative per-scenario). Declared now; MAD impl lands in US1.
    std::string lexicaseEpsilonMode = "constant";

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
    // S3 key form: "<run-id>/gen<N>.dmp" (resolved against `trackerSourceBucket`),
    // or absolute / relative local file path for offline-test convenience.
    std::string trackerSourceRun;
    // Bucket holding the source dmp. 035: M2 tracks an M1 run, so the source lives
    // in autoc-m1 while M2 output goes to s3Bucket (autoc-m2). Required in tracker
    // mode; NO fallback to s3Bucket (M2-era no-silent-default policy).
    std::string trackerSourceBucket;
    // Comma-separated path-variant indices (e.g. "0,1,2,3,4,5"). Empty = all.
    std::string trackerPathSubset;
    // Comma-separated wind-variant indices (e.g. "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19").
    // Empty = all. Cross-product with TrackerPathSubset per FR-011.
    std::string trackerWindSubset;

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
    // probability ⇒ deterministic per (scenario, gen) given the scenarioSeed-derived rabbit-class PRNG.
    // 0.10 default = ~"50% chance of dying within 7 ticks (~0.7s) inside
    // hull" → strong incentive to keep target outside the 1m sphere.
    double crashHullProbability = 0.10;

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

    // === 032 PHASE 1 — DERIVED PERCEPTUAL FEATURES =====================
    // [DerivedFeatures] section in autoc-tracker.ini. Inert in pathgen mode
    // (no derived-feature slots in PathgenInputs). Per spec.md Q4 +
    // contracts/ini_schema.md.
    //
    // CepGateThreshold: substitute neutral values for derived features
    // when EITHER beacon's CEP at the current tick is >= this. Default
    // matches kCepSentinelThreshold (1.25) from camera_projection.h.
    double cepGateThreshold = 1.25;  // raw-ok: ini-loaded config-struct field — inih::GetReal returns double; cast to float at the eval-pipeline consumption boundary
};

// 034 FR-010 — single source of truth for config parse + startup print.
// X(type, field, "IniKey") for every AutocConfig field that maps 1:1 to an
// ini key via inih (int/unsigned short → GetInteger, double → GetReal,
// std::string → Get). config.cc expands this to parse and autoc.cc expands it
// to print, so parse and the startup banner CANNOT drift — adding a knob here
// auto-parses AND auto-prints with no hand-maintained print line. Defaults +
// doc comments live on the struct above; post-parse validation (cepGateThreshold
// range, Mode enum) stays explicit in config.cc after the macro expansion.
// Order mirrors the struct sections for readability.
#define AUTOC_CONFIG_FIELDS(X) \
    X(int,            populationSize,            "PopulationSize") \
    X(int,            numberOfGenerations,       "NumberOfGenerations") \
    X(double,         crossoverProbability,      "CrossoverProbability") \
    X(double,         creationProbability,       "CreationProbability") \
    X(int,            tournamentSize,            "TournamentSize") \
    X(double,         swapMutationProbability,   "SwapMutationProbability") \
    X(int,            addBestToNewPopulation,    "AddBestToNewPopulation") \
    X(double,         nnMutationSigma,           "NNMutationSigma") \
    X(double,         nnCrossoverAlpha,          "NNCrossoverAlpha") \
    X(double,         nnSigmaFloor,              "NNSigmaFloor") \
    X(std::string,    nnWeightFile,              "NNWeightFile") \
    X(std::string,    nnInitMethod,              "NNInitMethod") \
    X(int,            controlIntervalMsec,       "ControlIntervalMsec") \
    X(int,            simNumPathsPerGen,         "SimNumPathsPerGeneration") \
    X(std::string,    generatorMethod,           "PathGeneratorMethod") \
    X(int,            evalThreads,               "EvalThreads") \
    X(std::string,    workerProgram,             "WorkerProgram") \
    X(unsigned short, workerPortOverride,        "WorkerPortOverride") \
    X(std::string,    s3Bucket,                  "S3Bucket") \
    X(std::string,    s3Profile,                 "S3Profile") \
    X(int,            evaluateMode,              "EvaluateMode") \
    X(int,            windScenarioCount,         "WindScenarios") \
    X(int,            randomPathSeedB,           "RandomPathSeedB") \
    X(int64_t,        seed,                      "Seed") \
    X(int,            demeticGrouping,           "DemeticGrouping") \
    X(int,            demeSize,                  "DemeSize") \
    X(double,         demeticMigProbability,     "DemeticMigProbability") \
    X(int,            enableEntryVariations,     "EnableEntryVariations") \
    X(int,            enableWindVariations,      "EnableWindVariations") \
    X(int,            servoModelEnabled,         "ServoModelEnabled") \
    X(int,            enableRabbitSpeedVariations, "EnableRabbitSpeedVariations") \
    X(double,         entryConeSigma,            "EntryConeSigma") \
    X(double,         entryRollSigma,            "EntryRollSigma") \
    X(double,         entrySpeedSigma,           "EntrySpeedSigma") \
    X(double,         windDirectionSigma,        "WindDirectionSigma") \
    X(double,         entryPositionRadiusSigma,  "EntryPositionRadiusSigma") \
    X(double,         entryPositionAltSigma,     "EntryPositionAltSigma") \
    X(int,            enableCraftVariations,     "EnableCraftVariations") \
    X(int,            enableHullCrashPenalty,    "EnableHullCrashPenalty") \
    X(double,         hullCrashPenaltyFactor,    "HullCrashPenaltyFactor") \
    X(double,         oobCrashPenaltyWeight,     "OobCrashPenaltyWeight") \
    X(double,         craftCGSigma,              "CraftCGSigma") \
    X(double,         craftDragSigma,            "CraftDragSigma") \
    X(double,         craftTrimSigma,            "CraftTrimSigma") \
    X(double,         craftThrustSigma,          "CraftThrustSigma") \
    X(double,         craftPitchEffSigma,        "CraftPitchEffSigma") \
    X(double,         craftRollEffSigma,         "CraftRollEffSigma") \
    X(double,         craftServoSlewSigma,       "CraftServoSlewSigma") \
    X(double,         craftThrustTauSigma,       "CraftThrustTauSigma") \
    X(int,            variationRampStep,         "VariationRampStep") \
    X(std::string,    selectionMode,             "SelectionMode") \
    X(std::string,    lexicaseEpsilonMode,       "LexicaseEpsilonMode") \
    X(double,         fitDistScaleBehind,        "FitDistScaleBehind") \
    X(double,         fitDistScaleAhead,         "FitDistScaleAhead") \
    X(double,         fitConeAngleDeg,           "FitConeAngleDeg") \
    X(double,         fitStreakThreshold,        "FitStreakThreshold") \
    X(double,         fitStreakRampSec,          "FitStreakRampSec") \
    X(double,         fitStreakMultiplierMax,    "FitStreakMultiplierMax") \
    X(double,         rabbitSpeedNominal,        "RabbitSpeedNominal") \
    X(double,         rabbitSpeedSigma,          "RabbitSpeedSigma") \
    X(double,         rabbitSpeedMin,            "RabbitSpeedMin") \
    X(double,         rabbitSpeedMax,            "RabbitSpeedMax") \
    X(double,         rabbitSpeedCycleMin,       "RabbitSpeedCycleMin") \
    X(double,         rabbitSpeedCycleMax,       "RabbitSpeedCycleMax") \
    X(std::string,    mode,                      "Mode") \
    X(std::string,    trackerSourceRun,          "TrackerSourceRun") \
    X(std::string,    trackerSourceBucket,       "TrackerSourceBucket") \
    X(std::string,    trackerPathSubset,         "TrackerPathSubset") \
    X(std::string,    trackerWindSubset,         "TrackerWindSubset") \
    X(double,         trailDistance,             "TrailDistance") \
    X(double,         lowSpeedTrailThreshold,    "LowSpeedTrailThreshold") \
    X(double,         lowSpeedTrailHysteresis,   "LowSpeedTrailHysteresis") \
    X(std::string,    crashHullShape,            "CrashHullShape") \
    X(double,         crashHullRadius,           "CrashHullRadius") \
    X(double,         crashHullProbability,      "CrashHullProbability") \
    X(double,         flightArenaRadius,         "FlightArenaRadius") \
    X(double,         flightArenaFloorAGL,       "FlightArenaFloorAGL") \
    X(double,         flightArenaCeilingAGL,     "FlightArenaCeilingAGL") \
    X(int,            cameraCount,               "CameraCount") \
    X(double,         cameraFOVHorizontalDeg,    "CameraFOVHorizontalDeg") \
    X(double,         cameraFOVVerticalDeg,      "CameraFOVVerticalDeg") \
    X(double,         cameraFrameRateHz,         "CameraFrameRateHz") \
    X(double,         cameraLatencyMs,           "CameraLatencyMs") \
    X(double,         cameraMountOffsetX,        "CameraMountOffsetX") \
    X(double,         cameraMountOffsetY,        "CameraMountOffsetY") \
    X(double,         cameraMountOffsetZ,        "CameraMountOffsetZ") \
    X(int,            beaconLeftWavelengthNm,    "BeaconLeftWavelengthNm") \
    X(int,            beaconRightWavelengthNm,   "BeaconRightWavelengthNm") \
    X(double,         beaconEmissionConeDeg,     "BeaconEmissionConeDeg") \
    X(double,         beaconLeftMountX,          "BeaconLeftMountX") \
    X(double,         beaconLeftMountY,          "BeaconLeftMountY") \
    X(double,         beaconLeftMountZ,          "BeaconLeftMountZ") \
    X(double,         beaconRightMountX,         "BeaconRightMountX") \
    X(double,         beaconRightMountY,         "BeaconRightMountY") \
    X(double,         beaconRightMountZ,         "BeaconRightMountZ") \
    X(double,         cepGateThreshold,          "CepGateThreshold")

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

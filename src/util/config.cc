#include "autoc/util/config.h"
#include <fstream>
#include <cstdlib>
#include <INIReader.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/client/ClientConfiguration.h>

// Static member definitions
AutocConfig* ConfigManager::config = nullptr;
bool ConfigManager::initialized = false;

void ConfigManager::initialize(const std::string& filename, std::ostream& out) {
    if (initialized) {
        return;
    }

    std::ifstream configFile(filename);
    if (!configFile.good()) {
        out << "FATAL ERROR: Configuration file '" << filename << "' not found!" << std::endl;
        exit(1);
    }
    configFile.close();

    INIReader reader(filename);
    if (reader.ParseError() != 0) {
        out << "FATAL ERROR: Cannot parse configuration file '" << filename << "'" << std::endl;
        exit(1);
    }

    config = new AutocConfig();

    // Evolution
    config->populationSize = reader.GetInteger("", "PopulationSize", config->populationSize);
    config->numberOfGenerations = reader.GetInteger("", "NumberOfGenerations", config->numberOfGenerations);
    config->crossoverProbability = reader.GetReal("", "CrossoverProbability", config->crossoverProbability);
    config->creationProbability = reader.GetReal("", "CreationProbability", config->creationProbability);
    config->tournamentSize = reader.GetInteger("", "TournamentSize", config->tournamentSize);
    config->swapMutationProbability = reader.GetReal("", "SwapMutationProbability", config->swapMutationProbability);
    config->addBestToNewPopulation = reader.GetInteger("", "AddBestToNewPopulation", config->addBestToNewPopulation);

    // NN-specific
    config->nnMutationSigma = reader.GetReal("", "NNMutationSigma", config->nnMutationSigma);
    config->nnCrossoverAlpha = reader.GetReal("", "NNCrossoverAlpha", config->nnCrossoverAlpha);
    config->nnWeightFile = reader.Get("", "NNWeightFile", config->nnWeightFile);
    config->nnInitMethod = reader.Get("", "NNInitMethod", config->nnInitMethod);

    // Simulation
    config->simNumPathsPerGen = reader.GetInteger("", "SimNumPathsPerGeneration", config->simNumPathsPerGen);
    config->evalThreads = reader.GetInteger("", "EvalThreads", config->evalThreads);
    config->generatorMethod = reader.Get("", "PathGeneratorMethod", config->generatorMethod);
    config->minisimProgram = reader.Get("", "MinisimProgram", config->minisimProgram);
    config->minisimPortOverride = static_cast<unsigned short>(reader.GetInteger("", "MinisimPortOverride", config->minisimPortOverride));

    // S3
    config->s3Bucket = reader.Get("", "S3Bucket", config->s3Bucket);
    config->s3Profile = reader.Get("", "S3Profile", config->s3Profile);

    // Eval mode
    config->evaluateMode = reader.GetInteger("", "EvaluateMode", config->evaluateMode);

    // Scenarios
    config->windScenarioCount = reader.GetInteger("", "WindScenarios", config->windScenarioCount);
    config->randomPathSeedB = reader.GetInteger("", "RandomPathSeedB", config->randomPathSeedB);
    config->seed = reader.GetInteger("", "Seed", config->seed);

    // Selection strategy (015)
    config->selectionMode = reader.Get("", "SelectionMode", config->selectionMode);
    config->nnSigmaFloor = reader.GetReal("", "NNSigmaFloor", config->nnSigmaFloor);

    // Demetic grouping (scenario assignment)
    config->demeticGrouping = reader.GetInteger("", "DemeticGrouping", config->demeticGrouping);
    config->demeSize = reader.GetInteger("", "DemeSize", config->demeSize);
    config->demeticMigProbability = reader.GetReal("", "DemeticMigProbability", config->demeticMigProbability);

    // Entry and wind direction variations
    config->enableEntryVariations = reader.GetInteger("", "EnableEntryVariations", config->enableEntryVariations);
    config->enableWindVariations = reader.GetInteger("", "EnableWindVariations", config->enableWindVariations);
    config->enableRabbitSpeedVariations = reader.GetInteger("", "EnableRabbitSpeedVariations", config->enableRabbitSpeedVariations);
    config->entryConeSigma = reader.GetReal("", "EntryConeSigma", config->entryConeSigma);
    config->entryRollSigma = reader.GetReal("", "EntryRollSigma", config->entryRollSigma);
    config->entrySpeedSigma = reader.GetReal("", "EntrySpeedSigma", config->entrySpeedSigma);
    config->windDirectionSigma = reader.GetReal("", "WindDirectionSigma", config->windDirectionSigma);

    // Entry position variations
    config->entryPositionRadiusSigma = reader.GetReal("", "EntryPositionRadiusSigma", config->entryPositionRadiusSigma);
    config->entryPositionAltSigma = reader.GetReal("", "EntryPositionAltSigma", config->entryPositionAltSigma);

    // Variation landscape ramp
    config->variationRampStep = reader.GetInteger("", "VariationRampStep", config->variationRampStep);

    // Fitness: point-accumulation scoring (022, conical surface V4)
    config->fitDistScaleBehind = reader.GetReal("", "FitDistScaleBehind", config->fitDistScaleBehind);
    config->fitDistScaleAhead = reader.GetReal("", "FitDistScaleAhead", config->fitDistScaleAhead);
    config->fitConeAngleDeg = reader.GetReal("", "FitConeAngleDeg", config->fitConeAngleDeg);
    config->fitStreakThreshold = reader.GetReal("", "FitStreakThreshold", config->fitStreakThreshold);
    config->fitStreakRampSec = reader.GetReal("", "FitStreakRampSec", config->fitStreakRampSec);
    config->fitStreakMultiplierMax = reader.GetReal("", "FitStreakMultiplierMax", config->fitStreakMultiplierMax);

    // Variable rabbit speed
    config->rabbitSpeedNominal = reader.GetReal("", "RabbitSpeedNominal", config->rabbitSpeedNominal);
    config->rabbitSpeedSigma = reader.GetReal("", "RabbitSpeedSigma", config->rabbitSpeedSigma);
    config->rabbitSpeedMin = reader.GetReal("", "RabbitSpeedMin", config->rabbitSpeedMin);
    config->rabbitSpeedMax = reader.GetReal("", "RabbitSpeedMax", config->rabbitSpeedMax);
    config->rabbitSpeedCycleMin = reader.GetReal("", "RabbitSpeedCycleMin", config->rabbitSpeedCycleMin);
    config->rabbitSpeedCycleMax = reader.GetReal("", "RabbitSpeedCycleMax", config->rabbitSpeedCycleMax);

    // === 030 TRACKER MODE (FR-011 + FR-019) =============================
    config->mode = reader.Get("", "Mode", config->mode);

    // Tracker source dmp + scenario subset
    config->trackerSourceRun = reader.Get("", "TrackerSourceRun", config->trackerSourceRun);
    config->trackerPathSubset = reader.Get("", "TrackerPathSubset", config->trackerPathSubset);
    config->trackerWindSubset = reader.Get("", "TrackerWindSubset", config->trackerWindSubset);

    // Tracker fitness: trail rabbit
    config->trailDistance = reader.GetReal("", "TrailDistance", config->trailDistance);
    config->lowSpeedTrailThreshold = reader.GetReal("", "LowSpeedTrailThreshold", config->lowSpeedTrailThreshold);
    config->lowSpeedTrailHysteresis = reader.GetReal("", "LowSpeedTrailHysteresis", config->lowSpeedTrailHysteresis);

    // Tracker fitness: crash hull
    config->crashHullShape = reader.Get("", "CrashHullShape", config->crashHullShape);
    config->crashHullRadius = reader.GetReal("", "CrashHullRadius", config->crashHullRadius);
    config->crashHullProbability = reader.GetReal("", "CrashHullProbability", config->crashHullProbability);

    // Tracker arena
    config->flightArenaRadius = reader.GetReal("", "FlightArenaRadius", config->flightArenaRadius);
    config->flightArenaFloorAGL = reader.GetReal("", "FlightArenaFloorAGL", config->flightArenaFloorAGL);
    config->flightArenaCeilingAGL = reader.GetReal("", "FlightArenaCeilingAGL", config->flightArenaCeilingAGL);

    // Camera config
    config->cameraCount = static_cast<int>(reader.GetInteger("", "CameraCount", config->cameraCount));
    config->cameraFOVHorizontalDeg = reader.GetReal("", "CameraFOVHorizontalDeg", config->cameraFOVHorizontalDeg);
    config->cameraFOVVerticalDeg = reader.GetReal("", "CameraFOVVerticalDeg", config->cameraFOVVerticalDeg);
    config->cameraFrameRateHz = reader.GetReal("", "CameraFrameRateHz", config->cameraFrameRateHz);
    config->cameraLatencyMs = reader.GetReal("", "CameraLatencyMs", config->cameraLatencyMs);
    config->cameraMountOffsetX = reader.GetReal("", "CameraMountOffsetX", config->cameraMountOffsetX);
    config->cameraMountOffsetY = reader.GetReal("", "CameraMountOffsetY", config->cameraMountOffsetY);
    config->cameraMountOffsetZ = reader.GetReal("", "CameraMountOffsetZ", config->cameraMountOffsetZ);

    // Beacon config
    config->beaconLeftWavelengthNm = static_cast<int>(reader.GetInteger("", "BeaconLeftWavelengthNm", config->beaconLeftWavelengthNm));
    config->beaconRightWavelengthNm = static_cast<int>(reader.GetInteger("", "BeaconRightWavelengthNm", config->beaconRightWavelengthNm));
    config->beaconEmissionConeDeg = reader.GetReal("", "BeaconEmissionConeDeg", config->beaconEmissionConeDeg);
    config->beaconLeftMountX = reader.GetReal("", "BeaconLeftMountX", config->beaconLeftMountX);
    config->beaconLeftMountY = reader.GetReal("", "BeaconLeftMountY", config->beaconLeftMountY);
    config->beaconLeftMountZ = reader.GetReal("", "BeaconLeftMountZ", config->beaconLeftMountZ);
    config->beaconRightMountX = reader.GetReal("", "BeaconRightMountX", config->beaconRightMountX);
    config->beaconRightMountY = reader.GetReal("", "BeaconRightMountY", config->beaconRightMountY);
    config->beaconRightMountZ = reader.GetReal("", "BeaconRightMountZ", config->beaconRightMountZ);

    // === 032 PHASE 1 — DERIVED PERCEPTUAL FEATURES =====================
    // [DerivedFeatures] section. Loud-fail on out-of-range threshold.
    config->cepGateThreshold = reader.GetReal("DerivedFeatures", "CepGateThreshold", config->cepGateThreshold);
    if (config->cepGateThreshold < 0.0 || config->cepGateThreshold > 2.0) {
        out << "FATAL ERROR: [DerivedFeatures] CepGateThreshold = "
            << config->cepGateThreshold
            << " is out of range [0.0, 2.0]. Defaults to 1.25 (matches "
            << "kCepSentinelThreshold). Per specs/032-tracker-nn-enhancements"
            << "/contracts/ini_schema.md." << std::endl;
        exit(1);
    }

    // === Mode validation (loud-fail per FR-011 mutual-exclusion) =======
    // Pathgen-mode: anything goes; tracker-* fields are inert defaults.
    // Tracker-mode: TrackerSourceRun is required; loud-fail if missing.
    // Strict mutual-exclusion of pathgen-only fields in tracker mode is
    // soft for v1 (warn on misconfig rather than reject) — the inih
    // reader doesn't expose "was this key in the file vs defaulted",
    // and most pathgen fields (RabbitSpeed*, Selection*, Demetic*) are
    // shared across modes anyway.
    if (config->mode != "pathgen" && config->mode != "tracker") {
        out << "FATAL ERROR: Mode = '" << config->mode
            << "' invalid; must be 'pathgen' or 'tracker'" << std::endl;
        exit(1);
    }
    if (config->mode == "tracker" && config->trackerSourceRun.empty()) {
        out << "FATAL ERROR: Mode = tracker requires TrackerSourceRun "
            << "(S3 key 'autoc-storage/<run-id>/gen<N>.dmp' or local path)"
            << std::endl;
        exit(1);
    }

    // Print S3 configuration
    if (config->s3Profile != "default") {
        out << "S3 Configuration: Using MinIO S3 (profile: " << config->s3Profile << ", bucket: " << config->s3Bucket << ")" << std::endl;
    } else {
        out << "S3 Configuration: Using AWS S3 (profile: " << config->s3Profile << ", bucket: " << config->s3Bucket << ")" << std::endl;
    }

    initialized = true;
}

AutocConfig& ConfigManager::getConfig() {
    if (!initialized) {
        std::cerr << "Error: ConfigManager not initialized. Call ConfigManager::initialize() first." << std::endl;
        static AutocConfig defaultConfig;
        return defaultConfig;
    }
    return *config;
}

bool ConfigManager::isInitialized() {
    return initialized;
}

std::shared_ptr<Aws::S3::S3Client> ConfigManager::getS3Client() {
    if (!initialized) {
        std::cerr << "Error: ConfigManager not initialized. Call ConfigManager::initialize() first." << std::endl;
        return nullptr;
    }

    const AutocConfig& cfg = getConfig();

    Aws::Client::ClientConfiguration clientConfig;
    Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy policy = Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::RequestDependent;

    if (cfg.s3Profile != "default") {
        clientConfig.endpointOverride = "http://localhost:9000";
        clientConfig.scheme = Aws::Http::Scheme::HTTP;
        clientConfig.verifySSL = false;
        policy = Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::Never;
    }

    auto credentialsProvider = Aws::MakeShared<Aws::Auth::ProfileConfigFileAWSCredentialsProvider>(
        "CredentialsProvider", cfg.s3Profile.c_str());

    return Aws::MakeShared<Aws::S3::S3Client>("S3Client",
        credentialsProvider,
        clientConfig,
        policy,
        false
    );
}

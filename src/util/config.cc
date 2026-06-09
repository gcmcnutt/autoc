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

namespace {
// 034 FR-010 — type-dispatched ini read used by the AUTOC_CONFIG_FIELDS macro
// expansion. Byte-identical to the prior hand-written reader calls:
// int / unsigned short → GetInteger, double → GetReal, std::string → Get.
template <typename T> T readIniField(INIReader& r, const char* key, const T& def);
template <> int readIniField<int>(INIReader& r, const char* key, const int& def) {
    return static_cast<int>(r.GetInteger("", key, def));
}
template <> int64_t readIniField<int64_t>(INIReader& r, const char* key, const int64_t& def) {
    // inih GetInteger parses via strtol → returns long (64-bit here), so a
    // pasted-back effectiveMasterSeed up to LONG_MAX round-trips losslessly.
    return static_cast<int64_t>(r.GetInteger("", key, static_cast<long>(def)));
}
template <> unsigned short readIniField<unsigned short>(INIReader& r, const char* key, const unsigned short& def) {
    return static_cast<unsigned short>(r.GetInteger("", key, def));
}
template <> double readIniField<double>(INIReader& r, const char* key, const double& def) {
    return r.GetReal("", key, def);
}
template <> std::string readIniField<std::string>(INIReader& r, const char* key, const std::string& def) {
    return r.Get("", key, def);
}
}  // namespace

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

    // 034 FR-010 — parse every field from the single-source AUTOC_CONFIG_FIELDS
    // macro (config.h). Byte-identical to the prior hand-written reader calls;
    // adding a knob to the macro auto-parses here AND auto-prints in autoc.cc.
    // Field-specific validation (CepGateThreshold range, Mode enum) stays below.
#define X(type, field, key) config->field = readIniField<type>(reader, key, config->field);
    AUTOC_CONFIG_FIELDS(X)
#undef X

    // === 032 PHASE 1 — CepGateThreshold range validation ===============
    // (Parsed above via the macro; loud-fail on out-of-range here.)
    if (config->cepGateThreshold < 0.0 || config->cepGateThreshold > 2.0) {
        out << "FATAL ERROR: CepGateThreshold = "
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
            << "(S3 key '<run-id>/gen<N>.dmp[.zst]' or local path)"
            << std::endl;
        exit(1);
    }
    // 035: source bucket is explicit (M2 source = an M1 run in autoc-m1, distinct
    // from s3Bucket=autoc-m2 output). No silent fallback to s3Bucket.
    if (config->mode == "tracker" && config->trackerSourceBucket.empty()) {
        out << "FATAL ERROR: Mode = tracker requires TrackerSourceBucket "
            << "(bucket holding TrackerSourceRun, e.g. autoc-m1; no s3Bucket fallback)"
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

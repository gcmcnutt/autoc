
// autoc.cc

/* -------------------------------------------------------------------
From skeleton/skeleton.cc
------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cstdlib>
#include <math.h>
#include <new>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <thread>
#include <chrono>
#include <memory>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <random>
#include <unistd.h>
#include <cstring>
#include <mutex>

#include "autoc/rpc/protocol.h"
#include "autoc/util/threadpool.h"
#include "autoc/autoc.h"
#include "autoc/util/logger.h"
#include "autoc/eval/pathgen.h"
#include "autoc/util/config.h"
#include "autoc/eval/variation_generator.h"
#include "autoc/nn/population.h"
#include "autoc/nn/serialization.h"
#include "autoc/nn/evaluator.h"
#include "autoc/eval/fitness_computer.h"
#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/selection.h"

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/client/ClientConfiguration.h>

#include <cereal/archives/binary.hpp>

using namespace std;

std::vector<std::vector<Path>> generationPaths;
std::vector<ScenarioDescriptor> generationScenarios;

// VARIATIONS1: Global sigma parameters, initialized at startup from config
static VariationSigmas gVariationSigmas = {0.0, 0.0, 0.0, 0.0, 0.0};
// Individual variation enable flags (from config)
static bool gEnableEntryVariations = false;
static bool gEnableWindVariations = false;

// Variable rabbit speed: Global config, initialized at startup from config
static RabbitSpeedConfig gRabbitSpeedConfig = RabbitSpeedConfig::defaultConfig();

// RAMP_LANDSCAPE: Gradual variation scaling (see specs/RAMP_LANDSCAPE.md)
static int gCurrentGeneration = 0;      // Updated at start of each generation
static int gTotalGenerations = 1;       // Set from config at startup
static int gVariationRampStep = 0;      // Set from config at startup (0 = disabled)

/**
 * Compute variation scale for current generation.
 * Scale ramps from 0.0 to 1.0 over the course of training.
 *
 * @return Scale factor to apply to variation offsets (0.0 to 1.0)
 */
static double computeVariationScale() {
    // Ramp from 0.0 (no variations) to 1.0 (full variations) over training.
    // Disabled (rampStep<=0), eval mode (totalGens<=1), or trivial: return 1.0.
    int numSteps = (gVariationRampStep > 0) ? gTotalGenerations / gVariationRampStep : 0;
    if (numSteps <= 1) return 1.0;

    int stepIndex = (gCurrentGeneration - 1) / gVariationRampStep;  // 1-based gen
    return static_cast<double>(std::min(stepIndex, numSteps - 1)) / static_cast<double>(numSteps - 1);
}

// ============================================================================
// Intercept-Budget Fitness Scaling (see specs/005-entry-fitness-ramp)
// ============================================================================

// ============================================================================
// Single PRNG Architecture: Pre-fetched scenario variations
// See specs/SINGLE_PRNG.md for design
// ============================================================================

/**
 * Pre-computed variations for a single scenario (wind variant).
 * Generated from GPrand() at startup, reused every generation.
 */
struct ScenarioVariations {
    unsigned int windSeed;                      // Seed for crrcsim CRRC_Random
    unsigned int rabbitSpeedSeed;               // Seed for local rabbit speed profile PRNG
    VariationOffsets entryOffsets;              // Heading, roll, pitch, speed, windDir
};

// Global pre-computed table (indexed by wind scenario index 0..N-1)
static std::vector<ScenarioVariations> gScenarioVariations;
static unsigned int gPathSeed = 0;        // Derived from GPrand() or RandomPathSeedB override
static bool gPathSeedFromOverride = false;// True if RandomPathSeedB was used

/**
 * Pre-fetch all scenario variations from rng at startup.
 * Called once before evolution begins.
 *
 * When a variation type is disabled, we still consume GPrand values to maintain
 * deterministic PRNG sequence, but store defaults in the table instead.
 * This way the table contains the actual values to send to sims - no filtering needed.
 *
 * @param numScenarios       Number of wind scenarios (windScenarioCount from config)
 * @param sigmas             Variation sigma parameters
 * @param rabbitCfg          Rabbit speed configuration
 * @param randomPathSeedB    Override path seed (-1 = derive from GPrand)
 * @param enableEntry        If false, store default entry offsets (0.0, 1.0 for speed)
 * @param enableWind         If false, store default wind offset (0.0)
 */
static void prefetchAllVariations(int numScenarios, const VariationSigmas& sigmas,
                                   const RabbitSpeedConfig& rabbitCfg, int randomPathSeedB,
                                   bool enableEntry, bool enableWind) {
    gScenarioVariations.clear();
    gScenarioVariations.reserve(numScenarios);

    // Derive pathSeed (unless overridden by config)
    if (randomPathSeedB == -1) {
        gPathSeed = static_cast<unsigned int>(rng::randLong());
        gPathSeedFromOverride = false;
    } else {
        gPathSeed = static_cast<unsigned int>(randomPathSeedB);
        gPathSeedFromOverride = true;
    }

    double totalDurationSec = SIM_TOTAL_TIME_MSEC / 1000.0;

    // When wind variations disabled, use same seed for all scenarios (identical thermals/gusts)
    // Still consume GPrand to maintain deterministic sequence
    unsigned int baseWindSeed = static_cast<unsigned int>(rng::randLong());

    for (int i = 0; i < numScenarios; i++) {
        ScenarioVariations sv;

        // Wind seed for crrcsim: unique per scenario if enabled, same for all if disabled
        if (enableWind) {
            sv.windSeed = (i == 0) ? baseWindSeed : static_cast<unsigned int>(rng::randLong());
        } else {
            // Disabled: all scenarios use same seed (identical thermals/gusts)
            // Still consume GPrand to keep PRNG sequence deterministic
            if (i > 0) { (void)rng::randLong(); }
            sv.windSeed = baseWindSeed;
        }

        // Entry/wind variations: always consume GPrand to maintain deterministic sequence,
        // but store defaults if that variation type is disabled
        VariationOffsets generated = generateVariationsFromGPrand(sigmas);

        if (enableEntry) {
            sv.entryOffsets.entryHeadingOffset = generated.entryHeadingOffset;
            sv.entryOffsets.entryRollOffset = generated.entryRollOffset;
            sv.entryOffsets.entryPitchOffset = generated.entryPitchOffset;
            sv.entryOffsets.entrySpeedFactor = generated.entrySpeedFactor;
            sv.entryOffsets.entryNorthOffset = generated.entryNorthOffset;
            sv.entryOffsets.entryEastOffset = generated.entryEastOffset;
            sv.entryOffsets.entryAltOffset = generated.entryAltOffset;
        } else {
            // Defaults: no offset
            sv.entryOffsets.entryHeadingOffset = 0.0;
            sv.entryOffsets.entryRollOffset = 0.0;
            sv.entryOffsets.entryPitchOffset = 0.0;
            sv.entryOffsets.entrySpeedFactor = 1.0;
            sv.entryOffsets.entryNorthOffset = 0.0;
            sv.entryOffsets.entryEastOffset = 0.0;
            sv.entryOffsets.entryAltOffset = 0.0;
        }

        if (enableWind) {
            sv.entryOffsets.windDirectionOffset = generated.windDirectionOffset;
        } else {
            // Default: no offset
            sv.entryOffsets.windDirectionOffset = 0.0;
        }

        // Rabbit speed: derive a per-scenario seed (profile generated locally by sim)
        sv.rabbitSpeedSeed = static_cast<unsigned int>(rng::randLong());

        gScenarioVariations.push_back(std::move(sv));
    }
}

/**
 * Log pre-fetched variations at startup for verification.
 * Format matches spec in SINGLE_PRNG.md.
 */
static void logPrefetchedVariations(int numScenarios, long seed) {
    *logger.info() << endl;
    *logger.info() << "=== Pre-fetched Scenario Variations (Seed=" << seed << ") ===" << endl;
    *logger.info() << "PathSeed: " << gPathSeed
                   << " (override: " << (gPathSeedFromOverride ? "yes" : "no")
                   << ")" << endl;
    *logger.info() << "Scenarios: " << numScenarios << endl;
    *logger.info() << endl;

    *logger.info() << "Scenario  WindSeed    RabbitSeed  Heading°   Roll°   Pitch°  Speed%  WindDir°  North°  East°  Down°" << endl;
    *logger.info() << "--------  ----------  ----------  --------  ------  ------  ------  --------  ------  -----  -----" << endl;

    for (int i = 0; i < numScenarios; i++) {
        const auto& sv = gScenarioVariations[i];

        std::ostringstream line;
        line << std::setw(4) << i << "      "
             << "0x" << std::hex << std::setw(8) << std::setfill('0') << sv.windSeed
             << "  0x" << std::setw(8) << std::setfill('0') << sv.rabbitSpeedSeed
             << std::dec << std::setfill(' ')
             << "  " << std::setw(7) << std::fixed << std::setprecision(2) << radToDeg(sv.entryOffsets.entryHeadingOffset)
             << "  " << std::setw(6) << radToDeg(sv.entryOffsets.entryRollOffset)
             << "  " << std::setw(6) << radToDeg(sv.entryOffsets.entryPitchOffset)
             << "  " << std::setw(5) << std::setprecision(1) << ((sv.entryOffsets.entrySpeedFactor - 1.0) * 100) << "%"
             << "  " << std::setw(7) << std::setprecision(2) << radToDeg(sv.entryOffsets.windDirectionOffset);

        // Position offsets columns
        const auto& o = sv.entryOffsets;
        line << "    " << std::setprecision(1)
             << std::setw(6) << o.entryNorthOffset
             << std::setw(7) << o.entryEastOffset
             << std::setw(7) << o.entryAltOffset;
        *logger.info() << line.str() << endl;
    }
    *logger.info() << endl;
}

// Helper to populate variation offsets in ScenarioMetadata from pre-fetched table
// Table already contains correct values: defaults when disabled, variations when enabled
// (filtering is done at prefetch time, not here)
// RAMP_LANDSCAPE: Offsets are scaled by computeVariationScale() for gradual introduction
static void populateVariationOffsets(ScenarioMetadata& meta) {
  int windIdx = meta.windVariantIndex;
  if (windIdx >= 0 && windIdx < static_cast<int>(gScenarioVariations.size())) {
    const auto& v = gScenarioVariations[windIdx].entryOffsets;

    // Get current scale (0.0 to 1.0 over training)
    double scale = computeVariationScale();

    // Angular offsets: scale toward 0
    meta.entryHeadingOffset = v.entryHeadingOffset * scale;
    meta.entryRollOffset = v.entryRollOffset * scale;
    meta.entryPitchOffset = v.entryPitchOffset * scale;
    meta.windDirectionOffset = v.windDirectionOffset * scale;

    // Speed factor: interpolate from 1.0 toward pre-computed value
    // At scale=0: factor=1.0 (nominal), at scale=1: factor=pre-computed
    meta.entrySpeedFactor = 1.0 + scale * (v.entrySpeedFactor - 1.0);

    // Position offsets: scale and clamp to safe arena bounds
    // RAMP_LANDSCAPE scaling applies here too (gradual introduction)
    double northScaled = v.entryNorthOffset * scale;
    double eastScaled = v.entryEastOffset * scale;
    double altScaled = v.entryAltOffset * scale;

    // Clamp horizontal to safe radius
    double horizDist = sqrt(northScaled * northScaled + eastScaled * eastScaled);
    if (horizDist > ENTRY_SAFE_RADIUS) {
      double clampFactor = ENTRY_SAFE_RADIUS / horizDist;
      northScaled *= clampFactor;
      eastScaled *= clampFactor;
    }

    // Clamp altitude to safe NED bounds (ENTRY_SAFE_ALT_MAX < ENTRY_SAFE_ALT_MIN since NED Down)
    if (altScaled < ENTRY_SAFE_ALT_MAX) altScaled = ENTRY_SAFE_ALT_MAX;
    if (altScaled > ENTRY_SAFE_ALT_MIN) altScaled = ENTRY_SAFE_ALT_MIN;

    meta.entryNorthOffset = northScaled;
    meta.entryEastOffset = eastScaled;
    meta.entryAltOffset = altScaled;

    // Rabbit speed seed for local profile generation (sigma=0 → constant speed automatically)
    meta.rabbitSpeedSeed = gScenarioVariations[windIdx].rabbitSpeedSeed;
  }
  // If windIdx out of range, leave defaults (shouldn't happen)
}

// Rabbit speed is now applied at runtime via odometer advancement in minisim.
// The applySpeedProfileToPath function has been removed as part of the
// odometer-based path traversal refactor.
std::atomic_ulong nanDetector = 0;
std::ofstream fout;
std::ofstream bout;
std::atomic<uint64_t> globalScenarioCounter{0};
std::atomic<uint64_t> globalSimRunCounter{0};

static bool bitwiseEqual(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

ThreadPool* threadPool;
std::string computedKeyName;

namespace {

unsigned int sanitizeStride(int stride) {
  int absStride = stride == 0 ? 1 : std::abs(stride);
  return static_cast<unsigned int>(absStride);
}

void rebuildGenerationScenarios(const std::vector<std::vector<Path>>& basePaths) {
  generationScenarios.clear();

  const AutocConfig& cfg = ConfigManager::getConfig();
  int windScenarioCount = std::max(cfg.windScenarioCount, 1);

  // Use pre-fetched windSeeds from gScenarioVariations (single PRNG architecture)
  // Fallback to 0 if pre-fetch hasn't run yet (shouldn't happen in normal flow)
  auto getWindSeed = [&](int windIdx) -> unsigned int {
    if (windIdx < static_cast<int>(gScenarioVariations.size())) {
      return gScenarioVariations[windIdx].windSeed;
    }
    return 0;  // Fallback (shouldn't happen)
  };

  if (basePaths.empty()) {
    ScenarioDescriptor scenario;
    scenario.pathList = basePaths;
    scenario.windVariantIndex = 0;
    scenario.windSeed = getWindSeed(0);
    scenario.windScenarios.push_back({getWindSeed(0), 0});
    generationScenarios.push_back(std::move(scenario));
    return;
  }

  if (cfg.demeticGrouping && cfg.demeSize > 0) {
    // DEMETIC MODE: Create one scenario per path variant.
    // Each scenario contains ONE path geometry evaluated across ALL wind conditions.
    // This allows demes to specialize on specific path geometries while being
    // robust to wind variations.
    for (size_t pathIdx = 0; pathIdx < basePaths.size(); ++pathIdx) {
      ScenarioDescriptor scenario;
      scenario.pathVariantIndex = static_cast<int>(pathIdx);
      for (int windIdx = 0; windIdx < windScenarioCount; ++windIdx) {
        scenario.pathList.push_back(basePaths[pathIdx]);
        WindScenarioConfig windScenario;
        windScenario.windSeed = getWindSeed(windIdx);
        windScenario.windVariantIndex = windIdx;
        scenario.windScenarios.push_back(windScenario);
      }
      if (!scenario.windScenarios.empty()) {
        scenario.windSeed = scenario.windScenarios.front().windSeed;
        scenario.windVariantIndex = scenario.windScenarios.front().windVariantIndex;
      } else {
        scenario.windSeed = getWindSeed(0);
        scenario.windVariantIndex = 0;
      }
      generationScenarios.push_back(std::move(scenario));
    }
  } else {
    // NON-DEMETIC MODE: Create ONE scenario containing ALL path variants × ALL winds.
    // Each individual evaluates on the same complete test suite for fair comparison.
    ScenarioDescriptor scenario;

    // Build wind scenarios list first
    for (int windIdx = 0; windIdx < windScenarioCount; ++windIdx) {
      WindScenarioConfig windScenario;
      windScenario.windSeed = getWindSeed(windIdx);
      windScenario.windVariantIndex = windIdx;
      scenario.windScenarios.push_back(windScenario);
    }

    // Add path×wind combinations in PATH-MAJOR order: [p0w0, p0w1, ..., p0w5, p1w0, ...]
    for (size_t pathIdx = 0; pathIdx < basePaths.size(); ++pathIdx) {
      for (int windIdx = 0; windIdx < windScenarioCount; ++windIdx) {
        scenario.pathList.push_back(basePaths[pathIdx]);
      }
    }

    if (!scenario.windScenarios.empty()) {
      scenario.windSeed = scenario.windScenarios.front().windSeed;
      scenario.windVariantIndex = scenario.windScenarios.front().windVariantIndex;
    } else {
      scenario.windSeed = getWindSeed(0);
      scenario.windVariantIndex = 0;
    }
    generationScenarios.push_back(std::move(scenario));
  }

  if (generationScenarios.empty()) {
    ScenarioDescriptor scenario;
    scenario.windVariantIndex = 0;
    scenario.windSeed = getWindSeed(0);
    scenario.pathList = basePaths;
    scenario.windScenarios.push_back({getWindSeed(0), 0});
    generationScenarios.push_back(std::move(scenario));
  }

  // Log scenario structure for verification (only once, not every generation)
  static bool logged = false;
  if (!logged) {
    logged = true;
    *logger.info() << "Scenario structure: " << generationScenarios.size() << " scenario(s)" << endl;
    for (size_t i = 0; i < generationScenarios.size(); ++i) {
      const auto& scenario = generationScenarios[i];
      if (scenario.pathVariantIndex == -1) {
        size_t numWinds = std::max(size_t(1), scenario.windScenarios.size());
        size_t numPaths = scenario.pathList.size() / numWinds;
        *logger.info() << "  Scenario " << i << ": ALL path variants"
                       << " (" << scenario.pathList.size() << " total flights = "
                       << numPaths << " paths × " << numWinds << " winds)" << endl;
      } else {
        *logger.info() << "  Scenario " << i << ": path variant " << scenario.pathVariantIndex
                       << " (" << scenario.pathList.size() << " flights across "
                       << scenario.windScenarios.size() << " winds)" << endl;
      }
    }
  }
}

struct PathFrame {
  gp_vec3 tangent = gp_vec3::UnitX();
  gp_vec3 insideNormal = gp_vec3::UnitZ();
  gp_vec3 binormal = gp_vec3::UnitY();
};

static gp_vec3 safeSegmentDirection(const std::vector<Path>& path, int idxA, int idxB) {
  if (path.empty()) {
    return gp_vec3::UnitX();
  }
  const int last = static_cast<int>(path.size()) - 1;
  idxA = std::clamp(idxA, 0, last);
  idxB = std::clamp(idxB, 0, last);
  if (idxA == idxB) {
    return gp_vec3::UnitX();
  }
  gp_vec3 delta = path[idxB].start - path[idxA].start;
  const gp_scalar norm = delta.norm();
  if (norm < static_cast<gp_scalar>(1e-5)) {
    return gp_vec3::UnitX();
  }
  return delta / norm;
}

static PathFrame computePathFrame(const std::vector<Path>& path, int index) {
  PathFrame frame;
  if (path.empty()) {
    return frame;
  }

  const int last = static_cast<int>(path.size()) - 1;
  const int clamped = std::clamp(index, 0, last);
  const int prevIdx = std::max(clamped - 1, 0);
  const int nextIdx = std::min(clamped + 1, last);

  frame.tangent = safeSegmentDirection(path, clamped, nextIdx);
  gp_vec3 prevDir = safeSegmentDirection(path, prevIdx, clamped);
  gp_vec3 nextDir = safeSegmentDirection(path, clamped, nextIdx);

  gp_vec3 curvature = nextDir - prevDir;
  // Remove any component that lies along the tangent
  curvature -= curvature.dot(frame.tangent) * frame.tangent;

  if (curvature.norm() < static_cast<gp_scalar>(1e-5)) {
    curvature = frame.tangent.cross(gp_vec3::UnitZ());
    if (curvature.norm() < static_cast<gp_scalar>(1e-5)) {
      curvature = frame.tangent.cross(gp_vec3::UnitY());
    }
  }
  if (curvature.norm() < static_cast<gp_scalar>(1e-5)) {
    curvature = gp_vec3::UnitZ();
  }
  frame.insideNormal = curvature.normalized();

  // Ensure the normal truly points "inside" the current turn (toward curvature)
  const gp_vec3 rawCurvature = nextDir - prevDir;
  if (rawCurvature.norm() > static_cast<gp_scalar>(1e-5) && rawCurvature.dot(frame.insideNormal) < 0.0f) {
    frame.insideNormal = -frame.insideNormal;
  }

  frame.binormal = frame.tangent.cross(frame.insideNormal);
  if (frame.binormal.norm() < static_cast<gp_scalar>(1e-5)) {
    frame.binormal = frame.tangent.cross(gp_vec3::UnitX());
  }
  if (frame.binormal.norm() < static_cast<gp_scalar>(1e-5)) {
    frame.binormal = gp_vec3::UnitY();
  }
  frame.binormal.normalize();

  // Re-orthogonalize insideNormal to eliminate accumulated numerical drift
  frame.insideNormal = (frame.binormal.cross(frame.tangent)).normalized();
  return frame;
}

const ScenarioDescriptor& scenarioForIndex(int scenarioIndex) {
  if (generationScenarios.empty()) {
    std::cerr << "FATAL: scenarioForIndex: generationScenarios is empty" << std::endl; exit(1);
  }
  int clampedIndex = ((scenarioIndex % static_cast<int>(generationScenarios.size())) + static_cast<int>(generationScenarios.size())) % static_cast<int>(generationScenarios.size());
  return generationScenarios[clampedIndex];
}

void warnIfScenarioMismatch() {
  const AutocConfig& cfg = ConfigManager::getConfig();
  if (cfg.demeticGrouping && cfg.demeSize > 0) {
    int demeCount = cfg.populationSize / cfg.demeSize;
    if (demeCount > 0 && generationScenarios.size() < static_cast<size_t>(demeCount)) {
      *logger.warn() << "Scenario count (" << generationScenarios.size()
                     << ") is smaller than deme count (" << demeCount
                     << "); scenarios will be reused across demes." << endl;
    }
  }
}

} // namespace

std::string generate_iso8601_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << "autoc-" << INT64_MAX - ms_since_epoch << '-';
  ss << std::put_time(std::gmtime(&itt), "%FT%T");
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << 'Z';
  return ss.str();
}

std::shared_ptr<Aws::S3::S3Client> getS3Client() {
  return ConfigManager::getS3Client();
}

void newHandler()
{
  cerr << "\nFatal error: Out of memory." << endl;
  exit(1);
}

// Get compiled topology as std::vector (needed by NNPopulation API)
static std::vector<int> getCompiledTopology() {
  return std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
}

// Compute fitness for an NN individual from EvalResults
// Same formula as MyGP::evalTask() lines 1210-1417
// 022: Point-accumulation fitness — delegates to computeScenarioScores + aggregateRawFitness
static double computeNNFitness(EvalResults& evalResults) {
  auto scenarioScores = computeScenarioScores(evalResults);
  return aggregateRawFitness(scenarioScores);
}

// Log per-step data from EvalResults to data.dat
// NN mode: actual NN inputs (normalized) and outputs captured in minisim, then diagnostics
static void logEvalResults(std::ofstream& fout, EvalResults& results) {
  bool printHeader = true;

  for (size_t i = 0; i < results.pathList.size(); i++) {
    auto& path = results.pathList.at(i);
    auto& aircraftStates = results.aircraftStateList.at(i);
    if (path.empty() || aircraftStates.empty()) continue;

    uint64_t scenarioSequence = 0;
    uint64_t bakeoffSequence = 0;
    int pathVariantIndex = 0;
    int windVariantIndex = 0;
    if (i < results.scenarioList.size()) {
      scenarioSequence = results.scenarioList.at(i).scenarioSequence;
      bakeoffSequence = results.scenarioList.at(i).bakeoffSequence;
      pathVariantIndex = results.scenarioList.at(i).pathVariantIndex;
      windVariantIndex = results.scenarioList.at(i).windVariantIndex;
    }

    // Position is already virtual (converted at producer boundary).
    // No manual offset subtraction needed.

    // Fitness computer for logging (mirrors fitness_decomposition.cc)
    const AutocConfig& cfg = ConfigManager::getConfig();
    int streakStepsToMax = static_cast<int>(cfg.fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0));
    if (streakStepsToMax < 1) streakStepsToMax = 1;
    FitnessComputer logFC(cfg.fitBehindScale, cfg.fitAheadScale, cfg.fitCrossScale,
                          cfg.fitStreakThreshold, streakStepsToMax, cfg.fitStreakMultiplierMax);
    logFC.resetStreak();

    gp_vec3 prevTangent = gp_vec3::UnitX();
    int simulation_steps = 0;

    int stepIndex = 0;
    while (++stepIndex < static_cast<int>(aircraftStates.size())) {
      auto& stepState = aircraftStates.at(stepIndex);
      int pathIndex = std::clamp(stepState.getThisPathIndex(), 0, static_cast<int>(path.size()) - 1);

      gp_vec3 aircraftPosition = stepState.getPosition();

      // Path tangent (same as fitness_decomposition.cc)
      gp_vec3 tangent;
      if (pathIndex + 1 < static_cast<int>(path.size())) {
        tangent = (path.at(pathIndex + 1).start - path.at(pathIndex).start);
        double tn = tangent.norm();
        if (tn > 0.01) { tangent = tangent / tn; prevTangent = tangent; }
        else { tangent = prevTangent; }
      } else {
        tangent = prevTangent;
      }

      // Along/cross decomposition
      gp_vec3 offset = aircraftPosition - path.at(pathIndex).start;
      double along = offset.dot(tangent);
      gp_vec3 lateral = offset - along * tangent;
      double lateralDist = lateral.norm();
      double distance = offset.norm();

      // Step score + streak (mirrors fitness computation)
      double stepPoints = logFC.computeStepScore(along, lateralDist);
      double multipliedScore = logFC.applyStreak(stepPoints);
      // Recover actual multiplier from the score ratio
      double mult = (stepPoints > 0.0) ? multipliedScore / stepPoints : 1.0;

      simulation_steps++;

      // Rabbit velocity
      gp_scalar rabbitVel = stepState.getRabbitSpeed();

      // Body-frame velocity
      gp_vec3 velocity_body = stepState.getOrientation().inverse() * stepState.getVelocity();

      // Distance to home
      gp_vec3 home(0, 0, 0);
      gp_scalar dhome = (home - aircraftPosition).norm();

      if (printHeader) {
        fout << "  Scn   Bake Pth/Wnd:Step:  Time Idx"
             << "  dPh-9  dPh-3  dPh-1   dPh0  dPh+1  dPh+5"
             << "  dTh-9  dTh-3  dTh-1   dTh0  dTh+1  dTh+5"
             << "   ds-9   ds-3   ds-1    ds0   ds+1   ds+5"
             << "  dd/dt"
             << "      qw      qx      qy      qz"
             << "     vel   gyrP   gyrQ   gyrR"
             << "   outPt   outRl   outTh"
             << "    pathX    pathY    pathZ"
             << "        X        Y        Z"
             << "    vxBdy    vyBdy    vzBdy"
             << "    dhome     dist   along   rabVl   stpPt    mult  rampSc"
             << "\n";
        printHeader = false;
      }

      const float* in = stepState.getNNInputs();
      const float* out = stepState.getNNOutputs();

      char outbuf[2048];
      sprintf(outbuf,
        "%06llu %06llu %03d/%02d:%04d: %06ld %3d"
        " % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f"
        " % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f"
        " % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f"
        " % 6.1f"
        " % 7.4f % 7.4f % 7.4f % 7.4f"
        " % 7.4f % 6.3f % 6.3f % 6.3f"
        " % 7.4f % 7.4f % 7.4f"
        " % 8.2f % 8.2f % 8.2f"
        " % 8.2f % 8.2f % 8.2f"
        " % 8.2f % 8.2f % 8.2f"
        " % 8.2f % 8.3f % 7.2f % 7.1f % 7.4f % 6.2f % 7.3f"
        "\n",
        static_cast<unsigned long long>(scenarioSequence),
        static_cast<unsigned long long>(bakeoffSequence),
        pathVariantIndex, windVariantIndex, simulation_steps,
        stepState.getSimTimeMsec(), pathIndex,
        in[0], in[1], in[2], in[3], in[4], in[5],
        in[6], in[7], in[8], in[9], in[10], in[11],
        in[12], in[13], in[14], in[15], in[16], in[17],
        in[18],
        in[19], in[20], in[21], in[22],
        in[23], in[24], in[25], in[26],
        out[0], out[1], out[2],
        path.at(pathIndex).start[0],
        path.at(pathIndex).start[1],
        path.at(pathIndex).start[2],
        stepState.getPosition()[0],
        stepState.getPosition()[1],
        stepState.getPosition()[2],
        velocity_body.x(), velocity_body.y(), velocity_body.z(),
        dhome,
        static_cast<gp_scalar>(distance),
        static_cast<gp_scalar>(along),
        rabbitVel,
        static_cast<gp_scalar>(stepPoints),
        static_cast<gp_scalar>(mult),
        static_cast<gp_scalar>(computeVariationScale())
      );
      fout << outbuf;
    }
  }
  fout.flush();
}

// NN evaluation mode: load weight file, run through scenarios, report fitness
static void runNNEvaluation(
    const std::string& startTime,
    std::ofstream& fout,
    std::ofstream& bout
) {
  const AutocConfig& cfg = ConfigManager::getConfig();

  *logger.info() << "NN Evaluation mode" << endl;
  *logger.info() << "  Weight file: " << cfg.nnWeightFile << endl;

  // Load NN weight file
  std::ifstream weightFile(cfg.nnWeightFile, std::ios::binary | std::ios::ate);
  if (!weightFile.is_open()) {
    *logger.error() << "Cannot open NN weight file: " << cfg.nnWeightFile << endl;
    exit(1);
  }
  std::streamsize fileSize = weightFile.tellg();
  weightFile.seekg(0, std::ios::beg);
  std::vector<uint8_t> fileData(fileSize);
  if (!weightFile.read(reinterpret_cast<char*>(fileData.data()), fileSize)) {
    *logger.error() << "Error reading NN weight file" << endl;
    exit(1);
  }
  weightFile.close();

  if (!nn_detect_format(fileData.data(), fileData.size())) {
    *logger.error() << "Weight file is not in NN01 format" << endl;
    exit(1);
  }

  NNGenome genome;
  if (!nn_deserialize(fileData.data(), fileData.size(), genome)) {
    *logger.error() << "Failed to deserialize NN genome" << endl;
    exit(1);
  }

  {
    std::ostringstream topo;
    for (size_t i = 0; i < genome.topology.size(); i++) {
      if (i > 0) topo << " -> ";
      topo << genome.topology[i];
    }
    *logger.info() << "  Topology: " << topo.str() << " (" << genome.weights.size() << " weights)" << endl;
  }
  *logger.info() << "  Stored fitness: " << std::fixed << std::setprecision(6) << genome.fitness << endl;

  // Serialize genome for RPC (same format minisim expects)
  std::vector<uint8_t> nnData;
  nn_serialize(genome, nnData);

  // Use scenario 0 (same as training)
  const ScenarioDescriptor& scenario = scenarioForIndex(0);
  uint64_t scenarioSequence = globalScenarioCounter.fetch_add(1, std::memory_order_relaxed) + 1;

  EvalData evalData;
  evalData.controllerType = ControllerType::NEURAL_NET;
  evalData.gp.assign(reinterpret_cast<const char*>(nnData.data()),
                     reinterpret_cast<const char*>(nnData.data() + nnData.size()));
  evalData.gpHash = hashByteVector(evalData.gp);
  evalData.isEliteReeval = false;
  evalData.pathList = scenario.pathList;
  evalData.scenario.scenarioSequence = scenarioSequence;
  evalData.scenario.bakeoffSequence = 0;

  evalData.scenarioList.clear();
  evalData.scenarioList.reserve(scenario.pathList.size());
  for (size_t idx = 0; idx < scenario.pathList.size(); ++idx) {
    ScenarioMetadata meta;
    meta.scenarioSequence = scenarioSequence;
    meta.enableDeterministicLogging = false;
    meta.bakeoffSequence = 0;
    size_t numWindScenarios = scenario.windScenarios.size();
    if (numWindScenarios > 0 && scenario.pathList.size() > 0) {
      size_t pathIdx = idx / numWindScenarios;
      size_t windIdx = idx % numWindScenarios;
      meta.pathVariantIndex = static_cast<int>(pathIdx);
      if (windIdx < scenario.windScenarios.size()) {
        meta.windVariantIndex = scenario.windScenarios[windIdx].windVariantIndex;
        meta.windSeed = scenario.windScenarios[windIdx].windSeed;
      }
    } else {
      meta.pathVariantIndex = static_cast<int>(idx);
    }
    populateVariationOffsets(meta);
    meta.rabbitSpeed = gRabbitSpeedConfig.nominal;
    evalData.scenarioList.push_back(meta);
  }
  if (!evalData.scenarioList.empty()) {
    evalData.scenario = evalData.scenarioList.front();
  }
  evalData.sanitizePaths();

  // Send to minisim and get results
  auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
  EvalResults evalResults;
  threadPool->enqueue([evalDataPtr, &evalResults](WorkerContext& context) {
    sendRPC(*context.socket, *evalDataPtr);
    evalResults = receiveRPC<EvalResults>(*context.socket);
  });
  threadPool->wait_for_tasks();

  // Compute decomposed fitness then aggregate
  auto evalScenarioScores = computeScenarioScores(evalResults);
  double fitness = aggregateRawFitness(evalScenarioScores);

  // Log per-step data to data.dat
  logEvalResults(fout, evalResults);

  // Per-scenario breakdown (same format as training loop)
  *logger.info() << "  Scenarios: " << endl;
  for (size_t s = 0; s < evalScenarioScores.size(); s++) {
    const auto& sc = evalScenarioScores[s];
    *logger.info() << "  [" << s << "] "
                   << (sc.crashed ? "CRASH" : "OK")
                   << " score=" << std::fixed << std::setprecision(2) << -sc.score
                   << " maxStrk=" << sc.maxStreak
                   << " strkSteps=" << sc.totalStreakSteps
                   << " maxMult=" << std::setprecision(1) << sc.maxMultiplier
                   << endl;
  }

  globalSimRunCounter.fetch_add(evalResults.pathList.size(), std::memory_order_relaxed);

  // Upload to S3 so renderer can display the eval results.
  // Key uses same reverse-time prefix as training (renderer auto-discovers newest run).
  // gen9999 = generation 1 in the reverse-sort scheme (10000 - gen).
  evalResults.gp.assign(reinterpret_cast<const char*>(nnData.data()),
                        reinterpret_cast<const char*>(nnData.data() + nnData.size()));
  evalResults.gpHash = hashByteVector(evalResults.gp);
  {
    std::string keyName = startTime + "/gen9999.dmp";
    auto s3Client = ConfigManager::getS3Client();
    if (s3Client) {
      std::ostringstream oss(std::ios::binary);
      { cereal::BinaryOutputArchive oa(oss); oa(evalResults); }
      auto stream = Aws::MakeShared<Aws::StringStream>("PutObject");
      *stream << oss.str();
      Aws::S3::Model::PutObjectRequest request;
      request.SetBucket(cfg.s3Bucket);
      request.SetKey(keyName);
      request.SetBody(stream);
      auto outcome = s3Client->PutObject(request);
      if (!outcome.IsSuccess()) {
        *logger.warn() << "S3 upload failed: " << outcome.GetError().GetMessage() << endl;
      } else {
        *logger.info() << "S3 upload: " << keyName << endl;
      }
    }
  }

  *logger.info() << "NN Eval fitness: " << std::fixed << std::setprecision(6) << fitness << endl;
  *logger.info() << "Stored fitness:  " << std::fixed << std::setprecision(6) << genome.fitness << endl;

  // Log to statistics file
  bout << "#NNEval fitness=" << std::fixed << std::setprecision(6) << fitness
       << " storedFitness=" << genome.fitness
       << " weightFile=" << cfg.nnWeightFile
       << " scenarios=" << evalResults.pathList.size()
       << endl;
}

// NN evolution main loop — runs when ControllerType=NN
static void runNNEvolution(
    const std::string& startTime,
    const std::chrono::steady_clock::time_point& runStartTime,
    std::ofstream& fout,
    std::ofstream& bout,
    const std::function<void(int)>& logGenerationStats
) {
  const AutocConfig& cfg = ConfigManager::getConfig();

  std::vector<int> topology = getCompiledTopology();
  int popSize = cfg.populationSize;
  int numGens = cfg.numberOfGenerations;


  *logger.info() << "NN Evolution mode" << endl;
  *logger.info() << "  Topology: " << NN_TOPOLOGY_STRING
                 << " (" << NN_WEIGHT_COUNT << " weights)" << endl;
  *logger.info() << "  Population: " << popSize << endl;
  *logger.info() << "  Generations: " << numGens << endl;
  *logger.info() << "  MutationSigma: " << cfg.nnMutationSigma << endl;
  *logger.info() << "  CrossoverAlpha: " << cfg.nnCrossoverAlpha << endl;
  *logger.info() << "  TournamentSize: " << cfg.tournamentSize << endl;
  *logger.info() << "  CrossoverProb: " << cfg.crossoverProbability << "%" << endl;
  *logger.info() << "  CreationProb: " << cfg.creationProbability << "%" << endl;
  *logger.info() << "  MutationOnlyProb: " << cfg.swapMutationProbability << "%" << endl;
  *logger.info() << "  Elitism: " << cfg.addBestToNewPopulation << endl;
  *logger.info() << "  SelectionMode: " << cfg.selectionMode << endl;
  *logger.info() << "  NNSigmaFloor: " << cfg.nnSigmaFloor << endl;

  // Set global sigma floor for mutation
  nn_sigma_floor = static_cast<float>(cfg.nnSigmaFloor);
  if (cfg.nnSigmaFloor > cfg.nnMutationSigma) {
    *logger.warn() << "NNSigmaFloor (" << cfg.nnSigmaFloor
                   << ") > NNMutationSigma (" << cfg.nnMutationSigma
                   << ") — floor exceeds initial sigma!" << endl;
  }

  // Initialize population
  NNPopulation pop;
  nn_init_population(pop, topology, popSize);

  // Set initial mutation sigma from config
  for (auto& ind : pop.individuals) {
    ind.mutation_sigma = static_cast<float>(cfg.nnMutationSigma);
  }

  *logger.info() << "Population initialized." << endl;

  // Generation loop (start at 1 like GP — gen0 is Xavier baseline, not stored)
  for (int gen = 1; gen <= numGens; gen++) {

    // RAMP_LANDSCAPE: Update current generation for variation scaling
    gCurrentGeneration = gen;

    // Generate paths for this generation
    generationPaths = generateSmoothPaths(const_cast<char*>(cfg.generatorMethod.c_str()),
                                          cfg.simNumPathsPerGen,
                                          SIM_PATH_BOUNDS, SIM_PATH_BOUNDS,
                                          gPathSeed);
    rebuildGenerationScenarios(generationPaths);

    // Evaluate each individual
    for (int ind = 0; ind < popSize; ind++) {
      NNGenome& genome = pop.individuals[ind];

      // Serialize genome to binary
      std::vector<uint8_t> nnData;
      nn_serialize(genome, nnData);

      // Build EvalData
      const ScenarioDescriptor& scenario = scenarioForIndex(ind % generationScenarios.size());
      uint64_t scenarioSequence = globalScenarioCounter.fetch_add(1, std::memory_order_relaxed) + 1;

      EvalData evalData;
      evalData.controllerType = ControllerType::NEURAL_NET;
      evalData.gp.assign(reinterpret_cast<const char*>(nnData.data()),
                         reinterpret_cast<const char*>(nnData.data() + nnData.size()));
      evalData.gpHash = hashByteVector(evalData.gp);
      evalData.isEliteReeval = false;
      // RAMP_LANDSCAPE: scale rabbit speed sigma (0→full over training)
      evalData.rabbitSpeedConfig = gRabbitSpeedConfig;
      evalData.rabbitSpeedConfig.sigma = gRabbitSpeedConfig.sigma * computeVariationScale();
      evalData.pathList = scenario.pathList;
      evalData.scenario.scenarioSequence = scenarioSequence;
      evalData.scenario.bakeoffSequence = 0;

      // Build scenario list
      evalData.scenarioList.clear();
      evalData.scenarioList.reserve(scenario.pathList.size());
      for (size_t idx = 0; idx < scenario.pathList.size(); ++idx) {
        ScenarioMetadata meta;
        meta.scenarioSequence = scenarioSequence;
        meta.enableDeterministicLogging = false;
        meta.bakeoffSequence = 0;

        size_t numWindScenarios = scenario.windScenarios.size();
        if (numWindScenarios > 0 && scenario.pathList.size() > 0) {
          size_t numBasePaths = scenario.pathList.size() / numWindScenarios;
          size_t pathIdx = idx / numWindScenarios;
          size_t windIdx = idx % numWindScenarios;
          meta.pathVariantIndex = static_cast<int>(pathIdx);
          if (windIdx < scenario.windScenarios.size()) {
            meta.windVariantIndex = scenario.windScenarios[windIdx].windVariantIndex;
            meta.windSeed = scenario.windScenarios[windIdx].windSeed;
          }
        } else {
          meta.pathVariantIndex = static_cast<int>(idx);
        }

        populateVariationOffsets(meta);
        meta.rabbitSpeed = gRabbitSpeedConfig.nominal;
        evalData.scenarioList.push_back(meta);
      }
      if (!evalData.scenarioList.empty()) {
        evalData.scenario = evalData.scenarioList.front();
      }
      evalData.sanitizePaths();

      // Send to minisim worker via ThreadPool
      auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
      threadPool->enqueue([&genome, evalDataPtr](WorkerContext& context) {
        sendRPC(*context.socket, *evalDataPtr);
        context.evalResults = receiveRPC<EvalResults>(*context.socket);
        globalSimRunCounter.fetch_add(context.evalResults.pathList.size(), std::memory_order_relaxed);

        // Compute decomposed fitness then aggregate
        genome.scenario_scores = computeScenarioScores(context.evalResults);
        genome.fitness = aggregateRawFitness(genome.scenario_scores);
      });
    }

    // Wait for all evaluations to complete
    threadPool->wait_for_tasks();

    // Find best individual
    int bestIdx = 0;
    for (int i = 1; i < popSize; i++) {
      if (pop.individuals[i].fitness < pop.individuals[bestIdx].fitness) {
        bestIdx = i;
      }
    }
    pop.best_fitness = pop.individuals[bestIdx].fitness;
    pop.best_index = bestIdx;

    // Compute population fitness stats
    double sumFitness = 0, minFitness = pop.individuals[0].fitness, maxFitness = pop.individuals[0].fitness;
    for (const auto& ind : pop.individuals) {
      sumFitness += ind.fitness;
      if (ind.fitness < minFitness) minFitness = ind.fitness;
      if (ind.fitness > maxFitness) maxFitness = ind.fitness;
    }
    double avgFitness = sumFitness / popSize;

    // Re-evaluate best individual and log per-step data to data.dat
    {
      NNGenome& bestGenome = pop.individuals[bestIdx];
      std::vector<uint8_t> nnData;
      nn_serialize(bestGenome, nnData);

      const ScenarioDescriptor& scenario = scenarioForIndex(bestIdx % generationScenarios.size());
      uint64_t scenarioSequence = globalScenarioCounter.fetch_add(1, std::memory_order_relaxed) + 1;

      EvalData evalData;
      evalData.controllerType = ControllerType::NEURAL_NET;
      evalData.gp.assign(reinterpret_cast<const char*>(nnData.data()),
                         reinterpret_cast<const char*>(nnData.data() + nnData.size()));
      evalData.gpHash = hashByteVector(evalData.gp);
      evalData.isEliteReeval = false;
      // Elite re-eval uses same ramp scale as current generation
      evalData.rabbitSpeedConfig = gRabbitSpeedConfig;
      evalData.rabbitSpeedConfig.sigma = gRabbitSpeedConfig.sigma * computeVariationScale();
      evalData.pathList = scenario.pathList;
      evalData.scenario.scenarioSequence = scenarioSequence;
      evalData.scenario.bakeoffSequence = 0;

      evalData.scenarioList.clear();
      evalData.scenarioList.reserve(scenario.pathList.size());
      for (size_t idx = 0; idx < scenario.pathList.size(); ++idx) {
        ScenarioMetadata meta;
        meta.scenarioSequence = scenarioSequence;
        meta.enableDeterministicLogging = false;
        meta.bakeoffSequence = 0;
        size_t numWindScenarios = scenario.windScenarios.size();
        if (numWindScenarios > 0 && scenario.pathList.size() > 0) {
          size_t pathIdx = idx / numWindScenarios;
          size_t windIdx = idx % numWindScenarios;
          meta.pathVariantIndex = static_cast<int>(pathIdx);
          if (windIdx < scenario.windScenarios.size()) {
            meta.windVariantIndex = scenario.windScenarios[windIdx].windVariantIndex;
            meta.windSeed = scenario.windScenarios[windIdx].windSeed;
          }
        } else {
          meta.pathVariantIndex = static_cast<int>(idx);
        }
        populateVariationOffsets(meta);
        meta.rabbitSpeed = gRabbitSpeedConfig.nominal;
        evalData.scenarioList.push_back(meta);
      }
      if (!evalData.scenarioList.empty()) {
        evalData.scenario = evalData.scenarioList.front();
      }
      evalData.sanitizePaths();

      auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
      EvalResults bestResults;
      threadPool->enqueue([evalDataPtr, &bestResults](WorkerContext& context) {
        sendRPC(*context.socket, *evalDataPtr);
        bestResults = receiveRPC<EvalResults>(*context.socket);
        globalSimRunCounter.fetch_add(bestResults.pathList.size(), std::memory_order_relaxed);
      });
      threadPool->wait_for_tasks();

      logEvalResults(fout, bestResults);

      // Determinism check: re-eval fitness must match stored fitness exactly (T106)
      auto reevalScores = computeScenarioScores(bestResults);
      double reevalFitness = aggregateRawFitness(reevalScores);
      double storedFitness = pop.individuals[bestIdx].fitness;
      if (!bitwiseEqual(reevalFitness, storedFitness)) {
        *logger.warn() << "NN_ELITE_DIVERGED: gen=" << gen
                       << " stored=" << std::fixed << std::setprecision(6) << storedFitness
                       << " reeval=" << reevalFitness
                       << " delta=" << std::scientific << (reevalFitness - storedFitness)
                       << endl;
      } else {
        *logger.info() << "NN_ELITE_SAME: gen=" << gen
                       << " fitness=" << std::fixed << std::setprecision(6) << storedFitness
                       << endl;
      }

      // Embed serialized NN genome in bestResults.gp for S3 storage
      // (renderer uses .gp field to detect format and extract fitness)
      bestResults.gp.assign(reinterpret_cast<const char*>(nnData.data()),
                            reinterpret_cast<const char*>(nnData.data() + nnData.size()));
      bestResults.gpHash = hashByteVector(bestResults.gp);

      // Save to S3 as cereal-serialized EvalResults
      {
        std::string keyName = startTime + "/gen" + std::to_string(10000 - gen) + ".dmp";
        auto s3Client = ConfigManager::getS3Client();
        if (s3Client) {
          std::ostringstream oss(std::ios::binary);
          { cereal::BinaryOutputArchive oa(oss); oa(bestResults); }

          auto stream = Aws::MakeShared<Aws::StringStream>("PutObject");
          *stream << oss.str();

          Aws::S3::Model::PutObjectRequest request;
          request.SetBucket(cfg.s3Bucket);
          request.SetKey(keyName);
          request.SetBody(stream);

          auto outcome = s3Client->PutObject(request);
          if (!outcome.IsSuccess()) {
            *logger.warn() << "S3 upload failed: " << outcome.GetError().GetMessage() << endl;
          }
        }
      }
    }

    // Log generation report
    *logger.info() << "Gen " << gen
                   << "  Best=" << std::fixed << std::setprecision(2) << minFitness
                   << "  Avg=" << avgFitness
                   << "  Worst=" << maxFitness
                   << "  Sigma=" << pop.individuals[bestIdx].mutation_sigma
                   << endl;

    // Log per-scenario decomposition for best individual
    const auto& bestScores = pop.individuals[bestIdx].scenario_scores;
    if (!bestScores.empty()) {
      *logger.info() << "  Scenarios: ";
      for (size_t s = 0; s < bestScores.size(); s++) {
        const auto& sc = bestScores[s];
        *logger.info() << "  [" << s << "] "
                       << (sc.crashed ? "CRASH" : "OK")
                       << " score=" << std::fixed << std::setprecision(2) << -sc.score
                       << " maxStrk=" << sc.maxStreak
                       << " strkSteps=" << sc.totalStreakSteps
                       << " maxMult=" << std::setprecision(1) << sc.maxMultiplier
                       << endl;
      }
    }

    // Streak diagnostics for best individual
    double avgMaxStreak = 0.0;
    double pctInStreak = 0.0;
    if (!bestScores.empty()) {
      double streakSum = 0.0, totalStrkSteps = 0.0, totalSteps = 0.0;
      for (const auto& sc : bestScores) {
        streakSum += sc.maxStreak;
        totalStrkSteps += sc.totalStreakSteps;
        totalSteps += sc.steps_completed;
      }
      avgMaxStreak = streakSum / bestScores.size();
      pctInStreak = (totalSteps > 0) ? 100.0 * totalStrkSteps / totalSteps : 0.0;
    }

    // Log to statistics file
    bout << "#NNGen gen=" << gen
         << " best=" << std::fixed << std::setprecision(6) << minFitness
         << " avg=" << avgFitness
         << " worst=" << maxFitness
         << " bestSigma=" << pop.individuals[bestIdx].mutation_sigma
         << " avgMaxStreak=" << std::setprecision(1) << avgMaxStreak
         << " pctInStreak=" << std::setprecision(1) << pctInStreak
         << std::endl;
    bout.flush();

    logGenerationStats(gen);

    // Evolve next generation (skip on last gen)
    if (gen < numGens) {
      NNEvolveParams evoParams;
      evoParams.tournament_size = cfg.tournamentSize;          // GP & NN
      evoParams.crossover_prob = cfg.crossoverProbability;     // GP & NN
      evoParams.creation_prob = cfg.creationProbability;       // GP & NN
      evoParams.mutation_prob = cfg.swapMutationProbability;   // GP & NN
      evoParams.crossover_alpha = static_cast<float>(cfg.nnCrossoverAlpha); // NN only
      evoParams.elitism_count = cfg.addBestToNewPopulation;    // GP & NN

      // Selection strategy (015)
      SelectionMode selMode = parseSelectionMode(cfg.selectionMode);
      if (selMode == SelectionMode::LEXICASE) {
        // Build scenario scores table for lexicase
        std::vector<std::vector<ScenarioScore>> allScores(popSize);
        for (int i = 0; i < popSize; i++) {
          allScores[i] = pop.individuals[i].scenario_scores;
        }
        evoParams.select = [allScores](const NNPopulation&) {
          return lexicase_select(allScores, static_cast<int>(allScores.size()));
        };
      } else if (selMode == SelectionMode::MINIMAX) {
        // Recompute fitness as minimax for tournament selection
        for (int i = 0; i < popSize; i++) {
          pop.individuals[i].fitness = minimax_fitness(pop.individuals[i].scenario_scores);
        }
        // Use default tournament selection (on updated fitness)
        evoParams.select = nullptr;
      }
      // SUM mode: evoParams.select stays nullptr → default tournament

      nn_evolve_generation(pop, evoParams);
    }
  }

  *logger.info() << "NN evolution complete! Best fitness: "
                 << std::fixed << std::setprecision(2) << pop.best_fitness << endl;
}

int main(int argc, char** argv)
{
  // Parse command line arguments
  std::string configFile = "autoc.ini";
  
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-i") == 0) {
      if (i + 1 < argc) {
        configFile = argv[i + 1];
        i++; // Skip the next argument since we consumed it
      } else {
        std::cerr << "Error: -i option requires a filename argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      std::cout << "Usage: " << argv[0] << " [-i config_file]" << std::endl;
      std::cout << "  -i config_file  Use specified config file instead of autoc.ini" << std::endl;
      std::cout << "  -h, --help      Show this help message" << std::endl;
      return 0;
    }
  }

  // Logger is initialized globally via logger.h — no setup needed

  // Set up a new-handler, because we might need a lot of memory, and
  // we don't know it's there.
  set_new_handler(newHandler);

  ConfigManager::initialize(configFile, *logger.info());
  const AutocConfig& cfg = ConfigManager::getConfig();

  // RNG seed: -1 means time-based
  long seed;
  if (cfg.seed == -1) {
    seed = static_cast<long>(time(NULL));
    *logger.info() << "Seed: -1 (auto) -> " << seed << endl;
  } else {
    seed = static_cast<long>(cfg.seed);
  }
  rng::seed(static_cast<uint64_t>(seed));

  // AWS setup
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  // initialize workers
  threadPool = new ThreadPool(ConfigManager::getConfig());

  // Print the configuration
  *logger.info() << "Config: pop=" << cfg.populationSize
                 << " gens=" << cfg.numberOfGenerations << endl;
  *logger.info() << "SimNumPathsPerGen: " << cfg.simNumPathsPerGen << endl;
  *logger.info() << "EvalThreads: " << cfg.evalThreads << endl;
  *logger.info() << "MinisimProgram: " << cfg.minisimProgram << endl;
  *logger.info() << "MinisimPortOverride: " << cfg.minisimPortOverride << endl;
  *logger.info() << "EvaluateMode: " << cfg.evaluateMode << endl;
  *logger.info() << "WindScenarios: " << cfg.windScenarioCount << endl;
  *logger.info() << "Seed: " << cfg.seed << endl;
  *logger.info() << "RandomPathSeedB: " << cfg.randomPathSeedB << endl;

  // Log VARIATIONS1 settings and initialize global sigmas
  *logger.info() << "EnableEntryVariations: " << cfg.enableEntryVariations << endl;
  *logger.info() << "EnableWindVariations: " << cfg.enableWindVariations << endl;

  // Initialize global variation parameters from config (degrees -> radians)
  // Store individual flags for selective application in populateVariationOffsets()
  gEnableEntryVariations = (cfg.enableEntryVariations != 0);
  gEnableWindVariations = (cfg.enableWindVariations != 0);
  gVariationSigmas = VariationSigmas::fromDegrees(
      cfg.entryConeSigma,
      cfg.entryRollSigma,
      cfg.entrySpeedSigma,  // already a fraction
      cfg.windDirectionSigma,
      cfg.entryPositionRadiusSigma,  // meters (no conversion needed)
      cfg.entryPositionAltSigma      // meters (no conversion needed)
  );

  // Initialize global rabbit speed config
  gRabbitSpeedConfig = RabbitSpeedConfig{
      cfg.rabbitSpeedNominal,
      cfg.rabbitSpeedSigma,
      cfg.rabbitSpeedMin,
      cfg.rabbitSpeedMax,
      cfg.rabbitSpeedCycleMin,
      cfg.rabbitSpeedCycleMax
  };

  // Log rabbit speed configuration
  *logger.info() << "RabbitSpeed: nominal=" << cfg.rabbitSpeedNominal << " m/s"
                 << " sigma=" << cfg.rabbitSpeedSigma << " m/s"
                 << " range=[" << cfg.rabbitSpeedMin << ", " << cfg.rabbitSpeedMax << "] m/s"
                 << " cycles=[" << cfg.rabbitSpeedCycleMin << ", " << cfg.rabbitSpeedCycleMax << "] s"
                 << (cfg.rabbitSpeedSigma > 0 ? " (VARIABLE)" : " (CONSTANT)") << endl;

  // RAMP_LANDSCAPE: Initialize variation ramp globals
  gTotalGenerations = cfg.numberOfGenerations;
  gVariationRampStep = cfg.variationRampStep;
  gCurrentGeneration = 0;
  if (gVariationRampStep > 0) {
    int totalSteps = gTotalGenerations / gVariationRampStep;
    *logger.info() << "VariationRamp: step=" << gVariationRampStep
                   << " gens, " << totalSteps << " steps over " << gTotalGenerations
                   << " gens (scale +=" << std::fixed << std::setprecision(1)
                   << (100.0 / totalSteps) << "% per step)" << endl;
  }

  // SINGLE PRNG ARCHITECTURE: Pre-fetch all scenario variations from GPrand()
  // This consumes from GPrand() BEFORE GP evolution, ensuring deterministic sequence
  // When a variation type is disabled, defaults are stored in the table (not filtered later)
  int windScenarioCount = std::max(cfg.windScenarioCount, 1);
  prefetchAllVariations(windScenarioCount, gVariationSigmas, gRabbitSpeedConfig,
                        cfg.randomPathSeedB,
                        gEnableEntryVariations, gEnableWindVariations);

  // Log pre-fetched variations for verification
  *logger.info() << "Sigmas: cone=" << cfg.entryConeSigma << "° "
                 << "roll=" << cfg.entryRollSigma << "° "
                 << "speed=" << (cfg.entrySpeedSigma * 100) << "% "
                 << "wind=" << cfg.windDirectionSigma << "° "
                 << "posR=" << cfg.entryPositionRadiusSigma << "m "
                 << "posAlt=" << cfg.entryPositionAltSigma << "m" << endl;
  logPrefetchedVariations(windScenarioCount, seed);

  // Open the main output file for the data and statistics file.
  // First set up names for data file.  Remember we should delete the
  // string from the stream, well just a few bytes
  ostringstream strOutFile, strStatFile;
  const char* filePrefix = cfg.evaluateMode ? "eval-" : "";
  strOutFile << filePrefix << "data.dat" << ends;
  strStatFile << filePrefix << "data.stc" << ends;
  fout.open(strOutFile.str());
  bout.open(strStatFile.str());

  // Set fixed-point notation for statistics file
  bout << std::fixed << std::setprecision(6);

  std::string startTime = generate_iso8601_timestamp();
  auto runStartTime = std::chrono::steady_clock::now();
  auto lastThroughputTime = runStartTime;
  uint64_t lastSimRunCount = 0;
  auto logGenerationStats = [&](int genIndex) {
    auto now = std::chrono::steady_clock::now();
    gp_scalar deltaSec = std::chrono::duration<gp_scalar>(now - lastThroughputTime).count();
    uint64_t currentRuns = globalSimRunCounter.load(std::memory_order_relaxed);
    uint64_t deltaRuns = currentRuns - lastSimRunCount;
    gp_scalar rate = (deltaSec > static_cast<gp_scalar>(0.0f))
      ? static_cast<gp_scalar>(deltaRuns) / deltaSec
      : static_cast<gp_scalar>(0.0f);
    bout << std::fixed << std::setprecision(2)
         << "#GenSimStats gen=" << genIndex
         << " sims=" << deltaRuns
         << " total=" << currentRuns
         << " durationSec=" << deltaSec
         << " rate=" << rate
         << std::setprecision(6) << std::endl;
    bout.flush();
    lastThroughputTime = now;
    lastSimRunCount = currentRuns;
  };

  // Generate initial paths using pre-fetched gPathSeed (single PRNG architecture)
  generationPaths = generateSmoothPaths(const_cast<char*>(cfg.generatorMethod.c_str()),
                                        cfg.simNumPathsPerGen,
                                        SIM_PATH_BOUNDS, SIM_PATH_BOUNDS,
                                        gPathSeed);

  // DEBUG: Log segment counts for each path
  std::cout << "\n=== Path Segment Counts (method=" << cfg.generatorMethod
            << ", seed=" << gPathSeed << ") ===" << std::endl;
  int maxSegments = 0;
  for (size_t i = 0; i < generationPaths.size(); i++) {
    int segCount = generationPaths[i].size();
    std::cout << "  Path " << i << ": " << segCount << " segments" << std::endl;
    if (segCount > maxSegments) maxSegments = segCount;
  }
  std::cout << "  Maximum: " << maxSegments << " segments" << std::endl;
  std::cout << "===" << std::endl << std::endl;

  rebuildGenerationScenarios(generationPaths);
  const int windsPerPath = std::max(cfg.windScenarioCount, 1);
  *logger.debug() << "Wind scenarios this generation: paths="
                 << cfg.simNumPathsPerGen
                 << " windsPerPath=" << windsPerPath
                 << " dispatchScenarios=" << generationScenarios.size()
                 << " totalEvaluations=" << generationScenarios.size() * windsPerPath
                 << endl;
  warnIfScenarioMismatch();

  if (cfg.evaluateMode) {
    // NN evaluation mode: load weight file, evaluate, report fitness
    runNNEvaluation(startTime, fout, bout);
  } else {
    // NN evolution mode
    runNNEvolution(startTime, runStartTime, fout, bout, logGenerationStats);
  }


  uint64_t totalRuns = globalSimRunCounter.load(std::memory_order_relaxed);
  gp_scalar durationSec = std::chrono::duration<gp_scalar>(std::chrono::steady_clock::now() - runStartTime).count();
  gp_scalar simsPerSec = (durationSec > static_cast<gp_scalar>(0.0f)) ? static_cast<gp_scalar>(totalRuns) / durationSec : static_cast<gp_scalar>(0.0f);
  bout << "#SimRuns " << totalRuns << " DurationSec " << durationSec << std::endl;
  bout.flush();
  *logger.info() << std::fixed << std::setprecision(2)
                 << "Simulation throughput: " << totalRuns << " runs in "
                 << durationSec << "s (" << simsPerSec << " sims/s)" << std::defaultfloat << std::endl;
}

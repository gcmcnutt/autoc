
// autoc.cc

/* -------------------------------------------------------------------
From skeleton/skeleton.cc
------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cstdlib>
#include <math.h>
#include <cmath>
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
#include "autoc/eval/craft_variation.h"      // 034 US4 — craft-class draws
#include "autoc/nn/mode.h"           // 030 M7a — getActiveModeStrategy
#include "autoc/nn/population.h"
#include "autoc/nn/serialization.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/telemetry.h"
#include "autoc/eval/fitness_computer.h"
#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/selection.h"
#include "autoc/eval/source_dmp_loader.h"  // 030 M6e tracker mode
#include "autoc/eval/crash_hull.h"         // 030 M7d.b — pCrashForGen
#include "autoc/util/scenario_prng.h"      // 033 §2.A — master/scenario/class PRNG chain
#include "autoc/util/run_id.h"             // 034 FR-015 — mode→run-id prefix routing
#include "autoc/util/s3_run_selector.h"    // 035 FR-P07/P09/P10 — shared S3 dmp I/O

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/client/ClientConfiguration.h>

#include <cereal/archives/binary.hpp>

using namespace std;

// 040 T015 — airframe obstruction straight from config, METRES in body frame.
// Datum: prop axle = (0,0,0); +x fwd, +y right, +z down; stations aft = -x.
// Every field is read; none carries a fallback here (Constitution VII), and
// contract_tracker_config_tests asserts the keys exist in every ini, so a
// struct default can never silently stand in for a missing one.
static autoc::eval::AirframeObstruction airframeObstructionFromConfig(
    const AutocConfig& cfg) {
    autoc::eval::AirframeObstruction a;
    a.enabled = (cfg.airframeObstructionEnabled != 0);
    a.wing_min = gp_vec3(static_cast<gp_scalar>(cfg.airframeWingMinX),
                         static_cast<gp_scalar>(cfg.airframeWingMinY),
                         static_cast<gp_scalar>(cfg.airframeWingMinZ));
    a.wing_max = gp_vec3(static_cast<gp_scalar>(cfg.airframeWingMaxX),
                         static_cast<gp_scalar>(cfg.airframeWingMaxY),
                         static_cast<gp_scalar>(cfg.airframeWingMaxZ));
    a.nose_min = gp_vec3(static_cast<gp_scalar>(cfg.airframeNoseMinX),
                         static_cast<gp_scalar>(cfg.airframeNoseMinY),
                         static_cast<gp_scalar>(cfg.airframeNoseMinZ));
    a.nose_max = gp_vec3(static_cast<gp_scalar>(cfg.airframeNoseMaxX),
                         static_cast<gp_scalar>(cfg.airframeNoseMaxY),
                         static_cast<gp_scalar>(cfg.airframeNoseMaxZ));
    a.prop_plane_x = static_cast<gp_scalar>(cfg.airframePropPlaneX);
    a.prop_axis_y = static_cast<gp_scalar>(cfg.airframePropAxisY);
    a.prop_axis_z = static_cast<gp_scalar>(cfg.airframePropAxisZ);
    a.prop_radius = static_cast<gp_scalar>(cfg.airframePropRadius);
    a.prop_attenuation = static_cast<gp_scalar>(cfg.airframePropAttenuation);
    return a;
}


std::vector<std::vector<Path>> generationPaths;
std::vector<ScenarioDescriptor> generationScenarios;

// 030 M6e — tracker-mode source trajectories (FR-001). Loaded once at
// startup when Mode = tracker; empty in pathgen mode. Per-scenario
// distribution to workers happens in buildEvalData.
static std::vector<SourceScenarioTrajectory> gSourceTrajectoryList;

// Parse a comma-separated index list (e.g. "0,1,2,3,4,5") into vector<int>.
// Trims whitespace; returns empty vector for empty input. Invalid tokens
// are skipped with a warning rather than failing — operator-friendly for
// the tracker config's path/wind subset fields.
static std::vector<int> parseCsvIndices(const std::string& csv) {
    std::vector<int> out;
    if (csv.empty()) return out;
    std::stringstream ss(csv);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // Trim leading/trailing whitespace.
        size_t b = token.find_first_not_of(" \t");
        size_t e = token.find_last_not_of(" \t");
        if (b == std::string::npos) continue;
        token = token.substr(b, e - b + 1);
        try {
            out.push_back(std::stoi(token));
        } catch (...) {
            std::cerr << "[parseCsvIndices] skipping invalid token '" << token << "'" << std::endl;
        }
    }
    return out;
}

// 033 §2.A — Master PRNG instance + scenarioSeed[K] table.
//
// gMasterPRNG is the single instance per autoc run. Init chain at startup:
//   1. effectiveSeed = (cfg.seed == -1) ? wall_clock : cfg.seed
//   2. gMasterPRNG.init(effectiveSeed)
//   3. rng::seed(gMasterPRNG.next())            — NN-evolution stream
//   4. gScenarioSeedTable[K] = gMasterPRNG.next()  for K in [0, M_total)
//
// gScenarioSeedTable.size() == paths × windScenarioCount (path-major
// linear index, matching scenarioMetaList layout). Populated by
// populateScenarioSeedTable() once the scenario count is known.
// scenarioSeed[K]==0 is converted to kSeedZeroSentinel before insert
// (Park-Miller breaks at zero).
static autoc::util::MasterPRNG gMasterPRNG;
static std::vector<uint64_t> gScenarioSeedTable;
// 033 §2.A — Originally-passed effective master seed (for dmp provenance).
// Captured at autoc startup right before gMasterPRNG.init(); used to
// populate EvalResults.effectiveMasterSeed so the dmp is self-describing.
static uint64_t gEffectiveMasterSeed = 0;

// 033 §2.A — Stamp the dmp provenance header right before serialize.
// Records the master seed that produced this dmp. Operator inspection of a
// dmp reads this to know "what produced this run". gEffectiveMasterSeed is
// populated at autoc startup.
static void stampEvalResultsProvenance(EvalResults& results) {
    results.effectiveMasterSeed = gEffectiveMasterSeed;
    // 038 P0-D-2 — stamp the self-describing run config (fitness cone / cadence /
    // crash penalty) so the dmp replays standalone without the live .ini
    // (renderer + dmp_dump prefer this block, P0-B/T010).
    const AutocConfig& cfg = ConfigManager::getConfig();
    RecordedRunConfig& rc = results.runConfig;
    rc.fitDistScaleBehind     = cfg.fitDistScaleBehind;
    rc.fitDistScaleAhead      = cfg.fitDistScaleAhead;
    rc.fitConeAngleDeg        = cfg.fitConeAngleDeg;
    rc.fitStreakThreshold     = cfg.fitStreakThreshold;
    rc.fitStreakRampSec       = cfg.fitStreakRampSec;
    rc.fitStreakMultiplierMax = cfg.fitStreakMultiplierMax;
    rc.simTimeStepMsec        = SIM_TIME_STEP_MSEC;
    rc.cadenceTickScale       = kCadenceTickScale;
    rc.enableHullCrashPenalty = cfg.enableHullCrashPenalty;
    rc.hullCrashPenaltyFactor = cfg.hullCrashPenaltyFactor;
    rc.oobCrashPenaltyWeight  = cfg.oobCrashPenaltyWeight;
}

// Compute (and cache) the scenarioSeed table once the per-run scenario
// count is known. Called from a single site after rebuildGenerationScenarios.
// Re-population is a no-op (table only built once per run).
static void populateScenarioSeedTable(size_t totalScenarioCount) {
    if (!gScenarioSeedTable.empty()) return;  // already populated this run
    gScenarioSeedTable.reserve(totalScenarioCount);
    for (size_t k = 0; k < totalScenarioCount; ++k) {
        uint64_t s = gMasterPRNG.next();
        if (s == 0) s = autoc::util::kSeedZeroSentinel;  // Park-Miller zero guard
        gScenarioSeedTable.push_back(s);
    }
    *logger.info() << "Populated scenarioSeedTable: " << totalScenarioCount
                   << " entries (first=0x" << std::hex
                   << gScenarioSeedTable.front() << ", last=0x"
                   << gScenarioSeedTable.back() << ")"
                   << std::dec << std::endl;
}

// VARIATIONS1: Global sigma parameters, initialized at startup from config
static VariationSigmas gVariationSigmas = {0.0, 0.0, 0.0, 0.0, 0.0};
// 034 US4 — craft-class sigmas, initialized at startup from config. The
// EnableCraftVariations master disable parallels EnableEntry/Wind/Rabbit
// (draw-and-discard when off — PRNG advances, deltas zeroed before write).
static autoc::eval::CraftSigmas gCraftSigmas;
static bool gEnableCraftVariations = false;
// Individual variation enable flags (from config)
static bool gEnableEntryVariations = false;
static bool gEnableWindVariations = false;
// 038 t7 — when set (tracker-only), the CHASE derives its per-scenario
// variation seed from the M1 SOURCE's recorded scenarioSeed (shared airspace)
// instead of the fresh M2 gScenarioSeedTable. See chaseScenarioSeedAt().
static bool gChaseUseSourceScenarioSeed = false;

// Per-scenario seed the CHASE uses for variation-class derivation at path-major
// index i. Default: the fresh M2 seed table. When gChaseUseSourceScenarioSeed
// (t7), return the paired M1 source scenario's recorded scenarioSeed so the
// chase's wind/thermal/gust/entry/craft/crash-hull match what the source flew
// (NN mutation + camera stay on the M2 seed). Caller must ensure i is in range;
// the startup guard enforces gSourceTrajectoryList.size() == gScenarioSeedTable
// .size() whenever the knob is on, so both are indexable by the same i.
static uint64_t chaseScenarioSeedAt(int i) {
    if (gChaseUseSourceScenarioSeed) {
        return gSourceTrajectoryList[i].variation.scenarioSeed;
    }
    return gScenarioSeedTable[i];
}

// Variable rabbit speed: Global config, initialized at startup from config
static RabbitSpeedConfig gRabbitSpeedConfig = RabbitSpeedConfig::defaultConfig();

// RAMP_LANDSCAPE: Gradual variation scaling (see specs/RAMP_LANDSCAPE.md)
static int gCurrentGeneration = 0;      // Updated at start of each generation
static int gTotalGenerations = 1;       // Set from config at startup
static int gVariationRampStep = 0;      // Set from config at startup (0 = disabled)
static float gEvalVariationScaleOverride = -1.0f;  // >=0 = use this instead of computing

/**
 * Compute variation scale for current generation.
 * Scale ramps from 0.0 to 1.0 over the course of training.
 * In eval mode, returns the stored value from the weight file.
 *
 * @return Scale factor to apply to variation offsets (0.0 to 1.0)
 */
static float computeVariationScale() {
    // Eval mode: use the exact scale stored in the weight file
    if (gEvalVariationScaleOverride >= 0.0f) return gEvalVariationScaleOverride;

    // Ramp from 0.0 to 1.0 over training.  Returns float for consistency with
    // serialized variation_scale (stored as float in NN01 weights).  All call
    // sites must see identical precision between training and eval.
    int numSteps = (gVariationRampStep > 0) ? gTotalGenerations / gVariationRampStep : 0;
    if (numSteps <= 1) return 1.0f;

    int stepIndex = (gCurrentGeneration - 1) / gVariationRampStep;  // 1-based gen
    return static_cast<float>(std::min(stepIndex, numSteps - 1)) / static_cast<float>(numSteps - 1);
}

// 038 T001 — member-level crash penalty, applied IN PLACE to a genome's per-scenario
// tracking scores. MUST be called on EVERY path that turns scenario scores into a genome
// fitness — population eval, elite determinism re-eval, AND eval-mode replay — so they all
// compute identical fitness. (t14 bug: the penalty was applied in the population eval but
// NOT the elite re-eval, so once it activated at the first ramp step the stored fitness
// (penalized) and the re-eval fitness (un-penalized) diverged every gen → spurious
// NN_ELITE_DIVERGED; the eval-mode gate would break the same way.) Score axis only (energy
// untouched). Gated; curriculum-ramped via computeVariationScale() [0..1] (eval reuses the
// stored variation_scale → reproducible); smooth; never clamps to 0.
//   - HullStrike (RARE):  factor^(K_hull·scale)  (relaxed 0.5→0.75; decisive but not
//     annihilating; fractional exponent ramps the bite in).
//   - Eval/OOB (COMMON):  exp(−weight·scale·K_oob/N)  (smooth, never 0, monotonic). weight=0 → no-op.
// See specs/038-accurate-m2/t12-retro.md.
static void applyCrashPenalty(std::vector<ScenarioScore>& scores) {
  const AutocConfig& c = ConfigManager::getConfig();
  if (!c.enableHullCrashPenalty || scores.empty()) return;
  int khull = 0, koob = 0;
  for (const auto& s : scores) {
    if (s.crashReason == CrashReason::HullStrike) ++khull;
    else if (s.crashReason == CrashReason::Eval)   ++koob;
  }
  const double scale = static_cast<double>(computeVariationScale());  // [0..1] curriculum ramp
  double mult = 1.0;
  if (khull > 0)
    mult *= std::pow(c.hullCrashPenaltyFactor, static_cast<double>(khull) * scale);
  if (c.oobCrashPenaltyWeight > 0.0 && koob > 0) {
    const double frac = static_cast<double>(koob) / static_cast<double>(scores.size());
    mult *= std::exp(-c.oobCrashPenaltyWeight * scale * frac);
  }
  if (mult != 1.0) {
    const gp_fitness m = static_cast<gp_fitness>(mult);
    for (auto& s : scores) s.score *= m;
  }
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
 * 033 cleanup: windSeed + rabbitSpeedSeed removed. Worker derives all
 * per-class seeds from meta.scenarioSeed via deriveClassSubSeeds()
 * (see autoc/util/scenario_prng.h). The entryOffsets values are
 * computed autoc-side from entryPRNG (deterministic per scenarioSeed)
 * and propagated to the worker via ScenarioMetadata for variation_scale
 * application.
 */
struct ScenarioVariations {
    VariationOffsets entryOffsets;              // Heading, roll, pitch, speed, windDir
    // 034 US4 — full-magnitude craft draw + the class sub-seed that produced
    // it. Copied to ScenarioMetadata at per-eval construction; the worker
    // scales via applyVariationScale() before passing to the FDM.
    autoc::eval::CraftDeltas craftDeltas;
    uint32_t craftSeed = 0;                     // = deriveClassSubSeeds(scenarioSeed).craft
};

// Global pre-computed table (indexed by wind scenario index 0..N-1)
static std::vector<ScenarioVariations> gScenarioVariations;
static unsigned int gPathSeed = 0;        // Derived from GPrand() or RandomPathSeedB override
static bool gPathSeedFromOverride = false;// True if RandomPathSeedB was used

/**
 * Pre-fetch all scenario variations from class-scoped PRNGs at startup.
 * Called once before evolution begins, AFTER populateScenarioSeedTable().
 *
 * 033 §2.A — Class PRNG semantics (034 FR-012: per-(path, wind)):
 *   - For each scenario K in [0, numScenarios) where numScenarios =
 *     paths × winds, derive class sub-PRNG seeds from gScenarioSeedTable[K]
 *     (path-major layout: K = pathIdx × winds + windIdx).
 *   - 5 class sub-seeds derived in append-only order: wind, rabbit,
 *     entry, craft (seeded but unused), camera (seeded but unused).
 *   - Entry-class draws (cone/roll/speed/position) consume from entryPRNG.
 *   - windDirectionOffset (wind-class per spec.md §2.A) consumes from windPRNG.
 *   - sv.windSeed = windPRNG.next() (uint32_t for crrcsim CRRC_Random seed).
 *   - sv.rabbitSpeedSeed = rabbitPRNG.next() (uint32_t for worker-side speed
 *     profile generator + crash-hull PRNG).
 *
 * Disabled-flag handling: each variation class still DRAWS to advance its
 * PRNG (so a future en/disable doesn't perturb the seed table for unrelated
 * classes), but the drawn variation VALUES are replaced by defaults before
 * insertion into sv. This realizes the "draw-and-discard" semantics of
 * spec.md §2.E. Wind-class shared-seed fallback (all scenarios = same wind
 * when disabled) preserved.
 *
 * 034 FR-012 — full per-(path, wind) variation table: `numScenarios` is the
 * TOTAL scenario count (paths × windScenarioCount). gScenarioVariations[K] is
 * indexed by the SAME linear path-major K as gScenarioSeedTable[K] and
 * scenarioMetaList[K], so entry-pose + wind-direction offsets are derived from
 * each scenario's own scenarioSeed — distinct per (path, wind), not shared
 * across paths. (Previously sized per-wind, which reused path-0's offsets for
 * all paths; closed in 034.)
 *
 * Preconditions: populateScenarioSeedTable() called first (so the seed
 * table is non-empty). rng::* is the NN-evolution stream and is NOT
 * consumed here (per spec.md §2.A NN-evolution / variation PRNG split).
 *
 * @param numScenarios       Total scenario count = paths × windScenarioCount
 * @param sigmas             Variation sigma parameters
 * @param rabbitCfg          Rabbit speed configuration (unused — kept for ABI)
 * @param randomPathSeedB    Override path seed (-1 = derive from rng::* NN stream)
 * @param enableEntry        If false, store default entry offsets but still draw
 * @param enableWind         If false, store default wind offset but still draw
 * @param enableCraft        If false, store default (zero) craft deltas + 1.0
 *                           thrust scale but still draw (PRNG advance preserves
 *                           cross-class determinism — see entry/wind precedent)
 */
static void prefetchAllVariations(int numScenarios, const VariationSigmas& sigmas,
                                   const RabbitSpeedConfig& /*rabbitCfg*/, int randomPathSeedB,
                                   bool enableEntry, bool enableWind, bool enableCraft) {
    gScenarioVariations.clear();
    gScenarioVariations.reserve(numScenarios);

    // Path seed derivation stays on the NN-evolution rng::* stream (it
    // controls evolution-side path generation, not per-scenario variation).
    if (randomPathSeedB == -1) {
        gPathSeed = static_cast<unsigned int>(rng::randLong());
        gPathSeedFromOverride = false;
    } else {
        gPathSeed = static_cast<unsigned int>(randomPathSeedB);
        gPathSeedFromOverride = true;
    }

    if (gScenarioSeedTable.empty()) {
        *logger.info() << "WARNING: prefetchAllVariations called before "
                          "populateScenarioSeedTable; variations will be "
                          "incorrectly seeded" << endl;
    }

    // 033 cleanup: with windSeed/rabbitSpeedSeed removed from
    // gScenarioVariations + ScenarioMetadata, autoc-side prefetch only
    // needs to compute the entry-class variation OFFSETS that the worker
    // applies via applyVariationScale. Per-class PRNG seeds (wind, rabbit,
    // entry, craft, camera) are derived ON THE WORKER from meta.scenarioSeed
    // via deriveClassSubSeeds() — eliminating the autoc→worker propagation
    // of the wind/rabbit-speed integer seeds.
    //
    // We still construct the entryPRNG autoc-side because the entry-pose
    // OFFSETS are computed here (so the variation_scale can be applied
    // worker-side via applyVariationScale). The wind/rabbit class PRNGs
    // need no autoc-side draws under the cleaned-up architecture.

    for (int i = 0; i < numScenarios; i++) {
        ScenarioVariations sv;

        // Construct per-scenario ScenarioRootPRNG from the K-th scenario
        // seed. 034 FR-012: K is the linear path-major scenario index
        // (paths × winds), parallel to gScenarioSeedTable + scenarioMetaList,
        // so each (path, wind) gets its own entry/wind offsets.
        const uint64_t scenarioSeed = (i < static_cast<int>(gScenarioSeedTable.size()))
            ? chaseScenarioSeedAt(i) : autoc::util::kSeedZeroSentinel;
        const autoc::util::ClassSubSeeds subseeds =
            autoc::util::deriveClassSubSeeds(scenarioSeed);

        // Entry-class draws: cone/roll/speed/position. Always advance the
        // entryPRNG to honor draw-and-discard when enableEntry=false.
        autoc::util::ClassPRNG entryPRNG(subseeds.entry);
        VariationOffsets entryDraw =
            generateEntryVariationsFromClassPRNG(entryPRNG, sigmas);

        if (enableEntry) {
            sv.entryOffsets.entryHeadingOffset = entryDraw.entryHeadingOffset;
            sv.entryOffsets.entryRollOffset = entryDraw.entryRollOffset;
            sv.entryOffsets.entryPitchOffset = entryDraw.entryPitchOffset;
            sv.entryOffsets.entrySpeedFactor = entryDraw.entrySpeedFactor;
            sv.entryOffsets.entryNorthOffset = entryDraw.entryNorthOffset;
            sv.entryOffsets.entryEastOffset = entryDraw.entryEastOffset;
            sv.entryOffsets.entryAltOffset = entryDraw.entryAltOffset;
        } else {
            // Defaults: no offset (entryPRNG already advanced above)
            sv.entryOffsets.entryHeadingOffset = 0.0;
            sv.entryOffsets.entryRollOffset = 0.0;
            sv.entryOffsets.entryPitchOffset = 0.0;
            sv.entryOffsets.entrySpeedFactor = 1.0;
            sv.entryOffsets.entryNorthOffset = 0.0;
            sv.entryOffsets.entryEastOffset = 0.0;
            sv.entryOffsets.entryAltOffset = 0.0;
        }

        // Wind-class windDirectionOffset (static per-scenario rotation of
        // base wind heading). Drawn from windPRNG; advance regardless of
        // enableWind to honor draw-and-discard semantics. NOTE: this is
        // separate from the worker-side windPRNG.next() that seeds
        // CRRC_Random — both use windPRNG seeded identically from
        // subseeds.wind, but their draws are independent (different
        // ClassPRNG instances at different consumer sites).
        autoc::util::ClassPRNG windPRNG(subseeds.wind);
        const double drawnWindDir =
            windDirectionOffsetFromClassPRNG(windPRNG, sigmas.windDirectionSigma);
        sv.entryOffsets.windDirectionOffset = enableWind ? drawnWindDir : 0.0;

        // 034 US4 — craft-class draws. The PRNG is ALWAYS advanced (draw-
        // and-discard semantics matching entry/wind) so toggling
        // EnableCraftVariations doesn't shift other classes' draws. Off ⇒
        // zero out the deltas (and 1.0 thrust scale) before writing.
        // Full-magnitude here; worker-side applyVariationScale() ramps per
        // eval. Sigma=0 on a single axis is a finer-grained no-op below
        // generateCraftFromClassPRNG (gaussian(0, 0) = 0).
        autoc::util::ClassPRNG craftPRNG(subseeds.craft);
        sv.craftSeed = subseeds.craft;
        autoc::eval::CraftDeltas craftDraw =
            autoc::eval::generateCraftFromClassPRNG(craftPRNG, gCraftSigmas);
        if (enableCraft) {
            sv.craftDeltas = craftDraw;
        } else {
            sv.craftDeltas = autoc::eval::CraftDeltas{};  // zeros + 1.0 thrust
        }

        gScenarioVariations.push_back(std::move(sv));
    }
}

/**
 * Log pre-fetched variations at startup for verification.
 * Format matches spec in SINGLE_PRNG.md.
 */
static void logPrefetchedVariations(int numScenarios, int64_t seed) {
    *logger.info() << endl;
    *logger.info() << "=== Pre-fetched Scenario Variations (Seed=" << seed << ") ===" << endl;
    *logger.info() << "PathSeed: " << gPathSeed
                   << " (override: " << (gPathSeedFromOverride ? "yes" : "no")
                   << ")" << endl;
    *logger.info() << "Scenarios: " << numScenarios << endl;
    *logger.info() << endl;

    // 033 cleanup: WindSeed + RabbitSeed columns removed (worker derives
    // them from meta.scenarioSeed via deriveClassSubSeeds). ScenarioSeed
    // is the master-derived per-scenario seed and is the new replay-key.
    //
    // 034 US4: appended Craft columns + CraftSeed at the right. Values are
    // DRAWN at full magnitude (variation_scale = 1.0); the per-eval APPLIED
    // value is drawn × computeVariationScale(gen) for additives, or
    // 1.0 + computeVariationScale(gen) × (drawn − 1.0) for craftThrustScale.
    // Operator can compute applied at any gen from this table plus the
    // per-gen rampSc value already emitted in data.dat / log lines.
    // Craft columns omitted when craft is disabled (EnableCraftVariations=0)
    // or all sigmas == 0 (no-op mode) to keep the table narrow.
    const bool any_craft_active = gEnableCraftVariations
        && ((gCraftSigmas.craftCGSigma       > 0.0)
            || (gCraftSigmas.craftDragSigma     > 0.0)
            || (gCraftSigmas.craftTrimSigma     > 0.0)
            || (gCraftSigmas.craftThrustSigma   > 0.0)
            || (gCraftSigmas.craftPitchEffSigma > 0.0)
            || (gCraftSigmas.craftRollEffSigma  > 0.0)
            || (gCraftSigmas.craftServoSlewSigma > 0.0)   // 037 actuator dynamics
            || (gCraftSigmas.craftThrustTauSigma > 0.0));

    {
        std::ostringstream hdr;
        hdr << "Scenario       ScenarioSeed  Heading°   Roll°   Pitch°  Speed%  WindDir°  North°  East°  Down°";
        if (any_craft_active) {
            hdr << "       cgU     drag    trimD    thrSc   pitEff   rolEff   svSlew   thrTau   pwmPh    CraftSeed";
        }
        *logger.info() << hdr.str() << endl;
    }
    {
        std::ostringstream sep;
        sep << "--------  -----------------  --------  ------  ------  ------  --------  ------  -----  -----";
        if (any_craft_active) {
            sep << "  --------  -------  -------  -------  -------  -------  -------  -------  -------  -------  -----------";
        }
        *logger.info() << sep.str() << endl;
    }

    for (int i = 0; i < numScenarios; i++) {
        const auto& sv = gScenarioVariations[i];
        const uint64_t scenarioSeed = (i < static_cast<int>(gScenarioSeedTable.size()))
            ? chaseScenarioSeedAt(i) : 0ull;

        std::ostringstream line;
        line << std::setw(4) << i << "      "
             << "0x" << std::hex << std::setw(16) << std::setfill('0') << scenarioSeed
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

        if (any_craft_active) {
            // 034 US4 — drawn (full-magnitude) craft values per scenario.
            // cg in meters, drag/pitEff/rolEff as fractions, trim in degrees
            // (rad in struct, displayed °), thrSc as multiplier (1.0 = nominal).
            const auto& cd = sv.craftDeltas;
            line << "  " << std::setprecision(4)
                 << std::setw(8) << static_cast<double>(cd.craftCGDelta)        // dimensionless (CG_arm MAC units)
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftDragDelta)      // fraction
                 << "  " << std::setprecision(2)
                 << std::setw(7) << radToDeg(static_cast<double>(cd.craftTrimDelta)) // deg
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftThrustScale)    // multiplier
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftPitchEffDelta)  // fraction
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftRollEffDelta)   // fraction
                 << "  " << std::setprecision(3)
                 << std::setw(7) << static_cast<double>(cd.craftServoSlew)      // 037: /s (servo slew, autoc [-1,1] units)
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftThrustTau)      // 037: s (thrust lag tau)
                 << "  " << std::setprecision(4)
                 << std::setw(7) << static_cast<double>(cd.craftServoPwmPhase)  // 037 v2: s (PWM latch phase)
                 << "  0x" << std::hex << std::setw(8) << std::setfill('0')
                 << sv.craftSeed
                 << std::dec << std::setfill(' ');
        }

        *logger.info() << line.str() << endl;
    }
    *logger.info() << endl;
}

// Rabbit speed is now applied at runtime via odometer advancement in the worker.
// The applySpeedProfileToPath function has been removed as part of the
// odometer-based path traversal refactor.
std::atomic_ulong nanDetector = 0;
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

  // 033 cleanup: getWindSeed lambda removed (windSeed field gone from
  // ScenarioVariations + WindScenarioConfig + ScenarioDescriptor).
  // ScenarioDescriptor/WindScenarioConfig now carry only windVariantIndex;
  // worker derives per-class seeds from meta.scenarioSeed at scenario start.

  if (basePaths.empty()) {
    ScenarioDescriptor scenario;
    scenario.pathList = basePaths;
    scenario.windVariantIndex = 0;
    scenario.windScenarios.push_back({0});
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
        windScenario.windVariantIndex = windIdx;
        scenario.windScenarios.push_back(windScenario);
      }
      if (!scenario.windScenarios.empty()) {
        scenario.windVariantIndex = scenario.windScenarios.front().windVariantIndex;
      } else {
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
      scenario.windVariantIndex = scenario.windScenarios.front().windVariantIndex;
    } else {
      scenario.windVariantIndex = 0;
    }
    generationScenarios.push_back(std::move(scenario));
  }

  if (generationScenarios.empty()) {
    ScenarioDescriptor scenario;
    scenario.windVariantIndex = 0;
    scenario.pathList = basePaths;
    scenario.windScenarios.push_back({0});
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

// 034 FR-015 — run-id prefix is caller-supplied so M1 (pathgen) and M2
// (tracker) dmps in the same S3 bucket are distinguishable by prefix
// ("autoc-" vs "tracker-"). The mode→prefix decision stays at the call site.
std::string generate_iso8601_timestamp(const std::string& runIdPrefix) {
  auto now = std::chrono::system_clock::now();
  auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << runIdPrefix << INT64_MAX - ms_since_epoch << '-';
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

// Bridge config → mode lookup. Lives here (not in mode.cc) to keep mode.cc
// free of the ConfigManager / AWS SDK dependency chain.
static const ModeStrategy& getActiveModeStrategy() {
  return getModeStrategyByName(ConfigManager::getConfig().mode.c_str());
}

// Get topology for the active mode (030 M7a runtime mode-select per FR-019).
// Pathgen mode → NN_TOPOLOGY (33 input); tracker mode → TRACKER_NN_TOPOLOGY
// (45 input). Read by population init + topology logging.
static std::vector<int> getCompiledTopology() {
  const ModeStrategy& mode = getActiveModeStrategy();
  return std::vector<int>(mode.topology, mode.topology + mode.num_layers);
}

// Get recurrent-layer flags for the active mode (spec 027, D-simple).
static std::vector<uint8_t> getCompiledRecurrent() {
  const ModeStrategy& mode = getActiveModeStrategy();
  std::vector<uint8_t> r(mode.num_layers);
  for (int i = 0; i < mode.num_layers; i++) r[i] = mode.recurrent[i] ? 1 : 0;
  return r;
}

// Compute fitness for an NN individual from EvalResults
// Same formula as MyGP::evalTask() lines 1210-1417
// 022: Point-accumulation fitness — delegates to computeScenarioScores + aggregateRawFitness
static double computeNNFitness(EvalResults& evalResults) {
  auto scenarioScores = computeScenarioScores(evalResults);
  return aggregateRawFitness(scenarioScores);
}

// 035 FR-P05 — the per-step data.dat writer (logEvalResults +
// logEvalResultsScenarioTracker) is retired; the S3 dmp is now the single
// training-trace artifact, inspected via the dmp-dump tool (FR-P06).

// Purpose of each caller for buildEvalData()
enum class EvalPurpose {
    Training,        // Per-individual fitness during evolution
    EliteReeval,     // Best-of-generation deterministic re-run
    StandaloneEval,  // evaluateMode=1 one-shot evaluation
};

struct EvalJob {
    const ScenarioDescriptor& scenario;
    const std::vector<uint8_t>& nnData;  // Already-serialized NN01 payload
    EvalPurpose purpose;
};

// 030 V1 + V1.5 priming (2026-05-08) — build the once-per-worker WorkerInit
// from the loaded source library + cfg + run-static path-generation +
// scenario variation table. Sent during ThreadPool worker startup (right
// after each worker's TCP accept), then cached on the worker side.
// Eliminates the per-eval sourceList copy explosion (V1) AND the per-eval
// pathList / scenarioList copy explosion (V1.5) that pinned ~25 GB
// resident on autoc at pop=5000 tracker training. See specs/BACKLOG.md
// "Worker-side scenario priming" entry.
//
// V1.5 preconditions: caller MUST have already invoked
// generateSmoothPaths + rebuildGenerationScenarios + prefetchAllVariations
// at startup, so generationScenarios[0] is populated. Pathgen-mode
// WorkerInit gets the scenario library too (it varies per-mode only in
// what's tracker-specific — sourceList stays empty for pathgen).
static WorkerInit buildWorkerInit() {
    const auto& cfg = ConfigManager::getConfig();
    WorkerInit init;
    init.mode = parseModeName(cfg.mode);

    // 037 T001 -- thread the validated control cadence to the worker (it has no
    // ConfigManager). config.cc already asserted controlIntervalMsec > 0 and
    // == SIM_TIME_STEP_MSEC at load, so this is a known-good value here.
    init.controlIntervalMsec = static_cast<unsigned long>(cfg.controlIntervalMsec);

    // 034 Phase 7 — worker-side wind gate. autoc-side already zeros
    // meta.windDirectionOffset when EnableWindVariations=0, but the worker
    // (inputdev_autoc) independently derives a per-scenario wind seed for
    // CRRC_Random; pass the flag through so the worker can collapse all
    // scenarios to a shared fixed wind seed when disabled. See protocol.h
    // for the full rationale.
    init.enableWindVariations = (cfg.enableWindVariations != 0);

    // 037 servo v2 — in-FDM servo model switch (PWM latch + slew); the
    // worker gates the fdm_larcsim servo block on it.
    init.servoModelEnabled = (cfg.servoModelEnabled != 0);

    // 030 V1.5 — run-static scenario library shared by both modes.
    // generateSmoothPaths(gPathSeed) is byte-identical every gen, so we
    // copy it exactly once into WorkerInit. scenarioMetaList carries
    // entry offsets at FULL SCALE (scale=1.0); the worker applies the
    // per-eval variation_scale via applyVariationScale before each eval.
    if (!generationScenarios.empty()) {
        const auto& scen = generationScenarios.front();
        init.pathList = scen.pathList;
        // Sanitize once at autoc-side before shipping (was previously
        // EvalData::sanitizePaths called per-eval; same fix-NaN logic,
        // applied once instead of 5000×).
        for (auto& pathGroup : init.pathList) {
            for (auto& path : pathGroup) {
                path.sanitize();
            }
        }

        const size_t numWindScenarios = std::max<size_t>(scen.windScenarios.size(), 1u);
        init.scenarioMetaList.reserve(scen.pathList.size());
        for (size_t idx = 0; idx < scen.pathList.size(); ++idx) {
            ScenarioMetadata meta;
            meta.bakeoffSequence = 0;
            meta.enableDeterministicLogging = false;  // overridden per-eval

            // Path-major layout: pathIdx = idx / numWinds; windIdx = idx % numWinds.
            // (Matches rebuildGenerationScenarios non-demetic case.)
            const size_t pathIdx = (numWindScenarios > 0)
                ? idx / numWindScenarios : idx;
            const size_t windIdx = (numWindScenarios > 0)
                ? idx % numWindScenarios : 0;
            meta.pathVariantIndex = static_cast<int>(pathIdx);
            if (windIdx < scen.windScenarios.size()) {
                meta.windVariantIndex = scen.windScenarios[windIdx].windVariantIndex;
                // (033 cleanup) meta.windSeed assignment removed — field
                // deleted from ScenarioMetadata. Worker derives the wind
                // sub-seed from meta.scenarioSeed via deriveClassSubSeeds.
            }
            // 033 §2.A — record the master-derived per-(path, wind)
            // scenarioSeed in meta. ScenarioRootPRNG on the worker side
            // reconstructs all 5 class sub-PRNGs from this single value.
            // Per spec.md Clarifications Q3 (sufficient for full replay).
            if (idx < gScenarioSeedTable.size()) {
                meta.scenarioSeed = chaseScenarioSeedAt(static_cast<int>(idx));
            }
            // Pre-populate entry offsets at full scale from the variation
            // table. 034 FR-012: index by the linear path-major scenario K
            // (= idx), so each (path, wind) gets its own offsets derived from
            // its own scenarioSeed (was windIdx → path-0 offsets shared).
            // Worker scales per-eval via applyVariationScale.
            if (idx < gScenarioVariations.size()) {
                const auto& v = gScenarioVariations[idx].entryOffsets;
                meta.entryHeadingOffset = v.entryHeadingOffset;
                meta.entryRollOffset = v.entryRollOffset;
                meta.entryPitchOffset = v.entryPitchOffset;
                meta.entrySpeedFactor = v.entrySpeedFactor;
                meta.windDirectionOffset = v.windDirectionOffset;
                meta.entryNorthOffset = v.entryNorthOffset;
                meta.entryEastOffset = v.entryEastOffset;
                meta.entryAltOffset = v.entryAltOffset;
                // 034 US4 — full-magnitude craft draws copied here; worker
                // will scale via applyVariationScale before passing to FDM.
                const auto& cd = gScenarioVariations[idx].craftDeltas;
                meta.craftCGDelta = cd.craftCGDelta;
                meta.craftDragDelta = cd.craftDragDelta;
                meta.craftTrimDelta = cd.craftTrimDelta;
                meta.craftThrustScale = cd.craftThrustScale;
                meta.craftPitchEffDelta = cd.craftPitchEffDelta;
                meta.craftRollEffDelta = cd.craftRollEffDelta;
                // 037 actuator-dynamics axes -- absolute physical values
                // (center + clamped Gaussian); worker ramps toward center.
                meta.craftServoSlew = cd.craftServoSlew;
                meta.craftThrustTau = cd.craftThrustTau;
                meta.craftServoPwmPhase = cd.craftServoPwmPhase;  // 037 servo v2
                meta.craftSeed = gScenarioVariations[idx].craftSeed;
                // (033 cleanup) rabbitSpeedSeed assignment removed — field
                // deleted from ScenarioMetadata + ScenarioVariations.
                // Worker derives rabbit-class PRNG seed from
                // meta.scenarioSeed via deriveClassSubSeeds.
            }
            meta.rabbitSpeed = gRabbitSpeedConfig.nominal;
            init.scenarioMetaList.push_back(meta);
        }
    }

    if (init.mode != Mode::TRACKER) {
        return init;
    }

    // 030 tracker library — copy the entire deduplicated source-trajectory
    // list once. Worker indexes into init.sourceList by scenario index;
    // per-eval EvalData no longer carries trajectory bytes.
    init.sourceList = gSourceTrajectoryList;

    init.cameraConfig.fov_h_deg = static_cast<gp_scalar>(cfg.cameraFOVHorizontalDeg);
    init.cameraConfig.fov_v_deg = static_cast<gp_scalar>(cfg.cameraFOVVerticalDeg);
    init.cameraConfig.mount_offset_body =
        gp_vec3(static_cast<gp_scalar>(cfg.cameraMountOffsetX),
                static_cast<gp_scalar>(cfg.cameraMountOffsetY),
                static_cast<gp_scalar>(cfg.cameraMountOffsetZ));

    init.beaconLeftConfig.wavelength_nm = static_cast<uint16_t>(cfg.beaconLeftWavelengthNm);
    init.beaconLeftConfig.emission_cone_deg = static_cast<gp_scalar>(cfg.beaconEmissionConeDeg);
    init.beaconLeftConfig.mount_body =
        gp_vec3(static_cast<gp_scalar>(cfg.beaconLeftMountX),
                static_cast<gp_scalar>(cfg.beaconLeftMountY),
                static_cast<gp_scalar>(cfg.beaconLeftMountZ));
    init.beaconLeftConfig.emission_axis_body = gp_vec3(0.0f, -1.0f, 0.0f);

    init.beaconRightConfig.wavelength_nm = static_cast<uint16_t>(cfg.beaconRightWavelengthNm);
    init.beaconRightConfig.emission_cone_deg = static_cast<gp_scalar>(cfg.beaconEmissionConeDeg);
    init.beaconRightConfig.mount_body =
        gp_vec3(static_cast<gp_scalar>(cfg.beaconRightMountX),
                static_cast<gp_scalar>(cfg.beaconRightMountY),
                static_cast<gp_scalar>(cfg.beaconRightMountZ));
    init.beaconRightConfig.emission_axis_body = gp_vec3(0.0f, +1.0f, 0.0f);

    // 040 T013 — obstruction geometry anchors to the camera mount, since the
    // thrust line's body-frame position is not yet measured (checklist A1b).
    init.airframeObstruction = airframeObstructionFromConfig(cfg);

    init.flightArena.radius_m = static_cast<gp_scalar>(cfg.flightArenaRadius);
    init.flightArena.floor_agl_m = static_cast<gp_scalar>(cfg.flightArenaFloorAGL);
    init.flightArena.ceiling_agl_m = static_cast<gp_scalar>(cfg.flightArenaCeilingAGL);

    init.crashHullRadius = static_cast<gp_scalar>(cfg.crashHullRadius);
    init.trailDistance = static_cast<gp_scalar>(cfg.trailDistance);
    init.cepGateThreshold = static_cast<gp_scalar>(cfg.cepGateThreshold);

    return init;
}

// 030 V1 + V1.5 priming (2026-05-08) — slim per-eval EvalData. The
// scenario library (pathList + scenarioMetaList) lives on the worker
// (cached from WorkerInit at startup); EvalData carries only NN bytes +
// per-eval/per-gen scalars. ~50 B per EvalData instead of ~7 MB.
//
// Worker-side reconstruction per scenario: copy init_.scenarioMetaList[i],
// override scenarioSequence + enableDeterministicLogging from this
// EvalData, then call applyVariationScale(meta, evalData.variationScale)
// — byte-equivalent to the legacy populateVariationOffsets path.
static EvalData buildEvalData(const EvalJob& job) {
    EvalData evalData;
    evalData.controllerType = ControllerType::NEURAL_NET;
    evalData.gp.assign(reinterpret_cast<const char*>(job.nnData.data()),
                       reinterpret_cast<const char*>(job.nnData.data() + job.nnData.size()));
    evalData.gpHash = hashByteVector(evalData.gp);

    // Bug 4 fix: StandaloneEval and EliteReeval both get elite treatment
    evalData.isEliteReeval = (job.purpose != EvalPurpose::Training);

    // Bug 3 fix: ALWAYS set rabbitSpeedConfig from the global (was missing in eval path).
    // rabbitSpeedConfig is gen-varying (sigma scaled by computeVariationScale)
    // so it stays in EvalData — small struct, cost is negligible.
    evalData.rabbitSpeedConfig = gRabbitSpeedConfig;
    evalData.rabbitSpeedConfig.sigma = gRabbitSpeedConfig.sigma * computeVariationScale();

    evalData.scenarioSequence =
        globalScenarioCounter.fetch_add(1, std::memory_order_relaxed) + 1;
    evalData.variationScale = static_cast<gp_scalar>(computeVariationScale());

    // 030 M11.preA.3 (2026-05-10) — pCrashThisGen is now a fixed Bernoulli
    // probability per NN tick (10Hz), no per-gen ramp. Constant across the
    // run gives deterministic per-(scenario, gen) crash-hull outcomes given
    // the rabbit-class-PRNG-seeded crash-hull PRNG (033 — was windSeed pre-
    // cleanup). Worker-side crashHullRadius + SPHERE shape come from
    // WorkerInit. Pathgen-mode leaves it at 0.
    if (parseModeName(ConfigManager::getConfig().mode) == Mode::TRACKER) {
        const auto& cfg = ConfigManager::getConfig();
        evalData.pCrashThisGen = static_cast<gp_scalar>(cfg.crashHullProbability);
    }

    return evalData;
}

// NN evaluation mode: load weight file, run through scenarios, report fitness
static void runNNEvaluation(
    const std::string& startTime
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

  // Validate topology matches the active mode's compile-time expectation
  // (030 M7a — runtime mode-select per FR-019). Eval-mode loads a
  // weight file that must match the .ini's Mode (pathgen → 33 / tracker → 45).
  {
    const ModeStrategy& mode = getActiveModeStrategy();
    std::vector<int> expectedTopology(mode.topology, mode.topology + mode.num_layers);
    if (genome.topology != expectedTopology) {
      std::ostringstream fileTopo, compiledTopo;
      for (size_t i = 0; i < genome.topology.size(); i++) {
        if (i > 0) fileTopo << ",";
        fileTopo << genome.topology[i];
      }
      for (size_t i = 0; i < expectedTopology.size(); i++) {
        if (i > 0) compiledTopo << ",";
        compiledTopo << expectedTopology[i];
      }
      *logger.error() << "NN topology mismatch: file has {" << fileTopo.str()
                      << "} but binary expects {" << compiledTopo.str()
                      << "}. Old weight files (e.g. 611-weight 27,16,8,3) are incompatible "
                      << "with the 023 direction-cosine input layout." << endl;
      exit(1);
    }
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
  *logger.info() << "  Generation: " << genome.generation
                 << "  VariationScale: " << std::fixed << std::setprecision(4)
                 << genome.variation_scale << endl;

  // Use the variation scale stored in the weight file — this is the exact value
  // computeVariationScale() returned at the gen these weights were saved during
  // training. No recomputation from ramp globals needed (which would require
  // matching NumberOfGenerations and VariationRampStep between train/eval configs).
  gEvalVariationScaleOverride = genome.variation_scale;

  // Serialize genome for RPC (same format the worker expects)
  std::vector<uint8_t> nnData;
  nn_serialize(genome, nnData);

  // TODO T044: iterate all scenarios for Bug 5 fix
  const ScenarioDescriptor& scenario = scenarioForIndex(0);
  EvalData evalData = buildEvalData({scenario, nnData, EvalPurpose::StandaloneEval});

  // Send to the worker and get results
  auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
  EvalResults evalResults;
  threadPool->enqueue([evalDataPtr, &evalResults](WorkerContext& context) {
    sendRPC(*context.socket, *evalDataPtr);
    evalResults = receiveRPC<EvalResults>(*context.socket);
  });
  threadPool->wait_for_tasks();

  // Compute decomposed fitness then aggregate
  auto evalScenarioScores = computeScenarioScores(evalResults);
  // 038 T001 — same crash penalty as training (eval reuses the stored variation_scale via
  // computeVariationScale's override), so eval-mode fitness reproduces the penalized training
  // fitness for the bitwise gate.
  applyCrashPenalty(evalScenarioScores);
  double storedFitness = genome.fitness;  // Save before overwrite (Bug 2 fix)
  double fitness = aggregateRawFitness(evalScenarioScores);

  // Determinism check: compare eval-computed fitness with stored fitness (T046)
  // Note: with Bug 3 fix (rabbitSpeedConfig now set), eval fitness may legitimately
  // differ from stored training-time fitness. This log is for visibility, not an error.
  if (!bitwiseEqual(fitness, storedFitness)) {
    *logger.info() << "NN_EVAL_DIFFERENT: eval=" << std::fixed << std::setprecision(6) << fitness
                   << " stored=" << storedFitness << endl;
  } else {
    *logger.info() << "NN_EVAL_SAME: fitness=" << std::fixed << std::setprecision(6) << fitness << endl;
  }

  // 035 FR-P05 — per-step data.dat retired; the dmp is the training trace.
  // Per-scenario breakdown (same format as training loop)
  *logger.info() << "  Scenarios: " << endl;
  const bool isTrackerMode = (cfg.mode == "tracker");
  for (size_t s = 0; s < evalScenarioScores.size(); s++) {
    const auto& sc = evalScenarioScores[s];
    *logger.info() << "  [" << s << "] "
                   << (sc.crashed ? "CRASH" : "OK")
                   << " reason=" << crashReasonToString(sc.crashReason)
                   << " score=" << std::fixed << std::setprecision(2) << -sc.score
                   << " maxStrk=" << sc.maxStreak
                   << " strkSteps=" << sc.totalStreakSteps
                   << " maxMult=" << std::setprecision(1) << sc.maxMultiplier
                   << endl;
    // 030 M11.wrap T088 + 327-330 — tracker-mode per-scenario diagnostics.
    // Emitted as indented continuation line; suppressed in pathgen mode.
    if (isTrackerMode) {
      const auto& d = sc.tracker_diag;
      *logger.info() << "      "
                     << "vis=" << std::fixed << std::setprecision(2) << d.vis_frac
                     << " inRamp=" << d.in_fit_ramp_frac
                     << " rng=[" << std::setprecision(1)
                     << d.range_min << "/" << d.range_med << "/" << d.range_p95 << "]"
                     << " loss=[far=" << d.loss_geom_too_far
                     << " ang=" << d.loss_geom_angle
                     << " over=" << d.loss_geom_overshoot
                     << " hull=" << d.loss_hull << "]"
                     << " over[flips=" << d.closure_flips
                     << " maxClose=" << std::setprecision(1) << d.max_closure_rate << "]"
                     << " fwd[lostMax=" << d.max_lost_sight_run
                     << " spiral=" << std::setprecision(3) << d.spiral_ratio
                     << " thrPt=" << std::setprecision(1) << d.thrash_rate_pt
                     << " thrRl=" << d.thrash_rate_rl << "]"
                     << endl;
    }
  }

  globalSimRunCounter.fetch_add(evalResults.pathList.size(), std::memory_order_relaxed);

  // Bug 2 fix: store eval-computed fitness, not the original stored fitness
  genome.fitness = fitness;
  std::vector<uint8_t> updatedNnData;
  nn_serialize(genome, updatedNnData);

  // Upload to S3 so renderer can display the eval results.
  // Key uses same reverse-time prefix as training (renderer auto-discovers newest run).
  // gen9999 = generation 1 in the reverse-sort scheme (10000 - gen).
  evalResults.gp.assign(reinterpret_cast<const char*>(updatedNnData.data()),
                        reinterpret_cast<const char*>(updatedNnData.data() + updatedNnData.size()));
  evalResults.gpHash = hashByteVector(evalResults.gp);
  stampEvalResultsProvenance(evalResults);  // 033 §2.A + §2.B
  {
    std::string keyName = startTime + "/gen9999.dmp.zst";  // 035 FR-P09 — compressed
    auto s3Client = ConfigManager::getS3Client();
    if (s3Client) {
      std::ostringstream oss(std::ios::binary);
      { cereal::BinaryOutputArchive oa(oss); oa(evalResults); }
      // FR-P09/P10 compress+tag+upload via the shared S3 dmp I/O — throws on
      // failure (fail-fast, Constitution VII).
      autoc::s3PutDmpBlob(*s3Client, cfg.s3Bucket, keyName, oss.str());
      *logger.info() << "S3 upload: " << keyName << endl;
    }
  }

  *logger.info() << "NN Eval fitness: " << std::fixed << std::setprecision(6) << fitness << endl;
  *logger.info() << "Stored fitness:  " << std::fixed << std::setprecision(6) << storedFitness << endl;

  // Per-gen telemetry marker — single source is the .log (data.stc retired, T052).
  *logger.info() << "#NNEval fitness=" << std::fixed << std::setprecision(6) << fitness
       << " storedFitness=" << storedFitness
       << " weightFile=" << cfg.nnWeightFile
       << " scenarios=" << evalResults.pathList.size()
       << endl;
}

// NN evolution main loop — runs when ControllerType=NN
static void runNNEvolution(
    const std::string& startTime,
    const std::chrono::steady_clock::time_point& runStartTime,
    const std::function<void(int)>& logGenerationStats
) {
  const AutocConfig& cfg = ConfigManager::getConfig();

  std::vector<int> topology = getCompiledTopology();
  int popSize = cfg.populationSize;
  int numGens = cfg.numberOfGenerations;


  *logger.info() << "NN Evolution mode" << endl;
  {
    const ModeStrategy& mode = getActiveModeStrategy();
    *logger.info() << "  Topology: " << mode.topology_string
                   << " (" << mode.weight_count << " weights, mode=" << mode.name << ")" << endl;
  }
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
  const std::vector<uint8_t> recurrent = getCompiledRecurrent();
  nn_init_population(pop, topology, recurrent, popSize);

  // Set initial mutation sigma from config
  for (auto& ind : pop.individuals) {
    ind.mutation_sigma = static_cast<float>(cfg.nnMutationSigma);
  }

  *logger.info() << "Population initialized." << endl;

  // Generation loop (start at 1 like GP — gen0 is Xavier baseline, not stored)
  for (int gen = 1; gen <= numGens; gen++) {

    // RAMP_LANDSCAPE: Update current generation for variation scaling
    gCurrentGeneration = gen;

    // 030 V1.5 (2026-05-08) — generateSmoothPaths + rebuildGenerationScenarios
    // formerly fired here every gen. They produce byte-identical output
    // every generation (gPathSeed is run-constant), so they were hoisted
    // to a single call at startup. Workers received the resulting
    // pathList + scenarioMetaList once via WorkerInit; per-eval EvalData
    // no longer carries them.

    // Evaluate each individual
    for (int ind = 0; ind < popSize; ind++) {
      NNGenome& genome = pop.individuals[ind];

      // Serialize genome to binary
      std::vector<uint8_t> nnData;
      nn_serialize(genome, nnData);

      // Build EvalData
      const ScenarioDescriptor& scenario = scenarioForIndex(ind % generationScenarios.size());
      EvalData evalData = buildEvalData({scenario, nnData, EvalPurpose::Training});

      // Send to the worker via ThreadPool
      auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
      threadPool->enqueue([&genome, evalDataPtr](WorkerContext& context) {
        sendRPC(*context.socket, *evalDataPtr);
        context.evalResults = receiveRPC<EvalResults>(*context.socket);
        globalSimRunCounter.fetch_add(context.evalResults.pathList.size(), std::memory_order_relaxed);

        // Compute decomposed fitness then aggregate
        genome.scenario_scores = computeScenarioScores(context.evalResults);
        // 038 T001 — member-level crash penalty (see applyCrashPenalty + t12-retro.md).
        // MUST also be applied identically in the elite re-eval + eval-mode paths.
        applyCrashPenalty(genome.scenario_scores);
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
      bestGenome.variation_scale = static_cast<float>(computeVariationScale());
      std::vector<uint8_t> nnData;
      nn_serialize(bestGenome, nnData);

      const ScenarioDescriptor& scenario = scenarioForIndex(bestIdx % generationScenarios.size());
      EvalData evalData = buildEvalData({scenario, nnData, EvalPurpose::EliteReeval});

      auto evalDataPtr = std::make_shared<EvalData>(std::move(evalData));
      EvalResults bestResults;
      threadPool->enqueue([evalDataPtr, &bestResults](WorkerContext& context) {
        sendRPC(*context.socket, *evalDataPtr);
        bestResults = receiveRPC<EvalResults>(*context.socket);
        globalSimRunCounter.fetch_add(bestResults.pathList.size(), std::memory_order_relaxed);
      });
      threadPool->wait_for_tasks();

      // 035 FR-P05 — per-step data.dat retired; the dmp is the training trace.
      // Determinism check: re-eval fitness must match stored fitness exactly (T106)
      auto reevalScores = computeScenarioScores(bestResults);
      // 038 T001 — apply the SAME crash penalty as the population eval, else a penalized
      // elite's stored (penalized) fitness diverges from this (un-penalized) re-eval every
      // gen once the ramp activates the penalty (the t14 NN_ELITE_DIVERGED bug).
      applyCrashPenalty(reevalScores);
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
      stampEvalResultsProvenance(bestResults);  // 033 §2.A + §2.B

      // Save to S3 as cereal-serialized EvalResults
      {
        std::string keyName = startTime + "/gen" + std::to_string(10000 - gen) + ".dmp.zst";  // 035 FR-P09
        auto s3Client = ConfigManager::getS3Client();
        if (s3Client) {
          std::ostringstream oss(std::ios::binary);
          { cereal::BinaryOutputArchive oa(oss); oa(bestResults); }
          // FR-P09/P10 compress+tag+upload via the shared S3 dmp I/O — throws on
          // failure (fail-fast, Constitution VII).
          autoc::s3PutDmpBlob(*s3Client, cfg.s3Bucket, keyName, oss.str());
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

    // 037 T005 — per-scenario [N] OK/CRASH lines + tracker per-scenario
    // diagnostics DROPPED from the training log (294 lines/gen). The elite
    // per-scenario data is fully reconstructable from the gen .dmp via
    // `dmp-dump --meta-only` (crash_reason, score, energy/stability,
    // max_streak, streak_steps, max_multiplier, steps — verified against the
    // t5 run 2026-06-10). Per-gen #NNGen/#GenCrash/#GenSimStats summaries
    // below are unchanged. The one-shot EVAL-mode breakdown (evalGenome)
    // keeps its per-scenario block. Per
    // memory:project_dmp_driven_analytics_backlog.
    const auto& bestScores = pop.individuals[bestIdx].scenario_scores;
    const bool isTrackerModeLoop = (cfg.mode == "tracker");  // #GenDiag (M2-only) below

    // Streak + stability + energy diagnostics for best individual.
    // stability and energy are SUMS of per-scenario scores (additive across
    // the 245-scenario landscape, same convention as genome.fitness for
    // tracking). All three negative; lower = better.
    double avgMaxStreak = 0.0;
    double pctInStreak = 0.0;
    double totalStability = 0.0;
    double totalEnergy = 0.0;
    if (!bestScores.empty()) {
      double streakSum = 0.0, totalStrkSteps = 0.0, totalSteps = 0.0;
      for (const auto& sc : bestScores) {
        streakSum += sc.maxStreak;
        totalStrkSteps += sc.totalStreakSteps;
        totalSteps += sc.steps_completed;
        totalStability += sc.stability_score;
        totalEnergy += sc.energy_score;
      }
      avgMaxStreak = streakSum / bestScores.size();
      pctInStreak = (totalSteps > 0) ? 100.0 * totalStrkSteps / totalSteps : 0.0;
    }

    // 028 telemetry: signal 1 (synthetic W_hh/W_xh activation ratio for best individual)
    //                signal 2 (population-level CV per weight block)
    // See specs/028-deeper-rnn/data-model.md and contracts/evolution_log_columns.md.
    const double whh_xh_ratio = compute_synthetic_activation_ratio(pop.individuals[bestIdx]);
    const PopulationBlockStats blockStats = compute_population_block_stats(pop.individuals);

    // Per-gen telemetry markers — single source is the .log (data.stc retired, T052).
    *logger.info() << "#NNGen gen=" << gen
         << " best=" << std::fixed << std::setprecision(6) << minFitness
         << " avg=" << avgFitness
         << " worst=" << maxFitness
         << " bestSigma=" << pop.individuals[bestIdx].mutation_sigma
         << " avgMaxStreak=" << std::setprecision(1) << avgMaxStreak
         << " pctInStreak=" << std::setprecision(1) << pctInStreak
         << " stability=" << std::setprecision(2) << totalStability
         << " energy=" << std::setprecision(2) << totalEnergy
         << " whh_xh_ratio=" << std::setprecision(4) << whh_xh_ratio
         << " w_xh0_cv=" << std::setprecision(4) << blockStats.w_xh0_cv
         << " w_xh1_cv=" << std::setprecision(4) << blockStats.w_xh1_cv
         << " w_hh_cv=" << std::setprecision(4) << blockStats.w_hh_cv
         << std::endl;

    // 030 M11.wrap diagnostics — per-gen CrashReason aggregate over the elite's
    // scenario set. Surfaces hull-strike vs arena-egress vs timeout distribution
    // without requiring dmp inspection. crashReason populated in fitness_decomposition.cc.
    {
      int cnt_none = 0, cnt_boot = 0, cnt_sim = 0, cnt_eval = 0;
      int cnt_timeLimit = 0, cnt_rabbitComplete = 0, cnt_hullStrike = 0;
      for (const auto& sc : bestScores) {
        switch (sc.crashReason) {
          case CrashReason::None:           cnt_none++; break;
          case CrashReason::Boot:           cnt_boot++; break;
          case CrashReason::Sim:            cnt_sim++; break;
          case CrashReason::Eval:           cnt_eval++; break;
          case CrashReason::TimeLimit:      cnt_timeLimit++; break;
          case CrashReason::RabbitComplete: cnt_rabbitComplete++; break;
          case CrashReason::HullStrike:     cnt_hullStrike++; break;
        }
      }
      *logger.info() << "#GenCrash gen=" << gen
           << " hullStrike=" << cnt_hullStrike
           << " eval=" << cnt_eval
           << " sim=" << cnt_sim
           << " boot=" << cnt_boot
           << " timeLimit=" << cnt_timeLimit
           << " rabbitComplete=" << cnt_rabbitComplete
           << " none=" << cnt_none
           << " total=" << static_cast<int>(bestScores.size())
           << std::endl;
    }
    // 030 M11.wrap T088 + 327-330 — per-gen tracker-mode diag aggregate. Sums
    // streak-loss counters + means visibility / range / forward-looking stats
    // across the elite's scenario set. Pathgen-mode mode skips emission.
    if (isTrackerModeLoop && !bestScores.empty()) {
      long total_far=0, total_ang=0, total_over=0, total_hull=0;
      long total_flips=0;
      double vis_sum=0, ramp_sum=0, rng_min_sum=0, rng_med_sum=0, rng_p95_sum=0;
      double max_close_max=0, lost_max_max=0;
      double spiral_sum=0, thr_pt_sum=0, thr_rl_sum=0;
      for (const auto& sc : bestScores) {
        const auto& d = sc.tracker_diag;
        total_far += d.loss_geom_too_far;
        total_ang += d.loss_geom_angle;
        total_over += d.loss_geom_overshoot;
        total_hull += d.loss_hull;
        total_flips += d.closure_flips;
        vis_sum += d.vis_frac;
        ramp_sum += d.in_fit_ramp_frac;
        rng_min_sum += d.range_min;
        rng_med_sum += d.range_med;
        rng_p95_sum += d.range_p95;
        if (std::abs(d.max_closure_rate) > std::abs(max_close_max)) max_close_max = d.max_closure_rate;
        if (d.max_lost_sight_run > lost_max_max) lost_max_max = d.max_lost_sight_run;
        spiral_sum += d.spiral_ratio;
        thr_pt_sum += d.thrash_rate_pt;
        thr_rl_sum += d.thrash_rate_rl;
      }
      const double N = static_cast<double>(bestScores.size());
      long loss_total = total_far + total_ang + total_over + total_hull;
      *logger.info() << "#GenDiag gen=" << gen
           << " loss_total=" << loss_total
           << " far=" << total_far
           << " angle=" << total_ang
           << " over=" << total_over
           << " hull=" << total_hull
           << " avgVis=" << std::fixed << std::setprecision(3) << (vis_sum / N)
           << " avgInRamp=" << (ramp_sum / N)
           << " avgRngMin=" << std::setprecision(2) << (rng_min_sum / N)
           << " avgRngMed=" << (rng_med_sum / N)
           << " avgRngP95=" << (rng_p95_sum / N)
           << " avgFlips=" << std::setprecision(2) << (static_cast<double>(total_flips) / N)
           << " maxClose=" << std::setprecision(2) << max_close_max
           << " maxLost=" << static_cast<int>(lost_max_max)
           << " avgSpiral=" << std::setprecision(4) << (spiral_sum / N)
           << " avgThrPt=" << std::setprecision(2) << (thr_pt_sum / N)
           << " avgThrRl=" << (thr_rl_sum / N)
           << std::endl;
    }

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
        // 035 FR-003 — MAD-relative epsilon when LexicaseEpsilonMode=mad,
        // else the constant 0.5-floor path (bit-reproduces prior runs).
        const bool useMadEps = (cfg.lexicaseEpsilonMode == "mad");
        // 038 US3 — add the aux span/closure-prediction lexicase axis when the
        // EnablePredictorHead ablation gate is on (tracker-only; harmless in
        // pathgen where prediction_score is 0 for all candidates).
        const bool includePredAxis = (cfg.enablePredictorHead != 0);
        evoParams.select = [allScores, useMadEps, includePredAxis](const NNPopulation&) {
          return lexicase_select(allScores, static_cast<int>(allScores.size()), useMadEps,
                                 0.05, includePredAxis);
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

  // 033 §2.A — Master seed → autoc-NN-PRNG seed + scenarioSeed[K] table.
  //
  // Init chain (per spec.md Clarifications Q3 + contracts/scenario_prng_chain.md):
  //   effectiveSeed = (cfg.seed == -1) ? wall_clock : cfg.seed
  //   MasterPRNG.init(effectiveSeed)
  //     ├── .next() → rng::seed (autoc-NN-evolution stream — population init,
  //     │            mutation, crossover, selection; isolated from variations)
  //     └── .next() × M_total → gScenarioSeedTable[0..M_total-1]
  //
  // M_total = paths × windScenarioCount (path-major linear scenario index).
  // 034 FR-012: gScenarioVariations[] is per-(path, wind) — sized M_total and
  // indexed by the linear K, parallel to gScenarioSeedTable + scenarioMetaList.
  // Each scenario's entry/wind offsets derive from its own scenarioSeed.
  // 034 FR-011 — int64_t end-to-end so a logged effectiveMasterSeed pasted
  // back into Seed=N round-trips without truncation (cfg.seed is now int64_t).
  int64_t seed;
  if (cfg.seed == -1) {
    seed = static_cast<int64_t>(time(NULL));
    *logger.info() << "Seed: -1 (auto) -> " << seed << endl;
  } else {
    seed = cfg.seed;
  }
  const uint64_t effectiveMasterSeed = static_cast<uint64_t>(seed);
  *logger.info() << "Effective master seed: " << effectiveMasterSeed
                 << " (operator: copy this value into eval-mode ini Seed=N to "
                    "reproduce this run bit-deterministically)" << endl;

  // MasterPRNG drives both the autoc-side NN-evolution seed AND the per-
  // scenario seed table. First draw → rng::seed (NN evo); subsequent
  // draws populate scenarioSeedTable. See research.md R6 for the
  // separation rationale.
  gEffectiveMasterSeed = effectiveMasterSeed;  // dmp provenance — recorded into EvalResults at serialize time
  autoc::util::MasterPRNG masterPRNG;
  masterPRNG.init(effectiveMasterSeed);

  rng::seed(masterPRNG.next());

  // scenarioSeedTable will be sized + populated AFTER scenario count is
  // known (path × wind cross product is computed downstream in scenario-
  // build). populateScenarioSeedTable() below is called once that count
  // is known. Stash masterPRNG on the global side for that later call.
  gMasterPRNG = std::move(masterPRNG);

  // AWS setup
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  // 030 V1 priming (2026-05-08) — ThreadPool construction MOVED below
  // (was here at startup, now after loadSourceDmp). Workers are primed
  // with WorkerInit at startup (one-shot RPC right after each worker's
  // TCP accept), so the source-trajectory library + camera/beacon/
  // airframe/flightArena configs MUST be ready before the threadpool
  // launches its workers. Tracker training previously OOM'd a 128 GB
  // box pre-gen-1 because every per-individual EvalData carried a deep
  // copy of the ~8.5 MB source library. See specs/BACKLOG.md
  // "Worker-side scenario priming" entry.

  // Print the configuration. 034 FR-010 — auto-print EVERY active config key
  // from the single-source AUTOC_CONFIG_FIELDS macro (config.h). Adding a knob
  // there auto-parses (config.cc) AND auto-prints here — a run's exact config
  // is always recoverable from its log alone, with no hand-maintained print
  // line to drift. (Tracker-mode keys print their inert defaults in pathgen.)
  *logger.info() << "=== AutocConfig ===" << endl;
#define X(type, field, key) *logger.info() << key << ": " << cfg.field << endl;
  AUTOC_CONFIG_FIELDS(X)
#undef X
  // Compile-time (non-ini) occlusion state — tracker-relevant only.
  if (cfg.mode == "tracker") {
    // 040 T014 — the single-AABB proxy is gone; obstruction is now three
    // primitives (wing slab / pod nose / prop disc) anchored to the camera
    // mount. Still ships DISABLED: the leading-edge mount that makes the
    // geometry meaningful lands in Stage D (T043).
    const auto obstruction = airframeObstructionFromConfig(cfg);
    *logger.info() << "AirframeObstruction: "
                   << (obstruction.enabled ? "enabled" : "DISABLED (transparent)")
                   << " — wing slab + pod nose + prop disc; see airframe_occlusion.h" << endl;
  }

  // 030 M6e — load source dmp at startup for tracker mode (FR-001 + FR-011).
  // Apply path × wind subset; result is the canonical per-scenario source-
  // trajectory bundle that buildEvalData attaches to each EvalData job.
  // Pathgen mode skips this entirely.
  // TODO M6e+: filterCrashedSourceScenarios needs crashReasonList from
  // EvalResults — extend loadSourceDmp to return both, or accept that
  // crashed source scenarios produce short trajectories handled by
  // TrackerStepper's natural source-exhaustion termination.
  if (cfg.mode == "tracker") {
    *logger.info() << "Loading source dmp for tracker mode: " << cfg.trackerSourceRun << endl;
    gSourceTrajectoryList = loadSourceDmp(cfg.trackerSourceRun);
    *logger.info() << "  Loaded " << gSourceTrajectoryList.size() << " scenarios" << endl;
    auto pathSubset = parseCsvIndices(cfg.trackerPathSubset);
    auto windSubset = parseCsvIndices(cfg.trackerWindSubset);
    gSourceTrajectoryList = filterByScenarioIndex(
        gSourceTrajectoryList, pathSubset, windSubset);
    *logger.info() << "  Final scenario count after subset filter: "
                   << gSourceTrajectoryList.size() << endl;
    if (gSourceTrajectoryList.empty()) {
      *logger.info() << "FATAL ERROR: tracker mode requires at least one source "
                        "scenario after filtering; got 0. Check TrackerSourceRun "
                        "and subset config." << endl;
      exit(1);
    }
    // 037: short corner-crash sources (< MIN_SCENARIO_TICKS) are kept (1:1 with
    // the 294 slots) and play through -- terminate cleanly via TimeLimit,
    // contributing their short real fitness. No skip/erase (operator 2026-06-09).
  }

  // 030 V1.5 priming (2026-05-08) — ThreadPool construction MOVED below,
  // past the prefetchAllVariations + generateSmoothPaths +
  // rebuildGenerationScenarios block. buildWorkerInit reads from those
  // globals (gScenarioVariations + generationScenarios), so the path
  // library + variation table must exist before workers spawn. They were
  // formerly called inside the gen loop too — V1.5 hoists them to a
  // single startup call (gPathSeed is run-constant, output is byte-
  // identical every gen anyway).

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

  // 034 US4 — craft-class sigmas + master enable flag. No degree→radian
  // conversion (CG is dimensionless CRRCSim units, trim is radians as
  // configured, others are fractions). EnableCraftVariations is the
  // macro-level disable that matches the entry/wind/rabbit pattern; when
  // off, the PRNG still advances per-scenario (draw-and-discard) and the
  // drawn deltas are zeroed before reaching the FDM.
  gEnableCraftVariations = (cfg.enableCraftVariations != 0);
  // 038 t7 — chase shares the M1 source's per-scenario airspace/airframe seed.
  gChaseUseSourceScenarioSeed = (cfg.trackerChaseUseSourceScenarioSeed != 0);
  gCraftSigmas.craftCGSigma       = cfg.craftCGSigma;
  gCraftSigmas.craftDragSigma     = cfg.craftDragSigma;
  gCraftSigmas.craftTrimSigma     = cfg.craftTrimSigma;
  gCraftSigmas.craftThrustSigma   = cfg.craftThrustSigma;
  gCraftSigmas.craftPitchEffSigma = cfg.craftPitchEffSigma;
  gCraftSigmas.craftRollEffSigma  = cfg.craftRollEffSigma;
  // 037 actuator-dynamics sigmas (servo slew / thrust lag tau).
  gCraftSigmas.craftServoSlewSigma = cfg.craftServoSlewSigma;
  gCraftSigmas.craftThrustTauSigma = cfg.craftThrustTauSigma;

  // Initialize global rabbit speed config.
  // EnableRabbitSpeedVariations=0 forces sigma=0 regardless of the .ini value
  // (same footgun-avoidance pattern as EnableEntryVariations/EnableWindVariations).
  const double effectiveRabbitSpeedSigma = (cfg.enableRabbitSpeedVariations != 0)
                                           ? cfg.rabbitSpeedSigma
                                           : 0.0;
  gRabbitSpeedConfig = RabbitSpeedConfig{
      cfg.rabbitSpeedNominal,
      effectiveRabbitSpeedSigma,
      cfg.rabbitSpeedMin,
      cfg.rabbitSpeedMax,
      cfg.rabbitSpeedCycleMin,
      cfg.rabbitSpeedCycleMax
  };

  // Log rabbit speed configuration
  *logger.info() << "RabbitSpeed: nominal=" << cfg.rabbitSpeedNominal << " m/s"
                 << " sigma=" << effectiveRabbitSpeedSigma << " m/s"
                 << " range=[" << cfg.rabbitSpeedMin << ", " << cfg.rabbitSpeedMax << "] m/s"
                 << " cycles=[" << cfg.rabbitSpeedCycleMin << ", " << cfg.rabbitSpeedCycleMax << "] s"
                 << (effectiveRabbitSpeedSigma > 0 ? " (VARIABLE)" : " (CONSTANT)") << endl;

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

  // 033 §2.A — Populate gScenarioSeedTable BEFORE prefetchAllVariations,
  // since the new class-PRNG-based prefetch consumes from per-scenario
  // ScenarioRootPRNGs derived from scenarioSeed[K]. Total scenario count
  // = paths × winds (path-major layout, matching scenarioMetaList).
  int windScenarioCount = std::max(cfg.windScenarioCount, 1);
  const size_t totalScenarioCount =
      static_cast<size_t>(std::max(cfg.simNumPathsPerGen, 1)) *
      static_cast<size_t>(windScenarioCount);
  populateScenarioSeedTable(totalScenarioCount);

  // 038 t7 — chase-shares-source-airspace guard (Constitution V loud-fail).
  // The swap in chaseScenarioSeedAt() pairs source scenario i with chase
  // scenario i positionally, which is only valid when the source list is 1:1
  // with the seed table (full path×wind, no subset). Tracker-only.
  if (gChaseUseSourceScenarioSeed) {
    if (cfg.mode != "tracker") {
      *logger.info() << "FATAL ERROR: TrackerChaseUseSourceScenarioSeed=1 is "
                        "tracker-only (mode=" << cfg.mode << ")." << endl;
      exit(1);
    }
    if (gSourceTrajectoryList.size() != gScenarioSeedTable.size()) {
      *logger.info() << "FATAL ERROR: TrackerChaseUseSourceScenarioSeed=1 "
                        "requires the source list (" << gSourceTrajectoryList.size()
                     << ") to be 1:1 with the scenario seed table ("
                     << gScenarioSeedTable.size() << ") — use the full path×wind "
                        "set (empty TrackerPathSubset/TrackerWindSubset)." << endl;
      exit(1);
    }
    *logger.info() << "TrackerChaseUseSourceScenarioSeed=1: chase shares the M1 "
                      "source's per-scenario wind/thermal/gust/entry/craft/"
                      "crash-hull seeds (" << gScenarioSeedTable.size()
                   << " scenarios); NN + camera stay on the M2 seed." << endl;
  }

  // Pre-fetch all scenario variations from class-scoped PRNGs derived
  // from scenarioSeed[K]. 034 FR-012: per-(path, wind) — gScenarioVariations
  // is sized totalScenarioCount (paths × winds) and indexed by the linear
  // path-major K, so each (path, wind) gets distinct entry/wind offsets.
  // When a variation type is disabled, defaults are stored in the table BUT
  // the class PRNG still draws (draw-and-discard per spec §2.E + Clarifications
  // 2026-05-21).
  prefetchAllVariations(static_cast<int>(totalScenarioCount), gVariationSigmas, gRabbitSpeedConfig,
                        cfg.randomPathSeedB,
                        gEnableEntryVariations, gEnableWindVariations,
                        gEnableCraftVariations);

  // Log pre-fetched variations for verification
  *logger.info() << "Sigmas: cone=" << cfg.entryConeSigma << "° "
                 << "roll=" << cfg.entryRollSigma << "° "
                 << "speed=" << (cfg.entrySpeedSigma * 100) << "% "
                 << "wind=" << cfg.windDirectionSigma << "° "
                 << "posR=" << cfg.entryPositionRadiusSigma << "m "
                 << "posAlt=" << cfg.entryPositionAltSigma << "m" << endl;
  logPrefetchedVariations(static_cast<int>(totalScenarioCount), seed);

  // 035 FR-P05 — data.dat output file retired; the dmp (S3) is the single
  // training-trace artifact, inspected post-hoc via the dmp-dump tool.

  // 034 FR-015 — tracker (M2) runs get a "tracker-" run-id prefix so they're
  // distinguishable from pathgen (M1) "autoc-" runs in the shared S3 bucket.
  std::string startTime = generate_iso8601_timestamp(
      autoc::runIdPrefixForMode(cfg.mode));
  auto runStartTime = std::chrono::steady_clock::now();

  // 030 M8b — explicit output-bucket logging (operator request 2026-05-07).
  // Same run-id, profile, and bucket are used across pathgen training,
  // pathgen eval, tracker training, and (eventual) tracker eval modes —
  // single log line covers all four. Per-gen training dmps land at
  // <prefix>/gen<10000-gen>.dmp (reverse-time so newest gen lists first);
  // eval-mode dmps at gen9999.dmp.
  //
  // Format `profile:bucket/key` matches nnextractor's genome.source
  // provenance convention so the prefix string is copy-pastable across
  // tools (renderer / *_dmp_inspect / nnextractor accept this form).
  *logger.info() << "Output S3 prefix: " << cfg.s3Profile << ":" << cfg.s3Bucket
                 << "/" << startTime << "/  (gen<N>.dmp per-gen; gen9999.dmp for eval mode)"
                 << endl;
  *logger.info() << "Run ID: " << startTime << endl;
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
    *logger.info() << std::fixed << std::setprecision(2)
         << "#GenSimStats gen=" << genIndex
         << " sims=" << deltaRuns
         << " total=" << currentRuns
         << " durationSec=" << deltaSec
         << " rate=" << rate
         << std::setprecision(6) << std::endl;
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
                 << " evalsPerIndividual=" << cfg.simNumPathsPerGen * windsPerPath
                 << endl;
  warnIfScenarioMismatch();

  // 030 V1 + V1.5 priming — initialize workers NOW that all run-static
  // state (gSourceTrajectoryList, gScenarioVariations, generationScenarios
  // / generationPaths) is populated. buildWorkerInit pulls from those
  // globals to construct the once-per-worker payload (sourceList,
  // pathList, scenarioMetaList, tracker configs); workers cache it and
  // per-eval EvalData stays at ~50 B.
  *logger.info() << "Priming workers (mode=" << cfg.mode
                 << ", pathList=" << generationScenarios.front().pathList.size()
                 << " scenarios"
                 << (parseModeName(cfg.mode) == Mode::TRACKER
                         ? (", sourceList=" + std::to_string(gSourceTrajectoryList.size()))
                         : std::string())
                 << ")..." << endl;
  threadPool = new ThreadPool(ConfigManager::getConfig(), buildWorkerInit());

  if (cfg.evaluateMode) {
    // NN evaluation mode: load weight file, evaluate, report fitness
    runNNEvaluation(startTime);
  } else {
    // NN evolution mode
    runNNEvolution(startTime, runStartTime, logGenerationStats);
  }


  uint64_t totalRuns = globalSimRunCounter.load(std::memory_order_relaxed);
  gp_scalar durationSec = std::chrono::duration<gp_scalar>(std::chrono::steady_clock::now() - runStartTime).count();
  gp_scalar simsPerSec = (durationSec > static_cast<gp_scalar>(0.0f)) ? static_cast<gp_scalar>(totalRuns) / durationSec : static_cast<gp_scalar>(0.0f);
  *logger.info() << "#SimRuns " << totalRuns << " DurationSec " << durationSec << std::endl;
  *logger.info() << std::fixed << std::setprecision(2)
                 << "Simulation throughput: " << totalRuns << " runs in "
                 << durationSec << "s (" << simsPerSec << " sims/s)" << std::defaultfloat << std::endl;
}

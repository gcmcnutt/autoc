#pragma once

// 030 M6e — ScenarioMetadata factored out of protocol.h to break the
// circular include path between protocol.h and source_trajectory.h.
//
// Both headers need ScenarioMetadata: protocol.h to define EvalData /
// EvalResults, and source_trajectory.h to define SourceScenarioTrajectory
// (which holds a ScenarioMetadata variation by value). Before this split,
// source_trajectory.h included protocol.h directly, which post-M6e tried
// to include source_trajectory.h back — guard-skipped, leaving
// SourceScenarioTrajectory undefined when EvalData referenced it.
//
// Definition + cereal serialize stays here; protocol.h re-includes this
// header so existing protocol.h consumers see the type unchanged.

#include <cstdint>

#include <cereal/cereal.hpp>

#include "autoc/types.h"  // gp_vec3

/*
 * Per-scenario metadata flowing from autoc → workers and recorded into
 * EvalResults for downstream replay. See plan.md / data-model.md.
 */
struct ScenarioMetadata {
    int pathVariantIndex = -1;   // -1 = unset/aggregated
    int windVariantIndex = -1;   // -1 = unset/aggregated
    unsigned int windSeed = 0;
    uint64_t scenarioSequence = 0;
    uint64_t bakeoffSequence = 0;
    bool enableDeterministicLogging = false;

    // VARIATIONS1: Entry and wind direction offsets (computed by autoc, applied by crrcsim)
    // All angles in radians, speed as multiplier
    double entryHeadingOffset = 0.0;   // radians, offset from path tangent
    double entryRollOffset = 0.0;      // radians, initial roll attitude
    double entryPitchOffset = 0.0;     // radians, initial pitch attitude
    double entrySpeedFactor = 1.0;     // multiplier on reference speed
    double windDirectionOffset = 0.0;  // radians, offset from base wind direction

    // Entry position offsets (see specs/005-entry-fitness-ramp)
    double entryNorthOffset = 0.0;     // meters, NED North
    double entryEastOffset = 0.0;      // meters, NED East
    double entryAltOffset = 0.0;       // meters, NED Down (negative=up)

    // Rabbit speed for odometer-based path traversal (m/s)
    double rabbitSpeed = 0.0;          // 0 = use default SIM_INITIAL_VELOCITY
    unsigned int rabbitSpeedSeed = 0;  // per-scenario seed for local speed profile PRNG

    // Raw→virtual origin offset captured at test start (NED meters).
    // Used by renderer to reconstruct raw positions for "all flights" display.
    // See docs/COORDINATE_CONVENTIONS.md "Virtual Frame" section.
    gp_vec3 originOffset = gp_vec3::Zero();

    template<class Archive>
    void serialize(Archive& ar, const std::uint32_t /*version*/) {
        ar(pathVariantIndex, windVariantIndex, windSeed, scenarioSequence,
           bakeoffSequence, enableDeterministicLogging, entryHeadingOffset,
           entryRollOffset, entryPitchOffset, entrySpeedFactor,
           windDirectionOffset, entryNorthOffset, entryEastOffset, entryAltOffset,
           rabbitSpeed, rabbitSpeedSeed, originOffset);
    }
};
CEREAL_CLASS_VERSION(ScenarioMetadata, 1)

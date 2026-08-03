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
    uint64_t scenarioSequence = 0;
    uint64_t bakeoffSequence = 0;
    bool enableDeterministicLogging = false;

    // 033 §2.A — per-scenario master-derived seed. Sole per-scenario PRNG
    // input post-033. Consumers reconstruct ScenarioRootPRNG from this and
    // derive 5 class sub-PRNGs (wind/rabbit/entry/craft/camera) via
    // autoc::util::deriveClassSubSeeds(). Sufficient for full replay; per-
    // class sub-seeds are NOT persisted (derived deterministically). Pre-
    // 033 windSeed + rabbitSpeedSeed fields removed in same PR per spec
    // Clarifications 2026-05-21 (Constitution III: no compatibility shims).
    uint64_t scenarioSeed = 0;

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
    // (033) rabbitSpeedSeed removed: worker derives the rabbit-class PRNG
    // seed from scenarioSeed via deriveClassSubSeeds() → .rabbit slot.

    // Raw→virtual origin offset captured at test start (NED meters).
    // Used by renderer to reconstruct raw positions for "all flights" display.
    // See docs/COORDINATE_CONVENTIONS.md "Virtual Frame" section.
    gp_vec3 originOffset = gp_vec3::Zero();

    // 034 US4 — craft variations (per-scenario airframe parameter draws).
    // Drawn at full magnitude per scenarioSeed (deterministic / replay
    // invariant); the per-eval applied magnitude is scaled by applyVariationScale()
    // — same pipeline as wind/entry. Training: scale = computeVariationScale(gen);
    // eval: scale = genome.variation_scale (saved in weight file, replayed via
    // gEvalVariationScaleOverride — not 100%, not recomputed).
    //
    // Defaults are no-op semantics: 0.0 deltas and 1.0 scale leave the FDM at
    // its nominal airframe. Worker derives the craft-class PRNG seed from
    // scenarioSeed via deriveClassSubSeeds().craft and writes the draws here
    // for the FDM to apply at scenario init (after the per-eval scale).
    // craftSeed is the persistence root for replay (the per-scenario PRNG
    // seed that drove these draws — recorded so a dmp alone reconstructs the
    // airframe state without the master seed). gp_scalar per Constitution VI.
    gp_scalar craftCGDelta = static_cast<gp_scalar>(0.0);       // CG arm offset (dimensionless CRRCSim MAC units)
    gp_scalar craftDragDelta = static_cast<gp_scalar>(0.0);     // CD_prof fractional Δ (≈ ±5%)
    gp_scalar craftTrimDelta = static_cast<gp_scalar>(0.0);     // Cm_0 trim offset (rad)
    gp_scalar craftThrustScale = static_cast<gp_scalar>(1.0);   // engine maxThrust multiplier
    gp_scalar craftPitchEffDelta = static_cast<gp_scalar>(0.0); // pitch authority fractional Δ
    gp_scalar craftRollEffDelta = static_cast<gp_scalar>(0.0);  // roll authority fractional Δ
    // 037 actuator-dynamics axes -- ABSOLUTE physical values (per-second /
    // seconds), not deltas. Defaults = nominal centers so a no-craft scenario
    // runs the nominal model. Set by inputdev_autoc on the per-scenario
    // actuator filter (servo slew, thrust lag) instead of the constants.
    // (servo first-order tau removed 2026-06-12 — v2 PWM-latch+slew has no lag
    // term; greenfield wire shrink, no version bump — old dmps fail-loud.)
    gp_scalar craftServoSlew = static_cast<gp_scalar>(24.0);    // servo slew rate (autoc [-1,1] units/s; v2 center ≈24.2)
    gp_scalar craftThrustTau = static_cast<gp_scalar>(0.150);   // thrust lag tau (s)
    uint32_t craftSeed = 0;                                     // craft-class PRNG seed (replay root)
    // 037 servo v2 — per-scenario 50 Hz PWM latch phase (s, uniform
    // [0, 0.020)); the real command dead-time. Appended after craftSeed so
    // the wire prefix is preserved (greenfield growth, no version bump).
    gp_scalar craftServoPwmPhase = static_cast<gp_scalar>(0.0);
    // 040 US6 (FR-021/FR-022) — CAMERA variation. The insertion point reserved
    // by FR-020 is now taken; appended after craftServoPwmPhase rather than
    // interleaved, so the wire prefix is preserved (greenfield growth, no
    // version bump — old dmps fail loud on length mismatch, by policy).
    //
    // Raw PRE-SCALE draws are recorded, not the post-ramp values, which is what
    // makes variation verifiable RAMP-INDEPENDENTLY via `dmp-dump --meta-only`.
    //
    // Defaults are the NOMINAL camera so a no-variation scenario is bit-identical
    // to having no camera-variation code at all. Note ambientScale defaults to
    // 1.0, not 0.0 — it is a scale, not a delta.
    gp_scalar cameraBoresightYawDeg = static_cast<gp_scalar>(0.0);
    gp_scalar cameraBoresightPitchDeg = static_cast<gp_scalar>(0.0);
    // Highest-impact term: roll rotates the image plane and so biases the
    // port→starboard tilt cue degree-for-degree, and tilt drives roll command.
    gp_scalar cameraRollDeg = static_cast<gp_scalar>(0.0);
    // OBSTRUCTION-path only (research R6) — 0.03° at 10 m for bearing, but ~15%
    // of propeller clearance.
    gp_vec3 cameraMountTranslation{static_cast<gp_scalar>(0.0), static_cast<gp_scalar>(0.0),
                                   static_cast<gp_scalar>(0.0)};
    gp_scalar cameraWingThicknessDelta = static_cast<gp_scalar>(0.0);
    // Held at nominal for the t2 pass (operator 2026-08-02: emitter stays
    // perfect until the lens+filter field tests pin SignalAmbientKnee).
    gp_scalar cameraAmbientScale = static_cast<gp_scalar>(1.0);
    uint32_t cameraSeed = 0;  // camera-class PRNG seed (replay root)

    template<class Archive>
    void serialize(Archive& ar, const std::uint32_t /*version*/) {
        // 033 cleanup: windSeed + rabbitSpeedSeed REMOVED from the walk.
        // scenarioSeed at the end (appended in T006). Old pre-033 dmps
        // fail-loud on cereal length mismatch — Constitution III no-shim +
        // Constitution V loud-fail safety net (intentional break).
        // 034 US4 craft variations appended after scenarioSeed per project
        // no-cereal-versioning policy: greenfield wire growth, no version
        // bump; old 034-pre-US4 dmps fail-loud on length mismatch.
        ar(pathVariantIndex, windVariantIndex, scenarioSequence,
           bakeoffSequence, enableDeterministicLogging, entryHeadingOffset,
           entryRollOffset, entryPitchOffset, entrySpeedFactor,
           windDirectionOffset, entryNorthOffset, entryEastOffset, entryAltOffset,
           rabbitSpeed, originOffset, scenarioSeed,
           craftCGDelta, craftDragDelta, craftTrimDelta, craftThrustScale,
           craftPitchEffDelta, craftRollEffDelta,
           craftServoSlew, craftThrustTau,  // 037 actuator dynamics (servoTau removed 2026-06-12)
           craftSeed,
           craftServoPwmPhase,  // 037 servo v2 -- appended, no version bump
           // 040 US6 camera variation -- appended, no version bump
           cameraBoresightYawDeg, cameraBoresightPitchDeg, cameraRollDeg,
           cameraMountTranslation, cameraWingThicknessDelta, cameraAmbientScale,
           cameraSeed);
    }
};
CEREAL_CLASS_VERSION(ScenarioMetadata, 1)

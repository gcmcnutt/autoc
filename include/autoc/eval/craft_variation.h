#pragma once

// 034 US4 — Craft (airframe) variation sampling.
//
// Parallel to `variation_generator.h` for the entry/wind classes: a single
// header-only module that draws per-scenario airframe-parameter deltas from
// a class PRNG. Drawn at full magnitude (variation_scale = 1.0); the per-eval
// scaling is the worker's `applyVariationScale()` job — same pipeline as
// wind/entry. See contracts/craft-variation-contract.md for the full spec
// including the FDM-application formulas.
//
// Why a separate module: craft is its own variation CLASS in the per-scenario
// PRNG cascade (`ClassSubSeeds::craft`), distinct from the entry class. The
// draws also have a different shape (additive vs multiplicative per axis)
// and different units (m for CG, rad for trim, fraction for the rest). The
// math is small and inline so a header is sufficient.

#include "autoc/types.h"            // gp_scalar
#include "autoc/util/scenario_prng.h"  // autoc::util::ClassPRNG

namespace autoc::eval {

/**
 * Sigma parameters for craft variation draws. All in their respective
 * intrinsic units (NOT fractions of nominal except where noted) so the
 * caller doesn't need a per-axis multiplier.
 *
 *   craftCGSigma         meters     — CG arm offset Gaussian sigma
 *   craftDragSigma       fraction   — CD_prof fractional sigma (so 0.05 = ±5%)
 *   craftTrimSigma       radians    — Cm_0 trim Gaussian sigma
 *   craftThrustSigma     fraction   — maxThrust fractional sigma (centered at 1.0)
 *   craftPitchEffSigma   fraction   — pitch-authority fractional sigma
 *   craftRollEffSigma    fraction   — roll-authority fractional sigma
 *
 * Constructed once per run from AutocConfig at startup. Zero σ on every
 * axis is a true no-op (every draw → 0.0 delta / 1.0 scale).
 */
struct CraftSigmas {
    double craftCGSigma = 0.0;
    double craftDragSigma = 0.0;
    double craftTrimSigma = 0.0;
    double craftThrustSigma = 0.0;
    double craftPitchEffSigma = 0.0;
    double craftRollEffSigma = 0.0;
};

/**
 * Drawn per-scenario craft deltas at FULL magnitude. The caller copies these
 * into `ScenarioMetadata.craft*` fields; the worker later scales them down
 * via `applyVariationScale(meta, variationScale)` before passing to the FDM.
 *
 * Units match ScenarioMetadata (gp_scalar). The thrust field is a SCALE
 * (1.0 = nominal), not a delta — matches the FDM-side multiplier semantics.
 */
struct CraftDeltas {
    gp_scalar craftCGDelta = static_cast<gp_scalar>(0.0);
    gp_scalar craftDragDelta = static_cast<gp_scalar>(0.0);
    gp_scalar craftTrimDelta = static_cast<gp_scalar>(0.0);
    gp_scalar craftThrustScale = static_cast<gp_scalar>(1.0);
    gp_scalar craftPitchEffDelta = static_cast<gp_scalar>(0.0);
    gp_scalar craftRollEffDelta = static_cast<gp_scalar>(0.0);
};

/**
 * Draw one full-magnitude set of craft deltas from a per-scenario craft
 * PRNG. Deterministic for a fixed PRNG state (which is itself derived from
 * `scenarioSeed` via `deriveClassSubSeeds(...).craft` upstream).
 *
 * Draw order is FROZEN — adding or reordering draws shifts the wire-format
 * value the seed produces and breaks bit-exact replay. To add a new craft
 * axis later, append the draw at the bottom and append the field to
 * `ScenarioMetadata` + `CraftDeltas` last.
 *
 * The thrust draw is centered at 1.0 (multiplicative scale, like
 * `entrySpeedFactor` in entry-class draws).
 */
inline CraftDeltas generateCraftFromClassPRNG(
        autoc::util::ClassPRNG& craftPRNG, const CraftSigmas& sigmas) {
    CraftDeltas d;
    d.craftCGDelta       = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftCGSigma));
    d.craftDragDelta     = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftDragSigma));
    d.craftTrimDelta     = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftTrimSigma));
    d.craftThrustScale   = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftThrustSigma));
    d.craftPitchEffDelta = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftPitchEffSigma));
    d.craftRollEffDelta  = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftRollEffSigma));
    return d;
}

}  // namespace autoc::eval

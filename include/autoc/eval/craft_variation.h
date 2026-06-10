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

#include <algorithm>               // std::min / std::max (clamp of dynamics axes)

#include "autoc/types.h"            // gp_scalar
#include "autoc/util/scenario_prng.h"  // autoc::util::ClassPRNG

namespace autoc::eval {

// 037 actuator-dynamics craft axes -- nominal physical CENTERS and positive
// physical clamp ranges. Unlike the six fractional/additive axes above, these
// three are absolute physical quantities (a 4-craft fleet differs in
// servos/motors). The Gaussian delta is added to the center, then clamped:
// a pure 2.5-sigma draw on tau_servo would go negative, so the clamp is
// MANDATORY. Centers/ranges are shared by the worker-side apply path
// (scenario_meta_apply.h) so disabled / sigma=0 collapses to the center.
constexpr gp_scalar kCraftServoTauCenter  = static_cast<gp_scalar>(0.020);  // s
constexpr gp_scalar kCraftServoTauMin     = static_cast<gp_scalar>(0.005);  // s
constexpr gp_scalar kCraftServoTauMax     = static_cast<gp_scalar>(0.050);  // s
constexpr gp_scalar kCraftServoSlewCenter = static_cast<gp_scalar>(6.0);    // /s (full-throw/s)
constexpr gp_scalar kCraftServoSlewMin    = static_cast<gp_scalar>(3.0);    // /s
constexpr gp_scalar kCraftServoSlewMax    = static_cast<gp_scalar>(9.0);    // /s
constexpr gp_scalar kCraftThrustTauCenter = static_cast<gp_scalar>(0.150);  // s
constexpr gp_scalar kCraftThrustTauMin    = static_cast<gp_scalar>(0.050);  // s
constexpr gp_scalar kCraftThrustTauMax    = static_cast<gp_scalar>(0.300);  // s

/**
 * Sigma parameters for craft variation draws. All in their respective
 * intrinsic units (NOT fractions of nominal except where noted) so the
 * caller doesn't need a per-axis multiplier.
 *
 *   craftCGSigma         dimensionless — CG arm Gaussian sigma (CRRCSim's CG_arm is in MAC/chord-fraction units; hb1_streamer nominal ≈ 0.28)
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
    // ini-loaded config-struct fields — inih::GetReal yields a 64-bit real;
    // gp_scalar conversion happens at the consumption boundary in
    // generateCraftFromClassPRNG when the drawn value is written into
    // ScenarioMetadata (gp_scalar throughout). Mirrors the VariationSigmas
    // struct convention in variation_generator.h.
    double craftCGSigma = 0.0;        // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftDragSigma = 0.0;      // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftTrimSigma = 0.0;      // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftThrustSigma = 0.0;    // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftPitchEffSigma = 0.0;  // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftRollEffSigma = 0.0;   // raw-ok: ini config-struct field (double per inih::GetReal)
    // 037 actuator-dynamics axes -- sigma in intrinsic units (seconds for the
    // tau axes, full-throw/s for slew). Center + clamp live in the constants
    // above; these are just the per-scenario Gaussian widths.
    double craftServoTauSigma = 0.0;  // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftServoSlewSigma = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftThrustTauSigma = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
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
    // 037 actuator-dynamics axes -- ABSOLUTE physical values (center + clamped
    // Gaussian delta), NOT deltas. Default = nominal center so the disabled /
    // sigma=0 path collapses to the nominal lag model. The worker ramps these
    // toward the center per-eval (applyVariationScale), same as the others.
    gp_scalar craftServoTau   = kCraftServoTauCenter;    // s
    gp_scalar craftServoSlew  = kCraftServoSlewCenter;   // /s (full-throw/s)
    gp_scalar craftThrustTau  = kCraftThrustTauCenter;   // s
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
    // 037 actuator-dynamics axes -- APPENDED at the bottom (draw order frozen;
    // see the contract note above). Each is center + Gaussian(sigma), then
    // clamped to its positive physical range. The clamp is mandatory: a
    // 2.5-sigma low draw on tau_servo (center 0.020, sigma 0.010) would reach
    // -0.005 and produce a non-physical / unstable filter.
    {
        gp_scalar servoTau = kCraftServoTauCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftServoTauSigma));
        d.craftServoTau = std::min(kCraftServoTauMax, std::max(kCraftServoTauMin, servoTau));

        gp_scalar servoSlew = kCraftServoSlewCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftServoSlewSigma));
        d.craftServoSlew = std::min(kCraftServoSlewMax, std::max(kCraftServoSlewMin, servoSlew));

        gp_scalar thrustTau = kCraftThrustTauCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftThrustTauSigma));
        d.craftThrustTau = std::min(kCraftThrustTauMax, std::max(kCraftThrustTauMin, thrustTau));
    }
    return d;
}

}  // namespace autoc::eval

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
// 037 servo model v2 (operator 2026-06-11, post-t6/t7 A/B + DSM-44 datasheet
// check in finding.md): the v1 lag-dominant model (tau 20 ms + slew 3–9,
// shaded slow) capped tracking. v2 is datasheet-shaped: a 50 Hz PWM command
// LATCH (per-scenario phase, 0–20 ms — the real dead-time) plus pure slew
// derived from the measured transit: ~50–60 ms per 60° of travel once
// latched, on a ~90° mechanical span (1000–2000 µs).
//
// UNITS (operator 2026-06-12, post-t8 audit): slew is in AUTOC command
// units/s — the [-1, +1] NN/INAV span, full range = 2.0 units = the full
// mechanical span. Platform code translates at its boundary (crrcsim
// surfaces are [-0.5, +0.5] = command/2, hence its ×0.5 on the cap; INAV
// would consume [-1,1] natively). center = (60°/0.055 s)/90° spans/s × 2.0
// units/span ≈ 24.2 units/s (full-span transit 82.5 ms); clamp brackets a
// fast no-load servo (32) down to a heavily loaded one (16). The pre-fix
// center 12.1 was derived in spans/s but consumed as units/s — an
// accidental 2×-slow servo (t6/t8 deadening; finding.md §t8-final).
constexpr gp_scalar kServoTransitSecPer60Deg = static_cast<gp_scalar>(0.055);
constexpr gp_scalar kServoMechSpanDeg        = static_cast<gp_scalar>(90.0);
constexpr gp_scalar kServoSpanCommandUnits   = static_cast<gp_scalar>(2.0);  // [-1,1] full range
constexpr gp_scalar kCraftServoSlewCenter =
    (static_cast<gp_scalar>(60.0) / kServoTransitSecPer60Deg) / kServoMechSpanDeg
    * kServoSpanCommandUnits;                                                // ≈24.2 /s (autoc [-1,1] units)
constexpr gp_scalar kCraftServoSlewMin    = static_cast<gp_scalar>(16.0);   // /s (heavy aero load)
constexpr gp_scalar kCraftServoSlewMax    = static_cast<gp_scalar>(32.0);   // /s (fast no-load)
constexpr gp_scalar kCraftServoPwmFrameSec = static_cast<gp_scalar>(0.020); // 50 Hz PWM command frame
constexpr gp_scalar kCraftThrustTauCenter = static_cast<gp_scalar>(0.150);  // s
constexpr gp_scalar kCraftThrustTauMin    = static_cast<gp_scalar>(0.050);  // s
constexpr gp_scalar kCraftThrustTauMax    = static_cast<gp_scalar>(0.300);  // s

// 043 US5 -- craftCmQ pitch-damping axis (FR-052b). ABSOLUTE physical value +
// clamp, like craftServoSlew above -- NOT a delta. Center + clamp pre-derived
// in crrcsim/models/hb1_streamer.xml: "Cm_q range -3.6 (no streamer) to -5.0
// (25 ft streamer)". The clamp is mandatory: a 2.5-sigma draw could otherwise
// leave the physical range. ⛔ This is the DYNAMIC pitch-damping side only;
// the static-margin side is already carried by craftCGDelta (CG vs neutral
// point) -- see contracts/craft-imu-axes.md.
constexpr gp_scalar kCraftCmQCenter = static_cast<gp_scalar>(-4.2);
constexpr gp_scalar kCraftCmQMin    = static_cast<gp_scalar>(-5.0);
constexpr gp_scalar kCraftCmQMax    = static_cast<gp_scalar>(-3.6);

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
    // 037 actuator-dynamics axes -- sigma in intrinsic units (autoc [-1,1]
    // command-units/s for slew, seconds for the thrust tau). Center + clamp
    // live in the constants above; these are just the per-scenario Gaussian
    // widths. (servo first-order tau removed 2026-06-12 — v2 is a PWM latch +
    // pure slew, no lag term; the dead draw is gone.)
    double craftServoSlewSigma = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
    double craftThrustTauSigma = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
    // 043 US5 -- IMU imperfection + pitch-damping axes. One sigma per GROUP
    // (all three misalign axes share craftImuMisalignSigma, etc.); the FDM feeds
    // the SAME FC IMU to INAV's rate feedback and the policy's observation, so
    // the axes must be consistent across both loops (contracts/craft-imu-axes.md).
    double craftImuMisalignSigma = 0.0; // raw-ok: ini config-struct field. deg; 2.5σ = ±5° foam-tape mount
    double craftGyroScaleSigma   = 0.0; // raw-ok: ini config-struct field. fraction; 2.5σ = ±5% cal tolerance
    double craftAccelScaleSigma  = 0.0; // raw-ok: ini config-struct field. fraction; 2.5σ = ±5%
    double craftAccelBiasSigma   = 0.0; // raw-ok: ini config-struct field. g; from the 041-t7 audit (accel_y ~1.7σ)
    double craftCmQSigma         = 0.0; // raw-ok: ini config-struct field. Cm_q Gaussian width (center/clamp above)
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
    gp_scalar craftServoSlew  = kCraftServoSlewCenter;   // /s (autoc [-1,1] units)
    gp_scalar craftThrustTau  = kCraftThrustTauCenter;   // s
    // 037 servo v2 -- per-scenario PWM latch phase, uniform [0, frame). An
    // engage episode lands at an arbitrary offset within the 50 Hz command
    // frame; this is the operator's "0-20 ms PWM lag jitter" (constant within
    // a scenario -- command instants are periodic, so the phase IS the lag).
    gp_scalar craftServoPwmPhase = static_cast<gp_scalar>(0.0);  // s, [0, 0.020)
    // 043 US5 -- IMU imperfection axes (deg / fraction / g) + pitch damping.
    // Misalign/bias are ADDITIVE (default 0 = perfect), gyro/accel scale are
    // MULTIPLICATIVE (default 1.0 = nominal), craftCmQ is an ABSOLUTE value
    // (default = center). Appended last so draw order is frozen.
    gp_scalar craftImuMisalignRoll  = static_cast<gp_scalar>(0.0);  // deg
    gp_scalar craftImuMisalignPitch = static_cast<gp_scalar>(0.0);  // deg
    gp_scalar craftImuMisalignYaw   = static_cast<gp_scalar>(0.0);  // deg
    gp_scalar craftGyroScaleX = static_cast<gp_scalar>(1.0);
    gp_scalar craftGyroScaleY = static_cast<gp_scalar>(1.0);
    gp_scalar craftGyroScaleZ = static_cast<gp_scalar>(1.0);
    gp_scalar craftAccelScaleX = static_cast<gp_scalar>(1.0);
    gp_scalar craftAccelScaleY = static_cast<gp_scalar>(1.0);
    gp_scalar craftAccelScaleZ = static_cast<gp_scalar>(1.0);
    gp_scalar craftAccelBiasX = static_cast<gp_scalar>(0.0);  // g
    gp_scalar craftAccelBiasY = static_cast<gp_scalar>(0.0);  // g
    gp_scalar craftAccelBiasZ = static_cast<gp_scalar>(0.0);  // g
    gp_scalar craftCmQ = kCraftCmQCenter;                     // pitch damping (absolute + clamp)
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
    // 037 actuator-dynamics axes -- APPENDED at the bottom. Each is center +
    // Gaussian(sigma), then clamped to its positive physical range. The clamp
    // is mandatory: a 2.5-sigma low draw could otherwise reach a non-physical
    // value. (servo first-order tau draw removed 2026-06-12 — v2 has no lag
    // term; draw order shifts accordingly, which is fine cross-build.)
    {
        gp_scalar servoSlew = kCraftServoSlewCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftServoSlewSigma));
        d.craftServoSlew = std::min(kCraftServoSlewMax, std::max(kCraftServoSlewMin, servoSlew));

        gp_scalar thrustTau = kCraftThrustTauCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftThrustTauSigma));
        d.craftThrustTau = std::min(kCraftThrustTauMax, std::max(kCraftThrustTauMin, thrustTau));
    }
    // 037 servo v2 -- PWM latch phase, APPENDED (draw order frozen). Drawn
    // UNCONDITIONALLY (even when the servo model is disabled) so toggling
    // ServoModelEnabled does not shift any other draw — same
    // draw-and-discard convention as the wind-class gating.
    d.craftServoPwmPhase = static_cast<gp_scalar>(craftPRNG.nextDouble())
        * kCraftServoPwmFrameSec;
    // 043 US5 -- IMU imperfection axes + pitch damping, APPENDED at the bottom
    // (draw order frozen; every draw above keeps its value). Drawn
    // UNCONDITIONALLY at their own sigma so sigma=0 is a bit-identical no-op
    // (gaussian(0)=0 for the additive axes; 1.0+gaussian(0)=1.0 for the scales;
    // center+gaussian(0)=center for cmQ) and toggling never shifts a draw --
    // same draw-and-discard convention as the servo/PWM axes above. 2.5-sigma
    // truncation is intrinsic to nextGaussian; no bespoke clip constants.
    d.craftImuMisalignRoll  = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftImuMisalignSigma));
    d.craftImuMisalignPitch = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftImuMisalignSigma));
    d.craftImuMisalignYaw   = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftImuMisalignSigma));
    d.craftGyroScaleX = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftGyroScaleSigma));
    d.craftGyroScaleY = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftGyroScaleSigma));
    d.craftGyroScaleZ = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftGyroScaleSigma));
    d.craftAccelScaleX = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftAccelScaleSigma));
    d.craftAccelScaleY = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftAccelScaleSigma));
    d.craftAccelScaleZ = static_cast<gp_scalar>(1.0 + craftPRNG.nextGaussian(sigmas.craftAccelScaleSigma));
    d.craftAccelBiasX = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftAccelBiasSigma));
    d.craftAccelBiasY = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftAccelBiasSigma));
    d.craftAccelBiasZ = static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftAccelBiasSigma));
    {
        // craftCmQ -- absolute center + clamp, like craftServoSlew (FR-052b).
        gp_scalar cmq = kCraftCmQCenter
            + static_cast<gp_scalar>(craftPRNG.nextGaussian(sigmas.craftCmQSigma));
        d.craftCmQ = std::min(kCraftCmQMax, std::max(kCraftCmQMin, cmq));
    }
    return d;
}

}  // namespace autoc::eval

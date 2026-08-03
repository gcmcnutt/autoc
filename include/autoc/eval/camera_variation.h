#pragma once

// 040 T071-T075 (US6, FR-021/FR-022/FR-023) — per-scenario camera variation.
//
// A DIRECT MIRROR of `craft_variation.h`, deliberately: same struct pair
// (Sigmas / Deltas), same header-only generator, same draw-and-discard call
// discipline in autoc.cc. The
// variation pipeline already works; camera is a new CLASS in it, not a new
// mechanism.
//
// WHAT IT IS FOR. Through 040 t1 the controller trained against a PERFECTLY
// known camera — exact boresight, exact roll, exact mount, identical in every
// scenario. That is the one assumption a real airframe cannot honour.
//
// ROLL IS THE HIGHEST-IMPACT TERM, not boresight. Under the angular
// representation a boresight error lands as a near-constant additive offset on
// both bearings — clean, and learnable as a subtraction. ROLL rotates the image
// plane, so it biases the port→starboard TILT cue degree-for-degree, and tilt
// drives the roll command. That is why roll gets its own sigma rather than being
// folded into a generic "alignment" one.
//
// TRANSLATION SPLITS BY CONSUMER (research R6). ±5 mm is 0.03° at 10 m —
// negligible for bearing — but swings propeller clearance ~15%, because the
// clear cone is atan((h − r_tip)/d) with a small numerator. It feeds the
// OBSTRUCTION path only.
//
// ---------------------------------------------------------------------------
// BOUNDS COME FROM THE EXISTING 2.5-SIGMA CLAMP — no bespoke clip constants.
// ---------------------------------------------------------------------------
// `ClassPRNG::nextGaussian` already truncates every draw at
// ±kGaussianSigmaClamp (2.5) standard deviations before scaling, so the project
// convention is to SET SIGMA SUCH THAT 2.5σ IS THE INTENDED HARD LIMIT — exactly
// as the entry class documents it (`entryConeSigma = 18.0 // 2.5sigma = 45 deg`).
// The spec's "hard clip at 20°" is therefore expressed as **sigma 8.0**, and the
// 1 cm mount box as **sigma 0.002 m**. Adding a second clip here would be a
// redundant mechanism that could silently disagree with the first.
// Clamp-not-resample also keeps the PRNG draw COUNT fixed, which is what the
// bit-exact replay contract rests on.
//
// NOT RAMPED, and that was decided before this landed:
// `scenario_meta_apply.h` states it outright — "future CAMERA variations
// (tracker mode) are the same category [as craft] — sensor/airframe diversity,
// not difficulty — and should likewise NOT be ramped when they land." So the
// draws sit at full magnitude from gen 0 and `applyVariationScale` leaves them
// alone, same as craft.
//
// SCOPE (operator 2026-08-02): CAMERA variation only — the emitter stays
// perfect. `ambientScale` is drawn and recorded so the plumbing exists and is
// verifiable, but ships at sigma 0 until the lens + bandpass field tests pin
// `SignalAmbientKnee` (031 field test #4). Deferred, not dropped.

#include "autoc/types.h"
#include "autoc/util/scenario_prng.h"

namespace autoc::eval {

struct CameraSigmas {
    // ini-loaded config-struct fields — inih::GetReal yields a 64-bit real, and
    // gp_scalar conversion happens at the CameraSigmas boundary in autoc.cc.
    // Mirrors CraftSigmas exactly.
    //
    // Each sigma is chosen so 2.5σ (the pipeline-wide truncation) equals the
    // intended envelope:
    //   boresight / roll   8.0 deg  → 2.5σ = 20 deg
    //   mount translation  0.002 m  → 2.5σ = 5 mm (the 1 cm box)
    double boresightSigmaDeg = 0.0;      // raw-ok: ini config-struct field (double per inih::GetReal)
    double rollSigmaDeg = 0.0;           // raw-ok: ini config-struct field (double per inih::GetReal)
    double mountTranslationSigmaM = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
    double wingThicknessSigmaM = 0.0;    // raw-ok: ini config-struct field (double per inih::GetReal)
    // DEFERRED — ships at 0 per the scope decision above.
    double ambientSigmaFrac = 0.0;       // raw-ok: ini config-struct field (double per inih::GetReal)
};

// Drawn per-scenario camera imperfections at FULL magnitude. Defaults are the
// NOMINAL camera, so sigma=0 is bit-identical to having no camera-variation code
// at all (FR-023). `ambientScale` defaults to 1.0 because it is a multiplicative
// scale, exactly like `craftThrustScale`.
struct CameraDeltas {
    gp_scalar boresightYawDeg = static_cast<gp_scalar>(0.0);
    gp_scalar boresightPitchDeg = static_cast<gp_scalar>(0.0);
    gp_scalar rollDeg = static_cast<gp_scalar>(0.0);
    gp_vec3 mountTranslation{static_cast<gp_scalar>(0.0), static_cast<gp_scalar>(0.0),
                             static_cast<gp_scalar>(0.0)};
    gp_scalar wingThicknessDelta = static_cast<gp_scalar>(0.0);
    gp_scalar ambientScale = static_cast<gp_scalar>(1.0);

    // 040 — cereal serialize for the WorkerInit RPC carry. RPC-ONLY: these
    // deliberately do not ride ScenarioMetadata, because that struct is
    // persisted in every dmp and changing it orphaned the pinned M1 source.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(boresightYawDeg, boresightPitchDeg, rollDeg, mountTranslation,
           wingThicknessDelta, ambientScale);
    }
};

/**
 * Draw order is FROZEN — adding or reordering draws shifts the value a seed
 * produces and breaks bit-exact replay. To add a camera axis later, append the
 * draw at the bottom and append the field to `CameraDeltas` last. Same contract
 * as craft. NOTE the draws ride WorkerInit, NOT ScenarioMetadata — see the
 * serialize comment below for why that distinction is load-bearing.
 *
 * The ambient draw is centered at 1.0 (multiplicative scale, like
 * `craftThrustScale`); every other axis is a zero-centered delta.
 */
inline CameraDeltas generateCameraFromClassPRNG(
        autoc::util::ClassPRNG& cameraPRNG, const CameraSigmas& sigmas) {
    CameraDeltas d;
    d.boresightYawDeg    = static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.boresightSigmaDeg));
    d.boresightPitchDeg  = static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.boresightSigmaDeg));
    d.rollDeg            = static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.rollSigmaDeg));
    d.mountTranslation   = gp_vec3(
        static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.mountTranslationSigmaM)),
        static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.mountTranslationSigmaM)),
        static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.mountTranslationSigmaM)));
    d.wingThicknessDelta = static_cast<gp_scalar>(cameraPRNG.nextGaussian(sigmas.wingThicknessSigmaM));
    d.ambientScale       = static_cast<gp_scalar>(1.0 + cameraPRNG.nextGaussian(sigmas.ambientSigmaFrac));
    return d;
}

}  // namespace autoc::eval

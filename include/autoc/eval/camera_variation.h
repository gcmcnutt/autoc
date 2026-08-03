#pragma once

// 040 T071-T075 (US6, FR-021/FR-022/FR-023) — per-scenario camera variation.
//
// Parallel to `craft_variation.h`: one header-only module that draws per-scenario
// camera imperfections from the reserved `camera` class PRNG. Drawn at FULL
// magnitude; per-eval ramp scaling is the worker's job, same pipeline as
// wind/entry/craft.
//
// WHAT IT IS FOR. Through 040 t1 the controller trained against a PERFECTLY known
// camera — exact boresight, exact roll, exact mount, identical in every scenario.
// That is the one assumption a real airframe cannot honour, and a controller that
// has only met a perfect camera has no reason to be robust to a crooked one.
//
// ROLL IS THE HIGHEST-IMPACT TERM, not boresight. Under the angular
// representation a boresight error lands as a near-constant additive offset on
// both bearings — clean, and learnable as a subtraction. ROLL rotates the image
// plane, so it biases the port→starboard TILT cue degree-for-degree, and tilt
// drives the roll command. That asymmetry is why roll gets its own axis rather
// than being folded into a generic "alignment" sigma.
//
// TRANSLATION SPLITS BY CONSUMER (research R6). ±5 mm is 0.03° at 10 m —
// negligible for bearing — but swings propeller clearance ~15%, because the clear
// cone is atan((h − r_tip)/d) with a small numerator. It therefore feeds the
// OBSTRUCTION path only; routing it into bearing as well would add arithmetic
// that cannot matter.
//
// SCOPE (operator 2026-08-02): CAMERA variation only — the emitter stays perfect.
// `ambientScale` is drawn and recorded so the plumbing exists and is verifiable,
// but ships at sigma 0 until the lens + bandpass field tests pin
// `SignalAmbientKnee` (031 field test #4). Deferred, not dropped.

#include <algorithm>

#include "autoc/types.h"
#include "autoc/util/scenario_prng.h"

namespace autoc::eval {

// HARD clip, not tail resampling (FR-021). Two reasons, and the second is the
// one that would actually bite:
//   * training should never see an implausibly misaligned camera; and
//   * resampling a tail draw would change the PRNG draw COUNT for that scenario,
//     shifting every subsequent draw and breaking the frozen draw-order contract
//     that bit-exact replay rests on. Clipping keeps the count fixed.
constexpr gp_scalar kCameraAlignmentHardClipDeg = static_cast<gp_scalar>(20.0);
// A 1 cm box about the nominal mount.
constexpr gp_scalar kCameraMountTranslationHardClipM = static_cast<gp_scalar>(0.005);

struct CameraSigmas {
    // ini-loaded config-struct fields — inih::GetReal yields a 64-bit real, and
    // gp_scalar conversion happens where the drawn value is written into
    // ScenarioMetadata. Mirrors CraftSigmas.
    double boresightSigmaDeg = 0.0;      // raw-ok: ini config-struct field (double per inih::GetReal)
    double rollSigmaDeg = 0.0;           // raw-ok: ini config-struct field (double per inih::GetReal)
    double mountTranslationSigmaM = 0.0; // raw-ok: ini config-struct field (double per inih::GetReal)
    double wingThicknessSigmaM = 0.0;    // raw-ok: ini config-struct field (double per inih::GetReal)
    // DEFERRED — ships at 0 per the scope decision above.
    double ambientSigmaFrac = 0.0;       // raw-ok: ini config-struct field (double per inih::GetReal)
};

// Drawn per-scenario camera imperfections at FULL magnitude. Defaults are the
// NOMINAL camera, so a zero-sigma draw is bit-identical to having no variation
// at all (FR-023) — note `ambientScale` defaults to 1.0, not 0.0, because it is
// a scale and not a delta.
struct CameraDeltas {
    gp_scalar boresightYawDeg = static_cast<gp_scalar>(0.0);
    gp_scalar boresightPitchDeg = static_cast<gp_scalar>(0.0);
    gp_scalar rollDeg = static_cast<gp_scalar>(0.0);
    gp_vec3 mountTranslation{static_cast<gp_scalar>(0.0), static_cast<gp_scalar>(0.0),
                             static_cast<gp_scalar>(0.0)};
    gp_scalar wingThicknessDelta = static_cast<gp_scalar>(0.0);
    gp_scalar ambientScale = static_cast<gp_scalar>(1.0);
};

// Draw order is FROZEN and append-only — yaw, pitch, roll, tx, ty, tz, wing,
// ambient. A new axis appends at the end; reordering silently changes every
// prior bake's camera.
inline CameraDeltas generateCameraFromClassPRNG(autoc::util::ClassPRNG& prng,
                                                const CameraSigmas& sigmas) {
    auto clipped = [](double v, gp_scalar limit) {
        const double l = static_cast<double>(limit);
        return static_cast<gp_scalar>(std::clamp(v, -l, l));
    };

    CameraDeltas d;
    d.boresightYawDeg = clipped(prng.nextGaussian(sigmas.boresightSigmaDeg),
                                kCameraAlignmentHardClipDeg);
    d.boresightPitchDeg = clipped(prng.nextGaussian(sigmas.boresightSigmaDeg),
                                  kCameraAlignmentHardClipDeg);
    d.rollDeg = clipped(prng.nextGaussian(sigmas.rollSigmaDeg),
                        kCameraAlignmentHardClipDeg);
    const gp_scalar tx = clipped(prng.nextGaussian(sigmas.mountTranslationSigmaM),
                                 kCameraMountTranslationHardClipM);
    const gp_scalar ty = clipped(prng.nextGaussian(sigmas.mountTranslationSigmaM),
                                 kCameraMountTranslationHardClipM);
    const gp_scalar tz = clipped(prng.nextGaussian(sigmas.mountTranslationSigmaM),
                                 kCameraMountTranslationHardClipM);
    d.mountTranslation = gp_vec3(tx, ty, tz);
    d.wingThicknessDelta =
        static_cast<gp_scalar>(prng.nextGaussian(sigmas.wingThicknessSigmaM));
    // A SCALE about 1.0 — and it must land on exactly 1.0 at zero sigma, which
    // is why the multiply is done in double and only then narrowed.
    d.ambientScale = static_cast<gp_scalar>(
        1.0 + prng.nextGaussian(sigmas.ambientSigmaFrac));
    return d;
}

}  // namespace autoc::eval

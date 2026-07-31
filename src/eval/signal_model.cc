// 040 T057-T059 — per-beacon link budget. See signal_model.h for the chain, and
// especially for the calibration-honesty note on the back-solved noise floor.

#include "autoc/eval/signal_model.h"

#include <algorithm>
#include <cmath>

namespace autoc::eval {

namespace {

constexpr gp_scalar kPi = static_cast<gp_scalar>(3.14159265358979323846);
constexpr gp_scalar kDegToRad = kPi / static_cast<gp_scalar>(180);

}  // namespace

SignalConfig hb1SignalConfig() {
    SignalConfig c;

    // 031 bench midpoint 1.35 µA·m² measured on a FIVE-CO-AIMED rig ÷ 5 faces.
    c.flux_constant = static_cast<gp_scalar>(0.27);
    c.optics_gain = static_cast<gp_scalar>(1.0);

    // Lumileds DS190 beam width.
    c.emission_flat_deg = static_cast<gp_scalar>(45.0);
    c.emission_half_power_deg = static_cast<gp_scalar>(75.0);

    // FR-033a: the envelope is asserted, and the floor is back-solved against it
    // so 0 dB lands exactly at the edge on beam peak. 0.27 / 100² = 2.7e-5 µA.
    // Split 80/20 between the varied ambient term (US6 draws this) and the fixed
    // sensor term; only the SUM is load-bearing, and a test pins it.
    c.detection_range_m = static_cast<gp_scalar>(100.0);
    c.ambient_floor = static_cast<gp_scalar>(2.16e-5);
    c.noise_floor = static_cast<gp_scalar>(0.54e-5);

    // 031 §4: sharing a detector element costs about one SNR tier.
    c.cdma_penalty_db = static_cast<gp_scalar>(3.0);

    // 0 dB is the decode floor; 20 dB is where the AGC-normalised metric tops
    // out. Linear-in-dB between, which is the assumed part.
    c.q_floor_db = static_cast<gp_scalar>(0.0);
    c.q_saturation_db = static_cast<gp_scalar>(20.0);

    // A 0.772 m pair at 0.375°/px resolves to 5 px at ≈23.6 m, which is the
    // ≈25 m separation-range reach the perception contract states. Below 5 px
    // the ±0.5 px quantisation is already >10% of the gap, so range inferred
    // from it is not usable — see the grid-convexity note in T026.
    c.separation_min_px = static_cast<gp_scalar>(5.0);
    // Within ~1.5 px the blobs merge into one detector element.
    c.shared_element_px = static_cast<gp_scalar>(1.5);

    return c;
}

// ---------------------------------------------------------------------------
// Emission.
// ---------------------------------------------------------------------------

gp_scalar emissionProfile(gp_scalar angle_deg, const SignalConfig& cfg) {
    const gp_scalar a = std::abs(angle_deg);
    const gp_scalar flat = cfg.emission_flat_deg;
    const gp_scalar shoulder = cfg.emission_half_power_deg - flat;

    if (a <= flat) return static_cast<gp_scalar>(1);
    if (shoulder <= static_cast<gp_scalar>(0)) return static_cast<gp_scalar>(0);

    // Raised cosine in t = (a − flat)/shoulder: 1 at t=0, 0.5 at t=1, 0 at t=2.
    const gp_scalar t = (a - flat) / shoulder;
    if (t >= static_cast<gp_scalar>(2)) return static_cast<gp_scalar>(0);
    return static_cast<gp_scalar>(0.5) *
           (static_cast<gp_scalar>(1) + std::cos(kPi * t / static_cast<gp_scalar>(2)));
}

gp_scalar emissionGain(const gp_vec3& dir_target_body,
                       const gp_vec3& outboard_axis_body,
                       const SignalConfig& cfg) {
    const gp_scalar dir_norm = dir_target_body.norm();
    const gp_scalar out_norm = outboard_axis_body.norm();
    constexpr gp_scalar kEps = static_cast<gp_scalar>(1e-9);
    if (dir_norm < kEps || out_norm < kEps) return static_cast<gp_scalar>(0);

    const gp_vec3 d = dir_target_body / dir_norm;
    const gp_vec3 outboard = outboard_axis_body / out_norm;

    // Cube minus base: the outboard face plus the four body axes orthogonal to
    // it. For a wingtip mount the outboard normal is ±y, so the other four are
    // fore/aft (±x) and up/down (∓z). Deriving them from the body frame rather
    // than hardcoding keeps a rolled or canted mount honest.
    const gp_vec3 axes[5] = {
        outboard,
        gp_vec3(static_cast<gp_scalar>(1), static_cast<gp_scalar>(0), static_cast<gp_scalar>(0)),
        gp_vec3(static_cast<gp_scalar>(-1), static_cast<gp_scalar>(0), static_cast<gp_scalar>(0)),
        gp_vec3(static_cast<gp_scalar>(0), static_cast<gp_scalar>(0), static_cast<gp_scalar>(1)),
        gp_vec3(static_cast<gp_scalar>(0), static_cast<gp_scalar>(0), static_cast<gp_scalar>(-1)),
    };

    gp_scalar total = static_cast<gp_scalar>(0);
    for (int i = 0; i < 5; ++i) {
        // Skip a body axis that coincides with the outboard face — a mount whose
        // outboard normal IS a body axis would otherwise count that face twice.
        // Never fires for the baseline ±y wingtip mount; it is here so a canted
        // mount degrades sensibly instead of silently gaining a sixth emitter.
        if (i > 0 && axes[i].dot(outboard) > static_cast<gp_scalar>(0.999)) continue;
        const gp_scalar c = std::clamp(d.dot(axes[i]), static_cast<gp_scalar>(-1),
                                       static_cast<gp_scalar>(1));
        total += emissionProfile(std::acos(c) / kDegToRad, cfg);
    }
    return total;
}

// ---------------------------------------------------------------------------
// Budget.
// ---------------------------------------------------------------------------

gp_scalar qFromSnrDb(gp_scalar snr_db, const SignalConfig& cfg) {
    const gp_scalar span = cfg.q_saturation_db - cfg.q_floor_db;
    if (span <= static_cast<gp_scalar>(0)) return static_cast<gp_scalar>(0);
    const gp_scalar t = std::clamp((snr_db - cfg.q_floor_db) / span,
                                   static_cast<gp_scalar>(0), static_cast<gp_scalar>(1));
    return t * static_cast<gp_scalar>(9);
}

SignalResult computeSignal(gp_scalar range_m,
                           gp_scalar emission_gain,
                           gp_scalar obstruction_attenuation,
                           bool shares_detector_element,
                           const SignalConfig& cfg) {
    SignalResult out;

    constexpr gp_scalar kMinRange = static_cast<gp_scalar>(1e-3);
    const gp_scalar r = std::max(range_m, kMinRange);

    // The chain. Obstruction is a plain multiplier — the propeller attenuates
    // rather than gates (FR-009), which is the term T014 computed and left
    // deliberately unconsumed until this module existed.
    out.received_ua = cfg.flux_constant * emission_gain * obstruction_attenuation *
                      cfg.optics_gain / (r * r);

    const gp_scalar floor_ua = cfg.ambient_floor + cfg.noise_floor;
    if (out.received_ua <= static_cast<gp_scalar>(0) ||
        floor_ua <= static_cast<gp_scalar>(0)) {
        out.snr_db = static_cast<gp_scalar>(-999);
        out.q = static_cast<gp_scalar>(0);
        return out;
    }

    out.snr_db = static_cast<gp_scalar>(10) * std::log10(out.received_ua / floor_ua);
    if (shares_detector_element) {
        // Degrades, never gates: the 031 single-detector rig is exactly this
        // configuration and decodes both codes reliably (FR-016).
        out.snr_db -= cfg.cdma_penalty_db;
    }
    out.q = qFromSnrDb(out.snr_db, cfg);
    return out;
}

}  // namespace autoc::eval

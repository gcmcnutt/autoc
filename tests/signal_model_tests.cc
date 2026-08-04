// 040 T048-T050 (US4) — the per-beacon link budget.
//
// WHAT THIS SUITE IS FOR. Through 039 range did not enter perception at all: a
// beacon at 5 m and one at 500 m produced identical observations. The only
// thing modulating CEP was where the beacon sat in frame. These tests pin the
// replacement — drive × emission(aspect) × 1/r² × obstruction ÷ ambient — and,
// more importantly, pin the two places that model is easy to get plausibly
// wrong:
//
//   1. The EMISSION PATTERN is a flat-top with shoulders, NOT a cosine power
//      law (FR-019). A cos^m law fitted to the same half-power angle under-reads
//      the flat region by ~16% and over-reads the skirt. `EmissionIsFlatTopNotCosinePowerLaw`
//      asserts the difference explicitly rather than just bounding the shape,
//      so the test cannot pass on a cosine law that happens to be close.
//
//   2. The FIVE-EMITTER ENCLOSURE is load-bearing for the DOMINANT geometry.
//      The enclosure is a 1 cm cube with one face against the wingtip, so it
//      emits on five faces. A single outboard axis — which is what BeaconConfig
//      described through 039 — puts the tail-chase direction at 90° off the only
//      beam, i.e. deep in the skirt, when in reality an aft-facing emitter is
//      pointed straight down the chase's throat. Modelling one axis would make
//      the model wrong in exactly the case M2 spends all its time in.
//
// DETERMINISM (FR-020): every function here is pure. No PRNG, no clock.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/signal_model.h"
#include "autoc/types.h"

using autoc::eval::emissionGain;
using autoc::eval::emissionProfile;
using autoc::eval::computeSignal;
using autoc::eval::hb1SignalConfig;
using autoc::eval::SignalConfig;
using autoc::eval::SignalResult;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

// Reference geometry: beam peak, unobstructed, sole occupant of its detector
// element. Everything else in this file is a departure from this.
SignalResult atRangeOnAxis(double range_m, const SignalConfig& cfg) {
    return computeSignal(static_cast<gp_scalar>(range_m),
                         static_cast<gp_scalar>(1.0),   // emission gain: on axis
                         static_cast<gp_scalar>(1.0),   // obstruction: clear
                         false,                          // detector element: sole
                         cfg);
}

}  // namespace

// ---------------------------------------------------------------------------
// T048 (FR-015, SC-004) — range.
// ---------------------------------------------------------------------------

TEST(SignalModel, SignalFallsMonotonicallyWithRange) {
    const SignalConfig cfg = hb1SignalConfig();

    double prev_db = 1e9;
    for (double r = 2.0; r <= 120.0; r += 0.5) {
        const SignalResult s = atRangeOnAxis(r, cfg);
        ASSERT_LT(static_cast<double>(s.snr_db), prev_db)
            << "SNR must fall strictly with range; broke at " << r << " m";
        prev_db = static_cast<double>(s.snr_db);
    }
}

TEST(SignalModel, RangeFalloffIsInverseSquare) {
    const SignalConfig cfg = hb1SignalConfig();

    // Doubling range must cost exactly 6.02 dB. This is what distinguishes a
    // real 1/r² term from a hand-tuned ramp that merely decreases.
    for (double r : {5.0, 10.0, 25.0, 50.0}) {
        const double near_db = static_cast<double>(atRangeOnAxis(r, cfg).snr_db);
        const double far_db = static_cast<double>(atRangeOnAxis(2.0 * r, cfg).snr_db);
        EXPECT_NEAR(near_db - far_db, 20.0 * std::log10(2.0), 1e-3)
            << "at " << r << " m → " << (2 * r) << " m";
    }
}

TEST(SignalModel, QualityGradientSpansTheAssertedDetectionEnvelope) {
    // SC-004 is about the GRADIENT, not a cutoff — FR-033a asserts the envelope
    // rather than deriving it, so the substantive property is that quality
    // actually varies across the envelope instead of being pinned at one end.
    //
    // A budget that saturated everywhere, or floored everywhere, would satisfy
    // "monotonic" while carrying no information. This is the test that catches
    // that, and it is why the noise floor is back-solved against the asserted
    // range (see signal_model.h) rather than taken from the bench's 10 nA decode
    // floor, which would put every range past ~12 m at the floor.
    const SignalConfig cfg = hb1SignalConfig();

    const double q_near = static_cast<double>(atRangeOnAxis(5.0, cfg).q);
    const double q_mid = static_cast<double>(atRangeOnAxis(25.0, cfg).q);
    const double q_far = static_cast<double>(atRangeOnAxis(100.0, cfg).q);

    EXPECT_GE(q_near, 8.5) << "close-in must reach the confident end of the 0-9 scale";
    EXPECT_LE(q_far, 0.5) << "envelope edge must reach the tentative end";
    EXPECT_GT(q_mid, 2.0) << "mid-envelope must not be pinned at the floor";
    EXPECT_LT(q_mid, 8.0) << "mid-envelope must not be pinned at saturation";
}

TEST(SignalModel, NoiseFloorIsCoherentWithTheAssertedDetectionRange) {
    // The shipped floor is BACK-SOLVED so 0 dB lands exactly at the asserted
    // detection range on beam peak. That coupling is invisible in the ini — the
    // two values look independent — so changing one without the other would
    // silently move the envelope edge. This is the guard.
    const SignalConfig cfg = hb1SignalConfig();

    const double floor_ua = static_cast<double>(cfg.ambient_floor + cfg.noise_floor);
    const double implied =
        static_cast<double>(cfg.flux_constant) /
        (static_cast<double>(cfg.detection_range_m) * static_cast<double>(cfg.detection_range_m));
    EXPECT_NEAR(floor_ua, implied, implied * 1e-6);

    // 0 dB lands at the envelope edge BEFORE ambient compression. The knee term
    // added 2026-08-02 costs a further 10·log10(transfer) — −0.09 dB at the
    // shipped (shade) ambient — so the assertion states the coupling exactly
    // rather than being loosened to hide it.
    const double transfer_db =
        10.0 * std::log10(static_cast<double>(atRangeOnAxis(100.0, cfg).transfer));
    EXPECT_NEAR(static_cast<double>(atRangeOnAxis(100.0, cfg).snr_db), transfer_db, 1e-3)
        << "0 dB must land at the asserted envelope edge on beam peak, "
           "less only the ambient-compression transfer";
    EXPECT_GT(transfer_db, -0.15) << "shade ambient must be essentially uncompressed";
}

// ---------------------------------------------------------------------------
// AMBIENT COMPRESSION (031 field test #4, 2026-08-02).
//
// Ambient does TWO things to a photodiode. Through 040 t1 this model captured
// only the second: (1) it forward-biases the junction so beacon current is
// SHUNTED AT THE SENSOR — a transfer loss — and (2) it adds shot noise. With
// only the additive floor, more emitter current always buys SNR at any ambient,
// i.e. the model would predict you can OUT-POWER THE SUN. The bench says
// otherwise.
// ---------------------------------------------------------------------------

TEST(SignalModel, AmbientCompressionIsNegligibleAtTheShippedAmbient) {
    // The shipped ambient is shade/overcast, and the knee sits 100× above it, so
    // the default case is essentially uncompressed. This is what preserves the
    // t1 bake's semantics — compression only bites once US6 draws ambient up.
    const SignalConfig cfg = hb1SignalConfig();
    EXPECT_GT(static_cast<double>(atRangeOnAxis(20.0, cfg).transfer), 0.98);
}

TEST(SignalModel, AmbientCompressionCollapsesTransferAsAmbientRises) {
    SignalConfig cfg = hb1SignalConfig();
    const double shade = static_cast<double>(cfg.ambient_floor);

    double prev = 1.1;
    for (double mult : {1.0, 10.0, 100.0, 1000.0}) {
        cfg.ambient_floor = static_cast<gp_scalar>(shade * mult);
        const double t = static_cast<double>(atRangeOnAxis(20.0, cfg).transfer);
        EXPECT_LT(t, prev) << "transfer must fall monotonically with ambient (×" << mult << ")";
        EXPECT_GT(t, 0.0) << "compression attenuates, it never gates";
        prev = t;
    }
    // Direct sun ≈ 100× shade puts the knee at parity ⇒ half the signal shunted.
    cfg.ambient_floor = static_cast<gp_scalar>(shade * 100.0);
    EXPECT_NEAR(static_cast<double>(atRangeOnAxis(20.0, cfg).transfer), 0.5, 0.01);
}

TEST(SignalModel, EmitterCurrentCannotBuyBackDirectSun) {
    // THE FIELD-TEST PROPERTY, as an executable assertion. 031 field test #4:
    // at ~6× emitter current with a bare PD, a SHADED sensor locks at ~20 ft and
    // a SUN-EXPOSED one fails at any distance — shadow alone flips it at fixed
    // emitter and distance.
    //
    // The loss sits upstream of every downstream multiplier and is far larger
    // than a realistic current increase recovers. If a future edit ever lets
    // 6× current reach shaded parity in sun, this model has gone back to
    // claiming you can out-power the sun, and this test is what stops it.
    const SignalConfig shade = hb1SignalConfig();

    SignalConfig sun6x = hb1SignalConfig();
    sun6x.ambient_floor = static_cast<gp_scalar>(shade.ambient_floor * 100.0);  // direct sun
    sun6x.flux_constant = static_cast<gp_scalar>(shade.flux_constant * 6.0);    // ×6 emitters

    const double shade_db = static_cast<double>(atRangeOnAxis(6.0, shade).snr_db);
    const double sun_db = static_cast<double>(atRangeOnAxis(6.0, sun6x).snr_db);

    EXPECT_LT(sun_db, shade_db - 10.0)
        << "×6 emitter current in direct sun must remain FAR below the shaded "
           "link at 1× — the bandpass filter is the gate, not emitter drive";
}

// ---------------------------------------------------------------------------
// T049 (FR-019) — emission pattern.
// ---------------------------------------------------------------------------

TEST(SignalModel, EmissionIsFlatToFortyFiveAndHalfPowerAtSeventyFive) {
    const SignalConfig cfg = hb1SignalConfig();

    // Flat top: near-constant out to ±45°.
    for (double a = 0.0; a <= 45.0; a += 5.0) {
        EXPECT_GT(static_cast<double>(emissionProfile(static_cast<gp_scalar>(a), cfg)), 0.99)
            << "flat region broke at " << a << "°";
    }
    // Half power at ±75°.
    EXPECT_NEAR(static_cast<double>(emissionProfile(static_cast<gp_scalar>(75.0), cfg)), 0.5, 1e-6);
    EXPECT_NEAR(static_cast<double>(emissionProfile(static_cast<gp_scalar>(-75.0), cfg)), 0.5, 1e-6);

    // Symmetric, and monotonically falling once past the flat top.
    double prev = 1.0;
    for (double a = 45.0; a <= 110.0; a += 1.0) {
        const double g = static_cast<double>(emissionProfile(static_cast<gp_scalar>(a), cfg));
        EXPECT_LE(g, prev + 1e-9) << "skirt must not rise; broke at " << a << "°";
        EXPECT_NEAR(g, static_cast<double>(emissionProfile(static_cast<gp_scalar>(-a), cfg)), 1e-9);
        prev = g;
    }
    EXPECT_NEAR(static_cast<double>(emissionProfile(static_cast<gp_scalar>(105.0), cfg)), 0.0, 1e-6);
    EXPECT_EQ(static_cast<double>(emissionProfile(static_cast<gp_scalar>(140.0), cfg)), 0.0);
}

TEST(SignalModel, EmissionIsFlatTopNotCosinePowerLaw) {
    // R4 rejected cos^m explicitly: it under-reads the flat region ~16% and
    // over-reads the skirt. Asserting only "flat to 45°" would pass on a cosine
    // law with a large enough exponent, so this test builds the competing model
    // and asserts we are measurably NOT it — otherwise the requirement is
    // decorative.
    const SignalConfig cfg = hb1SignalConfig();

    // The cos^m law that agrees with us at the half-power angle.
    const double m = std::log(0.5) / std::log(std::cos(75.0 * kDeg));

    // Flat region: we sit at unity, the cosine law has already sagged.
    const double cos_at_45 = std::pow(std::cos(45.0 * kDeg), m);
    const double ours_at_45 = static_cast<double>(emissionProfile(static_cast<gp_scalar>(45.0), cfg));
    EXPECT_LT(cos_at_45, 0.90) << "sanity: the fitted cosine law should sag by 45°";
    EXPECT_GT(ours_at_45 - cos_at_45, 0.10)
        << "flat top must sit well above the fitted cosine law in the flat region";

    // Skirt: the cosine law has a long tail, ours reaches zero.
    const double cos_at_100 = std::pow(std::cos(100.0 * kDeg) < 0.0 ? 0.0
                                                                   : std::cos(100.0 * kDeg), m);
    const double ours_at_100 = static_cast<double>(emissionProfile(static_cast<gp_scalar>(100.0), cfg));
    EXPECT_LT(ours_at_100, 0.10) << "our skirt must be nearly spent by 100°";
    (void)cos_at_100;
}

TEST(SignalModel, FiveFaceEnclosureIlluminatesTheTailChaseDirection) {
    // THE POINT OF THE FIVE-AXIS MODEL. The chase sits behind the target, so the
    // beacon→chase direction is roughly AFT (−x in target body frame) while the
    // wingtip beacon's outboard face points ±y. Under a single-axis model that
    // is 90° off beam — deep in the skirt — which would systematically
    // under-report signal in the one geometry M2 spends all its time in.
    //
    // The enclosure is a cube minus its base, so an AFT-facing emitter is
    // pointed straight down the chase's throat.
    const SignalConfig cfg = hb1SignalConfig();

    const gp_vec3 outboard(static_cast<gp_scalar>(0), static_cast<gp_scalar>(-1),
                           static_cast<gp_scalar>(0));  // left wingtip, outboard = −y
    const gp_vec3 aft(static_cast<gp_scalar>(-1), static_cast<gp_scalar>(0),
                      static_cast<gp_scalar>(0));

    const double tail = static_cast<double>(emissionGain(aft, outboard, cfg));
    const double single_axis = static_cast<double>(
        emissionProfile(static_cast<gp_scalar>(90.0), cfg));

    EXPECT_GT(tail, 1.0) << "aft emitter must be on-axis for a tail chase";
    EXPECT_GT(tail, 4.0 * single_axis)
        << "the five-face enclosure must beat the single-outboard-axis model "
           "decisively in the tail-chase direction";
}

TEST(SignalModel, EnclosureGainHasNoBlindSpotOutsideTheMountedFace) {
    // Cube minus base: the only direction that should go dark is INBOARD, where
    // the wing itself is. Anything else must retain usable gain, or the model
    // would invent dropouts the hardware does not have.
    const SignalConfig cfg = hb1SignalConfig();
    const gp_vec3 outboard(static_cast<gp_scalar>(0), static_cast<gp_scalar>(-1),
                           static_cast<gp_scalar>(0));

    for (double az = 0.0; az < 360.0; az += 10.0) {
        for (double el = -80.0; el <= 80.0; el += 10.0) {
            const double ca = std::cos(az * kDeg), sa = std::sin(az * kDeg);
            const double ce = std::cos(el * kDeg), se = std::sin(el * kDeg);
            const gp_vec3 dir(static_cast<gp_scalar>(ce * ca),
                              static_cast<gp_scalar>(ce * sa),
                              static_cast<gp_scalar>(se));
            // Skip the inboard hemisphere (+y here), which the wing occupies.
            if (dir.y() > static_cast<gp_scalar>(0.5)) continue;
            EXPECT_GT(static_cast<double>(emissionGain(dir, outboard, cfg)), 0.20)
                << "blind spot at az=" << az << " el=" << el;
        }
    }
}

// ---------------------------------------------------------------------------
// T050 (FR-016) — shared detector element.
// ---------------------------------------------------------------------------

TEST(SignalModel, SharedDetectorElementDegradesButNeverGates) {
    // FR-016 is emphatic that this configuration is FIELD-PROVEN, not a failure
    // mode: the 031 single-detector rig is exactly this case and decodes both
    // codes reliably. What merging removes is spatial separation — hence
    // separation-derived range — not detection.
    const SignalConfig cfg = hb1SignalConfig();

    const SignalResult sole = computeSignal(
        static_cast<gp_scalar>(30.0), static_cast<gp_scalar>(1.0),
        static_cast<gp_scalar>(1.0), false, cfg);
    const SignalResult shared = computeSignal(
        static_cast<gp_scalar>(30.0), static_cast<gp_scalar>(1.0),
        static_cast<gp_scalar>(1.0), true, cfg);

    EXPECT_LT(static_cast<double>(shared.snr_db), static_cast<double>(sole.snr_db))
        << "sharing must cost something";
    EXPECT_NEAR(static_cast<double>(sole.snr_db) - static_cast<double>(shared.snr_db),
                static_cast<double>(cfg.cdma_penalty_db), 1e-4)
        << "the cost must be exactly the measured penalty, not a guess";

    // The load-bearing half: still detected.
    EXPECT_GT(static_cast<double>(shared.received_ua), 0.0);
    EXPECT_GT(static_cast<double>(shared.q), 0.0)
        << "a shared element must not zero the signal — 031 decodes this case";
}

TEST(SignalModel, ObstructionAttenuatesTheBudgetMultiplicatively) {
    // The propeller attenuates rather than gates (FR-009). Its attenuation is
    // computed in the obstruction pass and enters here as a plain multiplier —
    // the term T014 left explicitly unconsumed until this stage existed.
    const SignalConfig cfg = hb1SignalConfig();

    const SignalResult clear = computeSignal(
        static_cast<gp_scalar>(20.0), static_cast<gp_scalar>(1.0),
        static_cast<gp_scalar>(1.0), false, cfg);
    const SignalResult through_prop = computeSignal(
        static_cast<gp_scalar>(20.0), static_cast<gp_scalar>(1.0),
        static_cast<gp_scalar>(0.18), false, cfg);

    EXPECT_NEAR(static_cast<double>(through_prop.received_ua),
                static_cast<double>(clear.received_ua) * 0.18,
                static_cast<double>(clear.received_ua) * 1e-5);
    EXPECT_GT(static_cast<double>(through_prop.received_ua), 0.0)
        << "attenuation must never gate";
}

// ---------------------------------------------------------------------------
// FR-020 — determinism.
// ---------------------------------------------------------------------------

TEST(SignalModel, IdenticalInputsProduceBitIdenticalOutputs) {
    const SignalConfig cfg = hb1SignalConfig();
    for (int i = 0; i < 64; ++i) {
        const gp_scalar r = static_cast<gp_scalar>(3.0 + 1.5 * i);
        const SignalResult a = computeSignal(r, static_cast<gp_scalar>(0.8),
                                             static_cast<gp_scalar>(0.9), false, cfg);
        const SignalResult b = computeSignal(r, static_cast<gp_scalar>(0.8),
                                             static_cast<gp_scalar>(0.9), false, cfg);
        EXPECT_EQ(a.received_ua, b.received_ua);
        EXPECT_EQ(a.snr_db, b.snr_db);
        EXPECT_EQ(a.q, b.q);
    }
}

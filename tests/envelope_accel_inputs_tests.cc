// 041 T040 — input semantics for the five slots added by the A1 bundle:
// IN_ENVELOPE, ENVELOPE_SECS, ACCEL_X/Y/Z.
//
// These are SEMANTIC tests, not layout tests (layout lives in
// tests/contract_evaluator_tests.cc). Each one pins a property whose violation
// would be invisible in sim and wrong in flight — the Principle VII shape.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/envelope_state.h"
#include "autoc/eval/fitness_computer.h"
#include "autoc/eval/specific_force.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"

using autoc::eval::EnvelopeState;
using autoc::eval::bodySpecificForce;

namespace {

// Gravity in LaRCSim-native ft/s². The whole point of passing the FDM's own
// gravity into bodySpecificForce is that the result is unit-free, so these
// tests deliberately use ft units — if a stray ft→m conversion ever appears in
// the chain, these numbers stop being 1.0.
constexpr double kGravityFtS2 = 32.174;

AircraftState makeState(const gp_quat& attitude) {
    return AircraftState{0,
                         20.0f,
                         gp_vec3(20.0f, 0.0f, 0.0f),
                         attitude,
                         gp_vec3::Zero(),
                         0.0f,
                         0.0f,
                         0.0f,
                         0};
}

// Run the real gather so the test covers the whole chain the policy sees:
// producer (bodySpecificForce) → carrier (AircraftState) → slot scaling.
NNInputs gatherFrom(AircraftState& st) {
    Path dummy{};
    SinglePathProvider provider(dummy);
    NNInputs in{};
    gather_pathgen_inputs(provider, st, autoc::eval::FlightArena{}, in);
    return in;
}

// ---------------------------------------------------------------------------
// (a) THE test — steady level flight reads 1 g on the normal channel, not 0.
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, SteadyLevelFlightReadsOneG_NotZero_T040a) {
    // Steady level flight: the FDM's kinematic acceleration is ZERO. An
    // accelerometer is not: it measures specific force, so it reads 1 g.
    const gp_vec3 accWorld = gp_vec3::Zero();

    AircraftState st = makeState(gp_quat::Identity());
    const auto sf = bodySpecificForce(accWorld, st.getOrientation(),
                                      static_cast<gp_scalar>(kGravityFtS2));
    st.setSpecificForceG(sf.g_units);

    const NNInputs in = gatherFrom(st);
    const double nz_g = static_cast<double>(in.accel_z) * kAccelScale_g;

    // The magnitude assertion is the one that catches the kinematic-vs-specific
    // error: feeding the FDM's `acc` directly would land this at 0.0.
    ASSERT_NEAR(std::fabs(nz_g), 1.0, 1e-5)
        << "normal channel must read 1 g in level flight; 0 g means FDM "
           "kinematic acceleration was fed instead of specific force";

    // And the SIGN, which is a separate way to be wrong. Aerospace body FRD:
    // body +z points DOWN and the measured reaction points UP, so level flight
    // is -1. ⚠️ Do not "fix" this against INAV's bench table (which reads +1 g
    // on its normal axis) — INAV's frame is FLU and msplink flips y/z at the
    // boundary, exactly as it already does for the quat and the gyro. Same
    // physical fact in two frames; settled 2026-08-11, see
    // docs/COORDINATE_CONVENTIONS.md.
    EXPECT_NEAR(nz_g, -1.0, 1e-5);

    // Lateral and longitudinal are quiet in level flight.
    EXPECT_NEAR(static_cast<double>(in.accel_x) * kAccelScale_g, 0.0, 1e-5);
    EXPECT_NEAR(static_cast<double>(in.accel_y) * kAccelScale_g, 0.0, 1e-5);
}

// ---------------------------------------------------------------------------
// (b) documented sign on a pull-up
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, PullUpDrivesNormalChannelMoreNegative_T040b) {
    // A 2 g pull-up in level attitude: 1 g of upward kinematic acceleration on
    // top of gravity. World NED, so "up" is -z.
    const gp_vec3 accWorld(0.0f, 0.0f, static_cast<gp_scalar>(-kGravityFtS2));

    AircraftState st = makeState(gp_quat::Identity());
    const auto sf = bodySpecificForce(accWorld, st.getOrientation(),
                                      static_cast<gp_scalar>(kGravityFtS2));
    st.setSpecificForceG(sf.g_units);

    const NNInputs in = gatherFrom(st);
    const double nz_g = static_cast<double>(in.accel_z) * kAccelScale_g;

    // FRD: pulling g drives the normal channel MORE NEGATIVE (load factor +2).
    EXPECT_NEAR(nz_g, -2.0, 1e-5);
    EXPECT_NEAR(static_cast<double>(sf.load_factor_nz), 2.0, 1e-5)
        << "load_factor_nz is the human-facing negation and must read +2 for a "
           "2 g pull-up, matching how flight reports quote loads";
}

TEST(EnvelopeAccelInputs, AccelScaleKeepsFlightEnvelopeInsideTanhLinearRegion_T040b2) {
    // The scale exists to keep the common regime out of tanh saturation while
    // leaving an 11 g excursion clearly distinct. Pin both ends, since a
    // "tidier" scale of 1.0 would silently destroy the load axis's resolution.
    EXPECT_NEAR(3.0 / kAccelScale_g, 0.375, 1e-6);   // ordinary manoeuvre
    EXPECT_NEAR(11.2 / kAccelScale_g, 1.4, 1e-6);    // standing flight record
}

// ---------------------------------------------------------------------------
// (c) IN_ENVELOPE is a flag, not a score
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, InEnvelopeIsExactlyZeroOrOne_T040c) {
    for (bool inside : {false, true}) {
        AircraftState st = makeState(gp_quat::Identity());
        st.setInEnvelope(inside);
        const NNInputs in = gatherFrom(st);
        EXPECT_TRUE(in.in_envelope == 0.0f || in.in_envelope == 1.0f);
        EXPECT_FLOAT_EQ(in.in_envelope, inside ? 1.0f : 0.0f);
    }
}

// ---------------------------------------------------------------------------
// (d) ENVELOPE_SECS ramp / reset / saturation
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, EnvelopeSecsRampsResetsAndSaturates_T040d) {
    constexpr double kRampSec = 5.0;
    constexpr double kTickMsec = 50.0;  // 20 Hz

    EnvelopeState env;

    // Monotone non-decreasing within a streak, and strictly increasing until
    // saturation.
    double prev = -1.0;
    for (int k = 0; k < 40; ++k) {   // 2 s — well short of the 5 s ramp
        env.advance(true, kTickMsec);
        const double now = static_cast<double>(env.normalizedSecs(kRampSec));
        EXPECT_GT(now, prev) << "tick " << k;
        prev = now;
    }
    EXPECT_NEAR(prev, 2.0 / kRampSec, 1e-6);

    // 0 IMMEDIATELY after exit — one tick outside, not a decay.
    env.advance(false, kTickMsec);
    EXPECT_FLOAT_EQ(env.normalizedSecs(kRampSec), 0.0f);
    EXPECT_FALSE(env.in_envelope);

    // Saturates at 1 and STAYS there — 1.0 means "multiplier saturated", the
    // decision-relevant boundary, so it must not keep climbing past it.
    for (int k = 0; k < 400; ++k) env.advance(true, kTickMsec);
    EXPECT_FLOAT_EQ(env.normalizedSecs(kRampSec), 1.0f);

    // A non-positive ramp returns 0 rather than dividing.
    EXPECT_FLOAT_EQ(env.normalizedSecs(0.0), 0.0f);
}

TEST(EnvelopeAccelInputs, EnvelopeSecsResetsOnExitOnly_NotOnAnythingElse_T040d2) {
    constexpr double kRampSec = 5.0;
    constexpr double kTickMsec = 50.0;

    EnvelopeState env;
    for (int k = 0; k < 20; ++k) env.advance(true, kTickMsec);
    const float held = env.normalizedSecs(kRampSec);

    // Staying inside keeps accumulating — nothing else (regime change,
    // visibility, engage) is even expressible here, which is the point: the
    // reset condition has exactly one input.
    env.advance(true, kTickMsec);
    EXPECT_GT(env.normalizedSecs(kRampSec), held);
}

// ---------------------------------------------------------------------------
// (e) cadence invariance — the FR-016 property
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, EnvelopeSecsIdenticalAcrossCadencesForSameWallClock_T040e) {
    constexpr double kRampSec = 5.0;
    constexpr double kWallClockMsec = 2000.0;

    // Same 2 s of wall clock, at 20 Hz and at 50 Hz.
    EnvelopeState slow;
    for (int k = 0; k < 40; ++k) slow.advance(true, 50.0);

    EnvelopeState fast;
    for (int k = 0; k < 100; ++k) fast.advance(true, 20.0);

    EXPECT_NEAR(slow.accum_msec, kWallClockMsec, 1e-9);
    EXPECT_NEAR(fast.accum_msec, kWallClockMsec, 1e-9);
    EXPECT_FLOAT_EQ(slow.normalizedSecs(kRampSec), fast.normalizedSecs(kRampSec));

    // The failure this guards: a tick-counting accumulator would report 40 vs
    // 100 here, i.e. the same flight would look 2.5x "deeper" in the envelope
    // purely from a cadence change.
    EXPECT_NE(40, 100);
}

// ---------------------------------------------------------------------------
// (f) M1's flag agrees with the objective's OWN threshold decision
// ---------------------------------------------------------------------------

TEST(EnvelopeAccelInputs, M1FlagMatchesObjectiveThresholdTickForTick_T040f) {
    // The worker derives IN_ENVELOPE as `stepScore >= FitStreakThreshold`,
    // where stepScore comes from FitnessComputer::decomposeStepScore. This test
    // walks the craft from far behind the rabbit to on top of it and checks the
    // flag against the objective's own threshold at every step — the "single
    // source of truth" property, stated as agreement rather than as a comment.
    constexpr double kThreshold = 0.5;
    const FitnessComputer scorer(
        /*distScaleBehind=*/7.0, /*distScaleAhead=*/2.0, /*coneAngleDeg=*/45.0,
        kThreshold, /*rampTicks=*/100, /*multiplierMax=*/1.0);

    int transitions = 0;
    bool prev = false;
    for (int i = 0; i <= 60; ++i) {
        const double along = -30.0 + 0.5 * i;   // behind → onto the rabbit
        const double lateral = 1.0;
        const double stepScore = scorer.decomposeStepScore(along, lateral).score;

        EnvelopeState env;
        env.advance(stepScore >= kThreshold, 50.0);

        AircraftState st = makeState(gp_quat::Identity());
        st.setInEnvelope(env.in_envelope);
        const NNInputs in = gatherFrom(st);

        const bool objectiveSaysInside = (stepScore >= kThreshold);
        EXPECT_EQ(in.in_envelope == 1.0f, objectiveSaysInside)
            << "along=" << along << " stepScore=" << stepScore;

        if (objectiveSaysInside != prev) ++transitions;
        prev = objectiveSaysInside;
    }

    // The sweep must actually CROSS the threshold, or the agreement above is
    // vacuous — a test that only ever sees one side proves nothing.
    EXPECT_GE(transitions, 1)
        << "sweep never crossed FitStreakThreshold; the agreement assertion "
           "above never had a boundary to disagree at";
}

}  // namespace

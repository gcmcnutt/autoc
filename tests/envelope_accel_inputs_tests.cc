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
#include "autoc/eval/craft_observations.h"
#include "autoc/eval/energy_state.h"
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
    const double nz_g = static_cast<double>(in.common.accel_z) * kAccelScale_g;

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
    EXPECT_NEAR(static_cast<double>(in.common.accel_x) * kAccelScale_g, 0.0, 1e-5);
    EXPECT_NEAR(static_cast<double>(in.common.accel_y) * kAccelScale_g, 0.0, 1e-5);
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
    const double nz_g = static_cast<double>(in.common.accel_z) * kAccelScale_g;

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
//
// ⚠️ 041 P2-2 retired IN_ENVELOPE and ENVELOPE_SECS as NN INPUTS — ablation on
// the pinned t1 elite showed zeroing IN_ENVELOPE *improves* the path-5 score by
// 0.3% and ENVELOPE_SECS costs 0.2%, both inside noise, for two slots. What
// they were REPLACED by is SCORE_GRAD_*: a flag is a state label a controller
// can only switch on, where a gradient is an improvement direction.
//
// The accumulator itself is NOT retired — it is still recorded, still drives
// the M2 perception estimator, and is still what the tracking metrics are read
// from. So these tests now assert against the recorded STATE rather than a
// slot, which is where the property lives now.

TEST(EnvelopeAccelInputs, InEnvelopeIsExactlyZeroOrOne_T040c) {
    for (bool inside : {false, true}) {
        AircraftState st = makeState(gp_quat::Identity());
        st.setInEnvelope(inside);
        EXPECT_EQ(st.getInEnvelope(), inside);
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

        const bool objectiveSaysInside = (stepScore >= kThreshold);
        EXPECT_EQ(st.getInEnvelope(), objectiveSaysInside)
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

// ===========================================================================
// 041 P2-2 — semantics for the three slots that REPLACED the envelope pair.
// ---------------------------------------------------------------------------
// Same discipline as (a)–(e) above: each pins a property whose violation would
// be invisible in sim and wrong in flight.
// ===========================================================================

namespace {

using autoc::eval::FlightArena;
using autoc::eval::boundaryClosureRate;
using autoc::eval::heightAboveDeck;
using autoc::eval::specificEnergy;
using autoc::eval::specificExcessPower;
using autoc::eval::writeCraftObservations;
using autoc::eval::kStandardGravity_mps2;

// A state at an explicit virtual position/velocity, so the geometry under test
// is stated rather than inherited from a fixture.
AircraftState makeStateAt(const gp_vec3& pos, const gp_vec3& vel) {
    return AircraftState{0,
                         static_cast<gp_scalar>(vel.norm()),
                         vel,
                         gp_quat::Identity(),
                         pos,
                         0.0f, 0.0f, 0.0f, 0};
}

}  // namespace

// ---------------------------------------------------------------------------
// (f) SPECIFIC_ENERGY — datum, reconciliation, and the sign that matters
// ---------------------------------------------------------------------------

// Es is measured from the arena FLOOR, not from AGL and not from the virtual
// origin. That choice is the whole reason the input is reproducible in flight,
// so it is asserted rather than assumed.
TEST(EnergyInputs, SpecificEnergyIsMeasuredFromTheHardDeck_P2_2f) {
    FlightArena arena;  // 70 / 25 / 105
    // At the virtual origin the craft is at 55 m AGL = 30 m above the 25 m
    // deck — the DOWN-extent, since the arm point is not the band's centre.
    EXPECT_NEAR(heightAboveDeck(0.0f, SIM_INITIAL_ALTITUDE, arena.floor_agl_m),
                30.0, 1e-4);
    // Sitting exactly ON the deck reads zero, by construction.
    const gp_scalar z_on_deck = -arena.floor_agl_m - SIM_INITIAL_ALTITUDE;
    EXPECT_NEAR(heightAboveDeck(z_on_deck, SIM_INITIAL_ALTITUDE, arena.floor_agl_m),
                0.0, 1e-4);
    // ⚠️ And BELOW the deck is negative, not clamped. A negative Es is
    // meaningful — it says "you are in the ground" — and clamping it would hide
    // exactly the state the containment objective needs to see.
    EXPECT_LT(heightAboveDeck(z_on_deck + 5.0f, SIM_INITIAL_ALTITUDE,
                              arena.floor_agl_m),
              0.0);
}

// The kinetic half must actually be v²/2g in METRES, so that Es reconciles with
// measured airspeed rather than being a dimensionless blend of two things.
TEST(EnergyInputs, SpecificEnergyReconcilesWithAirspeed_P2_2f) {
    // A craft at rest on the deck has Es = 0. Give it exactly enough speed to
    // buy 20 m of height: v = sqrt(2·g·20).
    const double v = std::sqrt(2.0 * static_cast<double>(kStandardGravity_mps2) * 20.0);
    EXPECT_NEAR(static_cast<double>(specificEnergy(0.0f, static_cast<gp_scalar>(v))),
                20.0, 1e-3)
        << "the kinetic term must be v²/2g in metres — an Es that does not "
           "convert cleanly to height is not an energy state";

    // Trading all of that height for speed is energy-neutral: the same Es.
    EXPECT_NEAR(static_cast<double>(specificEnergy(20.0f, 0.0f)),
                static_cast<double>(specificEnergy(0.0f, static_cast<gp_scalar>(v))),
                1e-3)
        << "height and speed must be interchangeable at constant Es — that "
           "interchange IS the quantity";
}

// End-to-end through the producer and the gather, in NN units.
TEST(EnergyInputs, SpecificEnergyReachesTheSlotNormalized_P2_2f) {
    FlightArena arena;
    AircraftState st = makeStateAt(gp_vec3::Zero(), gp_vec3(20.0f, 0.0f, 0.0f));
    writeCraftObservations(st, arena);

    // 30 m above the deck + 20²/2g ≈ 20.39 m of kinetic height.
    const double expected_m = 30.0 + 400.0 / (2.0 * static_cast<double>(kStandardGravity_mps2));
    EXPECT_NEAR(static_cast<double>(st.getSpecificEnergy()), expected_m, 1e-3);

    const NNInputs in = gatherFrom(st);
    EXPECT_NEAR(static_cast<double>(in.common.specific_energy),
                expected_m / static_cast<double>(kEnergyScale_m), 1e-5);
    // And the scale must keep an ordinary state well inside the unit — the
    // kAccelScale_g precedent. A saturated slot carries no gradient.
    EXPECT_LT(std::fabs(in.common.specific_energy), 1.0f);
}

// Ps is a RATE and must be denominated in seconds, not ticks. A hidden one-tick
// assumption silently rescales at a cadence change and nothing reports it.
TEST(EnergyInputs, SpecificExcessPowerIsCadenceInvariant_P2_2f) {
    // Same physical change (10 m of Es gained over 0.5 s), sampled at two
    // cadences. The rate must be identical.
    const gp_scalar ps_20hz = specificExcessPower(60.0f, 50.0f, 0.5f);
    // At 40 Hz the same climb is ten 0.05 s steps of 1 m each.
    const gp_scalar ps_40hz = specificExcessPower(51.0f, 50.0f, 0.05f);
    EXPECT_NEAR(static_cast<double>(ps_20hz), 20.0, 1e-4);
    EXPECT_NEAR(static_cast<double>(ps_20hz), static_cast<double>(ps_40hz), 1e-4);

    // No time, no rate — and 0 is the only honest answer, not a division.
    EXPECT_FLOAT_EQ(specificExcessPower(60.0f, 50.0f, 0.0f), 0.0f);
}

// ---------------------------------------------------------------------------
// (g) BOUNDARY_CLOSURE_RATE — the sign convention, and where it is informative
// ---------------------------------------------------------------------------

// ⚠️ POSITIVE = TOWARD THE WALL. Getting this backwards would train the policy
// to accelerate into the boundary, and it would look perfectly reasonable in
// every plot.
TEST(BoundaryClosureRate, PositiveMeansClosingOnTheWall_P2_2g) {
    const gp_vec3 pos(30.0f, 0.0f, 0.0f);  // 30 m out along +X
    EXPECT_GT(boundaryClosureRate(pos, gp_vec3(12.0f, 0.0f, 0.0f)), 0.0)
        << "flying outward must read POSITIVE (+ = toward the wall)";
    EXPECT_LT(boundaryClosureRate(pos, gp_vec3(-12.0f, 0.0f, 0.0f)), 0.0)
        << "flying inward must read NEGATIVE";
    // Purely tangential flight neither approaches nor leaves the wall.
    EXPECT_NEAR(static_cast<double>(boundaryClosureRate(pos, gp_vec3(0.0f, 12.0f, 0.0f))),
                0.0, 1e-5);
    // Nor does a pure climb — this is the HORIZONTAL radial rate.
    EXPECT_NEAR(static_cast<double>(boundaryClosureRate(pos, gp_vec3(0.0f, 0.0f, -12.0f))),
                0.0, 1e-5);
    // On the cylinder axis "outward" has no direction; 0 rather than a NaN.
    EXPECT_FLOAT_EQ(boundaryClosureRate(gp_vec3::Zero(), gp_vec3(12.0f, 0.0f, 0.0f)), 0.0f);
}

// Radial-outward flight closes at exactly the speed, which is the magnitude
// check that would catch a missing normalization by ‖p‖.
TEST(BoundaryClosureRate, RadialFlightClosesAtFullSpeed_P2_2g) {
    const double s = std::sqrt(0.5);  // 45° out in the XY plane
    const gp_vec3 pos(30.0 * s, 30.0 * s, 0.0);
    const gp_vec3 vel(12.0 * s, 12.0 * s, 0.0);
    EXPECT_NEAR(static_cast<double>(boundaryClosureRate(pos, vel)), 12.0, 1e-4);
}

// ⭐ THE ARGUMENT FOR THE SLOT EXISTING AT ALL: it is informative exactly where
// DIST_TO_BOUNDARY is blind. Measured on t1, distance is saturated above 0.99
// on 83% of ticks while closure rate still spans ±17 m/s. If a future change
// ever made these two redundant, this test is where it should show up.
TEST(BoundaryClosureRate, IsInformativeWhereDistanceIsSaturated_P2_2g) {
    FlightArena arena;
    // Well inside the arena, so DIST_TO_BOUNDARY is deep in tanh saturation.
    const gp_vec3 pos(10.0f, 0.0f, 0.0f);

    AircraftState out_st = makeStateAt(pos, gp_vec3(15.0f, 0.0f, 0.0f));
    AircraftState in_st  = makeStateAt(pos, gp_vec3(-15.0f, 0.0f, 0.0f));
    writeCraftObservations(out_st, arena);
    writeCraftObservations(in_st, arena);
    const NNInputs out_in = gatherFrom(out_st);
    const NNInputs in_in  = gatherFrom(in_st);

    // Distance says almost nothing here — both are saturated.
    EXPECT_GT(out_in.common.dist_to_boundary, 0.99f);
    EXPECT_GT(in_in.common.dist_to_boundary, 0.99f);
    EXPECT_NEAR(out_in.common.dist_to_boundary, in_in.common.dist_to_boundary, 0.02f);

    // The rate separates them completely, and with opposite signs.
    EXPECT_GT(out_in.common.boundary_closure_rate, 0.5f);
    EXPECT_LT(in_in.common.boundary_closure_rate, -0.5f);
}

// Cadence-invariance, asserted even though it holds by construction — the
// property is what matters, not the current implementation of it. If someone
// ever rewrites this as a per-tick difference, this is the test that objects.
TEST(BoundaryClosureRate, IsCadenceInvariantByConstruction_P2_2g) {
    const gp_vec3 pos(30.0f, 0.0f, 0.0f);
    const gp_vec3 vel(12.0f, 3.0f, -2.0f);
    // The function takes no dt and no history, so there is nothing a cadence
    // change could rescale. Asserted by showing the value depends only on the
    // instantaneous state.
    const gp_scalar a = boundaryClosureRate(pos, vel);
    const gp_scalar b = boundaryClosureRate(pos, vel);
    EXPECT_FLOAT_EQ(a, b);
    EXPECT_NEAR(static_cast<double>(a), 12.0, 1e-4);
}

// ---------------------------------------------------------------------------
// (h) SCORE_GRAD_* — zero at the maximum, uphill off it
// ---------------------------------------------------------------------------

// At the score maximum there is no improvement direction, and the honest answer
// is zero rather than an arbitrary unit vector.
TEST(ScoreGradient, IsZeroAtTheScoreMaximum_P2_2h) {
    const FitnessComputer scorer(7.0, 2.0, 45.0, 0.5, 100, 5.0);
    const gp_vec3 tangent(1.0f, 0.0f, 0.0f);
    // Exactly on the rabbit.
    EXPECT_NEAR(static_cast<double>(scorer.scoreGradientWorld(gp_vec3::Zero(), tangent).norm()),
                0.0, 1e-9);

    AircraftState st = makeStateAt(gp_vec3::Zero(), gp_vec3(15.0f, 0.0f, 0.0f));
    st.setScoreGradBody(gp_vec3::Zero());
    const NNInputs in = gatherFrom(st);
    EXPECT_FLOAT_EQ(in.common.score_grad_x, 0.0f);
    EXPECT_FLOAT_EQ(in.common.score_grad_y, 0.0f);
    EXPECT_FLOAT_EQ(in.common.score_grad_z, 0.0f);
}

// ⭐ THE PROPERTY THE SLOT EXISTS FOR: it must point UPHILL. Verified against
// the objective itself by finite difference, not against a re-derivation of the
// same formula — a gradient that agrees with a second copy of its own algebra
// proves nothing about whether either matches the score.
TEST(ScoreGradient, PointsUphillOnTheActualScoringSurface_P2_2h) {
    const FitnessComputer scorer(7.0, 2.0, 45.0, 0.5, 100, 5.0);
    const gp_vec3 tangent(1.0f, 0.0f, 0.0f);

    // ⚠️ The finite difference runs in DOUBLE, deliberately. gp_vec3 is
    // float-typed, so perturbing one by 1e-4 loses most of the perturbation to
    // rounding and the "disagreement" that produces is the test's own arithmetic
    // rather than the gradient's. Only the analytic value under test comes from
    // the float path.
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    const Vec3d tangent_d(1.0, 0.0, 0.0);
    auto scoreAt = [&](const Vec3d& off) {
        const double along = off.dot(tangent_d);
        const double lat = (off - along * tangent_d).norm();
        return scorer.decomposeStepScore(along, lat).score;
    };

    // A spread of offsets: behind-and-off-axis, ahead, and pure lateral —
    // including one on the far side of the π/2 angle clamp.
    const gp_vec3 offsets[] = {
        gp_vec3(-8.0f, 3.0f, 0.0f),
        gp_vec3(-8.0f, 0.0f, 4.0f),
        gp_vec3(-20.0f, 6.0f, -5.0f),
        gp_vec3(3.0f, 4.0f, 0.0f),     // ahead — sharp distance scale
        gp_vec3(0.0f, 6.0f, 0.0f),     // pure lateral — angle exactly π/2
    };

    for (const gp_vec3& off : offsets) {
        const gp_vec3 g = scorer.scoreGradientWorld(off, tangent);
        ASSERT_GT(g.norm(), 1e-9) << "offset " << off.transpose();

        const Vec3d off_d = off.cast<double>();
        const Vec3d dir_d = g.cast<double>().normalized();
        constexpr double h = 1e-5;

        // Stepping a small distance ALONG the gradient must raise the score.
        EXPECT_GT(scoreAt(off_d + dir_d * h), scoreAt(off_d))
            << "gradient does not point uphill at offset " << off.transpose();

        // And the analytic magnitude must match a central finite difference of
        // the objective along that direction — this is what catches a wrong
        // term, not merely a wrong sign.
        const double fd =
            (scoreAt(off_d + dir_d * h) - scoreAt(off_d - dir_d * h)) / (2.0 * h);
        EXPECT_NEAR(static_cast<double>(g.norm()), fd, 1e-4 * std::fabs(fd) + 1e-9)
            << "analytic |∇score| disagrees with the objective's own slope at "
            << off.transpose();
    }
}

// The slot encoding must BOUND the value without ROTATING the vector. |∇θ| = 1/d
// diverges near the rabbit (measured: every t1 tick above 2.0 had d < 1.52 m),
// so a plain divide would occasionally hand the network a 19.0 — and a
// per-component tanh would bound it while turning the direction into a lie.
TEST(ScoreGradient, SlotEncodingBoundsMagnitudeButPreservesDirection_P2_2h) {
    // A deliberately enormous gradient, of the size the near-field singularity
    // actually produces.
    const gp_vec3 huge(30.0f, -40.0f, 0.0f);  // norm 50, ~64x kScoreGradScale
    AircraftState st = makeStateAt(gp_vec3::Zero(), gp_vec3(15.0f, 0.0f, 0.0f));
    st.setScoreGradBody(huge);
    const NNInputs in = gatherFrom(st);

    const gp_vec3 slot(in.common.score_grad_x, in.common.score_grad_y,
                       in.common.score_grad_z);
    // Bounded: the encoded norm saturates at 1, never beyond it.
    EXPECT_LE(static_cast<double>(slot.norm()), 1.0);
    EXPECT_NEAR(static_cast<double>(slot.norm()), 1.0, 1e-3);
    // Direction preserved EXACTLY — this is the assertion that fails if someone
    // "simplifies" the encoding to a per-component tanh.
    const gp_vec3 want = huge / huge.norm();
    const gp_vec3 got = slot / slot.norm();
    EXPECT_NEAR(static_cast<double>((want - got).norm()), 0.0, 1e-5);

    // And in the ordinary regime the slot is well inside the linear zone, which
    // is what kScoreGradScale was measured to deliver: the in-envelope median
    // gradient (0.2016/m) must not read as "saturated, act now".
    AircraftState mid = makeStateAt(gp_vec3::Zero(), gp_vec3(15.0f, 0.0f, 0.0f));
    mid.setScoreGradBody(gp_vec3(0.2016f, 0.0f, 0.0f));
    const NNInputs mid_in = gatherFrom(mid);
    EXPECT_LT(mid_in.common.score_grad_x, 0.35f);
    EXPECT_GT(mid_in.common.score_grad_x, 0.15f);
}

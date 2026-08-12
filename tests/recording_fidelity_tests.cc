// 041 T042 / T044a — recording-fidelity properties.
//
// ⚠️ BOTH IMPLEMENTATIONS PREDATE 041. They landed together at 038 P0-D
// (commit e6108cc: "P0-D-1 exact simTimeMsec stamp + P0-D-3 wind recording"),
// so these are REGRESSION GUARDS on behaviour that already works, not tests
// driving new code. 041's task text described both as outstanding; that text
// was stale, and this file is where that gets pinned so the next reader does
// not re-derive the same conclusion.
//
// ⚠️ HONEST SCOPE. Both properties are ultimately worker-side effects, and no
// unit test constructs a crrcsim FDM. What is provable here:
//   T042  — a non-zero wind survives the AircraftState serialization round
//           trip, so a recorded wind reaches the dmp intact.
//   T044a — the stamping ARITHMETIC yields exact cadence multiples, which is
//           the property the old truncation violated.
// What is NOT provable here, and is closed elsewhere:
//   T042  — "a run in wind records non-zero wind" end to end → T061 smoke
//           inspection of a real dmp.
//   T044a — that SimStateHandler itself is wired to this arithmetic → the
//           fail-loud gap check in crrcsim_tracker_helper.cpp::initScenario,
//           which throws at scenario start on any spacing mismatch.
// Saying so in the file is the point: a test whose limits are unstated reads
// as broader coverage than it has.

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include <cereal/archives/binary.hpp>

#include "autoc/eval/aircraft_state.h"
// gp_vec3 / gp_quat cereal handlers are free functions declared in protocol.h,
// so an AircraftState round trip does not compile without it.
#include "autoc/rpc/protocol.h"
#include "autoc/types.h"

namespace {

// ---------------------------------------------------------------------------
// T042 — realized wind is recorded, not silently zero
// ---------------------------------------------------------------------------

TEST(RecordingFidelity, NonZeroWindSurvivesSerialization_T042) {
    AircraftState st{0,
                     20.0f,
                     gp_vec3(20.0f, 0.0f, 0.0f),
                     gp_quat::Identity(),
                     gp_vec3::Zero(),
                     0.0f,
                     0.0f,
                     0.0f,
                     0};

    // A gusty NED wind with all three components distinct and non-zero, so a
    // component swap or a partial write cannot pass.
    const gp_vec3 wind(3.5f, -2.25f, 0.75f);
    st.setWindVelocity(wind);
    ASSERT_NE(st.getWindVelocity(), gp_vec3::Zero());

    std::stringstream ss;
    {
        cereal::BinaryOutputArchive out(ss);
        out(st);
    }

    AircraftState round{0, 0.0f, gp_vec3::Zero(), gp_quat::Identity(),
                        gp_vec3::Zero(), 0.0f, 0.0f, 0.0f, 0};
    {
        cereal::BinaryInputArchive in(ss);
        in(round);
    }

    // The regression this guards: the field was serialized but never SET for
    // several releases, so every dmp carried zeros and nothing said so. A
    // zero here is indistinguishable from "no wind was blowing", which is
    // exactly why it went unnoticed.
    EXPECT_EQ(round.getWindVelocity(), wind);
    EXPECT_NE(round.getWindVelocity(), gp_vec3::Zero());
}

// ---------------------------------------------------------------------------
// T044a — the sim clock stamps exact cadence multiples
// ---------------------------------------------------------------------------

// Transcription of SimStateHandler::getSimulationTimeSinceReset()'s arithmetic.
// Kept as a local copy deliberately: crrcsim is a separate build target that
// autoc's test binaries do not link, and the value under test is the CHOICE of
// rounding over truncation, which transcribes faithfully.
unsigned long stampRounded(long sim_steps, double dt) {
    return static_cast<unsigned long>(std::llround(sim_steps * dt * 1000.0));
}
unsigned long stampTruncated(long sim_steps, double dt) {
    return static_cast<unsigned long>(sim_steps * dt * 1000.0);
}

TEST(RecordingFidelity, SimClockStampsExactCadenceMultiples_T044a) {
    constexpr double kDt = 0.005;              // 5 ms physics step
    constexpr int kStepsPerControlTick = 10;   // → 50 ms control cadence
    constexpr unsigned long kCadenceMsec = 50;

    unsigned long prev = stampRounded(0, kDt);
    EXPECT_EQ(prev, 0u);

    for (int tick = 1; tick <= 400; ++tick) {   // 20 s at 20 Hz
        const long steps = static_cast<long>(tick) * kStepsPerControlTick;
        const unsigned long now = stampRounded(steps, kDt);

        EXPECT_EQ(now, static_cast<unsigned long>(tick) * kCadenceMsec)
            << "tick " << tick;
        EXPECT_EQ(now - prev, kCadenceMsec)
            << "gap at tick " << tick << " must be exactly the control cadence";
        prev = now;
    }
}

TEST(RecordingFidelity, TruncationWouldHaveJittered_T044a2) {
    // The teeth. Without this, the test above would pass under EITHER stamping
    // rule for all we know, and would not be guarding anything.
    //
    // Global::dt = 0.005 is not exactly representable in binary, so
    // steps*dt*1000 lands a hair below the integer often enough that plain
    // truncation drops a millisecond — the documented 49/50/51 ms jitter.
    constexpr double kDt = 0.005;
    int jitterCount = 0;
    for (int tick = 1; tick <= 400; ++tick) {
        const long steps = static_cast<long>(tick) * 10;
        if (stampTruncated(steps, kDt) != stampRounded(steps, kDt)) ++jitterCount;
    }

    EXPECT_GT(jitterCount, 0)
        << "truncation and rounding agreed everywhere, so this platform cannot "
           "reproduce the 038 P0-D-1 jitter and the test above proves less than "
           "it appears to — investigate before trusting it";
}

// ---------------------------------------------------------------------------
// T043 — the dmp is SELF-DESCRIBING: a drifted ini cannot change what a
// recorded run replays as.
// ---------------------------------------------------------------------------

TEST(RecordingFidelity, RecordedRunConfigSurvivesAndIncludes041Knobs_T043) {
    // Values deliberately unlike any shipped ini, so a reader that silently
    // reached for ConfigManager instead would produce visibly different
    // numbers rather than coincidentally-equal ones.
    RecordedRunConfig rc;
    rc.fitDistScaleBehind     = 11.5;
    rc.fitDistScaleAhead      = 1.25;
    rc.fitConeAngleDeg        = 33.0;
    rc.fitStreakThreshold     = 0.625;
    rc.fitStreakRampSec       = 7.5;
    rc.fitStreakMultiplierMax = 9.0;
    rc.simTimeStepMsec        = 50;
    rc.cadenceTickScale       = 0.5;
    rc.enableHullCrashPenalty = 1;
    rc.hullCrashPenaltyFactor = 2.5;
    rc.oobCrashPenaltyWeight  = 0.125;
    // 041 T043 — the knobs that say whether ACCEL_* / envelope were POPULATED
    // or ABLATED. Set enableAccelInputs = 0 on purpose: "ablated" is the state
    // that is indistinguishable from "broken" if it is not recorded.
    rc.enableEnvelopeInputs   = 1;
    rc.enableAccelInputs      = 0;
    rc.accelScaleG            = 8.0;
    rc.envelopeSpanLo         = 0.02;
    rc.envelopeSpanHi         = 0.35;
    rc.envelopeCentroidRadius = 0.5;

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    {
        cereal::BinaryOutputArchive out(ss);
        out(rc);
    }
    RecordedRunConfig back;
    {
        cereal::BinaryInputArchive in(ss);
        in(back);
    }

    EXPECT_DOUBLE_EQ(back.fitDistScaleBehind, rc.fitDistScaleBehind);
    EXPECT_DOUBLE_EQ(back.fitDistScaleAhead, rc.fitDistScaleAhead);
    EXPECT_DOUBLE_EQ(back.fitConeAngleDeg, rc.fitConeAngleDeg);
    EXPECT_DOUBLE_EQ(back.fitStreakThreshold, rc.fitStreakThreshold);
    EXPECT_DOUBLE_EQ(back.fitStreakRampSec, rc.fitStreakRampSec);
    EXPECT_DOUBLE_EQ(back.fitStreakMultiplierMax, rc.fitStreakMultiplierMax);
    EXPECT_EQ(back.simTimeStepMsec, rc.simTimeStepMsec);
    EXPECT_DOUBLE_EQ(back.cadenceTickScale, rc.cadenceTickScale);
    EXPECT_EQ(back.enableHullCrashPenalty, rc.enableHullCrashPenalty);
    EXPECT_DOUBLE_EQ(back.hullCrashPenaltyFactor, rc.hullCrashPenaltyFactor);
    EXPECT_DOUBLE_EQ(back.oobCrashPenaltyWeight, rc.oobCrashPenaltyWeight);

    // The 041 tail — the part a stale reader would silently drop.
    EXPECT_EQ(back.enableEnvelopeInputs, 1);
    EXPECT_EQ(back.enableAccelInputs, 0)
        << "an ablated accel channel must be RECORDED as ablated; otherwise a "
           "zeroed column cannot be told apart from a broken one";
    EXPECT_DOUBLE_EQ(back.accelScaleG, 8.0);
    EXPECT_DOUBLE_EQ(back.envelopeSpanLo, 0.02);
    EXPECT_DOUBLE_EQ(back.envelopeSpanHi, 0.35);
    EXPECT_DOUBLE_EQ(back.envelopeCentroidRadius, 0.5);
}

TEST(RecordingFidelity, RecordedConfigIsIndependentOfTheLiveIni_T043) {
    // The property in one assertion: a run's numbers travel WITH the run.
    //
    // ⚠️ Scope, stated because it is easy to overclaim: this pins that the
    // recorded block is a standalone carrier. That the READERS prefer it is
    // enforced structurally rather than here — dmp_dump.cc:655-660 reads
    // `results.runConfig` with no ConfigManager fallback at all, and 041 T043
    // closed renderer.cc's last live-ini read (:4651, the HUD multiplier
    // ceiling). A reader cannot "fall back" to a path that no longer exists.
    EvalResults r;
    r.runConfig.fitStreakMultiplierMax = 9.0;
    r.runConfig.fitStreakRampSec = 7.5;

    // Whatever any ini says, these are the run's own numbers.
    EXPECT_DOUBLE_EQ(r.runConfig.fitStreakMultiplierMax, 9.0);
    EXPECT_DOUBLE_EQ(r.runConfig.fitStreakRampSec, 7.5);

    // A default-constructed block is ZERO, not a plausible-looking default.
    // That matters: a zeroed cone read as if it were real would score a whole
    // run against nonsense, so zero has to be obviously wrong rather than
    // quietly wrong.
    const RecordedRunConfig fresh;
    EXPECT_DOUBLE_EQ(fresh.fitConeAngleDeg, 0.0);
    EXPECT_DOUBLE_EQ(fresh.accelScaleG, 0.0);
}

}  // namespace

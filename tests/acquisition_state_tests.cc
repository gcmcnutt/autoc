// 040 T051-T056a (US4) — the per-beacon acquisition state machine.
//
// WHAT THIS SUITE IS FOR. Through 039, acquisition was instantaneous: a beacon
// that became geometrically visible was fully trusted on the same tick. That
// erased the reacquisition cost which 037's M2 de-risking identified as the
// documented tracking bottleneck (see project memory: M2 depth is
// architecture/perception-capped, and the named next lever is
// reacquire-through-blindness).
//
// The machine mirrors the SHIPPED GATEWARE FSM rather than inventing one —
// SEARCH → ACQUIRING → LOCKED → HOLD, with timings hardware-measured at N=31 via
// the decoder's recovery counter (research R12). Everything here is expressed in
// CONTROLLER TICKS at 20 Hz because that is the tolerance SC-005 states.
//
// THE BEHAVIOUR-DEFINING PROPERTY, and the reason these tests exist at all: the
// coast window is WALLCLOCK-driven (emitter↔receiver oscillator stability), not
// code-length driven. It is ~10 s, and the documented M2 worst-case blind window
// is ~8 s — INSIDE it. So most M2 reacquisitions are WARM (154 ms), not cold
// (308 ms), and "N=31 triples acquisition" holds only for the cold path M2
// rarely takes. If a future change collapses warm and cold into one number, the
// model loses the thing that makes it worth having; `WarmRelockIsCheaperThanCold`
// is the assertion standing in the way.
//
// DETERMINISM (FR-020): thresholds and time constants only. The 031 probability
// curves enter as deterministic gates, never as Bernoulli draws — sampling is
// disqualified outright because bit-replay is a project gate.

#include <gtest/gtest.h>

#include <vector>

#include "autoc/eval/acquisition_state.h"
#include "autoc/eval/camera_projection.h"  // kCepSentinelFloat / kCepSentinelThreshold
#include "autoc/types.h"

using autoc::eval::AcquisitionConfig;
using autoc::eval::AcquisitionState;
using autoc::eval::advanceAcquisition;
using autoc::eval::hb1AcquisitionConfig;
using autoc::eval::kCepSentinelThreshold;
using autoc::eval::LockState;
using autoc::eval::qualityFromAcquisition;

namespace {

// 20 Hz control loop — the operating cadence fixed by 037.
constexpr gp_scalar kTickMs = static_cast<gp_scalar>(50);

// A healthy, close-in signal: q at the confident end of the hardware's 0-9
// scale. Held constant so these tests isolate TIMING from signal strength.
constexpr gp_scalar kStrongQ = static_cast<gp_scalar>(9);

// Drive the machine for `ticks` and report how many elapsed before it first
// reached TRACKING. Returns -1 if it never did.
int ticksToTracking(AcquisitionState& s, const AcquisitionConfig& cfg, int ticks) {
    for (int i = 1; i <= ticks; ++i) {
        advanceAcquisition(s, true, kStrongQ, kTickMs, cfg);
        if (s.state == LockState::TRACKING) return i;
    }
    return -1;
}

void runVisible(AcquisitionState& s, const AcquisitionConfig& cfg, int ticks) {
    for (int i = 0; i < ticks; ++i) advanceAcquisition(s, true, kStrongQ, kTickMs, cfg);
}

void runBlind(AcquisitionState& s, const AcquisitionConfig& cfg, int ticks) {
    for (int i = 0; i < ticks; ++i)
        advanceAcquisition(s, false, static_cast<gp_scalar>(0), kTickMs, cfg);
}

}  // namespace

// ---------------------------------------------------------------------------
// T051 (FR-017, SC-005) — acquisition takes modelled time.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, ColdAcquireTakesTheMeasuredWindow) {
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();

    ASSERT_EQ(s.state, LockState::SEARCHING);

    const int n = ticksToTracking(s, cfg, 40);
    // 308 ms at 50 ms/tick = 6.16 ticks ⇒ 7. SC-005 allows ±1.
    ASSERT_GT(n, 0) << "must eventually lock on a strong signal";
    EXPECT_NEAR(n, 7, 1) << "cold acquire should land on the measured 308 ms";
    EXPECT_GT(n, 1) << "acquisition must NOT be instantaneous — that is the 039 behaviour "
                       "this whole story replaces";
}

TEST(AcquisitionState, WarmRelockIsCheaperThanCold) {
    // The single highest-value feature in the model (R12 decision 4). If this
    // ever reads equal, the coast-window design has been silently defeated.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();

    AcquisitionState cold;
    cold.reset();
    const int cold_ticks = ticksToTracking(cold, cfg, 40);

    AcquisitionState warm;
    warm.reset();
    ASSERT_GT(ticksToTracking(warm, cfg, 40), 0);
    runBlind(warm, cfg, 2);  // brief interruption, well inside the coast window
    const int warm_ticks = ticksToTracking(warm, cfg, 40);

    ASSERT_GT(warm_ticks, 0);
    // 154 ms at 50 ms/tick = 3.08 ticks ⇒ 4.
    EXPECT_NEAR(warm_ticks, 4, 1) << "warm relock should land on the measured 154 ms";
    EXPECT_LT(warm_ticks, cold_ticks)
        << "warm MUST be strictly cheaper than cold — this is the coast window's "
           "entire reason for existing";
}

TEST(AcquisitionState, ReacquisitionBeyondTheCoastWindowIsTrueCold) {
    // Operator reports the cold path does occur in flight (sun, reflections) at
    // losses beyond ~10 s, so it is modelled rather than assumed away.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();

    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);

    // Blind for longer than the coast window ⇒ the flywheel's rate estimate is
    // stale and the next lock must pay full price.
    const int coast_ticks =
        static_cast<int>(static_cast<double>(cfg.coast_window_ms) /
                         static_cast<double>(kTickMs)) + 4;
    runBlind(s, cfg, coast_ticks);
    EXPECT_EQ(s.state, LockState::SEARCHING);

    const int n = ticksToTracking(s, cfg, 40);
    ASSERT_GT(n, 0);
    EXPECT_NEAR(n, 7, 1) << "past the coast window, re-acquire is true-cold";
}

// ---------------------------------------------------------------------------
// T052 (FR-017a) — tentative lock reports a bearing.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, TentativeLockReportsLargeVarianceQualityThatImproves) {
    // FR-017a / FR-017c: the controller is NOT starved during the acquisition
    // window. The blob centroid exists before the code decodes — detection and
    // identification are separate operations — so a bearing is available
    // pre-lock and quality carries the discount.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();

    std::vector<double> quality;
    for (int i = 0; i < 7; ++i) {
        advanceAcquisition(s, true, kStrongQ, kTickMs, cfg);
        const gp_scalar cep = qualityFromAcquisition(s, false, cfg);
        // Never the sentinel: the beacon IS visible, it is merely untrusted.
        ASSERT_LT(static_cast<double>(cep), static_cast<double>(kCepSentinelThreshold))
            << "a tentative lock must still report a usable bearing (tick " << i << ")";
        quality.push_back(static_cast<double>(cep));
    }

    // Large at first...
    EXPECT_GT(quality.front(), 0.5)
        << "the first tentative tick must be explicitly untrusted";
    // ...improving monotonically toward confirmed.
    for (size_t i = 1; i < quality.size(); ++i) {
        EXPECT_LE(quality[i], quality[i - 1] + 1e-9)
            << "quality must improve (cep fall) as the code integrates, tick " << i;
    }
    EXPECT_LT(quality.back(), 0.1) << "confirmed lock on a strong signal must be confident";
}

TEST(AcquisitionState, UnresolvedIdentityInflatesQuality) {
    // FR-017d. Quality is the interface's ONLY confidence channel — separation
    // and tilt are bare values and the input vector is fixed at 58, so no
    // channel can be added. Tilt SIGN depends on beacon identity while
    // separation does not: a swapped pair flips tilt by 180°, and tilt drives
    // the roll command. Two cleanly-detected blobs would otherwise carry LOW cep
    // (confident positions) while identity is unknown — confidently wrong tilt
    // with nothing flagging it.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);

    const double resolved = static_cast<double>(qualityFromAcquisition(s, false, cfg));
    const double unresolved = static_cast<double>(qualityFromAcquisition(s, true, cfg));

    EXPECT_LT(resolved, 0.1) << "sanity: a confirmed strong lock is confident";
    EXPECT_GT(unresolved, resolved)
        << "unresolved identity MUST inflate quality — the whole point of FR-017d";
    EXPECT_GT(unresolved, 0.5) << "and must land in the large/tentative regime";
    EXPECT_LT(unresolved, static_cast<double>(kCepSentinelThreshold))
        << "but never masquerade as not-visible";
}

// ---------------------------------------------------------------------------
// T053 (FR-018, SC-005) — hold ride-through.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, InterruptionShorterThanTheHoldWindowNeverLosesTracking) {
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);

    // hold_max is 308 ms ⇒ 6.16 ticks. Interrupt for 4, comfortably inside.
    for (int i = 0; i < 4; ++i) {
        advanceAcquisition(s, false, static_cast<gp_scalar>(0), kTickMs, cfg);
        EXPECT_EQ(s.state, LockState::HOLDING)
            << "must coast, not drop to SEARCHING, at tick " << i;
    }
    EXPECT_TRUE(s.ever_locked);
}

TEST(AcquisitionState, InterruptionLongerThanTheHoldWindowDropsToSearching) {
    // The complement — without it the test above would pass on a machine that
    // simply never leaves HOLDING.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);

    runBlind(s, cfg, 8);  // > 6.16 ticks
    EXPECT_EQ(s.state, LockState::SEARCHING);
}

TEST(AcquisitionState, QualityIsSentinelWhileBlind) {
    // HOLDING is an INTERNAL coast, not a claim that the beacon is visible.
    // There is no blob, so there is no bearing, and the interface must say so.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);

    advanceAcquisition(s, false, static_cast<gp_scalar>(0), kTickMs, cfg);
    ASSERT_EQ(s.state, LockState::HOLDING);
    EXPECT_GE(static_cast<double>(qualityFromAcquisition(s, false, cfg)),
              static_cast<double>(kCepSentinelThreshold))
        << "coasting internally must not be reported as a visible beacon";
}

// ---------------------------------------------------------------------------
// T056a (FR-013, US3 acceptance scenario 6) — obstruction ride-through.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, ClearedObstructionReEstablishesViaRideThroughNotInstantly) {
    // DISTINCT FROM the signal-loss case above. T064a names the asymmetry: an
    // obstruction clears deterministically by GEOMETRY while a signal returns by
    // SNR. Both must enter the same hold machinery — only obstruction has a
    // knowable clear-time. The failure this guards is the tempting shortcut of
    // restoring the previous lock the instant the geometry clears, because "we
    // know it is the same beacon".
    const AcquisitionConfig cfg = hb1AcquisitionConfig();
    AcquisitionState s;
    s.reset();
    ASSERT_GT(ticksToTracking(s, cfg, 40), 0);
    ASSERT_EQ(s.state, LockState::TRACKING);

    // Propeller blade / nose shadow crosses: two ticks of hard block.
    runBlind(s, cfg, 2);
    ASSERT_EQ(s.state, LockState::HOLDING);

    // Geometry clears. The very next tick must NOT be TRACKING.
    advanceAcquisition(s, true, kStrongQ, kTickMs, cfg);
    EXPECT_NE(s.state, LockState::TRACKING)
        << "a cleared obstruction must re-establish through the hold path, "
           "not restore instantaneously";

    // ...and must complete on the WARM budget, since the coast window holds.
    int extra = 1;
    while (s.state != LockState::TRACKING && extra < 20) {
        advanceAcquisition(s, true, kStrongQ, kTickMs, cfg);
        ++extra;
    }
    EXPECT_NEAR(extra, 4, 1) << "recovery should follow the measured warm ride-through";
}

// ---------------------------------------------------------------------------
// T054 (FR-020, SC-006) — determinism.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, IdenticalInputSequencesProduceBitIdenticalState) {
    const AcquisitionConfig cfg = hb1AcquisitionConfig();

    // A deliberately awkward sequence: partial acquisitions, holds that expire,
    // holds that do not, and re-locks on both budgets.
    const std::vector<bool> visible = {
        true, true, true, false, false, true, true, true, true, true,
        false, false, false, false, false, false, false, false, true, true,
        true, true, true, false, true, true, true, true, true, true};

    AcquisitionState a, b;
    a.reset();
    b.reset();

    for (size_t i = 0; i < visible.size(); ++i) {
        const gp_scalar q = visible[i] ? kStrongQ : static_cast<gp_scalar>(0);
        advanceAcquisition(a, visible[i], q, kTickMs, cfg);
        advanceAcquisition(b, visible[i], q, kTickMs, cfg);

        ASSERT_EQ(a.state, b.state) << "tick " << i;
        ASSERT_EQ(a.time_in_state_ms, b.time_in_state_ms) << "tick " << i;
        ASSERT_EQ(a.time_since_loss_ms, b.time_since_loss_ms) << "tick " << i;
        ASSERT_EQ(a.credit_ms, b.credit_ms) << "tick " << i;
        ASSERT_EQ(a.q, b.q) << "tick " << i;
        ASSERT_EQ(a.ever_locked, b.ever_locked) << "tick " << i;
        ASSERT_EQ(qualityFromAcquisition(a, false, cfg),
                  qualityFromAcquisition(b, false, cfg))
            << "tick " << i;
    }
}

// ---------------------------------------------------------------------------
// T055 (FR-020a) — per-scenario reset.
// ---------------------------------------------------------------------------

TEST(AcquisitionState, ResetRestoresExactlyTheFreshState) {
    const AcquisitionConfig cfg = hb1AcquisitionConfig();

    AcquisitionState fresh;
    fresh.reset();

    // Run one deep enough to have filled every field with something non-trivial:
    // locked, coasted, expired, re-locked.
    AcquisitionState used;
    used.reset();
    runVisible(used, cfg, 10);
    runBlind(used, cfg, 9);
    runVisible(used, cfg, 6);
    runBlind(used, cfg, 3);
    ASSERT_NE(used.state, LockState::SEARCHING);
    ASSERT_TRUE(used.ever_locked);

    used.reset();

    // Bit-exact, not near. Unreset state leaks between scenarios and breaks the
    // bitwise gate — the trap FR-020a exists for, and the one the existing
    // situational-awareness state already carries a warning about.
    EXPECT_EQ(used.state, fresh.state);
    EXPECT_EQ(used.time_in_state_ms, fresh.time_in_state_ms);
    EXPECT_EQ(used.time_since_loss_ms, fresh.time_since_loss_ms);
    EXPECT_EQ(used.credit_ms, fresh.credit_ms);
    EXPECT_EQ(used.q, fresh.q);
    EXPECT_EQ(used.ever_locked, fresh.ever_locked);
}

TEST(AcquisitionState, ResetReproducesFirstTickBehaviourAfterADeepPriorRun) {
    // The observable form of the test above, mirroring the T007 reset-contract
    // pattern: assert what a SCENARIO sees, not merely what the struct holds.
    // A machine that reset its fields but kept, say, a warm coast would pass the
    // field comparison and fail here.
    const AcquisitionConfig cfg = hb1AcquisitionConfig();

    AcquisitionState virgin;
    virgin.reset();
    const int virgin_ticks = ticksToTracking(virgin, cfg, 40);

    AcquisitionState recycled;
    recycled.reset();
    runVisible(recycled, cfg, 25);
    runBlind(recycled, cfg, 4);
    runVisible(recycled, cfg, 12);
    recycled.reset();
    const int recycled_ticks = ticksToTracking(recycled, cfg, 40);

    EXPECT_EQ(recycled_ticks, virgin_ticks)
        << "a reset scenario must acquire COLD, exactly as a fresh one does — "
           "a leaked coast window would make it acquire warm and cheap";
}

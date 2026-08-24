// 038 P0-D FR-P0H (T009c) — situational-awareness input contract tests.
//
// Behavioral + determinism coverage for the T009b enrichment (mainlined
// baseline per Constitution I, not a spike):
//   (A) time_since_seen increments while blind, resets on a CEP-sentinel
//       re-sighting AND at scenario/engage start (SituationalAwarenessState
//       is reset per scenario in both steppers); exit_dir is the held
//       last-seen beacon-pair bearing and freezes through blindness.
//   (B) inward_body is a true BODY-FRAME radial-inward unit vector — proven
//       against a known attitude+position geometry INCLUDING an inverted /
//       knife-edge case where the inward direction lands entirely out of the
//       body xy-plane (a planar sin/cos heading would drop it).
//   M1 (pathgen) carries (B) but NOT (A) — its rabbit is always visible.
//   Per-scenario reset determinism: identical update sequence ⇒ identical
//   input trace (the FR-030 bitwise-gate invariant, at the unit level).

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "autoc/eval/aircraft_state.h"      // AircraftState, SinglePathProvider, SIM_TIME_STEP_MSEC
#include "autoc/eval/arena.h"               // FlightArena, inwardBodyDirection
#include "autoc/eval/camera_projection.h"   // kCepSentinelThreshold
#include "autoc/nn/evaluator.h"             // SituationalAwarenessState, gather_*_inputs
#include "autoc/nn/nn_inputs.h"             // TrackerInputs, NNInputs, kTimeSinceSeenScale_s
#include "autoc/types.h"

using autoc::eval::FlightArena;
using autoc::eval::inwardBodyDirection;
using autoc::eval::kCepSentinelThreshold;

namespace {

// Chase state at a given position/attitude; velocity along body-x cruise.
AircraftState makeChase(const gp_vec3& pos, const gp_quat& q) {
    return AircraftState{0, /*relVel=*/15.0f,
                         q * gp_vec3(15.0f, 0.0f, 0.0f),
                         q, pos, 0.0f, 0.0f, 0.0f, 0};
}

// A visible ("seen") beacon CEP is anything below the sentinel threshold; an
// invisible one is the dequantized sentinel marker.
constexpr float kSeenCep = 0.2f;
constexpr float kBlindCep = 1.5f;  // >= kCepSentinelThreshold (1.25)

// Fill a history window with a single beacon-pair sample across all slots.
TrackerHistoryWindow uniformHistory(float lx, float ly, float lcep,
                                    float rx, float ry, float rcep) {
    TrackerHistoryWindow h{};
    for (int i = 0; i < 6; ++i) {
        h.left_x[i] = lx;  h.left_y[i] = ly;  h.left_cep[i] = lcep;
        h.right_x[i] = rx; h.right_y[i] = ry; h.right_cep[i] = rcep;
        h.span[i] = 0.0f;
    }
    return h;
}

// ============================================================
// (B) inward_body — body-frame radial-inward unit vector
// ============================================================

TEST(SituationalInputs, InwardBodyLevelPointsAlongNegativeBodyX) {
    // Craft east of the axis, wings level, nose along +x: world inward is
    // straight back along the nose (-x); body frame == world frame.
    const gp_vec3 v = inwardBodyDirection(gp_vec3(10.0, 0.0, 0.0),
                                          gp_quat::Identity());
    EXPECT_NEAR(v.x(), -1.0, 1e-6);
    EXPECT_NEAR(v.y(),  0.0, 1e-6);
    EXPECT_NEAR(v.z(),  0.0, 1e-6);
}

TEST(SituationalInputs, InwardBodyRotatesWithYaw) {
    // Same position, but yawed +90° about world-Z: the inward direction is now
    // off the LEFT wing → +body-y. Proves the quaternion rotation is applied.
    const gp_quat q(Eigen::AngleAxis<gp_scalar>(M_PI / 2, gp_vec3::UnitZ()));
    const gp_vec3 v = inwardBodyDirection(gp_vec3(10.0, 0.0, 0.0), q);
    EXPECT_NEAR(v.x(), 0.0, 1e-6);
    EXPECT_NEAR(v.y(), 1.0, 1e-6);
    EXPECT_NEAR(v.z(), 0.0, 1e-6);
}

TEST(SituationalInputs, InwardBodyInvertedKnifeEdgeIsOutOfPlane) {
    // The case a planar sin/cos heading CANNOT represent: craft north of the
    // axis (world inward = -y), rolled 90° about body-x (knife edge). The
    // inward direction lands entirely on body-Z — no body xy-plane component
    // at all. A heading angle in the image/body plane would report (0,-1) and
    // silently drop the out-of-plane truth.
    const gp_quat q(Eigen::AngleAxis<gp_scalar>(M_PI / 2, gp_vec3::UnitX()));
    const gp_vec3 v = inwardBodyDirection(gp_vec3(0.0, 10.0, 0.0), q);
    EXPECT_NEAR(v.x(), 0.0, 1e-6);
    EXPECT_NEAR(v.y(), 0.0, 1e-6);
    EXPECT_NEAR(v.z(), 1.0, 1e-6);
    EXPECT_NEAR(v.norm(), 1.0, 1e-6);  // still a unit vector
}

TEST(SituationalInputs, InwardBodyOnAxisIsZero) {
    // Chase directly on the cylinder axis (hypot(x,y) < 1e-6): no defined
    // inward direction → Zero (never NaN from a divide-by-zero).
    const gp_vec3 v = inwardBodyDirection(gp_vec3(0.0, 0.0, 5.0),
                                          gp_quat::Identity());
    EXPECT_EQ(v.x(), 0.0);
    EXPECT_EQ(v.y(), 0.0);
    EXPECT_EQ(v.z(), 0.0);
}

// ============================================================
// (A) time_since_seen + held exit bearing
// ============================================================

TEST(SituationalInputs, TimeSinceSeenZeroWhenVisible) {
    SituationalAwarenessState sa;
    sa.update(kSeenCep, kSeenCep, kCepSentinelThreshold);
    TrackerInputs out{};
    sa.writeInputs(out);
    EXPECT_FLOAT_EQ(out.time_since_seen, 0.0f);
}

TEST(SituationalInputs, TimeSinceSeenIncrementsWhileBlind) {
    SituationalAwarenessState sa;
    const int kBlind = 8;
    for (int i = 0; i < kBlind; ++i) sa.update(kBlindCep, kBlindCep, kCepSentinelThreshold);
    TrackerInputs out{};
    sa.writeInputs(out);
    const float blindSec = kBlind * (SIM_TIME_STEP_MSEC / 1000.0f);
    EXPECT_FLOAT_EQ(out.time_since_seen, std::tanh(blindSec / kTimeSinceSeenScale_s));
    EXPECT_GT(out.time_since_seen, 0.0f);
}

TEST(SituationalInputs, TimeSinceSeenResetsOnResighting) {
    SituationalAwarenessState sa;
    for (int i = 0; i < 5; ++i) sa.update(kBlindCep, kBlindCep, kCepSentinelThreshold);
    // A single visible tick (CEP-sentinel crossing) zeroes the blind counter.
    sa.update(kSeenCep, kSeenCep, kCepSentinelThreshold);
    TrackerInputs out{};
    sa.writeInputs(out);
    EXPECT_FLOAT_EQ(out.time_since_seen, 0.0f);
}

// (038 US3 2026-07-05: exit_dir sin/cos removed — the SA state is now just the
//  blind counter feeding time_since_seen; the held-exit-bearing tests are gone.)

TEST(SituationalInputs, ResetClearsBlindCounter) {
    SituationalAwarenessState sa;
    for (int i = 0; i < 8; ++i) sa.update(kBlindCep, kBlindCep, kCepSentinelThreshold);
    sa.reset();
    TrackerInputs out{};
    sa.writeInputs(out);
    EXPECT_FLOAT_EQ(out.time_since_seen, 0.0f);
}

// A single-visibility CEP crossing must flip the counter; test both edges.
TEST(SituationalInputs, VisibilityUsesSentinelThresholdOnEitherBeacon) {
    SituationalAwarenessState sa;
    // Right beacon alone visible ⇒ still "seen".
    for (int i = 0; i < 3; ++i) sa.update(kBlindCep, kSeenCep, kCepSentinelThreshold);
    TrackerInputs out{};
    sa.writeInputs(out);
    EXPECT_FLOAT_EQ(out.time_since_seen, 0.0f);
    // Both at exactly the threshold ⇒ NOT seen (strict <).
    sa.update(kCepSentinelThreshold, kCepSentinelThreshold, kCepSentinelThreshold);
    sa.writeInputs(out);
    EXPECT_GT(out.time_since_seen, 0.0f);
}

// ============================================================
// Determinism — the FR-030 bitwise-gate invariant at the unit level
// ============================================================

TEST(SituationalInputs, IdenticalSequenceProducesIdenticalTrace) {
    // Two independently-reset states driven through the SAME mixed
    // visible/blind sequence must produce byte-identical input traces.
    SituationalAwarenessState a, b;
    a.reset();
    b.reset();

    struct Obs { float lcep, rcep; };
    const Obs seq[] = {
        {kSeenCep, kSeenCep}, {kBlindCep, kBlindCep}, {kBlindCep, kBlindCep},
        {kSeenCep, kSeenCep}, {kBlindCep, kSeenCep},  // right-only visible
        {kBlindCep, kBlindCep},
    };
    for (const Obs& o : seq) {
        a.update(o.lcep, o.rcep, kCepSentinelThreshold);
        b.update(o.lcep, o.rcep, kCepSentinelThreshold);
        TrackerInputs ta{}, tb{};
        a.writeInputs(ta);
        b.writeInputs(tb);
        EXPECT_EQ(std::memcmp(&ta, &tb, sizeof(TrackerInputs)), 0);
    }
}

// ============================================================
// Integration — gather_tracker_inputs (A)+(B) and gather_pathgen_inputs (B)
// ============================================================

TEST(SituationalInputs, GatherTrackerWritesBAndA) {
    const gp_quat q(Eigen::AngleAxis<gp_scalar>(M_PI / 2, gp_vec3::UnitZ()));
    AircraftState chase = makeChase(gp_vec3(10.0, 0.0, 0.0), q);
    TrackerHistoryWindow history =
        uniformHistory(0.6f, 0.8f, kSeenCep, 0.6f, 0.8f, kSeenCep);

    SituationalAwarenessState sa;
    sa.update(history.left_cep[5], history.right_cep[5], kCepSentinelThreshold);

    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, kCepSentinelThreshold, sa, out);

    // (B) inward_body matches the standalone helper (yaw case → +body-y).
    EXPECT_NEAR(out.common.inward_body_x, 0.0f, 1e-5);
    EXPECT_NEAR(out.common.inward_body_y, 1.0f, 1e-5);
    EXPECT_NEAR(out.common.inward_body_z, 0.0f, 1e-5);
    // (A) visible now → time_since_seen 0.
    EXPECT_FLOAT_EQ(out.time_since_seen, 0.0f);
}

TEST(SituationalInputs, GatherPathgenWritesBOnly) {
    // M1 gets (B): dist_to_boundary + inward_body. NNInputs carries no (A)
    // fields at all (target-lost cues are tracker-only) — the struct simply
    // has no time_since_seen / exit_dir members, which is the "M1 gets B not
    // A" contract at the type level.
    AircraftState chase = makeChase(gp_vec3(10.0, 0.0, 0.0), gp_quat::Identity());
    Path dummy{};
    SinglePathProvider provider(dummy);
    NNInputs in{};
    gather_pathgen_inputs(provider, chase, FlightArena{}, in);

    // Level, east of axis → inward along -body-x.
    EXPECT_NEAR(in.common.inward_body_x, -1.0f, 1e-5);
    EXPECT_NEAR(in.common.inward_body_y,  0.0f, 1e-5);
    EXPECT_NEAR(in.common.inward_body_z,  0.0f, 1e-5);
    // dist_to_boundary is the tanh-saturated ray distance ⇒ (0, 1).
    EXPECT_GE(in.common.dist_to_boundary, 0.0f);
    EXPECT_LT(in.common.dist_to_boundary, 1.0f);
}

}  // namespace

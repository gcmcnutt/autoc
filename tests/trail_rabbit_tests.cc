// 030 M7b — Trail rabbit contract tests (T039 + Session 2026-05-07 Q1).
//
// Math identity: rabbit = target_pos − velocity_unit × trail_distance.
// Degenerate fallback: |velocity| < 1e-3 ⇒ rabbit = target_pos.
//
// FR-008a nose-direction fallback intentionally NOT tested here —
// deferred to 031 backlog per Session 2026-05-07 Q1. v1 fallback is
// rabbit ≡ target.position when velocity is degenerate.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/source_trajectory.h"
#include "autoc/eval/trail_rabbit.h"

using autoc::eval::computeTrailRabbit;
using autoc::eval::kDefaultTrailDistance;

namespace {

SourceTickSample makeSample(const gp_vec3& pos, const gp_vec3& vel) {
    SourceTickSample s;
    s.simTimeMsec = 0.0;
    s.position = pos;
    s.orientation = gp_quat::Identity();
    s.velocity = vel;
    return s;
}

}  // namespace

TEST(TrailRabbit, ForwardFlyingTargetRabbitIsBehindByTrailDistance) {
    // Target at origin moving in world +X at 10 m/s. Trail rabbit
    // should be 3.048 m behind in world -X = (-3.048, 0, 0).
    SourceTickSample s = makeSample(gp_vec3(0, 0, 0), gp_vec3(10, 0, 0));
    gp_vec3 rabbit = computeTrailRabbit(s);
    EXPECT_NEAR(rabbit.x(), -3.048f, 1e-5f);
    EXPECT_NEAR(rabbit.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(rabbit.z(), 0.0f, 1e-5f);
}

TEST(TrailRabbit, DiagonalVelocityRabbitOnSameLine) {
    // Target at (5, 5, 5) moving in (1, 1, 0) direction at sqrt(2) m/s.
    // Velocity unit = (1/sqrt(2), 1/sqrt(2), 0). Rabbit = target - 3.048 ·
    // velocity_unit = (5 - 3.048/sqrt(2), 5 - 3.048/sqrt(2), 5).
    SourceTickSample s = makeSample(gp_vec3(5, 5, 5), gp_vec3(1, 1, 0));
    gp_vec3 rabbit = computeTrailRabbit(s);
    const float offset = 3.048f / std::sqrt(2.0f);
    EXPECT_NEAR(rabbit.x(), 5.0f - offset, 1e-4f);
    EXPECT_NEAR(rabbit.y(), 5.0f - offset, 1e-4f);
    EXPECT_NEAR(rabbit.z(), 5.0f, 1e-5f);
}

TEST(TrailRabbit, ConfigurableTrailDistance) {
    // Custom trail distance.
    SourceTickSample s = makeSample(gp_vec3(0, 0, 0), gp_vec3(20, 0, 0));
    gp_vec3 rabbit = computeTrailRabbit(s, /*trail_distance=*/10.0f);
    EXPECT_NEAR(rabbit.x(), -10.0f, 1e-5f);
}

TEST(TrailRabbit, ZeroVelocityCollapsesRabbitToTarget) {
    // Degenerate: |velocity| = 0. Rabbit = target_pos.
    SourceTickSample s = makeSample(gp_vec3(7.5f, -2.0f, 1.3f),
                                     gp_vec3(0, 0, 0));
    gp_vec3 rabbit = computeTrailRabbit(s);
    EXPECT_FLOAT_EQ(rabbit.x(), 7.5f);
    EXPECT_FLOAT_EQ(rabbit.y(), -2.0f);
    EXPECT_FLOAT_EQ(rabbit.z(), 1.3f);
}

TEST(TrailRabbit, NearZeroVelocityHitsThreshold) {
    // |velocity| < 1e-3 (just below threshold) → degenerate fallback.
    SourceTickSample s = makeSample(gp_vec3(1, 2, 3),
                                     gp_vec3(5e-4f, 0, 0));
    gp_vec3 rabbit = computeTrailRabbit(s);
    // |velocity| = 5e-4 < 1e-3 → degenerate; rabbit = target.position.
    EXPECT_FLOAT_EQ(rabbit.x(), 1.0f);
    EXPECT_FLOAT_EQ(rabbit.y(), 2.0f);
    EXPECT_FLOAT_EQ(rabbit.z(), 3.0f);
}

TEST(TrailRabbit, DescendingTargetRabbitOffsetUpward) {
    // Target at (0, 0, 0) descending in NED frame (+z = down) at
    // (0, 0, 5). Rabbit is "behind" the velocity vector → rabbit
    // is upward (negative z). Magnitude = 3.048.
    SourceTickSample s = makeSample(gp_vec3(0, 0, 0), gp_vec3(0, 0, 5));
    gp_vec3 rabbit = computeTrailRabbit(s);
    EXPECT_NEAR(rabbit.z(), -3.048f, 1e-5f);
    EXPECT_NEAR(rabbit.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(rabbit.y(), 0.0f, 1e-5f);
}

TEST(TrailRabbit, DeterministicPureFunction) {
    // Same input → same output, no PRNG / clock / history.
    SourceTickSample s = makeSample(gp_vec3(2, 3, 4), gp_vec3(1, 0, 0));
    gp_vec3 r1 = computeTrailRabbit(s);
    gp_vec3 r2 = computeTrailRabbit(s);
    EXPECT_FLOAT_EQ(r1.x(), r2.x());
    EXPECT_FLOAT_EQ(r1.y(), r2.y());
    EXPECT_FLOAT_EQ(r1.z(), r2.z());
}

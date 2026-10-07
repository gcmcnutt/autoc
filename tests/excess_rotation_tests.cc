// 043 T102 — contract tests for the excess-rotation measure.
//
// Pinned properties:
//   1. A straight path demands nothing: all craft rotation is excess.
//   2. A path that turns demands exactly its direction change; a craft that
//      rotates by the same amount scores ~zero excess.
//   3. A hard turn is not penalised (large demand, large rotation, ~zero excess)
//      while wandering on a straight line is — the operator's two sentences.
//   4. A pegged (constant, non-zero) rotation is NOT "smooth" — 015's Δ=0
//      exploit does not exist for this measure.
//   5. dt and ‖ω‖ are used as stated (cadence-consistent).
#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/excess_rotation.h"

using autoc::eval::ExcessRotationAccum;

namespace {
constexpr double kDt = 0.05;  // 20 Hz
gp_vec3 unit(double x, double y, double z) {
  gp_vec3 v(static_cast<gp_scalar>(x), static_cast<gp_scalar>(y), static_cast<gp_scalar>(z));
  return v / v.norm();
}
}  // namespace

TEST(ExcessRotation, StraightPathDemandsNothingSoAllRotationIsExcess) {
  ExcessRotationAccum a;
  const gp_vec3 t = unit(1, 0, 0);
  const gp_vec3 w(0.0f, 0.5f, 0.0f);  // 0.5 rad/s pitch rate, constant
  for (int i = 0; i < 100; ++i) a.step(t, w, kDt);
  EXPECT_NEAR(a.path_rotation, 0.0, 1e-9);
  EXPECT_NEAR(a.craft_rotation, 0.5 * kDt * 100, 1e-9);
  EXPECT_NEAR(a.excess(), 0.5 * kDt * 100, 1e-9);
}

TEST(ExcessRotation, TurningPathDemandsItsDirectionChange) {
  ExcessRotationAccum a;
  // Path turns 90 degrees over 100 ticks in the horizontal plane; craft yaws at
  // exactly the matching rate. Excess should be ~0.
  const double total = M_PI / 2.0;
  const double rate = total / (99 * kDt);
  for (int i = 0; i < 100; ++i) {
    const double ang = total * i / 99.0;
    a.step(unit(std::cos(ang), std::sin(ang), 0.0), gp_vec3(0.0f, 0.0f, static_cast<gp_scalar>(rate)), kDt);
  }
  // gp_vec3 is float: acos(dot) near 1 carries ~1e-5 rad per step (float dot error
  // ÷ sin(step angle)); ~1e-3 rad over 100 steps. Negligible against the tens of rad
  // a scenario accumulates (t3 median 47.8), so the contract is pinned at 5e-3.
  EXPECT_NEAR(a.path_rotation, total, 5e-3);
  // craft integrates over 100 ticks of dt at the 99-interval rate: 100/99 of total
  EXPECT_NEAR(a.craft_rotation, total * 100.0 / 99.0, 1e-6);
  EXPECT_NEAR(a.excess(), total / 99.0, 5e-3);  // one tick's worth — the integration edge, not wandering
}

TEST(ExcessRotation, HardTurnIsFreeWanderingOnALineIsNot) {
  // Hard turn: 180 degrees in 2 s, craft matches it.
  ExcessRotationAccum hard;
  const double tot = M_PI; const int n = 40; const double rate = tot / ((n - 1) * kDt);
  for (int i = 0; i < n; ++i) {
    const double ang = tot * i / (n - 1.0);
    hard.step(unit(std::cos(ang), std::sin(ang), 0.0), gp_vec3(0.0f, 0.0f, static_cast<gp_scalar>(rate)), kDt);
  }
  // Wander: straight path, craft oscillates ±1 rad/s in pitch for the same 2 s.
  ExcessRotationAccum wander;
  for (int i = 0; i < n; ++i) {
    const float q = (i % 2 == 0) ? 1.0f : -1.0f;
    wander.step(unit(1, 0, 0), gp_vec3(0.0f, q, 0.0f), kDt);
  }
  EXPECT_LT(hard.excess(), 0.1);                 // ~one tick of edge only
  EXPECT_NEAR(wander.excess(), 1.0 * kDt * n, 1e-9);  // 2.0 rad of pure excess
  EXPECT_GT(wander.excess(), 10.0 * hard.excess());
}

TEST(ExcessRotation, PeggedRotationIsNotSmooth) {
  // 015's exploit: a constant command has Δu = 0 and scored "perfectly smooth".
  // Here a constant NON-ZERO rotation accumulates linearly — it is not free.
  ExcessRotationAccum a;
  for (int i = 0; i < 200; ++i) a.step(unit(1, 0, 0), gp_vec3(2.0f, 0.0f, 0.0f), kDt);
  EXPECT_NEAR(a.excess(), 2.0 * kDt * 200, 1e-9);
  EXPECT_GT(a.excess(), 10.0);
}

TEST(ExcessRotation, UsesVectorNormAndDt) {
  ExcessRotationAccum a;
  a.step(unit(1, 0, 0), gp_vec3(3.0f, 4.0f, 0.0f), 0.1);  // ‖ω‖ = 5
  EXPECT_NEAR(a.craft_rotation, 0.5, 1e-9);
  EXPECT_NEAR(a.path_rotation, 0.0, 1e-9);  // first tick has no previous tangent
}

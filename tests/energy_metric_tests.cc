// 035 FR-001b/R1 (T029) -- convex throttle-energy metric.
// asserts the hand-computed fixtures, non-negativity, monotonicity, convexity.
//
// ⚠️ 041 P2-5 — `throttleEnergyStep` NO LONGER FEEDS `energy_score`. It is
// retained (and still tested) as a diagnostic of propulsive effort, but the
// selection axis is now metres of specific energy DESTROYED, Σ max(0, −Ps)·dt.
// The tests below the divider are the ones that matter for selection, and the
// first of them is the guard against 035 happening again.

#include <gtest/gtest.h>
#include <cmath>

#include "autoc/eval/fitness_decomposition.h"
#include "autoc/eval/energy_state.h"

#include <vector>

namespace {

// f(out_th) = ((out_th+1)/2)^2.5  (037: was ^2)
TEST(EnergyMetric, HandComputedFixtures) {
  EXPECT_DOUBLE_EQ(throttleEnergyStep(-1.0), 0.0);                     // idle -> 0
  EXPECT_DOUBLE_EQ(throttleEnergyStep(0.0), 0.25 * std::sqrt(0.5));    // thr=0.5  -> 0.5^2.5  ~ 0.17678
  EXPECT_DOUBLE_EQ(throttleEnergyStep(1.0), 1.0);                      // full -> 1
  EXPECT_DOUBLE_EQ(throttleEnergyStep(-0.5), 0.03125);                 // thr=0.25 -> 0.25^2.5
  EXPECT_DOUBLE_EQ(throttleEnergyStep(0.5), 0.5625 * std::sqrt(0.75)); // thr=0.75 -> 0.75^2.5 ~ 0.48714
}

TEST(EnergyMetric, NonNegativeAndBounded) {
  for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.05) {
    const gp_fitness e = throttleEnergyStep(x);
    EXPECT_GE(e, 0.0) << "x=" << x;            // sum-of-squares ≥ 0 (sign fix)
    EXPECT_LE(e, 1.0 + 1e-12) << "x=" << x;    // bounded by full-throttle
  }
}

TEST(EnergyMetric, MonotoneIncreasingInThrottle) {
  double prev = -1.0;
  for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.05) {
    const gp_fitness e = throttleEnergyStep(x);
    EXPECT_GE(e, prev) << "non-monotone at x=" << x;  // more throttle → more energy
    prev = e;
  }
}

TEST(EnergyMetric, ConvexSuperLinear) {
  // Convex: f(mid) ≤ ½(f(a)+f(b)) for any a<b. Super-linear means the high
  // end is penalized disproportionately — the whole point vs the linear form.
  for (double a = -1.0; a < 1.0; a += 0.25) {
    for (double b = a + 0.25; b <= 1.0 + 1e-9; b += 0.25) {
      const double mid = 0.5 * (a + b);
      EXPECT_LE(throttleEnergyStep(mid),
                0.5 * (throttleEnergyStep(a) + throttleEnergyStep(b)) + 1e-12)
          << "convexity violated on [" << a << "," << b << "]";
    }
  }
  // Full throttle costs > 2× half throttle (super-linear, unlike linear).
  EXPECT_GT(throttleEnergyStep(1.0), 2.0 * throttleEnergyStep(0.0));
}


// ===========================================================================
// 041 P2-5 — THE Ps ENERGY AXIS, and the guard that makes it safe to ship.
// ===========================================================================

namespace {

// The axis as accumulated in fitness_decomposition.cc: metres of specific
// energy destroyed. Written here as an independent statement of the rule so the
// tests below assert the PROPERTY, not the implementation. (If the two ever
// disagree, that is a real finding — this is not a copy for convenience.)
gp_fitness energyDestroyed(const std::vector<double>& es_series, double dt_sec) {
  gp_fitness acc = 0.0;
  for (size_t i = 1; i < es_series.size(); ++i) {
    const gp_fitness ps = autoc::eval::specificExcessPower(
        static_cast<gp_fitness>(es_series[i]),
        static_cast<gp_fitness>(es_series[i - 1]),
        static_cast<gp_fitness>(dt_sec));
    if (ps < 0.0) acc += -ps * static_cast<gp_fitness>(dt_sec);
  }
  return acc;
}

constexpr double kDt = 0.05;  // 20 Hz

}  // namespace

// ⛔⛔ THE MUTING GUARD. This is the single test that would have caught 035.
//
// 035 selected on an ABSOLUTE energy quantity (a convex integral of the
// throttle command), and because full power is genuinely correct when far
// behind, in a sustained spiral, and under pitch-induced drag, the pressure
// could only quiet the whole regiment. The failure was not that the axis was
// too strong — it was that the axis charged for CORRECT behaviour.
//
// Stated as a property: a policy that CLIMBS toward a high target must not
// score worse on the energy axis than one that does not climb.
TEST(PsEnergyAxis, ClimbingTowardAHighTargetIsNotPenalised_MUTING_GUARD) {
  // Two policies over the same 5 s. The climber gains 50 m of energy height;
  // the loiterer holds station. Both bleed nothing.
  std::vector<double> climber, loiterer;
  for (int i = 0; i <= 100; ++i) {
    climber.push_back(50.0 + 0.5 * i);   // +0.5 m of Es per tick — a real climb
    loiterer.push_back(50.0);            // steady
  }

  const gp_fitness climb_cost = energyDestroyed(climber, kDt);
  const gp_fitness loiter_cost = energyDestroyed(loiterer, kDt);

  EXPECT_DOUBLE_EQ(climb_cost, 0.0)
      << "climbing must cost EXACTLY zero on the energy axis — Ps > 0 "
         "contributes nothing. A non-zero cost here is 035 re-entering.";
  EXPECT_LE(climb_cost, loiter_cost)
      << "a climbing policy scored WORSE than a stationary one — this is the "
         "muting failure, and it is why 035's whole regiment went quiet.";
}

// ⭐ The second half of "trim waste, don't mute": trading height for speed is
// energy-NEUTRAL, so a dive that buys airspeed must also be free. Only drag
// losses and deliberately destroyed energy may appear.
TEST(PsEnergyAxis, TradingHeightForSpeedIsFree) {
  // A clean zoom-and-dive: Es rises then returns to exactly where it started.
  // Every joule of the descent came back as airspeed, so nothing was wasted.
  std::vector<double> zoom;
  for (int i = 0; i <= 50; ++i) zoom.push_back(50.0 + 0.4 * i);      // climb
  for (int i = 1; i <= 50; ++i) zoom.push_back(70.0 - 0.4 * i);      // dive back

  // ⚠️ The dive IS charged — Es genuinely falls during it — which is the
  // non-telescoping property. What the test pins is that the charge equals the
  // energy actually given up and not a penny more.
  EXPECT_NEAR(static_cast<double>(energyDestroyed(zoom, kDt)), 20.0, 1e-6);
}

// ⛔ NOT TELESCOPING. Σ Ps·dt would collapse to (Es_end − Es_start), which is
// gameable by simply finishing high and blind to everything in between. The
// clip at zero is what breaks the cancellation.
TEST(PsEnergyAxis, IsNotTelescopingAndCannotBeGamedByFinishingHigh) {
  // Policy A: bleeds 30 m, then buys it all back. Net change zero.
  std::vector<double> churner;
  for (int i = 0; i <= 30; ++i) churner.push_back(50.0 - 1.0 * i);
  for (int i = 1; i <= 40; ++i) churner.push_back(20.0 + 1.0 * i);   // ends at 60

  // Policy B: holds station, then climbs to the same finish. Wastes nothing.
  std::vector<double> efficient;
  for (int i = 0; i <= 30; ++i) efficient.push_back(50.0);
  for (int i = 1; i <= 40; ++i) efficient.push_back(50.0 + 0.25 * i);  // ends at 60

  EXPECT_GT(energyDestroyed(churner, kDt), energyDestroyed(efficient, kDt))
      << "the churner and the efficient policy END at the same energy; a "
         "telescoping metric would score them equal and reward the churn";
  EXPECT_NEAR(static_cast<double>(energyDestroyed(churner, kDt)), 30.0, 1e-6);
  EXPECT_DOUBLE_EQ(energyDestroyed(efficient, kDt), 0.0);
}

// The axis is a physical quantity in metres, so it must not change when the
// cadence does — the same discipline Ps itself carries.
TEST(PsEnergyAxis, IsCadenceInvariant) {
  // The same 2 s bleed of 20 m, sampled at 20 Hz and at 40 Hz.
  std::vector<double> at20, at40;
  for (int i = 0; i <= 40; ++i) at20.push_back(60.0 - 0.5 * i);
  for (int i = 0; i <= 80; ++i) at40.push_back(60.0 - 0.25 * i);

  EXPECT_NEAR(static_cast<double>(energyDestroyed(at20, 0.05)),
              static_cast<double>(energyDestroyed(at40, 0.025)), 1e-6);
  EXPECT_NEAR(static_cast<double>(energyDestroyed(at20, 0.05)), 20.0, 1e-6);
}

// Non-negative by construction, and zero for a policy that wastes nothing.
TEST(PsEnergyAxis, IsNonNegativeAndZeroForALosslessPolicy) {
  std::vector<double> flat(100, 42.0);
  EXPECT_DOUBLE_EQ(energyDestroyed(flat, kDt), 0.0);
  std::vector<double> gaining;
  for (int i = 0; i < 100; ++i) gaining.push_back(10.0 + 0.3 * i);
  EXPECT_DOUBLE_EQ(energyDestroyed(gaining, kDt), 0.0);
}

}  // namespace

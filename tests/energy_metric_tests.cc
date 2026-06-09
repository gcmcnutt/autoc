// 035 FR-001b/R1 (T029) — convex throttle-energy metric.
// energy_score accumulates throttleEnergyStep(out_th) = ((out_th+1)/2)² per tick.
// out_th∈[-1,1] (tanh NN output) maps to a [0,1] throttle fraction, squared.
// Asserts the hand-computed fixtures, non-negativity, monotonicity, convexity.

#include <gtest/gtest.h>

#include "autoc/eval/fitness_decomposition.h"

namespace {

// f(out_th) = ((out_th+1)/2)^2
TEST(EnergyMetric, HandComputedFixtures) {
  EXPECT_DOUBLE_EQ(throttleEnergyStep(-1.0), 0.0);    // idle → 0
  EXPECT_DOUBLE_EQ(throttleEnergyStep(0.0), 0.25);    // mid (thr=0.5) → 0.25
  EXPECT_DOUBLE_EQ(throttleEnergyStep(1.0), 1.0);     // full → 1
  EXPECT_DOUBLE_EQ(throttleEnergyStep(-0.5), 0.0625); // thr=0.25 → 0.0625
  EXPECT_DOUBLE_EQ(throttleEnergyStep(0.5), 0.5625);  // thr=0.75 → 0.5625
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

}  // namespace

// 037 T014 -- cadence-invariance tests (fills the T003 skeleton).
//
// Contract (contracts/cadence-config.md "Tests"): for a fixed synthetic
// trajectory, the per-tick stability and energy accumulators -- rescaled by
// cadenceTickScale (037 T018) -- produce the SAME physical total at 10 Hz vs
// the projected 20 Hz (within FP tolerance). A raw sum-per-tick accumulator
// roughly doubles when the tick count doubles -- that regression is exactly
// what this test catches.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/aircraft_state.h"          // cadenceTickScale / kCadenceTickScale
#include "autoc/eval/fitness_decomposition.h"   // throttleEnergyStep

namespace {

// Anchor the master cadence constant so this TU fails to compile if the
// symbol is renamed/removed -- cheap coupling to the thing under test.
static_assert(SIM_TIME_STEP_MSEC > 0, "control cadence must be positive");

// Replicates the T018 accumulation shape (fitness_decomposition.cc) at a
// given control interval over a fixed 2-second synthetic segment with
// constant NN outputs. Constant outputs make the time-integral exact, so
// rate-invariance is a strict equality up to FP associativity.
struct Totals {
  double stability;
  double energy;
};

Totals accumulate(unsigned long intervalMsec, float out_pt, float out_rl,
                  float out_th) {
  const int ticks = static_cast<int>(2000 / intervalMsec);
  const double scale = cadenceTickScale(intervalMsec);
  Totals t{0.0, 0.0};
  for (int i = 0; i < ticks; ++i) {
    const gp_fitness abs_pt = std::abs(static_cast<gp_fitness>(out_pt));
    const gp_fitness abs_rl = std::abs(static_cast<gp_fitness>(out_rl));
    t.stability += ((abs_pt - 1.0) + (abs_rl - 1.0)) * scale;
    t.energy += throttleEnergyStep(static_cast<gp_fitness>(out_th)) * scale;
  }
  return t;
}

TEST(CadenceInvariance, StabilityEnergyTotalsRateInvariant) {
  const Totals at10 = accumulate(100, 0.4f, -0.7f, 0.6f);
  const Totals at20 = accumulate(50, 0.4f, -0.7f, 0.6f);
  const Totals at50 = accumulate(20, 0.4f, -0.7f, 0.6f);

  EXPECT_NEAR(at10.stability, at20.stability, 1e-9);
  EXPECT_NEAR(at10.energy, at20.energy, 1e-9);
  EXPECT_NEAR(at10.stability, at50.stability, 1e-9);
  EXPECT_NEAR(at10.energy, at50.energy, 1e-9);
}

TEST(CadenceInvariance, TickScaleAnchorsToHistoricalTenHz) {
  // x1.0 at 100 ms is the bitwise regression-gate invariant (multiplying a
  // double by exactly 1.0 is an IEEE identity); 50 ms halves the per-tick
  // weight so totals stay on the historical t6 scale.
  EXPECT_EQ(1.0, cadenceTickScale(100));
  EXPECT_EQ(0.5, cadenceTickScale(50));
  EXPECT_EQ(kCadenceTickScale, cadenceTickScale(SIM_TIME_STEP_MSEC));
}

}  // namespace

// 037 T003 -- tick-rescale test scaffolding (skeleton).
//
// Purpose (filled in by 037 T015):
//   1. closing_rate for a known dDist is correct at two different control
//      intervals -- i.e. the rate uses the ACTUAL dt, never a hardcoded 0.1f
//      (037 T019 replaces the divisor in src/nn/evaluator.cc).
//   2. the engage-delay tick count equals ceil(EngageDelayMs / controlIntervalMsec)
//      at 10 / 20 / 50 Hz -- duration rate-independent (037 T020).
//
// This is a SKELETON (T003): registers the TU + CMake target. Real assertions
// land in T015, which removes the GTEST_SKIP. Until then the case is SKIPPED
// (not failed) so `run_autoc_tests` stays green.
//
// Per specs/037-20hz-control-loop/contracts/cadence-config.md "Tick-rescale".

#include <gtest/gtest.h>
#include <cmath>

namespace {

// The rate-independent engage-delay formula under test (T020 will move the
// real implementation into the stepper; this documents the contract).
constexpr int engageDelayTicks(int engageDelayMs, int controlIntervalMsec) {
  return (engageDelayMs + controlIntervalMsec - 1) / controlIntervalMsec;  // ceil
}

TEST(TickRescale, ClosingRateUsesActualDt) {
  GTEST_SKIP() << "037 T015: closing_rate correct at two intervals -- pending "
                  "T019 (replace hardcoded /0.1f with dt).";
}

TEST(TickRescale, EngageDelayTicksRateIndependent) {
  GTEST_SKIP() << "037 T015: assert engageDelayTicks(750, {100,50,20}) -- "
                  "pending T020 engage-delay implementation.";
  // Sketch of the intended assertions (kept compiled-but-skipped):
  EXPECT_EQ(engageDelayTicks(750, 100), 8);
  EXPECT_EQ(engageDelayTicks(750, 50), 15);
  EXPECT_EQ(engageDelayTicks(750, 20), 38);
}

}  // namespace

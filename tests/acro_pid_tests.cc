// ACRO PID math tests — exercise the shared helper used by inputdev_autoc.
// See spec 026 task T050. Verifies FF/P/I split, anti-windup, clamping,
// and reset behavior.

#include <gtest/gtest.h>
#include "autoc/eval/acro_pid.h"

namespace {

// 021-era empirical gains used by the live PID (also referenced in
// crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h).
constexpr AcroPidGains kRollGains{50.0, 40.0, 15.0, 350.0};
constexpr double kIntegLimit = 10.0;

}  // namespace

TEST(AcroPid, ZeroErrorZeroIntegralProducesZeroOutput) {
  AcroPidResult r = acroPidStep(kRollGains,
      {0.0, 0.0, 0.05}, 0.0, kIntegLimit);
  EXPECT_DOUBLE_EQ(r.error, 0.0);
  EXPECT_DOUBLE_EQ(r.newIntegral, 0.0);
  EXPECT_DOUBLE_EQ(r.ffTerm, 0.0);
  EXPECT_DOUBLE_EQ(r.pTerm, 0.0);
  EXPECT_DOUBLE_EQ(r.iTerm, 0.0);
  EXPECT_DOUBLE_EQ(r.output, 0.0);
  EXPECT_DOUBLE_EQ(r.clamped, 0.0);
  EXPECT_FALSE(r.saturated);
}

TEST(AcroPid, FeedforwardOnlyAtMatchedRate) {
  // desired = measured ⇒ error = 0, integrator stays at 0, only FF term fires.
  // FF = 50 * 3.0 / 350 = 0.4286
  AcroPidResult r = acroPidStep(kRollGains,
      {3.0, 3.0, 0.05}, 0.0, kIntegLimit);
  EXPECT_NEAR(r.ffTerm,   0.4285714285714286, 1e-12);
  EXPECT_DOUBLE_EQ(r.pTerm, 0.0);
  EXPECT_DOUBLE_EQ(r.iTerm, 0.0);
  EXPECT_NEAR(r.output, 0.4285714285714286, 1e-12);
  EXPECT_FALSE(r.saturated);
}

TEST(AcroPid, StepResponseHandComputed) {
  // desired = 5 rad/s, measured = 0, dt = 0.05 s, prev integral = 0.
  // error = 5; new_integ = 5 * 0.05 = 0.25
  // FF = 50 * 5 / 350     = 0.7142857142857143
  // P  = 40 * 5 / 350     = 0.5714285714285714
  // I  = 15 * 0.25 / 350  = 0.0107142857142857
  // out = 1.2964285714285715 → clamped to 1.0, saturated true
  AcroPidResult r = acroPidStep(kRollGains,
      {5.0, 0.0, 0.05}, 0.0, kIntegLimit);
  EXPECT_DOUBLE_EQ(r.error, 5.0);
  EXPECT_DOUBLE_EQ(r.newIntegral, 0.25);
  EXPECT_NEAR(r.ffTerm, 0.7142857142857143, 1e-12);
  EXPECT_NEAR(r.pTerm,  0.5714285714285714, 1e-12);
  EXPECT_NEAR(r.iTerm,  0.0107142857142857, 1e-12);
  EXPECT_NEAR(r.output, 1.2964285714285715, 1e-12);
  EXPECT_DOUBLE_EQ(r.clamped, 1.0);
  EXPECT_TRUE(r.saturated);
}

TEST(AcroPid, IntegratorAccumulatesAcrossTicks) {
  // Hold a constant 1 rad/s error for 4 ticks of 0.05 s; integrator should
  // climb 0.05 → 0.10 → 0.15 → 0.20 (well below limit), and I term scales.
  double integ = 0.0;
  AcroPidResult r;
  for (int i = 1; i <= 4; ++i) {
    r = acroPidStep(kRollGains, {1.0, 0.0, 0.05}, integ, kIntegLimit);
    integ = r.newIntegral;
    EXPECT_NEAR(integ, 0.05 * i, 1e-12) << "iter " << i;
  }
  // Final iteration at integ = 0.20:
  // I = 15 * 0.20 / 350 = 0.00857...
  EXPECT_NEAR(r.iTerm, 0.00857142857142857, 1e-12);
}

TEST(AcroPid, IntegratorClampedAtLimit) {
  // Massive sustained error should pin the integrator at +limit, not blow up.
  double integ = 0.0;
  AcroPidResult r;
  for (int i = 0; i < 1000; ++i) {
    r = acroPidStep(kRollGains, {100.0, 0.0, 0.05}, integ, kIntegLimit);
    integ = r.newIntegral;
  }
  EXPECT_DOUBLE_EQ(integ, kIntegLimit);  // pinned
  // Negative-direction sustained error pins at -limit
  integ = 0.0;
  for (int i = 0; i < 1000; ++i) {
    r = acroPidStep(kRollGains, {-100.0, 0.0, 0.05}, integ, kIntegLimit);
    integ = r.newIntegral;
  }
  EXPECT_DOUBLE_EQ(integ, -kIntegLimit);
}

TEST(AcroPid, IntegratorResetByPassingZeroPrev) {
  // Caller "resets" by passing 0 as prevIntegral (mirrors span-start reset).
  AcroPidResult primed = acroPidStep(kRollGains,
      {2.0, 0.0, 0.05}, 5.0, kIntegLimit);
  EXPECT_NEAR(primed.newIntegral, 5.0 + 2.0 * 0.05, 1e-12);

  AcroPidResult fresh = acroPidStep(kRollGains,
      {2.0, 0.0, 0.05}, 0.0, kIntegLimit);
  EXPECT_NEAR(fresh.newIntegral, 0.10, 1e-12);
  EXPECT_NE(primed.iTerm, fresh.iTerm);
}

TEST(AcroPid, NegativeErrorClampsToMinusOne) {
  // measured > desired ⇒ negative error → output drives towards -1.
  // desired = 0, measured = +5, prev integ = 0.
  // error = -5, new integ = -0.25
  // FF = 0, P = -0.5714..., I = -0.01071...
  // out = -0.58214... (no clamp); test a stronger case to hit clamp:
  AcroPidResult r = acroPidStep(kRollGains,
      {-5.0, 0.0, 0.05}, 0.0, kIntegLimit);
  EXPECT_DOUBLE_EQ(r.error, -5.0);
  EXPECT_DOUBLE_EQ(r.clamped, -1.0);
  EXPECT_TRUE(r.saturated);
}

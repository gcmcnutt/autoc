// 037 T015 -- tick-rescale tests (fills the T003 skeleton).
//
// Contract (contracts/cadence-config.md "Tick-rescale"):
//   1. closing_rate for a known dDist is correct -- the rate divides by the
//      ACTUAL history lag gap (100 ms by the R5 lag table), never an
//      implicit one-tick assumption (037 T019). The test is written against
//      physical values (m/s) so it holds at ANY compiled SIM_TIME_STEP_MSEC.
//   2. the engage-delay tick count equals ceil(EngageDelayMs /
//      controlIntervalMsec) at 10 / 20 / 50 Hz -- duration rate-independent
//      (037 T020, shared engageDelayTicks helper in aircraft_state.h).

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/eval/aircraft_state.h"  // AircraftState, engageDelayTicks, SIM_TIME_STEP_MSEC
#include "autoc/eval/sensor_lpf.h"     // 044 FcSensorFilterChain
#include "autoc/nn/evaluator.h"         // gather_pathgen_inputs
#include "autoc/nn/nn_inputs.h"         // NNInputs

namespace {

TEST(TickRescale, ClosingRateUsesActualLagDt) {
  AircraftState st{0,
                   20.0f,
                   gp_vec3(20.0f, 0.0f, 0.0f),
                   gp_quat::Identity(),
                   gp_vec3::Zero(),
                   0.0f,
                   0.0f,
                   0.0f,
                   0};

  // Linear approach at a known closing rate; fill the WHOLE history ring so
  // every lag slot holds real (not warm-up-clamped) samples.
  const double kClosingRate_mps = 20.0;
  const double stepSec = SIM_TIME_STEP_MSEC / 1000.0;
  const int n = AircraftState::HISTORY_SIZE + 4;
  for (int k = 0; k < n; ++k) {
    const double dist = 500.0 - kClosingRate_mps * stepSec * k;
    st.recordErrorHistory(gp_vec3::UnitX(), static_cast<gp_scalar>(dist),
                          static_cast<unsigned long>(k) * SIM_TIME_STEP_MSEC);
  }

  Path dummy{};
  SinglePathProvider provider(dummy);
  NNInputs in{};
  gather_pathgen_inputs(provider, st, autoc::eval::FlightArena{}, in);

  // Positive = approaching; must recover the physical rate regardless of the
  // compiled control interval (the lag pair spans 100 ms at every rate).
  //
  // 041 P2-8 — the slot is now scaled by kClosingRateScale_mps, so multiply it
  // back out before comparing. ⭐ Kept as physical-rate-times-scale rather than
  // hard-coding 1.25: this test exists to guard the LAG DT (that the pair spans
  // 100 ms at any cadence), and that property must stay readable independently
  // of whatever the normalization constant happens to be.
  EXPECT_NEAR(kClosingRate_mps, in.closing_rate * kClosingRateScale_mps, 1e-3);
}

TEST(TickRescale, EngageDelayTicksRateIndependent) {
  // 750 ms measured INAV handoff at 10 / 20 / 50 Hz.
  EXPECT_EQ(8, engageDelayTicks(750, 100));   // ceil(7.5)
  EXPECT_EQ(15, engageDelayTicks(750, 50));   // exact
  EXPECT_EQ(38, engageDelayTicks(750, 20));   // ceil(37.5)

  // Edges: zero delay = no window; sub-tick delay rounds up to one tick.
  EXPECT_EQ(0, engageDelayTicks(0, 100));
  EXPECT_EQ(1, engageDelayTicks(1, 100));
  EXPECT_EQ(1, engageDelayTicks(100, 100));
}

// ---------------------------------------------------------------------------
// 044 US2 (FR-008) — the flight controller's sensor filters, run at the FDM
// sub-step (5 ms) and READ at the 20 Hz decision tick, as the real chain is
// (filter at the FC rate, then sample over MSP). The policy reads their
// output; what it must see is their low-frequency group delay:
//   gyro  PT1 25 Hz            : rc = 1/(2*pi*25)          = 6.37 ms
//   accel biquad 15 Hz, Q 0.707: 2*zeta/wn = sqrt2/(2*pi*15) = 15.0 ms
// (14.7 ms in the 200 Hz discretisation). ⚠️ Not ~21 ms: 2/(2*pi*fc) is the
// critically-damped figure, which an earlier note used by mistake.
// ---------------------------------------------------------------------------
TEST(FcSensorFilters, GroupDelaysAtTheSubstep_044) {
  constexpr double kDt = 0.005;
  autoc::eval::FcSensorFilterChain chain(kDt);
  // A slow ramp on every axis: after settling, out(t) = in(t - delay).
  const double rate = 1.0;  // units per second
  double t = 0.0;
  for (int n = 0; n < 4000; ++n) {
    t = n * kDt;
    const gp_scalar v = static_cast<gp_scalar>(rate * t);
    chain.step(gp_vec3(v, v, v), gp_vec3(v, v, v));
  }
  for (int a = 0; a < 3; ++a) {
    const double gyroLagMs = 1000.0 * (rate * t - chain.gyro()[a]) / rate;
    const double accLagMs = 1000.0 * (rate * t - chain.accel()[a]) / rate;
    EXPECT_NEAR(gyroLagMs, 6.37, 0.2) << "axis " << a;
    EXPECT_NEAR(accLagMs, 15.0, 0.6) << "axis " << a;
  }
}

// The first sample seeds the state (a scenario starts in motion; ramping from
// zero would inject a transient the real sensor never has); reset re-seeds.
TEST(FcSensorFilters, FirstSampleSeedsAndResetReseeds_044) {
  autoc::eval::FcSensorFilterChain chain(0.005);
  EXPECT_FALSE(chain.seeded());
  chain.step(gp_vec3(1.0f, -2.0f, 3.0f), gp_vec3(0.1f, 0.2f, -1.0f));
  EXPECT_TRUE(chain.seeded());
  EXPECT_FLOAT_EQ(chain.gyro()[1], -2.0f);
  EXPECT_FLOAT_EQ(chain.accel()[2], -1.0f);
  chain.step(gp_vec3(1.0f, -2.0f, 3.0f), gp_vec3(0.1f, 0.2f, -1.0f));
  EXPECT_FLOAT_EQ(chain.gyro()[0], 1.0f);   // constant input stays exact
  chain.reset();
  EXPECT_FALSE(chain.seeded());
  chain.step(gp_vec3(5.0f, 5.0f, 5.0f), gp_vec3(2.0f, 2.0f, 2.0f));
  EXPECT_FLOAT_EQ(chain.gyro()[2], 5.0f);   // re-seeded, no memory of before
  EXPECT_FLOAT_EQ(chain.accel()[0], 2.0f);
}

// Read every 4th sub-step (the 20 Hz gather over 5 ms sub-steps): what is read
// is the FILTERED value at that sub-step, not the raw input. A step input read
// one tick later is still short of its final value on both channels.
TEST(FcSensorFilters, TwentyHzReadReturnsFilteredValue_044) {
  autoc::eval::FcSensorFilterChain chain(0.005);
  chain.step(gp_vec3::Zero(), gp_vec3::Zero());           // seed at 0
  for (int k = 0; k < 4; ++k) chain.step(gp_vec3(1.0f, 1.0f, 1.0f), gp_vec3(1.0f, 1.0f, 1.0f));
  const gp_scalar g = chain.gyro()[0], a = chain.accel()[0];  // the 20 Hz read
  EXPECT_GT(g, 0.4f); EXPECT_LT(g, 0.99f);                // PT1: 1-(1-k)^4 ≈ 0.96
  EXPECT_GT(a, 0.05f); EXPECT_LT(a, 0.9f);                // biquad lags further
  EXPECT_LT(a, g);
}

}  // namespace

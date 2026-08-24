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

}  // namespace

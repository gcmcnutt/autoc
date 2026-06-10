// 037 T003 -- cadence-invariance test scaffolding (skeleton).
//
// Purpose (filled in by 037 T014): for a fixed synthetic trajectory, the
// per-tick stability and energy accumulators must produce the SAME physical
// total at 10 Hz vs the projected faster cadence (within FP tolerance) once
// they are normalized per-second (037 T018). A raw sum-per-tick accumulator
// roughly doubles when the tick count doubles -- that regression is exactly
// what this test must catch.
//
// This is a SKELETON (T003): it registers the translation unit and the CMake
// target so the harness is in place. The real assertions land in T014, which
// will remove the GTEST_SKIP. Until then the case is SKIPPED (not failed) so
// `run_autoc_tests` stays green.
//
// Per specs/037-20hz-control-loop/contracts/cadence-config.md "Tests".

#include <gtest/gtest.h>

#include "autoc/eval/aircraft_state.h"  // SIM_TIME_STEP_MSEC (cadence master)

namespace {

// Anchor the master cadence constant so this TU fails to compile if the
// symbol is renamed/removed -- cheap coupling to the thing under test.
static_assert(SIM_TIME_STEP_MSEC > 0, "control cadence must be positive");

TEST(CadenceInvariance, StabilityEnergyTotalsRateInvariant) {
  GTEST_SKIP() << "037 T014: stability/energy per-second-normalized totals "
                  "equal at 10 Hz vs projected rate -- pending T018 rescale.";
}

}  // namespace

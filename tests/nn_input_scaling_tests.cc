// 041 P2-8 — input-scale parity guard.
//
// ⛔ THE BUG THIS EXISTS TO PREVENT is not a wrong number, it is a SILENT one.
// A unit sees Σ wᵢxᵢ, so an input's influence is weight × spread. When t5 ran,
// CLOSING_RATE carried stdev 7.53 and DIST_TO_BOUNDARY 0.036 — a 200× span —
// and the quiet slots were never selected on at all (every input's weight
// investment sat at ~1.0 from gen 1 to gen 475). Nothing failed; the network
// just could not hear half its inputs. A future slot added in raw units would
// reintroduce exactly that, and no existing test would notice.
#include <gtest/gtest.h>
#include "autoc/nn/nn_inputs.h"

namespace {

// Representative magnitudes measured over 131,493 t5 ticks (p95 of |x|).
TEST(InputScaling, EveryScaledSlotLandsInTheSameOrderOfMagnitude) {
    const float airspeed_p95 = 23.49f / kCruiseSpeed_mps;
    const float dist_p95     = 25.87f / kTargetDistScale_m;
    const float closing_p95  = 15.93f / kClosingRateScale_mps;
    const float gyro_p95     =  6.17f / kGyroScale_radps;

    // All four must land within [0.1, 3] — same order of magnitude as the
    // already-scaled slots (quat/inward ≈ 1, accel ≈ 0.3, Es ≈ 0.4).
    for (float v : {airspeed_p95, dist_p95, closing_p95, gyro_p95}) {
        EXPECT_GT(v, 0.1f);
        EXPECT_LT(v, 3.0f);
    }
}

// ⛔ The specific regression: raw units. If someone reverts a divisor, the p95
// blows past 3 and this fails loudly instead of costing another 500-gen bake.
TEST(InputScaling, RawUnitsWouldFailThisGuard) {
    EXPECT_GT(23.49f, 3.0f);   // raw airspeed m/s
    EXPECT_GT(25.87f, 3.0f);   // raw dist m
    EXPECT_GT(15.93f, 3.0f);   // raw closing rate m/s
    EXPECT_GT( 6.17f, 3.0f);   // raw gyro rad/s
}

// Scales must stay positive — a zero divisor is an inf into the first layer.
TEST(InputScaling, NoScaleIsZeroOrNegative) {
    EXPECT_GT(kCruiseSpeed_mps,     0.0f);
    EXPECT_GT(kTargetDistScale_m,   0.0f);
    EXPECT_GT(kClosingRateScale_mps,0.0f);
    EXPECT_GT(kGyroScale_radps,     0.0f);
    EXPECT_GT(kAccelScale_g,        0.0f);
    EXPECT_GT(kEnergyScale_m,       0.0f);
}

}  // namespace

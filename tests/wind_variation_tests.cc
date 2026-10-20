// 043 t4 (T087/T090) -- contract tests for the per-scenario wind envelope.
//
// These pin the three properties the bake's reproducibility rests on:
//   1. With every key at its default the realization IS the base wind, so an
//      ini without the keys (t3's as-run) reproduces bitwise.
//   2. The draws are always consumed in the same order and count, and they are
//      taken AFTER the first next() -- so the thermal/gust seed (drawnWindSeed)
//      is unchanged by the feature existing or being toggled.
//   3. The curriculum ramp behaves: scale 0 => base, scale 1 => full target.
#include <gtest/gtest.h>

#include "autoc/eval/wind_variation.h"
#include "autoc/util/scenario_prng.h"

using autoc::eval::WindDraws;
using autoc::eval::WindRealization;
using autoc::eval::WindVariationConfig;
using autoc::eval::drawWindVariation;
using autoc::eval::rampedThermalCountMax;
using autoc::eval::realizeWindVariation;
using autoc::util::ClassPRNG;

namespace {
constexpr double kBaseSpeedMps = 3.6576;  // 12 ft/s, the autoc_config.xml value
constexpr double kBaseTurb = 1.0;

WindVariationConfig t4Config() {
  WindVariationConfig c;
  c.windSpeedMinMps = 0.0;  c.windSpeedMaxMps = 8.0;
  c.turbIntensityMin = 0.5; c.turbIntensityMax = 2.0;
  c.gustLengthScaleMin = 0.25; c.gustLengthScaleMax = 1.0;
  c.thermalStrengthMin = 0.5; c.thermalStrengthMax = 2.0;
  c.thermalCountMax = 8;
  return c;
}
}  // namespace

TEST(WindVariation, DefaultConfigIsOffAndRealizesBaseExactly) {
  WindVariationConfig off;
  EXPECT_FALSE(off.anyEnabled());
  WindDraws extreme{1.0, 1.0, 1.0, 1.0};
  const auto r = realizeWindVariation(off, extreme, kBaseSpeedMps, kBaseTurb, 1.0, true);
  EXPECT_EQ(r.windSpeedMps, kBaseSpeedMps);   // exact: the no-change path must not touch the value
  EXPECT_EQ(r.turbIntensity, kBaseTurb);
  EXPECT_EQ(r.gustLengthScale, 1.0);
  EXPECT_EQ(r.thermalStrengthScale, 1.0);
  EXPECT_EQ(r.thermalCountMaxTarget, 0);
}

TEST(WindVariation, MasterDisableReturnsBaseEvenWithKeysSet) {
  const auto r = realizeWindVariation(t4Config(), WindDraws{0.9, 0.9, 0.9, 0.9},
                                      kBaseSpeedMps, kBaseTurb, 1.0, /*enabled=*/false);
  EXPECT_EQ(r.windSpeedMps, kBaseSpeedMps);
  EXPECT_EQ(r.turbIntensity, kBaseTurb);
  EXPECT_EQ(r.gustLengthScale, 1.0);
  EXPECT_EQ(r.thermalStrengthScale, 1.0);
}

TEST(WindVariation, RampZeroIsBaseRampOneIsTarget) {
  const auto cfg = t4Config();
  const WindDraws top{1.0, 1.0, 1.0, 1.0};
  const auto r0 = realizeWindVariation(cfg, top, kBaseSpeedMps, kBaseTurb, 0.0, true);
  EXPECT_DOUBLE_EQ(r0.windSpeedMps, kBaseSpeedMps);
  EXPECT_DOUBLE_EQ(r0.turbIntensity, kBaseTurb);
  EXPECT_DOUBLE_EQ(r0.gustLengthScale, 1.0);
  EXPECT_DOUBLE_EQ(r0.thermalStrengthScale, 1.0);

  const auto r1 = realizeWindVariation(cfg, top, kBaseSpeedMps, kBaseTurb, 1.0, true);
  EXPECT_DOUBLE_EQ(r1.windSpeedMps, 8.0);
  EXPECT_DOUBLE_EQ(r1.turbIntensity, 2.0);
  EXPECT_DOUBLE_EQ(r1.gustLengthScale, 1.0);      // u=1 => max of the range = 1.0
  EXPECT_DOUBLE_EQ(r1.thermalStrengthScale, 2.0);
  EXPECT_EQ(r1.thermalCountMaxTarget, 8);

  const WindDraws bottom{0.0, 0.0, 0.0, 0.0};
  const auto rb = realizeWindVariation(cfg, bottom, kBaseSpeedMps, kBaseTurb, 1.0, true);
  EXPECT_DOUBLE_EQ(rb.windSpeedMps, 0.0);         // calm day is reachable
  EXPECT_DOUBLE_EQ(rb.gustLengthScale, 0.25);
  EXPECT_DOUBLE_EQ(rb.turbIntensity, 0.5);
}

TEST(WindVariation, HalfRampIsHalfWayFromBase) {
  const auto cfg = t4Config();
  const auto r = realizeWindVariation(cfg, WindDraws{1.0, 0.0, 0.0, 0.0},
                                      kBaseSpeedMps, kBaseTurb, 0.5, true);
  EXPECT_DOUBLE_EQ(r.windSpeedMps, kBaseSpeedMps + 0.5 * (8.0 - kBaseSpeedMps));
}

TEST(WindVariation, DrawsAreDeterministicAndFollowTheSeedDraw) {
  // The worker does: drawnWindSeed = prng.next(); then drawWindVariation(prng).
  // Property 2: the first next() -- which seeds CRRC_Random for thermals and
  // gusts -- is identical whether or not the envelope draws follow it, and
  // the envelope draws are exactly four nextDouble() in a fixed order.
  const uint32_t subseed = 0xA5A5F00Du;
  ClassPRNG a(subseed), b(subseed), c(subseed);
  const uint32_t seedA = a.next();
  const uint32_t seedB = b.next();
  const uint32_t seedC = c.next();
  EXPECT_EQ(seedA, seedB);
  EXPECT_EQ(seedA, seedC);
  const auto da = drawWindVariation(a);
  const auto db = drawWindVariation(b);
  EXPECT_EQ(da.uSpeed, db.uSpeed);
  EXPECT_EQ(da.uTurb, db.uTurb);
  EXPECT_EQ(da.uGust, db.uGust);
  EXPECT_EQ(da.uThermal, db.uThermal);
  // Same four draws, taken by hand, in the same order.
  EXPECT_EQ(da.uSpeed, c.nextDouble());
  EXPECT_EQ(da.uTurb, c.nextDouble());
  EXPECT_EQ(da.uGust, c.nextDouble());
  EXPECT_EQ(da.uThermal, c.nextDouble());
  for (double u : {da.uSpeed, da.uTurb, da.uGust, da.uThermal}) {
    EXPECT_GE(u, 0.0);
    EXPECT_LT(u, 1.0);
  }
}

TEST(WindVariation, ThermalCountRampRoundsFromXmlTowardTarget) {
  EXPECT_EQ(rampedThermalCountMax(5, 0, 1.0), 5);   // target 0 => keep XML
  EXPECT_EQ(rampedThermalCountMax(5, 8, 0.0), 5);
  EXPECT_EQ(rampedThermalCountMax(5, 8, 1.0), 8);
  EXPECT_EQ(rampedThermalCountMax(5, 8, 0.5), 7);   // 6.5 rounds to 7 (lround away from zero)
}

TEST(WindVariation, SpeedNeverGoesNegative) {
  WindVariationConfig c;
  c.windSpeedMinMps = 0.0; c.windSpeedMaxMps = 8.0;
  const auto r = realizeWindVariation(c, WindDraws{0.0, 0.0, 0.0, 0.0}, kBaseSpeedMps, kBaseTurb, 1.0, true);
  EXPECT_GE(r.windSpeedMps, 0.0);
}

// 044 FR-010 / FR-009 — the wind class has ONE draw order, used by both sides:
//   [0] gust/thermal seed (worker), [1..4] envelope draws (worker),
//   [5..] direction offset Gaussian (autoc).
// windClassDrawsForScenario is what autoc logs in the scenario table, so it
// must equal what the worker realises: the worker's chain is reproduced here
// verbatim (deriveClassSubSeeds -> ClassPRNG(wind) -> next() -> draws).
// Before 044 autoc drew the direction from draws [0..1], so the direction
// shared its Box-Muller input with the speed draw [1] -- wind direction was a
// function of wind speed. The direction now comes strictly after the envelope.
TEST(WindVariation, ScenarioDrawsMatchTheWorkerChain_044) {
  const double kDirSigma = 3.14159265358979 / 2.0;
  for (uint64_t seed : {0x1234ull, 0xDEADBEEFCAFEBABEull, 0x528b9fe8256d2c31ull}) {
    const auto got = autoc::eval::windClassDrawsForScenario(seed, kDirSigma);

    ClassPRNG worker(autoc::util::deriveClassSubSeeds(seed).wind);
    const uint32_t workerGustSeed = worker.next();
    const WindDraws w = drawWindVariation(worker);
    EXPECT_EQ(got.gustSeed, workerGustSeed);
    EXPECT_EQ(got.envelope.uSpeed, w.uSpeed);
    EXPECT_EQ(got.envelope.uTurb, w.uTurb);
    EXPECT_EQ(got.envelope.uGust, w.uGust);
    EXPECT_EQ(got.envelope.uThermal, w.uThermal);
    // direction = the NEXT draws after the envelope, never a reused one
    EXPECT_EQ(got.directionOffset, worker.nextGaussian(kDirSigma));
  }
}

// The logged full-strength targets: speed in m/s when the speed range is on
// (NaN = the sim's base speed), multipliers otherwise -- the scenario-table
// columns windMps / turbX / gustLx / thermX.
TEST(WindVariation, FullStrengthTargetsAreTheRangeLerps_044) {
  const auto cfg = t4Config();
  WindDraws d; d.uSpeed = 0.25; d.uTurb = 0.5; d.uGust = 1.0; d.uThermal = 0.0;
  const auto t = autoc::eval::windEnvelopeTargets(cfg, d);
  EXPECT_DOUBLE_EQ(t.windMps, 2.0);
  EXPECT_DOUBLE_EQ(t.turbX, 1.25);
  EXPECT_DOUBLE_EQ(t.gustLx, cfg.gustLengthScaleMax);
  EXPECT_DOUBLE_EQ(t.thermX, cfg.thermalStrengthMin);
  const auto off = autoc::eval::windEnvelopeTargets(WindVariationConfig{}, d);
  EXPECT_TRUE(std::isnan(off.windMps));
  EXPECT_DOUBLE_EQ(off.turbX, 1.0);
  EXPECT_DOUBLE_EQ(off.gustLx, 1.0);
  EXPECT_DOUBLE_EQ(off.thermX, 1.0);
}

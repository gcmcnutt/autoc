// 043 T100 -- aeroStandard slot-3 switch: the historical 45-degree loop vs a
// SECOND seeded random course (RandomPathSeedA).
//
// What is pinned:
//   * The default is the historical loop, so a pinned run's eval ini (which
//     has no AeroStandardPath3 key) regenerates the path set it trained on.
//   * With randomA, slot 3 is a random course that differs from slot 5 (seed A
//     vs seed B through the same generator) and from the loop, every other
//     slot is byte-identical to before, and the path count is unchanged (so
//     ExpectedScenarioCount = 294 still holds).
//   * The second random course obeys the same arena bounds the fit test
//     already proves for SeededRandomB (same generator, same bounds).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/pathgen.h"

namespace {

struct OptionGuard {
  AeroStandardPath3 p3; unsigned seedA;
  OptionGuard() : p3(aeroStandardPath3()), seedA(aeroStandardSeedA()) {}
  ~OptionGuard() { setAeroStandardOptions(p3, seedA); }
};

std::vector<std::vector<Path>> aero(unsigned baseSeed = 13337u) {
  return generateSmoothPaths("aeroStandard", 6, SIM_PATH_BOUNDS, SIM_PATH_HEIGHT_BOUNDS, baseSeed);
}

bool samePath(const std::vector<Path>& a, const std::vector<Path>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if ((a[i].start - b[i].start).norm() > 1e-6f) return false;
  return true;
}

double maxRadius(const std::vector<Path>& p) {
  double r = 0.0;
  for (const auto& seg : p) r = std::max(r, std::hypot(double(seg.start[0]), double(seg.start[1])));
  return r;
}

}  // namespace

TEST(AeroPath3, DefaultIsTheHistoricalLoop) {
  OptionGuard g;
  setAeroStandardOptions(AeroStandardPath3::FortyFiveLoop, 12345u);
  const auto paths = aero();
  ASSERT_EQ(paths.size(), 6u);
  // The 45-degree loop starts at the canonical origin (turn=0 => (0,0,0)).
  ASSERT_FALSE(paths[3].empty());
  EXPECT_LT(paths[3].front().start.norm(), 1e-3f);
  // ...and is the short one: a 15 m-radius loop, far shorter than the random course.
  EXPECT_LT(paths[3].size(), paths[5].size() / 2);
}

TEST(AeroPath3, RandomASwapsOnlySlotThree) {
  OptionGuard g;
  setAeroStandardOptions(AeroStandardPath3::FortyFiveLoop, 12345u);
  const auto before = aero();
  setAeroStandardOptions(AeroStandardPath3::SeededRandomA, 12345u);
  const auto after = aero();
  ASSERT_EQ(after.size(), 6u);                       // ExpectedScenarioCount = 6 x 49 unchanged
  for (int i : {0, 1, 2, 4, 5}) EXPECT_TRUE(samePath(before[i], after[i])) << "slot " << i << " moved";
  EXPECT_FALSE(samePath(before[3], after[3]));       // slot 3 changed
  EXPECT_FALSE(samePath(after[3], after[5]));        // A != B: different seed, same generator
  // A random course does NOT start at the origin -- this is the cold-start
  // entry the operator wants on 2/6 of the regiment.
  EXPECT_GT(after[3].front().start.norm(), 1.0f);
}

TEST(AeroPath3, RandomAIsDeterministicInSeedA) {
  OptionGuard g;
  setAeroStandardOptions(AeroStandardPath3::SeededRandomA, 12345u);
  const auto a1 = aero();
  const auto a2 = aero();
  EXPECT_TRUE(samePath(a1[3], a2[3]));
  setAeroStandardOptions(AeroStandardPath3::SeededRandomA, 54321u);
  const auto a3 = aero();
  EXPECT_FALSE(samePath(a1[3], a3[3]));              // seed A actually drives it
  EXPECT_TRUE(samePath(a1[5], a3[5]));               // ...and does not touch B
}

TEST(AeroPath3, RandomAFitsTheSameBoundAsRandomB) {
  // Same generator and bounds as SeededRandomB; arena_path_fit_tests proves the
  // analytic bound (40 x 1.25 = 50 m worst radius). Sample a seed sweep here.
  OptionGuard g;
  const double bound = static_cast<double>(SIM_PATH_BOUNDS) * 1.25 + 1e-3;
  for (unsigned seedA = 1; seedA <= 100; ++seedA) {
    setAeroStandardOptions(AeroStandardPath3::SeededRandomA, seedA);
    const auto p = aero();
    EXPECT_LE(maxRadius(p[3]), bound) << "seedA " << seedA;
  }
}

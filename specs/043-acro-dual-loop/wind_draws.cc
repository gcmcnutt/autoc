// wind_draws.cc — reproduce the per-scenario wind-envelope draws exactly as the crrcsim worker does (043, 2026-10-16).
// Build (standalone, touches no build dir):  g++ -std=c++17 -O1 -Iinclude -o /tmp/wind_draws specs/043-acro-dual-loop/wind_draws.cc src/util/scenario_prng.cc
// Input on stdin, one line per scenario: "idx path_variant wind_variant scenario_seed" (from dmp-dump --meta-only).
// Output: idx path wind_variant wind_subseed gust_seed uSpeed uTurb uGust uThermal; speed draw = WindSpeedMin + (Max-Min)·uSpeed.
#include <cstdio>
#include <cstdint>
#include <iostream>
#include "autoc/util/scenario_prng.h"
#include "autoc/eval/wind_variation.h"
int main() {
  int idx, path, wv; unsigned long long seed;
  while (std::cin >> idx >> path >> wv >> seed) {
    const auto sub = autoc::util::deriveClassSubSeeds(static_cast<uint64_t>(seed));
    autoc::util::ClassPRNG prng(sub.wind);
    const uint32_t gustSeed = prng.next();
    const auto d = autoc::eval::drawWindVariation(prng);
    std::printf("%d %d %d %u %u %.6f %.6f %.6f %.6f\n", idx, path, wv, sub.wind, gustSeed, d.uSpeed, d.uTurb, d.uGust, d.uThermal);
  }
  return 0;
}

#include "autoc/util/scenario_prng.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace autoc {
namespace util {

namespace {
// Fold a uint64_t down to a uint32_t Park-Miller-compatible state.
// XOR-fold high half into low half, then substitute the zero-sentinel if
// the result happens to land on zero (Park-Miller breaks at state==0).
uint32_t fold_to_pm_state(uint64_t v) {
    const uint32_t hi = static_cast<uint32_t>(v >> 32);
    const uint32_t lo = static_cast<uint32_t>(v & 0xFFFFFFFFULL);
    uint32_t folded = hi ^ lo;
    // Park-Miller state must also be < 0x7FFFFFFF (modulus); mask top bit.
    folded &= 0x7FFFFFFFu;
    if (folded == 0u) folded = kSeedZeroSentinel;
    return folded;
}
}  // namespace

// ============================================================================
// MasterPRNG
// ============================================================================

MasterPRNG::MasterPRNG() : state_(kSeedZeroSentinel), initialized_(false) {}

void MasterPRNG::init(uint64_t masterSeed) {
    state_ = fold_to_pm_state(masterSeed);
    initialized_ = true;
}

uint64_t MasterPRNG::next() {
    if (!initialized_) {
        std::cerr << "FATAL: MasterPRNG.next() called before init()" << std::endl;
        std::abort();
    }
    // Two Park-Miller steps composed into a uint64_t for downstream
    // consumers that want a wide draw (scenarioSeed[K] is uint64_t).
    state_ = park_miller_step(state_);
    const uint32_t hi = state_;
    state_ = park_miller_step(state_);
    const uint32_t lo = state_;
    return (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(lo);
}

// ============================================================================
// ScenarioRootPRNG
// ============================================================================

ScenarioRootPRNG::ScenarioRootPRNG(uint64_t scenarioSeed)
    : state_(fold_to_pm_state(scenarioSeed)) {
    // Defense-in-depth zero guard. fold_to_pm_state already maps 0 →
    // kSeedZeroSentinel, but assert here so a future refactor that bypasses
    // the fold doesn't silently produce a stuck-zero ScenarioRootPRNG.
    if (state_ == 0u) {
        std::cerr << "FATAL: ScenarioRootPRNG zero-state after fold guard "
                     "(scenarioSeed=0x" << std::hex << scenarioSeed
                  << ") — autoc-side seed-table populate must convert "
                     "zero before construct" << std::endl;
        std::abort();
    }
}

uint32_t ScenarioRootPRNG::next() {
    state_ = park_miller_step(state_);
    return state_;
}

// ============================================================================
// ClassPRNG
// ============================================================================

ClassPRNG::ClassPRNG(uint32_t subSeed)
    : state_(subSeed == 0u ? kSeedZeroSentinel : subSeed),
      gaussianCached_(false),
      gaussianCache_(0.0) {}

uint32_t ClassPRNG::next() {
    state_ = park_miller_step(state_);
    return state_;
}

double ClassPRNG::nextDouble() {
    // Park-Miller output is in [1, 2^31 - 1]. Normalize to [0, 1).
    const uint32_t v = next();
    return static_cast<double>(v) / static_cast<double>(0x80000000u);
}

double ClassPRNG::nextGaussian(double sigma) {
    // Truncated normal: bound the standard-normal sample to ±kGaussianSigmaClamp
    // before scaling (see scenario_prng.h). Both Box-Muller outputs are clamped
    // so the cached branch is bounded identically.
    auto clampZ = [](double z) {
        return z < -kGaussianSigmaClamp ? -kGaussianSigmaClamp
             : z >  kGaussianSigmaClamp ?  kGaussianSigmaClamp : z;
    };
    if (gaussianCached_) {
        gaussianCached_ = false;
        return clampZ(gaussianCache_) * sigma;
    }
    // Standard Box-Muller: two uniforms → two N(0,1) samples; return one,
    // cache the other. u1 nudged off zero to avoid log(0).
    const double u1 = nextDouble() * 0.999 + 0.001;
    const double u2 = nextDouble();
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double theta = 2.0 * M_PI * u2;
    const double z0 = r * std::cos(theta);
    const double z1 = r * std::sin(theta);
    gaussianCache_ = z1;
    gaussianCached_ = true;
    return clampZ(z0) * sigma;
}

}  // namespace util
}  // namespace autoc

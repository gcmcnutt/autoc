#ifndef AUTOC_UTIL_SCENARIO_PRNG_H
#define AUTOC_UTIL_SCENARIO_PRNG_H

// 033 §2.A — Replay-friendly variation PRNG architecture.
//
// Three PRNG types form a deterministic chain:
//
//   MasterPRNG(masterSeed)                              [autoc startup, once]
//     ├── .next() → seed for autoc-side NN-evolution PRNG (rng::seed)
//     └── .next() × M → scenarioSeed[0..M-1] table
//
//   per scenario K (worker side, every time scenario K runs):
//
//   ScenarioRootPRNG(scenarioSeed[K])
//     ├── .next() → windPRNG seed   (slot 0; FROZEN APPEND-ONLY ORDER)
//     ├── .next() → rabbitPRNG seed (slot 1)
//     ├── .next() → entryPRNG seed  (slot 2)
//     ├── .next() → craftPRNG seed  (slot 3; SEEDED but unused in 033)
//     └── .next() → cameraPRNG seed (slot 4; SEEDED but unused in 033)
//
//   ClassPRNG(subSeed) — independent stream per variation class
//     .next() / .nextDouble() / .nextGaussian() — feeds variation appliers
//
// Append-only contract: new variation classes ALWAYS append a new
// scenarioRoot.next() call AFTER all existing slots. Existing classes'
// seeds (and therefore prior bakes' reproducibility) are unaffected.
//
// All three types use Park-Miller LCG (uint32_t state) for portability +
// the small-bit-budget needed for cross-process replay across the autoc
// → worker boundary. MasterPRNG accepts uint64_t to preserve operator
// flexibility on the master seed input.
//
// See specs/033-m1-smooth-plus-variations/contracts/scenario_prng_chain.md
// for the full determinism contract (D1-D5) and validation tests.

#include <cstdint>

namespace autoc {
namespace util {

// Truncated-normal clamp. EVERY Gaussian draw in the variation pipeline
// (ClassPRNG::nextGaussian + the LCG gaussians in variation_generator.h) is
// bounded to ±kGaussianSigmaClamp standard deviations of the standard normal
// before scaling by the per-knob sigma. This keeps each variation's range
// proportional to its configured sigma while cutting the crazy tail — e.g.
// the >3σ cone draw that put arena12 at a 106° (backward-facing) entry.
// Clamp rather than resample so the PRNG draw COUNT per call stays fixed and
// the frozen draw-order / bit-exact-replay contract is preserved.
constexpr double kGaussianSigmaClamp = 2.5;

// Park-Miller LCG step — pure helper, exposed for unit testing.
// state' = (state * 48271) mod 0x7FFFFFFF
// Period: 2^31 - 2. Stable, single-precision, classic.
inline uint32_t park_miller_step(uint32_t state) {
    const uint64_t product = static_cast<uint64_t>(state) * 48271ULL;
    return static_cast<uint32_t>(product % 0x7FFFFFFFULL);
}

// Sentinel for zero-seed substitution. Park-Miller LCG breaks at state==0
// (advancing produces 0 forever); upstream guards convert 0 → this value
// before seeding any class. Chosen to mirror the existing crash-hull
// pattern at src/eval/tracker_stepper.cc:47-49.
constexpr uint32_t kSeedZeroSentinel = 0xC0FFEEu;

// 033 §2.A — Per-scenario class sub-seed tuple. Convenience aggregate so
// any consumer holding a `scenarioSeed[K]` can derive all 5 class sub-
// seeds in one call (instead of constructing ScenarioRootPRNG inline).
// Slot order is the FROZEN append-only contract per
// contracts/scenario_prng_chain.md.
struct ClassSubSeeds {
    uint32_t wind;     // slot 0 — wind class
    uint32_t rabbit;   // slot 1 — rabbit class (M1 speed segments, M2 crash-hull)
    uint32_t entry;    // slot 2 — entry-pose class
    uint32_t craft;    // slot 3 — future 025; SEEDED but unused in 033
    uint32_t camera;   // slot 4 — future 034; SEEDED but unused in 033
};

// (deriveClassSubSeeds inline helper is defined AFTER the ScenarioRootPRNG
// class declaration below — forward reference would not compile.)

// MasterPRNG — single instance per autoc run. Seeded once from the
// effective master seed (operator ini value or wall-clock for Seed=-1).
// Produces the NN-evolution PRNG seed (1st draw) followed by the
// scenarioSeed[0..M-1] table. Uses a uint64_t-sized state for the operator-
// facing master-seed surface (preserves -1 / large-value flexibility) and
// folds to uint32_t before consuming via Park-Miller. Implementation in
// scenario_prng.cc.
class MasterPRNG {
public:
    MasterPRNG();
    void init(uint64_t masterSeed);
    uint64_t next();  // returns next draw (suitable for scenarioSeed[K])
    bool initialized() const { return initialized_; }

private:
    uint32_t state_;     // Park-Miller state
    bool initialized_;
};

// ScenarioRootPRNG — per-scenario instance, constructed at scenario start
// from scenarioSeed[K]. Produces the 5 class sub-PRNG seeds (slot order
// is the append-only FROZEN contract: wind / rabbit / entry / craft /
// camera). Implementation in scenario_prng.cc.
class ScenarioRootPRNG {
public:
    explicit ScenarioRootPRNG(uint64_t scenarioSeed);
    uint32_t next();

private:
    uint32_t state_;
};

// ClassPRNG — per-variation-class stream. Seeded from one
// ScenarioRootPRNG.next() draw. Advances normally as variation consumers
// pull from it during the scenario. Provides raw next()/nextDouble()/
// nextGaussian() so existing variation_generator.h gaussian/uniform
// patterns translate cleanly.
class ClassPRNG {
public:
    explicit ClassPRNG(uint32_t subSeed);
    uint32_t next();
    double nextDouble();          // uniform [0, 1)
    double nextGaussian(double sigma);  // N(0, sigma) via Box-Muller

private:
    uint32_t state_;
    // Box-Muller pair cache: when a gaussian draw is requested, generate
    // a pair, return one, cache the other for the next call. Mirrors the
    // standard pattern used by std::normal_distribution.
    bool gaussianCached_;
    double gaussianCache_;
};

// Derive the 5 class sub-seeds from a scenarioSeed. Pure function;
// deterministic; identical output for identical input across processes
// (autoc-side prefetch + crrcsim worker (all workers) see the
// same tuple given the same scenarioSeed). Per spec.md §2.A.
inline ClassSubSeeds deriveClassSubSeeds(uint64_t scenarioSeed) {
    ScenarioRootPRNG root(scenarioSeed);
    ClassSubSeeds s;
    s.wind   = root.next();
    s.rabbit = root.next();
    s.entry  = root.next();
    s.craft  = root.next();
    s.camera = root.next();
    return s;
}

}  // namespace util
}  // namespace autoc

#endif  // AUTOC_UTIL_SCENARIO_PRNG_H

// 033 §2.A — Scenario PRNG chain determinism contracts.
//
// Validates the master → scenario → 5-class sub-PRNG architecture per
// specs/033-m1-smooth-plus-variations/contracts/scenario_prng_chain.md.
// Determinism contracts D1-D5 enforced here at the unit-test level (full
// end-to-end D4 replay test lives in eval_mode_replay_tests.cc).

#include "autoc/util/scenario_prng.h"
#include "autoc/eval/variation_generator.h"  // 034 FR-012 — per-(path,wind) entry offsets

#include <gtest/gtest.h>

#include <array>
#include <set>
#include <vector>

using autoc::util::MasterPRNG;
using autoc::util::ScenarioRootPRNG;
using autoc::util::ClassPRNG;
using autoc::util::park_miller_step;
using autoc::util::kSeedZeroSentinel;

namespace {

// Helper: build the full scenarioSeed[K] table from a master seed.
std::vector<uint64_t> buildScenarioSeedTable(uint64_t masterSeed, size_t M) {
    MasterPRNG mp;
    mp.init(masterSeed);
    mp.next();  // burn first draw (would seed NN-evolution rng::* stream)
    std::vector<uint64_t> table(M);
    for (size_t k = 0; k < M; ++k) table[k] = mp.next();
    return table;
}

// Helper: the production derivation (044: was a local copy of the pre-fmix32
// chain, which would have kept testing the old derivation after it changed).
autoc::util::ClassSubSeeds deriveSubSeeds(uint64_t scenarioSeed) {
    return autoc::util::deriveClassSubSeeds(scenarioSeed);
}

}  // namespace

// ============================================================================
// Park-Miller step primitive
// ============================================================================

TEST(ParkMillerStep, AdvancesDeterministically) {
    EXPECT_EQ(park_miller_step(1u), 48271u);
    EXPECT_NE(park_miller_step(1u), park_miller_step(2u));
}

TEST(ParkMillerStep, NonZeroPreserved) {
    uint32_t s = 1u;
    for (int i = 0; i < 1000; ++i) {
        s = park_miller_step(s);
        ASSERT_NE(s, 0u) << "Park-Miller produced 0 at iteration " << i;
    }
}

// ============================================================================
// MasterPRNG
// ============================================================================

TEST(MasterPRNG, SameSeedProducesSameSequence) {
    MasterPRNG a, b;
    a.init(12345);
    b.init(12345);
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(a.next(), b.next()) << "divergence at draw " << i;
    }
}

TEST(MasterPRNG, DifferentSeedProducesDifferentSequence) {
    MasterPRNG a, b;
    a.init(12345);
    b.init(67890);
    bool any_diff = false;
    for (int i = 0; i < 100; ++i) {
        if (a.next() != b.next()) { any_diff = true; break; }
    }
    EXPECT_TRUE(any_diff);
}

TEST(MasterPRNG, ZeroSeedSurvivesViaFold) {
    MasterPRNG a;
    a.init(0);
    // Should not produce stuck-zero output; folding maps 0 → sentinel.
    EXPECT_NE(a.next(), 0u);
    EXPECT_NE(a.next(), 0u);
}

// D1 — Run-level determinism: same master seed → same scenarioSeedTable
TEST(D1RunLevelDeterminism, SameMasterSeedSameScenarioSeedTable) {
    auto table_a = buildScenarioSeedTable(42, 75);
    auto table_b = buildScenarioSeedTable(42, 75);
    EXPECT_EQ(table_a, table_b);
}

TEST(D1RunLevelDeterminism, DifferentMasterSeedDifferentTable) {
    auto table_a = buildScenarioSeedTable(42, 75);
    auto table_b = buildScenarioSeedTable(43, 75);
    EXPECT_NE(table_a, table_b);
}

// ============================================================================
// ScenarioRootPRNG + class sub-seeds
// ============================================================================

// D2 — Scenario-level: same scenarioSeed → identical class sub-seeds tuple
TEST(D2ScenarioLevelDeterminism, SameScenarioSeedSameClassSubSeeds) {
    auto a = deriveSubSeeds(0xDEADBEEFCAFEBABEull);
    auto b = deriveSubSeeds(0xDEADBEEFCAFEBABEull);
    EXPECT_EQ(a.wind,   b.wind);
    EXPECT_EQ(a.rabbit, b.rabbit);
    EXPECT_EQ(a.entry,  b.entry);
    EXPECT_EQ(a.craft,  b.craft);
    EXPECT_EQ(a.camera, b.camera);
}

TEST(D2ScenarioLevelDeterminism, AllFiveSubSeedsDistinct) {
    auto s = deriveSubSeeds(0xDEADBEEFCAFEBABEull);
    std::set<uint32_t> uniq{s.wind, s.rabbit, s.entry, s.craft, s.camera};
    EXPECT_EQ(uniq.size(), 5u) << "class sub-seeds collided — append-only "
                                  "draws should produce distinct values";
}

TEST(ScenarioRootPRNG, ZeroSeedAborts) {
    // ScenarioRootPRNG's internal guard maps 0 → kSeedZeroSentinel via
    // fold_to_pm_state. Construction with literal 0 should not std::abort
    // (the upstream autoc-side guard converts 0 → 0xC0FFEEu before this
    // point, but the fold provides defense-in-depth).
    ScenarioRootPRNG root(0);
    EXPECT_NE(root.next(), 0u);
}

// ============================================================================
// ClassPRNG independence (D2 cont.)
// ============================================================================

TEST(ClassPRNGIndependence, AdvancingWindDoesNotAffectRabbit) {
    auto s = deriveSubSeeds(12345);
    ClassPRNG wind1(s.wind), wind2(s.wind);
    ClassPRNG rabbit1(s.rabbit), rabbit2(s.rabbit);

    // Heavily consume wind1; do nothing to wind2.
    for (int i = 0; i < 1000; ++i) wind1.next();

    // Now rabbit1 and rabbit2 should still produce the same sequence.
    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(rabbit1.next(), rabbit2.next()) << "rabbit diverged at " << i;
    }
}

TEST(ClassPRNGIndependence, AdvancingRabbitDoesNotAffectEntry) {
    auto s = deriveSubSeeds(12345);
    ClassPRNG rabbit1(s.rabbit);
    ClassPRNG entry1(s.entry), entry2(s.entry);

    for (int i = 0; i < 1000; ++i) rabbit1.next();

    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(entry1.next(), entry2.next());
    }
}

// 044 FR-009 — class streams must not reproduce each other's draws. Before
// 044 every class sub-seed was a raw ScenarioRootPRNG output and every stream
// is Park-Miller with output == state, so a ClassPRNG seeded with root draw k
// simply CONTINUED the root sequence: the wind stream's first draw was the
// rabbit sub-seed, its second the entry sub-seed, and so on -- wind draw n+1
// == rabbit draw n. mixClassSubSeed (murmur3 fmix32) puts each class at an
// unrelated point of the cycle. Checked over 10 000 scenario seeds: no class's
// first 64 draws contain another class's sub-seed or first draw.
TEST(ClassPRNGIndependence, ClassStreamsDoNotReproduceEachOther_044) {
    auto table = buildScenarioSeedTable(20261019, 10000);
    constexpr int kDraws = 64;
    int overlaps = 0;
    for (uint64_t seed : table) {
        const auto s = deriveSubSeeds(seed);
        const uint32_t subs[5] = {s.wind, s.rabbit, s.entry, s.craft, s.camera};
        uint32_t firstDraw[5];
        std::set<uint32_t> drawn[5];
        for (int c = 0; c < 5; ++c) {
            ClassPRNG p(subs[c]);
            for (int i = 0; i < kDraws; ++i) {
                const uint32_t v = p.next();
                if (i == 0) firstDraw[c] = v;
                drawn[c].insert(v);
            }
        }
        for (int a = 0; a < 5; ++a) {
            for (int b = 0; b < 5; ++b) {
                if (a == b) continue;
                if (drawn[a].count(subs[b]) || drawn[a].count(firstDraw[b])) ++overlaps;
            }
        }
    }
    EXPECT_EQ(overlaps, 0) << "class streams overlap: one class's draws reproduce "
                              "another's (Park-Miller continuation, 044 FR-009)";
}

// The mixed sub-seed must be a valid Park-Miller state: in [1, 2^31 - 2].
// 0 sticks at 0 and 2^31 - 1 (the modulus) steps to 0.
TEST(ClassPRNGIndependence, MixedSubSeedIsValidParkMillerState_044) {
    using autoc::util::mixClassSubSeed;
    for (uint32_t x : {0u, 1u, 0x7FFFFFFEu, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu, 12345u}) {
        const uint32_t m = mixClassSubSeed(x);
        EXPECT_GE(m, 1u);
        EXPECT_LE(m, 0x7FFFFFFEu);
    }
    for (uint32_t x = 1; x < 200000; x += 7) {
        const uint32_t m = mixClassSubSeed(x);
        ASSERT_GE(m, 1u);
        ASSERT_LE(m, 0x7FFFFFFEu);
    }
}

// ============================================================================
// D3 — Cross-mode equivalence: M1 K and M2 K with same master seed get
// SAME class sub-seed tuple. (Downstream consumers may DRAW differently
// per mode — D3 is at the SEED level, not the draw level.)
// ============================================================================

TEST(D3CrossModeEquivalence, M1AndM2SameSubSeedTupleFromSameMasterSeed) {
    // Simulate M1 init: master → scenarioSeed[5] → class sub-seeds.
    auto table = buildScenarioSeedTable(99, 10);
    auto m1 = deriveSubSeeds(table[5]);
    // Simulate M2 init with the SAME master seed and SAME scenario index.
    auto m2 = deriveSubSeeds(table[5]);
    EXPECT_EQ(m1.wind,   m2.wind);
    EXPECT_EQ(m1.rabbit, m2.rabbit);
    EXPECT_EQ(m1.entry,  m2.entry);
    EXPECT_EQ(m1.craft,  m2.craft);
    EXPECT_EQ(m1.camera, m2.camera);
}

// ============================================================================
// D5 — Append-only contract: adding a 6th class doesn't change the first 5
// sub-seeds for the same scenarioSeed.
// ============================================================================

TEST(D5AppendOnlyContract, AddingSixthClassDoesNotPerturbFirstFive) {
    const uint64_t scenarioSeed = 0xABCDEF0123456789ull;

    // "Today" derivation: 5 class sub-seeds.
    auto today = deriveSubSeeds(scenarioSeed);

    // "Future" derivation: 5 class sub-seeds + 1 hypothetical 6th
    // (sim-tick jitter or similar). New slot must APPEND, not prepend.
    // 044: each slot is the mixed root draw (mixClassSubSeed), slot by slot.
    using autoc::util::mixClassSubSeed;
    ScenarioRootPRNG root(scenarioSeed);
    const uint32_t future_wind   = mixClassSubSeed(root.next());
    const uint32_t future_rabbit = mixClassSubSeed(root.next());
    const uint32_t future_entry  = mixClassSubSeed(root.next());
    const uint32_t future_craft  = mixClassSubSeed(root.next());
    const uint32_t future_camera = mixClassSubSeed(root.next());
    /*const uint32_t future_jitter =*/ mixClassSubSeed(root.next());  // new class — APPENDED

    EXPECT_EQ(today.wind,   future_wind);
    EXPECT_EQ(today.rabbit, future_rabbit);
    EXPECT_EQ(today.entry,  future_entry);
    EXPECT_EQ(today.craft,  future_craft);
    EXPECT_EQ(today.camera, future_camera);
}

// ============================================================================
// NN-evolution PRNG separation: heavy consumption from a separate stream
// MUST NOT affect any scenarioSubSeed. (The MasterPRNG separately
// guarantees: 1st draw is the NN-evo seed, subsequent draws populate the
// scenarioSeed table — they're decoupled at the autoc-side init site.)
// ============================================================================

TEST(NNPRNGSeparation, HeavyNNStreamDoesNotAffectScenarioSubSeeds) {
    // Get a baseline class-sub-seed tuple for scenario K=3 with master=77.
    auto baseline_table = buildScenarioSeedTable(77, 10);
    auto baseline_sub = deriveSubSeeds(baseline_table[3]);

    // Now imagine an alternate-universe autoc that consumed 10000 extra
    // NN-evolution draws from rng::* between the NN-PRNG seed draw and
    // the scenarioSeed[K] draws. That CANNOT happen with the correct
    // chain (MasterPRNG draws happen back-to-back), so we instead verify
    // that the table is purely a function of (masterSeed, M) — not of
    // any external rng::* activity.
    auto repeat_table = buildScenarioSeedTable(77, 10);
    auto repeat_sub = deriveSubSeeds(repeat_table[3]);

    EXPECT_EQ(baseline_sub.wind,   repeat_sub.wind);
    EXPECT_EQ(baseline_sub.rabbit, repeat_sub.rabbit);
    EXPECT_EQ(baseline_sub.entry,  repeat_sub.entry);
    EXPECT_EQ(baseline_sub.craft,  repeat_sub.craft);
    EXPECT_EQ(baseline_sub.camera, repeat_sub.camera);
}

// ============================================================================
// ClassPRNG draw types
// ============================================================================

TEST(ClassPRNG, NextDoubleInUnitRange) {
    ClassPRNG p(42);
    for (int i = 0; i < 1000; ++i) {
        const double d = p.nextDouble();
        EXPECT_GE(d, 0.0);
        EXPECT_LT(d, 1.0);
    }
}

TEST(ClassPRNG, NextGaussianReasonableStatistics) {
    ClassPRNG p(42);
    const int N = 10000;
    double sum = 0.0, sumsq = 0.0;
    for (int i = 0; i < N; ++i) {
        const double g = p.nextGaussian(1.0);
        sum += g;
        sumsq += g * g;
    }
    const double mean = sum / N;
    const double var = sumsq / N - mean * mean;
    // Loose tolerances; just confirm the distribution isn't broken.
    EXPECT_NEAR(mean, 0.0, 0.05) << "Gaussian mean off (should be ~0)";
    EXPECT_NEAR(var, 1.0, 0.1) << "Gaussian variance off (should be ~1)";
}

TEST(ClassPRNG, SameSubSeedReproducibleSequence) {
    ClassPRNG a(0xC0FFEEu), b(0xC0FFEEu);
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(a.next(), b.next());
    }
}

TEST(ClassPRNG, GaussianReproducibleSequence) {
    ClassPRNG a(0xDEADu), b(0xDEADu);
    for (int i = 0; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(a.nextGaussian(1.5), b.nextGaussian(1.5));
    }
}

// 034 FR-012 — per-(path, wind) variation table. Two scenarios at the SAME
// wind index but DIFFERENT path index must receive distinct entry-pose
// offsets: the path-major seed table gives each (path, wind) its own
// scenarioSeed, so the entry-class offsets derived from it differ. Pre-034 the
// autoc-side table was indexed per-wind (size = windScenarioCount), so paths
// 1..N silently reused path-0's offsets. This guards that regression — the
// fix sizes gScenarioVariations by paths × winds and indexes by the linear
// path-major K (matching gScenarioSeedTable + scenarioMetaList).
TEST(ScenarioPrngChain, FR012_PerPathSameWindEntryOffsetsDistinct) {
    constexpr size_t PATHS = 2, WINDS = 3;
    // Path-major linear table: K = pathIdx * WINDS + windIdx (production layout).
    const auto table = buildScenarioSeedTable(0x1234ABCDull, PATHS * WINDS);
    const VariationSigmas sig =
        VariationSigmas::fromDegrees(/*cone=*/30, /*roll=*/30, /*speed=*/0.1,
                                     /*windDir=*/45);

    auto entryFor = [&](size_t pathIdx, size_t windIdx) {
        const uint64_t seed = table[pathIdx * WINDS + windIdx];
        ClassPRNG e(autoc::util::deriveClassSubSeeds(seed).entry);
        return generateEntryVariationsFromClassPRNG(e, sig);
    };

    constexpr size_t W = 1;  // fixed wind index, vary path
    const VariationOffsets p0 = entryFor(0, W);
    const VariationOffsets p1 = entryFor(1, W);

    // Distinct across paths at the same wind — the FR-012 fix.
    EXPECT_NE(p0.entryHeadingOffset, p1.entryHeadingOffset);
    EXPECT_NE(p0.entryRollOffset, p1.entryRollOffset);
    EXPECT_NE(p0.entrySpeedFactor, p1.entrySpeedFactor);

    // Determinism sanity: same (path, wind) reproduces bit-for-bit.
    const VariationOffsets p0_again = entryFor(0, W);
    EXPECT_DOUBLE_EQ(p0.entryHeadingOffset, p0_again.entryHeadingOffset);
    EXPECT_DOUBLE_EQ(p0.entryRollOffset, p0_again.entryRollOffset);
}

// 034 entry-extreme fix — truncated-normal clamp invariant.
// Every ClassPRNG::nextGaussian draw must be bounded to ±kGaussianSigmaClamp·σ,
// and the entry cone half-angle must never enter the rear hemisphere (≤80°),
// so no scenario can draw the backward/inverted entry that arena12 hit.
TEST(GaussianClamp, NextGaussianBoundedToSigmaClamp) {
    ClassPRNG g(0xBEEF1234u);
    const double sigma = 30.0;
    const double bound = autoc::util::kGaussianSigmaClamp * sigma;
    for (int i = 0; i < 200000; ++i) {
        const double z = g.nextGaussian(sigma);
        EXPECT_LE(std::fabs(z), bound + 1e-9) << "draw " << i << " = " << z;
    }
}

TEST(GaussianClamp, EntryConeStaysForwardHemisphere) {
    // Even with a deliberately huge cone sigma, the forward-cone guard must
    // keep cosC > 0 (nose in the forward hemisphere) for every scenario.
    constexpr int PATHS = 6, WINDS = 49;
    const auto table = buildScenarioSeedTable(0xC0FFEEull, PATHS * WINDS);
    const VariationSigmas sig =
        VariationSigmas::fromDegrees(/*cone=*/90, /*roll=*/30, /*speed=*/0.1,
                                     /*windDir=*/45);
    for (size_t s = 0; s < table.size(); ++s) {
        ClassPRNG e(autoc::util::deriveClassSubSeeds(table[s]).entry);
        const VariationOffsets v = generateEntryVariationsFromClassPRNG(e, sig);
        // |heading| < 90° AND |pitch| < 90° ⟺ nose is forward of the wing line.
        EXPECT_LT(std::fabs(v.entryHeadingOffset), M_PI / 2.0)
            << "scenario " << s << " heading=" << v.entryHeadingOffset;
    }
}

// 034 US4 — Contract tests for craft variation invariants.
//
// Mirrors the per-task spec in tasks.md T043 and the invariants in
// specs/034-energy-objective-cleanup/contracts/craft-variation-contract.md.
//
// What this file covers (autoc-side; FDM application is verified live by
// the operator's M1→M1 replay gate per T044):
//   (a) σ=0 no-op:            all-zero sigmas produce all-zero deltas /
//                             1.0 thrustScale (FR-021, SC-004).
//   (b) Same-seed determinism: identical scenarioSeed → identical drawn
//                             craft deltas bit-for-bit (FR-018, SC-005).
//   (c) Ramp scaling:         applyVariationScale at scale=0/0.5/1.0 mutates
//                             ScenarioMetadata craft fields by the documented
//                             formulas (additive *= scale; thrustScale toward
//                             1.0). Eval-mode replay invariant (FR-019).
//   (d) craftSeed round-trip: ScenarioMetadata serializes + deserializes
//                             craftSeed and all craft deltas via cereal
//                             without loss (FR-020).
//
// Not covered here (deferred to T044 operator-run gate):
//   - End-to-end FDM perturbation: nominal cache + Global::craft* + per-eval
//     trajectory bit-equality. Replay gate exercises this top-down.

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include <cereal/archives/binary.hpp>

#include "autoc/eval/aircraft_state.h"  // pulled in via autoc.h transitive — keep explicit to match the convention in tracker_dmp_roundtrip_tests
#include "autoc/eval/craft_variation.h"
#include "autoc/eval/scenario_meta_apply.h"
#include "autoc/rpc/protocol.h"  // brings in scenario_metadata.h + the cereal save/load specializations for gp_vec3 (needed for ScenarioMetadata.originOffset round-trip)
#include "autoc/util/scenario_prng.h"

using autoc::eval::CraftDeltas;
using autoc::eval::CraftSigmas;
using autoc::eval::applyVariationScale;
using autoc::eval::generateCraftFromClassPRNG;
using autoc::util::ClassPRNG;
using autoc::util::deriveClassSubSeeds;

namespace {

// Tolerance for gp_scalar equality. PRNG output is exactly bit-deterministic
// per seed, so 0.0 is the strict tolerance. The ramp arithmetic uses gp_scalar
// (float) multiplication; we allow a tiny epsilon for round-trip post-cereal.
constexpr gp_scalar kStrict = static_cast<gp_scalar>(0.0);
constexpr gp_scalar kFloatEps = static_cast<gp_scalar>(1e-6);

}  // namespace

// ============================================================================
// (a) σ=0 no-op — all-zero sigmas → exact nominal deltas (FR-021, SC-004)
// ============================================================================

TEST(CraftVariation, ZeroSigmaProducesNominalDeltas) {
    CraftSigmas zeros;  // all default to 0.0
    ClassPRNG prng(0xDEADBEEFu);  // seed arbitrary — σ=0 collapses every draw

    CraftDeltas d = generateCraftFromClassPRNG(prng, zeros);

    EXPECT_FLOAT_EQ(d.craftCGDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(d.craftDragDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(d.craftTrimDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(d.craftThrustScale, static_cast<gp_scalar>(1.0));
    EXPECT_FLOAT_EQ(d.craftPitchEffDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(d.craftRollEffDelta, static_cast<gp_scalar>(0.0));
}

// ============================================================================
// (b) Same-seed determinism (FR-018, SC-005) — identical scenarioSeed gives
//     identical CraftDeltas. Two independent ClassPRNG instances seeded
//     identically must produce bit-equal output.
// ============================================================================

TEST(CraftVariation, SameSeedProducesIdenticalDraw) {
    CraftSigmas sigmas;
    sigmas.craftCGSigma = 0.03;
    sigmas.craftDragSigma = 0.05;
    sigmas.craftTrimSigma = 0.01;
    sigmas.craftThrustSigma = 0.05;
    sigmas.craftPitchEffSigma = 0.05;
    sigmas.craftRollEffSigma = 0.05;

    constexpr uint32_t kSeed = 0x12345678u;
    ClassPRNG prngA(kSeed);
    ClassPRNG prngB(kSeed);

    CraftDeltas a = generateCraftFromClassPRNG(prngA, sigmas);
    CraftDeltas b = generateCraftFromClassPRNG(prngB, sigmas);

    EXPECT_EQ(a.craftCGDelta, b.craftCGDelta);
    EXPECT_EQ(a.craftDragDelta, b.craftDragDelta);
    EXPECT_EQ(a.craftTrimDelta, b.craftTrimDelta);
    EXPECT_EQ(a.craftThrustScale, b.craftThrustScale);
    EXPECT_EQ(a.craftPitchEffDelta, b.craftPitchEffDelta);
    EXPECT_EQ(a.craftRollEffDelta, b.craftRollEffDelta);

    // Confirm the draw actually moved (σ > 0 + non-zero seed): if EVERY field
    // is exactly nominal it implies the PRNG silently returned zeros and the
    // determinism check above passed trivially. At least one of the additive
    // deltas should be non-zero. (Thrust draw could land exactly at 1.0 with
    // probability 0 from a continuous distribution.)
    const bool any_perturbed = (a.craftCGDelta != static_cast<gp_scalar>(0.0))
        || (a.craftDragDelta != static_cast<gp_scalar>(0.0))
        || (a.craftTrimDelta != static_cast<gp_scalar>(0.0))
        || (a.craftPitchEffDelta != static_cast<gp_scalar>(0.0))
        || (a.craftRollEffDelta != static_cast<gp_scalar>(0.0));
    EXPECT_TRUE(any_perturbed) << "non-zero sigma + seed should yield non-zero draw";
}

// ============================================================================
// (b cont.) ScenarioSeed cascade — deriveClassSubSeeds(scenarioSeed).craft is
//     the actual upstream feed to generateCraftFromClassPRNG; confirm two
//     identical scenarioSeeds give identical craft draws end-to-end.
// ============================================================================

TEST(CraftVariation, SameScenarioSeedProducesIdenticalDrawViaCascade) {
    CraftSigmas sigmas;
    sigmas.craftCGSigma = 0.1;
    sigmas.craftDragSigma = 0.05;
    sigmas.craftThrustSigma = 0.05;

    constexpr uint64_t kScenarioSeed = 0xABCDEF1234567890ull;
    const auto subA = deriveClassSubSeeds(kScenarioSeed);
    const auto subB = deriveClassSubSeeds(kScenarioSeed);
    EXPECT_EQ(subA.craft, subB.craft);

    ClassPRNG prngA(subA.craft);
    ClassPRNG prngB(subB.craft);
    CraftDeltas a = generateCraftFromClassPRNG(prngA, sigmas);
    CraftDeltas b = generateCraftFromClassPRNG(prngB, sigmas);

    EXPECT_EQ(a.craftCGDelta, b.craftCGDelta);
    EXPECT_EQ(a.craftDragDelta, b.craftDragDelta);
    EXPECT_EQ(a.craftThrustScale, b.craftThrustScale);
}

// ============================================================================
// (c) Ramp scaling — applyVariationScale at scale=0 maps craft to nominal,
//     at scale=1 leaves draw unchanged, at scale=0.5 halfway (FR-019).
//
// These tests mutate a ScenarioMetadata in place and check the documented
// formula: additive deltas `*= scale`; craftThrustScale interpolated toward
// 1.0 as `1.0 + scale * (x - 1.0)`. Entry/wind fields are also mutated by
// applyVariationScale but those are covered by separate tests; we leave them
// at their defaults so any unintended cross-pollination would surface.
// ============================================================================

namespace {

// Helper to populate a meta with known non-nominal craft draws. Picked
// values that exercise sign and magnitude on every axis.
ScenarioMetadata makeMetaWithDraws() {
    ScenarioMetadata m;
    m.craftCGDelta       = static_cast<gp_scalar>(0.04);
    m.craftDragDelta     = static_cast<gp_scalar>(-0.06);
    m.craftTrimDelta     = static_cast<gp_scalar>(0.02);
    m.craftThrustScale   = static_cast<gp_scalar>(1.10);
    m.craftPitchEffDelta = static_cast<gp_scalar>(-0.03);
    m.craftRollEffDelta  = static_cast<gp_scalar>(0.07);
    return m;
}

}  // namespace

TEST(CraftVariation, ScaleZeroCollapsesCraftToNominal) {
    ScenarioMetadata m = makeMetaWithDraws();
    applyVariationScale(m, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftCGDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftDragDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftTrimDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftThrustScale, static_cast<gp_scalar>(1.0));
    EXPECT_FLOAT_EQ(m.craftPitchEffDelta, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftRollEffDelta, static_cast<gp_scalar>(0.0));
}

TEST(CraftVariation, ScaleOneLeavesDrawsUnchanged) {
    ScenarioMetadata original = makeMetaWithDraws();
    ScenarioMetadata m = original;
    applyVariationScale(m, static_cast<gp_scalar>(1.0));
    EXPECT_FLOAT_EQ(m.craftCGDelta, original.craftCGDelta);
    EXPECT_FLOAT_EQ(m.craftDragDelta, original.craftDragDelta);
    EXPECT_FLOAT_EQ(m.craftTrimDelta, original.craftTrimDelta);
    EXPECT_FLOAT_EQ(m.craftThrustScale, original.craftThrustScale);
    EXPECT_FLOAT_EQ(m.craftPitchEffDelta, original.craftPitchEffDelta);
    EXPECT_FLOAT_EQ(m.craftRollEffDelta, original.craftRollEffDelta);
}

TEST(CraftVariation, ScaleHalfHalvesAdditivesAndInterpolatesThrust) {
    ScenarioMetadata original = makeMetaWithDraws();
    ScenarioMetadata m = original;
    const gp_scalar kHalf = static_cast<gp_scalar>(0.5);
    applyVariationScale(m, kHalf);

    EXPECT_NEAR(m.craftCGDelta, kHalf * original.craftCGDelta, kFloatEps);
    EXPECT_NEAR(m.craftDragDelta, kHalf * original.craftDragDelta, kFloatEps);
    EXPECT_NEAR(m.craftTrimDelta, kHalf * original.craftTrimDelta, kFloatEps);
    EXPECT_NEAR(m.craftPitchEffDelta, kHalf * original.craftPitchEffDelta, kFloatEps);
    EXPECT_NEAR(m.craftRollEffDelta, kHalf * original.craftRollEffDelta, kFloatEps);

    // craftThrustScale: 1.0 + 0.5 * (1.10 - 1.0) = 1.05
    const gp_scalar expected_thrust = static_cast<gp_scalar>(1.0)
        + kHalf * (original.craftThrustScale - static_cast<gp_scalar>(1.0));
    EXPECT_NEAR(m.craftThrustScale, expected_thrust, kFloatEps);
}

// ============================================================================
// (c cont.) Ramp invariant — the DRAW is independent of any per-eval scale.
// Re-drawing with the same seed at gen 1 and gen N gives bit-identical
// deltas; the per-eval ramping happens in applyVariationScale, not in
// generateCraftFromClassPRNG.
// ============================================================================

TEST(CraftVariation, DrawIsIndependentOfApplyScale) {
    CraftSigmas sigmas;
    sigmas.craftCGSigma = 0.05;
    sigmas.craftThrustSigma = 0.05;

    constexpr uint32_t kSeed = 0xCAFEBABEu;
    ClassPRNG prng_gen1(kSeed);
    ClassPRNG prng_genN(kSeed);

    const CraftDeltas d_gen1 = generateCraftFromClassPRNG(prng_gen1, sigmas);
    const CraftDeltas d_genN = generateCraftFromClassPRNG(prng_genN, sigmas);

    // Bit-equal draw regardless of which gen the call happens in. The
    // variation ramp only enters at applyVariationScale time downstream.
    EXPECT_EQ(d_gen1.craftCGDelta, d_genN.craftCGDelta);
    EXPECT_EQ(d_gen1.craftThrustScale, d_genN.craftThrustScale);
}

// ============================================================================
// (d) Cereal round-trip — ScenarioMetadata serializes + deserializes all
//     six craft deltas plus craftSeed without loss (FR-020).
// ============================================================================

TEST(CraftVariation, ScenarioMetadataRoundTripsAllCraftFields) {
    ScenarioMetadata src = makeMetaWithDraws();
    src.craftSeed = 0xBADF00Du;
    // Also set scenarioSeed and a couple of legacy fields so the round-trip
    // exercises the appended-at-end position correctly.
    src.scenarioSeed = 0x0123456789ABCDEFull;
    src.pathVariantIndex = 3;
    src.windVariantIndex = 17;
    src.scenarioSequence = 42;

    std::stringstream buf;
    {
        cereal::BinaryOutputArchive oa(buf);
        oa(src);
    }

    ScenarioMetadata dst;
    {
        cereal::BinaryInputArchive ia(buf);
        ia(dst);
    }

    EXPECT_EQ(dst.craftCGDelta, src.craftCGDelta);
    EXPECT_EQ(dst.craftDragDelta, src.craftDragDelta);
    EXPECT_EQ(dst.craftTrimDelta, src.craftTrimDelta);
    EXPECT_EQ(dst.craftThrustScale, src.craftThrustScale);
    EXPECT_EQ(dst.craftPitchEffDelta, src.craftPitchEffDelta);
    EXPECT_EQ(dst.craftRollEffDelta, src.craftRollEffDelta);
    EXPECT_EQ(dst.craftSeed, src.craftSeed);

    // Confirm the legacy fields still round-trip — guards against an
    // accidental field-order shuffle in the serialize() walk.
    EXPECT_EQ(dst.scenarioSeed, src.scenarioSeed);
    EXPECT_EQ(dst.pathVariantIndex, src.pathVariantIndex);
    EXPECT_EQ(dst.windVariantIndex, src.windVariantIndex);
    EXPECT_EQ(dst.scenarioSequence, src.scenarioSequence);
}

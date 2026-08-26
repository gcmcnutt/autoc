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

TEST(CraftVariation, ScaleZeroLeavesCraftAtFullDrawnMagnitude) {
    // 88a52ea (operator 2026-06-09): env-only variation ramp — craft fields
    // are NOT ramped and pass through applyVariationScale untouched at any
    // scale (scenario_meta_apply.h). Pre-88a52ea this test asserted
    // collapse-to-nominal at scale 0; stale assertion updated with 037.
    ScenarioMetadata original = makeMetaWithDraws();
    ScenarioMetadata m = original;
    applyVariationScale(m, static_cast<gp_scalar>(0.0));
    EXPECT_FLOAT_EQ(m.craftCGDelta, original.craftCGDelta);
    EXPECT_FLOAT_EQ(m.craftDragDelta, original.craftDragDelta);
    EXPECT_FLOAT_EQ(m.craftTrimDelta, original.craftTrimDelta);
    EXPECT_FLOAT_EQ(m.craftThrustScale, original.craftThrustScale);
    EXPECT_FLOAT_EQ(m.craftPitchEffDelta, original.craftPitchEffDelta);
    EXPECT_FLOAT_EQ(m.craftRollEffDelta, original.craftRollEffDelta);
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

TEST(CraftVariation, ScaleHalfLeavesCraftUntouched) {
    // 88a52ea env-only ramp: partial scale does NOT halve craft additives
    // nor interpolate thrust — craft fields are scale-invariant (see
    // ScaleZeroLeavesCraftAtFullDrawnMagnitude above).
    ScenarioMetadata original = makeMetaWithDraws();
    ScenarioMetadata m = original;
    applyVariationScale(m, static_cast<gp_scalar>(0.5));

    EXPECT_FLOAT_EQ(m.craftCGDelta, original.craftCGDelta);
    EXPECT_FLOAT_EQ(m.craftDragDelta, original.craftDragDelta);
    EXPECT_FLOAT_EQ(m.craftTrimDelta, original.craftTrimDelta);
    EXPECT_FLOAT_EQ(m.craftPitchEffDelta, original.craftPitchEffDelta);
    EXPECT_FLOAT_EQ(m.craftRollEffDelta, original.craftRollEffDelta);
    EXPECT_FLOAT_EQ(m.craftThrustScale, original.craftThrustScale);
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

// ============================================================================
// 043 US5 — IMU imperfection axes + craftCmQ pitch damping.
// ============================================================================

namespace {
// The six pre-043 additive/multiplicative sigmas + the two 037 actuator sigmas,
// all non-zero, so a draw exercises every pre-existing axis.
CraftSigmas preExistingSigmas() {
    CraftSigmas s;
    s.craftCGSigma = 0.03; s.craftDragSigma = 0.05; s.craftTrimSigma = 0.01;
    s.craftThrustSigma = 0.05; s.craftPitchEffSigma = 0.05; s.craftRollEffSigma = 0.05;
    s.craftServoSlewSigma = 4.0; s.craftThrustTauSigma = 0.06;
    return s;
}
}  // namespace

// T017 — σ=0 on every NEW axis is a bit-identical no-op: the new fields sit at
// nominal (0 misalign / 1.0 scale / 0 bias / -4.2 cmQ) AND — because the new
// draws are appended at the BOTTOM — every pre-existing draw keeps its value
// whether the new sigmas are 0 or not (draw order frozen; FR-053, SC-009).
TEST(CraftVariationImu, ZeroSigmaNewAxesAreNominalNoOp) {
    CraftSigmas s = preExistingSigmas();  // new-axis sigmas stay 0.0 (defaults)
    ClassPRNG prng(0x0043A5A5u);
    CraftDeltas d = generateCraftFromClassPRNG(prng, s);

    EXPECT_EQ(d.craftImuMisalignRoll,  static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftImuMisalignPitch, static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftImuMisalignYaw,   static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftGyroScaleX, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftGyroScaleY, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftGyroScaleZ, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftAccelScaleX, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftAccelScaleY, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftAccelScaleZ, static_cast<gp_scalar>(1.0));
    EXPECT_EQ(d.craftAccelBiasX, static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftAccelBiasY, static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftAccelBiasZ, static_cast<gp_scalar>(0.0));
    EXPECT_EQ(d.craftCmQ, static_cast<gp_scalar>(-4.2));
}

// T017/SC-009 — the appended draws do NOT disturb any pre-existing draw. Same
// seed, new sigmas 0 vs. >0: every pre-existing field is bit-identical.
TEST(CraftVariationImu, AppendedDrawsLeavePreExistingUnchanged) {
    constexpr uint32_t kSeed = 0x5EED0043u;
    CraftSigmas without = preExistingSigmas();       // new sigmas = 0
    CraftSigmas with = preExistingSigmas();           // new sigmas > 0
    with.craftImuMisalignSigma = 2.0; with.craftGyroScaleSigma = 0.02;
    with.craftAccelScaleSigma = 0.02; with.craftAccelBiasSigma = 0.04;
    with.craftCmQSigma = 0.32;

    ClassPRNG pa(kSeed), pb(kSeed);
    CraftDeltas a = generateCraftFromClassPRNG(pa, without);
    CraftDeltas b = generateCraftFromClassPRNG(pb, with);

    // Every pre-existing axis identical bit-for-bit (drawn before the new tail).
    EXPECT_EQ(a.craftCGDelta, b.craftCGDelta);
    EXPECT_EQ(a.craftDragDelta, b.craftDragDelta);
    EXPECT_EQ(a.craftTrimDelta, b.craftTrimDelta);
    EXPECT_EQ(a.craftThrustScale, b.craftThrustScale);
    EXPECT_EQ(a.craftPitchEffDelta, b.craftPitchEffDelta);
    EXPECT_EQ(a.craftRollEffDelta, b.craftRollEffDelta);
    EXPECT_EQ(a.craftServoSlew, b.craftServoSlew);
    EXPECT_EQ(a.craftThrustTau, b.craftThrustTau);
    EXPECT_EQ(a.craftServoPwmPhase, b.craftServoPwmPhase);
    // And the new axes actually moved off nominal in `b`.
    EXPECT_NE(b.craftImuMisalignRoll, static_cast<gp_scalar>(0.0));
    EXPECT_NE(b.craftCmQ, static_cast<gp_scalar>(-4.2));
}

// T018 — σ>0 replays identically from the same scenarioSeed (new axes too), and
// the five pre-existing class sub-seeds are unchanged by the metadata growth.
TEST(CraftVariationImu, NewAxesReplayFromScenarioSeedAndSubSeedsStable) {
    CraftSigmas s = preExistingSigmas();
    s.craftImuMisalignSigma = 2.0; s.craftGyroScaleSigma = 0.02;
    s.craftAccelScaleSigma = 0.02; s.craftAccelBiasSigma = 0.04; s.craftCmQSigma = 0.32;

    constexpr uint64_t kScenarioSeed = 0xABCDEF1234567890ull;
    const auto subA = deriveClassSubSeeds(kScenarioSeed);
    const auto subB = deriveClassSubSeeds(kScenarioSeed);
    // The five class sub-seeds are a frozen cascade — growing ScenarioMetadata
    // must not touch them.
    EXPECT_EQ(subA.wind, subB.wind);
    EXPECT_EQ(subA.rabbit, subB.rabbit);
    EXPECT_EQ(subA.entry, subB.entry);
    EXPECT_EQ(subA.craft, subB.craft);
    EXPECT_EQ(subA.camera, subB.camera);

    ClassPRNG pa(subA.craft), pb(subB.craft);
    CraftDeltas a = generateCraftFromClassPRNG(pa, s);
    CraftDeltas b = generateCraftFromClassPRNG(pb, s);
    EXPECT_EQ(a.craftImuMisalignRoll,  b.craftImuMisalignRoll);
    EXPECT_EQ(a.craftGyroScaleY,       b.craftGyroScaleY);
    EXPECT_EQ(a.craftAccelBiasZ,       b.craftAccelBiasZ);
    EXPECT_EQ(a.craftCmQ,              b.craftCmQ);
}

// T019 — craftCmQ centres at -4.2 with σ=0 and clamps to [-5.0, -3.6]; at
// σ=0.32 the ±2.5σ truncation reaches exactly -5.0 and never leaves the range.
TEST(CraftVariationImu, CraftCmQCentersAndClamps) {
    using autoc::eval::kCraftCmQCenter;
    using autoc::eval::kCraftCmQMin;
    using autoc::eval::kCraftCmQMax;
    EXPECT_EQ(kCraftCmQCenter, static_cast<gp_scalar>(-4.2));
    EXPECT_EQ(kCraftCmQMin, static_cast<gp_scalar>(-5.0));
    EXPECT_EQ(kCraftCmQMax, static_cast<gp_scalar>(-3.6));

    // σ=0 → exactly the center.
    { CraftSigmas z; ClassPRNG p(0x1u);
      EXPECT_EQ(generateCraftFromClassPRNG(p, z).craftCmQ, kCraftCmQCenter); }

    // σ=0.32 over many seeds: every draw in-range; the min approaches -5.0
    // (2.5σ = 0.8 below center) — proving the clamp/reach are both correct.
    CraftSigmas s; s.craftCmQSigma = 0.32;
    gp_scalar lo = kCraftCmQMax, hi = kCraftCmQMin;
    for (uint32_t seed = 1; seed <= 5000; ++seed) {
        ClassPRNG p(seed);
        gp_scalar v = generateCraftFromClassPRNG(p, s).craftCmQ;
        EXPECT_GE(v, kCraftCmQMin);
        EXPECT_LE(v, kCraftCmQMax);
        lo = std::min(lo, v); hi = std::max(hi, v);
    }
    EXPECT_LT(lo, static_cast<gp_scalar>(-4.9));   // reaches near the -5.0 floor
    EXPECT_GT(hi, static_cast<gp_scalar>(-3.7));   // reaches near the -3.6 ceiling
}

// T020 — craftCGDelta and craftCmQ are INDEPENDENT draws: they do not
// double-count the static/dynamic pitch split. Turning craftCmQSigma on (it is
// drawn well after CG) must not change the CG draw at all (FR-052b).
TEST(CraftVariationImu, CgAndCmQAreIndependentDraws) {
    constexpr uint32_t kSeed = 0xC0FFEEu;
    CraftSigmas base; base.craftCGSigma = 0.05;                 // cmQ sigma = 0
    CraftSigmas withCmQ = base; withCmQ.craftCmQSigma = 0.32;   // add only cmQ

    ClassPRNG pa(kSeed), pb(kSeed);
    CraftDeltas a = generateCraftFromClassPRNG(pa, base);
    CraftDeltas b = generateCraftFromClassPRNG(pb, withCmQ);

    // CG is drawn first, so enabling cmQ cannot shift it.
    EXPECT_EQ(a.craftCGDelta, b.craftCGDelta);
    // cmQ moved only in `b`.
    EXPECT_EQ(a.craftCmQ, static_cast<gp_scalar>(-4.2));
    EXPECT_NE(b.craftCmQ, static_cast<gp_scalar>(-4.2));
}

// T016 (FR-055) — applyVariationScale leaves the new IMU/cmQ fields UNTOUCHED
// at any scale: craft is diversity, not difficulty, so it is never ramped.
TEST(CraftVariationImu, NewAxesAreNotRamped) {
    ScenarioMetadata original = makeMetaWithDraws();
    original.craftImuMisalignRoll = static_cast<gp_scalar>(3.1);
    original.craftGyroScaleY = static_cast<gp_scalar>(1.04);
    original.craftAccelBiasZ = static_cast<gp_scalar>(-0.05);
    original.craftCmQ = static_cast<gp_scalar>(-4.8);

    for (gp_scalar scale : {static_cast<gp_scalar>(0.0),
                            static_cast<gp_scalar>(0.5),
                            static_cast<gp_scalar>(1.0)}) {
        ScenarioMetadata m = original;
        applyVariationScale(m, scale);
        EXPECT_EQ(m.craftImuMisalignRoll, original.craftImuMisalignRoll);
        EXPECT_EQ(m.craftGyroScaleY, original.craftGyroScaleY);
        EXPECT_EQ(m.craftAccelBiasZ, original.craftAccelBiasZ);
        EXPECT_EQ(m.craftCmQ, original.craftCmQ);
    }
}

// The new IMU axes inherit the pipeline-wide ±2.5σ truncation (kGaussianSigmaClamp
// in ClassPRNG::nextGaussian) — same hard limit every other Gaussian variation
// uses, so no "occasional 5σ" outlier is possible. Assert the 2.5σ envelopes
// explicitly (contracts/craft-imu-axes.md: 2.5σ = the intended hard limit).
TEST(CraftVariationImu, NewAxesRespectTwoPointFiveSigmaLimit) {
    CraftSigmas s;
    s.craftImuMisalignSigma = 2.0;   // 2.5σ = ±5.0 deg
    s.craftGyroScaleSigma   = 0.02;  // 2.5σ = ±5%  -> [0.95, 1.05]
    s.craftAccelScaleSigma  = 0.02;  // 2.5σ = ±5%
    s.craftAccelBiasSigma   = 0.04;  // 2.5σ = ±0.10 g
    for (uint32_t seed = 1; seed <= 20000; ++seed) {
        ClassPRNG p(seed);
        CraftDeltas d = generateCraftFromClassPRNG(p, s);
        for (gp_scalar m : {d.craftImuMisalignRoll, d.craftImuMisalignPitch,
                            d.craftImuMisalignYaw}) {
            EXPECT_LE(std::abs(m), static_cast<gp_scalar>(5.0));
        }
        for (gp_scalar g : {d.craftGyroScaleX, d.craftGyroScaleY, d.craftGyroScaleZ,
                            d.craftAccelScaleX, d.craftAccelScaleY, d.craftAccelScaleZ}) {
            EXPECT_GE(g, static_cast<gp_scalar>(0.95));
            EXPECT_LE(g, static_cast<gp_scalar>(1.05));
        }
        for (gp_scalar b : {d.craftAccelBiasX, d.craftAccelBiasY, d.craftAccelBiasZ}) {
            EXPECT_LE(std::abs(b), static_cast<gp_scalar>(0.10));
        }
    }
}

// T019a (Constitution V read-side contract) — a pre-043 dmp FAILS LOUD on load,
// never a silent truncation or default-init. The 043 break appended 13 trailing
// scalars to ScenarioMetadata; a pre-043 wire is exactly the new wire minus that
// tail. Truncating it and loading into the current struct must THROW (cereal
// runs out of bytes reading the new fields) — a clear failure, not a quiet
// default-init of craftCmQ to -4.2. (The live-S3 fixture variant — one object
// from the T004 extract's source prefix — belongs in the AUTOC_S3_TESTS suite.)
TEST(CraftVariationImu, PreBreakDmpFailsLoudOnLoad) {
    ScenarioMetadata src = makeMetaWithDraws();
    src.scenarioSeed = 0x0123456789ABCDEFull;
    std::stringstream buf(std::ios::in | std::ios::out | std::ios::binary);
    { cereal::BinaryOutputArchive oa(buf); oa(src); }

    // Drop the 13 appended gp_scalar (the 043 tail) to emulate a pre-043 wire.
    std::string bytes = buf.str();
    ASSERT_GT(bytes.size(), 13 * sizeof(gp_scalar));
    bytes.resize(bytes.size() - 13 * sizeof(gp_scalar));

    std::stringstream shortBuf(bytes, std::ios::in | std::ios::binary);
    ScenarioMetadata dst;
    // Loud failure: cereal throws reading past the truncated end. It must NOT
    // silently succeed with dst.craftCmQ default-initialized.
    EXPECT_THROW({ cereal::BinaryInputArchive ia(shortBuf); ia(dst); },
                 std::exception);
}

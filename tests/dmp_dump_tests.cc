// 035 FR-P09 — zstd round-trip + ratio spike for the dmp serialization
// boundary. The contract (contracts/dmp-dump-cli.md "Guarantees") states the
// unit test IS the spike: it asserts decompressed == original byte-for-byte and
// reports the achieved ratio so level-19 can be retuned if it disappoints.
//
// End-to-end dmp-dump CSV/YAML output is exercised against a real dmp at the
// operator GATE (T028); this suite covers the compression primitive in
// isolation (no S3 / no EvalResults fixture needed).

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "autoc/util/zstd_io.h"

namespace {

// A blob with realistic structure: repeated floats (like a cereal trace) so
// zstd has something to exploit, plus some incompressible noise.
std::string makeBlob(size_t n) {
  std::string s;
  s.reserve(n);
  uint32_t x = 0x12345678u;
  for (size_t i = 0; i < n; ++i) {
    // ~75% structured (low entropy), ~25% LCG noise.
    if (i % 4 == 0) { x = x * 1664525u + 1013904223u; s.push_back(static_cast<char>(x >> 24)); }
    else            { s.push_back(static_cast<char>('A' + (i % 16))); }
  }
  return s;
}

}  // namespace

TEST(ZstdIo, RoundTripByteIdentical) {
  for (size_t n : {size_t{0}, size_t{1}, size_t{1024}, size_t{1 << 20}}) {
    const std::string orig = makeBlob(n);
    const std::string comp = autoc::zstdCompress(orig);
    const std::string back = autoc::zstdDecompress(comp);
    ASSERT_EQ(back.size(), orig.size()) << "size mismatch at n=" << n;
    EXPECT_EQ(back, orig) << "byte mismatch at n=" << n;
  }
}

TEST(ZstdIo, CompressesStructuredBlob) {
  const std::string orig = makeBlob(4 * 1024 * 1024);  // 4 MB
  const std::string comp = autoc::zstdCompress(orig);
  const double ratio = static_cast<double>(orig.size()) / comp.size();
  std::cout << "[zstd] level=" << autoc::kZstdLevel
            << " orig=" << orig.size() << " comp=" << comp.size()
            << " ratio=" << ratio << "x\n";
  EXPECT_LT(comp.size(), orig.size());  // must actually shrink
  EXPECT_GT(ratio, 1.5);                // structured data should compress well
}

TEST(ZstdIo, IsZstdKey) {
  EXPECT_TRUE(autoc::isZstdKey("run/gen9410.dmp.zst"));
  EXPECT_TRUE(autoc::isZstdKey("a.zst"));
  EXPECT_FALSE(autoc::isZstdKey("run/gen9410.dmp"));
  EXPECT_FALSE(autoc::isZstdKey("zst"));
  EXPECT_FALSE(autoc::isZstdKey(""));
}

TEST(ZstdIo, DecompressRejectsGarbage) {
  EXPECT_THROW(autoc::zstdDecompress("not a zstd frame at all"), std::runtime_error);
}

// ---------------------------------------------------------------------------
// 041 T011 (FR-017a) — body-frame specific force and the derived load factor.
//
// This is trap 3 of the feature: a real accelerometer measures SPECIFIC FORCE
// (a - g); the FDM's `acc` is `a` alone. Using the latter puts a constant ~1 g
// error into the most load-relevant axis, invisible in sim and wrong in flight.
// The single assertion that catches it is that steady LEVEL flight reads 1 g on
// the normal channel, not 0.
//
// The math is shared (autoc/eval/specific_force.h) between `dmp-dump --physics`
// (T010, the reader) and `gather_inputs` (T039, the NN input), so this suite
// pins the definition for both.
// ---------------------------------------------------------------------------

#include "autoc/eval/specific_force.h"

namespace {

// NED world, q_EB earth->body. Identity attitude = wings level, nose north.
constexpr gp_scalar kG = static_cast<gp_scalar>(9.80665);

gp_quat levelAttitude() { return gp_quat::Identity(); }

// Positive pitch θ (about body +Y) = nose up, per COORDINATE_CONVENTIONS.md.
gp_quat pitchedNoseUp(gp_scalar radians) {
    return gp_quat(Eigen::AngleAxis<gp_scalar>(radians, gp_vec3::UnitY()));
}

}  // namespace

TEST(SpecificForce, SteadyLevelFlightReadsOneG_NotZero_T011) {
    // Steady level flight: zero kinematic acceleration in the world frame.
    const gp_vec3 acc_world = gp_vec3::Zero();
    const auto sf = autoc::eval::bodySpecificForce(acc_world, levelAttitude(), kG);

    // THE assertion. Kinematic acceleration would give 0 here.
    EXPECT_NEAR(static_cast<double>(sf.load_factor_nz), 1.0, 1e-9)
        << "steady level flight must read +1 g on the normal channel; 0 means "
           "kinematic acceleration was used instead of specific force";

    // Body +z points DOWN while the measured reaction points UP, so the raw
    // component is -1 and the load factor is its negation. Both are pinned so a
    // future sign 'fix' has to break a test rather than a flight.
    EXPECT_NEAR(static_cast<double>(sf.g_units.z()), -1.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(sf.g_units.x()), 0.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(sf.g_units.y()), 0.0, 1e-9);
}

TEST(SpecificForce, PullUpRaisesLoadFactorAboveOne_T011) {
    // A pull-up accelerates the craft UPWARD, i.e. toward -z in NED.
    const gp_scalar a = static_cast<gp_scalar>(2.0) * kG;
    const gp_vec3 acc_world(0, 0, -a);
    const auto sf = autoc::eval::bodySpecificForce(acc_world, levelAttitude(), kG);

    // 1 g of gravity reaction + 2 g of manoeuvre = 3 g. The documented sign is
    // POSITIVE for a pull-up (this is the +11.2 g quoted in the flight reports).
    EXPECT_NEAR(static_cast<double>(sf.load_factor_nz), 3.0, 1e-9);
    EXPECT_GT(static_cast<double>(sf.load_factor_nz), 1.0);
}

TEST(SpecificForce, PushOverGivesNegativeLoadFactor_T011) {
    // Pushing over past free-fall: accelerating DOWNWARD (+z in NED) harder
    // than gravity alone. Free fall itself reads exactly 0 g.
    const auto freefall = autoc::eval::bodySpecificForce(
        gp_vec3(0, 0, kG), levelAttitude(), kG);
    EXPECT_NEAR(static_cast<double>(freefall.load_factor_nz), 0.0, 1e-9)
        << "free fall is the definition of zero specific force";

    const auto pushover = autoc::eval::bodySpecificForce(
        gp_vec3(0, 0, static_cast<gp_scalar>(2.0) * kG), levelAttitude(), kG);
    EXPECT_NEAR(static_cast<double>(pushover.load_factor_nz), -1.0, 1e-9);
}

TEST(SpecificForce, AttitudeRotatesGravityIntoTheLongitudinalAxis_T011) {
    // Nose up 90 degrees, stationary: the reaction points UP, which is now body
    // +x. This is the sim counterpart of the INAV bench table's "nose up 90 deg
    // -> [+2050, ~0, ~0]" row (docs/COORDINATE_CONVENTIONS.md), and it is what
    // makes the hardware convention check at T073 a comparison rather than a
    // fresh derivation.
    const gp_scalar quarter = static_cast<gp_scalar>(M_PI / 2.0);
    const auto sf = autoc::eval::bodySpecificForce(
        gp_vec3::Zero(), pitchedNoseUp(quarter), kG);

    EXPECT_NEAR(static_cast<double>(sf.g_units.x()), 1.0, 1e-6);
    EXPECT_NEAR(static_cast<double>(sf.g_units.y()), 0.0, 1e-6);
    EXPECT_NEAR(static_cast<double>(sf.g_units.z()), 0.0, 1e-6);
    // Normal channel is unloaded in this attitude — the load has moved axes.
    EXPECT_NEAR(static_cast<double>(sf.load_factor_nz), 0.0, 1e-6);
}

TEST(SpecificForce, NonPositiveGravityReturnsZeroRatherThanGuessing_T011) {
    // A dmp with no recorded gravity has no defensible answer. Returning zeros
    // is visible in a report; inventing 9.81 would silently rescale every load
    // column by the ratio of the true value to the guess.
    const auto sf = autoc::eval::bodySpecificForce(
        gp_vec3(1, 2, 3), levelAttitude(), static_cast<gp_scalar>(0));
    EXPECT_EQ(static_cast<double>(sf.load_factor_nz), 0.0);
    EXPECT_EQ(static_cast<double>(sf.g_units.norm()), 0.0);
}

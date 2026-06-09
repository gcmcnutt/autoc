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

#pragma once
// 035 FR-P09 — zstd compress/inflate at the dmp serialization boundary.
// Header-only (inline) so no new build unit; consumers link libzstd transitively
// via autoc_common. One-shot ZSTD_compress/ZSTD_decompress over an in-memory
// cereal blob. Round-trip byte-equality is the gate (tests/dmp_dump_tests.cc).

#include <string>
#include <stdexcept>
#include <zstd.h>

namespace autoc {

// Default compression level (R4: zstd-19 ≈ xz ratio at far higher throughput;
// one elite dmp per gen, so compress latency is negligible vs a multi-min gen).
constexpr int kZstdLevel = 19;

inline std::string zstdCompress(const std::string& in, int level = kZstdLevel) {
    const size_t bound = ZSTD_compressBound(in.size());
    std::string out(bound, '\0');
    const size_t n = ZSTD_compress(out.data(), bound, in.data(), in.size(), level);
    if (ZSTD_isError(n)) {
        throw std::runtime_error(std::string("zstd compress failed: ") + ZSTD_getErrorName(n));
    }
    out.resize(n);
    return out;
}

inline std::string zstdDecompress(const std::string& in) {
    const unsigned long long sz = ZSTD_getFrameContentSize(in.data(), in.size());
    if (sz == ZSTD_CONTENTSIZE_ERROR) {
        throw std::runtime_error("zstd decompress: input is not a valid zstd frame");
    }
    if (sz == ZSTD_CONTENTSIZE_UNKNOWN) {
        throw std::runtime_error("zstd decompress: unknown frame content size");
    }
    std::string out(static_cast<size_t>(sz), '\0');
    const size_t n = ZSTD_decompress(out.data(), out.size(), in.data(), in.size());
    if (ZSTD_isError(n)) {
        throw std::runtime_error(std::string("zstd decompress failed: ") + ZSTD_getErrorName(n));
    }
    out.resize(n);
    return out;
}

// True if an S3 key / filename names a compressed dmp.
inline bool isZstdKey(const std::string& key) {
    return key.size() >= 4 && key.compare(key.size() - 4, 4, ".zst") == 0;
}

}  // namespace autoc

// 035 FR-P07 / FR-P07b — shared S3 run-selector implementation.
#include "autoc/util/s3_run_selector.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <unistd.h>

#include <aws/core/Aws.h>
#include <aws/core/utils/memory/stl/AWSStringStream.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>
#include <aws/s3/model/ListObjectsV2Request.h>
#include <aws/s3/model/PutObjectRequest.h>

#include "autoc/util/zstd_io.h"  // FR-P09 compress/inflate primitive

namespace autoc {

namespace {
constexpr int kGenEncodingBase = 10000;          // key encodes (10000 - actualGen)
const std::string kRunPrefix = "autoc-";          // uniform prefix (FR-P07b; tracker- retired)
}  // namespace

int extractGenNumber(const std::string& key) {
    // "<run-id>/gen<N>.dmp" or "<run-id>/gen<N>.dmp.zst"
    static const std::regex pat(R"(gen(\d+)\.dmp(?:\.zst)?$)");
    std::smatch m;
    if (std::regex_search(key, m, pat) && m.size() > 1) {
        return kGenEncodingBase - std::stoi(m[1].str());
    }
    return -1;
}

std::string pickLatestRunPrefix(const std::vector<std::string>& commonPrefixes) {
    // Reverse-time encoding: run-id number = INT64_MAX - ms, so a NEWER run has a
    // SMALLER number ⇒ lexicographically-SMALLEST "autoc-" run-id = newest (this
    // is why the old nnextractor took the first/least-sorted common-prefix).
    // Filter to the uniform "autoc-" prefix; the bucket is the mode discriminator.
    std::string best;
    for (const auto& p : commonPrefixes) {
        if (p.rfind(kRunPrefix, 0) != 0) continue;  // not an "autoc-…" run
        if (best.empty() || p < best) best = p;
    }
    return best;
}

std::string pickLatestGenKey(const std::vector<std::string>& keys) {
    std::string best;
    int bestGen = -1;
    for (const auto& k : keys) {
        const int g = extractGenNumber(k);
        if (g > bestGen) { bestGen = g; best = k; }
    }
    return best;
}

namespace {
// Collect either common-prefixes (delimiter set) or object keys (no delimiter)
// across all pages of a ListObjectsV2 over `bucket`/`prefix`.
std::vector<std::string> listAll(const Aws::S3::S3Client& s3, const std::string& bucket,
                                 const std::string& prefix, bool wantCommonPrefixes) {
    std::vector<std::string> out;
    Aws::S3::Model::ListObjectsV2Request req;
    req.SetBucket(bucket);
    req.SetPrefix(prefix);
    if (wantCommonPrefixes) req.SetDelimiter("/");
    bool truncated = false;
    do {
        auto outcome = s3.ListObjectsV2(req);
        if (!outcome.IsSuccess()) {
            throw std::runtime_error("S3 ListObjectsV2 failed on bucket '" + bucket +
                                     "' prefix '" + prefix + "': " +
                                     outcome.GetError().GetMessage());
        }
        const auto& res = outcome.GetResult();
        if (wantCommonPrefixes) {
            for (const auto& cp : res.GetCommonPrefixes()) out.push_back(cp.GetPrefix());
        } else {
            for (const auto& obj : res.GetContents()) out.push_back(obj.GetKey());
        }
        truncated = res.GetIsTruncated();
        if (truncated) req.SetContinuationToken(res.GetNextContinuationToken());
    } while (truncated);
    return out;
}
}  // namespace

std::string findLatestRun(const Aws::S3::S3Client& s3, const std::string& bucket) {
    const auto prefixes = listAll(s3, bucket, kRunPrefix, /*wantCommonPrefixes=*/true);
    const std::string latest = pickLatestRunPrefix(prefixes);
    if (latest.empty()) {
        throw std::runtime_error("No '" + kRunPrefix + "' runs found in S3 bucket: " + bucket);
    }
    return latest;
}

std::string findLatestGenKey(const Aws::S3::S3Client& s3, const std::string& bucket,
                             const std::string& runPrefix) {
    const auto keys = listAll(s3, bucket, runPrefix, /*wantCommonPrefixes=*/false);
    const std::string latest = pickLatestGenKey(keys);
    if (latest.empty()) {
        throw std::runtime_error("No gen dmp found under run prefix: " + runPrefix +
                                 " (bucket " + bucket + ")");
    }
    return latest;
}

std::vector<std::string> listRunGenKeys(const Aws::S3::S3Client& s3, const std::string& bucket,
                                        const std::string& runPrefix) {
    const auto keys = listAll(s3, bucket, runPrefix, /*wantCommonPrefixes=*/false);
    std::vector<std::pair<int, std::string>> g;
    for (const auto& k : keys) {
        const int n = extractGenNumber(k);
        if (n >= 0) g.emplace_back(n, k);
    }
    std::sort(g.begin(), g.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<std::string> out;
    out.reserve(g.size());
    for (const auto& p : g) out.push_back(p.second);
    return out;
}

// --- 045 F1: local dmp cache -------------------------------------------------

namespace {
namespace fs = std::filesystem;

std::string inflateIfZstd(const std::string& key, const std::string& raw) {
    return isZstdKey(key) ? zstdDecompress(raw) : raw;  // FR-P09 inflate legacy-safe
}

// Write `bytes` to `p` atomically: a uniquely named tmp file in the same dir, then
// rename, so a concurrent reader never sees a partial entry.
void dmpCacheWriteAtomic(const fs::path& p, const std::string& bytes) {
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    if (ec) {
        throw std::runtime_error("dmp cache: cannot create " + p.parent_path().string() + ": " +
                                 ec.message());
    }
    static std::atomic<unsigned> seq{0};
    const fs::path tmp = p.string() + ".tmp." + std::to_string(::getpid()) + "." +
                         std::to_string(seq.fetch_add(1));
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.close();
        if (!out) {
            fs::remove(tmp, ec);
            throw std::runtime_error("dmp cache: cannot write " + tmp.string());
        }
    }
    fs::rename(tmp, p, ec);
    if (ec) {
        std::error_code ignore;
        fs::remove(tmp, ignore);
        throw std::runtime_error("dmp cache: cannot rename " + tmp.string() + " -> " + p.string() +
                                 ": " + ec.message());
    }
}
}  // namespace

DmpCacheConfig dmpCacheConfigFromEnvValue(const char* value) {
    DmpCacheConfig c;
    if (value != nullptr) c.dir = value;
    return c;
}

const DmpCacheConfig& dmpCacheConfigFromEnv() {
    static const DmpCacheConfig cfg = [] {
        DmpCacheConfig c = dmpCacheConfigFromEnvValue(std::getenv("AUTOC_DMP_CACHE"));
        if (!c.enabled()) {
            std::cerr << "dmp cache: disabled (AUTOC_DMP_CACHE unset)\n";
            return c;
        }
        std::error_code ec;
        fs::create_directories(c.dir, ec);
        if (ec) {
            throw std::runtime_error("dmp cache: AUTOC_DMP_CACHE=" + c.dir +
                                     " cannot be created: " + ec.message());
        }
        std::cerr << "dmp cache: " << c.dir << "\n";
        return c;
    }();
    return cfg;
}

std::string dmpCachePath(const DmpCacheConfig& cfg, const std::string& bucket,
                         const std::string& key) {
    if (key.empty() || key.front() == '/' || key.find("..") != std::string::npos ||
        bucket.empty() || bucket.find('/') != std::string::npos) {
        throw std::runtime_error("dmp cache: refusing unsafe bucket/key: " + bucket + "/" + key);
    }
    return (fs::path(cfg.dir) / bucket / key).string();
}

std::string dmpFetchBlobCached(const DmpCacheConfig& cfg, const std::string& bucket,
                               const std::string& key,
                               const std::function<std::string()>& fetchRaw) {
    if (!cfg.enabled()) return inflateIfZstd(key, fetchRaw());

    const fs::path p = dmpCachePath(cfg, bucket, key);
    std::error_code ec;
    if (fs::is_regular_file(p, ec)) {
        std::ifstream in(p, std::ios::binary);
        if (!in) throw std::runtime_error("dmp cache: cannot read " + p.string());
        const std::string raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        try {
            return inflateIfZstd(key, raw);
        } catch (const std::exception& e) {
            throw std::runtime_error("dmp cache: corrupt entry " + p.string() + " (" + e.what() +
                                     "); delete it to re-fetch from s3://" + bucket + "/" + key);
        }
    }

    const std::string raw = fetchRaw();
    std::string blob;
    try {
        blob = inflateIfZstd(key, raw);  // validate BEFORE caching: a bad object never lands
    } catch (const std::exception& e) {
        throw std::runtime_error("dmp fetch: s3://" + bucket + "/" + key + " failed to inflate (" +
                                 e.what() + ")");
    }
    dmpCacheWriteAtomic(p, raw);
    return blob;
}

std::string s3GetDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                         const std::string& key) {
    return dmpFetchBlobCached(dmpCacheConfigFromEnv(), bucket, key, [&]() -> std::string {
        Aws::S3::Model::GetObjectRequest req;
        req.SetBucket(bucket);
        req.SetKey(key);
        auto outcome = s3.GetObject(req);
        if (!outcome.IsSuccess()) {
            throw std::runtime_error("S3 GetObject failed for s3://" + bucket + "/" + key +
                                     ": " + outcome.GetError().GetMessage());
        }
        std::ostringstream oss;
        oss << outcome.GetResult().GetBody().rdbuf();
        return oss.str();
    });
}

void s3PutDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                  const std::string& key, const std::string& blob) {
    auto stream = Aws::MakeShared<Aws::StringStream>("s3PutDmpBlob");
    *stream << zstdCompress(blob);  // FR-P09
    Aws::S3::Model::PutObjectRequest req;
    req.SetBucket(bucket);
    req.SetKey(key);
    req.SetBody(stream);
    req.SetTagging("retain=expire");  // FR-P10 — ephemeral; bucket lifecycle reaps it
    auto outcome = s3.PutObject(req);
    if (!outcome.IsSuccess()) {
        throw std::runtime_error("S3 PutObject failed for s3://" + bucket + "/" + key +
                                 ": " + outcome.GetError().GetMessage());
    }
}

}  // namespace autoc

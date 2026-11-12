#pragma once
// 035 FR-P07 / FR-P07b — one shared S3 run-selector replacing the duplicated,
// autoc--hardcoded logic in tools/nnextractor.cc + tools/renderer.cc.
// See specs/035-energy-lexicase-objective/contracts/s3-selector.md.
//
// Bucket-relative + uniform-prefix ("autoc-"): the *bucket* is the mode
// discriminator (M1/M2/eval each own bucket), so the same code finds runs in
// any bucket. The `tracker-` run-id prefix (T033) is retired.

#include <functional>
#include <string>
#include <vector>

// Forward-declare so this header pulls NO AWS includes — pure-logic consumers
// (the unit tests) compile without the AWS SDK on their include path.
namespace Aws { namespace S3 { class S3Client; } }

namespace autoc {

// Parse the actual generation number out of a dmp key, inverting the 10000-N
// reverse-sort encoding. Accepts "<run-id>/gen<N>.dmp" and ".dmp.zst".
// Returns actualGen = 10000 - N, or -1 if the key doesn't match.
// (Fixes the renderer bug where it returned raw N; nnextractor was correct.)
int extractGenNumber(const std::string& key);

// --- pure pickers (exposed for testing over synthetic lists) ----------------

// Latest run prefix among S3 common-prefixes: the lexicographically-SMALLEST
// "autoc-"-prefixed run-id (reverse-time encoding INT64_MAX-ms ⇒ newer run has a
// smaller id ⇒ smallest string = newest). Non-"autoc-" entries (legacy
// "tracker-", etc.) are ignored. "" if none.
std::string pickLatestRunPrefix(const std::vector<std::string>& commonPrefixes);

// Key of the highest actualGen among dmp keys under a run. "" if none match.
std::string pickLatestGenKey(const std::vector<std::string>& keys);

// --- AWS wrappers (defined in the .cc, which includes the SDK) --------------

// List runs under `bucket`; return the latest run-id prefix (trailing '/').
// Throws std::runtime_error on S3 error or empty bucket (fail-loud, FR-P07).
std::string findLatestRun(const Aws::S3::S3Client& s3, const std::string& bucket);

// Within `runPrefix`, return the key of the latest-gen dmp.
// Throws std::runtime_error on S3 error or no dmp under the prefix (fail-loud).
std::string findLatestGenKey(const Aws::S3::S3Client& s3, const std::string& bucket,
                             const std::string& runPrefix);

// All gen-dmp keys under `runPrefix`, sorted ASCENDING by actualGen (gen 1,2,…).
// For per-gen run-summary analytics straight off the dmps. Empty if none.
std::vector<std::string> listRunGenKeys(const Aws::S3::S3Client& s3, const std::string& bucket,
                                        const std::string& runPrefix);

// --- compressed dmp I/O (FR-P09/P10) — ONE shared path for every tool --------
// Compress/inflate + tag live here so autoc.cc, source_dmp_loader, nnextractor,
// and renderer all leverage the same code (no per-site zstd/get/put boilerplate).

// --- 045 F1: local dmp cache (specs/045-m2-tracking/contracts/dmp-cache.md) --
// A read-through cache in front of the archive, so reports on the training box
// stop re-downloading dmps (S3 egress). S3 stays the record. An entry holds the
// bytes exactly as stored in S3 (compressed for ".zst" keys) at
// <dir>/<bucket>/<key>. Enabled only by AUTOC_DMP_CACHE. A cache that cannot be
// written, or an entry that fails to inflate, throws naming the path; there is
// no silent fall-back to S3 (Constitution VII). Keys are unique per run + gen
// and never rewritten, so there is no invalidation.
struct DmpCacheConfig {
    std::string dir;  // empty ⇒ disabled ({} is the "off" value)
    bool enabled() const { return !dir.empty(); }
};

// Parse the value of AUTOC_DMP_CACHE: nullptr or "" ⇒ disabled.
DmpCacheConfig dmpCacheConfigFromEnvValue(const char* value);

// The process-wide config: reads AUTOC_DMP_CACHE once and logs it once to stderr
// ("dmp cache: <dir>" or "dmp cache: disabled (AUTOC_DMP_CACHE unset)"). stderr,
// because the reports parse the tools' stdout. A set but uncreatable dir throws.
const DmpCacheConfig& dmpCacheConfigFromEnv();

// <dir>/<bucket>/<key>. Throws on a key that is absolute or contains "..".
std::string dmpCachePath(const DmpCacheConfig& cfg, const std::string& bucket,
                         const std::string& key);

// Read-through fetch of one dmp, returning the inflated cereal blob.
// Disabled ⇒ fetchRaw() every time. Hit ⇒ read the entry (corrupt ⇒ throw naming
// the path; fetchRaw() is not called). Miss ⇒ fetchRaw() once, inflate it (a bad
// archive object throws and is NOT cached), then store it (tmp + rename) and
// return it. `fetchRaw` returns the object's bytes as stored in S3.
std::string dmpFetchBlobCached(const DmpCacheConfig& cfg, const std::string& bucket,
                               const std::string& key,
                               const std::function<std::string()>& fetchRaw);

// Download bucket/key and return the raw cereal blob, inflating if key ends
// ".zst" (FR-P09), through the local dmp cache when AUTOC_DMP_CACHE is set (045 F1).
// Fail-loud: throws std::runtime_error on S3 error (read path) and on cache errors.
std::string s3GetDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                         const std::string& key);

// zstd-compress `blob` and upload to bucket/key (key should end ".dmp.zst"),
// always tagging the object retain=expire (FR-P10) so the bucket lifecycle can
// reap it. Needs IAM s3:PutObject + s3:PutObjectTagging (the tag rides the same
// PutObject). Fail-loud: throws std::runtime_error on any S3 error — no
// warn-and-continue (Constitution VII). Milestone runs are re-tagged
// retain=keep out-of-band.
void s3PutDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                  const std::string& key, const std::string& blob);

}  // namespace autoc

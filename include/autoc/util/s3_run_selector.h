#pragma once
// 035 FR-P07 / FR-P07b — one shared S3 run-selector replacing the duplicated,
// autoc--hardcoded logic in tools/nnextractor.cc + tools/renderer.cc.
// See specs/035-energy-lexicase-objective/contracts/s3-selector.md.
//
// Bucket-relative + uniform-prefix ("autoc-"): the *bucket* is the mode
// discriminator (M1/M2/eval each own bucket), so the same code finds runs in
// any bucket. The `tracker-` run-id prefix (T033) is retired.

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

// --- compressed dmp I/O (FR-P09/P10) — ONE shared path for every tool --------
// Compress/inflate + tag live here so autoc.cc, source_dmp_loader, nnextractor,
// and renderer all leverage the same code (no per-site zstd/get/put boilerplate).

// Download bucket/key and return the raw cereal blob, inflating if key ends
// ".zst" (FR-P09). Fail-loud: throws std::runtime_error on S3 error (read path).
std::string s3GetDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                         const std::string& key);

// zstd-compress `blob` and upload to bucket/key (key should end ".dmp.zst"),
// tagging the object retain=expire (FR-P10). Returns "" on success, else the
// error message — caller chooses warn-and-continue (training) vs fail.
std::string s3PutDmpBlob(const Aws::S3::S3Client& s3, const std::string& bucket,
                         const std::string& key, const std::string& blob);

}  // namespace autoc

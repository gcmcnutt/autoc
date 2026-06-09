# Contract — shared S3 run-selector (FR-P07 / FR-P07b)

One module (`src/util/s3_run_selector.{h,cc}`) replacing duplicated logic in
`tools/nnextractor.cc` (≈47,105) and `tools/renderer.cc` (≈1778,1898).

## API
```cpp
namespace autoc {
// Parse the gen number out of a dmp key. Inverts the 10000-N encoding.
// Accepts "<run-id>/gen<N>.dmp" and "<run-id>/gen<N>.dmp.zst".
// Returns actualGen = 10000 - N. Throws/returns sentinel on non-match.
int extractGenNumber(const std::string& key);

// List runs under `bucket`, return the lexicographically-latest run prefix
// (run-ids sort newest-first by construction: INT64_MAX - ms). Bucket-relative,
// prefix uses uniform "autoc-". Returns the run-id prefix (no trailing gen).
std::string findLatestRun(const Aws::S3::S3Client&, const std::string& bucket);

// Within a run prefix, return the key of the highest actualGen dmp (latest gen).
std::string findLatestGenKey(const Aws::S3::S3Client&, const std::string& bucket,
                             const std::string& runPrefix);
}
```

## Guarantees / behavior
- **`.zst`-aware:** matches both `gen<N>.dmp` and `gen<N>.dmp.zst`.
- **Invert bug fixed:** `extractGenNumber` returns `10000 - N` (renderer previously returned raw
  `N` — the bug this contract repairs; nnextractor was already correct). All callers now agree.
- **Prefix-agnostic / bucket-relative:** prefix filter is the uniform `"autoc-"` (FR-P07b retires
  the `tracker-` branch); the *bucket* distinguishes mode. Same code finds M1, M2, eval runs in
  their respective buckets.
- **Deterministic ordering:** "latest run" = max run-id string (reverse-time encoding);
  "latest gen" = max `actualGen`. No reliance on S3 LastModified.
- **Fail-loud:** empty bucket / no dmp under prefix → clear error, no silent "" return that a
  caller would treat as a valid key.

## Tests (`tests/s3_run_selector_tests.cc`)
- `extractGenNumber` inverts encoding for `.dmp` and `.dmp.zst`; rejects malformed keys.
- latest-run / latest-gen selection over a synthetic key list.
- uniform-prefix: a `tracker-`-named legacy key is NOT matched (documents the retirement);
  an `autoc-`-named key in any bucket is matched.

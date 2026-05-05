# Contract: Source M1 dmp loading (FR-001)

Replaces the prior `playback_file_format.md` contract (which described a `.crrclog` intermediate file format for the now-obsolete dmp-to-playback conversion tool — see [research.md R1](../research.md) for the FR-001 revision rationale).

## Surface

```cpp
namespace autoc::source {

struct SourceTickSample {
  double simTimeMsec;
  gp_vec3 position;
  gp_quat orientation;
  gp_vec3 velocity;
  gp_vec3 angularRate;
};

struct SourceScenarioTrajectory {
  int sourceScenarioIndex;
  ScenarioMetadata variation;
  std::vector<SourceTickSample> samples;
};

// Loads source dmp from S3 (or local disk if path is local).
// Throws on:
//   - S3 / local read failure
//   - Cereal version mismatch (Constitution V loud-fail rule)
//   - Validation failure (see Validation below)
std::vector<SourceScenarioTrajectory> loadSourceDmp(const std::string& s3_key_or_path);

// Optional subset selector per FR-011 (TrackerScenarioSubset).
std::vector<SourceScenarioTrajectory> filterByScenarioIndex(
    std::vector<SourceScenarioTrajectory>&& all,
    const std::vector<int>& scenario_indices);

}  // namespace autoc::source
```

## Source dmp accepted format

- Cereal-serialized `EvalResults` from a prior pathgen-mode autoc run (see `include/autoc/rpc/protocol.h:234-336`).
- Schema version: 1 (pre-FR-015a, "version-0 / pre-versioning"; treated as version 1 with the documented assumption that the existing field layout matches `CEREAL_CLASS_VERSION 1`) OR 2+ if the source is itself a tracker-mode dmp (forward-compat — tracker-mode dmps loaded as sources in future iterations of 030+).
- Required fields used: `aircraftStateList[scenario][tick]`, `scenarioList[scenario]`, `crashReasonList[scenario]`. Other fields ignored.

## Validation

Per `data-model.md §1`:
- `samples` non-empty; minimum tick count `MIN_SCENARIO_TICKS = 30` (3 seconds at 10 Hz). Rejects truncated / crashed source scenarios.
- `samples[i].simTimeMsec` strictly monotonically increasing.
- `orientation` quat magnitude in `[0.99, 1.01]` (allows minor numerical drift).
- `position` distance from origin < 10 km (sanity bound).
- Source dmp's `crashReasonList[i]` may be used to skip crashed source scenarios; v1 default behavior is "include if `samples.size() >= MIN_SCENARIO_TICKS`."

Validation failures throw with a clear message identifying the source scenario index + reason.

## S3 key format

Per FR-011 + R1 carry-forward:
- Pattern: `<profile-prefix>/<run-id>/gen<N>.dmp`
- Concrete example: `autoc-storage/autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9609.dmp`
- The same string round-trips with xiao log entries (no rewriting).
- Local-disk path is also accepted (autoc-tracker.ini may use either form).

## Determinism contract

Per FR-009: `loadSourceDmp(same_key)` produces an identical `vector<SourceScenarioTrajectory>` across runs. Cereal binary deserialization is deterministic; no PRNG / timestamp-dependent operations occur during load.

## Test coverage

`tests/source_dmp_loading_tests.cc` (M3 deliverable):
- Load-known-fixture-dmp test (pre-versioning layout, version-1-assumption path).
- Load-tracker-mode-dmp test (forward-compat, version-2 layout).
- Validation rejection tests: truncated scenario, non-monotonic timestamps, drifted quat magnitude.
- S3 key parser round-trip test.

## Citations

- `tools/nnextractor.cc:177-192` (existing cereal `EvalResults` deserialization pattern)
- `include/autoc/rpc/protocol.h:234-336` (source `EvalResults` schema)
- 030 spec FR-001 (load source dmp directly)
- 030 spec FR-009, FR-010, FR-011
- research.md R1 (in-memory load decision)
- research.md R8 (schema gap analysis)

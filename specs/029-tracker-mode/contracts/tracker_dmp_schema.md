# Contract: Tracker-mode `EvalResults` schema

**Producer**: tracker-mode `autoc` evaluation worker (`src/autoc.cc` + crrcsim FDM eval pipeline)
**Consumer**: renderer (`tools/renderer.cc`), analysis scripts, future ablation tooling

**Schema basis**: extends existing `EvalResults` ([`include/autoc/rpc/protocol.h:234`](../../../include/autoc/rpc/protocol.h)). Per FR-015 and [no-cereal-versioning policy](../../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md), this is a clean schema bump — old `.dmp` files become unloadable by tracker-aware tools, but pathgen-mode tools continue unchanged.

## Schema additions

```cpp
struct EvalResults {
    // ===== existing pathgen-mode fields, unchanged =====
    std::vector<char> gp;
    uint64_t gpHash = 0;
    std::vector<CrashReason> crashReasonList;
    std::vector<std::vector<Path>> pathList;
    std::vector<std::vector<AircraftState>> aircraftStateList;
    ScenarioMetadata scenario;
    std::vector<ScenarioMetadata> scenarioList;
    std::vector<std::vector<DebugSample>> debugSamples;
    std::vector<std::vector<PhysicsTraceEntry>> physicsTrace;
    int workerId = -1;
    int workerPid = 0;
    int workerEvalCounter = 0;

    // ===== 029 tracker-mode additions =====
    bool tracker_mode = false;       // schema discriminator; false for pathgen dumps
    std::vector<TrackerScenarioState> trackerScenarios;  // empty when tracker_mode == false

    template<class Archive>
    void serialize(Archive& ar, const std::uint32_t version) {
        ar(/* existing fields */, tracker_mode, trackerScenarios);
    }
    // ...
};
```

```cpp
struct TrackerScenarioState {
    std::string library_entry_path;       // e.g., "library/000.crrclog"
    std::string library_metadata_json;    // serialized library metadata for this scenario
    std::vector<TickCameraState> cameraStates;  // per-tick — same length as aircraftStateList[scenario]
};

struct TickCameraState {
    Vector3f camera_pos_world;
    Quaternionf camera_orientation_world;
    int camera_config_index;              // index into a per-dump array of CameraConfig instances
                                          // (most runs have 1 config; multi-camera variants have N)
    BeaconProjectionResult beacon_left;   // see beacon_projection_api.md
    BeaconProjectionResult beacon_right;
};
```

## Validation rules

When `tracker_mode == false` (pathgen-mode dump):
- `trackerScenarios` MUST be empty
- All existing pathgen-mode fields MUST be populated as today

When `tracker_mode == true` (tracker-mode dump):
- `trackerScenarios.size() == aircraftStateList.size()` (one tracker-state per scenario)
- `trackerScenarios[s].cameraStates.size() == aircraftStateList[s].size()` (one per-tick state per aircraft tick)
- `library_entry_path` MUST be non-empty
- `library_metadata_json` SHOULD parse as valid JSON matching the schema in [data-model.md §2.2](../data-model.md)

## Backward compatibility — CLEAN CUT

Per [no-cereal-versioning policy](../../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md): this is a **clean schema bump, no compatibility shims**.

**Important — cereal does NOT skip unknown fields automatically.** The version bump (`CEREAL_CLASS_VERSION(EvalResults, 1) → 2`) plus the additional `tracker_mode` / `trackerScenarios` fields means:

- **Pre-029 binaries reading 029-produced dumps**: fail at deserialization. The new fields trip the version check and the missing-from-old-struct fields would corrupt the stream. No mitigation in code; operator must rebuild tooling against the 029 schema.
- **029 binaries reading pre-029 dumps**: fail at deserialization (version mismatch). The bump is intentionally non-shim-able. Pre-029 dumps remain readable only by pre-029 binaries.
- **No shared dumps across the boundary**: 029 ships with a unified schema where every dump (whether pathgen or tracker mode) carries the v2 layout. The `tracker_mode` flag distinguishes contents within the unified schema; the version bump distinguishes pre-029 from 029-onward.

This matches the project policy of "greenfield schema changes only, no backward compat" — old `.dmp` archives stay readable by old tooling, all new tooling reads the new schema, and there is no version-conditional code path to maintain.

## Renderer consumer behavior

- Renderer detects `tracker_mode == true` flag at load time
- If true: enables tracker-mode views (3rd-person dual-aircraft + 1st-person camera-POV per FR-012). Loads the corresponding library entry from `library_entry_path` for target craft visualization.
- If false: existing pathgen-mode rendering (rabbit + path).

## Test surface

`tests/tracker_mode_integration_tests.cc` (NEW for 029):

| Test | Assertion |
|---|---|
| `SchemaRoundtrip_TrackerMode` | Construct an `EvalResults` with `tracker_mode = true` + populated `trackerScenarios`. Cereal-serialize, deserialize, compare. All fields preserved. |
| `SchemaRoundtrip_PathgenMode` | Same but with `tracker_mode = false`. Existing fields preserved; `trackerScenarios` empty. |
| `MixedFleetTooling_PathgenReadsNewSchema` | Construct an `EvalResults` from a tracker-mode run. Existing `nnextractor` reads the existing fields without error (the new fields land but are ignored). |
| `RendererDispatch_TrackerFlag` | Construct both a tracker-mode and pathgen-mode dump. Renderer correctly dispatches to dual-aircraft view vs path view based on `tracker_mode` flag. |

## Open contract decisions

1. **camera_config_index granularity**: per-tick or per-scenario? v1: per-scenario (camera config doesn't change mid-scenario). Possible future: per-tick if config can be PRNG-varied within a scenario. Pin v1 in implementation.
2. **library_metadata_json deduplication**: 245 scenarios × identical run-level metadata = redundant data. Could store run-level metadata once in `EvalResults` and per-scenario provenance separately. Decide based on dump size impact (likely negligible vs the per-tick aircraft state).

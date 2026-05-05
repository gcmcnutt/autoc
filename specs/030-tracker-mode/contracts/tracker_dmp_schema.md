# Contract: Tracker-mode `EvalResults` schema (FR-015 + FR-015a + Constitution V)

**Producer**: tracker-mode `autoc` evaluation worker (`src/autoc.cc` + crrcsim FDM pipeline)
**Consumer**: renderer (`tools/renderer.cc` — M9), per-tick dmp extractor (`tools/aircraft_state_extractor.cc` — M11a), analytics scripts

Refreshed 2026-05-04: includes FR-015a versioning per Constitution V, and FR-015's two embedded classes (camera view + copied target trajectory).

## Schema additions to `EvalResults` (cereal class version 2)

```cpp
// include/autoc/rpc/protocol.h additions

namespace autoc::eval {

struct CameraViewSample {
  // Camera pose at this tick (world frame; for renderer's 1st-person mode + reverse-projection)
  gp_vec3 camera_pose_world_pos;
  gp_quat camera_pose_world_orient;
  float camera_fov_h_deg;
  float camera_fov_v_deg;
  // Per-beacon observations (2 beacons × 1 camera = 2 in v1)
  BeaconObservation beacon_left;
  BeaconObservation beacon_right;

  template <class Archive>
  void serialize(Archive& ar) {
    ar(camera_pose_world_pos, camera_pose_world_orient,
       camera_fov_h_deg, camera_fov_v_deg,
       beacon_left, beacon_right);
  }
};

struct CopiedTargetSample {
  // Copied from the source-scenario target trajectory at the matching sim_time
  // (FR-015 self-containedness: M2 dmp doesn't reference M1 source dmp at playback)
  gp_vec3 position;
  gp_quat orientation;
  gp_vec3 velocity;
  // Per-tick fitness state:
  gp_vec3 trail_rabbit_position;
  bool inside_crash_hull;
  bool used_nose_trail_fallback;  // R10 telemetry

  template <class Archive>
  void serialize(Archive& ar) {
    ar(position, orientation, velocity,
       trail_rabbit_position, inside_crash_hull, used_nose_trail_fallback);
  }
};

}  // namespace autoc::eval

// Existing EvalResults extended:

struct EvalResults {
  // === Existing fields (cereal version 1, unchanged) ===
  /* gp, gpHash, crashReasonList, pathList, aircraftStateList,
     scenario, scenarioList, debugSamples, physicsTrace, ... */

  // === New tracker-mode fields (cereal version 2) ===
  std::vector<std::vector<CameraViewSample>> cameraViewList;        // [scenario][tick]
  std::vector<std::vector<CopiedTargetSample>> targetTrajectoryList; // [scenario][tick]
  // Telemetry (per-scenario aggregates):
  std::vector<int> arenaEgressCount;       // [scenario] — #ticks egressed (for telemetry)
  std::vector<int> hullStrikeCount;        // [scenario] — #p_crash fires per scenario

  template <class Archive>
  void serialize(Archive& ar, std::uint32_t const version) {
    // version 1 (pathgen) and version 2 (tracker) share the existing fields:
    ar(gp, gpHash, crashReasonList, pathList, aircraftStateList,
       scenario, scenarioList, debugSamples, physicsTrace, /* ... */);
    if (version >= 2) {
      ar(cameraViewList, targetTrajectoryList,
         arenaEgressCount, hullStrikeCount);
    }
  }
};

CEREAL_CLASS_VERSION(EvalResults, 2);  // bumped from 1 in M1 (FR-015a groundwork) → committed at M8
```

## Read-side back-compat (FR-015a + Constitution V)

```cpp
EvalResults loadDmp(const std::string& s3_key_or_path) {
  std::ifstream f(...);
  cereal::BinaryInputArchive ar(f);
  EvalResults result;
  try {
    ar(result);  // cereal handles per-version field deserialization
  } catch (cereal::Exception& e) {
    throw std::runtime_error(
      "Failed to load dmp '" + s3_key_or_path + "': " + e.what()
      + " (Constitution V: dmp readers must fail loudly on schema mismatch)");
  }
  // ... validation
  return result;
}
```

**Pre-versioning dmps**: dmps written before the FR-015a version-field add (M1) are treated as cereal class version 1 (the documented pre-versioning assumption). Cereal's serialize-with-version automatically handles this — version-1 dmps deserialize without the new tracker fields.

**Forward incompatibility**: if a future cereal class version 3 dmp is read by a v2-only reader, cereal will throw — matching Constitution V's loud-fail rule. No silent truncation.

## Self-containedness property (FR-015 + D13)

The M2 dmp's `targetTrajectoryList` is a *copy* of the source-scenario target trajectory data the M2 run consumed at training time. The renderer (M9) reads `targetTrajectoryList` directly to render the target craft + beacons in 3rd-person view — never reaches into the M1 source dmp at playback.

**Implication for analytics**: an analyst given only the M2 dmp can fully replay the run. No external M1 dmp dependency. Same property pathgen-mode dmps have today (path geometry recoverable from path-name + scenario seed).

## Storage budget

Per-tick per-scenario in version-2 fields: roughly
- `CameraViewSample`: ~20 floats = 80 bytes
- `CopiedTargetSample`: ~15 floats + 2 bytes = ~62 bytes
- Total: ~150 bytes/tick

Per-scenario at 30s × 10 Hz: ~45 KB.
Per-gen at 245 scenarios: ~11 MB additional vs pathgen-mode.
Acceptable; cereal binary is reasonably efficient.

## Determinism (FR-009)

Same input scenario + same NN weights + same `autoc-tracker.ini` ⇒ bit-identical M2 dmp output. All per-tick PRNG draws (e.g., `p_crash`) use seeded subsequences derived from `ScenarioMetadata.scenarioSequence`.

## Test coverage

`tests/tracker_dmp_roundtrip_tests.cc` (M8 deliverable):
- Serialize / deserialize identity for a synthetic version-2 dmp (small fixture).
- Pre-versioning dmp (a fixture pathgen-mode v1 dmp) loads correctly with new fields empty.
- Version mismatch (synthesized v3 read by v2-aware reader) throws cleanly.
- Self-containedness: a renderer-mock loading only the M2 dmp produces correctly-rendered scene state without M1 source.

## Citations

- `include/autoc/rpc/protocol.h:234-336` (existing `EvalResults` schema)
- 030 spec FR-015 (two embedded classes)
- 030 spec FR-015a (versioning per Constitution V)
- 030 spec D13 (self-containedness clarification)
- [Constitution V](../../../.specify/memory/constitution.md) — versioned persistence artifacts
- research.md R8 (no source-side schema change required for v1)
- research.md R9 (renderer reads from M2 dmp, not M1)

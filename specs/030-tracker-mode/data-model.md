# 030 — Data Model

Entities, schemas, validation rules, and per-tick data flow for tracker mode. Anchored to [spec.md](./spec.md) FRs and the [research.md](./research.md) R1–R10 findings.

> **Heritage**: this is a fresh rewrite (2026-05-04) replacing the prior 029-era data model that was keyed off the obsolete `.crrclog` library / `(x, y, visible)` interface. Where prior content carries forward unchanged, citations note carry-forward; otherwise the prior version is archived as alternatives in research.md.

## 1. Source-scenario target trajectory (in-memory, no on-disk format)

**Per FR-001 + R1 + R8**: 030 v1 does NOT introduce a new on-disk file format for source target trajectories. Instead, the source M1 dmp's `EvalResults` is loaded directly at autoc startup; per-scenario target trajectories live in autoc memory.

**In-memory representation**:

```cpp
// include/autoc/eval/source_trajectory.h (illustrative)
struct SourceTickSample {
  double simTimeMsec;             // M1 source timestamp at this sample
  gp_vec3 position;               // target world position (NED, meters)
  gp_quat orientation;            // target body→world quat
  gp_vec3 velocity;               // target velocity (NED, m/s)
  gp_vec3 angularRate;            // target body rates (p, q, r, rad/s)
};

struct SourceScenarioTrajectory {
  int sourceScenarioIndex;        // index into source dmp's scenarioList
  ScenarioMetadata variation;     // copied from source dmp scenarioList[i]
  std::vector<SourceTickSample> samples;  // copied from aircraftStateList[i]
  // Validation: samples sorted by simTimeMsec; sample count >= MIN_SCENARIO_TICKS
};
```

**Source population**: at autoc startup, `loadSourceDmp(s3_key) → vector<SourceScenarioTrajectory>` reads the source `EvalResults` (cereal binary load, pattern follows `tools/nnextractor.cc:177-192`), iterates `aircraftStateList[scenario]` × `scenarioList[scenario]`, and produces one `SourceScenarioTrajectory` per source scenario. Optional filter via `TrackerScenarioSubset` per FR-011.

**Validation rules**:
- `samples` non-empty per scenario (`MIN_SCENARIO_TICKS = 30` — 3 seconds at 10 Hz; rejects truncated / crashed source scenarios from contributing useless data; gate by `crashReasonList[i]` from source dmp).
- `samples[i].simTimeMsec` monotonically increasing.
- `orientation` quat magnitude ≈ 1.0 (validates source data integrity — older dmps occasionally had non-unit quats from numerical drift).
- `position` reasonably bounded (sanity check: distance from origin < 10 km).

**Citations**:
- `include/autoc/rpc/protocol.h:234-336` (source `EvalResults`)
- `include/autoc/eval/aircraft_state.h:295-490` (per-tick `AircraftState` getters)
- research.md R8 (schema gap analysis — none for v1)

---

## 2. Tracker-mode autoc.ini schema

**Per FR-011**: new sibling config `autoc-tracker.ini`. Mutual exclusion with pathgen-mode: tracker-only fields, plus mode dispatch by config-file path.

**Schema sketch** (illustrative; concrete inih sections):

```ini
[Source]
TrackerSourceRun = autoc-storage/autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9609.dmp
TrackerScenarioSubset = 17       ; v1: single scenario index. List/range/slice deferred.

[TrackingFitness]
TrailDistance = 3.048             ; meters; 10 ft per FR-008
LowSpeedTrailThreshold = 2.0      ; m/s; below this → nose-trail per R10
LowSpeedTrailHysteresis = 0.5     ; m/s; ±0.5 around threshold

[CrashHull]
CrashHullShape = SPHERE           ; v1 only SPHERE supported
CrashHullRadius = 1.0             ; meters per FR-008b
PCrashGen0 = 0.0                  ; per R3 curriculum anneal
PCrashGenRamp = 100               ; gens 0-100 ramp
PCrashGenPlateau = 200            ; gens 200+ at plateau value
PCrashPlateau = 0.30

[Arena]
FlightArenaRadius = 80.0          ; m horizontal, per R2 default
FlightArenaFloorAGL = 5.0         ; m AGL hard floor
FlightArenaCeilingAGL = 100.0     ; m AGL ceiling

[Camera]                          ; FR-003 v1 baseline
CameraCount = 1
CameraFOVHorizontal = 120.0
CameraFOVVertical = 90.0
CameraFrameRateHz = 30
CameraLatencyMs = 0
CameraMountOffsetX = 0.0          ; on top of wing-chord; calibrate against airframe model
CameraMountOffsetY = 0.0
CameraMountOffsetZ = -0.05        ; m above wing surface (NED, +Z down)

[Beacon]                          ; FR-004 v1 baseline
BeaconLeftWavelength = 850        ; nm
BeaconRightWavelength = 940
BeaconEmissionConeDeg = 270
BeaconLeftMountX = 0.0            ; wingtip body-frame position; populate from airframe model
BeaconLeftMountY = -0.45          ; left wingtip
BeaconLeftMountZ = 0.0
BeaconRightMountX = 0.0
BeaconRightMountY = +0.45         ; right wingtip
BeaconRightMountZ = 0.0

[Population]
; ... reuses pathgen-mode population/seed/etc. fields unchanged
```

**Validation**: at autoc startup, parse + assert all required tracker fields present; reject if pathgen-only fields present. Loud failure on invalid combinations.

**Citations**:
- 030 spec FR-011 (autoc-tracker.ini structure + mutual exclusion)
- 030 spec D4 (autoc-tracker.ini detail)
- research.md R3 (`p_crash` curriculum schema)

---

## 3. Camera + beacon configuration entities

**Per FR-003 + FR-004**: structs in `include/autoc/eval/camera_config.h` + `beacon_config.h` populated from the autoc-tracker.ini. v1 baseline values committed; PRNG-varied dimensions architectural-only (sigmas at zero in v1 per D13).

```cpp
// camera_config.h
struct CameraConfig {
  // Compile-time-fixed (target-hardware spec)
  enum class Projection { PLANAR_PINHOLE, SPHERICAL_FISHEYE } projection = Projection::PLANAR_PINHOLE;
  float fov_h_deg = 120.0f;
  float fov_v_deg = 90.0f;
  float frame_rate_hz = 30.0f;
  float latency_ms = 0.0f;
  // Color filter / shutter (v1 stubs — interface in place)
  // ...

  // PRNG-varied per scenario (v1 sigmas at zero; populated from autoc-tracker.ini)
  gp_vec3 mount_offset_body{0, 0, -0.05};   // top-of-wing-chord
  gp_quat mount_orientation_body{1, 0, 0, 0};  // forward-aligned with body +x
  // ...
};

// beacon_config.h
struct BeaconConfig {
  uint16_t wavelength_nm;
  float emission_cone_deg = 270.0f;
  gp_vec3 mount_body;                         // wingtip body-frame position
  gp_vec3 emission_axis_body{0, 1, 0};        // outward; left = -y, right = +y
  // PRNG-varied infrastructure (v1 sigmas at zero — D1 / FR-004 variation hooks)
};
```

---

## 4. Beacon observation (per-tick perception output)

**Per FR-005 + FR-007 + FR-017**: the projection module's per-tick output for one (camera, beacon) pair.

```cpp
// camera_projection.h
struct BeaconObservation {
  // NN-facing dequantized values (fp32 at the boundary):
  float screen_x;       // [-1, +1] uncalibrated screen-relative; arbitrary if invisible
  float screen_y;       // [-1, +1] same
  float cep;            // [0, 1.0] for visible, == kCepSentinelFloat (1.5f) for invisible

  // For dmp / renderer (kept separate; FR-015 self-containedness):
  bool was_quantized;
  int8_t raw_x_int8;    // post-quantization storage
  int8_t raw_y_int8;
  int8_t raw_cep_int8;  // INT8_MIN ⇔ invisible
};
```

**Round-trip property** (per R7): `dequantize(quantize(x_in_minus_one_plus_one)) ≈ x` within 1/127.0 step. Sentinel: `dequantize_cep(INT8_MIN) == kCepSentinelFloat`. Tested in `beacon_projection_tests.cc` (M5).

---

## 5. NN inputs (typed, per FR-006 + R4)

```cpp
// nn_inputs.h (illustrative — exact enum order locked in M2)
enum class NNInput : uint16_t {
  // Beacon left × 6 history slots × 3 channels (x, y, cep) = 18
  BEACON_L_X_TM5,    BEACON_L_Y_TM5,    BEACON_L_CEP_TM5,
  BEACON_L_X_TM4,    BEACON_L_Y_TM4,    BEACON_L_CEP_TM4,
  BEACON_L_X_TM3,    BEACON_L_Y_TM3,    BEACON_L_CEP_TM3,
  BEACON_L_X_TM2,    BEACON_L_Y_TM2,    BEACON_L_CEP_TM2,
  BEACON_L_X_TM1,    BEACON_L_Y_TM1,    BEACON_L_CEP_TM1,
  BEACON_L_X_NOW,    BEACON_L_Y_NOW,    BEACON_L_CEP_NOW,
  // Beacon right (mirror, 18 more)
  BEACON_R_X_TM5,    /* ... */          BEACON_R_CEP_NOW,
  // Aircraft state (8)
  QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
  AIRSPEED,
  GYRO_P, GYRO_Q, GYRO_R,
  COUNT  // = 44 per spec FR-006
};

struct NNInputMeta {
  const char* name;
  float range_min;
  float range_max;
  const char* units;
};

inline constexpr NNInputMeta kNNInputMeta[static_cast<size_t>(NNInput::COUNT)] = { /* per-entry */ };

static_assert(static_cast<size_t>(NNInput::COUNT) == TRACKER_INPUT_COUNT,
              "NNInput enum count mismatch with topology");
```

**Total**: 36 beacon (2 × 6 × 3) + 8 aircraft state = **44 fp32 inputs** matching FR-006.

**Per-tick population**: the worker fills `inputs[NNInput::COUNT]` at each NN tick using the strategy:
- For each of 6 history slots `t ∈ {-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now}`: dequantize the recorded `BeaconObservation` from the slot's tick; assign to `inputs[NNInput::BEACON_L_X_T*]`, etc.
- Aircraft state from current chase craft pose.

**Citations**:
- 030 spec FR-006 (full files-to-touch list + naming)
- research.md R4 (enum+constexpr metadata pattern)

---

## 6. Trail rabbit (per-tick fitness target)

**Per FR-008 + FR-008a + R10**: computed each tick from the current target-craft sample.

```cpp
// trail_rabbit.h
struct TrailRabbit {
  gp_vec3 position;     // world (NED, meters) — fitness uses this as the rabbit point
  bool used_nose_trail; // true if degenerate-velocity branch fired (R10 fallback)
};

TrailRabbit computeTrailRabbit(const SourceTickSample& target_sample,
                                float trail_distance,
                                float low_speed_threshold,
                                float low_speed_hysteresis,
                                bool prev_was_nose_trail);
```

**Math**:
```
target_speed = |target_sample.velocity|

// Hysteretic switch (R10):
if (prev_was_nose_trail) {
  using_nose_trail = (target_speed < threshold + hysteresis);
} else {
  using_nose_trail = (target_speed < threshold - hysteresis);
}

if (using_nose_trail) {
  // Nose direction = body +x rotated to world by target's quat
  nose_world = target_sample.orientation * gp_vec3(1, 0, 0);
  rabbit = target_sample.position - nose_world * trail_distance;
} else {
  velocity_unit = target_sample.velocity / target_speed;
  rabbit = target_sample.position - velocity_unit * trail_distance;
}
```

**Validation**: `trail_distance >= 0.5m` (sanity); `low_speed_threshold >= 0.5m/s`.

**Citations**:
- 030 spec FR-008 + FR-008a
- research.md R10 (hard-switch with hysteresis at 2 m/s)

---

## 7. Crash hull (per-tick collision check)

**Per FR-008b + R3**: sphere intersection + probabilistic firing.

```cpp
// crash_hull.h
enum class CrashHullShape { SPHERE, AABB_HB1, MESH_AIRFRAME };

struct CrashHull {
  CrashHullShape shape = CrashHullShape::SPHERE;
  float sphere_radius = 1.0f;  // v1 default
};

bool isInsideHull(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const SourceTickSample& target_sample);

// Returns true if (a) inside the hull AND (b) p_crash random draw fires
bool didCrashFire(const CrashHull& hull,
                  const gp_vec3& chase_pos,
                  const SourceTickSample& target_sample,
                  float p_crash_this_tick,
                  PRNG& scenario_rng);
```

**`p_crash_this_tick` curriculum** (per R3):
```
gen = current generation index;
if (gen < gen0) → 0.0
elif (gen < genRamp) → linear interp from 0 to plateau
else → plateau

// Defaults: gen0 = 0, genRamp = 100, plateau = 0.30, plateau_starts = 200
```

**Mode gating**: only called from tracker-mode fitness path; pathgen-mode never invokes it.

**Citations**:
- 030 spec FR-008b
- research.md R3 (curriculum-anneal v1 default)

---

## 8. Tracker-mode dmp output schema (FR-015)

**Per FR-015 + FR-015a + Constitution V**: extended `EvalResults` with two new classes of per-tick data, version-bumped to `CEREAL_CLASS_VERSION(EvalResults, 2)`.

**Schema additions** (sketch):

```cpp
// rpc/protocol.h additions
struct CameraViewSample {
  // From the chase craft's perception module per FR-005 (one camera)
  gp_vec3 camera_pose_world_pos;
  gp_quat camera_pose_world_orient;
  float camera_fov_h_deg;
  float camera_fov_v_deg;
  // 2 beacons × 1 camera = 2 BeaconObservations per tick
  BeaconObservation beacon_left;
  BeaconObservation beacon_right;
};

struct CopiedTargetSample {
  // Copied from SourceScenarioTrajectory.samples[i] at the matching sim_time
  gp_vec3 position;
  gp_quat orientation;
  gp_vec3 velocity;
  gp_vec3 trail_rabbit_position;  // computed per FR-008
  bool inside_crash_hull;          // FR-008b telemetry
};

struct EvalResults {
  // Existing fields (unchanged from version 1):
  // gp, gpHash, crashReasonList, pathList, aircraftStateList, scenario, scenarioList, ...

  // New (version 2 = tracker mode):
  std::vector<std::vector<CameraViewSample>> cameraViewList;       // [scenario][tick]
  std::vector<std::vector<CopiedTargetSample>> targetTrajectoryList; // [scenario][tick]

  // FR-015a versioning: cereal CEREAL_CLASS_VERSION bumped 1 → 2
  template <class Archive>
  void serialize(Archive& ar, std::uint32_t const version) {
    // version 1: existing fields
    ar(gp, gpHash, ...);
    if (version >= 2) {
      ar(cameraViewList, targetTrajectoryList);
    }
  }
};
CEREAL_CLASS_VERSION(EvalResults, 2);
```

**Read-side back-compat** (R9 + Constitution V loud-fail rule):
- Pathgen-mode dmps written at version 1 deserialize the existing fields normally; new fields are absent (cereal handles this via the version-aware `serialize`).
- Pre-versioning dmps (those written before the FR-015a version-field add at M1) are treated as version 1 with documented assumption — readers verify the assumption in a startup check and loud-fail if the schema doesn't actually match.
- A future version 3 read by a v2-only reader fails loudly (don't silently truncate).

**Self-containedness property** (per FR-015 design intent + D13 storage clarification): the M2 dmp's `targetTrajectoryList` is a *copy* of the source-scenario target trajectory data the M2 run consumed. Renderer (M9) reads M2 dmp directly; never reaches into M1 source dmp at playback time.

**Storage cost** (back-of-envelope): per scenario per tick: ~10 floats (target pos+quat+vel+trail) + ~20 floats (camera view) = ~30 floats = 120 bytes. At 30s × 10 Hz × 245 scenarios per gen = ~88 MB per gen. Acceptable.

**Citations**:
- `include/autoc/rpc/protocol.h:234-336` (existing EvalResults)
- 030 spec FR-015 (two embedded classes)
- 030 spec FR-015a (versioning per Constitution V)
- 030 spec D14 (timing-model copies vs interpolation)

---

## 9. State transitions / per-tick data flow

**Tracker-mode worker per-tick flow** (M6 + M7 implementations, driven by FR-018 timing model):

```
M2 source-tick t_i arrives:
  1. target_state = source_traj.sample(t_i)        // in-memory lookup; v1 path
                                                   // post-v1: RobotProgrammable.advanceTo(t_i)
                                                   // when crrcsim multi-aircraft display lands
  2. BeaconProjector.projectBeacons(chase_state, target_state, camera_config, beacon_config)
     → BeaconObservation × 2 (left, right) — analytic projection,
       int8 quantization round-trip, sentinel handling
  3. NN history slot shift:
     - oldest slot dropped; new BeaconObservation added at NOW slot
     - 6-slot ring per (beacon × {x, y, cep}) = 36 fp32 NN inputs
  4. NN inputs gathered per FR-006 / R4 typed interface
  5. NN forward pass → out = (pitch, roll, throttle)
  6. crrcsim chase-craft physics integrated until t_{i+1} (FR-018):
     - msp commands sent to crrcsim
     - crrcsim runs micro physics steps until simulated time >= t_{i+1}
  7. Fitness contributions for this tick:
     - TrailRabbit.compute(target_sample[i]) → rabbit position
     - cone-surface fitness against rabbit (existing FitnessComputer reused)
     - CrashHull check → if fired, scenario terminates with penalty
     - Arena bound check → if egressed, scenario terminates with penalty
  8. Record per-tick state for M2 dmp output (FR-015):
     - chase AircraftState → aircraftStateList
     - CameraViewSample → cameraViewList
     - CopiedTargetSample → targetTrajectoryList
  9. Loop: i++, fetch t_{i+1}, repeat from step 1
```

**Pathgen-mode flow**: unchanged. The strategy-pattern split (R5) means `PathgenStepper` runs the existing per-tick logic without touching this new pipeline.

**Determinism invariant** (per FR-009 + R3 + R7 + R10):
- All per-tick PRNG draws (p_crash, etc.) use a dedicated subsequence of the per-scenario PRNG seeded from `ScenarioMetadata.scenarioSequence` and the source's joint-PRNG params. Same scenario index + same source dmp + same `autoc-tracker.ini` ⇒ bit-identical M2 dmp output.
- int8 quantization round-trip (R7) is deterministic; floating-point math in projection (R3-research-era math, still valid) is deterministic on the same hardware.
- crrcsim physics integration is deterministic per existing project guarantees.

---

## 10. Arena (FR-016)

```cpp
// arena.h (illustrative)
struct FlightArena {
  float radius_m = 80.0f;       // R2 default
  float floor_agl_m = 5.0f;
  float ceiling_agl_m = 100.0f;
};

enum class ArenaEgressKind { OK, OUTSIDE_RADIUS, BELOW_FLOOR, ABOVE_CEILING };

ArenaEgressKind checkArenaBounds(const FlightArena& arena, const gp_vec3& chase_pos_world);
```

**Per-tick check** (M7): chase craft world position vs arena cylinder. Egress kind recorded for telemetry; scenario terminates with arena-egress penalty.

**Citations**:
- 030 spec FR-016
- research.md R2 (parallel `FLIGHT_ARENA_*` constants vs extending `ENTRY_SAFE_*`)

---

## Summary — entity inventory

| # | Entity | Lives in | Created at | Consumed by |
|---|---|---|---|---|
| 1 | `SourceScenarioTrajectory` | autoc memory | startup, from M1 dmp | TrackerStepper, BeaconProjector (v1 path); `RobotProgrammable` post-v1 |
| 2 | `autoc-tracker.ini` | disk (`-i autoc-tracker.ini`) | operator | autoc startup parsing |
| 3 | `CameraConfig` / `BeaconConfig` | from ini | startup | BeaconProjector |
| 4 | `BeaconObservation` | per-tick | BeaconProjector | NN inputs (M2 typed interface), M2 dmp `cameraViewList` |
| 5 | `NNInput` enum | header | compile time | evaluator, xiao mirror, analysis scripts |
| 6 | `TrailRabbit` | per-tick | trail_rabbit.cc | FitnessComputer (cone-surface), M2 dmp `targetTrajectoryList` |
| 7 | `CrashHull` + curriculum | per-scenario + per-tick | crash_hull.cc | FitnessComputer (penalty), scenario termination |
| 8 | `EvalResults` v2 schema | M2 dmp | per-best-of-gen at S3 write | renderer (M9), per-tick dmp extractor (M11a) |
| 9 | per-tick data flow | runtime | TrackerStepper | M2 dmp + fitness aggregation |
| 10 | `FlightArena` | from ini | startup | per-tick arena check |

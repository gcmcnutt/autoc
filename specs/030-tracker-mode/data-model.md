# 029 — Data Model

Entities, schemas, validation rules, and per-tick data flow for tracker mode. Anchored to spec.md FRs and the research.md findings (R1 playback format, R2 dmp deserialization, R7 type-safe sensor interface).

## 1. Library entry (`.crrclog` playback file)

**Source**: produced by `tools/dmp_to_playback` from one source-run `.dmp` per scenario.
**Format**: existing crrcsim `.crrclog` binary (per R1) — *no new format invented*.
**Consumer**: `crrcsim/src/mod_robots/CRRC_AirplaneSim_Playback`, instantiated per scenario as the target aircraft.

### 1.1 File structure

```
+-----------------------------------+
| ASCII XML header                  |  Root: CRRCSim_record (enforced by reader)
| - airplane.file = hb1_streamer.xml|  v1: same model as training craft
| - airplane.graphics = ...         |
| - scenery.file = ...              |
| - wind.* (seed, direction, etc.)  |  Inherited from source scenario
| - description (free text)         |
+-----------------------------------+
| '\n' separator byte               |
+-----------------------------------+
| Tagged record stream (binary, LE) |
| Each record:                       |
|   1 byte tag                       |
|   variable body                    |
+-----------------------------------+
```

### 1.2 Record types

| Tag | Use in 029 v1 | Body | Cadence |
|---|---|---|---|
| `0x00` | **Position/attitude** (load-bearing) | 20 bytes: `double timestep` + `float[3] pos` + `int16[3] euler` (each ÷ ROBOT_EULER_TO_INT16 = 32767/2π) | One per FDM tick from source recording |
| `0x01` | NOT USED — reader has no case for it | — | omit |
| `0x02` | NOT USED — F3F sync markers irrelevant for tracker | — | omit |
| `0x03` | **Provenance metadata** (used in 029) | ASCII XML carrying source-run id, source gen, source scenario index, joint-PRNG params (path/wind/craft variation indices) | One at file end |

### 1.3 Endianness + invariants

- Native little-endian (per crrcsim convention; documented at `crrcsim/src/mod_robots/robotfile.h:36`)
- Python writer: `struct.pack('<d3f3h', ...)` per 0x00 record body
- Constant timestep matching crrcsim's autoc tick (`Global::dt × multiloop`) — playback consumer interpolates linearly between consecutive 0x00 records

### 1.4 Validation rules

- XML root element name MUST be `CRRCSim_record` (enforced by `fdm_playback.cpp:143–146`; converter must emit exactly this)
- `airplane.file` MUST resolve to a valid crrcsim aircraft model (hb1_streamer.xml in v1)
- 0x00 records MUST have monotonic-non-decreasing accumulated timestep
- Final 0x03 provenance record is OPTIONAL but recommended for debuggability and joint-PRNG re-seeding (FR-010)

## 2. Library directory

A directory containing the 245 `.crrclog` files (one per source scenario), produced from a single source pathgen-run + chosen source gen.

### 2.1 Layout

```
<library_root>/
├── library_metadata.json    # Source run id, source gen, conversion timestamp,
│                            # variation params per scenario, sha256 of dmp source
├── 000.crrclog              # Scenario 0 (path 0, wind 0)
├── 001.crrclog              # Scenario 1 (path 0, wind 1)
├── ...
└── 244.crrclog              # Scenario 244 (path 4, wind 48 — last)
```

### 2.2 Library metadata schema (`library_metadata.json`)

```json
{
  "source_run_id": "more-rnn3-2026-04-26T...",
  "source_gen": 600,
  "source_dmp_path": "s3://autoc-storage/.../gen9400.dmp",
  "source_dmp_sha256": "...",
  "conversion_timestamp": "2026-04-29T...",
  "scenario_count": 245,
  "scenarios": [
    {
      "index": 0,
      "filename": "000.crrclog",
      "path_variant": 0,
      "wind_variant": 0,
      "wind_seed": 12345,
      "craft_variation": null,
      "entry_pose": {"cone_phi_deg": 0.0, "cone_theta_deg": 0.0, "roll_deg": 0.0, "speed_factor": 1.0},
      "tick_count": 350,
      "duration_sec": 35.0,
      "crashed": false
    },
    ...
  ]
}
```

### 2.3 Validation rules

- `scenario_count` MUST equal source run's per-gen scenario count (245 for Path A configs)
- `tick_count` per scenario MUST equal the count of 0x00 records in that scenario's `.crrclog` file
- Crashed scenarios (`crashed: true`): library can either include partial trajectory OR omit the scenario entirely (FR-013 decision; documented in `library_metadata.json` if included)

## 3. Camera configuration

Per FR-003 / FR-003a / FR-003b / FR-003c / FR-003d / FR-003e. Compile-time constants for v1 with the architectural class split (compile-time fixed vs PRNG-varied) preserved.

### 3.1 `CameraConfig` struct (`include/autoc/eval/camera_config.h`)

```cpp
struct CameraConfig {
    // Compile-time fixed (target-hardware spec)
    enum SensorType { SENSOR_GENERIC_RGB_BAYER, ... };
    SensorType sensor_type;
    int frame_rate_hz;            // v1: 30
    float exposure_ms;            // v1: 0
    float readout_ms;             // v1: 0 (latency split per R6)
    enum ProjectionGeometry { PLANAR_PINHOLE, FISHEYE_SPHERICAL, ... };
    ProjectionGeometry geometry;  // v1: PLANAR_PINHOLE
    enum ColorFilter { DUAL_PASS_IR, FULL_VISIBLE, ... };
    ColorFilter color_filter;     // v1: DUAL_PASS_IR
    enum ShutterMode { GLOBAL, ROLLING_HORIZONTAL, ROLLING_VERTICAL };
    ShutterMode shutter;          // v1: GLOBAL

    // PRNG-varied per scenario (held at nominal in v1; future variants sample)
    Vector3f mount_offset_body;   // body-frame meters; v1: nose-forward
    Quaternionf mount_orientation; // identity v1
    float fov_horiz_deg;          // v1: 120
    float fov_vert_deg;           // v1: derived from horizontal + sensor aspect

    // Aberrations — interface present, identity in v1
    float radial_k1, radial_k2, radial_k3;        // v1: 0
    float tangential_p1, tangential_p2;            // v1: 0
    float chromatic_shift_per_channel[2];          // v1: 0
    float vignetting_falloff;                      // v1: 0
    float motion_blur_kernel_sigma;                // v1: 0
};
```

### 3.2 Validation rules

- `frame_rate_hz` MUST be ≥ NN tick rate (10 Hz). Lower means dropped NN ticks; not supported.
- `frame_rate_hz / 10` (frames per NN tick) MUST be integer for clean buffer sizing in v1.
- `geometry == PLANAR_PINHOLE` requires `fov_horiz_deg < 180`. Wider angles MUST use `FISHEYE_SPHERICAL`.
- `mount_offset_body` magnitude MUST be < 1 m for cheap-camera physical realism.

### 3.3 Compile-time defaults (v1)

```cpp
// include/autoc/eval/camera_config.h
constexpr CameraConfig kDefaultCameraV1 = {
    SENSOR_GENERIC_RGB_BAYER,
    /*frame_rate_hz=*/ 30,
    /*exposure_ms=*/ 0.0f,
    /*readout_ms=*/ 0.0f,
    PLANAR_PINHOLE,
    DUAL_PASS_IR,
    GLOBAL,
    /*mount_offset_body=*/ {0.5f, 0.0f, 0.0f},  // 50 cm forward of CG (nose)
    /*mount_orientation=*/ Quaternionf::Identity(),
    /*fov_horiz_deg=*/ 120.0f,
    /*fov_vert_deg=*/  90.0f,                   // assuming 4:3 sensor aspect
    // ... aberrations all zero
};
```

## 4. Beacon configuration

Per FR-004.

### 4.1 `BeaconConfig` struct

```cpp
struct BeaconConfig {
    enum WingtipSide { LEFT, RIGHT };
    WingtipSide side;
    Vector3f position_body;        // body-frame position on target craft
    float wavelength_nm;           // 850 (left) or 940 (right) for v1 IR
    float emission_cone_deg;       // >180 in v1 (omnidirectional in hemisphere)
    float emission_intensity;      // brightness; affects detection threshold
};

constexpr BeaconConfig kBeaconLeftV1 = { LEFT,  /*pos*/{0, -wingspan/2, 0}, /*nm*/850, /*cone*/200, /*intensity*/1.0f };
constexpr BeaconConfig kBeaconRightV1 = { RIGHT, /*pos*/{0, +wingspan/2, 0}, /*nm*/940, /*cone*/200, /*intensity*/1.0f };
```

### 4.2 Validation rules

- `wavelength_nm` MUST be in the IR range (>700 nm) for v1 IR-color baseline
- `emission_cone_deg` ≥ 180 ensures attitude-independent visibility per spec assumption

## 5. Beacon projection output (per-camera per-tick)

Per FR-005. **The minimal 3-float collapsed output** that the NN sees, mirroring the deployed FPGA centroid extractor.

### 5.1 `BeaconProjectionResult` struct

```cpp
struct BeaconProjectionResult {
    float screen_x;       // [-1, +1] normalized to camera FOV; 0 = center
    float screen_y;       // [-1, +1] normalized
    float visible;        // 1.0 = beacon detected, 0.0 = not detected (FOV / behind / threshold)
};
```

### 5.2 Validation rules

- When `visible == 0`, `screen_x` and `screen_y` MUST be 0 (FR-007 — no fabricated coords)
- When `visible == 1`, `(screen_x, screen_y)` MUST be in valid normalized range for the projection geometry

## 6. NN sensor input layout (tracker mode)

Per FR-006 (type-safe sensor interface).

### 6.1 Input enumeration

Tracker-mode inputs replace pathgen-mode's `target_x[6]` / `target_y[6]` / `target_z[6]` / `dist[6]` with beacon-coordinate history. Aircraft state inputs (quat, airspeed, gyros) carry over unchanged.

```cpp
enum class TrackerSensorInput : int {
    // Beacon history per tracker-mode design (4 history slots × 3 fields × 2 beacons)
    BEACON_L_X_T_500MS_PAST,    BEACON_L_Y_T_500MS_PAST,    BEACON_L_VISIBLE_T_500MS_PAST,
    BEACON_L_X_T_400MS_PAST,    BEACON_L_Y_T_400MS_PAST,    BEACON_L_VISIBLE_T_400MS_PAST,
    BEACON_L_X_T_300MS_PAST,    BEACON_L_Y_T_300MS_PAST,    BEACON_L_VISIBLE_T_300MS_PAST,
    BEACON_L_X_T_200MS_PAST,    BEACON_L_Y_T_200MS_PAST,    BEACON_L_VISIBLE_T_200MS_PAST,
    BEACON_L_X_T_100MS_PAST,    BEACON_L_Y_T_100MS_PAST,    BEACON_L_VISIBLE_T_100MS_PAST,
    BEACON_L_X_NOW,             BEACON_L_Y_NOW,             BEACON_L_VISIBLE_NOW,
    BEACON_R_X_T_500MS_PAST,    BEACON_R_Y_T_500MS_PAST,    BEACON_R_VISIBLE_T_500MS_PAST,
    BEACON_R_X_T_400MS_PAST,    BEACON_R_Y_T_400MS_PAST,    BEACON_R_VISIBLE_T_400MS_PAST,
    BEACON_R_X_T_300MS_PAST,    BEACON_R_Y_T_300MS_PAST,    BEACON_R_VISIBLE_T_300MS_PAST,
    BEACON_R_X_T_200MS_PAST,    BEACON_R_Y_T_200MS_PAST,    BEACON_R_VISIBLE_T_200MS_PAST,
    BEACON_R_X_T_100MS_PAST,    BEACON_R_Y_T_100MS_PAST,    BEACON_R_VISIBLE_T_100MS_PAST,
    BEACON_R_X_NOW,             BEACON_R_Y_NOW,             BEACON_R_VISIBLE_NOW,
    // 6 history slots × 3 fields × 2 beacons = 36 beacon inputs

    // Aircraft state (unchanged from pathgen mode)
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,

    COUNT  // 36 + 8 = 44 inputs total — wait, recount per Q5 design
};
```

**Note on count**: Q5 in spec.md spelled out 4 history slots × 2 axes (x,y) × 2 beacons = 16 beacon-related screen-coord floats + 1 in_fov flag at "now" per beacon = 18 total target-related floats, plus 8 aircraft = 26. The above enumerates 6 history slots (matching the existing `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` decision from clarifications) with per-sample visibility flags = 36 + 8 = 44. **Plan reconciles to 6 slots × 3 fields × 2 beacons = 36 + 8 aircraft = 44.** Higher than the spec's pre-clarify estimate of ~32 because the per-sample-visibility-flag decision adds 6 more flags (5 past + now per beacon) × 2 beacons = 12 flags. Acceptable input-dim growth; weight count grows by ~360 (44 × 8 first-layer factor estimate) — noise compared to the 1923-weight baseline.

### 6.2 Topology constants

```cpp
constexpr int NN_INPUT_COUNT = static_cast<int>(TrackerSensorInput::COUNT);  // 44 in tracker mode
// Pathgen mode keeps existing NN_INPUT_COUNT = 33 unchanged
```

The mode selector chooses which enum drives `NN_INPUT_COUNT`. Per FR-011, both modes coexist; the build is mode-conditional or runtime-mode-selected (decision pinned in plan §2.4).

### 6.3 Validation rules

- `BEACON_*_X` / `BEACON_*_Y` MUST be in [-1, +1] when `BEACON_*_VISIBLE` is 1; arbitrary when 0
- `BEACON_*_VISIBLE` MUST be exactly 0.0 or 1.0 (binary in v1; future: continuous channel response)
- `QUAT_*` MUST satisfy `qw² + qx² + qy² + qz² ≈ 1.0` per existing convention

## 7. Tracker-mode `EvalResults` extension

Per FR-015. Extends existing `include/autoc/rpc/protocol.h` `EvalResults` schema with camera view data. Schema bump per [no cereal versioning policy](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) — old `.dmp` files become unloadable by tracker-aware tools, but pathgen-mode tooling continues unchanged.

### 7.1 Added fields

```cpp
struct EvalResults {
    // ... existing pathgen fields unchanged ...

    // 029 tracker-mode additions (optional; populated only when training_mode == TRACKER):
    bool tracker_mode = false;                // schema discriminator
    std::vector<std::vector<CameraTickState>> camera_state_per_scenario;
    // camera_state_per_scenario[scenario][tick] = camera pose + projection results at that tick
};

struct CameraTickState {
    Vector3f camera_pos_world;
    Quaternionf camera_orientation_world;
    CameraConfig camera_config_snapshot;       // for renderer's camera-POV mode
    BeaconProjectionResult beacon_left;
    BeaconProjectionResult beacon_right;
};
```

### 7.2 Validation rules

- If `tracker_mode == true`, `camera_state_per_scenario.size() == aircraftStateList.size()` (one camera-state vector per scenario)
- Per-tick `camera_state_per_scenario[s].size() == aircraftStateList[s].size()` (one entry per aircraft tick)
- `tracker_mode == false` for pathgen-mode dumps (backward-compat with existing pathgen tooling)

## 8. Frame buffer / history window

Per FR-003e (multi-frame-per-tick → NN input mapping).

### 8.1 Per-camera ring buffer

```cpp
struct CameraFrameBuffer {
    std::deque<BeaconProjectionResult> beacon_left_history;
    std::deque<BeaconProjectionResult> beacon_right_history;
    int capacity;  // = ceil(camera.frame_rate_hz * 1.0s) = ~30 at 30 Hz
};
```

### 8.2 Sample selection at NN tick

Per US1 outcome → tracker-mode design: at each NN tick, pull frames at offsets `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` seconds. With 30 Hz camera × 10 Hz NN tick:
- `now` = frame index 0 (newest)
- `-0.1s` = frame index -3 (3 frames back, exactly 100 ms)
- `-0.2s` = frame index -6
- `-0.3s` = frame index -9
- `-0.4s` = frame index -12
- `-0.5s` = frame index -15

All offsets quantize cleanly to 30 Hz frame indices.

### 8.3 Validation rules

- Buffer capacity MUST hold enough frames to support the deepest history offset (-0.5s @ 30 Hz = 15 frames; round up to 16)
- Sample at deepest offset before buffer is full = treat as `visible=0` for that slot (warm-up state)

## 9. Mode selector

Per FR-011.

### 9.1 `TrainingMode` enum

```cpp
enum class TrainingMode {
    PATHGEN,    // existing pathgen-mode (rabbit on synthetic path)
    TRACKER,    // tracker mode (target craft from playback library)
};
```

### 9.2 Config-driven selection

- `autoc.ini` (existing): implicit `TrainingMode = PATHGEN`
- `autoc-tracker.ini` (new): explicit `TrainingMode = TRACKER` + `LibraryDirectory = <path>` + tracker-mode-specific knobs

The runtime `TrainingMode` value drives:
- Scenario construction (synthesized rabbit vs library entry)
- NN input layout (33 pathgen vs 44 tracker)
- Sensor interface enum (`PathgenSensorInput` vs `TrackerSensorInput`)
- Eval pipeline (rabbit position vs robot position)
- `EvalResults` dump variant (pathgen-only vs `tracker_mode = true`)

### 9.3 Validation rules

- Mode is determined at autoc startup from the loaded config; cannot change mid-run
- A given training run is exactly one mode (no mid-run switching)
- A `.dmp` file is exactly one mode (no mixed-mode dumps)

## 10. State transitions — none

029 introduces no new state-machine entities. All data is per-scenario / per-tick snapshot data, computed and emitted in line with the existing evolution loop. The autoc / crrcsim per-scenario lifecycle (variation seed → scenario init → tick loop → eval results) is preserved unchanged; the only differences are:
- Scenario init loads a library entry instead of synthesizing a path
- Tick loop projects beacons in addition to existing per-tick work
- Eval results carry the camera state extension when `tracker_mode == true`

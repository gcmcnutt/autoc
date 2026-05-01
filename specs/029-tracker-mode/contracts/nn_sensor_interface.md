# Contract: Type-safe NN sensor interface

**Producer**: `nn_input_computation.cc` (existing) + new tracker-mode input computation
**Consumer**: NN forward-pass (`evaluator.cc`), data.dat emission (`autoc.cc`), xiao-side input gathering (`xiao/src/msplink.cpp`), data.dat parser (`specs/019-improved-crrcsim/sim_response.py`), genome ablation tool (future BACKLOG item)

**Source**: rolled in from BACKLOG `Type-Safe NN Sensor Interface` (per FR-006 and [research.md R7](../research.md#r7--type-safe-nn-sensor-interface--implementation-cost-estimate)). Lands as a Phase 0 PR before tracker-mode-specific work begins.

## Goal

Replace opaque `float[NN_INPUT_COUNT]` indexing with named/typed enum-tagged inputs:

```cpp
// Before (existing, magic-number layout):
float inputs[33];  // What's at index 0? You read evaluator.cc to find out.

// After (typed):
SensorInputs inputs;
inputs[GYRO_P] = ...;
inputs[BEACON_L_X_NOW] = ...;  // tracker mode
```

## Two enum domains

Per FR-011, pathgen mode and tracker mode have distinct sensor layouts. The interface accommodates both via two enums, mode-selected at compile or runtime:

### Pathgen-mode inputs (`PathgenSensorInput`, 33 entries)

Mirrors today's `NNInputs` struct ([include/autoc/nn/nn_inputs.h](../../../include/autoc/nn/nn_inputs.h)):

```cpp
enum class PathgenSensorInput : int {
    TARGET_X_T_900MS_PAST,    TARGET_X_T_300MS_PAST,    TARGET_X_T_100MS_PAST,
    TARGET_X_NOW,             TARGET_X_T_100MS_FUTURE,  TARGET_X_T_500MS_FUTURE,
    TARGET_Y_T_900MS_PAST,    TARGET_Y_T_300MS_PAST,    TARGET_Y_T_100MS_PAST,
    TARGET_Y_NOW,             TARGET_Y_T_100MS_FUTURE,  TARGET_Y_T_500MS_FUTURE,
    TARGET_Z_T_900MS_PAST,    TARGET_Z_T_300MS_PAST,    TARGET_Z_T_100MS_PAST,
    TARGET_Z_NOW,             TARGET_Z_T_100MS_FUTURE,  TARGET_Z_T_500MS_FUTURE,
    DIST_T_900MS_PAST,        DIST_T_300MS_PAST,        DIST_T_100MS_PAST,
    DIST_NOW,                 DIST_T_100MS_FUTURE,      DIST_T_500MS_FUTURE,
    CLOSING_RATE,
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    COUNT  // 33
};
```

(Note: US1's past-only experiment shifts the time semantics of these slots without changing the enum count — same names, different time meaning per `nn_inputs.h` comment.)

### Tracker-mode inputs (`TrackerSensorInput`, 44 entries)

Per [data-model.md §6.1](../data-model.md):

```cpp
enum class TrackerSensorInput : int {
    BEACON_L_X_T_500MS_PAST, BEACON_L_Y_T_500MS_PAST, BEACON_L_VISIBLE_T_500MS_PAST,
    BEACON_L_X_T_400MS_PAST, BEACON_L_Y_T_400MS_PAST, BEACON_L_VISIBLE_T_400MS_PAST,
    BEACON_L_X_T_300MS_PAST, BEACON_L_Y_T_300MS_PAST, BEACON_L_VISIBLE_T_300MS_PAST,
    BEACON_L_X_T_200MS_PAST, BEACON_L_Y_T_200MS_PAST, BEACON_L_VISIBLE_T_200MS_PAST,
    BEACON_L_X_T_100MS_PAST, BEACON_L_Y_T_100MS_PAST, BEACON_L_VISIBLE_T_100MS_PAST,
    BEACON_L_X_NOW,          BEACON_L_Y_NOW,          BEACON_L_VISIBLE_NOW,
    BEACON_R_X_T_500MS_PAST, BEACON_R_Y_T_500MS_PAST, BEACON_R_VISIBLE_T_500MS_PAST,
    BEACON_R_X_T_400MS_PAST, BEACON_R_Y_T_400MS_PAST, BEACON_R_VISIBLE_T_400MS_PAST,
    BEACON_R_X_T_300MS_PAST, BEACON_R_Y_T_300MS_PAST, BEACON_R_VISIBLE_T_300MS_PAST,
    BEACON_R_X_T_200MS_PAST, BEACON_R_Y_T_200MS_PAST, BEACON_R_VISIBLE_T_200MS_PAST,
    BEACON_R_X_T_100MS_PAST, BEACON_R_Y_T_100MS_PAST, BEACON_R_VISIBLE_T_100MS_PAST,
    BEACON_R_X_NOW,          BEACON_R_Y_NOW,          BEACON_R_VISIBLE_NOW,
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    COUNT  // 44
};
```

## Per-input metadata table (compile-time)

```cpp
struct SensorInputDescriptor {
    const char* name;        // for ablation tool, logging, debug
    const char* unit;        // "rad/s", "m", "screen-frac", etc.
    float min_value;         // valid range lower bound
    float max_value;         // valid range upper bound
    bool is_categorical;     // true for VISIBLE flags (binary)
};

constexpr SensorInputDescriptor kPathgenSensorMeta[] = {
    {"TARGET_X_T_900MS_PAST", "unit-vec", -1.0f, +1.0f, false},
    // ... 33 entries
};

constexpr SensorInputDescriptor kTrackerSensorMeta[] = {
    {"BEACON_L_X_T_500MS_PAST", "screen-frac", -1.0f, +1.0f, false},
    // ...
    {"BEACON_L_VISIBLE_T_500MS_PAST", "binary", 0.0f, 1.0f, true},
    // ... 44 entries
};
```

## Compile-time topology

```cpp
template<typename SensorEnum>
struct SensorInputs {
    float values[static_cast<int>(SensorEnum::COUNT)];

    constexpr float& operator[](SensorEnum input) {
        return values[static_cast<int>(input)];
    }
};

constexpr int NN_INPUT_COUNT_PATHGEN = static_cast<int>(PathgenSensorInput::COUNT);  // 33
constexpr int NN_INPUT_COUNT_TRACKER = static_cast<int>(TrackerSensorInput::COUNT);  // 44

// Mode selector — runtime or compile-time per plan §2.4 decision
constexpr int NN_INPUT_COUNT = (TRAINING_MODE == PATHGEN) ? NN_INPUT_COUNT_PATHGEN : NN_INPUT_COUNT_TRACKER;
```

## Migration touchpoints

Per R7 research, the refactor touches **12 files / ~270-330 LOC**:

| File | Migration scope |
|---|---|
| `include/autoc/nn/nn_inputs.h` | Replace `NNInputs` struct with enum-indexed `SensorInputs<PathgenSensorInput>` template instance |
| `include/autoc/nn/sensor_interface.h` (NEW) | Enum definitions + descriptor table + topology constants |
| `src/nn/nn_input_computation.cc` | Migrate field-name accesses to enum-indexed `inputs[QUAT_W]` etc. |
| `src/autoc.cc` data.dat header + format string | Auto-generate from descriptor table (DRY) |
| `tests/contract_evaluator_tests.cc` | Update topology assertions to use named constants |
| `tests/nn_evaluator_tests.cc` | Same |
| `xiao/src/msplink.cpp` | Migrate xiao-side input gathering to typed interface |
| `xiao/src/generated/nn_program_generated.cpp` | Regenerated by `nn2cpp` (auto) |
| `tools/nn2cpp.cc` | Emit enum-aware C code |
| `specs/019-improved-crrcsim/sim_response.py` | Auto-generate from descriptor table OR pin to enum names |
| `include/autoc/eval/aircraft_state.h` | `nnInputs_` array sizing uses `NN_INPUT_COUNT` constant |
| `include/autoc/autoc.h` | Remove duplicate magic-number defines (DISTANCE_TARGET etc.) — single source of truth in sensor_interface.h |

## Validation rules

The producer (input gathering code) MUST:
- Initialize all entries in the SensorInputs array per tick — no uninitialized reads
- Set each input within its declared `[min_value, max_value]` range; out-of-range values trigger a debug-build assert
- For categorical flags, write exactly 0.0f or 1.0f

The consumer (NN forward-pass) MUST:
- Read inputs only via the typed enum interface, not raw float[] indexing
- The compile-time topology constant MUST equal `static_cast<int>(EnumName::COUNT)` — checked by static_assert

## Backward compatibility

- **Pathgen-mode behavior**: identical to today. Same 33-input layout, same per-input semantics, same data.dat schema. The refactor is purely a re-typing of how the inputs are accessed in code; the on-disk and on-wire representations are unchanged.
- **Tracker-mode behavior**: net-new. 44-input layout co-exists in tree but is only active when `TrainingMode = TRACKER`.

## Test surface

`tests/nn_sensor_interface_tests.cc` (NEW for 029, lands in Phase 0):

| Test | Assertion |
|---|---|
| `EnumCount_MatchesPathgenLayout` | `static_cast<int>(PathgenSensorInput::COUNT) == 33` |
| `EnumCount_MatchesTrackerLayout` | `static_cast<int>(TrackerSensorInput::COUNT) == 44` |
| `RoundtripInputs_PathgenIdentity` | Set every pathgen input to a unique known value, read each back via the typed interface, all match |
| `RoundtripInputs_TrackerIdentity` | Same for tracker mode |
| `MetadataLookup_NamesMatchEnum` | For each enum value, `kPathgenSensorMeta[(int)e].name` exists and matches the enum name string |
| `RangeValidation_OutOfRangeAsserts` | Debug build: writing 99.0 to an input with max=1.0 triggers assert |
| `BackwardCompat_DataDatHeader_Stable` | The data.dat file header generated from the new descriptor table is byte-identical to the old hand-coded header (for pathgen mode) |

## Why land in Phase 0 (before any tracker-mode-specific work)

Per R7 research:
- Doing the refactor *as part of* tracker-mode work mixes two failure modes (interface bug vs tracker-mode semantic bug) — debugging becomes harder
- Doing the refactor *after* tracker-mode-specific code lands creates strictly more total work — every tracker-mode-specific access has to be re-typed, doubling the migration surface
- FR-006's name-based ablation (`--zero-input BEACON_L_*`) is *load-bearing* for the genome ablation tool (separate BACKLOG item) — that tool needs the type-safe interface to work

So Phase 0 of 029 implementation is: type-safe sensor interface PR. No tracker-mode-specific code in this PR. Build green, all 028 tests pass, xiao build green. Then Phase 2.2 onward proceeds.

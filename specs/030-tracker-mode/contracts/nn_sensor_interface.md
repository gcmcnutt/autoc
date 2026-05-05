# Contract: Type-safe NN sensor interface (FR-006 + R4)

**Producer**: NN-input gathering (sim-side `evaluator.cc`; xiao-side `msplink.cpp`)
**Consumer**: NN forward-pass (`evaluator.cc`), `data.dat` emission (`autoc.cc`), data.dat parser (`specs/019-improved-crrcsim/sim_response.py`), [genome ablation tool](../../BACKLOG.md), per-tick dmp extractor (M11a)

Refreshed 2026-05-04: typed names use the `(x, y, CEP)` interface (was `(x, y, visible)` in the prior version).

## Surface

```cpp
// include/autoc/nn/nn_inputs.h

namespace autoc::nn {

enum class NNInput : uint16_t {
  // ============================================================
  // Beacon left observations (3 channels × 6 history slots = 18)
  // ============================================================
  BEACON_L_X_TM5,    BEACON_L_Y_TM5,    BEACON_L_CEP_TM5,
  BEACON_L_X_TM4,    BEACON_L_Y_TM4,    BEACON_L_CEP_TM4,
  BEACON_L_X_TM3,    BEACON_L_Y_TM3,    BEACON_L_CEP_TM3,
  BEACON_L_X_TM2,    BEACON_L_Y_TM2,    BEACON_L_CEP_TM2,
  BEACON_L_X_TM1,    BEACON_L_Y_TM1,    BEACON_L_CEP_TM1,
  BEACON_L_X_NOW,    BEACON_L_Y_NOW,    BEACON_L_CEP_NOW,

  // ============================================================
  // Beacon right observations (mirror, 18)
  // ============================================================
  BEACON_R_X_TM5,    BEACON_R_Y_TM5,    BEACON_R_CEP_TM5,
  BEACON_R_X_TM4,    BEACON_R_Y_TM4,    BEACON_R_CEP_TM4,
  BEACON_R_X_TM3,    BEACON_R_Y_TM3,    BEACON_R_CEP_TM3,
  BEACON_R_X_TM2,    BEACON_R_Y_TM2,    BEACON_R_CEP_TM2,
  BEACON_R_X_TM1,    BEACON_R_Y_TM1,    BEACON_R_CEP_TM1,
  BEACON_R_X_NOW,    BEACON_R_Y_NOW,    BEACON_R_CEP_NOW,

  // ============================================================
  // Aircraft state (8)
  // ============================================================
  QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
  AIRSPEED,
  GYRO_P, GYRO_Q, GYRO_R,

  COUNT  // sentinel; total = 44 per FR-006
};

struct NNInputMeta {
  const char* name;          // canonical name; appears in data.dat headers
  float range_min;
  float range_max;
  const char* units;
};

inline constexpr size_t kNNInputCount = static_cast<size_t>(NNInput::COUNT);

inline constexpr NNInputMeta kNNInputMeta[kNNInputCount] = {
  // Beacon left
  { "BEACON_L_X_TM5",   -1.0f, +1.0f, "screen_norm" },
  { "BEACON_L_Y_TM5",   -1.0f, +1.0f, "screen_norm" },
  { "BEACON_L_CEP_TM5",  0.0f,  1.5f, "cep_or_sentinel" },
  // ... (one entry per enum value, total = 44)
  { "GYRO_R", -50.0f, +50.0f, "rad/s" },
};

static_assert(static_cast<size_t>(NNInput::COUNT) == TRACKER_INPUT_COUNT,
              "NNInput enum count must match topology weight count");

inline constexpr const char* nameOf(NNInput input) {
  return kNNInputMeta[static_cast<size_t>(input)].name;
}

}  // namespace autoc::nn
```

## Population pattern

Sim-side per-tick gathering (in `nn_gather_inputs()`):

```cpp
std::array<float, kNNInputCount> inputs{};

// Beacon observations (gathered from history ring per FR-005)
const auto& obs_l_now = beacon_history_left[NOW_SLOT];
inputs[static_cast<size_t>(NNInput::BEACON_L_X_NOW)] = obs_l_now.screen_x;
inputs[static_cast<size_t>(NNInput::BEACON_L_Y_NOW)] = obs_l_now.screen_y;
inputs[static_cast<size_t>(NNInput::BEACON_L_CEP_NOW)] = obs_l_now.cep;
// ... (each history slot, each channel, each beacon)

// Aircraft state
inputs[static_cast<size_t>(NNInput::QUAT_W)] = chase_state.orientation.w();
// ... 

// Pass to NN forward-pass:
nn_evaluate(inputs.data(), kNNInputCount, ...);
```

## Cross-platform mirroring

The same `nn_inputs.h` header is consumed by:

1. **autoc desktop training binary** — primary consumer.
2. **xiao firmware** — same header included; `msplink.cpp` populates `inputs[]` with on-device perception output (FR-006 full-scope rolled-in BACKLOG item lands this in M2).
3. **`tools/aircraft_state_extractor.cc`** (M11a) — reads NN inputs from M2 dmps; column headers in CSV use `nameOf(NNInput::...)`.
4. **`specs/019-improved-crrcsim/sim_response.py`** — parses `data.dat` headers using the same canonical names emitted by autoc.

Since the header is the single source of truth for both name and index, schema changes propagate by recompile + automatic header consumption — no parallel updates across files.

## Data.dat header emission

`autoc.cc` emits the data.dat header by walking `kNNInputMeta`:

```cpp
for (size_t i = 0; i < kNNInputCount; ++i) {
  out << kNNInputMeta[i].name;
  if (i + 1 < kNNInputCount) out << "\t";
}
out << "\n";
```

The Python parser (`sim_response.py`) reads the header line and indexes by name match — no hardcoded column positions.

## Compile-time invariants

- `static_assert(NNInput::COUNT == TRACKER_INPUT_COUNT)` — wires the enum count to the topology's expected weight count. Adding / removing an input forces the topology weight count to be updated in lockstep, or the build fails.
- `kNNInputMeta` array size must match `NNInput::COUNT` (asserted by the static array declaration).

## Test coverage

`tests/nn_sensor_interface_tests.cc` (M2 deliverable):
- Round-trip name → enum → name identity.
- `nameOf(BEACON_L_CEP_NOW) == "BEACON_L_CEP_NOW"`.
- `kNNInputCount == 44` (matches FR-006).
- `kNNInputMeta` entries align with enum positions (parallel-array integrity).

`tests/contract_evaluator_tests.cc` (updated in M2):
- Existing pathgen-mode evaluator still passes against the new typed gather; behavior-identical.

## Backwards-compatibility note (Constitution III)

This is a clean-cut replacement of the magic-number `float[]` indexing pattern. No backwards-compat shim is provided — all consumer sites are updated in M2 (full files-to-touch list per [BACKLOG.md type-safe sensor interface entry](../../BACKLOG.md)). Any code reaching for index `5` instead of `BEACON_L_CEP_TM5` is a bug; build-time errors flag it during the M2 cutover.

## Citations

- 030 spec FR-006 (concrete naming requirements + dynamic ranges)
- research.md R4 (enum + constexpr metadata pattern)
- BACKLOG.md "[NEXT] Type-Safe NN Sensor Interface" (full files-to-touch list)
- [Constitution III](../../../.specify/memory/constitution.md) — no compatibility shims

# Data Model — 023 OOD and Engage Fixes

**Feature**: 023-ood-and-engage-fixes
**Phase**: 1 (Design & Contracts)
**Last updated**: 2026-04-08

Core data structures and invariants introduced or modified by 023. This document
is the authoritative reference for struct layout, field semantics, and validation
rules. Implementation must match this document; deviations require spec update.

---

## 1. `NNInputs` (NEW — `include/autoc/nn/nn_inputs.h`)

The canonical struct-of-floats describing every input the NN sees. Replaces the
current pattern of `float nnInputs[NN_INPUT_COUNT]` addressed by magic indices.

### Header contract

```cpp
// include/autoc/nn/nn_inputs.h
//
// CONSTITUTIONAL NOTE — SERIALIZATION CONTRACT
// ---------------------------------------------
// The field declaration order in this struct IS the on-disk byte order for:
//   - cereal serialization of AircraftState::nnInputs_
//   - data.dat column order
//   - nn2cpp generated xiao inference code
//   - sim_response.py parser column indexing
//
// Reordering fields is a format-breaking change equivalent to a cereal schema
// change. Adding fields is safe (compiler will force populating them via
// designated-initializer construction). Removing fields requires a version bump
// in all parsers.
//
// DO NOT add padding or non-float members. Layout is asserted at compile time.

#pragma once
#include <cstddef>

struct NNInputs {
    // Time samples at [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]
    // Past samples from history buffer, lookahead samples from path projection.
    float target_x[6];   // body-frame unit-vec x (was dPhi in 022)
    float target_y[6];   // body-frame unit-vec y (was dTheta partial in 022)
    float target_z[6];   // body-frame unit-vec z (NEW in 023)
    float dist[6];       // m, euclidean distance to rabbit (past + lookahead)

    // Scalar state
    float closing_rate;  // m/s, positive = approaching rabbit
    float quat_w;        // body orientation quaternion
    float quat_x;
    float quat_y;
    float quat_z;
    float airspeed;      // m/s (currently groundspeed — see known-gaps in research.md)
    float gyro_p;        // rad/s body rate, aerospace RHR (roll rate)
    float gyro_q;        // pitch rate
    float gyro_r;        // yaw rate
};

static_assert(sizeof(NNInputs) == 33 * sizeof(float),
              "NNInputs layout must be contiguous float[33] with no padding");
static_assert(alignof(NNInputs) == alignof(float),
              "NNInputs must be float-aligned for inner-loop matrix multiply");

// Single source of truth for input count. Everything downstream reads this.
constexpr int NN_INPUT_COUNT = sizeof(NNInputs) / sizeof(float);
```

### Field semantics

| Field | Units | Range | Notes |
|---|---|---|---|
| `target_x[6]` | dimensionless | [-1, 1] | Unit vector X component, body frame. `target_x[3]` is "now", `target_x[0..2]` are past history samples (0=oldest), `target_x[4..5]` are path-projection lookahead |
| `target_y[6]` | dimensionless | [-1, 1] | Body-frame Y component. Same time layout. |
| `target_z[6]` | dimensionless | [-1, 1] | Body-frame Z component. Same time layout. Invariant: `target_x[i]² + target_y[i]² + target_z[i]² == 1` (unit vector) |
| `dist[6]` | m | [0, ∞) | Euclidean distance aircraft→rabbit. Same time layout. |
| `closing_rate` | m/s | ℝ | `(dist[now] - dist[prev_tick]) / dt`, sign: + = approaching |
| `quat_w/x/y/z` | dimensionless | [-1, 1] | Unit quaternion, body orientation in NED. Invariant: `w² + x² + y² + z² == 1` |
| `airspeed` | m/s | [0, ∞) | Currently groundspeed from sim; labeled "airspeed" for NN compat. Real-flight reconciliation in 024+. |
| `gyro_p/q/r` | rad/s | ℝ | Body angular rates. Aerospace RHR. Inverted pitch/yaw polarity vs INAV per 021 finding. |

### Time sample indexing

All `[6]` arrays use this layout:

| Index | Time | Source |
|---|---|---|
| 0 | now − 0.9s | History buffer |
| 1 | now − 0.3s | History buffer |
| 2 | now − 0.1s | History buffer |
| 3 | now | Current tick |
| 4 | now + 0.1s | Path projection ahead |
| 5 | now + 0.5s | Path projection ahead |

### Relationship to existing storage

- `AircraftState::nnInputs_` (in `include/autoc/eval/aircraft_state.h`): changes
  from `std::array<float, NN_INPUT_COUNT>` to `NNInputs`. Cereal serialization
  updates to round-trip the struct via `CEREAL_NVP` on each field (or positional
  member iteration if simpler — both work because the struct is POD).
- `data.dat` column order: one column per `float` slot in `NNInputs`, in
  declaration order: `target_x_0, target_x_1, ..., target_z_5, dist_0, ..., gyro_r`.
  Column header names derived from field names via a single `emitColumnHeaders()`
  helper, not hand-maintained.
- xiao flash log: same column order, same header names. `nn2cpp` generates
  field-name access in the xiao C++ code, not integer indexing.

### Validation rules

1. **Sizeof contract**: `sizeof(NNInputs) == NN_INPUT_COUNT * sizeof(float)` —
   enforced at compile time via `static_assert`. Catches accidental padding
   introduction (e.g., adding a `double` or a non-float member).
2. **Alignment contract**: `alignof(NNInputs) == alignof(float)` — ensures
   `reinterpret_cast<const float*>(&inputs)` is safe for the matrix multiply
   inner loop in `evaluator.cc`.
3. **Topology match**: `NN_TOPOLOGY[0] == NN_INPUT_COUNT` — enforced in
   `include/autoc/nn/topology.h` via `static_assert`.
4. **Unit vector invariant** (runtime, not compile time): the `target_{x,y,z}[i]`
   triple must satisfy `x² + y² + z² ≈ 1` for every time sample. Enforced by
   `computeTargetDir()` always returning a normalized vector. Tests cover this
   for both the normal path and the singularity fallback.
5. **Quaternion unit norm** (runtime): `quat_w² + x² + y² + z² ≈ 1`. Already
   enforced by the sensor math layer; NN trusts the input.

### Deliberate-break verification

To confirm the compile-time safety works, Phase 0a.3 adds then reverts a test
modification that proves the assertion fires:

```cpp
struct NNInputs {
    float target_x[6];
    // ... all other fields ...
    float gyro_r;
    float DELIBERATE_BREAK; // UNDO before commit
};
// Expected: static_assert(sizeof(NNInputs) == 33 * sizeof(float)) FAILS at compile time.
```

This test is NOT committed. It is run manually during Phase 0a.3, compile output
captured and noted in the commit message as "deliberate-break verification
confirmed compile error", then reverted.

---

## 2. NN Topology (UPDATED — `include/autoc/nn/topology.h`)

### Current (022) → New (023)

| | 022 | 023 |
|---|---|---|
| Input count | 27 | 33 |
| Hidden 1 | 16 | 32 |
| Hidden 2 | 8 | 16 |
| Output | 3 | 3 |
| Total weights | 611 | 1667 |
| Topology string | `"27,16,8,3"` | `"33,32,16,3"` |

### Weight arithmetic

```
Layer 1 (input → h1): 33 * 32 + 32 = 1088
Layer 2 (h1 → h2):    32 * 16 + 16 = 528
Layer 3 (h2 → out):   16 *  3 +  3 = 51
Total:                              1667
```

### Topology constants (after refactor)

```cpp
// include/autoc/nn/topology.h

#include "autoc/nn/nn_inputs.h"

constexpr int NN_INPUT_COUNT  = ::NN_INPUT_COUNT;  // from nn_inputs.h — single source
constexpr int NN_HIDDEN1_SIZE = 32;
constexpr int NN_HIDDEN2_SIZE = 16;
constexpr int NN_OUTPUT_COUNT = 3;

constexpr int NN_NUM_LAYERS = 4;
constexpr int NN_TOPOLOGY[NN_NUM_LAYERS] = {
    NN_INPUT_COUNT, NN_HIDDEN1_SIZE, NN_HIDDEN2_SIZE, NN_OUTPUT_COUNT
};

constexpr int NN_WEIGHT_COUNT =
    (NN_INPUT_COUNT * NN_HIDDEN1_SIZE + NN_HIDDEN1_SIZE) +
    (NN_HIDDEN1_SIZE * NN_HIDDEN2_SIZE + NN_HIDDEN2_SIZE) +
    (NN_HIDDEN2_SIZE * NN_OUTPUT_COUNT + NN_OUTPUT_COUNT);

constexpr const char* NN_TOPOLOGY_STRING = "33,32,16,3";

// Compile-time sanity check
static_assert(NN_WEIGHT_COUNT == 1667, "Weight count arithmetic inconsistent");
```

### Validation rules

1. `NN_TOPOLOGY[0] == NN_INPUT_COUNT == sizeof(NNInputs) / sizeof(float)` —
   static_assert in both `topology.h` and `nn_inputs.h`.
2. `NN_WEIGHT_COUNT == 1667` — static_assert catches hand-editing errors.
3. Topology string matches numeric values — runtime check in `evaluator.cc`
   during NN initialization.

---

## 3. NN Outputs (unchanged from 022, documented for completeness)

```cpp
// 3 outputs, all tanh-activated to [-1, 1]:
//   nn_output[0]: pitch command  — surface deflection in MANUAL mode
//   nn_output[1]: roll command   — surface deflection in MANUAL mode
//   nn_output[2]: throttle       — motor command, [-1, 1] maps to [0, full]
//                                  (sim bridge handles the interpretation)
```

### Authority limit (NEW, Change 8)

The three outputs are multiplied by `NNAuthorityLimit` (config) before being
handed to the sim bridge / servo output:

```cpp
output_scaled[i] = output_raw[i] * NNAuthorityLimit;  // i = 0,1,2
```

- Default: `NNAuthorityLimit = 1.0` (no change from pre-023 behavior)
- Change 8 iteration: set to `0.5` and retrain if baseline behavior shows
  saturation / bang-bang
- Must be applied IDENTICALLY on xiao and in sim — mismatch = sim-to-real gap
- xiao reads the value from a compile-time constant that matches the training
  config; does NOT have independent runtime control

---

## 4. `EngageDelayWindow` (NEW — CRRCSim side, Change 1b)

Not a persistent struct — ephemeral per-scenario state inside CRRCSim's
`inputdev_autoc.cpp`. Documented here for design reference.

### Fields

```cpp
// crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp (file-local)
struct EngageDelayState {
    bool window_active;          // true for the first N ticks after engage
    int ticks_remaining;         // countdown, starts at EngageDelayMs / SimTimeStepMs
    // No "last stick" capture — during the window, stick is hard-coded centered
    // (zero pitch/roll/throttle command) per Q2 clarify decision.
};
```

### State transitions

```
[scenario start] → window_active=true, ticks_remaining=ceil(EngageDelayMs/dt)
                ↓
[each tick]     → if window_active:
                      stick = {0, 0, 0}
                      ticks_remaining -= 1
                      if ticks_remaining == 0: window_active = false
                  else:
                      stick = NN_outputs * NNAuthorityLimit
                ↓
[scenario end]  → [next scenario resets window_active=true]
```

### Interaction with Change 1 history reset

The history buffer pre-fill (Change 1) happens at `window_active = true` start.
NN inputs are valid and populated from tick 0 of the window; only NN OUTPUTS
are suppressed. So the NN is running forward passes, the history buffer is
being overwritten with real in-flight samples, but the sim ignores the outputs
in favor of centered stick. This matches real flight: INAV is in pass-through
but not yet applying commands, xiao is computing NN outputs every tick that
aren't yet making it to servos.

### Config

```ini
# autoc.ini / autoc-eval.ini
EngageDelayMs                   = 750       # ms, matches INAV handoff delay
```

New field. Default = 750 (matches measured INAV delay). Setting to 0 disables
the window (useful for debugging — NN takes over immediately, matches pre-023
behavior).

---

## 5. `scenarioForIndex()` and eval-mode scenario iteration (UPDATED, Phase 0b)

Not a new struct, but a behavioral change to how eval mode picks scenarios.

### Current (022) behavior

- Training: `scenarioForIndex(ind % generationScenarios.size())` — spreads
  individuals across scenarios
- Training elite re-eval: `scenarioForIndex(bestIdx % generationScenarios.size())`
- **Eval: `scenarioForIndex(0)`** — hard-coded first scenario only (Bug 5)

### New (023) behavior

- Training: unchanged
- Eval: **iterate over all scenarios**:
  ```cpp
  for (size_t s = 0; s < generationScenarios.size(); ++s) {
      EvalJob job{scenarioForIndex(s), nnBytes, EvalPurpose::StandaloneEval};
      EvalData evalData = buildEvalData(job);
      // ... sendRPC, receiveRPC, computeScenarioScores ...
      accumulator += scores;
  }
  aggregateRawFitness(accumulator) → eval_fitness;
  ```

This matches the training aggregation behavior and ensures demetic mode works
correctly in eval (where path variants live in different demes of the scenario
table and training sees all of them).

---

## 6. `ScenarioScore` (unchanged from 022)

Kept for reference — 022 already simplified this to:

```cpp
struct ScenarioScore {
    double score;            // negated points, lower = better
    bool crashed;
    int steps_completed;
    // Streak diagnostics (post-022):
    int max_streak;
    int total_streak_steps;
    double max_multiplier;
};
```

No changes in 023.

### New invariant (Phase 0b acceptance test)

For any (weight, scenario) pair, training and eval must produce
**byte-identical** `ScenarioScore`. Enforced via new test
`tests/eval_determinism_tests.cc` that:
1. Loads a fixed NN weight file
2. Picks a fixed scenario
3. Runs it through `buildEvalData(..., EvalPurpose::Training)` → `ScenarioScore_A`
4. Runs it through `buildEvalData(..., EvalPurpose::StandaloneEval)` → `ScenarioScore_B`
5. Asserts `memcmp(&score_a, &score_b, sizeof(ScenarioScore)) == 0`

---

## 7. Configuration knobs (UPDATED — `autoc.ini` / `autoc-eval.ini`)

New fields added by 023:

```ini
# Engage delay window (Change 1b) — CRRCSim simulates INAV's ~750ms handoff.
# During the window, NN runs and updates history buffer but outputs are ignored
# (centered stick applied to aircraft).
EngageDelayMs                   = 750       # ms, set to 0 to disable

# NN output authority limit (Change 8) — cap output magnitude before sim bridge
# or xiao servo output. Default 1.0 = no limit. Reduced to 0.5 in Change 8
# iteration if baseline shows saturation / bang-bang behavior.
NNAuthorityLimit                = 1.0       # dimensionless, [0.0, 1.0]
```

### Config validation

Both fields must be in valid range at startup (`src/util/config.cc`):
- `EngageDelayMs` ∈ [0, 5000] — negative is nonsense, >5s is absurd
- `NNAuthorityLimit` ∈ (0.0, 1.0] — zero would disable NN entirely, >1 would
  amplify outputs beyond the tanh range which is outside the NN's learned
  distribution

Values outside these ranges produce a fatal error on `ConfigManager::initialize()`,
not a warning.

---

## Cross-references

- **spec.md Change 1** — engage transient history reset (uses `NNInputs` struct)
- **spec.md Change 1b** — engage delay window (this doc's §4)
- **spec.md Change 6** — 3-cosine bearing representation (this doc's §1, §2)
- **spec.md Change 8** — authority limit (this doc's §3, §7)
- **spec.md Prerequisite** — type-safe NN interface (this doc's §1, §2)
- **train-eval-code-dedup.md** — Phase 0b refactor details
- **contracts/nn_interface.md** — API-level contract for NN input gathering
- **contracts/engage_delay.md** — timing contract for engage delay window
- **contracts/history_reset.md** — semantics of `resetHistory()`

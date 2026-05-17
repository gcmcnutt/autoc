# Contract: `gather_tracker_inputs` extension

**Function**: `gather_tracker_inputs` in [src/nn/evaluator.cc:430-480](../../../src/nn/evaluator.cc#L430)
**Signature** (unchanged):
```cpp
void gather_tracker_inputs(const AircraftState& chase,
                           const TrackerHistoryWindow& history,
                           const autoc::eval::FlightArena& arena,
                           TrackerInputs& out);
```

**Producer of**: `TrackerInputs` (54 fp32 slots)
**Consumer of**: `AircraftState`, `TrackerHistoryWindow` (extended with `span[6]`), `FlightArena`, `[DerivedFeatures] CepGateThreshold` (read once at startup, threaded into the function via a global or function-local constant — see implementation note below)

## Behavior extension (over 030)

030's `gather_tracker_inputs` populates slots 0..44 from the history window and chase state. The 032 phase 1 extension adds population of slots 45..53 in this order:

### 1. `beacon_pair_span[0..5]` (slots 45..50)

```cpp
for (int i = 0; i < 6; ++i) {
  out.beacon_pair_span[i] = history.span[i];  // raw-ok: NN-byte-format primitive
}
```

Direct copy from the cached history window — span values are computed and gated upstream in `projectAndShiftHistory` (per [data-model.md §3](../data-model.md#3-per-tick-data-flow-extended)).

### 2. `span_rate` (slot 51)

```cpp
out.span_rate = history.span[5] - history.span[4];  // raw-ok: NN-byte-format primitive
```

One-tick raw diff. No CEP-gate at this layer — span values already gated upstream. Visibility transitions produce signed step artifacts (intentional; see data-model.md §3.1).

### 3. `target_tilt_sin`, `target_tilt_cos` (slots 52, 53)

```cpp
const bool cep_gated_now =
    history.left_cep[5]  >= kCepGateThreshold ||
    history.right_cep[5] >= kCepGateThreshold;

if (cep_gated_now) {
  out.target_tilt_sin = 0.0f;
  out.target_tilt_cos = 1.0f;
} else {
  // Intermediates use gp_scalar per Constitution VI (eval-pipeline scalars).
  // NN-byte-format conversion happens at the slot writes below.
  const gp_scalar dx = static_cast<gp_scalar>(history.right_x[5]) - static_cast<gp_scalar>(history.left_x[5]);
  const gp_scalar dy = static_cast<gp_scalar>(history.right_y[5]) - static_cast<gp_scalar>(history.left_y[5]);
  const gp_scalar pair_dist = std::sqrt(dx*dx + dy*dy);

  if (pair_dist < static_cast<gp_scalar>(kTiltDegenerateEpsilon)) {  // 1e-4 in NDC screen-fraction units
    out.target_tilt_sin = 0.0f;
    out.target_tilt_cos = 1.0f;
  } else {
    const gp_scalar theta = std::atan2(dy, dx);
    out.target_tilt_sin = static_cast<float>(std::sin(theta));  // raw-ok: NN-byte-format slot write
    out.target_tilt_cos = static_cast<float>(std::cos(theta));  // raw-ok: NN-byte-format slot write
  }
}
```

> **Type-domain note**: Inputs (history slots) are raw `float` per the NN-byte-format whitelist. Intermediates lift to `gp_scalar` for the trig math (Constitution VI). The slot writes cast back to `float` with `raw-ok` annotation. This is the same pattern used in 030's `dist_to_boundary` computation (`evaluator.cc:475-479`).

`kCepGateThreshold` is loaded from `autoc-tracker.ini::[DerivedFeatures] CepGateThreshold` at startup (default 1.25, matching `kCepSentinelThreshold`). Held in a translation-unit-local `constexpr` if statically configured, OR in a config struct member that's plumbed to `gather_tracker_inputs` via the existing config-loading machinery — implementation choice deferred to /speckit.tasks.

`kTiltDegenerateEpsilon = 1e-4f` (NDC screen-fraction units, per research.md R4).

## Pre-conditions

- `history` has been populated by `projectAndShiftHistory` at the same tick — i.e., `history.left_*[5]`, `history.right_*[5]`, and `history.span[5]` reflect the current tick's projection
- `chase` is valid (non-NaN orientation quat, valid airspeed)
- `arena` is the same `FlightArena` instance used for OOB-termination checks (single source of truth per Session 2026-05-07 Q1 in 030)

## Post-conditions

- All 54 slots of `out` are populated (no uninitialized memory)
- All runtime invariants from [nn_sensor_interface_v54.md](./nn_sensor_interface_v54.md) hold
- Function is pure and deterministic: same inputs produce bitwise-identical outputs

## Deterministic invariant

The function MUST NOT call any nondeterministic API: no PRNG, no clock, no thread-local state, no `std::this_thread`. Order of arithmetic operations is fixed by the source order. Verified by the rebuild-perf.sh bitwise-equal regression gate.

## `projectAndShiftHistory` companion extension

`tracker_stepper.cc::projectAndShiftHistory` (and the crrcsim helper mirror) gains span computation + history shift:

```cpp
void TrackerStepper::projectAndShiftHistory(const SourceTickSample& target) {
  // Shift slots [1..5] → [0..4] for ALL channels (NDC + NEW span)
  for (int i = 0; i < 5; ++i) {
    history_.left_x[i]   = history_.left_x[i + 1];    // raw-ok: NN-byte-format primitive
    history_.left_y[i]   = history_.left_y[i + 1];    // ... existing 030 shifts ...
    history_.left_cep[i] = history_.left_cep[i + 1];
    history_.right_x[i]  = history_.right_x[i + 1];
    history_.right_y[i]  = history_.right_y[i + 1];
    history_.right_cep[i]= history_.right_cep[i + 1];
    history_.span[i]     = history_.span[i + 1];      // raw-ok: NN-byte-format primitive — 032 NEW
  }

  // Project port + starboard beacons (unchanged from 030)
  // ... existing projection code ...
  BeaconObservation left  = projectBeacon(...);  // for beacon_left_  (target body -y mount)
  BeaconObservation right = projectBeacon(...);  // for beacon_right_ (target body +y mount)

  // Write NDC into slot [5] (unchanged from 030)
  history_.left_x[5]   = left.screen_x;
  history_.left_y[5]   = left.screen_y;
  history_.left_cep[5] = left.cep;
  history_.right_x[5]  = right.screen_x;
  history_.right_y[5]  = right.screen_y;
  history_.right_cep[5]= right.cep;

  // 032 NEW: compute span[5] with CEP-gating
  const bool cep_gated =
      left.cep  >= kCepGateThreshold ||
      right.cep >= kCepGateThreshold;
  if (cep_gated) {
    history_.span[5] = 0.0f;
  } else {
    // Intermediates use gp_scalar per Constitution VI.
    const gp_scalar dx = static_cast<gp_scalar>(right.screen_x) - static_cast<gp_scalar>(left.screen_x);
    const gp_scalar dy = static_cast<gp_scalar>(right.screen_y) - static_cast<gp_scalar>(left.screen_y);
    history_.span[5] = static_cast<float>(std::sqrt(dx*dx + dy*dy));  // raw-ok: NN-byte-format slot write
  }

  // ... existing 030 last_camera_view_ / last_target_sample_ recording ...
}
```

### Scenario-start replicate-first-valid (per spec Q5)

`initScenario()` already calls `projectAndShiftHistory(source_.samples[0])` six times to pre-fill the NDC history. That same loop now also pre-fills `span[6]` — no scenario-start code change required beyond the in-function span computation above; the existing replicate-first-valid loop transparently does the right thing for span.

If tick 0 happens to be CEP-gated (e.g., starting geometry has target out of camera FOV), the neutral substitution (span = 0) replicates across all 6 history slots, matching the spec Q4 + Q5 conventions.

## Crrcsim mirror

`crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp::projectAndShiftHistory` (the local copy in the crrcsim helper) gets the identical extension. Two implementations of the same algorithm is a recognized smell — see [data-model.md §8](../data-model.md#8-cross-platform-mirroring) on consolidating these into a shared helper. /speckit.tasks should weigh "consolidate now" vs "mirror now, consolidate as a separate backlog item."

## Test coverage

See [nn_sensor_interface_v54.md test coverage](./nn_sensor_interface_v54.md#test-coverage). Additionally:

- `tests/gather_tracker_inputs_tests.cc`: end-to-end test of the extended function. Cases include:
  - Both beacons visible, target dead-ahead at mid-range → span ~ 0.3, tilt ~ (0, 1)
  - Both beacons visible, target rolled 90° → tilt ~ (1, 0) or (-1, 0) depending on roll sign
  - Either beacon CEP ≥ threshold → all 3 derived feature outputs at neutral values
  - Degenerate pair (both project to same NDC point, both CEP < threshold) → tilt at neutral via degenerate-epsilon guard
  - Multi-tick: span_rate from two consecutive frames matches the manual subtraction

## Citations

- [spec.md](../spec.md) §1.5 Feature B
- [research.md](../research.md) R2, R3, R4
- [data-model.md](../data-model.md) §1, §3
- [030 nn_sensor_interface contract](../../030-tracker-mode/contracts/nn_sensor_interface.md)

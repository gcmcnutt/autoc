# Contract: History Buffer Reset on Engage (Change 1)

**Feature**: 023 | **Phase**: 1 | **Scope**: `AircraftState::resetHistory()` semantics

## Motivation

Analysis of the 2026-04-07 flight found that re-engages (2nd+ autoc engage per
flight) inherit stale history buffer contents from the previous disengage state.
Closing rate `dd/dt` computed from `(stale_prev - fresh_now)` produced spurious
values exceeding +1100 m/s in real flight — garbage NN inputs for the first ~9
samples (900 ms) until the buffer cycled through.

Clarified 2026-04-08 (Q1): use Option A — pre-fill all history slots with
current in-autoc geometry on every engage transition (not just the first).

## Contract

### Signature

```cpp
// include/autoc/eval/aircraft_state.h
class AircraftState {
public:
    // ... existing members ...

    /**
     * Reset NN history buffer to a "flat" state matching current in-autoc geometry.
     *
     * Called at every autoc engage transition (false → true). Pre-fills all
     * history samples AND lookahead samples in NNInputs with the values derived
     * from (aircraft state, armed path) at the current tick. After calling:
     *
     *   - All target_x/y/z time samples contain the same unit vector
     *   - All dist time samples contain the same value
     *   - closing_rate is 0 (all history samples are identical, so dd/dt = 0)
     *   - Quaternion, airspeed, gyro fields are from the current state (unchanged)
     *
     * @param armedPath  The path the autoc is about to track (rabbit comes from here)
     * @param currentState The aircraft's state at the engage transition
     */
    void resetHistory(const Path& armedPath, const AircraftState& currentState);
};
```

### Semantics

Called **on every autoc transition `false → true`**, not just the first. There
is no "first engage special case" — same code runs every time.

After `resetHistory()` returns:

1. `nnInputs_.target_x[0..5]`, `target_y[0..5]`, `target_z[0..5]` all contain
   the same unit vector (the one pointing from aircraft to current rabbit
   position, in body frame, as computed by `computeTargetDir()`).
2. `nnInputs_.dist[0..5]` all contain the same value (current aircraft-to-rabbit
   distance).
3. `nnInputs_.closing_rate = 0.0f`. All history samples are identical, so
   `dd/dt = (dist[3] - dist[2]) / dt = 0`.
4. `nnInputs_.quat_w/x/y/z` set from `currentState.getQuaternion()` (no time
   series for these — they're scalars).
5. `nnInputs_.airspeed` set from `currentState.getVelocity().norm()` (or the
   current groundspeed convention).
6. `nnInputs_.gyro_p/q/r` set from `currentState.getAngularRates()`.

No fields are left uninitialized. Designated-initializer or field-by-field
assignment, exhaustive.

### Interaction with Change 1b (engage delay window)

`resetHistory()` runs **at the engage transition**, BEFORE the first tick of
the delay window. The delay window then advances history normally for the next
~7 ticks. By the time the delay window closes, the history buffer has been
completely overwritten by real in-flight samples from the delay window itself.

This means the pre-fill's practical impact is smaller than first thought:

- During the delay window, the NN is running but its outputs are ignored.
- The pre-fill protects the NN's first few forward-pass inputs from stale
  garbage, but those forward-pass outputs don't reach the servos.
- By the time the NN's outputs DO reach the servos (tick 8+), the history
  has been naturally refilled.

Still necessary — those early forward passes influence the NN's internal
state evolution (e.g., any kind of recurrent / memory-like behavior the
policy may have learned) — just not as dominant as originally framed.

### Shared code (sim + xiao)

`resetHistory()` must execute the **identical** computation on both sim and
xiao sides. Implementation:

- Define `computeTargetDir(vec3 target_body, float dist, vec3 rabbit_vel_dir_body)`
  in a header that both sim and xiao include (probably
  `include/autoc/nn/nn_input_computation.h` — NEW).
- `AircraftState::resetHistory()` calls this helper.
- CRRCSim, minisim, and xiao all call `AircraftState::resetHistory()` at the
  engage transition.

No divergence between sim and xiao reset code. Any future change to reset
semantics lands in one place.

## `computeTargetDir()` helper contract

### Signature

```cpp
// include/autoc/nn/nn_input_computation.h (NEW)

#include <Eigen/Core>

struct Vec3 { float x, y, z; };  // or use gp_vec3 from existing headers

constexpr float DIR_NUMERICAL_FLOOR = 1e-4f;  // meters

/**
 * Compute the unit target direction vector in body frame.
 *
 * Normal case: returns target_body / dist (unit vector).
 * Singularity case (dist < DIR_NUMERICAL_FLOOR): returns rabbit_vel_dir_body
 * (the path tangent in body frame, already a unit vector).
 *
 * The hard switch at DIR_NUMERICAL_FLOOR is intentional per Q3 clarify
 * decision. At 10 Hz + 18 m/s the aircraft covers ~1.8m/tick; any "smooth"
 * blend window is traversed in a fraction of a tick so it's a step change
 * either way. The floor is only triggered when the divide is numerically
 * unsafe (~0.0001 m).
 */
Vec3 computeTargetDir(Vec3 target_body, float dist, Vec3 rabbit_vel_dir_body);
```

### Pseudocode

```cpp
Vec3 computeTargetDir(Vec3 target_body, float dist, Vec3 rabbit_vel_dir_body) {
    if (dist < DIR_NUMERICAL_FLOOR) {
        return rabbit_vel_dir_body;  // fallback at true singularity
    }
    const float inv_dist = 1.0f / dist;
    return {
        target_body.x * inv_dist,
        target_body.y * inv_dist,
        target_body.z * inv_dist
    };
}
```

### Invariants

1. **Output is always unit-normed** (within FP rounding): `x² + y² + z² ≈ 1`.
   Fallback case relies on caller passing a unit-normed `rabbit_vel_dir_body`.
2. **No trig**. No `cos`, `sin`, `atan2`, or table lookups. Only float divide
   and multiply.
3. **Branch-free fast path** (optional optimization, not a contract): the
   normal case is `target_body * (1/dist)`, 3 multiplies and 1 divide. On
   Cortex-M33 FPU: ~18 cycles. Negligible.

### Test coverage (`tests/nn_inputs_tests.cc`)

```cpp
TEST(ComputeTargetDirTest, NormalCase) {
    // target = (3, 4, 0), dist = 5 → (0.6, 0.8, 0)
}

TEST(ComputeTargetDirTest, SingularityUsesRabbitVelocity) {
    // dist = 1e-5, rabbit_vel_dir = (1, 0, 0) → (1, 0, 0)
}

TEST(ComputeTargetDirTest, UnitVectorInvariant) {
    // Random (target, dist) pairs over a few thousand samples.
    // Assert x² + y² + z² ∈ (0.9999, 1.0001) for all.
}

TEST(ComputeTargetDirTest, NoTrigCallsInAssembly) {
    // Optional: disassemble the compiled helper, assert no call to cosf/sinf/atan2f.
    // Skip if build infrastructure doesn't support this.
}
```

## Files touched

| File | Change |
|---|---|
| `include/autoc/eval/aircraft_state.h` | Add `resetHistory()` declaration |
| `src/eval/aircraft_state.cc` | Implement `resetHistory()` using `computeTargetDir()` |
| `include/autoc/nn/nn_input_computation.h` | NEW — declare `computeTargetDir()` and `DIR_NUMERICAL_FLOOR` |
| `src/nn/nn_input_computation.cc` | NEW — implement `computeTargetDir()` |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | Call `resetHistory()` at engage transition |
| `tools/minisim.cc` | Call `resetHistory()` at scenario start (minisim engage) |
| `xiao/src/msplink.cpp` | Call `resetHistory()` at autoc enable transition (currently around L402) |
| `tests/engage_reset_tests.cc` | NEW — unit tests for `resetHistory()` semantics |
| `tests/nn_inputs_tests.cc` | Add `computeTargetDir()` tests |

## Verification

### Unit tests (`tests/engage_reset_tests.cc` NEW)

```cpp
TEST(ResetHistoryTest, AllHistorySlotsIdentical) {
    // Construct synthetic AircraftState + Path
    // Call resetHistory()
    // Assert nnInputs_.target_x[0] == nnInputs_.target_x[1] == ... == [5]
    // Same for target_y, target_z, dist
}

TEST(ResetHistoryTest, ClosingRateZero) {
    // After resetHistory(), nnInputs_.closing_rate == 0.0f
}

TEST(ResetHistoryTest, UnitVectorPopulated) {
    // After resetHistory(), target_x/y/z at every time sample is a unit vector
}

TEST(ResetHistoryTest, ScalarFieldsFromCurrentState) {
    // Quaternion, airspeed, gyro fields match the currentState argument
}

TEST(ResetHistoryTest, SingularityFallbackOnZeroDistance) {
    // If aircraft is exactly at rabbit position (dist < 1e-4),
    // the direction comes from path tangent, not NaN
}

TEST(ResetHistoryTest, FirstEngageAndReEngageAreIdentical) {
    // Call resetHistory() twice with different prior states in between.
    // Both calls must produce identical nnInputs_ given identical
    // (armedPath, currentState) arguments.
    // i.e., no "first engage special case" leakage.
}
```

### Integration test (Phase 3 Milestone A)

Run one CRRCSim scenario with verbose logging. Log `nnInputs_` on every tick
during the engage delay window. Assert:

1. On tick 0 (engage transition): `target_x[0..5]` all equal, `dist[0..5]` all
   equal, `closing_rate == 0`.
2. On ticks 1-7 (window active): history shifts normally, closing_rate becomes
   nonzero as samples diverge.
3. Compare first-engage and re-engage behavior across two different scenarios:
   both must show the identical pattern.

If any of these fail, stop and fix before proceeding.

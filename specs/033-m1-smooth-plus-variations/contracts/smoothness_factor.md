# Contract: per-tick smoothness factor + `applyStreak()` integration

**Producer**: `compute_smoothness_factor()` pure helper in [include/autoc/eval/derived_features.h](../../../include/autoc/eval/derived_features.h) (extended from 032 phase-1 pattern).
**Consumer**: `FitnessComputer::applyStreak()` in [src/eval/fitness_computer.cc](../../../src/eval/fitness_computer.cc) (extended signature) called from per-tick caller in `PathgenStepper` / `TrackerStepper` per-tick loop (which also owns the previous-NN-output tracking state).

## Pure-math contract

```cpp
namespace autoc::eval {

enum class SmoothnessMotionMode {
  Pythagorean,  // sqrt(dpt² + drl² + dth²);    motion_max = sqrt(12) ≈ 3.464
  Sum,          // |dpt| + |drl| + |dth|;       motion_max = 6.0
  Max,          // max(|dpt|, |drl|, |dth|);    motion_max = 2.0
};

inline gp_scalar compute_smoothness_factor(
    gp_scalar dpt, gp_scalar drl, gp_scalar dth,
    gp_scalar floor,
    SmoothnessMotionMode mode = SmoothnessMotionMode::Pythagorean);
```

### Returns

`gp_scalar` in `[floor, 1.0]`:
- **1.0** ⇔ `motion == 0` (controller at rest, no Δ since last tick)
- **floor** ⇔ `motion ≥ motion_max` (maximum bang-bang on all 3 axes simultaneously)
- **Linear interpolation** between for partial motion: `factor = 1.0 - (1.0 - floor) × min(motion / motion_max, 1.0)`

### Inputs

- `dpt`, `drl`, `dth`: per-tick Δ for pitch / roll / throttle output. Each is `curr_output - prev_output` where outputs are NN-tanh-saturated to `[−1, +1]`. Max possible |Δ| per axis is 2.0 (full-swing from −1 to +1 in one tick).
- `floor`: configuration value from `cfg.smoothnessPenaltyFloor`. Range-checked at config-load time to `[0.0, 1.0]`. Loud-fail on out-of-range.
- `mode`: configuration value from `cfg.smoothnessMotionMode`. Default `Pythagorean`.

### Edge cases

- **Floor == 1.0**: factor always 1.0 — no-op, back-compat / regression-test mode. Stepper still calls the function (for code-path test coverage); just returns 1.0 every tick.
- **Floor == 0.0**: factor in `[0.0, 1.0]`. At max-motion, stepPoints completely annihilated. (Per spec, this is allowed by range check but extreme.)
- **All Δs == 0 (controller at rest)**: factor = 1.0; multiplicative no-op on stepPoints.
- **Single-axis bang-bang** under Pythagorean mode: `motion = 2.0` → `2.0/3.464 ≈ 0.577` → factor ≈ `1.0 − 0.5 × 0.577 = 0.712`. Single-axis bang-bang costs ~29% of stepPoints with floor=0.5.
- **All-3-axis bang-bang** under Pythagorean mode: `motion = sqrt(12) = 3.464` → ratio = 1.0 (capped) → factor = floor = 0.5. Maximum penalty.
- **First tick of scenario**: stepper passes `dpt = drl = dth = 0` (no previous output to diff against). Factor = 1.0. No "false positive" penalty on scenario start.

## `FitnessComputer::applyStreak()` signature extension

```cpp
// include/autoc/eval/fitness_computer.h (modified)
class FitnessComputer {
public:
  // ... existing methods unchanged ...

  // Apply smoothness penalty (multiplicative on stepPoints) THEN streak multiplier.
  // smoothness_factor in [0, 1]: 1.0 = no penalty (passes stepPoints through unmodified);
  // per spec 033 §2.B operator-preferred form, 0.5 is the YOLO start.
  //
  // NO DEFAULT VALUE per Constitution III (no compat shims). Every existing caller
  // is updated in the 033 PR to pass an explicit smoothness_factor — production
  // callers pass the per-tick computed factor; unit-test sites that exercise the
  // streak machinery in isolation pass an explicit `static_cast<gp_scalar>(1.0)`.
  gp_fitness applyStreak(gp_scalar stepPoints, gp_scalar smoothness_factor);
};

// src/eval/fitness_computer.cc (modified at line ~57)
gp_fitness FitnessComputer::applyStreak(gp_scalar stepPoints,
                                         gp_scalar smoothness_factor) {
  // ... existing streak-state-update logic (consecutive-streak counter, etc.) ...
  const gp_scalar penalized = stepPoints * smoothness_factor;
  return static_cast<gp_fitness>(penalized * multiplier);
  //                                  ^^^^^ existing multiplier computation
}
```

### Multiplication order is the contract

`final_step = stepPoints × smoothness_factor × streak_multiplier`

NOT `final_step = (stepPoints + smoothness_factor) × streak_multiplier` (which would be additive penalty)
NOT `final_step = stepPoints × max(smoothness_factor, streak_multiplier)` (would clobber streak signal)
NOT `final_step = stepPoints × streak_multiplier × smoothness_factor` (mathematically same — commutative — but the operational reading is "discount per-tick value, THEN compound"; preserving this order in code reads cleaner)

## Per-tick stepper integration

```cpp
// In PathgenStepper::stepOnce() and TrackerStepper::stepOnce() — pattern is identical

// Stepper member state (initialized in initScenario):
//   gp_scalar prev_out_pt_, prev_out_rl_, prev_out_th_;
//   bool prev_out_valid_;

void Stepper::stepOnce() {
  // 1. NN forward-pass (existing)
  const gp_scalar curr_pt = state.getPitchCommand();
  const gp_scalar curr_rl = state.getRollCommand();
  const gp_scalar curr_th = state.getThrottleCommand();

  // 2. Compute smoothness factor (NEW)
  gp_scalar dpt = 0, drl = 0, dth = 0;
  if (prev_out_valid_) {
    dpt = curr_pt - prev_out_pt_;
    drl = curr_rl - prev_out_rl_;
    dth = curr_th - prev_out_th_;
  }
  const gp_scalar smoothness_factor =
      autoc::eval::compute_smoothness_factor(
          dpt, drl, dth,
          static_cast<gp_scalar>(cfg.smoothnessPenaltyFloor),
          cfg.smoothnessMotionMode);

  // 3. Compute step score (existing) + apply with smoothness (MODIFIED)
  const gp_scalar stepPoints = fc_.computeStepScore(along, lateral_dist);
  const gp_fitness final = fc_.applyStreak(stepPoints, smoothness_factor);
  scenario_fitness_ += final;

  // 4. Update prev_out state for next tick (NEW)
  prev_out_pt_ = curr_pt;
  prev_out_rl_ = curr_rl;
  prev_out_th_ = curr_th;
  prev_out_valid_ = true;
}

void Stepper::initScenario() {
  // ... existing init ...
  prev_out_valid_ = false;  // reset; first tick of next scenario gets factor=1.0
}
```

## Validation tests

`tests/derived_features_tests.cc` (EXTENDED — pure math):
- `compute_smoothness_factor(0,0,0, floor=0.5, Pythagorean) == 1.0` (rest = no penalty)
- `compute_smoothness_factor(2,2,2, floor=0.5, Pythagorean) == 0.5` (max-motion = full penalty)
- `compute_smoothness_factor(2,0,0, floor=0.5, Pythagorean) ≈ 0.7113` (single-axis bang-bang)
- `compute_smoothness_factor(1,1,1, floor=0.5, Pythagorean) ≈ 0.75` (sqrt(3)/sqrt(12) = 0.5 ratio → factor 0.75)
- `compute_smoothness_factor(2,2,2, floor=1.0, Pythagorean) == 1.0` (no-op floor)
- `compute_smoothness_factor(3,3,3, floor=0.5, Pythagorean) == 0.5` (out-of-range Δ clamps to motion_max)
- `compute_smoothness_factor(*, floor=0.0, *) ∈ [0, 1]` range invariant
- Mode comparison: same Δ values, different modes, different factors (Pythagorean vs Sum vs Max)
- Property: `factor ∈ [floor, 1.0]` for ALL inputs (fuzz random Δs in `[-3, +3]` range, all floors in `[0, 1]`, all modes)

`tests/fitness_computer_tests.cc` (EXTENDED — integration):
- `applyStreak(100, smoothness=1.0)` matches existing baseline (back-compat)
- `applyStreak(100, smoothness=0.5)` returns half of `applyStreak(100, smoothness=1.0)` for the same streak state
- Streak state update is identical regardless of smoothness factor (factor doesn't perturb the streak machinery itself)

`tests/stepper_smoothness_tests.cc` (NEW — per-tick state):
- `prev_out_valid_` is false at scenario start; first tick gets factor=1.0
- Constant-output controller (no Δs) gets factor=1.0 every tick
- Bang-bang controller (full-swing pitch every tick) gets factor near `floor` after the first tick

## Type-domain compliance (Constitution VI)

- `compute_smoothness_factor` inputs/outputs are `gp_scalar` (eval-pipeline scalars).
- Intermediates (motion, motion_max, t) are `gp_scalar`.
- `std::sqrt`, `std::max`, `std::min`, `std::abs` are math-library-imposed signatures — implicit float promotion handled by `gp_scalar` typedef matching `float`. No raw `float` annotation needed in the helper.
- `applyStreak`'s `smoothness_factor` parameter is `gp_scalar`.
- Per-tick `dpt/drl/dth` in the stepper are computed from `state.getPitchCommand()` etc. which return `gp_scalar` — no conversion at the boundary.

## Citations

- [spec.md](../spec.md) §2.B Clarifications Q4 (Pythagorean default) + Q5 (worker-side placement)
- [research.md](../research.md) R1 (applyStreak insertion) + R8 (pure-math placement in derived_features.h)
- [Constitution I](../../../.specify/memory/constitution.md) — TDD: pure-math tests written first
- [Constitution VI](../../../.specify/memory/constitution.md) — type-domain discipline

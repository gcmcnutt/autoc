# Contract: ini schema extension — flat smoothness knobs

**REVISED 2026-05-21**: This contract originally proposed a `[Smoothness]`
section header. Operator feedback during /speckit.implement turn 2 corrected
the design — autoc.ini convention is flat keys (parsed via
`reader.GetReal("", "Key", default)`); the `[DerivedFeatures]` section in
autoc-tracker.ini is a 032-era exception, not the convention. The two
smoothness knobs (`SmoothnessPenaltyFloor`, `SmoothnessMotionMode`) are
flat keys at the top level of each ini file. Section header references
below are corrected; the schema, validation, and WorkerInit propagation
remain unchanged.

**Producer**: operator (hand-edits ini files), bake automation scripts.
**Consumer**: `src/util/config.cc::ConfigManager::load()` parses the section into `AutocConfig` fields; values are propagated to workers via `WorkerInit` priming RPC; workers consume per-tick in `compute_smoothness_factor()` calls.

## Schema

Flat keys at top-level (no section header — autoc.ini convention).

```ini
# 033 phase-1 smoothness knobs (flat keys; no section header)
# Lower bound for the multiplicative per-step smoothness factor.
# factor = 1.0 (when motion = 0, controller at rest)
# factor = floor (when motion is at its mode-dependent maximum)
# Range: [0.0, 1.0]. Loud-fail on out-of-range.
#
# Operator-preferred starting value per spec 033 §2.B Q4: 0.5 (YOLO start).
# Setting to 1.0 disables the penalty (back-compat / regression-test mode).
SmoothnessPenaltyFloor=0.5

# How per-tick output Δs (dpt, drl, dth) are combined into the scalar
# `motion` term that drives the linear interpolation.
#   pythagorean: sqrt(dpt² + drl² + dth²);    motion_max = sqrt(12) ≈ 3.464
#   sum:         |dpt| + |drl| + |dth|;       motion_max = 6.0
#   max:         max(|dpt|, |drl|, |dth|);    motion_max = 2.0
# Default: pythagorean (per Q4 operator choice — penalizes simultaneous-axis
# bang-bang harder than single-axis, encouraging spread control effort).
# Loud-fail on unknown value.
SmoothnessMotionMode=pythagorean
```

## File coverage

Add the two flat smoothness keys to all three operator ini files:

- `autoc.ini` (M1 minisim / pathgen)
- `autoc-tracker.ini` (M2 crrcsim tracker, primary 033 bake target — though 033 phase 1 is M1-only, M2 inherits the section seamlessly)
- `autoc-tracker-minisim.ini` (M2 minisim tracker, used for fast iteration)

All three ini files MUST have identical default values to keep config-parity verification simple.

## `AutocConfig` field surface

```cpp
// include/autoc/util/config.h (modified)

namespace autoc::eval {
enum class SmoothnessMotionMode {  // re-exported / aliased from derived_features.h
  Pythagorean,
  Sum,
  Max,
};
}

struct AutocConfig {
  // ... existing fields unchanged ...

  // 033 §2.B — multiplicative per-step smoothness penalty
  double smoothnessPenaltyFloor = 0.5;                  // raw-ok at ini-boundary; cast to gp_scalar at consumer
  autoc::eval::SmoothnessMotionMode smoothnessMotionMode =
      autoc::eval::SmoothnessMotionMode::Pythagorean;
};
```

### Type-domain notes

- `smoothnessPenaltyFloor` is `double` at the config boundary (raw-ok annotated as per existing pattern), cast to `gp_scalar` at the worker call site (`compute_smoothness_factor` call in stepper).
- `smoothnessMotionMode` enum lives in `autoc::eval::` namespace alongside `compute_smoothness_factor()` (single home for smoothness-related types).

## Parsing behavior

`src/util/config.cc::ConfigManager::load()` extension. Parse from the
default section (`""`) per autoc.ini flat-key convention (matches the
`reader.GetReal("", "Key", default)` pattern already used for evolution +
fitness knobs):

```cpp
// Read SmoothnessPenaltyFloor with range check
const double rawFloor = iniReader.GetReal("", "SmoothnessPenaltyFloor", 0.5);
if (rawFloor < 0.0 || rawFloor > 1.0) {
  LOG_FATAL("SmoothnessPenaltyFloor out of range [0.0, 1.0]: " << rawFloor);
  std::abort();
}
cfg.smoothnessPenaltyFloor = rawFloor;

// Read SmoothnessMotionMode with enum mapping
const std::string rawMode = iniReader.GetString("", "SmoothnessMotionMode", "pythagorean");
if      (rawMode == "pythagorean") cfg.smoothnessMotionMode = SmoothnessMotionMode::Pythagorean;
else if (rawMode == "sum")         cfg.smoothnessMotionMode = SmoothnessMotionMode::Sum;
else if (rawMode == "max")         cfg.smoothnessMotionMode = SmoothnessMotionMode::Max;
else {
  LOG_FATAL("SmoothnessMotionMode unknown: '" << rawMode << "' (expected: pythagorean|sum|max)");
  std::abort();
}
```

### Defaults

- `SmoothnessPenaltyFloor=0.5` — YOLO start per Q4
- `SmoothnessMotionMode=pythagorean` — per Q4

Defaults intentionally chosen so a freshly-checked-out repo with unedited ini files runs the 033 reward shape (not the no-penalty back-compat). Operator who wants 029-style back-compat (no penalty) must explicitly set `SmoothnessPenaltyFloor=1.0`.

### Missing section / missing keys

- If either smoothness key is absent from the ini: default applies (parse-with-fallback via inih `GetReal`/`GetString`). LOG a one-shot WARN at load time so the operator notices their ini is pre-033.

## `WorkerInit` propagation

`crrcsim` is a separate process, so config values must cross the RPC boundary the same way `cepGateThreshold` does (per 032 precedent).

```cpp
// include/autoc/rpc/protocol.h (modified)

struct WorkerInit {
  // ... existing fields unchanged ...
  double smoothnessPenaltyFloor;                  // 033 — passed from AutocConfig
  uint8_t smoothnessMotionMode;                   // 033 — enum-to-uint8 for wire stability (0=Pythagorean, 1=Sum, 2=Max)
  // ... cerial serialize() walk extended with both fields ...
};
```

Worker (`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`) decodes back to its local enum + scalar at WorkerInit receipt, then uses on every per-tick `compute_smoothness_factor()` call.

Minisim path (`src/eval/...`) skips WorkerInit and consumes `AutocConfig` directly via `ConfigManager` singleton (existing pattern).

## Validation tests

`tests/contract_config_tests.cc` (EXTENDED):

- `parse_smoothness_keys_defaults`: ini lacking the two smoothness flat keys → fields default to `0.5` + `Pythagorean`, WARN logged
- `parse_smoothness_floor_in_range`: floor=0.7 → field equals 0.7
- `parse_smoothness_floor_out_of_range_high`: floor=1.5 → LOG_FATAL + abort
- `parse_smoothness_floor_out_of_range_low`: floor=-0.1 → LOG_FATAL + abort
- `parse_smoothness_mode_pythagorean`: mode="pythagorean" → enum Pythagorean
- `parse_smoothness_mode_sum`: mode="sum" → enum Sum
- `parse_smoothness_mode_max`: mode="max" → enum Max
- `parse_smoothness_mode_unknown_loud_fail`: mode="banana" → LOG_FATAL + abort
- `worker_init_smoothness_roundtrip`: serialize WorkerInit with non-default floor + mode → deserialize → assert bitwise-equal

## Operator-facing impact

- Existing autoc.ini, autoc-tracker.ini, autoc-tracker-minisim.ini will gain the section in the 033 commit.
- Operator who has local-modified ini files should `git diff` the section in to their local copies (or accept the commit's version and re-apply their other tweaks).
- The `SmoothnessPenaltyFloor=1.0` setting is the "disable" lever for operators who want to A/B compare with smoothness OFF without code changes.

## Citations

- [spec.md](../spec.md) §2.B (smoothness penalty)
- [spec.md](../spec.md) Clarifications Q4 (Pythagorean) + Q5 (worker side)
- [smoothness_factor.md](./smoothness_factor.md) (consumer of the floor + mode values)
- [research.md](../research.md) R1 + R8 (smoothness placement)
- [Constitution VI](../../../.specify/memory/constitution.md) — type-domain discipline (raw-ok at ini boundary, gp_scalar at consumer)

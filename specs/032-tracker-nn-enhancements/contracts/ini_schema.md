# Contract: `autoc-tracker.ini` schema extension

**File**: `/home/gmcnutt/autoc/autoc-tracker.ini`
**Parser**: inih (existing dependency, no new code)

## New section: `[DerivedFeatures]`

```ini
[DerivedFeatures]
CepGateThreshold                = 1.25
# Derived features (beacon_pair_span, span_rate, target_tilt_*) substitute
# neutral values when EITHER beacon's CEP at the current tick is >= this
# threshold. Default matches kCepSentinelThreshold from
# include/autoc/eval/camera_projection.h (any beacon visible at the existing
# 030 sentinel boundary is also gated here).
# Lower values gate more aggressively on noisy-but-present detections.
# Per spec Q4 + research.md R2.
```

## Loader contract

The autoc-tracker.ini loader (existing inih-based code in `src/util/config.cc` + struct in `include/autoc/util/config.h`) gains parsing for:

- `CepGateThreshold`: floating-point, range [0.0, 2.0]. Validation: reject < 0 or > 2.0 with a clear error message at startup. Default 1.25.

Loaded once at autoc startup and threaded to the eval pipeline via the existing config struct mechanism. Per-eval reload is NOT supported (no use case in phase 1).

## NOT included: `EnableDerivedFeatures` runtime flag

Earlier drafts of this contract proposed an `EnableDerivedFeatures = 0|1` ini knob to support phase-1b B-off attribution bakes (per research.md R7). **Removed 2026-05-16** per operator direction: greenfield M2 means the code IS the change; if we want "without derived features" we `git checkout` a pre-032 commit and bake from there. The runtime toggle would add a dead conditional branch + dead config + dead test for a hypothetical follow-on that git already supports. See research.md R7 for the updated attribution methodology.

## NOT in v1 (intentional exclusions)

These are tempting to add but rejected:

- `TiltDegenerateEpsilon` — held at compile-time constant `kTiltDegenerateEpsilon = 1e-4f` per research.md R4. No operational case for tuning at runtime.
- `SpanSmoothingAlpha` — span is raw NDC distance; no smoothing. Smoothed trend already lives in the span[6] history.
- `SpanRateGateMode` — phase-1 span_rate has no own CEP-gate (mechanically derives from span[5]-span[4]); if visibility transitions create training instability, address via code revision, not a runtime flag.
- `EnableIdentityStableOrdering` — sim ordering is identity-stable by construction (research.md R1); no toggle.
- Per-feature enable flags (`EnableSpanHistory`, `EnableSpanRate`, `EnableTilt`, or a single `EnableDerivedFeatures`) — greenfield M2 means the code IS the change; B-off attribution comes from git revert per research.md R7, not a runtime knob.

## M1 isolation

`autoc.ini` (M1 pathgen mode config) is NOT touched. The `[DerivedFeatures]` section is tracker-mode-only and ignored if present in `autoc.ini`. M1 bakes remain bitwise-equal pre/post 032.

## Validation tests

`tests/contract_tracker_config_tests.cc` (extension):

- Default values: omit `[DerivedFeatures]` entirely → `CepGateThreshold == 1.25f`
- Explicit override: set `CepGateThreshold = 0.75` → loaded value matches
- Canonical default: repo-root `autoc-tracker.ini` ships with `CepGateThreshold = 1.25`
- Out-of-range rejection: `CepGateThreshold = -1.0` calls `exit(1)` with a clear error message naming the bad value (loud-fail at startup; not unit-testable without process fork)

## Citations

- [spec.md](../spec.md) Clarifications Q4 (CEP threshold), Q7 (attribution-bake mechanism)
- [research.md](../research.md) R2 (threshold default), R6 (ini schema), R7 (attribution-bake toggle)
- [feedback_cli_config_flag_convention](../../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_cli_config_flag_convention.md) — autoc uses `-i <ini>` flag (no CLI change in phase 1)

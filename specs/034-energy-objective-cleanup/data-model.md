# Phase 1 Data Model: 034 Schema Deltas

**Date**: 2026-05-29
**Policy**: Greenfield cereal changes — **NO version revision for transport or .dmp during the M2 initiative** (operator directive 2026-05-29; `feedback_no_cereal_versioning`). No version bump, no migration shim, no bespoke fail-loud test; old dmps are orphaned by the training reset. Constructor-supplied members carry NO in-class defaults (Constitution VII). New eval-pipeline scalars are `gp_scalar` (Constitution VI).

---

## Entities changed

### `WorkerInit` (`include/autoc/rpc/protocol.h`)
- **REMOVE**: `smoothnessPenaltyFloor`, `smoothnessMotionMode` (+ their `serialize()` entry). Scenario-invariant smoothness config is gone.
- **No craft fields here** — craft params are per-scenario, not scenario-invariant.

### `ScenarioMetadata` (`include/autoc/rpc/protocol.h`)
- **REMOVE**: `smoothnessPenaltyFloor`, `smoothnessMotionMode` (+ `serialize()` + `reset()` entries). *(Non-obvious duplicate carrier — flagged in research R1.)*
- **ADD** (US4): per-scenario craft draw — the realized deltas + the seed that produced them:
  - `gp_scalar craftCGDelta` — additive on `CG_arm`
  - `gp_scalar craftDragDelta` — fractional on `CD_prof` (×(1+δ))
  - `gp_scalar craftTrimDelta` — additive on `Cm_0`
  - `gp_scalar craftThrustScale` — multiplicative on max thrust
  - `gp_scalar craftPitchEffDelta` — fractional on pitch control authority (`Cm_de`/`CL_de`)
  - `gp_scalar craftRollEffDelta` — fractional on roll control authority (aileron→roll coeff); independent of pitch → slight asymmetry
  - *(servo lag time-constant DEFERRED per D1 — not added)*
  - `uint32_t craftSeed` — the sub-seed that generated this scenario's craft draw (FR-020; reproducibility/replay)
  - All appended to `serialize()` in declared order; reset in `reset()`. No in-class defaults (Constitution VII) — assigned at construction from the craft sampler.
- **Camera-seed forward-compat**: leave a documented insertion point so a future `cameraSeed` can append without another schema break (FR-020 note). Not added now.

### Dropped fold-ins (D2 — verified already satisfied in-tree)
- **FR-013** crash-hull PRNG already seeds from `deriveClassSubSeeds(scenarioSeed).rabbit` (`crrcsim_tracker_helper.cpp:54-56`). No change.
- **FR-014** `mod_inputdev` already links `autoc_common` (`crrcsim/src/mod_inputdev/CMakeLists.txt:26-30`). No change.

### `AircraftState` (`include/autoc/eval/aircraft_state.h`)
- **REMOVE**: `smoothnessFactor_` member, `getSmoothnessFactor()`/`setSmoothnessFactor()`, and the `serialize()` entry. dmp per-tick state no longer carries smoothness.

### `AutocConfig` (`include/autoc/util/config.h`)
- **REMOVE**: `smoothnessPenaltyFloor` (double), `smoothnessMotionMode` (string) + their parse (`config.cc:170-191`).
- **RENAME**: `minisimProgram` → `workerProgram`, `minisimPortOverride` → `workerPortOverride` (clean, no alias).
- **ADD** (US4): craft-variation magnitude knobs — fractional-σ Gaussian, ramped via shared `applyVariationScale()`:
  - `gp_scalar craftCGSigma`, `craftDragSigma`, `craftTrimSigma`, `craftThrustSigma`, `craftPitchEffSigma`, `craftRollEffSigma`. Default 0.0 in `.ini` = no-op. (No servo-lag sigma — deferred.)
- **X-macro target**: the struct + parse + startup-print become a single `AUTOC_CONFIG_FIELDS(X)` list (US3 #1). Field count ~82 + craft knobs.

### `data.dat` (writers in `src/autoc.cc`)
- **REMOVE** the `smooth` column (pathgen idx 41 / tracker idx 62): header label, format token, value source. Downstream columns shift down by one.
- **NO craft columns added** (craft is per-scenario constant → lives in `ScenarioMetadata`/dmp, not per-tick trace; R2 decision). NN_INPUT_COUNT constants untouched.

### `.ini` files (`autoc.ini`, `autoc-tracker.ini`, `autoc-eval*.ini`)
- **REMOVE** `[Smoothness]` sections.
- **RENAME** `MinisimProgram`→`WorkerProgram`, `MinisimPortOverride`→`WorkerPortOverride`.
- **ADD** `[Craft]` section with the σ knobs (default 0.0).

---

## CRRCSim-side carriers (`crrcsim/src/global.h`)
Mirror the existing `Global::entry*Offset` pattern — add `Global::craftCGDelta / craftDragDelta / craftTrimDelta / craftThrustScale [/ craftServoTau]`, populated at the per-scenario reset hook (`inputdev_autoc.cpp:494-520`) and consumed in `initAirplaneState()` / `engine()`.

## Validation rules
- **No-op**: every σ = 0 → all deltas exactly 0 (additive) / 1.0 (multiplicative) → bit-identical to nominal craft (FR-021, SC-004).
- **Determinism**: `craftSeed` = a class sub-seed from `deriveClassSubSeeds(scenarioSeed)`; same `scenarioSeed` → same craft draw bit-for-bit (FR-018, SC-005). Ramping: drawn deltas are stored at full magnitude in `ScenarioMetadata`; `applyVariationScale()` scales them per eval (FR-019). Eval mode replays the saved `genome.variation_scale` exactly — same mechanism as wind/entry.
- **Type domain**: all craft scalars `gp_scalar`; FDM coefficient targets (`CD_prof`, `Cm_0`, `CG_arm`) are CRRCSim-native `double`/`SCALAR` — the autoc→crrcsim boundary is the `raw-ok` annotation site if a cast is needed.
- **No version ceremony**: no version-field bump on transport or dmp; old dmps orphaned by training reset (not migrated). No bespoke fail-loud contract test added — rely on normal cereal error paths if a stale dmp is loaded.

## State transitions
Craft params are static per scenario (one Gaussian draw at startup, deterministic per `scenarioSeed`). The per-eval `applyVariationScale()` adjusts the *applied* magnitude based on `genome.variation_scale` (training: gen-based ramp; eval: saved scale replay). The FDM applies the final scaled deltas once at scenario init and they remain constant through the trajectory.

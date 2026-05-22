# Variation PRNG architecture — operator guide

**Status**: 033 phase 1 in flight. This document describes the design that lands across 033 phase 1 + cleanup pass; current branch state is transitional (see "Transitional state" section at the end).

## What it is

033 introduces a **master-seed → per-scenario sub-PRNG** architecture for all per-scenario variation draws (wind, rabbit, entry pose, future craft, future camera). The goal: **bit-deterministic replay** per `(NN, scenarioSeed[K])` — the same NN + same scenario seed produces bit-identical per-tick aircraft state across runs, modes (M1 pathgen / M2 tracker), and downstream tooling (eval mode, renderer, tracker-mode source library).

Before 033 the joint-PRNG architecture meant scenarios drew sequentially from a single global stream; en/disabling a variation type **shifted** every downstream draw, blurring ablation experiments and breaking cross-mode wind equivalence.

## The master-seed contract

Two operator-facing knobs in `[Evolution]` of `autoc.ini` / `autoc-tracker.ini`:

```ini
Seed = -1     # -1 = wall-clock (printed at startup); N = reproduce a prior run
```

At autoc startup the **effective master seed** is computed and logged on the very first line:

```
[INFO] Effective master seed: 1729123456789 (operator: copy this value into eval-mode ini Seed=N to reproduce this run bit-deterministically)
```

Copy that value into a later run's `Seed=N` (or eval-mode ini) and the entire variation table is reproduced bit-for-bit.

## Init chain (autoc-side)

```
effectiveSeed = (cfg.seed == -1) ? wall_clock_ns : cfg.seed

MasterPRNG.init(effectiveSeed)
  ├── .next() → rng::seed(...)              [autoc-side NN-evolution stream
  │                                          — population init, mutation,
  │                                          crossover, selection. Isolated
  │                                          from variation streams.]
  └── .next() × M_total → scenarioSeed[0..M_total-1]
       (M_total = paths × windScenarioCount, path-major layout)
```

`scenarioSeed[K]==0` is converted to `kSeedZeroSentinel = 0xC0FFEEu` before insert (Park-Miller LCG breaks at zero — defense-in-depth zero guard).

## Per-scenario sub-PRNG derivation (worker-side, per scenario K)

```
ScenarioRootPRNG root(scenarioSeed[K])
  ├── .next() → windSubSeed     (slot 0 — wind class)
  ├── .next() → rabbitSubSeed   (slot 1 — rabbit class: M1 speed segments, M2 crash-hull)
  ├── .next() → entrySubSeed    (slot 2 — entry class: cone/roll/speed/position)
  ├── .next() → craftSubSeed    (slot 3 — future 025 craft variations; SEEDED but unused in 033)
  └── .next() → cameraSubSeed   (slot 4 — future 034 camera variations; SEEDED but unused in 033)

ClassPRNG windPRNG(windSubSeed)
ClassPRNG rabbitPRNG(rabbitSubSeed)
ClassPRNG entryPRNG(entrySubSeed)
```

Each class PRNG is an independent Park-Miller LCG. They advance only as their downstream variation consumers pull draws.

### Append-only contract (FROZEN)

Adding a new variation **class** (e.g., sim-tick jitter as the 6th class) **appends** a new `root.next()` call **after** all existing slots. Existing classes' seeds (and therefore prior bakes' reproducibility) are unaffected. This is the central determinism guarantee.

Adding a new variable **within** an existing class adds another draw inside that class's PRNG — doesn't add a new slot seed.

### What each class draws

| Class | Active in M1 | Active in M2 | What it draws |
|---|---|---|---|
| **wind** | ✓ | ✓ | `windPRNG.next()` → uint32 for CRRCSim `CRRC_Random::reset` (per-frame gust + thermal vortex placement). `windPRNG.nextGaussian(sigma)` → `windDirectionOffset` (static per-scenario rotation of base wind heading). |
| **rabbit** | ✓ (speed segments) | ✓ (crash-hull PRNG state) | `rabbitPRNG.next()` → uint32 for worker-side rabbit-speed profile generator (M1) OR crash-hull Bernoulli PRNG seed (M2). Both modes seed identically; downstream consumption diverges. |
| **entry** | ✓ | ✓ | `entryPRNG` consumed by `generateEntryVariationsFromClassPRNG()` for cone heading + pitch, roll, speed factor, and (if enabled) cylindrical position + altitude offsets. |
| **craft** | future (025) | future (025) | Seeded; unused in 033. When 025 lands, mass/drag/prop perturbations will draw from `craftPRNG`. |
| **camera** | n/a (no camera) | future (034) | Seeded; unused in 033. When camera variations land, FOV/mount/lens distortion perturbations will draw from `cameraPRNG`. |

## `Enable<X>Variations=0` semantics

Per spec §2.E + Clarifications 2026-05-21:

```
Enable<X>Variations = 1  → class is active; variation values are applied
Enable<X>Variations = 0  → class STILL DRAWS from its PRNG, but the value
                           is DISCARDED before applying to the scenario
```

This **draw-and-discard** behavior ensures the per-class PRNG state advances identically regardless of whether the variation is currently in use. Switching `Enable...=0 → 1` later (e.g., enabling craft variations after 025 lands) doesn't perturb the wind/rabbit/entry seed values for prior bakes — they consumed the same draws.

In 033 phase 1 only `EnableEntryVariations` / `EnableWindVariations` / `EnableRabbitSpeedVariations` exist (the three currently-active classes). `EnableCraftVariations` / `EnableCameraVariations` are **not added as ini knobs** in 033 — those classes are seeded-but-unconsumed in 033 phase 1, and the future 025/034 PRs add the knob + draw-and-discard + consumer atomically. Per spec Clarifications 2026-05-21.

## Reproducibility recipe

To reproduce a specific bake exactly:

1. Copy the **effective master seed** from the training run's first log line:
   ```
   [INFO] Effective master seed: 1729123456789
   ```
2. Edit a copy of the appropriate ini file (`autoc.ini` or `autoc-tracker.ini`):
   ```ini
   Seed = 1729123456789
   ```
3. Re-run autoc with `-i <copied-ini>`.

The `scenarioSeed[K]` table will populate bit-identically; per-scenario variations (wind, rabbit, entry) will be drawn from class PRNGs with the same seeds; per-tick aircraft state across the whole run will match bit-for-bit (M1) and bit-identical-per-scenario (M2).

### Single-scenario eval replay

To replay only scenario K from a recorded dmp via eval mode:

1. Inspect the dmp to read `scenarioList[K].scenarioSeed`:
   ```bash
   build/tracker_dmp_inspect <dmp-key-or-path>
   ```
   Look at the `scenarioSeed (033 §2.A) per scenario:` block — each scenario's seed is printed in hex (e.g., `[042] 0xa1b2c3d4e5f60718`).
2. Eval mode reads `scenarioList[K].scenarioSeed` directly and reconstructs all class PRNGs for that scenario via the chain above. The master seed is **not** needed for single-scenario replay; the scenario seed is sufficient.

This is the **D4 determinism contract** (`eval_results(NN, scenarioSeed[K]) === bit-identical across runs`) — verifiable by re-running the eval and asserting `aircraftStateList[K]` matches the training trace.

## D-contracts (formal determinism guarantees)

| ID | Name | Guarantee |
|---|---|---|
| D1 | Run-level | Same `cfg.seed` (or same wall-clock for `Seed=-1`) → identical `scenarioSeedTable[0..M-1]`. |
| D2 | Scenario-level | Same `scenarioSeed[K]` → identical class sub-seed tuple (wind, rabbit, entry, craft, camera). |
| D3 | Cross-mode | M1 and M2 of scenario K with same master seed get the SAME class sub-seed tuple. Downstream consumption may DRAW differently per mode; D3 is at the SEED level. |
| D4 | Eval = training | Single-scenario replay through eval mode produces bit-identical per-tick `aircraftStateList[K]` to the training trace. |
| D5 | Append-only | Adding a new variation CLASS appends one new `scenarioRoot.next()` call AFTER existing slots. Existing slots' seeds and prior bakes' reproducibility are unaffected. |

Test enforcement: [tests/scenario_prng_tests.cc](../tests/scenario_prng_tests.cc) covers D1, D2, D3, D5 at the unit level. D4 is the end-to-end regression contract (validated via eval-mode replay test in the next-turn cleanup pass).

## Transitional state (033 phase 1, current branch)

The 033 phase 1 implementation lands in two slices to keep the diff reviewable:

### What's in place now (foundational + autoc-side init chain)

- `MasterPRNG`, `ScenarioRootPRNG`, `ClassPRNG` types ([include/autoc/util/scenario_prng.h](../include/autoc/util/scenario_prng.h))
- `gScenarioSeedTable` populated at startup; `scenarioSeed` field on `ScenarioMetadata`
- `prefetchAllVariations` refactored to consume class PRNGs (variations are now deterministic per `scenarioSeed[K]`)
- `tracker_dmp_inspect` prints per-scenario `scenarioSeed` for replay

### What lands in the cleanup pass

- Legacy `windSeed` / `rabbitSpeedSeed` fields removed from `ScenarioMetadata` + all 28+ touch sites in [src/autoc.cc](../src/autoc.cc) eliminated
- `gScenarioVariations` table expanded from per-wind (size = windScenarioCount) to per-(path, wind) (size = paths × windScenarioCount), matching the per-K master-seed table
- Consumer rewires: crash-hull PRNG seed source (TrackerStepper), worker-side rabbit-speed profile generator, entry-pose populator — all switched to draw from the new class-PRNG-derived values
- crrcsim worker (`inputdev_autoc.cpp` + `crrcsim_tracker_helper.cpp`) reconstructs `ScenarioRootPRNG` per scenario start; routes `windPRNG.next()` into `SimStateHandler::reset(uint32_t)`
- Replay tests (`tests/wind_replay_tests.cc`, `tests/eval_mode_replay_tests.cc`)

During the transitional state the OLD legacy fields are still populated (from new class-PRNG-derived values), so downstream consumers (minisim, crrcsim, renderer) continue to work unchanged.

## Citations

- [specs/033-m1-smooth-plus-variations/spec.md](../specs/033-m1-smooth-plus-variations/spec.md) §2.A + §2.E + Clarifications Q2/Q3/Q5 + 2026-05-21
- [specs/033-m1-smooth-plus-variations/contracts/scenario_prng_chain.md](../specs/033-m1-smooth-plus-variations/contracts/scenario_prng_chain.md) — formal D1-D5 contracts + validation tests
- [specs/033-m1-smooth-plus-variations/research.md](../specs/033-m1-smooth-plus-variations/research.md) R3 (class structure), R6 (NN-evolution decoupling), R7 (CRRCSim wind site)
- [.specify/memory/constitution.md](../.specify/memory/constitution.md) Principle III (no compatibility shims) — drives append-only contract; III (no legacy single-stream toggle)

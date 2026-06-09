# 033 — Data Model (Phase 1)

Entities, schemas, validation rules, and per-tick data flow for the master-seed PRNG architecture, the per-scenario sub-PRNG class chain, and the smoothness penalty factor. Anchored to [spec.md](./spec.md) Clarifications Q1–Q5 and [research.md](./research.md) R1–R8.

> **Heritage**: 032 phase-1 data-model already added the 9 derived perceptual features to `TrackerInputs` (45→54). 033 doesn't touch that — adds `scenarioSeed[K]` to `ScenarioMetadata` + `EvalResults`, and adds `smoothness_factor` as a per-tick computed scalar consumed inside `FitnessComputer::applyStreak()`.

## 1. `ScenarioMetadata` extension

Per [research.md R2](./research.md#r2--scenario-seed-plumbing-extend-scenariometadata-with-scenarioseed-coexists-with-windseed-initially):

```cpp
// include/autoc/rpc/scenario_metadata.h (extended)

struct ScenarioMetadata {
  // ----- existing fields (unchanged) -----
  unsigned int windSeed;          // legacy joint-stream seed; deprecated path
  uint64_t scenarioSequence;      // monotonic counter
  // ... existing variation offset fields (lines 36-49) ...

  // ----- 033 phase-1 NEW field (appended) -----
  uint64_t scenarioSeed;          // per-scenario root for sub-PRNG class chain (R3)

  template<class Archive>
  void serialize(Archive& ar) {
    ar(windSeed, scenarioSequence,
       /* existing variation offset fields */,
       scenarioSeed);  // appended — old EvalData/WorkerInit fail-loud on read
  }
};
```

**Validation rules**:
- `scenarioSeed != 0` (Park-Miller LCG requires non-zero seed; autoc-side guard converts 0 → `0xC0FFEEu` per existing `tracker_stepper.cc:47-49` pattern)
- One unique `scenarioSeed` per scenario index `K ∈ [0, M-1]` — operator can confirm uniqueness in eval logs

**Wire impact**: `WorkerInit.scenarioMetaList` (existing field at [protocol.h:170](../../include/autoc/rpc/protocol.h#L170)) carries the extended `ScenarioMetadata` automatically; per-eval `EvalData` doesn't change shape (already references scenarios by index).

## 2. `EvalResults` extension

Per [research.md R4](./research.md#r4--scenarioseedk-dmp-recording-add-scenarioseedlist-vector-to-evalresults):

```cpp
// include/autoc/rpc/protocol.h (extended)

struct EvalResults {
  // ----- existing fields (unchanged) -----
  // workerId, workerPid, workerEvalCounter, gpHash, gp, scenario,
  // scenarioList, aircraftStateList, cameraViewList, targetTrajectoryList,
  // arenaEgressCount, hullStrikeCount, crashReasonList, ...

  // ----- 033 phase-1 NEW field (appended) -----
  std::vector<uint64_t> scenarioSeedList;  // parallel-indexed with scenarioList; one entry per scenario this eval ran

  template<class Archive>
  void serialize(Archive& ar) {
    // existing serialize calls in original order
    // ...
    ar(scenarioSeedList);  // appended — no cereal version bump
  }
};
```

**Validation rules**:
- `scenarioSeedList.size() == scenarioList.size()` (parallel-indexed invariant)
- `scenarioSeedList[k] == scenarioList[k].scenarioSeed` (redundant but cross-checks the persist site)

**Reader contract**: a reader can pull `scenarioSeedList[k]` to deterministically reproduce scenario K's variations without needing the source `WorkerInit`. Use case: analysis scripts loading a dmp + replaying one specific scenario for inspection.

## 3. PRNG architecture entities

Per [research.md R3 + R6](./research.md#r3--per-scenario-sub-prng-class-structure-5-classes-wind--rabbit--entry--craft--camera):

### 3.1 `MasterPRNG` (autoc-side, run-init only)

```cpp
// Existing pattern; just made explicit
// One instance per autoc process, initialized from autoc.ini Seed value.
class MasterPRNG {
public:
  void init(uint64_t seed);   // wall-clock if Seed=-1, explicit otherwise
  uint64_t next();             // 64-bit advance
};
```

Used at autoc startup only. Consumed by:
1. `rng::seed(masterPRNG.next())` — seeds the existing `rng::*` global stream for NN-evolution consumers ([src/nn/population.cc:11-13](../../src/nn/population.cc#L11-L13))
2. `scenarioSeedTable[k] = masterPRNG.next()` for k in [0, M-1] — populates per-scenario root seeds

### 3.2 `ScenarioRootPRNG` (worker-side, per-scenario init)

```cpp
// Worker-side, instantiated at the start of each scenario from ScenarioMetadata.scenarioSeed
class ScenarioRootPRNG {
public:
  ScenarioRootPRNG(uint64_t scenarioSeed);
  uint32_t next();   // returns class sub-PRNG seed
};
```

Consumed by:
- `windPRNG.seed = scenarioRoot.next()`
- `rabbitPRNG.seed = scenarioRoot.next()`
- `entryPRNG.seed = scenarioRoot.next()`
- `craftPRNG.seed = scenarioRoot.next()` (currently inert)
- `cameraPRNG.seed = scenarioRoot.next()` (currently inert)
- `... extension slots ...`

**Critical contract**: the order of `scenarioRoot.next()` calls IS the frozen append-only contract. Adding a new variation class adds one new `.next()` call at the END (after all existing classes). Reordering or inserting in the middle breaks reproducibility of every prior bake.

### 3.3 `ClassPRNG` (worker-side, one per variation class per scenario)

```cpp
// Park-Miller LCG, same algorithm as the existing TrackerStepper::prng_state_
// (src/eval/tracker_stepper.cc:47-49). One instance per class per scenario.
class ClassPRNG {
public:
  ClassPRNG(uint32_t seed);
  uint32_t next();              // advance + return; consumed by variation values
  double nextNormalized();      // [0, 1) — convenience for Gaussian draws etc.
};
```

Consumers:
- `windPRNG` — used by `SimStateHandler::reset(uint32_t)` ([crrcsim/src/SimStateHandler.cpp:246-256](../../crrcsim/src/SimStateHandler.cpp#L246-L256)); CRRCSim's per-frame wind machinery already advances this PRNG normally
- `rabbitPRNG` — consumed by pathgen rabbit-speed segment generator (M1) and crash-hull Bernoulli draws (M2; replacing the existing `tracker_stepper.cc:47-49` PRNG init)
- `entryPRNG` — consumed by entry pose offset computation at scenario init
- `craftPRNG` / `cameraPRNG` — seeded but unused in phase 1 (future-proofs append-only contract)

## 4. Smoothness factor (per-tick scalar)

Per [research.md R1 + R8](./research.md#r8--smoothness-penalty-pure-math-placement-extend-includeautocevalderived_featuresh):

### 4.1 Pure-math helper

```cpp
// include/autoc/eval/derived_features.h (extended)

namespace autoc::eval {

enum class SmoothnessMotionMode {
  Pythagorean,  // sqrt(dpt² + drl² + dth²); motion_max = sqrt(12) ≈ 3.46
  Sum,          // |dpt| + |drl| + |dth|;   motion_max = 6
  Max,          // max(|dpt|, |drl|, |dth|); motion_max = 2
};

// Returns smoothness factor in [floor, 1.0].
// floor ∈ [0, 1]: 0 = annihilating penalty at max motion; 1.0 = no-op penalty.
inline gp_scalar compute_smoothness_factor(
    gp_scalar dpt, gp_scalar drl, gp_scalar dth,
    gp_scalar floor,
    SmoothnessMotionMode mode = SmoothnessMotionMode::Pythagorean) {
  gp_scalar motion;
  gp_scalar motion_max;
  switch (mode) {
    case SmoothnessMotionMode::Pythagorean:
      motion = std::sqrt(dpt * dpt + drl * drl + dth * dth);
      motion_max = std::sqrt(static_cast<gp_scalar>(12.0));
      break;
    case SmoothnessMotionMode::Sum:
      motion = std::abs(dpt) + std::abs(drl) + std::abs(dth);
      motion_max = static_cast<gp_scalar>(6.0);
      break;
    case SmoothnessMotionMode::Max:
      motion = std::max({std::abs(dpt), std::abs(drl), std::abs(dth)});
      motion_max = static_cast<gp_scalar>(2.0);
      break;
  }
  const gp_scalar t = std::min(motion / motion_max, static_cast<gp_scalar>(1.0));
  return static_cast<gp_scalar>(1.0) - (static_cast<gp_scalar>(1.0) - floor) * t;
}

}  // namespace autoc::eval
```

### 4.2 Per-tick state (stepper-owned)

```cpp
// Per-scenario state inside PathgenStepper / TrackerStepper:
struct PrevNNOutput {
  gp_scalar pitch;
  gp_scalar roll;
  gp_scalar throttle;
};
PrevNNOutput prev_;       // updated each tick after applyStreak
bool prev_valid_;         // false at scenario start; true after first tick
```

Per-tick flow:
1. NN forward-pass produces `(pitch, roll, throttle) ∈ [−1, +1]³`
2. If `prev_valid_`, compute `dpt = curr.pitch - prev_.pitch`, etc.; else dpt=drl=dth=0 (no penalty on first tick of a scenario — no Δ defined)
3. `smoothness_factor = compute_smoothness_factor(dpt, drl, dth, cfg.smoothnessPenaltyFloor, cfg.smoothnessMotionMode)`
4. Pass `smoothness_factor` to `fc.applyStreak(stepPoints, smoothness_factor)` (extended signature per R1)
5. Update `prev_ = curr; prev_valid_ = true;`

**Validation rules**:
- `0.0 ≤ smoothnessPenaltyFloor ≤ 1.0` (config range check; loud-fail on startup if violated)
- First-tick smoothness_factor == 1.0 (no penalty until Δ is computable)
- `smoothness_factor ∈ [smoothnessPenaltyFloor, 1.0]` for all subsequent ticks (math is bounded)

## 5. `FitnessComputer::applyStreak()` extension

Per [research.md R1](./research.md#r1--smoothness-penalty-insertion-point-inside-fitnesscomputerapplystreak-itself):

```cpp
// include/autoc/eval/fitness_computer.h (modified)

class FitnessComputer {
public:
  // ... existing methods ...

  // Apply smoothness penalty (multiplicative on stepPoints) THEN streak multiplier.
  // smoothness_factor defaults to 1.0 (no-op) so existing tests/callers unaffected.
  gp_fitness applyStreak(gp_scalar stepPoints,
                         gp_scalar smoothness_factor = 1.0);
};

// src/eval/fitness_computer.cc:57 (extended)
gp_fitness FitnessComputer::applyStreak(gp_scalar stepPoints,
                                         gp_scalar smoothness_factor) {
  // ... existing streak-state-update logic ...
  const gp_scalar penalized = stepPoints * smoothness_factor;
  return static_cast<gp_fitness>(penalized * multiplier);
}
```

**Wire impact**: signature change is binary-compatible for callers that don't pass the new arg (default arg). New callers in the stepper path provide the computed factor.

## 6. Per-tick data flow (extended end-to-end)

```text
┌─────────────────────────────────────────────────────────────────┐
│ scenario start (worker, per scenario K)                          │
│                                                                  │
│ 1. Read scenarioSeed[K] from ScenarioMetadata                   │
│ 2. ScenarioRootPRNG root(scenarioSeed[K]);                      │
│ 3. ClassPRNG windPRNG(root.next()),                             │
│              rabbitPRNG(root.next()),                           │
│              entryPRNG(root.next()),                            │
│              craftPRNG(root.next()),    // inert                │
│              cameraPRNG(root.next());   // inert                │
│ 4. SimStateHandler::reset(windPRNG.next());  // crrcsim wind   │
│ 5. Compute entry-pose offsets using entryPRNG                   │
│ 6. Init pathgen rabbit-speed segments (M1) using rabbitPRNG     │
│ 7. Init crash-hull PRNG state (M2) using rabbitPRNG             │
│ 8. PathgenStepper / TrackerStepper init: prev_valid_ = false    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ per tick (PathgenStepper / TrackerStepper::stepOnce)             │
│                                                                  │
│ 1. NN forward pass → (pt, rl, th) outputs                       │
│ 2. If prev_valid_:                                               │
│      dpt = curr.pt - prev_.pt; drl = ...; dth = ...             │
│    Else: dpt = drl = dth = 0                                     │
│ 3. smoothness_factor = compute_smoothness_factor(                │
│      dpt, drl, dth, cfg.floor, cfg.mode);                       │
│ 4. stepPoints = fc.computeStepScore(along, lateralDist);        │
│ 5. final = fc.applyStreak(stepPoints, smoothness_factor);       │
│      // internally: returns stepPoints * smoothness_factor       │
│      //                        * streak_multiplier               │
│ 6. accumulate `final` into per-scenario fitness                 │
│ 7. prev_ = curr; prev_valid_ = true                              │
│ 8. wind PRNG advances normally during FDM tick (CRRCSim path)   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ scenario end                                                     │
│                                                                  │
│ - PathgenStepper / TrackerStepper resets prev_valid_ on next     │
│   initScenario                                                   │
│ - PRNGs are scenario-scoped — discarded on scenario boundary     │
│   (per R3 reset semantics; next scenario re-seeds from           │
│   scenarioSeed[K+1] independently)                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ eval end (worker → autoc, dmp persist)                           │
│                                                                  │
│ - EvalResults.scenarioSeedList[k] = scenarioSeed[k] for each     │
│   k the worker ran                                               │
│ - cereal serialize walks scenarioSeedList alongside other        │
│   parallel-indexed lists                                         │
└─────────────────────────────────────────────────────────────────┘
```

## 7. ini schema addition

```ini
# autoc.ini / autoc-tracker.ini / autoc-tracker-minisim.ini

[Smoothness]
# Multiplicative penalty floor on per-tick stepPoints. Computed as
# (1.0 - (1 - floor) × min(motion / motion_max, 1.0)), where motion is the
# Pythagorean aggregate of per-tick NN-output Δ across pitch/roll/throttle.
# 1.0 = no-op (back-compat / regression-test mode)
# 0.5 = "half-credit at max-motion bang-bang" (033 phase 1 YOLO start)
# 0.3 = harsh; 0.7 = mild
# Range: [0.0, 1.0]. Out-of-range fails loud at startup.
SmoothnessPenaltyFloor          = 0.5

# Motion aggregate mode: "pythagorean" | "sum" | "max"
# Default "pythagorean" per 033 /clarify Q4.
SmoothnessMotionMode            = pythagorean
```

## 8. M1 / M2 mode coverage

| Mode | scenarioSeed[K] applied | Smoothness penalty active | Notes |
|---|---|---|---|
| M1 pathgen (`autoc.ini`) | ✓ | ✓ (phase 1 target) | Phase 1 bake validates here |
| M2 tracker (`autoc-tracker.ini`) | ✓ | ✗ (phase 1) → ✓ (phase 2) | Wire is in place; just set floor=1.0 in phase 1 |
| M2 minisim (`autoc-tracker-minisim.ini`) | ✓ | Same as M2 tracker | Mirror config |

Phase 1's M1-only smoothness test is a config-level toggle: M2 ini files keep `SmoothnessPenaltyFloor = 1.0` (no-op) until phase 2 lights it up.

## 9. Cross-mode equivalence invariant (testable)

**Invariant**: for the same `(masterSeed, scenarioIndex)`, M1 and M2 bakes see identical wind, rabbit, entry, craft, camera draws — only the consumption differs (M2 ignores rabbit-speed; M1 ignores crash-hull). This is the core determinism contract from spec §2.A.

**Test**: instrument a regression test that runs M1 and M2 with the same `masterSeed`, captures the first 10 wind draws + first 10 entry draws + first 10 rabbit-speed draws per mode, and asserts equality across modes.

## 10. Recap — scope diff vs 032

| Layer | 032 phase 1 | 033 phase 1 |
|---|---|---|
| NN sensor inputs | 45 → 54 (new derived features) | unchanged at 54 |
| Topology | 32 → 16r → 3 hidden | unchanged |
| Fitness | cone score × streak multiplier | cone score × **smoothness factor** × streak multiplier |
| PRNG architecture | joint single-stream global | **master → scenarioSeed[K] → 5 class sub-PRNGs**, separate from NN-evolution PRNG |
| dmp schema | `TrackerInputs` field added | **`scenarioSeedList` vector** added to EvalResults; **`scenarioSeed`** added to ScenarioMetadata |
| Cereal version | no bump | no bump (same policy) |
| M1 regression vs prior | bitwise-equal | **intentionally breaks** (PRNG-stream decoupling) |

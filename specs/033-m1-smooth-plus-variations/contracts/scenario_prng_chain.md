# Contract: master → scenario → class sub-PRNG init chain

**Producer**: autoc-side run init (`src/autoc.cc` startup + per-eval scenario dispatch) + worker-side scenario start (`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` + helpers).
**Consumer**: NN-evolution path (`src/nn/population.cc` via existing `rng::*` globals) + scenario variation appliers (wind, rabbit, entry, future craft/camera).

## Surface — autoc-side init chain

```cpp
// Conceptual init flow at autoc startup (src/autoc.cc, after ini parse)

// 1. Determine effective master seed
const uint64_t effectiveSeed = (cfg.seed < 0)
    ? static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count())
    : static_cast<uint64_t>(cfg.seed);

// 2. Print for reproducibility (operator copies into eval-mode ini to reproduce)
logger.info() << "Effective master seed: " << effectiveSeed;

// 3. Init MasterPRNG
MasterPRNG masterPRNG;
masterPRNG.init(effectiveSeed);

// 4. Seed the existing rng::* global stream from master.next() — NN evolution consumes this
rng::seed(masterPRNG.next());

// 5. Populate scenario seed table from subsequent master draws
const size_t M = scenarios.size();  // total number of scenarios in the table
std::vector<uint64_t> scenarioSeedTable(M);
for (size_t k = 0; k < M; ++k) {
  scenarioSeedTable[k] = masterPRNG.next();
}

// 6. Populate ScenarioMetadata.scenarioSeed field for each scenario
for (size_t k = 0; k < M; ++k) {
  scenarioMetaList[k].scenarioSeed = scenarioSeedTable[k];
}
```

## Surface — worker-side per-scenario init

```cpp
// Conceptual flow at scenario start on the worker (inputdev_autoc.cpp + helpers)

// 1. Read scenarioSeed from worker-cached ScenarioMetadata
const uint64_t scenarioSeed = workerInit.scenarioMetaList[scenarioIdx].scenarioSeed;

// 2. Construct scenario root PRNG
ScenarioRootPRNG scenarioRoot(scenarioSeed);

// 3. Derive class sub-PRNG seeds — APPEND-ONLY ORDER (FROZEN CONTRACT)
const uint32_t windSubSeed   = scenarioRoot.next();  // slot 0 — wind
const uint32_t rabbitSubSeed = scenarioRoot.next();  // slot 1 — rabbit (M1) + crash hull (M2)
const uint32_t entrySubSeed  = scenarioRoot.next();  // slot 2 — entry pose
const uint32_t craftSubSeed  = scenarioRoot.next();  // slot 3 — future 025; SEEDED but unused
const uint32_t cameraSubSeed = scenarioRoot.next();  // slot 4 — future 034/035; SEEDED but unused
// future slot 5+ — append-only, new variation classes go HERE at the end

// 4. Construct class PRNGs
ClassPRNG windPRNG(windSubSeed);
ClassPRNG rabbitPRNG(rabbitSubSeed);
ClassPRNG entryPRNG(entrySubSeed);

// 5. Apply to variation consumers
SimStateHandler::reset(windPRNG.next());  // CRRCSim wind init takes uint32_t seed
applyEntryPoseOffsets(entryPRNG);          // entry cone/roll/speed/position
initRabbitSpeedSegments(rabbitPRNG);       // M1 only
initCrashHullPRNG(rabbitPRNG);             // M2 only — same class PRNG, different consumers
```

## Determinism contract

**(D1) Run-level**: same `cfg.seed` (or same wall-clock value captured for `cfg.seed = -1`) → identical NN-evolution PRNG seed AND identical `scenarioSeedTable[0..M-1]`. Verifiable by re-running autoc with the printed effective seed and capturing the same log line for the first NN evolution draw.

**(D2) Scenario-level**: same `scenarioSeed[K]` → identical wind/rabbit/entry/craft/camera draws. Independent of `K` (no cross-scenario PRNG state).

**(D3) Cross-mode**: M1 and M2 of scenario K with same master seed get the **same** `(windSubSeed, rabbitSubSeed, entrySubSeed, craftSubSeed, cameraSubSeed)` tuple. Different consumption (M2 uses crash-hull from `rabbitPRNG`; M1 uses rabbit-speed segments from `rabbitPRNG`) is downstream of the seed. Both modes draw from the same starting state.

**(D4) Eval = training (single elite, single scenario)**: replaying the elite NN through scenario K via eval mode produces bit-identical per-tick `aircraftStateList[K]` to what the worker emitted during training. This is the regression-test contract.

**(D5) Append-only**: adding a new variation CLASS (e.g., simulator timing jitter as the 6th class) appends one new `scenarioRoot.next()` call AFTER all existing slots. Existing classes' draws and therefore prior bakes' reproducibility are unaffected.

## Validation tests

`tests/scenario_prng_tests.cc` (NEW):
- `same_master_seed → same_scenario_seed_table`: run init twice with the same seed; assert scenarioSeedTable bit-equal
- `different_master_seed → different_scenario_seed_table`: different seeds give different tables
- `same_scenario_seed → same_class_sub_seeds`: same scenarioSeed → identical (windSubSeed, rabbitSubSeed, entrySubSeed, craftSubSeed, cameraSubSeed) tuple
- `class_prng_independence`: advancing windPRNG doesn't affect rabbitPRNG state
- `append_only_contract`: adding a 6th class doesn't change the first 5 sub-seeds for the same scenarioSeed
- `nn_prng_separation`: consuming heavily from `rng::*` (NN-evolution stream) doesn't affect any scenarioSubSeed
- `cross_mode_equivalence`: simulate M1 init and M2 init for scenario K with same masterSeed → assert same (windSubSeed, rabbitSubSeed, entrySubSeed, craftSubSeed, cameraSubSeed)

`tests/wind_replay_tests.cc` (NEW, integration):
- Run scenario K twice via the worker with same scenarioSeed → assert per-tick wind values bit-equal (requires capturing wind values through a test-only hook)

## Failure modes + sentinel behavior

- **scenarioSeed == 0**: Park-Miller LCG breaks at zero seed. Loud-fail at `ScenarioRootPRNG` construction (assert + abort). Autoc-side guard converts 0 → `0xC0FFEEu` before populating the table (mirror existing pattern from `tracker_stepper.cc:47-49`).
- **Replay of old dmp**: pre-033 dmps lack the `scenarioSeed` field. Cereal length mismatch → loud fail (per Constitution V no-shim policy). Operator must re-run training with the new binary if they want post-033 replays.
- **Master seed printing collision**: if two simultaneous autoc processes both pick `cfg.seed = -1`, wall-clock collision is theoretically possible. Mitigate: log the effective seed prominently at startup so operator can detect collisions visually.

## Citations

- [spec.md](../spec.md) §2.A + Clarifications Q2, Q3, Q5
- [research.md](../research.md) R2 (ScenarioMetadata extension), R3 (class structure), R6 (NN-evolution decoupling), R7 (CRRCSim wind site)
- [Constitution III](../../../.specify/memory/constitution.md) — no compatibility shims (drives append-only contract, no legacy single-stream toggle)
- [project_v15_determinism_candidates](../../../.claude/projects/-home-gmcnutt-autoc/memory/project_v15_determinism_candidates.md) — non-determinism concerns this contract resolves

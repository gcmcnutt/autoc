# Train/Eval Code Deduplication Survey (autoc 023 research)

> Audit of `src/autoc.cc` comparing the training evolution loop against the
> one-shot `runNNEvaluation()` path, identifying every place the two have
> drifted apart. Feeds the Phase 0b refactor design for autoc 023
> (OOD + Engage fixes).

## Executive Summary

- **Total divergences found: 6** (3 known from BACKLOG, 3 previously unknown).
- **Known bugs confirmed:**
  - **Bug 1 — different metric:** already resolved. Both paths call
    `computeScenarioScores()` + `aggregateRawFitness()` on the result of a
    single RPC to the minisim. Confirmed at
    `src/autoc.cc:799-800` (eval) and `src/autoc.cc:985-986` (train, per-individual)
    and `src/autoc.cc:1074-1075` (train, elite re-eval).
  - **Bug 2 — stale S3 fitness:** confirmed. `runNNEvaluation()` copies the
    *original* serialized NN bytes back into `evalResults.gp` at
    `src/autoc.cc:823-824`. Those bytes embed the training-time fitness (NN01
    format). The newly computed eval fitness is printed/logged only
    (`src/autoc.cc:847`) and never written into either `genome.fitness` or a
    dedicated field on `EvalResults`.
  - **Bug 3 — eval path never sets `rabbitSpeedConfig`:** confirmed.
    `EvalData evalData;` at `src/autoc.cc:751` defaults
    `rabbitSpeedConfig = RabbitSpeedConfig::defaultConfig()` =
    `{nominal=16.0, sigma=0.0, min=8.0, max=25.0, cycleMin=0.5, cycleMax=5.0}`
    (see `include/autoc/eval/variation_generator.h:139-141` and
    `include/autoc/rpc/protocol.h:135`). The training path overrides this at
    `src/autoc.cc:939-940` (per-individual) and `src/autoc.cc:1028-1029`
    (elite re-eval). Eval-mode rabbit is a constant 16 m/s regardless of
    `RabbitSpeed*` keys in `autoc-eval.ini`.

- **Previously-unknown divergences:**
  - **Bug 4 — `isEliteReeval` lies to the minisim.** The training path uses
    this flag to toggle deterministic logging inside the sim worker (see
    BACKLOG reference to elite re-eval). Eval mode hard-codes
    `evalData.isEliteReeval = false` (`src/autoc.cc:756`) while also setting
    `meta.enableDeterministicLogging = false` (`src/autoc.cc:766`). A one-shot
    "eval this weight" run *is* effectively an elite re-eval and should set
    both to `true` so the sim captures deterministic per-step data matching
    what training-time elite re-eval captures. Symptoms: eval-mode `data.dat`
    may be missing fields the renderer expects to see.
  - **Bug 5 — scenario selection asymmetry.** Training picks
    `scenarioForIndex(ind % generationScenarios.size())` so individuals are
    spread across scenarios (`src/autoc.cc:929`). Elite re-eval uses
    `scenarioForIndex(bestIdx % generationScenarios.size())` (`src/autoc.cc:1018`).
    `runNNEvaluation()` hard-codes scenario 0 (`src/autoc.cc:748`). In
    demetic mode this means eval *only* scores the first path-variant deme —
    silently discarding the other N-1 paths in the scenario table. Training
    aggregates across all scenarios; eval does not.
  - **Bug 6 — elite re-eval uses its own copy of the scenario-list
    populator.** The per-individual loop (`src/autoc.cc:946-971`) and the
    elite re-eval loop (`src/autoc.cc:1034-1056`) are near-identical
    transcriptions of each other. Any future fix lands in one and misses the
    other. The eval path (`src/autoc.cc:761-783`) is a third copy of the same
    code. All three carry the same scaffolding (numWindScenarios computation,
    `populateVariationOffsets`, `meta.rabbitSpeed = gRabbitSpeedConfig.nominal`)
    with small drift.

- **Recommended refactor scope:** extract a single
  `buildEvalData(const ScenarioDescriptor&, const std::vector<char>& nnBytes,
  EvalMode mode)` helper that every call site uses. The three existing copies
  collapse to one function, which fixes bugs 3/4/5/6 structurally and gives
  bug 2 a single clean place to write the computed fitness back.

---

## Training Path

### Startup (before evolution, shared with eval)

`main()` at `src/autoc.cc:1209` performs setup that both paths use:

1. `src/autoc.cc:1237` — `ConfigManager::initialize(configFile, ...)` loads
   `autoc.ini` / `autoc-eval.ini`.
2. `src/autoc.cc:1248` — `rng::seed()`.
3. `src/autoc.cc:1255` — `threadPool = new ThreadPool(...)`.
4. `src/autoc.cc:1276-1285` — global sigmas + entry/wind enable flags.
5. `src/autoc.cc:1293-1300` — `gRabbitSpeedConfig` populated from config.
   This is THE global the eval path fails to read.
6. `src/autoc.cc:1310-1312` — ramp globals (`gTotalGenerations`, etc).
7. `src/autoc.cc:1325-1327` — `prefetchAllVariations(...)` fills
   `gScenarioVariations[]` (wind seeds, entry offsets, rabbit-speed seeds).
8. `src/autoc.cc:1376-1379` — `generateSmoothPaths(...)` into
   `generationPaths`.
9. `src/autoc.cc:1393` — `rebuildGenerationScenarios(generationPaths)` fills
   `generationScenarios`.
10. `src/autoc.cc:1403` — branch: `if (cfg.evaluateMode) runNNEvaluation
    else runNNEvolution`.

Everything in steps 1-9 is shared. The divergence starts at step 10.

### `runNNEvolution()` — `src/autoc.cc:859-1207`

Generation loop at `src/autoc.cc:908-1203`. For each generation:

1. `src/autoc.cc:911` — `gCurrentGeneration = gen` (drives
   `computeVariationScale()`).
2. `src/autoc.cc:914-917` — regenerate `generationPaths` with `gPathSeed`.
3. `src/autoc.cc:918` — `rebuildGenerationScenarios(generationPaths)`.
4. `src/autoc.cc:921-988` — per-individual evaluation loop:
   - Pick scenario: `scenarioForIndex(ind % generationScenarios.size())`
     (`src/autoc.cc:929`).
   - Allocate `scenarioSequence` from global counter
     (`src/autoc.cc:930`).
   - Fill `EvalData`:
     - `controllerType = NEURAL_NET` (L933)
     - `gp` = serialized NNGenome bytes (L934-935)
     - `gpHash = hashByteVector(evalData.gp)` (L936)
     - `isEliteReeval = false` (L937)
     - **`rabbitSpeedConfig = gRabbitSpeedConfig; sigma *= computeVariationScale();`** (L939-940)
     - `pathList = scenario.pathList` (L941)
     - `scenario.scenarioSequence` / `bakeoffSequence` on the top-level
       `evalData.scenario` (L942-943)
   - Build `scenarioList` vector (L946-971): one `ScenarioMetadata` per
     flight, demetic-aware `pathVariantIndex`/`windVariantIndex`/`windSeed`
     decoding, then `populateVariationOffsets(meta)`, then
     `meta.rabbitSpeed = gRabbitSpeedConfig.nominal`.
   - `evalData.scenario = evalData.scenarioList.front()` (L972-974).
   - `evalData.sanitizePaths()` (L975).
   - Dispatch through `threadPool->enqueue(...)`: `sendRPC` → `receiveRPC`,
     then `computeScenarioScores()` + `aggregateRawFitness()` stored directly
     on `genome.scenario_scores` / `genome.fitness` (L978-987).
5. `src/autoc.cc:991` — `threadPool->wait_for_tasks()`.
6. `src/autoc.cc:994-1001` — find best individual.
7. `src/autoc.cc:1013-1117` — **elite re-eval block**. Builds a *second*
   `EvalData` for `pop.individuals[bestIdx]`, near-identical code to the
   per-individual loop (see "Divergence Table" below — rows for "elite
   re-eval copy").
   - Logs to `data.dat` via `logEvalResults(fout, bestResults)` (L1071).
   - Runs a determinism check (`bitwiseEqual`) at L1074-1087.
   - Re-serializes NN bytes into `bestResults.gp` and uploads to S3 as
     `gen<10000-gen>.dmp` (L1091-1116).
8. `src/autoc.cc:1119-1168` — logging/stats.
9. `src/autoc.cc:1171-1202` — `nn_evolve_generation()`.

### Globals read by training

- `gVariationSigmas`, `gEnableEntryVariations`, `gEnableWindVariations`
  (via `populateVariationOffsets` / `prefetchAllVariations`)
- `gRabbitSpeedConfig` — read at L939, L1028, L969, L1054
- `gScenarioVariations` (via `populateVariationOffsets`)
- `gCurrentGeneration` / `gTotalGenerations` / `gVariationRampStep` (via
  `computeVariationScale()`)
- `generationPaths` / `generationScenarios`
- `globalScenarioCounter`, `globalSimRunCounter`

---

## Eval Path

### `runNNEvaluation()` — `src/autoc.cc:697-856`

1. `src/autoc.cc:702` — get `cfg` (same as training).
2. `src/autoc.cc:708-720` — load NN01 weight file from disk.
3. `src/autoc.cc:722-725` — format detect.
4. `src/autoc.cc:727-731` — `nn_deserialize` into `NNGenome genome`.
   `genome.fitness` now carries the *training-time* fitness.
5. `src/autoc.cc:744-745` — `nn_serialize(genome, nnData)` — round-tripped
   bytes that still encode that training-time fitness.
6. `src/autoc.cc:748` — **`scenarioForIndex(0)`** — hard-coded scenario index.
7. `src/autoc.cc:749` — scenarioSequence from global counter.
8. `src/autoc.cc:751-759` — fill `EvalData`:
   - `controllerType = NEURAL_NET`
   - `gp` = serialized bytes, `gpHash`
   - **`isEliteReeval = false`** (hard-coded; training also hard-codes this
     to `false`, so not a drift, but see Bug 4)
   - `pathList = scenario.pathList`
   - `scenario.scenarioSequence`, `bakeoffSequence`
   - **MISSING: `evalData.rabbitSpeedConfig` never assigned** — inherits the
     default `{16.0, 0.0, 8.0, 25.0, 0.5, 5.0}` from
     `include/autoc/rpc/protocol.h:135`.
9. `src/autoc.cc:761-783` — build `scenarioList` loop:
   - Third copy of the per-individual/elite-reeval logic. Slightly
     abbreviated (doesn't compute the unused `numBasePaths` local that the
     training per-individual copy computes at L956).
   - Calls `populateVariationOffsets(meta)` (L780).
   - Sets `meta.rabbitSpeed = gRabbitSpeedConfig.nominal` (L781) — this
     *does* read the global, so only the `sigma` is wrong. Symptom:
     bench rabbit speed is the nominal value baked into `gRabbitSpeedConfig`,
     but the variation sigma is whatever the default 0.0 was, so the bench
     is effectively a constant-speed rabbit at the configured nominal.
10. `src/autoc.cc:787` — `evalData.sanitizePaths()`.
11. `src/autoc.cc:790-796` — enqueue one task, `sendRPC` → `receiveRPC`.
12. `src/autoc.cc:799-800` —
    `computeScenarioScores()` + `aggregateRawFitness()` (SAME call as
    training; Bug 1 is fixed).
13. `src/autoc.cc:803` — `logEvalResults(fout, evalResults)`.
14. `src/autoc.cc:806-816` — per-scenario console breakdown.
15. `src/autoc.cc:823-845` — **upload to S3 with the original NN bytes
    copied into `evalResults.gp`** and S3 key `startTime + "/gen9999.dmp"`.
    The eval-computed `fitness` from L800 is NOT written anywhere durable.
16. `src/autoc.cc:847-855` — console + `data.stc` logging.

### Globals the eval path reads

- `gRabbitSpeedConfig.nominal` (read for `meta.rabbitSpeed` only)
- `gScenarioVariations` (via `populateVariationOffsets`)
- `gCurrentGeneration`/ramp globals (via `computeVariationScale()` inside
  `populateVariationOffsets`). NOTE: in eval mode `gCurrentGeneration == 0`
  because the generation loop never runs, so
  `computeVariationScale()` returns `1.0` via its early-exit at `src/autoc.cc:80`.
- `generationPaths` / `generationScenarios` (via `scenarioForIndex(0)`)
- `globalScenarioCounter`, `globalSimRunCounter`

### Globals the eval path does NOT read (but training does)

- **`gRabbitSpeedConfig` as a whole** — this is Bug 3.

---

## Divergence Table

| # | Stage | Training path | Eval path | Divergent? | Bug ID |
|---|---|---|---|---|---|
| 1 | Config load | `ConfigManager::initialize()` in `main()` L1237 | same | No (shared) | — |
| 2 | Global state init (sigmas, rabbit, ramp, prefetch, paths) | `main()` L1276-1393 | same | No (shared) | — |
| 3 | Scenario selection | `scenarioForIndex(ind % N)` L929 / `(bestIdx % N)` L1018 | `scenarioForIndex(0)` L748 | **Yes** — eval only scores one scenario; in demetic mode this silently drops path variants 1..N-1 | Bug 5 (new) |
| 4 | `EvalData` controllerType / gp / gpHash | L933-936 | L752-755 | No | — |
| 5 | `EvalData.isEliteReeval` | `false` L937 / `false` L1026 | `false` L756 | Semantically drifted: eval *is* a one-shot precise re-eval and arguably should set `true` (and set `meta.enableDeterministicLogging = true`) | Bug 4 (new) |
| 6 | `EvalData.rabbitSpeedConfig` | `= gRabbitSpeedConfig; sigma *= computeVariationScale()` L939-940, L1028-1029 | **never assigned** → `RabbitSpeedConfig::defaultConfig()` = `{16.0, 0.0, 8.0, 25.0, 0.5, 5.0}` | **Yes** — eval uses constant 16 m/s regardless of ini | Bug 3 |
| 7 | `EvalData.pathList` | `= scenario.pathList` L941 / L1030 | `= scenario.pathList` L757 | No | — |
| 8 | scenarioSequence bookkeeping | L930, L1019 | L749 | No | — |
| 9 | Build `scenarioList` | L946-971 (Copy A) | L761-783 (Copy C) | Yes, by duplication. Copy A computes an unused `numBasePaths` local at L956 that Copy C and Copy B (below) omit. Harmless today but an early symptom of drift. | Bug 6 (new) |
| 9b | Same build, elite re-eval copy | L1034-1056 (Copy B) | — | Yes, by duplication inside the training file itself | Bug 6 (new) |
| 10 | `populateVariationOffsets` | called per `meta` L968, L1053 | called per `meta` L780 | No | — |
| 11 | `meta.rabbitSpeed` seed | `= gRabbitSpeedConfig.nominal` L969, L1054 | `= gRabbitSpeedConfig.nominal` L781 | No (all three read the global) | — |
| 12 | `evalData.scenario = front()` | L972, L1057 | L784-786 | No | — |
| 13 | `sanitizePaths()` | L975, L1060 | L787 | No | — |
| 14 | Dispatch through threadpool | L978-987 | L790-796 | No (structure is the same) | — |
| 15 | Per-individual fitness math | `computeScenarioScores` + `aggregateRawFitness` L985-986 | same L799-800 | No (Bug 1 is fixed) | Bug 1 (fixed) |
| 16 | Store computed fitness on genome | `genome.fitness = aggregate...` L986 | **never stored on `genome`** (only local `double fitness`) | **Yes** — downstream S3 upload still carries the training-time fitness embedded in the original NN01 bytes | Bug 2 |
| 17 | Determinism check (`bitwiseEqual`) | L1077-1087 (elite re-eval vs stored) | **not performed** | Yes — eval silently diverges without an error | (latent) |
| 18 | S3 key name | `gen<10000-gen>.dmp` L1097 | `gen9999.dmp` L827 | By design, not a bug; flagged so the refactor preserves it | — |
| 19 | S3 payload — `evalResults.gp` | `= serialized NNGenome` (fresh) L1091-1092 | **= serialized NNGenome** (original bytes) L823-824 | Yes — payload is the same shape, but eval never rewrites `genome.fitness` before serializing, so the renderer reads the training-time fitness field. | Bug 2 |
| 20 | `data.dat` logging (`logEvalResults`) | L1071 | L803 | No (shared helper) | — |

**Rows marked divergent:** 3, 5, 6, 9, 9b, 16, 17, 19 → **6 underlying
bugs** (rows 9/9b are two instances of the same duplication issue; rows
16/19 both flow from Bug 2's root cause).

---

## Refactor Proposal

### Proposed helper

```cpp
// src/autoc.cc (file-local or under autoc/eval/)

enum class EvalPurpose {
  Training,        // Per-individual fitness during evolution
  EliteReeval,     // Best-of-generation deterministic re-run
  StandaloneEval,  // evaluateMode=1 one-shot weight evaluation
};

struct EvalJob {
  const ScenarioDescriptor& scenario;
  const std::vector<char>& nnBytes;   // Already-serialized NN01 payload
  EvalPurpose purpose;
};

// Populates every field of EvalData, respecting all globals:
//  - gRabbitSpeedConfig (with computeVariationScale() applied to sigma)
//  - gScenarioVariations (via populateVariationOffsets)
//  - globalScenarioCounter (advances by one per call)
// Also sets isEliteReeval and meta.enableDeterministicLogging consistently
// based on `purpose`.
EvalData buildEvalData(const EvalJob& job);
```

### Inputs the helper needs (and who supplies them)

| Input | Training (per-ind) | Training (elite) | Eval |
|---|---|---|---|
| `ScenarioDescriptor` | `scenarioForIndex(ind % N)` | `scenarioForIndex(bestIdx % N)` | **Currently `scenarioForIndex(0)`** — should become "same pick rule as training elite re-eval" so the eval path covers the same scenario as training elite re-eval does, OR a configurable flag |
| `nnBytes` | `nn_serialize(genome)` | `nn_serialize(bestGenome)` | `nn_serialize(genome)` (genome loaded from file) |
| `purpose` | `Training` | `EliteReeval` | `StandaloneEval` (behaves like `EliteReeval` for determinism logging) |

The helper reads `gRabbitSpeedConfig`, `computeVariationScale()`,
`globalScenarioCounter`, `gScenarioVariations`, and calls
`populateVariationOffsets` internally — no caller needs to remember them.

### What it returns / populates

A fully-built `EvalData` ready for `sanitizePaths()` + `sendRPC()`. Every
field that is currently touched by any of the three call sites is set.

### Which caller currently uses each parameter differently

| Field | Training per-ind | Training elite | Eval |
|---|---|---|---|
| `isEliteReeval` | `false` | `false` (also silently "should be true" IMO) | `false` |
| `rabbitSpeedConfig` | ramp-scaled | ramp-scaled | **default (16.0, 0.0)** — this is Bug 3 |
| scenario index | `ind % N` | `bestIdx % N` | `0` |
| determinism-log flag on meta | `false` | `false` | `false` |
| genome.fitness write-back | yes (L986) | elite flag keeps existing | **no** — Bug 2 |

### Migration order

1. **Extract the helper without behavior changes.** Move the per-individual
   scenario-list build code (`src/autoc.cc:932-975`) into
   `buildEvalData(EvalJob)` and route the per-individual loop through it.
   Run tests; expect zero fitness delta because it is the exact same code.

2. **Route training elite re-eval through the helper.** Delete the copy at
   `src/autoc.cc:1021-1060`. Run tests; expect zero delta because Copy B and
   Copy A are already functionally identical.

3. **Route `runNNEvaluation()` through the helper.** Delete
   `src/autoc.cc:751-787`. At this step the eval path will *start* setting
   `rabbitSpeedConfig` correctly — this is the behavior change that fixes
   Bug 3. Acceptance test must be updated to expect the new (correct)
   fitness.

4. **Fix Bug 2 inside the eval path.** After the helper call,
   `runNNEvaluation()` should do
   `genome.fitness = aggregateRawFitness(...); nn_serialize(genome, nnData);
   evalResults.gp.assign(nnData...)`. The renderer will then read the
   eval-computed fitness.

5. **Fix Bug 5 (scenario-0-only).** Either loop over all scenarios in eval
   mode (matching training), or expose a config key. Recommended: match
   training — iterate `for (size_t s = 0; s < generationScenarios.size();
   ++s) buildEvalData({scenarioForIndex(s), ...})` and aggregate across all
   of them, so demetic training runs can be evaluated faithfully.

6. **Fix Bug 4 (isEliteReeval / deterministic logging).** Set
   `purpose = EliteReeval` in `runNNEvaluation()` so the sim captures the
   deterministic per-step trail that `data.dat` expects.

7. **Add a determinism check in eval mode.** Mirror the training
   `bitwiseEqual` check at `src/autoc.cc:1077-1087`: after computing eval
   fitness, log whether it matches the stored NN01 fitness, instead of
   silently reporting both numbers. This catches future regressions.

8. **Verify end-to-end.** See acceptance criterion below.

### Risks during migration

- **The elite re-eval block has a side effect the per-individual loop does
  not have: it calls `logEvalResults(fout, bestResults)` into `data.dat` and
  uploads to S3.** If the helper is over-zealous and merges these steps
  into itself, training will start writing `data.dat` once per individual
  (wrong). Keep `logEvalResults` and the S3 upload *outside* the helper.
  The helper's sole job is `EvalData` construction.
- **The eval path's `data.dat` has the "eval-" prefix via
  `src/autoc.cc:1342-1344`.** That logic lives in `main()` and is
  independent of the helper — don't disturb it.
- **`gCurrentGeneration == 0` in eval mode → `computeVariationScale()` returns
  1.0** via the early exit at `src/autoc.cc:80` (`numSteps <= 1`). This is
  fine and should be preserved — the helper will naturally get scale=1.0 in
  eval mode and scale<=1.0 in training mode.
- **`globalScenarioCounter` must still increment once per `EvalData`.**
  Since the helper owns the `fetch_add`, callers must not increment it
  separately (today the eval path increments it at L749 and the training
  per-individual loop at L930 — they both need to be removed in favor of
  the helper).

### Acceptance criterion

The refactor is considered correct iff ALL of the following hold:

1. **Reproduces betterz2 gen-400 fitness.** Loading
   the saved betterz2 weights (stored NN01 fitness = −34771) via eval mode
   with `autoc-eval.ini` configured to match the training-time config
   (same paths, same scenarios, same rabbit speed config) reproduces the
   training-time fitness to within floating-point rounding.
   *This is a non-trivial bar because today the eval path runs at rabbit
   speed 16 m/s constant instead of the configured 13±2 m/s — the
   refactor is the first time this number should actually match.*

2. **Bitwise identical `ScenarioScore` for a given (weight, scenario) pair.**
   A test that takes one NN weight and one `ScenarioDescriptor`, runs it
   once through the training per-individual path and once through the eval
   path, must produce byte-identical `ScenarioScore` values (same
   `maxStreak`, `totalStreakSteps`, `maxMultiplier`, `score`, `crashed`,
   `steps_completed`, `steps_total`).

3. **No recurrence by construction.** After the refactor, Bug 1/2/3/4/5/6
   cannot recur because the divergent code no longer exists — there is one
   `buildEvalData()` helper and every call site routes through it. A unit
   test asserts that `buildEvalData()` always sets `rabbitSpeedConfig` to
   something other than the struct default.

4. **Determinism check passes in eval mode.** The new
   `bitwiseEqual`-style check in `runNNEvaluation()` logs "SAME" (not
   "DIVERGED") when an eval run is performed against a weight file that
   was saved during a training run with identical seed + config.

---

## Files Audited

- `/home/gmcnutt/autoc/src/autoc.cc`
  - `main()` — L1209-1420
  - `runNNEvaluation()` — L697-856
  - `runNNEvolution()` — L859-1207
  - Per-individual eval block — L921-988
  - Elite re-eval block — L1013-1117
  - `computeNNFitness()` — L546-549
  - `logEvalResults()` — L553-694
  - `rebuildGenerationScenarios()` — L310-418
  - `populateVariationOffsets()` — L241-285
  - `prefetchAllVariations()` — L125-194
  - `computeVariationScale()` — L76-84
  - Globals: `gVariationSigmas` L57; `gEnableEntryVariations`/`gEnableWindVariations` L59-60; `gRabbitSpeedConfig` L63; ramp globals L66-68; `gScenarioVariations` L106; `gPathSeed` L107-108; `generationPaths`/`generationScenarios` L53-54; `globalScenarioCounter`/`globalSimRunCounter` L293-294
- `/home/gmcnutt/autoc/src/eval/fitness_decomposition.cc`
  - `computeScenarioScores()` L9-88
  - `aggregateRawFitness()` L90-96
- `/home/gmcnutt/autoc/include/autoc/eval/fitness_decomposition.h`
  - `struct ScenarioScore` L11-25
- `/home/gmcnutt/autoc/include/autoc/rpc/protocol.h`
  - `struct EvalData` L127-154 (default `rabbitSpeedConfig` at L135)
- `/home/gmcnutt/autoc/include/autoc/eval/variation_generator.h`
  - `RabbitSpeedConfig::defaultConfig()` L139-141 — the `{16.0, 0.0, ...}`
    that Bug 3 inherits
- `/home/gmcnutt/autoc/specs/BACKLOG.md` — "Eval Fitness Computation — Bugs"
  section L109-132

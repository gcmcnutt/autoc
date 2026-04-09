# Contract: Common Evaluator Interface (`buildEvalData`)

**Feature**: 023 | **Phase**: 1 | **Scope**: shared helper consolidating training + eval scenario construction (Phase 0b refactor)

## Motivation

Pre-023, `src/autoc.cc` contained **three near-identical copies** of the
scenario-list construction code:

1. Per-individual training loop (`src/autoc.cc:932-975`)
2. Elite re-eval block (`src/autoc.cc:1021-1060`)
3. Standalone eval mode (`src/autoc.cc:751-787`)

This duplication caused the BACKLOG "Eval Fitness Computation — Bugs"
family (Bugs 1/2/3) and two additional latent bugs (4 and 5) discovered
during the Phase 0 train/eval code dedup survey. See
`train-eval-code-dedup.md` for the full divergence table.

Phase 0b consolidates the three copies into one `buildEvalData(EvalJob)`
helper. **This contract pins down the helper's callin/callout surface so
future implementation changes cannot silently break one caller while
keeping another working.**

## Interface

### Types

```cpp
// src/autoc.cc or include/autoc/eval/build_eval_data.h (NEW)

enum class EvalPurpose {
    Training,        // Per-individual fitness during evolution generation loop
    EliteReeval,     // Best-of-generation deterministic re-run + S3 upload + data.dat
    StandaloneEval,  // evaluateMode=1 one-shot weight evaluation
};

struct EvalJob {
    const ScenarioDescriptor& scenario;  // From scenarioForIndex() at the call site
    const std::vector<char>& nnBytes;    // Already-serialized NN01 payload
    EvalPurpose purpose;                 // Tags the call site for behavior differences
};
```

### Signature

```cpp
/**
 * Build a fully-populated EvalData for one NN evaluation.
 *
 * Reads the following globals (CALLIN surface — see §Callins below):
 *   - gRabbitSpeedConfig
 *   - gScenarioVariations
 *   - gCurrentGeneration / gTotalGenerations / gVariationRampStep
 *     (via computeVariationScale())
 *   - globalScenarioCounter (advanced atomically by one per call)
 *   - generationPaths (via scenario.pathList)
 *
 * Populates the following fields on EvalData (CALLOUT surface — see §Callouts):
 *   - controllerType = NEURAL_NET
 *   - gp (copy of nnBytes)
 *   - gpHash = hashByteVector(gp)
 *   - isEliteReeval = (purpose != Training)
 *   - rabbitSpeedConfig = gRabbitSpeedConfig with sigma *= computeVariationScale()
 *   - pathList = job.scenario.pathList
 *   - scenario = first element of scenarioList
 *   - scenarioList = built from job.scenario per demetic-aware logic
 *   - scenario.scenarioSequence / bakeoffSequence from global counter
 *   - enableDeterministicLogging = (purpose != Training)
 *
 * Does NOT:
 *   - Call sanitizePaths() — caller's responsibility after this returns
 *   - sendRPC / receiveRPC — caller's responsibility
 *   - Log to data.dat — caller's responsibility (only EliteReeval and
 *     StandaloneEval do this; Training does not)
 *   - Upload to S3 — caller's responsibility
 *   - Touch genome.fitness — caller's responsibility after scoring
 *
 * @param job  EvalJob specifying scenario, NN bytes, and purpose
 * @return     Fully-populated EvalData ready for sanitizePaths() + sendRPC
 */
EvalData buildEvalData(const EvalJob& job);
```

## Callin surface (globals the helper reads)

The helper is NOT pure — it reads globals. This is a design choice to keep the
caller API narrow (pass one `EvalJob`, get one `EvalData`) rather than
threading every config value through the signature. But the set of globals
read IS a contract: adding a new global read is a behavior change that must
be reflected in every caller's understanding of what the helper depends on.

| Global | Source file | Used for |
|---|---|---|
| `gRabbitSpeedConfig` | `src/autoc.cc:63` | Copied into `evalData.rabbitSpeedConfig`, sigma scaled by variation ramp |
| `gScenarioVariations` | `src/autoc.cc:106` | Read inside `populateVariationOffsets()` (which the helper calls) |
| `gEnableEntryVariations` | `src/autoc.cc:59` | Read inside `populateVariationOffsets()` |
| `gEnableWindVariations` | `src/autoc.cc:60` | Read inside `populateVariationOffsets()` |
| `gVariationSigmas` | `src/autoc.cc:57` | Read inside `populateVariationOffsets()` |
| `gCurrentGeneration` | `src/autoc.cc:66` | Read by `computeVariationScale()` for ramp scaling |
| `gTotalGenerations` | `src/autoc.cc:67` | Read by `computeVariationScale()` |
| `gVariationRampStep` | `src/autoc.cc:68` | Read by `computeVariationScale()` |
| `globalScenarioCounter` | `src/autoc.cc:293` | `fetch_add(1)` for `scenario.scenarioSequence` |
| `globalSimRunCounter` | `src/autoc.cc:294` | `fetch_add(1)` for `scenario.bakeoffSequence` |

**Invariant**: this list IS the complete callin surface. Any future
modification that reads an additional global violates the contract unless it
also updates this list AND the corresponding contract test in
`tests/build_eval_data_contract_tests.cc`.

## Callout surface (fields the helper writes on `EvalData`)

Every `EvalData` field the helper writes must be documented here. Any field
on `EvalData` that the helper does NOT write must be explicitly flagged as
"caller's responsibility" or "left at struct default". Silently defaulted
fields are the failure mode this contract prevents (see Bug 3 in the dedup
survey — `rabbitSpeedConfig` was silently defaulted to `{16.0, 0.0, ...}`).

| `EvalData` field | Written by helper? | Value |
|---|---|---|
| `controllerType` | Yes | `NEURAL_NET` |
| `gp` | Yes | Copy of `job.nnBytes` |
| `gpHash` | Yes | `hashByteVector(gp)` |
| `isEliteReeval` | Yes | `(job.purpose != EvalPurpose::Training)` |
| `rabbitSpeedConfig` | **Yes (Bug 3 fix)** | `gRabbitSpeedConfig` with `sigma *= computeVariationScale()` |
| `engage_delay_ms` | Yes | `gEngageDelayMs` (new in 023, from Change 1b) |
| `pathList` | Yes | `job.scenario.pathList` |
| `scenario` | Yes | First element of `scenarioList` after construction |
| `scenarioList` | Yes | Built per-flight, demetic-aware, with `populateVariationOffsets` applied |
| Each `ScenarioMetadata.pathVariantIndex` | Yes | Demetic decoding |
| Each `ScenarioMetadata.windVariantIndex` | Yes | Demetic decoding |
| Each `ScenarioMetadata.windSeed` | Yes | From `gScenarioVariations[windVariantIndex].windSeed` |
| Each `ScenarioMetadata.rabbitSpeed` | Yes | `gRabbitSpeedConfig.nominal` (the scalar seed; actual profile is sigma-driven) |
| Each `ScenarioMetadata.enableDeterministicLogging` | **Yes (Bug 4 fix)** | `(job.purpose != EvalPurpose::Training)` |
| Each `ScenarioMetadata.entryOffset/Cone/Roll/Speed/etc` | Yes | Set by `populateVariationOffsets(meta)` |
| Each `ScenarioMetadata.originOffset` | Caller's responsibility | Set by CRRCSim at scenario start (per 022 coordinate cleanup) |

**Fields the helper does NOT write** (must be preserved by callers or left
at struct default if appropriate):

- `EvalData`'s outer `scenario.scenarioSequence` / `bakeoffSequence` —
  written INSIDE the helper via the global counters
- Anything RPC-protocol-internal (cereal metadata, etc.)

### Left at struct default

None. Every meaningful field is either written by the helper or explicitly
set by the caller after the helper returns. If a future `EvalData` field
is added to `protocol.h` that isn't covered by the helper OR a caller,
the contract test MUST fail until it's classified.

## Caller contract (what each caller must do AFTER the helper returns)

All three callers (Training, EliteReeval, StandaloneEval) must:

```cpp
EvalData evalData = buildEvalData(EvalJob{scenario, nnBytes, purpose});
evalData.sanitizePaths();                          // required
sendRPC(evalData);
auto response = receiveRPC();
auto scores = computeScenarioScores(response);
double fitness = aggregateRawFitness(scores);
// Now store fitness — purpose-specific:
```

### Purpose-specific post-processing

| Caller | Additional steps after aggregateRawFitness |
|---|---|
| `Training` | `genome.scenario_scores = scores; genome.fitness = fitness;` |
| `EliteReeval` | `genome.fitness = fitness;` + `logEvalResults(fout, response);` + `nn_serialize(genome, bytes); uploadToS3(bytes, "gen"+N+".dmp");` + optional `bitwiseEqual` determinism check vs previously stored weights |
| `StandaloneEval` | **Bug 2 fix**: `genome.fitness = fitness; nn_serialize(genome, nnData); evalResults.gp = nnData;` BEFORE `uploadToS3(evalResults)`. Plus optional determinism check. |

The helper MUST NOT do any of these steps itself. They are caller-specific.
The helper's sole job is `EvalData` construction.

## Contract tests (`tests/build_eval_data_contract_tests.cc` NEW)

One test per (field × caller) pair, plus cross-caller consistency checks.

### Test A: "no field defaulted silently"

For each `EvalPurpose` value, call `buildEvalData()` with a synthetic
`EvalJob` and assert that every field in the callout table is NOT equal
to its struct default. If any field comes out at its struct default
(e.g., `rabbitSpeedConfig = {16.0, 0.0, ...}` when the test fixture
set `gRabbitSpeedConfig = {13.0, 2.0, ...}`), the test fails loud with
the field name.

```cpp
TEST(BuildEvalDataContract, TrainingPurposePopulatesEveryField) {
    SetUpGlobalsForTest();  // sets gRabbitSpeedConfig, gScenarioVariations, etc.
    EvalJob job{getTestScenario(), getTestNNBytes(), EvalPurpose::Training};
    EvalData ed = buildEvalData(job);

    EXPECT_EQ(ed.controllerType, NEURAL_NET);
    EXPECT_NE(ed.rabbitSpeedConfig.nominal, 16.0)
        << "Bug 3 regression: rabbitSpeedConfig still at default";
    EXPECT_EQ(ed.rabbitSpeedConfig.nominal, gRabbitSpeedConfig.nominal);
    // ... one assertion per field in the callout table ...
}

TEST(BuildEvalDataContract, EliteReevalPurposePopulatesEveryField) { ... }
TEST(BuildEvalDataContract, StandaloneEvalPurposePopulatesEveryField) { ... }
```

### Test B: "purpose-specific field values"

```cpp
TEST(BuildEvalDataContract, TrainingHasNoDeterministicLogging) {
    auto ed = buildEvalData(EvalJob{..., EvalPurpose::Training});
    EXPECT_FALSE(ed.isEliteReeval);
    for (const auto& meta : ed.scenarioList) {
        EXPECT_FALSE(meta.enableDeterministicLogging);
    }
}

TEST(BuildEvalDataContract, EliteReevalHasDeterministicLogging) {
    auto ed = buildEvalData(EvalJob{..., EvalPurpose::EliteReeval});
    EXPECT_TRUE(ed.isEliteReeval);
    for (const auto& meta : ed.scenarioList) {
        EXPECT_TRUE(meta.enableDeterministicLogging);
    }
}

TEST(BuildEvalDataContract, StandaloneEvalHasDeterministicLogging) {
    // Bug 4 fix: standalone eval semantically IS an elite re-eval
    auto ed = buildEvalData(EvalJob{..., EvalPurpose::StandaloneEval});
    EXPECT_TRUE(ed.isEliteReeval);
    for (const auto& meta : ed.scenarioList) {
        EXPECT_TRUE(meta.enableDeterministicLogging);
    }
}
```

### Test C: "cross-caller consistency"

Pair of tests that confirm two different purposes produce
**byte-identical** `EvalData` for every field EXCEPT the purpose-gated
ones (`isEliteReeval`, `enableDeterministicLogging`).

```cpp
TEST(BuildEvalDataContract, TrainingAndStandaloneEvalDifferOnlyInPurposeFields) {
    SetUpGlobalsForTest();
    auto scenario = getTestScenario();
    auto nn = getTestNNBytes();

    auto ed_training = buildEvalData({scenario, nn, EvalPurpose::Training});
    auto ed_eval = buildEvalData({scenario, nn, EvalPurpose::StandaloneEval});

    // Reset the purpose-gated fields on both so we can compare the rest
    ed_training.isEliteReeval = false;
    ed_eval.isEliteReeval = false;
    for (auto& m : ed_training.scenarioList) m.enableDeterministicLogging = false;
    for (auto& m : ed_eval.scenarioList)     m.enableDeterministicLogging = false;

    // The counters will have advanced, so compare by field, not by memcmp on EvalData
    // But the scenario metadata content should match field-by-field:
    EXPECT_EQ(ed_training.rabbitSpeedConfig.nominal,
              ed_eval.rabbitSpeedConfig.nominal);
    EXPECT_EQ(ed_training.rabbitSpeedConfig.sigma,
              ed_eval.rabbitSpeedConfig.sigma);
    EXPECT_EQ(ed_training.pathList.size(),
              ed_eval.pathList.size());
    EXPECT_EQ(ed_training.scenarioList.size(),
              ed_eval.scenarioList.size());
    for (size_t i = 0; i < ed_training.scenarioList.size(); ++i) {
        EXPECT_EQ(ed_training.scenarioList[i].pathVariantIndex,
                  ed_eval.scenarioList[i].pathVariantIndex);
        EXPECT_EQ(ed_training.scenarioList[i].windVariantIndex,
                  ed_eval.scenarioList[i].windVariantIndex);
        EXPECT_EQ(ed_training.scenarioList[i].rabbitSpeed,
                  ed_eval.scenarioList[i].rabbitSpeed);
        // ... etc ...
    }
}
```

### Test D: "callin surface"

Tests that assert the helper reads what it says it reads, and nothing else.

```cpp
TEST(BuildEvalDataContract, ReadsGRabbitSpeedConfig) {
    // Set gRabbitSpeedConfig to a unique sentinel
    gRabbitSpeedConfig = {42.0, 3.14, 8.0, 99.0, 1.0, 2.0};
    auto ed = buildEvalData({getTestScenario(), getTestNNBytes(), EvalPurpose::Training});
    EXPECT_EQ(ed.rabbitSpeedConfig.nominal, 42.0);
    EXPECT_DOUBLE_EQ(ed.rabbitSpeedConfig.sigma, 3.14 * computeVariationScale());
}

TEST(BuildEvalDataContract, AdvancesGlobalScenarioCounter) {
    auto before = globalScenarioCounter.load();
    buildEvalData({getTestScenario(), getTestNNBytes(), EvalPurpose::Training});
    auto after = globalScenarioCounter.load();
    EXPECT_EQ(after - before, 1);
}
```

### Test E: "eval path scenario iteration" (Bug 5 fix)

```cpp
TEST(BuildEvalDataContract, StandaloneEvalCoversAllScenarios) {
    // Set up a scenario table with N > 1 demes
    // The caller loop for StandaloneEval should iterate all N, not just 0
    // This is the test that Bug 5 (scenarioForIndex(0) hard-coded in eval) is fixed.
    // Note: this test is on the EVAL CALLER LOOP not the helper itself.
    // Lives in eval_determinism_tests.cc rather than build_eval_data_contract_tests.cc.
}
```

## Relationship to other tests

- **`tests/eval_determinism_tests.cc`** (specified in plan.md) — verifies
  end-to-end determinism: eval on saved betterz2 weights reproduces training
  fitness within FP rounding. High-level correctness.
- **`tests/build_eval_data_contract_tests.cc`** (this contract) — verifies
  structural invariants on the helper's output. Low-level correctness that
  catches drift before it reaches determinism testing.

Both are needed. The contract test catches "someone added a new field to
EvalData and forgot to wire it up in the helper for the StandaloneEval
path" — a specific failure mode that the determinism test would miss
(because determinism might still hold for the particular field's default).

## Migration order (Phase 0b, from train-eval-code-dedup.md)

The contract test suite must exist BEFORE the refactor begins. That way:

1. First commit: add `buildEvalData()` stub + contract tests. Tests FAIL
   because the helper isn't implemented.
2. Second commit: implement the helper by moving code from the training
   per-individual loop (`src/autoc.cc:932-975`). Training path starts
   calling it. Contract tests pass for `EvalPurpose::Training`.
3. Third commit: route training elite re-eval through the helper. Delete
   the elite copy at `src/autoc.cc:1021-1060`. Contract tests pass for
   `EvalPurpose::EliteReeval`.
4. Fourth commit: route `runNNEvaluation()` through the helper. Delete
   the eval copy at `src/autoc.cc:751-787`. Contract tests pass for
   `EvalPurpose::StandaloneEval`. **This is the commit that fixes
   Bug 3** (rabbitSpeedConfig now populated from the global).
5. Fifth commit: Fix Bug 2 in eval path post-processing (assign
   `genome.fitness` before serialize).
6. Sixth commit: Fix Bug 5 (iterate all scenarios in eval mode).
7. Seventh commit: Fix Bug 4 (use `EvalPurpose::StandaloneEval` which
   already sets `isEliteReeval=true` via the helper).
8. Eighth commit: Add `eval_determinism_tests.cc` acceptance test and
   verify betterz2 gen-400 fitness reproduces.

Between each commit, the full test suite must pass. No half-refactored
state on the branch per Constitution II.

## Files touched by Phase 0b

| File | Change |
|---|---|
| `include/autoc/eval/build_eval_data.h` | NEW — `EvalPurpose`, `EvalJob`, `buildEvalData()` declaration |
| `src/autoc.cc` | Delete 3 copies of scenario-list populator; add helper call in each call site |
| `tests/build_eval_data_contract_tests.cc` | NEW — this contract's test suite |
| `tests/eval_determinism_tests.cc` | NEW — end-to-end acceptance test (from plan.md) |
| `include/autoc/rpc/protocol.h` | Add `engage_delay_ms` field (from Change 1b — may be same commit or separate) |

## Future shape: class hierarchy (recommended follow-up refactor)

The `EvalJob { ..., EvalPurpose }` + branching-on-enum design above is a
**minimal-churn migration shape**. It collapses three duplicate copies of
scenario construction into one function with purpose-gated behavior.

The **cleaner long-term shape** is an `Evaluator` base class with three
concrete subclasses — one per purpose — where implementation-specific
behavior lives in method overrides rather than `if (purpose == ...)`
branches inside the helper.

```cpp
class Evaluator {
public:
    virtual ~Evaluator() = default;

    // Pure construction — shared across all implementations.
    // Uses the template method pattern: construction is in the base class,
    // purpose-specific values come from virtual hooks.
    EvalData buildEvalData(const ScenarioDescriptor& scenario,
                           const std::vector<char>& nnBytes);

    // Caller runs sanitizePaths + sendRPC + receiveRPC + computeScenarioScores
    // + aggregateRawFitness, then calls afterScoring() with the results.
    virtual void afterScoring(NNGenome& genome,
                              double fitness,
                              const ScenarioScores& scores,
                              const EvalData& evalData,
                              const RPCResponse& response) = 0;

protected:
    // Purpose-specific knobs exposed as virtual hooks
    virtual bool isEliteReeval() const = 0;
    virtual bool enableDeterministicLogging() const = 0;
};

class TrainingEvaluator : public Evaluator {
    bool isEliteReeval() const override { return false; }
    bool enableDeterministicLogging() const override { return false; }
    void afterScoring(NNGenome& g, double f, ...) override {
        g.scenario_scores = scores;
        g.fitness = f;
        // No data.dat log, no S3 upload — training writes these only for
        // the elite at end-of-generation.
    }
};

class EliteReevalEvaluator : public Evaluator {
    bool isEliteReeval() const override { return true; }
    bool enableDeterministicLogging() const override { return true; }
    void afterScoring(NNGenome& g, double f, ...) override {
        g.fitness = f;
        logEvalResults(fout_, response);
        std::vector<char> bytes;
        nn_serialize(g, bytes);
        uploadToS3(bytes, s3KeyForElite(currentGeneration_));
    }
};

class StandaloneEvalEvaluator : public Evaluator {
    // Bug 4 fix: StandaloneEval semantically IS an elite re-eval
    bool isEliteReeval() const override { return true; }
    bool enableDeterministicLogging() const override { return true; }
    void afterScoring(NNGenome& g, double f, ...) override {
        g.fitness = f;                   // Bug 2 fix
        std::vector<char> bytes;
        nn_serialize(g, bytes);          // Serialize AFTER writing fitness
        evalResults.gp = bytes;          // Store in evalResults
        uploadToS3(evalResults, "gen9999.dmp");
    }
};
```

### Benefits vs the enum approach

1. **Compiler enforces "every concrete evaluator implements every hook"**
   via pure virtual. Adding a new hook (e.g., `bool shouldCaptureBlackbox()
   const = 0;`) forces all three subclasses to address it explicitly.
   Same compile-time safety philosophy as the type-safe NN interface.
2. **No enum-dispatch branching** inside the helper. Purpose-specific
   behavior lives in one class per purpose — easier to read and reason
   about.
3. **Contract test becomes "each subclass passes the same base-class
   suite"** — strictly simpler than the enum-parameterized test.
4. **Natural extension path** — if we later want a fourth evaluator
   (e.g., "ensemble benchmark evaluator" that averages over multiple
   scenarios per NN), it's a new subclass rather than another enum value
   and another `if`-branch everywhere.

### Why enum first, class hierarchy second

The migration order specified above lands the functional fixes (Bugs
1/2/3/4/5/6) in the enum form because:

- Minimal diff per commit — easier to review each bug fix in isolation
- Determinism tests land between the fixes and the OO refactor, so the
  OO refactor inherits a known-correct baseline
- If the OO refactor turns out to be over-engineered for the actual call
  patterns we have (three callers, likely stable for a while), we can
  skip it and keep the enum form. Whereas if we try OO first and find
  we need to revert, we've wasted commits.

### Recommended follow-up commit (Phase 0b step 9)

After the Phase 0b migration completes (step 8 — eval determinism
acceptance test passes), a single commit upgrades the helper from the
enum shape to the class hierarchy shape. This commit:

- Creates `class Evaluator` base + three concrete subclasses
- Migrates each call site from `buildEvalData(EvalJob{..., purpose})` to
  instantiating the appropriate subclass and calling its methods
- Removes `EvalPurpose` enum entirely
- Updates contract tests to use the class hierarchy
- **Must pass all existing tests unchanged** (determinism,
  contract assertions) — the refactor changes access pattern, not
  behavior

This is a "same behavior, different shape" refactor that's safe to do
AFTER the correctness fixes land. It should NOT be attempted before
them, because combining behavior change + structural refactor in one
commit is exactly what created the Bug 3/4/5/6 drift in the first place.

### Interaction with type-safe NN interface

The OO refactor has a parallel to the type-safe NN interface in
`contracts/nn_interface.md`:

- `NNInputs` struct catches "someone added a new input and didn't
  populate it" at compile time.
- `Evaluator` base class catches "someone added a new post-scoring
  behavior and didn't implement it in one of the three callers" at
  compile time.

Both are about using the compiler as a tool to prevent silent drift
between implementations of the same abstract interface. Good design.

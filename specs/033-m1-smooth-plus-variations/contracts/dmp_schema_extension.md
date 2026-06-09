# Contract: `EvalResults` dmp schema extension — `scenarioSeedList[K]`

**Producer**: autoc-side dmp write site (`src/autoc.cc` near existing `EvalResults` population — same write path that already records `scenarioList` / `variationRampList` / etc.).
**Consumer**: `tools/tracker_dmp_inspect.cc` (operator inspection), eval-mode replay path (`src/eval/...` — bit-identical reproduction of training run from a single dmp), future intercept-analysis scripts.

## Schema delta

```cpp
// include/autoc/rpc/protocol.h (modified)

struct EvalResults {
  // ... existing fields unchanged ...
  std::vector<gp_fitness> scenarioFitnessList;     // existing — per-scenario fitness
  std::vector<gp_scalar>  variationRampList;       // existing — per-scenario variation ramp
  std::vector<uint64_t>   scenarioSeedList;        // NEW — per-scenario master-derived seed (033 §2.A)
  // ... other existing fields unchanged ...

  template <class Archive>
  void serialize(Archive& ar) {
    // ... existing field walks unchanged ...
    ar(scenarioFitnessList);
    ar(variationRampList);
    ar(scenarioSeedList);                          // NEW — appended at end of serialize block
    // ... other existing field walks unchanged ...
  }
};
```

### Field semantics

- **`scenarioSeedList[K]`**: the `uint64_t` master-derived seed for scenario K, as populated by autoc-side init (see `scenario_prng_chain.md` step 6).
- **Parallel indexing**: `scenarioSeedList[K]` corresponds to the same scenario K as `scenarioList[K]`, `scenarioFitnessList[K]`, `variationRampList[K]`. Length invariant: `scenarioSeedList.size() == scenarioList.size()` (asserted on read).
- **Sufficient for full replay**: given `scenarioSeedList[K]`, the worker can deterministically derive all 5 class sub-seeds (wind, rabbit, entry, craft, camera) — no need to persist them individually.

## NO cereal version bump

Per [feedback_no_cereal_versioning](../../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) and the precedent established by 032 (tracker dmp v=2 added derived-feature fields without bumping). 033 follows the same policy:

- DO NOT bump `CEREAL_CLASS_VERSION(EvalResults, ...)`.
- Append the new `scenarioSeedList` `ar()` call AT THE END of the serialize block — order-of-appearance is the on-disk encoding.
- Pre-033 dmps lack the field → cereal's read will hit EOF mid-stream → throws → loud-fail at deserialization. This is the desired behavior per Constitution III (no shims).

### Why no version bump

- Project policy: M1 binary reproducibility is enforced by checksum + bake, not by schema version. Schema growth is greenfield; old dmps stop being readable on the new binary.
- Op flow: when a phase-N dmp becomes unreadable on phase-(N+1) binary, the operator re-bakes (cheap during active dev; deferred dmps are not a load-bearing asset).

### What happens if a pre-033 dmp is loaded into a 033 binary

```text
[ERROR] cereal::archive — read past end of input stream while loading EvalResults::scenarioSeedList
abort()
```

Mitigation: operator notices, decides whether to re-bake or use the pre-033 binary tag for that specific replay. The fail-loud is the contract; there is no fallback path.

## Write site

```cpp
// src/autoc.cc, where existing EvalResults instance is populated
// (search for "scenarioFitnessList.push_back" or "variationRampList" to find the
// existing per-scenario population site — the same loop)

for (size_t k = 0; k < scenarios.size(); ++k) {
  evalResults.scenarioList.push_back(scenarios[k]);            // existing
  evalResults.scenarioFitnessList.push_back(scenarioFitnesses[k]);  // existing
  evalResults.variationRampList.push_back(variationRamps[k]);  // existing
  evalResults.scenarioSeedList.push_back(scenarioSeedTable[k]); // NEW
}
```

`scenarioSeedTable[k]` is the same table populated by `MasterPRNG.next()` per `scenario_prng_chain.md` step 5.

## Read sites

### `tools/tracker_dmp_inspect.cc`

Add a line near the existing per-scenario inspection output:

```cpp
std::cout << "  scenarioSeed:   0x" << std::hex << std::setw(16) << std::setfill('0')
          << evalResults.scenarioSeedList[k] << std::dec << "\n";
```

### Eval-mode replay path (`src/eval/...`)

The eval path consumes `scenarioSeedList[K]` as input to the `ScenarioRootPRNG` constructor for replaying that specific scenario:

```cpp
// Pseudo-code in eval-mode path
const uint64_t scenarioSeed = evalResults.scenarioSeedList[K];
ScenarioRootPRNG scenarioRoot(scenarioSeed);
// ... rest follows the same chain as worker (scenario_prng_chain.md steps 3-5)
```

This is the "(D4) Eval = training" guarantee from `scenario_prng_chain.md`.

## Validation tests

`tests/tracker_dmp_roundtrip_tests.cc` (EXTENDED):

- `scenarioSeedList_roundtrip`: serialize `EvalResults` with non-trivial scenarioSeedList → deserialize → assert bitwise-equal vector
- `scenarioSeedList_length_invariant`: read deserialized `EvalResults` → assert `scenarioSeedList.size() == scenarioList.size()`
- `pre_033_dmp_loud_fail`: attempt to load a dmp that lacks the scenarioSeedList field (or simulated equivalent — truncated stream) → assert cereal throws (or assert hard-fail behavior matches Constitution III)
- `scenarioSeed_zero_guard`: write scenarioSeedList containing the value 0 → assert serialize/deserialize roundtrips (the 0-guard is a downstream `ScenarioRootPRNG` concern, not a serialization concern — the dmp layer is value-agnostic)

`tests/eval_mode_replay_tests.cc` (NEW or EXTENDED):

- `single_scenario_replay_bit_identical`: bake a small run, capture training dmp → load via eval mode → replay scenario K → assert per-tick `aircraftStateList[K]` bitwise-equal to training trace. THIS IS THE D4 ENFORCEMENT TEST.

## Mode coverage — M1 and M2 both inherit

- M1 (pathgen) dmp: `EvalResults` is the same schema as M2; `scenarioSeedList` populated on both code paths from the same autoc-side write site
- M2 (tracker) dmp: identical schema growth
- No separate "tracker-mode dmp" extension; the existing `EvalResults` schema is the shared substrate

## Operator-facing impact

- Existing pre-033 `.dmp` files in S3 or local become unreadable by the 033 binary. Operator must rebake (or keep a pre-033 binary tag for one-off replays).
- New dmps grow by `8 bytes × num_scenarios` (e.g., 75 scenarios × 8B = 600B per dmp — negligible).
- `tracker_dmp_inspect` operator output shows scenarioSeed in hex on the per-scenario block, enabling the operator to feed a specific scenarioSeed back into `eval`-mode invocation for replay.

## Citations

- [spec.md](../spec.md) §2.A (PRNG architecture) + Work Item 3 (record scenarioSeed in EvalResults)
- [research.md](../research.md) R4 (EvalResults extension)
- [scenario_prng_chain.md](./scenario_prng_chain.md) (this contract supplies the value persisted here)
- [feedback_no_cereal_versioning](../../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) — no version bump policy
- [Constitution III](../../../.specify/memory/constitution.md) — no compatibility shims (drives the fail-loud-on-old-dmp behavior)
- [Constitution V](../../../.specify/memory/constitution.md) — versioned persistence artifacts (M2 exemption per project policy)

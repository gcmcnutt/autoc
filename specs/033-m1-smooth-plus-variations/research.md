# 033 — Research (Phase 0)

Resolves every NEEDS CLARIFICATION from [plan.md](./plan.md) Technical Context and locks the design decisions that the contracts + data-model depend on. Each finding follows Decision / Rationale / Alternatives, anchored to specific source files + line numbers from the 2026-05-20 code survey.

---

## R1 — Smoothness penalty insertion point: inside `FitnessComputer::applyStreak()` itself

**Decision**: Modify `FitnessComputer::applyStreak()` signature in [src/eval/fitness_computer.cc:57](../../src/eval/fitness_computer.cc#L57) to accept an additional `gp_scalar smoothnessFactor` parameter (default 1.0 = no-op for tests that don't care). Internal multiplication order: `return stepPoints * smoothnessFactor * multiplier`. Callers ([src/eval/fitness_decomposition.cc:169](../../src/eval/fitness_decomposition.cc#L169) and the per-tick loop callers in `pathgen_stepper.cc` / `tracker_stepper.cc`) compute `smoothnessFactor` from per-tick NN-output Δ via `compute_smoothness_factor()` (new pure helper in `derived_features.h`) and pass it through.

**Rationale**:
- Single integration point per Constitution III (no shim). `applyStreak()` becomes the contract: "give me stepPoints + smoothness factor + (managed internally: streak state), I give you final per-tick contribution to fitness."
- Streak-multiplier dynamics preserved exactly: the smoothness penalty applies to the BASE stepPoints, then streak compounds on the discounted value. Smooth controllers compound more (because each per-tick contribution is bigger), bang-bang controllers compound less. Operator-spec'd behavior per §2.B.
- Per-tick math is local — no cross-tick state needed beyond the streak machinery `applyStreak` already manages. Tracking "previous NN output for Δ computation" lives in the per-tick caller (stepper), not in `FitnessComputer` itself.
- Default value of 1.0 for the new parameter means existing test code in `fitness_computer_tests.cc` continues to compile; only new tests need to exercise non-1.0 paths.

**Alternatives considered**:
- **Add penalty in caller BETWEEN `decomposeStepScore()` and `applyStreak()`** (i.e., `multiplied = applyStreak(stepPoints * factor)` outside `applyStreak`). Rejected because it requires every caller to remember the order; one missed callsite silently disables smoothness for that mode. Single-internal-multiply is the safer contract.
- **Separate `applyStreakSmooth()` function**. Rejected — two parallel paths invite drift; the legacy callers that don't know about smoothness pass `factor=1.0` and get behavior-identical results.
- **Compute smoothness inside `applyStreak` from NN output state**. Rejected because `FitnessComputer` doesn't currently hold per-tick NN output history; threading it in would entangle `FitnessComputer` with the stepper-owned NN state. Cleaner to compute factor in the stepper and pass it as scalar.

---

## R2 — Scenario seed plumbing: extend `ScenarioMetadata` with `scenarioSeed`; coexists with `windSeed` initially

**Decision**: Add a new field `uint64_t scenarioSeed` to `ScenarioMetadata` ([include/autoc/rpc/scenario_metadata.h:26-65](../../include/autoc/rpc/scenario_metadata.h#L26-L65)). This becomes the per-scenario root for the new sub-PRNG class chain. The existing `windSeed` field stays in place initially (single-class consumer of the wind sub-PRNG); future cleanup may collapse it once all classes route through scenarioSeed-derived sub-PRNGs. Cereal serialize at [scenario_metadata.h:57](../../include/autoc/rpc/scenario_metadata.h#L57) gains the new field at the END (append-only) — old EvalData/WorkerInit dmps with the prior shape will fail-loud on read per project policy.

**Rationale**:
- Per spec Clarifications Q3 + Q5: `scenarioSeed[K]` is the SINGLE root that derives all class sub-PRNGs. The existing `windSeed` field is then derived from scenarioSeed → wind class sub-PRNG → first draw. Both can coexist during phase 1 implementation while we migrate consumers one-by-one; eventual cleanup removes `windSeed` when the wind path is fully routed through the chain.
- `EvalData` (per-eval wire payload, [include/autoc/rpc/protocol.h:210-220](../../include/autoc/rpc/protocol.h#L210-L220)) already carries `scenarioSequence + variationScale`. Adding `scenarioSeed[K]` to the worker-side cache via `WorkerInit.scenarioMetaList` (which IS where `ScenarioMetadata` lives, per [include/autoc/rpc/protocol.h:171-179](../../include/autoc/rpc/protocol.h#L171-L179)) means the worker has the seed available at scenario init without further wire-protocol changes.
- Autoc-side seed generation site: [src/autoc.cc:139-195](../../src/autoc.cc#L139-L195) (currently uses `rng::randLong()` from the global stream — to be changed to consume from a dedicated scenario-seed sub-stream of `MasterPRNG` per the spec init chain).

**Alternatives considered**:
- **Replace `windSeed` entirely in one shot**. Rejected — too many callsites depend on `windSeed` (crash-hull PRNG, wind init); incremental migration is safer. Phase 1 lands the new field + uses it for new code; phase 2/3 migrates the legacy windSeed consumers.
- **Derive scenarioSeed on the worker side from `scenarioSequence`**. Rejected — that re-introduces the "scenarioSequence drifts across runs" non-determinism documented in [project_v15_determinism_candidates](../../.claude/projects/-home-gmcnutt-autoc/memory/project_v15_determinism_candidates.md). Autoc generates scenarioSeed from masterPRNG once, persists it, ships to workers.

---

## R3 — Per-scenario sub-PRNG class structure: 5 classes (wind / rabbit / entry / craft / camera)

**Decision**: At scenario start (worker-side), derive 5 class sub-PRNGs from `scenarioSeed[K]`:

```cpp
ScenarioRootPRNG scenarioRoot(scenarioSeed[K]);
ClassPRNG windPRNG   (scenarioRoot.next());  // wind direction, strength, gust profile, vortex
ClassPRNG rabbitPRNG (scenarioRoot.next());  // rabbit speed, crash hull (M2)
ClassPRNG entryPRNG  (scenarioRoot.next());  // entry cone, roll, speed, position
ClassPRNG craftPRNG  (scenarioRoot.next());  // future 025 — mass, prop, drag
ClassPRNG cameraPRNG (scenarioRoot.next());  // future 034/035 — FOV, mount, lens
// extension slot — future variation classes: scenarioRoot.next() append-only
```

Each `ClassPRNG` is a self-contained PRNG (Park-Miller LCG, matching existing crash-hull PRNG per `tracker_stepper.cc` lines 47-49 pattern). Within a scenario, the class PRNG advances naturally as its consumers draw values. Multiple variables within a class share one PRNG (e.g., wind direction + wind strength + per-tick gust profile all consume from `windPRNG`).

**Rationale**:
- Per spec Clarifications Q5: "sub-PRNGs organized by VARIATION CLASS not per-variable slot". Append-only contract means adding a new variable within an existing class doesn't invalidate older scenario seeds; adding a new CLASS adds a new `scenarioRoot.next()` slot at the END.
- 5 classes match the spec table in §2.A. `craft` and `cameraPRNG` are seeded today but unused (no consumers) — that's correct per spec: slot is consumed for replay-stability so adding craft variations in 025 doesn't disturb wind/rabbit/entry sequences.
- Park-Miller LCG is the same algorithm already used in `TrackerStepper::prng_state_` ([src/eval/tracker_stepper.cc:47-49](../../src/eval/tracker_stepper.cc#L47-L49)) for crash-hull Bernoulli draws — single PRNG family across the codebase keeps the audit surface small.

**Alternatives considered**:
- **One PRNG per variable** (the original spec table had slots 0-8 for individual variables). Rejected per /clarify Q5 — class grouping is cleaner; per-variable adds bookkeeping without determinism benefit.
- **Use a stronger PRNG (xorshift, PCG)**. Rejected for phase 1 — match existing Park-Miller for codebase consistency; if the determinism properties of LCG become limiting we revisit in a backlog spec.
- **Cross-class sharing via a single PRNG**. Rejected — defeats the append-only contract and breaks cross-mode equivalence (M1's rabbit-speed draw would shift M2's wind draw if shared).

---

## R4 — `scenarioSeed[K]` dmp recording: add `scenarioSeedList` vector to `EvalResults`

**Decision**: Add `std::vector<uint64_t> scenarioSeedList` to `EvalResults` ([include/autoc/rpc/protocol.h:330-460](../../include/autoc/rpc/protocol.h#L330-L460)), parallel-indexed with `scenarioList`. Persist in cereal serialize at [protocol.h:354-367](../../include/autoc/rpc/protocol.h#L354-L367) at the END of the serialize sequence (append-only). One entry per scenario the eval ran. NO cereal class version bump per project policy [feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md); old dmps fail-loud on cereal archive-length mismatch.

**Rationale**:
- Per spec Clarifications Q5: scenario seed alone is sufficient for full replay — workers re-derive all class PRNGs deterministically from it; no need to record per-class sub-seeds.
- Adding to `EvalResults` (the dmp top-level struct) puts it in the right place for downstream consumers: renderer, eval pipeline, S3 dump tooling all read EvalResults.
- Parallel-indexed vector matches the existing convention (`scenarioList`, `cameraViewList`, `targetTrajectoryList`, `aircraftStateList` are all parallel-indexed per scenario index). Easy to extend per-scenario data going forward.
- Append-only cereal serialize position is the constitutional requirement (Principle V "writers MUST embed the version field at a stable, parseable offset that does not itself depend on later schema content" — adding a NEW field at the end doesn't invalidate earlier field positions).

**Alternatives considered**:
- **Record per-class sub-seeds (5 vectors)**. Rejected per /clarify Q5 — trivially derivable from scenarioSeed; redundant.
- **Record in `ScenarioMetadata.scenarioSeed` (already there per R2)**. Yes, AND ALSO add to `EvalResults`. The metadata field is for in-flight worker init; the EvalResults field is for after-the-fact replay. They're populated from the same source. Belt-and-suspenders for the load-side: if someone loads an EvalResults outside the WorkerInit context (e.g., the renderer or an analysis script), the seed is available without re-deriving from scenarioSequence.
- **Add to ScenarioMetadata only** (skip EvalResults). Rejected — `scenarioMetaList` lives in `WorkerInit` not `EvalResults`; an analysis script reading just the dmp wouldn't see the seed.

---

## R5 — M1 baseline streak% number: ≈ 30 maxStreak steps (extracted from 029 pastonly3 gen-800)

**Decision**: Phase-1 success criterion "streak% ≥ 2/3 of 029 pastonly3 baseline" pins to **avgMaxStreak ≥ 30 steps** as the M1 floor. Derived from [logs/autoc-029-pastonly3.log:237430-237450](../../logs/autoc-029-pastonly3.log#L237430-L237450) gen-800 elite telemetry: typical late-gen elite hits maxStreak=50 (the configured ramp-cap from `FitStreakRampSec * NN_TICK_HZ`) on most scenarios; average across the elite-eval ≈ 42-45 steps. 2/3 of 45 ≈ 30.

**Rationale**:
- Concrete number from actual data; not an estimate.
- `avgMaxStreak` is the right metric (vs `pctInStreak` which is a different `#GenDiag` column) because the user's framing is "if NN becomes too sluggish to maintain streaks, it drops" — streak duration is the load-bearing observation. The cap at 50 steps (5 sec at 10Hz) is set by `FitStreakRampSec * SIM_TIME_STEP_HZ`; an NN that holds streak to the cap is hitting the ceiling, an NN that drops to 30 is at the 2/3 mark.
- Operator can re-derive at any time: `grep "Gen 800" logs/autoc-029-pastonly3.log` → look for `avgMaxStreak` in the surrounding `#GenDiag` (column position per `src/autoc.cc` emit).

**Open**: cross-check that the eval-mode and training-mode `avgMaxStreak` numbers match for the same elite — variation-ramp pressure makes training-time number noisy per [project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md). For 033's success criterion the comparator should be **eval-mode plateau-avgMaxStreak** (variation-stable), not the training-time per-gen #GenDiag value. Will be re-extracted during 033 phase-1 closeout from an eval re-run of the 029 elite.

**Alternatives considered**:
- **Use `pctInStreak` as the metric**. Rejected as primary because it's % of ticks in streak, which conflates "long-streak rare" vs "many-short-streaks". `avgMaxStreak` captures duration directly.
- **Use raw best-fitness as the comparator** (e.g., "≥ 2/3 of 029 best-fitness"). Rejected because best-fitness includes the full streak compounding and is more sensitive to variation-ramp pressure; avgMaxStreak is the cleaner sluggishness proxy.

---

## R6 — NN-evolution PRNG decoupling: split today's single `rng::*` global into two streams

**Decision**: Keep the existing `rng::randLong / randDouble / randGaussian / randInt` global API in `include/autoc/util/rng.h` — back-compat for NN-evolution consumers ([src/nn/population.cc:11-13](../../src/nn/population.cc#L11-L13) population init / mutation / crossover / selection). Seed this stream from `masterPRNG.next()` at autoc startup. Per-scenario variation seeding (currently calling into the same `rng::*` stream via `joint_prng.cc`) migrates to consume from a separate `scenarioSeedTable[K]` that's populated from `masterPRNG.next()` calls 2..M+1 (after the NN-evolution stream seed).

The autoc-side change is therefore: `masterPRNG.init(masterSeed)` → `rng::seed(masterPRNG.next())` (NN evolution stream) → `for k in 0..M-1: scenarioSeedTable[k] = masterPRNG.next()`. The variation seeds-derivation in `src/autoc.cc:139-195` stops calling `rng::randLong()` and instead pulls from `scenarioSeedTable[k]`.

**Rationale**:
- Per spec Q3 init chain + survey finding F: today, NN evolution and scenario variation share the same global `rng::*` stream — interleaved consumption means the SEQUENCE of values either side sees depends on the OTHER. 033 splits them.
- Survey confirms `rng::*` is the single global PRNG used both by `src/nn/population.cc` (NN evolution) and by per-scenario `windSeed` generation (`src/autoc.cc:190`). Decoupling means: NN evolution's draws are independent of how many scenarios exist or how variation seeds are computed; variation seed table is independent of NN evolution's mutation/crossover order.
- Result: changing `SimNumPathsPerGeneration` or adding/removing a variation class doesn't shift NN evolution's draws — clean cross-config ablation.

**Alternatives considered**:
- **Replace `rng::*` global entirely with a thread-local injected PRNG**. Rejected as too invasive for 033 — keeps global helpers (back-compat) but seeds from a controlled source.
- **Make `rng::*` per-scenario-reseedable**. Rejected — would entangle NN evolution code with scenario boundaries; cleaner to keep `rng::*` as the autoc-side NN-evolution stream and put variation in its own machinery.

---

## R7 — CRRCSim wind PRNG: already takes a seed; just need to feed it the right one

**Decision**: No change to crrcsim internals. `SimStateHandler::reset(unsigned int windSeed)` at [crrcsim/src/SimStateHandler.cpp:246-256](../../crrcsim/src/SimStateHandler.cpp#L246-L256) already accepts a wind seed parameter and calls `CRRC_Random::reset(windSeed)` at line 249 BEFORE the standard reset that initializes the wind field. The only 033 change is: the AUTOC SIDE (`inputdev_autoc.cpp` + helpers) must pass the **wind sub-PRNG seed derived from scenarioSeed[K]** (per R3 class chain) instead of the existing `windSeed` field (which today comes from the joint single-stream).

Specifically: at scenario init on the worker, derive `windSubSeed = ClassPRNG(scenarioRoot.next()).asUint32()` (first draw from the wind class PRNG; or just pass the class PRNG seed itself), then pass it to `SimStateHandler::reset(windSubSeed)`.

**Rationale**:
- crrcsim already has the right machinery — only the SEED VALUE passed in changes. Zero crrcsim-side code change required.
- `CRRC_Random::reset()` (line 90 of `mod_misc/crrc_rand.h`) re-seeds the static state at scenario start; wind PRNG draws during the scenario then advance deterministically. Replays of the same scenario with the same seed produce identical wind tick-by-tick.
- The existing flow `SimStateHandler::reset(windSeed)` is called per-scenario from `inputdev_autoc.cpp` — that's where the seed source switches from "the joint-stream `ScenarioMetadata.windSeed`" to "the new wind sub-PRNG seed derived from `scenarioSeed[K]`".

**Alternatives considered**:
- **Plumb a separate windSeed slot through `EvalData`**. Unnecessary — the wind seed is fully derivable on the worker side from `scenarioSeed[K]` (already part of `ScenarioMetadata` after R2). Sending it redundantly would just bloat the wire.
- **Reseed CRRCSim wind mid-scenario**. No — CRRCSim's per-frame wind consumption needs continuity; reseed only at scenario boundary.

---

## R8 — Smoothness penalty pure-math placement: extend `include/autoc/eval/derived_features.h`

**Decision**: Add `compute_smoothness_factor()` as a pure-math inline helper in [include/autoc/eval/derived_features.h](../../include/autoc/eval/derived_features.h) alongside the existing `compute_pair_span` and `compute_tilt` from 032 phase 1. Signature:

```cpp
inline gp_scalar compute_smoothness_factor(gp_scalar dpt, gp_scalar drl, gp_scalar dth,
                                           gp_scalar floor,
                                           SmoothnessMotionMode mode);
```

Returns `[floor, 1.0]` based on Pythagorean (default) or sum/max aggregate of the Δ vector. `motion_max` is mode-dependent (Pythagorean: √12; sum: 6; max: 2).

**Rationale**:
- Single home for pure-math derivation helpers in the codebase ("derived features" is the right semantic bucket — smoothness is derived from per-tick NN-output Δ).
- Header-only inline for performance — called per-tick from stepper.
- Pure function = trivially testable in isolation (extend `tests/derived_features_tests.cc`).

**Alternatives considered**:
- **New `include/autoc/eval/smoothness.h` header**. Rejected — `derived_features.h` is already the established pattern; one more helper doesn't justify a new file.
- **Implement inline in `fitness_computer.cc`**. Rejected — couples pure-math to the fitness-computer surface; harder to unit-test the math in isolation.

---

## Phase 0 exit gate

All NEEDS CLARIFICATION items in plan.md Technical Context are resolved:
- Smoothness penalty insertion point: R1
- Scenario seed plumbing: R2
- Per-scenario class structure: R3
- Dmp schema add point: R4
- M1 baseline streak%: R5
- NN-evolution PRNG decoupling mechanism: R6
- CRRCSim wind PRNG site: R7
- Smoothness pure-math placement: R8

Phase 1 (data-model.md + contracts/ + quickstart.md) follows.

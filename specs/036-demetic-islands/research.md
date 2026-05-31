# Research: Demetic Islands for Basin-Diversity Preservation

**Status**: Mini-research / decision input (2026-05-30). Not yet a feature spec.
**Triggered by**: 034 test4 spiral-attractor lock confirmed at gen 139 (throttle pinned at 0.999, roll relaxed but rotation-rate stable). Reopens the question of whether maintaining multiple GA subpopulations could simultaneously hold the spiral basin **and** the rare non-spiral basin we've seen ~1/3 of the time.

## Motivation

Three related findings converge here:

1. **Basin lottery is intrinsic** ([project_032_phase1_setup](../../.claude/projects/-home-gmcnutt-autoc/memory/project_032_phase1_setup.md)): pop=8000/wind=36 gives ~2-of-3 climbers; the other ~1-of-3 stalls early. We currently treat this as "just rerun the bake" — but it's diversity destruction at the population level.
2. **Evolved-strategy is spiral-locked** ([project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md)): tight-spiral is objective-optimal under no-future inputs; once the population finds it, it crowds out alternative strategies. Confirmed live in 034 test4 — roll relaxed but rotation-rate (~210°/s path-0, ~230°/s path-5) is now flat from gen 84, controller is no longer exploring.
3. **Scalar-multi-objective collapses** ([project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)): when 033 floored smoothness into the scalar fitness, the population collapsed into a single Pareto corner. 035 takes lexicase as the answer — *islands* are an orthogonal mechanism that could complement it.

The unifying frustration is the same: **the GA can find a tight-spiral optimum, but it cannot simultaneously hold a "good non-spiral" candidate while doing so**. A single panmictic population is biased toward whichever attractor it found first.

## What demetic / island GAs actually are (classical)

A canonical island-model GA:

- Population of size `N` is partitioned into `D` subpopulations ("islands" / "demes") of size `N/D` each.
- Each island runs its **own** selection + variation cycle, independent of the others.
- Each island evaluates individuals on **the same fitness landscape** (same evaluation set / objective). The point is NOT scenario-level parallelism — it's *selection-pressure isolation*.
- Periodically (every `M` generations, or with per-individual probability `μ_mig`), some best-of-deme individuals migrate to a neighboring deme. Migration topologies vary: ring, fully-connected, random.
- Result: each island can occupy a different basin; migration provides cross-pollination without destroying local optima.

Key property: **islands trade convergence speed for diversity preservation**. A single basin found in one island doesn't immediately overwrite the rest of the population.

The "speedup" framing — each deme evaluates on a *subset* of scenarios to cut per-gen sim cost — is a different idea (sometimes called "cooperative coevolution" or "scenario sharding"). It's defensible as a speedup but loses the *diversity-preservation* property because individuals are now evaluated on different fitness functions and aren't directly comparable.

Our existing code is the latter. The user wants to move to the former.

## What we have today (audit)

Code surface is small post-GP-framework teardown:

| Site | What it does |
|---|---|
| [util/config.h:54-57](../../include/autoc/util/config.h#L54-L57) | Declares `demeticGrouping` (int flag), `demeSize` (default 1000), `demeticMigProbability` (default 0.0) — but the autoc.ini value is 20 (unscaled — looks like %, not used) |
| [util/config.h:199-201](../../include/autoc/util/config.h#L199-L201) | X-macro entries so the three knobs are parsed + printed at startup |
| [autoc.cc:417-437](../../src/autoc.cc#L417-L437) | `rebuildGenerationScenarios` demetic branch: produces one `ScenarioDescriptor` per path, each containing all wind variants. **One scenario = one path × N winds**. |
| [autoc.cc:1441](../../src/autoc.cc#L1441) | Training loop: `scenarioForIndex(ind % generationScenarios.size())`. Individual `ind` → scenario `ind mod nScenarios`. **Each individual sees ONE path × N winds.** This is the scenario-shard speedup, not classical islands. |
| [autoc.cc:577-587](../../src/autoc.cc#L577-L587) | `warnIfScenarioMismatch`: warns if `nScenarios < demeCount`. Cosmetic; doesn't change behavior. |
| `bakeoffSequence` on `ScenarioMetadata` | Logging-only field; surfaces in data.dat columns; no GA loop reads it |
| [eval/selection.cc](../../src/eval/selection.cc) | **Zero deme awareness.** Tournament / lexicase selection runs over the full population regardless of `demeticGrouping`. |
| `demeticMigProbability` | **Parsed but referenced nowhere.** Dead config. |

Status check across both inis: **both autoc.ini and autoc-tracker.ini ship `DemeticGrouping = 0`**. The feature is off in production; it's been off since at least the 028 codebase. What remains is leftover-from-GP plumbing.

## Gap vs. classical islands

| Property | Classical islands | What we have | Gap |
|---|---|---|---|
| Population partitioned | ✓ N/D per island | ✗ (single pop) | **Missing** |
| Same fitness landscape per island | ✓ | ✗ (each individual sees a different scenario subset) | **Wrong shape** — what we have is sharding, not islands |
| Selection within island | ✓ | ✗ (global tournament) | **Missing** |
| Migration tick | ✓ ring/random copy of best | ✗ (config parsed but unused) | **Missing** |

So porting to classical islands is **net additions**, not subtractions. The current dead code (`demeticGrouping=1` branch in `rebuildGenerationScenarios`, `DemeticMigProbability` parsing) is in the way and should be removed or repurposed.

## Proposed minimal port (sketch)

Just to size the work — not a spec yet:

1. **Repurpose / rename** the three config knobs:
   - `IslandCount` (int, default 1 = panmictic = today's behavior)
   - `MigrationInterval` (int, gens between migration ticks, default 0 = off)
   - `MigrationFraction` (double, fraction of each island that migrates per tick, e.g. 0.05 = 5%)
2. **Delete** the scenario-shard `demeticGrouping` branch in `rebuildGenerationScenarios`. **All islands evaluate on the full path × wind matrix** (currently 6 × 36 = 216 per individual). This is the key spirit-fix.
3. **Partition population** into `IslandCount` contiguous chunks of size `popSize / IslandCount`. Selection/crossover/mutation in [selection.cc](../../src/eval/selection.cc) gets an island index and operates within bounds.
4. **Migration tick**: every `MigrationInterval` gens, copy `MigrationFraction × islandSize` best individuals from each island to a neighbor (ring topology to keep it simple). Replace the worst in the destination.
5. **Determinism**: migration is PRNG-driven; uses a class-seed cascade from the master seed (consistent with [project_variation_design_principles](../../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md)).
6. **Telemetry**: per-island best/avg/worst in the `#NNGen` line (extend the schema with `island0_best=…` etc., or emit a per-island companion line `#IslandStats island=K best=…`).

Estimated effort: ~1–1.5 days of focused work post-035. Selection refactor is the meaty part; the rest is plumbing.

## Expected effects (predictions to test)

Tied to currently-open project memories so the bake outcomes are interpretable:

- **Basin diversity preserved**: with IslandCount=8 and a stuck-basin rate of ~33%, naive expectation is at least one island finds a non-stuck climber on most bakes. Migration would then propagate that signal across the population. If true, the per-bake basin-lottery rate ([project_032_phase1_setup](../../.claude/projects/-home-gmcnutt-autoc/memory/project_032_phase1_setup.md)) should drop materially.
- **Spiral lock weakened**: with 8 islands, the population can sustain at least one *non-spiral* island, even if the others spiral. Whether migration eventually overwrites that island depends on selection pressure differential — would need to measure. ([project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-axc/memory/project_evolved_strategy_vs_airframe.md))
- **Convergence is slower per gen** by a factor roughly ~`IslandCount` (each island has fewer effective mating opportunities). This is the classical islands trade. Compensated by reduced rerun rate.
- **Per-gen cost is unchanged**: each individual still does 216 sims. Compute scales with `popSize × scenarios`, not islands.

## Risks & costs

- **Convergence slow-down** could push wall-clock per usable bake up — need to test on the 034 baseline.
- **Migration replaces the worst** — if all islands converge to the same basin, migration is a no-op AND we've spent compute on the slower convergence. Worst case: islands without migration are weakly worse than panmictic; islands with too-aggressive migration are panmictic with extra steps.
- **Tuning surface grows**: 3 new knobs (IslandCount, MigrationInterval, MigrationFraction) need ablation. Cheapest first attack would be IslandCount=8, MigrationInterval=40 (one ramp step), MigrationFraction=0.05.
- **Determinism / seed cascade** needs careful design to keep [reference_perf_build_reproducibility](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_perf_build_reproducibility.md) intact. The master-seed → per-island sub-seed split needs the same care as `deriveClassSubSeeds`.
- **Interaction with lexicase (035)**: islands + lexicase is well-studied (Spector et al.) — they compose. But the eps-MAD work in 035 needs to land first; combining a new selection method with a new population structure in the same bake would be hard to attribute.

## Where this fits in the roadmap

**Queued order (decided 2026-05-30): 034 → 035 → 036.** Demetic islands lands *after* 035 regardless of 035 outcome.

Reasoning:

- **034 craft variations are static-per-scenario** — one CG/drag/trim/thrust/pitch-eff/roll-eff draw applied at scenario init, held for the trajectory. That's a *passive challenge*, not a search-pressure change. So variations are **not expected to fix**:
  - The throttle bang-bang attractor — a heavier or draggier airframe still spirals fine at full throttle; saturation isn't punished by the variation.
  - The basin lottery — the spiral basin is still the geometric optimum under variations; no reason the no-spiral basin gets a relative lift. Expect ~20–30% stuck-bake rate to persist.
- **035 attacks the scalar-collapse issue at the selection-rule level** ([project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)) — most-causal intervention on the multi-objective shape of the problem.
- **036 islands attack the population-structure level** — orthogonal to 035's selection-rule change. Whether or not 035 finds an energy-axis win, the basin-diversity problem and bang-bang lock are likely still live, and islands are the next tool that addresses them at the right level.

So this is **back-pocket research now**, but **not contingent on 035** — it's queued. Picked up after 035's outcome doc, with the live 034/035 bake telemetry informing the predictions in the section above.

## Open questions

For future /clarify:

1. **Topology**: ring vs. random-pair vs. fully-connected migration. Ring is simplest and likely sufficient.
2. **Migration size**: top-N copy vs. tournament-emit. Top-N is simpler and well-studied.
3. **Receiver replacement**: worst-N replace vs. random replace. Worst-N is more aggressive (faster propagation) but loses minority strategies faster.
4. **Island heterogeneity**: should islands run *different* selection rules (one tournament, one lexicase, one minimax)? Big design space; defer.
5. **Per-island scenario rotation**: do islands eventually see *different* path/wind subsets to encourage specialization? This brings back the scenario-sharding idea in a controlled way; defer to "v2".
6. **Interaction with the dmp / S3 trace**: each island's per-gen best is currently the only thing written to dmp. With islands we'd want per-island bests in the dmp. Adds a new dimension to the trace.

## Related memories

- [project_basin_lottery](../../.claude/projects/-home-gmcnutt-autoc/memory/project_032_phase1_setup.md) — 2-of-3 climbers at pop=8000/wind=36
- [project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md) — spiral is objective-optimal under no-future
- [project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md) — single-fitness aggregation collapses to one Pareto corner
- [project_variation_design_principles](../../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md) — determinism via class-PRNG cascade (must extend for island migration)

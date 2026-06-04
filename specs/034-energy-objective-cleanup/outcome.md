# 034 Outcome — M1/M2 Cleanup + Craft Variations (+ emergent: variation clamps, M1 validation)

**Date:** 2026-06-03
**Status:** Implementation complete; all tasks.md items closed. M1 validated against the pastonly3 gold standard; M2 run is the remaining operator step (config staged).

## What 034 delivered (planned scope)

- **US1 — minisim retired.** `tools/minisim.cc` + `PathgenStepper` deleted; crrcsim is the sole worker path; config `Minisim*`→`Worker*` (no back-compat alias).
- **US2 — smoothness removed.** `SmoothnessMotionMode` / `compute_smoothness_factor` / all mirrors stripped; schema cleaned (cereal, no version bump). Behavior-neutral (smoothness was already bypassed).
- **US3 — tech-debt fold-ins.** Config X-macro single-source (auto-print every key), right-sized seed cascade, per-(path,wind) variation table (FR-012), `tracker-`/`autoc-` run-id prefix (+ test), lighter eval return (deferred).
- **US4 — craft variations.** Per-scenario airframe diversity (CG/drag/trim/thrust/pitch-eff/roll-eff + craftSeed) through `ScenarioMetadata`, ramped via shared `applyVariationScale`, deterministic, no-op at σ=0, eval-replay exact.
- **US5 — craft-variation signal confirmed.** See [craft-variation-signal-report.md](craft-variation-signal-report.md): cross-generation sign test shows craft draws move the controller in the physically-correct direction (thrSc→speed 60/60, thrSc→throttle-backoff 55/60, rolEff→roll-rate, pitEff→less-saturation) — overwhelming significance. The closure criterion for 034.
- **data.stc retired (T052–T055).** Per-gen markers moved to the run `.log`; single telemetry source; plot scripts read the `.log` directly.

## Emergent work this session (beyond original plan)

### Variation clamps (arena12 → truncated-normal)
An entry-cone tail draw put scenario "arena12" at a 106° (backward-facing) entry — an unrecoverable degenerate outlier. Root cause: every variation draw was an **unbounded** Gaussian/half-normal. Fix: **truncate the Gaussian primitive at ±2.5σ** (`kGaussianSigmaClamp` in `scenario_prng.h`, applied in `ClassPRNG::nextGaussian` + the LCG lambdas), plus an absolute 80° forward-cone guard. Clamp (not resample) → PRNG draw-count fixed → determinism preserved. Range stays proportional to each knob's σ; only the crazy tail is cut. Two invariant tests added. **Replay note:** clamps changed tail draws, so post-clamp runs do not bit-reproduce pre-clamp seeds (greenfield, no cereal bump).

### M1 learner + objective validated; population-size finding
A diagnostic sweep (basic-m1) and a full run resolved the "did we break the objective / is the learner stuck?" question definitively — **no**:
- **basic-m1** (pop 3000, single longSequential, 16 winds): the learner climbs fast and reliably on the *current* objective; variations *help* (steeper climb). Objective not broken. See [project_m1_basic_learner_validated](../../.claude/projects/-home-gmcnutt-autoc/memory/project_m1_basic_learner_validated.md).
- **Population-size finding** ([findings-popsize-variations.md](findings-popsize-variations.md)): the **8000/216** run plateaued in the throttle-peg attractor (throttle pegged 1.0, streak flat ~2.4); the **5000/294** run tracked the gold standard with throttle *modulating*. Population is a search-area knob — 8000 over-converges, 5000 holds diversity. "Feels like tuning," not a principled fix (energy/035 is the real lever).

## Headline M1 result — origm1-5000×49 ≈ pastonly3

The production-shape run (pop 5000, 6 paths × 49 winds = 294 scenarios, all 4 variations + clamps, lexicase; run-id `autoc-9223370256441628515-2026-06-02T15:12:27.292Z`, master seed `1780413146`) **converged to pastonly3-class on every axis** and was stopped at gen 590 (sigma at the 0.05 floor — converged):

| metric | origm1 gen 590 | 029 pastonly3 (gen 800) |
|---|---|---|
| best fitness | **−55,269.63** | −55,944.66 |
| avgMaxStreak | **37.9** | 37.7 |
| pctInStreak | 41.5% | 39.7% |
| course completion | **294/294, 0 crash** | 294/294 |
| sigma | 0.0508 (floor) | 0.0500 (floor) |

It tracked pastonly3 curve-for-curve the whole way (interweaving within a few %), reached pastonly3-class streak ~50–100 gens *ahead of schedule*, and reproduced pastonly3's documented **back-half smoothing** — roll-rate (`dctrl`) roughly halved from gen 171→470 and all axes pulled toward the amplitude budget, with **throttle modulating** (not pegged). See `034-origm1-vs-pastonly3_evolution_progress.png` + `034-origm1-5000x49_per_axis_*.png`.

This both (a) confirms the objective + learner are healthy at the original-M1 operating point, and (b) produces a genuinely strong M1 controller — the source for M2.

## Pipeline validation (eval-parity, bit-exact)

train → S3 dmp → nnextractor → eval reproduces fitness to the last digit, twice:
- gen 475: `#NNEval -51913.339637` == dmp stored == training.
- gen 590: `#NNEval -55269.634183` == dmp `gen9410.dmp` stored == training `#NNGen`.

(dmp naming confirmed `gen<10000−N>.dmp`.) FP-deterministic replay holds on the current clamped binary.

## M2 readiness

- **`autoc-tracker.ini`** staged: matches `autoc.ini` (pop 5000, 6×49, all variations, ramp 40), `TrackerSourceRun = …/gen9410.dmp` (gen 590).
- **PRNG audit clean** (see prior session note): all M2 variation PRNGs derive from the master seed via `deriveClassSubSeeds(meta.scenarioSeed)` (scenarioSeed = `gMasterPRNG.next()`); the M1 source dmp contributes trajectory only, no recorded seed; crash-hull is rabbit-class-seeded (v1.5 fix). M2 is reproducible from its own `Seed=N`.

## Status of tasks

All `tasks.md` items closed (T001–T055); T035/T036 explicitly deferred to backlog (fitness-to-worker refactor, not a gate). `autoc-eval.ini` / `autoc-tracker.ini` edits held uncommitted pending M2 start. The crrcsim `windSeed` debug-log fix is committed (source) but unbuilt — lands at the next rebuild.

## Next

1. **M2 run** against `gen9410.dmp` (operator).
2. **Routing decision:** fold the smoothness rework into 035, or keep it standalone? (This run self-smoothed under a pure tracking objective — argues for going 034 → 035 directly.)
3. **035 energy-as-lexicase** — design discussion captured in `specs/035-energy-lexicase-objective/spec.md` (measure total energy input: non-linear throttle + induced drag, lexicase not scalar, energy-not-smoothness; M2 is high-energy → energy objective matters most there).

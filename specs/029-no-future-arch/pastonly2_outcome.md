# 029 US1 outcome — past-only NN sensor input baseline (more-rnn4-pastonly2)

**Run name**: `more-rnn4-pastonly2`
**Branch**: `029-no-future-arch` (working branch)
**Spec**: [spec.md](./spec.md), [plan.md](./plan.md)
**Run period**: 2026-04-30 22:58 → 2026-05-02 (gens 1-800, ~38h)
**Final commit**: 7427aa1 (pivot doc commit; underlying code change ccbd837)

## Summary verdict

**029 US1 → PASS on training-time fitness; PASS on on-distribution eval; FAIL (brittle) on out-of-distribution eval tiers.**

The recurrent NN architecture *can* train effectively from past-only inputs — but the resulting controller is more brittle than the with-future-input baseline (more-rnn3). The architectural assumption is validated; the *strategy* the architecture evolves under no-future inputs is a tighter-spiral, less-margin controller that works on its training envelope but loses ground when pushed out-of-distribution.

This advances the path to 030 tracker mode but tempers it: tracker-mode-class controllers will need extra robustness work (or matched airframe), not just architectural validation.

## Final state

- **Training-time best fitness** (gen 800): **−52,188.58**
- **Fixed-eval tier0-repro fitness**: −52,188.58 (matches — determinism preserved bitwise)
- **more-rnn3 final** (comparator): −50,422.88
- **pastonly2 vs more-rnn3 on training-time fitness**: ~3.5 % deeper (better)

Late-run telemetry (gen 800):
- bestSigma = 0.0505 (at NNSigmaFloor)
- avgMaxStreak = 41.2
- pctInStreak = 49.1 %
- whh_xh_ratio = 0.56 (recurrent block engaged through to end)
- Crash rate: 1/245 (0.4 %) at training-time gen 800

## SC-001 evaluation (within ±10 % of more-rnn3 fitness)

| Comparison | more-rnn3 | pastonly2 | Δ | SC-001 |
|---|---|---|---|---|
| Training-time best (gen 800) | −50,423 | −52,189 | +3.5 % deeper | **PASS** |
| Fixed-eval tier0-repro | −50,423 | −52,189 | +3.5 % deeper | **PASS** |

PASS by significant margin on the load-bearing fitness comparison.

## Eval suite — per-tier (load-bearing finding)

[`eval-results/2026-05-02T17:23:29Z/`](../../eval-results/2026-05-02T17:23:29Z/) vs more-rnn3 prior at [`eval-results/2026-04-30T04:44:11Z/`](../../eval-results/2026-04-30T04:44:11Z/):

| Tier | more-rnn3 | pastonly2 | Δ OK | Δ score | Tier verdict |
|---|---|---|---|---|---|
| tier0-repro (245, exact seed) | 245/245 ✓ score 205.8 | 244/245 ✓ score 213.0 | −1 | +3.5 % | ≈ equal |
| **tier1-aeroStandard (245, novel seed)** | **245/245 ✓ 213.6** | **230/245 ✗ 182.4** | **−15** | **−14.6 %** | pastonly2 brittle |
| tier2-progressive (49) | 48/49 ✓ 853.9 | 47/49 ✓ 838.2 | −1 | −1.8 % | ≈ equal |
| tier2-long (49) | 49/49 ✓ 207.1 | 49/49 ✓ 220.1 | 0 | +6.3 % | ≈ equal (slight pastonly2 win) |
| **tier2-random (144, 12 paths × 12 winds)** | **144/144 ✓ 216.5** | **130/144 ✗ 197.5** | **−14** | **−8.8 %** | pastonly2 brittle |
| **tier3-stress (144, 120 % sigmas)** | **143/144 ✓ 213.2** | **133/144 ✗ 190.6** | **−10** | **−10.6 %** | pastonly2 brittle |
| tier3-quiet (1, no variation) | 1/1 ✓ 316.6 | 1/1 ✓ 275.4 | 0 | −13.0 % | pastonly2 lower score, both pass |

**Bottom line**: pastonly2 fails 3 of 7 tiers (tier1, tier2-random, tier3-stress), all probing out-of-distribution scenarios. more-rnn3 passed all 7. On-distribution tiers are equivalent; off-distribution pastonly2 loses 10-15 scenarios per tier and ~10 % score.

This is the **brittleness signal** — pastonly2 is fitness-equivalent on its training distribution but has less generalization capacity than more-rnn3.

### Why brittle?

Two contributing factors:

**(1) Strategy-level brittleness** (per [project_evolved_strategy_vs_airframe.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md)): the past-only architecture evolved a **tight-spiral patrol** strategy — high-roll-rate, sustained-stick orbital coverage to substitute for prediction. The strategy is fitness-optimal on the training envelope but commits hard to the spiral pattern with little reserve capacity. When tier1 / tier2-random / tier3-stress push wind sigmas / paths / entry conditions outside the training envelope, the spiral pattern can't adapt.

**(2) Training distribution gap — path-5 (random aerostandard) was excluded from training** (operator finding 2026-05-02): autoc.ini had `SimNumPathsPerGeneration = 5         # skip the random path` for pastonly2. Path 5 is the random-aerostandard variant — it's the OOD generator that tier1's "novel seed" + tier2-random's "12 random paths" sample from. So pastonly2 *never saw random-aerostandard geometries during training*, which directly explains a chunk of the tier1 / tier2-random brittleness. The exclusion was originally rationalized as complexity reduction back when entry-position variability was *also* enabled (`EntryPositionRadiusSigma`, `EntryPositionAltSigma`); both have since been disabled (currently 0.0 in autoc.ini), so the original justification no longer holds. Path 5 has been re-enabled (`SimNumPathsPerGeneration = 6`) for the pastonly3 iteration.

more-rnn3 (with-future inputs) didn't need to spiral because it had path-lookahead. Its smoother path-following carried more margin and generalized better — it likely also benefited less from path-5 exclusion because its strategy generalizes more easily across path geometries anyway.

## Per-axis aggressiveness — late run (last 50 gens)

| Axis | dCtrl mean | over-budget (0.27) | \|out\| mean | over-budget (0.67) |
|---|---|---|---|---|
| pitch | 0.371 | 235/245 (96 %) | 0.679 | 144/245 (59 %) |
| roll | **0.424** | 245/245 (100 %) | 0.617 | 20/245 (8 %) |
| throttle | 0.376 | 215/245 (88 %) | 0.774 | 243/245 (99 %) |

Multi-axis modulation; no single-axis bang-bang. Roll \|out\| 0.617 firmly under the 0.67 budget. The shift from "roll-only bang-bang" (gen 115) → "pitch+roll bang-bang" (gen 314) → "roll+throttle bang-bang" (gen 369) → multi-axis modulation (gen 400+) was the strategy maturation arc.

## Per-path airframe rotation totals (degrees, last 50 gens mean)

| Path | Total \|Δroll\|° | Total \|Δpitch\|° | Optimal-ish ratio |
|---|---|---|---|
| 0 | 4,273° | 983° | ~12× operator estimate |
| 1 | 5,791° | 1,151° | ~64× |
| 2 | 5,608° | 1,468° | ~16× |
| 3 | 2,070° | 578° | ~46× |
| 4 | 7,181° | 1,739° | — |

Spiral-tracker signature confirmed across all 5 paths. Path-3 ratio (~46×) and path-1 ratio (~64×) are the strongest tight-spiral evidence.

The "stick smoother + airframe rotation faster" pattern (sustained-stick spiral, not bang-bang spiral) was load-bearing — see [project_evolved_strategy_vs_airframe.md §strengthening signal](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md). Late-run dCtrl trended *down* while total \|Δroll\|° trended *up*.

## Curve shape

Slow-start / mid-run-acceleration / late-plateau pattern, opposite of with-future runs' classic long tail:

- gen 0-200: slow descent, fitness plateau-ish (~−12k → ~−18k)
- gen 200-400: acceleration begins as predictor evolves inside hidden state
- gen 400-550: steepest descent (−25k → −51k in 150 gens)
- gen 550-800: late plateau at sigma-floor (~−51k to −53k, with variation-ramp ceiling pushing back to −52k by gen 800)

Per [project_no_future_curve_shape.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_no_future_curve_shape.md): shape itself is the diagnostic signal that the architecture is evolving an internal predictor.

## Architectural decision (operator, in-flight)

[project_evolved_strategy_vs_airframe.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md): **continue with current fixed-wing airframe through M1.3 and M2 (real-flight-with-beacons)**, accept the spiral-tracker strategy. Don't fire 029 T061-T065 prediction-aiding architecture experiments. X-wing tail-sitter platform spec is backlog. Real-flight outcome will trigger either prediction-aiding arch OR X-wing platform if needed.

## Implications for next milestones

- **M1.3 (Flight test of pastonly)** — controller is ready: weights at `gen_800.dat`, builds clean from current `029-no-future-arch` branch. Operator already has eval-suite output for cross-reference. **Watch for**: brittleness manifest in real-flight wind/turbulence (more variation than training envelope captures); the spiral-tracker strategy may degrade more rapidly than more-rnn3 did under real conditions.
- **025 craft variations (next sim feature)** — the brittleness finding *raises the value* of 025. Broadening the joint-PRNG sample to include craft parameters should produce a less-brittle controller — robustness comes from variation diversity at training time. The eval-suite tier1/tier2-random/tier3-stress failures are exactly what 025 is designed to address. **Hypothesis to verify**: a 025-trained controller passes more eval tiers than pastonly2 does, even if its peak fitness is similar.
- **030 (tracker mode)** — controller architecture is validated. The brittleness finding informs the camera/perception design: noisy or partially-occluded beacon inputs may exacerbate the spiral-tracker's brittleness, reinforcing the case for variation in camera config (per FR-003f minimal-calibration property).

## Outputs

- `data.dat` (final, ~14 GB) — gens 1-800 per-tick training records
- `data.stc` (final) — gens 1-800 per-gen telemetry (`#NNGen` lines)
- `logs/autoc-029-pastonly2.log` — full training log (worker output included)
- `eval-results/2026-05-02T17:23:29Z/` — eval-suite tiers + weights
- `specs/029-no-future-arch/pastonly2_evolution.png` — final 6-panel evolution + crash-rate
- `specs/029-no-future-arch/pastonly2_per_axis.png` — final 4-panel per-axis (mean dCtrl, mean \|out\|, per-path \|Δroll\|°, per-path \|Δpitch\|°)
- `specs/029-no-future-arch/pastonly2_aggressiveness.png` — gen-800 per-axis distribution histograms

## Next iteration — pastonly3 (path-5 included)

Driven by the path-5 finding above: re-run pastonly with all 6 paths in the joint-PRNG sample to test the hypothesis that *training distribution gap* (not just architectural strategy) was responsible for a chunk of pastonly2's tier1/tier2-random brittleness.

**Configuration changes for pastonly3** (vs pastonly2):
- `SimNumPathsPerGeneration` = 5 → **6** ([autoc.ini:35](../../autoc.ini#L35)) — random-aerostandard now in training
- All other knobs identical (Path A: pop 5000 × 800 gens, recurrent NN, single seed)
- Per-gen scenario count: 245 → **294** (6 paths × 49 winds) — ~20 % more eval cost per gen, ~46h calendar instead of ~38h

**Hypothesis to verify**:
- pastonly3 tier1 / tier2-random pass rates *significantly improve* over pastonly2 (closing the gap to more-rnn3's 100 %)
- pastonly3 tier3-stress likely improves modestly (random-path coverage helps, but not all of tier3's stress is path-distribution)
- Training-time fitness comparable to pastonly2 (the harder per-gen eval may shift the fitness *number*, but per-axis + per-path signatures should track)

If pastonly3 tier1/tier2-random both PASS, the path-5 exclusion was a meaningful contributor to the brittleness — strengthens the case for tighter alignment between training-envelope and eval-tier sample distributions in future runs (precursor to 025 craft variations).

If pastonly3 still shows tier1/tier2-random brittleness, the residual is genuine *architectural* brittleness (spiral-tracker reserve issue) — that's the part 025 craft variations must address.

## Branch decision

**029 US1 → PASS** (with brittleness caveat documented + path-5 follow-up planned as pastonly3).

Sequencing options:
- **(a) Run pastonly3 in current `029-no-future-arch` branch** before M1.3 flight test — gets a less-brittle controller into the air, possibly with comparable or better behavior in real-flight wind/turbulence
- **(b) Flight-test pastonly2 *and* pastonly3 in parallel** — pastonly3 trains on the main repo (~46h) while M1.3 prep proceeds for pastonly2; flight test whichever ends up ready first, or both back-to-back
- **(c) Skip pastonly3 and go straight to M1.3 + 025** — accept pastonly2 as the no-future baseline and let 025 craft variations subsume the brittleness fix

Operator currently leaning (b): run pastonly3 next on the main repo while staying open to flight-testing pastonly2. After pastonly3 completes, flight-test the better controller (or both).

Then per [staged path](../../.claude/projects/-home-gmcnutt-autoc/memory/project_perception_control_two_loop.md):

1. **M1.3 — Flight test** of pastonly2 and/or pastonly3 on current fixed-wing
2. **025 — Craft variations** (sim training, joint-PRNG includes craft params)
3. M3.5 / 025b — control analysis + flight test of 025 controller
4. 030 — sim-beacons learnable (controller-side training, no hardware)
5. (...continued per staged path)

---

## Appendix A — Eval suite raw output (2026-05-02T17:23:29Z)

Source: `eval-results/2026-05-02T17:23:29Z/summary.txt`. NN extracted via `build/nnextractor -o nn_weights.dat` from S3 dump `autoc-9223370259246861370-2026-05-01T03:58:34.437Z/gen9200.dmp`. Genome: 33→32→16→3 topology, 1923 weights, fitness −52188.577427, sigma 0.050482.

```
Test                      Pass  OK/Total   Completion    Score   Strk  Mult
------------------------------------------------------------------------------------
tier0-repro               PASS  244/245 ( 99.5%)  score=213.0  strk=41.2  mult=4.30  fitness=-52188.577427
tier1-aeroStandard        FAIL  230/245 ( 93.8%)  score=182.4  strk=37.3  mult=3.99
tier2-progressive         PASS   47/ 49 ( 95.9%)  score=838.2  strk=49.7  mult=4.98
tier2-long                PASS   49/ 49 (100.0%)  score=220.1  strk=46.7  mult=4.73
tier2-random              FAIL  130/144 ( 90.2%)  score=197.5  strk=30.1  mult=3.41
tier3-stress              FAIL  133/144 ( 92.3%)  score=190.6  strk=29.5  mult=3.36
tier3-quiet               PASS    1/  1 (100.0%)  score=275.4  strk=50.0  mult=5.00
```

Renderer commands for inspection per tier (from raw `summary.txt`):

```bash
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier0-repro.ini -k autoc-9223370259112165870-2026-05-02T17:23:29.937Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier1-aero.ini  -k autoc-9223370259112163249-2026-05-02T17:23:32.558Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier2-prog.ini  -k autoc-9223370259112155723-2026-05-02T17:23:40.084Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier2-long.ini  -k autoc-9223370259112153954-2026-05-02T17:23:41.853Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier2-random.ini -k autoc-9223370259112152900-2026-05-02T17:23:42.907Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier3-stress.ini -k autoc-9223370259112149743-2026-05-02T17:23:46.064Z/
build/renderer -i eval-results/2026-05-02T17:23:29Z/tier3-quiet.ini  -k autoc-9223370259112147001-2026-05-02T17:23:48.806Z/
```

## Appendix B — Operator note: tier0-repro seed pinning between runs

The first eval-suite invocation (`2026-05-02T17:22:52Z`, aborted via Ctrl-C before completing) reported tier0-repro **FAIL** with `eval_fitness=-48579.69` vs `stored_fitness=-52188.58` (MISMATCH). The second invocation (`2026-05-02T17:23:29Z`, completed cleanly) reported tier0-repro **PASS** with EXACT bitwise match.

This was **not** a determinism bug — it was an operator configuration step. Tier0-repro requires the eval `Seed` pinned to the original training run's actual seed (pastonly2's resolved seed = `1777588111`, per the training launch log). The first invocation ran without that pin updated, so the eval rolled a different `Seed` and naturally produced a different fitness number. After updating the tier0-repro config with pastonly2's seed, the second invocation reproduced exactly as expected. Standard pre-eval hygiene step.

Of secondary interest: between the two invocations, **tier1-aeroStandard** flipped PASS (run 1, score 193.7) → FAIL (run 2, score 182.4) — that's normal seed-driven variance for tier1's "novel seed" envelope (each invocation rolls a fresh tier1 seed by design). The tier1 PASS/FAIL threshold is borderline for pastonly2 against this seed distribution; pastonly3 should clear it more decisively if the path-5 hypothesis holds.
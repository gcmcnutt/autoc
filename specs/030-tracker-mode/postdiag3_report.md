# postdiag3 Training Report — T-102 32r topology experiment

**Run id**: `autoc-030-postdiag3-crrcsim-hullon-pop5000-32r`
**Started**: 2026-05-13
**Stopped**: 2026-05-15 20:23 (operator SIGINT at gen 544 of planned 800 — outcome already clear by gen 500)
**Source dmp**: pastonly3 gen9200 (identical to postdiag2)
**Topology**: 45 → 32 → **32r** → 3 (3,651 weights — 4× recurrent W_hh capacity vs 16r)
**Config**: identical to postdiag2 (pop=5000, gens=800, CrashHullProbability=0.10, all variations live, VariationRampStep=40)

## TL;DR

**Recurrent state capacity is NOT the binding constraint on 030 v1.** Doubling the recurrent layer (16r → 32r, +768 W_hh weights) did not lift the fitness ceiling — at every comparable gen through 540, the 32r run is **700-1500 points worse** than the 16r baseline. Determinism contract holds (540/540 NN_ELITE_SAME, 0 DIVERGED).

**Decision: drop back to 16r as v1 baseline.** 032 starts from postdiag2's 16r topology, not 32r. The 32r weights have no operational reason to ship — they cost more per-tick compute + more xiao flash for zero fitness gain.

## Comparison vs postdiag2 (16r baseline)

| metric | postdiag2 (16r) | postdiag3 (32r) | delta |
|---|---|---|---|
| weights | 2,307 | 3,651 | +58% |
| recurrent W_hh | 256 | 1,024 | +300% |
| best fitness | **-17,060** (gen 517) | -16,382 (gen 519) | **+678 (worse)** |
| determinism (DIVERGED count) | 0/542 | 0/540 | matched |

Same-gen comparison (sampled):

| gen | 16r elite | 32r elite | 32r gap |
|---|---|---|---|
| 287 | -16,429 | -15,107 | +1,322 worse |
| 326 | (post-ramp) | -15,298 | ~1100 worse |
| 379 | (post-ramp) | -15,792 | ~1100 worse |
| 403 | -16,650 | -15,650 | +1,000 worse |
| 517 | **-17,060** (peak) | -16,156 | +904 worse |
| 540 | -16,853 | -16,226 | +627 worse |

The gap has narrowed modestly as 32r kept climbing past gen 500, but never crossed 16r's peak. Best 32r-baseline trajectories below 16r's at every variation-ramp boundary.

## What was confirmed

1. **State capacity is not binding.** 4× recurrent capacity → zero fitness lift. M2 plateau hypothesis #1 (from M11.preA.5) **falsified**.
2. **Determinism contract robust at 32r weight count.** No NN_ELITE_DIVERGED events across 540 gens of larger weight vector. Joint-PRNG / windSeed fix from M11.preA.3 generalizes.

## Operator qualitative observations (playback at gen 522)

Tracking flights are **visibly good** — chase intercepts, follows, holds trail briefly. But two patterns persist regardless of generation:

1. **Overruns persist.** Chase passes through the cone and shoots past the target. The leading hypothesis: chase has no absolute-distance signal — only apparent point separation between the 4 NDC beacons. With known target wingspan/pyramid geometry, **angular separation is a direct range proxy**, but the NN has to learn that mapping implicitly from 4 NDC pairs across 6 ticks of history. This is exactly what 032 phase 1's `span` + `span-rate` features address explicitly.
2. **Per-scenario chaos shuffles, doesn't refine.** When best fitness ticks up by ~100 points across a few gens, individual-scenario behavior changes dramatically — the overruns happen in different places, the chaos pattern reshuffles — but the *amount* of chaos is roughly conserved. This is the classic signature of GA stuck in local optima: the gradient surface is too ambiguous to reward incremental tightening, so selection trades one failure mode for another rather than refining toward smoother control.

Both observations point at the **same root cause: the NN input vector doesn't carry enough disambiguating signal** to give the GA a smooth gradient. Adding state capacity widens the function class the NN can represent, but doesn't add information — the perception bottleneck is upstream of architecture.

## Routing into 032

032 phase 1 (locked feature set): **beacon-pair span[6] + span-rate + tilt** as derived NN inputs.

Why this fits the postdiag3 observations:

- **Span attacks overrun directly**: span IS apparent angular extent — the missing distance proxy. Closing rate of span (span-rate) is the closing-rate proxy. Chase gets explicit "I'm closing fast" / "I'm at proper trail spacing" signals instead of having to infer them from beacon NDC differences.
- **Span/span-rate breaks the local-optima trap**: a clean "closer / farther" gradient gives the GA something concrete to climb instead of the current "find a different bad spot" reshuffle. Even if span doesn't fully resolve front-back pose ambiguity (separately deferred to M3), it should smooth the fitness landscape enough that selection can refine within a strategy rather than thrashing between strategies.

If 032 phase 1 lifts the ceiling meaningfully → visibility-time signal richness was the binding constraint, ship. If ceiling stays flat → FOV blindness or pose ambiguity earn their place; design phase 2 then.

## Status

- **Run state**: operator-stopped via SIGINT at gen 544 of planned 800 (2026-05-15 20:23). Final elite -16,226; best ever -16,382 at gen 519. Determinism contract held end-to-end (544/544 NN_ELITE_SAME, 0 DIVERGED across all variation-ramp events at the larger weight count).
- **Reports generated** (gen 544 final): [evolution_progress](autoc-030-postdiag3-crrcsim-hullon-pop5000-32r_evolution_progress.png), [gen_diag](autoc-030-postdiag3-crrcsim-hullon-pop5000-32r_gen_diag.png), [per_axis_time_series](autoc-030-postdiag3-crrcsim-hullon-pop5000-32r_per_axis_time_series.png), [per_axis_aggressiveness](autoc-030-postdiag3-crrcsim-hullon-pop5000-32r_per_axis_aggressiveness.png).
- **v1 baseline**: postdiag2 (16r) remains the keeper. 32r weights have no operational reason to ship — slower per-tick on xiao + bigger flash footprint for negative fitness lift.
- **Routing**: revert `TRACKER_NN_HIDDEN2_SIZE` to 16 before 032 phase 1 starts. 032 layers `span[6] + span-rate + tilt(sin,cos)` onto the 16r topology — see [specs/032-tracker-nn-enhancements/spec.md](../032-tracker-nn-enhancements/spec.md).

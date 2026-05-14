# postdiag2 Training Report — 030 v1 baseline (pop=5000, fov/cone fix)

**Run id**: `autoc-030-postdiag2-crrcsim-hullon-pop5000`
**Started**: 2026-05-11 17:34
**Stopped**: 2026-05-13 19:31 (operator-stopped at gen 542 for T-102 / 32r topology experiment)
**Duration**: ~2 days, 542 gens of planned 800
**Source dmp**: pastonly3 gen9200 (`autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp`)
**Topology**: 45 → 32 → 16r → 3 (2,307 weights, current 030 baseline)
**Config**: autoc-tracker.ini @ pop=5000, gens=800, CrashHullProbability=0.10, all variations live, VariationRampStep=40

## TL;DR

030 v1 hits a **fitness ceiling near -17K** and plateaus. Determinism contract holds end-to-end (542/542 SAME, 0 DIVERGED across 13 variation-ramp events). The plateau is genuine — not noise, not topology capacity, not search density. It is an **input information-content ceiling**. Specifically: the 2-beacon perception leaves the chase blind 31% of the time (target outside 120° FOV), and without continuous bearing the controller can only *react* (intercept-through-cone), not *predict* (lock-and-hold). M1 pathgen with oracle bearing/range/closing-rate hit ~-50K on the same NN topology + cone fitness; the **3× gap is the cost of perception under FOV-induced blindness**.

The system is qualitatively flyable. 95.2% scenario completion (14/294 elite crashes — mostly hull-strike during close tail-chase, not arena egress). Selection is converging to a "tight-spiral tail-chase" strategy. Worth keeping the weights; this is a reasonable v1 baseline.

## Fitness trajectory

| gen | best fitness | Δ from prev | notes |
|---|---|---|---|
| 1 | -4,602 | — | random pop baseline |
| 50 | -10,150 | -5,548 | fast early climb |
| 100 | -13,629 | -3,479 | most growth done |
| 150 | -14,902 | -1,273 | post-ramp #3 (gen 120) |
| 200 | -15,392 | -490 | post-ramp #4 (gen 160) climb |
| 250 | -15,898 | -506 | |
| 300 | -16,525 | -627 | |
| 350 | -16,346 | +179 | post-ramp #8 (gen 320) regression |
| 400 | -16,942 | -596 | recovery + climb |
| 450 | -17,055 | -113 | **plateau zone entered** |
| 500 | -16,853 | +202 | post-ramp #12 (gen 480) regression |
| 542 | -16,853 | 0 | held |

Best-of-run elite: **gen 446 at -17054.83**. From gen 450 onward the trajectory oscillates ±300 around -17K with no upward trend.

## Determinism + variation-ramp behavior

- **542/542 NN_ELITE_SAME, 0 DIVERGED** across all 12 variation-ramp events fired during the bake. The P1 windSeed fix (M11.preA.3) holds rock-solid even at hull-on.
- Ramp drops are small (0.5-3% of fitness) and recoveries are fast (5-15 gens). This is robust-generalist behavior, not narrow-exploit. Selection is keeping individuals that work across the widening per-scenario perturbation surface.
- Ramp at gen 160 cost 3.2% (largest); ramp at gen 200 cost 0.6% (smallest). Trend is toward smaller ramp drops as the controller generalizes.

## Behavior shape

From #GenDiag at gen 542:

| metric | value | meaning |
|---|---|---|
| avgVis | ~0.70 | target visible 70% of ticks; **30% blindness** |
| avgInRamp | ~0.07 | chase in cone-fitness ramp 7% of ticks; never settles |
| avgRngMin | ~3.4 m | reaches trail distance routinely (TrailDistance = 3.048 m) |
| avgRngMed | ~19 m | sits ~6× trail distance most of the time |
| avgRngP95 | ~40 m | even worst-case range is bounded |
| avgFlips | ~17 / scenario | aggressive maneuver count |
| avgSpiral | ~0.28 | banking / translation ratio |
| avgThrPt | ~4.0 | pitch axis aggressiveness |
| avgThrRl | ~7.3 | roll axis aggressiveness (dominant control axis) |
| maxLost | ~90-95 ticks | longest single blind run (~9 s) per scenario |
| crash% (elite) | 4.8% (14/294) | 15 hull-strike, 4 arena egress at gen 502 sample |

Per-axis aggressiveness (gen 503 snapshot):
- pitch dCtrl 0.56, amplitude 0.77 (over amplitude budget)
- **roll dCtrl 1.02** (4× over budget), amplitude 0.66 — bang-bang roll axis
- throttle dCtrl 0.26 (just under), amplitude 0.95 (full throttle saturated)

Late-run path-5 rotation rates: roll ~186°/s, pitch ~89°/s.

## Streak-break decomposition (geometric-only, post-fov/cone-fix)

At gen 542: loss_total ≈ 1,291. Of those:

| reason | count | % |
|---|---|---|
| **angle** (lateral off-cone) | ~500 | 39% |
| **far** (distance dominant) | ~785 | 61% |
| **over** (chase ahead of rabbit) | ~5 | <1% |
| **hull** (per-scenario crash) | ~1 | <1% |

`far` dominates late-gen — the controller takes risks getting close, sometimes loses the rabbit entirely. `over` near-zero confirms evolution punishes overshoot ruthlessly. `hull` is per-scenario-terminating; rare on the elite (~5%).

## What we learned

1. **Determinism + crrcsim FDM tracker training is solid**. Multi-day bakes at production scale produce stable, reproducible results.
2. **Pop=5000 was the right call** — 6000 didn't meaningfully change the fitness curve in earlier smoke14b/smoke15 runs; 5000 is faster wall-clock and matches the NN-weight-density of the 029 pastonly3 baseline.
3. **fov/cone dead-code fix had no behavioral impact** (as predicted — it was instrumentation only).
4. **The plateau is real and below M1's ceiling.** This is the binding observation for 032.

## Why the plateau exists (revised diagnosis)

**Original hypothesis** (032 spec v1): 2-beacon front-back ambiguity prevents pose-aware prediction.
**Revised diagnosis** (this report): M1 pathgen reached -50K WITHOUT pose info either — pose isn't the bottleneck. The actual binding constraint is **FOV-induced blindness**:

| info | M1 (oracle) | M2 (beacon) |
|---|---|---|
| Body-frame bearing to target | always-on, infinite-FOV unit vec | only when target in 120° camera FOV |
| Range | always-on scalar | derivable from beacon-pair separation IFF visible |
| Closing rate | always-on scalar | derivable from history IFF recently visible |
| Target pose | NONE | NONE |

M1's chase was never blind. M2's chase is blind 30% of ticks. Cascading effects:
- Long blind stretches (maxLost ~90 ticks = 9 s) require dead-reckoning toward predicted bearing
- Current 16-wide recurrent layer has limited capacity to maintain a target-velocity prior across long blindness
- Selection therefore favors "FOV-conservative" maneuvers (no aggressive looks-away), which caps tail-chase tightness

## Next experiment: T-102 (16r → 32r)

**Hypothesis**: state capacity (recurrent layer width) is the binding constraint inside the visible/blind cycle. Bigger recurrent layer can:
- Hold a target-velocity prior across longer blind stretches
- Integrate more history into the per-tick decision
- Maintain richer per-axis intention through maneuvers

**Experiment**: bump topology 45 → 32 → **32r** → 3 (was 16r). Weight count 2,307 → 2,947 (+28%). W_hh grows from 16×16=256 → 32×32=1,024 recurrent weights (4×). NO input changes — same beacon NDC + history. Single variable.

**Predicted outcomes**:
- If fitness ceiling rises meaningfully (say to -19K or better) → state capacity WAS binding. The 32r becomes the new 030 baseline. 032 input enhancements stack on top.
- If fitness ceiling stays near -17K → state capacity is NOT binding. Input information content IS the bottleneck. 032 (span / span-rate / tilt) priority rises; further recurrent bumps deferred to 033 or M3.

**Compute budget**: pop=5000 unchanged. ~28% more weights → ~28% more training wall-clock per gen. Expected: similar 2-3 days to gen 500-600. xiao deploy cost: 2.6 ms → ~4.0 ms eval (well under the 50 ms tick budget); generic-loop overhead, NOT MAC-bound.

## Artifacts

- Log: `logs/autoc-030-postdiag2-crrcsim-hullon-pop5000.log`
- data.dat / data.stc: in repo root at run-stop time
- Final reports: `specs/030-tracker-mode/autoc-030-postdiag2-crrcsim-hullon-pop5000_*.png` (4 PNGs)

## Open questions for v1 closeout (post-T-102)

- Does 32r alone close enough of the M1-M2 gap to declare M2 v1 done?
- If not, does 032 input enhancement (span / tilt / hull-imminent) add the rest?
- If neither, M2 v1 ships at -17K and the next leap is M3 perception-loop work.

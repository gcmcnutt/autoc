# 034 Stuck-Basin Investigation — log and 033 cross-check

**Date**: 2026-05-30 → 2026-05-31
**Status**: Open. Investigation produced an unexpected finding: the regression may predate the 034 checkpoint.

## Trigger

After the 034 cleanup commit (`7e8a90f`), four fresh-seed M1 pathgen bakes (test3 / test4 / test5 / test6) all converged to the same throttle-pinned spiral attractor and never broke out, despite varying master seeds:

| Run | Master seed | Final best | Notes |
|---|---|---|---|
| test3 | 1779993857 | ~-12k (stalled) | abandoned ~gen 90 |
| test4 | 1780162827 | -14,728 (frozen) | ran to gen 245 |
| test4b | 1780162827 | matched test4 bit-exact through gen 6 | determinism replay |
| test5 | 1780195220 | -14,7xx (stalled, see PNGs) | ran to gen 154 |
| test6 | 1780211286 | -9,120 (frozen gen 175+) | ran to gen 253 with `RandomPathSeedB` reverted 2468 → 13337 |

All four landed in the same attractor signature: throttle amplitude = 0.999–1.000, dCtrl ≈ 0; roll relaxed; pitch bang-bang to rail; per-path roll rate ~200–230°/s. test4b proved 034 is bit-deterministic given a fixed seed (eval-parity already established this for the elite; test4b confirmed it for the full training trajectory).

## Path-5-seed isolation experiment

The 034 checkpoint bundled an unrelated `RandomPathSeedB` change (13337 → 2468). Test6 reverted to 13337 to isolate the path-geometry contribution. **It still stalled** (-9,120 frozen). → path-5 geometry is not the regression driver.

## 033-final binary cross-check

To disambiguate "034 checkpoint introduced a regression" from "M1 basin lottery is just hard at pop=8000/wind=36", checked out `395b39f` (033 wrap commit, autoc) + `19b6367` (matching crrcsim pointer), rebuilt via `scripts/rebuild-perf.sh`, ran a fresh-seed bake (`033-bisect1`, Seed=-1 → 1780250847).

Result: **033-final binary ALSO stalled**. Best fitness froze at -11,432 from gen 137 onward (kill at gen 149). Tracked 033-r1's curve through ~gen 130, then arrested where r1 continued climbing.

Initial reaction was "regression confirmed inside 034 checkpoint", but the proper reading is harder: this is **n=1 sample of 033-final at fresh seed**, and it stalled too. Same binary that produced 033-r1 (climber, seed 1779919143) can also produce stalls (bisect1, seed 1780250847). Basin lottery.

## Baseline re-grounding

The "2-of-3 climbers" anchor that this investigation kept citing comes from commit `5935961` ("docs(032 basin-landscape)"). The three runs (`pop8000-wind36-r1/r2/r3-data.stc` in `specs/032-tracker-nn-enhancements/`) are filed under 032's spec dir for chronological reasons but are **M1 pathgen bakes**, not tracker — the commit body explicitly calls out "M1 climb rate to 2/3 (67%)" and the r3 stall description ("throttle amplitude 1.000 σ=0.000, throttle dCtrl 0.000") is the canonical M1 throttle-peg signature.

Re-tallied M1 baseline at pop=8000/wind=36:

| Code | Climbers / total |
|---|---|
| 032-end (`a0d9c16`): r1 + r2 + r3 | 2 / 3 |
| 033 final: pop8000-wind36-r1 | 1 / 1 |
| **Pre-034 documented** | **3 / 4 = 75%** |
| 034 checkpoint: test3 + test4 + test5 + test6 | 0 / 4 |
| 033 final today: bisect1 | 0 / 1 |
| **Combined recent runs** | **0 / 5** |

P(0 climbs in 4 trials | 75% base rate) = 0.4%. P(0 in 5) ≈ 0.1%. **The stall streak is statistically meaningful**, but the 033-final-binary also being in the streak means **the regression likely predates the 034 checkpoint**.

## Candidate explanation: 033 PRNG rework (`acf732f`)

The commit chain between `a0d9c16` (last code that produced 75% climb on documented runs) and `395b39f` (where bisect1 stalled today):

```
a0d9c16 (032-end, 2/3 climbers)
↓
51b4e99 Merge PR #4 from 032
86f5483, a62257a crrcsim submodule chores
acf732f feat(033 phase 1): master-seed PRNG architecture + M1 smoothness penalty  ← strongest single candidate
a886951 fix(032/033): remove ConfigManager from worker-side code + Constitution VII
84a7399 chore(033): bump crrcsim submodule for gather_tracker_inputs sig fix
9176afd, acbf248 checkpoint(033 phase-3/4 docs) — note: phase-4 already observed "smoothness-bypass bake still stalls"
5465cb7 Merge 032 into 033
395b39f docs(033): wrap
```

`acf732f` reworked the master-seed PRNG cascade. Same master seed produces entirely different scenario draws across the cascade boundary, so the basin landing odds shift wherever the cascade lands you. Phase-4's own stall report ("smoothness-bypass bake still stalls — bisect against 030") was already observing this pattern but it was attributed to smoothness wiring.

This is consistent with `project_v15_determinism_candidates.md` noting "non-determinism hurts evolution signal, not just elite-reeval" — but here the issue is deterministic-but-differently-seeded scenarios, not non-determinism.

## Next-bisect option (not started)

To confirm `acf732f` as the regression introduction:
1. Checkout `a0d9c16` (autoc) + its crrcsim pointer
2. Rebuild via `scripts/rebuild-perf.sh`
3. Run 2–3 fresh-seed bakes with current `autoc.ini`'s settings adapted to 032-end's available config keys
4. If 2/3 climb, regression is conclusively `acf732f` or one of the 4 chores between it and a0d9c16
5. If those also stall, regression is older still or there's environmental drift (libc, gcc, hw) — much harder problem

Cost: ~7h per bake × 2–3 bakes. Defer unless the deeper-lever options (036 islands, 035 lexicase) don't materialize as productive.

## Reframe for 034 forward path

The "034 broke M1" narrative weakens significantly with bisect1's stall. The productive next moves are unchanged from the [036 demetic islands research](036-demetic-islands/research.md) framing:

- **036 (deme islands)** still the highest-leverage lever — directly attacks single-attractor dominance regardless of which commit introduced the rate shift
- **035 (energy lexicase)** still the most-causal selection-rule fix for throttle-pin attractor
- **034 US4 (craft variations)** unchanged in status — orthogonal to the basin-lottery problem (see `project_craft_variations_not_diversity_fix`); doesn't need to wait on bisect resolution

The bisect-further-back work is a separate sub-track to inform 036's design; not a blocker for US4 or US5.

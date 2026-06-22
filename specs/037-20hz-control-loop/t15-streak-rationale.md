# t15 — widening the streak cone (FitStreakThreshold 0.5 → 0.3): why

**Date**: 2026-06-20. **Change**: `FitStreakThreshold` 0.5 → 0.3 in the tracker inis. **Only** delta
from t14 (same crash penalty: hull 0.75 ramped + OOB exp, same t10 source, MAD-ε). Tools:
`src/analytics/cone_topo.py` (cone topography), `src/analytics/score_by_path.py` (per-path generalization).

## The problem t15 addresses: t14 converged *shallow*

The crash-penalty arc (t12 displacement → t13 collapse → t14 curriculum-ramped fix) succeeded at
**safety**: t14 holds hull-strikes ~0, OOB low, 290/294 complete, no population collapse, no
divergence. But it **converged to a shallow-but-safe policy**: `pctInStreak` sat flat at **~7.4 for
~190 generations** (gens 100→290) with no upward trend — a touch below t11's 9.4 and, unlike t11/t12,
not climbing. It learned the *easy* half of the objective (complete the path, don't crash) and parked;
deep tracking never developed. `score_by_path` corroborated: shallow per-step tracking, uneven across
paths (path 5 long-but-shallow, path 0 erratic).

The concern: the crash penalty makes crashing expensive, so the controller plays it safe (track loosely,
don't risk closing in), and **nothing in the fitness landscape pulls it deeper**.

## The mechanism (cone_topo): the streak reward is unreachable on hard courses

The deep-tracking reward is the **streak multiplier** (up to 5×), which only accrues while the per-tick
step score ≥ `FitStreakThreshold`. `cone_topo.py` plots the step-score field
(`FitnessComputer::decomposeStepScore`) and shows the threshold contour = the boundary of the
streak-eligible region. Key findings:

- The streak region is an **entirely behind-the-rabbit lobe** — directly ahead the angle clamps to 90°,
  capping score at ~0.2, so streak **never** accrues when overshooting toward the target/hull (by design).
- At **threshold 0.5** the eligible zone is **tight**: reaches **7.0 m behind**, **±2.6 m** lateral,
  **26 m²**.

So on the hard courses, when the struggling chase lags further back or drifts off-axis, it flies
**outside** the streak zone → streak resets every wobble → **no multiplier gradient** → no incentive to
engage and tighten. The 5× reward is unreachable from where it actually flies. That is a concrete
mechanism for the flat pctInStreak.

## The fix: lower the threshold to give a gradient from further out

Dropping to **0.3** opens the eligible region **3.5×**: reaches **10.7 m behind**, **±5.7 m** lateral,
**91 m²**. A lagging/off-axis chase now:
- **banks streak from further out**, and
- **stops losing it on every wobble** (holds the streak through looser moments) →
- a **smoother gradient** that rewards getting into the general tail-chase zone and then pulls tighter.

Crucially the **score itself still peaks at the trail rabbit** — a tight tick ×5 ≫ a loose tick ×5 — so
0.3 does **not** reward looseness; it lets the controller *hold* a streak it would otherwise reset,
which is the gradient the shallow controller is missing. This is the "reshape the cone/streak gradient
for the harder paths" lever flagged in the t12 retro, and a sibling to 038 US5 (reward-gradient shaping).

## Risk & what to watch

- **3.5× is aggressive** — it *could* reward "hang loosely in the wide zone" enough to stay shallow.
  0.4 was the gentler middle; we chose 0.3 to see a clear impact (operator 2026-06-20).
- **Success signal**: `pctInStreak` **climbs** (vs t14's flat ~7.4) **without** hull/OOB regressing and
  without the population collapsing (gen_runtime keeps rising, gen-1 avg healthy). If pctInStreak climbs
  and crashes stay suppressed, the lever worked. If crashes creep back up (looser tracking → closer
  passes → hull), the threshold was too low.
- Compare t15 vs t14/t11 on `pctInStreak` and `score_by_path` (per-step depth per path).

## Config

t15 = t14 with `FitStreakThreshold = 0.3` (was 0.5) in `autoc-tracker.ini` + the two eval tracker inis
(matched for bitwise eval). Everything else unchanged. Run: `logs/autoc-037-t15-m2-streak0p3.log`.

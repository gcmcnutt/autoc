# 034 Findings — Population Size & Variation/Scenario Count

**Status:** live observation (2026-06-02), runs in progress — read as hypothesis, not settled result. Each data point is n=1 and confounded by seed; the *control-character* difference is the signal, not the absolute fitness.

## The two runs being compared

Both are M1 pathgen, aeroStandard 6 paths, all four variation classes ON (entry+wind+rabbit+craft) with the ±2.5σ Gaussian clamps in place, lexicase, ramp step 40, fresh seeds.

| run | pop | scenarios | outcome |
|---|---|---|---|
| `034-bigfull-clamped` | **8000** | 6×36 = **216** | climbed to ~−9k by gen ~120 then **plateaued**; **throttle pegged at 1.0** on 216/216 (amplitude 0.995, σ 0.005); avgMaxStreak flat ~2.4. Killed at gen ~296. |
| `034-origm1-5000x49` | **5000** | 6×49 = **294** | **tracks 029 pastonly3 nearly curve-for-curve** through gen 171 (−24.2k vs pastonly3 −25.4k); avgMaxStreak building to 13.6; **throttle MODULATING** (amplitude 0.835, max 0.944 — not pegged); bang-bang axis migrated to roll (dctrl 1.16). 0/294 crashes. |

## The observation (operator framing, 2026-06-02)

- **Population is a search-area knob, and 8000 vs 5000 matters here.** The larger pop (8000) converged into the throttle-peg Pareto corner and plateaued; the smaller pop (5000) held diversity long enough to track the gold-standard climber with throttle modulation intact. Bigger population is NOT automatically better — at 8000 it appears to over-converge onto the saturation attractor faster.
- **Scenario count (294 vs 216) cuts two ways.** More scenarios both (a) **damp any single scenario's influence** (one hard/degenerate scenario can't dominate selection) and (b) **normalize the per-scenario variations toward a noise floor** — i.e. with enough scenarios the entry/wind/craft/rabbit draws average out into general robustness pressure rather than distinct, exploitable signals. 294 is more averaging than 216.
- **The 8000:5000 distinction "feels like tuning"** — a sensitive knob rather than a principled fix. It's not satisfying to rely on "use 5000 not 8000"; the real lever for the saturation attractor is selection-rule (energy lexicase → 035), not population size. But the practical upshot right now is real: **the 5000/294 run is producing good control examples** (throttle modulation + roll-bang-bang character matching pastonly3's trajectory), which is exactly the signal we need.

## Caveat — still early, may re-saturate

This is logged at **gen 171/800**. The 5000/294 run is on the pastonly3 curve *so far*, but:
- It **may sharpen and fall back into saturation** (throttle-peg) in the back half as it converges — the throttle modulation is not guaranteed to survive.
- The variation ramp doesn't hit full strength until gen 800, and craft+rabbit (which pastonly3 never had) are still ramping — divergence from the pastonly3 curve could appear after gen ~200.
- pastonly3 itself only smoothed its roll in the back half (gen 280+); whether this run follows is unknown.

**Decision: let it ride.** Don't draw a conclusion until it's deeper (gen 400+) and ideally repeated on a second seed. The premature read ("5000 beats 8000") could flip.

## Links
- [project_m1_basic_learner_validated](../../.claude/projects/-home-gmcnutt-autoc/memory/project_m1_basic_learner_validated.md) — pop tracks scenario/behavioral diversity, not NN weight count
- [project_m1_basin_lottery_actual_rate](../../.claude/projects/-home-gmcnutt-autoc/memory/project_m1_basin_lottery_actual_rate.md) — the stuck-basin / saturation history
- [project_bangbang_axis_migration](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md) — dominant bang-bang axis is controller-specific (here: roll, with throttle freed)
- [project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md) — saturation = Pareto-corner collapse; 035 energy-lexicase is the principled lever
- pastonly3 gold standard: `specs/029-no-future-arch/pastonly3_outcome.md` (−55,944 @ gen 800, 294 scenarios — directly comparable scale)

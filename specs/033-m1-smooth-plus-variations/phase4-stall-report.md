# 033 Phase-4 stall report

**Status:** Phase-4 manually stopped at gen 422 / 800. Smoothness penalty **bypassed in fitness math** (paranoid removal — see `src/eval/fitness_computer.cc::applyStreak` and `src/eval/fitness_decomposition.cc::computeScenarioScores`), reducing fitness to the 029 form `stepPoints × multiplier`. Elite still flattens — peaks at gen 300 (-30,813) then *slightly regresses* to -29,605 by gen 422. The penalty is **not** what was suppressing M1 in phases 1–3.

## Symptom

Best/Avg vs 029-pastonly3 at matched gens:

| gen | phase-4 Best | pastonly3 Best | phase-4 Avg | pastonly3 Avg | phase-4 Sigma | pastonly3 Sigma |
|----:|-------------:|---------------:|------------:|--------------:|--------------:|----------------:|
| 100 | -11,748 | -13,590 | -2,634 | -2,800 | 0.176 | 0.17 |
| 250 | -28,973 | -37,061 | -5,595 | -10,547 | 0.125 | 0.11 |
| 300 | **-30,813 (peak)** | -38,036 | -5,931 | -12,320 | 0.125 | 0.10 |
| 400 | -29,688 | -47,275 | -6,019 | -19,779 | 0.125 | 0.07 |
| 422 | -29,605 | -51,015 | -6,366 | -21,498 | 0.125 | 0.07 |

phase-4 Best plateaus / regresses after gen 300. pastonly3 was still climbing actively through gen 400+. Population pressure (Avg) gap is ~3.4× at gen 422 — most of phase-4's population never moves past the early-gen basin.

bestSigma pinned at the floor (0.125) from gen ~255 onward — no further annealing once the plateau is reached.

## Per-axis aggressiveness at gen 422 (sum-axes budget ≤ 0.80 dCtrl, ≤ 2.00 amplitude)

| axis     | `<|Δ|>` mean | over-budget | `<|out|>` mean | over-budget |
|----------|-------------:|------------:|---------------:|------------:|
| pitch    | 0.49 | 98%   | 0.73 | 94%  |
| **roll** | **0.99** | **100%** | 0.82 | 99% |
| throttle | 0.43 | 86%   | 0.87 | 100% |

Sum-dCtrl = 1.91 / 0.80 budget. **Roll is the bang-bang axis** (~3.7× per-axis budget), consistent with the RNN topology baseline [[project_bangbang_axis_migration]]. With smoothness penalty bypassed, the controller has no pressure to refine — but its *failure to climb fitness* is the surprise, since fitness no longer pays for smoothness.

## What this rules out

| candidate | verdict |
|-----------|---------|
| Smoothness penalty wiring suppressed M1 climb | **Ruled out.** `applyStreak` reduced to 029 form (`stepPoints × multiplier`); 3 disabled smoothness tests; still stuck. |
| Smoothness floor parsing / silent enable | Already ruled out in phase-3 (see [phase3-stall-report.md](phase3-stall-report.md)) and now structurally bypassed. |
| Throttle dead-neuron saturation (phase-3 mode) | **Different mode this time.** Phase-4 throttle modulates (mean 0.87, σ 0.05) — not pegged at +1.0. The stall is a roll-pinned spiral with active throttle, not the throttle-pegged spiral of phase-3. |

## What's left as the suspect

The regression vs 029-pastonly3 must be in **non-fitness M1 code that changed in 030 or 032**. Candidates surveyed in phase-3 (PRNG cascade rework, sensor pipeline additions, etc.) remain candidates here — but with the smoothness penalty now structurally removed, the bisect must target *something else*.

**Decision (operator, 2026-05-24):** abandon further iteration within 033; bisect against 030 endpoint code with the pastonly3 seed.

- 029 end-of-branch + pastonly3 PRNG → **bit-identical replay confirmed** (see this session's replay run, gens 1–207 verified identical).
- 033 current code + pastonly3-like config → **flattens at -30k** (this report).
- **Next:** 030 end-of-branch + pastonly3 PRNG. If it matches pastonly3 → regression is in 032+033. If it flattens like phase-4 → regression was introduced earlier in 030.

032 was M2-focused; the M1 training loop *should* be untouched, but the bit-identical-or-similar-trajectory test will tell us whether that claim holds.

## Artifacts

- [033-phase4_evolution_progress.png](033-phase4_evolution_progress.png) — fitness/streak/stability/energy/sigma/whh_xh_ratio/block-CV, all 422 gens vs 029-pastonly3
- [033-phase4_per_axis_aggressiveness.png](033-phase4_per_axis_aggressiveness.png) — gen 422 snapshot, 294 scenarios
- [033-phase4_per_axis_time_series.png](033-phase4_per_axis_time_series.png) — dCtrl/amplitude per axis across all 422 gens
- Phase-4 code state: `applyStreak` bypass + 3 disabled smoothness unit tests (committed alongside this report).

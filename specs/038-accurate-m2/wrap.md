# 038 Wrap — Accurate M2 → handoff to 039 (xiao) / 040

**Date**: 2026-07-10. Companion to [outcome.md](outcome.md) (per-task findings log) and the run reports
in this directory (`autoc-038-t1…t10-*`). Verdict: **fairly acceptable results** (operator). The closing
exercise — 7 **random** paths × 7 variations, all novel — is a HARD task and the setup is relentless
(every scenario is unseen geometry under full-magnitude env/craft/entry variation, no easy cases), and
both controllers came through it crash-free.

## 1. The branch arc (t1–t10)

| run | what | verdict |
|---|---|---|
| t1/t2 | enriched-input M1 + basic-m1 diagnostic | enrichment tax real but modest; t1 was judged pre-takeoff — takeoff is a gen 250–400 event |
| t3 | enriched full-M1, 800 gens | ≈ parity with best-ever (037 t10) — situational-awareness inputs land free of control-quality regression |
| t5 | M1 source rebake (gen 800, best −41083.52) | PINNED — the M2 source library + the shipping `nn_weights.dat` |
| t4 | enriched M2 baseline | reproduces the 037 perception-capped ceiling — sensors alone don't move it |
| t6 | US3 predictor head (passive lexicase tie-break) | forms weakly at best; passive scoring doesn't couple to control; persistence baseline shows the as-posed objective is structurally worthless |
| t7 | shared source-env seeds (chase flies the target's airspace) | env fidelity **ruled out** as the ceiling cause |
| t8 | full variations from gen 0 (ramp off) | curriculum **ruled out** too; same shelf |
| t9 | equidistant (angular) projection + honest span target | slow start (the rectilinear tan-stretch was an accidental edge-alarm training aid) but recovered; **first run where all four predictor error curves grind down monotonically** — the tie-break CAN couple when the target is honest, just ~10× too slow as a tie-break |
| t10 | cross-eval wrap exercise (below) | M2 generalization measured for the first time |

Reward shaping, env fidelity, curriculum, extra sensors, passive prediction, W_hh capacity: all now
eliminated as the M2 depth limiter. What remains (040 material): perceptual history depth / internal
memory, structural prediction (consume the forecast), and possibly plain physics (a tail-chase of a
max-performance gen-800 path may sit near the achievable floor).

## 2. t10 — the M2 generalization read (2026-07-10, wrap exercise)

The four-step novel-path eval harness (BACKLOG "M2 novel-path eval harness") ran end-to-end for the
first time, all seed −1:

1. **M1 novel eval** — t5 elite on `random` pathgen, **7 paths × 7 winds = 49 scenarios** (master seed
   1783719858, run `autoc-9223370253134917267-…` in `autoc-eval`): **49/49 RabbitComplete, zero
   crashes**, −141.7/scenario vs −139.7/scenario on the 294-scenario training set — **no measurable
   generalization gap** for M1 on random geometry. Renderer check: reasonable on complex paths.
2. **M2 eval off that novel source** — t9 elite, `TrackerSourceRun/Bucket` repointed at the step-1 dmp,
   same 7×7 grid (master seed 1783720007, run `autoc-9223370253134767363-…`): **49/49 complete, zero
   hull strikes, zero OOB**. Median tracking error 15–19 m, %<5 m 6–9%, uniform across all 7 paths —
   no pathological path.

**Comparability caveat (operator read, confirmed in the data)**: raw per-scenario score is NOT
comparable across path classes — score accumulates per tick and the random scenarios are ~2× longer
(mean 896 steps ≈ 45 s vs 451 ≈ 23 s on the training set). Length-normalized: **0.076 pts/step novel vs
0.117 in-distribution (~65%), pctInStreak 3.7% vs 7.5%**. And the shortfall is a *path-class* effect as
much as generalization: the random class is a LONG complex path that plays as a mostly-continuous
**random-intercept** problem — engagement is actually best early (first-quarter median error 14.4 m,
17% <5 m) and bleeds off as the target's sustained maneuvering pulls away (last quarter 17.6 m, 4%),
so the settled tail-chase regime where streak points concentrate is scarce by construction (the M1
random-intercept analogue of [project_path5_random_intercept], now the M2 stressor).

**Read**: fairly acceptable. Degradation is graceful and uniform — weaker point-rate on a harder,
longer, intercept-dominated path class; error-when-engaged stays in the known ~17 m
perception-ceiling band; zero crashes under the full variation load on never-seen geometry. For a
controller whose depth is architecture-capped in-distribution, that is the expected — and acceptable —
shape.

Reports: `autoc-038-t10-m{1,2}-random49-eval_*.png` (per-axis aggressiveness both; intercept +
score_by_path for M2). Logs: `logs/autoc-038-t10-m{1,2}-random49-eval.log`. Configs: `autoc-eval.ini`
(random 7×7) + `autoc-eval-tracker.ini` (repointed) as committed on this branch.

## 3. Control-quality gate (the 038 co-equal criterion)

Per-axis on the novel set: **M1** stays near its training envelope (throttle amplitude remains the hot
axis, dCtrl pitch/roll ~0.25–0.27) — the enrichment did not regress control character. **M2** remains
saturated on all axes (dCtrl 0.46–0.63, amplitude 0.83–0.94) — unchanged from training; smoothness for
the tracker is 040+ work, downstream of the depth levers.

## 4. Toolkit deltas this branch

- `generate_pngs.sh m2` grew to **11 reports** (+ `predictor_analysis` with permanent persistence
  baseline, gen_runtime, mode_progress, score_by_path); intercept A/B falls back to the compare run's
  latest gen.
- `score_by_path.py`: mosaic handles odd path counts; winds-per-path derived from the meta (was
  hardcoded 49 — silently mis-bucketed any non-49 grid).
- Renderer: angular reticle ticks on the POV panel (equidistant NDC is linear-in-angle).
- The t10 harness itself: novel-source M2 eval is now config-only standing practice.

## 5. Handoff

- **039 (next): xiao back in shape.** Firmware regen is REQUIRED before any live-pathgen flash: the
  generated `nn_program_generated.cpp` still calls the pre-038 `gather_pathgen_inputs` signature; the
  xiao build must add `arena.cc`; regen via `nn2cpp -i <weights> -a <R,F,C>` (M1 topology now 37 in /
  2051 weights). See outcome.md T009b "Still open" + the 030-era xiao backlog entries.
- **→ 040 (M2 depth + camera):** the elevation package for the predictor (consume the forecast as
  inputs; first-class lexicase axis; re-target across blindness) — keep `VariationRampStep=0` for these
  weak-signal experiments; the streak-proxy input (close the reward↔observation Markov gap); hybrid
  projection fallback (planar x/y + ray-angle span) if angular NDC ever binds; camera variations
  (mount-alignment 6-DOF first — clean additive-offset robustness target under angular NDC); US2
  two-timescale recurrence still parked with its unpark trigger now armed (US1+US3 did not move the
  ceiling as posed). All detailed in `specs/BACKLOG.md` → "038 deferrals".
- **t10 harness becomes the wrap-gate**: future M2 features report the novel-set generalization gap
  (pctInStreak / error-distance, novel vs training) alongside the training numbers.

# 041 — M2 Depth (seed)

**Status**: SEED — not a spec. Written 2026-08-06 at the 040 wrap so the next feature starts from the
operator's scoping rather than from a blank page. Run `/speckit.specify` against this to produce
`spec.md`.

**Predecessor**: [040 camera redo](../040-camera-redo/outcome.md) — CLOSED 2026-08-06.

---

## Why this feature exists

040's verdict, in the operator's words: *a better camera model, much closer to real, and training
results compared to the prior camera more or less the same — that is the going concern.*

Perception got materially harder in six directions (acquisition cost, range-dependent quality,
obstruction, separation-range cutoff, 17% smaller beacon separation, per-scenario camera
misalignment) and competence did not move. Two independent readings of the same conclusion:

- **037 said it from one side** — M2 depth is reward-invariant, so it is not the objective's scale
  ([[project_m2_tracking_ceiling]]).
- **040 says it from the other** — it is not perception fidelity either.

So the cap is in the **controller**, and 041 goes looking for it.

The sharpest single number 040 produced is the T085 generalization gap. Same elite, its own 294
training scenarios vs 49 novel ones:

| | median err | p90 | **inside 5 m** |
|---|---:|---:|---:|
| trained geometry | 13.03 m | 28.82 m | **15.3%** |
| novel geometry | 15.00 m | 29.15 m | **8.4%** |

**Standoff generalizes. Tight tracking does not.** The controller holds unfamiliar targets at almost
exactly the same distance and converts that hold into a close chase half as often. Whatever the
tight-tracking skill rests on, a meaningful part of it is memorized geometry.

---

## Scope — three threads, operator 2026-08-06

### 1. Redo M1

The M1 every M2 bake trains against is `autoc-m1 · autoc-9223370253553029228-2026-07-06T01:35:46.579Z`
— pinned since 038, and now **pre-dating the objective's one-tick target-offset fix** (`68f64ab`),
which corrected a 28% error in the definition of the trailing task. M1 has been optimizing the same
wrong target since 030.

Open questions for the spec, not decided here:
- Does the one-tick fix change M1's landscape enough to be worth a rebake on its own, or only as part
  of a larger change?
- Rebaking M1 **moves the M2 source**, so every M2 number since 038 loses its comparator. Sequencing
  matters: a new M1 baseline probably wants its own M2 re-baseline immediately behind it.
- [[project_m1_basin_lottery_actual_rate]] — M1 climbs reliably at pop 3000 / single longSequential /
  16 winds and unreliably at pop 8000 / 49 winds. Any rebake plan has to state which regime it runs in
  and why.

### 2. Control aggressiveness, M1 and M2

Standing project thread, now with flight evidence behind it. The per-axis reports
(`per_axis_aggressiveness`, `per_axis_time_series`) are already the gate
([[project_038_regression_gate]]); what is missing is a lever that moves them without costing tracking.

Live context to fold in:
- **The bang-bang axis migrates** across controllers and eras — 037 pushed it roll → throttle once the
  servo model got real ([[project_bangbang_axis_migration]]).
- **Flight data says the loads are real**: routine ±7–8 g, a +11.2 g / −8.4 g record on 2026-07-20, and
  86% throttle saturation — see the 039 pitch-lever entry in [BACKLOG.md](../BACKLOG.md).
- **Scalar penalties do not work here.** [[project_scalar_multiobjective_collapse]] — 033 phase 1 died
  in a Pareto corner on exactly this. Lexicase/Pareto or nothing.
- **Group axes physically**, bank (pitch+roll) vs throttle, not per-NN-output
  ([[project_smoothness_axis_grouping]]).

### 3. A predictor that works

**Already measured — do not re-litigate the design.** The span/closure head carries no information:
r(Δspan) ≈ 0 at every horizon, best error at generation 1, and the head occupies **a third of the
lexicase pool**, so it is actively harmful rather than inert. Full measurement, plus the E1–E4 ladder
(ablate → is it learnable at all → fix the objective then actuate → value head instead of world
model), is in **[BACKLOG.md](../BACKLOG.md) → "Make the predictor earn its keep"**. Start at **E1**
(ablation, hours on M1) and **E2** (offline regressor, minutes, no simulation) — either can kill the
whole branch cheaply.

Note the interaction with thread 2: E1 returns a third of selection pressure to the axes that mean
something, so it is a candidate lever for aggressiveness as much as for prediction.

---

## What 041 inherits, ready to use

- **Two pinned novel-geometry eval sources** (040 t5 and 038 t10), each 7 random paths × 7 winds flown
  by the pinned M1. Using both is how you tell a real generalization difference from a 49-scenario
  sampling artefact. Harness: `autoc-eval.ini` → `autoc-eval-tracker.ini`, ~11 s end to end.
- **The t4 comparator**: `autoc-m2 · autoc-9223370251039771221-2026-08-04T03:43:24.586Z/`, 800 gens,
  converged at gen 660.
- **A calibratable perception model** — every physical quantity classified M/D/A in
  [config-surface.md](../040-camera-redo/contracts/config-surface.md), with SC-012 proving that
  substituting all 15 assumed values needs no code change.
- **11-report toolkit**: `scripts/generate_pngs.sh m2 <log> [--compare …]`.

## Traps 040 paid for — do not re-pay them

1. **A baseline's weights expire when the dmp schema moves.** The 038-t9 elite is no longer loadable
   by a current binary. If 041 re-baselines, archive `nn_weights*.dat` (NN01, stable) alongside the
   dmp — see the 040 deferral in [BACKLOG.md](../BACKLOG.md).
2. **Land objective changes BETWEEN baselines, never between a run and its comparator.** The one-tick
   fix landing mid-feature cost 040 its clean camera-variation delta.
3. **A diagnostic that does not vary is telling you something.** t2 and t3 capping identically at
   ~44 s across a tenfold envelope change was the obstruction bug, not the envelope.
4. **`set -e` does not fire through a pipe.** A failed `nnextractor | tail` silently re-flew the
   previous weights and nearly entered the record as a baseline result.

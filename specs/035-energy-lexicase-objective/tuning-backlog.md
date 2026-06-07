# 035 modelling / incentive backlog — before exiting the feature

Observations from the t6 (035 lexicase) per-axis aggressiveness analysis, 2026-06-06.
The intent here is **proper incentive — better, proper modelling** — NOT hand-tuning penalty
knobs. These are physics/objective candidates to experiment with now that the plumbing
(buckets, zstd, dmp-dump, detached `scripts/train.sh`) is solid. Resolve / promote / drop
before closing 035.

## What we saw

Per-axis aggressiveness, t6 @ gen 111 vs 034-test4 @ gen 209 (the variation-stable
cross-run comparator; both M1, RNN topology):

| axis | 034-test4 dctrl / mag | t6 dctrl / mag | reading |
|---|---|---|---|
| throttle | 0.005 / 0.997 (frozen, pinned full) | 0.157 / 0.968 | now **modulating** |
| pitch    | 0.321 / **0.827** (saturated) | 0.429 / **0.578** | **de-saturated** (the real energy win) |
| roll     | 0.425 / 0.390 (bang-bang axis) | **1.126** / 0.867 | roll bang-bang **amplified** |

Substantive change vs 034 M1: lexicase escapes 034's "throttle-pinned + pitch-saturated +
roll-bang-bang" corner and **re-allocates** — frees throttle, de-saturates pitch,
concentrates change-rate on roll.

Cross-run survey of per_axis_time_series (M1 + M2): the bang-bang axis is **set at breakout
(~gen 50–100) and persists — no clean *within-run* late flips** (033-phase4 pitch spike ~gen
150 reverts; 034-origm1-5000x49 590-gen pitch narrows toward roll but never overtakes).
Flips occur **across** runs/basins (032-pop8000-wind36-r1 is pitch-dominant), consistent with
[[project_bangbang_axis_migration]] — each basin commits early.

## Energy-cost ranking of aggressiveness (operator physics call)

Aggressiveness is NOT equally expensive per axis:

1. **Pitch — most expensive, and the well-modeled one.** Elevator → AoA → induced drag ∝ CL²,
   a direct airspeed/energy loss the LaRCSim FDM computes accurately. The energy-relevant term
   is pitch **amplitude** (sustained AoA), not change-rate.
2. **Throttle — moderate, mostly averaged.** PWM/electric: energy ≈ time-average duty cycle, so
   command wiggle ≈ holding the mean. Real extra cost (motor inrush, prop spin-up inertia, ESC
   switching) is **un-modeled**.
3. **Roll — least, indirect.** Ailerons redistribute lift; cost is 2nd-order adverse-yaw /
   sideslip drag, also **under-modeled**.

Consequence: the controller dumps aggressiveness on exactly the two axes whose real costs are
under-modeled (roll, throttle) while respecting the one the sim prices correctly (pitch). Where
the sim is accurate it behaves; the roll+throttle "free lunch" is partly the sim under-charging.

## Direction: fix the MODELLING, not the penalty knobs

The goal is **proper incentive — or better, proper modelling** — NOT hand-tuning a per-axis
dctrl budget. A weighted dctrl penalty is a non-physical proxy; composing scalar smoothness
penalties is exactly the corner-collapse trap of [[project_scalar_multiobjective_collapse]].
Get the physics right and the existing energy objective disincentivizes the expensive axes by
construction; the per-axis dctrl/mag readouts then stay **diagnostics, not objectives**.

Priority order:

1. **(Best) Model the real per-axis energy costs**, so "energy" in the objective = true energy:
   - aileron profile + adverse-yaw / sideslip drag → charges roll aggressiveness (currently the
     under-charged "free" axis the controller exploits),
   - a throttle-transient term: motor inrush + prop spin-up inertia + ESC switching → charges
     throttle cycling (today ≈ free because energy ≈ PWM time-average).
   Pitch/induced-drag is already modeled accurately by LaRCSim, which is why the controller
   already respects pitch. Close the two gaps and the incentive self-corrects. These are the
   "revisit pre-hardware retraining" sim-fidelity items — same pattern as
   [[project_cep_realism_backlog]].
2. **(If a control-effort term is still needed) derive it from physical energy**, per-axis,
   from the drag/power model above — not an arbitrary uniform or hand-weighted dctrl budget.
3. **(Deprecated) hand-weighting the flat 0.27 dctrl budget.** Recorded only to mark it as the
   wrong frame: it tunes a proxy and invites scalar-collapse. Use dctrl/mag purely to *measure*
   whether the modelling fix worked.

## Verification
- **Maturity-matched re-comparison.** Re-run the t6-vs-034-test4 per-axis comparison at t6
  **gen ~210** (test4 was gen 209) for the apples-to-apples verdict — t6's gen-111 numbers are
  only ~20 gens post-breakout and the roll dctrl may still be settling.
- After any modelling fix, the *test* is whether roll/throttle aggressiveness drops **without**
  an added smoothness penalty — i.e. the energy objective alone now charges for it.

## Watch items as t6 matures
- **pitch amplitude** must stay low (it's the real energy term); fine if pitch *dctrl* rises so
  long as amplitude doesn't follow it back up.
- whether roll dctrl refines down or stays ~1.1–1.2 (settled strategy vs early-breakout
  overshoot).

Related: [[project_pitch_only_smoothness]], [[project_smoothness_axis_grouping]],
[[project_cep_realism_backlog]] (sim-fidelity-before-hardware pattern).

# 037 Outcome: Faster Control Loop (10 Hz → 20 Hz)

**Date**: 2026-06-11. **Verdict: NO-GO on cadence — RT case B.** The roll bang-bang is
**objective/architecture-optimal, not a 10 Hz sampling artifact**. Doubling the control rate does
not change the converged control character. Operator decision: **return to 10 Hz** (the 037 bundle
stays); smoothness/tracking work routes to actuator-model fidelity, perception, and the recurrent
architecture — not loop rate. US1b (M2 retrain), US2 (embedded 20 Hz), and US3 (50 Hz) are
**gated OFF**. This is the cheap no-go the spec priced in.

## The three-arm experiment

| run | config | tracking (final) | roll at tracking depth |
|---|---|---|---|
| **035-t6** (baseline) | 10 Hz, ideal servo | pctInStreak 24.7% @610, holds 2.30 s | **ac −0.24, flips 56%, dCtrl ~1.0** |
| **037-t6** | 20 Hz + servo (tau 20 ms, slew 3–9) | **capped: pctInStreak 1.6% @267** | smooth (ac +0.65, flips 16%) — **plant-imposed** |
| **037-t7** | 20 Hz, servo `#if 0` | pctInStreak 20.4% @610, holds 2.07 s | **ac −0.26, flips 58.6%, dCtrl 0.93** |

- **t7 vs 035-t6 (cadence isolated, matched ideal-servo physics)**: tracking lands in the same
  league (slightly behind at matched gen; still refining when stopped), and the converged roll is
  **statistically indistinguishable** from the 10 Hz baseline. The character arc over t7's life:
  smooth while incompetent (ac +0.9) → roughening through competence (+0.34 @113) → full
  anti-persistent relay at deep tracking (−0.26 @610). 20 Hz delays the bang-bang; it does not
  remove it. Relay has spread multi-axis (throttle flips 22%, pitch 13% at the end).
- **t6 vs t7 (servo isolated, matched 20 Hz)**: the modeled servo (params later shown shaded slow
  vs the DSM-44 datasheet — see finding.md) **prevents tight tracking entirely**. Its "smooth"
  roll was the plant low-passing the same relay commands, not learned finesse; the muscle simply
  migrated to throttle (the un-lagged actuator).
- **Gate (T027)**: FAIL both legs at tracking depth — autocorr −0.26 (target ≥0), sign-flips
  58.6% (target ≤36%). Measured by `dealias_metrics.py` on the final elite dmp, mean-of-scenarios.

## Per-gen gate-metric trajectories

See `autoc-037-t{6,7}-m1-20hz*_dynamics_progress.png` (new chart, `dynamics_progress.py`): lag-1
autocorr / sign-flips / saturation / **regime occupancy** (tracking–intercept–patrol, operator's
framing: stpPt ≥ 0.5 = tracking) / target distance, per generation. The t7 chart shows the
smooth→relay transition tracking exactly with the tracking-band opening — the visual core of the
verdict. Completions are a red-herring metric in this regime (spiral arrives without tracking);
judge by pctInStreak / occupancy / distance (`project_servo_era_progress_metrics`).

## Pinned artifacts (S3, `retain=expire` — pin if needed)

- t6: `autoc-m1/autoc-9223370255738048515-2026-06-10T18:38:47.292Z/` (gens 1–267)
- t7: `autoc-m1/autoc-9223370255710101450-2026-06-11T02:24:34.357Z/` (gens 1–610, 0 ELITE_DIVERGED)
- t3 (10 Hz + servo arm): `autoc-m1/autoc-9223370255788564066-2026-06-10T04:36:51.741Z/` (pre-flip
  layout — readable only from a `9592dea`-era build)
- Final PNG sets in this dir; smoke runs t4 (bug-finder) / t5 in `logs/` + finding.md.

## What 037 bought (kept at 10 Hz — the bundle is rate-generic)

1. **The closed question**: "run the loop faster" is answered with evidence before any firmware
   spend, exactly as the spec's cheap-no-go path intended.
2. **The servo finding**: a realistic (even pessimistically-parameterized) servo caps this
   controller family's tracking — a hardware-relevant ceiling independent of cadence. Servo model
   (in-FDM exact lag+slew + per-scenario draws) built, debugged, toggleable; DSM-44 datasheet
   check bounds the real params (finding.md).
3. **Cadence as config**: 2-knob rate flip (SIM_TIME_STEP_MSEC + ini), startup triple assertion,
   rate-independent engage delay, tick-scale-anchored fitness (scores comparable across rates).
4. **Time-based log-spaced history** (1.6 s window, counts unchanged, integral at 10/20/50 Hz).
5. **Env-only variation ramp** (craft at full magnitude from gen 0 — fleet-robust controllers) +
   entry-cone tighten + energy^2.5.
6. **Fail-loud schema boundaries**: NN-history layout marker v2; M2 source-tick-spacing check.
7. **Two latent bug fixes**: servo slew×lag compounding (36a94f3); framesPerEval=1
   pending-command starvation (f81fd31) — found by the basic-m1 smoke discipline, which is now a
   standing rule for loop-plumbing changes.
8. **Measurement toolkit**: `dealias_metrics.py`, `dynamics_progress.py` (regime occupancy),
   T005 slim log + dmp-complete per-scenario data, REPORTS.md pipeline with incremental caches.
9. **Observations for the next features**: 037-era runs went 3/3 climbers at full scale (basin
   lottery looks mitigated — fixed-seed caveat, `project_m1_basin_lottery_actual_rate`); residual
   crash floor ~13–23/294 persists at deep tracking (cause unknown: full-impact craft vs history
   basis — the 10 Hz t8 run will help attribute); streak diagnostics are tick-denominated (2× at
   20 Hz — compare pctInStreak or convert to seconds, never raw avgMaxStreak across rates).

## Route forward (operator, 2026-06-11)

1. **Back to 10 Hz** with the 037 bundle (this flip happens immediately after this doc's commit).
2. **Servo model v2** (next runs): replace the pessimistic lag-dominant model with the
   datasheet-shaped one — **0–20 ms PWM frame lag** (50 Hz command latch) + **slew from the
   0.07 s/60° transit** (≈50–60 ms of pure travel per 60° once latched) + small residual tau.
   Question to answer: is there a parameter point where a *little* realistic filtering coexists
   with tracking? 10 Hz no-servo confirm run (t8) first to re-anchor; then servo-v2 (t9).
3. If even honest-parameter filtering kills tracking → the lever is **perceptual fidelity and the
   R in RNN** (and possibly airframe/X-wing), not actuator modeling.

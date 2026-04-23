# 026 — INAV ACRO Delegation (non-bang-bang control via rate-mode inner loop)

**Status**: PLAN — scope locked, implementation sequence in
[`plan.md`](./plan.md).

**Companions**:
- [`research.md`](./research.md) — git history of prior experiments
  (021 ACRO PID build + revert, 023 pt3 filter), INAV filter
  inventory, analysis of whether ACRO delegation settles bang-bang
  or displaces it, objective-function options, escalation ladder
  (previous-output feedback, Pareto selection).
- [`plan.md`](./plan.md) — implementation phases, ordering (sim
  signal before flight prep), INAV-knob audit on the flight config,
  go/no-go gate table, stopping rules.

## Clarifications

### Session 2026-04-23

- Q: Training variations for cadence8 ACRO retrain — full (cadence7 parity) vs relaxed? → A: Full variations identical to cadence7, with smoke testing first.
- Q: Go/no-go gate — hard thresholds or organic early-signal judgment? → A: Keep the plan.md gate (dCtrl < 0.8, fitness ≥ -25000) but treat the call as organic — early fitness-curve shape and qualitative output-histogram behavior can trigger an early no-go without waiting for gen 400.
- Q: Sim yaw handling under ACRO (given HB1 has no controllable rudder)? → A: Leave yaw passive in sim — no PID, no rudder deflection. Matches real HB1 which has a tail fin for stability but no rudder surface. INAV yaw PID output has no physical actuator on this airframe either.
- Q: xiao log `out=[pt,rl,th]` token — rename to `rate=...` under ACRO, or leave? → A: Keep the `out=` literal token unchanged. Semantic shift documented in comments / docs / new diagnostic fields; existing parsers (`plot_bangbang_flight.py`, `cmd_response_scatter_lagged.py`, `join_flight_analysis.py`) continue working without modification.
- Q: Throttle bang-bang — if cadence8 fixes pitch/roll but throttle still pins at +1, does that clear the 026 gate? → A: Yes — gate on pitch/roll dCtrl only. Throttle bang-bang is envelope-dominated (wind + HB1 Vmax), not architecture-dominated; separate control-architecture change (airspeed PID or INAV CRUISE mode) is a 027+ item.

## Summary

Delegate low-level rate stabilization to INAV's ACRO-mode PID + matching
inner-loop emulation in CRRCSim, so the NN only has to answer "what
rate do I want?" instead of "how do I keep the aircraft from oscillating
while I try to hit a target?" This restores the 021 design intent,
which was shelved on 2026-04-07 on the basis of 022's successful
direct-servo training — a call that was right then but is superseded
now that cadence7 has hit the full-throw bang-bang ceiling inherent to
current-state-only MLPs (see
[`flight-results/flight-20260422/FLIGHT_REPORT.md`](../../flight-results/flight-20260422/FLIGHT_REPORT.md)).

## Problem statement

cadence7's trained NN, deployed on flight-20260422, produced clean
sim-to-real sign correlation on all control axes but collapsed to
full-throw bang-bang: pitch and roll outputs pile up at ±1, throttle
pinned at +1 ≥ 90 % of the time. Mean `|Δout|` per tick ≈ 1.0; mean
`|out|` per tick ≈ 2.2 out of a 3.0 ceiling.

Training statistics are essentially identical to flight statistics — the
NN is doing what it was trained to do. The architecture cannot express
smoother control: a single-pass, current-state-only MLP has no machinery
for integration, no explicit time-history of its own outputs, no
low-pass filter structure. Evolution against that input vector will, in
the limit, always find bang-bang as the optimum.

See also
[`specs/024-sim-real-fidelity/cadence7_bang_bang_evolution.md`](../024-sim-real-fidelity/cadence7_bang_bang_evolution.md)
for the empirical bang-bang characterization.

## Primary approach: INAV ACRO delegation

**NN outputs `outPt`, `outRl`, `outTh` are reinterpreted as desired body
rates (and throttle unchanged)**. `outPt ∈ [-1, +1]` × `ACRO_MAX_RATE_PITCH`
= desired pitch rate in rad/s; same for roll and yaw. INAV's ACRO-mode
rate PID (already on the FC) tracks the commanded rate via its
feed-forward + P + I controller, driving surface deflections. The NN
becomes a pure planner; the inner loop is physical PID with
well-understood behavior.

The 021 data-model [`021-xiao-ahrs-crosscheck/data-model.md`](../021-xiao-ahrs-crosscheck/data-model.md#L60-L73)
captured the *semantics* shift cleanly:

> In ACRO, output=0 means "hold current attitude" (rates damp to zero
> via INAV PID). In MANUAL, output=0 means "servos centered" (aircraft
> does whatever physics dictates). ACRO gives the NN a stable baseline
> to work from — it only needs to command rate deltas for steering.

**Why this settles (or at least pushes back on) bang-bang**: with a
consistent rate-mapping, the fitness landscape is smoother — small
input changes produce predictable output changes — so selection for
moderation happens faster. ACRO's "natural zero" also means that
outputting 0 *doesn't cost fitness*, which in MANUAL it does (no
built-in stabilization). See research §2 for the full analysis
including what ACRO does *not* fix on its own.

## Scope

### In scope

- Re-enable the 021 ACRO rate-PID code in CRRCSim `inputdev_autoc.cpp`
  (already preserved in git at commit 9809dd6). Use the existing
  `ACRO_MAX_RATE_*` and `ACRO_FF/P/I` constants in
  `inputdev_autoc.h:63-90` — they already track flight-measured rate
  limits. **Pitch and roll only** — yaw stays passive (no PID, no
  rudder input). HB1 has a tail fin for stability but no controllable
  rudder; INAV's yaw PID has no actuator to drive on this airframe, so
  sim mirrors by leaving yaw alone. The `ACRO_FF_YAW / P_YAW / I_YAW`
  header constants stay as documentation but aren't wired. (The earlier
  "HB1 rudder-moment calibration" note in the 025 spec is based on a
  misread of HB1's tail geometry; needs revision on the 025 branch.)
- Add INAV-equivalent filters in sim: gyro LPF and D-term LPF on the
  inner loop. Specific INAV params match the bench config
  (`xiao/inav-bench.cfg`); cross-reference the inventory in research §3.
- Decide the INAV param set for the *next flight* (PID gains, rate
  limits, filter cutoffs, rc_filter). Flight FC config will be set
  from those.
- **Compile-time constants for now**. `ACRO_MAX_RATE_*` and `ACRO_FF/P/I`
  stay in the header. No new ini knobs. No XML properties. We can
  promote to `hb1_streamer.xml` later if variation sensitivity warrants.
- Xiao/INAV side: switch the engage-time mode from MANUAL to ACRO. MSP
  channel mapping may need adjustment (see 021 research R7). INAV config
  must have ACRO mapped to the corresponding aux channel value.
- Additional diagnostics in `data.dat`: log the PID internal state so
  we can see what the inner loop is doing during training (desired vs
  actual rate per axis, integrator state, surface output after PID).
  No backward compatibility — existing parsers break cleanly when the
  format changes.
- Additional state in the eval S3 payload / renderer data so the
  renderer can later visualize PID behavior (desired-rate vs
  achieved-rate traces, integrator trajectory, saturation markers).
- Training: same fitness, same topology, same variations
  (`EnableEntryVariations=1`, `EnableWindVariations=1`,
  `EnableRabbitSpeedVariations=1`, `VariationRampStep=40` — all
  identical to cadence7), same rest-of-everything else. The only
  change is that the NN's action space is now "desired rate" instead
  of "surface deflection." Apples-to-apples comparison; if ACRO
  alone doesn't bend the dCtrl/|out| curve we know it's not the
  structural change alone. A 50–100 gen smoke run precedes the full
  400-gen run to catch regressions early (see `plan.md` Phase 2.2).
  Retrain to produce **cadence8** (or rate1, TBD naming).
- Measurement: reuse
  [`plot_control_aggressiveness.py`](../024-sim-real-fidelity/plot_control_aggressiveness.py)
  and
  [`plot_bangbang_flight.py`](../024-sim-real-fidelity/plot_bangbang_flight.py)
  to compare the new training to cadence7. Primary go/no-go metric:
  dCtrl plateau drops ≥ 20 % at tracking-fitness ≥ 70 % of cadence7.

### Out of scope

- **minisim**: **N/A** — minisim is offline analysis, not flight
  hardware. It stays on its current simple kinematic model. No ACRO
  PID in minisim.
- `hb1_streamer.xml` changes. Compile constants stay in the header. A
  future 027/028 could move them into the airframe config if we want
  per-airframe variation (which is 025 territory anyway).
- **Backward compatibility** on data.dat format, log format, xiao log
  format. Format changes are allowed; old files just don't parse in new
  tools. Downstream scripts (sim_polar_viz, cmd_response_scatter,
  bangbang_flight) get whatever updates they need to read the new
  fields.
- Previous-output feedback inputs (option A in research). Kept in reserve
  as escalation if ACRO alone doesn't bring dCtrl down enough.
- Fitness-side changes (Pareto selection, quiet-scenario lexicase).
  Likewise reserved. Go/no-go on ACRO-only first.
- Recurrent NN (option D in research). Out of scope for the foreseeable
  future — gradient-trained RNN/LSTM is out of project style.

### Definitions left to plan phase

- Which specific INAV params we want for next flight (exact gains,
  rates, filter cutoffs). Starting point is the 021-era values in
  `inputdev_autoc.h` and the bench config.
- Exact data.dat diagnostics schema.
- Exact S3 state schema for future renderer visualization.
- NN input-vector update (if any). 021 removed "previous commands"
  inputs on the assumption that ACRO integration replaced them; whether
  we keep that decision or re-add prior outputs is a plan-phase call.
- Xiao/INAV AUX channel remap for ACRO vs MANUAL if CH6 needs to move.

## Validation

Gate decisions are organic, not strictly automated. The plan.md gate
thresholds (dCtrl < 0.8, fitness ≥ -25000) are anchors, not pass/fail
checklists. Early signals — fitness-curve shape before gen 100, dCtrl
trend through the first variation ramp, output histogram qualitative
spread — can trigger an early no-go (stop and escalate to Phase 4)
before a full 400-gen run. Likewise, an obviously-converging run can
proceed to flight even if one threshold is mid-margin.

**Gate is pitch/roll-dominant.** The dCtrl metric is evaluated on
pitch + roll only. Throttle bang-bang may persist in cadence8 because
the root cause is envelope-limited (wind + rabbit groundspeed above
HB1 Vmax), which ACRO delegation does not address. If pitch and roll
move meaningfully off bang-bang but throttle stays pinned at +1,
that still clears 026; the throttle architecture change (airspeed
PID or INAV CRUISE mode) is deferred to 027+.

1. Sim retrain to ≥ 400 gens with ACRO loop active. Call it **cadence8**
   (or rate1).
2. `plot_fitness_ramp.py` — fitness trajectory comparable to cadence7.
3. `plot_control_aggressiveness.py` — dCtrl plateau drops measurably
   versus cadence7's (1.0, 2.2).
4. Eval suite passes (tier 0 bitwise determinism captured against new
   binary; tier 1–3 must pass).
5. Deploy to flight FC with matching INAV ACRO config.
6. Bench preflight per the cadence7 checklist in
   [`specs/024-sim-real-fidelity/cadence7_sensor_response_analysis.md`](../024-sim-real-fidelity/cadence7_sensor_response_analysis.md),
   plus ACRO-specific: verify rate response on bench (stick full-throw
   → aircraft rotates at the commanded rate).
7. Flight test. `plot_bangbang_flight.py` on the new xiao log — output
   histograms should spread away from ±1 toward the middle. New
   `plot_rate_tracking_flight.py` (to be written) — per-axis
   commanded-rate vs achieved-rate scatter, showing INAV PID tracking
   quality in real flight.

If bang-bang persists after ACRO retrain, *then* we reach for the
escalation options in research §4: previous-output feedback (option A)
or Pareto selection on (tracking, control-effort).

## Relationship to 025

025 (craft variations) remains BLOCKED on 026. The HB1 rudder-moment
calibration work in 025 becomes *more* relevant after ACRO — ACRO's
PID surfaces sim-vs-real yaw-authority discrepancies that MANUAL's
direct control partially masks. Sequence is 026 → 025 when 026 is
flying satisfactorily.

## Open questions for plan phase

1. Exact INAV param set for the flight FC — start from bench config and
   021 header constants, adjust based on flight tuning. Does this
   warrant a dedicated bench-tuning session before 026 implementation?
2. data.dat diagnostic schema — which PID internals matter for
   post-flight analysis? Desired rate, achieved rate, P/I/FF
   contributions, integrator state, saturation bits.
3. S3 additional-state payload — serialize what the renderer will want
   to show later, without committing to the renderer visualization now.
4. NN input vector — keep cadence7's 33 inputs, or revisit 021's "drop
   previous-command inputs" decision (which was predicated on ACRO
   being on)?
5. CRRCSim ACRO PID re-enablement: direct revert of 07c4832, or is
   there a cleaner landing given the 024 refactors intervened?

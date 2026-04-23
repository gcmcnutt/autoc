# 026 — NN Temporal State (non-bang-bang control)

**Status**: SKETCH — problem statement + candidate approaches. To be
specified and scoped before implementation.

**Relationship to 025**: 026 is a prerequisite for 025. Craft variations
(025's scope) under the current bang-bang policy mostly add noise to
selection; variations will select meaningfully for robustness only after
the NN has the structural capacity to produce smooth control.

## Problem statement

cadence7's trained NN, deployed on flight-20260422, produced clean
sim-to-real sign correlation on all control axes but collapsed to
full-throw bang-bang: pitch and roll outputs pile up at ±1, throttle
pinned at +1 ≥90% of the time. Mean `|Δout|` per tick ≈ 1.0; mean `|out|`
per tick ≈ 2.2 out of a 3.0 ceiling. These are essentially identical to
cadence7 *training* statistics — the NN is doing what it was trained to
do. See
[`specs/024-sim-real-fidelity/cadence7_bang_bang_evolution.md`](../024-sim-real-fidelity/cadence7_bang_bang_evolution.md)
and
[`flight-results/flight-20260422/FLIGHT_REPORT.md`](../../flight-results/flight-20260422/FLIGHT_REPORT.md)
for the empirical characterization.

**Root cause** (from the flight report's "What's next"): the NN sees
only *current-state* inputs — current target direction cosines (with
6-tap history via HIST_PAST), current quat, current gyro, current
airspeed, current distance. It has **no machinery to build an
integrator, no explicit time-history of its own outputs, no low-pass
filter structure**. A purely instantaneous-input MLP will, in the limit,
always find bang-bang as the optimum — because smooth control
requires internal state the NN cannot represent.

The 023-era training did feed prior-state control into the NN; that was
removed before 024. The weak downward dCtrl drift observed in cadence7
training (≈30% over gens 90–400) shows that selection already has some
appetite for smoothness — it's just too thin to overcome the immediate
gain of full-throw commands without structural help.

## Candidate approaches

Ordered by complexity / risk (lowest first). Only one or two should
land; the others inform 025 and beyond.

### A. Previous-output feedback (simplest)

Add `outPt_prev`, `outRl_prev`, `outTh_prev` (or a HIST_PAST-style window
of the NN's own recent outputs) to the NN input vector. Evolution can
learn to use them as low-pass state ("ouput close to last tick") or as
change-sensing ("act on difference").

- **Scope**: +3 inputs (or +3×N for N-deep history). MLP topology stays.
- **Training**: straightforward — one more tensor column into the input
  layer. Existing evolution operators work unchanged.
- **Deployment**: xiao already has `cached_*_cmd` state for the MSP
  heartbeat; feed those back into the next NN tick.
- **Risk**: NN may still find "ignore the history and go bang-bang"
  because it's allowed to. Evolution must *want* the smoothness — 025's
  craft variations may provide that pressure, but 026 alone may not
  show much improvement over 024 unless we also add something on
  fitness.
- **Was this tried pre-024?** Yes — 023-era training had prior-state
  control inputs; removed because interactions with lexicase weren't
  favorable. Revisit with the current clean baseline.

### B. Error integral as an explicit input

Add `∫(target - position)·dt` as an NN input, per axis. Direct analog
of PID's I-term — evolution gets a free "accumulated tracking error"
signal instead of having to build one.

- **Scope**: 3 new inputs (one per axis). Plus bookkeeping to maintain
  the integral across NN ticks, reset on engage start.
- **Training**: same as A plus per-scenario integral state.
- **Risk**: integral windup on bang-bang — if NN can't prevent
  integrator saturation, fitness tanks in long-duration paths. Needs an
  anti-windup bound. Ugly.

### C. Pre-defined filter node in the NN (architectural)

Add an explicit first-order low-pass (IIR) node with evolvable α to the
output stage. `out_smooth = α * out_raw + (1-α) * out_smooth_prev`. The
NN learns both what to command AND how smoothly to command it.

- **Scope**: architectural — node type added to the topology. GP-style
  trees handle this naturally; our fixed-topology MLP does not.
- **Training**: evolution now tunes α per output axis. Could also allow
  per-cell α to build more complex IIR networks.
- **Risk**: topology changes invalidate all prior weights. Full
  retrain. Scope creep into "arbitrary GP tree" territory.

### D. Recurrent NN (hidden state between ticks)

Actual RNN-style hidden vector that persists across NN ticks. LSTM /
GRU / simple RNN cell.

- **Scope**: significant — NN forward pass becomes stateful, xiao
  embedded side needs to carry hidden vector through `nn_program_generated`.
  Training needs to expose and initialize hidden state per scenario.
- **Training**: recurrence is known to be hard for evolution; gradient
  methods typically outperform. If we go this way it's a broader
  rethink.
- **Risk**: largest structural change. Probably not the right first
  step.

### E. Discourage bang-bang via fitness (no-tunables constraint)

Not a structural NN change, but a selection-side nudge. Mentioned in the
024 bang-bang note: autoc fitness doesn't use tunable coefficients, so
an explicit `|Δout|²` penalty with a weight is ruled out by project
rule. Two possible no-tunable forms:

- **Lexicase cases that require quietness** — synthetic "cruise"
  scenarios added to the per-individual case list where high dCtrl is
  penalized in the success/fail sense, not via a weight.
- **Pareto selection** on (tracking, control-effort) — a structural
  change to `SelectionMode` that keeps both "aggressive-and-accurate"
  and "quiet-and-moderate" champions in the population.

Most likely combines with A/B/C to provide the pressure that evolution
needs to *use* the new temporal machinery for smoothing.

## Validation strategy

For whichever approach lands, the measurement setup already exists:

1. Train to ≥ gen 400 on the new architecture.
2. Run [`plot_control_aggressiveness.py`](../024-sim-real-fidelity/plot_control_aggressiveness.py)
   on the training `data.dat`. Compare the dCtrl / |out| plateau to
   cadence7 (1.5 / 2.2). Movement off that point in either direction is
   meaningful.
3. Target regime (from the cadence7 2×2 table): **high dCtrl + low
   |out|** = "fine-grained tracking". If the plateau moves toward (>1,
   <1.5), we've found smoother control.
4. Eval suite passes.
5. Flight-test. Run `plot_bangbang_flight.py` on the xiao log —
   histograms of outputs should spread from ±1 extremes toward the
   middle.

## What this spec is NOT

- A commitment to any specific approach. Pick one or two from A–E and
  draft a proper plan.
- An implementation schedule. 026 inherits the "no tunables" project
  constraint; approach B is suspect on that ground alone.
- A replacement for 025. 025 (craft variations) lines up right after
  026 lands with a smooth baseline; the rudder-moment calibration work
  proposed there remains.

## Open questions

1. Do we re-use cadence7's topology (33 → 32 → 16 → 3) with added
   inputs, or is this the moment to revisit topology (maybe a small
   dedicated "feedback stage" hidden layer)?
2. Xiao carries stateful NN execution if approach D is chosen — any
   restriction from the `nn_program_generated.cpp` generator?
3. The 024 cadence-fix guardrails (deterministic training, frame-
   counter cadence) all carry forward. Anything else to preserve?
4. How much of the existing eval-suite tier 0 stays valid when the
   input layout changes? Presumably tier 0 is just "bitwise match a
   frozen reference", regenerate after the change.

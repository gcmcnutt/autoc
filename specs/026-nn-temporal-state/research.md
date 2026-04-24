# 026 Research — Temporal state, ACRO delegation, and INAV filters

Companion to [`spec.md`](./spec.md). Captures:

1. Git history of rate-mode / PID / filter experiments (021 ACRO, 023 pt3).
2. Structural analysis: does ACRO delegation settle bang-bang or displace it?
3. INAV filter inventory on the current bench/flight FC — which to model in sim.
4. Objective-function options under the "no tunable coefficients" project rule.
5. Concrete candidate experiments.

## 1. History — rate mode was tried, then deferred

### 021 — ACRO rate PID (crrcsim side)

crrcsim commits 9809dd6 (Mar 31) added a rate PID that mapped NN output to
desired body rate via INAV-style FF/P/I, with `ACRO_MAX_RATE_ROLL/PITCH/YAW`
sourced from INAV config. Commit 07c4832 (Apr 2) reverted the PID, leaving
the *constants* in the header as a bookmark. Commit cf8d406 (Apr 2) updated
those constants to flight-measured rate limits (430/300/180 °/s) and
re-calibrated PID_SCALE=350 — still "unused, kept for reference."

autoc-side, the architectural decision was captured on 2026-04-07 in
[`specs/021-xiao-ahrs-crosscheck/tasks.md`](../021-xiao-ahrs-crosscheck/tasks.md)
Phase 2:

> **ARCHITECTURAL DECISION (2026-04-07)**: Rate PID is intentionally
> disabled. The NN receives gyro rates as inputs (p, q, r at indices 24–26
> of the 27-input vector) and learns its own rate-control strategy. NN
> outputs map directly to surface deflections (MANUAL mode in INAV). The
> rate PID infrastructure exists in code but is not active.

**The reasoning as best as the record preserves it**: 022's betterz2 run
(400 gens, V4 conical, best −34771) produced a usable controller using
*gyro inputs + direct servo outputs* — so the PID intermediate layer was
deemed unnecessary. The NN was learning its own damping via the gyro
inputs, and going through a second loop was thought to add sim-to-real
coupling without a clear training win.

What the 021 design documented *in favor* of ACRO (from
[`021-xiao-ahrs-crosscheck/spec.md`](../021-xiao-ahrs-crosscheck/spec.md#L113-L131)
and [`data-model.md`](../021-xiao-ahrs-crosscheck/data-model.md#L60-L73)):

- NN output semantics clean: "what rate do I want?" instead of "what
  stick position?"
- **Natural zero**: in ACRO, output = 0 means "hold current attitude."
  In MANUAL, output = 0 means "servos centered" — the aircraft does
  whatever physics dictates, which depends on trim + wind + coupling.
- Sim-to-real gap shrinks to the rate-tracking problem (small) instead
  of the full aero response problem (large).
- Coupling (roll/pitch/yaw cross terms) handled by INAV PID at 1 kHz
  rather than the NN at 10 Hz.
- Speed-regime robustness: INAV PID adapts to dynamic pressure; same
  stick = same rate across the envelope.

### 023 — pt3 RC smoothing filter (ABANDONED)

crrcsim commit 9032e56 (Apr 13) added INAV's `rc_smoothing.c` pt3 filter
(3rd-order cascade Butterworth) on NN outputs. Commit 435ad94 (same day)
reverted with an explicit finding logged in
[`specs/BACKLOG.md`](../BACKLOG.md):

> **test5 (10 kHz passthrough)**: confirmed filter code path is
> deterministic and doesn't degrade convergence when effectively
> disabled.
> **test6 (40 Hz cutoff)**: training stunted — best stuck at −2225
> through 55 gens vs test4's −4410 at the same point. Avg fitness ~40%
> lower, pctInStreak 3% vs 12%. The filter changes dynamics enough that
> the GA can't find productive policies.
> **20 Hz also tried**: even worse stunting.

The interpretation: a *pre-filter* between NN output and servo surface
mechanically prevents the quick corrections the NN needs during early
evolution, so selection starves. "Smoothness must come from fitness-based
incentives."

### What's changed since 2026-04-07 that makes this worth re-opening

1. **024 cadence fix**: sim cadence is now exactly 100 ms (cadence7
   confirmed). 023-era runs had the 117 ms cadence bug, and 021's ACRO
   PID ran at 39 ms outer frames. Comparative fitness numbers across
   the eras are noisy.
2. **024 coordinate conventions**: gyro/quat/accel signs now verified
   clean end-to-end
   ([`docs/COORDINATE_CONVENTIONS.md`](../../docs/COORDINATE_CONVENTIONS.md),
   `sensor_self_check.py` 7 of 8 PASS). 021's ACRO PID had to share the
   debugging surface with convention-uncertainty; that confound is gone.
3. **AHRS validated on flight**: the gravity-projection check in
   [`specs/024-sim-real-fidelity/plot_gravity_check.py`](../024-sim-real-fidelity/plot_gravity_check.py)
   confirms AHRS tracks pure-gyro integration over 18 s without
   systematic drift. Rate-mode control depends on accurate rate sensing
   — 024 showed that's in hand.
4. **cadence7 demonstrated the ceiling**: NN learns "its own rate
   control" exactly as the 2026-04-07 decision predicted, and that
   control is **full-throw bang-bang**. `|out|` ~2.2 / 3.0, `|Δout|`
   ~1.0 / tick, throttle pinned at +1. Trained policy is doing what it
   was trained to do; the issue is that the architecture cannot express
   smoother control.

So: the 2026-04-07 call was right *at the time*, given 022's evidence.
cadence7 is the evidence that shifts the call.

## 2. Will ACRO settle bang-bang, or just displace it?

Short answer: **displace it significantly, but not eliminate it alone.**
ACRO is a structural improvement; to fully settle bang-bang it wants a
selection-side complement.

### What ACRO does mechanically

With NN outputting desired body rates and INAV's PID tracking them:

- **Surface-level chatter drops**. INAV PID integrator + pt3 filter on
  rate error keeps the control surface moving smoothly to achieve the
  commanded rate. Even if NN flips rate-cmd between ticks, the surface
  doesn't thrash at the servo — it rate-limits through the PID
  response. Servo life + battery draw visibly improve.
- **Aircraft motion amplitude is rate-limit-clipped**. At max rate
  ~430 °/s roll and 100 ms tick boundary, the worst-case per-tick
  rotation is ~43°. That is aerobatic but not uncontrolled-departure.
- **"Natural zero" appears**. NN rate=0 → aircraft holds attitude
  (PID integrator holds surface against wind/coupling). In MANUAL,
  stick=0 ≠ hold. Evolution gets a default it can do for free.

### What ACRO does NOT do

If the NN still commands bang-bang rates (`±1` alternating), the
aircraft still gets yanked around — just between commanded rate extremes
instead of between commanded surface extremes. Sustained visual
oscillation remains. PID smooths the *transition between* bang-bang
setpoints; it doesn't refuse to follow the setpoint.

### Why ACRO still helps even if the NN bangs rates

Under path-tracking fitness:

- **MANUAL bang-bang**: surface +0.5 → aircraft rolls some amount
  depending on dynamic pressure, airspeed, wind. Rate is NOT consistent.
  NN can't predict "how much roll will I get" — it has to re-evaluate
  each tick.
- **ACRO bang-bang**: NN says "rate = +1" → aircraft rolls at ~430 °/s
  regardless of speed regime. Consistent.

The consistent rate mapping means the fitness landscape is *smoother
under ACRO* — a small input change produces a predictable output change
— which typically selects against bang-bang faster, because tiny
corrections start being reliably meaningful. Under MANUAL, "a slightly
smaller input" might produce a less-predictable response, so "just go
full throw" is a robust-if-crude strategy.

Plus the natural-zero effect: if the NN ever outputs 0 by accident, it
doesn't *lose* fitness (aircraft stays stable); in MANUAL, 0 output is a
distinct failure mode. Selection is kinder to moderate outputs under
ACRO than under MANUAL.

**Net**: ACRO alone probably brings bang-bang amplitude down (not to
zero) and makes selection for smoother control more efficient. Combined
with a fitness-side nudge (section 4), there's a plausible path to
regime-2 (fine-grained tracking).

### Counter-evidence to watch for

The 021 ACRO PID was tried briefly and the run didn't reveal improvement;
it was reverted on "022 did fine without it" grounds. If we re-run with
ACRO, we need to actually *measure* — fitness curve, `|out|` plateau,
`|Δout|` plateau, `plot_control_aggressiveness.py` comparison to cadence7.
The tools are in place.

## 3. INAV filter inventory (bench config, `xiao/inav-bench.cfg`)

| Filter | Value | What it does | Sim-matching relevance |
|---|---|---|---|
| `gyro_anti_aliasing_lpf_hz` | 250 | Anti-alias before downsampling | Transparent to NN — sim gyro is FDM-clean |
| `gyro_main_lpf_hz` | 25 | Main gyro LPF (what NN sees via MSP) | **Matters for sim** — NN gyro inputs should pass through a similar LPF in sim if we want sim noise characteristics to approximate flight |
| `gyro_filter_mode` | STATIC | No dynamic LPF scaling | Clean |
| `dynamic_gyro_notch_enabled` | ON | Vibration notch, adapts to prop RPM | Cancels prop-vibration noise in gyro; sim has no vibration to model |
| `gyro_adaptive_filter_*` | ON | Extra adaptive noise rejection | Same — vibration-targeted |
| `acc_lpf_hz` | 15 | Accel LPF (NOT NN input today, but used by INAV AHRS) | AHRS-internal; sim AHRS uses clean accel |
| `rc_filter_lpf_hz` | 50 | Filter on RC input (NN commands via MSP) | **Potentially matters** — at 50 Hz LPF and 10 Hz NN, smoothing is minimal. Different from the 20/40 Hz pt3 that stunted training. |
| `dterm_lpf_hz` | 10 | PID D-term LPF | Only active if ACRO is used |
| `servo_lpf_hz` | 0 | Off — no servo rate filter | Sim matches |

**What to consider modeling in sim for 026**:

- **If ACRO is adopted**: sim needs an equivalent rate PID + optional
  D-term LPF. The 021 code is the template. Gains come from the 021
  header tuning (not INAV's numbers); filter cutoffs picked to match
  sim's own cadence rather than copying INAV (see "Sim PID
  implementation notes" below).
- **rc_filter at 50 Hz**: mostly benign at the 10 Hz NN command rate
  (5× above) — can skip in sim. If the bench T210 shows rate-tracking
  lag we don't explain, revisit.
- **Gyro LPF in sim**: worthwhile as part of the ACRO inner loop. Sim
  cutoff **40 Hz** (not 25) — picked for sim cadence, see below.

**What to NOT model**:

- Notches and adaptive filters — they target prop vibration that doesn't
  exist in sim. Model them and you introduce a sim-side noise that isn't
  really there.
- `acc_lpf_hz` 15 — the NN doesn't consume accel today; skip.

### Sim PID implementation notes

Same algorithm structure as INAV, sim-specific gains, sim runs at FDM
rate — but the filter cutoffs pick "round numbers" that line up with
the sim's own cadence rather than inheriting INAV's values verbatim.

#### Algorithm

Discrete-time PID per axis (pitch and roll; yaw passive per
clarification Q3):

```
error = setpoint_rate - filtered_measured_rate
iterm = clamp(iterm + error·dt, ±I_MAX)
out   = (FF·setpoint_rate + P·error + I·iterm) / PID_SCALE
out   = clamp(out, -1, +1)   # surface deflection normalized
```

- FF + P + I + integrator anti-windup clamp (±10 rad, matches 021's
  preserved design).
- D gain stays 0 initially (021 code had it zero; INAV's flight config
  has `fw_d_pitch=5, fw_d_roll=7, fw_d_yaw=0` — nonzero but small).
  Add D only if rate-tracking rise time during bench
  characterization (T210) undershoots.
- Pre-filter: 1-pole LPF on measured body rate before error compute.
- Post-filter: PT2 (2nd-order) LPF on the D-term output if D is
  enabled. Dormant for now.

#### Gain numbers

INAV ships gains in its internal units (deci-degree-per-second PID math
scaled to PWM range). Sim works in rad/s natively. **These aren't
directly comparable**. The `ACRO_FF/P/I/PID_SCALE` constants in
[`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h#L68-L90)
were tuned empirically for CRRCSim FDM response, not copied from INAV.

021-era gains: `FF=50, P=40, I=15, PID_SCALE=350`. Tuned at
`Global::dt = 0.003` (333 Hz). Under 024's `dt = 0.005` (200 Hz) these
likely want a mild re-tune — the integrator accumulates at a different
rate per tick. Plan.md Phase 1.9 / T015 smoke test exercises this;
T210 bench characterization is the final sim-to-real match.

**Match criterion**: rate step response shape — rise time, overshoot,
settling. Not gain numbers. If sim step response diverges > 30 % from
the flight FC's bench-measured response, re-tune the compile-time
constants and re-smoke; do NOT retrain from this.

#### Loop rate

- Sim PID runs at the FDM tick rate: **200 Hz** (dt = 0.005).
- INAV PID runs at its configured `looptime`, typically **1 kHz**.
- 5× rate difference. **Not a correctness problem**: noise-free FDM
  doesn't need the faster loop, HB1's natural frequencies are well
  below 100 Hz, and we're training a 10 Hz planner — 200 Hz is ample
  margin above everything that matters.

#### Filter cutoffs — re-derived for sim cadence

INAV's bench config carries values (`gyro_main_lpf_hz = 25`,
`dterm_lpf_hz = 10`) that were tuned historically for MEMS gyro noise
on 1 kHz PID loops. The sim has a different noise profile (essentially
none) and different rate structure. Instead of inheriting INAV's
numbers, sim picks cutoffs that match its own cadence:

Sim cadence:
- FDM tick: **200 Hz** (dt = 0.005).
- Outer cycleLength: **20 Hz** (50 ms, framesPerEval = 2).
- NN command rate: **10 Hz** (100 ms).

Chosen sim filter cutoffs:
- **Gyro LPF: 40 Hz** (2× outer frame, 4× NN rate, 1/5 FDM). Light
  filtering — enough to smooth any numerical FDM noise without masking
  real body-rate dynamics. The 023 pt3 experiment (ABANDONED) was
  cutting at 20-40 Hz on the *command* path — below 4× the NN rate —
  which is why it stunted training. 40 Hz on the *feedback* path is
  different: it filters noise, not commands.
- **D-term LPF: 20 Hz** (matches outer frame). Keeps D from
  reacting to sub-outer-tick numerical jitter. Dormant unless D gain
  is later enabled.

Both are configured as compile-time constants in
`inputdev_autoc.h` next to `ACRO_MAX_RATE_*`:

```cpp
#define ACRO_GYRO_LPF_HZ   40.0   // 2× outer frame, 4× NN rate
#define ACRO_DTERM_LPF_HZ  20.0   // outer frame rate
```

Discrete α computed at sim init from cutoff + `Global::dt`:

```
α_single_pole = exp(-2π · fc · dt)
```

At `dt = 0.005, fc = 40`: α ≈ `exp(-2π·40·0.005) = exp(-1.257) ≈ 0.285`.
So `y = 0.285·y_prev + 0.715·x_new`. Very light filtering, preserves
signal.

At `dt = 0.005, fc = 20` (D-term): α ≈ `exp(-0.628) ≈ 0.534`. Heavier.

#### Context — cadence7 shipped without any of this

cadence7 trained stably and flew controllably on
MANUAL-mode-direct-servo + no inner filters. The ACRO PID + LPFs
we're adding in 026 are about giving the NN's action space a
different semantic (rate instead of deflection) and delegating the
stabilization loop to a physics-correct PID rather than requiring the
NN to learn one implicitly. Starting values come from 021-era
empirical tuning; they're a sensible baseline, not a final answer.
T210 bench characterization verifies before flight.

#### What we're NOT trying to match

- INAV's gain numbers.
- INAV's PID loop rate.
- Every nuance of INAV's PID (iterm lock, iterm throw limit, Smith
  predictor, TPA). Those tune around sensor noise and airframe-specific
  quirks on the FC; sim's noise-free FDM doesn't need them.
- INAV's filter cutoffs as-is — sim picks rate-appropriate cutoffs.

#### What we ARE trying to match

- Rate step response *shape* on a commanded-rate step input.
- "Natural zero" semantic: rate = 0 → aircraft holds attitude.
- Max achievable rates: `ACRO_MAX_RATE_*` in the header already match
  flight-measured peaks (430 / 300 / 180 °/s roll/pitch/yaw).

## 4. Objective-function options under the "no tunables" constraint

Explicit coefficient-weighted penalties (e.g. `−k·|Δout|²`) are ruled
out by project rule. Valid options:

### (E1) Structural selection change — Pareto or tournament on `(tracking, effort)`

Switch `SelectionMode` from `lexicase` to a Pareto front on two
objectives: path-tracking (current fitness) and control effort (e.g.
sum of `|out|` or `|Δout|` over the scenario). Non-dominated individuals
survive, keeping both "aggressive-and-accurate" and
"moderate-and-moderately-accurate" champions in the gene pool.

- Pro: no tunables. Coefficients replaced by structure.
- Pro: preserves the "aggressive is fine when it's the best tracker"
  regime.
- Con: biggest architectural change to training code. Pareto on
  fitness-per-scenario (vs aggregate) gets expensive with 245
  scenarios.

### (E2) Lexicase cases that implicitly reward quietness

Add synthetic "cruise"/"hold" test cases to the per-individual case
list — straight legs, low wind, where the objectively best policy is
to hold attitude and throttle steady. Lexicase's case-by-case
elimination then naturally favors NNs that *can* hold steady. No
explicit penalty term.

- Pro: minimal code change — just add scenarios.
- Pro: no tunable.
- Con: only works if we have a scenario where quiet is *objectively*
  correct. Current rabbit-chase fitness doesn't quite make that — even
  on a straight leg the rabbit is moving and some pitch/roll/throttle
  adjustment is always needed. Might need a new scenario type
  ("station keeping" with no rabbit) with a separate fitness.

### (E3) Let ACRO's natural zero + Pareto do the work

The most elegant option: ACRO makes "rate=0" a free default, lexicase
on rabbit tracking already exists, and if Pareto is added on top (E1),
individuals who discover "rate=0 works most of the time" gain a
distinct axis. No special scenarios needed; the reward for quietness
comes out of the lexicase tracking objective itself because rate=0
during steady flight *actually does* produce better tracking under
ACRO than bang-bang does.

- Pro: combines structural and selection benefits.
- Pro: validates cleanly — if it works, dCtrl plateau drops measurably
  on the same tracking fitness.
- Con: depends on E1's engineering cost.

### (E4) Just ACRO, no selection change — measure first

Run ACRO with the existing lexicase + tracking fitness. If the
"consistent rate mapping makes the landscape smoother" argument is
right, training will naturally find lower-amplitude policies even
without explicit pressure. If not, E2 or E1 come next.

- Pro: cheapest experiment.
- Pro: generates direct data on whether structural change alone is
  enough.
- Con: risks looking like 021 all over again if we don't measure well.
  Mitigation: `plot_control_aggressiveness.py` and
  `plot_bangbang_flight.py` make "before/after" easy to show.

## 5. Candidate experiments, ranked cheapest first

### Experiment 1: ACRO-sim + gyro-LPF, same training setup

**Implementation sketch**:

- Re-enable the 021 ACRO PID code in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`.
  The block is preserved in history at 9809dd6; diff it back in.
- Update the rate limits to the current `ACRO_MAX_RATE_*` constants
  (already in the header from cf8d406) which track the flight-measured
  values.
- Add a **40 Hz** single-pole LPF on the body-rate values before the
  PID error computation. Cutoff picked to match sim cadence (2×
  outer-frame rate, 4× NN rate), not copied from INAV's 25 Hz (which
  was tuned for MEMS gyro noise at 1 kHz PID loops). See "Sim PID
  implementation notes" above.
- D-term LPF: **20 Hz** PT2 (outer-frame rate). Dormant unless D gain
  is enabled later.
- Skip modeling INAV's 50 Hz `rc_filter_lpf_hz` in sim — at the 10 Hz
  NN command rate a 50 Hz filter is benign; if T210 rate-tracking
  shows a lag we don't explain, revisit.
- Xiao side: flip `CH6` override from MANUAL (1000) to ACRO
  (1500). INAV does the rate PID work.

**Training**:

- Same topology, same fitness, same everything else. Retrain 400 gens.
- Call it cadence8 or rate1.

**Validation**:

- `plot_fitness_ramp.py` — does it reach similar best fitness?
- `plot_control_aggressiveness.py` on the new data.dat — does the
  dCtrl/|out| plateau drop below cadence7's (1.0, 2.2)?
- `plot_bangbang_flight.py` on a tier1 eval — do the output histograms
  spread away from ±1?

**Go/no-go**: if dCtrl plateau drops >20% with tracking fitness
≥ cadence7 × 0.7, ACRO is working and we move toward flight. If not,
onto experiment 2.

### Experiment 2: Add (A) previous-output feedback inputs

Only if Experiment 1 doesn't hit the smoothness threshold. Adds 3 (or
3×N) more inputs — previous NN outputs. Tests whether the NN uses them
given ACRO already in place.

Same measurement tools; compare to cadence8.

### Experiment 3: Pareto selection (E1)

Structural change to the training loop. Largest cost. Reserved for when
Experiments 1 and 2 both fall short.

## 6. Open questions for the spec

1. Do we do Experiment 1 in the sim-only first, or wire the xiao ACRO
   flip at the same time so retrain → flash → bench → fly is one clean
   cycle?
2. Current NN outputs are 3 (pitch, roll, throttle). Do we add rudder
   when going to ACRO? The HB1 has rudder authority; leaving it at 0
   discards a control channel. Small scope expansion but physically
   meaningful.
3. If Experiment 1 helps, does the HB1 rudder under-modeling (025
   rudder-moment calibration) become more or less important? My hunch:
   *more*, because ACRO will reveal that sim-side yaw authority is off.
4. Is there appetite for E2 (lexicase quiet-scenario) as a hedge on
   Experiment 1, running in parallel? Slightly more training cost per
   generation but more informative if Experiment 1 is close.

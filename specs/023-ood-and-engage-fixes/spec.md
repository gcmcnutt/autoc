# 023 — OOD Coverage, Engage Transient, Throttle Discipline

**Status**: Draft
**Priority**: P0 — blocking next flight test
**Created**: 2026-04-07
**Predecessor**: [022-tracking-cone-fitness](../022-tracking-cone-fitness/spec.md)

## Clarifications

### Session 2026-04-08

- Q: Engage transient fix — which reset approach? → A: Option A (pre-fill history slots with current geometry on every engage transition)
- Q: INAV engage delay modeling — which approach? → A: Option A (simulate delay in sim pipeline). During the ~0.75s window, CRRCSim applies centered stick (zero pitch/roll/throttle command) — craft flies open-loop under FDM + wind, no autopilot. NN runs and updates its history buffer every tick, but its outputs are ignored by the sim bridge until the window expires.
- Q: Type-safe NN sensor interface — which approach? → A: Option A (struct-of-floats with named fields). Memory layout identical to `float[NN_INPUT_COUNT]` via `static_assert`; compiler flags every site that adds/removes/reorders fields. Worth the 1.5–2x migration cost vs enum indexing for permanent protection against the 021-style silent serialization corruption bug class.
- Q: Authority-limit iteration (Change 8) — when to trigger Phase 3c retrain? → A: Deferred to plan phase. Determine the trigger criterion empirically after seeing Phase 3b baseline behavior under the 3-cosine representation. The 3-cosine change is the primary intervention; authority limiting is a conditional follow-up whose threshold should be set from observed data, not guessed.
- Q: Streak threshold ramp — include in 023? → A: No. Leave parked in BACKLOG per 022 verdict (betterz2 converged without it). Revisit only if 023 Phase 3a shows an early-generation "no streak signal" problem under the new 3-cosine representation.

## Problem

Flight 2026-04-07 (first 022-trained flight) showed control intent but failed
to track. Analysis in [flight-report.md](../../flight-results/flight-20260407/flight-report.md)
identified three distinct root causes that compound:

### 1. Stale History on Re-Engage (CRITICAL)

Checking all 4 autoc engages in the 2026-04-07 flight:

```
Flight 1 Engage 1 (first):    CLEAN (dist-9=1.3, dd/dt=+0.0)
Flight 1 Engage 2 (re-engage): STALE (dist-9=104.9, dd/dt=+1097)
Flight 2 Engage 1 (first):    CLEAN
Flight 2 Engage 2 (re-engage): STALE (dist-9=122.6, dd/dt=+1142)
```

**Pattern**: History is reset on the **first** autoc engage per flight,
but NOT on subsequent re-engages. The temporal buffers retain stale values
from the previous disengage state.

On re-engage:
- `dist` history: [122, 118, 115, 1.3] — step function with garbage history
- Closing rate `dd/dt`: **+1142 m/s** (computed from stale prev − fresh now)
- Stays corrupted for ~9 samples (900ms) until buffer cycles through

### 1b. INAV Engage Delay (~750ms)

Independent of the history bug, INAV has a ~750ms delay between "NN
switch enabled" and "MANUAL mode active". During that window:
- NN is computing outputs
- History buffer is being filled with in-flight data
- But commands are NOT yet reaching servos (still in ACRO stabilized)
- Aircraft continues on previous trajectory

**Interaction with training**:
- Current training assumes NN commands take effect immediately
- Real flight: aircraft has ~750ms of free flight after "engage"
- Training with `EntryPositionRadiusSigma=15` puts NN at 15m offset
- Real flight: that 15m training sample does NOT match real condition where
  the aircraft has been drifting for 750ms before commands matter
- Training should either:
  - Simulate the INAV delay (feed NN inputs but ignore outputs for 750ms)
  - Or accept that real entry conditions have wider variance than trained

**Net effect of 1+2**: on re-engages, aircraft gets garbage inputs for the
entire window where commands would otherwise start mattering.

### 2. Out-of-Distribution Inputs After Failed Intercept

Comparing sim (gen 400) training distribution to real flight NN inputs:

| Input | Sim median | Real median | % Real outside sim p1-p99 |
|-------|-----------|-------------|---------------------------|
| dist (m) | 4.4 | 56.5 | **67.9%** |
| dd/dt | +0.4 | -7.2 | 15.4% |
| dPhi (rad) | 0.055 | 0.093 | 38.6% |
| dTheta (rad) | 0.407 | 0.444 | 41.1% |
| vel (m/s) | 14.7 | 16.8 | 14.4% |

Sim p99 distance is only 27m — the NN has no training at 30m+. Once the real
aircraft falls behind the rabbit (which it does in <3 seconds due to the
engage transient), it's operating in a regime the NN never saw. The policy
has no "recover from far away" skill because closed-loop training never
produced that state.

### 3. Throttle Saturation (even in-distribution)

Sim median throttle output is +0.976 — the NN mostly commands full throttle.
- Sim: 55.5% of samples > 0.9 (near full)
- Real: 97-99% > 0.9

Point-accumulation fitness rewards path closure and streak accumulation.
Sim drag caps speed at ~18 m/s regardless, so the NN has no incentive to
modulate throttle. On real hardware with slightly more thrust headroom, the
same policy runs the aircraft ~30% faster than trained.

## Goals

1. **Eliminate the engage transient bug** — NN sees clean history from first
   sample after engage.
2. **Expand training distribution to match real flight conditions** — NN
   sees 30-50m distances, diverging closing rates, extreme bearings.
3. **Add throttle discipline** — NN learns to modulate power instead of
   defaulting to full.
4. **Verify real-sim dynamics match** — reconcile real Vmax ~25 m/s with
   sim Vmax ~18 m/s (though wind-corrected comparison shows this gap is
   smaller than first thought).

## Scope

### In Scope

**Prerequisites (blocking, must land before main work):**
- Type-safe NN sensor interface (compiler-enforced topology consistency — enabler
  for the 3-cosine bearing representation, and protection against future silent
  topology drift). Historically BACKLOG item; promoted to 023 hard prerequisite.
- Train/eval scenario construction de-duplication refactor (NOT a one-line fix).
  BACKLOG enumerates three eval fitness bugs (Bug 1 metric drift, Bug 2 stale
  S3 fitness, Bug 3 missing rabbit speed config); all three are symptoms of a
  single root cause: training and eval paths have drifted copies of the same
  scenario setup code. The fix is a refactor that extracts scenario construction
  into a shared helper used by both paths. Acceptance: eval on saved betterz2
  weights reproduces the training-time fitness within FP rounding. Without this
  determinism cross-check, every 023 Phase 1 benchmark is contestable.

**Plan-phase research (COMPLETE 2026-04-08):**

- **INAV signal path audit** (`docs/inav-signal-path-audit.md`) — deep read of
  `~/inav` source. Audited every filter, delay, smoothing, rate limit, LPF,
  notch, and latency-inducing stage between xiao MSP input and servo output,
  and between raw gyro and xiao MSP gyro input (which is actually
  `MSP2_INAV_LOCAL_STATE` — `MSP2_GYRO` does not exist as a separate endpoint
  in this fork; gyro is piggybacked). **Top findings:**
  1. **`gyro_main_lpf_hz = 25` is the dominant NN-input delay** — PT1 @ 25 Hz
     on the gyro output, ~6.4 ms group delay. Baked into what xiao reads via
     `MSP2_INAV_LOCAL_STATE`. Default is 60 Hz.
  2. **`setpoint_kalman_enabled = ON` is a misnomer** — despite the CLI name,
     it is a gyro measurement Kalman stacked on top of the 25 Hz PT1.
     Adaptive, state-dependent delay. Should be OFF.
  3. **`rc_filter_lpf_hz = 250` is in the MANUAL-mode path** — PT3 applied to
     rcCommand before the mixer, NOT gated by flight mode. ~1.9 ms delay at
     250 Hz, would be ~9.5 ms at the 50 Hz default. Another servo_lpf-class
     surprise.
  4. **Servo LPF confirmed OFF** — `servo_lpf_hz = 0` already applied.
  5. **TASK_SERIAL runs at 100 Hz LOW priority, processes one MSP command
     per invocation** — up to 10 ms MSP ingress jitter under load.
- **Measured MSP2 round-trip latency** (from xiao blackbox logs):
  **median ~23 ms, max 42 ms** (xiao-observed fetch+eval+send), plus 0-15 ms
  INAV-internal post-receive delay, plus up to 20 ms PWM quantization →
  ~25 ms typical / ~55 ms worst-case end-to-end.
- **750 ms engage delay root cause: PROBABLE but unconfirmed** —
  `failsafe_recovery_delay = 5` × 100 ms + `PERIOD_RXDATA_RECOVERY` in
  `src/main/rx/msp_override.c:103-104` is the leading candidate. Needs a
  single-value bench test (`set failsafe_recovery_delay = 0` and re-measure)
  to confirm. Fallback hypothesis: xiao-side delay between enabling
  `MSPRCOVERRIDE` AUX and enabling `MANUAL` AUX (out of scope for the audit).
  Runs as a Phase 1 bench test during implementation.
- **INAV escape-hatch architectures** — audit concludes config changes alone
  should remove most reducible delay; escape-hatches are NOT required for
  023. Three 024+ candidates documented in the audit doc:
  direct xiao→servo PWM/PPM, drop GPS during autoc spans, or higher-rate
  NN on xiao with IMU-only inputs. None are 023 deliverables.
- **Train/eval code duplication survey**
  (`specs/023-ood-and-engage-fixes/train-eval-code-dedup.md`) — found **6
  divergences** between `runNNEvaluation()` and the training path:
  - **Bug 1** (metric drift): already resolved in 022
  - **Bug 2** (stale S3 fitness): eval copies original NN01 bytes into
    `evalResults.gp` at `src/autoc.cc:823-824` without rewriting
    `genome.fitness`. Fix: assignment before `nn_serialize`.
  - **Bug 3** (missing rabbit speed config): `src/autoc.cc:751` never
    assigns `evalData.rabbitSpeedConfig`, inherits `{16.0, 0.0, ...}` default.
  - **Bug 4 NEW** (`isEliteReeval` lies): eval hard-codes `false`; should
    be `true` so deterministic logging matches training elite re-eval.
  - **Bug 5 NEW** (scenario selection asymmetry): eval hard-codes
    `scenarioForIndex(0)`. In demetic mode this silently drops N-1 path
    variants. Fix: iterate all scenarios, match training aggregation.
  - **Bug 6 NEW** (triple-duplication): three near-identical copies of the
    scenario-list populator in `src/autoc.cc` (per-individual L932-975,
    elite re-eval L1021-1060, eval L751-787). Root cause of the whole bug
    class. Fix: extract `buildEvalData(EvalJob)` helper, delete all three
    copies.
- **Phase 0b refactor acceptance criterion**: eval on saved betterz2 weights
  (gen 400, fitness −34771) reproduces the training-time fitness within FP
  rounding. This is the first time the eval path will have been correct
  since the codebase existed, because Bug 3 alone (constant-16 rabbit
  instead of configured 13±2) has been corrupting every eval-mode run.

**Diagnostic Phase (happens after prereqs, during implementation):**
- Eval-mode random-path benchmark (depends on Phase 0b determinism fix)
- Engage transient fix (Change 1) — can be tested against current 022 NN
  before any retraining lands

**Core training changes:**
- Engage transient fix (history buffer seeding or continuous recording)
- 3-cosine bearing representation (replace dPhi/dTheta with direction cosines —
  structural elimination of atan2 wrapping; requires type-safe NN prerequisite)
- Discontinuity-forcing training paths (purpose-built paths that place the
  aircraft in target-behind / target-below / inverted regimes to exercise the
  bearing representation during training; complements the 3-cosine change)
- Training regime expansion (position, speed, attitude variations)
- Throttle lexicase dimension (was deferred from 022)
- Energy / authority-limit lexicase dimension (NEW — 10Hz overcorrection hypothesis:
  craft tendency to overcorrect may benefit from limiting command throw to ~50%
  and retraining. Iterative: train, examine learned behavior, then constrain
  authority and retrain. More likely to give a signal than a pure energy penalty.)
- Rabbit speed simplification (consider fixing 13 m/s)

**Robustness Phase:**
- Craft parameter variations (pulled in from 015 T140–T143: control effectiveness,
  damping, drag, mass/inertia, control phase delay ± stddev per scenario)
- Sim dynamics tuning if necessary after training changes

**Validation:**
- Next-flight blackbox analysis: structural check that projection discontinuities
  in the bearing inputs are **zero** (not just reduced) — the 3-cosine change must
  make wrapping impossible, and real flight must confirm it.

## Open Research Areas (notes for 023 planning)

### Eval-Mode Discovery: Random-Path Recovery IS Hard Even in Sim

When evaluating trained NN with random paths (SeededRandomB), results show
routine hard-lost-track-re-engage failures. This matches what we're seeing
on real flight. **So the simulation DOES reproduce the failure mode** when
it runs paths outside the training distribution.

**Implication**: the gap isn't necessarily sim-to-real — the NN is brittle
in sim too when fed OOD geometry. Training with only the 5 deterministic
paths produced a policy that works on those specific geometries but not on
novel ones (or recovery scenarios).

**Hypothesis**: the real flight failure is less about dynamics mismatch and
more about the policy being narrow. A policy that works on random paths in
sim will likely work on real hardware (assuming dynamics are roughly right).

**Action**: benchmark current NN on random-path eval, measure the same
metrics (streak, distance distribution). Compare to the real flight metrics.
If they match, the gap is policy brittleness, not sim-to-real.

### Craft Parameter Variations (Robustness Training)

**Proposal**: apply stddev-level variations to aircraft dynamics parameters
during training to force policy robustness. Not perfect modeling — just
"the response should be in the ballpark, not exact."

Candidate parameters to vary per-scenario (Gaussian around nominal):
- **Control effectiveness**: Cl_da, Cm_de ± 10-20%
- **Damping**: Cl_p, Cm_q ± 10-20%
- **Drag**: CD_prof ± 10%
- **Mass and inertia**: ± 5%
- **Control phase delay**: 0-200ms between command and servo response
- **Gyro noise / bias**: small amount

A NN that tracks across this variation envelope is robust to the sim-to-real
dynamics gap. The current NN only ever saw the exact hb1_streamer.xml
dynamics, so it may have overfit to those specific parameter values.

**Caveat**: more variation = slower evolution. Start with one or two
parameters (control effectiveness + phase delay) before going wider.

### Phase Delay Research

Sources of phase delay not currently in sim:
1. **10Hz sample rate** — already in sim (100ms step). Not a "delay" per se,
   but a minimum loop latency.
2. **INAV servo LPF** — confirmed OFF (`servo_lpf_hz=0`) for 2026-04-07 flight.
3. **MSP communication latency** — xiao→INAV→servo takes 10-20ms. Bound via
   end-of-span xiao logs (Diagnostic Phase task).
4. **Actual servo rise time** — physical servos take 50-100ms to move full range
5. **Control force buildup** — aerodynamic response to surface deflection
   isn't instant (boundary layer, etc.) — probably negligible but nonzero
6. **Induced drag change** — response to surface deflection is nonlinear
   and depends on speed

**INAV-side filter audit (research task)**: previously we argued MANUAL mode
is pure pass-through, then discovered `servo_lpf` was NOT. Do not assume the
rest of the INAV signal path is clean. Enumerate every filter between RC
command input and servo output, and between raw gyro and MSP gyro output:
- `gyro_hardware_lpf` (stage 1 sensor filter)
- `gyro_main_lpf_hz` (stage 2 software filter)
- `gyro_main_lpf_type`
- Dynamic notch (matrix filter)
- Static notch filters
- `dterm_lpf_hz`, `dterm_lpf2_hz` (only if rate PID active — currently not,
  but audit the pass-through path regardless)
- `rc_smoothing_*` (if enabled)
- Any MSP-side smoothing on received RC

For each: document current setting, whether it affects the NN signal path in
MANUAL mode, and whether it is modeled in sim. Research-first (document),
code changes only if findings demand them. This research lands during
speckit plan phase.

Sim currently models none of these explicitly. Could add a configurable
command delay at the `inputdev_autoc` boundary — NN computes at t, command
takes effect at t+delay.

**Experiment**: train with 0-150ms random command delay per scenario. Does
the resulting NN survive real flight better?

### Xiao NN Correctness Verification (Historical Experiment)

Old concern: does `nn2cpp` generated C++ produce bit-identical output to
the sim NN? This was verified at some point but not recently.

**Experiment**: feed xiao a sequence of NN inputs that match a specific
sim scenario from data.dat, capture xiao's output, compare to sim output.
Any mismatch indicates:
- Floating-point rounding differences
- Code generation bugs in nn2cpp
- Compiler optimization differences
- Integer vs float type coercion issues

**Setup**: one-time test harness on xiao. Feed via serial or a hardcoded
test vector. Output via logPrint to flash.

**Prior art**: there may be an old test harness in `xiao/postflight/` or
`tools/` that did this — search before reimplementing.

### Camera / Second AHRS Cross-Check

Current AHRS source is INAV's IMU fusion. We've verified it's "mostly right"
by quaternion norm stability and attitude range observations. But during
aggressive maneuvers (480°/s rates), IMU integration can drift briefly.

**Options**:
- **Onboard camera**: optical flow or fiducial tracking for independent
  position reference
- **Xiao onboard IMU**: already discussed in backlog, different sensor chip
  (LSM6DS3 vs INAV's primary IMU), independent integration
- **Ground-station observation**: GPS-track comparison, but GPS has its own
  latency and is already in the loop

**Priority**: medium. The AHRS is probably correct, but having an independent
check would rule it out conclusively during debugging.

### dPhi/dTheta Discontinuity Research

Current definitions in `sensor_math.cc`:

```cpp
// dPhi: body YZ projection, angle from -Z
dPhi = fastAtan2(target_y_body, -target_z_body)

// dTheta: body XZ projection, angle from X
dTheta = fastAtan2(-target_z_body, target_x_body)
```

Both are `atan2(...)` outputs in `(-π, π]`. Problems:

1. **dTheta jumps when target passes behind aircraft**: as the aircraft
   overshoots the rabbit, `x_body` crosses zero, `atan2(-z, x)` can jump
   ~π in one NN cycle. This happens frequently during failed tracking
   (aircraft is faster than rabbit, overshoots, then target is "behind").
2. **dPhi jumps in inverted flight**: when the aircraft rolls through ±180°,
   body Z flips relative to target, dPhi jumps discontinuously.
3. **Both sensitive to axial singularities**: when target is exactly above/
   below (dTheta) or ahead/behind (dPhi), the atan2 output is indeterminate.

These discontinuities corrupt the NN history buffer: a single-sample jump
of π radians looks identical to "teleport event" in the dPhi/dTheta
temporal signal.

**Alternative representations** (no discontinuities):

#### Option A: Direction cosines (3 per axis = 6 scalars)

Instead of two angles, output the normalized target direction vector in
body frame: `(x_hat, y_hat, z_hat)` where `||v|| = 1`.

- **Pros**: no discontinuities, no singularities, purely linear
- **Cons**: 3 inputs instead of 2 (or 6 if both now and lookahead)
- **Same info content**: atan2 angles are a reduction of the unit vector
- **Easy to train**: NN doesn't need to learn periodic boundaries

#### Option B: Sin/Cos pair (4 scalars — 2 angles × 2 components each)

Keep the angle semantics but output `(sin(dPhi), cos(dPhi), sin(dTheta), cos(dTheta))`.

- **Pros**: no discontinuities, reconstructs original angle
- **Cons**: 4 inputs instead of 2
- **Common in ML**: standard trick for representing periodic quantities

#### Option C: Cross-product components (body-frame lateral error vectors)

Instead of angles, output `lateral = (target - R * forward) × v_hat` — the
component of the error perpendicular to the aircraft's velocity direction.

- **Pros**: directly tells the NN "turn which way by how much"
- **Cons**: doesn't cleanly separate pitch and roll errors
- **Less interpretable** but may be more natural for control learning

#### Option D: Quaternion-based (6 scalars — imaginary part of two quats)

Compute the quaternion that rotates the aircraft's forward axis to point
at the target. Output the 3 imaginary components (the 4th is determined by
unit norm). Do the same for the "up" direction if relevant.

- **Pros**: fundamentally smooth, shortest-path rotation
- **Cons**: more computation, harder to interpret

**Recommended investigation**:
- **Option A (direction cosines)** is the simplest upgrade path. Replace
  dPhi/dTheta with 3-vector target direction in body frame. History buffer
  would be 3 × 4 slots = 12 inputs instead of current 2 × 4 = 8 (for past
  history), plus 3 × 2 = 6 for forecast (was 4). Total input increase: ~9.
- Verify on sim: would the NN train as well or better with the smoother
  representation?
- Check if `inputdev_autoc.cpp` logs the raw values somewhere we can compare
  — do dPhi/dTheta actually jump discontinuously in flight? (Check xiao NN
  logs for large single-sample changes.)

**Connection to current failure**: EMPIRICAL EVIDENCE CONFIRMED.
Analysis of 2026-04-07 flight logs:

```
Flight 1 (271 samples):
  dPhi  max step delta: 5.9 rad    (near 2π — full wrap)
  dTheta max step delta: 6.2 rad   (near 2π)
  dPhi jumps > π/2: 14 samples (5.2%)
  dTheta jumps > π/2: 20 samples (7.4%)
  dPhi jumps > π:   12 samples   (clearly discontinuities)
  dTheta jumps > π: 17 samples

Flight 2 (359 samples):
  Similar stats — 5% of samples have |Δ bearing| > 90° in 100ms
```

**~1 discontinuity per second on average**. The NN history buffer is
continuously polluted with 2π jumps that have no physical meaning. A
smooth neural network policy cannot handle this cleanly.

**Priority**: HIGH. This is not a speculation — it's happening at 1Hz
on real flights. Replacing dPhi/dTheta with a smooth representation
(direction cosines, Option A) would eliminate a whole class of training/
flight inconsistency.

**CRITICAL UPDATE**: the same discontinuity analysis run on sim data
reveals a 40x gap:

```
           Sim gen400       Real flight
dPhi jumps/sec    0.02           ~0.8
dTheta jumps/sec  0.03           ~1.0
Max delta         6.16 rad       6.27 rad  (~2π in both)
Scenarios with 0 jumps: 137 of 245 (56%)
Scenarios with 6+ jumps:  3 of 245 (1%)
```

**The NN's training distribution contains essentially NO discontinuity
events.** The happy paths in sim almost never trigger atan2 wrapping —
only the worst 5 scenarios (aggressive vertical maneuvers on paths 3 and
4) see multiple jumps. Over 95% of sim training is on fully-smooth bearing
data.

Then real flight hits discontinuities every ~1 second (40x the sim rate)
and the NN has **no learned response** for that input pattern. It's
operating on inputs it literally never saw during training.

**This is arguably the biggest finding of the analysis.** The dPhi/dTheta
representation is broken for the regime real flights operate in, and the
fix (smooth representation) is straightforward.

**Why sim doesn't see it**:
- Sim NN learns to keep the rabbit in the "good zone" (near on-axis,
  within a few meters) where atan2 arguments don't cross discontinuity
  boundaries
- Closed-loop training: if the NN ever does hit a discontinuity, it likely
  loses score catastrophically, so evolution selects against behaviors
  that go near discontinuity-prone regions
- Result: trained policy stays in smooth regions during sim evaluation,
  never develops robust handling of wrap events

**Why real flight sees it**:
- Real aircraft overshoots rabbit due to higher speeds / phase delay /
  stale history transient
- Once aircraft is past the rabbit (target in body -X direction),
  `atan2(-z, x)` has `x` near zero → dTheta bounces wildly
- When inverted (body Z flipped), dPhi bounces the same way
- One misstep → discontinuity → NN sees garbage → larger misstep → cascade

**Fix urgency**: HIGH. This is a direct sim-to-real gap in the observation
space, not just dynamics. The NN is fundamentally blind to a regime it
must handle on real hardware.

### Note: Launch Speed is Ground, Not Airspeed (Hand-Launch Model)

`fdm_larcsim.cpp` `initAirplaneState()` writes `flVelocity = velocity_rel
* trimmedFlightVelocity` into `v_V_local` (ground frame) with no wind
subtraction. This models a stationary-pilot hand launch: at the moment of
release, ground velocity = arm velocity, and wind immediately affects the
airmass-relative airspeed.

For our autoc use case, this means the aircraft starts with a small
launch-transient inconsistency when there's wind (ground ≠ airspeed).
The FDM resolves this over the first ~1 second. Not a significant issue
given our `WindDirectionSigma=45°` and the attitude/speed variations we
already train with — the launch transient is small noise on top of
large intentional variations.

**Worth noting, not worth fixing** unless we see specific issues.

### Consolidated "Simple Path" Strategy

Combining user's guidance:
1. **Higher sample rate**: 10Hz → 20Hz (or more) to reduce phase delay
2. **Slower rabbit**: fix at lower speed so the NN has more time to respond
3. **Zero entry offset** (EntryPositionRadiusSigma=0) — start simple
4. **Special intercept paths**: add purpose-built paths that exercise
   intercept-from-far scenarios, where the rabbit starts well ahead and
   the aircraft has to actively close

Order of operations:
- First: fix engage transient + run eval-mode benchmarks on random paths
- Then: tune the basic regime (simple paths, zero offset, slow rabbit)
  until sim tracks reliably
- Then: add back path complexity one variation at a time
- Finally: flight test at each stable checkpoint

## Out of Scope (deferred to 024+)

- V2 asymmetric cross-track scoring (above/below, inside/outside turn)
- Non-circular cone shape
- Multi-point leading (future rabbit positions)
- Heading alignment bonus
- Adaptive scales based on path curvature
- **Higher NN sample rate (10 Hz → 20/50 Hz)** — requires full retrain AND
  xiao log format compression (current .txt format would ~double at 20 Hz,
  eroding flash budget). The log-format work is the real blocker, not the
  NN inference. When picked up, treat as a joint effort: log format change
  + SimTimeStepMs change + history buffer resize + retrain.
- **Xiao onboard IMU AHRS cross-check (LSM6DS3 + Madgwick)** — independent
  from the 023 AHRS reliability assessment which uses existing INAV AHRS.
  This is the "add a second physical sensor" version. Stays deferred.
- **Wider NN capacity experiment (~5x weights)** — separate A/B against
  the 023-tuned {33, 32, 16, 3} baseline once 023 lands and visual/LED
  tracking work begins. The 023 hidden-layer sizing is already forward-
  compatible with that work; a deliberate capacity experiment is its own
  feature.
- **Third lexicase dimension: energy / chatter penalty** (`Σ d²output²`
  from 022 dev report) — Change 8's authority-limit iteration is 023's
  hedge against bang-bang. A formal energy lexicase dimension is a
  post-023 candidate if authority limiting alone is insufficient.

## Proposed Changes

### Change 1: Engage Transient Fix (CRITICAL)

Root cause: xiao `recordErrorHistory()` is called during autoc but the
buffer is NOT reset when autoc re-engages (second+ engage per flight).
First engage works because buffer is zero-initialized at boot.

**Fix (LOCKED 2026-04-08)**: On every autoc transition (`false → true`),
pre-fill ALL history slots in `AircraftState` with the current in-autoc
geometry computed against the newly-armed path:
- Compute `target_x_body / target_y_body / target_z_body` (new 3-cosine
  representation — see Change 6) from current aircraft state and path
- Compute `dist` from current aircraft position to rabbit
- Push the same computed values into every history slot (past samples
  AND lookahead samples get the current-tick values)
- First NN call sees a flat history matching reality: closing rate is
  zero, all past samples equal the "now" sample.

No special-case for first vs re-engage — the same reset happens every
time. Closing rate is derived as `(now - prev) / dt`, which is zero when
all slots are pre-filled identically. No spurious large gradient on the
first post-engage tick.

**Code location**: `xiao/src/msplink.cpp` — at the autoc enable transition
(currently line ~402). Add a `resetHistory(const Path& armedPath, const
AircraftState& currentState)` method to `AircraftState` that performs the
pre-fill computation. Sim side (CRRCSim `inputdev_autoc.cpp`) uses the
same method at each scenario start — verifies the training path and
runtime path share the same reset code.

**Scope note — interaction with Change 1b (INAV engage delay)**: the
pre-fill reset's practical impact is smaller than it looks at first.
Change 1b adds a ~0.75s delay window in which the NN is running and
updating its history buffer but the sim bridge ignores its outputs.
By the time NN commands actually matter, the history has been
overwritten by ~7 real in-flight samples. The pre-fill reset
protects the first handful of samples inside the delay window
against stale pre-engage contents — those samples influence the NN's
later state even though their outputs are ignored. Still necessary
but not dominant.

**Also check**: does this same bug exist in CRRCSim? If the sim re-uses
aircraft state across evaluations, it might have the same stale-history
pattern that the NN was trained against. Probably not (each scenario
creates a fresh state) but worth verifying.

### Change 1b: INAV Engage Delay Modeling

The INAV ~750ms delay between "switch on" and "commands effective" means
the entry position variation we train with is NOT what the real aircraft
sees. Current training places the aircraft at `EntryPositionRadiusSigma`
offset and starts NN control immediately. Real flight has the aircraft
drifting for ~0.75s before NN commands reach the servos.

**Fix (LOCKED 2026-04-08)**: Simulate the delay in the CRRCSim bridge.

**Delay-window behavior** (the full ~0.75s after autoc engage):
- NN runs every tick, computes outputs, and updates its internal history
  buffer normally. The history buffer sees real in-flight samples
  (dPhi/dTheta/dist etc. against the armed path) and fills naturally
  over the window.
- **CRRCSim ignores the NN's outputs** during the window. Instead it
  applies centered stick: `pitch_cmd = 0, roll_cmd = 0, throttle_cmd = 0`.
- Craft flies open-loop under FDM + wind during the window. No autopilot,
  no attitude hold, no stabilization. Matches the real handoff: user
  lines up the release, throws the switch, and the aircraft coasts
  briefly on momentum + wind while INAV processes the mode change.
- After the window expires, the sim bridge starts applying NN outputs
  normally. From that tick forward, training and flight are identical.

**Configuration**:
- Add `EngageDelayMs` to `autoc.ini` (default: 750).
- Runtime global `gEngageDelayMs` read at startup.
- Xiao-side constant matching the sim value. The sim models the delay
  so that training conditions match what the aircraft experiences on
  real flights; xiao doesn't need any new behavior because the delay
  is imposed by INAV itself.

**Interaction with Change 1 (history buffer reset)**: Change 1's pre-fill
still runs at engage transition, but its scope is narrower than initially
thought. By the time NN commands actually take effect (end of delay
window), the buffer has already been overwritten with ~7–8 real ticks
of genuine in-flight samples. The reset primarily protects the first
few samples of the window against stale pre-engage contents. Still
worth doing — those early samples influence the NN's internal state
even though its outputs are ignored — but it is not the dominant fix.

**Cost in training time**: each scenario gains ~7 ignored ticks (~14 if
we go to 20 Hz). Trivial overhead.

**Real-flight delay reduction**: see Change 1c below.

### Change 1c: INAV Config-Only Delay Reductions (from audit findings)

The Phase 0 INAV signal path audit
(`docs/inav-signal-path-audit.md`) identified five config-only changes to
`xiao/inav-hb1.cfg` that remove unmodeled phase delay from both the NN
input path and the servo command path. **No INAV code changes; no sim
changes.** All are pure config tweaks applied and verified on bench
before any flight test.

| # | Config key | Current | Recommended | Effect |
|---|---|---|---|---|
| 1 | `gyro_main_lpf_hz` | `25` | `60` (default) or higher | Removes ~4 ms from gyro→xiao path (6.4 ms → 2.65 ms) |
| 2 | `setpoint_kalman_enabled` | `ON` | `OFF` | Removes an adaptive Kalman stage on `gyroADCf` — state-dependent group delay. This is NOT a setpoint filter despite the name; it operates on gyro measurements. Should never have been on for NN use. |
| 3 | `rc_filter_lpf_hz` | `250` | `0` | Disables the PT3 on `rcCommand` in the MANUAL-mode path. Removes ~1.9 ms. Surprise finding — we assumed MANUAL was pass-through (same class of error as `servo_lpf_hz`). |
| 4 | `dynamic_gyro_notch_enabled` | *(unknown)* | `OFF` for fixed-wing | Dynamic notches are primarily for multirotor vibration rejection; our airframe and NN input are not sensitive to 100 Hz+ spectra. Also removes a state-dependent secondary dynamic notch that runs regardless of the enable flag. |
| 5 | `failsafe_recovery_delay` | `5` (500 ms) | `0` | **Bench test, not a production change.** The audit flagged this as the PROBABLE root cause of the 750 ms engage delay (via `PERIOD_RXDATA_RECOVERY` + `failsafe_recovery_delay * 100 ms` in `src/main/rx/msp_override.c:103-104`). Test: set to 0, re-measure bench engage delay. If it drops, confirmed. If not, the delay is xiao-side. |

**Implementation** (Phase 1 bench work, not training):
1. Back up current `xiao/inav-hb1.cfg`.
2. Apply changes 1-4 one at a time, rebuild xiao, bench test, measure
   MSP2 round-trip latency via xiao blackbox logs at each step.
3. Expected cumulative improvement: ~6-10 ms removed from the NN input
   path, ~2 ms removed from the servo output path.
4. Separately, run change 5 as a dedicated bench test to confirm/refute
   the 750 ms engage delay hypothesis.
5. Document measured before/after numbers in a Phase 1 summary note
   appended to `docs/inav-signal-path-audit.md`.

**Interaction with `EngageDelayMs` (Change 1b)**: if change 5 succeeds in
reducing the 750 ms mystery delay, lower `EngageDelayMs` config default
to match the new measured value. The sim should always model the real
delay — not a constant 750.

**Interaction with sim dynamics (Change 5)**: config changes here do NOT
change the sim. They only change what xiao/INAV actually do on real
hardware. The sim is already simulating a worst-case delay via
Change 1b. If these config changes reduce real-hardware delay below the
sim's modeled delay, the sim becomes more conservative than reality —
acceptable. The reverse (sim faster than reality) would be a sim-to-real
gap.

**Not in scope for 023**: the audit flagged several items as "unknown"
because they required either bench measurement or tracing constants
beyond the source read. These stay in the audit doc's "Sections marked
unknown" section, not this spec:
- Exact `PERIOD_RXDATA_RECOVERY` value (change #5 is the test)
- `FEATURE_THR_VBAT_COMP` active status
- Adaptive Kalman / dynamic notch exact group delay
- Hardware stage-1 anti-alias filter behavior

### Change 2: Training Distribution — Far-Start Intercept Coverage

Sim training needs to include far-start conditions and recovery scenarios
so the NN learns "approach from 50m" as part of its policy.

**Increase entry position variation**:
- `EntryPositionRadiusSigma = 15` → `30-50`
- Real flights will have similar or larger offsets between where the
  aircraft happens to be and where the path originates
- With current scoring surface (behind_scale=7m), scores are small at 30m+
  — evolution has weaker gradient but SOME signal (Lorentzian tail)

**Increase entry speed variation**:
- `EntrySpeedSigma = 0.1` → `0.25-0.3`
- Sim sees 9-17 m/s entries instead of 12-14
- Matches real aircraft entry speeds more closely

**Enable 6th path (SeededRandomB)**:
- `SimNumPathsPerGeneration = 5` → `6`
- Longer varied path with random turns
- Tests recovery from lost tracks, generalization

**Consider larger EntryRollSigma**:
- Currently 30° — real flight showed 27-33% inverted time
- Increase to 60-90° to train recovery from wider attitude range

### Change 3: Throttle Lexicase Dimension

From 022 clarifications: "mean_throttle deferred as a future second dimension."
Now needed.

**Add `mean_throttle` to `ScenarioScore`** (re-add — was removed in 022 cleanup):
```cpp
struct ScenarioScore {
    double score;
    double mean_throttle;  // [-1, +1], lower = less energy
    bool crashed;
    // ... (streak diagnostics)
};
```

**Lexicase filter**: per scenario, apply `score` filter first (primary),
then `mean_throttle` filter (secondary). Evolution picks individuals that
score well AND use less throttle.

**Epsilon**: `score` epsilon 0.05 (5%), `throttle` epsilon ~0.1 (10%).

### Change 4: Rabbit Speed Simplification

Current sim: variable 10-18 m/s (sigma 2, nominal 13). Xiao: fixed 13 m/s.

**Recommendation: fix sim rabbit at 13 m/s too** for this iteration.
- `RabbitSpeedSigma = 0`
- Eliminates one training variable during debug phase
- Can reintroduce variation later once basic tracking works
- Matches xiao exactly

### Change 5: Sim Dynamics (Investigate, maybe adjust)

Wind-corrected real airspeed is within sim range. Don't touch dynamics for
now. Re-examine after training changes take effect.

If after next retrain the NN still runs full throttle on real and the
speed issue persists, consider:
- `CD_prof`: 0.18 → 0.22 (more drag, sim Vmax ~20 m/s)
- Or thrust reduction to match real Vmax

### Change 6: Bearing Representation — Direction Cosines

**Motivation**: The current `dPhi`/`dTheta` inputs are `atan2` outputs that
wrap at ±π. Analysis of the 2026-04-07 flight found ~1 wrap event per second
in real flight versus ~0.02/sec in sim — a 40x gap. The NN has essentially
no training on discontinuity events. Replace with direction cosines to
eliminate wrapping **structurally** (not statistically).

**Representation**: Instead of two angles per time sample, output the unit
vector pointing from aircraft to target, expressed in body frame:
`(tx, ty, tz)` where `tx² + ty² + tz² = 1`. Same 6 time samples
(`[-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]`).

**Input delta**: +6 slots per time-sample set. Inputs grow from 27 to 33.

**Singularity handling (dist ≈ 0)**: When aircraft is exactly at the rabbit,
the direction vector is undefined. Use a hard numerical floor, not a blend:
```cpp
const float DIR_NUMERICAL_FLOOR = 1e-4f;  // meters

vec3 computeTargetDir(vec3 target_body, float dist,
                      vec3 rabbit_vel_dir_body) {
    if (dist < DIR_NUMERICAL_FLOOR) {
        return rabbit_vel_dir_body;  // path-tangent fallback at true singularity
    }
    return target_body * (1.0f / dist);
}
```

Rationale for hard switch over a blended window: at 10 Hz and 18 m/s the
aircraft covers ~1.8 m per tick. Any "smooth blend" window (e.g. 0.5 m) is
traversed in a fraction of a tick, so the NN sees a step change either
way. A blend adds tuning surface without removing the jump. The straight
normalization handles "aircraft flies past rabbit" correctly — the direction
vector rotates naturally through the pass. The fallback triggers only when
the math is literally unsafe.

**Lookahead samples** (`+0.1s`, `+0.5s`): use future rabbit position on the
path, same as today. Project into *current* body frame, then direction-
cosine normalize. No semantics change beyond the final projection.

**Compute cost (xiao)**: No trig. Direction cosines eliminate `atan2`
without introducing `cos`/`sin`. Per-sample cost is vector-subtract,
length (`sqrt`), divide — all hardware FPU ops on nRF52840. No cos tables,
no lookup, no approximation needed. Net MAC count for the full inference
goes from ~584 to ~1600 (see NN Topology below), still under 300 μs per
call on Cortex-M33 — negligible against any sample rate we'd consider.

### NN Topology After Change 6

**New topology: {33, 32, 16, 3}, 1667 weights** (vs. current {27, 16, 8, 3},
611 weights — 2.7x weight count).

**Input layout (33)**:
```
 0- 5: target_x_body [-0.9,-0.3,-0.1,now,+0.1,+0.5]s   [-1,1]  unit-vec component
 6-11: target_y_body [-0.9,-0.3,-0.1,now,+0.1,+0.5]s   [-1,1]
12-17: target_z_body [-0.9,-0.3,-0.1,now,+0.1,+0.5]s   [-1,1]
18-23: dist          [-0.9,-0.3,-0.1,now,+0.1,+0.5]s   m
   24: dDist/dt closing rate (m/s)
25-28: quaternion (w, x, y, z)                         [-1,1]
   29: airspeed (m/s)
30-32: gyro rates (p, q, r) rad/s                      std RHR
```

**Hidden layers**: 32 (was 16), 16 (was 8). Rationale: the +6 input slots
already force a topology migration; the type-safe NN sensor interface
prerequisite makes the migration cost ~linear in the number of touched
files, not quadratic in "how much we changed." While we're paying that
cost, bump hidden capacity to match the richer input representation.

**Weight arithmetic**:
- Layer 1: `33 * 32 + 32` = 1088
- Layer 2: `32 * 16 + 16` = 528
- Layer 3: `16 * 3 + 3`   = 51
- **Total: 1667**

**Xiao inference cost**:
- ~1616 MAC + 51 tanh per forward pass
- At ~3.5 cycles/MAC on Cortex-M33 + FPU: ~280 μs/inference
- Against 100 ms tick (10 Hz): 0.28% CPU. Against 50 ms tick (20 Hz): 0.56%.
- **Compute is not the constraint.** Xiao log format compression is the
  real 20 Hz constraint (already noted in Parked section).

**Evolution cost**: ~2.7x weights means the search space for mutation is
larger. First-baseline walltime estimate: 1.5–2x current (022 betterz2 was
~400 gens × pop 3000 × 245 scenarios). Tolerable with 022's training
infrastructure.

**Forward compatibility**: Hidden layer sizing (32, 16) is also sized with
visual target tracking in mind (wingtip-LED camera frontend). A visual
frontend is likely to produce a wider preprocessed feature vector than
current 33 inputs. Paying for wider hidden layers now avoids doing this
migration twice. When visual-NN work lands (post-023, on GB10 GPU for
training), the downstream NN backend already has capacity headroom. Not
a 023 deliverable — just a sizing decision that costs nothing now and
saves a migration later.

**Not a 5x experiment**: An earlier discussion considered scaling weights
~5x as an independent capacity experiment. Deferred post-023 as a separate
A/B against the 023-tuned baseline. This spec's bump (2.73x with the
actual numbers) is a "paid for by the input change" size adjustment, not
a research experiment on NN capacity.

**Logging coverage — all new inputs and all outputs**: when inputs grow
from 27 to 33, `data.dat` must gain 6 new columns (one per new
`target_x/y/z` × 6-tick history slot) and the header must emit the new
field names. NN outputs (`pitch, roll, throttle`) continue to be logged
as before. Xiao flight logs gain the same new input columns. This is a
hard requirement from the Prerequisite section's "Logging preservation"
clause — see Phase 0a.1 verification test. No field may be dropped as
a side effect of the refactor, and no new field may be added to the
struct without a corresponding column in both `data.dat` and the xiao
log format.

### Change 7: Discontinuity-Forcing Training Paths

Sim training under 022 almost never triggers bearing discontinuities
(~0.02/sec vs real ~0.8/sec). The NN therefore never develops a response
to wrap events, and real flight exposes the gap. Change 6 fixes the
representation so wraps *can't happen* — but the NN also needs training
examples in the regimes that previously produced them (target behind
aircraft, target below inverted aircraft, close overshoot).

Add purpose-built training paths that force the aircraft into these
regimes:
- **Chase-from-behind**: rabbit starts 30m ahead, aircraft must actively
  close. Time in "target ahead" regime = most of the path.
- **Overshoot recovery**: rabbit stops at waypoint, aircraft must decelerate
  or turn to avoid flying past. Forces "target behind" regime.
- **Inverted sustained**: a path segment that requires sustained inverted
  tracking (loop or split-s extended through the top).
- **Vertical descent**: rabbit below aircraft at entry, aircraft must pitch
  down and dive. Forces "target below" in NED while aircraft is near-level.

These are Phase 2 additions — land after the base 5-path retrain stabilizes
under the new representation.

### Change 8: Authority-Limit Iteration (Energy / Bang-Bang)

**Hypothesis**: The 10 Hz sample rate creates a tendency for the NN to
overcorrect — commanding full surface throw in one tick, compensating the
next. This is exacerbated by CRRCSim's dynamics being "stretched thin" at
full-power / full-command inputs (sim artifacts that don't hold in
reality). The result is a policy that looks fine in sim but produces
bang-bang behavior in real flight.

**Approach** (iterative, not a single change):

1. **Train baseline** with all other 023 changes in place (especially
   Change 6, the 3-cosine bearing representation). Examine what the
   NN actually learns — throttle distribution, surface deflection
   distribution, `d²output²` chatter, per-axis saturation fraction.

2. **Set trigger criterion empirically (plan phase)**. Phase 3b baseline
   observation produces the data that determines whether Phase 3c
   authority-limit retrain is worth running. Specific trigger thresholds
   (mean throttle, saturation fraction, chatter metric) are NOT set in
   this spec — they are set after the baseline run, by the plan-phase
   analysis of Phase 3b results. The 3-cosine representation change is
   the primary intervention; Phase 3c is a conditional follow-up.

3. **If triggered**: apply an *authority limit* at the NN output clamp.
   Cap pitch/roll command magnitude at ~50% of full throw during
   training AND inference. Retrain (from scratch or from baseline weights
   as a warm start — decided during plan phase).

4. **Compare**: does the authority-limited policy track as well in sim?
   How does it behave in real flight?

This is likely to produce a stronger signal than a pure `Σ d²output²`
chatter penalty (the "Phase 5+" proposal from 022 dev report), because
the authority limit *forces* the change rather than penalizing it.
Penalty-based approaches require tuning an epsilon; authority limits are
a hard cap with no tuning surface.

**Not a lexicase dimension** (at least not initially) — this is a
configuration knob on the NN output boundary. If the iterative approach
validates that limiting authority helps, we can consider adding
`max(|output|)` or `Σ d²output²` as a lexicase dimension in a follow-up.

**Configuration**:
- Add `NNAuthorityLimit = 1.0` to `autoc.ini` (default = no limit).
- Applied as `output = tanh(raw) * NNAuthorityLimit` before interpretation
  in sim bridge AND on xiao. Must be identical both sides.
- Xiao-side env var or hardcoded constant matching training value.

### Prerequisite: Type-Safe NN Sensor Interface

**Why blocking**: Change 6 grows NN inputs from 27 to 33. Under the current
hand-maintained topology pattern, this forces updates to 9+ files in
lock-step (see BACKLOG "Type-Safe NN Sensor Interface" for the full list).
The 021 redesign (29→27) shipped with silent serialization corruption from
this class of bug. Doing it one more time is asking for the same thing.

**Goal**: The compiler enforces topology consistency. Adding, removing,
or reordering an input must either propagate automatically everywhere,
or produce a compile error at every site that assumed the old layout.

**Locked approach (2026-04-08)**: Named struct-of-float-arrays.

```cpp
// include/autoc/nn/nn_inputs.h
struct NNInputs {
    // 6 time samples each: [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]
    float target_x[6];   // unit-vec x in body frame (was dPhi)
    float target_y[6];   // unit-vec y in body frame (was dTheta, partial)
    float target_z[6];   // unit-vec z in body frame (new)
    float dist[6];       // metres
    float closing_rate;  // m/s, +ve = approaching
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float airspeed;      // m/s
    float gyro_p;        // rad/s body rate, aerospace RHR
    float gyro_q;
    float gyro_r;
};

static_assert(sizeof(NNInputs) == 33 * sizeof(float),
              "NNInputs layout must be contiguous float[33] with no padding");
static_assert(alignof(NNInputs) == alignof(float),
              "NNInputs must be float-aligned for matrix multiply");

constexpr int NN_INPUT_COUNT = sizeof(NNInputs) / sizeof(float);
```

**Why this specific approach**:
- **Runtime cost is zero.** Struct of `float[N]` members has identical
  memory layout to a bare `float[33]`. Matrix multiply via
  `reinterpret_cast<const float*>(&inputs)` gets the same pointer the
  inner loop wanted anyway. All three candidate approaches (struct,
  enum indexing, template tags) compile to identical machine code.
- **`static_assert` catches the exact 021 failure mode.** The 021
  serialization corruption happened because `NN_INPUT_COUNT` and the
  actual layout drifted. Under this approach, any drift produces a
  compile error: the `sizeof` assertion fails if padding is introduced
  or fields are added without updating the contract.
- **Adding fields forces visiting every site.** Designated-initializer
  construction (`NNInputs{.target_x = ..., .dist = ..., ...}`) makes
  the compiler flag any missed field at every construction point.
- **Field names become self-documenting.** Access is `inputs.gyro_p`
  not `inputs[26]`. `data.dat` columns emit with field names.
  `sim_response.py` reads by name, not position.

**Rejected alternatives**:
- **Enum-class indexing** (`inputs[static_cast<int>(Input::GYRO_P)]`):
  smallest migration but only catches typos. Does not catch adding an
  enum value without a populator, off-by-one, or size drift. Preserves
  the "floats addressable by number" pattern we are trying to eliminate.
- **Template tag types** (`get<GyroP>(inputs)`): equivalent safety to
  the struct for accesses but weaker for construction, and adds
  template machinery with no corresponding benefit.

**Constitutional note on struct field order**: the struct field
declaration order is part of the serialization contract. Reordering
fields is a format-breaking change equivalent to changing the cereal
schema. Add a comment at the top of the header stating this, so future
refactors don't silently break saved weights / data.dat parsers.

**Minimum requirements**:
- `NNInputs` struct as the single source of truth for input layout
- `static_assert(sizeof(NNInputs) == N * sizeof(float))` contract
- `NN_INPUT_COUNT` derived from `sizeof(NNInputs) / sizeof(float)`,
  not hand-maintained as a separate constant
- Serialization via direct member access (cereal `CEREAL_NVP` or
  member-by-member), not `reinterpret_cast`
- `nn_gather_inputs()` populates fields by name, not by index
- `nn2cpp` codegen emits field-name access, not integer indexing
- `data.dat` header emits column names derived from field names
- `sim_response.py` parses by column name
- Xiao `msplink.cpp` populates fields by name

**Non-goals (for 023)**: Unit metadata, range validation, runtime
introspection, registry pattern. The point is compile-time safety, not
a full sensor framework.

**Logging preservation — HARD REQUIREMENT**: both `data.dat` (sim side)
and xiao flight logs must continue to emit **every NN input field and
every NN output field, one column per field, every tick**, with no
regression in coverage from the current output. The refactor changes
*how* fields are accessed, not *whether* they are logged. Specifically:

- `data.dat` per-step row: one column for every member of `NNInputs`
  (e.g. `target_x_0, target_x_1, ..., target_z_5, dist_0, ..., gyro_r`)
  plus the 3 NN output columns (`pitch, roll, throttle`) plus existing
  meta columns (timestamps, path state, fitness diagnostics, etc.).
- `data.dat` header: one header name per column, derived from struct
  field names (e.g. `target_x_0` for `target_x[0]`, `gyro_p` for
  `gyro_p`). Header order matches column order exactly.
- Xiao log: same coverage. Every NN input and every NN output written
  to flash every NN cycle, in the compact binary or CSV format the
  xiao log already uses. Column/field names must stay consistent with
  `data.dat` so downstream analysis scripts can cross-reference.
- `sim_response.py` and any other parser must read by column name
  (post-refactor), not positional index. This is also part of the
  type-safety goal — if a parser reads by name, silent column
  reordering produces a KeyError at parse time, not a garbage plot.

**Phase 0a.1 verification test**: Before declaring the refactor done,
run a training scenario under the pre-refactor binary and the
post-refactor binary against the same seed and weights. Compare the
two `data.dat` files:
- Column count must match: `27 + 3 + meta_count` before, same after.
- Column headers must exist for every new named field.
- Row values for every NN input and every NN output must match within
  FP rounding.
- Any xiao log captured from a test flight / bench run (pre-refactor)
  must be replayable against the post-refactor parser without data loss.

If any NN input or output stops being logged as a side effect of the
refactor, the refactor is broken and must be reverted — this is not
optional cleanup.

**Rationale**: `data.dat` and xiao logs are the ground truth for every
post-flight diagnostic 023 depends on (discontinuity rate, throttle
saturation fraction, `d²output²` chatter, input distribution vs sim).
Losing coverage for even one field means a post-flight analysis
question becomes impossible to answer. 023's success criteria #6
(next-flight zero discontinuities) cannot be verified without the
per-tick bearing input columns.

**Files in scope** (from BACKLOG list, updated with struct approach):
- `include/autoc/nn/nn_inputs.h` (NEW — canonical struct definition)
- `include/autoc/nn/topology.h` (derives `NN_INPUT_COUNT` from struct)
- `include/autoc/autoc.h` (remove duplicate defines)
- `include/autoc/eval/aircraft_state.h` (store `NNInputs` not `float[]`,
  update cereal serialization to round-trip the struct)
- `src/nn/evaluator.cc` (nn_gather_inputs → populate named fields;
  matrix multiply uses `reinterpret_cast<const float*>(&inputs)` at
  the single entry point of forward())
- `src/autoc.cc` (data.dat format string, header emits field names,
  field indexing becomes field access)
- `tests/contract_evaluator_tests.cc` (layout assertions by name)
- `tests/nn_evaluator_tests.cc` (input layout assertions by name)
- `tools/nn2cpp/*` (codegen emits struct-field access for xiao)
- `specs/019-improved-crrcsim/sim_response.py` (parse by column name)
- `xiao/src/msplink.cpp` (populate fields by name, no integer index math)

**Migration order**:

1. **Phase 0a.1** — Add the `NNInputs` struct at 27 inputs matching the
   current layout. Derive `NN_INPUT_COUNT` from `sizeof`. Update all
   consumers to access by name but keep behavior identical. Verify all
   tests pass. **No behavior change, just refactor.** Verify by
   deliberately breaking the struct (add an unused field, see compile
   errors at every call site; remove it).
2. **Phase 0a.2** — Apply the 27→33 input change in one commit, with
   the new `target_x`/`target_y`/`target_z` arrays replacing `dPhi`/
   `dTheta`. The compiler catches every site that missed the migration.
3. **Phase 0a.3** — Run the deliberate-break test again (bump
   `sizeof(NNInputs)` by adding a throwaway field) to verify the
   static_assert catches the 021 failure mode.

## Parameter Schedules for CRRCSim Milestones

Configuration knob values for the CRRCSim training milestones (Phase 3
in Implementation Order). Milestones are observational layering — use
judgment on ordering, same pattern as 022 betterz1/betterz2.

### CRRCSim Milestone A — Zero-Variation Baseline

```ini
# Fitness (unchanged from 022)
FitDistScaleBehind              = 7.0
FitDistScaleAhead               = 2.0
FitConeAngleDeg                 = 45.0
FitStreakThreshold              = 0.5
FitStreakRampSec                = 5.0
FitStreakMultiplierMax          = 5.0

# Training distribution: ZERO VARIATION
EnableEntryVariations           = 0     # (022 T024b-style master switch; add similar for all groups)
EnableWindVariations            = 0
EnableRabbitSpeedVariations     = 0     # from 022 T024b
EntryConeSigma                  = 0     # was 30  — no attitude variation
EntryRollSigma                  = 0     # was 30  — no roll variation
EntrySpeedSigma                 = 0     # was 0.1 — no speed variation
EntryPositionRadiusSigma        = 0     # was 15  — start at path origin
EntryPositionAltSigma           = 0     # was 3   — no altitude offset
WindDirectionSigma              = 0     # was 45  — no wind variation

# Rabbit speed: FIXED, slower than real cruise for reaction time
RabbitSpeedSigma                = 0     # was 2    — fixed speed
RabbitSpeedNominal              = 10.0  # was 13.0 — slower for more reaction time

# Paths: keep 5 deterministic, no random
SimNumPathsPerGeneration        = 5     # unchanged

# SimTimeStepMs stays at 100ms (10 Hz). 20 Hz is Out of Scope for 023.

# NN authority limit: start at 1.0 (no limit). Change 8 retrain may
# drop this to 0.5 if baseline behavior examination shows saturation.
# Trigger criterion determined empirically during plan phase.
NNAuthorityLimit                = 1.0
```

Goal: rock-solid tracking on 1 path (expand to 5 deterministic once it
works) with NO variations, under the new 3-cosine representation. If the
NN can't track here, nothing else matters.

### CRRCSim Milestone B — Layer in Complexity

Once Milestone A is solid, add variations back ONE AT A TIME, measuring
retention of Milestone A quality at each step:

1. Wind direction variation (`EnableWindVariations=1`, `WindDirectionSigma=45`)
2. Entry attitude cone (`EnableEntryVariations=1`, `EntryConeSigma=30`, small → large)
3. Entry speed variation (`EntrySpeedSigma=0.1`, small → large)
4. Rabbit speed variation (`EnableRabbitSpeedVariations=1`, `RabbitSpeedSigma=2.0`)
5. Entry position offset (`EntryPositionRadiusSigma=5` → `15` → `30-50`)
   - Larger values cover INAV delay drift per Change 1b
6. 6th path (`SimNumPathsPerGeneration=6` — enables SeededRandomB)
7. Discontinuity-forcing paths (Change 7 — chase, overshoot, inverted, descent)

Each addition: one training run, measure:
- Fitness retention vs Milestone A baseline
- Per-path brittleness (any specific path scoring >10x worse than others?)
- Discontinuity rate in data.dat — should stay at zero; if not, the
  representation change has a bug
- Throttle saturation fraction and `d²output²` chatter

### CRRCSim Milestone C — Robustness (dynamics variations)

Craft parameter stddev variations (from 015 T140–T143):
- Control effectiveness: `Cl_da`, `Cm_de` ± 10–20%
- Damping: `Cl_p`, `Cm_q` ± 10–20%
- Drag: `CD_prof` ± 10%
- Mass / inertia: ± 5%
- Control phase delay: 0–200 ms between command and servo response
  (informed by Phase 1a INAV filter audit findings in
  `docs/inav-signal-path-audit.md`)

Start narrow (control effectiveness + phase delay only), widen based on
training results. Forces policy to generalize across a neighborhood of
the nominal `hb1_streamer.xml` dynamics.

## Success Criteria

1. **No engage transient** — first NN call after autoc engage produces
   sensible closing rate (|dd/dt| < 20 m/s, not 1000+)
2. **Training covers real-flight distribution** — sim p99 distance > 40m,
   sim p99 dd/dt extends to negative values (diverging)
3. **Throttle mean < 0.5** in sim training — NN uses < half-throttle on average
4. **Real flight achieves sustained tracking** — distance stays below 30m for
   > 5 seconds on at least one autoc span
5. **Bearing representation is structurally wrap-free** — post-Change 6, the
   NN bearing inputs are unit vectors by construction. No `atan2` in the
   input pipeline. Verified by code inspection and by the type-safe NN
   interface's compile-time checks.
6. **Next-flight blackbox shows zero projection discontinuities** — real
   flight analysis (same methodology as 2026-04-07) must find **zero**
   large-step events in the bearing input sequence. Not "below 0.1/sec" —
   zero. The structural change means wraps cannot occur; if any appear,
   the representation change was incomplete somewhere in the pipeline.
7. **Eval-mode determinism** — running eval on saved gen-N weights
   reproduces the training-time fitness within FP rounding. This is the
   acceptance criterion for the Bug 3 fix and a precondition for every
   other diagnostic in this feature.
8. **Type-safe NN interface is compile-enforced** — changing
   `NN_INPUT_COUNT` in one place either propagates automatically or
   produces compile errors at every dependent site. Verified by
   deliberately breaking the count in a test branch.
9. **Logging coverage preserved and extended** — `data.dat` (sim) and
   xiao flight logs emit one column per NN input field AND one column
   per NN output field (`pitch`, `roll`, `throttle`), every tick. After
   Change 6 expands inputs to 33, the logs must include all 33 new
   input columns plus the 3 output columns plus existing meta columns.
   No NN input or output field may disappear from the logs as a side
   effect of the type-safe refactor. Verified by comparing pre-refactor
   and post-refactor `data.dat` files at the same seed/weights — column
   counts and per-tick values for inputs/outputs must match within FP
   rounding (pre-refactor at 27 inputs vs post-refactor at 27 inputs
   during Phase 0a.1, then 33 inputs after Phase 0a.2).

## Implementation Order

The execution structure is a **3-rung validation ladder**: minisim smoke
test → CRRCSim training milestones → xiao field-test prep. This mirrors
the successful 022 pattern (betterz1 / betterz2) where incremental
observational milestones proved more useful than rigid phase gates.

### Phase 0: Prerequisites (blocking)

Land these before any training run, in order.

**0a. Type-safe NN sensor interface refactor** — see Prerequisite section
above for the full struct-of-floats design. Three sub-steps:
- 0a.1: Add `NNInputs` struct at 27 inputs, refactor all access sites,
  keep behavior identical. Verify `data.dat` columns pre/post refactor
  match within FP rounding.
- 0a.2: Grow to 33 inputs (3-cosine bearing representation, Change 6).
  Compiler catches every migration site via designated-initializer
  construction + `static_assert`.
- 0a.3: Deliberately-break verification — add a throwaway field, confirm
  compile error at the `sizeof` assertion. Revert.

**0b. Train/eval scenario construction de-duplication** — NOT a one-line
fix. `runNNEvaluation()` in `src/autoc.cc` has drifted from the training
scenario setup path, producing the eval fitness bug family (Bugs 1/2/3
in BACKLOG). Extract scenario construction into a shared helper used by
both paths. Acceptance: eval on saved betterz2 weights reproduces the
training-time fitness within FP rounding. This is the determinism
cross-check; without it no 023 benchmark is trustworthy.

### Phase 1: Plan-Phase Research (concurrent with Phase 0)

Research output location depends on durability: `~/autoc/docs/` for
long-lived reference material that may outlive 023, the feature dir for
one-off surveys that become obsolete once their refactor lands.

**1a. INAV signal path audit** — COMPLETE. See
`docs/inav-signal-path-audit.md`. Deep read of `~/inav` source found the
gyro filter stack (`gyro_main_lpf_hz`, `setpoint_kalman_enabled`) and
`rc_filter_lpf_hz` PT3 as unmodeled delay contributors in the MANUAL-mode
path. Gyro data reaches xiao via `MSP2_INAV_LOCAL_STATE` (not `MSP2_GYRO`
— that endpoint does not exist in this fork; gyro is piggybacked). Five
config-only recommendations documented, no INAV code changes required.

**1b. INAV escape-hatch architectures** — if 1a finds the delay is
architectural, document alternatives as 024+ candidates: direct
xiao→servo PWM/PPM bypassing INAV control loop, drop GPS from NN inputs
during autoc, run NN on IMU-only at higher rate. Captured as a section
of the same `docs/inav-signal-path-audit.md` document. Not 023 scope —
023 uses the current topology, this just documents the pivot path if
chronic.

**1c. MSP2 latency bound** — parse `flight-results/flight-20260407/flight_log_*.txt`
end-of-span samples. Bound the current round-trip latency. Appended to
`docs/inav-signal-path-audit.md`.

**1d. Train/eval code duplication survey** — side-by-side read of
`runNNEvaluation()` and the training path. Enumerate every divergence.
Output: `specs/023-ood-and-engage-fixes/train-eval-code-dedup.md`
(one-off — feature-scoped, discarded after Phase 0b refactor lands).
Feeds Phase 0b refactor design.

### Phase 2: Minisim Smoke Test

**Purpose**: prove the new code path can train *anything*. Not a tracking
quality gate — a plumbing gate.

**What runs**: 1 path, no variations, minisim backend. The 3-cosine
representation, type-safe NN struct, engage delay window, Change 1 history
reset, logging coverage, and eval determinism all wired together.

**Pass criteria**:
- Build compiles, all unit tests pass
- Minisim launches, runs 50+ gens without NaN / hang / segfault
- `data.dat` emits all 33 + 3 + meta columns with named headers
- Fitness decreases monotonically on average across gens (NN is learning
  something; actual value doesn't matter)
- Eval on any gen-N weight reproduces the training fitness within FP
  rounding (Phase 0b determinism check)

**What is NOT validated here**: tracking quality, real-dynamics behavior,
variation handling (minisim has no wind / entry variations / craft
parameter variations). Those all move to Phase 3.

**Exit**: if minisim smoke test passes, promote to CRRCSim. If it fails,
stop and fix; promoting a broken pipeline to CRRCSim only wastes training
cycles on a known-bad baseline.

### Phase 3: CRRCSim Training Milestones

**Purpose**: train the actual flight-ready NN. Milestones are observational
layering, not strict gates — same pattern as betterz1/betterz2.

**Starting state**: 1 path, zero-variation baseline with the new
representation. Fixed rabbit at 10 m/s. All entry/wind/rabbit-speed
variation enables OFF. No discontinuity-forcing paths yet. Authority
limit OFF.

**Layering pattern** (work back from the successful 022 approach):
Observe what works on a simple scenario, add the next variation, see it
still works, add the next. Not a strict ordering — use judgment. Typical
progression:

1. **1 path, no variations** — the minisim baseline ported to CRRCSim.
   Real FDM dynamics, Change 1b engage delay window active, Change 1
   history reset active. Verify: NN learns the path, `data.dat` logs
   are clean, discontinuity count in the log is zero (structural
   wrap-freedom from Change 6).

2. **Examine learned behavior** — throttle distribution, surface
   deflection distribution, `d²output²` chatter. Does Change 8's
   authority-limit iteration need to run? Set the trigger criterion
   empirically here (this is the Q4 deferred-to-plan decision).

3. **If triggered: Change 8 authority-limit retrain** — apply
   `NNAuthorityLimit=0.5`, retrain. Compare.

4. **Layer in complexity** — winds, entry attitude/speed, entry position,
   rabbit speed, extra paths, discontinuity-forcing paths (Change 7).
   One at a time, retraining briefly between each. Watch per-path
   brittleness, per-axis saturation, streak diagnostics.

5. **Full curriculum** — all variations on, 5+ paths, discontinuity-forcing
   paths included. This is the 022-betterz-equivalent big training run.

6. **Craft parameter variations (Phase 5 in spec, blended here)** —
   control effectiveness, damping, drag, mass/inertia, control phase
   delay informed by Phase 1a filter audit findings. Start narrow
   (control effectiveness + phase delay), widen.

**CRRCSim phase exits**: flight-ready weights deployable to xiao, all
success criteria 1–9 met at the sim level. Zero discontinuities in
`data.dat` across all training scenarios (structural check on Change 6).

### Phase 4: Xiao Field-Test Prep

1. **Flash the final weights** via `nn2cpp` codegen (which now emits
   struct-field access for the type-safe interface).
2. **Bench verification** — pre-arm, confirm MSP gyro + autoc state
   extensions still healthy, servos respond per calibration.
3. **Pre-flight checklist** — existing ~/autoc/preflight memory drives
   this; verify that the 023 changes (new NN, new history reset, new
   input layout) don't change any pre-flight step.
4. **Field test** — fly. Blackbox analysis on landing must show **zero
   projection discontinuities** in the bearing inputs (Success Criterion
   #6 — structural, not statistical). Post-flight metrics tracked:
   distance distribution, dd/dt, throttle saturation, attitude range,
   discontinuity count (must be zero), per-axis control effort.
5. **Iteration** — if flight fails, diagnose against the diagnostic
   suite from Phase 3 and iterate. Escape-hatch architectures (Phase 1b)
   come into play only if flight fails AND the failure is consistent
   with the escape-hatch hypothesis.

### Deferred to End-of-Feature or BACKLOG

- **AHRS reliability assessment** (from 018 T260–T267). 022 coordinate
  cleanup + 023 representation change are big enough that AHRS is a
  last-resort suspect, not a first-look diagnostic. Quaternion norm
  stability from 2026-04-07 already suggests AHRS is mostly right.
  Check only if 023 flight reveals unexplained attitude inconsistency.
  Likely endpoint: BACKLOG entry.
- **Xiao NN correctness check** (015 T190–T191, nn2cpp bit-identical
  test). Reference-only — don't block on it. Reopen if sim-vs-flight
  output divergence is suspected.

## Questions for Clarification

- **Variation ramp with bigger sigmas**: with `EntryPositionRadiusSigma=30`,
  gen 1 still sees 0% (ramp=0). At gen 41 (ramp 11%) sigma effective is 3.3m.
  At gen 401 full 30m. Do we need a faster ramp to catch up, or keep the
  current 10-step ramp schedule? (plan-phase tuning decision)
- **Throttle epsilon**: how tight should lexicase throttle filtering be?
  Too tight overrides score quality. Too loose has no effect. Start at 0.1?
  (plan-phase tuning decision)

## Research Candidates (end-of-spec, not in main path)

These are interesting ideas considered during scoping but deliberately not
in the 023 implementation. Parked here so they aren't lost.

### Per-Scenario Worst-Case Lexicase Dimension

Current lexicase uses sum-of-negated-scores as its primary dimension. An
individual that scores well on 244/245 scenarios and catastrophically bad
on 1 has similar total fitness to a mediocre-everywhere individual, so
evolution is not pressured to raise the floor. This is the betterz1 path-3
brittleness pattern: the NN finds a stable orbit on the hard path instead
of actually tracking it.

**Proposal**: add `min(scenario_scores)` as a third lexicase dimension
(after score and throttle). Forces evolution to raise the worst-case, not
just the mean.

**Why not in 023 main path**: introduces another dimension to tune (
epsilon, relative weight against score). The mean is already moving
thanks to the 022 conical surface — evidence for "we need floor pressure"
is weak until we see a Phase 3+ run stall the same way betterz1 did. Park
as a candidate for Phase 5+ if brittleness persists.

### Third Lexicase Dimension — Energy / Chatter Penalty

The 022 dev report proposes `Σ d²output²` (second-derivative of control
outputs) as an energy proxy that penalizes bang-bang without penalizing
sustained maneuvers. See the development report for the worked examples
and path-fairness analysis.

**Why not in 023 main path**: Change 8 (authority-limit iteration) is
023's bang-bang hedge, and it's expected to give a stronger signal
because it's a hard cap rather than a tuning knob. If Phase 3c shows
authority limiting is insufficient or produces side effects, re-open
the energy lexicase dimension as a follow-up.

### Wider NN Capacity Experiment

A ~5x weight count experiment against the 023-tuned {33, 32, 16, 3}
baseline. Cost is linear in walltime on evolution side; xiao compute is
still well under budget at 5x. Interesting coupled with the visual/LED
frontend work because visual inputs will want more capacity downstream.

**Why not in 023 main path**: confounds the representation change with a
capacity change. Run both experiments independently for clean A/B signal.

## Parent-Feature Close-Outs Driven by 023

023 subsumes or retargets work from several parent features. Close-out
actions for each:

### 021 — Xiao AHRS Crosscheck

**Status**: CLOSED by 022. 022 betterz2 (V4 conical, 400 gens) produced
a fundamentally better training approach than anything 021's rate-PID +
IMU-crosscheck path produced. The 021 pivot to "NN learns its own rate
control via gyro inputs" is already shipped as part of 022 (gyro p/q/r
at indices 24–26).

**Remaining 021 items**:
- T070–T074 (characterization flight): superseded — 023 *is* the next
  characterization flight under a better controller.
- T083–T084 (post-flight analysis with betterz2): DONE — see
  `flight-results/flight-20260407/flight-report.md`.
- Mark the 021 spec itself CLOSED in its status line.

### 020 — Pre-Flight Pipeline

**Status**: T515–T518 superseded. Latency calibration work overlaps 023
Phase 1 diagnostic (MSP2 latency bound, INAV filter audit). BIG training
is superseded by 022 training infrastructure.

**Action**: Mark T515–T518 as SUPERSEDED BY 023 in 020's tasks.md.

### 019 — Improved CRRCSim

**Status**: T400–T434 (full axis characterization + sim tuning) remain
relevant but are not blocking 023. 023 Change 5 explicitly defers sim
dynamics changes until training results demand them.

**Action**: Keep 019 open. Reference from 023 Change 5 as "unblock path
if Phase 3/4 training fails to close the dynamics gap." Explicit decision
gate in 023 Phase 3b (learned behavior examination).

### 018 — Flight Analysis

**Status**: Several items pulled into 023 Phase 1 diagnostic:
- T260–T267 (AHRS reliability assessment) — 023 Phase 1 item 4.
- T273b (INAV RC filter modeling) — 023 Phase 1 item 2 (filter audit).
- T273h (MSP2 latency reduction) — 023 Phase 1 item 3.

**Action**: Mark these items as IN PROGRESS UNDER 023 in 018's tasks.md.
Remaining 018 post-flight-analysis work (T290–T300) stays in 018 as
ongoing analysis infrastructure.

### 015 — NN Training Improvements

**Status**:
- T140–T143 (aircraft variation / sim-to-real) — pulled into 023 Phase 5
  (Robustness). Implementation happens on 023 branch.
- T190–T191 (Cross-ISA NN bit-identical check) — reference-only in 023
  Phase 1 item 5. Not blocking.
- T130–T133 (path-relative smoothness) — superseded by 023 Changes 3/8
  (throttle lexicase + authority limit).
- T165 (sum mode legacy tearout) — stays in 015, refactoring not
  related to 023.

**Action**: Mark T140–T143 as IN PROGRESS UNDER 023, T190–T191 as
REFERENCED BY 023, T130–T133 as SUPERSEDED BY 023 in 015's tasks.md.

### 014 — NN Training Signal

**Status**: T066–T068 (output directory management) — infrastructure
cleanup, tangentially related to flight debug but not load-bearing for
023. Stays in 014.

# 023 — OOD Coverage, Engage Transient, Throttle Discipline

**Status**: Draft
**Priority**: P0 — blocking next flight test
**Created**: 2026-04-07
**Predecessor**: [022-tracking-cone-fitness](../022-tracking-cone-fitness/spec.md)

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

### 2. INAV Engage Delay (~750ms)

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

- Engage transient fix (history buffer seeding or continuous recording)
- Training regime expansion (position, speed, attitude variations)
- Throttle lexicase dimension (was deferred from 022)
- Rabbit speed simplification (consider fixing 13 m/s)
- Sim dynamics tuning if necessary after training changes

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
2. **INAV servo LPF** — was turned off for this flight (`servo_lpf_hz=0`)
3. **MSP communication latency** — xiao→INAV→servo takes 10-20ms
4. **Actual servo rise time** — physical servos take 50-100ms to move full range
5. **Control force buildup** — aerodynamic response to surface deflection
   isn't instant (boundary layer, etc.) — probably negligible but nonzero
6. **Induced drag change** — response to surface deflection is nonlinear
   and depends on speed

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
- Higher NN sample rate (10 Hz → 20/50 Hz) — requires full retrain with
  different history buffer sizing
- NN authority scaling at xiao boundary (quick experiment, can do separately
  without waiting for full retrain)

## Proposed Changes

### Change 1: Engage Transient Fix (CRITICAL)

Root cause: xiao `recordErrorHistory()` is called during autoc but the
buffer is NOT reset when autoc re-engages (second+ engage per flight).
First engage works because buffer is zero-initialized at boot.

**Fix**: At every autoc transition (`false → true`), reset the history
buffers in `AircraftState`. Either:
- **Option A**: Pre-fill with current in-autoc geometry (dPhi, dTheta, dist
  computed against newly-armed path, pushed to all history slots)
- **Option B**: Zero out all history slots
- **Option C**: Remove the "first engage" special case and always reset
  the same way whether first or re-engage

Options A and C are equivalent if the first-engage path also pre-fills.
Option B is simplest (just `memset`), but closing rate becomes `(0-now)/dt`
which can still be spurious on the first sample.

**Recommended: Option A** — reset-and-fill with current values. First NN
call sees a flat history matching reality, closing rate is zero.

**Code location**: `xiao/src/msplink.cpp` — at the autoc enable transition
(currently line ~402). Also check `include/autoc/eval/aircraft_state.h`
for a `resetHistory()` method or add one.

**Also check**: does this same bug exist in CRRCSim? If the sim re-uses
aircraft state across evaluations, it might have the same stale-history
pattern that the NN was trained against. Probably not (each scenario
creates a fresh state) but worth verifying.

### Change 1b: INAV Engage Delay Modeling

The INAV 750ms delay between "switch on" and "commands effective" means
the entry position variation we train with is NOT what the real aircraft
sees. Current training places the aircraft at `EntryPositionRadiusSigma`
offset and starts NN control immediately. Real flight has the aircraft
drifting for 750ms before NN commands matter.

**Options**:

**A. Simulate the delay** — in sim, feed NN inputs for the first ~750ms
but ignore NN outputs (use current pilot-like stick position or ACRO
behavior). After delay, start applying NN commands.
- Matches real behavior exactly
- Complicates sim pipeline — need to simulate "ACRO pilot" for 750ms
- Requires defining what INAV does during the delay (it's the last
  pilot stick, probably near-neutral for handoff)

**B. Widen entry position variation** — current `EntryPositionRadiusSigma=15`
assumes aircraft starts near path. Increase to 30-50 so training covers
positions the aircraft actually reaches after 750ms of drift.
- Simpler: no sim pipeline changes
- Less accurate: training samples don't correspond to "just post-delay"
  states, just "some far-away state"
- But combined with intercept training, the NN learns to converge from
  any entry point within the larger sigma

**C. Do both** — widen sigma AND simulate delay.

**Recommended: B** for first iteration. A is complex and the exact
pre-engage dynamics are not cleanly modeled anyway. If B doesn't work,
revisit A.

**Question for user**: is "reduce the INAV delay" even possible? Could
xiao pre-arm / pre-engage to cut the 750ms down? That would be the
cleanest fix but may require INAV-side changes.

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

## Parameters — Simple First, Complexity Later

User guidance: get simple working well in sim AND real, then layer
complexity back. Strategy:

### Phase 1: Simplest Possible (get it working)

```ini
# Fitness (unchanged from 022)
FitDistScaleBehind              = 7.0
FitDistScaleAhead               = 2.0
FitConeAngleDeg                 = 45.0
FitStreakThreshold              = 0.5
FitStreakRampSec                = 5.0
FitStreakMultiplierMax          = 5.0

# Training distribution: ZERO ENTRY OFFSETS (023 phase 1)
EntryConeSigma                  = 0     # was 30  — no attitude variation
EntryRollSigma                  = 0     # was 30  — no roll variation
EntrySpeedSigma                 = 0     # was 0.1 — no speed variation
EntryPositionRadiusSigma        = 0     # was 15  — start at path origin
EntryPositionAltSigma           = 0     # was 3   — no altitude offset
WindDirectionSigma              = 0     # was 45  — no wind variation

# Rabbit speed: FIXED, slower
RabbitSpeedSigma                = 0     # was 2    — fixed speed
RabbitSpeedNominal              = 10.0  # was 13.0 — slower for more reaction time

# Paths: keep 5 deterministic, no random
SimNumPathsPerGeneration        = 5     # unchanged

# NN rate (if feasible without full retrain infrastructure changes):
SimTimeStepMs                   = 50    # was 100 — 20Hz for less phase delay
```

Goal: get rock-solid tracking on the 5 deterministic paths with NO
variations at all. This is the baseline — if the NN can't track here,
nothing else matters.

### Phase 2: Layer in Complexity (one variable at a time)

Once Phase 1 is solid, add back:
1. Wind direction variation (sim sees real wind conditions)
2. Entry attitude cone (small at first, grow)
3. Entry speed variation (small)
4. Rabbit speed variation (small)
5. Entry position offset (small, then grow to cover INAV delay drift)
6. 6th path (SeededRandomB) — random generalization
7. Craft parameter variations (control effectiveness, phase delay)

Each addition: one training run, measure retention of Phase 1 quality.

### Phase 3: Robustness (dynamics variations)

Add stddev variations on dynamics parameters (see "Craft Parameter
Variations" research section). Forces policy to generalize across
a neighborhood of the nominal hb1_streamer.xml dynamics.

### Special: Intercept Paths

Add purpose-built paths that test intercept-from-far. E.g., path starts
~30m ahead of the aircraft's natural heading, so the NN must actively
chase. Would be a 7th and 8th path in the aeroStandard set.

## Success Criteria

1. **No engage transient** — first NN call after autoc engage produces
   sensible closing rate (|dd/dt| < 20 m/s, not 1000+)
2. **Training covers real-flight distribution** — sim p99 distance > 40m,
   sim p99 dd/dt extends to negative values (diverging)
3. **Throttle mean < 0.5** in sim training — NN uses < half-throttle on average
4. **Real flight achieves sustained tracking** — distance stays below 30m for
   > 5 seconds on at least one autoc span

## Implementation Order

### Diagnostic Phase

1. **Eval-mode benchmark**: run current 022 NN through eval with random paths.
   Measure distance/streak stats. Compare to real flight stats. Does sim
   reproduce the failure mode?
2. **Xiao NN correctness check**: feed xiao a known sim sequence, verify
   output matches sim bit-for-bit (or within rounding). Rules out code
   generation / FP issues.
3. **Engage transient fix**: xiao-side history reset on every engage.
   Independent of retraining. Test with current NN to see if flight behavior
   improves.

### Training Phase 1 (Simplest)

4. **Zero-variation training**: retrain with all entry sigmas = 0, fixed
   rabbit at 10 m/s, 5 deterministic paths only. Verify NN learns to track
   the basic paths perfectly.
5. **20Hz rate (if possible)**: bump SimTimeStepMs to 50 if it doesn't
   break the history buffer sizing elsewhere. Reduces phase delay.

### Training Phase 2 (Layer Complexity)

6. One variation at a time, confirming each doesn't destroy Phase 1 quality:
   - Wind direction variation
   - Entry attitude/speed variations
   - Rabbit speed variation
   - Entry position offset (small → large)
   - 6th random path

### Flight Validation

7. Phase 1 weights → flight test. Confirm basic tracking on path 0 and 2.
8. Each Phase 2 addition → flight test, verify retention.
9. If fails: diagnose (OOD? dynamics? phase delay?) and iterate.

### Robustness Phase (later)

10. Craft parameter stddev variations
11. Special intercept paths
12. Higher sample rate infrastructure (20 Hz or more)

## Questions for Clarification

- **Streak threshold ramp**: 022 backlog had a deferred streak threshold ramp
  (0.1 → 0.5 over training). Worth including in 023 or skip?
- **Variation ramp with bigger sigmas**: with `EntryPositionRadiusSigma=30`,
  gen 1 still sees 0% (ramp=0). At gen 41 (ramp 11%) sigma effective is 3.3m.
  At gen 401 full 30m. Do we need a faster ramp to catch up, or keep the
  current 10-step ramp schedule?
- **Throttle epsilon**: how tight should lexicase throttle filtering be?
  Too tight overrides score quality. Too loose has no effect. Start at 0.1?

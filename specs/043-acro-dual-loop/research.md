# 043 — Phase 0 research

**Date**: 2026-08-24 · **Spec**: [spec.md](spec.md) (governs) · **Definition**: spec.md § What ACRO is

⭐ **R1 and R9 are answered here from source**, because they size the sim work. R3–R8, R10, R11 are scoped
with a method and an owner phase — several are *deliberately* answered by measurement later, per the spec.

---

## R1 — What INAV's fixed-wing rate loop actually does to a setpoint

**Decision: the model targets `pidApplyFixedWingRateController` as it runs on the config of record.**
Read from the fork at `~/inav` (branch `autoc`), `src/main/flight/pid.c`.

### The loop, in full

```
rateTarget = pidRcCommandToRate(rcCommand, rates)      // ±500 stick ⇒ ±rate×10 °/s
rateTarget = constrain(rateTarget, ±GYRO_SATURATION_LIMIT)
rateError  = rateTarget − gyroRate

fwRateAttenuation(rateTarget, rateError) → aP, aI, aD  // see below — NOT constant gains

P  = ptermFilter(rateError × kP) × aP
D  = dtermLpf(prevGyroRate − gyroRate) × kD/dT × dBoost × aD
FF = rateTarget × kFF                                   // unfiltered, setpoint-only
I += rateError × kI × dT × aI                           // skipped while itermFreezeActive
I  = constrain(I, ±limit × pidItermLimitPercent/100)

axisPID = constrain(P + FF + I + D, ±500)
```

Then `servo[target] += axisPID × smixRate/100`.

### ⭐ Finding 1 — FF dominates, and `Cntrl_Omega` has no FF at all

Gains are the config value divided by a fixed multiplier (`pid.h`), times TPA:

| | config | multiplier | **k** |
|---|---:|---:|---:|
| `fw_p_roll` | 15 | 31 | 0.484 |
| `fw_i_roll` | 3 | 4 | 0.750 |
| `fw_d_roll` | 7 | 1905 | 0.003675 |
| **`fw_ff_roll`** | **50** | 31 | **1.613** |
| `fw_p_pitch` | 15 | 31 | 0.484 |
| `fw_i_pitch` | 5 | 4 | 1.250 |
| `fw_d_pitch` | 5 | 1905 | 0.002625 |
| **`fw_ff_pitch`** | **70** | 31 | **2.258** |

⭐ **`tpa_rate = 0` on this config, so `tpaFactor = 1.0`** — no throttle-dependent gain scaling. One fewer
mechanism to model.

At the *measured* operating point (roll-rate mean |p| = 88 °/s), `FF = 1.613 × 88 = 142` of a ±500 budget,
while P against a 20 °/s error contributes `0.484 × 20 = 9.7` before attenuation. ⛔ **This is a
feed-forward controller with PID trim, not a PID.** A PID-only model would be wrong by roughly an order of
magnitude in the dominant term.

### ⭐ Finding 1a — WHY it is FF-dominant: a stable planform does not need a PID

Operator 2026-08-24: *"in general PID is not needed for stable planform aircraft (vs an unstable quad)."*
⭐ **This is the frame that makes every other finding in R1 make sense**, and it should be read before them.

A multirotor is **open-loop unstable** — without a rate loop closing at kHz it cannot fly at all, so its
PID *is* the aircraft. A stable planform is **open-loop stable**: positive static margin gives pitch
stiffness and the airframe returns to trimmed AoA on its own. ⛔ So the fixed-wing rate loop is not there to
create stability. It is:

- **feed-forward**: "to get rate *r*, deflect this much" — the actual control law, and why `kFF` ≫ `kP`;
- **plus a small PID for trim and disturbance rejection** — correction, not stabilization.

That single fact predicts the rest of R1:

| observation | explained by |
|---|---|
| `kFF` 1.61/2.26 vs `kP` 0.484 | FF is the control law; PID is the correction on top |
| P and D **attenuate toward zero as commanded rate rises** | during a commanded manoeuvre you want the FF mapping; closed-loop correction would be fighting the airframe's own (correct) response |
| loop gain **highest at zero commanded rate** | that is precisely where disturbance rejection is wanted and where no manoeuvre is being flown |
| 039's *untuned* PID still damped the plant to 24 °/s | it only had to add damping to an already-benign plant — it was not holding an unstable one together |

⚠️ **And it explains why autoc's MANUAL-mode architecture was a bad fit.** Direct surface commands at 20 Hz
threw away the feed-forward mapping that INAV's loop already knows, and asked the policy to rediscover it
through a plant with 147° of lag.

### ⛔ Consequence — the model's fidelity now rests on the airframe's OPEN-LOOP stability

If the control law is mostly feed-forward, then **what the aircraft does in response is set by the airframe,
not by the loop**. So a sim whose planform is *more* stable than the real article will produce a
better-behaved modelled inner loop than reality — and the model would look correct while flattering itself.

⚠️ **This is a known open item, inherited.** The 039 backlog entry (*"Pitch marginal-stability levers"*)
already lists **sim static-margin match** as a lever, with an acceptance test: reproduce ~141 °/s pitch RMS
vs ~95 roll replaying the recorded 20 Hz command stream. It also records that aft CG explained only about a
quarter of the excess pitch motion across the 07-13/07-20 A/B, leaving the wing leading-edge geometry and
the streamer in the confound pool.

**Three consequences for 043**, all routed to existing requirements rather than new scope:

1. **R4** (which 037-era constants the flight contradicts) should now explicitly include the
   **static-margin / pitch-damping class** (`Cm_alpha`, `Cm_q`), because FF-dominance makes them
   first-order for the modelled loop rather than background plant detail.
2. **FR-056's static-margin craft axis** moves from "standing candidate" to the **best-motivated** of the
   craft-realism additions: it trains across CG placement instead of tuning the sim to one article, which
   is the right answer when `n = 2` and the spread is still an assumption.
3. ⚠️ **SC-002's divergence must be reported per-axis against the flight's ACRO segments**, not pooled —
   pitch is where the open-loop stability question lives, and pooling would hide it.

### ⭐ Finding 2 — P and D are attenuated by a GAUSSIAN in the setpoint

`fwRateAttenuation` computes `dampingFactor = attenuation(rateTarget, maxRate × fw_iterm_lock_rate_threshold/100)`,
and `attenuation(x, w) = gaussian(x, 0, w/2.35482)`. On this config `fw_iterm_lock_rate_threshold = 40`:

| axis | maxRate | width | **σ** | aP at 0 | at 20 °/s | at 88 °/s |
|---|---:|---:|---:|---:|---:|---:|
| roll | 360 °/s | 144 | **61.2** | 1.00 | 0.95 | **0.36** |
| pitch | 120 °/s | 48 | **20.4** | 1.00 | **0.61** | 0.00 |

⛔ **The gains are not constant — they collapse as commanded rate rises**, and `aP = aD = dampingFactor`.
Two consequences that matter to 043 directly:

1. ⭐ **Highest closed-loop gain is at ZERO commanded rate** — exactly the near-trim condition the outer
   loop spends most of its time in, and exactly where a limit cycle would live. This is a first-class
   candidate mechanism for residual ringing and must be in the model.
2. ⚠️ **The roll/pitch asymmetry is worse than R8 assumed.** σ scales with `maxRate`, so `pitch_rate = 12`
   gives pitch a 3× narrower attenuation window than roll. Pitch goes essentially pure-FF above ~40 °/s;
   roll still has 36% of P at 88 °/s. **The two axes are running materially different controllers.**

### Finding 3 — the I-term has a lock, not just a clamp

`aI = MIN(dampingFactor, hardZero)` where `hardZero` is 0 when **both** `|rateError| > 10% maxRate` **and**
the setpoint moved above 20% maxRate within the last 500 ms (`fw_iterm_lock_*`). Plus a hard clamp at
`pid_iterm_limit_percent = 33` ⇒ **±165** of the ±500 pidsum.
⚠️ Relevant because FR-019 makes the I-term load-bearing for the whole variation regime: the term that
absorbs trim is *itself* suppressed during manoeuvring.

### Finding 4 — D is on the gyro, not the error

`delta = previousRateGyro − gyroRate`, PT2-filtered at `dterm_lpf_hz = 10`, scaled by `kD/dT`. ⭐ Derivative
**on measurement**, so a setpoint step does not produce a D spike — which matters because the 20 Hz outer
loop delivers a step every tick. `d_boost_min = d_boost_max = 1.000` on the AIRPLANE profile ⇒ **D-boost is
a no-op**; do not model it.

### ⭐ Finding 5 — the servo path is IDENTICAL in MANUAL and ACRO

`servos.c` feeds `rcCommand[axis]` (±500) in MANUAL and `axisPID[axis]` (±500) in stabilized modes into the
*same* mixer at the same `rate/100`. `smix speed = 0`, so no per-mix rate limiting.
⭐ **So the sim's existing surface-command path stays valid unchanged.** The ACRO model only changes what
*fills* that ±500 — a much smaller change than "replace the control path", and it preserves the 037 servo
model's position downstream exactly where it belongs.

### Also confirmed

- `rate_accel_limit_roll_pitch = 0` — setpoint acceleration limiting **off** for roll/pitch (FR-019b).
- `GYRO_SATURATION_LIMIT` clamps the setpoint before the loop.
- Yaw: computed, but reaches no surface — no rudder (spec FR-018).

---

## R9 — Reuse `Cntrl_Omega`, or build fresh?

**Decision: reuse the FRAMEWORK, write the MATH. Add `Cntrl_InavFwRate` to `crrcsim/src/mod_cntrl/`.**
✅ **Confirmed by the operator 2026-08-24.**

### Rationale

The valuable part of what exists is the **plumbing**, and it is already correct:

| asset | state |
|---|---|
| `Controller` base class + `LoadList` XML registry | ✅ reuse as-is; add one `else if` |
| gains loaded from XML, no rebuild | ✅ exactly FR-014 |
| `fdm_larcsim.cpp:246` already calls `env->ControllerCallback(...)` per substep | ✅ reuse |
| ⭐ hook fires **before** the 037 servo model (~line 282) | ✅ servo lag correctly lands **inside** the rate loop — the property that makes the modelled phase margin honest |
| `CRRCMath::PT1` filter helper | ✅ reuse |

⛔ But `Cntrl_Omega`'s **math** matches almost nothing R1 found. It is a textbook PID: no feed-forward, no
setpoint-Gaussian attenuation, a different anti-windup (conditional integration on `|pd| > 1`), D on
measurement but without INAV's PT2/scaling, and a *"massive expo"* stick map INAV does not have. Adapting
it would mean replacing every term while keeping the class name — a shim that reads as reuse and is not.

**Estimated size**: one new controller class, ~200–250 lines, in an already-wired framework. This is the
*smaller* of the two readings of R9, and it is smaller because the framework carries the integration risk.

### Alternatives considered

- **Extend `Cntrl_Omega` in place** — rejected: every term changes; it would break the existing controller
  for any other model that uses it, for no saving.
- **Write a bespoke loop inside `inputdev_autoc.cpp`** — rejected: bypasses the XML config path FR-014
  requires, and puts the loop *outside* the substep hook, which would silently move the servo model
  outside the rate loop and destroy the phase fidelity that is the whole point.
- **Full INAV source port** — rejected per the spec's fidelity bar (*"a decent stab, then measure"*): buys
  precision the FDM cannot resolve at the aero limit, and drags in INAV's scheduler and filter stack.

### ⚠️ Open sub-question for the plan

**Inner-loop rate**: the controller runs at the FDM substep. `Global::dt` defaults to `0.002777 s` and is
**ms-quantized** at `crrc_main.cpp:499` ⇒ **~333 Hz**, against INAV's 2 kHz. ZOH phase difference at 5 Hz is
~2.3° (negligible against a 147° budget), but the **D-term and integrator discretization** differ more, and
D is `delta × kD/dT` — explicitly dt-scaled. ⛔ Confirm the as-run `dt` and record the justification; do not
assume. Sub-stepping the controller inside the FDM step is the fallback if 333 Hz proves insufficient.

---

## R3 / R4 — actuator pin, and which 037 constants the flight contradicts

**Method**: bench servo step-response on the flight article → place the real servo inside the existing
`craftServoSlew` (16–32 units/s, centre 24.2) and `craftServoPwmPhase` (0–20 ms) spread. Then a
constant-by-constant pass marking each **"contradicted by 041-t7 and changed"** or **"checked and
unchanged"** (FR-022). ⭐ **Scope now explicitly includes the static-margin / pitch-damping class**
(`Cm_alpha`, `Cm_q`) — see Finding 1a: under a feed-forward-dominant loop these set the modelled response
directly.
⚠️ Now higher-stakes than when written: R1 Finding 5 puts the servo model **inside** the rate loop, so its
accuracy sets the modelled inner loop's phase margin. INAV's gains were tuned against the *real* servo — a
wrong servo model produces a loop with the wrong stability margin, which could show a stable inner loop
where the aircraft rings. **Owner**: Phase 5 (bench), before the bake.

## R5 / R6 / R10 — variations

**R5 (inventory + ramp policy)**: documentation task, resolvable now against
`scenario_meta_apply.h` and `autoc.ini`. ⛔ Known defect to fix: the ini claims craft *"RAMPS with
wind/entry"*; the code ramps **only** the environmental classes.
**R6 (which axes, what spread)**: ⭐ the **static-margin axis is now the best-motivated addition** (Finding
1a) — it trains across CG placement rather than tuning the sim to one article, which is the right move at
`n = 2`. **R6 + R10 (which survive the architecture change)**: ⚠️ answered by
measurement *after* the model exists, and their verdicts are **sim verdicts, provisional on the flight** —
the buried axes resurface at the aero limit, which is where the plant model is weakest (spec FR-059).
**Owner**: Phase 2–3 for the inventory and the new axes; Phase 7+ for the observability verdicts.

## R2 — are the two loops at arm's length?

⛔ **Not answerable yet, by design.** Requires the model *and* a short 45-input baseline. R1 sharpens the
candidates, though: **rate-tracking error** now has a concrete mechanism to reveal — the Gaussian
attenuation means the loop's authority *falls* as commanded rate rises, so a large setpoint is tracked
*worse*, not better. If any inner-loop input earns its keep, this is the leading candidate.
**Owner**: Phase 7, before the bake (a "yes" changes the input vector).

## R7 — INAV telemetry rate

Bench measurement. `blackbox_rate_denom = 32` ⇒ 60 Hz today. Either answer is a result.
**Owner**: Phase 6 (bench).

## R8 — the ±360 / ±120 °/s asymmetry

⚠️ **R1 Finding 2 made this bigger than it looked.** The asymmetry is not only in reachable rate; it
propagates into the *attenuation width* (σ 61.2 roll vs 20.4 pitch), so the two axes run materially
different effective controllers. ⛔ Still no config change (gains and rates stay as-is per the spec) — but
the plan must **document the per-axis effective gain curve**, because it is a likely contributor to the
per-axis behaviour differences 041 reported. **Owner**: Phase 4, as model documentation.

## R11 — INAV phase-delay parameters

⭐ Reframing stands and R1 strengthens it: with `aP = aD = 1.0` at zero commanded rate, the loop is at
maximum gain exactly where sensor delay costs the most margin. 6.4 ms of gyro PT1 is 11° at 5 Hz but
**115° at 50 Hz**. Candidates per FR-012a; ⛔ every change must land on **both** sides.
**Owner**: Phase 5–6, computed against the model, bench-verified before the bake.


---

# Addendum 2026-08-25 — the knob inventory, and the 333 Hz question

Prompted by the operator: *"how are we going to go about emulating INAV in the sim… will the model be
roughly the same — how many knobs do we have to worry about (or perhaps model in variations?)"*

## ⛔ Correction — `acc_lpf_hz` is NOT in the rate loop

Spec FR-011/FR-013 list `acc_lpf_hz = 15 (BIQUAD)` as an inner-loop term at 15 ms, *"the largest sensor
delay"*. **Wrong for ACRO.** `pidApplyFixedWingRateController` reads `pidState->gyroRate` and nothing else;
the accelerometer reaches the controller only through `attitude.raw[]`, which **only ANGLE/HORIZON read**.

⭐ So the 15 ms accel delay contributes **zero phase to the ACRO loop**. It still matters — it shapes the
policy's `ACCEL_*` inputs and the `quat` it observes — but that is the **observation** path, not the
control path. Model it there; ⛔ do not put it in the inner loop, and drop it from the FR-012a phase-delay
candidate list on inner-loop grounds.

## The knob inventory

**~17 numbers, of which 10 are read straight off the flying config and 4 are INAV defaults we never touch.**

### A. Must model — the controller (14)

| knob | roll | pitch | source |
|---|---:|---:|---|
| `kP` | 0.484 | 0.484 | `fw_p_*/31` |
| `kI` | 0.750 | 1.250 | `fw_i_*/4` |
| `kD` | 0.003675 | 0.002625 | `fw_d_*/1905` |
| `kFF` | 1.613 | 2.258 | `fw_ff_*/31` |
| `maxRate` °/s | 360 | 120 | `*_rate × 10` |
| `dterm_lpf_hz` / type | 10 / PT2 | ← | config |
| `itermLockRateThresholdPct` / `engageThresholdPct` / `lockTimeMaxMs` | 40 / 10 / 500 | ← | INAV defaults |
| `itermLimitPct` / `pidSumLimit` | 33 / 500 | ← | INAV defaults |

### B. Must model — one sensor filter (1)

`gyro_main_lpf_hz = 25` (PT1), 6.4 ms. ⭐ **The only sensor filter inside the ACRO loop.**

### C. Model as ABSENT — with the reason recorded

| knob | value | why absent |
|---|---|---|
| TPA | `tpa_rate = 0` | `tpaFactor = 1.0` — identity |
| D-boost | `min = max = 1.000` | identity |
| setpoint accel limit | `rate_accel_limit_roll_pitch = 0` | off |
| anti-aliasing LPF | 250 Hz PT1 | **1.15° at 5 Hz** — below the noise floor of this exercise |
| **`acc_lpf_hz`** | 15 BIQUAD | ⛔ not in the rate loop at all — see the correction above |
| `pidLevel` / self-levelling | — | ANGLE-only; including it is the FR-019a defect |
| yaw output | — | no rudder |

### ⚠️ D. One genuinely open item — the dynamic gyro notch

`dynamic_gyro_notch_q = 250` is **divided by 100** (`dynamic_gyro_notch.c:43`) ⇒ **Q = 2.5**, i.e. a *broad*
notch (~12 Hz wide at a 30 Hz centre), not the razor a raw "250" suggests. It cannot be dismissed on
narrowness.

But it is **dynamic**: it tracks the detected vibration peak, and `dynamic_gyro_notch_min_hz = 30` is only
the floor. A 5.5″ prop at flight RPM puts its fundamental in the low hundreds of Hz, where a Q-2.5 notch
contributes essentially nothing at 2–10 Hz.

⛔ **Unresolved tension**: the README records that the *measured* roll-off on flight data is **steeper than
PT1 alone**, and credits the notch. If the notch really is sitting near its 30 Hz floor in flight, it is
within a factor of six of the control band and does contribute phase.
**Action**: read the notch centre frequency from the 041-t7 blackbox (it is logged) before deciding.
Cheap, and it converts an assumption into a measurement. Until then: **model as absent, flagged.**

## Is 333 Hz enough? — yes, and here is the arithmetic

The controller runs at the FDM substep (`Global::dt` = 0.002777 s, ms-quantized ⇒ ~333 Hz) against INAV's
2 kHz. Extra ZOH phase from the slower rate is `½ × dt × f × 360°`:

| frequency | 333 Hz | 2 kHz | **difference** |
|---|---:|---:|---:|
| 5 Hz (the oscillation band) | 2.7° | 0.45° | **2.3°** |
| 10 Hz (predicted crossover after fixes) | 5.4° | 0.9° | **4.5°** |
| 30 Hz | 16.2° | 2.7° | 13.5° |

⭐ **In the band that matters the discrepancy is under 5°, against a budget of 147°.** The operator's
reasoning holds and is the right one: *an airplane has nothing to say above ~20 Hz*, whereas a quad's
motor/prop harmonics couple straight into its control loop and demand the kHz rate.

Two supporting details:
- **The dt-scaled terms are dt-consistent.** `I += error × kI × dt` and `D = delta × kD/dt` both give the
  same physical result at either rate; only the filter coefficients must be recomputed from the actual dt.
- **D differentiates a band-limited signal.** The gyro is PT1-filtered at 25 Hz and the D path PT2 at
  10 Hz, so there is no content near the 166 Hz Nyquist to alias.

⚠️ **Confirm the as-run `Global::dt`** rather than assuming the default, and record the justification.
Sub-stepping the controller inside the FDM step is the fallback, and is not expected to be needed.

## Which knobs deserve VARIATION rather than a pinned value?

⭐ Vary what is **uncertain**, not what is **known**. The gains are known exactly — they are read from the
config — so varying them for *uncertainty* would be theatre.

There are two honest cases for varying them anyway:

1. ⚠️ *(declined — recorded for the follow-on)* **Robustness to a future retune.** FR-012a already opens the door to changing INAV parameters, and
   the loop may be tuned properly later. A policy trained across a gain range survives that; one trained
   on today's exact numbers may not.
2. **Insurance against our model of the gains being slightly wrong** — the `/31`, `/4`, `/1905`
   multipliers, the TPA assumption, the attenuation curve.

**Recommendation was: ONE axis, not seventeen** — a single *inner-loop authority* scale on the whole
`{kP, kI, kD, kFF}` set.

⛔ **DECLINED by the operator 2026-08-25 — spec.md § Clarifications (session 3) governs.** `craftGyroScale`
alone carries the commanded-rate-≠-achieved-rate uncertainty; shipping both would double-count it, and the
gains are known exactly from the config so varying them for *uncertainty* is theatre. Robustness to a
future INAV retune is real but is a follow-on concern. ⭐ **The double-count caution below is what
survived, and it was applied again** to split `craftCmQ` (dynamic) from the existing `craftCGDelta`
(static) rather than adding a redundant static-margin axis.

⚠️ **And check it against what is already there before adding it.** `craftGyroScale` (contracts/craft-imu-axes.md)
already perturbs *commanded rate vs achieved rate* — a gyro scale error and an FF scale error are close to
the same disturbance from the policy's seat. ⛔ Adding both without thinking would double-count the one
uncertainty that matters most. Decide which carries it; do not ship both by default.

⛔ **Do NOT vary**: the I-term lock constants, `itermLimitPct`, `pidSumLimit`, the attenuation thresholds.
They are INAV defaults, we will not change them, and each one varied is a knob the plan must later explain.

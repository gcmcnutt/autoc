# 043-t2 — flight analysis (2026-09-05)

Genome **gen 800** (`weight_id=3af8e3ab787b75a5`), first flight of the ACRO dual-loop architecture.
Logs: `flight-results/flight-20260905/`. Four engagements, 1011 ticks, 0 gaps, 0 overruns, 0 drops.

⭐ **Headline: the architecture worked; a frame bug in the engage prefill fed the policy 800 ms of
6–8× out-of-scale input at the start of every engagement, and the sim does not have it.**

---

## 1. ⛔ ROOT CAUSE — the engage history prefill uses the wrong frame

At engage the xiao pre-fills all six NN history slots so the policy starts with consistent geometry
(`AircraftState::resetHistory`, `xiao/src/msplink.cpp:864`). It is being called **one MSP cycle too
early**:

| line | what happens |
|---|---|
| `msplink.cpp:768` | `test_origin_offset` = absolute INAV position, captured at engage |
| `msplink.cpp:864` | `resetHistory(flight_path[0].start, tangent)` — **fires here** |
| `msplink.cpp:1087` | `position_rel = position_raw - test_origin_offset` — **only on the NEXT MSP update** |

So at the moment of the prefill, `aircraft_state.position` still holds the **absolute** NED position
(≈ the origin) while `flight_path[0].start` is **arena-relative** (≈ 0). `resetHistory` computes
`craftToTarget = targetPos − position`, which evaluates to **−origin**.

⭐ The prediction is exact — the stale distance should equal ‖arena origin‖:

| run | arena origin (N,E,D) | ‖origin‖ | observed `dist_0` |
|---|---|---:|---:|
| flight span 1 | (143.4, −25.7, −70.0) | 161.6 | **162.2** |
| flight span 2 | (187.8, −29.5, −90.8) | 210.7 | **208.0** |
| flight span 3 | (157.4, −36.4, −84.9) | 182.5 | **183.2** |
| flight span 4 | (162.8, −7.6, −95.8) | 189.0 | **189.5** |
| bench 2026-09-04 | (−3.76, 4.01, 0.48) | 5.5 | **5.5** |

The residual is just the craft's own offset from the origin at engage. This is not a hypothesis.

**Consequence.** `kTargetDistScale_m = 26`, so the policy is fed `dist ≈ 6.2–8.0` in normalised units
against a trained range of roughly 0–2, in **5 of the 6 history slots**, together with the matching stale
`target_x/y/z` direction cosines — until the ring flushes at the 800 ms lag, i.e. **16 ticks**.
`closing_rate` is corrupted for the first ~2 ticks by the same mechanism.

⛔ **The network is recurrent (`45→32→16r→3`).** `recurrent_reset` fires correctly at tick 0, so the hidden
state starts clean — and is then driven for 16 ticks by inputs it has never seen. The corruption does not
end when the buffer flushes.

### ⚠️ Why the bench could not catch this

The magnitude of the garbage is ‖origin‖, and the bench sits **5.5 m** from the NED datum while the field
puts the aircraft **160–210 m** from it. The bench reproduces the bug *structurally* — the same fill
pattern is visible in both bench logs — but at an amplitude that is inside the trained range and therefore
harmless. ⭐ **A static bench near the datum cannot surface this class of fault. That is the lesson, not
that the bench was run badly.**

### The sim is correct, which is why training never saw it

`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp:754` assigns
`aircraftState = initialState` **before** the prefill, with a comment saying exactly why, and
`virtualInitialPos = initialPos − pathOriginOffset` is already arena-relative. The sim prefills all six
slots with the correct geometry and `closing_rate = 0`.

⇒ **This is a pure sim↔real divergence in the input vector**, not a policy failure and not a physics
mismatch. The policy was asked a question it was never trained on, four times.

---

## 2. Did ACRO work? — YES, unambiguously

| check | result |
|---|---|
| Mode, all four spans | **100.0% / 99.7% / 100.0% / 100.0%** `ARM\|MSPRCOVERRIDE`, no MANUAL. ACRO held for every engaged tick. |
| Clock join | **−944.5 ppm**, 1011/1011 ticks matched within 30 ms, median skew 4.2 ms. ⭐ Cross-validates 041-t7's −970 ppm. |
| Yaw silence | `axisRate[2]` = `axisF[2]` = **0.0** in all four spans (FR-018). |
| Loop health | 0 overruns, 0 resyncs, 0 drops, maxLate 15–25 ms across 1011 ticks (SC-006 ✅). |

### The rate loop tracks — with 68–84 ms of lag and about half the gain

Measured on the blackbox at its native 59 Hz (no join needed — both signals are INAV's own), sweeping lag:

| axis | best lag | r @ best | slope | r @ zero lag |
|---|---:|---:|---:|---:|
| ROLL | **84 ms** | +0.72 … +0.79 | 0.51 … 0.61 | +0.10 … +0.57 |
| PITCH | **68 ms** | +0.46 … +0.77 | 0.26 … 0.67 | +0.36 … +0.72 |

⚠️ Two things to sit with. The loop **delivers only ~50–60% of the commanded rate**, and the residual
delay is **68–84 ms** — the same order as the 81.6 ms phase budget 043 set out to defeat. Moving the
stabilisation inboard did not remove the phase; the NN command still arrives over a 20 Hz ZOH, and the
airframe/actuator lag is untouched. ⭐ This is the number that decides whether 043's thesis is complete.

### SC-001 — roll-rate spectrum: real improvement, not a fix

| roll-rate power | 0–1 Hz | 1–3 Hz | 3–5 Hz | 5–10 Hz | mean \|p\| |
|---|---:|---:|---:|---:|---:|
| 041-t7 **real** (baseline) | 30.6% | — | **30.1%** | 7.4% | 87.3 |
| 041-t7 **sim** (target) | 55.9% | — | 12.6% | 2.3% | 88.4 |
| **043-t2 real, all spans** | 40.2% | 33.3% | **17.7%** | 8.2% | 86.6 |
| per span | | | 30.3 / 33.6 / **15.2** / **5.7**% | | |

⭐ **3–5 Hz fell 30.1% → 17.7%**, about halfway to sim, at matched rate amplitude (86.6 vs 87.3 — the
amplitude match 041 established still holds). ⚠️ But **5–10 Hz did not improve** (7.4% → 8.2%), and the
per-span spread is enormous: spans 1–2 are no better than 041-t7, spans 3–4 beat the sim target. The
improvement is not uniform, and spans 1–2 are the two most affected by the prefill bug.

⛔ **Pitch is the new problem.** 67.2% of pitch-rate power now sits at **1–3 Hz** with RMS **110.7 °/s** —
against 039's references of 24 °/s for INAV acro *"on a rail"*, 50 for pilot MANUAL, and 141 for NN-direct.
The oscillation did not disappear; in pitch it moved **down** in frequency and stayed large.

---

## 3. Sim vs real dynamics — NOT similar (SC-002)

| measure | sim gen 800 | real | ratio |
|---|---:|---:|---:|
| body accel \|a\| median | 1.49 g | **2.33 g** | **1.56×** |
| body accel p95 | 4.19 g | 5.61 g | 1.34× |
| mean \|p\| (roll rate) | 97.9 °/s | 89.4 | 0.91× |
| mean \|q\| (pitch rate) | 65.1 °/s | **91.3** | **1.40×** |
| mean \|r\| (yaw rate) | 52.3 °/s | 29.9 | 0.57× |

⚠️ **Body acceleration is 1.56× sim** — the direct descendant of 041-t7's *"roll acceleration is 1.66× sim
at matching rate amplitude"*. ⛔ **043 did not close that gap.** Note this is the opposite sign to the
first read at the field: the real aircraft pulls **more** g than the sim, not less.

Pitch rate is 1.4× sim while roll is 0.9× and yaw 0.6× — the divergence is now **concentrated in pitch**,
which is where the open-loop stability question lives (research.md Finding 1a).

---

## 4. Tracking — clearly worse than sim (SC-003 ✗)

| distance to rabbit | median | mean | <5 m | <10 m | <20 m |
|---|---:|---:|---:|---:|---:|
| sim gen 800 | 4.1 m | 6.3 m | **60.7%** | 84.7% | 95.5% |
| **real** | 11.1 m | 16.2 m | **16.2%** | 44.3% | 81.0% |

Close-tracking occupancy collapsed **60.7% → 16.2%**. Per-span the craft starts on the rabbit and then
falls behind monotonically — median distance by tick window:

| span | ticks 0–15 | 16–40 | 41+ |
|---|---:|---:|---:|
| 1 | 3.3 | 13.7 | 21.9 |
| 2 | 3.1 | 12.8 | 14.9 |
| 3 | 1.6 | 8.0 | 9.3 |
| 4 | 2.0 | 6.8 | 32.1 |

⭐ Ticks 0–15 are exactly the corrupted window; the craft is close there only because the rabbit *starts*
on it. It never recovers the gap afterwards.

## 5. Throttle — the modulation is real and it is not in the sim

| | sign-changes | % railed |
|---|---:|---:|
| sim gen 800 (median over 294 scenarios) | 1.89 Hz | **22%** |
| **real, whole flight** | **3.96 Hz** | **73%** |
| span 1 / 2 / 3 / 4 | 1.21 / 2.41 / **6.51** / 1.31 Hz | 85 / 79 / 62 / 83% |
| bench 2026-09-04 (static) | 0.23 Hz | 85% |

Real throttle switches **2× faster** than sim and is railed **3.3×** as often; span 3 reaches **6.51 Hz**.
⭐ This is the same *class* of finding as 041's roll oscillation — a channel that is smooth in sim and
chattering in the air — now in throttle. It is a genuine sim↔real divergence and is not explained by the
prefill bug alone (it persists long past the 800 ms window).

## 6. Span 1 — the "idle dive", explained

66 ticks, 3.3 s, pilot took it back (`DISENGAGE_REASON=1`, servo switch). Throttle **74% at the idle
rail**, pitch mean −0.59 with a sustained nose-down hold, descent reaching **8.6 m/s**, airspeed decaying
21.3 → 14.6 m/s.

⭐ It engaged at **18.5 m/s while climbing at 2.9 m/s** — already fast against the 13 m/s trained cruise —
and then took 800 ms of `dist ≈ 6.2` garbage into a freshly-reset recurrent state. The response is the
documented OOD nose-down rail (`specs/BACKLOG.md`), the same signature the 2026-09-04 bench produced and
the same one that kills 218 of 294 `random` scenarios. **The pilot's read was correct.**

## 7. Span 4 discontinuity — INAV vertical estimator reset, NOT GPS

At t = 267.41 s, `pos_d` steps **−70.50 → −24.12 m** (46.4 m) and `vel_d` jumps −4.3 → **+20.1 m/s**,
while `pos_n`/`pos_e` step a normal 1 m. Horizontal is untouched; GPS is fine.

The blackbox shows the cause building for seconds beforehand: `navPos[2]` climbing while `BaroAlt` falls,
with `navEPV` growing monotonically **398 → 998** — then at the step, `navEPV` snaps back to **390**. That
is the estimator rejecting its own vertical state and re-initialising to the measurement.

| reset | t (s) | step | EPV before → after | navPos−baro gap |
|---|---:|---:|---:|---:|
| 1 | 265.51 | +23.9 m | 998 → 369 | +27.2 m |
| 2 | 267.37 | **−45.5 m** | 994 → 390 | **+85.6 m** |
| 3 | 277.80 | +13.1 m | 999 → 353 | +0.3 m |

⛔ **The vertical estimator was unhealthy for the whole flight, not just span 4.** `navPos[2] − BaroAlt` by
span: **+0.5 / +11.2 / +4.3 / +3.5 m** median, with maxima **−11.0 / +27.4 / +22.2 / +85.8 m**. A healthy
estimator tracks baro to a few metres. `accVib` is **2039** in span 1 and **4340–4596** in spans 2–4.

⚠️ This corrupts the NN's altitude-derived inputs directly: `specific_energy` is scaled by
`kEnergyScale_m = 145`, so a 27 m altitude error is **0.19** in normalised units, and `dist_to_boundary`
and the engage-arena floor/ceiling geometry ride on the same estimate. Two of the three resets land
**inside span 4**.

⭐ Suspect ordering: high `accVib` → accelerometer fusion drives vertical velocity drift → EPV grows →
reset. The 2026-09-04 accel recalibration left `ins_gravity_cmss` at **972.092** against a true ~979,
still ~0.7% low, which biases exactly this integration.

---

## 8. What this flight does and does not tell us

**Established.** ACRO engages, holds, and releases correctly; the NN commands INAV's rate loop end to end;
yaw stays silent; the 20 Hz loop is healthy; the clock-join method reproduces at −944.5 ppm; 3–5 Hz roll
power dropped meaningfully.

⛔ **Not established, because the input vector was corrupted at every engagement.** No conclusion about the
*policy's* competence — tracking, throttle discipline, or the OOD rail — is safe from this flight. The
comparison that SC-002 and SC-003 need is against a flight where the prefill is right.

**Ordered by expected value:**

1. ⭐ **Fix the prefill frame** (`msplink.cpp`: re-express `aircraft_state.position` into the engage frame
   before line 864, mirroring CRRCSim's ordering). One-line class of change; it invalidates nothing else.
2. **Re-fly.** Everything in §3–§5 needs re-measuring against a clean engage before it can be attributed.
3. **Investigate the vertical estimator** — `accVib` 4300+ and an 85 m excursion are their own problem and
   feed the NN directly.
4. Only then judge the 043 thesis on §2's 68–84 ms residual lag and 0.5–0.6 loop gain.

⚠️ **Add a non-static engage check to the bench protocol.** The bench passed every gate and could not have
caught this, because the fault scales with distance from the NED datum.

---

# Addendum (2026-09-05) — streak, and how far the rate model is off

## 9. ⭐ Is streak measured, or fed to the model? — **Neither, since 041 P2-2**

⛔ **`streak` is NOT an NN input, and has not been one since 041.** It was deliberately removed and replaced
by `score_grad`. From `include/autoc/nn/nn_inputs.h:264`:

> Operator: *"streak was a crude proxy for rewarding in-track range."* A binary flag is a **STATE LABEL** —
> a controller can only switch on it. A gradient is an **IMPROVEMENT DIRECTION**: which way to move, in
> body axes, to score more, **weighted by the streak multiplier** so it carries how much reward is
> currently at stake.

So streak reaches the policy through exactly one channel: `score_grad_x/y/z`, the spatial gradient of the
step score scaled by the streak multiplier. Streak itself is a **fitness-side** quantity only.

### The cone constants are NOT broken

Flown genome (baked into the firmware) vs `autoc.ini` — identical on every term:

| | behind | ahead | cone half-angle | streak threshold | ramp |
|---|---:|---:|---:|---:|---:|
| `autoc.ini` | 7.0 m | 2.0 m | 45.0° | 0.5 | 5.0 s |
| flown firmware | 7.000 | 2.000 | 45.000 | 0.500 | 5.000 |

⇒ **No threshold, slope or orientation was broken.** The geometry the policy flew is the geometry it
trained against.

### ⛔ But the guidance signal has NO long-range component, and this flight lived at long range

`|score_grad|` against distance, measured from the sim (which spans both regimes):

| distance | 0–2 m | 2–4 | 4–6 | 6–8 | 8–10 | 10–15 | 15–20 | **20–30** | **30–60** |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| median \|sg\| | 0.703 | 0.526 | 0.144 | 0.071 | 0.048 | 0.029 | 0.015 | **0.006** | **0.001** |
| fraction ≈ 0 | 1.3% | 0.0% | 0.0% | 0.0% | 5.8% | 4.0% | 6.6% | **92.2%** | **100%** |

⭐ **Beyond ~20 m the gradient is dead** — 92% of ticks carry no usable direction at all. That is inherent
to a Lorentzian with `distScaleBehind = 7 m`; it is not a bug, but it means **the reward field gives no
way home once you are outside it.**

Where each run spent its time:

| | 0–4 m | 4–8 m | 8–15 m | 15–30 m | >30 m |
|---|---:|---:|---:|---:|---:|
| sim gen 800 | **47.9%** | 31.0% | 13.1% | 7.0% | 1.0% |
| **REAL** | 8.1% | 27.0% | 31.7% | **22.5%** | **10.8%** |

Sim is inside 8 m — where the gradient is strong — **78.9%** of the time. The flight managed **35.1%**, and
spent **a third of the flight beyond 15 m**, where the signal is nearly gone.

Measured `|score_grad|` in flight is **7× weaker than sim** (median 0.0271 vs 0.1895) and **26.8% of ticks
are at ≈ 0** against sim's 5.1%. Per span:

| span | median \|sg\| | ticks ≈ 0 | median dist |
|---|---:|---:|---:|
| 1 | 0.0038 | **72.7%** | 14.8 m |
| 2 | 0.0129 | 42.7% | 14.2 m |
| 3 | 0.0478 | 1.6% | 8.6 m |
| 4 | 0.0228 | 42.6% | 12.8 m |

⭐ **Span 1 flew three-quarters of its life with no improvement direction at all** — and span 1 is the one
that dove. ⭐ Span 3, the only span that stayed near 8 m, is also the only one with a live gradient (1.6%
dead) — and it has the best 3–5 Hz number in §2 (15.2%).

⇒ **The failure is self-reinforcing.** Corrupted prefill pushes the craft off the rabbit → distance passes
~15 m → `score_grad` collapses → the policy has no direction to recover on → distance grows. The answer to
*"the craft never got close"* is that after the first second it had nothing telling it how.

## 10. How far off is the rate model, exactly?

Commanded rate → achieved body rate, both sides, lag-swept:

| | best lag | r | slope (delivered fraction) |
|---|---:|---:|---:|
| **sim** ROLL | ~150 ms | +0.78 | **0.74** |
| **REAL** ROLL | **84 ms** | +0.72 … +0.79 | **0.51 … 0.61** |
| **sim** PITCH | ~100 ms | +0.66 | **0.43** |
| **REAL** PITCH | **68 ms** | +0.46 … +0.77 | **0.26 … 0.67** |

⚠️ The sim is sampled at 20 Hz, so its lag resolves only to ±50 ms; the real side is blackbox-native
59 Hz (±17 ms). Treat the sim lag as coarse.

⭐ **The error runs in both directions, which is why it is worth fixing rather than trimming.** The real
aircraft is **faster** than the model (84 ms vs ~150 ms in roll) but has **less authority** (delivers
0.51–0.61 of commanded rate against the model's 0.74). The *correlation* is comparable (~0.78 both), so
the model has the right shape and the wrong constants: **too laggy, too strong.**

A policy trained against a loop that is slower and more powerful than reality will command as though its
inputs will be followed, and get about **75–80%** of the roll rate it expects, **68 ms sooner** than it
learned to expect it. That is a plausible contributor to the throttle chatter in §5 and the pitch energy
at 1–3 Hz in §2, independent of the prefill bug.

⇒ **Route to Phase 6 (T046–T050a), which never ran.** This is exactly the actuator/plant pinning
SC-010 called for, and it now has measured numbers to hit instead of assumed ones.

---

# Addendum 2 — playback reconstruction, and where the responsiveness work should go

## 11. Renderer / xiao playback: streak is RECONSTRUCTIBLE, not stored

| | stepScore | streak flag | multiplier | score_grad |
|---|---|---|---|---|
| **sim** (dmp) | ✅ `stpPt` | derived from `stpPt ≥ 0.5` | ✅ `mult` | ✅ `sgX/Y/Z` (+ `sgx/y/z` raw) |
| **xiao** (v4 log) | ❌ | ❌ | ❌ | ✅ `score_grad_x/y/z` (post-multiplier, tanh'd) |

The xiao *computes* all of it on board (`msplink.cpp` — `score_envelope.advance(stepScore >= threshold)`)
but logs only the gradient. ⭐ **It does not need to log the rest**: the v4 log carries `pos_n/e/d` and
`rabbit_n/e/d`, and the cone constants are baked into the firmware, so the renderer can rebuild the exact
surface. `FitnessComputer` is compiled into the xiao firmware precisely so the objective has ONE definition
(041 P5-3) — the renderer should call it rather than re-implement the Lorentzian.

Reconstructed for this flight (tangent from successive rabbit positions):

| span | stepScore median | **in-streak** |
|---|---:|---:|
| 1 | 0.024 | **3.0%** |
| 2 | 0.113 | 4.0% |
| 3 | 0.277 | **17.3%** |
| 4 | 0.177 | 4.4% |
| **whole flight** | | **9.8%** |
| **sim gen 800** (recorded `stpPt`) | 0.549 | **54.6%** |

⇒ **9.8% real against 54.6% sim.** That is the tracking result stated in the objective's own currency.

## 12. ⚠️ Correction — the bearing signal never degrades

An earlier claim in §9 that the policy has "no way home" beyond 20 m was **wrong**, and the operator caught
it. Measured over this flight:

| range | 0–4 m | 4–8 | 8–15 | 15–30 | >30 |
|---|---:|---:|---:|---:|---:|
| median \|target\| (direction cosines) | 1.0000 | 1.0000 | 1.0000 | 1.0000 | **1.0000** |
| median \|score_grad\| | 0.162 | 0.088 | 0.028 | 0.011 | 0.0005 |

⭐ `target_x/y/z` are **unit direction cosines at every range**, and `dist` is linear (not tanh) precisely
so the ratio structure survives at all ranges (041 P2-8). The policy always knows **where** the target is
and **how far**. What decays is only *how much reward is at stake* — `score_grad` is a **quantitative
nudge**, not the navigation signal. Operator's framing, and it is the correct one.

## 13. Responsiveness: what this flight can and cannot pin

⛔ **In-flight actuator identification is not available from this log.** Fitting elevon deflection (common
= pitch, differential = roll) against achieved rate *and* against angular acceleration gives r = 0.03–0.19
in both modes. Cause: `blackbox_rate_denom` drops **96.87%** of loop iterations (299,398 of 309,057), so
the 59 Hz record cannot support a derivative-based plant fit. ⇒ **Raise the blackbox rate before the next
flight if in-air actuator identification is wanted**, or accept that T046's bench step-response is the
instrument for this — which is what the plan already said.

What this flight *does* pin, cleanly, is the **closed loop** (both sides lag-swept, §10): real roll 84 ms
/ gain 0.51–0.61 against the model's ~150 ms / 0.74, r ≈ 0.78 on both.

### The controller model is NOT the gap

`crrcsim/src/mod_cntrl/cntrl_inavfwrate/` models INAV's fixed-wing rate loop **including the
`pidSumLimit` clamp** (`cntrl_inavfwrate.cpp:85-88`), the FF-dominant gains and the Gaussian P/D
attenuation. ⇒ The 0.74-vs-0.55 authority gap is **not** an un-modelled controller behaviour. It is
airframe aero or actuator — i.e. exactly Phase 6's territory, not `cntrl_inavfwrate`'s.

## 14. ⭐ Are we a broken record? — the guard is already written, and it says wait

Operator, commit `2c691aa` (2026-08-23), *before* this feature was planned:

> `craftServoSlew` and `craftThrustTau` are **ALREADY variation classes** in `ScenarioMetadata`… The sim
> can express a spread of actuator responses today; what is missing is knowing **where the real airframe
> sits inside it**. The **2nd flight article** is what makes the spread measurable — two airframes give a
> real read rather than a single sample.
>
> ⛔ And this is a **FINE-TUNE, not a re-derivation.** The 037-era actuator and latency constants were
> measured on this same airframe and much is unmodified. Do not treat every constant as suspect; find the
> ones the flight data actually contradicts and leave the rest alone.

The prescribed pre-work — xiao log self-sufficiency, a bench servo step-response to pin the 30 ms term,
and a pass over the 037 constants — **is Phase 6 (T046–T050a), which never ran** before the bake.

⇒ **We are not repeating ourselves; we are arriving at the step we skipped.** The sim latency model
(`gComputeLatencyMsec`, staged-command application in `inputdev_autoc.cpp:1545`) and servo v2 (PWM latch +
slew, `ServoModelEnabled = 1`) both already exist and were measured on this airframe.

### Recommended order — ⛔ do NOT tune `hb1_streamer.xml` or the latency constants yet

1. **Reflash and re-fly** with the prefill fix. Every closed-loop number above was measured while the
   policy was being driven out of distribution; they are contaminated as a tuning target.
2. **Run T046** — bench servo step-response. Deterministic, weather-free, and the 30 ms actuator term is
   the largest purely-modelled term in the phase budget.
3. **Raise `blackbox_rate_denom`** for the next flight so the plant is measurable in the air at all.
4. **Then** compare across MANUAL / ACRO / the 041-t7 and 039 records, and fine-tune only the constants
   the data contradicts.
5. **Train last.** ⛔ Baking now would spend 800 generations against constants we are about to change —
   and the 043-t2 bake already cost that once.

⚠️ `n = 1` article and `n = 1` flight. The repo's own standard for moving the actuator constants is the
**second article**. Nothing here is strong enough to overturn 037-era numbers measured on this airframe.

---

# Addendum 3 — INAV hb1 config audit against the sim dynamics

Source: `xiao/inav-hb1.cfg` (the 2026-09-04 flight-article capture) vs
`crrcsim/models/hb1_streamer.xml` + `crrcsim/src/mod_cntrl/cntrl_inavfwrate/`.

## 15. ⭐ The rate controller is a PARAMETER-EXACT replica — no gap here

Every gain converts with **one consistent scale factor per term, across both axes**:

| axis | term | FC | sim model | ratio |
|---|---|---:|---:|---:|
| roll | P / I / D / FF | 15 / 3 / 7 / 50 | 0.484 / 0.750 / 0.003675 / 1.613 | 0.0322667 / 0.25 / 0.000525 / 0.03226 |
| pitch | P / I / D / FF | 15 / 5 / 5 / 70 | 0.484 / 1.250 / 0.002625 / 2.258 | 0.0322667 / 0.25 / 0.000525 / 0.032257 |

And every structural parameter matches outright:

| | FC | sim |
|---|---|---|
| `gyro_main_lpf_hz` / `gyroLpfHz` | 25 | **25** ✅ |
| `dterm_lpf_hz` / `dtermLpfHz` | 10 | **10** ✅ |
| `fw_iterm_lock_rate_threshold` | 40 | **40** ✅ |
| `pid_iterm_limit_percent` / `itermLimitPct` | 33 | **33** ✅ |
| `pidSumLimit` | ±500 | **500** ✅ |
| rates → `maxRate` | 36 / 24 | **360 / 240** ✅ |
| `rate_accel_limit_roll_pitch` | **0** (no setpoint accel limit) | n/a — correctly absent |
| `servo_pwm_rate` 50 Hz | 50 | modelled as servo v2 PWM latch, 50 Hz, 0–20 ms ✅ |

⛔ **`fw_reference_airspeed = 1500` is NOT a rate-loop gain scaler.** INAV uses it only for the
coordinated-turn calculation in TURN_ASSIST (`settings.yaml:1997`), and TURN_ASSIST never appears in this
flight's mode flags. It is at the default and is irrelevant here. ⇒ The airspeed-scaling hypothesis for the
0.74-vs-0.55 authority gap is **dead**.

⇒ **The controller is exonerated.** The gap is airframe aero, actuator, or the measurement — Phase 6.

## 16. ⛔ Found one real divergence: the sim feeds the NN UNFILTERED sensors

| channel | real (what the NN gets) | sim (what the NN gets) |
|---|---|---|
| gyro | `gyro.gyroADCf` — post `gyro_main_lpf_hz = 25` PT1 **+ dynamic notch** (2D, Q 250, ≥30 Hz) | `eom01->getOmegaBody()` — **raw FDM body rates** |
| accel | `acc.accADCf` — post `acc_lpf_hz = 15` **BIQUAD** | `computeSpecificForce(...)` — **raw**, unfiltered |

The controller's *internal* gyro filtering is modelled (`gyroLpfHz = 25` inside `cntrl_inavfwrate`), but
the **NN's own sensor inputs** are not. Un-modelled lag on the policy's input vector:

| filter | group delay | phase @3 Hz | phase @5 Hz | gain @5 Hz |
|---|---:|---:|---:|---:|
| `gyro_main_lpf_hz = 25` (PT1) | **6.4 ms** | 6.8° | 11.3° | 0.981 |
| `acc_lpf_hz = 15` (biquad) | **21.2 ms** | 22.6° | 36.9° | 0.900 |

⭐ **21 ms on the accelerometer channel is 42% of a 20 Hz tick**, and accel is one of the 041 P5-1 inputs
the whole feature depends on. The policy trained on an accel signal that responds instantly and flies one
that lags a fifth of a tick and is 10% attenuated at 5 Hz.

⚠️ **This does not explain the 1.56× accel magnitude — it argues the opposite.** The real channel is
*low-passed* and still reads higher, so the airframe genuinely pulls more g than the model. The filter
finding and the magnitude finding are independent.

## 17. Two config items still unexamined, both cheap

- ⛔ **T050 never ran: are the servos digital?** `servo_pwm_rate = 50` costs 10 ms mean / 20 ms max
  quantisation. The sim models it faithfully, so it is not a *divergence* — but if the servos are digital,
  raising it buys up to **20 ms of the 81.6 ms phase budget for free, on both sides**. This is the single
  cheapest phase lever left and it has never been checked.
- **`gyro_main_lpf_hz = 25` is aggressive** for a fixed wing and is an FR-012a/T048 candidate — matched in
  sim, so again not a divergence, but 6.4 ms of the budget sits there.

## 18. Verdict on "is it the sim, or INAV?"

**Neither, on the controller.** The INAV rate loop and its sim replica agree on every gain, every filter
frequency, every limit, and the rate scaling. ⭐ That is a genuinely good result and it narrows the search
sharply: 043's ACRO model is not where the sim↔real gap lives.

**What is left, ranked:**

1. **Airframe aero / actuator constants** (`hb1_streamer.xml` FDM block) — the 0.74-vs-0.55 authority gap
   and the 1.56× body accel both point here. ⇒ Phase 6, and the operator's `2c691aa` rule applies: change
   only what the data contradicts, and prefer the 2nd article over an n=1 read.
2. **The un-modelled NN sensor filtering** (§16) — 21 ms on accel, 6.4 ms on gyro. Cheap to add on the sim
   side, and it is a *fidelity* fix rather than a tuning change, so it does not need the 2nd article.
3. **`servo_pwm_rate`** — check first (T050), because if the servos are digital it changes the plant on
   both sides before anything is tuned to the old one.

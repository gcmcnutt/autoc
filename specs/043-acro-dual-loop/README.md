# 043 — ACRO dual-loop (seed)

> ⭐ **[`spec.md`](spec.md) now GOVERNS** (written 2026-08-23 from this document). This README is the
> **derivation-of-record** — read it for *why*, and `spec.md` for *what*. Where they conflict, `spec.md`
> wins; its § Clarifications carries the operator decisions taken since.
>
> Three questions this document left open were closed before planning: the "materially smoother" bar is
> **subjective first** (041 AC-1 pattern); the yaw question **dissolved — this airframe has no rudder**
> (verified in the `smix` mixer and the FDM servo model); and 043 ships **one bake with everything**.
>
> ⭐ **And two attributes this document does not yet carry, which govern the crrcsim work** (operator
> 2026-08-23):
> 1. Inputs and outputs are **unchanged in count, magnitude and polarity** — only the outputs'
>    *interpretation* changes. A zero command means **zero commanded rotation RATE**, so the inner loop's
>    I-term **buries the craft-balance variations**; they resurface through the energy channel and at
>    physical limits. A P/PD-only inner-loop model would look correct and mis-model the entire variation
>    regime. See `spec.md` FR-019, FR-059/FR-059a, SC-012/SC-013.
> 2. ⛔ **ACRO is RATE control — and therefore implicitly NOT ANGLE.** `pidLevel`'s self-levelling term
>    runs only under ANGLE/HORIZON/ANGLEHOLD; **ACRO never reads attitude at all**. So zero command means
>    *stop rotating*, there is **no self-levelling and no recovery behaviour beyond the envelope**, and
>    upset recovery stays the policy's job exactly as under MANUAL. `spec.md` § **What ACRO is** carries
>    the definition and everything that follows from it — read that section first.

**Created 2026-08-23** after the 041-t7 flight. Repurposes the 043 slot: the M2 tracking work that used to
live here moved to [044](../044-m2-tracking/README.md), which is waiting on 042's camera parameters anyway.

⛔ **This runs BEFORE any M2 training.** M2 inherits whatever control architecture M1 lands on; training a
tracker on top of a control loop we already know oscillates would bake the problem in one milestone deeper.

---

## Why — the 041-t7 flight

The flight worked, and most of it matched sim closely once measured properly. **Pitch and roll oscillate at
2–5 Hz**, and that is the one thing that did not.

⭐ **What is NOT the problem** (measured, engaged-only, distance-standardized against the same genome in sim):

| | sim | real | verdict |
|---|---:|---:|---|
| throttle, distance-standardized | 0.720 | 0.788 | ✅ 56% of the raw gap was WHERE it flew |
| roll rate, mean \|p\| | 88.4 °/s | 87.3 °/s | ✅ amplitude matches |
| `specific_energy` (NN units) | 0.339 | 0.348 | ✅ 0.11σ |
| `dist_to_boundary` | 0.985 | 0.987 | ✅ 0.04σ |

⛔ **What IS the problem** — the same motion, redistributed upward in frequency:

| roll-rate power | sim | real |
|---|---:|---:|
| 0–1 Hz | 55.9% | 30.6% |
| 3–5 Hz | 12.6% | **30.1%** |
| 5–10 Hz | 2.3% | **7.4%** |

Roll *acceleration* is **1.66×** sim at matching rate amplitude. Confirmed at 60 Hz blackbox (only 2.3% of
energy above 10 Hz, so the 20 Hz finding was not an aliasing artifact).

## ⭐ The phase budget — why this is architecture, not tuning

| source | delay | phase @5 Hz |
|---|---:|---:|
| **ZOH of the 20 Hz loop itself** | **25.0 ms** | 45° |
| **actuator (037 model, incl. 50 Hz servo PWM)** | **30.0 ms** | 54° |
| MSP fetch (mean) | 13.0 ms | 23° |
| eval + send | 7.2 ms | 13° |
| gyro PT1 @ 25 Hz | 6.4 ms | 11° |
| **TOTAL** | **81.6 ms** | **147°** |

180° of lag is oscillation. Predicted crossover **6.1 Hz**; observed band **2–5 Hz**. The mechanism is
identified.

⛔ **The gyro filter is 8% of the budget.** The two dominant terms — the 20 Hz ZOH and the actuator — are
**67%**. Every filter/baud/loop-rate fix *combined* buys 32 ms and moves crossover to 10 Hz, still inside
the band. ⭐ **You cannot damp a 2–5 Hz limit cycle from a 20 Hz loop. Muting the oscillation and dulling
the controller are the same knob at this bandwidth.**

## The approach — command RATES, not surfaces

INAV already runs a rate controller at `looptime:500` = **2 kHz**, 100× the xiao loop, and it is
well-tuned: **ACRO flew every non-autoc segment of the 041-t7 flight and the operator judges it rock
solid.** The dual loop is not new machinery to build; it is machinery already flying that autoc bypasses.

The change reframes the NN's job from *"what surface deflection right now"* — a stabilization task it is
structurally unequipped for at 20 Hz — to *"what rate do I want"*, which 20 Hz handles fine. The ZOH and
actuator lag move INSIDE a loop with 100× the bandwidth to fight them.

Config of record: [`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg), `platform_type = AIRPLANE`, so the live
gains are `fw_p_pitch 15 / i 5 / d 5` and `fw_p_roll 15 / i 3 / d 7`. ⭐ **Use it as-is to start** — it is
proven on this airframe. Tuning ACRO, INAV, or anything else is in scope later, but the baseline must be
the configuration that already works.

## Order of work (operator 2026-08-23)

1. **PRE-WORK — xiao log self-sufficiency.** Before anything else: the log must carry enough for the
   renderer and post-flight analysis without a blackbox join.
   * ⛔ `FlightStateRecord` (armed-but-not-engaged breadcrumb) currently carries **only** `pos/vel/quat` —
     no controls, no gyro, no accel. So non-autoc segments cannot be compared against engaged ones, which
     is exactly the comparison ACRO work needs. The renderer is not failing to display them; they were
     never recorded.
   * ⚠️ Budget it against `kBudgetTicks` / the 2 MB QSPI region — the existing `static_assert` caps tick
     storage at 70% of the region, and adding fields tightens it.
   * ⭐ This session's analysis needed a blackbox clock-join for *everything*. The join worked (−970 ppm
     fit, cross-validated to 0.5% against `ARM|MANUAL|MSPRCOVERRIDE`), but it should not be required.
2. **Model ACRO in sim.** The FDM must respond to rate commands through a model of INAV's fw PID, not to
   surface deflections. ⚠️ Also fold in the sensor filters (gyro PT1 25 Hz, accel BIQUAD 15 Hz —
   both PROVEN active from the flight's own blackbox header, `acc_notch_hz:0` so no accel notch).
3. **Bench: how fast can we actually collect INAV data?** `blackbox_rate_denom = 32` gives 60 Hz today.
   Operator: *"maybe with acro we don't need to"* go faster — the point of measuring is to find out.
4. **Train**, then **fly**.

## Levers available, with what each is worth

| lever | delay removed | note |
|---|---:|---|
| ⭐ **ACRO dual loop** | reframes 55 ms | the only order-of-magnitude move |
| `servo_pwm_rate` 50 → 200+ Hz | ~7 ms | ⚠️ **found 2026-08-23** — a 50 Hz servo update is 20 ms period; check the servos are digital first |
| `acc_lpf` 15 → 60 Hz | 11 ms | ⚠️ check `accVib` before loosening; 15 Hz smells like inherited quad tuning |
| MSP 115200 → 460800 | ~5 ms | 039 T021, closed-not-run — untested, not disproven |
| `gyro_lpf` 25 → 100 Hz | 4.8 ms | measured roll-off is steeper than PT1 alone (dynamic notch at 30 Hz, Q 250) |
| xiao loop 20 → 35 Hz | 10.7 ms | MSP-capped, not CPU-capped (eval is 1.6 ms) |

## ⚠️ Open, and it matters — pin the actuator model first

The **30 ms actuator figure is from the 037 model, not measured on this airframe**, and it is the largest
single term in the budget. If it is wrong the crossover estimate moves. `servo_pwm_rate = 50` says at
least 10 ms of it is real, but the rest is modelled, not observed.

⭐ **Servo response is ALREADY a variation class** (`craftServoSlew`, `craftThrustTau` in
`ScenarioMetadata`; the first-order servo tau draw was removed 2026-06-12 when v2 went to PWM-latch+slew).
So the sim can already express a spread of actuator responses — what is missing is knowing where the real
airframe sits inside that spread.

⚠️ **Operator 2026-08-23: the 2nd flight article will make this apparent** — two airframes give the first
real read on servo-response spread rather than a single sample. Until then the variation range is an
assumption.

⭐ **And some earlier figures probably have not changed** — the 037-era actuator and latency constants were
measured on this same airframe and much of it is unmodified. This is a **fine-tune of the sim model**, not
a re-derivation. ⛔ Do not treat every constant as suspect; find the ones the flight data actually
contradicts and leave the rest alone.

**Pre-work, then, is threefold** (all before modelling ACRO):
1. xiao log self-sufficiency (§ Order of work item 1),
2. a bench servo step-response to pin the actuator term,
3. a pass over the 037 constants asking *which of these does the 041-t7 flight actually contradict?*

---

## ⭐ THIS FEATURE WAS ALREADY PREDICTED — 039 wrap, 2026-07-13

⛔ **Read the backlog entry *"Pitch marginal-stability levers — choose after the next articles fly"*
before designing anything here.** It was written after the FIRST 20 Hz flight, gated on `n>1` flight
articles, and it names this feature as one of its four levers:

> **Architectural**: NN over INAV's rate loop (acro+override instead of manual+override) — the untuned PID
> already damps this plant to 24 °/s. Changes the action space (setpoints, not surfaces); sim must match;
> **a feature of its own, NOT a tweak.**

⭐ **And it already measured the case**, pitch-rate RMS by regime on the 2026-07-13 flight:

| regime | pitch-rate RMS |
|---|---:|
| INAV **acro**, untuned PID | **24 °/s** — *"on a rail"* |
| pilot MANUAL | 50 °/s — *"on edge"* |
| **NN-direct** (`MANUAL\|MSPRCOVERRIDE`, raw surfaces) | **141 °/s** |

⭐ **The inner loop is 5.9× better than what autoc does today, measured, on this airframe.** That entry
also concluded the oscillation is *"closed-loop plant sensitivity, not command roughness"* — the NN's
pitch commands were the CALM axis — which is the same conclusion 041's phase budget reached independently
two flights later.

⚠️ **The gate has now fired**: `n>1` articles exist and the 041-t7 flight is the second data point. ⛔ The
2026-07-13 decision *"NO sim recalibration from this n=1 airframe"* was correct then and is now expired —
but see the actuator note below: fine-tune what the data contradicts, not everything.

⭐ **The other three levers stay on the table and are cheaper**: forward CG ballast (hardware), sim
static-margin match (with a stated acceptance test — reproduce ~141 °/s pitch RMS vs ~95 roll replaying
the recorded 20 Hz command stream), and a static-margin craft-variation axis. ⚠️ **Consider whether one of
those is enough before committing to the architecture change.**

---

## Pre-work, as scoped by the operator 2026-08-23

1. **xiao log fidelity** — see § Order of work item 1.
   ⭐ **Sweep in**: backlog *"[NEXT] Export RC Commands to Xiao Log"* (`xiao/src/msplink.cpp`) — *"log RC
   commands throughout entire flight for full playback visualization"*. That entry IS this pre-work item,
   filed earlier and unactioned. `FlightStateRecord` carrying only pos/vel/quat is the same gap.
2. **Analysis of INAV acro mode** — what the 2 kHz rate loop actually does to a setpoint, and what the
   `fw_*` gains mean for the plant. Config of record is `xiao/inav-hb1.cfg`, `platform_type = AIRPLANE`.
3. **Change `autoc=y` to force ACRO, and unforce it** — today it forces MANUAL.
   ⚠️ **Sweep in**: backlog *"[NEXT / 023 follow-up] INAV Fork Patch: mspOverrideInit First-Frame Bug
   (C1)"* — MSPRCOVERRIDE engage pays a **200 ms floor** even at `failsafe_recovery_delay = 0`, because
   pre-connection ticks keep bumping `validRxDataFailedAt`. We are touching the override path anyway, and
   200 ms of engage latency matters more in a dual-loop design than it did before.
4. **IMU variations** — accel, rate core axis, a quat transform, and light scaling variation on all axes.
   ⭐ **Sweep in**: backlog *"5th PRNG class slot: mount-alignment 6-DOF first"*. It was scoped for the
   CAMERA, but the machinery is identical and the operator's framing is explicit: *"similar to the m2
   camera work"*. Doing IMU first makes the camera case a second consumer of a proven slot rather than
   the pathfinder.
   ⭐ **These fold into the existing 294 scenarios** — operator: *"the training regiment doesn't really
   need bigger population."* They are static-per-scenario like craft variations, not a new axis count.
   ⚠️ **Motivation is measured**: the 041-t7 sensor audit found `accel_y` **1.70σ** off sim at matched
   distance — sim sits at exactly 0.000 (perfectly symmetric, no mount error) while real carries a
   constant lateral bias. The real IMU is foam-taped to the underside of the upper surface; the operator
   estimates **±5° in all directions**, and similar for GPS.
5. **crrcsim implementation of the intended ACRO filtering**, constants in the existing XML.
   ⛔ **Sweep in as a WARNING, not a task**: backlog *"[ABANDONED] INAV pt3 RC Smoothing Filter in CRRCSim
   (023 Phase 9a)"*. That attempt replicated INAV's pt3 smoother at 333 Hz in `inputdev_autoc.cpp`, and
   **test6 at a 40 Hz cutoff stunted training** — best stuck at −2225 through 55 gens vs −4410 baseline,
   `pctInStreak` 3% vs 12%. Conclusion recorded: *"the filter changes dynamics enough that the GA can't
   find productive policies."*
   ⚠️ **That is not a reason to skip this**, because ACRO is a rate CONTROLLER, not a command smoother —
   it should make the plant easier, where the pt3 filter made it harder. But it is direct evidence that
   adding filtering to crrcsim can silently destroy trainability, so the acceptance test must include
   *"does a known-good genome still train"*, not only *"does it fly."*

## Other backlog items worth sweeping while these components are open

| item | why now |
|---|---|
| ⭐ *Formal input normalization — measured statistics, not hand-derived constants* | HIGH VALUE / LOW COST, and IMU variations touch the input path anyway. Would also close 044's Gate 2 by construction. |
| *`nnextractor -g` takes the FILE number; `dmp-dump --gen` takes the GENERATION* | a live footgun — cost real time during 041 analysis |
| *crrcsim `mod_inputdev` — link `autoc_common` instead of cherry-picking sources* | we are opening `inputdev_autoc.cpp` for item 5; the AWSSDK link fix (`17e6c3e`) already moved the parent in this direction |
| *Type-Safe NN Sensor Interface* | item 4 edits the sensor interface; six stale-label bugs in 041 were all "a caller was not updated" |
| *Simulator Sampling Time Variation* | the 20 Hz ZOH is 25 ms of the 81.6 ms budget — varying it is directly on-topic |
| ⚠️ *460800 baud-raise (039 T021, closed-not-run)* | ⛔ **NOT on 043's path** — deferred until there is an ACRO signal, on the cleaner-wired 2nd article. See § Transport. |
| ⚠️ *Renderer playback enhancements / streak overlay / X-display portability* | only if the renderer is genuinely opened; otherwise leave — 043 is a control-loop feature |

## ⚠️ Transport: DEFERRED, and deliberately so — model the latency, do not chase it

Operator 2026-08-23: *"baud raise should be gated on checksums on the transport of course."* Correct, and
the situation is worse than "unmonitored".

The CRC **is** verified — MSP v2 `crc8_dvb_s2` in `xiao/src/MSP.cpp`. But the failure path is:

```cpp
uint8_t checksum = _stream->read();
if (checksumCalc == checksum) {
    return true;
}
}          // <-- falls through to the outer while(1) and silently re-reads
}
```

⛔ **A corrupt frame does not return false, does not increment a counter, and is not logged.** It loops and
keeps reading until `_timeout`. So a transport error surfaces ONLY as a longer `fetch` time —
indistinguishable from INAV's scheduler being slow. `fetch` max is already 27–34 ms against a 50 ms tick,
so there is room to hide a lot of retries.

⚠️ **Therefore raising the baud rate today could double the error rate and the only symptom would be
fetch-time creep**, which reads as jitter. That is the same failure shape 041 hit six times: a real error
rendered as something benign.

⭐ **BUT THE BAUD RAISE IS NOT ON 043'S PATH.** Operator 2026-08-23: *"seems we can live with the latency
for now, we simulate it — and once we get some signal on acro perf we can perhaps speed up the transport."*

That is the right call and it follows from the feature's own premise:

* ⭐ **Latency that is faithfully modelled is a property, not a defect.** The 13 ms MSP fetch is part of
  the 81.6 ms budget 043 exists to model. If the sim carries it, the NN trains against it and the
  aircraft flies what it trained on. Removing 5 ms while the sim still ignores it would *widen* the
  sim-real gap, not close it.
* ⭐ **The physical layer is about to change anyway.** The 2nd flight article is being built and wired
  more cleanly, so today's harness is not the one that matters. Measuring error rate on hardware being
  replaced spends effort on a moving target.
* ⭐ **ACRO changes the requirement.** If the inner loop damps the plant to ~24 °/s as 039 measured, the
  outer loop's latency sensitivity drops sharply — the fast stabilization no longer depends on the link.
  **The right time to ask "is the transport fast enough" is AFTER there is an ACRO signal**, because the
  answer may be "comfortably, already".

**So: measured evidence says nothing is broken at 115200 today.** The 2026-08-23 flight logged
`state_valid` on every tick — **1,682 of 1,682 valid, zero fetch failures across four spans**.

⚠️ ⛔ **Two limits on that reassurance, recorded so it is not over-read:**
1. `state_valid` is `false` only on FULL TIMEOUT. A frame that was corrupted and silently re-read still
   logs `1`. So it proves **zero total losses**, not zero errors — retries stay hidden inside `fetch`.
2. It only covers **INAV → xiao**. The `xiao → INAV` direction carries `MSP_SET_RAW_RC` — the CONTROL
   OUTPUTS — and is uninstrumented on both ends. INAV drops a bad frame to `MSP_IDLE` with no counter and
   **holds the previous RC values**, so a lost command flies stale surfaces with nothing recording it.
   ⭐ `rc_sent[3]` is already logged, so a **sequence number the xiao knows it sent** would expose gaps
   from the log alone, without a fork change.

**If and when the transport is revisited** (post-ACRO-signal, on the new article): instrument first, keep
the counters in permanently as a standing harness-health signal, baseline at 115200, then step
**230400 before 460800** — a 2× step with counters running gives the error-rate *slope*, where a single
4× jump gives one point and no gradient.

⭐ This also retro-fits the 039 T021 conclusion. It was closed **not-run** because *"115200 proved
sufficient in flight (zero overruns)"* — but zero *overruns* is not zero *errors*, and with retries hidden
inside `fetch` the two are not the same measurement. The lever is still unexercised, and now so is its
safety check.

---

## ⭐ MODELLING THE CONFIG *RATES* IS THE KEY PART

Operator 2026-08-23: *"modelling the config **rates** is key."*

⛔ **The gains are the easy half; the rates are what make the model right.** A crrcsim ACRO model that
implements `fw_p_roll = 15` and stops there will be wrong, because the loop's behaviour is set as much by
*when* things happen as by *how hard*. The rates that must be modelled, all readable from
[`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg) and the flight's own blackbox header:

| rate | value on the flying config | why it matters |
|---|---|---|
| `looptime` | **500 µs = 2 kHz** | the inner loop's actual cadence — the whole reason it can damp what a 20 Hz loop cannot |
| `servo_pwm_rate` | **50 Hz** | ⚠️ 20 ms period ⇒ ~10 ms ZOH. Real, measured, and part of the 30 ms actuator term |
| `motor_pwm_rate` | 16 kHz | not in the control path but pins the ESC model |
| `gyro_lpf_hz` | **25** (PT1) | 6.4 ms group delay INSIDE the inner loop |
| `acc_lpf_hz` | **15** (BIQUAD) | 15 ms — the largest sensor delay, on the channel 041 added |
| `dynamicGyroNotchMinHz` / Q | **30 / 250** | measured roll-off is steeper than PT1 alone; the notch is doing real work above 10 Hz |
| `dterm_lpf_hz` / type | 10 / biquad | D-term filtering shapes the inner loop's own phase margin |
| MSP `TASK_SERIAL` | **100 Hz, LOW priority** | the fetch-time tail comes from here, not from payload size |
| xiao control loop | **20 Hz** | 25 ms ZOH — the outer loop's own contribution |

⭐ **These are cascaded rates, not one rate.** 2 kHz PID inside a 50 Hz servo inside a 20 Hz outer loop,
with sensor filters at 15–25 Hz feeding the innermost one. Getting the *ratios* right matters more than
getting any single constant exactly right — a model with all the right gains and one wrong rate will
oscillate somewhere the real aircraft does not.

⚠️ **`servo_pwm_rate = 50` deserves its own look.** It is the one rate that is plainly low for a control
surface, it is ~10 ms of the largest term in the phase budget, and raising it is a config change. ⛔ Check
the servos are digital first.

Constants go in the existing XML per the operator's scoping, so the ones we might change stay changeable
without a rebuild.

## 🔬 RESEARCH PHASE — are the two loops at arm's length?

Operator 2026-08-23: *"we have a plan research phase to note if we need other feedback into the RNN
regarding this — or are the two loops at arm's length? e.g. we feedback our command outputs and we see
actual filtered rates and accel coming back — anything else needed?"*

⭐ **This is the central design question of the feature and it should be answered BEFORE the sim work, not
after.** It decides the NN's input and output contracts, and everything downstream is built on them.

**What the NN would have today, unchanged:**
* it **commands** rates (the new action space),
* it **observes** the resulting filtered `GYRO_P/Q/R` and `ACCEL_*` coming back,
* plus everything else in the existing 45.

So there is already a closed observation loop: command out, consequence in. **The question is whether that
is sufficient**, and it is genuinely open. Candidates to evaluate, none pre-judged:

1. ⭐ **Its own previous command** (efference copy). The NN sees the *result* of its last setpoint but not
   the setpoint itself. With inner-loop lag, result-at-time-t reflects a command from 2–3 ticks ago, and
   the network currently has to infer that from recurrent state. ⚠️ Note the RNN *may already* encode
   this — `W_hh` effective rank is 11.2–11.4 of 16, so there is unused capacity, but "unused" is not
   "unable".
2. **Rate-tracking error** — commanded rate minus achieved rate. If the inner loop is saturating or
   rate-limited, this is the only channel that would reveal it, and it is exactly what a human pilot
   feels as "the aircraft isn't following."
3. **Inner-loop health** — I-term accumulation or output saturation from INAV. ⛔ Would need a new MSP
   field; do not add speculatively.
4. **Nothing.** ⭐ **The genuine possibility, and the cheapest outcome.** If the inner loop tracks
   setpoints well, it is a near-ideal actuator and the outer loop needs no visibility into it at all —
   that is the definition of "at arm's length", and it is what the 039 measurement (24 °/s, *"on a rail"*)
   hints at.

⛔ **Answer it with the sim once ACRO is modelled, not by adding inputs first.** 041's lesson was that
inputs added on reasoning rather than measurement got de-weighted — the eight added there are *still*
unproven (T068). The discipline that worked was: measure the spread, find what is unreachable, fix that.
⭐ Cheapest experiment: model ACRO, train with the CURRENT 45 inputs, and only then ablate/add — a
rate-tracking-error input either earns its keep against that baseline or it does not.

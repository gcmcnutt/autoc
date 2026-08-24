# 043 — ACRO dual-loop (seed)

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
| ⛔ *460800 baud-raise (039 T021, closed-not-run)* | worth ~5 ms — but **GATED on transport error visibility first**, see below |
| ⚠️ *Renderer playback enhancements / streak overlay / X-display portability* | only if the renderer is genuinely opened; otherwise leave — 043 is a control-loop feature |

## ⛔ GATE ON THE BAUD RAISE — transport errors are invisible today

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

**Gate, in order:**
1. Count CRC failures, framing errors and timeouts; expose them in `SpanSummary` beside the existing
   `PipelineStats` (they are already logged per span, so the carrier exists).
2. Establish the error rate **at 115200** as the baseline — it may not be zero now.
3. Only then raise to 460800, and compare error rate as well as latency.

⭐ This also retro-fits the 039 T021 conclusion. It was closed **not-run** because *"115200 proved
sufficient in flight (zero overruns)"* — but zero *overruns* is not zero *errors*, and with retries hidden
inside `fetch` the two are not the same measurement. The lever is still unexercised, and now so is its
safety check.

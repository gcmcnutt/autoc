# 043-t2 — second ACRO flight (2026-09-06): what the two fixes actually bought

Same genome (gen 800, `weight_id=3af8e3ab787b75a5`). Two changes since 2026-09-05:

1. **xiao** — the engage-prefill frame fix (`07ad93b`)
2. **INAV** — `rc_expo 20 → 0`, `setpoint_kalman_enabled OFF` (`3da6e22`)

Logs: `flight-results/flight-20260906/`. **3 spans, 1279 ticks, 0 gaps, 0 overruns, 0 drops.**
⭐ Spans 341 / 439 / 499 ticks — no 3-second pilot-rescue like 09-05's span 1.

⚠️ Two changes landed together, so attribution is only safe where the mechanism is axis- or
channel-specific. Where it is not, this says so.

---

## 1. Both fixes verified in the air

| | 2026-09-05 | 2026-09-06 |
|---|---|---|
| `dist[0..4]` at tick 0 | **162.2 / 208.0 / 183.2 / 189.5 m** (= ‖origin‖) | ✅ **0.00 m**, all three spans |
| `rcData→rcCommand` fit | **expo 20** (resid 4.4 / 2.8) | ✅ **expo 0** (resid 4.11 / 2.90, vs 25.94 / 25.45 at expo20) |

The engage transient is gone and the action space is linear. ⇒ **For the first time the policy was asked a
question it was trained on.**

## 2. ⭐ What improved — control quality, clearly

| measure | 09-05 | 09-06 | sim gen800 |
|---|---:|---:|---:|
| `out_pitch` railed (\|x\|>0.95) | **13.0%** | **3.8%** | 2% |
| pitch `\|axisF\|` at ±500 limit | 10.8% | **3.9%** | — |
| throttle rail-switching | 3.96 Hz | **2.56 Hz** | 1.89 Hz |
| throttle Δ/tick | 0.364 | **0.263** | — |
| ⭐ **roll 5–10 Hz power** | 8.2% | **4.4%** | 2.3% |
| pitch rate-loop delivered fraction | 0.51 | **0.70** | 0.43 |
| vertical-estimator resets | 3 | **1** | — |

⭐ **The OOD nose-down rail is largely gone** — pitch railing fell 3.4× and the pitch PID stopped pinning
the ±500 budget. That is mechanism-specific to the prefill fix: the policy is no longer driven by 800 ms
of 6–8× out-of-scale input into a fresh recurrent state.

⭐ **Roll 5–10 Hz power halved (8.2% → 4.4%)** — now *better* than the 041-t7 baseline's 7.4% and
approaching the sim's 2.3%. The high-frequency roll content 043 set out to remove is genuinely reducing.

⭐ **Throttle chatter dropped a third** and is now within 1.4× of sim, from 2.1×.

## 3. ⛔ What did NOT improve — tracking

| | 09-05 | 09-06 | sim gen800 |
|---|---:|---:|---:|
| median distance | 11.1 m | 11.5 m | **4.1 m** |
| mean distance | 16.2 m | 13.1 m | 6.3 m |
| ticks < 5 m | 16.2% | **12.0%** | **60.7%** |
| **in-streak** | 9.8% | **7.4%** | **54.6%** |

⚠️ 09-05's in-streak is *reconstructed* from a v4 log; 09-06's is the firmware's own `step_score` (v5), so
09-06 is the more trustworthy number and it is the lower one.

⛔ **This is the finding that matters.** With the input bug removed and the action space linear, tracking
is **unchanged** — still ~7× short of sim on occupancy. ⇒ **The tracking gap was never the prefill bug.**
09-05 could not attribute it; 09-06 can: it is a genuine policy / sim-fidelity gap.

Also unmoved or worse:

| | 09-05 | 09-06 | note |
|---|---:|---:|---|
| **roll 3–5 Hz power** | 17.7% | **17.4%** | ⛔ SC-001's headline band did not move |
| roll rate-loop delivered fraction | 0.57 | 0.58 | unchanged despite expo removal |
| body accel median | 2.33 g | **3.01 g** | sim 1.49 g — gap widened to 2.0× |
| pitch mean \|q\| | 89.5 °/s | **106.5** | pitch got busier |
| pitch 1–3 Hz power | 67.2% | **70.3%** | ⇐ the "wiggle" |
| `accVib` median | 2714 | **4452** | estimator still unhealthy (max gap −57.8 m) |

## 4. ⭐ The axis asymmetry is the actionable result

Removing expo raised **pitch** delivery 0.51 → 0.70 but left **roll** at 0.57 → 0.58.

⛔ **It is not pidSum clipping.** Roll `|axisF|` median is only 176–190 against the ±500 budget and
saturates on just 1.7–3.1% of samples — roll operates far from the limit. (Pitch sits at 365 and *was*
saturating 10.8% of the time before the fix.)

⇒ **Roll's delivered fraction is plant-limited, not command-limited.** Pitch was being held back by the
command path (expo + OOD railing) and freeing it worked; roll is held back by the airframe/actuator, and
no config change will move it. ⭐ That is precisely what **T046** and the aero constants address, and it
is now measured rather than assumed.

⚠️ Note the sim errs in **opposite directions per axis**: real roll delivers **0.58** vs sim **0.74**
(sim too strong), real pitch delivers **0.70** vs sim **0.43** (sim too weak). A single scalar correction
cannot fix both — the per-axis aero terms have to move.

## 5. ⛔ Retraction: the "gyro Kalman saved 10 ms" claim does not hold

`bench-notes.md` §"bench verification #3" recorded a 10.6 ms pipeline improvement attributed to
`setpoint_kalman_enabled = OFF`. **The flight does not support it.**

| run | Kalman | MSP fetch avg |
|---|---|---:|
| bench 09-04 | ON | 13.0 ms |
| flight 09-05 | ON | 13.3 ms |
| **bench 09-05** | **OFF** | **2.9 ms** |
| **flight 09-06** | **OFF** | **13.0 ms** |

Same config on either side of the fast run ⇒ **the 2.9 ms bench run is an outlier, not a treatment
effect.** GPS state was equivalent (fix 2, 7–8 sats) in both bench and flight, so that is not the
explanation either; the likeliest cause is a favourable phase between the xiao's 20 Hz request and the
FC's serial task scheduling on that particular run.

⇒ **The xiao pipeline remains ~20 ms**, and `COMPUTE_LATENCY_MSEC_DEFAULT = 30` is overstated by ~10 ms,
not ~20. The direction of the sim-fidelity correction is unchanged; the magnitude is halved.

## 6. Verdict and routing

**043's thesis holds, partially and reproducibly.** Against 041-t7's 30.1% roll 3–5 Hz, ACRO gives
**17.7% / 17.4%** on two independent flights — a real, repeatable improvement, now n=2. And 5–10 Hz is
down to 4.4%, better than baseline. ⛔ But the 3–5 Hz band has stopped moving, and it is still 1.4× the
sim's 12.6%.

**Tracking is the open problem and it is now cleanly attributable.** Not the prefill bug, not expo, not
the ACRO model (§15 of `flight-analysis.md` showed the controller is a parameter-exact replica).

**Next, in order:**

1. ⭐ **T046** — the DSM-44 loaded step-response. §4 has now *measured* that roll is plant-limited; this is
   the instrument for it, and it is weather-free.
2. **Per-axis aero correction** — the sim is too strong in roll and too weak in pitch. One scalar will not
   do; `hb1_streamer.xml` per-axis terms have to move, informed by (1).
3. **Sim latency** — `COMPUTE_LATENCY_MSEC_DEFAULT` 30 → ~20 (corrected per §5), plus the un-modelled NN
   sensor filtering (accel 15 Hz ≈ 21 ms, gyro 25 Hz ≈ 6.4 ms).
4. **Then bake.** ⛔ Not before — every constant above is one the genome would train against.

⚠️ The vertical estimator (`accVib` 4452, gap max −57.8 m, one reset) is unaddressed and feeds
`specific_energy` and `dist_to_boundary` directly. It is its own problem and it did not improve.

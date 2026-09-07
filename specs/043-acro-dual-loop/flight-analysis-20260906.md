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

---

## 7. ⛔ The span-3 discontinuity — the SAME vertical-estimator failure, second flight running

Operator spotted it in the chase trace. It is the Z channel again, and it is INAV's vertical estimator
resetting, not GPS.

**Span 3, tick 489 of 499 (t = 24.45 s):**

| | before | after | |
|---|---:|---:|---|
| `pos_d` | +10.12 m | **−22.44 m** | **32.6 m step** |
| horizontal step | | 0.77 m | ⇐ normal; the fault is Z-only |
| `vel_d` | −3.62 | −9.40 | |
| ⭐ `specific_energy` | 25.5 m | **62.4 m** | ⭐ **+36.9 m = +0.254 in NN units** |
| `airspeed` | 10.51 m/s | 13.99 | (derived from \|v\|, so it inherits the bad `vel_d`) |

⛔ **That Es step is 56% of the entire range that channel covers across the whole flight**
(+0.089 … +0.545), injected in a single tick, into an input the policy uses for energy and throttle.

The INAV trace shows the same build-up as 2026-09-05: `navPos[2]` drifting away from `BaroAlt` (58 m vs
114 m — reading **56 m LOW** this time), `navEPV` climbing monotonically 842 → **999** (the ceiling), then
at the step `navPos[2]` snaps 66.1 → 98.4 m and **`navEPV` resets 999 → 471**. Classic reject-and-
reinitialise.

⚠️ Note the sign flipped between flights — 09-05 drifted **high** and corrected down, 09-06 drifted **low**
and corrected up. Not a fixed bias; consistent with accelerometer-driven vertical-velocity drift under
vibration.

### It is not just the resets — the altitude is wrong for a fifth of every flight

| \|navPos[2] − BaroAlt\| | 09-05 | 09-06 | in Es NN units |
|---|---:|---:|---:|
| median | 1.8 m | 3.1 m | — |
| p90 | 14.4 m | 8.5 m | — |
| max | 85.8 m | 57.8 m | — |
| **> 5 m** | **28.8%** of flight | **21.8%** | 0.034 |
| **> 10 m** | **15.0%** | **8.6%** | 0.069 |
| **> 20 m** | **5.9%** | **2.7%** | 0.138 |

⇒ The discrete resets are rare and land near the end of a flight, so they corrupt a handful of ticks. The
**continuous drift is the bigger problem**: for roughly a fifth of every flight the policy's altitude —
and therefore `specific_energy`, `dist_to_boundary`, and the engage-arena floor/ceiling geometry — is off
by more than 5 m, and for ~1 tick in 10 by more than 10 m.

⭐ **`accVib` is the common factor**: median **2714** (09-05) → **4452** (09-06), peaks over 6000, and the
vertical fusion leans on the accelerometer. Higher vibration this flight, worse drift per unit time.

⛔ **This is not a 043 problem and no 043 lever touches it.** It predates the ACRO work, it appears on both
flights, and it corrupts NN inputs directly. It needs its own item:

1. **Chase the vibration first** — `accVib` 4452 median is mechanical (prop balance, motor mount, FC
   soft-mount). Everything downstream is fusion trying to cope with a bad accelerometer.
2. `ins_gravity_cmss = 972.092` is still ~0.7% below true g — a standing bias on the same signal.
3. Only then consider leaning the vertical fusion harder on the baro.

⚠️ **It does not explain the flat tracking** in §3 — the resets are too few and too late in each span. But
it is a real, measured corruption of the policy's energy channel, and it will keep degrading any
sim↔real energy comparison until it is fixed.

---

## 8. Baro and GPS — the operator's questions, answered and one left open

### The baro IS configured, and the fusion is entirely stock

`baro_hardware = SPL06` — a real barometer, present and selected. Every vertical-fusion weight sits at the
**INAV default**, so nothing here has been tuned for this airframe:

| setting | value | INAV default |
|---|---:|---:|
| `inav_w_z_baro_p` | 0.350 | **0.35** |
| `inav_w_z_baro_v` | 0.100 | **0.1** |
| `inav_w_z_gps_p` | 0.200 | **0.2** |
| `inav_w_acc_bias` | 0.010 | **0.01** |
| `inav_baro_epv` | 100 | **100** |

⇒ The estimator is running INAV's out-of-the-box vertical fusion on an airframe whose accelerometer sees
`accVib` 4452 and whose baro has never been characterised. That is the whole problem in one line.

### ⛔ The physics, for scale

Dynamic pressure as apparent altitude (1 hPa ≈ 8.3 m):

| airspeed | 10 | 13 | **15** | 18 | 20 | 25 m/s |
|---|---:|---:|---:|---:|---:|---:|
| apparent altitude error at FULL ram | 5.1 | 8.6 | **11.4** | 16.5 | 20.3 | 31.8 m |

⭐ A flying wing at cruise carries **~11 m of altitude error for every unit of dynamic pressure the static
port sees**. The operator's "not tested in all attitude for accuracy" is exactly the right worry.

### What the data shows — and it differs between the two flights

`BaroAlt − GPS_altitude` in **level flight only** (`|vz| < 1 m/s`, which removes any baro/GPS lag
confound — and note that isolating level flight made 09-05 *stronger*, not weaker):

| speed bin | 0–6 | 6–10 | 10–14 | 14–18 | 18–30 m/s | corr |
|---|---:|---:|---:|---:|---:|---:|
| **09-05** | −0.7 | −2.8 | −7.9 | −9.4 | **−13.5 m** | **−0.685** |
| **09-06** | −0.1 | +3.5 | +3.8 | +2.0 | +2.3 m | −0.086 |

⭐ 09-05 is a textbook ram-pressure signature: monotonic, ~13 m of swing, and the sign (baro reads low as
speed rises) says the port sees **positive** pressure. At ~20 m/s full ram would be 20.3 m, so the port
would be seeing roughly **two-thirds of dynamic pressure** — i.e. barely a static port at all.

⛔ **But it does not reproduce on 09-06**, same aircraft, and I cannot resolve why from the logs:

- **Wind is ruled out** — fitting groundspeed against course gives 3.1 m/s (09-05) vs 2.1 m/s (09-06),
  similar, and both flights show the same mean airspeed ~13.3 m/s.
- **Vertical-rate lag is ruled out** — restricting to level flight *strengthened* the 09-05 correlation.
- ⚠️ **Speed and altitude are collinear** — `corr(speed, baroAlt)` is **+0.68 / +0.75** in these flights
  (the aircraft flies faster when higher), and on 09-05 the divergence correlates with altitude
  (−0.507) almost as well as with speed (−0.683). **The two cannot be separated from flight data.**

⇒ Something real differs between the flights and the logs cannot say what. **This needs a ground test, not
more flying.**

### ⭐ Two ground tests that settle it, ~20 minutes, no flying

Both answer the operator's question directly and neither needs weather:

1. **Ram test.** FC powered, airframe stationary, blow air over it at ~15 m/s from the front (leaf blower
   or a car window) and watch `BaroAlt` on the Configurator. A good static port moves **< 1 m**. If it
   swings 5–10 m, the port is exposed and 09-05's signature is real.
2. **Attitude test.** FC powered, stationary, no airflow. Rotate the airframe through pitch and roll
   (±30°, ±60°, inverted) and watch `BaroAlt`. It must not move. Movement means the port is
   pressure-coupled to orientation — which on a flying wing usually means it is venting into the fuselage
   rather than to a static source.

⇒ If either fails, the fix is mechanical (relocate/shield the port, or foam-damp the FC bay), not a
fusion-weight change. ⛔ Do not tune `inav_w_z_baro_p` first: leaning harder on a baro that is reading
airspeed would trade one error for another.

### GPS — 09-06 was materially worse, and 7 is the ceiling not the average

| | sats min / med / max | 3D fix | hdop med | epv max |
|---|---|---:|---:|---:|
| **09-05** | 7 / **10** / 11 | 100% | 1.78 | 448 |
| **09-06** | 5 / **7** / **7** | 100% | 2.25 | 551 |

⚠️ **Note the max: 09-06 never exceeded 7 satellites for the whole flight**, and dipped to 5. That is not
normal sky variation — a healthy receiver under open sky wanders. A flat ceiling at 7 points at antenna
placement, shielding, or interference rather than the sky.

ⓘ No loss of 3D fix in either flight (100% both), so this did not cause the estimator resets directly —
GPS vertical weight is only 0.2 against baro's 0.35. But it degrades the horizontal solution the whole
tracking metric rides on, and it is worth a look before the next flight: check the GPS antenna's ground
plane and its separation from the video TX / ESC.

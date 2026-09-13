# 043-t3 — third ACRO flight (2026-09-13): the pitch oscillation is NOT the rate loop

Genome **gen 9200** (`weight_id=a5d097da18bf33ed`, `firmware_id=1e8342d6a38d55f6`), program
`autoc-m1/autoc-9223370248021823368-2026-09-08T02:02:32.439Z/gen9200.dmp.zst` — the t3 bake
([`artifacts-t3/MANIFEST.md`](artifacts-t3/MANIFEST.md)).

Logs: `flight-results/flight-20260913/`. **3 spans, 618 ticks, 0 gaps, 0 overruns, 0 resyncs, 0 drops.**
Stiff wind — operator called 290–300°; INAV's own estimator says **from 307° at 4.2 m/s median, 6.7 max**.

⭐ **Headline: the 2.0–2.6 Hz pitch oscillation is unchanged from the 041-t7 baseline in frequency,
band share and amplitude — and t7 flew with INAV's rate loop BYPASSED. The oscillation is therefore not
produced by the rate loop, and the 043 phase-budget thesis does not explain it.**

Two independent problems sit alongside it: **prop imbalance** (`accVib` is a direct function of motor
output, r = 0.75) feeding the vertical-estimator resets, and **wind speed is not a variation class**,
which shows up as a one-sided downwind tracking bias.

---

## 0. Span numbering — settle this once

The operator's first read placed the discontinuity in "test span 3". It is in **span 2**. The three
engagements, from the xiao log's `INAV_CLOCK` anchors (xiao clock + ~375 ms ⇒ blackbox time):

| span | INAV t (s) | ticks | path | disengage |
|---|---|---:|---:|---|
| 1 | 373.8 – 390.9 | 341 | 0 | reason 5 — path complete |
| 2 | **412.6 – 419.6** | 141 | 2 | **reason 1 — servo switch (pilot rescue)** |
| 3 | 442.3 – 449.1 | 136 | 3 | reason 5 — path complete |

⚠️ What makes span 3 *look* guilty on a whole-flight altitude trace is the **419.6–442.3 s coast between
spans 2 and 3**, which carries the flight's worst numbers (a 123.7 m step, a +119 m `navPos[2]−BaroAlt`
gap) and sits immediately before span 3's engage. Span 3 itself is clean: gap median **+2.9 m**, max
−11.9 m, no `pos_d` step above **0.75 m/tick**.

⭐ **Method note for anyone re-deriving this.** `mspOverrideFlags == 2` in the blackbox marks the fully
engaged window and agrees with the xiao ENGAGE/DISENGAGE events to ~1 s. Use it — it is the **only** way
to time-align the 041-t7 baseline, whose xiao log is format v4 and which `flightlog_decode.py` (v5)
correctly refuses to parse. Do not write a v4 converter (see [`flight-analysis.md`](flight-analysis.md)
§ addendum on v4/v5).

---

## 1. ⛔ The blackbox rate reverted to 1/32 — again

```
H P interval:1/32      looptime:500
Data rate 59Hz ... 276489 loop iterations weren't logged (145364ms, 96.87%)
```

| log | `P interval` | rate |
|---|---|---|
| 2026-09-07 `103927` | 1/32 | 59 Hz |
| 2026-09-07 `104749` | **1/4** | **500 Hz** |
| 2026-09-07 `114406` (×3) | **1/4** | **500 Hz** |
| **2026-09-13 (this flight)** | **1/32** | **59 Hz** |

`blackbox_rate_denom` was changed in the CLI mid-session on 09-07 and never persisted. This is the second
consecutive flight to lose the measurement [`flight-analysis.md`](flight-analysis.md) §13 asked for.

⛔ **There is no high-rate fallback.** `gyroRaw[0..2]` is in the field list, but "raw" here means
*pre-filter*, not *pre-decimation* — it is written into the same 59.4 Hz frame as everything else
(8920 samples, Nyquist **29.7 Hz**; mean `|gyroRaw − gyroADC|` is 17.5 counts, i.e. the same aliased
stream with the notch/LPF removed). `debug_mode` is 0, and INAV's debug fields decimate identically.
**Nothing in this file is above 30 Hz.**

The 2–3 Hz band this analysis lives in is safe. What is lost is §5's open question — whether the ~2.1 Hz
tone has a structural parent above 30 Hz — and any derivative-based plant fit.

⇒ **Set `blackbox_rate_denom 4` AND `save` on the bench, not at the field, and verify by reading the
header back before the next flight.**

---

## 2. ⛔ Wind speed is not a variation class

[`scenario_metadata.h:48`](../../include/autoc/rpc/scenario_metadata.h#L48) carries exactly one wind field:

```cpp
double windDirectionOffset = 0.0;  // radians, offset from base wind direction
```

There is no speed field. The base comes from [`crrcsim/autoc_config.xml`](../../crrcsim/autoc_config.xml)
line 100, `<wind velocity="12" direction="330" turbulence="1"/>`, and `T_Wind::setVelocity`
([`config.cpp:180`](../../crrcsim/src/config.cpp#L180)) stores it with **no unit conversion** into a member
its header documents as *"Wind in ft/sec"*. ⇒ every one of the 294 scenarios flies a fixed
**12 ft/s = 3.66 m/s**, direction-jittered σ = 45°.

| | training | 2026-09-13 |
|---|---|---|
| direction | base 330°, σ 45° | **from 307°** — 0.5σ, well inside |
| speed | **3.66 m/s, fixed, zero variation** | **4.2 m/s median, 6.2 p90, 6.7 max** |

⇒ direction was in-distribution; **speed was 1.15× the single trained value, gusting to 1.8×**, along the
one axis training holds constant.

### What that costs, measured

Craft-minus-rabbit decomposed onto the wind axis:

| span | mean track | **downwind** | crosswind | vertical | total trk err (med) |
|---|---|---:|---:|---:|---:|
| 1 | 338° | **+27.4 m** (−10.0 … +51.0) | +3.3 | +6.4 | 33.6 |
| 2 | 116° (*running with* the wind) | +3.3 m | +16.4 | −3.6 | 24.5 |
| 3 | 49° | **+25.2 m** (−0.4 … +33.9) | +12.8 | +9.9 | 31.8 |

⭐ **~80% of spans 1 and 3's tracking error is a single one-sided downwind bias, not tracking noise.** The
craft essentially never gets upwind of the rabbit; the range on span 3 (−0.4 … +33.9) barely crosses zero.
Span 2, the one leg running downwind, has no bias at all — which is what rules out a coincidence.

⚠️ Not to be confused with an airspeed-slot mismatch. `AIRSPEED` is **groundspeed** on both sides —
[`msplink.cpp:1144`](../../xiao/src/msplink.cpp#L1144) `setRelVel(velocity.norm())` against
[`inputdev_autoc.cpp:897`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L897)
`v = velocity_vector.norm()`, whose own comment says "ground speed magnitude". Sim and flight agree.
The gap is in the **environment distribution**, not the input vector.

---

## 3. ⭐ Prop imbalance — measurable, and it drives the estimator resets

`accVib` against motor output, this flight, whole armed period:

| motor | n | accVib med | p90 |
|---|---:|---:|---:|
| 1050–1100 | 645 | **669** | 1369 |
| 1100–1300 | 1874 | 1429 | 1928 |
| 1300–1500 | 1940 | 2824 | 4499 |
| 1500–1700 | 1096 | 4980 | 5561 |
| 1700–1900 | 1361 | **5336** | 5921 |
| 1900–2001 | 1679 | 4634 | 5742 |

**r(motor, accVib) = 0.75.** An 8× monotonic rise from idle to power. ⭐ **Clean at idle, filthy under
load ⇒ it is the prop, not the airframe, not the IMU mount, not turbulence.** (Operator: the prop hole was
hand-drilled without balancing.)

And the t3 genome parks itself in the worst band — `out_throttle` is at the **+rail for 85 / 87 / 90%** of
ticks in spans 1/2/3 (mean +0.82 / +0.85 / +0.87), giving `motor > 1900` for **90.5%** of engaged ticks
against 041-t7's **68.1%** (engaged-only `accVib` median 4661 vs 4327).

⇒ chain: **throttle railed → `accVib` ~5000 sustained → accel fusion drift → `navEPV` pinned at 999 →
vertical estimator rejects its own state and re-initialises.**

### The span-2 discontinuity, tick by tick

Six resets this flight (vs 3 on 09-05, 1 on 09-06), each with `navEPV` saturating at 999 then snapping
back, while `navPos[0]/[1]` move a normal ~1 m — horizontal is untouched, GPS is fine.

| segment | accVib med | EPV med / max | `navPos[2]−baro` med / max |
|---|---:|---:|---:|
| pre-engage | 5180 | 336 / 397 | −3.1 / −10.5 |
| span 1 | 4651 | 402 / 753 | +1.0 / −20.6 |
| coast 1→2 | 4795 | 565 / **999** | +7.8 / +34.6 |
| **span 2** | 4997 | **816 / 999** | +8.4 / **+93.2** |
| **coast 2→3** | 3570 | 688 / 998 | +21.4 / **+119.0** |
| span 3 | 3726 | 661 / 743 | +2.9 / −11.9 |
| post | 1452 | 606 / 692 | −1.2 / +8.5 |

What the **NN** was handed, from its own log:

```
t=416.94  pos_d= -4.81  vel_d= +6.87   Es=+0.46  dBnd=+0.96
t=417.00  pos_d=-40.38  vel_d=-15.88   Es=+0.78  dBnd=+0.71   <- 36 m / 22.7 m/s in ONE tick
...
t=419.04  pos_d=-39.00  vel_d=-13.77   Es=+0.60  dBnd=+0.64
t=419.10  pos_d=+47.69  vel_d=+49.17   Es=+0.78  dBnd=+1.00   <- 87 m / 63 m/s, boundary rails
t=419.24  SERVO_SWITCH -> disengage
```

Max per-tick step in the NN input stream: **span 1 = 0.75 m, span 2 = 86.69 m, span 3 = 0.75 m.**
The pilot pulled it 0.14 s after the second step. ⭐ **The rescue was correct and immediate.**

### The static port is chronically noisy, but it is NOT what changed

Baro residual about a 1 s moving mean:

| | in flight | on the ground |
|---|---:|---:|
| 2026-09-13 | **4.23 m RMS** | 0.95 m |
| 2026-08-23 (t7) | **4.51 m RMS** | 0.72 m |

Identical between the two flights. ⇒ the port placement is eating ~4.5 m of altitude resolution whenever
the airframe is moving — worth fixing on its own — but it cannot explain why *this* flight's estimator
diverged and t7's did not. **`accVib` is the variable that moved.**

---

## 4. Control quality

| span | ticks | trk err med / p90 | mean step_score | pitch rail | roll rail | thr +rail | gnd spd med | TAS(horiz) med |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 341 | 33.6 / 52.9 m | 0.064 | 0.3% | 1.2% | **85%** | 11.2 | 13.2 |
| 2 | 141 | 24.5 / 49.3 m | 0.060 | 0.0% | 0.7% | **87%** | 16.1 | 14.7 |
| 3 | 136 | 31.8 / 36.0 m | 0.078 | 0.0% | 0.0% | **90%** | 9.2 | 11.3 |

⭐ **The OOD nose-down rail is gone** — pitch railing is 0.0–0.3% against 13.0% on 09-05 and 3.8% on
09-06, and roll railing is ≤1.2%. The prefill fix plus the t3 bake have held.

⚠️ The throttle signature has **changed character** rather than improved: 09-05/09-06 showed rail
*switching* (3.96 → 2.56 Hz); t3 shows near-continuous full throttle (switching 1.29–1.56 Hz, mean
+0.82…+0.87). Part of that is honest wind response — span 3 held 9.2 m/s ground against a headwind — but
90% railed is what puts the airframe in the worst vibration band for the whole span (§3).

---

## 5. ⛔ The pitch oscillation: unchanged from t7, and NOT the rate loop

Reproduce with [`pitch_spectrum.py`](pitch_spectrum.py) (stdlib only — there is no numpy on the bench host):

```
python3 specs/043-acro-dual-loop/pitch_spectrum.py \
  flight-results/flight-20260913/blackbox_log_2026-09-13_112457.01.csv --manual
python3 specs/043-acro-dual-loop/pitch_spectrum.py \
  flight-results/flight-20260823/blackbox_log_2026-08-23_151941.01.csv
```

### 5.1 The comparison

| | **043-t3 (2026-09-13)** | **041-t7 (2026-08-23)** |
|---|---|---|
| INAV mode when engaged | `ARM\|MSPRCOVERRIDE` — **ACRO, rate loop live** | `ARM\|MANUAL\|MSPRCOVERRIDE` — **MANUAL, rate loop BYPASSED** |
| NN commands | rotation rates → 2 kHz ACRO loop | surface deflections, direct passthrough |
| wind | 4.2 m/s | **1.2 m/s (calm)** |
| genome | gen 9200 | gen 800 |
| **pitch peak** | **2.32 / — / 2.09 Hz** | **2.21 / 2.09 / 2.21 / 1.98 Hz** |
| **power in 2–3 Hz** | **56.2 / 30.4 / 74.6%** | **53.5 / 69.3 / 44.8 / 37.0%** |
| **gyro pitch RMS** | **12.7 / 13.3 / 13.3 °/s** | **12.3 / 13.9 / 11.0 / 11.5 °/s** |

(t3 span 2's global peak lands at 0.23 Hz because the estimator blow-up dumps power below 0.5 Hz; its
in-band content is still 30.4% at 2–3 Hz.)

⛔ **Same frequency, same band share, same amplitude — across a control-architecture change, a genome
change, and a 3.5× wind change.** 043 moved the NN from surface deflections to rate commands and the pitch
oscillation did not move at all.

⭐ **And the mode flags are the decisive part.** t7 flew `MANUAL` — INAV's fixed-wing rate loop was not in
the path; servos were driven straight from `rcCommand`. t3 flew ACRO with that loop fully in command. The
oscillation is identical in both. ⇒ **INAV's rate loop does not produce it and cannot be tuned out of it.**
The only loop common to both flights is **NN ↔ airframe at 20 Hz.**

⚠️ Do **not** compare the two flights' `axisRate[1]`-derived loop gains. On t7 that setpoint was computed
but never acted on (MANUAL), which is why its r is only 0.44–0.49 against t3's 0.65–0.72. The **gyro**
comparison above is apples-to-apples — same sensor, same units, same 59.4 Hz frame — the gain comparison
is not.

### 5.2 Same-air control group: it appears when the NN engages

MANUAL stretches from **this same flight**, same airframe, same air, pilot flying:

| segment | pitch peak | power in 2–3 Hz |
|---|---|---:|
| MANUAL 347–375 s | 0.81 Hz | **13.6%** |
| MANUAL 392–413 s | 0.12 Hz | **10.1%** |
| MANUAL 420–443 s | 0.46 Hz | **7.5%** |
| MANUAL 450–497 s | 0.70 Hz | **16.4%** |
| ACRO span 1 | 2.32 Hz | **56.2%** |
| ACRO span 3 | 2.09 Hz | **74.6%** |

⇒ Not turbulence, not the airframe free-flying. It is present only with the policy in the loop.

### 5.3 It is real motion, not an aliasing artefact

Three independent arguments, worth keeping because §1 leaves the >30 Hz band unobserved:

1. **Two sample rates agree.** The 59.4 Hz blackbox and the 20 Hz xiao stream both place it at ~2.1 Hz
   (xiao span 3: `gyro_q` peak 2.19 Hz, 80.8% in 2–3 Hz). A genuine alias lands at *different* apparent
   frequencies under different sampling.
2. ⭐ **`servo[0]`/`servo[1]` carry it** — 41.9% / 33.4% in 2–3 Hz, peaking at 2.09 Hz in span 3. Servo
   outputs are **computed, not sampled**; they cannot alias.
3. **The NN commands it.** `out_pitch` (xiao, span 3) peaks at 2.19 Hz with 46.6% of its power in 2–3 Hz,
   and INAV's `axisRate[1]` setpoint peaks at 2.09 Hz with 44.2%. The network is actively driving at the
   oscillation frequency, not merely riding it.

### 5.4 The rate loop amplifies it (t3, where the gain is meaningful)

| span | f₀ | gyro | setpoint | **gain** | phase | r |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 2.32 Hz | 14.8 °/s | 8.6 °/s | **1.71** | −7.8° | 0.66 |
| 2 | 2.79 Hz | 13.2 °/s | 7.9 °/s | **1.68** | −71.4° (−71 ms) | 0.65 |
| 3 | 2.09 Hz | 17.2 °/s | 11.1 °/s | **1.55** | −40.2° (−53 ms) | 0.72 |

Against the **0.51–0.70 delivered fraction** measured at low frequency in
[`flight-analysis.md`](flight-analysis.md) §10 and [`flight-analysis-20260906.md`](flight-analysis-20260906.md)
§2. ⇒ a **resonant peak of ~1.6×** at 2.1–2.3 Hz sitting inside a loop the NN closes at 20 Hz.
Amplitude ~25–36 °/s peak-to-peak.

### 5.5 What it is not, and the one open question

⚠️ **Not the short-period mode.** t7's four calm-air spans cover 12.4–15.1 m/s groundspeed (≈ TAS at
1.2 m/s wind) and sit at 2.21 / 2.09 / 2.21 / 1.98 Hz — the *fastest* span shows the *lowest* frequency.
ω_sp ∝ V predicts a 22% spread; the observed spread is within ±2 bins of ~2.1 Hz and has the wrong sign.
**The frequency is speed-independent**, which points at a fixed-timescale mechanism, not an aero mode.

Remaining candidates, none yet separable from this data:

- a **structural / actuator mode** (elevon linkage compliance, servo backlash) — the 500 Hz blackbox
  would show its parent; the 59 Hz record cannot
- a **limit cycle** set by the 50 ms tick plus plant lag — which would be architecture-independent and
  therefore fits the t7↔t3 invariance better than anything else
- the **RNN's own recurrent dynamics** — but two different genomes (gen 800, gen 9200) trained under two
  different features landing on the same 2.1 Hz argues against it

### 5.6 Roll, for completeness

3–5 Hz is comparable between the two flights. What is new is the high band: `out_roll` from the xiao
carries **30.0 / 37.4 / 30.4%** of its power in 5–10 Hz across the three spans. At a 20 Hz tick that is
2–4 samples per cycle — the roll **command** is flipping near tick-to-tick. This is the 041 roll signature
and it lives in the command, not the response.

---

## 6. What this flight establishes

**Established.**

- ⭐ The 2.0–2.6 Hz pitch oscillation is **invariant** across surface-deflection/MANUAL and
  rate-command/ACRO control, across two genomes, and across a 3.5× wind change — and is absent in MANUAL
  segments of the same flight. **It is not INAV's rate loop, and it is not the phase budget 043 targeted.**
- The OOD nose-down rail is gone (pitch railing ≤0.3%).
- Prop imbalance is quantified and is the proximate driver of the vertical-estimator resets.
- Wind speed has no variation class, and the cost is a ~25 m one-sided downwind tracking bias.
- The engage prefill and action-space fixes from 09-06 have held: 0 gaps, 0 overruns, 0 drops.

⛔ **Not established.** Nothing about the *policy's* altitude-related behaviour in span 2 — the input
vector carried an 87 m step. And no conclusion about the oscillation's physical parent, because §1 left
everything above 30 Hz unobserved.

**Ordered by expected value:**

1. ⭐ **Balance the prop.** Single highest-leverage item: it is measured (r = 0.75), it is the input to
   the estimator resets, and it is cheap. Re-run the `accVib`-vs-motor table afterwards as the check.
2. ⭐ **Set `blackbox_rate_denom 4` and `save` on the bench**, then read the header back. Two flights
   have now been lost to this. §5.5 cannot be resolved without it.
3. ⭐ **Re-open the 043 thesis against §5.** The oscillation survived the architecture change intact. A
   bench step-response on the elevon linkage (T046's instrument) is the cheapest next discriminator
   between "structural/actuator mode" and "20 Hz limit cycle".
4. **Add a wind-speed variation class** (`windSpeedScale` alongside `windDirectionOffset` in
   `ScenarioMetadata`). ⛔ This changes the wire format — Phase 1 / FR-057 ordering applies, and the t7
   baseline must be extracted first.
5. Re-site the static port (4.2–4.5 m RMS in flight, chronic across both flights).
6. `ins_gravity_cmss` is still **972.092** against a true ~979 — 0.7% low, biasing exactly the vertical
   integration that is failing.

---

## Appendix — reproducing this

```bash
# decode (blackbox-tools, INAV 8.0.0 build)
~/blackbox-tools/obj/blackbox_decode flight-results/flight-20260913/blackbox_log_2026-09-13_112457.TXT

# xiao side (v5 log, v5 decoder)
python3 src/analytics/flightlog_decode.py \
  flight-results/flight-20260913/flight_log_2026-09-13T18-24-34_flight_001.bin \
  -o /tmp/xiao_ticks.csv --flightpath /tmp/xiao_path.csv

# spectra + rate-loop gain, this flight and the t7 baseline
python3 specs/043-acro-dual-loop/pitch_spectrum.py \
  flight-results/flight-20260913/blackbox_log_2026-09-13_112457.01.csv --manual
python3 specs/043-acro-dual-loop/pitch_spectrum.py \
  flight-results/flight-20260823/blackbox_log_2026-08-23_151941.01.csv
```

⚠️ The 041-t7 **xiao** log is format v4 and will not decode with the v5 decoder — by design. Everything
in §5's t7 column comes from its blackbox alone, with spans found from `mspOverrideFlags`.

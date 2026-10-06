# 043 — prop vibration + static-port study (2026-10-04)

Two logs, both `flight-results/flight-20261004/`:

| log | what | rate | duration |
|---|---|---|---|
| `blackbox_log_2026-10-04_100946.TXT` | flight | `P interval 1/4` → 484 Hz | 141.6 s |
| `blackbox_log_2026-10-04_141959.TXT` | **outdoor bench**, craft held, throttle cycled | `P interval 1/2` → 973.6 Hz | 28.8 s |

Config of record: [`../../xiao/INAV_8.0.0_cli_20261004_142204.txt`](../../xiao/INAV_8.0.0_cli_20261004_142204.txt)
(`debug_mode = VIBE`, trimmed `blackbox` include flags — § 6).

⭐ **Headline 1 — the prop vibration is 2-blade BLADE-PASS, not imbalance.** The dominant gyro line is
`F`, and the shaft order sits at `F/2` at a median of only 7.6 % of its amplitude. At WOT `F` = 367.7 Hz ⇒ shaft
183.9 Hz = **11 032 RPM**. Blade-pass beats shaft-order **~13:1** (median), so the prop is *well
balanced* and what remains is intrinsic 2/rev excitation. **Balancing buys nothing.**

⭐ **Headline 2 — vibration is not a threat to the 043 rate loop.** `gyroRaw` carries 19 °/s RMS above
20 Hz; the filter chain removes 92–96 % of it, leaving **< 1.2 °/s** in `gyroADC`. `accClipCount = 0`
across the whole bench run including WOT. No action needed.

⭐ **Headline 3 — the static port has a real airspeed-dependent error: ~11 m on this flight** (§ 5),
and the baro is the heaviest contributor to an input the NN actually consumes. ⛔ **But it is NOT the
cause of the large `navPos[2]` jumps** (§ 5.1) — and neither is vibration (§ 5.2). Every jump across
every flight in the repo is **autoc-engagement-associated**, and `navEPV` hitting `inav_max_eph_epv`
separates troubled flights from clean ones 20/20.

---

## 1. ⛔ The 484 Hz flight log ALIASES the prop line — do not read its spectrum directly

The prop line lives at 250–400 Hz. The flight log samples at **484 Hz (Nyquist 242 Hz)**, so the line
folds down and appears to move **downward** as throttle rises:

| throttle | apparent (in CSV) | true = 484 − apparent |
|---|---|---|
| 1100–1300 | 227.7 Hz | 256 Hz |
| 1450–1600 | 161.6 Hz | 322 Hz |
| 1600–1750 | 148.4 Hz | 335 Hz |
| 1750–2000 | 91.7 Hz | 392 Hz |

⛔ **The flat ~89 Hz and ~160 Hz "peaks" in the flight log are this fold-down, not structural modes.**
An earlier read of the flight log treated them as a candidate fixed resonance; the bench run disproves
it (§ 3). Anyone re-deriving from a `1/4`-rate log must de-alias first.

The onboard `gyroPeak*` fields are **not** affected — they come from INAV's own FFT at
`1e6/looptime/2` = 1000 Hz sampling (0–500 Hz, 15.6 Hz bins; `FFT_WINDOW_SIZE 64` in `src/main/flight/gyroanalyse.h`),
and they agreed with the de-aliased values to within ~15 Hz. That cross-check is what made the
de-aliasing trustworthy before the bench run existed.

## 2. The order analysis (bench, un-aliased)

Method: `gyroRaw[1]` (pitch, **pre-filter**), 2048-pt Hann, `fs` = 973.63 Hz from
`(n−1)/(t[-1]−t[0])` — *not* median-`dt`, which overstates `fs` by 1.09 % here and puts a 4 Hz
systematic error on every number. Dominant line `F` is taken directly (no halving), then amplitudes are
measured at `F/2`, `1.5F`, `2F`. `1.5F` is the non-harmonic control.

| quantity | value |
|---|---|
| `F` vs motor command | **r = +0.9257** |
| `A(F/2) / A(F)` | mean 0.128, **median 0.076** |
| `A(1.5F) / A(F)` (non-harmonic control) | **0.015** |
| `A(2F) / A(F)` | 0.013 |
| WOT (motor > 1880, n = 6) | `F` = 367.7 Hz, `F/2` = 183.9 Hz = **11 032 RPM** |
| WOT amplitudes | `A(F)` = 12.81 °/s, `A(F/2)` = 0.66 °/s |

All figures above are the output of [`vibration_order.py`](vibration_order.py) on
`blackbox_log_2026-10-04_141959.01.csv` — re-running it reproduces them exactly.

`A(F/2)` is a median **7.6 %** of `A(F)` (mean 12.8 %) against a **1.5 %** non-harmonic control floor —
so the subharmonic is **real** (5–8× the floor) but **small**. That is the whole diagnosis: `F` is the
2/rev blade-pass line, `F/2` is the 1/rev shaft line (imbalance), and blade-pass beats shaft-order by
roughly an order of magnitude.

Sample of the sweep (both ramps repeat):

| t (s) | motor | `F` (Hz) | `A(F/2)` | `A(F)` | shaft RPM |
|---|---|---|---|---|---|
| 5.8 | 1271 | 223.4 | 0.56 | 7.38 | 6 703 |
| 7.4 | 1533 | 270.5 | 0.24 | 5.27 | 8 115 |
| 9.5 | 1970 | 370.3 | 0.41 | 13.56 | 11 110 |
| 15.3 | 1308 | 234.8 | 1.28 | 15.92 | 7 045 |
| 25.2 | 1921 | 366.5 | 0.92 | 14.96 | 10 996 |

⭐ **Hardware corroboration, found after the fact** in [`../../xiao/`](../../xiao/): the eCalc sheets name
an **EMAX EcoII 2207-2400 Kv** on **3S** with **APC Electric E 5.25×4.25** / **Master Airscrew GF 5.5×4**.
Both props are **2-blade**, and 2400 Kv on 3S loaded lands right at ~11 000 RPM. Three independent
routes — order ratio, measured `F/2`, and the hardware spec — agree.

One transition window (t = 20.5 s, `F` = 132.6 Hz while the throttle is still coming up) mis-picks a
low line as `F`; it is the main cost to r. A harmonic-sum search for `f0` was tried and is **worse**: a lone strong line at `F` scores
better as `f0 = F` than as `f0 = F/2`, so it reports blade-pass as the shaft order. Use the dominant
line + subharmonic test instead.

## 3. ⭐ No structural resonance in 40–487 Hz — closes t3 § 5

[`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 5 left open whether the ~2.1 Hz pitch
tone has "a structural parent above 30 Hz". Test: a swept line cannot survive a **25th-percentile
spectrum** across windows; an always-present line can. Every candidate bin came back with
`median/p25` of **1.9–2.8** (a fixed line would sit near 1.0). **Nothing in 40–487 Hz is
throttle-independent.** No motor-excited structural mode exists in that band.

Caveat: the bench holds the airframe, which changes the structural boundary conditions, and it cannot
excite a purely *aerodynamic* mode (buffet). This rules out a mechanical/motor-driven parent, not an
aero one.

## 4. ⛔ Correction to the t3 record — "prop imbalance" is the wrong attribution

[`flight-analysis-20260913.md`](flight-analysis-20260913.md) reports **"prop imbalance (`accVib` is a
direct function of motor output, r = 0.75)"**. The `accVib`-vs-throttle correlation is real and stands.
The *attribution* does not: imbalance is the 1/rev line, and 1/rev is a median 7.6 % of the blade-pass line.
`accVib` rises with throttle because **blade-pass excitation** rises with throttle, not because the prop
is out of balance. Chasing prop balance on the strength of that correlation would have been wasted work.

⭐ Worth stating plainly because the prior was reasonable: t3 § 3 records that *"the prop hole was
hand-drilled without balancing"*, which is exactly the history that makes imbalance the obvious
suspect. The order analysis says the 1/rev line is small **anyway** — a median 7.6 % of blade-pass,
`A(F/2)` = 0.66 °/s at WOT. Hand-drilled or not, this prop is running acceptably true, and the residual
vibration is the 2/rev term you cannot balance out of a two-blade prop.

Severity, bench, motor spinning:

| axis | `gyroRaw` > 20 Hz | `gyroADC` > 20 Hz | removed |
|---|---|---|---|
| roll | 18.99 °/s | 1.13 °/s | 94.1 % |
| pitch | 18.89 °/s | 0.75 °/s | 96.0 % |
| yaw | 5.28 °/s | 0.43 °/s | 91.8 % |

`accClipCount` = **0** over 28.8 s. `accVib` median 3116 on the bench vs 4894 in flight — *lower* with
the prop at full song than in flight, which confirms the flight `accVib` is dominated by gusts and
manoeuvring, not the prop. Most of the attenuation is `gyro_main_lpf_hz = 25` (a PT1, ≈ −23 dB at
340 Hz), not the dynamic notch.

⚠️ **Amplitudes above ~256 Hz are understated and cannot be fixed from the log side.** INAV 8 hardcodes
the MPU6000/6500 hardware DLPF to `GYRO_LPF_256HZ` — `mpuGyroConfigs[]` in
`src/main/drivers/accgyro/accgyro_mpu.c` has *only* 256 Hz entries, and the `gyro_lpf:0` in the blackbox
header is a hardcoded literal in the header printer, **not** a live setting. There is no CLI knob. True
frequencies are correct; amplitudes at the top of the sweep are attenuated. For real amplitudes at
250–400 Hz the FC gyro is the wrong instrument — that needs an external accelerometer.

## 5. ⭐ Static port: ~11 m airspeed-dependent error

**Bench (craft held stationary ⇒ true altitude constant):** `BaroAlt` swings **2.00 m** while sitting
still. Noise scales hard with throttle — σ **4.1 cm at idle → 35.3 cm at WOT**. But there is **no
propwash bias**: idle mean 155.1 cm vs WOT 156.1 cm (**+1.0 cm**), r = −0.056. So propwash adds
*noise*, not offset. Noise filters out; offset would not.

**Flight (the real test — the bench has no airspeed):** `BaroAlt` vs `GPS_altitude`, both mean-removed.
Static-source error should scale with V². Controlling for the obvious confound — speed and climb rate
are anti-correlated in energy-exchange manoeuvres, and GPS altitude lags — by filtering on GPS
`velned[2]`:

| filter | n | corr(err, V²) |
|---|---|---|
| all in-flight | 1416 | −0.420 |
| \|vz\| < 200 cm/s | 627 | −0.642 |
| \|vz\| < 100 cm/s | 359 | **−0.686** |
| \|vz\| < 50 cm/s | 177 | −0.678 |

⭐ **The correlation gets STRONGER under the level-flight filter.** GPS lag would make it *weaker*, so
this is a genuine static-source error, not an artifact. In near-level flight:

| GPS speed | mean baro−GPS error |
|---|---|
| 10–14 m/s | **+6.72 m** |
| 14–18 m/s | +0.93 m |
| 18+ m/s | **−5.12 m** |

⇒ an **11.4 m speed-dependent swing**. Sign: baro reads *low* at speed ⇒ the static port is picking up
ram pressure. The operator's suspicion that the port "is not well defined" is **confirmed**.

⚠️ GPS here is weak in its own right — **6 satellites** throughout, median `epv` 395 cm. So absolute
GPS altitude is only good to ~4–6 m. But GPS error is *not* airspeed-correlated, which is what makes the
V² trend attributable to the baro.

### 5.1 ⛔ The static port is NOT what causes the large Z jumps — vibration is

The question this section was written to answer: are the +93 m / +119 m `navPos[2]−BaroAlt` excursions
in [`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 3 a static-pressure problem? **No.**

**Magnitude is 20× short.** Measured speed-dependent baro error, same method as § 5, on both flights:

| flight | speed-dependent swing | corr(err, V²) \|vz\|<100 | err sd |
|---|---|---|---|
| t3 2026-09-13 (6 resets, diverged) | **4.9 m** | −0.085 | 13.03 m |
| t7 2026-08-23 (1 reset, control) | **4.1 m** | −0.204 | 5.77 m |

⭐ **t3 and t7 are the same on this metric**, so the static port cannot explain why t3 diverged and t7
did not. This independently re-confirms t3 § 3's conclusion — and it is **not** the same test: t3
measured residual about a *1 s moving mean*, which high-passes away precisely the slow V² bias, so that
check was blind to it. Testing the bias directly still clears the baro.

⚠️ Note the speed-dependent error is **4–5 m on t3/t7 but 11.4 m on 2026-10-04** (§ 5). The port error
is condition-dependent, so § 5's 11.4 m is not a constant of the airframe. Even 11.4 m is far short.

**The amplifier is in INAV, and it is driven by vibration.** `navigation_pos_estimator.c:771`:

```c
const float accWeight = navGetAccelerometerWeight();
vectorScale(&ctx.estPosCorr, &ctx.estPosCorr, 1.0f/accWeight);   // <-- correction GAIN
vectorScale(&ctx.estVelCorr, &ctx.estVelCorr, 1.0f/accWeight);
```

`accWeight` falls as vibration rises (`updateIMUEstimationWeight`, same file): `accVib` is clamped to
1.0–3.0 g and mapped linearly to **1.0 → 0.3**, then multiplied by 0.5 whenever the accel clips. Since
blackbox `accVib = accGetVibrationLevel() * acc_1G` with `acc_1G = 2048`, log units convert as
`g = accVib/2048`:

| segment | `accVib` | g | `accWeight` | correction gain |
|---|---:|---:|---:|---:|
| t3 span 2 | 4997 | 2.44 | 0.496 | **2.02×** |
| t3 pre-engage | 5180 | 2.53 | 0.465 | 2.15× |
| t7 engaged (control) | 4327 | 2.11 | 0.611 | 1.64× |
| t3 post-flight | 1452 | 0.71 | 1.000 | 1.00× |
| *floor at 3 g* | 6144 | 3.00 | 0.300 | 3.33× |
| *floor with clipping* | — | — | 0.150 | **6.67×** |

⇒ chain, now with the gain named: **throttle railed 90 % → blade-pass excitation maxed → `accVib`
≈ 2.4 g → `accWeight` ≈ 0.5 → every vertical position/velocity correction doubled → estimator
under-damped → `EPV` climbs to `max_eph_epv` (`inav_max_eph_epv = 1000` cm — *this is what "EPV pinned
at 999" is*) → `estZCorrectOk` fails, vertical velocity decayed to zero, Z re-initialises → step.**

⛔ **But `accVib` is NOT what separates the flights that jump from the flights that don't** — see
§ 5.2. The `1.0f/accWeight` gain is real code and a real destabilising term, but the all-flights survey
rules it out as the *cause*. Treat it as a contributing gain, not the trigger.

⚠️ Honest limit: the 2.0× gain alone does not arithmetically turn a 5–10 m sensor disagreement into a
90 m step. Quantifying it end-to-end needs an estimator replay, not a log read — not attempted here.

### 5.2 ⭐ All-flights survey: every Z jump is autoc-associated, and vibration does not predict it

All 21 `.TXT` files in `flight-results/` **re-decoded from source** (29 logs — see the ⛔ data-integrity
note below), scanning for per-sample `|Δ navPos[2]| > 5 m` and tagging each by `mspOverrideFlags == 2`.
**23 logs carry `navPos[2]`; 7 distinct flights have events, 24 events in total** (6 in-span,
18 post-disengage).

⭐ **Every one of the 24 events is inside an engage span or within 10.3 s of the end of one.**

| log | engage spans | events | worst step | where |
|---|---|---:|---:|---|
| 2026-03-20 | 29-40, 55-60, 83-89, 126-139, 167-173 s | 2 | 23.0 m | +7.7 s, +2.9 s after disengage |
| 2026-03-27 | 32-41, 65-68, 89-92, 119-124 s | 9 | **overflow** | all +2.0…+3.0 s after disengage |
| 2026-04-07 #2 | 32-52, 78-94 s | 2 | 23.0 m | 1 in-span, 1 at +5.3 s |
| 2026-04-26 #1 | 25-33, 55-78, 100-117 s | 1 | 20.1 m | +4.3 s after disengage |
| 2026-09-05 | 24-26, 43-58, 77-99, 129-138 s | 3 | 45.5 m | 2 in-span, 1 at +10.2 s |
| 2026-09-06 | 25-42, 54-77, 94-119 s | 1 | 32.1 m | in-span |
| 2026-09-13 (t3) | 28-46, 67-73, 96-104 s | 6 | **123.7 m** | 2 in-span, 4 at +0.2…+4.2 s |

⇒ the "unengaged" events are the **post-disengage wake** of an already-destabilised estimator, not
independent human-control events. **No event occurs at launch or landing.** This is the operator's
hypothesis, confirmed: under human control alone, this does not happen.

⚠️ **But engagement is necessary, not sufficient.** 15 of the 23 logs are clean, and **13 of those 15
had autoc engaged** — including the four highest engagement fractions in the whole set (2026-08-23 at
43.3 %, 04-26 #2 at 40.9 %, 05-03 #1 at 40.3 %, 05-17 at 35.9 %). So engaging autoc does not by itself
produce a jump; something else has to co-occur. Finding that second factor is the open question.

⛔ **And vibration is ruled out as the cause.** The two highest-`accVib` logs that carry `navPos[2]`
were **never engaged** and have **zero** events, while the flight with the third-most events has the
*lowest* vibration of the group:

| log | engaged | `accVib` median | g | gain | events |
|---|---|---:|---:|---:|---:|
| 2026-09-07 `103927` #2 | **0 %** | 5377 | **2.63** | 2.32× | **0** |
| 2026-04-03 #2 | **0 %** | 4755 | **2.32** | 1.86× | **0** |
| 2026-09-05 | 29.7 % | 2714 | **1.33** | 1.13× | **3** |
| 2026-09-13 (t3) | 20.8 % | 3811 | 1.86 | 1.43× | 6 |

⇒ t3 § 3's `accVib` attribution and § 5.1's `accWeight` amplifier both fail as *causes*. Engagement is
the discriminator.

⭐ **One marker separates perfectly, 20/20 logs with no exceptions:** `navEPV` reaching the
`inav_max_eph_epv = 1000` cm cap. Every log with an event peaks at 999 (or 65535 on 03-27's overflow);
every clean log peaks at **463–936**. Note the events themselves fire at `navEPV` 324–676, so the cap is
a *flight-level* marker of a troubled estimator, not the instantaneous trigger. **`inav_max_eph_epv` is
a tunable and is the first thing to investigate.**

### Hypotheses tested and eliminated

| hypothesis | verdict |
|---|---|
| Accelerometer hitting its rail | ⛔ **No.** FSR is hardcoded `INV_FSR_16G` = ±16 g — the *widest* the MPU6000/6500 offers (`accgyro_mpu6000.c:107`, `acc_1G = 512*4 = 2048`). `ACC_CLIPPING_THRESHOLD_G = 15.9` per axis. At event times filtered accel is **≤ 8.8 g**. |
| MSP polling starving the FC loop | ⛔ **No.** Frame `dt` engaged vs unengaged is identical: median 16.8 vs 16.8 ms, p99 17.3 vs 17.3, max 17.5 vs 17.5. |
| Static-pressure error | ⛔ **No** — § 5.1. |
| Vibration / `accVib` / `accWeight` | ⛔ **Not the cause** — table above. |
| Something specific to NN engagement | ⭐ **Open, and where the evidence points.** |

⚠️ **The clipping hypothesis is not fully closed.** `accSmooth` is filtered at `acc_lpf_hz = 15`, so a
single-sample 16 g spike reads far lower, and `accClipCount` lives in `debug[3]` under `debug_mode =
VIBE` — which **no historical flight has** (`debug_mode:0` on all of them). ⇒ **set `debug_mode = VIBE`
for the next autoc flight**; it is the one cheap measurement that would close it.

### ⛔ Data-integrity trap found while doing this

The flash was not erased before 2026-07-20 and 2026-09-07 `103927`, so **log index 1 in those files is
the *previous* flight**, still sitting in flash. The tracked `…_2026-07-20_175707.01.csv` is byte-identical
to 07-13's, and `…_2026-09-07_103927.01.csv` to 09-06's — both are the **wrong flight**. More generally
`.01.csv` only ever holds **log 1**, so 2026-04-03 #2, 04-07 #2, 04-26 #2, 05-03 #2, 07-20 #2 and
09-07 #2 had never been analysed at all. ⇒ always `blackbox_decode --index N` across every index, and
`flash_erase` before each flight.

### Recommendation

⛔ **"IMU integration is fine for short flights" does not hold** and INAV offers no such mode. Pure
inertial altitude double-integrates: error ≈ ½·b·t². A 0.01 g accel bias gives **176 m at 60 s**; these
flights run 142 s. A 1° attitude error leaks 0.0175 g and is worse. `inav_allow_dead_reckoning` is
already `OFF`. The real choice is baro+GPS vs GPS-mostly — GPS vertical is noisy but **bounded**.

⛔ **The NN *is* fed INAV's estimated vertical position — altitude is squarely in the control path.**
`state.altitude` / `altitude_valid` (`xiao/include/state.h`) are indeed vestigial and `MSP_ALTITUDE` is
never requested, but that is a red herring: the xiao pulls a **custom** `MSP2_AUTOC_STATE`
(`msplink.cpp:639`) whose payload carries `int32_t pos[3]  // cm in NEU frame`
(`xiao/include/MSP.h:263`), and `msplink.cpp:892` feeds it straight into
`aircraft_state.setPosition(neuVectorToNedMeters(state.autoc_state.pos) - …)`. The vertical component is
`posEstimator.est.pos.z` — the same quantity the blackbox logs as `navPos[2]`.

⇒ every estimator reset lands in the NN's `pos_d` input. That is exactly what
[`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 3 recorded as an **86.69 m single-tick
step** in the NN input stream. `failsafe_procedure = DROP` does mean failsafe itself needs no altitude.

⇒ **this raises the priority of the `GPS_ONLY` change**, because `inav_w_z_baro_p` (0.350) >
`inav_w_z_gps_p` (0.200) means the chronically-noisy baro is currently the **heaviest** contributor to an
NN input. It is *not*, however, the cause of the big jumps — see § 5.1.

⛔ **But the current config weights the bad sensor highest.** `inav_default_alt_sensor = GPS` does
**not** mean GPS-only — INAV's own description: *"Settings GPS and BARO always use both sensors unless
there is an altitude error between the sensors that exceeds a set limit."* And
`inav_w_z_baro_p = 0.350` > `inav_w_z_gps_p = 0.200`. So baro currently carries **more** weight than GPS.

⇒ **The surgical change:**

```
set inav_default_alt_sensor = GPS_ONLY
save
```

`GPS_ONLY` zeroes the baro weight while GPS-Z is valid and keeps baro as a **backup** if GPS drops
(`navigation_pos_estimator.c`: `wBaro = default == GPS_ONLY && EST_GPS_Z_VALID ? 0 : 1`).

⛔ **Do not set `baro_hardware = NONE`.** It costs the backup, OSD and telemetry altitude for zero gain,
and silently changes behaviour if failsafe is ever moved off `DROP` or a nav mode is enabled.

Fixing the port itself is only worth it if altitude ever becomes load-bearing — it is not today.

## 6. ⛔ Blackbox rate did not persist as 1/1 — third instance

Requested `blackbox_rate_denom = 1`; the dump and the log header both read **1/2**.

This is **not** an INAV clamp: `max_denom = 4096000/looptime` = **8192**, and the only normaliser forces
`1/1` when `num >= denom` (`blackbox.c` `blackboxValidateConfig`). `1/1` is reachable.

[`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 1 already records two instances of the
rate silently reverting ("changed in the CLI mid-session on 09-07 and never persisted"). Likely
mechanism, now identified: **the Configurator's Blackbox tab writes the entire blackbox config over
MSP** (`fc_msp.c`, `blackboxConfigMutable()->includeFlags = sbufReadU32(src)` and neighbours), so
opening that tab after a CLI `save` overwrites the rate. ⇒ set it in the CLI, `save`, and **do not
revisit the Blackbox tab**; verify with `get blackbox_rate_denom` and by reading `P interval` back out
of the log.

⭐ **1/2 was adequate anyway — don't chase 1/1.** Nyquist landed at 486.8 Hz, above the 367 Hz WOT
blade-pass, so the fundamental and the subharmonic were captured un-aliased. The run was healthy:
**0.02 % missing iterations**, 45.2 kB/s, looptime 513 µs ± 21.6 (*better* than the 519 µs seen at 1/4).

Remaining gap: at WOT the **second** blade-pass harmonic (4× shaft ≈ 734 Hz) is above Nyquist even at
1/2, so the `A(2F) ≈ 0` entries there mean *unmeasurable*, not *absent*. Where it is in band at low/mid
RPM it measures ~0.2 °/s. Only a `1/1` run would close that, and the evidence says it is not worth a
flight.

### Settings used (reproduce with these)

```
set blackbox_rate_num = 1
set blackbox_rate_denom = 2          # 1 is reachable but did not persist; 2 is sufficient
set debug_mode = VIBE                # debug[0..2] = accVibeLevels xyz, debug[3] = accClipCount
blackbox GYRO_RAW                    # ESSENTIAL - pre-filter tap; gyroADC is gutted by gyro_main_lpf_hz=25
blackbox MOTORS                      # RPM proxy - escRPM is garbage (no ESC telemetry)
blackbox RC_COMMAND
blackbox ACC
blackbox PEAKS_P
blackbox -PEAKS_R
blackbox -PEAKS_Y
blackbox -SERVOS
blackbox -NAV_ACC
blackbox -NAV_POS
blackbox -NAV_PID
blackbox -MAG
blackbox -ATTI
blackbox -RC_DATA
blackbox -QUAT
save
```

Then `flash_erase` before each run — flashfs needs pre-erased space and does **not** erase on the fly,
which is what keeps 50–200 ms NOR sector-erase stalls out of the data. Capacity: `flash_info`. The
F722-**MINI** logs to onboard SPI NOR (`USE_FLASHFS`/M25P16, `target/MATEKF722SE/target.h`) — it is the
F722-**SE** that gets the SD card, so capacity is the binding limit (28.8 s ≈ 1.3 MB at 1/2).

⚠️ `escRPM` reads a constant `1778576784` — no ESC telemetry, hence `rpm_gyro_filter_enabled = OFF`.
An RPM filter would be the right tool for a line that sweeps 170 Hz, but it needs that telemetry wired
up first. Not needed while the 25 Hz LPF already leaves < 1.2 °/s.

## 6. ⭐ What autoc does to the static port — and the ±7 g pitch bang-bang

Two operator hypotheses for the § 5.2 "second co-factor", both tested: **(a)** static-port error as a
function of AOA, since autoc slams the pitch; **(b)** quantisation / the accel LPF.

### 6.1 Measurement budget — the data is sufficient

`BaroAlt` updates at **~27 Hz** (median hold of 2 samples at 59 Hz logging, consistent across all 29
logs regardless of log rate). So its true Nyquist is ~13.5 Hz and lag resolution is **~37 ms** — a
100–300 ms `control → AOA → accel` chain would have been resolvable. 21 engaged logs, thousands of
engaged samples each. **The data was not the limitation.**

### 6.2 ⭐ The static port is 3–17× over-physical, but ONLY when engaged

Assumption-free test, no modelling: between consecutive `BaroAlt` **updates**, how far does the baro
move, against the most it physically could at the estimator's own vertical speed (`|navVel[2]| × dt`)?

| log | engaged p95 | engaged max | physical bound p95 | unengaged p95 |
|---|---:|---:|---:|---:|
| 2026-09-13 (t3) | 759 cm | **2611 cm** | 46 cm | 75 cm |
| 2026-08-23 (t7) | 528 cm | 1316 cm | 34 cm | 75 cm |
| 2026-07-20 #2 | 586 cm | **2194 cm** | 43 cm | 66 cm |
| 2026-04-26 #1 | 538 cm | 1582 cm | 53 cm | 76 cm |

⇒ **engaged, the baro moves 3–17× further than physically possible (p95 of the step against the p95 bound; worst flights 14–17×) (spikes to 26 m); unengaged it is
roughly physical.** Raw t3 trace mid-engagement — `6575 → 8269 cm` in one 17 ms sample, a **21 m step**,
while `navPos[2]` descends smoothly at −7.7 m/s with no jump:

```
  38.91     6575     6700      268  -3.401
  38.93     8269     6701      206  -5.955   <- +21 m in one sample
  38.95     8269     6698      134  -7.062
  38.96     7693     6696       74  -6.221
```

Corroborating, paired within each flight (same port, same weather, same day): baro residual RMS about a
0.5 s moving mean is **3.6–12.8× higher engaged** in **every** flight, with mean `|pitch rate|`
**2.9–5.7×** higher. ⇒ hypothesis (a) **confirmed**: autoc's pitch slamming wrecks the static source.

⚠️ **No lead.** Cross-correlating baro residual against normal-accel residual peaks at **lag 0** (one
flight at −17 ms = one sample), r = **−0.65 to −0.89**. So the `control → AOA → accel` ordering is *not*
resolvable: the pressure error is simultaneous with the normal-load swing, consistent with AOA driving
the port error directly rather than a delayed cascade. Phase alone cannot separate "AOA artifact" from
"real motion" — both predict lag 0. **Amplitude** is what separates them: at ~2 Hz even a ±1 g
oscillation displaces only **6 cm** (`A = a/ω²`), so the 1.5–7.6 m baro swings are physically impossible
as altitude. The baro is not measuring height at these timescales.

### 6.3 ⛔ Quantisation and the accel LPF are categorically not it

| | value |
|---|---|
| `acc_1G = 2048` LSB/g at ±16 g ⇒ quantum | **0.49 mg = 4.8 mm/s²** |
| actual signal | **6–11 g** |
| `acc_lpf_hz = 15` at the ~2 Hz oscillation | negligible attenuation |

Four orders of magnitude between the quantum and the signal. Quantisation cannot make metres. **Ruled
out.**

### 6.4 ⭐ The pitch oscillation is pulling ±7 g

`|accSmooth[2]|` runs **p99 5.2–7.5 g, max 8.4–11.4 g** across engaged flights, and the raw t3 trace
swings **−7.1 g to +8.4 g at ~2 Hz**. That is the *15 Hz-filtered* value, so raw transients are higher —
against `ACC_CLIPPING_THRESHOLD_G = 15.9`. The rail is roughly **2× away on filtered data and closer on
raw**, which is why the clipping question in § 5.2 stays open and why `debug_mode = VIBE` matters on the
next autoc flight.

⭐ **This reframes the 2–3 Hz pitch oscillation.** It is not a tracking nuisance — it is a **±7 g
structural and aerodynamic loading cycle** that simultaneously destroys the static source and pushes the
accelerometer toward its rail. It is the common parent of most of what this document measures.

### 6.5 ⛔ Still not the discriminator

Like `accVib`, the static-port error is **universal to engaged flights and does not separate jumps from
clean**: t7 is clean and shows the same 3–17× over-physical baro steps as t3's six jumps. GPS quality
does not separate either — JUMP `epv` medians span 286–419 cm while CLEAN spans 226–**572** cm, i.e.
clean flights include worse GPS than any jump flight.

| factor | real? | discriminates jump vs clean? |
|---|---|---|
| Blade-pass vibration (§ 4) | yes | **no** |
| Static-port AOA error (§ 6.2) | **yes, 3–17× over-physical** | **no** |
| GPS quality (sats, `epv`) | varies | **no** |
| Loop timing (§ 5.2) | no effect | — |
| Accel rail | **unconfirmed, ~2× away** | unknown |
| `navEPV` reaching `inav_max_eph_epv` | — | ⭐ **yes, 23/23** |

⇒ Four confirmed-but-non-discriminating factors and one perfect marker. The honest reading is that this
may be a **threshold-crossing race** rather than a distinct root cause — bad conditions are present on
every engaged flight, and whether `navEPV` crosses the 1000 cm cap decides whether it becomes a reset.
If so the levers are `inav_max_eph_epv` and `inav_w_z_baro_p`, **and above all removing the ±7 g
excitation** — not a further hunt for a missing cause.

Reproduce § 6 with [`static_port_aoa.py`](static_port_aoa.py) (stdlib only; decode the `.TXT` first).

## 7. Bench-test protocol that produced this

Outdoor, craft held, prop on, throttle cycled idle → WOT → idle twice over 28.8 s (double ramp, so
every RPM is visited twice and repeatability is checkable). **Bench beats flight for this**: in the
flight log throttle is entangled with attitude and gusts, which held the frequency/throttle correlation
to r ≈ 0.6–0.84; the bench ramp isolates it.

Limits to keep in mind: holding the airframe changes structural boundary conditions, and static (no
forward airspeed) loading means lower RPM per throttle than in flight — so the bench is right for
*identifying orders* and wrong for *absolute in-flight amplitudes* or aero modes.

Reproduce with [`vibration_order.py`](vibration_order.py).

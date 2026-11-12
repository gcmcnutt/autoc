# 045 N1 — drift report (US3, FR-006 / FR-006a, SC-003)

Report of record for tasks T024–T029, 2026-11-12. Offline, on the committed blackbox and xiao logs of 11-08 (article 1, 044-t1
g800, 4 spans) and 10-18 (article 1, 043, two flights: log `.01` with 4 spans and log `.02` with 3 spans). Data:
[`n1/`](n1/). Method and caveats per section; reproduction at the end.

## Headline

1. **The GPS-derived reference is not valid under load on these flights.** The GPS-implied |specific force| falls behind the
   IMU's as load rises: ratio ≈ 1.0 at 1.5–2 g, 0.82–0.85 at 2.5–3.5 g, 0.71–0.73 at 3.5–5 g, 0.55–0.63 above 5 g, the same on all
   three logs. This needs no attitude, so it is the receiver's side. It matches `gps_dyn_model = AIR_2G` (u-blox dynamics capped at
   2 g), which all three flights flew. ⇒ Attitude can be checked against GPS only at the **low-load moments** (≤ 2 g) inside the
   spans. GPS velocity truth is also taken only at low-load instants. AIR_4G is on article 1 since 11-11: the re-fly must repeat this
   table (`span_attitude.py` writes it).
2. **The committed logs cannot resolve attitude drift below ~10° in the spans.** Against the low-load reference, INAV and
   gyro-only-from-engage are **statistically indistinguishable in every span** (whole-span fits differ by ≤ 1 σ, σ = 6–30°), even
   though the two sources disagree with each other by 0.6–14.6° inside the spans. Neither source is shown better under load.
3. **Cruise windows before engage show a consistent INAV heading offset**: −5 to −19° in all seven 10-18 windows. The five
   best-constrained windows (σ 3–5°, four on 10-18 and one on 11-08) read −8.6 to −18.8°. Tilt in those five is 1–7°, which is
   unresolved. The magnitude is close to the site's magnetic
   declination (≈ 12.9° E). Check the sign convention and the in-flight declination before attributing it to the compass (R19);
   the re-fly's box segment is the test.
4. **A span-local IMU propagator drifts fast**: horizontal velocity error 2–26 m/s and position 4–146 m at 10 s; 5–53 m/s and
   45–636 m at the end of the 15–22 s spans, with either attitude source. INAV's GPS-aided estimate stays at 0.4–6 m/s and 1–6 m,
   except in the 11-08 freeze (20 m/s, 80 m). ⇒ Any companion frame needs GNSS aiding for velocity and position, not only attitude.
5. **The 11-08 freeze was triggered by `gps_min_sats`, not by a lost fix.** The receiver kept a 3D fix with **5** satellites
   (GPS EPH 4.1–5.4 m) for 4.5 s. INAV dropped it because 5 < `gps_min_sats` 6. After the 1.5 s GPS timeout, EPH grew as eᵗ from
   4.3 m, crossed the 10 m cap at 7.7 s, and the estimate froze. It froze for 2.3 s, then reset by 106 m on reacquire, with a
   25 m Z step 6 s later. Multi-GNSS (13–20 sats on the bench) removes this 5↔6-satellite edge.
6. **Budget**: < 1° over a span is **not reachable** by INAV's AHRS, un-aided gyro (any of the three parts) or an on-chip-fusion
   part. A GNSS-velocity-aided filter is the only candidate. On this receiver and config its reachable floor is estimated at
   **~2–3°** (it cannot be measured here). The working target becomes **"smooth (no resets) and ≲ 3°"**, with 1° kept as the design
   aim, to be re-checked on the re-fly logs (research R4 § N1 result).

## 1. Spans and envelope (research R3 columns; `n1/span_envelope.csv`)

Engaged spans are runs of `mspOverrideFlags == 2`. 11-08 and 10-18 `.01` match research R3. 10-18 `.02` is new: R3 had only `.01`
decoded.

| flight · span | length | gross rotation roll / pitch / yaw | net rotation roll / pitch / yaw | max rate r / p / y (°/s) | \|f\| mean · p95 · time > 2 g |
|---|---|---|---|---|---|
| 11-08 · 1 | 16.7 s | 1553 / 1969 / 505° | -598 / **1314** / 240° | 423 / 401 / 120 | 3.80 · 6.57 g · 83 % |
| 11-08 · 2 | 21.9 s | 1749 / 2214 / 588° | -54 / **1351** / 19° | 345 / 346 / 82 | 2.85 · 5.36 g · 70 % |
| 11-08 · 3 | 21.1 s | 1920 / 2160 / 621° | -392 / **1075** / 271° | 341 / 324 / 104 | 2.87 · 5.69 g · 68 % |
| 11-08 · 4 | 21.2 s | 1873 / 1976 / 592° | -482 / **961** / 209° | 301 / 244 / 104 | 2.43 · 4.88 g · 58 % |
| 10-18a · 1 | 6.3 s | 1015 / 728 / 276° | -13 / **403** / -35° | 553 / 276 / 154 | 3.93 · 8.26 g · 74 % |
| 10-18a · 2 | 16.5 s | 1703 / 1511 / 509° | -446 / **962** / 149° | 427 / 245 / 121 | 2.85 · 5.77 g · 66 % |
| 10-18a · 3 | 22.1 s | 2805 / 1959 / 736° | -170 / **1108** / 195° | 454 / 301 / 149 | 2.73 · 6.01 g · 58 % |
| 10-18a · 4 | 21.5 s | 1987 / 1918 / 642° | -644 / **857** / 172° | 477 / 302 / 116 | 2.54 · 5.78 g · 54 % |
| 10-18b · 1 | 14.5 s | 1342 / 1164 / 459° | -339 / **571** / 169° | 418 / 266 / 136 | 2.51 · 5.86 g · 49 % |
| 10-18b · 2 | 22.5 s | 2556 / 2227 / 712° | -568 / **1275** / 133° | 505 / 304 / 165 | 3.13 · 6.25 g · 71 % |
| 10-18b · 3 | 46.7 s | 4721 / 4343 / 1414° | -1138 / **2782** / 273° | 538 / 306 / 172 | 2.96 · 6.00 g · 65 % |

## 2. The reference and its validity (`n1/gps_vs_imu_load.csv`)

**Method** (`span_attitude.py`). For each GPS interval (≈ 100 ms; de-lagged by a fitted 0.125–0.175 s; both ends 3D fix, ≥ 5 sats),
the GPS specific force is f_GPS = Δv_GPS / Δt − g (NED). A candidate attitude R predicts R · f_IMU (accSmooth, averaged over the
interval). Over a window, the best earth-frame correction rotation (Wahba / SVD) is the source's attitude error. It is split into
**tilt** (about horizontal axes) and **heading** (about down). The 1 σ comes from the fit residuals and the window's geometry.
Windows with < 10 usable intervals give no estimate.

**Validity**: binned by IMU load (all intervals in the airborne interval):

| \|f_IMU\| | 11-08: GPS/IMU \|f\| ratio · residual | 10-18a | 10-18b |
|---|---|---|---|
| < 1.5 g | 1.21 · 4.5 m/s² | 1.15 · 3.2 | 1.09 · 2.7 |
| 1.5–2.0 g | **1.02** · 6.2 | **0.95** · 5.4 | **0.99** · 5.4 |
| 2.0–2.5 g | 0.92 · 9.4 | 0.91 · 9.0 | 0.96 · 6.4 |
| 2.5–3.5 g | 0.83 · 13.0 | 0.85 · 13.9 | 0.88 · 10.6 |
| 3.5–5.0 g | 0.74 · 20.4 | 0.78 · 21.0 | 0.74 · 19.0 |
| 5.0–9.0 g | 0.56 · 34.9 | 0.57 · 42.3 | 0.63 · 32.1 |

- Ratio > 1 at < 1.5 g is GPS differencing noise rectified into the magnitude. The ratio ≈ 1 band is 1.5–2 g.
- The ratio falls monotonically above 2 g on all three logs. That is the u-blox AIR_2G dynamic model smoothing the receiver's
  velocity. The residual (median |f_GPS − R f_IMU|, with INAV attitude) grows from 3–6 m/s² to 32–42 m/s².
- A linear receiver model (first-order low-pass + lag fitted to the in-span data) only reduces the in-span RMS from 23 to 15 m/s².
  The limiting is not linear, so it cannot be modelled away.
- ⇒ The reference uses only intervals with |f_IMU| ≤ 2 g (`--max-g`), over 4 s windows (`--window`). That excludes 50–83 % of
  each span. The per-interval residual of 3–6 m/s² at 1–2 g is 15–25° of direction noise, so σ per window is large.

## 3. Attitude per span (`n1/attitude_drift.csv`)

Errors in degrees vs the low-load GPS reference. "gyro vs INAV" is the direct disagreement between the two sources (no GPS) at
5 / 10 / 17 s / end ("—" = past the span end, or < 10 usable intervals). "pre" is INAV in the 4 s before engage (cruise / turns,
INAV accelerometer-aided). "whole span" is one fit over all low-load intervals of the span (n = intervals).

| flight · span | gyro vs INAV at 5 / 10 / 17 / end | pre: INAV tilt / heading | whole span: INAV | whole span: gyro-only | n |
|---|---|---|---|---|---|
| 11-08 · 1 | 14.2 / 6.4 / — / 12.5 | 9.5 ± 15.1 / -18.2 ± 19.7 | 48.2 ± 29.5 | 56.4 ± 29.6 | 27 |
| 11-08 · 2 | 14.6 / 11.4 / 5.3 / 1.6 | 9.2 ± 10.1 / +6.2 ± 14.5 | 19.0 ± 12.0 | 17.0 ± 12.0 | 65 |
| 11-08 · 3 | 0.9 / 1.0 / 12.7 / 6.6 | 1.3 ± 4.6 / **-8.6 ± 3.4** | 13.1 ± 12.0 | 13.4 ± 11.9 | 64 |
| 11-08 · 4 | 2.2 / 8.3 / 6.2 / 0.6 | 2.1 ± 2.2 / -6.9 ± 8.4 | 5.8 ± 9.5 | 5.3 ± 9.4 | 90 |
| 10-18a · 1 | 2.0 / — / — / 1.6 | 5.0 ± 5.7 / -15.3 ± 15.0 | (125.8 ± 26.5) | (108.8 ± 26.5) | 16 |
| 10-18a · 2 | 4.7 / 1.1 / — / 8.3 | 7.4 ± 5.4 / **-18.8 ± 4.6** | 7.7 ± 9.9 | 11.0 ± 9.9 | 53 |
| 10-18a · 3 | 10.2 / 1.8 / 6.7 / 3.4 | 7.7 ± 6.3 / -5.2 ± 8.2 | 22.0 ± 8.3 | 21.8 ± 8.4 | 96 |
| 10-18a · 4 | 2.2 / 8.2 / 8.4 / 1.8 | 5.2 ± 2.9 / **-10.3 ± 2.8** | 7.7 ± 6.7 | 7.7 ± 6.7 | 100 |
| 10-18b · 1 | 4.8 / 8.9 / — / 3.7 | 1.6 ± 2.5 / **-12.6 ± 3.0** | 7.6 ± 8.0 | 4.7 ± 8.0 | 73 |
| 10-18b · 2 | 3.1 / 7.6 / 5.9 / 11.1 | 5.1 ± 3.4 / -8.0 ± 8.2 | 16.1 ± 11.3 | 15.2 ± 11.3 | 69 |
| 10-18b · 3 | 4.8 / 1.7 / 4.7 / 11.4 | 3.5 ± 4.3 / **-12.8 ± 3.1** | 10.0 ± 6.2 | 7.6 ± 6.2 | 166 |

Reading it:
- **Attitude drift at 17 s (FR-006).** The direct gyro-vs-INAV disagreement at 17 s is 4.7–12.7° (7 spans reach 17 s). Against
  GPS, the 4 s windows at 17 s have σ 11–35° or too few low-load intervals (the CSV has every window). The logs do not say which
  source drifted.
- **INAV vs gyro-only under load (US3 AS2).** In 10 of the 11 whole-span fits |INAV − gyro-only| ≤ 8.2°, against σ 6–30°. In the
  11th, 10-18a span 1, it is 17° against σ 26.5°. No span resolves a difference beyond 1 σ. The direct disagreement (up to 14.6°) is real, but which side holds it cannot be decided with this
  reference. 10-18a span 1 (6.3 s, n = 16, the 8 g span) is an outlier: an unstable fit, in brackets.
- **Heading in cruise.** Where geometry and noise allow (σ ≤ 5°), INAV's heading is off by −8.6 to −18.8° (5 windows, ≥ 2.5 σ),
  while tilt is 1.3–7.4° and within 2 σ. Consistent with the 10–25° heading scatter of the 044 analysis and with R19. See headline 3
  for the declination caveat.
- **Floors in this method.** The 240 Hz blackbox logs a 25 Hz-filtered gyro, so gyro-only integration at 240 Hz is fine for
  filtered rates (aliasing is small below the 120 Hz Nyquist), but the raw gyro is aliased (R7). accSmooth is 15 Hz filtered.
  Accelerometer scale errors of 1–1.5 % (bench |a| 0.985–0.999 g) rotate the predicted direction by ≲ 1°. At 1:32 blackbox rate
  (62.5 Hz) on the re-fly, gyro-only integration will be coarser.

## 4. Velocity and position, span-local (`n1/nav_drift.csv`)

Integration of R · f_IMU + g from engage, starting at the de-lagged GPS velocity and position, with INAV's attitude
(`inav_att`) or gyro-only attitude (`gyro_att`). It is compared with GPS at the nearest low-load (≤ 2 g) GPS sample within ±1.5 s
of each time. `inav_est` is INAV's own estimate (navVel; navPos − navPos at engage) on the same samples.
Cells: |Δv| horizontal · |Δp| horizontal.

| flight · span | 10 s: inav_att | 10 s: gyro_att | 10 s: INAV estimate | end: inav_att | end: gyro_att | end: INAV estimate |
|---|---|---|---|---|---|---|
| 11-08 · 1 | 6.1 m/s · 74 m | 6.0 m/s · 68 m | **20.0 m/s · 80 m** (the freeze) | 11.9 m/s · 93 m | 14.3 m/s · 105 m | 2.3 m/s · 4 m |
| 11-08 · 2 | 5.3 m/s · 17 m | 2.1 m/s · 4 m | 6.0 m/s · 2 m | 5.2 m/s · 70 m | 5.7 m/s · 45 m | 2.5 m/s · 5 m |
| 11-08 · 3 | 11.0 m/s · 56 m | 11.8 m/s · 58 m | 1.8 m/s · 5 m | 19.3 m/s · 195 m | 18.9 m/s · 230 m | 4.8 m/s · 4 m |
| 11-08 · 4 | 7.6 m/s · 44 m | 5.2 m/s · 36 m | 0.4 m/s · 5 m | 12.1 m/s · 153 m | 10.2 m/s · 122 m | 6.3 m/s · 4 m |
| 10-18a · 1 | — | — | — | 5.1 m/s · 18 m | 1.5 m/s · 3 m | 2.3 m/s · 3 m |
| 10-18a · 2 | 7.8 m/s · 40 m | 9.5 m/s · 46 m | 5.5 m/s · 5 m | 12.5 m/s · 98 m | 10.3 m/s · 108 m | 3.0 m/s · 3 m |
| 10-18a · 3 | 26.0 m/s · 146 m | 21.1 m/s · 114 m | 2.0 m/s · 5 m | 52.8 m/s · 636 m | 43.6 m/s · 510 m | 1.6 m/s · 2 m |
| 10-18a · 4 | 14.7 m/s · 71 m | 14.5 m/s · 68 m | 0.8 m/s · 4 m | 25.4 m/s · 260 m | 28.9 m/s · 271 m | 0.9 m/s · 6 m |
| 10-18b · 1 | 11.4 m/s · 36 m | 9.5 m/s · 32 m | 2.2 m/s · 6 m | 10.1 m/s · 73 m | 12.1 m/s · 73 m | 2.0 m/s · 1 m |
| 10-18b · 2 | 4.6 m/s · 40 m | 7.1 m/s · 53 m | 3.5 m/s · 3 m | 17.5 m/s · 174 m | 22.7 m/s · 222 m | 1.8 m/s · 6 m |
| 10-18b · 3 | 4.8 m/s · 43 m | 7.1 m/s · 53 m | 3.2 m/s · 5 m | — (no low-load GPS sample) | — | — |

- Both attitude sources drift alike. A mean acceleration error of 0.5–2.5 m/s² over a span explains these numbers. That equals
  1–5° of attitude error at 3 g, or **an accelerometer scale error of ~1.5 % at 3 g** (≈ 0.45 m/s² ⇒ ~9 m/s over 20 s; bench |a|
  0.985 g). So un-aided velocity is metres-per-second wrong within a span even with perfect attitude, and position is hopeless.
- Vertical errors are smaller (CSV). One cell is inconsistent: 10-18a span 1's 41 m vertical position error with a 0.3 m/s
  vertical velocity error points at a GPS altitude glitch, not drift.
- GPS truth has its own error: EPH 1.7–5.2 m in these samples, velocity σ ~0.2–0.5 m/s at low load.

## 5. INAV estimator faults (`n1/faults.csv`)

From the blackbox, per engaged span, resampled to the xiao's 20 Hz tick (`nav_span_scan.py --blackbox`; definitions in
data-model.md):

| flight · span | fault | time in span | magnitude |
|---|---|---|---|
| 11-08 · 1 | LOW_SATS (5 sats, 3D fix kept) | 1.90–3.15 s, 5.40–9.90 s, 10.30–10.40 s | min 5 |
| 11-08 · 1 | INVALID_XY (EPH ≥ cap) + UNTRUSTED_XY | 7.70–10.05 s (2.35 s) | EPH 10.0 m (the cap) |
| 11-08 · 1 | FROZEN | 7.75–10.05 s (2.30 s) | — |
| 11-08 · 1 | STEP_XY (reset on reacquire) | 10.05 s | **106.3 m** |
| 11-08 · 1 | STEP_Z | 15.90 s | **25.4 m** |
| 10-18a · 1 | STEP_Z ×3 (altitude flagged valid throughout) | 3.05, 3.70, 4.60 s | **82.6, 101.0, 52.8 m** |
| all other spans (9) | none | — | — |

- No NO_FIX and no GPS_GAP (update interval > 0.2 s) in any span. The receiver kept reporting.
- 10-18a span 1's vertical steps happened with `navFlags` altitude "trusted" and no satellite loss. Validity bits do not catch
  them; the step detector does (R5's baro/GPS altitude fade).
- The xiao-CSV mode on 11-08 finds the same faults: 46 frozen ticks and a 105 m step, plus a 25.7 m Z step. Its span clock sits
  ~0.7 s off the blackbox's because the xiao span starts before INAV's override flag sets.
- **Clean-tick residual distribution** (`n1/nav_residuals.csv`; 4523 ticks, faults and ±1 s around them excluded):
  horizontal p50 0.14 / p99 0.54 / p99.9 0.74 / max **1.00 m**; vertical p50 0.07 / p99 0.54 / p99.9 1.38 / max **1.59 m** per
  50 ms tick.

## 6. Attitude-stability budget (FR-006a; details and datasheet terms in research.md R4)

Over a 17–22 s span with 860–1350° net pitch (2782° in the 47 s 10-18b span 3), ~2000° gross on the other axes and mean |f|
2.4–3.9 g:

| candidate source | dominant terms | < 1° over a span? |
|---|---|---|
| **INAV AHRS** (MPU6000 + mag + GPS course; accelerometer ignored above 15 °/s) | gyro terms as below + heading pulled by mag / GPS course; measured cruise heading offsets −5…−19° | **No.** Not shown better than gyro-only under load (§ 3) |
| **un-aided gyro, MPU6000** (article 1) | calibrated scale ~0.3 % → 3–4°; g-sensitivity 0.1 °/s/g → ~6°; cross-axis and bias ~2° each | **No** (~7–8° RSS even calibrated) |
| **un-aided gyro, ICM42688P** (article 2) | calibrated scale ~0.1 % → ~1.1–1.5°; cross-axis ~2° after calibration; bias ~0.4°; g-sensitivity not specified | **No** (~2.5–4°) |
| **un-aided gyro, LSM6DS3TR-C** (xiao) | values partly unconfirmed (R4); calibrated scale ~0.1 % → 1–1.4°; bias drift 2–6° | **No** (~3–7°) |
| **xiao gyro + GNSS-velocity aiding** (N4 design) | aiding resolves attitude through specific force. On this receiver: per-interval residual 3–6 m/s² at ≤ 2 g (15–25° direction noise), and invalid above ~2 g (AIR_2G) | **Not demonstrated.** Estimated floor ~2–3° over a span (≈ 20° / √(60–75 low-load intervals)) on AIR_2G; better with AIR_4G + multi-GNSS (re-measure on the re-fly). Updates must be gated above the receiver's dynamic limit |
| **on-chip fusion part** (game-rotation-vector engines) | gravity / mag aided: the same failure under sustained load | **No** (argument, not measured) |

**Target revisit (research R4 decision 4)**: the < 1° target is not reachable with the hardware and receiver as flown. Working
target for the companion frame: **smooth (no internal resets) and ≲ 3°**, with 1° kept as the design aim. Re-evaluate after the
re-fly, using the same scripts on its logs (AIR_4G, multi-GNSS, the new blackbox rate).

## Pinned values (T029)

- **FROZEN**: horizontal velocity exactly 0 and horizontal position unchanged vs the previous tick, for **≥ 2 consecutive ticks**
  (0.1 s). In 4614 engaged ticks across 11 spans, the only occurrence is the 11-08 span-1 freeze (2.30 s).
- **STEP_XY = 5 m per 50 ms tick** (clean max 1.00 m, p99.9 0.74 m; the real fault 106 m).
- **STEP_Z = 5 m per tick**, raised from the provisional 3 m: the clean max is 1.59 m (p99.9 1.38 m), so 3 m left < 2× margin.
  The real faults are 25–101 m.
- **`inav_max_eph_epv` = 3000 cm** (the top of research R5's 2000–3000 cm range). By the measured eᵗ growth from 4.3 m after the
  1.5 s GPS timeout, a 30 m cap would have shortened the 11-08 freeze from 2.3 s to ~1.1 s. A 20 m cap gives ~1.5 s. **No
  sensible cap covers the observed 4.5 s outage** (it needs ~90 m). The reset on reacquire remains either way (R5). A higher cap
  trades a velocity freeze, a discontinuity the policy sees, for smooth drift while "valid". RTH is not the escape (R8), and N3
  logs EPH regardless. The direct levers for the observed event are **more satellites** (multi-GNSS, applied 11-11) and, as an
  operator decision, **`gps_min_sats` 6 → 5**. The receiver held a 3D fix with 5 satellites and GPS EPH 4.1–5.4 m throughout, so
  INAV, not the receiver, discarded GPS. That is class C, evidence here.

## What the logs could not show (recorded, not estimated)

- Attitude drift of either source to better than ~10° inside the spans (§ 3). Truth under load does not exist in these logs.
- The xiao's own IMU (not logged before N4), the mount resonances (aliased at 240 Hz; R7), and the in-flight gyro scale factor
  (needs the bench turn test, checklist C5).
- Whether the cruise heading offset is the compass, the declination handling or a reference-convention sign. That needs the
  re-fly's box segment.

## Reproduce

From the repo root, with the blackbox decodes present (`.01` decodes as in MANIFEST § Preservation; the 10-18 second log needs
`~/blackbox-tools/obj/blackbox_decode --index 2 flight-results/flight-20261018/blackbox_log_2026-10-18_125223.TXT`):

```bash
python3 specs/045-m2-tracking/span_attitude.py              # § 1–3: attitude_drift.csv, span_envelope.csv, gps_vs_imu_load.csv
python3 specs/045-m2-tracking/nav_dead_reckon.py --span     # § 4: nav_drift.csv
python3 specs/045-m2-tracking/nav_span_scan.py --blackbox   # § 5: faults.csv, nav_residuals.csv
python3 specs/045-m2-tracking/nav_span_scan.py flight-results/flight-20261108/flight_log_2026-11-09T00-14-08_flight_001.csv  # xiao cross-check
```

All three default to every known flight (`--flight 1108 | 1018a | 1018b` to select) and write to `specs/045-m2-tracking/n1/`.
Options: `--window` / `--max-g` (reference), `--step-xy` / `--step-z` / `--eph-cap-cm` (fault thresholds; INVALID = EPH/EPV at or above the cap, UNTRUSTED = INAV's own navFlags). US5's per-span flag summary and `flight_gates.py --drop` arguments: `nav_span_scan.py --blackbox [--bb LOG --xiao CSV] [--eph-cap-cm 3000 for the re-fly]`; test: `python3 specs/045-m2-tracking/test_nav_flags.py`.

# Flight analysis 2026-11-08 — t1 g800, the 044 acceptance flight (T053 / T054)

Genome t1 g800 (`a5ed3521ea1e70b7`, created gen 730, unrolled build), article 1, FC `INAV 8.0.0 (38ff0d29)` with the config of record
(`xiao/inav-hb1.cfg` @ 91abd4d), blackbox `1/8`, rates `36,24,4`. One flight: xiao `flight_log_2026-11-09T00-14-08_flight_001.bin`
(4 engaged spans, 1635 ticks, 82 s; paths 0 StraightAndLevel, 2, 4, 1; 0 overruns / gaps / drops) + blackbox
`blackbox_log_2026-11-08_160651` (3:15, 250 Hz). Operator: *"fairly decent, a bit less bang-bang … we did seem to move bang-bang to
roll"*. Streamer full as on 10-18, light to no wind, no configuration change (operator, § 4).

Reference: the two 10-18 t6 g752 flights (7 spans), recomputed by the same code. Scripts: [`flight_gates.py`](flight_gates.py) (xiao
ticks, 20 Hz, the criteria), [`bench_join.py`](bench_join.py) (xiao ↔ blackbox), [`../043-acro-dual-loop/airframe_id.py`](../043-acro-dual-loop/airframe_id.py)
(airframe FRF), and a 250 Hz blackbox cross-check of the bands (§ 2).

## 1. The criteria (medians over engaged spans)

The 10-18 reference numbers in spec.md mix two forms of "band RMS": the rate bands (97 / 56 / 51 °/s) reproduce with the **raw** rate
RMS (a turn's steady pitch rate included — 043's `candidate_compare` form), the load band (1.88 g) only **mean-removed** (else gravity
counts as oscillation). `flight_gates.py` prints both and judges each criterion two ways: against the spec's absolute line, and against
10-18 recomputed by the same code (≥ 15 % below — the spec's own rule, robust to the form).

| criterion | 10-18 (same code) | **11-08** | absolute line | vs 10-18 | verdict |
|---|---|---|---|---|---|
| **SC-001** roll 3–5 Hz RMS | 101 °/s (99 mean-removed) | **41** | < 82 ✅ | **−60 %** ✅ | **PASS** |
| **SC-002** pitch 3–5 Hz RMS | 56.6 (49.7) | **50.4** (43.9) | < 48 ❌ (✅ mean-removed) | −11 % ❌ (need −15 %) | **MISS** |
| **SC-002** pitch 2–3 Hz RMS | 52.0 (45.7) | **70.2** (63.5) | ≤ 51 ❌ | **+35 %** ❌ | **MISS** |
| **SC-003** median distance to rabbit | 12.2 m | **9.1 m** | within 9–14 m ✅ | better | **PASS** |
| **SC-004** roll corner (flight FRF) | 5.05 Hz | **2.97 Hz** | inside the class (1.7–5.8 Hz by the flight method, centre 3.24) ✅ | — | **PASS** |
| **SC-005** 1.5–5 Hz load RMS (mean-removed) | 1.91 g | **1.90 g** | < 1.88 ❌ by 0.02 | −0.7 % (no rise) | **marginal** |
| total \|a_z\| p99 (not gated) | 7.0 g | 6.8 g | — | | |

**Verdict: SC-001 passes decisively; SC-002 misses — the 2–3 Hz pitch band rose 35 %.** Per clarification 2 a miss on SC-002 opens
another bake / flight cycle, cause analysed first; 045 waits. The miss sits where `gate_estimate.py` flagged the risk (g800 predicted
54 °/s at 2–3 Hz vs the ≤ 51 line) but larger: 70.

Per span (xiao): spans 2–4 are uniform (roll 3–5 Hz 33–42, pitch 2–3 Hz 70–76); span 1 (path 0, the entry span) is the outlier on
roll and pitch 3–5 Hz (84 / 83) — 10-18's first span was its outlier too.

> **Addendum 2026-11-09**: span 1 was flown on a **faulted nav estimate** (GPS dropped below `gps_min_sats`, horizontal estimate
> coasted, froze, then reset 105 m; altitude hard-reset 26 m near the end) — § 6. Without span 1 (§ 7): SC-001 / 003 still pass,
> **SC-005 now passes** (1.79 g), pitch 3–5 Hz passes in the mean-removed form only, and **SC-002 2–3 Hz still misses (+36 %)**. The
> verdict on SC-002 does not change.

## 2. Where the oscillation went

Band RMS of the gyro, xiao (20 Hz), median over spans, °/s:

| band Hz | 0.25–1 | 1–1.5 | 1.5–2 | 2–2.5 | 2.5–3 | 3–4 | 4–5 | 5–7 |
|---|---|---|---|---|---|---|---|---|
| roll 10-18 | 54 | 22 | 22 | 28 | 51 | 63 | 70 | 41 |
| **roll 11-08** | 65 | 38 | 34 | 34 | 34 | 32 | 25 | 20 |
| pitch 10-18 | 43 | 28 | 36 | 35 | 30 | 36 | 31 | 18 |
| **pitch 11-08** | 38 | 31 | 37 | 46 | 44 | 39 | 20 | 15 |

250 Hz blackbox, same windows (6 / 4 engaged segments, raw form): roll 3–5 Hz 95 → **42**, pitch 3–5 Hz 55.5 → **53.9** (−3 %),
pitch 2–3 Hz 53 → **69** (+30 %), roll 1–2 Hz 30 → **52**, roll 5–10 Hz 46 → 23. The xiao and blackbox agree; the blackbox is harsher
on pitch 3–5 Hz (−3 % vs −11 %).

- **Roll**: the 3–5 Hz dither the old plant could not see is gone (−56 to −60 %), and roll content moved **down** to 0.25–2 Hz (+20 to
  +70 %) — larger, slower roll swings, which read as more roll activity from the ground. This is the "moved to roll" impression.
- **Pitch**: content moved **down** from 4–5 Hz into **2–3 Hz** — the band where this airframe's short period sits (§ 3).

**The commands do not show bang-bang moving to roll.** The roll command is calmer on every measure: tick-to-tick |Δu| 0.39 → 0.25,
time at the cap 3.3 % → 0.1 %, 3–5 Hz 0.30 → 0.15, sign flips 7.3 → 5.8 /s. The pitch command is busier (|Δu| 0.18 → 0.25, flips
3.8 → 4.3 /s, 1–3 Hz up), and throttle chatters more at 5–10 Hz (|Δu| 0.26 → 0.42) while sitting at the cap less (88 → 78 %).

## 3. The airframe was less damped in pitch than on 10-18

`airframe_id.py`, same code on both days (airframe FRF through the setpoint; coherence 0.88–0.97):

| | 10-18 (full streamer) | **11-08** |
|---|---|---|
| pitch \|H\| peak (normalized at 1 Hz) | 1.61 at 1.95–2.44 Hz | **2.21 at 2.93 Hz** |
| fit (b), `I_yy` 0.0094: `Cm_q` / ζ | **−3.54 / 0.55** | **−1.11 / 0.38** |
| roll first-order corner | 5.05 Hz | **2.97 Hz** |

`Cm_q` −1.1 is the **bare** airframe's model value; −3.5 was 10-18's streamer-on value (043 T119). So on 11-08 the aircraft amplified
2–3 Hz pitch about 37 % more than on 10-18, with the same rate loop. That alone is about the size of the 2–3 Hz rise (+30 to +35 %);
the policy cannot be judged on SC-002 until the configuration is known. The `CraftCmQ` class spans −3.5 … −1.0, so 11-08 sits at the
bare edge of what g800 trained on. The roll corner also halved (5.05 → 2.97 Hz) on the same airframe — not explained yet (a streamer
should add little roll damping; CG, battery or wing changes would). The 11-08 corner happens to sit on the class centre (3.24 Hz).

## 4. Operator answers (2026-11-08) and what they mean

**Same streamer, light to no wind, nothing changed** — except that the rubber bands holding the wing on may be oxidized. So the
configuration of record was the same as 10-18, yet the airframe fit (§ 3) shows bare-level pitch damping and a roll corner halved:
**something physical changed**, and a compliant wing mount (wing moving relative to the fuselage-mounted FC) is a plausible cause of
both. Not measured. SC-002 therefore stands as a **miss with an unresolved cause** (airframe vs policy). Operator decision: close 044,
carry SC-002 to the backlog — re-fly g800 on fresh bands with an airframe re-ID before any corrective bake ([outcome.md](outcome.md) § 4).

## 5. Join and loop health

`bench_join.py`: clock map −1034 ppm (10 anchors); every tick matched within 2.2 ms; position xiao vs `navPos` p50 0.26 m. **100 %
pure ACRO** on every engaged tick (no priming-window ticks — the hand-off held), yaw `axisRate[2]` exactly 0. Authority at the best
lag: `out_pitch → axisRate[1]` r −0.95…−0.97, −227…−234 °/s/unit (nominal −240); `out_roll → axisRate[0]` +0.96…+0.97, +328…+336
(nominal +360). Pipeline p50 / p95 / max 12.4 / 21.6 / 42 ms; eval 2.95 ms (unrolled).

⚠️ The pass-through correlation peaks with the xiao clock shifted **+60–70 ms** (bench: +60). That is **not** a measured transport
delay: the tick timestamp is taken after the fetch and every `INAV_CLOCK` anchor carries the INAV time of the *last* fetch (up to a
tick stale), so the clock map itself is biased by up to a tick. The sim models 10 ms (`COMPUTE_LATENCY_MSEC_DEFAULT`). A clean
measurement needs the INAV sample time per tick — filed in BACKLOG.

## 6. Span 1 was flown on a faulted nav estimate (added 2026-11-09)

The xiao log shows two one-tick discontinuities in span 1 (path 0): at tick 215 → 216 (10.78 s) the position jumps **+105.0 m N,
−19.8 m E** and horizontal velocity goes from exactly 0 to 21.9 m/s; at tick 332 → 333 (16.63 s) `pos_d` jumps **+25.7 m** (lower) and
`vel_d` −6.4 → +17.5 m/s. Spans 2–4 have no step over 0.7 m. Neither is a logging fault: the xiao recorded what `MSP2_AUTOC_STATE`
delivered, and the blackbox (`navPos` / `navVel` / `navEPH` / `navEPV`, G frames at 10 Hz) shows INAV's position estimator doing it.
Clock map: span-1 `INAV_CLOCK` anchors, INAV ≈ xiao + 375–390 ms.

**Horizontal — GPS below `gps_min_sats`, then a hard reset.** GPS was UBLOX, GPS constellation only (Galileo / GLONASS / BeiDou OFF),
5–7 sats all flight (G-frame histogram 5: 70, 6: 1432, 7: 538), HDOP 2.5–3.5, GPS EPH 3.3–5.5 m, EPV 6–8 m in span 1.

| INAV s | xiao span-1 tick | event |
|---|---|---|
| 378.47 | ~122 (6.1 s) | sats 6 → 5 < `gps_min_sats = 6` in a steep climb and direction reversal: GPS unusable, the estimate coasts on the accelerometer and diverges — `navVel` N −3…−7 m/s while GPS shows +19…+21 |
| 380.76 | 170 (8.5 s) | `navEPH` reaches `inav_max_eph_epv` (1000 cm), `navFlags` 37 → 33: XY invalid; `inav_allow_dead_reckoning = OFF` ⇒ position frozen, velocity 0 for **2.32 s** |
| 382.97 | — | sats back to 6, GPS usable again |
| 383.11 | 216 (10.8 s) | estimate invalid + GPS valid ⇒ **reset to the GPS position and velocity** (`navigation_pos_estimator.c` `estimationCalculateCorrection_XY_GPS`): the 105 m step |

The GPS itself never dropped out — 3D fix every frame, continuous track; it was below the satellite count INAV requires.

**Vertical — the GPS↔baro fade, then a hard reset.** With `inav_default_alt_sensor = GPS`, INAV fades the baro out as |GPS − baro|
grows from 10 to 20 m (`max_eph_epv` → 2×); GPS altitude wandered −10…+27 m from the estimate over this flight. With the baro faded,
`navEPV` climbed (311 cm at 383 s → 998 at 388.93 s) until the estimate went invalid and was **hard-reset**: 388.93 s `navPos[2]`
91.0 → 65.4 m (the xiao's tick 333), and again after disengage at 390.31 s, 77.0 → 107.4 m (xiao breadcrumbs). Both times `navEPV`
reset to the GPS EPV (772 / 754 cm ≈ G-frame 770 / 755), i.e. the GPS-reset branch; the reset altitudes do not line up with the logged
`GPS_altitude` at one offset, so the target value is not fully explained.

**Why the coast went wrong: attitude under load, not the accelerometer.** No accel clipping all flight (`debug_mode` VIBE,
`debug[3]` 0), and no fixed misalignment: rotating `accSmooth` to the earth frame with INAV's `attitude[]` and fitting it against
GPS-derived acceleration gives a best heading offset of 0° (median, moderate-attitude windows). But the 378.5–380.8 s window averaged
**4.4 g** (peak 6.6), and the IMU-integrated velocity change came out ~85° rotated from GPS (+14 m/s east vs +23 m/s north). Over the
flight, 2.3 s dead-reckoning error grows with manoeuvre severity (p50 2.9 → 11 m/s as max pitch goes < 30° → > 70°; best-fit offset
scatter ±10–25°). That points at attitude/heading error in sustained pulls (`ahrs_acc_ignore_rate 15` ⇒ gyro + mag only, QMC5883
under-calibrated); the GPS reference is itself a 5–7-sat, 10 Hz velocity, so this is indirect. INAV then decays velocity toward zero
without GPS (`inav_w_xy_res_v` 0.5/s ÷ accWeight). Z vibration holds accWeight at its 0.30 floor in every span, so that decay runs
~1.7/s; replaying that rule reproduces the logged `navVel` within ~2–3 m/s. Every satellite drop in the flight followed a 2.7–3.9 g
mean (peaks 5–7.5 g) with `gps_dyn_model = AIR_2G`. Routed to the
[045 navigation pre-task](../045-m2-tracking/README.md#-pre-task--navigation--attitude-integrity-filed-2026-11-09).

**Scope and consequences.**
- The only XY-invalid interval in the flight and the only two Z resets; both inside span 1 (or just after it).
- The policy flew **ticks ~122–215 (≈ 4.7 s of 17)** on a diverging, then frozen state, was handed the 105 m step, and the last 8 ticks
  after the Z step. `state_valid` was 1 throughout: it means *the MSP fetch succeeded*. `MSP2_AUTOC_STATE` carries no estimator
  validity (no EPH / EPV / nav flags), so the xiao cannot tell — nothing disengaged.
- The airframe FRF (§ 3) is setpoint → gyro and does not read the nav state; it stands.
- Same family on 10-18: flight 1 span 1 (141 ticks) has three vertical-only steps, −82 / +101 / −52 m at 3.8 / 4.4 / 5.4 s — not
  investigated further; dropping it leaves the 10-18 medians unchanged.
- This is the BACKLOG's standing `GPS_ONLY` + Galileo item (043 close) and the "estimator fault" row of the NN-pathology table
  (09-05 / 09-13) recurring; routed in BACKLOG § 044 deferrals.

## 7. The gates without span 1

`flight_gates.py ... --drop x1108.csv#1` (reference unchanged; dropping the 10-18 first spans as well leaves every reference median
identical):

| criterion | 10-18 | 11-08 all spans | **11-08 without span 1** | verdict without span 1 |
|---|---|---|---|---|
| SC-001 roll 3–5 Hz | 101 | 41 | **40.0** (−60 %) | **PASS** |
| SC-002 pitch 3–5 Hz, raw (mean-removed) | 56.6 (49.7) | 50.4 (43.9) | **50.4 (42.1)** | raw −11 % **MISS**; mean-removed −15.4 % and < 48 PASS |
| SC-002 pitch 2–3 Hz, raw (mean-removed) | 52.0 (45.7) | 70.2 (63.5) | **70.7 (63.8)** | +36 % **MISS** |
| SC-003 median distance | 12.2 m | 9.1 m | **8.6 m** | **PASS** (no regression) |
| SC-005 1.5–5 Hz load, mean-removed | 1.91 g | 1.90 g | **1.79 g** (−6.6 %) | **PASS** (< 1.88) |

Span 1 was carrying SC-005 over the line and part of pitch 3–5 Hz; it was **not** carrying the 2–3 Hz pitch rise — spans 2–4 are each
70–76 °/s on their own. The SC-002 open gate (airframe vs policy, BACKLOG) is unchanged.

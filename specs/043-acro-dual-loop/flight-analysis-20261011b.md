# 043-t5 — fifth ACRO flight (2026-10-11 afternoon): same genome, NAV_POS back, estimator read directly

Second flight of the day on the same article as [`flight-analysis-20261011.md`](flight-analysis-20261011.md)
(the "morning flight" below): **t5 gen 512**, `weight_id=1731d937e3af9c1c`, `firmware_id=c3eb28e6b938dd1a`,
program `…2026-10-09T01:04:08.995Z/gen9512.dmp.zst`. ⚠️ Provenance unchanged — no `.dat` for `1731d937…`
in `artifacts-t5/` yet (MANIFEST § 5h step 1 is still open).

Logs: `flight-results/flight-20261011/blackbox_log_2026-10-11_143538.TXT` and
`flight_log_2026-10-11T21-42-19_flight_002.bin`. **3 spans, 1585 ticks, 0 gaps, 0 overruns, 0 resyncs,
0 drops; all three ended by path completion.** Blackbox at **239 Hz** (`P interval 1/8`), 169 s, 3 of
~40 500 iterations unreadable (0.00 %). Calm again: INAV wind 2.3–2.9 m/s per span (vector median
0.9–1.5 m/s). GPS 9–11 satellites (morning: 7).

⭐ **The § 6 blackbox config from the morning doc flew**: `NAV_POS` is back (`navPos/navVel/navEPH/navEPV/
navTgt*`), `GYRO_RAW` and `debug_mode = VIBE` kept, rate 1/8. The flash was **not** erased — both logs are in
it (log 1 = the morning flight, 9.76 MB; log 2 = this one, 3.37 MB). Decode with `--index 2`.

⭐ **Headline: tracking matches or beats the morning (median error 8–15 m, span 2 in-streak 25 %, the highest
of any ACRO span), the ~2 Hz pitch cycle is unchanged for the fifth flight, and the vertical estimator is now
read directly — `navEPV` peaked at 541 cm against the 1000 cm cap, the 20/20 jump marker reads CLEAN for the
first time on an engaged ACRO flight.** Two corrections to the record below: the 16-tick engage dead-time
(§ 5) and a second 10× unit slip in two helper scripts (§ 2).

---

## 0. Spans

xiao `INAV_CLOCK` anchors (xiao + 596 → 447 ms across the flight, ~0.1 % drift; interpolate):

| span | INAV t (s) | ticks | path | override applied (bb flag 3→2) | disengage |
|---|---|---:|---|---|---|
| 1 | 167.58 – 184.62 | 341 | 0 StraightAndLevel | 168.32 (+0.74 s) | reason 5 — path complete |
| 2 | 200.64 – 222.57 | 439 | 2 HorizontalFigureEight | 201.37 (+0.73 s) | reason 5 — path complete |
| 3 | 236.81 – 277.01 | 805 | 5 SeededRandomB | 237.55 (+0.74 s) | reason 5 — path complete |

Fig-8 and RandomB are the two paths the morning flight did not fly; with the morning's four this genome has now
flown all six of the t5 regiment. The "override applied" column is the subject of § 5.

---

## 1. ⭐ Tracking — level with the morning on the easy paths, first high in-streak span

| span | path | trk err med / p90 / max (m) | `step_score` mean | in-streak (≥ 0.5) | thr +rail | thr mean | `motor > 1900` | pitch −rail | gnd spd med |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | Straight | **8.1** / 26.7 / 30.1 | **0.286** | 9.7 % | 42 % | +0.36 | 65 % | 9.1 % (11 ticks applied at entry, § 5) | 12.5 |
| 2 | Fig-8 | **9.6** / 16.0 / 21.8 | **0.344** | **25.5 %** | 47 % | +0.65 | 76 % | 9.3 % | 13.7 |
| 3 | RandomB | 14.9 / 28.7 / 40.7 | 0.163 | 2.6 % | 54 % | +0.79 | 86 % | 4.7 % | 12.8 |
| *morning* | Str / Spiral / Perch / randA | 13.2 / 10.2 / 15.2 / 14.5 med | 0.225 / 0.259 / 0.204 / 0.176 | 9 / 5 / 4 / 5 % | 46–59 % | +0.54…+0.66 | 74–79 % | 2–11 % | 12.7–13.6 |
| *t3 09-13* | 0 / 2 / 3 | 33.6 / 24.5 / 31.8 med | 0.064 / 0.060 / 0.078 | — | 85–90 % | +0.82…+0.87 | 90.5 % | 0.0–0.3 % | 9–16 |

- **Straight-line median error 8.1 m is the best single span of the five ACRO flights** (morning 13.2 m on the
  same path, same genome, same day) — the spread between the two is the day-to-day / entry noise floor on this
  genome, so cross-flight deltas under ~5 m should not be read as genome effects.
- **Fig-8 at 0.344 / 25.5 % in-streak** is the first span of any ACRO flight to spend a quarter of its ticks
  inside the streak threshold. p90 error 16 m, max 21.8 m — it never got far from the rabbit.
- **RandomB cold start** is the hardest path in the sim too: g422 scores 0.25 / 28.1 % on it (MANIFEST § 5g).
  Flight / sim ratio per path: Straight **0.29** (0.286 vs 0.98), Fig-8 **0.44** (0.344 vs 0.78), RandomB
  **0.65** (0.163 vs 0.25). The sim-to-real gap narrows as the path gets harder — the easy paths are where the
  sim overstates the policy most, which is consistent with the gap living in the dynamics (§ 2) rather than in
  path geometry.
- Roll rail **0.0 %** in all three spans. Throttle rail 42–54 % (morning 46–59, t3 85–90); throttle sign
  switching 2.8 / 5.8 / 2.6 /s.
- **Downwind bias**: craft − rabbit on INAV's wind axis **+4.5 / +0.4 / −1.3 m** mean (median +0.1 / 0.0 /
  −1.0) — [`f1011b_wind_bias.py`](f1011b_wind_bias.py). Zero, as in the morning, on a second calm flight.
  The windy-day test is still owed.

---

## 2. ⛔ The pitch cycle — fifth flight, unchanged; and a second unit slip

Same method as the morning: block-average to 60 Hz ([`f1011_bb_decimate.py`](f1011_bb_decimate.py), factor
**4** at 239 Hz) then [`pitch_spectrum.py`](pitch_spectrum.py) `--manual`.

| | **afternoon** spans 1 / 2 / 3 | **morning** 1 / 2 / 3 / 4 | **t3 (09-13)** | **041-t7** |
|---|---|---|---|---|
| pitch peak | **2.34 / 1.99 / 2.22 Hz** | 2.01 / 2.01 / 1.77 / 2.01 | 2.32 / — / 2.09 | 2.21 / 2.09 / 2.21 / 1.98 |
| power in 2–3 Hz | 36.5 / 31.3 / 47.3 % | 63.5 / 50.7 / 44.8 / 39.3 | 56.2 / 30.4 / 74.6 | 53.5 / 69.3 / 44.8 / 37.0 |
| power in 1–2 Hz | 28.7 / 49.4 / 35.9 % | 21.0 / 39.7 / 40.5 / 35.5 | — | — |
| gyro pitch RMS | **124 / 153 / 140 °/s** | 147 / 145 / 131 / 122 | 127 / 133 / 133 | 123 / 139 / 110 / 115 |
| loop gain at peak | 1.45 / 1.37 / 1.42 | 1.52 / 1.45 / 1.44 / 1.45 | 1.71 / 1.68 / 1.55 | n/a |
| phase at peak | −54 / −47 / −51° (−64 / −65 / −64 ms) | — | — | — |
| MANUAL stretches, 2–3 Hz | **3.0 / 3.0 / 13.7 / 4.2 %** | 3.3–8.2 % | 7.5–16.4 % | — |

Same amplitude band (±200 °/s peak, RMS 124–153 °/s), same gain 1.4–1.5, present only with the policy
engaged. The tone is more smeared across 1–3 Hz than on t3 (the 1–2 Hz share is now equal to or larger
than the 2–3 Hz share in spans 1–2), i.e. the cycle is less of a pure resonance and more of a broad
bang-bang, but its energy has not moved. The −64 ms phase lag between setpoint and gyro at the peak is
constant across all three spans and is the ZOH + actuator figure the 043 spec budgeted.

Rate loop ([`f1011_bb_rate_loop.py`](f1011_bb_rate_loop.py), **units corrected, see below**): pitch setpoint
median **−138 / −129 / −114 °/s** with p5 / p95 at **−198 / +237 °/s** (the 240 °/s `rates` cap), gyro median
−40 / −51 / −64 °/s, p5 / p95 ±205–256 °/s; PID sum ≥ 500 on 6.0 / 11.8 / 5.7 % of samples; feed-forward
share of |output| 1.01 / 0.99 / 0.97. Pilot pre-engage: setpoint p5 / p95 −115 / +26 °/s. The NN spends
its median tick commanding ~130 °/s nose-up — the bank-and-pull regime — and the rate loop is pure FF.

⛔ **Second units correction.** The morning commit fixed `pitch_spectrum.py` but
[`f1011_bb_span_stats.py`](f1011_bb_span_stats.py), [`f1011_bb_rate_loop.py`](f1011_bb_rate_loop.py) and
[`f1011_bb_hr_spectrum.py`](f1011_bb_hr_spectrum.py) still divided gyro / setpoint by 10. Consequences in
the morning doc: § 2's "setpoint median −14 °/s, gyro median −4 to −7 °/s" are **−140 and −40 to −70 °/s**;
§ 3's `gyroADC[1] > 20 Hz RMS` "0.19–0.21 °/s" is **1.9–2.1 °/s**. The gyro-RMS row of its § 2 table came
from `pitch_spectrum.py` and was already right. All three scripts are fixed in this commit; the morning
doc carries a one-line erratum pointing here. Band shares and gains were never affected.

Roll: `out_roll` 5–10 Hz share **16.6 / 19.9 / 19.6 %** (morning 13–20 %, t3 30–37 %). Holds.

---

## 3. Above 20 Hz at 1/8 rate — the expected aliasing, nothing else

[`f1011b_bb_hr_spectrum.py`](f1011b_bb_hr_spectrum.py) (fs-agnostic successor; NSEG 2048 = 0.117 Hz bins at
239 Hz, gyro in deg/s):

| span | `gyroRaw[1]` 20–50 Hz | 50–120 Hz | observed line | INAV FFT peak (`gyroPeakPitch[2]`) → alias | `gyroADC[1]` > 20 Hz RMS | `accSmooth[2]` > 20 Hz |
|---|---:|---:|---|---|---:|---:|
| 1 | 0.5 % | 11.0 % | 92 Hz | 376 Hz → 102 Hz | 1.90 °/s | 0.03 g |
| 2 | 0.4 % | 9.7 % | 104 Hz | 379 Hz → 99 Hz | 2.05 °/s | 0.03 g |
| 3 | 0.4 % | 10.3 % | 110 Hz | 380 Hz → 98 Hz | 2.04 °/s | 0.04 g |

At 1/8 the raw gyro is a decimated sample, so the 2/rev blade-pass line folds to ~100 Hz — that is the
whole 50–120 Hz content (45–51 °/s RMS in `gyroRaw[1]`), exactly as the morning § 6 predicted when it
recommended 1/8. The one unaliased window, **20–50 Hz, holds 0.4–0.5 %** — nothing between the pitch cycle
and the blade line. The filtered `gyroADC[1]` above 20 Hz is **1.9–2.1 °/s** (same as the morning once the
unit is corrected). The morning's "above 30 Hz closed in flight" stands; this log adds no new line.

---

## 4. ⭐ Z — the estimator read directly, with the 20/20 marker visible

[`f1011b_bb_z_survey.py`](f1011b_bb_z_survey.py) on the blackbox's own `navPos[2]`, `navVel[2]`,
`navEPH/EPV` — no reconstruction this time.

- **Zero per-sample steps > 2 m** on `navPos[2]` across all 40 500 samples; largest **0.47 m** (at 280.8 s,
  the post-disengage wake after span 3). 50 ms vertical rate p99 15.2 m/s, max 27.6 m/s (whole log).
  xiao-side `pos_d` max per-tick step 0.62 / 0.69 / 1.00 m (morning 0.56–1.31; t3 86.7).
- ⭐ **`navEPV` whole-log max 541 cm, median 285**, against `inav_max_eph_epv = 1000`. The jump-flight marker
  ([`vibration-analysis-20261004.md`](vibration-analysis-20261004.md) § "20/20": every jump log peaks at 999,
  clean logs 463–936) reads clean, and this is the first ACRO-engaged log where it could be read at all. The
  morning flight was *consistent with* a clean estimator; this one *shows* it.
- `navEPH` median 189, max 279 cm. `navFlags` = 37 throughout (no flag change at any engage / disengage).

| segment | `navEPV` med / p95 / max (cm) | est − baro med / p95 / max (m, rel. ARM) | `navVel[2]` p95 / max (m/s) | `accVib` | `accWeightFactor` |
|---|---|---|---|---|---|
| pre-engage | 233 / 239 / 239 | +4.1 / 6.4 / 8.3 | 7.7 / 9.1 | 3719 | 0.71 |
| **engaged 1** | 279 / 288 / 290 | +4.6 / 10.4 / 13.7 | 8.3 / 11.1 | **6681** | **0.30** |
| manual after 1 | 287 / 333 / 334 | +8.1 / 10.6 / 13.0 | 7.7 / 11.5 | 2214 | 0.97 |
| **engaged 2** | 255 / 295 / 296 | +10.5 / 19.5 / 21.3 | 8.6 / 10.9 | **7169** | **0.30** |
| manual after 2 | 274 / 292 / 292 | +4.7 / 8.0 / 11.9 | 8.2 / 10.3 | 3775 | 0.70 |
| **engaged 3** | 346 / 419 / 438 | +9.6 / 19.0 / 23.7 | 12.0 / 16.4 | **7324** | **0.30** |
| manual after 3 | 398 / 520 / 541 | +4.8 / 18.8 / 24.5 | 11.9 / 16.6 | 2065 | 1.00 |

- **Estimate − baro** engaged p95 / max **10–19.5 / 14–24 m** — wider than the morning (8–12 / 12–17) and
  still inside the MANUAL-era envelope from the 09-13 A4 survey (8.6–29 / 12.5–42.7). The baro is being
  abused the same way as in the morning: engaged per-update step p95 **492–542 cm**, max 1005–1128 cm,
  residual RMS 3.5–3.7 m (manual 0.3–0.7 m), update rate 29 Hz. `inav_default_alt_sensor` is not in the
  header; nothing says the 10-04 `GPS_ONLY` recommendation was applied, and the baro residuals say it was not.
- **Estimate − GPS** engaged p95 **3.8 / 4.5 / 6.7 m**, max 5–8 m. GPS `eph` 185 / `epv` 262 cm median
  (morning 257–305 / 388–454) — a better constellation than the morning, which is the likely reason
  `navEPV` sat lower (285 vs the morning's unobservable value).
- **VIBE**: engaged `accVib` **6681–7324**, the highest of any flight in the repo (morning 5775–5909, t3 4661,
  t7 4327), `accWeightFactor` **0.30** (3.3× correction gain, morning 0.34), `accClipCount` **0** in every
  engaged span (the only clips are 30 at touchdown). `|az|` p99 6.1–7.4 g, max 8.7 g vs the 15.9 g
  threshold. ⇒ **the chain "throttle railed → accVib → accel fusion drift → EPV at 999 → jump" is broken at
  the EPV link with accVib at its all-time high.** Whatever made t3's six resets, it is not vibration
  magnitude, and after five flights it is not this genome's flying either.

---

## 5. ⚠️ Engage: the first 16 ticks of every span are not applied — correction to the morning § 5

The morning doc read the first 20 / 14 / 23 ticks of nose-down rail as an applied entry transient. They are
mostly not applied. Blackbox `mspOverrideFlags` at each engage ([`blackbox.c:1434`](../../../inav/src/main/blackbox/blackbox.c#L1434)
`2·boxActive + 1·mspOverrideInFailsafe`):

```
xiao ENGAGE (INAV clock)   flag 1→3 (box active, MSP still in failsafe)   flag 3→2 (override applied)
   167.58 s                      167.55 s                                       168.32 s   (+0.74 s)
   200.64 s                      200.63 s                                       201.37 s   (+0.73 s)
   236.81 s                      236.78 s                                       237.55 s   (+0.74 s)
morning: 262.11 / 296.87 / 338.18 / 380.48  →  262.83 / 297.61 / 338.92 / 381.25   (+0.72…+0.77 s)
```

Between 3 and 2 the pilot's receiver channels are what the rate loop flies (`rcCommand[1]` = 0, throttle at
the pilot's value, in the raw dump), and the xiao's commands — logged as `out_*` at full rail — go nowhere.
This is INAV's `rxDataRecoveryPeriod` = `PERIOD_RXDATA_RECOVERY` 200 ms + `failsafe_recovery_delay` 5 ×
100 ms ([`msp_override.c:104`](../../../inav/src/main/rx/msp_override.c#L104)), the **783 ms measured on the
bench on 09-01 and deliberately kept** (tasks T057, [`bench-notes.md`](bench-notes.md) § 783 ms) because it
primes the NN history buffers. Known, documented, correct — but the morning's entry read did not account for it.

Re-read with the first 16 ticks removed ([`f1011_xiao_entry.py`](f1011_xiao_entry.py), quat pitch):

| span | at switch (tick 0) | at override (tick 16) | applied −1.0 rail after tick 16 | pitch min in first 3 s |
|---|---|---|---|---|
| 1 Straight | +3.9°, `vel_d` −0.8 | **+17.4°, −4.7 m/s** (pilot pulled during the hold) | **11 ticks** (16–26) | −51° at tick 31 |
| 2 Fig-8 | +5.7°, +0.2 | +6.4°, −0.9 (level) | 2 ticks (16–17), then +0.72 by tick 20 | −25° at tick 52 |
| 3 RandomB | +22.1°, −6.0 | +23.5°, −6.3 | none (pulled up: rabbit 20 m away, 8 m above) | n/a (+74° at tick 28) |
| *morning 1 / 2 / 3* | +30 / +7 / +26° | +35 / +8 / +19° | ≈ 6 / 3 / 7 ticks | −16 / −18 / −34° |

Three things follow:

1. **The applied rail is 2–11 ticks, not 14–23**, and the nose-down excursion that follows it (to −51° here)
   is the policy's own response in the half-second after it gets the aircraft, not a prefill artefact.
2. **"Hand off level" means level at tick 16, not at the switch.** Span 1 was level at the switch and nose-high
   at override because the pilot kept flying through the 0.75 s hold. The procedural fix is "hold the
   attitude you want the NN to receive for a full second after the switch".
3. The rabbit holds at the origin for the same 16 ticks while the craft covers ~10 m at 13 m/s, so **every span
   begins with the target ~10 m behind the craft** — by construction, and absent from the sim's latched-level
   entry ([`NEXT-T4-RUN.md`](NEXT-T4-RUN.md)). Entry dither (T085/T086) should model the hold, not just the
   attitude. Span 2, entered level, shows what the policy does with that: a 2-tick dip and a clean recovery.

---

## 6. Blackbox config — the morning's recommendation, as flown

| | morning (10-04 bench config) | **this flight** |
|---|---|---|
| rate | 1/2 → 966 Hz | **1/8 → 239 Hz** |
| flash | 2.75 MB/min | **1.19 MB/min** |
| lost iterations | 0.00 % | 0.00 % (3) |
| `NAV_POS` | stripped (§ 4 rebuilt from xiao) | **present** — § 4 direct |
| `GYRO_RAW` / `VIBE` | yes / yes | yes / yes |
| 2–3 Hz band | intact | intact (60 Hz decimation by 4) |
| blade-pass line | 363–375 Hz, resolved | aliases to ~100 Hz (expected, characterised) |

Everything § 6 asked for, bought at 43 % of the flash rate. Keep it. Two housekeeping items: the flash was
not erased between the flights (13.1 MB holds both), so `flash_erase` before the next session; and the
rate-denominator was confirmed from the header (`P interval:1/8`), not the Configurator.

---

## 7. What this flight adds

**Established.**

- ⭐ g512 repeats: five of seven spans across the two flights have median error ≤ 15 m, two of six t5 paths
  (Fig-8, RandomB) now flown, Fig-8 at 25 % in-streak is the best ACRO span to date. Same-path repeat
  (Straight 13.2 → 8.1 m) sets the ~5 m noise floor for cross-flight comparisons.
- ⭐ **The estimator was clean *and seen to be clean*: `navEPV` max 541 cm, zero steps > 0.5 m**, with
  `accVib` at an all-time high and `accWeightFactor` at 0.30. The vibration → EPV → jump chain does not
  close on this genome.
- ⛔ Pitch cycle: fifth flight, same ±200 °/s, gain 1.4–1.5, −64 ms phase. **D1 stands.**
- The engage hold is 0.74 s of pilot control + 16 unapplied NN ticks; the morning § 5 entry read is corrected
  (applied rail 2–11 ticks).
- Two more helper scripts had the 10× gyro unit slip; corrected, morning figures restated in § 2.

**Not established.** Whether tracking survives wind (two calm days now). Whether `GPS_ONLY` would shrink the
10–24 m est − baro spread (not flown; header does not carry the setting).

**Ordered by expected value** (morning list, re-ranked):

1. ⭐ **Record g512** — `.dat` + hash into `artifacts-t5/`, eval suite, MANIFEST § 5h. Still open after two
   flights on it.
2. ⭐ **D1** (T093–T096). The only pitch-cycle lever left; now five flights deep.
3. **Entry dither models the 0.8 s hold** (target ~10 m behind at hand-off), not just attitude — § 5 item 3.
   Procedural: hold the hand-off attitude for a second *after* the switch.
4. Windy-day flight on g512.
5. `GPS_ONLY` decision (unchanged); `flash_erase` before the next session.

---

## Appendix — reproducing this

```bash
# blackbox: log 2 of 2 in the flash (log 1 is the morning flight)
~/blackbox-tools/obj/blackbox_decode --index 2 flight-results/flight-20261011/blackbox_log_2026-10-11_143538.TXT
# -> blackbox_log_2026-10-11_143538.02.csv / .02.gps.csv

# xiao (v5)
python3 src/analytics/flightlog_decode.py \
  flight-results/flight-20261011/flight_log_2026-10-11T21-42-19_flight_002.bin \
  -o /tmp/f2/xiao_ticks.csv --flightpath /tmp/f2/xiao_path.csv

# §2: 60 Hz block-average (factor 4 at 239 Hz), then the spectrum script
python3 specs/043-acro-dual-loop/f1011_bb_decimate.py \
  flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv /tmp/f2/bb_60hz.csv 4
python3 specs/043-acro-dual-loop/pitch_spectrum.py /tmp/f2/bb_60hz.csv --manual

# §1, §5: xiao per-span tables, rail timing, entry
python3 specs/043-acro-dual-loop/f1011_xiao_span_stats.py /tmp/f2/xiao_ticks.csv
python3 specs/043-acro-dual-loop/f1011_xiao_pitch_rail.py /tmp/f2/xiao_ticks.csv
python3 specs/043-acro-dual-loop/f1011_xiao_entry.py /tmp/f2/xiao_ticks.csv

# §1: downwind bias (xiao positions × blackbox wind)
python3 specs/043-acro-dual-loop/f1011b_wind_bias.py /tmp/f2/xiao_ticks.csv \
  flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv

# §2–4: blackbox per-span, rate loop (deg/s), full-rate spectrum, Z survey from NAV_POS
python3 specs/043-acro-dual-loop/f1011_bb_span_stats.py   flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv
python3 specs/043-acro-dual-loop/f1011_bb_rate_loop.py    flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv
python3 specs/043-acro-dual-loop/f1011b_bb_hr_spectrum.py flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv
python3 specs/043-acro-dual-loop/f1011b_bb_z_survey.py    flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.csv \
  flight-results/flight-20261011/blackbox_log_2026-10-11_143538.02.gps.csv
```

Stdlib-only, as before. The decoded `.02.csv` is 22 MB; the `.TXT` and the xiao `.bin` are the sources of record.

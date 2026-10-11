# 043-t5 — fourth ACRO flight (2026-10-11): best tracking yet, same pitch cycle, no Z jumps

Genome **t5 gen 512** (`weight_id=1731d937e3af9c1c`, `firmware_id=c3eb28e6b938dd1a`), program
`autoc-m1/autoc-9223370245346926812-2026-10-09T01:04:08.995Z/gen9512.dmp.zst` — the t5 bake
([`artifacts-t5/MANIFEST.md`](artifacts-t5/MANIFEST.md)). Dmp names are `10000 − gen`
([`autoc.cc:1639`](../../src/autoc.cc#L1639)), so `gen9512` is gen 512.

⚠️ **Provenance.** This is *not* one of the three § 5g snapshot candidates (g400 / g422 / g480); it was
cut from the live run after the gen-485 read. `nn_weights.dat` in this checkout hashes `3af8e3ab787b75a5`,
not `1731d937…`, and no `.dat` for it sits in `artifacts-t5/` — the flight article was built off-repo.
Record it (MANIFEST § 5h) and run the eval suite on it for the record before any further flight.

Logs: `flight-results/flight-20261011/`. **4 spans, 1930 ticks, 0 gaps, 0 overruns, 0 resyncs, 0 drops.**
Blackbox at **966 Hz** (`P interval 1/2`, the 2026-10-04 bench config left in place — § 6), 213 s,
0.00 % missing iterations, one log in flash (it *was* erased). Calm: INAV wind estimate **1.8–2.9 m/s**
median per span (t3 flew in 4.2).

⭐ **Headline: tracking is 2–3× tighter than any previous ACRO flight and throttle railing fell from
85–90 % to 46–59 %, yet the ~2 Hz pitch cycle is unchanged in amplitude (±200 °/s, ±7 g) — and a 10×
unit slip in the earlier docs is corrected below (§ 2). The vertical estimator never stepped: zero jumps
above 5 m anywhere in the flight, the NN's `pos_d` input moved at most 1.3 m per tick.**

> ⚠️ **Erratum (afternoon flight, same day)** — see [`flight-analysis-20261011b.md`](flight-analysis-20261011b.md):
> (1) § 2's rate-loop medians and § 3's `gyroADC[1] > 20 Hz RMS` are **10× low** (the `f1011_bb_*` scripts still
> divided by 10; the § 2 table itself is right) — setpoint median ≈ −140 °/s, gyro median −40…−70 °/s, > 20 Hz RMS
> 1.9–2.1 °/s. (2) § 5's 20 / 14 / 23-tick entry rails include the 16 ticks before INAV applies the MSP override
> (0.72–0.77 s, the T057 783 ms hold); the *applied* rail was ≈ 6 / 3 / 7 ticks and the hand-off attitude that
> matters is the one at tick 16.

---

## 0. Spans

From the xiao `INAV_CLOCK` anchors (xiao clock + 463 → 262 ms across the flight; the offset drifts
~0.1 %, so interpolate between anchors, don't use one constant):

| span | INAV t (s) | ticks | path | disengage |
|---|---|---:|---|---|
| 1 | 262.1 – 279.1 | 341 | 0 StraightAndLevel | reason 5 — path complete |
| 2 | 296.9 – 317.7 | 418 | 1 SpiralClimb | reason 5 — path complete |
| 3 | 338.2 – 360.7 | 451 | 4 HighPerchSplitS | reason 1 — servo switch (pilot) |
| 4 | 380.5 – 416.4 | 720 | 3 randomA | reason 1 — servo switch (pilot) |

`mspOverrideFlags == 2` in the blackbox agrees with these to < 1 s, as on 09-13.

---

## 1. ⭐ Tracking — best of the four ACRO flights

| span | path | trk err med / p90 / max (m) | `step_score` mean | in-streak (≥ 0.5) | thr +rail | thr mean | pitch rail (nose-down) | gnd spd med |
|---|---|---|---:|---:|---:|---:|---:|---:|
| 1 | Straight | **13.2** / 22.6 / 25.3 | **0.225** | 9 % | 53 % | +0.54 | 10.9 % (5.3 % after entry) | 13.4 |
| 2 | Spiral | **10.2** / 22.9 / 27.2 | **0.259** | 5 % | 59 % | +0.63 | 4.8 % (1.1 %) | 13.0 |
| 3 | HighPerch | **15.2** / 27.9 / 30.6 | **0.204** | 4 % | 46 % | +0.62 | 6.4 % (1.5 %) | 13.6 |
| 4 | randomA | **14.5** / 34.8 / 46.5 | **0.176** | 5 % | 51 % | +0.66 | 2.4 % (2.5 %) | 12.7 |
| *t3 09-13* | 0 / 2 / 3 | 33.6 / 24.5 / 31.8 med | 0.064 / 0.060 / 0.078 | — | 85–90 % | +0.82…+0.87 | 0.0–0.3 % | 9–16 |

- **Median tracking error 10–15 m against t3's 25–34 m; `step_score` 3× t3's.** Roll railing ≤ 0.3 %.
- **The one-sided downwind bias is gone.** Craft − rabbit projected on INAV's wind axis: **+5.0 / −2.3 /
  +5.5 / +1.8 m** against t3's +27.4 / +3.3 / +25.2. ⚠️ Not separable from the day: wind was 2–3 m/s
  (t3 4.2) *and* this genome trained under the t4/t5 0–8 m/s envelope. A windy flight decides it.
- **Throttle discipline changed**: +rail 46–59 % (t3 85–90 %), `motor > 1900` for 74–79 % of engaged
  samples (t3 90.5 %), switching 2.5–3.7 /s. Still the worst vibration band most of the time (§ 4).
- ⚠️ **Still a third of the sim.** The t5 § 5g per-path table has g422 at score/step **0.98 / 0.91 /
  0.75 / 0.37** and in-streak **72 / 70 / 64 / 39 %** on these four paths; the flight is 0.18–0.26 and
  4–9 %. (g512 ≠ g422, but the same run a few dozen gens later.) The sim-to-real tracking gap is ~3×,
  down from ~10× on t3.
- Spans 3 and 4 were ended by the pilot, not by path completion; both were clean at the switch.

---

## 2. ⛔ The pitch cycle is unchanged — and the earlier docs are 10× low on its amplitude

Reproduce with [`pitch_spectrum.py`](pitch_spectrum.py) on a 60 Hz block-average of the 966 Hz log
([`f1011_bb_decimate.py`](f1011_bb_decimate.py), factor 16) so the Welch segments match 09-13 / t7 exactly.

| | **t5 g512 (10-11)** spans 1 / 2 / 3 / 4 | **t3 (09-13)** 1 / 2 / 3 | **041-t7 (08-23)** 1–4 |
|---|---|---|---|
| pitch peak | **2.01 / 2.01 / 1.77 / 2.01 Hz** | 2.32 / — / 2.09 | 2.21 / 2.09 / 2.21 / 1.98 |
| power in 2–3 Hz | **63.5 / 50.7 / 44.8 / 39.3 %** | 56.2 / 30.4 / 74.6 | 53.5 / 69.3 / 44.8 / 37.0 |
| power in 1–2 Hz | 21.0 / 39.7 / 40.5 / 35.5 % | — | — |
| gyro pitch RMS | **147 / 145 / 131 / 122 °/s** | 127 / 133 / 133 | 123 / 139 / 110 / 115 |
| loop gain at peak | 1.52 / 1.45 / 1.44 / 1.45 | 1.71 / 1.68 / 1.55 | (MANUAL — n/a) |
| MANUAL control group, 2–3 Hz | **3.3–8.2 %** (5 stretches) | 7.5–16.4 % | — |

Same amplitude, same gain peak, the tone a little lower and broader (1.8–2.0 Hz with more 1–2 Hz
content in spans 2–4), present only with the NN engaged. The t5 excess-rotation axis moved sim `rotRatio`
from ~5.1× to ~4.5× and **did not move flight pitch-rate RMS at all** — exactly what the 09-13 addendum
A1 predicted: the resonance the policy would have to learn to avoid does not exist in the sim (D1).

⛔ **Units correction to the record.** `gyroADC[]` and `axisRate[]` in the blackbox are **deg/s**
([`blackbox.c:1674`](../../../inav/src/main/blackbox/blackbox.c#L1674) `lrintf(gyro.gyroADCf[i])`; the
decoder's `gyroScale` is 1 and the CSV carries the raw value). `pitch_spectrum.py` divided by 10, so every
°/s figure in [`flight-analysis-20260906.md`](flight-analysis-20260906.md) and
[`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 5 is **10× low** — the t3/t7 numbers in the
table above are the originals ×10. Three independent checks:

1. The xiao's own `gyro_q` (rad/s ÷ 6): span-1 RMS 0.429 × 6 × 57.3 = **147.5 °/s**; blackbox **147.2**.
2. Physics: ±7 g at 14 m/s needs q = a/V ≈ 280 °/s peak. 12 °/s would give 0.2 g.
3. The setpoint reaches **−198 / +238 °/s** at p5 / p95 against the 240 °/s `rates` cap.

⇒ **The cycle is ~±200 °/s peak, not ±25.** It is a ±7 g, ±200 °/s bang-bang at 2 Hz. The 09-13 addendum
already flagged the "≈ 10× apart" absolute-scale mismatch without resolving it; this is the resolution.
The script is fixed in this commit; ratios (gain, band share) in the old docs are unaffected.

Rate-loop detail, this flight (966 Hz, [`f1011_bb_rate_loop.py`](f1011_bb_rate_loop.py)): setpoint median
−14 °/s with p5/p95 at the cap; gyro median −4 to −7 °/s; PID-sum magnitude ≥ 500 (mixer saturation) on
only **1.8–7.1 %** of samples; feed-forward is 98–100 % of the output — R1's "FF-dominant, not a PID"
holds in flight.

Roll: the xiao `out_roll` 5–10 Hz share is **18.5 / 20.4 / 13.3 / 14.7 %** against t3's 30–37 % — the
tick-to-tick roll flipping is roughly halved. The one visible flight-side effect of the t5 axis.

---

## 3. ⭐ Above 30 Hz — closed in flight, engaged

The 966 Hz log answers t3 § 5.5's open question with the policy in the loop
([`f1011_bb_hr_spectrum.py`](f1011_bb_hr_spectrum.py), 8192-pt Welch, 0.118 Hz bins):

| engaged span | `gyroRaw[1]` 20–300 Hz | `gyroRaw[1]` 300–480 Hz | dominant line | `gyroADC[1]` > 20 Hz RMS | `accSmooth[2]` > 20 Hz RMS |
|---|---:|---:|---|---:|---:|
| 1 | 1.4 % | 9.8 % | 375 Hz | 0.19 °/s | 0.03 g |
| 2 | 1.1 % | 12.6 % | 367 Hz | 0.20 °/s | 0.03 g |
| 3 | 1.3 % | 16.3 % | 369 Hz | 0.21 °/s | 0.03 g |
| 4 | 1.6 % | 17.2 % | 363 Hz | 0.21 °/s | 0.03 g |

Nothing throttle-independent between 20 and 300 Hz; the only content above 20 Hz is the 2/rev blade-pass
line at 363–375 Hz (`gyroPeakPitch[2]` median 344–362 Hz, i.e. INAV's own FFT agrees), and the filter
chain leaves **0.2 °/s** of it. The bench result
([`vibration-analysis-20261004.md`](vibration-analysis-20261004.md) § 3) now holds in flight with the
airframe free and loaded. ⇒ t3 § 5.5's "structural / actuator mode" candidate is out; **D1 (a real pitch
short period the sim lacks) is the only standing explanation** for the 2 Hz cycle.

---

## 4. ⭐ Z — no estimator steps, with the cleanest ACRO altitude record so far

⚠️ The 10-04 bench config has `blackbox -NAV_POS`, so `navPos[]`, `navVel[]`, `navEPH/EPV` are absent.
The whole-flight `navPos[2]` trace was **rebuilt from the xiao**: armed-not-engaged breadcrumbs
(`pos_raw_d`, ~20 Hz) plus engaged ticks (`origin_d + pos_d`), aligned through the `INAV_CLOCK` anchors
([`f1011_xiao_z_survey.py`](f1011_xiao_z_survey.py)). Both are `posEstimator.est.pos.z` via
`MSP2_AUTOC_STATE`, so this is the same quantity the blackbox would have logged.

- **Zero samples with |Δz| > 5 m** across 4270 samples, 236–450 s — in-span *and* the post-disengage
  wakes where 18 of the repo's 24 historical events sit. |dz/dt| p99 16 m/s.
- **NN input `pos_d` max per-tick step: 0.81 / 0.56 / 0.81 / 1.31 m** (t3 span 2: 86.69 m).
- Estimate − baro while engaged, med / p95 / max: **+0.8 / 8.1 / 12.1**, −2.2 / 11.8 / 15.9,
  −1.0 / 10.7 / 17.3, −0.4 / 9.1 / 12.5 m. Against the 09-13 A4 survey: t3 **60.2 / 93.2**, t7 12.8 / 20.4,
  the ten MANUAL flights 8.6–29 / 12.5–42.7. ⇒ this flight sits with t7, inside the MANUAL envelope.
- Estimate − GPS: p95 4–8 m. GPS: 7 satellites throughout, `epv` 388–454 cm, `eph` 257–305 cm.

### What `debug_mode = VIBE` adds (first autoc flight with it)

| segment | `accVib` med | g | `accWeightFactor` (debug[4]) med | correction gain | `accClipCount` (debug[3]) |
|---|---:|---:|---:|---:|---:|
| engaged 1–4 | 5775–5909 | **2.8–2.9** | **0.34–0.36** | **~2.9×** | **0, all four spans** |
| pre-engage | 4818 | 2.35 | 0.53 | 1.9× | 0 |
| manual between | 2764–4735 | 1.3–2.3 | 0.54–0.88 | 1.1–1.9× | 0 |
| landing | 2163 | 1.1 | 0.98 | 1.0× | 40 (touchdown) |

- ⭐ **Accelerometer clipping is closed.** Zero clips in every engaged span; `|accSmooth[2]|` p99 5.9–7.6 g,
  max 8.85 g, against the 15.9 g threshold. The "unconfirmed, ~2× away" row in vibration-analysis § 6.5
  is now a no.
- Engaged `accVib` is **higher** than t3's (5900 vs 4661) and the estimator's accel-correction gain was
  **2.9× vs t3's 2.0×** — with no jump. One more flight where vibration / `accWeight` fails to discriminate.

### The operator's "less aggressive" hypothesis, measured

| | t5 g512 (10-11) | t3 (09-13, 6 resets) | t7 (08-23, clean) |
|---|---|---|---|
| wind | **1.8–2.9 m/s** | 4.2 | 1.2 |
| `vel_d` p95 (engaged) | **7.0–10.6 m/s** | 10.3–13.5 | 9.7–18.2 (MANUAL era) |
| throttle +rail | **46–59 %** | 85–90 % | 68 % (`motor > 1900`) |
| baro over-physical step p95 (engaged) | **441–527 cm** | 759 | 528 |
| `|az|` p99 / max | 5.9–7.6 / 8.9 g | 5.2–7.5 / 11.4 | — |
| gyro pitch RMS | 122–147 °/s | 127–133 | 110–139 |
| `accVib` engaged | 5900 | 4661 | 4327 |

Supported on the **vertical-speed and throttle** axes and on the baro abuse (back to t7's level), **not** on
the pitch cycle or g-load, which are unchanged; and the day was calmer. With `NAV_POS` off the 20/20
marker (`navEPV` reaching `inav_max_eph_epv`) is unobservable, so this flight **cannot say whether the
estimator approached the cap** — it is consistent with the hypothesis, not a discriminator. Restore
`NAV_POS` (§ 6) before the next ACRO flight; that single field set is what turns a clean flight into evidence.

⚠️ `inav_default_alt_sensor` is still `GPS` with `inav_w_z_baro_p 0.350 > inav_w_z_gps_p 0.200` in the
config of record — the 10-04 `GPS_ONLY` recommendation was not applied. The baro remains the heaviest
contributor to `pos_d`, and this flight's engaged baro residual RMS is 3.0–3.6 m (manual 0.5–1.3).

---

## 5. ⚠️ The nose-down rail is an entry transient, not the 09-05 OOD hold

Spans 1–3 were handed off **nose-high and climbing**: pitch +30° / +7° / +26°, `vel_d` −4.6 / −3.8 / −4.6
m/s. The rabbit holds at the origin for the first 16 ticks (0.8 s), so the NN sees a target below and
behind it and rails `out_pitch = −1.0` for **20 / 14 / 23 ticks** (0.7–1.15 s), pitching through to
−16° / −18° / −34° before it settles ([`f1011_xiao_entry.py`](f1011_xiao_entry.py)). Span 4 entered at +28°
climbing 7.4 m/s but the randomA rabbit starts 22 m *above* and 38 m away, so it pulled up instead.

Post-entry (after tick 40) the nose-down rail is **5.3 / 1.1 / 1.5 / 2.5 %** in short runs (longest 16
ticks, span 1 at ticks 200–215) — up from t3's 0–0.3 % but nothing like 09-05's sustained hold with an
8.6 m/s descent. Sim entry is latched level at 0.1–1.1 m
([`NEXT-T4-RUN.md`](NEXT-T4-RUN.md), dither deferred T085/T086); **hand-off attitude is not a variation
class**, so the first second of every span is out of distribution by construction. Cheapest fix is
procedural: hand off from level flight. The structural one is entry dither.

---

## 6. ⛔ Blackbox config for the next flight

The 2026-10-04 bench settings flew. What they bought and cost:

**Good** — 966 Hz with 0.00 % loss; `GYRO_RAW` + `VIBE` closed two open questions in one flight (no >30 Hz
parent in flight, § 3; no accel clipping, § 4) and gave `accWeightFactor` per tick.

**Bad** — `-NAV_POS` removed `navPos/navVel/navEPH/navEPV`, the only estimator-side evidence and the one
marker that separates jump flights 20/20 (§ 4 had to be rebuilt from the xiao). `-SERVOS` and `-QUAT`
cost nothing this time (the "servos carry it" argument is settled; xiao `quat_*` at 20 Hz sufficed).
Flash: **9.76 MB in 213 s = 2.75 MB/min**; read the ceiling from `flash_info` before a longer session.

Both questions `1/2` was kept for are now answered, bench and flight. Recommended:

```
set blackbox_rate_denom = 8          # ~250 Hz: keeps the 3-4 Hz short period, the 25 Hz LPF corner
                                     # and the notch behaviour; 4x the endurance. Only the 360 Hz
                                     # blade-pass line aliases, and it is characterised.
set debug_mode = VIBE                # keep: accWeightFactor + accClipCount
blackbox NAV_POS                     # RESTORE: navPos[2], navVel[2], navEPV -- the Z discriminator
blackbox GYRO_RAW                    # keep (cheap at 1/8)
blackbox ACC
blackbox MOTORS
blackbox RC_COMMAND
blackbox PEAKS_P
save
```

Set it in the CLI, `save`, **do not open the Configurator Blackbox tab afterwards** (it rewrites the
rate — three prior instances), `get blackbox_rate_denom` to confirm, `flash_erase` before the flight, and
read `P interval` back out of the header after it. Decide `inav_default_alt_sensor = GPS_ONLY` separately
(§ 4) — it changes the estimator, so it belongs in the record if flown.

---

## 7. What this flight establishes

**Established.**

- ⭐ t5 g512 is the best-tracking ACRO genome flown: median error 10–15 m, `step_score` 3× t3, roll
  railing ≤ 0.3 %, throttle rail halved, downwind bias gone (on a calm day).
- ⛔ The 2 Hz pitch cycle is **invariant across t7 / t3 / t5** — four flights, three genomes, two
  architectures — at **±200 °/s, ±7 g** (the earlier ±25 °/s figures were a 10× unit slip). The t5 smoothness
  axis moved the sim and not the aircraft. Nothing above 20 Hz in flight but blade-pass. **D1 stands alone.**
- Accelerometer clipping is ruled out in flight. Vibration / `accWeight` fails to discriminate for the
  third time, now with the gain at 2.9×.
- No estimator step anywhere in the flight; `pos_d` moved ≤ 1.31 m per tick.
- The engage prefill, action-space and 20 Hz loop are solid: 0 gaps / overruns / drops over 1930 ticks.

⛔ **Not established.** Why *this* ACRO flight stayed clean — `navEPV` was not logged. Whether the tracking
gain is the genome or the calm — no windy flight yet with a t4/t5 genome.

**Ordered by expected value:**

1. ⭐ **Record and qualify g512**: MANIFEST § 5h, `nn_weights.dat` / `.dat` into `artifacts-t5/`, eval
   suite for the record. It is the flight baseline of record for t5 until something beats it in the air.
2. ⭐ **Restore `NAV_POS`, drop to `1/8`** (§ 6) — one flight with the marker visible converts § 4 into evidence.
3. ⭐ **D1** (T093–T096) — the only lever left on the pitch cycle. Every other candidate is now excluded by data.
4. Hand off from level flight; schedule entry dither (T085/T086).
5. A windy-day flight on the same genome to split "genome" from "calm" in § 1.
6. `GPS_ONLY` decision; static-port re-site stays where it was.

---

## Appendix — reproducing this

```bash
# blackbox (blackbox-tools, INAV 8.0.0 build)
~/blackbox-tools/obj/blackbox_decode flight-results/flight-20261011/blackbox_log_2026-10-11_104616.TXT

# xiao (v5)
python3 src/analytics/flightlog_decode.py \
  flight-results/flight-20261011/flight_log_2026-10-11T17-43-33_flight_001.bin \
  -o /tmp/f/xiao_ticks.csv --flightpath /tmp/f/xiao_path.csv

# §2: 60 Hz block-average, then the unchanged spectrum script (now reporting deg/s)
python3 specs/043-acro-dual-loop/f1011_bb_decimate.py \
  flight-results/flight-20261011/blackbox_log_2026-10-11_104616.01.csv /tmp/f/bb_60hz.csv 16
python3 specs/043-acro-dual-loop/pitch_spectrum.py /tmp/f/bb_60hz.csv --manual

# §1, §5: xiao per-span tables, rail sign/timing, entry transient
python3 specs/043-acro-dual-loop/f1011_xiao_span_stats.py /tmp/f/xiao_ticks.csv
python3 specs/043-acro-dual-loop/f1011_xiao_pitch_rail.py /tmp/f/xiao_ticks.csv
python3 specs/043-acro-dual-loop/f1011_xiao_entry.py /tmp/f/xiao_ticks.csv

# §2–4: blackbox per-span (wind, accVib, VIBE, g-load, baro), rate loop, full-rate spectrum
python3 specs/043-acro-dual-loop/f1011_bb_span_stats.py flight-results/flight-20261011/blackbox_log_2026-10-11_104616.01.csv
python3 specs/043-acro-dual-loop/f1011_bb_rate_loop.py  flight-results/flight-20261011/blackbox_log_2026-10-11_104616.01.csv
python3 specs/043-acro-dual-loop/f1011_bb_hr_spectrum.py flight-results/flight-20261011/blackbox_log_2026-10-11_104616.01.csv

# §4: navPos[2] rebuilt from the xiao (needs the two xiao CSVs in /tmp/f, the 60 Hz CSV and the gps CSV)
python3 specs/043-acro-dual-loop/f1011_xiao_z_survey.py /tmp/f \
  flight-results/flight-20261011/flight_log_2026-10-11T17-43-33_flight_001.bin /tmp/f/bb_60hz.csv \
  flight-results/flight-20261011/blackbox_log_2026-10-11_104616.01.gps.csv
```

All scripts are stdlib-only (no numpy on this host). The decoded `.01.csv` is 103 MB at 1/2 rate — too
large to track; the `.TXT` and the xiao `.bin` are the sources of record.

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

# 043 ACRO dual-loop — outcome (closed 2026-10-18)

**Scope:** the NN commands **rotation rates** into INAV's 2 kHz ACRO loop instead of surface deflections at 20 Hz, to remove the
2–5 Hz pitch/roll oscillation of the 041-t7 flight. Spec of record [spec.md](spec.md); mid-feature record
[checkpoint-20261013.md](checkpoint-20261013.md); run records [t3](artifacts-t3/MANIFEST.md), [t4](artifacts-t4/MANIFEST.md),
[t5](artifacts-t5/MANIFEST.md), [t6](artifacts-t6/MANIFEST.md); exit flight [flight-analysis-20261018.md](flight-analysis-20261018.md).

**Result in one paragraph.** The dual loop works and tracks ~3× better than anything before it (t5). The pitch cycle that motivated
the feature turned out not to be the inner loop at all: it was the outer loop's relay against an airframe resonance **the sim did not
have**, because the model's pitch inertia was 7× too small. Measuring the inertia (bifilar pendulum), making the sim's pitch plant
honest and baking on it (t6) **halved the 2 Hz pitch cycle in the air** on the 2026-10-18 exit flight, and the flight's own airframe
response confirmed the measured inertia within ~10 %. The same flight exposed the next error: the sim's **roll** mode is ~4.5× too slow,
and roll oscillation rose. Exit verdict (T118): **partly met** — pitch yes, load no, roll regressed. Six bakes, seven ACRO flights.

## 1. Results

| | 041-t7 (surfaces @ 20 Hz) | 043-t3 | 043-t5 gen 488 (flown 10-11) | **043-t6 gen 752 (flown 10-18)** |
|---|---|---|---|---|
| flight median distance to rabbit | — | 25–34 m | 9.6–13.9 m | **9.1–13.6 m** |
| pitch-rate RMS in flight | 141 °/s (039 NN-direct ref.) | — | 144–148 °/s | **106–109 °/s** |
| pitch 2–3 Hz share (abs. 2–3 Hz RMS) | 2.0–2.2 Hz cycle | — | 49 % (103 °/s) | **22 % (51 °/s)** |
| pitch 3–5 Hz share (abs.) | — | — | 4 % (29 °/s) | 27 % (56 °/s) ⚠️ |
| pitch command p5 | — | — | −227 … −239 °/s (on the cap) | **−106 … −126 °/s** |
| roll-rate RMS / roll 3–5 Hz share | 30.1 % at 3–5 Hz | — | 108–116 °/s / 26 % | 135–139 °/s / **50 %** ⚠️ |
| \|a_z\| p99 | — | — | 6.5–6.7 g | 7.0–7.2 g |
| sim regiment `pctInStreak` / crashes | 38.3 % / 2.7 % | 54.6 % (calmer) | 51.1 % / 0 (t5 gen 800) | 46.7 % / 0 (t6 gen 800, harder plant) |
| sim cold start (tier2-random) | — | 122/144 | 144/144 | 144/144 |

What moved, and why: the 2 Hz pitch cycle fell because the plant became honest (§ 2) — not because of the rate architecture or any
smoothness objective. The pitch energy that remains moved up to 3–5 Hz (small-amplitude command dither the real airframe passes and the
model filters a little more). Roll got worse for the same reason pitch used to be bad: a sim airframe that low-passes the dither.

## 2. What caused it — the chain of findings

1. **ACRO works as built** (t1–t3, flights 08-31 … 09-13): the 2 kHz inner loop follows the NN's rate command faithfully; zero command
   = zero rate (SC-012); loop health clean on every flight (0 gaps / overruns / resyncs).
2. **Tracking and robustness came from the M1 problem, not the architecture**: crash cost and arena price (t3), the 0–8 m/s wind
   envelope and a second random course (t4/t5), crash-free selection. Downwind bias gone; cold-start 144/144.
3. **Objectives cannot price what the plant hides** (t4/t5): a standalone excess-rotation case was won by a passive flyer (t4 stalled);
   as a tie-break inside the tracking case (t5) it cut roll/yaw rotation but made pitch *more* bang-bang — the sim low-passed the dither.
4. **The pitch cycle was plant mode placement** (Phase 10c–10e): sim and aircraft closed-loop FRFs agreed to 1 Hz, then the aircraft
   resonated at 2.5 Hz while the sim kept rising to 6 Hz; model short period 6.9 Hz / ζ 0.81 vs the aircraft's ~3 Hz / ζ 0.2–0.3;
   cause `I_yy` 0.0013. Measured **0.0094 kg·m²** (bifilar, [actuator-pin.md](actuator-pin.md)); `Cm_q` −4.2 → −1.0; NN command applied
   at the 5 ms sub-step (it had been one 50 ms frame late by construction). Ladder: gen 488 then cycled in the sim like its flights.
5. **t6 on the honest plant** out-tracked and out-smoothed gen 488 on that plant; in **one equal environment** (T122: same seed, every
   genome at full variation strength — eval otherwise reuses each genome's save-time ramp) the latest elite was best on pitch *and*
   tracking; early "smooth" elites had been selected in milder air (gen 523: 6 arena exits at full strength).
6. **Flight confirms the inertia** (servo outputs logged, airframe FRF through the setpoint): with the model's stiffness the flight
   implies `I_yy` ≈ 0.0086 (bifilar 0.0094; 0.0013 rejected). The flight is more damped than the model — the **full streamer** was towed
   on both flights (streamer-on `Cm_q` ≈ −3.5 vs the bare −1.0).
7. **Roll is the next plant error**: flight roll subsidence corner **~5 Hz** vs the model's 1.1 Hz. t6 rescaled roll derivatives by the
   inertia ratio to keep roll "unchanged from t5" — but t5's roll mode was already too slow; the original `Cl_p` −0.47 with the measured
   `I_xx` 0.0069 would have landed near 3.9 Hz.
8. **What the policy sees in time** ([policy-temporal-view.md](policy-temporal-view.md)): history exists only for the target bearing;
   no craft sensor has any; the memory holds the previous command exactly but not the rate history; the rate derivative (the damping
   term) is mostly unavailable to a one-layer read-out. Proposal and prediction → 044.

**Not the cause** (measured): vibration / accel clipping; anything above 20 Hz but blade pass; aliasing of the filtered gyro / accel z
into the 20 Hz samples (≤ 1.6 % on 50 ms differences); compute latency (the sim carried *more*); the 20 Hz rate alone.

## 3. Spec success criteria

| SC | status at close |
|---|---|
| SC-001 smoother in the air | **pitch: met** (2–3 Hz power halved, cmd off the cap); **roll: not met** — 3–5 Hz share 50 % vs 041-t7's 30.1 % |
| SC-001a magnitude | pitch-rate RMS −27 %, 2–3 Hz RMS 103 → 51 °/s; roll RMS +22 %, 3–5 Hz RMS 58 → 97 °/s; the inner loop alone moved pitch 0 % (t5) |
| SC-002 emulation divergence measured | met — FRFs, short period, airframe ID in flight (pitch within ~10 %, roll ~4.5× slow) |
| SC-003 tracking does not regress | met (3× better than t3; level between t5 and t6 flights) |
| SC-004 trainability survives the plant change | met (T092 gate; t6 trained, ~2× slower start) |
| SC-005 041 agreement holds | partially re-verified (clock-join routine; formal cross-validation not repeated) |
| SC-006 loop health | met on every flight |
| SC-008 / SC-013 variation regime legible | **not completed** → BACKLOG (T079/T080) |
| SC-009 IMU axes free when off | met (bitwise gates) |
| SC-010 actuator measured | step tests and the airframe FRF yes; servo bench step (T046) no → BACKLOG |
| SC-011 arm's-length answered | met — inner-loop visibility not the lever; the *own-rate history* question replaces it (→ 044) |
| SC-012 zero command → zero rate | met |
| SC-014 flies sensibly, recovery is the policy's | met — seven flights, all spans ended cleanly |

## 4. Method and tools that outlive 043

- Equal-environment candidate selection: `set_eval_scale.py` + eval tier 0 with one fixed seed + `candidate_compare.py` (sim and flight
  with one metric code, calibrated through a flown genome).
- Airframe identification from flight: `airframe_id.py` (setpoint-referenced FRF; short-period and roll-mode fits).
- Policy introspection: `pitch_input_attribution.py`, `nn_replay_probe.py` (exact replay), `rate_history_availability.py`,
  `history_lag_study.py`, `damping_frontier.py`; spectra `sim_pitch_spectrum.py`, `pitch_cycle_progress.py` (in the PNG set).
- Aliasing budget `sensor_alias_budget.py`; wind draws reproduced exactly `wind_draws.cc` / `windiest_scenarios.py`; `bifilar_period.py`.
- Data hygiene: departed (tumbling) scenarios excluded in every least-squares analysis; FC config frozen at the sim's gains for flights.

## 5. Carried forward

| item | to |
|---|---|
| Craft-sensor history (gyro p/q/r + accel z at 50/100 ms), input-layout clean-up, FC sensor-delay model | **044** (requirements) |
| Roll plant correction (`Cl_p/I_xx`, partial, as a variation class) + honesty ladder; streamer-on `Cm_q` class | **044** |
| Scenario PRNG class-stream independence fix; log the wind-envelope draws; explicit eval variation-scale key | **044** (baseline boundary) |
| Flight discipline: streamer state per span (T119), FC config frozen to the sim, level boot | **044** procedure |
| M2 tracking and its two gates | **045** (renumbered from 044) |
| Article 2 (bifilar, flights, FR-056 n = 2), clean `I_xx` repeat, doublets / formal fit, per-cell step sweep, servo bench step | BACKLOG |
| Command-side measures (tie-break only), relay-amplitude flight, INAV phase-delay params, servo rate, roll dead-band | BACKLOG |
| Observability audit (SC-008/013), entry dither / heading / path library, six random courses, windy-day flight, GPS_ONLY + Galileo | BACKLOG |
| Throughput: per-tick cost A/B (t2→t3 +47 %), worker-side scoring / `EvalResultsLite`, dmp write overlap | BACKLOG |
| \|a_z\| p99 < 5 g criterion — unreachable by pitch smoothing at this tracking aggression; revisit with 044's exit | 044 exit design |

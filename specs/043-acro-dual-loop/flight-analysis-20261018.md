# Flight analysis 2026-10-18 — t6 gen-752 elite, the 043 exit flight (T124/T125/T118)

Genome `7faeea2d022cbb95` (t6 elite, created gen 750; picked by T122 in one equal environment). Two flights, same session:
flight 1 = blackbox log 1 + `flight_log_2026-10-18T19-56-02_flight_001.bin` (4 engaged spans, 68 s); flight 2 = blackbox log 2 +
`flight_log_2026-10-18T20-20-32_flight_002.bin` (3 spans, 85 s), more wind, from ~350° (operator). FC config of record
`xiao/inav-hb1.cfg` @ 91abd4d (10-11 loop, altitude GPS, blackbox 1/8 with SERVOS + ATTI). Operator: *"subjectively the pitch
oscillation is reduced, some increase in roll oscillation — in general fairly good flight."* ⚠️ Streamer state per span not recorded
(T119) — needed for the damping reading in § 3.

## 1. The exit measures (xiao logs, same code as the 10-11 g488 flights; medians over engaged spans)

| | g488 10-11 #1 | g488 10-11 #2 | **g752 10-18 #1** | **g752 10-18 #2** | T122 prediction |
|---|---|---|---|---|---|
| pitch-rate RMS °/s | 144 | 148 | **106** | **109** | ≈ 91 |
| pitch 2–3 Hz share | 48 % | 50 % | **21 %** | **23 %** | ≈ 17 % |
| pitch command p5 / p95 °/s | −227 / 198 | −239 / 198 | **−126 / 159** | **−106 / 144** | ≈ −165 / 140 |
| pitch command flips/s | 2.7 | 2.9 | 3.8 | 3.7 | ≈ 4.8 |
| \|a_z\| p99 g | 6.7 | 6.5 | 7.2 | 7.0 | ≈ 6.0 |
| roll-rate RMS °/s | 108 | 116 | **139** | **135** | — |
| roll 3–5 Hz share | 26 % | 27 % | **43 %** | **50 %** | — |
| roll command flips/s | 5.9 | 4.9 | 7.3 | 7.1 | — |
| median distance to rabbit m | 13.9 | 9.6 | 13.6 | 9.1 | — |

**T118 verdict: partly met.** Pitch: RMS −27 % and below the g488 band (124–153) on every span but flight 1's 7-s span 1 ✓; 2–3 Hz
share more than halved, within 1.5× of MANUAL's 3–16 % ✓; command off the cap ✓; tracking within noise of g488 ✓. Not met:
\|a_z\| p99 < 5 g ✗ (7.0–7.2, slightly *worse*). New regression outside the criteria: **roll** — RMS +22 %, the 3–5 Hz roll share
nearly doubled. The pitch reduction landed between the old flights and the prediction (107 vs 91 °/s predicted).

## 2. Is the measured inertia right? — the bare-airframe response ([`airframe_id.py`](airframe_id.py))

With servo outputs logged, elevator δe = (servo0 − servo1)/2 and aileron δa = −(servo0 + servo1)/2 (mixing fitted from the data,
R² 0.95–0.96). The airframe FRF is taken through the NN's setpoint (H = S_rq / S_rδ), which keeps feedback from biasing it;
coherence 0.81–0.98 over 0.5–6 Hz, 157 s pooled. Model fits at the flight's median airspeed, 15 m/s:

| pitch: model short period fitted to the flight FRF | best value | fit residual |
|---|---|---|
| `I_yy` free, model `Cm_a` −0.55 / `Cm_q` −1.0 | **0.0072** (flight 1) / **0.0069** (flight 2) kg·m² | 0.02 |
| `I_yy` = 0.0094 (bifilar) | — | 0.04 |
| `I_yy` = 0.0013 (pre-t6 file) | — | 0.39 ✗ |
| `Cm_a`, `Cm_q` free at `I_yy` 0.0094 | `Cm_a` −0.60 (model −0.55), `Cm_q` −3.5 (model −1.0) | 0.01 |

The flight's airframe peaks at 2.0–2.5 Hz (|H| ×1.6 of the 1 Hz value, −58° at 2.4 Hz). **The frequency placement confirms the
bifilar inertia**: with the model's stiffness, `Cm_a/I_yy` from flight is −64 against the model's −59, i.e. an implied `I_yy` of
0.0086 — within 9 % of the 0.0094 measured on the pendulum; the old 0.0013 is rejected outright. The fit wants **more pitch damping**
than the model (ζ ≈ 0.55 vs 0.46; `Cm_q` ≈ −3.5 vs −1.0) — the `I_yy`-free fit gets there by shrinking `I_yy` to 0.007, the free-
coefficient fit by raising `Cm_q`. A streamer-on flight (T119: ΔCm_q ≈ −2 D_s l_t²/(q̄Sc̄²)) would read exactly this way; the state is
needed to decide between the two.

## 3. Roll — the sim's roll mode is ~4.5× too slow, which explains the roll regression

| roll subsidence (p/δa = Lδ /(s − Lp)) | corner | τ |
|---|---|---|
| flight, both flights | **5.0 Hz** (4.9 / 5.1) | 32 ms |
| model, `I_xx` 0.0069, `Cl_p` −0.135 (t6) | 1.1 Hz | 140 ms |
| model, `I_xx` 0.024, `Cl_p` −0.47 (t5 — same ratio) | 1.1 Hz | 140 ms |
| model, `I_xx` 0.0069 with the pre-rescale `Cl_p` −0.47 | 3.9 Hz | 41 ms |

The measured roll response is flat to ~4 Hz; the model's falls off from 1.1 Hz. t6 rescaled the roll derivatives by the inertia ratio
"so roll is unchanged from t5" — but t5's roll mode was already ~4.5× too slow, so the rescale preserved the error; keeping the
original `Cl_p` −0.47 with the measured `I_xx` would have landed near the flight. It is the pitch story again on the other axis: a
sluggish sim roll airframe low-passes the policy's roll dither (t6 elites flip roll 7–8/s in sim, more than t5), so selection never
paid for it; the real airframe passes it, and the 3–5 Hz roll share doubled. `I_xx` itself stays provisional (bifilar repeat owed);
the flight fixes the **ratio** `Cl_p/I_xx` ≈ 4.5× the model's.

## 4. What this means for the wrap and the next step

- Pitch: the honest pitch plant did what it was built for — the 2 Hz cycle is roughly halved in the air, and the inertia behind it is
  confirmed by flight within ~10 %. Residual: more damping in the air than the model (streamer or `Cm_q`).
- Roll: the next plant correction is known and cheap — restore the roll damping-to-inertia ratio (`Cl_p` ≈ −0.47…−0.6 at
  `I_xx` 0.0069, i.e. a roll corner of 4–5 Hz at 15 m/s), check `Cl_da` with the same FRF, then let a bake price roll dither.
- Load: \|a_z\| p99 stayed ~7 g on both genomes; the 5 g line is not reachable by pitch smoothing alone at this tracking aggression.
- Caveats: one airspeed for the model (15 m/s median, p10–p90 10–21 m/s); servo lag folded into a fitted pure delay (25–40 ms);
  first-order roll model; streamer state unknown.

# Sim pitch-fidelity proposal — put the short period where the aircraft has it (2026-10-13)

Operator: *"we are now back to having good recent pitch data for flight article 1. can we propose a simulator change
(phase delay, gain, natural frequency) that gets sim closer to real?"* Data: the two 2026-10-11 flights (966 / 239 Hz
blackbox, gen 488), [`actuator-pin.md`](actuator-pin.md) § 2 / § 7 (bench step tests), the gen-488 dmp. Tools:
[`frf_pitch_compare.py`](frf_pitch_compare.py), [`sim_pitch_spectrum.py`](sim_pitch_spectrum.py).

## 1. What the data says — the gap is mode placement, not delay

Closed inner loop, **gyro / rate-setpoint** (INAV rate loop + servo + airframe as one block), same genome in both worlds.
Flight = the four engaged spans of the morning flight (coherence ≥ 0.8 shown bold-worthy; 3–3.5 Hz bins have low
coherence — the loop does not excite them); sim = medians over the 294 scenarios of gen 488's dmp.

| f (Hz) | 0.5 | 1.0 | 1.5 | 2.0 | **2.5** | 3.0 | 4.0 | 5.0 | 6.0 |
|---|---|---|---|---|---|---|---|---|---|
| **flight \|H\|** | 0.46–0.50 | 0.54–0.78 | 0.95–1.30 | 1.28–1.50 | **1.53–1.63** | 0.4–0.9 (coh 0.1–0.35) | 0.7–1.1 | 0.2–0.6 | 0.35–0.66 |
| **flight phase** | −6…−25° | −13…−28° | −29…−35° | **−44…−49°** | −49…−64° | — | −98…−138° | — | −99…−170° |
| **sim \|H\|** | 0.48 | 0.65 | 0.87 | 0.95 | 1.04 | 1.17 | 1.31 | 1.45 | **1.54** |
| **sim phase** | −10° | −12° | −32° | **−48°** | −64° | −83° | −125° | −163° | (wrapped) |

- **Below 1 Hz the two are the same block** (0.48 / 0.65 and ~−10°): steady gain and the FF-dominant rate loop are right.
- **Phase agrees to 2.5 Hz** (−48° at 2 Hz in both; −64° at 2.5). The pure delay the sim carries in excess of the aircraft
  (one tick, tasks T111) is not what is visible here — it sits in what the *policy* samples, not in the plant block.
- **Gain shape is the gap.** The aircraft has a resonance: \|H\| peaks at 1.5–1.6 at 2.5 Hz and rolls off above 3 Hz. The
  sim's gain keeps **rising to 1.54 at 6 Hz** — a heavily damped mode well above the band of interest.

The model file explains it. Short-period approximation from [`hb1_streamer.xml`](../../crrcsim/models/hb1_streamer.xml)
(`Cm_a −0.55`, `Cm_q −4.2`, `CL_a 4.7`, `Cm_de −0.36`, **`I_yy 0.0013`**, m 0.515, S 0.136, c̄ 0.178):

| | 11 m/s | 14 m/s | 17 m/s |
|---|---|---|---|
| **sim today**: f_n / ζ | 5.4 Hz / 0.81 | **6.9 Hz / 0.81** | 8.3 Hz / 0.81 |
| **aircraft** (bench steps § 2; FRF above) | 2.7 Hz | **~3.0 Hz / ζ ≈ 0.2–0.3** | 3.3 Hz |

`I_yy = 0.0013 kg·m²` is a 515 g airframe with a **5 cm** pitch radius of gyration — against `I_xx 0.024` and `I_zz 0.026`.
For a 76 cm span, 18 cm chord flying wing with the motor and battery along the chord, 9–12 cm (0.004–0.008 kg·m²) is the
plausible range. A 5–7× understated pitch inertia is exactly what makes the sim's short period twice the aircraft's frequency
and four times its damping: it peaks at 85 ms with the input and never rings (§ 7 of actuator-pin), while the aircraft peaks
at 133–166 ms and rings at ~3 Hz.

Why this is the thing that matters for the pitch cycle: the policy is a saturating relay (first-principles § 11). In flight it
locks onto the 2.5 Hz gain peak (\|H\| 1.6, loop gain 1.4–1.5 measured) and produces the ±200 °/s, ±7 g cycle. In the sim
there is no gain peak below 5 Hz, so the same relay's energy spreads to 5–10 Hz where a 20 Hz loop cannot organise a coherent
cycle — the "dithering" the t5 axis then rewarded (MANIFEST-t5 § 7a).

## 2. The proposed change — in order, each with its check

**Phase delay**: none to add to the plant. Fix the one the sim has in excess — apply the pending command at the 5 ms sub-step
instead of the 50 ms frame top (tasks T112a). That is a policy-sampling fix, not a plant fix; the FRF says plant phase already
matches to 2.5 Hz. Servo: v2 has slew only, no lag term; the matched phase says no large lag is missing below 2.5 Hz
(T046's bench step can confirm).

**Natural frequency and damping** — the actual change, three parameters and one constraint:

| step | parameter | from → to | what it sets | how it is pinned |
|---|---|---|---|---|
| 1 | **`I_yy`** | 0.0013 → **measured** (expect 0.006–0.010) | both f_n and ζ, ×1/√I and ×1/√I | **bifilar-pendulum measurement on article 1 AND article 2** (30 min each; the second article gives n = 2 for free). This is a measurement, not a fit — do it first. |
| 2 | **`Cm_a`** | −0.55 → ~−0.9 (with I_yy ≈ 0.008) | f_n and its growth with airspeed (f ∝ V) | the step-test / doublet frequencies 2.7 → 3.0 → 3.3 Hz at < 12 / 12–17 / ≥ 17 m/s (T095 doublets on both articles) |
| 3 | **`Cm_q`** (and `CL_a`) | −4.2 → ~−1.0 (…−0.5); `CL_a` 4.7 → 3.8 only if ζ cannot reach ~0.25 otherwise | ζ. ⚠️ The lift-curve term Z_α/V alone floors ζ at ≈ 0.26 at 3 Hz with `CL_a 4.7`; the aircraft's 0.18 (fast band, rough) may not be reachable without lowering `CL_a`, which also moves trim/performance — check against flight airspeed/throttle before accepting | log-decrement of the doublet ring-down; the closed-loop \|H\| peak 1.5–1.6 at 2.5 Hz |
| 4 | **`Cm_de`** | keep `Cm_de / Cm_a` ≈ constant (−0.36 → ~−0.59 at `Cm_a −0.9`) | steady pitch rate per elevator, i.e. the low-frequency \|H\| 0.48 that already matches | low-frequency FRF bins unchanged; arm-C full-stick rate (331 °/s linearised) unchanged |
| 5 | `CraftCmQSigma` | re-centre on the fitted `Cm_q`, re-clamp | the US5 variation class | T096 as written |

Candidate operating points (analytic, before any measurement — to show the target is reachable):

| I_yy | Cm_a | Cm_q | CL_a | f_n 11 / 14 / 17 m/s | ζ |
|---|---|---|---|---|---|
| 0.0060 | −0.55 | −4.2 | 4.7 | 2.5 / 3.2 / 3.9 | 0.59 |
| **0.0078** | **−0.90** | **−1.0** | 4.7 | **2.4 / 3.0 / 3.7** | **0.34** |
| 0.0078 | −0.90 | −0.5 | 3.8 | 2.3 / 3.0 / 3.6 | 0.26 |
| 0.0100 | −1.10 | −0.5 | 3.8 | 2.3 / 2.9 / 3.5 | 0.26 |

## 3. Validation ladder (no step skipped)

1. **Plant alone**: the step-test harness (`hb1_streamer_steptest.xml`, T093/T094) must give peak **133–166 ms**, overshoot
   ~2.5×, ring at ~3 Hz, frequency rising with airspeed — per cell, not pooled (T093's tool gap).
2. **Closed inner loop**: `frf_pitch_compare.py --sim` on an eval dmp of gen 488 must reproduce the flight column: \|H\| peak
   1.5–1.6 at ~2.5 Hz, roll-off above 3 Hz, phase −48° at 2 Hz, low-frequency bins unchanged.
3. **The policy test — the whole point**: gen 488 evaluated in the corrected sim should **show the 2 Hz cap-riding cycle**
   (`sim_pitch_spectrum.py`: 2–3 Hz share rising toward the flight's 31–63 %, cap % rising toward p5/p95-at-cap). If it does,
   the sim can finally price the behaviour and every measure already in the pool (rotation, energy, load) sees it. If it does
   not, the remaining candidates are the policy-sampling delay (T112a) and the servo (T046) — run T112's sweep then.
4. **Trainability gate** (T092 / T044 discipline) before any bake on the new plant.
5. **Flight on article 2 with gen 488** (T114): same frequency ⇒ loop-generic; shifted ⇒ airframe-specific — either way it
   bounds how far a single-article fit can be trusted.

## 4. What this does and does not claim

- It claims the dominant sim-to-real gap in pitch is the short-period placement, and names a first-order, *measurable* cause.
- It does not claim the numbers in § 2 are the fit; they show the target is reachable with physically plausible values. The
  fit comes from the inertia measurement and the doublets (T095) — on both articles.
- The bake that follows will produce a different policy; nothing about g488's flight numbers carries over except as the
  comparison baseline (the sim-to-real tracking gap was ~3×; the honest plant should shrink it and should remove the pitch
  cycle's invisibility).

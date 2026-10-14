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

## 2b. ⭐ The t6 plant — proposed 2026-10-13 from the provisional inertias (operator: "we have time for a full training run")

Measured on article 1 ([actuator-pin.md § Inertia](actuator-pin.md)): `I_yy` 0.0094 [0.0090–0.0095], `I_xx` 0.0069 (provisional,
[0.0068–0.0070]), flight mass 0.510 kg bare. Article 2 is not built, so the article spread is unknown; the streamer state of the
flight data is unknown; the doublets are not flown. So this is a **"one thing changes" plant**: pitch gets the measured inertia and a
short period placed in the aircraft's band; roll and yaw keep **exactly the dynamics t5 trained on** (their derivatives were tuned
against the wrong inertias and matched the aircraft's roll response — rescaling them by the inertia ratio preserves that match
bit-for-bit in the linear regime). One run, one flight, one comparison.

**`crrcsim/models/hb1_streamer.xml`** (and `hb1_streamer_steptest.xml`, which shares the block):

| block | key | now | **t6** | why |
|---|---|---|---|---|
| `mass_inertia` | `Mass` | 0.515 | **0.510** | measured |
| | `I_yy` | 0.0013 | **0.0094** | measured (bifilar) |
| | `I_xx` | 0.024 | **0.0069** | measured (provisional) |
| | `I_zz` | 0.026 | **0.016** | flat-body estimate `I_xx + I_yy`; not measured |
| pitch `m` | `Cm_a` | −0.55 | **−1.00** | places f_n at 2.3 / 2.9 / 3.5 Hz (11 / 14 / 17 m/s) vs aircraft 2.7 / 3.0 / 3.3 |
| | `Cm_q` | −4.2 | **−1.0** | with the real inertia `Cm_q` is a weak lever on ζ (0.32–0.38 over −0.3…−1.8); −1.0 is a physical mid value |
| | `Cm_de` | −0.36 | **−0.65** | keeps `Cm_de/Cm_a` ⇒ steady pitch rate per elevator unchanged (arm-C 331 °/s full stick; low-frequency \|H\| 0.48 already matches) |
| | `Cm_0`, `CL_a`, stall block | unchanged | unchanged | ζ lands at **0.35** with `CL_a 4.7`; the aircraft's 0.2–0.3 is of unknown streamer state — leave for the doublets (T095) rather than touch lift/trim now |
| roll `l` | `Cl_b, Cl_p, Cl_r, Cl_da` | −0.05, −0.47, 0.05, 0.22 | **× 0.2875** (= 0.0069/0.024) → −0.0144, −0.135, 0.0144, 0.0633 | preserves the roll time constant and steady rate t5 trained on (the composite 134 ms response matched flight) |
| yaw `n` | `Cn_b, Cn_p, Cn_r, Cn_da` | 0.07, −0.008, −0.04, −0.010 | **× 0.615** (= 0.016/0.026) → 0.043, −0.0049, −0.0246, −0.0062 | preserves yaw dynamics (unmeasured either way) |

⚠️ The rescaled roll derivatives are physically small (`Cl_p −0.135`); that is the sim telling us the aircraft's 134 ms roll rise is
probably actuator-dominated, not airframe-dominated — the servo/airframe split is T046's job, not t6's. Nothing in pitch depends on it.

**`include/autoc/eval/craft_variation.h`** — the pitch-damping class (operator: "pitch damping is a variation to add to craft
variations"; it exists, re-centre it): `kCraftCmQCenter` −4.2 → **−1.0**, `kCraftCmQMin/Max` [−5.0, −3.6] → **[−2.0, −0.2]**;
`CraftCmQSigma` 0.32 → **0.6** (2.5 σ reaches the clamps). ⚠️ With the real inertia this class spans only ζ ≈ 0.31–0.39 — honest but
narrow. The lever that actually spreads the mode is `Cm_a` (frequency) and `CL_a` (damping); a `CraftCmASigma` class is the right
addition once the doublets say what the spread is (BACKLOG, not t6). Compile-time constants ⇒ rebuild.

**Latency (T111/T112a)** — apply the pending NN command at the 5 ms physics sub-step instead of the 50 ms frame top. **Separate
commit, evaluated separately** (see the ladder): the plant change alone first, then the latency fix on top — two 20-second evals of
gen 488 give T112's answer without the sweep.

**Not in t6** (so the comparison stays clean): entry dither, course changes, pool changes, any objective change. t6 = t5's ini
(`ExcessRotationAxisMode = conditional`, same regiment, same seeds policy) on the new plant. The conditional rotation tie-break
stays — on a plant that cycles, it can finally see pitch.

### ✅ What was actually applied for t6 (2026-10-13 evening, after the ladder ran) — record in [actuator-pin.md](actuator-pin.md) § "The t6 plant"

| item | proposed above | **applied** | why it moved |
|---|---|---|---|
| `Mass`, `I_yy`, `I_xx`, `I_zz` | 0.510 / 0.0094 / 0.0069 / 0.016 | **as proposed** | — |
| `Cm_a` / `Cm_de` | −1.00 / −0.65 | **−0.55 / −0.36 (the original values)** | the closed-loop FRF of gen 488 chose it: −1.0 put the resonance at 3–4 Hz with \|H\| 2.6 (flight 2.5 Hz, 1.6); −0.55 lands peak height, cycle amplitude and frequency on the flight column. Only the inertia and `Cm_q` were wrong. |
| `Cm_q` | −1.0 | **−1.0** | — |
| roll `Cl_*` × 0.2875, yaw `Cn_*` × 0.615 | as proposed | **as proposed** | roll FRF of gen 488 unchanged to first order (verified) |
| `kCraftCmQCenter` −1.0, clamp [−2.0, −0.2], `CraftCmQSigma` 0.6 | as proposed | **as proposed** | tests re-pinned |
| sub-step latency (T112a) | separate, evaluated separately | **included** — evaluated separately first | it closed ~30 % of the amplitude gap and 10–15° of phase; and gen 488 tracks *better* with it (29 → 40 % `pctInStreak`), i.e. the policy had been fighting a tick it never had in the air |
| `Global::craftCmQ` default | — | **NaN (unset) + xml fallback in the FDM** | found by the ladder: the hard-coded −4.2 silently overrode the xml in every non-autoc run |
| step-test harness | — | `trimGain` 0.004, `trimSec` 6.0 | the old gain departed the new plant and overflowed the FDM |

Result of the sanity check (step 3 of the ladder, the flown genome against its own flights): **the sim now cycles like the aircraft** —
pitch-rate RMS 105–123 °/s (flight 124–153), peak 1.9–2.3 Hz (2.0–2.3), 2–3 Hz share 24–47 % (31–63 %), closed-loop \|H\| 1.34 at 2.5 Hz
(1.5–1.6), tracking degraded to 40 % with 3 crashes. Residuals: resonance ~0.5 Hz high, ~20° extra lag at 2–2.5 Hz, ring less sharp than
the bench steps (ζ ≈ 0.35 vs 0.2–0.3). Baselines on the t6 plant for the record: t3 genome −54922.067792 (was −88026.619367 on its own
plant), gen 488 −65928.363124 (was −95067.305736).

### Ladder before launch (hours, not days) and the flight sanity check

1. Apply the xml + class constants; operator `rebuild-perf.sh`; ctest. New eval baselines (the t3 gate number moves — record it).
2. **Step test** on the shared block (T093/T094 harness): pitch peak **133–166 ms** (was 85), overshoot ~2.5×, ring at ~3 Hz, frequency
   rising with airspeed. Roll step unchanged from t5's sim.
3. **Gen 488 in the new sim, eval mode** (`sim_pitch_spectrum.py`, `frf_pitch_compare.py --sim`): does the flown genome now show the
   **2 Hz cap-riding cycle** (2–3 Hz share toward the flight's 31–63 %, cap % up from 0–1 %) and does the closed-loop \|H\| peak land at
   ~2.5 Hz / 1.5–1.6? ⭐ **This is the sanity check of the sim against a flight we already have** — no new sortie needed for it. If the sim
   now reproduces g488's flight behaviour, the plant is close enough to train on. If not, stop; the remaining candidates are latency
   (step 4) and the resonance damping (`CL_a`).
4. Add the sub-step latency fix; repeat step 3. Keep it if it moves g488 toward the flight; T112's decision tree applies.
5. **T092 trainability smoke** (T044 recipe: basic-m1, pop 3000, single `longSequential`, 16 winds, ~30 gens) — a plant that rings
   must still be learnable.
6. Launch **t6** (`logs/autoc-043-t6-m1-plant.log`), MANIFEST first; early calls at gens 5 / 40 as in t5.
7. **Flight sanity check, decided after step 3 and the first t6 panels**: fly the t6 genome on article 1 with the g488 flights as the
   baseline (same scripts): pitch-rate RMS, 2–3 Hz share, setpoint at the cap, tracking error. If step 3 showed the sim cycling with g488
   and t6's genome does not cycle in the sim, the flight tells us whether "does not cycle in the honest sim" transfers — which is the
   feature's exit criterion (T118).

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

## 4. The streamer — how much accuracy the no-streamer craft needs (operator 2026-10-13)

The launch configuration tows a 20–25 ft crepe streamer that can be cut at any time, so the aircraft the policy flies is
*two* plants per sortie. The model is literally `hb1_streamer.xml` ("mimic the real test article towing a ~25 ft streamer";
`Cm_q −4.2` = "streamer damping", variation class `CraftCmQSigma` spanning −3.6 no-streamer … −5.0 streamer). ⚠️ Neither the
bench step tests (actuator-pin § 2) nor the 10-11 flight docs record the streamer state, so the 3 Hz / ζ 0.2–0.3 and the FRF
column above are of **unknown configuration** — most likely streamer-on (launch config). From here every span gets a streamer
flag (T119).

**What the streamer does, physically.** It is a trailing drag force D_s applied through the tail attachment at arm l_t behind
the CG; the streamer is not rigid, so it adds **no inertia** (`I_yy` is unaffected — measure it bare) and acts only through that
tension:

- static margin: ΔCm_α ≈ −D_s·l_t / (q̄ S c̄) — restoring (weathervane)
- pitch damping: ΔCm_q ≈ −2·D_s·l_t² / (q̄ S c̄²)
- trim: a rearward tension at the tail, plus whatever vertical offset the attachment has from the CG

With q̄ S c̄ = 2.5 N·m, q̄ S c̄² = 0.446 N·m², l_t ≈ 0.25 m and a crepe streamer's drag at 13 m/s somewhere in **1–3 N** (flapping
ribbons are draggy; the throttle delta before/after the cut will say), the streamer adds **ΔCm_α −0.1 … −0.3** and **ΔCm_q −0.3
… −0.8**. On the fitted plant of § 2 (`Cm_a −0.9`, `Cm_q −1.0`, ζ 0.34) that moves f_n by a few % and ζ by roughly **+0.03 …
+0.10** — the streamer-on aircraft is the *better-damped* one, the bare aircraft the *least* damped, i.e. the one the relay locks
onto most readily. The file's assumed 1.4-unit spread (−3.6 → −5.0) would need ~5 N of drag at the tail; that number predates
any measurement and should not be carried.

**Accuracy the bare craft actually needs.** The policy does not need an exact ζ; it needs the sim to present a resonance **in the
right band** (f_n within ~±15 %, i.e. 2.5–3.5 Hz at cruise, with the right growth with airspeed — that is `I_yy`, `Cm_a` and
mass, none of which the streamer touches) and a damping **range** whose *low end is the bare aircraft*. So:

- `I_yy`: measure bare, once per article, to ~±10 % — the frequency placement rests on it.
- ζ bare: know it to about **±0.05** (a log-decrement over 3 ring-down cycles gives that) — this sets the low end of the
  `CraftCmQSigma` class, which is where the hard case lives.
- ζ with streamer: the same flight, before the cut; its *difference* from bare is the measurement of D_s·l_t, and the class's
  high end. Precision beyond ±0.05 buys nothing: article-to-article spread (T114, n = 2) and airspeed already move ζ by that much.
- Re-centre `CraftCmQSigma` on the **bare** value and let the class span bare → streamer-on (plus the article spread), rather than
  centring on a streamer value with the bare case 2.5 σ out as today.

In short: the streamer is a bounded, directly measurable perturbation that belongs in the variation class; the thing that must
be *right* is the bare aircraft's mode placement, and "right" means band and slope, not the third decimal.

## 5. What this does and does not claim

- It claims the dominant sim-to-real gap in pitch is the short-period placement, and names a first-order, *measurable* cause.
- It does not claim the numbers in § 2 are the fit; they show the target is reachable with physically plausible values. The
  fit comes from the inertia measurement and the doublets (T095) — on both articles.
- The bake that follows will produce a different policy; nothing about g488's flight numbers carries over except as the
  comparison baseline (the sim-to-real tracking gap was ~3×; the honest plant should shrink it and should remove the pitch
  cycle's invisibility).

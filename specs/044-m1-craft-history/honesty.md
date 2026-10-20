# 044 — roll honesty ladder (FR-006, research R11, tasks T022–T024) — 2026-10-19

The plant change must reproduce the flown genome's real roll behaviour **before** any baseline or smoke is taken on it — the 043 pitch
ladder pattern. Build: US4 fixes + US2 plant (craftClP class with `Cl_da` scaled, CmQ re-centred, FC sensor filters at the sub-step).

## Verdict

**Rungs 2 and 3 reproduce the 2026-10-18 flight closely; rung 1 places the class where it was designed by the flight's own fit
method, with one criterion missed as worded and explained. Proceeding to baselines and smoke A** (operator may overrule — the deviation
is the steady-gain line below).

| rung | criterion (R11 / tasks) | result | |
|---|---|---|---|
| 1 corner | within ±20 % of 3 / 5 / 1.5 Hz at 15 m/s | flight-method FRF fit: **3.24 / 5.81 Hz** (+8 / +16 %); slow clamp by step fit **1.70 Hz** (+13 %) | ✅ |
| 1 steady gain | roll rate per aileron within ±5 % of t6 | `Cl_da/|Cl_p|` exact (unit test); full-FDM step gain **+13…+20 %** vs t6 | ⚠️ not met as worded |
| 1 saturation | rate loop unsaturated at the fast clamp | surface output not recorded; policy roll-command cap 11 % (fast) / 8 % (mid) / 3.6 % (slow) on untrained g752 | ⓘ |
| 2 FRF | recorded, not gated | 2.5–5 Hz closed-loop roll \|H\| **0.82–0.92** vs flight 0.8–1.0 (t6 plant 0.45–0.58) | ✅ |
| 3 roll share | higher on the new plant, calm and all | 3–5 Hz share **9 → 36 %** (all), **5 → 34 %** (calm); 3–5 Hz roll RMS **27 → 112 / 19 → 96 °/s** vs flight **97** | ✅✅ |

## (1) Step test (T022)

Standalone `Cntrl_StepTest` (`crrcsim/steptest_config.xml`, servo model off → bare airframe), temporary copies of
`hb1_streamer_steptest.xml` with `Cl_p` set and `Cl_da = 0.0633 · Cl_p / −0.135` (exactly what `craftRollCoefficients` gives the FDM);
5 roll cells per model at 12–22 m/s. `roll_step_corner.py` fits each cell two ways; corners ∝ V, so quoted at 15 m/s.

| `Cl_p` | formula corner | step fit (first order, no delay) | flight-method FRF fit (first order + free delay, 0.5–8 Hz) | step gain K (deg/s per unit aileron) |
|---|---|---|---|---|
| −0.135 (t6) | 1.13 Hz | 1.35 Hz | ill-conditioned (corner below the window; residual 0.27) | 940 @ 17.7 m/s |
| −0.18 (slow clamp) | 1.51 Hz | **1.70 Hz** | 2.87 Hz (residual 0.08 — poor) | 1103 @ 15.5 m/s |
| −0.36 (centre) | 3.01 Hz | 3.83 Hz | **3.24 Hz**, 8 ms delay (residual 0.00) | 1127 @ 18.1 m/s |
| −0.60 (fast clamp) | 5.02 Hz | 6.75 Hz | **5.81 Hz**, 4 ms delay (residual 0.00) | 1062 @ 15.9 m/s |

- The class moves the corner in proportion to `Cl_p` as designed (step-fit ratios to t6: 1.26 / 2.84 / 5.0 vs `Cl_p` ratios 1.33 /
  2.67 / 4.44). The full FDM runs 20–35 % faster than the `Cl_p`-only formula on a step fit (other lateral derivatives contribute);
  by the fit the flight's ~5 Hz came from, centre and fast clamp land within +8 / +16 %.
- **Steady gain**: `p_ss = Cl_da·2V/(|Cl_p|·b)` is held exactly (unit test `RollCoefficientsKeepSteadyRollRatePerAileron_044`), but the
  full 6-DOF step gain rises 13–20 % over t6 at similar airspeeds (lateral–directional coupling: with a slower roll mode, sideslip and yaw
  build further before the rate settles). The change moves toward the aircraft: the 2026-09-07 MANUAL flight measured ~1330 deg/s per unit
  aileron at a median 14.3 m/s (actuator-pin.md § 7); t6 scales to ~760 there, the centre to ~890. **Not re-tuned** — `Cl_da` scaling
  keeps the design invariant; the residual is physics the flight also has.

## (2) Closed-loop roll FRF of t6 g752 vs the 2026-10-18 flight — recorded, not gated (T023)

`roll_frf_compare.py`; setpoint → roll rate; flight `axisRate[0]` → `gyroADC[0]` at 240 Hz over the seven engaged spans; sim `rollCmd`
× 360 → `gyrP` at 20 Hz, median over scenarios, departed excluded.

| f (Hz) | flight \|H\| (typical span) | t6 plant | 044 plant |
|---|---|---|---|
| 2.5 | 0.72–0.96 | 0.58 | **0.92** |
| 3.0 | 0.80–1.05 | 0.58 | **0.90** |
| 3.5 | 0.86–1.00 | 0.59 | **0.92** |
| 4.0 | 0.76–0.98 | 0.53 | **0.83** |
| 5.0 | 0.51–0.86 | 0.45 | **0.82** |

Phase: flight −100° (3 Hz) / −127° (5 Hz); both sims lag a further ~35–50°, largely the logging point (the sim's setpoint is the 20 Hz
command before the 10 ms compute latency and ZOH; the flight's is the FC's 240 Hz `axisRate` after them). Magnitude is the honest
comparison and the 044 plant matches it.

## (3) Equal-environment eval of t6 g752 (T024) — gate

Seed 20261018, full strength. t6 arm = the 043 T122 candidate run (043 build = `043-final`, scale patched to 1.0); 044 arm =
`eval_suite.sh … 0 20261018 1.0` on this build (draws differ by construction — 294-scenario medians). `roll_share.py`.

| plant | subset | n | roll RMS | 3–5 Hz share | 3–5 Hz roll RMS | departed |
|---|---|---|---|---|---|---|
| t6 (`043-final`) | all | 293 | 96 | 9 % | 27 | 1 |
| t6 (`043-final`) | calm | 106 | 87 | 5 % | 19 | |
| **044 plant** | all | 290 | 186 | **36 %** | **112** | 4 |
| **044 plant** | calm | 115 | 168 | **34 %** | **96** | |
| flight 2026-10-18 | — | 7 spans | | ~50 % | **97** | |

**Gate passes**: the plant now charges the roll dither the aircraft showed — the untrained g752 dithers in roll on it at the flight's
magnitude. (Tracking on the new plant: score 256 → 203, streak 87 → 75 %, completion 290 → 291/294 — the genome was trained on the
slow roll mode.)

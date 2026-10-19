# 044 — M1 craft-sensor history + roll plant (seed, 2026-10-18)

**Status: not started.** Created at the 043 wrap (operator 2026-10-18): *"we talked about the next one being an M2 run but we prob
should do one more M1 with roll adjustments (probably not the full way) and gyro/accel history … ok to rearrange the NN inputs so they
are adjacent … ok to rename 044 to not be M2."* M2 moved to [045](../045-m2-tracking/README.md). Predecessor:
[043 outcome](../043-acro-dual-loop/outcome.md).

## Why

The 043 exit flight (2026-10-18, t6 gen 752) halved the 2 Hz pitch cycle once the sim's pitch plant was honest — and showed two things
left: (1) the remaining pitch excess is **3–5 Hz command dither** (pitch 3–5 Hz RMS 29 → 56 °/s while 2–3 Hz fell 103 → 51 °/s), and
(2) **roll oscillation rose** (3–5 Hz roll share 26 → 50 %) because the sim's roll mode is ~4.5× too slow, so selection never paid for
roll dither. 043's analysis ([policy-temporal-view.md](../043-acro-dual-loop/policy-temporal-view.md) §§ 4–8) also found that the
policy has **no history of any craft sensor**; its memory holds its previous command but not what the airframe did, and the rate
derivative — the damping term — is mostly unavailable to it. 044 gives the policy that signal and fixes the plant term that hid roll.

## Requirements

### R1. Craft-sensor history in the input vector

| | decision | evidence |
|---|---|---|
| signals | **gyro p, q, r and accel z** | gyro: the damping term on all three axes (roll now first-class; yaw adds a little to pitch prediction through coupling). accel z: the short-period's α state (pitch rate + a_z *now* already predict the rate 100 ms ahead as well as two gyro samples). |
| not accel x / y | **no history** | in flight they alias 17–35 % of a 50 ms difference (the FC's 15 Hz accel filter passes vibration the 20 Hz sample folds down) — the sim has none, so it would be a sim-to-real gap fed straight into the inputs; and they predict nothing (accel y: lateral specific force σ 0.009; accel x: slow speed changes already carried by airspeed and Es). |
| lags | **50 and 100 ms**, a craft-only table `{100, 50, 0}` | first 50 ms lag carries most of the gain (flight pitch R² 0.11 → 0.54); 100 ms is needed for roll (0.09 → 0.19) and in sim for pitch; 150 / 200 ms add ≤ 0.02. Physics: for a 2.5–3 Hz pitch mode 50 ms ≈ 50°, 100 ms ≈ quadrature; for 3–5 Hz roll dither 50 ms = 54–90° (the useful one), 100 ms is near-inverted at 5 Hz; 200 ms is a half period and 400 ms an identical copy at 2.5 Hz. |
| explicit command history | **no** | the recurrent state holds the previous command exactly (o₍t−1₎ = tanh(W₃ h₍t−1₎ + b)); t−2/t−3 add +0.01–0.03. |
| cost | +8 slots → **53 M1 / 74 M2 inputs**, +256 weights (2307 → 2563) | xiao: gyro / accel already arrive every tick (local ring buffer, no link cost); forward pass +~1 %. |

**Will we regret two lags and four signals?** Unlikely, and cheap to revisit: the measured gain beyond {50, 100} is ≤ 0.02–0.04 R²,
accel x/y would add noise, and the recurrent layer extends memory where needed. The lag table and the signal list must each be **one
constant** so a later change is a one-line edit plus the (unavoidable) format break. Revisit trigger: the 044 smoke shows the slots
used heavily at 100 ms (gain still rising at the last lag), or a future airframe adds yaw control (then accel y earns a look).

Prediction carried from 043 (§ 8): at equal tracking, ~10–15 % less 1.5–4 Hz pitch rate and 25–70 % calmer commands; in a bake, the
smooth regime sooner and a lower dither floor rather than a big RMS drop. **Falsifier**: a plant+history smoke that is not faster than
a plant-only smoke to the same smoothness.

### R2. Input layout clean-up (format break; allowed — 044 starts new baselines anyway)

History sits **next to its current value, oldest first**, as the target arrays already do. Proposed 28-float craft block (shared by M1
and M2): `quat w,x,y,z · airspeed · gyro_p[3] · gyro_q[3] · gyro_r[3] · accel_x · accel_y · accel_z[3] · specific_energy ·
boundary_closure_rate · dist_to_boundary · inward x,y,z · score_grad x,y,z` (names `GYRO_Q_TM2 / _TM1 / GYRO_Q`, display `gyrQ-2`,
`gyrQ-1`, `gyrQ`). Code scan 2026-10-18 — 39 files reference the layout:

- **Follow the enum automatically** (verify, don't rewrite): `nn_inputs.h` (struct, both enums, `CRAFT_COMMON_INPUT_META`, asserts),
  `topology.h` (weight-count assert), `evaluator.cc` (gather), `aircraft_state.h` (history ring — extend to craft sensors, recorded at
  the same point as the target history, sim and xiao), `mode.{h,cc}`, `input_mask.h`, `craft_observations.h`, `envelope_state.h`,
  `tracker_stepper.h`, `inputdev_autoc.cpp`, `autoc.cc` (data.dat header), `dmp_dump.cc`, `nn2cpp.cc`, `renderer.cc`,
  `xiao/include/flight_log_format.h` (scales indexed by name — add 8 entries, **bump the log version**), `xiao/src/msplink.cpp`.
- **Hand-maintained copies — must be edited**: `src/analytics/flightlog_decode.py` (`INPUT_NAMES`, length-checked),
  `xiao/web/flight_logger.html` (BLE downloader, the third decoder), `src/analytics/input_investment.py`.
- **Tests with counts / indices** (14): `nn_layout`, `nn_sensor_interface`, `shared_input_block`, `contract_evaluator`, `nn_evaluator`,
  `nn2cpp_unroll`, `input_mask`, `gather_tracker_inputs`, `flightlog_roundtrip`, `recording_fidelity`, `arena`, `predictor_head`, …
- **043 analysis scripts** assume 45 inputs at fixed offsets — port them to read columns by name.
- **Historical scripts** (023/024/027/029/037/039) stay as they are; they read old logs.
- ⛔ Before the break (the FR-057 lesson): keep a readable copy of what the new build cannot read — tag the last 45-input commit, and
  archive tick CSVs + eval results of the flown genomes (t5 g488, t6 g752) and the t6 final.

### R3. Roll plant — partly toward the flight, as a variation class

Flight: roll subsidence corner **~5 Hz** at 15 m/s; model 1.1 Hz (`Cl_p` −0.135 at `I_xx` 0.0069). Corner ∝ |Cl_p|/I_xx, so at the
measured `I_xx`: 1.5 Hz ↔ −0.18, **3 Hz ↔ −0.36**, 5 Hz ↔ −0.60. Operator: *"probably not the full way"* → a **`CraftClP` class**
like `CraftCmQ`: centre ≈ −0.36 (3 Hz), clamp ≈ [−0.18, −0.60] so training spans the old sim and the flight. ⚠️ **Scale `Cl_da` with
`Cl_p`** so the steady roll rate per aileron (p_ss = Cl_da·2V/(|Cl_p|·b)) is unchanged — the INAV roll feed-forward was tuned on the
real aircraft and the closed-loop roll gain matched before; only the mode's speed is wrong. Honesty ladder before any bake (as pitch in
043): step test; closed-loop roll FRF vs the 10-18 flight; **gen 752 on the new plant shows its flight roll 3–5 Hz rise**.

### R4. Pitch damping class spans the streamer

Both 10-18 flights towed the full streamer; the flight fit gives streamer-on `Cm_q` ≈ −3.5 against the bare −1.0 the t6 class is centred
on (clamp −2.0 … −0.2). Re-centre `CraftCmQ` to span bare → streamer-on (≈ −0.5 … −4), weighted toward how it is flown.

### R5. Sim fidelity rider: FC sensor filter delays

Model the FC's gyro PT1 25 Hz (~6.4 ms) and accel biquad 15 Hz (~21 ms) at the 5 ms FDM sub-step (`sensor_lpf.h` option a). With
accel z differences becoming inputs, the 21 ms accel lag is no longer negligible.

### R6. Baseline-boundary fixes (044 starts new baselines, so land them here)

- **Scenario PRNG independence**: class streams are shifted copies of one Park–Miller sequence (BACKLOG); hash sub-seeds before
  seeding `ClassPRNG`; unit test that no class stream contains another's sub-seed.
- **Log the wind-envelope draws** (speed, turbulence, gust length, thermal) in the startup scenario table.
- **Explicit eval variation scale** (ini key) instead of reusing the weights file's save-time ramp value — 043 T122 had to patch copies
  of the weights (`set_eval_scale.py`) to compare generations on equal air.

### R7. Procedure carried from 043

Equal-environment candidate selection (T122 method) before any flight; FC config frozen to the sim's gains (diff the CLI dump);
blackbox 1/8 with SERVOS + ATTI (the airframe ID needs servos); streamer state per span (T119); boot level and still; flash erased.

## Exit

A flight of the 044 genome against the 10-18 t6 flight, same streamer and FC config, scripts of record (`candidate_compare.py`,
`airframe_id.py`): absolute **3–5 Hz pitch RMS and 3–5 Hz roll RMS down**, 2–3 Hz pitch RMS no worse, tracking no regression, no new
crash mode. Decide the `|a_z|` p99 line before the bake (043 showed it does not move with pitch smoothing at this tracking aggression).

## Order (each gates the next)

1. Archive / tag for the format break (R2 ⛔). 2. Plant on the 45-input build: R3 + R4 + R5 with the honesty ladder. 3. R6 fixes →
new eval baselines, determinism gates. 4. R1 + R2 format break (xiao, log version, decoders, tests). 5. Two smokes — plant-only (45
inputs) vs plant + history (53) — for attribution and the falsifier. 6. Bake `autoc-044-t1-…`. 7. Candidate selection, flight, analysis.

# 044 — M1 craft-sensor history + roll plant (requirements draft, 2026-10-18)

**Status: not started — requirements for operator review.** Created at the 043 wrap. Operator: *"one more M1 with roll adjustments
(probably not the full way) and gyro/accel history … ok to rearrange the NN inputs so they are adjacent … the craft is already pretty
smooth and we do want to get going again on M2 based on all the M1 updates, AND based on real-world camera work in the other repos."*
So 044 is deliberately **lean: one bake, one flight**, then [045 M2](../045-m2-tracking/README.md). Predecessor:
[043 outcome](../043-acro-dual-loop/outcome.md).

## Decisions already made (not reopened in 044)

- **INAV stays the bridge** (operator 2026-10-18): failsafe, estimation (AHRS / GPS / baro), pilot modes, logging and the ACRO fast
  loop remain on INAV through 044 and 045. Embedding GPS / IMU / the fast loop on the companion (xiao or successor) is a later
  hardware step that should also shorten the lag; no ACRO-vs-direct experiment in 044.
- **Objective, selection and regiment are unchanged from t6** (conditional excess-rotation tie-break, 294 scenarios, wind envelope,
  ramp 40) — 044 changes what the policy sees and what the plant does, nothing else, so the flight compares against t6 gen 752.
- **FC config of record is frozen** at `xiao/inav-hb1.cfg` 91abd4d (the 10-18 flight).

## Why

The 043 exit flight (t6 gen 752, full streamer) halved the 2 Hz pitch cycle once the sim's pitch plant was honest, and left two
things: (1) the remaining pitch excess is **3–5 Hz command dither** (3–5 Hz pitch RMS 29 → 56 °/s while 2–3 Hz fell 103 → 51 °/s);
(2) **roll oscillation rose** (3–5 Hz roll RMS 58 → 97 °/s) because the sim's roll mode is ~4.5× too slow, so selection never paid for
roll dither. And the policy has **no history of any craft sensor**: its memory holds its previous command, not what the airframe did,
so the damping term is mostly out of its reach ([policy-temporal-view.md](../043-acro-dual-loop/policy-temporal-view.md) §§ 4–8).

## Functional requirements

### Inputs

- **FR-001 Craft-sensor history.** The input vector carries **gyro p, q, r and accel z at 100 ms and 50 ms** in addition to their
  current values (+8 slots: M1 45 → **53**, M2 66 → **74**; weights 2307 → 2563). The signal list and the lag table
  (`{100, 50, 0}` ms, a craft-only table separate from the target's `{800 … 0}`) are each **one constant**.
  *Accept*: slot counts and weight count asserted; changing a lag or adding a signal is a one-constant edit plus the format break.
  *Why these*: 50 ms carries most of the gain on both axes, 100 ms is needed for roll; longer lags added ≤ 0.02–0.04 R²; accel x/y
  alias 17–35 % on a 50 ms difference in flight (a sim-to-real gap) and predicted nothing; the recurrent state already holds the
  previous command exactly, so no command history.
- **FR-002 One recording point, both sides.** Craft history is recorded at the same point as the target history (before the NN
  runs, once per 20 Hz tick) by the same code in the sim and on the xiao, from the same values the gather writes; at engage / scenario
  start it is pre-filled with the current reading, as the target history is. *Accept*: a unit test drives a known sequence and checks
  every lag slot; sim and xiao gathers produce identical vectors from identical state.
- **FR-003 Layout clean-up.** Each sensor's history sits **next to its current value, oldest first** (as the target arrays do). The
  shared 28-float craft block (M1 and M2): `quat w,x,y,z · airspeed · gyro_p[3] · gyro_q[3] · gyro_r[3] · accel_x · accel_y ·
  accel_z[3] · specific_energy · boundary_closure_rate · dist_to_boundary · inward x,y,z · score_grad x,y,z`; enum names
  `GYRO_Q_TM2 / GYRO_Q_TM1 / GYRO_Q`, display `gyrQ-2 / gyrQ-1 / gyrQ`. Because M2 shares this block, it is designed now so 045 does
  not need a second craft-block break. *Accept*: one metadata table drives every name; the two enums' shared tails asserted identical.
- **FR-004 Format break, completely.** Every consumer moves in one change (code scan 2026-10-18, 39 files):
  - automatic via the enum (verify): `nn_inputs.h`, `topology.h`, `evaluator.cc`, `aircraft_state.h`, `mode.{h,cc}`, `input_mask.h`,
    `craft_observations.h`, `envelope_state.h`, `tracker_stepper.h`, `inputdev_autoc.cpp`, `autoc.cc`, `dmp_dump.cc`, `nn2cpp.cc`,
    `renderer.cc`, `xiao/src/msplink.cpp`;
  - `xiao/include/flight_log_format.h`: 8 scale entries by name and a **log-version bump** (old decoders must refuse new logs);
  - hand-maintained name lists: `src/analytics/flightlog_decode.py` (`INPUT_NAMES`), `xiao/web/flight_logger.html`,
    `src/analytics/input_investment.py`;
  - the 14 tests with counts / indices; the 043 analysis scripts ported to read columns by name;
  - historical scripts (023 … 039) untouched — they read old logs.
  *Accept*: clean `rebuild-perf.sh`, all tests pass, xiao builds, a bench flight log round-trips through both decoders.
- **FR-005 Preserve what the new build cannot read.** Before the break: tag the last 45-input commit; archive tick CSVs and eval
  results for t5 g488, t6 g752 and t6 g800. *Accept*: those artifacts readable from the archive or by checking out the tag.

### Plant

- **FR-006 Roll mode, partly toward the flight, as a class.** New `CraftClP` variation class (like `CraftCmQ`): centre `Cl_p` ≈ −0.36
  (roll corner ~3 Hz at 15 m/s with `I_xx` 0.0069), clamp ≈ [−0.18, −0.60] (1.5 … 5 Hz; the 10-18 flight sits near 5 Hz). **`Cl_da`
  scales with `Cl_p`** so the steady roll rate per aileron (p_ss = Cl_da·2V/(|Cl_p|·b)) is unchanged — only the mode's speed was wrong,
  and INAV's roll feed-forward was tuned on the real aircraft. *Accept (honesty ladder, before any bake)*: step test; closed-loop roll
  FRF vs the 10-18 flight; **t6 gen 752 on the new plant shows a higher 3–5 Hz roll share** than on the t6 plant, moving toward the
  flight's 50 %.
- **FR-007 Pitch damping spans the streamer.** Re-centre `CraftCmQ` to cover bare → streamer-on (≈ −0.5 … −4, centre ≈ −2; the 10-18
  airframe fit gives streamer-on ≈ −3.5, the t6 class covered −2.0 … −0.2). *Accept*: class range and centre recorded with the fit.
- **FR-008 FC sensor filter delays.** Model the FC's gyro PT1 25 Hz (~6.4 ms) and accel biquad 15 Hz (~21 ms) at the 5 ms FDM sub-step
  (`sensor_lpf.h` option a). *Accept*: step input shows the expected group delays; g752 eval moves by a recorded, small amount.

### Baseline-boundary fixes (044 starts new baselines anyway)

- **FR-009 Independent variation streams.** Hash each class sub-seed before seeding `ClassPRNG` (class streams are currently shifted
  copies of one Park–Miller sequence). *Accept*: unit test — no class stream contains another class's sub-seed in its first N draws.
- **FR-010 Log the wind-envelope draws** (speed, turbulence, gust length, thermal) as columns of the startup scenario table.
- **FR-011 Explicit eval variation scale.** An ini key sets eval's variation scale (default: the weights file's stored value, as
  today) so generations can be compared on equal air without patching weights.

### Process

- **FR-012 Attribution smokes.** Two smoke runs on the new plant before the bake: plant-only (45 inputs) and plant + history (53).
  *Falsifier for FR-001*: plant + history is not faster to the same pitch / roll smoothness.
- **FR-013 One bake** (`autoc-044-t1-…`), t6 regiment; MANIFEST before launch; PNG set gains **roll** panels (cap %, flips/s, 3–5 Hz
  share) next to pitch, since roll is now an exit measure.
- **FR-014 Flight candidate by equal environment** (T122 method: fixed never-trained seed, full variation strength), then the 043
  flight procedure: frozen FC config, blackbox 1/8 with SERVOS + ATTI, streamer state per span, level boot, flash erased.

## Success criteria (the 044 flight vs the 10-18 t6 gen-752 flight, same streamer and FC config, scripts of record)

- **SC-001** 3–5 Hz **roll** RMS down (baseline 97 °/s).
- **SC-002** 3–5 Hz **pitch** RMS down (baseline 56 °/s), 2–3 Hz pitch RMS not worse (51 °/s).
- **SC-003** Tracking no regression (median distance to rabbit 9–14 m); no new crash or failsafe mode.
- **SC-004** The roll honesty check holds in flight: `airframe_id.py` roll corner lies inside the `CraftClP` class range.

## Non-goals

M2 / camera (→ 045, which also absorbs the camera work from the other repos); any INAV change; objective or selection changes;
article 2 (fly it only if it is ready and its FC config is frozen to the same gains).

## Open questions for the operator

1. Roll class centre at ~3 Hz ("not the full way") — or closer to the flight's 5 Hz?
2. The \|a_z\| p99 < 5 g line: drop it, raise it, or keep it as a watched number (043 showed it does not move with pitch smoothing)?
3. Full 800-gen bake, or shorter to reach M2 sooner (t6 reached its best equal-environment elite late, gen 750)?
4. Should accel x/y history stay out even for M2, given the tracker shares the craft block (FR-003)?

# 044 — Phase 0 research

Each item: **Decision / Rationale / Alternatives considered.** Evidence for the requirement-level choices (signals, lags, roll corner,
thresholds) is in [README.md](README.md) and the 043 documents it cites; this file resolves the *how*.

## R1. Roll damping class (FR-006)

- **Decision**: a `craftClP` axis built exactly like `craftCmQ`: absolute value, Gaussian about a centre, clamped; drawn in autoc from
  the craft PRNG, carried in `ScenarioMetadata` (appended), applied in `fdm_larcsim` as `Cl_p = craftClP` (NaN-unset → xml nominal, as
  `craftCmQ` does). Centre **−0.36** (roll corner ≈ 3 Hz at 15 m/s with `I_xx` 0.0069), clamp **[−0.60, −0.18]** (≈ 5 … 1.5 Hz), σ
  **0.10** (2.4 σ reaches either clamp). **Aileron authority scales with it**: `Cl_da = Cl_da_ref · (craftClP / Cl_p_ref) · (1 +
  craftRollEffDelta)` with `Cl_p_ref` −0.135, `Cl_da_ref` 0.0633 (the t6 pair), so p_ss = Cl_da·2V/(|Cl_p|·b) is unchanged for every
  draw and the existing roll-effectiveness variation still applies on top.
- **Rationale**: reuses a proven pipeline (draw → metadata → worker → FDM, logged in the scenario table, recorded in the dmp); the
  corner is linear in |Cl_p|/I_xx (L_p = q̄ S b² Cl_p / (2 V I_xx)), so the class maps directly to the measured 1.1 → 5 Hz span.
- **Alternatives**: a fixed `Cl_p` at 3 Hz (rejected — the class also buys robustness across the uncertain span, and the flight sits at
  its edge); changing `I_xx` instead (rejected — `I_xx` is provisional and inertia also moves cross-coupling; the flight fixes the
  ratio); not scaling `Cl_da` (rejected — would cut sim roll rate per aileron ~2.7× and break the parity with INAV's roll FF).

## R2. Pitch damping class re-centre (FR-007)

- **Decision**: `kCraftCmQCenter` **−2.25** (midpoint of bare −1.0 and streamer-on −3.5), clamp **[−3.5, −1.0]**, `CraftCmQSigma`
  **0.5** (2.5 σ reaches either clamp).
- **Rationale**: the clarification fixes the midpoint and the span; the flight fit (`airframe_id.py`, free `Cm_q` at `I_yy` 0.0094) gives
  streamer-on ≈ −3.5; the t6 class covered −2.0 … −0.2. Tests pinning the old constants are re-pinned to the named constants.
- **Alternatives**: centre on streamer-on (operator chose the midpoint); bimodal bare/streamer (not chosen).

## R3. FC sensor filter delays (FR-008)

- **Decision**: run INAV-equivalent filters at the **5 ms FDM sub-step** inside crrcsim: gyro PT1 25 Hz on body rates, accel biquad
  15 Hz (Q 0.707) on body specific force, updated from the existing per-sub-step hook (`autoc_apply_pending_substep`, called every
  sub-step); the 20 Hz gather reads the filtered values. Filter state resets with the scenario and seeds from the first sample (no
  start transient), per `sensor_lpf.h` (option a, its classes already written and kept for this).
- **Rationale**: mirrors the real chain (filter at the fast rate, then sample) — the 2026-09 attempt failed only because it filtered at
  the 20 Hz gather. Header-only classes in an already-linked target: no CMake change expected (if one appears, Constitution IV's clean
  `rebuild-perf.sh` applies).
- **Alternatives**: pure delays folded into the staged-command latency path (rejected — sub-tick delay on sensors, not commands, and a
  filter also shapes noise); filtering in autoc at the gather (rejected — the documented Nyquist failure).

## R4. Independent variation streams (FR-009)

- **Decision**: in `deriveClassSubSeeds`, pass each root output through a **32-bit mixing finalizer** (murmur3 `fmix32`) before it
  becomes a class sub-seed, then apply the zero guard. One function, shared by autoc and crrcsim (header), so both sides agree.
- **Rationale**: Park–Miller output = state, so seeding a class stream with a root output continues the root sequence; any bijective
  mixer breaks the overlap without changing generator, period or determinism. Test: for 10 000 scenario seeds, no class's first 64
  draws contain another class's sub-seed or first draw.
- **Alternatives**: a different generator for class streams (larger change, same effect); offsetting streams by skipping N draws
  (still one sequence — only delays the overlap).
- **Consequence**: every scenario draw changes ⇒ new eval baselines; cross-version replay is not expected (replay-scope rule).

## R5. Wind-draw logging (FR-010)

- **Decision**: autoc computes each scenario's wind draws with the same pure functions the worker uses
  (`deriveClassSubSeeds` → `ClassPRNG(wind)` → skip the gust-seed draw → `drawWindVariation`) and adds four full-strength columns to the
  startup scenario table: `windMps turbX gustLx thermX`.
- **Rationale**: the draws are deterministic functions of data autoc already holds; no RPC change. (043's `wind_draws.cc` does the same
  offline and becomes unnecessary.)

## R6. Explicit eval variation scale (FR-011)

- **Decision**: ini key **`EvalVariationScale`** (double, default **−1** = use the weights file's stored scale, today's behaviour;
  0…1 overrides). Logged at eval start next to the stored value; startup rejects values outside {−1} ∪ [0, 1].
- **Rationale**: replaces 043's weights-copy patch (`set_eval_scale.py`); default preserves every existing eval and the tier-0 bitwise
  gate. Adds one key to the config contract count.

## R7. Craft history recording (FR-001, FR-002)

- **Decision**: a craft ring in `AircraftState` beside the target ring, holding body rates (p, q, r) and specific force z per tick,
  depth `kNNCraftHistoryLagsMsec[0] / SIM_TIME_STEP_MSEC + 1` (= 3). **`recordErrorHistory` grows a craft-sample argument** so both
  call sites (crrcsim `inputdev_autoc`, xiao `msplink`) must change together — no second function that one side could forget
  (no-fallback rule). `resetHistory` pre-fills from the current reading. The M2 stepper paths (`tracker_stepper`,
  `crrcsim_tracker_helper`) record the same ring, since the craft block is shared.
- **Rationale**: one recording point and one code path on both targets is what makes FR-002 testable; the values recorded are the
  ones the gather already uses (post-filter in sim per R3, FC-filtered on the aircraft).

## R8. Layout and versions (FR-003, FR-004)

- **Decision**: craft block 20 → 28 floats in the order of spec FR-003; enum names `GYRO_P_TM2, GYRO_P_TM1, GYRO_P, …, ACCEL_Z_TM2,
  ACCEL_Z_TM1, ACCEL_Z`; `CRAFT_COMMON_INPUT_META` regenerated; M1 53 inputs / 2563 weights, M2 74 / 3235. Version bumps:
  `EvalResults::kSchemaVersion` 4 → **5** (metadata gains `craftClP`, ticks carry the wider input vector), flight log `kFormatVersion`
  5 → **6** (8 new scale entries by name), NN input layout marker bumped; all readers fail loud on the old version (Constitution V).
- **Rationale**: a single format break carrying every layout change; name-indexed consumers follow automatically, the three
  hand-maintained name lists are edited in the same change (README code scan).

## R9. Attribution smokes (FR-012, SC-006)

- **Decision**: both arms on the new plant **and** the new PRNG (R1–R4 land first), same seed, `autoc.ini` regiment scaled down:
  population 3000, 6 paths × 16 winds (96 scenarios), **200 generations**, variation ramp step 10. Arm A = last 45-input build (plant
  only); arm B = after the layout break (plant + history). Metric per generation from the elite's tick dump: median 3–5 Hz roll-rate
  RMS, median pitch-rate RMS and 3–5 Hz share, `pctInStreak`. **SC-006 outcome** = the first generation at which arm B's elite is at or
  below arm A's best-of-run smoothness on both axes with `pctInStreak` ≥ arm A's at the same generation; recorded either way (the bake
  keeps history regardless — clarification 1).
- **Rationale**: same air for both arms; 200 gens covers t6's takeoff and relay-shedding phase (relay came off by gen ~100–150 at
  full scale); 96 scenarios keeps each smoke to roughly a day on the box.
- **Alternatives**: full-regiment smokes (too slow for an A/B); basic-m1 single path (too narrow to see roll on random courses).

## R10. Profiling (FR-015)

- **Decision**: (a) **per-tick cost A/B** — time a fixed tier-1 eval of one genome with `perf stat` / `perf record` on a worker, with
  and without the INAV rate-loop controller node in the model (same build), and with the per-tick `EvalTick` payload trimmed in a
  throwaway build; (b) **parent profile** — `perf record -g` on autoc across one training generation (smoke config), attributing time
  to deserialize / scoring / selection / dmp write. Record both in `profile.md`. Adopt only changes that leave a tier-0 replay bitwise
  identical (e.g. overlapping the dmp upload with the next generation's dispatch); everything else → BACKLOG.
- **Rationale**: answers the open t2 → t3 +47 % per tick question and the parent's 161 % CPU / 25 s serial phase, without risking
  results before a bake.

## R11. Roll honesty ladder (FR-006 acceptance)

- **Decision**: three checks on the 45-input build before the smokes: (1) **step test** (`Cntrl_StepTest`) roll steps at 15 m/s at
  class centre and both clamps → corner ≈ 3 / 5 / 1.5 Hz and steady roll rate per aileron equal to the t6 plant's; (2) **closed-loop
  roll FRF** of g752 (setpoint → roll rate) on the new plant vs the 10-18 flight blackbox; (3) **g752 equal-environment eval** (fixed
  seed, scale 1.0) on t6 vs new plant: 3–5 Hz roll share rises toward the flight's 50 %.
- **Rationale**: the 043 pitch ladder pattern — prove the plant reproduces the known genome's real behaviour before training on it.

## R12. Preservation before the break (FR-005)

- **Decision**: tag the last 45-input commit **`044-last-45in`** (local tag; operator pushes); pin `retain=keep` on the 043 T122 /
  exit-flight eval dumps in `autoc-eval` (they upload as `retain=expire`) and on the t6 g800 eval-suite dumps; record their keys in the
  044 MANIFEST. The tagged build reads them; no copies in git (≈ 120 MB per tick CSV).
- **Rationale**: Constitution VIII (pin what matters, provenance in the repo) plus the FR-057 lesson from 041/043.

## R13. Run reporting (FR-013)

- **Decision**: extend `pitch_cycle_progress.py` with roll rows (roll-rate RMS, 3–5 Hz roll share) — it already logs roll cap and
  flips — and port the 043 analysis scripts to read columns by name.

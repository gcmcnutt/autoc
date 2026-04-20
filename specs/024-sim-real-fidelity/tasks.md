---

description: "Task list for 024 — Sim/Real Fidelity"
---

# Tasks: 024 — Sim/Real Fidelity

**Input**: Design documents from `/specs/024-sim-real-fidelity/`
**Prerequisites**: [spec.md](./spec.md), [plan.md](./plan.md), [research.md](./research.md),
[data-model.md](./data-model.md), [contracts/](./contracts/), [quickstart.md](./quickstart.md)

**Tests**: Contract/unit tests are ON for this feature per constitution I (Testing-First).
WI15 carries the main test scope; individual fix tasks also get paired regression tests
as noted.

**Organization**: Grouped by user story (maps to spec WI priority order). Each story can
be worked independently once its prerequisites are met.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no unresolved dependencies).
- **[Story]**: `[US1]`..`[US6]` — the six user-story phases below.
- File paths are absolute from repo root.

## Path Conventions

- Autoc source: `src/`, `include/autoc/`, `tools/`
- CRRCSim (submodule): `crrcsim/src/`
- Xiao: `xiao/src/`
- Tests: `tests/` (Google Test)
- Analysis scripts: `flight-results/flight-20260417/` (flight-side),
  `specs/024-sim-real-fidelity/` (sim-side)
- Docs: `docs/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Minimal — project already initialized. Only confirm branch health and
prerequisites before user-story work.

- [x] T001 Verify branch builds clean on both sides: `scripts/rebuild-perf.sh` (autoc+crrcsim, full rebuild) and `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. Fail the feature immediately if main-branch baseline is broken. **Subsequent rebuilds during 024 use incremental compilation** (plain `make` or partial `scripts/rebuild-perf.sh`) unless source-tree structure changes (new directories, new targets, new dependencies). — **Deferred full build to Phase 5** when C++ changes land. Branch cut clean from merged-to-main state; Phase 2 is Python-only and doesn't require C++ build verification yet.
- [x] T002 [P] Confirm analysis environment: numpy 2.4.3 + matplotlib 3.10.8 available. scipy NOT required — numpy's `polyfit`, `corrcoef`, and `linalg` cover all statistics needed. Documented in [quickstart.md](./quickstart.md).
- [x] T003 [P] Regenerate the gen-400 sim slice used as reference: `head -1 /home/gmcnutt/autoc/test4-data.dat > /tmp/gen400_p0_p2.dat && grep -E '^1400400 000000 00[02]/00:' /home/gmcnutt/autoc/test4-data.dat >> /tmp/gen400_p0_p2.dat` — **351 rows (1 header + 350 data)** ✓

**Checkpoint**: Build baseline is green; analysis tooling ready; reference sim slice in place.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The shared `sensor_self_check.py` module is the workhorse for both US1
(flight audit) and US2 (sim audit). Must be in place before either story runs.

**⚠️ CRITICAL**: No user-story implementation (US1 onward) until Phase 2 is complete.

- [x] T010 Create `flight-results/flight-20260417/sensor_self_check_lib.py` — shared library containing: (a) canonical-signal data structures matching `data-model.md`; (b) source-format readers (CSV / xiao log / data.dat) honoring `contracts/*_contract.md`; (c) cross-check computations (position↔velocity, quat↔gyro, euler↔quat, accel↔gravity, heading↔track, mag↔heading, attitude-vector↔velocity-direction, command↔attitude-change); (d) **blackbox↔xiao-log fusion join** — joins INAV blackbox rows and xiao-log rows by `inav_ms` (xiao column 3, already in ms) = `blackbox time(us) / 1000`. Flag: this join crosses the xiao-space ↔ raw-INAV-space boundary — **prime convention-drift hotspot**; any silent transformation applied in one source but not the other will appear here first as scatter divergence.
- [x] T011 [P] Create `flight-results/flight-20260417/sensor_self_check.py` — CLI driver that auto-detects source format (blackbox CSV vs sim data.dat vs xiao log), imports the library, runs all cross-checks, emits console summary, saves PNG report, and writes `sensor_self_check_report.md`.
- [x] T012 [P] Add constant `ACC_1G_LSB = 256` and the canonical conversion functions `blackbox_to_canonical()`, `xiao_log_to_canonical()`, `sim_data_to_canonical()` to `sensor_self_check_lib.py`.
- [x] T013 [P] Smoke-test the library with a small synthetic quaternion sequence: given `q_EB = I` (identity), assert `rate_from_quat_pairs` returns zero; given a known sinusoidal roll, assert the body-X rate matches the derivative. No pytest required; embed as `if __name__ == '__main__': self_test()` inside the library.

**Checkpoint**: Analysis library + driver in place and self-tested. US1 and US2 can proceed in parallel.

---

## Phase 3: User Story 1 — Full-flight sensor self-consistency audit (Priority: P1) 🎯 MVP

**Goal**: Establish ground truth on what flight-20260417 actually shows — every NN-visible
sensor signal cross-checked for self-consistency per `docs/COORDINATE_CONVENTIONS.md`
and `docs/INAV_BLACKBOX.md`. Produces the list of "what's wrong and needs fixing" that
drives US3.

**Independent Test**: run `python3 flight-results/flight-20260417/sensor_self_check.py
flight-results/flight-20260417/blackbox_log_2026-04-17_173039.01.csv` — reports per-check
PASS (sign not inverted) or FAIL with Pearson r values; emits PNG + MD report. Any FAIL
becomes a hypothesis for US3.

**Pass criterion per clarification**: directional only — a check fails only if slope
sign is inverted. Correlation strength is reported but not gated.

- [x] T020 [US1] Implement `check_position_vs_velocity_integration()` in `sensor_self_check_lib.py`: integrate `nav_vel` (NED m/s canonical) over blackbox samples; compare to `nav_pos` delta across the same interval; return slope + r for each axis.
- [x] T021 [P] [US1] Implement `check_gyro_vs_quat_delta()` in same lib: compute body-rate from consecutive quat samples via `q_delta = q_prev.inverse() * q_curr; rate = 2 * q_delta.vec / dt`. Scatter against logged `gyro_ADC` (post canonical-conversion). Return slope + r per axis. This is the check that currently fails on flight pitch.
- [x] T022 [P] [US1] Implement `check_euler_quat_vs_attitude()`: extract Euler from our canonical q_EB via Eigen-equivalent `eulerAngles(2,1,0)`; compare to `attitude[]` canonical (decideg→rad, pitch-negated). Threshold: slope positive per axis.
- [x] T023 [P] [US1] Implement `check_accel_vs_gravity()`: for samples with gyro magnitude < 0.2 rad/s (quasi-steady), body-Z accel should be ≈ g×cos(θ)×cos(φ) (where θ, φ from quat). For sustained coordinated turn, body-Z ≈ g/cos(bank). Report sign and rough magnitude match.
- [x] T024 [P] [US1] Implement `check_heading_vs_track()`: quat-derived heading ψ vs `atan2(nav_vel_E, nav_vel_N)`. For samples with groundspeed > 5 m/s, compare and report persistent offset (wind + declination).
- [x] T025 [P] [US1] Implement `check_mag_vs_heading()`: magADC normalized body vector rotated to world via canonical q_EB should align with magnetic north (offset by local declination). Report persistent error.
- [x] T025a [P] [US1] Implement `check_attitude_vector_vs_velocity_direction()`: rotate body-forward `(1,0,0)` to world via canonical q_EB (our transform), compare to normalized `nav_vel`. During coordinated flight with groundspeed > 5 m/s, nose direction should align with velocity direction within wind/sideslip tolerance. This check is **especially sensitive to quaternion convention errors** — catches NEU↔NED mixups that quat↔gyro alone might not expose (both sides go through the same transform in quat↔gyro, so a self-cancelling error stays hidden; but attitude-vector-vs-velocity triangulates via an independent signal).
- [ ] T025b [P] [US1] Implement `check_cmd_vs_attitude_change()`: for xiao engage spans (fusion join required — xiao log has the commands, blackbox has the attitude rates). Scatter NN `out=[pt,rl,th]` at time t against body rates from blackbox quat-delta at t+Δ. Carries forward 023 `cmd_response_scatter` results; the check is now a formal cross-source correlation that MUST be sign-positive per axis. Pitch sign-flip here is the 023 finding we're chasing.
- [ ] T025c [US1] Implement `blackbox_xiao_fusion_join()` in the library: reads both sources, produces a unified timeline keyed by `inav_ms`, handles rate-mismatch (xiao ~10 Hz vs blackbox ~200 Hz) with nearest-neighbour within ±50 ms window. **Emit a diagnostic summary**: count of matched/unmatched xiao samples, median join skew in ms, samples where skew > 50 ms (possibly INAV clock jump or msp stall). This join is the fusion point flagged in T010 as the convention-drift hotspot.
- [x] T026 [US1] Wire all eight checks (1–6 plus T025a, T025b) into the driver's `run_all_checks()`; produce an 8-panel PNG + markdown report. Color each panel by PASS/FAIL banner. Fusion-join diagnostic appended to report.
- [x] T027 [US1] Run on `flight-results/flight-20260417/blackbox_log_2026-04-17_173039.01.csv` + paired xiao log. Commit resulting PNG + report as the WI1 baseline. Expected pre-fix result: check 2 (quat↔gyro) fails on pitch, check 7 (attitude-vector↔velocity) likely fails similarly, check 8 (cmd↔attitude-change) shows the known pitch inversion. Others PASS or WARN.
- [x] T028 [US1] Capture each FAIL/WARN in spec.md's "Running Findings Log" with category (bug / workaround / fallback / debt / question), location, and target WI. Cite blackbox column and line-of-script producing the failure. **Actively flag suspected workarounds/fallbacks**: if a check passes "only because" of a conversion step that looks like a patch rather than first-principles math, note it even if the check passes. These are the hidden hackarounds the feature is meant to expose.

**Checkpoint**: US1 MVP complete. Findings log populated with the real flight's actual shortfalls. Done regardless of whether fixes land here — that's US3.

---

## Phase 4: User Story 2 — Sim (CRRCSim + minisim) conformance audit (Priority: P1, parallel to US1)

**Goal**: Run the same cross-checks on sim data; confirm sim is the convention reference
(expected) OR find the sim bugs that compromise US3 diagnoses. This story can run
parallel to US1.

**Independent Test**: `python3 flight-results/flight-20260417/sensor_self_check.py
/tmp/gen400_p0_p2.dat` — all 6 checks PASS; sim is identity reference. Any FAIL is a
sim bug to fix in US3 before trusting sim's baseline.

- [x] T030 [US2] Implement `sim_data_to_canonical()` reader in `sensor_self_check_lib.py` per `contracts/sim_data_dat_contract.md`. Data.dat columns are already in canonical stack conventions (minimal transform).
- [x] T031 [P] [US2] Extend the sensor checks to handle sim: position/velocity cross-check adapts to use `X Y Z` + body-velocity-rotated-to-world; quat↔gyro same as flight; accel/mag not in sim data.dat (skip or mark N/A); heading↔track uses `X Y Z` velocity direction; attitude-vector↔velocity-direction uses the same body-forward check; cmd↔attitude-change uses data.dat's `outPt outRl outTh` as the command side (no fusion needed — sim is single-source).
- [x] T032 [US2] Run audit on `/tmp/gen400_p0_p2.dat` (CRRCSim). Commit result as the sim-reference baseline.
- [ ] T033 [P] [US2] Minisim conformance audit. Step 1: inspect `tools/minisim.cc` for fixed-attitude-scenario support. If present, run with a known-attitude config (e.g., 30° pitch up + 30° right bank initial), emit data.dat, pipe to audit tool. If NOT present, add a minimal test mode: CLI flag or hardcoded short scenario that initializes `aircraft_orientation` at a compound attitude, runs 1 s of zero-command integration, writes data.dat compatible with the audit reader. Expected audit result: quat↔gyro FAIL (minisim's q_EB vs q_WB bug — fixed in US3 5.b).
- [x] T034 [US2] Document sim-side findings in spec.md Running Findings Log. CRRCSim issues (if any) and minisim q_EB failure expected.

**Checkpoint**: US2 complete. Sim provides the reference baseline; any sim-side surprises feed US3.

---

## Phase 5: User Story 3 — Root-cause fixes (Priority: P1, depends on US1+US2)

**Goal**: Fix every FAIL from US1 and US2. Each fix lands in its own small commit with
a paired cross-check showing failure→pass transition. Ordered by risk and dependencies.

**Independent Test**: for each fix, re-run the relevant US1/US2 check; the previously
failing axis now PASSes. No regressions on other checks.

### 5.a — msplink pitch-axis / quat convention (the big one)

- [x] T040 [US3] Write a bench-derived test harness in `tests/msplink_quat_convention_tests.cc`: for each attitude in the WI5 table (B1–B5), construct an INAV-format body→earth quaternion, pass through `neuQuaternionToNed`-equivalent, assert the resulting `q_EB` rotates a known world vector to the expected body-frame vector within 1e-3. Initially fails (no fix yet). Gates T041.
- [x] T041 [US3] Modify `xiao/src/msplink.cpp` `neuQuaternionToNed()` (around line 170): replace simple conjugate with the correct NEU→NED composition + aerospace-RHR pitch sign. Source derivation in comment citing `docs/INAV_BLACKBOX.md` and `docs/COORDINATE_CONVENTIONS.md`. Get T040 passing.
- [x] T042 [US3] Bench verification on flight FC (post-flash). Held 9 poses (B1 level-N, B2 nose-up-30°, B2′ nose-down-30°, B3 RWD-30°, B3′ RWU-30°, B4 nose-E, B4′ nose-W, B5 compound pitch-up/RWD, B6 inverted) plus per-axis slow-rotation rate-sense checks. Finding from initial bench: T041's qz-only flip was insufficient — qy was still in INAV's nose-down-positive convention. **Updated transform to `(w, x, -y, -z)` — flip qy AND qz.** Re-benched: all 9 poses and all 3 rate-sense tests conform to aerospace q_EB + aerospace gyro. Also: switched Nav State log from partially-transformed `gyro_raw=` (raw INAV) to fully-transformed `gyro=` (aerospace rad/s) for consistency with `quat=` on the same line.
- [x] T043 [P] [US3] Xiao rebuild **for bench use only, not flight**: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. Commit T041 + T043 together as "fix msplink quat convention". Old weights + fixed msplink = flight regression (trained-against-wrong-convention mismatch) — this build stays on the bench until US6 paired with retrained weights.
- [x] T044 [US3] Update `flight-results/flight-20260417/sensor_self_check_lib.py` to a **two-quat split**: `quat` = aerospace q_EB (qz flipped) for all static lookups; `quat_raw` = INAV NEU (no flip) for kinematic rate checks (`quat↔gyro`, `cmd↔rate`). Xiao (post-T041) and sim have no NEU↔NED reflection so set `quat_raw = quat`. Matches the T041 msplink boundary convention; self-test passes.
- [x] T045 [US3] Re-ran `sensor_self_check.py` on `flight-results/flight-20260417/blackbox_log_2026-04-17_173039.01.csv` with T044 in place. Check 2 (gyro↔quat-delta) now PASSes slope +1.00/+0.99/+1.00; Check 4 (accel↔gravity) ax slope went −4.83→+0.60 (sign restored); Check 5 (heading↔track) PASS +1.17; Check 7 (nose↔vel) N/E pass, D weak −0.076 noise-dominated (|r|=0.08, aircraft mostly level — not a convention bug). Satisfies spec Validation #1.

### 5.b — Minisim q_EB canonicalization (WI7) — **P2 (nice-to-have)**

Per spec Critical Milestones: minisim is NOT on the M1 or M2 critical path.
It is not in training (CRRCSim is) and not on xiao. Fold in if time permits;
otherwise these tasks can defer and remain in BACKLOG. T033 US2 check will
show the minisim bug expected; documenting it (without fixing) is acceptable
at feature close.

- [ ] T050 [US3-P2] Write `tests/minisim_convention_tests.cc`: construct `AircraftState` with a known non-identity attitude (e.g., 30° right bank, nose east); assert `orientation.inverse() * gravity_world` gives the expected body-frame gravity, and that `minisimAdvanceState` with zero commands advances the aircraft along its nose direction in world frame. Initially fails.
- [ ] T051 [US3-P2] Modify `include/autoc/eval/aircraft_state.h` `minisimAdvanceState`: change composition to `aircraft_orientation = delta_body * aircraft_orientation` (pre-multiply for q_EB); change velocity rotation to `velocity_world = aircraft_orientation.inverse() * velocity_body`.
- [ ] T052 [US3-P2] Modify `tools/minisim.cc:148`: drop the initial-velocity pre-rotation or switch to `q_EB.inverse() * (V,0,0)` form.
- [ ] T053 [US3-P2] Revisit gyro-rate computation at `aircraft_state.h:379-382` — with q_EB composition, verify the body-rate formula direction. Update test to pin the correct sign.
- [ ] T054 [US3-P2] Update `docs/COORDINATE_CONVENTIONS.md` with a subsection explicitly naming `AircraftState::aircraft_orientation` as the canonical q_EB holder. Reference test file.
- [ ] T055 [US3-P2] Verify US2 (T033) minisim audit now PASSes all checks.

### 5.c — Rotted post-flight analysis scripts (WI10)

- [ ] T060 [P] [US3] Rewrite `specs/018-flight-analysis/correlate_flight.py` to use `sensor_self_check_lib.py` canonical conversions (apply Z-flip on navPos Z, quat conjugate). Keep functionality; use library.
- [ ] T061 [P] [US3] Rewrite `specs/019-improved-crrcsim/scripts/verify_flight_log.py` to use new NN input layout (direction cosines at tX/tY/tZ instead of dPhi/dTheta indices).
- [ ] T062 [P] [US3] Rewrite `specs/022-tracking-cone-fitness/flight_nn_polar_viz.py` similarly — direction cosines not bearing angles.

### 5.d — NN input layout drift guard (WI9)

- [ ] T070 [US3] Add `constexpr int NN_INPUT_LAYOUT_VERSION = 1;` to `include/autoc/nn/nn_inputs.h` alongside the existing `NN_INPUT_COUNT`.
- [ ] T071 [US3] Emit `layout_version=1` in data.dat header comment line (modify header print in `src/autoc.cc` near line 636) and in xiao log banner (`xiao/src/msplink.cpp` boot section).
- [ ] T072 [US3] Reader assertion: `sensor_self_check_lib.py` and renderer read the version field and bail with a clear error if mismatch.

### 5.e — Xiao rabbit logging sanity (WI8)

- [x] T080 [US3] Audit `xiao/src/msplink.cpp` NN log line emission — confirmed `rabbit=[x,y,z]` field was **missing** from current NN log format. Added `rabbit=[%.2f,%.2f,%.2f]` at end of log format; value is `targetPos` (virtual-NED m) from `getInterpolatedTargetPosition()` — the actual NN target, not a reconstruction.
- [ ] T081 [US3] Add a boot-time log line asserting `sizeof(NNInputs) == 33 * sizeof(float)` (the existing static_assert also prints a line on boot).

**Checkpoint**: US3 complete. After T044 (analysis-library fix), re-running US1 on flight-20260417 historical CSV **now PASSes** — the blackbox data was always correct; we just interpret it correctly now. T042 bench verifies the xiao-side msplink fix behaves the same way on the live pipeline. US2 sim audit passes after T055 (minisim q_EB fix).

**No flight deployment from this phase.** All xiao rebuilds here are bench artifacts only. Flight-hardware weights remain untouched — the current flight-ready firmware + old weights is the last known "flies-at-all" config, and we preserve it until US6 can deploy a matched set (fixed msplink + fixed cadence + retrained weights) as a single atomic update.

US6 flight is the in-motion confirmation on fresh data.

---

## Phase 6: User Story 4 — Sim cadence fix to 100 ms (Priority: P1, parallel to US3)

**Goal**: Bring sim eval cadence to exactly 100 ms/sample. Preserves determinism.
Unblocks retrain at clean cadence.

**Independent Test**: `awk 'NR>1 {print $4}' <new_data.dat> | awk 'NR>1 {print $1-prev; prev=$1}' | sort -u` returns exactly `100`. Eval suite passes (tier0 determinism bitwise matches).

- [ ] T090 [US4] Modify `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp:341` from strict-greater to accumulator-based per research.md §3. Code: declare a `static double evalAccum = 0.0;`, accumulate `(simTimeMsec - lastUpdateTimeMsec)`, fire when `evalAccum >= gEvalUpdateIntervalMsec`, then subtract to preserve fractional remainder.
- [ ] T091 [US4] Fix the stale `SIM_FPS = 25.0` comment at `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h:53`: replace "~40 Hz physics tick assumption" with accurate explanation ("matches gameSpeed from autoc_config.xml; used only for overflow bucket calc").
- [ ] T092 [US4] Rebuild both sides: `scripts/rebuild.sh`.
- [ ] T093 [US4] Run tier0 eval with existing weights: confirm determinism preserved (bitwise match against reference). If broken, investigate `evalAccum` initialization in each scenario.
- [ ] T094 [US4] Run a short training-format eval that produces a data.dat; verify cadence exactly 100 ms via awk check.
- [ ] T095 [P] [US4] Add WI13 design comments near T090 site documenting how 50 ms (20 Hz) lands — change only the interval constant, no code restructure.

**Checkpoint**: US4 complete. Sim emits data at exactly 100 ms cadence; eval determinism preserved; 20 Hz readiness documented.

---

## Phase 7: User Story 5 — Compound-attitude bench verification (Priority: P1, depends on US3 msplink fix)

**Goal**: Bench-verify the US3 msplink quat convention fix on actual flight hardware. Compound attitudes expose sign/axis bugs single-axis tests cannot.

**Scope limitation**: bench tests verify **gyro and quaternion conventions only** (plus accel/mag as secondary via gravity-vector and compass direction). **Position and velocity are NOT bench-verifiable** — no GPS lock indoors, no motion on a stationary bench. Those checks (WI1 checks 1, 5 partly, 6, 7, 8) remain flight-only and are validated in WI6.

**Independent Test**: physically hold FC at each B1–B5 attitude; record data; compare extracted Euler to expected within ±5°. Gyro near-zero at each static hold.

- [ ] T100 [US5] Author `specs/024-sim-real-fidelity/bench_attitude_check.py`: reads a xiao serial dump (or blackbox snapshot) of static holds; for each, extracts the canonical q_EB and converts to Euler; prints expected vs actual; flags deviations > 5°. Also checks: gyro magnitude near zero during hold (confirms signal quality), body-Z accel ≈ g and lateral components ≈ g×sin(roll/pitch) (gravity-vector cross-check), mag vector direction roughly horizontal-north (modulo declination).
- [ ] T101 [US5] Physical bench procedure: power up flight FC on workbench. Hold each attitude B1–B5 from spec WI5 table for ~5 seconds while capturing xiao serial log. Save logs per attitude in `flight-results/bench-20260419-compound-attitude/` (or current-dated dir). Capture in both blackbox (`.TXT`) and xiao serial — they should agree after canonical transform.
- [ ] T102 [US5] Run `bench_attitude_check.py` on captured logs. Every attitude must pass within ±5° on all three Euler components. Gyro should be near-zero (< 0.05 rad/s). Gravity vector should match expected body-frame projection.
- [ ] T103 [US5] Append the verification results table to `docs/COORDINATE_CONVENTIONS.md` (alongside the existing single-axis bench table). Include pass/fail per attitude per column (quat/Euler, gyro, accel, mag). Note explicitly what bench CAN and CANNOT verify: position + velocity require flight.
- [ ] T104 [US5] If any attitude fails on quat/Euler, revisit US3 T041 msplink fix derivation. If gyro/accel/mag fails but quat passes, log as a secondary convention issue (possibly a calibration or board-alignment quirk; may warrant a follow-up WI). Do NOT proceed to US6 retrain/flight with a failing quat/Euler bench.

**Checkpoint**: US5 complete. Conventions bench-verified on the actual flight FC. **Still no flight deployment** — the FC is on the bench. The bench-verified xiao binary goes back on the shelf until US6 pairs it with retrained weights. Unlocks US6.

---

## Phase 8: User Story 6 — Retrain + eval + flight test (Priority: P1, depends on US1-US5)

**Goal**: Close the feature. Retrain the NN on fixed-cadence sim, pass the eval suite, **deploy** the paired (fixed xiao + retrained weights) to flight hardware as a single atomic update, fly the corrected pipeline, demonstrate sim-parity dynamics.

**The single deployment event of 024**: until this phase, no changes have been flashed to the flight FC or xiao. All prior xiao rebuilds (US3, US5) were bench-only. This phase produces the first and only flight-ready firmware package: msplink fix + cadence fix + retrained weights, deployed together.

**Independent Test**: all six prior stories complete; training + eval + flight produce sim-matching scatter results (cmd→rate slope sign-correct across all three axes, quat-derived rate matches gyro).

- [ ] T110 [US6] Retrain from the existing topology (33→32→16→3, unchanged per clarification). Target 400 generations on the fixed-cadence sim. Commit weights in the standard location.
- [ ] T111 [US6] Eval suite: run tier0 (repro determinism), tier1 (novel seed), tier2 (generalization — random paths, craft variations, long), tier3 (stress + quiet). All must pass.
- [ ] T112 [US6] Regenerate `xiao/src/generated/nn_program_generated.cpp` via `tools/nn2cpp/`. Verify topology and weight count match expectations.
- [ ] T113 [US6] Xiao rebuild with the retrained `nn_program_generated.cpp` and all accumulated msplink/rabbit-logging fixes from US3: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. Any build error investigates immediately. This is the first build that will touch flight hardware.
- [ ] T113a [US6] **Deploy to flight FC**: flash the xiao binary from T113. Confirm boot banner shows expected schema version and weight count. This is the single deployment event of 024.
- [ ] T114 [US6] Preflight checklist walk. Use project memory (`project_preflight_checklist.md`). Bench-test failsafe chain **on the flight-deployed binary** (re-run the T102 compound-attitude holds as a sanity check that the flight build matches the bench build).
- [ ] T115 [US6] Flight test — pilot warmup, short xiao engage span(s), pilot-flown coordinated maneuvers (level circle, climb, dive) for post-flight audit data. Varied-throttle span during engage if possible.
- [ ] T116 [US6] Post-flight: run `sensor_self_check.py` on the new blackbox CSV. All cross-checks PASS. Run `gyro_vs_quat.py` on the new xiao log. Slope ≈ 1.0 on all three axes with positive sign.
- [ ] T117 [US6] Close the feature: summary commit documenting the progression from WI1 findings through US6 flight verification. Update spec.md Validation checkboxes.

**Checkpoint**: Feature CLOSED. Sim-to-real dynamic control response verified matching. Non-linear effects, craft variations, bang-bang → 025.

---

## Phase 9: Polish & Cross-Cutting

**Purpose**: Lower-priority tech debt and long-tail cleanup from the running findings log. Can happen in parallel with the critical path (US1–US6) or after.

- [ ] T200 [P] Test coverage (WI15): add `tests/engage_reset_tests.cc` — 6 contract tests for `resetHistory()` covering engage-transition pre-fill, buffer index consistency, current-sim-tick semantics (per clarification #4).
- [ ] T201 [P] Test coverage (WI15): add `tests/engage_delay_tests.cc` — 3 contract tests for the 750 ms delay window (stick-centered output during delay, cruise-throttle derivation, first-real-command timing).
- [ ] T202 [P] Test coverage (WI15): add `tests/nn_inputs_tests.cc` — unit-vector invariant on direction cosines, schema version assertion, poison-value completeness.
- [x] T203 Renderer legacy INAV blackbox path (WI11): deleted. Removed `parseBlackboxData`, `loadBlackboxData`, `Renderer::extractTestSpans`, the `-d/--decoder` CLI option, `decoderCommand`/`csvLines` globals, `MSPRCOVERRIDE_FLAG` macro, and `inDecodeMode` flag. The `blackbox*` naming in the shared rendering path (points, states, tapes, actors) is retained — it's populated from xiao logs now. Renderer builds clean; all 11 desktop tests still pass. Downstream `(inDecodeMode || inXiaoMode)` checks simplified to `inXiaoMode`. The old full-conjugate bug at former lines 1720-1735 is gone along with the rest.
- [ ] T204 [P] Training run archive policy (WI12): write one-page `docs/TRAINING_RUN_ARCHIVE.md` documenting naming, retention, and the current canonical run (`test4-data.dat`).
- [ ] T205 [P] 20 Hz future-readiness design note (WI13): document in `research.md` appendix — `EVAL_UPDATE_INTERVAL_MSEC_DEFAULT` as real config knob, decide `HIST_PAST`/`FORECAST_OFFSETS` rescaling policy (ticks vs ms), note xiao hybrid-timer design for 50 ms.
- [ ] T210 Workarounds audit (WI14): walk the 5 known suspect items in spec.md WI14 list. Per item, disposition = removed, replaced, or documented as legitimate. Capture results in spec.md Running Findings Log.
- [ ] T211 [P] Final spec.md pass: move resolved items from Running Findings Log into the corresponding WI "done" sections. Ensure Validation items all have checkboxes matching actual state.

---

## Dependencies

The critical path maps to the two milestones defined in spec.md:

```
Setup (T001-T003) → Foundational lib (T010-T013)
   ↓
M1: CRRCSim is right
  T030-T032, T034 (US2 sim audit — skip T033 minisim)
  + T090-T095 (US4 cadence fix)
  + any CRRCSim-side fixes surfaced in T034
   ↓
M2: Xiao IO matches CRRCSim
  T020-T028 (US1 flight audit — can run parallel to M1)
  + T040-T045 (US3 5.a msplink + analysis-library quat fix)
  + T060-T081 (US3 5.c/5.d/5.e script/guard/rabbit fixes, parallel)
  + T100-T104 (US5 bench verification on flight FC)
   ↓
US6 (T110-T117 + T113a): retrain + atomic deployment + flight
   ↓
FEATURE CLOSED

P2 (nice-to-have, can defer):
  T033 (minisim audit in US2)
  T050-T055 (US3 5.b minisim q_EB fix)
  — parallel to any phase after foundational; do not gate on these

Polish (T200-T211) — parallel to any phase after Phase 2
```

### Critical path

**M1 fastest path**: T001-T003 → T010-T013 → (T030-T032 ∥ T090-T095) → gate on sim data.dat passing all 8 cross-checks at 100 ms exactly.

**M2 fastest path**: (parallel with M1) T001-T003 → T010-T013 → T020-T028 → T040-T045 → T100-T104 → gate on compound-attitude bench PASS and flight-20260417 historical audit PASS (post-T044).

**Feature close path**: M1 ∥ M2 → T110-T117 + T113a → done.

### Parallel opportunities

- **Phase 2**: T011–T013 all parallelizable after T010.
- **Phase 3 (US1)**: T021–T025 parallelizable after T020 establishes the driver; all touch the same lib file but can be split by cross-check function.
- **Phase 4 (US2)**: parallel to Phase 3 entirely.
- **Phase 5 (US3)**: 5.a (msplink), 5.b (minisim), 5.c (rotted scripts), 5.d (layout guard), 5.e (rabbit logging) are all independent subgroups — can parallelize across developers.
- **Phase 9 (Polish)**: all [P]-marked; can run concurrently with critical path.

## Implementation Strategy

**MVP scope** = US1 (Phase 3) + US2 (Phase 4). The two audits together produce the diagnostic report feeding M1 (sim correctness) and M2 (xiao parity). Everything downstream (US3 fixes, US4 cadence, US5 bench, US6 flight) follows from the audit findings and the pre-planned spec items.

**Milestone-first ordering**: within the critical path, prioritize M1 before M2 — the sim is the reference; we can't validate xiao parity against a suspect reference. M1 tasks (US2 CRRCSim audit + US4 cadence fix) are the first items to complete end-to-end.

**Incremental delivery**: each story's completion is a shippable increment, but **no story before US6 deploys to flight hardware**. The flight FC stays on its current "last known flies-at-all" firmware throughout US1–US5. Strategy note: retrain is the only path to a correct flight; old weights + fixed msplink would regress (weights were trained against pre-fix conventions).

- **After US1**: "here's what's wrong with flight-20260417." Analysis artifact. No code changes yet. Flight hardware untouched.
- **After US2**: "sim is (or isn't) the reference." If sim has bugs, we fix them first. Still no hardware changes.
- **After US3**: "we think we found the bugs; bench-only xiao build proves them locally." Flight hardware untouched — weights-to-conventions mismatch means this build would fly *worse* than the current one.
- **After US4**: "training produces clean 100 ms cadence." Training team unblocked — this is a prerequisite for US6 retrain. No hardware deployment.
- **After US5**: "bench confirms conventions on the real FC hardware." FC returns to its shelf until US6 pairs the fix with new weights.
- **After US6**: retrain + eval + paired deployment + flight test. Feature closed. This is the only hardware-update event in 024.

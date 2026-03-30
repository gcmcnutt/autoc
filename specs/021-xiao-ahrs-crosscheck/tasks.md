# Tasks: 021 — AHRS Cross-Check + NN Input Redesign

**Input**: Design documents from `/specs/021-xiao-ahrs-crosscheck/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Organization**: Sim-first approach. All sim-side changes (CRRCSim rate PID,
NN input redesign, training validation) are completed and proven BEFORE
touching flight hardware. INAV/xiao changes are plumbing to replicate
what the sim already provides.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US*]**: User story mapping:
  - US1: CRRCSim ACRO rate PID (sim-side inner loop)
  - US2: NN input redesign (new sensor vector)
  - US3: Training + sim validation
  - US4: INAV MSP extension (gyro in MSP2_AUTOC_STATE)
  - US5: Xiao consumer + ACRO mode override
  - US6: Safety overrides
  - US7: Characterization flight + analysis (parallel, when props arrive)

---

## Phase 1: Bench Verification

**Purpose**: Confirm gyro/accel polarity and ACRO config on bench hardware before any code changes.

- [ ] T001 Ground polarity check on bench: with props off, verify gyroADC[0] positive for right-wing-down roll, gyroADC[1] positive for nose-up pitch, accSmooth[2] positive when level. Use INAV configurator or short blackbox capture. Per `docs/COORDINATE_CONVENTIONS.md`.
- [ ] T002 [P] Verify INAV ACRO PID config on bench: check `fw_p/i/d/ff_roll/pitch/yaw`, `rc_expo`, rate limits match `xiao/inav-bench.cfg`. Document actual values.

**Checkpoint**: Gyro polarity confirmed. ACRO gains documented. Safe to implement sim-side rate PID.

---

## Phase 2: CRRCSim ACRO Rate PID (US1) — MVP

**Goal**: Implement rate PID in CRRCSim that converts NN rate commands to surface deflections, matching INAV's ACRO behavior. This is the sim-side inner loop.

**Independent Test**: Train a short run (50 gens) with rate PID active and existing 29 inputs. NN should converge — completion rate >80% on aeroStandard paths. Compare control smoothness to BIG3 baseline.

- [ ] T010 [US1] Extract body angular rates (p, q, r) from LaRCSim FDM in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`. FDM stores rates in `v_R_omega_total.r[0-2]` (rad/s). Convert to deg/s. Wire to autoc RPC state.
- [ ] T011 [US1] Implement rate PID in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`: for each axis, compute `output = FF*cmd + P*(cmd_rate - actual_rate) + I*integral(error)`. Use INAV gains: roll FF=50 P=5 I=7, pitch FF=50 P=5 I=7, yaw FF=60 P=6 I=10. No D term. Rate limits: roll 560°/s, pitch 400°/s, yaw 240°/s.
- [ ] T012 [US1] Add rate PID config to `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`: define PID gains, rate limits, expo=0 as compile-time defaults. Add env var overrides (`AUTOC_ACRO_*`) for tuning without rebuild.
- [ ] T013 [US1] Add ACRO mode toggle: env var `AUTOC_ACRO_MODE=1` (default ON). When OFF, revert to direct servo mapping (MANUAL mode, for regression testing). Guard the rate PID code path.
- [ ] T014 [US1] Rebuild CRRCSim: `cd crrcsim/build && make -j8`. Smoke test: run a short training (10 gens, pop=100) with ACRO mode on. Verify NN outputs are interpreted as rate commands and surface deflections follow.

**Checkpoint**: CRRCSim has rate PID. NN outputs rate commands. Sim behavior plausible.

---

## Phase 3: NN Input Redesign (US2)

**Goal**: Change NN input vector from 29 to 27 inputs. Remove alpha/beta and previous commands, add rate gyros from FDM.

**Independent Test**: Train 50 gens with new inputs + ACRO rate PID. Compare convergence to Phase 2 baseline (same PID, old inputs). New inputs should converge at least as well.

- [ ] T020 [US2] Wire body rates (p, q, r) through autoc RPC protocol: add gyro rate fields to `AircraftState` in `include/autoc/eval/aircraft_state.h`. CRRCSim populates from FDM (already extracted in T010).
- [ ] T021 [US2] Update `nn_gather_inputs()` in `src/nn/evaluator.cc`: remove alpha/beta (inputs 24-25), remove previous commands (inputs 26-28), add gyro rates p/q/r scaled by max_rate (560/400/240 deg/s → [-1,1]). Reindex to 27 total inputs.
- [ ] T022 [US2] Update NN topology config: change input count from 29 to 27 in `autoc.ini` and `autoc-eval.ini`. Hidden layers stay 16→8→3. Verify topology string matches: `27 -> 16 -> 8 -> 3`.
- [ ] T023 [P] [US2] GoogleTest: verify input vector construction in `tests/` — create known AircraftState with specific quaternion, velocity, gyro rates, path geometry. Assert input vector matches expected 27 values.
- [ ] T024 [US2] Rebuild autoc: `bash scripts/rebuild.sh`. Run eval with new topology (expect poor results — this validates the pipeline, not the controller).

**Checkpoint**: 27-input NN compiles and trains. Gyro rates wired in sim. Alpha/beta and previous commands removed.

---

## Phase 4: Training + Sim Validation (US3)

**Goal**: Validate the new architecture converges. Short runs first, then production.

**Independent Test**: Eval suite passes (>95% completion on aeroStandard). Smoothness metrics comparable or better than BIG3.

- [ ] T030 [US3] Short training run: 50 gens, pop=1000 with ACRO rate PID + 27 inputs. Verify convergence trajectory (fitness should decrease). Check for obvious issues (NaN, divergence, zero outputs).
- [ ] T031 [US3] Analyze short run: extract response curves with `specs/019-improved-crrcsim/sim_response.py` (adapt for rate outputs if needed). Compare control smoothness to BIG3. Specifically check: are rate commands proportional or bang-bang?
- [ ] T032 [US3] If convergence OK: BIG production run (400 gens, pop=3000). Monitor early generations for fitness trajectory.
- [ ] T033 [US3] Eval suite on BIG run weights: `./scripts/eval_suite.sh <weights.dmp>`. All tiers.
- [ ] T034 [US3] Response analysis: compare sim rate commands vs characterization flight data (if available from US7). Tune CRRCSim PID gains if mismatch found.

**Checkpoint**: Trained controller with new architecture. Eval passes. Sim validated.

---

## Phase 5: INAV MSP Extension (US4)

**Goal**: Add filtered gyro[3] to MSP2_AUTOC_STATE response. Only proceed after sim validation (Phase 4) confirms the architecture works.

**Independent Test**: Bench test: xiao receives and logs gyro rates from INAV. Compare to blackbox gyroADC[0-2] — should match within rounding.

- [ ] T040 [US4] Extend MSP2_AUTOC_STATE handler in `~/inav/src/main/fc/fc_msp.c`: add `gyro[3]` (int16, deci-deg/s, = `lrintf(gyro.gyroADCf[axis] * 10)`) after existing payload. New payload: 38 + 6 = 44 bytes.
- [ ] T041 [US4] Build INAV for bench: `cd ~/inav/build && cmake .. && make MAMBAF722_2022A`. Flash to bench FC.
- [ ] T042 [US4] Build INAV for flight: `make MATEKF722MINI`. Store binary for later deployment.

**Checkpoint**: INAV sends gyro rates via MSP. Both targets built.

---

## Phase 6: Xiao Consumer + ACRO Mode (US5)

**Goal**: Xiao parses extended MSP, populates gyro rates for NN, overrides to ACRO mode.

**Independent Test**: Bench test with autoc enabled: xiao log shows gyro rate values, flight mode shows ACRO|MSPRCOVERRIDE, NN outputs interpreted as rate commands.

- [ ] T050 [US5] Update `msp_autoc_state_t` in `xiao/include/MSP.h`: add `int16_t gyro[3]` field. Update payload size constant.
- [ ] T051 [US5] Update MSP parser in `xiao/src/msplink.cpp`: extract gyro[3] from extended response. Scale: divide by 10 to get deg/s.
- [ ] T052 [US5] Update xiao `nn_gather_inputs`: populate gyro rate inputs from MSP data. Scale by max_rate per axis (560/400/240 deg/s → [-1,1]).
- [ ] T053 [US5] Change ACRO mode override: update CH6 value in `xiao/src/msplink.cpp` to select ACRO instead of MANUAL. Update INAV `aux` config in `xiao/inav-bench.cfg` and `xiao/inav-hb1.cfg`.
- [ ] T054 [US5] Set INAV `rc_expo = 0` in both bench and flight configs for linear rate mapping.
- [ ] T055 [US5] Extract weights from Phase 4 BIG run, `nn2cpp`, build xiao: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.
- [ ] T056 [US5] Bench verification: arm, enable autoc, verify xiao log shows gyro rates, ACRO mode flag, servo response to NN commands.

**Checkpoint**: Full pipeline on bench: INAV → MSP (with gyro) → xiao → NN (27 inputs) → rate commands → INAV ACRO PID → servos.

---

## Phase 7: Safety Overrides (US6)

**Goal**: Distance-from-origin sphere check on xiao. Disables autoc if aircraft drifts too far.

**Independent Test**: Bench test: manually inject position >100m from origin, verify autoc disables and log shows safety event.

- [ ] T060 [US6] Implement safety check in `xiao/src/msplink.cpp`: on autoc enable, capture origin position. Each tick, compute distance. If > threshold (100m, compile-time configurable), set autoc=N and log safety event.
- [ ] T061 [US6] Add safety state to xiao log: `safety=OK` or `safety=TRIPPED dist=X.Xm`.
- [ ] T062 [US6] Build and bench test safety override.

**Checkpoint**: Safety override active.

---

## Phase 8: Characterization Flight + Analysis (US7)

**Goal**: Fly the choreography from `flight-choreography.md`. Collect response data for sim tuning.

**Note**: Runs in parallel with Phases 2-6 whenever props arrive. No code changes needed — uses current firmware with updated blackbox config. Data feeds back into T034 (PID tuning).

- [ ] T070 [P] [US7] Apply blackbox config to flight hardware: `xiao/inav-hb1.cfg` changes via INAV CLI (1/32 rate, GYRO_RAW + ACC enabled).
- [ ] T071 [P] [US7] Characterization flight: follow `specs/021-xiao-ahrs-crosscheck/flight-choreography.md`. MANUAL mode, no autoc. Roll/pitch/throttle steps at multiple speeds. ACRO segments if comfortable.
- [ ] T072 [US7] Decode blackbox. Verify <10 decode failures. Check Z continuity.
- [ ] T073 [US7] Response analysis: `specs/019-improved-crrcsim/flight_response.py --axis all`. Compare to sim. Focus on rate response to step inputs.
- [ ] T074 [US7] Gyro filter analysis: compare gyroRaw vs gyroADC during step inputs. Assess 25Hz LPF lag. Document filter recommendations.

**Checkpoint**: Response data collected. Sim PID tuning validated or updated.

---

## Phase 9: Flight Test

**Goal**: Fly with new NN (ACRO mode, 27 inputs, rate commands). Expect sustained tracking.

- [ ] T080 Flash INAV to flight hardware (MATEKF722MINI) with MSP gyro extension.
- [ ] T081 Deploy xiao with production weights + safety override + ACRO mode.
- [ ] T082 Ground check: arm, verify ACRO mode, servo response, safety cutoff test.
- [ ] T083 Flight test: engage autoc, observe tracking. Collect blackbox + xiao logs.
- [ ] T084 Post-flight analysis: trajectory vs path, rate commands vs response, sustained tracking >10s.

**Checkpoint**: Flight validated. Architecture proven.

---

## Phase 10: Polish

- [ ] T090 [P] Update `docs/COORDINATE_CONVENTIONS.md` with gyro rate wiring details.
- [ ] T091 [P] Update `specs/BACKLOG.md`: move completed items, add new findings.
- [ ] T092 Commit final configs.

---

## Dependencies & Execution Order

### Critical Path (sim-first)

```
T001-T002 (bench polarity/config check)
    ↓
T010-T014 (CRRCSim rate PID — MVP)
    ↓
T020-T024 (NN input redesign)
    ↓
T030-T034 (training + sim validation)
    ↓  ← T074 feeds back PID tuning here
T040-T042 (INAV MSP extension)
    ↓
T050-T056 (xiao consumer + ACRO)
    ↓
T060-T062 (safety overrides)
    ↓
T080-T084 (flight test)
    ↓
T090-T092 (polish)
```

### Parallel Track (characterization flight)

```
T070-T074 (whenever props arrive, no code dependency)
    ↓
feeds back into T034 (CRRCSim PID tuning)
```

### Parallel Opportunities Within Phases

- **Phase 1**: T001, T002 can run in parallel
- **Phase 3**: T023 (GoogleTest) parallel with T020-T021
- **Phase 5**: T040, T041 can overlap (design MSP while building)
- **Phase 8**: entirely parallel with Phases 2-6

---

## Implementation Strategy

### MVP: CRRCSim Rate PID (Phase 2)

1. Implement rate PID in CRRCSim
2. Train with existing 29 inputs but ACRO mode
3. **STOP and VALIDATE**: does ACRO help convergence?
4. If yes: proceed to Phase 3. If no: investigate PID tuning.

### Incremental

1. Phase 1 → bench polarity confirmed
2. Phase 2 → ACRO in sim (MVP — validates architecture)
3. Phase 3 → new 27 inputs (validates sensor changes)
4. Phase 4 → production training (validates at scale)
5. Phase 5+6 → INAV/xiao plumbing (replicates sim on hardware)
6. Phase 7 → safety (required for flight)
7. Phase 8 → characterization data (parallel, feeds back to PID)
8. Phase 9 → flight test (endgame)

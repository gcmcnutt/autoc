# Tasks: 021 — AHRS Cross-Check + NN Input Redesign

**Input**: Design documents from `/specs/021-xiao-ahrs-crosscheck/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Organization**: Tasks follow plan.md phasing. CRRCSim rate PID is implemented
and validated in sim FIRST, then INAV/xiao changes, then training, then flight.
Phase 1 (characterization flight) runs in parallel whenever props arrive.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US*]**: User story mapping:
  - US1: CRRCSim ACRO rate PID (sim-side inner loop)
  - US2: NN input redesign (new sensor vector)
  - US3: INAV MSP extension (accel + gyro in MSP2_AUTOC_STATE)
  - US4: Xiao consumer + ACRO mode override
  - US5: Training + validation
  - US6: Safety overrides
  - US7: Characterization flight + analysis (parallel, when props arrive)

---

## Phase 1: Setup

**Purpose**: Config changes and analysis tools. No code changes to autoc/crrcsim/inav.

- [ ] T001 Update INAV flight config for characterization: apply blackbox changes from `xiao/inav-hb1.cfg` (1/32 rate, GYRO_RAW + ACC enabled, NAV_PID/ATTI/NAV_ACC dropped) to flight hardware via INAV CLI.
- [ ] T002 [P] Verify INAV ACRO PID config on flight hardware: check `fw_p/i/d/ff_roll/pitch/yaw`, `rc_expo`, rate limits match `xiao/inav-hb1.cfg`. Document values.
- [ ] T003 [P] Ground polarity check: with props off, verify gyroADC[0] positive for right-wing-down roll, gyroADC[1] positive for nose-up pitch, accSmooth[2] positive when level. Per `docs/COORDINATE_CONVENTIONS.md`.

**Checkpoint**: Flight hardware configured. Polarity verified. Ready for characterization flight.

---

## Phase 2: CRRCSim ACRO Rate PID (US1) — MVP

**Goal**: Implement rate PID in CRRCSim that converts NN rate commands to surface deflections, matching INAV's ACRO behavior. This is the sim-side inner loop.

**Independent Test**: Train a short run (50 gens) with rate PID active. NN should converge — completion rate >80% on aeroStandard paths. Compare control smoothness to BIG3 baseline.

- [ ] T010 [US1] Extract body angular rates (p, q, r) from LaRCSim FDM in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`. FDM stores rates in `v_R_omega_total.r[0-2]` (rad/s). Convert to deg/s. Wire to autoc RPC state.
- [ ] T011 [US1] Implement rate PID in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`: for each axis, compute `output = FF*cmd + P*(cmd_rate - actual_rate) + I*integral(error)`. Use INAV gains: roll FF=50 P=5 I=7, pitch FF=50 P=5 I=7, yaw FF=60 P=6 I=10. No D term. Rate limits: roll 560°/s, pitch 400°/s, yaw 240°/s.
- [ ] T012 [US1] Add rate PID config to `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`: define PID gains, rate limits, expo=0 as compile-time defaults. Add env var overrides (`AUTOC_ACRO_*`) for tuning without rebuild.
- [ ] T013 [US1] Add ACRO mode toggle: env var `AUTOC_ACRO_MODE=1` (default ON). When OFF, revert to direct servo mapping (MANUAL mode, for regression testing). Guard the rate PID code path.
- [ ] T014 [US1] Rebuild CRRCSim: `cd crrcsim/build && make -j8`. Smoke test: run a short training (10 gens, pop=100) with ACRO mode on. Verify NN outputs are interpreted as rate commands and surface deflections follow.

**Checkpoint**: CRRCSim has rate PID. NN outputs rate commands. Sim behavior plausible.

---

## Phase 3: NN Input Redesign (US2)

**Goal**: Change NN input vector from 29 to 27 inputs. Remove alpha/beta and previous commands, add rate gyros.

**Independent Test**: Train 50 gens with new inputs + ACRO rate PID. Compare convergence to Phase 2 baseline (same PID, old inputs). New inputs should converge at least as well.

- [ ] T020 [US2] Wire body rates (p, q, r) through autoc RPC protocol: add gyro rate fields to `AircraftState` in `include/autoc/eval/aircraft_state.h`. CRRCSim populates from FDM.
- [ ] T021 [US2] Update `nn_gather_inputs()` in `src/nn/evaluator.cc`: remove alpha/beta (inputs 24-25), remove previous commands (inputs 26-28), add gyro rates p/q/r scaled by max_rate (560/400/240 deg/s → [-1,1]). Reindex to 27 total inputs.
- [ ] T022 [US2] Update NN topology config: change input count from 29 to 27 in `autoc.ini` and `autoc-eval.ini`. Hidden layers stay 16→8→3. Verify topology string matches: `27 -> 16 -> 8 -> 3`.
- [ ] T023 [US2] Update xiao `nn_gather_inputs` equivalent in `xiao/src/msplink.cpp` (or wherever xiao builds the NN input vector): same changes as T021 — remove alpha/beta, remove previous commands, add gyro rates. Gyro rates come from MSP2_AUTOC_STATE (Phase 4, stub with zeros for now).
- [ ] T024 [P] [US2] GoogleTest: verify input vector construction in `tests/` — create known AircraftState with specific quaternion, velocity, gyro rates, path geometry. Assert input vector matches expected 27 values.
- [ ] T025 [US2] Rebuild autoc: `bash scripts/rebuild.sh`. Run eval with new topology on existing weights (expect poor results — this validates the pipeline, not the controller).

**Checkpoint**: 27-input NN compiles and trains. Gyro rates wired in sim. Alpha/beta and previous commands removed.

---

## Phase 4: INAV MSP Extension (US3)

**Goal**: Add filtered gyro[3] to MSP2_AUTOC_STATE response. (Accel deferred — only needed for cross-check, not NN input.)

**Independent Test**: Bench test: xiao receives and logs gyro rates from INAV. Compare logged values to blackbox gyroADC[0-2] at same timestamp — should match within rounding.

- [ ] T030 [US3] Extend MSP2_AUTOC_STATE handler in `~/inav/src/main/fc/fc_msp.c`: add `gyro[3]` (int16, deci-deg/s, = `lrintf(gyro.gyroADCf[axis] * 10)`) after existing payload. New payload size: 38 + 6 = 44 bytes.
- [ ] T031 [US3] Build INAV for bench: `cd ~/inav/build && cmake .. && make MAMBAF722_2022A`. Flash to bench FC.
- [ ] T032 [US3] Build INAV for flight: `make MATEKF722MINI`. Store binary for later deployment.

**Checkpoint**: INAV sends gyro rates via MSP. Both targets built.

---

## Phase 5: Xiao Consumer + ACRO Mode (US4)

**Goal**: Xiao parses extended MSP, populates gyro rates for NN, overrides to ACRO mode instead of MANUAL.

**Independent Test**: Bench test with autoc enabled: xiao log shows gyro rate values, flight mode shows ACRO|MSPRCOVERRIDE, servo response reflects rate PID (not direct deflection).

- [ ] T040 [US4] Update `msp_autoc_state_t` in `xiao/include/MSP.h`: add `int16_t gyro[3]` field. Update payload size constant.
- [ ] T041 [US4] Update MSP parser in `xiao/src/msplink.cpp`: extract gyro[3] from extended response. Store in AircraftState. Scale: divide by 10 to get deg/s.
- [ ] T042 [US4] Update xiao `nn_gather_inputs`: replace stubs from T023 with actual gyro values from MSP. Scale by max_rate per axis.
- [ ] T043 [US4] Change ACRO mode override: update CH6 value in `xiao/src/msplink.cpp` to select ACRO instead of MANUAL. Update INAV `aux` config in `xiao/inav-bench.cfg` and `xiao/inav-hb1.cfg` to map CH6 value to ACRO mode.
- [ ] T044 [US4] Set INAV `rc_expo = 0` in both bench and flight configs for linear rate mapping.
- [ ] T045 [US4] Build xiao: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.
- [ ] T046 [US4] Bench verification: arm, enable autoc, verify xiao log shows gyro rates, ACRO mode flag, servo response to NN commands.

**Checkpoint**: Full pipeline working on bench: INAV → MSP (with gyro) → xiao → NN (27 inputs) → rate commands → INAV ACRO PID → servos.

---

## Phase 6: Training + Validation (US5)

**Goal**: Production training run with new architecture. Validate convergence and control dynamics.

**Independent Test**: Eval suite passes (>95% completion on aeroStandard). Response curves from training data show rate commands, not servo saturations. Smoothness metrics comparable or better than BIG3.

- [ ] T050 [US5] Short training run: 50 gens, pop=1000 with ACRO rate PID + 27 inputs. Verify convergence trajectory (fitness should decrease). Check for obvious issues (NaN, divergence, zero outputs).
- [ ] T051 [US5] Analyze short run: extract response curves with `specs/019-improved-crrcsim/sim_response.py` (may need adaptation for rate outputs). Compare control smoothness to BIG3.
- [ ] T052 [US5] If convergence OK: BIG production run (400 gens, pop=3000). Monitor early generations for fitness trajectory.
- [ ] T053 [US5] Eval suite on BIG run weights: `./scripts/eval_suite.sh <weights.dmp>`. All tiers.
- [ ] T054 [US5] Response analysis: compare sim rate commands to characterization flight data (if available from US7). Tune CRRCSim PID gains if mismatch found.
- [ ] T055 [US5] Extract weights, `nn2cpp`, build xiao with production weights. Deploy to bench for final verification.

**Checkpoint**: Trained controller with new architecture. Eval passes. Ready for flight.

---

## Phase 7: Safety Overrides (US6)

**Goal**: Distance-from-origin sphere check on xiao. Disables autoc if aircraft drifts too far.

**Independent Test**: Bench test: manually inject position >100m from origin in MSP data, verify autoc disables and log entry appears.

- [ ] T060 [US6] Implement safety check in `xiao/src/msplink.cpp`: on autoc enable, capture origin position. Each tick, compute distance. If > threshold (100m, compile-time configurable), set autoc=N and log safety event.
- [ ] T061 [US6] Add safety state to xiao log format: `safety=OK` or `safety=TRIPPED dist=X.Xm`.
- [ ] T062 [US6] Build and bench test safety override.

**Checkpoint**: Safety override active. Aircraft cannot fly away indefinitely under NN control.

---

## Phase 8: Characterization Flight + Analysis (US7)

**Goal**: Fly the choreography from `flight-choreography.md`. Collect response data for sim tuning.

**Note**: This phase runs in parallel with Phases 2-5 whenever props arrive. Data feeds back into T054 (PID tuning).

- [ ] T070 [P] [US7] Characterization flight: follow `specs/021-xiao-ahrs-crosscheck/flight-choreography.md`. MANUAL mode, no autoc. Execute roll/pitch/throttle characterization maneuvers at multiple speeds.
- [ ] T071 [P] [US7] Decode blackbox: `~/blackbox-tools/obj/blackbox_decode <file>.TXT`. Verify <10 decode failures. Check Z continuity (no corruption).
- [ ] T072 [US7] Response analysis: run `specs/019-improved-crrcsim/flight_response.py --axis all` and `compare_response.py`. Compare to sim BIG3 baseline.
- [ ] T073 [US7] Gyro analysis: compare gyroRaw vs gyroADC during step inputs. Assess 25Hz LPF lag at NN tick rate. Document filter recommendations.
- [ ] T074 [US7] ACRO segments analysis (if flown): measure rate response to stick input. Compare to CRRCSim rate PID response. Tune PID gains if needed.

**Checkpoint**: Response data collected. Sim PID tuning validated or updated.

---

## Phase 9: Flight Test

**Goal**: Fly with new NN (ACRO mode, 27 inputs, rate commands). Expect sustained tracking.

- [ ] T080 Flash INAV to flight hardware (MATEKF722MINI) with MSP gyro extension.
- [ ] T081 Deploy xiao with production weights + safety override + ACRO mode.
- [ ] T082 Ground check: arm, verify ACRO mode, servo response, safety cutoff test.
- [ ] T083 Flight test: engage autoc, observe tracking. Collect blackbox + xiao logs.
- [ ] T084 Post-flight analysis: compare flight trajectory to path, analyze rate commands vs response, check for sustained tracking >10s.

**Checkpoint**: Flight validated. Path tracking sustained. Architecture proven.

---

## Phase 10: Polish

**Purpose**: Cleanup after validation.

- [ ] T090 [P] Update `docs/COORDINATE_CONVENTIONS.md` with gyro rate wiring details.
- [ ] T091 [P] Update `specs/BACKLOG.md`: move completed items, add new findings.
- [ ] T092 Commit final configs (`inav-bench.cfg`, `inav-hb1.cfg`, `autoc.ini`, `autoc-eval.ini`).

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (config) ─────────────────────────────────────────────────────┐
                                                                      │
Phase 2 (CRRCSim rate PID) ──→ Phase 3 (NN inputs) ──┐               │
                                                       ├→ Phase 6 (training)
Phase 4 (INAV MSP) ──→ Phase 5 (xiao consumer) ──────┘       │
                                                               ↓
Phase 7 (characterization flight) ──→ feeds back to ──→ Phase 6 (PID tuning)
                                                               │
                                                               ↓
                                                        Phase 7 (safety)
                                                               │
                                                               ↓
                                                        Phase 9 (flight test)
                                                               │
                                                               ↓
                                                        Phase 10 (polish)
```

### Critical Path

```
T010-T014 (CRRCSim PID) → T020-T025 (NN inputs) → T050-T051 (short training) →
  T030-T032 (INAV MSP) → T040-T046 (xiao) → T052-T055 (BIG training) →
  T060-T062 (safety) → T080-T084 (flight)
```

### Parallel Opportunities

- **Phase 7** (characterization flight) runs in parallel with Phases 2-5
- **T020** (AircraftState) and **T024** (GoogleTest) can run in parallel
- **T030-T032** (INAV builds) can start once MSP spec is designed, parallel with T020-T025
- **T060-T062** (safety) can start after T040 (xiao basics), parallel with T052 (training)

---

## Implementation Strategy

### MVP: CRRCSim Rate PID Only (Phase 2)

1. Implement rate PID in CRRCSim
2. Train with existing 29 inputs but ACRO mode
3. Validate convergence — does ACRO help even without input changes?
4. If yes: proceed. If no: investigate PID tuning before continuing.

### Incremental

1. Phase 2 → ACRO mode in sim (MVP validation)
2. Phase 3 → New inputs (convergence comparison)
3. Phase 4+5 → INAV/xiao integration
4. Phase 6 → Production training
5. Phase 7+8 → Safety + flight test

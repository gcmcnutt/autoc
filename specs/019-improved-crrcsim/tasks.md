# Tasks: Improved CRRCSim Fidelity

**Input**: Design documents from `/specs/019-improved-crrcsim/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Organization**: Tasks are grouped by user story (axis characterization phases) to enable
independent implementation and testing. Task IDs T400+ continue from 018's T303.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Analysis tooling that all characterization phases depend on

- [ ] T400 [P] Create `scripts/flight_response.py`: extract response curves from flight blackbox CSV. Inputs: axis (throttle/roll/pitch), CSV path. Outputs: ascii art response curve (command magnitude vs response metric, binned by airspeed regime: slow <12, cruise 12-16, fast >16 m/s). Parse `rcCommand[0-2]`, `navVel[0-2]`, `navPos[0-2]`, `attitude[0-2]`, `flightModeFlags` columns. Filter to MSPRCOVERRIDE spans + pilot recovery segments.

- [ ] T401 [P] Create `scripts/sim_response.py`: extract response curves from data.dat. Inputs: axis, data.dat path, generation range. Outputs: ascii art response curve (same format as T400). Parse fields: outPt/outRl/outTh (NN commands), vel (airspeed), qw/qx/qy/qz (orientation), X/Y/Z (position). Compute attitude rates from consecutive quaternion steps, climb rate from consecutive Z.

- [ ] T402 Create `scripts/compare_response.py`: overlay flight and sim response curves side-by-side. Inputs: flight CSV + data.dat + axis. Outputs: combined ascii art showing both curves with ratio annotation. This is the primary tuning feedback tool — used in every iteration.

- [ ] T403 Create `autoc-mini.ini`: copy of `autoc.ini` with `Generations=20` for quick tuning iterations. Keep all other params (pop, paths, scenarios) identical to production config so response curves are comparable.

**Checkpoint**: Tooling ready — can extract and compare response curves from existing data.

---

## Phase 2: Throttle Characterization (CO1)

**Goal**: Get the energy/speed envelope right. Roll and pitch authority depend on airspeed.

**Independent Test**: Run `scripts/compare_response.py --axis throttle` on flight blackbox + sim data.dat. Sim Vmax, Vcruise, climb rate at full throttle, descent rate at idle should be within ~15% of flight values.

- [ ] T410 [US1] Extract flight throttle response curve using `scripts/flight_response.py --axis throttle` on `eval-results/bench-20260322/blackbox_log_2026-03-22_212133.01.csv`. Document: Vmax (full throttle level), Vcruise (~50% throttle), climb rate at full throttle, descent rate at idle. Note streamer drag signature (Vmax caps ~17 m/s).

- [ ] T411 [US1] Extract sim throttle response curve using `scripts/sim_response.py --axis throttle` on `20260323-data.dat --gen-range 1-5` (varied throttle before saturation). Also extract steady-state from gen 400 (full throttle, useful for Vmax). Document same metrics as T410.

- [ ] T412 [US1] Compare throttle curves using `scripts/compare_response.py --axis throttle`. Identify specific mismatches: Vmax ratio, climb rate ratio, descent rate ratio. Document which hb1_streamer.xml parameters to adjust (F, CD_prof, V_ref per data-model.md).

- [ ] T413 [US1] Tune throttle parameters in `crrcsim/models/hb1_streamer.xml`: adjust F (thrust), CD_prof (parasitic drag), V_ref (engine reference speed) based on T412 mismatch ratios. Document each change with rationale.

- [ ] T414 [US1] Run mini training (20 gens) with `autoc-mini.ini` to regenerate sim response curves. Extract new throttle curve with `scripts/sim_response.py`. Re-compare. Iterate T413-T414 until throttle curves converge.

**Checkpoint**: Sim speed envelope matches flight within ~15%. Vmax, climb rate, descent rate comparable.

---

## Phase 3: Roll Characterization (CO2)

**Goal**: Roll rate per unit command matches flight within 2×.

**Independent Test**: Run `scripts/compare_response.py --axis roll`. Sim roll rate per unit command at cruise speed within 0.5-2.0× of flight.

- [ ] T420 [US2] Extract flight roll response curve using `scripts/flight_response.py --axis roll` on flight blackbox. Compute roll rate (deg/s) per unit rcCommand at slow/cruise/fast airspeeds. Use single logged elevon + symmetry assumption for deflection estimate.

- [ ] T421 [US2] Extract sim roll response curve using `scripts/sim_response.py --axis roll` on data.dat. Use early gens where roll varies (BIG3 roll output: mean -0.04, 8.7% right, 16.2% left — earlier gens more varied). Filter to steps where pitch/throttle are ~steady.

- [ ] T422 [US2] Compare roll curves using `scripts/compare_response.py --axis roll`. Identify mismatch ratio across speed bins. 018 estimate: sim ~2× slow.

- [ ] T423 [US2] Tune roll parameters in `crrcsim/models/hb1_streamer.xml`: adjust Cl_da (roll effectiveness), Cl_p (roll damping) if needed. Roll was close in 018 (sim 287°/s vs flight 270°/s) so changes may be minimal.

- [ ] T424 [US2] Run mini training, extract roll curve, re-compare. Iterate T423-T424 until roll curves converge. Verify throttle curves (Phase 2) haven't regressed.

**Checkpoint**: Roll rate within 2× of flight. Throttle envelope still good.

---

## Phase 4: Pitch Characterization (CO3)

**Goal**: Pitch rate per unit command within 2× of flight. Full pitch + full throttle must not produce survivable looping in sim.

**Independent Test**: Run `scripts/compare_response.py --axis pitch`. Sim pitch rate at cruise within 0.5-2.0× of flight. Manual test: full pitch + full throttle from level flight in sim produces stall/departure, not stable loop.

- [ ] T430 [US3] Extract flight pitch response curve using `scripts/flight_response.py --axis pitch` on flight blackbox. Pilot recovery pulls (-400 to -480 rcPitch) are clean step-like inputs — best source. Compute pitch rate per unit command at slow/cruise/fast.

- [ ] T431 [US3] Extract sim pitch response curve using `scripts/sim_response.py --axis pitch` on data.dat. Gen 1 (pitch mean=0.83, not yet saturated) is the best source for transients. If insufficient clean data, note need for sim step test.

- [ ] T432 [US3] Compare pitch curves using `scripts/compare_response.py --axis pitch`. Document mismatch ratio. Current: 5-7× (018 data). After 018 tuning (Cm_de -0.32→-0.19, Cm_q -3.0→-4.2): still ~2.3× too fast.

- [ ] T433 [US3] Tune pitch parameters in `crrcsim/models/hb1_streamer.xml`: adjust Cm_de (effectiveness), Cm_q (damping/streamer model), potentially I_yy (pitch inertia) or CL_max (stall). Document each change.

- [ ] T434 [US3] Run mini training, extract pitch curve, re-compare. Iterate T433-T434 until pitch curves converge. Verify roll and throttle curves haven't regressed.

- [ ] T435 [US3] Stall/departure test: in a mini training run, verify that full pitch + full throttle from level flight produces stall or departure, not survivable looping. Check data.dat: aircraft should crash or depart within 2-3s of sustained full pitch, not oscillate indefinitely.

**Checkpoint**: Pitch rate within 2× of flight. Full pitch departs. Roll and throttle still good.

---

## Phase 5: Integration & Iteration (CO1-CO3)

**Goal**: Cross-axis validation. All three axes match flight simultaneously. Flight replay segments show reasonable agreement.

**Independent Test**: Overlay all three axis curves. Mini training run (20 gens) produces non-saturated, varied NN outputs. Flight replay 1-2s segments show trajectory within 2× of actual.

- [ ] T440 [US4] Overlay all three axis response curves (sim vs flight) using `scripts/compare_response.py --axis all`. Identify cross-axis regressions: did pitch tuning break throttle? Did throttle speed change alter roll authority?

- [ ] T441 [US4] If cross-axis regressions found, iterate back to the affected axis phase (T413/T423/T433) and re-tune. Re-validate all axes after each adjustment.

- [ ] T442 [US4] Flight replay validation: extract NN commands from flight blackbox MSPRCOVERRIDE spans, play through CRRCSim from matched initial conditions (position, velocity, attitude). Compare resulting trajectory over 1-2s segments. This may require a replay mode in the sim or a custom script. Document divergence.

- [ ] T443 [US4] Run a 20-gen mini training with final tuned model. Verify NN outputs are varied (not saturated) and response curves in data.dat resemble flight blackbox curves. This is a pre-validation of CO7 before investing in full training.

**Checkpoint**: All axes within 2× of flight simultaneously. Mini training shows non-degenerate behavior.

---

## Phase 6: Sensor Scaling Audit (Pre-flight Gate)

**Goal**: Verify unit consistency across INAV→xiao→NN and CRRCSim→autoc→NN paths.

**Independent Test**: Pick 3-5 matched physical states (sim + flight). NN input vectors are numerically comparable at the same physical state.

- [ ] T450 [P] [US5] Code inspection — INAV side: read MSP2_INAV_LOCAL_STATE response format in INAV source (`~/inav/src/main/fc/fc_msp.c`). Document exact fields, types, units, coordinate frame. Trace through xiao `msplink.cpp` parser to NN input computation. Reference `docs/COORDINATE_CONVENTIONS.md`.

- [ ] T451 [P] [US5] Code inspection — CRRCSim side: read FDM state extraction in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`. Document where ft→m, ft/s→m/s conversions happen. Trace through to NN input computation in autoc evaluator. Reference `docs/COORDINATE_CONVENTIONS.md`.

- [ ] T452 [US5] Compare normalizers: verify dPhi/dTheta/dist are computed identically in both paths (same formula, same units, same sign conventions). Document any discrepancies. Check if 018 NED/NEU fix introduced compensating errors.

- [ ] T453 [US5] Spot-check: pick 3-5 matched physical states from flight blackbox and sim data.dat (e.g., straight-level at 13 m/s heading north). Compare NN input vectors numerically. Differences >10% flag a scaling issue.

**Checkpoint**: Sensor scaling verified or discrepancies documented and fixed.

---

## Phase 7: Pipeline Latency + Retrain (CO4, CO5, CO6, CO7)

**Goal**: Reduce pipeline latency, lock in sim latency, produce a non-degenerate trained controller.

**Independent Test**: Bench-measured pipeline latency matches COMPUTE_LATENCY. BIG training data.dat response curves visually resemble flight blackbox curves.

### INAV Firmware (~/inav, autoc branch)

- [ ] T460 [P] [US6] Fix servo logging in `~/inav/src/main/blackbox/blackbox.c`: change servo index loop to start from `minServoIndex` instead of 0. Flying wing: logs servo[1] and servo[2] (both elevons) instead of servo[0] (unused) and servo[1].

- [ ] T461 [P] [US6] Add MSP2_AUTOC_STATE handler in `~/inav/src/main/fc/fc_msp.c`: register command ID in MSP2 user range (0x4000-0x7FFF). Pack payload per data-model.md: pos[3] int32 cm, vel[3] int16 cm/s, quat[4] int16 /10000, rc[4] uint16 PWM, armingFlags uint32. Total 38 bytes.

- [ ] T462 [US6] Build INAV autoc branch with T460+T461 changes. Verify compiles for STM32F405 target.

### Xiao Firmware (xiao/)

- [ ] T463 [P] [US6] Add MSP2 request/response support in `xiao/src/MSP.cpp` if not already present (MSP v2 framing with CRC8).

- [ ] T464 [US6] Replace 3 MSP calls in `xiao/src/msplink.cpp` with single MSP2_AUTOC_STATE request. Parse 38-byte response into existing state fields (pos, vel, quat, rc, armingFlags). Keep MSP_STATUS as fallback (polled every 10th tick) for diagnostics during development.

- [ ] T465 [P] [US6] Add flight mode channel override in `xiao/src/msplink.cpp`: include channel 6 in `msp_override_channels`, set to 1000 (MANUAL mode). Removes pilot transmitter switch dependency.

- [ ] T466 [US6] Build xiao firmware with T463-T465 changes: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`.

### Bench Testing

- [ ] T467 [US6] Flash INAV (T462) and xiao (T466) to flight hardware. Bench test:
  - Verify both elevons appear in blackbox CSV (T460 fix)
  - Confirm blackbox sample rate at 1/8 (higher resolution for next flight)
  - Measure pipeline latency with custom MSP command: log fetch time in xiao, compare to 018 baseline (35ms fetch → target 8-12ms)

- [ ] T468 [US6] Record bench-measured fetch latency from T467. Update `COMPUTE_LATENCY_MSEC_DEFAULT` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` to match total measured pipeline (fetch + eval + send).

### Training

- [ ] T469 [US6] BIG training run with calibrated hb1_streamer.xml (Phases 2-4) + updated COMPUTE_LATENCY (T468). Use production `autoc.ini` (400 gens, pop=3000).

- [ ] T470 [US6] Validate CO7: extract response curves from T469 data.dat using `scripts/sim_response.py`. Compare to flight blackbox using `scripts/compare_response.py --axis all`. Response curves should visually resemble flight data across the operating envelope.

**Checkpoint**: Calibrated sim + correct latency + non-degenerate training. Ready for flight test.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Cleanup and documentation

- [ ] T480 [P] Document final hb1_streamer.xml parameter values and tuning rationale in `crrcsim/models/hb1_streamer.xml` XML comments (following 018 convention).
- [ ] T481 [P] Update `docs/COORDINATE_CONVENTIONS.md` with any findings from sensor scaling audit (Phase 6).
- [ ] T482 [P] Update `docs/sensor-pipeline.md` with custom MSP2_AUTOC_STATE command documentation.
- [ ] T483 Archive BIG3 018 training artifacts: note in `eval-results/` that `20260323-data.*` is the degenerate baseline.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately
- **Throttle (Phase 2)**: Depends on Setup (T400-T402 tooling)
- **Roll (Phase 3)**: Depends on Setup. Can start after Phase 2 (throttle envelope needed for accurate speed bins)
- **Pitch (Phase 4)**: Depends on Setup. Should follow Phase 2-3 (airspeed and roll context)
- **Integration (Phase 5)**: Depends on Phases 2-4 all reaching initial convergence
- **Sensor Audit (Phase 6)**: Independent of Phases 2-5 tuning. Can run in parallel with tuning. BLOCKS Phase 7 training.
- **Pipeline + Retrain (Phase 7)**: Depends on Phases 2-5 (calibrated model) AND Phase 6 (scaling verified). INAV/xiao firmware tasks (T460-T466) can start in parallel with tuning.
- **Polish (Phase 8)**: After Phase 7

### Parallel Opportunities

- T400, T401 can run in parallel (different scripts, different data sources)
- T450, T451 can run in parallel (INAV vs CRRCSim code inspection)
- T460, T461 can run in parallel (different INAV source files)
- T463, T465 can run in parallel (different xiao features)
- INAV/xiao firmware (T460-T466) can be developed in parallel with sim tuning (Phases 2-4)
- Sensor audit (Phase 6) can run in parallel with tuning (Phases 2-4)

### Within Each Tuning Phase

- Extract flight curve → Extract sim curve → Compare → Tune → Mini train → Re-compare
- Iterate until convergence (not count-limited)
- Verify previous axes haven't regressed after each tuning pass

---

## Implementation Strategy

### MVP First (Phase 2: Throttle Only)

1. Complete Phase 1: Setup (analysis tooling)
2. Complete Phase 2: Throttle characterization
3. **STOP and VALIDATE**: Throttle curves match. FDM mental model established.
4. Proceed to Roll, then Pitch

### Parallel Work Streams

While tuning (Phases 2-4), in parallel:
- Start INAV/xiao firmware work (T460-T466) — independent codebase
- Start sensor scaling audit (T450-T453) — code inspection, no build dependency

### Final Gate

Phase 7 T470 is the exit gate: data.dat response curves from BIG training must
visually resemble flight blackbox curves. If not, iterate back to tuning phases.

---

## Notes

- Task IDs T400-T483 span this feature (continuing from 018's T303)
- Tuning iteration is convergence-driven — iterate until curves match, not a fixed count
- Mini training runs (20 gens) are the feedback mechanism, not full BIG runs
- Full controls are normal for attitude flying — the metric is response fidelity, not saturation avoidance
- INAV stays on 8.0.0 autoc branch — minimal local changes only
- Bench test pipeline latency before updating sim — the measured value drives COMPUTE_LATENCY

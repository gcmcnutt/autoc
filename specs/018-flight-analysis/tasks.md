# Tasks: Flight Analysis & Sim-to-Real Verification

**Input**: Design documents from `/specs/018-flight-analysis/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. Task IDs (T200+) continue from 015's numbering.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Tooling prerequisites and existing verification results

- [x] T200 [P] Position X/Y correlation verified: xiao `pos_raw` matches INAV `navPos[0,1]/100` within 0.01m
- [x] T201 [P] Position Z convention verified: `xiao_z = -inav_z/100` (intentional NEU→NED)
- [x] T202 [P] Quaternion conjugation verified: INAV body→earth conjugated to earth→body, matches within 0.02
- [x] T203 Timestamp correlation verified: xiao col3 correlates with INAV `time(us)/1000`, ~200ms MSP latency, stable across 5 spans
- [x] T204 Board alignment confirmed: flight hardware `align_board_yaw=0`, bench has 138° offset (bench-specific IMU mount)
- [x] T220 [P] Renderer log format updated: GP State→Nav State, GP Control→NN Control, compact NN line parsing
- [x] T221 Basic flight tape rendering works in `-x` mode (xiao-only)

**Checkpoint**: Pipeline verified at wire level. Tooling foundation ready.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core analysis tools that all user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T205 Create automated join tool in `scripts/correlate_flight.py`: takes INAV CSV + xiao flight log, produces joined timeline with matched correlated ticks (±50ms). Output per-tick: time, xiao pos/vel/quat, inav pos/vel/quat, deltas. Based on `/tmp/correlate_flight.py` prototype.

- [ ] T208 Document sensor pipeline in `docs/sensor-pipeline.md`: full chain INAV sensors → MSP wire → xiao receive → aircraft_state → NN inputs. Include coordinate conventions (NED/NEU, reference `docs/COORDINATE_CONVENTIONS.md`), unit conversions (cm→m, decideg→rad, quat×10000→float), sign flips, conjugation, board alignment.

- [ ] T250 Parse NN inputs from `data.dat` in `tools/renderer.cc`: determine what cereal-serialized data.dat stores per tick (29 NN inputs or raw aircraft state). If raw state, recompute NN inputs. Extract dPhi/dTheta/dist/quat for projection.

**Checkpoint**: Join tool and data access ready — user story implementation can begin.

---

## Phase 3: User Story 1 — Sensor Pipeline Proof (Priority: P1)

**Goal**: Prove the data pipeline from INAV sensors to NN inputs is correct, with clear documentation of every conversion.

**Independent Test**: Run join tool on 2026-03-20 flight data. Position X/Y error < 0.5m. Z shows consistent negation. Timestamps correlate within 250ms.

- [ ] T206 [P] [US1] Velocity correlation in `scripts/correlate_flight.py`: verify xiao vel matches INAV `navVel/100` (cm/s→m/s). X/Y partially checked (~0.06m/s), Z needs negation verification.

- [ ] T207 [US1] MSP latency characterization in `scripts/correlate_flight.py`: from joined data, measure actual temporal offset (transport delay vs polling interval). Document implications for control stability (at 16 m/s, 200ms = 3.2m uncompensated motion).

- [ ] T209 [P] [US1] Cross-ISA NN output comparison: synthesize 1000+ NN input vectors from sim, feed to xiao Cortex-M4F via `pio test` in `xiao/test/`, compare outputs to desktop aarch64. Acceptance: max divergence < 0.01 per output. Key deliverable: characterize FP latency/drift between aarch64 and Cortex-M4F — this determines if weights trained on desktop are viable on flight hardware or need quantization/retraining.

- [ ] T210 [US1] Post-flight NN replay: take flight log NN input lines, feed to desktop NN (extend `scripts/verify_flight_log.py`), compare to xiao's actual outputs. Quantifies real-world FP drift.

**Checkpoint**: Pipeline correctness proven with documented evidence. Sensor-pipeline.md complete.

---

## Phase 4: User Story 2 — Rabbit Position Visualization (Priority: P1)

**Goal**: Visualize where the rabbit actually was AND where the NN thought it was. Misalignment = coordinate convention bug.

**Independent Test**: On sim data, projected rabbit MUST overlay actual rabbit within 1m. On flight data, direct rabbit matches known path geometry.

### Sim Validation First (prove the math)

- [ ] T251 [US2] Add projected rabbit overlay for sim data in `tools/renderer.cc`: for each sim tick, take dPhi/dTheta/dist/quat from data.dat, project to world coords, render as cyan path.

- [ ] T252 [US2] Verify projection alignment: run renderer with BIG-3 tier0-repro eval. Cyan projected path must lie exactly on red actual path. Fix until aligned.

### Pre-flight firmware changes (blockers for next flight)

- [ ] T221a Remove MSP_ARM_CYCLE_COUNT delay in `xiao/include/main.h` and `xiao/src/msplink.cpp`:
  set MSP_ARM_CYCLE_COUNT=0 or remove countdown logic. INAV's own freshness guard (790ms,
  from `failsafe_recovery_delay=5`) already provides safety — our 200ms delay on top is
  unnecessary and just means the NN commands for 200ms before INAV listens. Confirmed by
  latency analysis: all 5 spans show consistent 790ms override activation delay regardless.

- [ ] T221b Use MANUAL flight mode (not ACRO) for NN flights. In ACRO mode, INAV treats
  RC commands as **rate commands** processed through expo → rate scaling → PID controller →
  mixer. The sim applies NN output directly to control surfaces. MANUAL mode is closest
  to sim behavior — rcCommand goes more directly to servos. Set via INAV mode tab or
  transmitter switch assignment.

- [ ] T221c Refactor to single 20Hz loop in `xiao/src/controller.cpp` and `xiao/src/msplink.cpp`:
  Remove the separate 50ms ticker ISR. Replace with a single 50ms main loop:
  - Every tick (50ms): send cached RC commands (heartbeat, keeps INAV >5Hz)
  - Every Nth tick (N=2 → 10Hz, matching training): fetch MSP state → NN eval →
    update cache → send RC immediately (same tick, minimal latency)
  Currently the 10Hz NN eval and 20Hz send ticker are unsynchronized independent timers
  with ISR bus contention — worst case 49ms wasted latency. Single loop eliminates:
  - ISR/task bus lock contention (`tryLockMspBusFromIsr` vs `tryLockMspBusFromTask`)
  - Random 0-49ms send delay after NN eval
  - `mspSendPending` recovery path
  Expected: pipeline latency drops from ~48ms to ~5ms on NN ticks.
  MSP_UPDATE_INTERVAL_MSEC stays 100ms, MSP_SEND_INTERVAL_MSEC stays 50ms.
  Future: N=1 (20Hz NN) is feasible if MSP fetch+NN+send fits in 50ms (~38ms measured),
  but would require retraining at 20Hz.
  **IMPORTANT**: crrcsim has `COMPUTE_LATENCY_MSEC_DEFAULT=40` in `inputdev_autoc.h` —
  the NN was trained with 40ms delay between sensor sample and command application.
  Real measured pipeline is ~48ms (close match!). If T221c reduces real latency to ~5ms,
  must also reduce sim latency to match, OR keep sim at 40ms as conservative margin.
  Env override: `AUTOC_COMPUTE_LATENCY_MSEC=N`. Decision depends on T221c results.

- [ ] T222 [P] [US2] Add rabbit world position to xiao NN log line in `xiao/src/msplink.cpp`: append `rabbit=[x,y,z]` (world-relative) to NN line. Position already computed in `getInterpolatedTargetPosition()`.

- [ ] T222b [US2] Bench test rabbit logging: upload firmware, run bench test with path 0, verify rabbit position in log follows expected StraightAndLevel path geometry (20m south, 180° turn, 40m north). Cross-check first/last rabbit position against path definition.

- [ ] T223 [US2] Renderer: parse direct rabbit position from NN line in `tools/renderer.cc`, render as red path overlay + blue error bars (aircraft→rabbit).

- [ ] T224b [US2] Fix inverse projection math in `tools/renderer.cc`: handle atan2 front/back ambiguity using closed-form solution from research.md (sign disambiguation via `cos(dTheta)`). Clamp near singularities.

- [ ] T224c [US2] Render projected rabbit as cyan alongside direct rabbit (red) in `tools/renderer.cc`. Misalignment reveals convention mismatches.

- [ ] T253 [US2] Apply working projection to flight log data. Compare cyan (projected) vs red (direct). Document any convention mismatches found.

- [ ] T225 [US2] Renderer HUD overlay in `tools/renderer.cc`: display current dPhi, dTheta, dist, quat, NN outputs while scrubbing through flight timeline.

**Checkpoint**: Dual rabbit rendering working on both sim and flight data. Convention mismatches identified.

---

## Phase 5: User Story 3 — Output Mapping Verification (Priority: P2)

**Goal**: Verify NN commands produce correct aircraft response (right sign, right channel, right gain).

**Independent Test**: Plot NN command vs attitude rate response for each axis. Sign must be consistent.

- [ ] T230 [US3] Document NN output→RC mapping: compare crrcsim `inputdev_autoc.cpp` with xiao `msplink.cpp`. Verify channel assignment (out[0]=pitch? roll? throttle?), sign conventions, range mapping ([-1,1] → [1000,2000]). Document in `docs/sensor-pipeline.md`.

- [ ] T231 [US3] INAV RC override path: examine INAV source (`~/inav/`, branch autoc) for `MSP_SET_RAW_RC` handling. Determine if values go direct to servos/ESC or through PID/mixer. Document.

- [ ] T232 [US3] Command-response verification in `scripts/correlate_flight.py`: from correlated data, verify pitch-up command → aircraft pitches up (within 300ms transport delay). Same for roll and throttle. Flag any inverted axes.

**Checkpoint**: Output mapping documented and verified. Any sign flips identified and flagged for firmware fix.

---

## Phase 6: User Story 4 — Aircraft Dynamics Identification (Priority: P2)

**Goal**: Measure real aircraft control rates, delays, power, drag. Feed back into crrcsim model.

**Independent Test**: Measured pitch rate gain from flight data compared to crrcsim prediction. Gap quantified.

### Control Response Identification

- [ ] T240 [P] [US4] Pitch axis characterization in `scripts/flight_charts.py`: from correlated data, extract transport delay (ms), rate gain (deg/s per unit), bandwidth (Hz), damping.

- [ ] T241 [P] [US4] Roll axis characterization in `scripts/flight_charts.py`: roll rate per unit command, time constant, roll-yaw coupling.

- [ ] T242 [P] [US4] Throttle/speed characterization in `scripts/flight_charts.py`: max climb rate, drag deceleration, speed response delay. Compare to crrcsim engine model.

### Model Calibration

- [ ] T243 [US4] Measured vs simulated comparison: replay NN commands through crrcsim, compare attitude/position trajectories. Quantify sim-to-real gap per axis.

- [ ] T244 [US4] Update `crrcsim/models/hb1.xml`: adjust FDM parameters (CL/CD, control effectiveness, thrust curve, mass/inertia) to match measured characteristics. Document each change.

- [ ] T245 [US4] Validation run: BIG-3 eval suite against updated model. Fitness within 2× of original baseline.

### Design Center

- [ ] T246 [US4] Define variation envelope: from T240-T242 measurements + manufacturing tolerances, define ±range for each parameter. Input to Phase 6 training (T140-T143 from 015).

- [ ] T247 [US4] Retrain with calibrated model: BIG training with updated hb1.xml + aircraft variations. Compare robustness to BIG-3 baseline.

**Checkpoint**: Calibrated aircraft model. Sim-to-real gap quantified. Variation ranges defined.

---

## Phase 7: User Story 5 — AHRS Reliability Assessment (Priority: P2)

**Goal**: Assess sensor quality — is INAV's AHRS good enough for NN control?

**Independent Test**: Heading from quaternion matches GPS ground track within 10° in straight flight. Drift < 0.5 deg/min during stationary.

- [ ] T260 [P] [US5] Heading consistency in `scripts/flight_charts.py`: compare quaternion yaw vs GPS ground track (`atan2(vel_e, vel_n)`). Plot difference vs time across all test spans.

- [ ] T261 [P] [US5] Attitude drift in `scripts/flight_charts.py`: during stationary periods, measure quaternion drift rate (deg/min for roll, pitch, yaw).

- [ ] T262 [P] [US5] Position accuracy in `scripts/flight_charts.py`: GPS noise during stationary periods. Compute CEP and altitude std dev. Compare to NN tracking resolution (~5-10m).

- [ ] T263 [P] [US5] Velocity vector analysis in `scripts/flight_charts.py`: compare velocity direction vs heading quaternion. Difference = sideslip + wind.

- [ ] T264 [US5] Generate standard flight analysis charts: heading, attitude, position scatter, velocity vs time. Output to `eval-results/flight-YYYYMMDD/analysis/`.

- [ ] T265 [P] [US5] AHRS vs bank angle analysis in `scripts/flight_charts.py`: plot heading error (AHRS vs GPS ground track) as a function of bank angle. Does heading accuracy degrade above 30°? 45°? 60°? Identify the attitude envelope where AHRS is trustworthy.

- [ ] T266 [P] [US5] GPS update rate vs attitude in `scripts/flight_charts.py`: from blackbox GPS frames, check if GPS update rate drops or position jumps increase during aggressive maneuvers. Plot GPS fix quality (EPH/EPV from navEPH/navEPV columns) vs bank angle.

- [ ] T267 [P] [US5] Gyro-only attitude reconstruction in `scripts/flight_charts.py`: integrate raw gyro rates (gyroADC columns) over short windows (1-5s) and compare to AHRS quaternion. Quantify drift rate of pure gyro integration. This establishes the fallback accuracy if AHRS fusion fails — relevant to whether a simpler sensor model (gyro + vision) is viable for 017.

**Checkpoint**: AHRS quality assessment complete. Known biases and drift rates documented.

---

## Phase 8: User Story 6 — Temporal Jitter Characterization (Priority: P2)

**Goal**: Measure real MSP timing behavior. Determine if simulator needs timing noise injection.

**Independent Test**: MSP interval statistics: mean ~100ms, std dev < 10ms.

- [ ] T270 [P] [US6] MSP update interval measurement in `scripts/correlate_flight.py`: from xiao timestamps, compute inter-sample intervals (mean, std, min, max, histogram). Flag dropped/delayed samples.

- [ ] T271 [P] [US6] NN eval jitter in `scripts/correlate_flight.py`: measure Nav State→NN line latency per tick. Expected 3-8ms. Check consistency in flight vs bench.

- [ ] T272 [US6] End-to-end pipeline delay in `scripts/correlate_flight.py`: total latency from INAV sensor → actuator command. MSP transport + NN eval + RC override send.

- [ ] T273 [US6] Simulator jitter injection: based on T270-T272 findings, add `SimTickJitter` config parameter in `autoc.ini` and `src/autoc.cc`. Training with variable sample intervals.

- [ ] T274 [US6] Sample rate increase feasibility: evaluate 10Hz→20Hz xiao MSP poll rate. Check MSP bandwidth, NN eval budget, INAV state update rate. Document trade-offs.

**Checkpoint**: Timing characterized. Jitter injection implemented if needed.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T280 [P] Move flight analysis doc to `specs/015-nn-training-improvements/flight-analysis-20260320.md` → `docs/` for long-term reference
- [ ] T281 [P] Quickstart.md validation: run full analysis workflow end-to-end on 2026-03-20 flight data per `quickstart.md`
- [ ] T282 Add `eval-results/` and `core*` to `.gitignore` (partially done)
- [ ] T283 Carried forward from 015: T124c (document genome.fitness), T125 (per-scenario scores in data.stc), T163 (remove SinglePathProvider), T164 (GP legacy cleanup), T165 (remove sum mode), T166 (memory leak check)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Complete — verified in prior session
- **Foundational (Phase 2)**: T205/T208 (join tool + docs) and T250 (data.dat parsing) — BLOCKS all stories
- **US1 & US2 (Phases 3-4)**: Both P1, can proceed in parallel after Phase 2
- **US3-US6 (Phases 5-8)**: All P2, can proceed in parallel after Phase 2, but benefit from US1/US2 findings
- **Polish (Phase 9)**: After all desired stories complete

### User Story Dependencies

- **US1 (Pipeline Proof)**: After Phase 2 — no dependencies on other stories
- **US2 (Rabbit Viz)**: After Phase 2 — T251/T252 (sim validation) blocks T253 (flight application). T222 (firmware) blocks T223 (renderer)
- **US3 (Output Mapping)**: After Phase 2 — benefits from US1 correlation tool
- **US4 (Dynamics)**: After US1+US3 confirmed (can't measure dynamics if sensors/outputs wrong)
- **US5 (AHRS)**: After Phase 2 — independent of other stories, but informs US4 confidence
- **US6 (Jitter)**: After Phase 2 — independent, informs US4 timing assumptions

### Parallel Opportunities

- All Phase 1 tasks [P] were completed in parallel during prior session
- T206, T209 can run in parallel (different files: correlate script vs pio test)
- T240, T241, T242 can all run in parallel (independent axis analysis in same script)
- T260, T261, T262, T263 can all run in parallel (independent chart generators)
- T270, T271 can run in parallel (different timing measurements)

---

## Implementation Strategy

### MVP First (US1 + US2)

1. Complete Phase 2: join tool + data.dat parsing
2. Complete US1: pipeline proof with documentation
3. Complete US2: rabbit visualization — sim first, then flight
4. **STOP and VALIDATE**: Can we see where the NN thought the rabbit was vs where it actually was?
5. This alone identifies the coordinate convention bugs that caused the bad flight

### Incremental Delivery

1. Phase 2 → Foundation ready
2. US1 + US2 → Pipeline + visualization (MVP — identifies convention bugs)
3. US3 → Output mapping (confirms or rules out sign/channel issues)
4. US5 + US6 → Sensor quality + timing (characterizes real-world noise)
5. US4 → Dynamics + model calibration (final step: calibrate sim, retrain)

---

## Notes

- Task IDs T200-T274 continue from 015's numbering (T100-T191)
- Completed tasks from the 015 flight analysis session are preserved as [x]
- Phase ordering follows the "prove math on sim first, then apply to flight" principle
- US4 (dynamics) is last because it requires all other stories to be resolved first — can't measure real dynamics from data with corrupted sensors or wrong output mapping

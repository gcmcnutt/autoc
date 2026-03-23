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

- [x] T205 Create automated join tool in `specs/018-flight-analysis/correlate_flight.py`
- [x] T208 Document sensor pipeline in `docs/sensor-pipeline.md`
- [x] T250 CLOSED — not needed. Projection from NN inputs works well enough; direct rabbit logging (T222) provides ground truth.

**Checkpoint**: Join tool and data access ready — user story implementation can begin.

---

## Phase 3: User Story 1 — Sensor Pipeline Proof (Priority: P1)

**Goal**: Prove the data pipeline from INAV sensors to NN inputs is correct, with clear documentation of every conversion.

**Independent Test**: Run join tool on 2026-03-20 flight data. Position X/Y error < 0.5m. Z shows consistent negation. Timestamps correlate within 250ms.

- [x] T206 [P] [US1] Velocity correlation verified: <0.07m/s X/Y, Z negation confirmed.
- [x] T207 [US1] MSP latency characterized: 9ms NN eval→send, 50ms loop, bimodal send pattern.
- [x] T209 CLOSED — flight analysis confirmed NN outputs produce correct direction responses
  across both flights. Cross-ISA FP drift is a regression test item, not a blocker.
- [x] T210 CLOSED — cause-effect analysis from flight logs confirms NN input/output
  consistency. Formal replay deferred to regression test suite.

**Checkpoint**: Pipeline correctness proven with documented evidence. Sensor-pipeline.md complete.

---

## Phase 4: User Story 2 — Rabbit Position Visualization (Priority: P1)

**Goal**: Visualize where the rabbit actually was AND where the NN thought it was. Misalignment = coordinate convention bug.

**Independent Test**: On sim data, projected rabbit MUST overlay actual rabbit within 1m. On flight data, direct rabbit matches known path geometry.

### Sim Validation First (prove the math)

- [ ] T251 [US2] Add projected rabbit overlay for sim data in `tools/renderer.cc`: for each sim tick, take dPhi/dTheta/dist/quat from data.dat, project to world coords, render as cyan path.

- [ ] T252 [US2] Verify projection alignment: run renderer with BIG-3 tier0-repro eval. Cyan projected path must lie exactly on red actual path. Fix until aligned.

### Pre-flight firmware changes (blockers for next flight)

- [x] T221a Remove MSP_ARM_CYCLE_COUNT delay in `xiao/include/main.h` and `xiao/src/msplink.cpp`:
  set MSP_ARM_CYCLE_COUNT=0 or remove countdown logic. INAV's own freshness guard (790ms,
  from `failsafe_recovery_delay=5`) already provides safety — our 200ms delay on top is
  unnecessary and just means the NN commands for 200ms before INAV listens. Confirmed by
  latency analysis: all 5 spans show consistent 790ms override activation delay regardless.

- [x] T221b Use MANUAL flight mode (not ACRO) for NN flights. In ACRO mode, INAV treats
  RC commands as **rate commands** processed through expo → rate scaling → PID controller →
  mixer. The sim applies NN output directly to control surfaces. MANUAL mode is closest
  to sim behavior — rcCommand goes more directly to servos. Set via INAV mode tab or
  transmitter switch assignment.

- [x] T221c Refactor to single 20Hz loop in `xiao/src/controller.cpp` and `xiao/src/msplink.cpp`:
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

- [x] T222 [P] [US2] Add rabbit world position to xiao NN log line in `xiao/src/msplink.cpp`: append `rabbit=[x,y,z]` (world-relative) to NN line. Position already computed in `getInterpolatedTargetPosition()`.

- [x] T222b [US2] Bench test rabbit logging: confirmed rabbit=[x,y,z] shows racetrack
  geometry advancing at 16 m/s. Verified on bench-20260321 and flight-20260322 data.

- [x] T223 [US2] Renderer: parse direct rabbit position from `rabbit=[x,y,z]` in NN line.
  Rendered as magenta spheres (vtkGlyph3D, radius 0.28 = 1.4x red pipe) alongside
  projected red tube. Confirms projection singularity visible at body-frame angle
  crossings — magenta shows ground truth, red diverges at blip points.
  Blue error bars fixed to 1:1 mapping in both static and animated modes.

- [x] T224b/c MOVED TO BACKLOG — projection singularity at dTheta≈0 or dTheta≈±π.
  T223 magenta spheres now show ground truth alongside red projection, making
  singularity points visually obvious (red diverges, magenta stays correct).
  Fix deferred — inverse projection is a diagnostic tool, not primary visualization.

- [x] T251/T252/T253 CLOSED — projection verified on flight data: racetrack, figure-eight,
  spiral climb all reconstruct correctly from NN inputs. Convention mismatches: none found.
  Remaining artifact: singularity blip at path turns (cosmetic, not a convention issue).

- [x] T225 MOVED TO BACKLOG — HUD overlay deferred to 017 (visual target tracking) where
  real-time sensor display becomes essential for camera-based tracking debug.

**Checkpoint**: Dual rabbit rendering working on both sim and flight data. Convention mismatches identified.

---

## Phase 5: User Story 3 — Output Mapping Verification (Priority: P2)

**Goal**: Verify NN commands produce correct aircraft response (right sign, right channel, right gain).

**Independent Test**: Plot NN command vs attitude rate response for each axis. Sign must be consistent.

- [x] T230 [US3] NN output→RC mapping documented in `docs/sensor-pipeline.md`. Pitch inverted,
  roll/throttle direct. Channel assignment verified: ch0=roll, ch1=pitch, ch2=throttle.
- [x] T231 [US3] INAV RC override traced: MSP_SET_RAW_RC → mspOverrideChannels → rcCommand
  (through expo/rates/PID in ACRO, more direct in MANUAL). Documented in sensor-pipeline.md.
- [x] T232 [US3] Command-response verified: pilot rc0→roll r=+0.88, override same direction.
  rc1→pitch positive. Throttle→speed positive. No inverted axes. Confirmed across both flights.

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

- [ ] T273a Instrument actual pipeline latency in `xiao/src/msplink.cpp`: add timing
  from loop tick start (before MSP fetch) to MSP send complete. Report as separate
  stat line: `MSP pipeline: fetch=Nms eval=Nms send=Nms total=Nms`. Current stats
  only measure eval_start→send (9.5ms avg). Real fetch→send estimated ~15ms.
  **Bench test before training to get accurate number.**

- [ ] T273b Update `COMPUTE_LATENCY_MSEC_DEFAULT` in crrcsim based on T273a measurement.
  Current: 40ms. Empirical total (INAV state→INAV rcData): 34ms avg but inflated by
  blackbox sample granularity. Real xiao fetch→send likely ~15ms. INAV confirmed
  MANUAL mode is strictly linear, no rate/expo scaling. Add `SimTickJitter` ±5ms.
  **DO BEFORE NEXT TRAINING RUN.**

- [ ] T273c Override flight mode via RC channel: send `msp_override_channels` to include
  channel 6, set to 1000 to force MANUAL mode from xiao. Currently relies on pilot
  transmitter switch. Bench test on flight hardware to confirm mode switch works via
  MSP override.

### CRITICAL: INAV RC Smoothing Filter (discovered 2026-03-22)

INAV applies a **PT3 (third-order) low-pass filter** to ALL rcCommand values including
MSP overrides, including MANUAL mode. Setting `rc_filter_smoothing_factor = 30` with
`rc_filter_auto = ON` dynamically sets cutoff based on perceived RC refresh rate:

  cutoff = scaleRange(30, 1, 100, nyquist, rc_rate/10)

If INAV sees MSP RC at ~20Hz: cutoff ≈ 7.7Hz → PT3 at 7.7Hz would heavily smooth
our 10Hz NN commands (~60-80ms rise time to 90% for step inputs).
If INAV sees MSP RC at ~50Hz: cutoff ≈ 19Hz → moderate smoothing (~30ms rise time).

**This is completely unmodeled in crrcsim.** The sim applies commands directly with only
slew rate limiting. Real aircraft response is additionally smoothed by this filter.

**Action items** (add to crrcsim before next training run):
- [ ] T273d Determine actual RC refresh rate seen by INAV for MSP override channel.
  Check blackbox `rxUpdateRate` column or INAV CLI `status` output. This determines
  whether filter is at 7Hz (devastating) or 57Hz (moderate).
- [ ] T273e Either disable RC smoothing for MSP override (`set rc_filter_auto = OFF`,
  `set rc_filter_lpf_hz = 250`) OR model the PT3 filter in crrcsim after slew limiter.
  Disabling is simpler but may cause servo jitter. Modeling is more accurate for training.
- [ ] T273f If modeling: add PT3 filter in `inputdev_autoc.cpp` after slew limiting,
  matching INAV's `pt3FilterApply` with same cutoff frequency. This trains the NN to
  anticipate the smoothing and command accordingly.

### Flight hardware config findings (2026-03-22 flight had these ACTIVE, now disabled):
- `rc_filter_auto = ON`, `rc_filter_lpf_hz = 50` → PT3 filter was active, added ~30-35ms
  **NOW DISABLED**: `rc_filter_auto = OFF`, `rc_filter_lpf_hz = 250`
- `manual_rc_expo = 35` → non-linear center-stick deadening, unmodeled in sim
  **NOW DISABLED**: `manual_rc_expo = 0`
- These two changes should reduce attitude response from ~81ms to ~45-50ms

### Pipeline timing (bench-measured T273a):
- MSP fetch: 34.9ms avg (serial at 115200, multiple requests)
- NN eval: 4.5ms avg
- MSP send: 10.1ms avg
- Total: 49.4ms avg
- Interval: 101.7ms avg (rock solid 100ms cycle)
- Flight attitude response (with old filter+expo): 81ms (55ms beyond RC delivery)
- Expected with filter/expo disabled: ~45-50ms (close to CRRCSim's 40ms)

### Sim calibration approach:
Two independent calibrations:
1. **hb1.xml aerodynamics** — measure from blackbox `servo → attitude` response.
   Flying wing: servo[0,1] are elevons (mixed pitch+roll). De-mix first:
   `pitch_deflection = (servo[0] + servo[1]) / 2`
   `roll_deflection = (servo[0] - servo[1]) / 2`
   Then measure rate per unit deflection at various airspeeds. Check mixer
   coefficients match between INAV config and crrcsim hb1.xml.
2. **COMPUTE_LATENCY** — measured at 49ms (bench). With INAV filter/expo disabled,
   this is the dominant delay. CRRCSim's 40ms is close. Could increase to 50ms or
   keep as slight conservative margin.

### Notes from flight analysis
- Streamer: 25ft crepe adds significant parasitic drag and pitch damping. The craft
  can lose the streamer mid-flight, changing dynamics. Consider this as an aircraft
  variation parameter in training.
- INAV MANUAL mode: confirmed strictly linear, no rate/expo. RC→servo is direct (through
  mixer for flying wing elevon blending, but no PID or rate scaling).
- Sim latency 40ms is slightly conservative vs real ~34ms (empirical). Safe to keep as-is
  or reduce to match. The key unknown is the MSP fetch duration — T273a will measure it.

- [ ] T274 [US6] Sample rate increase feasibility: evaluate 10Hz→20Hz xiao MSP poll rate.
  Deferred — current 10Hz NN with 20Hz sends is adequate.

**Checkpoint**: Timing characterized. Jitter injection implemented if needed.

---

## Phase 8b: Post-Flight Analysis — 2026-03-22 (Priority: P1)

**Goal**: Triangulate AHRS accuracy, flight dynamics, and sim fidelity from the
second flight data. No ground truth video — infer from multiple data sources.
Key assumption: craft is roughly level heading south when autoc enables.

### Q1: AHRS Accuracy — Is reported attitude where the craft actually was?

- [ ] T290 [P] GPS ground track vs AHRS heading: during straight-ish flight (between
  test spans), compare `atan2(navVel[1], navVel[0])` to quaternion yaw. Consistent
  offset = magnetometer bias. Varying offset = AHRS drift or sideslip. Plot over time.

- [ ] T291 [P] Bank angle vs turn rate: during turns, GPS trajectory curvature implies
  a specific bank angle (`phi = atan(v² / (R*g))`). Compare to AHRS-reported roll.
  If AHRS says 30° bank but GPS curvature implies 45°, AHRS is underreporting.

- [ ] T292 [P] Gravity vector check: in between test spans (pilot flying level), the
  accelerometer should read ~[0, 0, -g] in body frame. Compute `q * [0,0,-g] * q_conj`
  and compare to `accSmooth[0-2]`. Difference = AHRS tilt error.

- [ ] T293 [P] Altitude consistency: compare barometric altitude (BaroAlt) to GPS
  altitude (navPos[2]) to NN-reported Z. Do they agree? If baro drifts differently
  from GPS, position fusion may be compromised during test spans.

- [ ] T294 Pre-test attitude: for each span, capture attitude at NN enable and compare
  to expected (~level, ~180° heading). If the pilot has the craft roughly south and
  level, attitude should confirm. Large deviation = AHRS was already wrong.

### Q2: Flight Dynamics — Do control inputs produce expected responses?

- [ ] T295 [P] Per-axis rate gain: for each test span, compute attitude rate (deg/s)
  per unit RC deviation from 1500. Compare across spans. Is roll gain consistent?
  Is pitch gain consistent? Or does it vary wildly (suggesting the craft was in
  different flight regimes / the NN was saturating)?

- [ ] T296 [P] Z-axis investigation: span 1 (racetrack) and span 3 (spiral climb) should
  both show altitude changes from NN throttle commands. Check if pitch-up + throttle
  produces climb or dive. If two spans show opposite Z results, determine if:
  - NN commanded different directions (check rc values)
  - Same commands produced opposite Z response (dynamics or AHRS issue)
  - The craft was inverted (quat analysis)

- [ ] T297 [P] IMU tumble detection: scan quaternion history for signs of gimbal lock
  or heading reversal. Look for: qw crossing zero, heading jumping >90° between ticks,
  roll exceeding ±90° without corresponding GPS trajectory change.

- [ ] T298 Recovery behavior: when the craft goes out of tolerance (>30° from expected),
  does the NN attempt to recover? Or does it go degenerate (pegged outputs)? This
  informs whether we need a recovery training phase or just better dynamics matching.

### Q3: Sim Fidelity — How close is hb1.xml to the real craft?

- [ ] T299 [P] Compare attitude rates: from `eval-data.dat`, compute typical pitch rate
  and roll rate per unit NN command in sim. Compare to flight measurements from T295.
  Express as ratio: `real_rate / sim_rate`. If >2× or <0.5×, the sim model is far off.

- [ ] T300 [P] Compare speed/altitude envelope: sim typical speed range, climb rate at
  full throttle, descent rate at idle. Compare to flight GPS speed and altitude rates.
  Is the real craft faster/slower? Does it climb/descend at different rates?

- [ ] T301 [P] Input scaling comparison: verify that sim NN output [-1,1] → crrcsim
  elevator/aileron/throttle produces the same effective deflection as NN output [-1,1]
  → INAV RC override → MANUAL mode → servo. There may be an INAV rate/expo scaling
  that makes real commands weaker or stronger than sim commands.

- [ ] T302 hb1.xml parameter review: list the key parameters in
  `crrcsim/models/hb1.xml` (mass, wing area, CL/CD, control effectiveness, thrust)
  and compare to known/estimated values for the real aircraft. Flag any obviously
  wrong parameters.

- [ ] T303 Diagnostic script: create `specs/018-flight-analysis/flight_dynamics.py`
  that reads both `eval-data.dat` (sim) and flight INAV CSV + xiao log, produces
  side-by-side comparison charts for each axis: command → rate, speed envelope,
  altitude rate vs throttle.

**Checkpoint**: Error circles narrowed — AHRS accuracy quantified, dynamics mismatch
measured as ratios, sim calibration gaps identified with specific parameters to adjust.

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

## Current Priority Order

### Completed
- Phases 1-3: Setup, foundation, pipeline proof — all verified
- Phase 4 (partial): Firmware refactor, rabbit logging, projection working
- Phase 5: Output mapping — signs correct, documented
- Two flights analyzed, conventions confirmed correct

### Before next training run
1. [x] **T223** — Direct rabbit in renderer (done — magenta spheres)
2. [x] **T273a** — Pipeline timing instrumented (fetch=35ms eval=4.5ms send=10ms total=49ms)
3. [x] **T301** — INAV MANUAL mode: confirmed linear (rate=100, expo NOW 0, filter NOW off)
4. [ ] **T273g** — Bench servo response: run autoc on flight hardware with blackbox at 1/8 rate.
   Measure NN command → servo[0,1] step response. No filter/expo. Characterize servo
   actuator delay and slew rate. Compare to crrcsim's slew model.
5. [ ] **T295/T299** — Rate gain from flight data: de-mix elevons, measure servo→attitude
   rate at various airspeeds. Compare to crrcsim eval-data.dat. Express as ratio.
6. [ ] **T302** — hb1.xml parameter audit: git log shows earlier tuning. Review key params
   (mass, CL/CD, control effectiveness, thrust) vs known aircraft specs + streamer drag.
7. [ ] **T273b** — Update COMPUTE_LATENCY based on T273a (49ms measured, 40ms current).
   With filter/expo disabled, 40ms is close. May keep as-is or bump to 50ms.
8. [ ] **T244** — Update hb1.xml with measured ratios from T295/T299.
9. [ ] **T273h** — Research: custom MSP command to reduce fetch latency.
   Currently 3 sequential requests (MSP_STATUS + MSP2_INAV_LOCAL_STATE + MSP_RC)
   at 115200 baud = 35ms avg. Each request has serial round-trip overhead (request
   frame + wait + response frame + parse). A single custom MSP2 command in INAV
   (branch autoc) returning all needed fields in one response could reduce to ~12-15ms
   (single round-trip, ~100 bytes payload at 115200 = ~9ms wire time + overhead).
   Estimated pipeline reduction: 50ms → 27ms total. Would need INAV firmware change
   (add MSP2_AUTOC_STATE handler in fc_msp.c) + xiao parser update.
   Also consider: do we need MSP_STATUS every tick? If not, fetch it every Nth tick
   and save one round-trip immediately (~7ms saved, no INAV changes needed).

### Before next flight
7. **T240-T242** — Formal pitch/roll/throttle characterization scripts
8. **T298** — Recovery behavior: does NN attempt recovery or go degenerate past 30m?
9. **T245** — Validation eval suite with updated model
10. **T246-T247** — Expand training: ±50m entry sigma, recovery training phase

### After next training→flight cycle
- [ ] **T273h** — Custom MSP command (reduce fetch 35ms→12ms)
- [ ] **T273c** — Override flight mode channel from xiao (force MANUAL)
- [ ] **T273i** — Cherry-pick INAV servo logging fix (commits 957f23d5d + 376346d8f).
  Known bug in our INAV 8.0.0 autoc branch: blackbox logs servo[0] (unused) and
  servo[1] (left elevon) but misses servo[2] (right elevon). `getServoCount()=2`
  but loop indexes from 0 not from `minServoIndex=1`. Fixed upstream with per-servo
  conditions (AT_LEAST_SERVOS_N). Need both elevons for proper de-mixing.

### Deferred to backlog
- T224b/c — Projection singularity fix (cosmetic, T223 eliminates need)
- T225 — Renderer HUD overlay (→ 017 visual tracking feature)
- T274 — Sample rate increase feasibility (10Hz adequate)
- T260-T267 — AHRS formal charts (plausible per analysis, formal scripts later)
- T290-T294 — AHRS deep dive with gravity/centripetal (after next flight with video)

---

## Notes

### CRITICAL: Flight vs Sim Dynamics Mismatch (measured 2026-03-22)

Command → attitude rate (°/s per rcCommand unit):

| Speed | Roll flight/sim | Pitch flight/sim |
|-------|----------------|-----------------|
| Slow 12m/s | 0.42/0.21 = **2.0×** | 0.24/0.03 = **7.4×** |
| Cruise 16m/s | 0.54/0.24 = **2.3×** | 0.25/0.04 = **6.0×** |
| Fast 20m/s | 0.40/0.24 = **1.7×** | 0.26/0.05 = **4.9×** |

Real craft 2× roll, 5-7× pitch more responsive than sim. With expo now removed,
true ratios even higher. Flight data had PT3 filter ON — with filter OFF response
will be faster. NN trained on sim's sluggish pitch overdrives real craft.

hb1.xml current key params (last updated 2025-11-21, 7 commits total):
- Cm_de = -0.32 (pitch moment per elevator) — needs ~5-7× increase
- Cl_da = 0.14 (roll moment per aileron) — needs ~2× increase
- I_yy = 0.0013 (pitch inertia) — verify if realistic for 505g 30in wing
- Prior tuning contaminated by unmodeled filter/expo

CRRCSim processing chain: 40ms COMPUTE_LATENCY applied before FDM as a block
delay. Reality: 50ms transport then 22ms servo ramp concurrent with aero response.
Current model is slightly pessimistic but close. Refine by moving delay to transport
stage and adding servo ramp model if needed.

---

- Task IDs T200-T303 span this feature
- Pipeline conventions verified correct across two flights
- Key gap: sim dynamics gain ~0.7× real aircraft (NN overdrives, then degenerates >30m)
- Path forward: calibrate hb1.xml → adjust sim latency → expand training envelope → retrain → fly

# Tasks: Pre-Flight Pipeline

**Input**: Design documents from `/specs/020-pre-flight-pipeline/`
**Prerequisites**: plan.md, spec.md

**Organization**: Tasks follow the phased execution order from plan.md (A→B→C).
Phase A must complete before B; B before C. Renderer fix (CO7) is independent.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[CO]**: Which critical outcome this task belongs to

---

## Phase A: Servo Fix + Baseline Bench

**Goal**: Fix INAV servo logging, investigate board alignment, establish bench baseline.

**Independent Test**: Blackbox CSV from bench run shows servo[1] and servo[2] columns with non-zero values for both elevons.

- [x] T500 [P] [CO3] Fix servo logging index in `~/inav/src/main/blackbox/blackbox.c`: change servo loop to start from `minServoIndex` instead of 0. Flying wing logs servo[1]+servo[2] (both elevons) instead of servo[0] (unused)+servo[1].

- [ ] T501 — **Moved to 021** (AHRS cross-check needed first). Board alignment investigation: analyze existing flight data (`eval-results/bench-20260322/`) for pitch bias at 50% cruise. Compare reported attitude vs expected level. Determine how much is board alignment (170° vs 180°) vs natural AoA vs IMU drift. Document findings.

- [x] T502 [P] [CO3] Set blackbox sample rate to 1/8 in INAV CLI config for higher resolution logging.

- [x] T503 [CO3] Build INAV for bench target: `cd ~/inav && mkdir build && cd build && cmake .. && make MAMBAF722_2022A`. NOTE: disconnect GPS module before flashing.

- [x] T504 [CO3] Build+deploy xiao (wire-compatible, no code changes yet): `cd ~/autoc/xiao && pio run -e xiaoblesense_arduinocore_mbed`. Load recent training weights.

- [x] T505 [CO3] Bench run: capture blackbox log, decode with blackbox-tools, verify both elevon servo columns appear with non-zero data in CSV.

**Checkpoint**: Both elevons logged. Board alignment findings documented. Bench baseline established.

---

## Phase B: MSP2_AUTOC_STATE + Flight Mode Override

**Goal**: Consolidate 3 MSP calls into 1, add flight mode override, measure latency.

**Independent Test**: Xiao parses MSP2_AUTOC_STATE response correctly (position, velocity, quat, rc match INAV state). Pipeline latency measured and logged. Flight mode override activates MANUAL without TX switch.

### INAV side

- [ ] T506 [P] [CO1] Rename `MSP2_INAV_LOCAL_STATE` to `MSP2_AUTOC_STATE` in `~/inav/src/main/fc/fc_msp.c` (command ID `0x210E` stays). Extend handler to pack full 38-byte payload: pos[3] int32 cm NEU, vel[3] int16 cm/s NEU, quat[4] int16 /10000 body→earth, rc[4] uint16 PWM, armingFlags uint32.

- [ ] T507 [CO1] Build INAV with MSP2 changes: clean build for MAMBAF722_2022A. Disconnect GPS, flash.

### Xiao side

- [ ] T508 [P] [CO1] Rename `MSP2_INAV_LOCAL_STATE` to `MSP2_AUTOC_STATE` in `~/autoc/xiao/include/MSP.h`. Update `msp_local_state_t` struct to match extended 38-byte payload (add rc[4], armingFlags).

- [ ] T509 [CO1] Replace all 3 MSP calls (`MSP_STATUS`, `MSP2_INAV_LOCAL_STATE`, `MSP_RC`) with single `MSP2_AUTOC_STATE` request in `~/autoc/xiao/src/msplink.cpp`. Parse extended payload into existing state fields (pos, vel, quat from local_state; rc from rc; armingFlags from status). Remove all three old fetch calls — the consolidated command carries everything.

- [ ] T510 [CO4] Add flight mode channel override in `~/autoc/xiao/src/msplink.cpp`: include RC channel 6 (bit 5) in MSP override channels, set to 1000 (→ MANUAL mode). Separately, update INAV CLI config during bench setup: `set msp_override_channels = 47` (adds bit 5 to existing mask 15).

- [ ] T511 [CO1] Build xiao with MSP2 + flight mode changes: `pio run -e xiaoblesense_arduinocore_mbed`.

### Bench verification

- [ ] T512 [CO1] Bench test MSP2: verify xiao receives and parses correct position, velocity, quaternion, rc values from MSP2_AUTOC_STATE response. Cross-check against INAV configurator display.

- [ ] T513 [CO2] Measure pipeline latency: xiao logs fetch→eval→send timing per tick. Record median, p95, p99. Expected: ~8-12ms (down from ~35ms with 3 separate calls).

- [ ] T514 [CO4] Verify flight mode override: confirm INAV enters MANUAL mode when xiao sends ch6=1000 via MSP override, without pilot TX switch input.

**Checkpoint**: Single MSP2 call working. Latency measured. Flight mode override confirmed.

---

## Phase C: Latency Calibration + Training

**Goal**: Calibrate sim latency to match bench measurement, fix renderer, run final training.

**Independent Test**: Response curves from BIG training visually resemble flight blackbox curves across operating envelope. Renderer playback shows path reveal synced with rabbit position.

- [ ] T515 [CO2] Update `COMPUTE_LATENCY_MSEC_DEFAULT` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` to match bench-measured pipeline latency from T513. Rebuild CRRCSim.

- [ ] T516 [P] [CO7] Fix renderer path reveal in `tools/renderer.cc`: use rabbit odometer from AircraftState to determine path reveal progress instead of proportional step count. Handles variable rabbit speed correctly.

- [ ] T517 [CO6] BIG training run: production `autoc.ini` (400 gens, pop=3000) with calibrated hb1_streamer.xml + correct COMPUTE_LATENCY + cone entry variations + variable rabbit speed.

- [ ] T518 [CO6] Validate training: extract response curves from data.dat, compare to flight blackbox. Throttle cruise should be ~55-65%, roll rate within 2× of flight. Deploy winning weights to xiao.

**Checkpoint**: Flight-ready controller. Calibrated sim. Working renderer playback.

---

## Phase D: Polish & Pre-Flight

**Purpose**: Final cleanup before flight test.

- [ ] T519 [P] Document final COMPUTE_LATENCY value and bench measurement in spec comments.
- [ ] T520 [P] [CO5] If board alignment confirmed wrong on bench (T501), update INAV config: `set align_board_roll = 1800`.
- [ ] T521 Rebuild INAV for flight target: `rm -rf build && mkdir build && cd build && cmake .. && make MATEKF722MINI`.
- [ ] T522 Final bench verification with flight-target firmware before deploying to aircraft.

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase A (servo fix, baseline bench)
  └─ Phase B (MSP2 + flight mode override)
       └─ Phase C (latency calibration + BIG training)
            └─ Phase D (polish + flight target rebuild)
                 └─ Flight test
```

### Parallel Opportunities

- **Phase A**: T500, T501, T502 can run in parallel (different files/repos)
- **Phase B**: T506 (INAV) and T508 (xiao) can run in parallel (different repos)
- **Phase C**: T516 (renderer) is independent of T515/T517 (sim+training)
- **Phase D**: T519, T520 can run in parallel

### Within Phase B (critical path)

```
T506 (INAV MSP2) ──→ T507 (INAV build)  ──→ T512 (bench verify)
T508 (xiao MSP2) ──→ T509 (xiao consumer) → T510 (mode override) → T511 (xiao build) → T512
```

---

## Implementation Strategy

### MVP: Phase A only
1. Servo fix + bench baseline = immediate value for post-flight analysis
2. Can fly with existing pipeline if MSP2 work takes longer

### Incremental
1. Phase A → bench baseline (servo data, alignment findings)
2. Phase B → latency reduction + mode override (biggest pipeline improvement)
3. Phase C → sim calibration + final training (flight-ready controller)
4. Phase D → flight target build + deploy

# Implementation Plan: 021 — AHRS Cross-Check + NN Input Redesign

**Branch**: `021-xiao-ahrs-crosscheck` | **Date**: 2026-03-28 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/021-xiao-ahrs-crosscheck/spec.md`

## Summary

Three-phase effort to (1) instrument the aircraft for AHRS cross-checking and
characterization data collection, (2) redesign NN inputs based on flight data,
switching from absolute quaternion + invalid alpha/beta to gravity vector +
rate gyros with ACRO mode inner loop, and (3) retrain and validate in flight.

The critical insight: the current NN has no angular rate feedback and uses
sensor inputs (alpha/beta) that are physically meaningless on real hardware.
ACRO mode delegates inner-loop stabilization to INAV's proven 1kHz PID,
letting the NN focus on outer-loop path tracking.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), C (INAV autoc branch), C++ (xiao/PlatformIO), Python 3.11 (analysis)
**Primary Dependencies**: Eigen (math), cereal (serialization), CRRCSim LaRCSim FDM, INAV MSP protocol, LSM6DS3 IMU
**Storage**: File-based — blackbox CSV, xiao flash logs, S3 for training artifacts
**Testing**: GoogleTest (autoc), bench flights (xiao/INAV), flight test
**Target Platform**: Linux aarch64 (training), Seeed XIAO BLE Sense nRF52840 (embedded), INAV STM32F722 (FC)
**Project Type**: Embedded control system — neuroevolution trainer + flight controller
**Performance Goals**: 30ms MSP pipeline tick, 100Hz local IMU read, NN eval <5ms
**Constraints**: nRF52840 64MHz Cortex-M4F, 256KB RAM; MSP2_AUTOC_STATE payload size; SPI flash blackbox write budget
**Scale/Scope**: 3 repos (autoc, crrcsim, inav), ~26 NN inputs (down from 29), 3 outputs

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Testing-First | PASS | Phase 1 is data collection (no code to test). Phase 2 changes to evaluator.cc covered by existing GoogleTest. Xiao IMU code is exploratory (exemption applies). |
| II. Build Stability | PASS | Each phase leaves all three repos buildable. INAV config changes don't affect build. |
| III. No Compatibility Shims | PASS | Clean-cut: old NN inputs removed, new ones added. No backwards compat needed — retraining is required anyway. |
| IV. Unified Build | PASS | No new build dependencies for autoc/crrcsim. Xiao adds LSM6DS3 lib to platformio.ini. |

## Project Structure

### Source Code (changes per repo)

```text
# INAV (~/inav) — Phase 2 only
src/main/fc/fc_msp.c              # Extend MSP2_AUTOC_STATE: +accel[3], +gyro[3]

# CRRCSim (~/autoc/crrcsim)
src/mod_inputdev/inputdev_autoc/
  inputdev_autoc.cpp               # Phase 2: rate PID for ACRO mode
  inputdev_autoc.h                 # Phase 2: ACRO rate limits config

# autoc (~/autoc)
src/nn/evaluator.cc                # Phase 2: new input vector (gravity, gyro rates)
include/autoc/eval/aircraft_state.h # Phase 2: add accel, gyro fields
include/autoc/eval/sensor_math.h   # Phase 2: gravity vector computation

# xiao (~/autoc/xiao)
src/ahrs.cpp                       # Phase 1: LSM6DS3 + Madgwick (new file)
include/ahrs.h                     # Phase 1: AHRS interface (new file)
src/msplink.cpp                    # Phase 1: log cross-check. Phase 2: parse accel/gyro
include/MSP.h                      # Phase 2: extend msp_autoc_state_t
include/state.h                    # Phase 2: add accel/gyro to AircraftState

# Analysis scripts
specs/021-xiao-ahrs-crosscheck/
  analyze_crosscheck.py            # Phase 1: AHRS delta analysis
  characterize_response.py         # Phase 1: pilot maneuver response curves
```

## Execution Phases

### Phase 1: Instrumentation + Characterization Flight

**Goal**: Collect data to inform Phase 2 design. No NN changes. Fly MANUAL only.

**No INAV code changes** — config only (blackbox fields, already done).

```
Phase 1A: Xiao AHRS cross-check code
  ├── LSM6DS3 driver + Madgwick filter (new: ahrs.cpp/h)
  ├── Log q_local alongside q_inav each MSP tick
  ├── Log raw accel as gravity vector cross-check
  └── Build xiao

Phase 1B: Characterization flight (MANUAL mode, no autoc)
  ├── Pilot maneuvers: rolls, pitches, throttle steps at various speeds
  ├── Blackbox: GYRO_RAW + ACC + QUAT + SERVOS + RC_COMMAND + etc @ 1/32
  └── Collect xiao log with AHRS cross-check data

Phase 1C: Analysis
  ├── AHRS divergence: q_inav vs q_local delta over time
  ├── Gravity vector: accel vs quaternion-derived, identify bias
  ├── Response curves: gyroADC rate vs rcCommand per axis, binned by speed
  ├── Gyro filter analysis: compare gyroRaw vs gyroADC, assess filter params
  └── Document findings → inform Phase 2 parameter choices
```

### Phase 2: NN Input Redesign + ACRO Mode

**Goal**: New sensor inputs, ACRO inner loop in sim, retrain.

Order matters: INAV first (provides the data), then CRRCSim (consumes it in sim),
then autoc NN topology (uses the new inputs).

```
Phase 2A: INAV MSP extension
  ├── Add accel[3] (filtered, int16 milli-g) to MSP2_AUTOC_STATE
  ├── Add gyro[3] (filtered, int16 deg/s scaled) to MSP2_AUTOC_STATE
  ├── Build INAV for both MAMBA (bench) and MATEK (flight)
  └── Bench verify: xiao receives and logs accel/gyro correctly

Phase 2B: CRRCSim ACRO rate PID
  ├── Extract body rates (p,q,r) from LaRCSim FDM (already computed internally)
  ├── Extract body-frame accel from FDM (gravity + centripetal)
  ├── Wire accel[3] + gyro[3] through autoc RPC protocol to evaluator
  ├── Implement rate PID in inputdev_autoc:
  │     NN output [-1,1] → desired rate (linear, expo=0)
  │     PID: error = desired_rate - actual_rate → surface deflection
  │     Roll max 560°/s, pitch 400°/s, yaw 240°/s (match INAV config)
  ├── PID gains: start from INAV's fw_p/i/d/ff values, tune to match
  │   Phase 1 measured response curves
  └── Rebuild crrcsim

Phase 2C: autoc NN topology + inputs
  ├── New input vector in evaluator.cc:
  │     Path sensors (19 inputs — dPhi/dTheta/dist history+forecast, closing rate)
  │     Gravity vector body frame (3 inputs — from accel, normalized)
  │     Rate gyros (3 inputs — p,q,r filtered, scaled to [-1,1] by max_rate)
  │     Airspeed (1 input)
  │     Total: 26 inputs
  ├── Remove: quaternion (4), alpha/beta (2), previous commands (3)
  ├── Hidden layer topology considerations:
  │     Current: 29→16→8→3 (643 weights)
  │     Option A: 26→16→8→3 (571 weights) — same hidden, fewer input connections
  │     Option B: 26→20→10→3 (753 weights) — wider hidden for richer representation
  │     Option C: 26→16→3 (323 weights) — single hidden, simpler
  │     Decision: start with Option A (minimal change), evaluate convergence
  ├── Input scaling: all inputs normalized to ~[-1,1] range
  │     gravity: already [-1,1] (g-normalized)
  │     gyro: divide by max_rate (560°/s roll, 400°/s pitch, 240°/s yaw)
  │     path sensors: already in radians/meters (existing scaling)
  ├── Update aircraft_state.h with accel/gyro fields
  ├── Update xiao msplink.cpp to parse extended MSP2_AUTOC_STATE
  ├── Update xiao nn_gather_inputs equivalent
  └── GoogleTest: verify input vector construction with known state

Phase 2D: Safety overrides
  ├── Distance-from-origin sphere check on xiao
  │     On autoc enable: capture origin position
  │     Each tick: if distance > threshold (e.g. 100m), disable autoc
  ├── Log safety events to flash
  └── Threshold configurable (compile-time initially)
```

### Phase 3: Training + Flight Validation

```
Phase 3A: Training
  ├── BIG training run with new inputs + ACRO rate PID in sim
  ├── Eval suite: aeroStandard + random + stress tiers
  ├── Compare convergence and smoothness to BIG3 baseline
  └── Response curve analysis: sim vs Phase 1 flight data

Phase 3B: Deploy + Flight
  ├── Extract weights, nn2cpp, build xiao
  ├── Flash INAV (MATEK) with extended MSP + blackbox config
  ├── ACRO mode + safety overrides active
  ├── Ground check: arm, verify ACRO mode, servo response, safety cutoff
  └── Flight test: autoc spans, expect sustained tracking >10s
```

## Key Design Decisions

### Why ACRO not ANGLE mode
- ANGLE mode caps bank angle (~45° default), limiting maneuver envelope
- ACRO preserves full rate authority while INAV PID handles stabilization
- The NN learns to command rates, not attitudes — more robust to attitude errors
- ACRO's rate PID runs at 1kHz, providing much faster inner-loop than NN at 30Hz

### Why gravity vector not quaternion
- Quaternion encodes heading (irrelevant — path sensors already capture heading error)
- Gravity vector gives bank/pitch relative to earth directly (3 inputs vs 4)
- Available from accelerometer without AHRS fusion — fewer failure modes
- In turns, centripetal acceleration shifts gravity vector — this is useful information
  (tells the NN it's in a turn, not just banked)

### Why remove alpha/beta
- Computed from GPS groundspeed (5-10Hz, stale) + AHRS quaternion
- No wind correction — not true angle of attack
- Measured ±70° in flight — physically impossible, pure noise
- Without pitot tube, no valid airspeed-based AoA measurement possible

### Hidden layer topology
- Start with 26→16→8→3 (minimal change from current 29→16→8→3)
- The reduced input count (26 vs 29) slightly reduces first-layer weights
- If convergence is poor, widen to 26→20→10→3
- Deeper networks (3+ hidden) not warranted — control problem is reactive, not sequential

## Complexity Tracking

No constitution violations requiring justification.

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|-----------|
| ACRO rate PID in CRRCSim doesn't match INAV | NN transfers poorly | Tune PID from Phase 1 measured response data |
| Gravity vector noisy during maneuvers (centripetal) | NN confused by non-gravity acceleration | Train with realistic accel (FDM includes centripetal). NN learns to handle it. |
| 26 inputs insufficient for path tracking | Convergence regression | Fall back to 29→16→8→3 with rate gyros added (32 inputs) |
| Blackbox still corrupts at 1/32 with GYRO_RAW added | Lose flight data | Bench test first; drop GYRO_RAW if needed |

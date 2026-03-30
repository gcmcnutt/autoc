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

### Sim-First Approach

All sim-side changes are implemented and validated BEFORE touching flight hardware.
The CRRCSim rate PID + NN input redesign + training validation prove the architecture
works in simulation. Only then do we implement the INAV/xiao plumbing to replicate
the same data flow on real hardware.

The characterization flight runs in parallel whenever props are available — it provides
real-world response data that feeds back into CRRCSim PID tuning but does not block
the sim-side work.

### Phase 1: Bench Verification

**Goal**: Confirm gyro/accel polarity and ACRO config before any code changes.
Quick sanity check on bench hardware — tilt frame, read sensors.

### Phase 2: CRRCSim ACRO Rate PID (MVP)

**Goal**: Implement rate PID in CRRCSim. This is the sim-side inner loop that
converts NN rate commands to surface deflections, matching INAV's ACRO behavior.
Train with existing 29 inputs to validate ACRO mode helps convergence.

### Phase 3: NN Input Redesign

**Goal**: Change NN inputs from 29 to 27. Remove alpha/beta and previous commands,
add rate gyros from FDM. Train and compare convergence to Phase 2 baseline.

### Phase 4: Training + Sim Validation

**Goal**: Production training run. Validate at scale. Eval suite. Response analysis.
If characterization flight data is available, use it to tune CRRCSim PID gains.

### Phase 5-6: INAV MSP + Xiao (hardware plumbing)

**Goal**: Replicate on hardware what sim already provides. Extend MSP2_AUTOC_STATE
with gyro[3]. Xiao parses it, feeds NN, overrides to ACRO mode.
Only proceed after sim validation (Phase 4) confirms architecture works.

### Phase 7: Safety Overrides

**Goal**: Distance-from-origin sphere check. Required before flight test.

### Phase 8: Characterization Flight (parallel track)

**Goal**: Fly choreography from `flight-choreography.md` whenever props arrive.
MANUAL mode, no autoc. Provides response data for CRRCSim PID tuning.
Feeds back into Phase 4 analysis.

### Phase 9: Flight Test

**Goal**: Fly with trained NN in ACRO mode. Expect sustained tracking >10s.

See `tasks.md` for detailed task breakdown and dependency graph.

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

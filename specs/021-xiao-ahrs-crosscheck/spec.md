# 021 — Xiao Onboard AHRS Cross-Check

**Status**: Draft
**Priority**: P0 — blocker for flight trust
**Created**: 2026-03-28

## Problem

Mar 27 flight showed the NN commanding reasonable inputs (roll ±0.6, pitch ±0.7)
but the aircraft entering uncontrolled 360° rolls and tumbles. INAV's AHRS
quaternion was self-consistent (norm=1.0, no discontinuities), but we cannot
verify whether it accurately reflects physical attitude during aggressive
maneuvering. The aircraft may have been in a different attitude than reported.

Without an independent attitude reference, we cannot distinguish:
- NN produces correct commands but sim-vs-real gain mismatch causes over-rotation
- INAV AHRS diverges under aggressive maneuvering, NN sees wrong attitude
- Both (AHRS drifts AND gain mismatch compound)

## Solution

Add a local AHRS on the Xiao BLE Sense using the onboard LSM6DS3TR-C
IMU (3-axis accel + 3-axis gyro). Run a Madgwick filter at 50-100Hz,
log the resulting quaternion alongside INAV's quaternion for post-flight
comparison.

This is a diagnostic/cross-check tool, not a replacement for INAV's AHRS.
The local AHRS is accel+gyro only (no magnetometer), so it will drift in
yaw, but roll and pitch should be reliable over the ~10s autoc spans.

## Design

### IMU Read (100Hz, timer-driven)
- I2C read of LSM6DS3 accel[3] + gyro[3] at 100Hz via hardware timer
- Decoupled from MSP tick — runs independently
- ~12 bytes per read at 400kHz I2C = negligible bus time

### Madgwick Filter
- Standard Madgwick AHRS (accel+gyro, no mag)
- ~50μs per update on Cortex-M4F with hardware FPU
- Output: quaternion (w, x, y, z) in same convention as INAV
- Beta parameter tuned for 100Hz update rate

### Zeroing
- On arm (or autoc enable): capture current INAV quaternion as reference
- Zero the local AHRS to match INAV's orientation at that moment
- Both sources start aligned; drift reveals disagreement

### Logging
- Every MSP tick (~20Hz), log both quaternions to flash:
  - `q_inav=[w,x,y,z]` (from MSP2_AUTOC_STATE)
  - `q_local=[w,x,y,z]` (from local Madgwick)
  - `q_delta=angle_between(q_inav, q_local)` (scalar, degrees)
- Post-flight: plot delta over time per autoc span
- >5° divergence over 10s span = AHRS trust issue

### Compute Budget
- IMU read + Madgwick @ 100Hz: ~1-2% CPU
- No impact on MSP pipeline or NN eval timing
- FPU build flags already set (-mfloat-abi=hard, -mfpu=fpv4-sp-d16)

## Dependencies
- LSM6DS3 Arduino library (add to platformio.ini lib_deps)
- No INAV changes required
- No changes to MSP protocol or NN eval

## Related Backlog Items
- [project_xiao_imu_crosscheck.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_xiao_imu_crosscheck.md) — original backlog item
- [project_board_alignment.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_board_alignment.md) — 170° vs 180° roll, related AHRS issue
- BACKLOG.md: "GPS Dropout Handling During NN Control" — related sensor trust

## Success Criteria
1. Local AHRS quaternion logged alongside INAV quaternion during flight
2. Post-flight delta analysis script shows divergence timeline
3. Roll/pitch agreement <2° during straight-and-level, <5° during moderate turns
4. Clear answer: does INAV AHRS track reality during aggressive maneuvers?

## Out of Scope
- Replacing INAV's AHRS with local AHRS for NN control
- Magnetometer fusion (yaw drift is acceptable for ~10s spans)
- Real-time AHRS switching or voting logic

# 021 — AHRS Cross-Check + NN Input Redesign

**Status**: Draft
**Priority**: P0 — blocker for flight trust and next training cycle
**Created**: 2026-03-28

## Problem

Mar 27 flight showed the NN commanding reasonable inputs (roll ±0.6, pitch ±0.7)
but the aircraft entering uncontrolled 360° rolls and tumbles. INAV's AHRS
quaternion was self-consistent (norm=1.0, no discontinuities), but we cannot
verify whether it accurately reflects physical attitude during aggressive
maneuvering.

Additionally, analysis of the NN's current 29 inputs revealed fundamental issues:
- **alpha/beta** (inputs 24-25) are computed from GPS groundspeed + AHRS quaternion.
  In flight these swing ±70° — physically meaningless. GPS is 5-10Hz, stale vs
  30Hz NN tick, and measures groundspeed not airspeed. No wind correction.
- **No rate gyros** — the NN sees quaternion snapshots 30ms apart with no
  angular rate information. It cannot damp oscillations or anticipate overshoot.
- **Absolute quaternion** — encodes heading, which is irrelevant to control.
  Path-relative sensors already capture heading error.
- **Previous commands as inputs** (26-28) — a hack for learned integration.

The NN is trying to learn both inner-loop stabilization AND outer-loop path
tracking from high-level inputs, with no rate feedback. This works in sim
where dynamics match perfectly but fails on real hardware.

## Solution: Three Phases

### Phase 1: Instrumentation + Data Collection Flight

Fly with current NN (expect tumbling) to collect diagnostic data:

**Xiao AHRS cross-check:**
- Read LSM6DS3 accel+gyro at 100Hz via timer
- Run Madgwick filter → local quaternion
- Log both q_inav and q_local each MSP tick
- Also log raw accel as gravity vector cross-check (should always point down)

**INAV ACRO mode for autoc:**
- Change autoc flight mode from MANUAL to ACRO
- ACRO gives gyro-based rate damping on the inner loop (INAV PID at 1kHz)
- NN commands desired rates, not raw servo deflections
- Should immediately reduce the uncontrolled tumbling

**Pilot characterization maneuvers (outside autoc):**
- Variable-rate rolls at different airspeeds (slow/cruise/fast)
- Pitch doublets at different attitudes and speeds
- Throttle steps from cruise
- Provides ground-truth response data for sim tuning

**Blackbox config:**
- Rate 1/32 (verified: 3 failed frames vs 4645)
- Add ACC back in for accelerometer cross-reference
- Keep: NAV_POS, ACC, RC_DATA, RC_COMMAND, MOTORS, SERVOS, QUAT, MAG

### Phase 2: NN Input Redesign

Based on Phase 1 data, redesign the sensor inputs for the next training cycle.

**Research items:**

#### Gravity vector method
- Accelerometer gives gravity direction in body frame directly
- During steady flight (no acceleration): accel ≈ [0, 0, -g] rotated by attitude
- Provides bank angle and pitch without AHRS quaternion dependency
- 3 inputs replace 4 quaternion inputs
- Available from both INAV (MSP) and local LSM6DS3
- Cross-check: both sources should agree on "down"

#### Proposed input set (body-frame, all relative)
```
Path tracking (existing, validated):
  dPhi history + forecast      (6 inputs — heading bearing to rabbit)
  dTheta history + forecast    (6 inputs — elevation bearing to rabbit)
  dist history + forecast      (6 inputs — range to rabbit)
  closing rate                 (1 input)

Attitude/dynamics (new):
  gravity vector, body frame   (3 inputs — from accel, replaces quaternion)
  rate gyros p, q, r           (3 inputs — angular rates for damping)
  airspeed                     (1 input — keep, but need pitot for accuracy)

Removed:
  quaternion (4)               — replaced by gravity vector
  alpha/beta (2)               — invalid without airspeed sensor
  previous commands (3)        — unnecessary with proper rate feedback

Total: ~26 inputs (down from 29)
```

#### CRRCSim / minisim presentation
- Gravity vector: compute from quaternion in FDM, add centripetal correction
- Rate gyros: available from FDM (p, q, r body rates)
- Both trivial to extract — LaRCSim already computes them internally
- Need to wire through the autoc RPC protocol to the NN evaluator

#### ACRO mode in sim
- CRRCSim currently applies NN output directly as surface deflection
- With ACRO, NN output should be interpreted as rate commands
- Need a simple rate PID in the sim's autoc input device, or
- Configure CRRCSim's existing control model to match INAV's ACRO gains

### Phase 3: Training + Flight Validation

- Train with redesigned inputs + ACRO inner loop
- Sim ACRO PID tuned to match INAV's measured rates from Phase 1
- Validate with flight test

## Phase 1 Design Details

### Xiao AHRS (same as original spec)
- IMU read: I2C LSM6DS3 at 100Hz, timer-driven
- Madgwick filter: accel+gyro, no mag, ~50μs per update
- Zero on autoc enable to match INAV orientation
- Log q_local alongside q_inav each tick, plus angle-between delta

### Gravity vector cross-check
- Raw accelerometer from LSM6DS3: accel[3] in body frame
- During non-maneuvering flight: should equal q.inverse() * [0,0,-g]
- Log raw accel alongside computed gravity-from-quaternion
- Disagreement = centripetal effects (expected in turns) or AHRS error

### ACRO mode switch
- Change xiao CH6 override to select ACRO instead of MANUAL
- Check INAV config: `aux` mode assignments for ACRO vs MANUAL
- Verify ACRO PID rates are reasonable for hb1 (check current config)
- On bench: confirm servo response to NN commands through ACRO PID

### Pilot characterization protocol
Document specific maneuvers for the pilot to perform during non-autoc segments:
1. Wings-level, cruise speed → full left roll → hold 2s → level (repeat right)
2. Wings-level, cruise speed → half-stick pitch up → hold → release
3. Level cruise → throttle chop to idle → hold 3s → full throttle
4. 45° bank turn left, steady → roll to 45° right
5. Repeat 1-2 at slow speed and fast speed

## INAV ACRO Configuration Check

Before flight, verify these INAV settings match expected behavior:
```
# Check current values:
get rate_accel_limit_roll_yaw
get rate_accel_limit_pitch
get max_rate_inclination_roll
get max_rate_inclination_pitch
get fw_p_roll
get fw_i_roll
get fw_d_roll
get fw_p_pitch
get fw_i_pitch
get fw_d_pitch
```

## Dependencies
- LSM6DS3 Arduino library (add to platformio.ini lib_deps)
- Phase 1: no INAV code changes (just config)
- Phase 2: changes to autoc evaluator.cc, CRRCSim autoc input device, xiao msplink
- Phase 3: full retraining

## Success Criteria

### Phase 1
1. AHRS cross-check data collected — divergence timeline per span
2. ACRO mode confirmed working with current NN (less tumbling expected)
3. Clean blackbox Z data (no decode corruption)
4. Pilot characterization data for all maneuvers
5. Gravity vector logged from both INAV and local IMU

### Phase 2
6. New NN input set implemented in sim, trains successfully
7. Gravity vector and rate gyros wired through CRRCSim → evaluator
8. ACRO PID in sim matches INAV measured rates within 2x

### Phase 3
9. Flight test with redesigned controller — sustained path tracking >10s
10. No uncontrolled rotation

## Out of Scope
- Pitot tube / airspeed sensor (would fix alpha, but hardware change)
- Magnetometer fusion (yaw drift acceptable for ~10s spans)
- Real-time AHRS switching or voting
- GPS-dependent inputs (unreliable, intermittent)

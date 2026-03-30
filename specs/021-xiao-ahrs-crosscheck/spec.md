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

#### Accelerometer / gravity vector (cross-check only, NOT NN input)
- Accelerometer measures gravity + inertial acceleration — NOT pure gravity
- At aggressive attitudes (>45° bank, pullouts, inverted): centripetal forces
  dominate and the accel vector does not indicate "down"
- Quaternion retained as NN attitude input for all-attitude flight
- Accelerometer value for AHRS cross-check: compare accel direction vs
  quaternion-derived gravity to detect AHRS divergence under load
- **INAV source**: filtered accel available via MSP (extend MSP2_AUTOC_STATE
  with accel[3] in Phase 2, for cross-check logging)
- **Xiao local LSM6DS3**: independent cross-check source (Phase 1 diagnostic)

#### Proposed input set
```
Path tracking (existing, validated):
  dPhi history + forecast      (6 inputs — heading bearing to rabbit)
  dTheta history + forecast    (6 inputs — elevation bearing to rabbit)
  dist history + forecast      (6 inputs — range to rabbit)
  closing rate                 (1 input)

Attitude (keep):
  quaternion (w,x,y,z)         (4 inputs — earth-relative attitude, needed for
                                all-attitude flight: inverted, knife-edge, etc.
                                Gravity vector from accel is NOT a reliable
                                attitude proxy under centripetal loads.)

Dynamics (new):
  rate gyros p, q, r           (3 inputs — INAV filtered gyro via MSP, post-LPF.
                                Provides damping feedback the NN previously lacked.
                                Critical for ACRO mode: NN commands desired rates,
                                needs to see current rates to modulate.)
  airspeed                     (1 input — keep, GPS-derived groundspeed for now)

Removed:
  alpha/beta (2)               — computed from GPS groundspeed + quaternion,
                                measured ±70° in flight — physically meaningless
                                without pitot tube
  previous commands (3)        — unnecessary with ACRO rate feedback. The ACRO
                                PID provides the integration/damping these were
                                hacking around.

Total: 27 inputs (down from 29)
```

Note: gyro filter params (LPF cutoff, dynamic notch) need research.
Current filter config may be tuned for PID stability, not NN input
fidelity. Phase 1 characterization flight should log both GYRO_RAW
and filtered gyroADC to compare and determine if filter adjustment needed.

#### Control architecture — two-loop separation

```
Outer loop (NN, 10Hz on xiao):
  Inputs: path sensors + quaternion + rate gyros + airspeed
  Outputs: desired roll rate, desired pitch rate, throttle
  Job: intercept and tracking — "where do I need to go"

Inner loop (INAV ACRO PID, 1kHz on FC):
  Inputs: gyro rates (raw, 1kHz)
  Setpoints: NN desired rates (held between 10Hz updates)
  Outputs: servo deflections (elevon mixer)
  Job: stabilization and rate control — "hold this rate"
  At center stick (NN output = 0): rates damp to zero, aircraft
  holds current attitude vector. This is the key property — the
  NN doesn't need to actively stabilize, only steer.

Benefits:
  - Sim-to-real gap shrinks: NN only needs rate dynamics to match,
    not full aerodynamic response to servo deflection
  - Coupling handled: INAV PID at 1kHz resolves roll/pitch/yaw
    coupling that 10Hz NN cannot react to
  - Speed regime robustness: INAV PID adapts to dynamic pressure
    changes (same stick = same rate at any speed)
  - Future: NN tick rate can increase to 20-50Hz for tighter
    tracking without changing the architecture
```

#### CRRCSim / minisim presentation
- Gravity vector: compute from quaternion in FDM, add centripetal correction
- Rate gyros: available from FDM (p, q, r body rates)
- Both trivial to extract — LaRCSim already computes them internally
- Need to wire through the autoc RPC protocol to the NN evaluator

#### ACRO mode in sim
- CRRCSim currently applies NN output directly as surface deflection
- With ACRO, NN output is interpreted as desired angular rate via INAV's rate curve
- INAV mapping: NN [-1,1] → INAV rate curve → [-max_rate, +max_rate] deg/s
- INAV expo set to 0 (linear mapping): max_rate = rate_param * 10 + 200
- Current hb1 config: roll 560°/s, pitch 400°/s, yaw 240°/s
- CRRCSim needs matching rate PID in autoc input device:
  - NN output [-1,1] → desired rate via same linear curve
  - PID compares desired rate vs FDM body rate (p,q,r) → surface deflection
  - PID gains tuned to match INAV's measured response from Phase 1

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

## Clarifications

### Session 2026-03-28
- Q: How should the NN-to-ACRO command mapping work? → A: Use INAV's rate curve with expo=0 (linear). NN [-1,1] maps through INAV's standard rate formula. CRRCSim must implement matching rate PID with same curve.
- Q: Which source for NN gravity vector input during flight? → A: REVISED — gravity vector is NOT a reliable attitude proxy under centripetal loads. Quaternion retained as NN input. Accelerometer used for AHRS cross-check only, not NN input.
- Q: Raw or filtered gyro rates for NN input? → A: Filtered (post-LPF, pre-PID). Matches what INAV's own PID acts on. Research task: document current gyro filter params and assess if they need adjustment for NN use.
- Q: Safety overrides in Phase 1 or 2? → A: Phase 2. Implement as distance-from-autoc-origin sphere check. Refine later with altitude floor, energy bounds. Foundation for future upper-level planner.
- Q: MSP2_AUTOC_STATE extension timing? → A: Wait for Phase 2. Phase 1 gets gyro/accel from blackbox, no MSP change needed.
- Note: gyroADC[0-2] (filtered) is always logged in blackbox base fields. gyroRaw[0-2] logged when GYRO_RAW enabled. Both available for Phase 1 characterization.
- Note: PEAKS_R/P/Y (dynamic notch filter frequencies, 9 fields) stay OFF — PID tuning tool, not relevant for NN or characterization.

## Out of Scope
- Pitot tube / airspeed sensor (would fix alpha, but hardware change)
- Magnetometer fusion (yaw drift acceptable for ~10s spans)
- Real-time AHRS switching or voting
- GPS-dependent inputs (unreliable, intermittent)

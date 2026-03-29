# Research: 021 — AHRS Cross-Check + NN Input Redesign

## R1: INAV Gyro Filter Chain

**Decision**: Use INAV's filtered gyro (gyroADCf) for NN input, but evaluate
whether the 25Hz main LPF cutoff introduces too much lag at the NN's 30Hz tick rate.

**Current filter chain** (in order, at ~1kHz loop rate):
1. Anti-aliasing LPF @ 250Hz (at 8kHz gyro sample rate)
2. Main LPF @ 25Hz (PT1, STATIC mode) ← **potential concern**
3. Dynamic notch (Q=250, min 30Hz, 2D mode)
4. RPM filter: OFF
5. LULU filter: OFF
6. Kalman: not configured

**Concern**: 25Hz LPF on a 30Hz NN tick means the gyro signal is bandwidth-limited
below the NN's Nyquist. During a step input (sharp roll command), the filtered
gyro will show a delayed, smoothed rise vs the actual angular rate. The NN sees
"I commanded a roll 30ms ago but the rate hasn't risen yet" — exactly the lag
that causes overshoot.

**Alternatives considered**:
- Use gyroRaw (pre-filter): too noisy, vibration-dominated
- Raise main LPF to 100Hz: would provide cleaner rate signal for NN, but may
  destabilize INAV's own PID if it relies on the 25Hz filtering
- Separate filter path for NN vs PID: requires INAV code change, adds complexity

**Resolution**: Phase 1 characterization flight logs both gyroRaw and gyroADC.
Compare during pilot step inputs to quantify the lag. If >1 sample delay at 30Hz
(>33ms group delay), consider raising LPF or adding a separate NN-specific filter
tap in INAV.

## R2: CRRCSim Rate PID Implementation

**Decision**: Implement a simple PID in `inputdev_autoc.cpp` that maps NN rate
commands to surface deflections, matching INAV's ACRO behavior.

**INAV ACRO PID (active config)**:
- Roll:  FF=50, P=5, I=7, D=0
- Pitch: FF=50, P=5, I=7, D=0
- Yaw:   FF=60, P=6, I=10, D=0
- Feedforward-dominant: FF does most of the work, P/I handle steady-state

**CRRCSim implementation approach**:
- LaRCSim FDM already computes body rates (P_body, Q_body, R_body) internally
- Extract these via the FDM interface (or compute from quaternion derivative)
- PID: `output = FF*cmd + P*(cmd_rate - actual_rate) + I*integral(error)`
- No D term (matching INAV config)
- Rate limits: roll 560°/s, pitch 400°/s, yaw 240°/s
- PID gains: start from INAV values, tune to match Phase 1 measured step responses

**Rationale**: Must match INAV's actual rate response so the NN's trained
rate commands produce similar physical outcomes on real hardware.

## R3: Gravity Vector from Accelerometer

**Decision**: Normalize INAV's filtered accelerometer to unit vector for NN input.

**Computation**:
```
accel_body = [ax, ay, az]  // from INAV filtered accel (milli-g or similar)
gravity_unit = accel_body / |accel_body|  // normalize to unit vector
```

**In steady flight**: gravity_unit ≈ R^T * [0, 0, -1] where R is body-to-earth rotation.
- Wings level: gravity_unit ≈ [0, 0, -1]
- 30° bank right: gravity_unit ≈ [0, 0.5, -0.87]
- 90° bank: gravity_unit ≈ [0, 1, 0]

**During maneuvers**: centripetal acceleration adds to gravity. In a coordinated
2g turn, |accel| = 2g and the vector tilts. This is actually useful information —
it tells the NN the load factor, not just the bank angle. The NN should learn to
handle this naturally since the sim FDM produces the same centripetal effects.

**In CRRCSim**: compute from FDM's body-frame acceleration output, which already
includes gravity + inertial forces.

**Scaling**: already in [-1, 1] range after normalization. No additional scaling needed.

## R4: NN Hidden Layer Topology

**Decision**: Start with 26→16→8→3 (minimal change), evaluate convergence.

**Current**: 29→16→8→3 = 643 weights
**Proposed**: 26→16→8→3 = 571 weights (11% fewer)

**Analysis**:
- Reducing inputs from 29→26 removes 48 first-layer weights (3 fewer inputs × 16 neurons)
- The hidden layers (16→8→3) are unchanged
- The NN's capacity to represent the control law is essentially the same
- The inputs are arguably more informative (rate gyros provide derivative info
  that the NN previously had to infer from consecutive quaternion samples)

**If convergence is poor**:
- Option B: 26→20→10→3 = 753 weights — wider hidden layers
- The wider first hidden layer (20) gives more features to extract from the
  new gravity+rate input combination
- Monitor: if BIG training plateaus above BIG3's fitness, try wider

**Not recommended**:
- Deeper (3+ hidden layers): control is reactive, not sequential. Depth adds
  latency in the forward pass with no representational benefit.
- Recurrent (LSTM/GRU): would help with temporal patterns but massively
  increases weight count and training complexity. The history inputs (dPhi/dTheta/dist
  past samples) already provide a fixed temporal window.

## R5: Input Scaling

**Decision**: All inputs normalized to approximately [-1, 1].

| Input | Raw range | Scaling |
|-------|-----------|---------|
| dPhi, dTheta | [-π, π] rad | divide by π |
| dist | [0, ~100] m | divide by 50 (center at 1.0) |
| closing rate | [-30, 30] m/s | divide by 30 |
| gravity vector | [-1, 1] | already normalized |
| gyro p (roll rate) | [-560, 560] °/s | divide by 560 |
| gyro q (pitch rate) | [-400, 400] °/s | divide by 400 |
| gyro r (yaw rate) | [-240, 240] °/s | divide by 240 |
| airspeed | [5, 25] m/s | (v - 15) / 10 → [-1, 1] |

Note: existing path sensor scaling may differ — check evaluator.cc for current
normalization before changing. Some inputs may be fed raw (radians, meters).

## R6: LSM6DS3 on Xiao BLE Sense

**Decision**: Use Arduino LSM6DS3 library via PlatformIO for Phase 1 cross-check.

**Hardware**: LSM6DS3TR-C at I2C address 0x6A
- 3-axis accel: ±2/4/8/16g, 16-bit
- 3-axis gyro: ±125/250/500/1000/2000°/s, 16-bit
- No onboard fusion — raw sensor data only

**Library**: `arduino-libraries/Arduino_LSM9DS1` or Seeed's `Seeed_Arduino_LSM6DS3`
- Check PlatformIO registry for nRF52840 compatibility
- Need: `begin()`, `readAcceleration()`, `readGyroscope()`

**Madgwick filter**: Use open-source MadgwickAHRS (Sebastian Madgwick, x-io Technologies)
- Single .h/.cpp, no dependencies
- `MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az)` → quaternion
- Beta parameter: start at 0.1 for 100Hz update rate
- ~50μs per update on Cortex-M4F with FPU

**I2C timing**: 400kHz fast mode, 14 bytes per read (accel+gyro), ~0.3ms per transaction.
At 100Hz: 30ms budget per read, well within margin.

## R7: ACRO Mode Channel Mapping

**Decision**: Change xiao CH6 override from MANUAL to ACRO mode selection.

**Current**: CH6 = 1000 → MANUAL mode (via `aux` config in INAV)
**New**: CH6 value that selects ACRO mode instead

**INAV `aux` configuration**: need to check current mode/channel mapping.
The `aux` lines in inav-hb1.cfg define which channel values activate which modes.
May need to remap or add an ACRO range on CH6.

**For Phase 1**: not needed (flying MANUAL only, no autoc spans).
**For Phase 2**: update xiao to send CH6 value that selects ACRO, update INAV aux config.

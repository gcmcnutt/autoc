# Sensor Pipeline: INAV → Xiao → NN → Servos

**Last verified**: 2026-03-21 against flight data from 2026-03-20

This document traces every value conversion from INAV sensors through to NN inputs
and from NN outputs back to servo commands. Reference: `docs/COORDINATE_CONVENTIONS.md`.

---

## 1. INAV MSP Export (MSP2_INAV_LOCAL_STATE)

**Source**: `~/inav/src/main/fc/fc_msp.c` lines 672-693

INAV exports state in **NEU frame** (North-East-Up):

| Field | Wire format | Units | Frame |
|-------|-------------|-------|-------|
| timestamp_us | uint32 | microseconds since boot | — |
| pos[0] | int32 | centimeters | North |
| pos[1] | int32 | centimeters | East |
| pos[2] | int32 | centimeters | **Up** (positive = above origin) |
| vel[0] | int32 | cm/s | North |
| vel[1] | int32 | cm/s | East |
| vel[2] | int32 | cm/s | **Up** |
| q[0-3] | float32 ×4 | unitless | quaternion (w,x,y,z) body→NEU |

**Note**: INAV's internal frame is NEU. The quaternion represents body→earth (NEU).

---

## 2. Xiao MSP Receive

**Source**: `xiao/include/MSP.h` lines 260-266, `xiao/src/msplink.cpp` line 295

```cpp
struct msp_local_state_t {
  uint32_t timestamp_us;  // μs since INAV boot
  int32_t pos[3];         // cm, NEU
  int32_t vel[3];         // cm/s, NEU
  float q[4];             // quaternion (w,x,y,z) body→NEU
} __attribute__((packed));
```

MSP request: `performMspRequest(MSP2_INAV_LOCAL_STATE, &state.local_state, sizeof(...))`

Timestamp conversion: `state.inavSampleTimeMsec = state.local_state.timestamp_us / 1000`

---

## 3. NEU → NED Conversion

**Source**: `xiao/src/msplink.cpp` lines 132-153

### Position & Velocity

```cpp
static gp_vec3 neuVectorToNedMeters(const int32_t vec_cm[3]) {
  gp_scalar north =  vec_cm[0] * 0.01f;   // N: same sign
  gp_scalar east  =  vec_cm[1] * 0.01f;   // E: same sign
  gp_scalar down  = -vec_cm[2] * 0.01f;   // D: NEGATED (Up→Down)
  return gp_vec3(north, east, down);
}
```

| Axis | INAV (NEU, cm) | Xiao (NED, m) | Conversion |
|------|---------------|---------------|------------|
| X/North | pos[0] | position.x | ÷100 |
| Y/East | pos[1] | position.y | ÷100 |
| Z | pos[2] (Up) | position.z (Down) | ÷100, **negated** |

**Verified**: Position X/Y matches INAV within 0.01m. Z negation confirmed
(`xiao_z = -inav_z/100`). Velocity same conversion. See flight-timing-20260320.md.

### Quaternion

```cpp
static gp_quat neuQuaternionToNed(const float q[4]) {
  gp_quat attitude(q[0], -q[1], -q[2], -q[3]);  // Full conjugate
  attitude.normalize();
  return attitude;
}
```

| Component | INAV | Xiao | Conversion |
|-----------|------|------|------------|
| w | q[0] | w | same |
| x | q[1] | -q[1] | negated |
| y | q[2] | -q[2] | negated |
| z | q[3] | -q[3] | negated |

INAV sends body→NEU. Conjugation gives NEU→body ≡ earth→body (in NED frame context).

**Note on board alignment**: INAV's MSP quaternion is in sensor/IMU frame. `align_board_yaw`
is applied internally for INAV's own heading display but NOT to the MSP quaternion.
Flight hardware has `align_board_yaw=0` (no offset). Bench hardware has ~138° offset
due to different IMU mount — not a pipeline bug.

---

## 4. Origin Offset (Armed Position)

**Source**: `xiao/src/msplink.cpp` lines 816-859

When NN control activates, the current position is captured as `test_origin_offset`.
All subsequent positions are made relative:

```cpp
position_rel = position_raw - test_origin_offset;
```

- `pos_raw` in Nav State log = absolute INAV position (NED meters)
- `pos` in Nav State log = origin-relative (NED meters, 0,0,0 at arm point)
- Velocity is always absolute (not origin-relative)

---

## 5. NN Input Computation (sensor_math)

**Source**: `src/eval/sensor_math.cc` lines 144-164

All sensor math operates in **NED frame**, converting to **body frame** via the quaternion.

### dPhi (roll-plane bearing to target)

```cpp
gp_vec3 craftToTarget = targetPos - aircraftState.getPosition();
gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
return fastAtan2(target_local.y(), -target_local.z());
```

Projects target vector into body frame, computes angle in body YZ plane from body -Z axis.

### dTheta (pitch-plane bearing to target)

```cpp
gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
return fastAtan2(-target_local.z(), target_local.x());
```

Projects target vector into body frame, computes angle in body XZ plane from body +X axis.

### dist (Euclidean distance to target)

```cpp
gp_scalar dist = (targetPos - aircraftState.getPosition()).norm();
```

### 29 NN Input Layout

| Index | Name | Source | Units |
|-------|------|--------|-------|
| 0-5 | dPhi history+forecast | sensor_math | radians |
| 6-11 | dTheta history+forecast | sensor_math | radians |
| 12-17 | dist history+forecast | sensor_math | meters |
| 18 | dDist/dt | derivative | m/s |
| 19-22 | quaternion (w,x,y,z) | aircraft_state | unitless |
| 23 | airspeed | velocity.norm() | m/s |
| 24 | alpha (angle of attack) | atan2(-vel_body.z, vel_body.x) | radians |
| 25 | beta (sideslip) | atan2(vel_body.y, vel_body.x) | radians |
| 26-28 | previous command feedback | pitch, roll, throttle | [-1,1] |

History/forecast temporal slots: `[-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]`

---

## 6. NN Output → RC Commands

**Source**: `xiao/src/msplink.cpp` lines 882-901

NN outputs are tanh-activated: range [-1.0, +1.0].

### Channel Mapping

| NN output | Axis | Formula | PWM range | Sign |
|-----------|------|---------|-----------|------|
| out[0] | Pitch | `1500 - cmd * 500` | [2000, 1000] | **INVERTED** |
| out[1] | Roll | `1500 + cmd * 500` | [1000, 2000] | DIRECT |
| out[2] | Throttle | `1500 + cmd * 500` | [1000, 2000] | DIRECT |

**Pitch inversion**: NN +1.0 (pitch up) → PWM 1000. This matches CRRCSim where
`elevator = -pitchCommand / 2.0` (positive pitch command → negative elevator deflection
→ nose up).

### MSP Send Channel Assignment

```cpp
state.command_buffer.channel[0] = cached_roll_cmd;    // AETR ch0 = roll
state.command_buffer.channel[1] = cached_pitch_cmd;   // AETR ch1 = pitch
state.command_buffer.channel[2] = cached_throttle_cmd; // AETR ch2 = throttle
```

**IMPORTANT**: INAV channel remapping (`rcmap`) may reorder these. The `msp_override_channels`
bitmask determines which channels are actually overridden. Verify INAV config matches.

---

## 7. CRRCSim Equivalent (Training Environment)

**Source**: `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` lines 950-962

```cpp
inputs->elevator  = -pitchCommand / 2.0;          // NN [-1,1] → sim [-0.5, 0.5], INVERTED
inputs->aileron   =  rollCommand / 2.0;            // NN [-1,1] → sim [-0.5, 0.5], DIRECT
inputs->throttle  =  throttleCommand / 2.0 + 0.5;  // NN [-1,1] → sim [0, 1], DIRECT
```

| NN output | Sim input | NN +1.0 → | NN -1.0 → | NN 0.0 → |
|-----------|-----------|-----------|-----------|----------|
| pitch | elevator | -0.5 (nose up) | +0.5 (nose down) | 0.0 |
| roll | aileron | +0.5 (right) | -0.5 (left) | 0.0 |
| throttle | throttle | 1.0 (full) | 0.0 (idle) | 0.5 (half) |

**Sim latency model**: commands are staged in `PendingCommand` and applied after
`gComputeLatencyMsec` (default 40ms). This models the real MSP transport + NN eval delay.
Measured real-world steady-state latency is 48ms — close match.

---

## 8. INAV Processing of Override Commands

**Source**: `~/inav/src/main/rx/msp_override.c`, `~/inav/src/main/fc/fc_core.c`

MSP RC override values go through INAV's **full control pipeline**:

1. `mspOverrideChannels()` replaces rcChannels[].data with xiao values
2. `rxGetChannelValue()` reads overridden values
3. `rcCommand[ROLL] = getAxisRcCommand(...)` — applies **expo curve** + **deadband**
4. `rcCommand *= rates[axis] / 100` — applies **rate scaling**
5. `applyRateDynamics()` — applies **rate dynamics filter**
6. **PID controller** tracks commanded rates → servo output
7. **Mixer** combines pitch/roll/yaw → individual servo deflections

**Critical for flight mode selection**:
- **ACRO mode**: rcCommand is a **rate command** (deg/s). PID tracks it.
- **MANUAL mode**: rcCommand goes more directly to servos. Closer to sim behavior.
- **CRRCSim**: applies commands directly to control surfaces. No PID.
- **Recommendation**: Use MANUAL mode for NN flights (T221b).

---

## 9. Timing & Latency

See `specs/018-flight-analysis/flight-timing-20260320.md` for detailed measurements.

| Stage | Latency | Notes |
|-------|---------|-------|
| INAV sensor → MSP delivery | ~0-50ms | Within one xiao poll cycle |
| Xiao Nav State → NN eval | 2.5ms ±1.5ms | Fast NN forward pass |
| NN eval → MSP RC send | 0-50ms | **Unsynchronized ticker** (T221c) |
| MSP RC → INAV rcData | ~0-10ms | MSP frame processing |
| INAV freshness guard | 791ms ±7ms | **One-time at activation** |
| CRRCSim compute latency | 40ms | `COMPUTE_LATENCY_MSEC_DEFAULT` |
| Clock drift | -1.04 ms/sec | xiao vs INAV oscillators |

**Total steady-state pipeline**: ~50-100ms (sim models 40ms).

---

## Verification Status

| Check | Status | Evidence |
|-------|--------|----------|
| Position X/Y | ✅ Verified | < 0.01m error across 2013 samples |
| Position Z negation | ✅ Verified | `xiao_z = -inav_z/100` consistent |
| Velocity X/Y | ✅ Verified | < 0.07 m/s error |
| Velocity Z negation | ✅ Verified | Same as position |
| Quaternion conjugation | ✅ Verified | < 0.02 error on matched samples |
| Timestamp correlation | ✅ Verified | Consistent ~515ms offset, -1.04ms/s drift |
| Board alignment | ✅ Verified | Flight hw: 0°, bench: 138° (IMU mount) |
| NN eval latency | ✅ Verified | 2.5ms mean |
| RC round-trip | ✅ Verified | 48ms mean (steady-state) |
| Freshness guard | ✅ Verified | 791ms ±7ms across 5 spans |
| Pitch sign convention | ⚠️ Not flight-verified | Matches sim, needs response verification |
| Roll sign convention | ⚠️ Not flight-verified | Matches sim, needs response verification |
| Throttle sign convention | ⚠️ Not flight-verified | Matches sim, needs response verification |

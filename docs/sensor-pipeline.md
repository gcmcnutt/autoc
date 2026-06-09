# Sensor Pipeline: INAV → Xiao → NN → Servos

> **⚠️ SCOPE: pathgen mode (M0/M1) only.**
> This document was last verified 2026-03-21 against the **23-input pathgen NN**
> (rabbit direction-cosines + distance + chase Euler/quat + relVel + dDist/dt + …).
> The 030 **tracker-mode NN has 45 different inputs** (36 wingtip-beacon NDC slots
> with 6-tick history + chase quat + cruise-normalized airspeed + body gyro +
> tanh-saturated dist-to-boundary). For tracker-mode see
> [COORDINATE_CONVENTIONS.md → 030 Tracker-Mode NN Inputs](COORDINATE_CONVENTIONS.md#030-tracker-mode-nn-inputs-45-floats).
> Xiao firmware does NOT implement tracker-mode yet (see BACKLOG).

**Last verified**: 2026-03-21 against flight data from 2026-03-20

This document traces every value conversion from INAV sensors through to NN inputs
and from NN outputs back to servo commands, **for the pathgen-mode NN only**.
Reference: `docs/COORDINATE_CONVENTIONS.md`.

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
| Pitch sign convention | ✅ Verified | Pilot r=+0.88, override r=+0.04 (same direction, weaker due to chaos) |
| Roll sign convention | ✅ Verified | Pilot r=+0.25, override matches (high rc0 → roll right in both modes) |
| Throttle sign convention | ✅ Verified | r=+0.26 to +0.52 across all spans (more throttle → more speed) |
| Sim↔flight quat convention | ✅ Verified | Both use negative qw for southerly headings, same cause-effect pattern |
| Sim↔flight cause-effect | ✅ Verified | pitch→ΔdTheta and roll→ΔdPhi correlations match sign and magnitude |

## 10. Sim↔Flight Cause-Effect Verification (2026-03-21)

**Critical finding**: The "inverted" correlations (pitch_cmd → negative ΔdTheta, roll_cmd →
negative ΔdPhi) are **identical in sim and flight**. This is correct behavior in the
body-frame sensor convention:

| Correlation | Sim (41990 samples) | Flight (453 samples) | Match? |
|-------------|--------------------|--------------------|--------|
| pitch_cmd → ΔdTheta | r=-0.18 (3-step) | r=-0.18 (avg) | ✅ Same sign/magnitude |
| roll_cmd → ΔdPhi | r=-0.25 (3-step) | r=-0.35 (avg) | ✅ Same sign |
| thr_cmd → Δvel | r=+0.46 | r=+0.35 | ✅ Same sign |

**Interpretation**: When the NN commands pitch up, dTheta *decreases* because the target
moves downward in body frame (nose went up, target that was ahead is now more below).
This is geometrically correct. The NN learned this relationship during training and its
commands are appropriate for the convention.

**Quaternion convention match**: Both sim and flight show `qw < 0` for southerly headings.
The bench test table in `COORDINATE_CONVENTIONS.md` was verified with nose-north (qw=+1.0).
For nose-south (yaw=180°), `qw = cos(90°) = 0`, and INAV/crrcsim both choose the
negative-w branch. The conjugation in `neuQuaternionToNed` handles this consistently.

**Conclusion**: The sensor pipeline conventions are correct end-to-end. The first flight
failed due to other factors (ACRO mode PID interference, 790ms activation delay, possible
aircraft model mismatch), NOT coordinate convention errors.

**Action items for next flight** (no convention changes needed):
- T221b: Use MANUAL mode (eliminates PID rate tracking, closer to sim's direct control)
- T221c: Synchronized send loop (reduces pipeline latency from ~48ms to ~5ms)
- T221a: Remove unnecessary 200ms arm countdown

---

## 11. Tracker-Mode Sensor Pipeline (030 + 032)

Tracker mode is an alternate evaluation pipeline parallel to the M1 pathgen path documented in §1–§10. It feeds the **same NN forward-pass machinery** with a **different sensor surface** — instead of synthesized bearing-to-target scalars (`dPhi/dTheta/dist`), tracker mode projects two physical wingtip beacons through a chase-mounted camera and presents the NN with **NDC beacon observations plus chase attitude/state + derived perceptual features**.

Today this is sim-only — autoc desktop training via the crrcsim worker. Xiao firmware does NOT yet implement tracker mode; this section is the **xiao-migration prep contract** so when xiao tracker support lands, it inherits the sim invariants exactly.

### 11.1 Input vector shape (54 floats — 032 phase 1)

Full layout in [COORDINATE_CONVENTIONS.md §"030/032 Tracker-Mode NN Inputs"](COORDINATE_CONVENTIONS.md). Headline:

| Slots | What | Source |
|---|---|---|
| 0–35  | 2 beacons × {x, y, CEP} × 6-tick history (100ms grid) | `projectBeacon` (per-tick) → shifted history ring |
| 36–43 | Chase quat (4) + cruise-normalized airspeed (1) + body gyro (3) | `chase.getOrientation/getRelVel/getGyroRates` |
| 44    | Soft-saturated forward arena margin | `tanh(distanceToBoundary / kDistToBoundaryScale_m)` |
| 45–53 | **032 phase 1 derived features**: beacon-pair span ×6 history, span_rate, target_tilt (sin, cos) | `compute_pair_span` + `compute_tilt` in `include/autoc/eval/derived_features.h` |

### 11.2 Identity-stable beacon slot mapping (Feature A invariant)

`beacon_l_*` slots ALWAYS carry the **port** beacon (target body −y mount, red wingtip). `beacon_r_*` slots ALWAYS carry the **starboard** beacon (target body +y mount, green wingtip). This is **independent of which beacon lands on which image side** at any given tick — target heading away swaps image sides; the slot mapping does not swap.

**Sim enforcement (current)**: `tracker_stepper.cc::projectAndShiftHistory` (autoc reference impl) and `crrcsim_tracker_helper.cpp::projectAndShiftHistory` (crrcsim worker) both construct projections using `beacon_left_` config → store in `left` local → write to `history_.left_*`. No NDC-x sort. The mapping is by mount identity, period.

**Xiao migration contract** (for when xiao tracker-mode lands):
- FPGA emits per-detection `(x, y, CEP, code_id)` tuples
- Xiao firmware MUST map `code_id → {port, starboard}` via the Gold-code-to-physical-port table (build-time constant in xiao firmware, or MSP-configured)
- Xiao firmware MUST populate `TrackerInputs::beacon_l_*` from the port-keyed detection and `beacon_r_*` from the starboard-keyed detection, regardless of image-plane position
- Xiao firmware MUST NEVER sort by image-plane `x` or by `code_id` numeric value
- When a code is undetected for the current frame, populate the slot with the CEP-sentinel pattern (`screen_x = 0, screen_y = 0, cep = kCepSentinelFloat`) — same convention as sim's invisible-beacon case

**Why this matters**: if port/starboard swap between consecutive ticks, the NN sees apparent +180° tilt flips on otherwise-steady target → contradictory gradients → no tilt-based learning possible. The contract test in `tests/gather_tracker_inputs_tests.cc` guards against accidental sim-side regression.

### 11.3 CEP-gating + neutral-substitution rule (032 phase 1 Q4)

The new 032 derived features (`beacon_pair_span[6]`, `span_rate`, `target_tilt_sin/cos`) are **CEP-gated per-tick**: if EITHER beacon's CEP at the current tick is ≥ `CepGateThreshold` (default 1.25, matching `kCepSentinelThreshold` from `camera_projection.h`), the derived features are **substituted with neutral values**:

| Derived feature | Neutral value (CEP-gated) | Meaning |
|---|---|---|
| `beacon_pair_span[5]` (now) | 0.0 | "no closing-distance signal" |
| `span_rate` | 0.0 (derives mechanically from `span[5] - span[4]`) | "no relative motion signal" |
| `target_tilt_sin` | 0.0 | "no roll signal" |
| `target_tilt_cos` | 1.0 | "wings level relative — no roll-pressure on controller" |

Together these represent "target straight ahead, in formation, no action needed." The NN gets quiet inputs during blind ticks rather than misleading non-zero values.

**Gate threshold knob**: `[DerivedFeatures] CepGateThreshold` in `autoc-tracker.ini` (default 1.25). Lower values gate more aggressively on noisy-but-present detections. Threaded to the crrcsim worker via `WorkerInit.cepGateThreshold` (workers are separate processes; no ConfigManager).

**Consequence acknowledged in 032 phase 1**: this convention deliberately gives the NN "everything's fine" inputs during blind ticks — it cannot distinguish "actually fine" from "blind, fine-looking inputs are masking it." Phase-2 lost-sight patrol would add explicit `vis_now` / `ticks_since_seen` signals on top. Captured in 032 spec.md §1.5.

### 11.4 Modality vs fault — design framing for the next sensor iteration

Beacon invisibility in this design is a **MODALITY** (geometry-driven, 270° wingtip emission cones + target-relative attitude), NOT a sensor fault. Many target attitudes naturally hide one or both beacons. Next-iteration sensor work should extract intent from partial visibility (single-beacon bearing + history, soft-fail substitution, per-beacon staleness timers), NOT engineer for fault robustness (that's a multi-sensor-era problem). Captured in 032 spec.md §1.7 and project memory `project_sensor_modality_vs_fault.md`.

### 11.5 History pre-fill at scenario start

`initScenario` calls `projectAndShiftHistory(source.samples[0])` six times to pre-fill the 6-tick history with a replicated tick-0 projection (so the NN's first real tick sees a coherent stationary-source history, not a step from zero). The 032 phase-1 span history (`history.span[6]`) inherits this convention automatically — same projection runs six times, span is computed inside `projectAndShiftHistory`, so all 6 history slots get the same first-valid span value at tick 0.

If the first tick is CEP-gated (e.g., starting geometry has target out of camera FOV), the neutral substitution applies during the pre-fill loop just as it would during normal-operation ticks — single convention covers both. No special-case "scenario-start neutral fill" branch.

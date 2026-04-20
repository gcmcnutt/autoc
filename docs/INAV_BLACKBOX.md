# INAV Blackbox CSV — Field-by-Field Convention Reference

**Fork**: `gcmcnutt/inav` (custom, `3b9715954` added quaternion logging in Jul 2025).
**Decoder**: `~/blackbox-tools` (passthrough, zero transformations).
**Consumer conventions**: [COORDINATE_CONVENTIONS.md](./COORDINATE_CONVENTIONS.md).

Source-cited conventions for every INAV blackbox CSV column used by autoc.
Companion to `COORDINATE_CONVENTIONS.md`; this file is the ground truth for
translating "what INAV logs" into "what our stack expects."

Every value in the CSV is **raw INAV internal units**. Any transformation to
autoc's stack (NED meters, q_EB Hamilton, aerospace RHR rad/s) happens at
the read boundary in consumer code. Never silently in the decoder.

## Timestamp

| Field | Source | Units | Notes |
|---|---|---|---|
| `time (us)` | `blackbox.c:1664` | µs | FC local monotonic clock. Not sim-aware. |

## Position & Velocity (navigation state)

| Field | Source | Units | Frame | Conversion |
|---|---|---|---|---|
| `navPos[0..2]` | `blackbox.c:1776` | cm | **NEU** (North, East, **Up**) | `/100, negate Z` → NED m |
| `navRealVel[0..2]` | `blackbox.c:1777` | cm/s | NEU | `/100, negate Z` → NED m/s |
| `navTgtPos[0..2]` | `blackbox.c:1780` | cm | NEU* | `/100, negate Z` (*see note) |
| `navTargetVel[0..2]` | `blackbox.c:1779` | cm/s | NEU | `/100, negate Z` |
| `navHeading` | slow frame | decideg | 0–3600 | `/10` → deg |
| `navTargetHeading` | `blackbox.c:1782` | decideg | 0–3600 | `/10` → deg |
| `navEPH`, `navEPV` | `blackbox.c:1773-1774` | cm | — | `/100` → m (GPS error) |
| `navSurface` | `blackbox.c:1783` | cm | AGL | `/100` → m |

*Note on `navTgtPos[2]`: internal nav state uses Down-positive for the Z
altitude control, but the blackbox emits the raw int32 regardless. Verify
sign at WI1 cross-check time.

## Attitude & Orientation

### Quaternion (custom-fork emission)

| Field | Source | Units | Convention |
|---|---|---|---|
| `quaternion[0..3]` | `blackbox.c:1733-1736` | ×10000 int16 | Scalar-first `[w,x,y,z]`, body→earth, INAV non-aerospace pitch |

**To convert to our stack's `q_EB` (Hamilton, aerospace RHR, NED)**:
1. Scale: `q_float = raw / 10000`.
2. Convert NEU→NED.
3. Correct INAV's pitch sign (nose-down-positive internally) to aerospace.

The exact composition is **deferred to WI5 compound-attitude bench verification**
(autoc spec 024). A simple conjugate is NOT sufficient — observed in flight
data as a pitch-axis sign inversion in `gyro_vs_quat.py` analysis.

The source filter is **Mahony AHRS** (gyro + accel + mag fusion) at `imu.c`.
The logged quaternion is therefore an *estimate* subject to complementary
filter lag and drift-correction behavior — not pure gyro integration.

### Euler angles

| Field | Source | Units | Order | Notes |
|---|---|---|---|---|
| `attitude[0]` | `blackbox.c:1729` | decideg | **roll** | aerospace convention (right-wing-down = +) |
| `attitude[1]` | `blackbox.c:1730` | decideg | **pitch** | **INAV** convention — nose-up = negative |
| `attitude[2]` | `blackbox.c:1731` | decideg | **yaw** | wrapped to [0, 3600] (0–360°) |

Derived from quaternion in `imu.c:566-568`. Cross-check: applying our own
Euler extraction to the (corrected) q_EB should match `attitude[]` after
negating pitch.

## Angular Rate (Gyro)

| Field | Source | Units | Order | Notes |
|---|---|---|---|---|
| `gyroADC[0]` | `blackbox.c:1674` | deg/s | roll (body +X, p) | Matches aerospace RHR |
| `gyroADC[1]` | `blackbox.c:1674` | deg/s | pitch (body +Y, q) | **INAV: nose-down = +** |
| `gyroADC[2]` | `blackbox.c:1674` | deg/s | yaw (body +Z, r) | **INAV: nose-left = +** |

Raw deg/s (**not × 16** as some hypotheses suggested). Post-calibration,
post-filter, pre-integration.

**Conversion to aerospace RHR rad/s**:
- `p = gyroADC[0] × π/180`
- `q = −gyroADC[1] × π/180` (negate)
- `r = −gyroADC[2] × π/180` (negate)

The xiao msplink `neuGyroToBodyRadS` function already performs this.

## Linear Acceleration

| Field | Source | Units | Frame | Notes |
|---|---|---|---|---|
| `accADC[0]` | `blackbox.c:1675` | `acc_1G/256` LSB | body +X (forward) | |
| `accADC[1]` | `blackbox.c:1675` | same | body +Y (right) | |
| `accADC[2]` | `blackbox.c:1675` | same | body +Z (down) | At level rest ≈ +256 |
| `accVib` | `blackbox.c:1701` | same scale | — | Vibration magnitude |

`acc_1G` is a per-device constant; typical ICM-series IMUs use 256 LSB/g.
Conversion: `accel_ms2 = accADC × (1/256) × 9.81`.

Body FRD frame matches aerospace body convention on all three axes. No sign
flips needed.

## Magnetometer

| Field | Source | Units | Frame |
|---|---|---|---|
| `magADC[0..2]` | `blackbox.c:1687` | raw ADC | body `[X, Y, Z]` |

Raw counts, no scaling to physical units. Use for direction only (normalize).

## Barometric altitude

| Field | Source | Units | Reference |
|---|---|---|---|
| `BaroAlt` | `blackbox.c:1751` | cm | Above ground (powerup calibration) |

Positive = above. `/100` → m AGL.

## Radio Control

| Field | Source | Units | Range | Element Order |
|---|---|---|---|---|
| `rcData[0..3]` | `blackbox.c:1725` | µs PWM | 1000–2000 | **AETR**: roll, pitch, throttle, yaw |
| `rcCommand[0..3]` | `blackbox.c:1726` | mixed | varies | **Roll, pitch, yaw, throttle** — *different order than rcData!* |

`rcCommand` has processing applied (stick expo, smoothing, deadband). The
throttle index swap between `rcData[2]` and `rcCommand[3]` is a frequent
source of bugs.

## Actuator Outputs

| Field | Source | Units | Range |
|---|---|---|---|
| `servo[N]` | `blackbox.c:1768` | µs PWM | 1000–2000 |
| `motor[N]` | `blackbox.c:1744` | µs PWM | min ~1000 to max ~2000 |

Starting index of servo logging is `minServoIndex` to skip unused channels.

## Flight State Flags

| Field | Source | Type | Notes |
|---|---|---|---|
| `flightModeFlags` | `blackbox.c:1438` | bitmask | RC *requested* modes |
| `flightModeFlags2` | `blackbox.c:1439` | bitmask | Extended flags |
| `activeFlightModeFlags` | `blackbox.c:1408` | bitmask | Actually *active* modes |
| `mspOverrideFlags` | `blackbox.c:1434` | bits 0–1 | Bit 0: failsafe; Bit 1: MSP override active |
| `navState` | `blackbox.c:1771` | enum | See `navigation.h` for values |
| `navFlags` | `blackbox.c:1772` | bitfield | See `navigation.c` |

## Sample Rate

| Category | Rate |
|---|---|
| Main frame (IMU, nav, RC, servo, motor, quat) | ~200 Hz (blackbox loop rate) |
| Slow frame (flight mode flags, navState) | ~10 Hz or on change |

## blackbox-tools (decoder) behavior

**Completely faithful passthrough.** No unit conversion, no sign flip, no
frame rotation applied during CSV export. All scaling / frame correction /
convention translation is the consumer's responsibility.

- `blackbox_decode.c:921` — quaternion stored `UNIT_RAW`, no `/10000`
- `blackbox_decode.c:910` — gyro stored `UNIT_RAW`, no `× π/180`
- `blackbox_decode.c:269–275` — navPos/Vel `UNIT_RAW`
- `blackbox_decode.c:913-914` — attitude `UNIT_RAW`, decideg
- Frame transformations only occur under `--simulate-imu` flag, and those
  produce separate derived columns rather than transforming the logged fields.

## Quick-reference conversion table

| Field | Raw → SI | Raw → our stack (aerospace + NED) |
|---|---|---|
| `time` | µs | — |
| `navPos[0,1,2]` | `/100` → m NEU | `/100`, negate Z → m NED |
| `navRealVel` | `/100` → m/s NEU | `/100`, negate Z → m/s NED |
| `quaternion` | `/10000` → unit quat body→earth NEU | WI5-bench-derived transform to q_EB aerospace |
| `attitude[0,1,2]` | `/10` → deg (roll, INAV-pitch, yaw) | `/10`, negate pitch → deg aerospace |
| `gyroADC[0,1,2]` | `× π/180` → rad/s (INAV body) | `× π/180`, negate [1] and [2] → rad/s aerospace |
| `accADC[0,1,2]` | `× 1/256 × 9.81` → m/s² body FRD | same (no transform needed) |
| `BaroAlt` | `/100` → m AGL | — |
| `magADC[0,1,2]` | raw | normalize for direction |

## Known gotchas

1. **navPos Z is up-positive**. Forgetting the negate is an easy slip.
2. **rcData vs rcCommand element order differs** — throttle moves from index
   2 (rcData) to index 3 (rcCommand).
3. **Attitude pitch is nose-down-positive** in the logged value, matching
   the configurator UI display convention.
4. **Quaternion conversion is not a simple conjugate** — composition of
   NEU→NED + pitch convention flip is needed. WI5 bench characterizes it.
5. **blackbox-tools applies no transforms** — don't look for "silent fixes"
   in the decoder output.
6. Custom-fork-only: mainline INAV blackbox does not have the `quaternion[0..3]`
   columns. Tools that assume mainline will error cleanly at column-lookup;
   confirm you're analyzing a log from `gcmcnutt/inav`.

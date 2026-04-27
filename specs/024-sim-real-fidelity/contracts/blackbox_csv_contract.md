# Contract: INAV Blackbox CSV

**Reader role**: `sensor_self_check.py` (WI1) and any future flight-analysis tool.
**Producer**: `~/blackbox-tools/blackbox_decode` against `.TXT` logs emitted by
`gcmcnutt/inav` (custom fork).
**Convention reference**: [docs/INAV_BLACKBOX.md](../../../docs/INAV_BLACKBOX.md).

## Format

- CSV with header row on line 1.
- Whitespace / comma separated. Field names come from the log H frame metadata;
  decoder does not invent names.
- Each row is one logged sample (main frame). Sample rate ~200 Hz.
- Separate `.01.gps.csv` and `.01.gps.gpx` files for GPS-only data; not required
  for WI1.

## Required columns

If any are missing, reader must **fail loud** with a message identifying the
missing column and this contract.

| Column | Raw type | Raw units | Frame |
|---|---|---|---|
| `time (us)` | int64 | µs | — |
| `navPos[0]`, `[1]`, `[2]` | int32 | cm | NEU |
| `navRealVel[0]`, `[1]`, `[2]` | int16 | cm/s | NEU |
| `quaternion[0]`, `[1]`, `[2]`, `[3]` | int16 | ×10000 | body→earth NEU, INAV pitch |
| `gyroADC[0]`, `[1]`, `[2]` | int16 | deg/s | body, INAV pitch/yaw |
| `accADC[0]`, `[1]`, `[2]` | int16 | `acc_1G/256` LSB | body FRD |
| `attitude[0]`, `[1]`, `[2]` | int16 | decideg | roll, pitch_INAV, yaw |
| `magADC[0]`, `[1]`, `[2]` | int16 | raw ADC | body |
| `BaroAlt` | int32 | cm AGL | — |
| `rcData[0]`, `[1]`, `[2]`, `[3]` | int16 | µs PWM | AETR |
| `rcCommand[0]`, `[1]`, `[2]`, `[3]` | int16 | mixed | roll/pitch/yaw/throttle |
| `mspOverrideFlags` | uint8 | bitmask | bit 0 failsafe, bit 1 active |

## Transformations applied at read boundary

The reader **must** convert before emitting canonical signals to downstream
analysis:

```python
# Position (NEU cm → NED m)
pos_ned_m = [navPos[0]/100.0, navPos[1]/100.0, -navPos[2]/100.0]

# Velocity (NEU cm/s → NED m/s)
vel_ned_ms = [navRealVel[0]/100.0, navRealVel[1]/100.0, -navRealVel[2]/100.0]

# Quaternion (WI5 characterizes; transform placeholder here)
#   q_EB = f(quaternion / 10000, NEU→NED, INAV_pitch → aerospace)
# Exact f is deferred to WI5 bench result.

# Gyro (INAV deg/s → aerospace rad/s)
p = gyroADC[0] * pi/180
q = -gyroADC[1] * pi/180  # negate pitch
r = -gyroADC[2] * pi/180  # negate yaw

# Accel (LSB → m/s²)
ACC_1G_LSB = 256
accel_ms2 = [accADC[i] * (1.0/ACC_1G_LSB) * 9.81 for i in range(3)]

# Euler
roll_rad = attitude[0] * 0.1 * pi/180
pitch_rad = -attitude[1] * 0.1 * pi/180  # negate pitch
yaw_rad = attitude[2] * 0.1 * pi/180

# Mag (normalize for direction only)
mag_vec = np.array(magADC[:3], dtype=float)
mag_dir = mag_vec / max(np.linalg.norm(mag_vec), 1e-9)
```

## Unknown / deferred

- **Exact quaternion transform** is the subject of WI5 compound-attitude
  bench characterization. Reader should either:
  1. Emit the raw post-scale quat and let downstream code apply the transform,
  2. Or apply a parameterized transform with clear docs about which bench-
     derived variant is in use.
- **navTgtPos[2] frame** (NEU or NED internal) — document what the WI1 data
  actually shows; the read boundary applies the same transform as navPos.

## Error modes

- **Missing required column** → abort with contract violation message.
- **Custom-fork vs mainline check** → reader asserts presence of `quaternion[0]`
  column; if absent, bail with a message pointing at the custom-fork requirement.
- **Sample rate sanity** → reader checks that median `time (us)` delta is
  within [1 ms, 50 ms] (expected ~5 ms at 200 Hz); warn if out of range.
- **Time monotonicity** → if `time (us)` rolls back or jumps, warn and skip.

## Testing

- Contract test against `flight-results/flight-20260417/blackbox_log_*.csv`:
  reader produces canonical signals, Pearson r of (read+transformed signal)
  vs (known sim signal at comparable attitude) is sign-correct.
- Smoke test: read head of CSV, assert presence of all required columns.

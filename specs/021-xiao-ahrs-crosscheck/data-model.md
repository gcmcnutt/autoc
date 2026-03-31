# Data Model: 021 — AHRS Cross-Check + NN Input Redesign

## MSP2_AUTOC_STATE Extended Payload (Phase 2)

Current (38 bytes):
```
pos[3]      int32   cm NEU          12 bytes
vel[3]      int16   cm/s NEU         6 bytes
quat[4]     int16   /10000 body→earth 8 bytes
flags       uint32  flightModeFlags  4 bytes
rc[2]       uint16  ch8, ch9 PWM     4 bytes
timestamp   uint32  microseconds     4 bytes
                                    --------
                              Total: 38 bytes
```

Extended (Phase 3, +6 bytes = 44 bytes):
```
pos[3]      int32   cm NEU          12 bytes
vel[3]      int16   cm/s NEU         6 bytes
quat[4]     int16   /10000 body→earth 8 bytes
flags       uint32  flightModeFlags  4 bytes
rc[2]       uint16  ch8, ch9 PWM     4 bytes
timestamp   uint32  microseconds     4 bytes
gyro[3]     int16   deci-deg/s body  6 bytes  ← NEW
                                    --------
                              Total: 44 bytes
```

Scaling:
- gyro: 10 = 1°/s. Range ±560°/s → ±5600. int16 sufficient.

Note: accel[3] deferred — only needed for AHRS cross-check (022), not NN input.

## NN Input Vector (Phase 2)

```
Index  Name                    Source              Scale
-----  ----                    ------              -----
0-3    dPhi history (t, t-1, t-2, t-3)  path+quat   radians
4-5    dPhi forecast (+0.1s, +0.5s)     path+quat   radians
6-9    dTheta history                    path+quat   radians
10-11  dTheta forecast                   path+quat   radians
12-15  dist history                      path+pos    meters
16-17  dist forecast                     path+pos    meters
18     closing rate                      dist deriv  m/s
19-22  quaternion (w, x, y, z)           AHRS        [-1,1] (unit quat)
23     airspeed                          vel mag     m/s
24-26  gyro rates (p, q, r)             gyro filt   rad/s (raw, no normalization)
-----
Total: 27 inputs

Changes from current (29 inputs):
  Removed: alpha/beta (2 inputs) — invalid without airspeed sensor
  Removed: previous commands (3 inputs) — unnecessary with ACRO rate feedback
  Added:   gyro rates p,q,r (3 inputs) — angular rate damping
  Net:     -5 + 3 = -2 inputs
```

## NN Output Vector (reinterpreted for ACRO)

```
Index  Name       Range    MANUAL (old)              ACRO (new)
-----  ----       -----    ------------              ----------
0      pitch      [-1,1]   direct servo deflection   desired pitch rate (×400°/s)
1      roll       [-1,1]   direct servo deflection   desired roll rate (×560°/s)
2      throttle   [-1,1]   direct throttle            throttle (unchanged)

Key difference: in ACRO, output=0 means "hold current attitude" (rates damp
to zero via INAV PID). In MANUAL, output=0 means "servos centered" (aircraft
does whatever physics dictates). ACRO gives the NN a stable baseline to
work from — it only needs to command rate deltas for steering.
```

## Xiao Local AHRS State — DEFERRED to 022

```
// Deferred: LSM6DS3 + Madgwick AHRS cross-check
// See 022 spec when created
```

## Safety Override State (Phase 8)

```
struct SafetyState {
    float origin[3];         // captured on autoc enable (NEU meters)
    float distance_from_origin;  // computed each tick
    float max_distance;      // threshold (e.g. 100m)
    bool tripped;            // latched on first violation
};
```

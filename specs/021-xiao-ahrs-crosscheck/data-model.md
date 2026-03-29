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

Extended (Phase 2, +12 bytes = 50 bytes):
```
pos[3]      int32   cm NEU          12 bytes
vel[3]      int16   cm/s NEU         6 bytes
quat[4]     int16   /10000 body→earth 8 bytes
flags       uint32  flightModeFlags  4 bytes
rc[2]       uint16  ch8, ch9 PWM     4 bytes
timestamp   uint32  microseconds     4 bytes
accel[3]    int16   milli-g body     6 bytes  ← NEW
gyro[3]     int16   deci-deg/s body  6 bytes  ← NEW
                                    --------
                              Total: 50 bytes
```

Scaling:
- accel: 1000 = 1g. Range ±4g → ±4000. int16 sufficient.
- gyro: 10 = 1°/s. Range ±560°/s → ±5600. int16 sufficient.

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
19-21  gravity vector body frame         accel norm  [-1,1]
22-24  gyro rates (p, q, r)             gyro filt   /max_rate → [-1,1]
25     airspeed                          vel mag     m/s
-----
Total: 26 inputs
```

## NN Output Vector (unchanged)

```
Index  Name       Range    Meaning in ACRO
-----  ----       -----    ---------------
0      pitch      [-1,1]   desired pitch rate (×400°/s max)
1      roll       [-1,1]   desired roll rate (×560°/s max)
2      throttle   [-1,1]   throttle command (mapped to 0-1)
```

## Xiao Local AHRS State (Phase 1)

```
struct LocalAHRS {
    float q[4];           // Madgwick quaternion output (w,x,y,z)
    float accel_raw[3];   // LSM6DS3 accel (g)
    float gyro_raw[3];    // LSM6DS3 gyro (°/s)
    uint32_t last_update_us;
    bool valid;
};
```

## Xiao Log Format Extension (Phase 1)

Current:
```
Nav State: pos_raw=[...] pos=[...] vel=[...] quat=[...] armed=Y ...
```

Extended:
```
Nav State: pos_raw=[...] pos=[...] vel=[...] quat=[...] q_local=[w,x,y,z] q_delta=X.X armed=Y ...
```

## Safety Override State (Phase 2)

```
struct SafetyState {
    float origin[3];         // captured on autoc enable (NEU meters)
    float distance_from_origin;  // computed each tick
    float max_distance;      // threshold (e.g. 100m)
    bool tripped;            // latched on first violation
};
```

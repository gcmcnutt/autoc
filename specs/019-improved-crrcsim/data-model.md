# Data Model: 019 Improved CRRCSim Fidelity

**Date**: 2026-03-24 | **Branch**: `019-improved-crrcsim`

## Entities

### hb1_streamer.xml (tuning target)

The CRRCSim aircraft model file. XML format loaded by `fdm_larcsim::LoadFromXML()`.

**Tunable parameters by phase:**

| Phase | Parameter | Current | Physical meaning | Tuning effect |
|-------|-----------|---------|------------------|---------------|
| 1 (Throttle) | F (thrust) | 5.0 N | Static thrust | Higher = faster, more climb |
| 1 (Throttle) | CD_prof | 0.18 | Parasitic drag | Higher = slower Vmax |
| 1 (Throttle) | V_ref (engine) | 11.1 m/s | Thrust reference speed | Shapes thrust-vs-speed curve |
| 2 (Roll) | Cl_da | 0.14 | Aileron roll moment | Higher = faster roll rate |
| 2 (Roll) | Cl_p | -0.55 | Roll damping | More negative = more damped |
| 3 (Pitch) | Cm_de | -0.19 | Elevator pitch moment | More negative = faster pitch |
| 3 (Pitch) | Cm_q | -4.2 | Pitch damping (streamer) | More negative = more damped |
| 3 (Pitch) | I_yy | 0.0013 | Pitch inertia | Lower = faster pitch response |
| 3 (Pitch) | CL_max | 1.2 | Stall lift limit | Lower = earlier stall |

### Response curve data (comparison artifact)

Extracted from flight blackbox and data.dat. Not persisted as a formal entity — generated
on-the-fly by analysis scripts, displayed as ascii art or simple plots.

**Per-axis curve:**
- X axis: control input magnitude (0 to 1.0 for throttle, -1 to +1 for pitch/roll)
- Y axis: response metric (speed for throttle, rate deg/s for roll/pitch)
- Binned by airspeed regime (slow <12, cruise 12-16, fast >16 m/s)

### data.dat fields used for sim-side extraction

From the header: `Scn Bake Pth/Wnd:Step: Time Idx dPh... dTh... ds... dddt qw qx qy qz vel alpha beta outPt outRl outTh cmdP cmdR cmdT pathX pathY pathZ X Y Z vxBody vyBody vzBody dhome dist attDlt rabVl intSc rampSc`

Key fields for response curves:
- `outPt/outRl/outTh` — NN output commands [-1, 1]
- `vel` — airspeed (m/s)
- `qw/qx/qy/qz` — orientation quaternion (derive attitude rates from consecutive steps)
- `X/Y/Z` — position (derive climb rate from consecutive Z values)
- `alpha/beta` — angle of attack, sideslip

### Flight blackbox CSV fields used for real-side extraction

Key columns (from 018 analysis):
- `rcCommand[0-2]` — pilot/NN commands (roll, pitch, throttle)
- `attitude[0-2]` — Euler angles (derive rates)
- `navVel[0-2]` — NED velocity (cm/s)
- `navPos[0-2]` — NED position (cm)
- `accSmooth[0-2]` — body-frame accelerometer
- `flightModeFlags` — MSPRCOVERRIDE detection
- `quaternion[0-3]` — body→earth quat (flight-results CSVs, scaled /10000)

### MSP2_AUTOC_STATE (new, Phase 6)

Custom MSP2 command payload. Binary, little-endian.

| Offset | Field | Type | Units |
|--------|-------|------|-------|
| 0 | navPos[0] (N) | int32 | cm |
| 4 | navPos[1] (E) | int32 | cm |
| 8 | navPos[2] (D) | int32 | cm |
| 12 | navVel[0] (N) | int16 | cm/s |
| 14 | navVel[1] (E) | int16 | cm/s |
| 16 | navVel[2] (D) | int16 | cm/s |
| 18 | quat[0] (w) | int16 | /10000 |
| 20 | quat[1] (x) | int16 | /10000 |
| 22 | quat[2] (y) | int16 | /10000 |
| 24 | quat[3] (z) | int16 | /10000 |
| 26 | rcData[0] (roll) | uint16 | PWM us |
| 28 | rcData[1] (pitch) | uint16 | PWM us |
| 30 | rcData[2] (throttle) | uint16 | PWM us |
| 32 | rcData[3] (yaw) | uint16 | PWM us |
| 34 | armingFlags | uint32 | bitfield |
| 38 | (total) | | 38 bytes |

## State transitions

### Tuning iteration cycle (per axis)

```
[Extract real curve] → [Extract sim curve] → [Compare]
    ↑                                            |
    |                                     [Good enough?]
    |                                       /         \
    |                                    yes           no
    |                                     |             |
    |                              [Next axis]    [Adjust hb1.xml]
    |                                             [Mini training]
    └─────────────────────────────────────────────────┘
```

### Overall feature flow

```
Phase 1 (Throttle) → Phase 2 (Roll) → Phase 3 (Pitch) → Phase 4 (Integration)
    → Phase 5 (Sensor audit) → Phase 6 (Pipeline + retrain)
```

Phases 1-3 iterate independently. Phase 4 may loop back to 1-3 for cross-axis fixes.
Phase 6 is terminal — produces the trained controller for flight test.

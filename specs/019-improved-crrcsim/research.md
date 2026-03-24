# Research: 019 Improved CRRCSim Fidelity

**Date**: 2026-03-24 | **Branch**: `019-improved-crrcsim`

## R1: CRRCSim FDM aero model structure

**Decision**: LaRCSim coefficient-based model with linear stability derivatives.

**Rationale**: The FDM in `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` computes
forces/moments from linear coefficient tables loaded from hb1_streamer.xml. Each axis
is a sum of terms: base + alpha contribution + rate damping + control input. This means
tuning is straightforward — each coefficient has a predictable effect on one response
characteristic.

**Key equations (from fdm_larcsim.cpp aero()):**
```
Cm = Cm_0 + Cm_a*alpha + Cm_q*Qhat + Cm_de*elevator
Cl = Cl_b*beta + Cl_p*Phat + Cl_da*aileron
CD = CD_prof + CD_CLsq*(CL - CL_CD0)^2 + CD_ELsq*elev^2 + CD_AIsq*ail^2
```

Where Qhat/Phat are dimensionless rates (normalized by chord/span and airspeed).

**Alternatives considered**: None — CRRCSim's FDM is fixed. We tune coefficients, not
the model structure.

## R2: Current hb1_streamer.xml parameter values

**Decision**: Use current 018-tuned values as baseline, iterate from there.

| Parameter | Current value | Physical meaning |
|-----------|--------------|------------------|
| Cm_de | -0.19 | Elevator pitch effectiveness |
| Cm_q | -4.2 | Pitch damping (high = streamer drag) |
| Cm_a | -0.55 | Alpha static stability |
| Cl_da | 0.14 | Aileron roll effectiveness |
| Cl_p | -0.55 | Roll damping |
| CD_prof | 0.18 | Parasitic drag (high = streamer) |
| CL_a | 4.7 | Lift slope (per radian) |
| CL_max | 1.2 | Max lift (stall) |
| Mass | 0.515 kg | Aircraft + streamer |
| I_yy | 0.0013 kg*m^2 | Pitch inertia |
| I_xx | 0.024 kg*m^2 | Roll inertia |
| Thrust F | 5.0 N | Static thrust |
| U_ref | 11.0 m/s | Reference airspeed |

**Rationale**: 018 tuning brought roll close (sim 287°/s vs flight 270°/s) but pitch
is still off (sim 515°/s vs flight 127°/s after first pass, reduced to ~295°/s after
Cm_de/Cm_q adjustment but still 2.3× too fast). CD_prof increased from 0.065 to 0.18
to match real Vmax ~17 m/s (was ~32 m/s).

## R3: NN output → FDM control mapping

**Decision**: Mapping is correct and verified in 018. No changes needed.

```
elevator = -pitchCommand / 2.0    [-0.5, 0.5]
aileron  = rollCommand / 2.0      [-0.5, 0.5]
throttle = throttleCommand/2 + 0.5 [0.0, 1.0]
```

Full stick (NN ±1) maps to ±0.5 control input. FDM: `Cm = Cm_de * elevator` so
at full stick: `Cm = -0.19 * 0.5 = -0.095`.

**Rationale**: 018 audit confirmed sim ±0.5 = full deflection = real ±500 rcCmd = full
servo. No scaling confusion between sim and real paths.

## R4: Unit conversions at CRRCSim boundary

**Decision**: FDM uses feet/ft-per-sec internally. The inputdev_autoc bridge converts
to metres/m/s for AircraftState. All GP/NN operations are in SI + radians.

**Risk**: The ft→m conversion is in the bridge code, not the FDM. If any code path
reads FDM state without going through the bridge, it gets feet. Phase 5 sensor audit
will verify this by code inspection.

**Alternatives considered**: N/A — this is existing architecture, not a choice.

## R5: Custom MSP2 command feasibility

**Decision**: Add MSP2_AUTOC_STATE (command ID in MSP2 user range 0x4000-0x7FFF) to
INAV autoc branch. Payload: pos[3] + vel[3] + quat[4] + rc[4] + armingFlags = 38 bytes.

**Wire time analysis** (115200 baud, 1 byte = 10 bits = 86.8 µs):

Current 3-request pipeline (measured 35ms avg):
```
  MSP_STATUS request:          ~7 bytes  =  0.6 ms
  MSP_STATUS response:        ~27 bytes  =  2.3 ms
  + INAV scheduling delay                 ~  7 ms
  MSP2_INAV_LOCAL_STATE req:  ~10 bytes  =  0.9 ms
  MSP2_INAV_LOCAL_STATE rsp:  ~70 bytes  =  6.1 ms
  + INAV scheduling delay                 ~  7 ms
  MSP_RC request:              ~7 bytes  =  0.6 ms
  MSP_RC response:            ~39 bytes  =  3.4 ms
  + INAV scheduling delay                 ~  7 ms
  ─────────────────────────────────────────────────
  Total wire:                             ~ 14 ms
  Total scheduling (3 turnarounds):       ~ 21 ms
  Total:                                  ~ 35 ms  ✓ matches measurement
```

Custom MSP2_AUTOC_STATE (single round trip):
```
  Request:   10 bytes MSP2 frame (no payload)  =  0.9 ms
  Response:  10 bytes overhead + 38 payload    =  4.2 ms
  + INAV scheduling delay (1 turnaround)       ~  7 ms
  ─────────────────────────────────────────────────
  Total wire:                                  ~  5 ms
  Total scheduling:                            ~  7 ms
  Total fetch:                                 ~ 12 ms (possibly 8-10ms)
```

**Dominant cost is serial turnarounds, not wire time.** Eliminating 2 of 3 round trips
saves ~20ms. Marshalling on both sides is microseconds (negligible).

**Updated pipeline estimate:**
```
  Current:  fetch(35) + eval(5) + send(10) = 50ms total
  Custom:   fetch(8-12) + eval(5) + send(10) = 23-27ms total
```

**Dependency**: This is a dependent project (INAV firmware + xiao firmware) that must
be bench tested on flight hardware before updating COMPUTE_LATENCY in CRRCSim. The
measured value from bench becomes the sim latency for the next BIG training run.
Sequence: build → flash → bench measure → update sim → train.

**Implementation scope**:
- INAV: Add handler in `src/main/fc/fc_msp.c` for MSP2_AUTOC_STATE. Pack fields from
  existing internal state. Minimal local change to autoc branch.
- Xiao: Replace 3 MSP calls in `msplink.cpp` with single MSP2 request + parser.
- Bench: measure actual fetch latency, update COMPUTE_LATENCY_MSEC_DEFAULT to match.

## R6: INAV servo logging fix

**Decision**: Minimal local fix to servo index loop in autoc branch.

**Rationale**: Bug in INAV 8.0.0: `getServoCount()=2` but loop starts at index 0 instead
of `minServoIndex=1` for flying wings. Logs servo[0] (unused) and servo[1] (left elevon),
misses servo[2] (right elevon). Fix: change loop to iterate from minServoIndex. 2-line
change, bench verifiable.

## R7: PhysicsTraceEntry availability for tuning

**Decision**: PhysicsTraceEntry captures full FDM state (coefficients, forces, moments)
per timestep but only for elite reeval runs. For tuning, we can either:
1. Use data.dat NN inputs/outputs (always logged, coarser but sufficient)
2. Run targeted elite reeval to get full physics traces for specific scenarios

**Rationale**: For response curve extraction, data.dat provides NN commands + aircraft
state (position, velocity, quaternion) at 100ms intervals. This is sufficient to compute
attitude rates and speed changes. Full PhysicsTraceEntry adds coefficient-level detail
(CL, CD, Cm values) useful for debugging but not required for initial tuning.

# 020: Pre-Flight Pipeline

**Status**: Spec draft
**Predecessor**: 019-improved-crrcsim (sim fidelity + sensor audit)
**Goal**: Close the INAV→xiao→NN pipeline gaps, reduce latency, and produce a flight-ready controller.

## Problem Statement

019 established a tuned CRRCSim model and verified sensor scaling parity between desktop and xiao.
Before the next flight test, the pipeline itself needs fixes:

1. **Pipeline latency**: CRRCSim trains with COMPUTE_LATENCY=40ms but real xiao pipeline is ~10ms.
   Training at 4× real latency produces a conservative controller that under-reacts.
2. **INAV data fetch**: Xiao makes 3 separate MSP calls per tick (~35ms total). A single
   consolidated MSP2 command reduces this to ~8ms.
3. **INAV servo logging**: Blackbox logs servo[0] (unused) instead of servo[1-2] (both elevons).
   Post-flight analysis needs real servo deflection data.
4. **Flight mode override**: Currently requires pilot transmitter switch. Xiao should set the
   mode channel directly via MSP override.
5. **Board alignment**: INAV `align_board_roll=1700` (170°) instead of 1800 — ~10° pitch bias.
6. **Renderer playback**: Variable rabbit speed breaks progressive path reveal in training playback.

## Critical Outcomes

- **CO1**: Single MSP2_AUTOC_STATE command replaces 3 MSP calls (latency: 35ms → ~8ms)
- **CO2**: COMPUTE_LATENCY calibrated to bench-measured pipeline (~10-15ms)
- **CO3**: Both elevons logged in blackbox (servo[1] + servo[2])
- **CO4**: Xiao sets flight mode channel via MSP override (no pilot switch)
- **CO5**: Board alignment verified on bench (170° → 180° if confirmed)
- **CO6**: BIG training run with calibrated model + correct latency
- **CO7**: Renderer path reveal works with variable rabbit speed

## Scope

### In scope (P0 — blocks flight):
- INAV: MSP2_AUTOC_STATE custom command, servo logging fix, build
- Xiao: MSP2 consumer, flight mode override, build
- Bench test: latency measurement, board alignment check
- CRRCSim: COMPUTE_LATENCY update
- Training: BIG run with final parameters

### In scope (P1 — improves analysis):
- Renderer: path reveal fix for variable rabbit speed
- Board alignment correction if confirmed on bench

### Deferred (post-flight):
- Xiao onboard IMU cross-check (LSM6DS3TR-C AHRS)
- Full quaternion entry variation (CRRCSim native quat init)

## Technical Context

### Repositories and branches:
- `~/autoc` — autoc, renderer, shared code (branch: 020-pre-flight-pipeline)
- `~/autoc/crrcsim` — CRRCSim submodule (branch: 020-pre-flight-pipeline)
- `~/inav` — INAV custom fork (branch: autoc)
- `~/autoc/xiao` — Xiao PlatformIO project

### Key files:
- `~/inav/src/main/blackbox/blackbox.c` — servo logging
- `~/inav/src/main/fc/fc_msp.c` — MSP command handlers
- `~/autoc/xiao/src/msplink.cpp` — MSP parsing, NN control loop
- `~/autoc/xiao/src/MSP.cpp` — MSP protocol framing
- `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` — COMPUTE_LATENCY

### MSP2_AUTOC_STATE payload (38 bytes):
```
pos[3]       int32   cm      NED (converted from INAV NEU)
vel[3]       int16   cm/s    NED
quat[4]      int16   /10000  body→earth (xiao conjugates to earth→body)
rc[4]        uint16  PWM     channels 0-3
armingFlags  uint32          for arm state detection
```

### Bench hardware:
- STM32F405 flight controller with INAV autoc branch
- Xiao BLE Sense connected via UART
- Both on bench for flashing and testing

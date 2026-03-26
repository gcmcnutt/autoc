# 020: Pre-Flight Pipeline — Implementation Plan

## Execution Order

### Phase A: Servo Fix + Baseline Bench (simplest first)

1. **INAV servo logging fix** — change servo index loop in `blackbox.c` to start from
   `minServoIndex`. Flying wing: logs servo[1]+servo[2] (both elevons) instead of
   servo[0] (unused) + servo[1].
2. **Board alignment investigation** — analyze existing flight data for pitch bias.
   Compare reported attitude at 50% cruise "level" flight vs expected 0° pitch.
   The board alignment defines "level" relative to the airframe — at cruise the
   aircraft may fly at a natural angle of attack, so "level" in INAV may not mean
   0° pitch in flight. Determine how much of the observed bias is board alignment
   vs natural AoA vs IMU drift. Bench level-surface test to verify alignment setting.
3. **Blackbox sample rate** — set to 1/8 across the board for higher resolution.
4. **Build INAV** for MAMBAF722_2022A, flash to bench FC.
   - **NOTE**: Must temporarily disconnect GPS module to flash firmware on MAMBA.
5. **Build+deploy xiao** — wire-compatible (no xiao code changes yet), load recent
   training .dat weights.
6. **Bench run** → capture blackbox → verify both elevons appear in CSV log.

### Phase B: MSP2_AUTOC_STATE + Flight Mode Override

7. **INAV: Add MSP2_AUTOC_STATE handler** in `fc_msp.c`. Register command ID in
   MSP2 user range (0x4000-0x7FFF). Pack 38-byte payload: pos, vel, quat, rc,
   armingFlags per spec.md.
8. **Xiao: Replace 3 MSP calls with single MSP2_AUTOC_STATE** in `msplink.cpp`.
   Parse 38-byte response into existing state fields. Keep MSP_STATUS as fallback
   (polled every Nth tick) for diagnostics during development.
9. **Xiao: Flight mode channel override** (CO4) — include channel 6 in
   `msp_override_channels`, set to desired mode value. Removes pilot transmitter
   switch dependency. Bundle with MSP2 changes.
10. **Build+flash both** (INAV + xiao). Bench test:
    - Verify MSP2 response parses correctly (position, velocity, quat, rc)
    - Measure pipeline latency: xiao logs fetch→eval→send timing
    - Confirm flight mode override works (INAV enters correct mode without TX switch)

### Phase C: Latency Calibration + Training

11. **Update COMPUTE_LATENCY** in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`
    to match bench-measured total pipeline (fetch + eval + send). Expected: 40ms → ~10-15ms.
    Consider adding ±5ms jitter for robustness training.
12. **Renderer: fix path reveal** (CO7) — use rabbit odometer from AircraftState instead
    of proportional step count. Independent of bench work, can be done in parallel.
13. **BIG training run** — production autoc.ini (400 gens, pop=3000) with:
    - Calibrated hb1_streamer.xml (019 throttle passes)
    - Correct COMPUTE_LATENCY from bench measurement
    - Cone entry variations + variable rabbit speed
14. **Validate** — extract response curves, compare to flight blackbox. Deploy winning
    weights to xiao for flight test.

## Build Notes

### INAV (~/inav, autoc branch):
```
cd ~/inav
mkdir build        # clean build required
cd build
cmake ..
make MAMBAF722_2022A
```
- **IMPORTANT**: Disconnect GPS module before flashing MAMBA board.
- Bench target: MAMBAF722_2022A (STM32F722)
- Flight target: MATEKF722MINI — rebuild with clean `build/` dir for flight FC before actual flight.

### Xiao (~/autoc/xiao):
```
cd ~/autoc/xiao
pio run -e xiaoblesense_arduinocore_mbed
```

### Blackbox configuration:
- Sample rate: 1/8 (set via INAV configurator or CLI)
- Applies to both bench and flight configs

## Dependencies

```
Phase A (servo fix, baseline)
  └─ Phase B (MSP2 + flight mode override)
       └─ Phase C (latency calibration + BIG training)
            └─ Flight test
```

Renderer path reveal (step 12) is independent — can run in parallel with any phase.

# Flight Data Analysis — 2026-03-20 First NN Flight Test

## Data Sources

### INAV Blackbox
- **File**: `blackbox_log_2026-03-20_183938.TXT`
- **Decode**: `~/blackbox-tools/obj/blackbox_decode <file>` (produces `.01.csv`, `.01.gps.csv`, `.01.gps.gpx`, `.01.event`)
- **Duration**: 03:22, 60Hz, 12155 frames
- **Key columns**: `time (us)`, `navPos[0-2]` (cm, NEU), `navVel[0-2]` (cm/s), `quaternion[0-3]` (×10000), `attitude[0-2]` (decideg), `rcData[0-3]`, `servo[0-1]`, `motor[0]`, `mspOverrideFlags`

### Xiao Flight Log
- **File**: `flight_log_2026-03-21T01-47-21.txt`
- **Format**: `#<seq> <xiao_ms> <inav_ms> <level> <message>`
- **Rate**: ~10Hz (100ms NN eval cycle)
- **Key lines**: `Nav State:`, `NN Control:`, `Nav Control:`, `NN:`, `Path armed:`

## Timestamp Correlation

### Clock Reference
- **Column 2** (xiao_ms): milliseconds since xiao boot
- **Column 3** (inav_ms): milliseconds since INAV boot, sampled by xiao via MSP
- **INAV blackbox**: `time (us)` = microseconds since INAV boot
- **Common clock**: INAV boot time (column 3 in xiao = `time (us)` / 1000 in blackbox)

### Measured Offset
Consistent ~200ms offset between xiao's copy of INAV timestamp and blackbox timestamp
at the same event. This is MSP polling latency — xiao samples INAV state via serial,
there's a transport delay. The offset is stable (187-251ms across all 5 spans).

| Span | Xiao inav_ms | Blackbox time/1000 | Offset |
|------|-------------|-------------------|--------|
| 1 | 182,900 | 182,715 | ~185ms |
| 2 | 209,073 | 208,822 | ~251ms |
| 3 | 237,004 | 236,773 | ~231ms |
| 4 | 279,723 | 279,501 | ~222ms |
| 5 | 321,572 | 321,385 | ~187ms |

### Implications for Control Dynamics
The ~200ms MSP latency means the NN is operating on state data that's ~200ms old.
This is not simulated in crrcsim (sim delivers state instantly). This temporal delay
affects control responsiveness — the aircraft has moved during the 200ms between
INAV measuring state and the NN computing a response. At 16 m/s that's ~3.2m of
uncompensated motion. This may explain some of the tracking error in flight vs sim.

**Future work**: Add configurable MSP latency simulation to training (backlog item:
Simulator Sampling Time Variation).

## Test Spans

Five NN-controlled test passes identified in one flight:

| Span | Path | Duration | Origin NED |
|------|------|----------|------------|
| 1 | StraightAndLevel | 12.1s | [198.41, -18.73, -76.56] |
| 2 | StraightAndLevel | 6.0s | [188.96, 3.43, -63.30] |
| 3 | SpiralClimb | 7.0s | [173.03, 6.40, -68.27] |
| 4 | HorizontalFigureEight | 14.4s | [170.03, 27.18, -66.26] |
| 5 | StraightAndLevel | 6.0s | [176.27, 10.84, -63.15] |

### Identifying Spans
- **INAV blackbox**: `mspOverrideFlags` column transitions (0→1→3→2→0 cycle per span)
- **Xiao log**: `NN Control: Switch enabled` / `Nav Control: Switch disabled` bracket each span
- **Cross-check**: Both sources identify same 5 spans at correlated timestamps

## Sensor Pipeline Verification

### Position (X/Y)
- **Match**: mean error 0.000m / 0.008m, max 0.48m
- Xiao `pos_raw` matches INAV `navPos[0,1]` / 100 exactly (cm→m conversion)
- Pipeline: INAV navPos (NEU, cm) → MSP → xiao converts to NED (m)

### Position (Z)
- **Intentional negation**: `xiao_z = -inav_z`
- INAV `navPos[2]` is altitude-positive-up; xiao converts to NED Z-down
- Verified: `xiao_z + inav_z/100 ≈ 0.00` across all samples

### Quaternion
- **Correct conjugation**: INAV sends body→earth `(w,x,y,z)`, xiao conjugates to earth→body `(w,-x,-y,-z)`
- **Verified**: 19/20 samples match within 0.02 tolerance (1 mismatch during aggressive maneuvering = timing offset)
- Board alignment (`align_board_yaw`) is applied inside INAV before quaternion formation
- Flight hardware has `align_board_yaw=0` (no additional compensation)
- Bench hardware has `align_board_yaw=900, align_board_roll=1700` (IMU mounted differently)
- The 138° heading offset observed on bench was bench-specific, not a pipeline bug

### Velocity
- X/Y: mean error < 0.06 m/s, consistent with position
- Z: intentional negation (same as position Z)

## NN Output Analysis

Flight was "hilariously bad" — NN outputs oscillating wildly:
- Throttle: swinging -1.0 to +1.0 every few ticks
- Roll: flipping sign rapidly
- Pitch: large swings

### Root Cause Investigation (ongoing)
The sensor pipeline is verified correct. Possible causes:
1. **NN output → RC channel mapping**: sign or channel assignment mismatch between
   crrcsim's `inputdev_autoc.cpp` and xiao's `msplink.cpp` RC override
2. **INAV RC override behavior**: does INAV apply the override values directly to
   servos, or does it run them through its own PID/mixer? If PID is active, it
   fights the NN commands
3. **Cross-ISA FP divergence**: Cortex-M4F (float32 only) vs aarch64 (float64)
   could produce different NN outputs for same inputs (see T190/T191)
4. **Aircraft dynamics mismatch**: real aircraft has different response characteristics
   than crrcsim model — gain/timing differences could cause instability

## Renderer Usage

### Modes
- **Default**: training data from S3 (simulation results with path)
- **`-d <xiao_log>`**: xiao device log overlaid on training data (flight tape on sim path)
- **`-x <xiao_log>`**: xiao-only mode (flight tape only, no simulation data)

### Current Status
- Renderer updated to parse new log format (Nav State, NN Control, NN line)
- Flight tape rendering works (`-x` mode shows 5 test spans)
- **Missing**: rabbit path overlay and error bars (blue lines from aircraft to target).
  These were removed when `GP Input: rabbit=[x,y,z] vec=[x,y,z]` was consolidated
  into compact `NN:` line. Need to add rabbit position back to xiao log before next flight.

### Controls
- `t/r` — next/previous test span
- `a` — show all flight data

## Blackbox Decode Instructions

```bash
# Decode all flights in a blackbox file
~/blackbox-tools/obj/blackbox_decode <blackbox_file>.TXT

# Decode specific flight index to stdout
~/blackbox-tools/obj/blackbox_decode --index 0 --stdout <blackbox_file>.TXT

# Output files created next to source:
#   .01.csv      — main data (60Hz, ~100 columns)
#   .01.gps.csv  — GPS data
#   .01.gps.gpx  — GPX track for mapping
#   .01.event    — event markers (sync beep timestamp)
```

## TODO Before Next Flight

- [ ] Add rabbit world position back to xiao `NN:` log line (for renderer error bars)
- [ ] Investigate NN output→RC channel mapping (sign/assignment vs crrcsim)
- [ ] Investigate INAV RC override path (direct to servos vs through PID?)
- [ ] Run T190 cross-ISA NN output comparison (Cortex-M4F vs aarch64)
- [ ] Consider adding MSP latency simulation to training config

# Data Model: Flight Analysis

## Entities

### FlightSession

A single power-on-to-power-off recording from one flight outing.

**Attributes**:
- `date`: flight date (YYYY-MM-DD)
- `inav_blackbox_file`: path to raw INAV blackbox (.TXT)
- `inav_csv_file`: path to decoded CSV (.01.csv)
- `inav_gps_file`: path to GPS CSV (.01.gps.csv)
- `xiao_log_file`: path to xiao flight log
- `nn_weights_id`: S3 key of NN weights used (from NN Control log line)
- `test_spans`: list of TestSpan entities

**Relationships**: contains 1+ TestSpan, references 1 NNWeights

### TestSpan

An NN-active control segment within a FlightSession.

**Attributes**:
- `span_index`: sequential number within the session (1-based)
- `path_type`: path name (StraightAndLevel, SpiralClimb, HorizontalFigureEight, etc.)
- `path_index`: path selector index (0-5)
- `path_segments`: total path segments
- `origin_ned`: NED position at activation [x, y, z] meters
- `start_time_inav_ms`: INAV timestamp at activation
- `start_time_xiao_ms`: xiao timestamp at activation
- `duration_sec`: span duration
- `ticks`: list of CorrelatedTick entities

**Relationships**: belongs to 1 FlightSession, contains N CorrelatedTick

### CorrelatedTick

A single NN evaluation moment with matched data from both INAV and xiao.

**Attributes**:
- `tick_index`: sequential within span
- `inav_time_us`: INAV blackbox timestamp (microseconds since boot)
- `xiao_time_ms`: xiao millis() timestamp
- `xiao_inav_ms`: xiao's copy of INAV timestamp (column 3)

Sensor data (from xiao Nav State, cross-checked against INAV):
- `pos_raw`: raw NED position [x, y, z] meters (world frame)
- `pos`: relative NED position [x, y, z] meters (from origin)
- `vel`: NED velocity [vn, ve, vd] m/s
- `quat`: earth-to-body quaternion [w, x, y, z]

NN evaluation (from xiao NN line):
- `path_index`: current path segment index
- `nn_inputs[29]`: full NN input vector
- `nn_outputs[3]`: NN output [pitch, roll, throttle] in [-1, 1]
- `rc_commands[3]`: PWM values sent to INAV [1000-2000]

Direct position (from xiao NN line, after T222):
- `rabbit_pos`: rabbit world position [x, y, z] meters (relative to origin)

Projected position (computed by analysis tool):
- `rabbit_projected`: reconstructed rabbit position from dPhi/dTheta/dist/quat

INAV cross-reference (from blackbox CSV at matched timestamp):
- `inav_pos`: INAV navPos converted to meters
- `inav_vel`: INAV navVel converted to m/s
- `inav_quat`: INAV quaternion (raw, not conjugated)
- `inav_attitude`: INAV roll/pitch/heading in degrees
- `inav_rc`: INAV rcData (RC override values received)
- `inav_servo`: INAV servo outputs
- `inav_msp_override`: mspOverrideFlags value

Deltas (computed):
- `delta_pos`: xiao_pos_raw - inav_pos (pipeline error)
- `delta_vel`: xiao_vel - inav_vel
- `delta_quat`: xiao_quat - inav_quat (after conjugation)
- `delta_rabbit`: rabbit_pos - rabbit_projected (convention mismatch indicator)

### AircraftModel

Parameters from crrcsim's FDM model (hb1.xml), calibrated from flight data.

**Attributes**:
- `model_file`: path to hb1.xml
- `pitch_rate_gain`: deg/s per unit command (measured from flight)
- `roll_rate_gain`: deg/s per unit command
- `throttle_speed_gain`: m/s² per unit throttle
- `transport_delay_ms`: command-to-response delay
- `max_climb_rate`: m/s at full throttle
- `drag_decel_rate`: m/s² at idle throttle

**Relationships**: calibrated from N FlightSession measurements

## State Transitions

### TestSpan lifecycle (from mspOverrideFlags):

```
PILOT_CONTROL (flags=0)
  → servo switch on (flags=1)
  → path armed + NN enabled (flags=3)
  → path complete or servo switch off (flags=2)
  → autoc disabled (flags=0)
  → servo reset (flags=0, re-arm allowed)
```

### Analysis pipeline:

```
Raw Data → Decode → Join → Validate → Analyze → Calibrate
  ↓           ↓        ↓       ↓          ↓          ↓
.TXT       .csv    joined   deltas    charts     hb1.xml
.txt                timeline  flags    dynamics   updated
```

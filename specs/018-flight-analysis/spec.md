# Feature Specification: Flight Analysis & Sim-to-Real Verification

**Feature Branch**: `018-flight-analysis`
**Created**: 2026-03-20
**Status**: Draft
**Input**: First NN flight test (2026-03-20) completed — pipeline works end-to-end but flight behavior was poor. Need tooling to diagnose coordinate conventions, sensor pipeline correctness, temporal dynamics, and aircraft model calibration.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Sensor Pipeline Proof (Priority: P1)

An engineer runs a join tool on INAV blackbox data and xiao flight logs. For each NN
evaluation tick, the tool shows the INAV sensor values alongside the xiao-received values
and flags any discrepancies in position, velocity, quaternion, or timing. The engineer
can confirm that the NN is receiving correct sensor data from the flight controller, or
identify exactly where data is corrupted or misinterpreted.

**Why this priority**: Garbage in = garbage out. If the sensor pipeline is wrong, nothing
downstream (NN training, coordinate conventions, model calibration) matters. This is the
foundation for all other analysis.

**Independent Test**: Run the join tool on the 2026-03-20 flight data. Position X/Y should
match within 0.01m (already verified). Z should show intentional NEU→NED negation.
Quaternion should match after conjugation. Timestamps should correlate within 200ms MSP latency.

**Acceptance Scenarios**:

1. **Given** a decoded INAV blackbox CSV and a xiao flight log from the same flight,
   **When** the join tool runs, **Then** it produces a correlated timeline with per-tick
   sensor comparison and delta columns for position, velocity, and quaternion.

2. **Given** correlated data during an NN-active test span, **When** examining position
   deltas, **Then** X/Y error < 0.5m and Z shows consistent negation (NED convention).

3. **Given** correlated data, **When** examining timestamps, **Then** xiao INAV-timestamp
   column correlates with blackbox time within ±250ms with stable offset across all spans.

**Documentation output**: `~/autoc/docs/sensor-pipeline.md` describing the full chain with
coordinate conventions, unit conversions, sign flips, and timing model.

---

### User Story 2 - Rabbit Position Visualization (Priority: P1)

An engineer loads flight data in the renderer and sees both where the rabbit actually was
(direct position) and where the NN thought the rabbit was (projected from body-frame NN
inputs). If these don't align, there is a coordinate convention mismatch. If they do
align but the aircraft doesn't follow, the problem is in the output mapping or dynamics.

**Why this priority**: The core diagnostic for "why did the flight go wrong?" Without this,
we can't distinguish sensor bugs from output bugs from dynamics mismatches.

**Independent Test**: First validate projection on sim data (data.dat) — projected and actual
rabbit paths MUST overlay perfectly since the same code computed both. Then apply to flight data.

**Acceptance Scenarios**:

1. **Given** a sim eval run (e.g. BIG-3 tier0-repro), **When** the renderer shows
   projected rabbit path alongside actual rabbit path, **Then** they overlay within
   1m across the entire flight (numerical precision only).

2. **Given** updated xiao firmware with direct rabbit logging, **When** the renderer loads
   a flight log, **Then** the red rabbit path (direct) matches the expected path geometry
   (StraightAndLevel = racetrack, SpiralClimb = helix, etc.).

3. **Given** both direct and projected rabbit paths rendered on flight data, **When** comparing
   them, **Then** any misalignment reveals the specific convention mismatch (axis flip,
   sign error, frame rotation).

---

### User Story 3 - Output Mapping Verification (Priority: P2)

An engineer examines correlated flight data to verify that NN commands produce the expected
aircraft response: pitch-up command → aircraft pitches up, right-roll command → aircraft
rolls right, throttle-up → aircraft accelerates. If any axis is inverted or channels are
swapped, this analysis identifies exactly where.

**Why this priority**: Even if sensor inputs are correct, inverted outputs make the aircraft
do the opposite of the trained policy. Must verify after sensor pipeline (P1) is confirmed.

**Independent Test**: From correlated flight data, plot NN command vs aircraft attitude rate
response for each axis. The sign should be consistent — positive command should consistently
produce positive response (with transport delay).

**Acceptance Scenarios**:

1. **Given** correlated NN output and INAV attitude data during a test span, **When** the
   NN commands a pitch change, **Then** the aircraft attitude rate changes in the same
   direction within 300ms (accounting for MSP transport + servo lag).

2. **Given** the RC override path in INAV, **When** MSP RC commands are sent, **Then**
   they reach servos without being modified by INAV's PID controller (direct passthrough).

---

### User Story 4 - Aircraft Dynamics Identification (Priority: P2)

An engineer analyzes flight data to extract the real aircraft's control rates, power
available, drag, and delay characteristics. These measurements feed back into the crrcsim
aircraft model (hb1.xml) to calibrate the simulator, reducing the sim-to-real gap and
establishing the design center for aircraft variation training.

**Why this priority**: Depends on Stories 1-3 being resolved (can't measure dynamics if
sensors or outputs are wrong). The calibrated model enables sim-to-real transfer learning.

**Independent Test**: Measure pitch rate gain from flight data. Compare to crrcsim's
prediction for the same command sequence. Quantify the gap.

**Acceptance Scenarios**:

1. **Given** corrected flight data with known-good sensor and output mapping, **When** analyzing
   pitch/roll/throttle command-response pairs, **Then** extract: transport delay (ms),
   rate gain (deg/s per unit command), bandwidth (Hz), and damping ratio for each axis.

2. **Given** measured flight parameters, **When** updating hb1.xml and re-running the eval
   suite, **Then** fitness does not degrade more than 50% from baseline (indicating the
   model update is reasonable, not catastrophic).

---

### User Story 5 - AHRS & Sensor Quality Assessment (Priority: P2)

An engineer produces standard charts showing heading consistency, attitude drift, position
noise, and velocity vector alignment from flight data. These determine whether INAV's
AHRS is reliable enough for NN-based control or if sensor quality is a limiting factor.

**Why this priority**: Lower priority because sensor quality is unlikely to be the primary
issue — INAV's AHRS is well-validated for conventional flight. But for NN control with
body-frame inputs, even small biases may matter.

**Independent Test**: During stationary pre-arm period, quaternion drift should be < 0.5 deg/min.
GPS CEP should be < 3m. Heading from quaternion should match GPS ground track within 10° in
straight flight.

**Acceptance Scenarios**:

1. **Given** flight data with pre-arm stationary period, **When** measuring attitude drift,
   **Then** produce drift rate in deg/min for roll, pitch, yaw with confidence intervals.

2. **Given** straight-and-level flight segments, **When** comparing AHRS heading to GPS
   ground track, **Then** systematic offset < 15° (correctable) or random noise < 5° RMS.

---

### User Story 6 - Temporal Jitter Characterization (Priority: P2)

An engineer measures the actual MSP update interval, NN eval latency, and end-to-end
pipeline delay from flight data. If jitter is significant (>15% of 100ms cycle), the
simulator needs timing noise injection for training robustness.

**Why this priority**: Training assumes perfect 100ms steps. If reality is 80-120ms with
occasional dropouts, the NN may be unstable. But this is a refinement concern — coordinate
conventions (P1) are the blocking issue.

**Independent Test**: From xiao log timestamps, compute MSP interval statistics. Expected:
mean ~100ms, std dev < 10ms.

**Acceptance Scenarios**:

1. **Given** xiao flight log timestamps during NN-active spans, **When** computing
   inter-sample intervals, **Then** produce: mean, std dev, min, max, histogram,
   and count of dropped/delayed samples.

2. **Given** measured jitter > 15% of nominal interval, **When** adding `SimTickJitter`
   config parameter, **Then** training produces an NN that handles variable intervals
   without fitness degradation > 20%.

---

### Edge Cases

- What happens when GPS drops out during NN-active control? (position freezes, NN sees stale data)
- How does the system handle MSP communication loss? (xiao should detect and disable autoc)
- What if INAV's quaternion has gimbal lock singularity? (NN uses quaternion directly, no Euler conversion — should be immune)
- What happens when the aircraft enters extreme attitudes (>60° bank, >30° pitch)? The NN was trained with moderate attitudes — extreme attitudes may produce garbage inputs.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a tool that joins INAV blackbox CSV with xiao flight log by timestamp, producing per-tick correlated sensor data.
- **FR-002**: System MUST log rabbit world position directly in xiao NN telemetry for unambiguous flight analysis.
- **FR-003**: Renderer MUST parse current log format (Nav State, NN Control, NN line) and render flight tape with rabbit path overlay.
- **FR-004**: System MUST be able to project NN body-frame inputs (dPhi, dTheta, dist, quat) back to world coordinates for visualization.
- **FR-005**: Projection function MUST produce correct results on simulation data (data.dat) before being applied to flight data.
- **FR-006**: System MUST document the complete sensor pipeline with coordinate conventions, unit conversions, and timing model.
- **FR-007**: System MUST provide standard flight analysis charts (heading, attitude, position, velocity, timing) for each flight session.
- **FR-008**: System MUST support replaying NN inputs from flight logs through the desktop NN to detect cross-ISA FP divergence.

### Key Entities

- **Flight Session**: One power-on-to-power-off cycle containing multiple test spans. Identified by INAV blackbox file + xiao flight log file.
- **Test Span**: An NN-active control segment within a flight session. Identified by mspOverrideFlags transitions in INAV and NN Control lines in xiao.
- **Correlated Tick**: A single NN evaluation moment with matched INAV and xiao sensor data, NN inputs/outputs, and RC commands.
- **Aircraft Model (hb1.xml)**: crrcsim's flight dynamics model, calibrated from flight data measurements.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Sensor pipeline join tool matches 99%+ of xiao Nav State ticks to INAV blackbox rows within ±50ms.
- **SC-002**: Position correlation error < 0.5m for X/Y, Z shows consistent NED negation with < 0.5m residual.
- **SC-003**: Projected rabbit path on sim data overlays actual rabbit path within 1m across 100% of ticks.
- **SC-004**: Next flight test shows visible tracking intent — aircraft turns toward path, follows general direction, not random flailing. Qualitative improvement from first flight. Rough proxy: at least one test span where aircraft maintains path-relative heading for >50% of the span duration.
- **SC-005**: Measured aircraft dynamics (pitch rate, roll rate, throttle response) documented with ±20% confidence intervals.
- **SC-006**: Updated crrcsim model (hb1.xml) passes BIG-3 eval suite with fitness within 2× of original baseline.

## Clarifications

### Session 2026-03-21

- Q: What does "completion > 50%" mean for SC-004? → A: Qualitative — visible tracking intent, aircraft turns toward path and follows general direction. "Less hilarious" than first flight. Not a rigorous comp= metric at this stage.

## Assumptions

- INAV blackbox format is stable (v8, decoded by blackbox-tools at ~/blackbox-tools)
- xiao flight log format is stable (current Nav State + NN line format)
- The ~200ms MSP transport latency is assumed to be a roughly constant offset — US6 (T270-T272) will verify whether this holds or if jitter is significant
- crrcsim's FDM is capable of representing the real aircraft's dynamics with sufficient fidelity after parameter tuning
- The DGX Spark (GB10) is the primary development and analysis platform

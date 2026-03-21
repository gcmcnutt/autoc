# 018: Flight Analysis & Coordinate Convention Verification

## Motivation

First NN flight test (2026-03-20) demonstrated the full pipeline works end-to-end
(sensor data flows correctly, NN evaluates, RC commands are sent) but flight behavior
was poor. Root cause investigation requires tooling to understand what the NN sees
vs what it should see, correlate with INAV blackbox data, and verify coordinate
conventions match between sim (crrcsim) and real hardware (INAV + xiao).

The core problem: **if the NN's perception of the world doesn't match the sim's
perception, the trained policy produces wrong commands.** This feature builds the
tools to diagnose and fix that.

## Requirements

### R1: Flight Log Parsing & Correlation
- Parse xiao flight logs (Nav State, NN I/O, control spans)
- Parse INAV blackbox CSV (decoded via `blackbox_decode`)
- Temporal correlation via shared INAV timestamp (~200ms MSP latency measured)
- Identify test spans from both data sources independently, cross-validate
- Produce joined timeline: for each NN eval tick, show INAV state + xiao state + NN I/O

### R2: Rabbit Position Reconstruction
- **Direct logging**: Add rabbit world position to xiao `NN:` log line for next flight
- **Inverse projection**: Reconstruct rabbit position from NN inputs (dPhi, dTheta, dist, quat)
  - Current approach (two independent atan2 inversions) is numerically unstable
  - Need a solver that handles front/back ambiguity in the atan2 projections
  - Or use a different projection approach (e.g. iterative least-squares on the forward model)
- **Validation**: reconstructed rabbit path from flight data should match the known
  path geometry (StraightAndLevel = racetrack, SpiralClimb = helix, etc.)
- **Sim validation**: same reconstruction applied to `data.dat` from desktop sim should
  produce a clean path — if it doesn't, the reconstruction math is wrong

### R3: Coordinate Convention Verification
- **The central question**: does the NN see the same world in flight as it does in sim?
- Compare: crrcsim's `inputdev_autoc.cpp` coordinate transforms vs xiao's `msplink.cpp`
- Verify axis conventions: NED vs NEU, body frame orientation, sign of Z, quat convention
- Verify output mapping: NN output [-1,1] → RC PWM [1000,2000] → INAV servo/motor
  - Channel assignment (which output = pitch, roll, throttle)
  - Sign conventions (positive pitch = nose up or down?)
  - INAV RC override path: direct to servos or through PID/mixer?
- Cross-ISA FP divergence (T190/T191 from 015): Cortex-M4F vs aarch64 NN output comparison

### R4: Flight Dynamics Analysis
- From correlated flight data, infer actual aircraft response to NN commands
- Compare to crrcsim model: control surface deflection → attitude rate response
- Identify gain/timing mismatches between sim and real
- Feed into aircraft variations feature (Phase 6 from 015): what parameters need
  variation in training to cover real-world dynamics?
- Characterize MSP temporal delay impact on control loop stability

### R5: Renderer Updates
- Parse new log format (Nav State, NN Control, NN line) — DONE in 015
- Render reconstructed rabbit path (red) and error vectors (blue) from NN inputs
- Support overlay of known path geometry on flight data (`-d` mode with path)
- Display NN input/output values in HUD for debugging

## Carried Forward from 015

### Open Tasks
- T124c: Document/assert genome.fitness is always aggregateRawFitness
- T125: Store per-scenario lexicase scores in data.stc
- T130-T133: Path-relative smoothness (on hold)
- T140-T143: Aircraft parameter variation (depends on R4 findings)
- T155: Renderer arena layout for multi-path
- T161: Full test suite run
- T163: Remove SinglePathProvider
- T164: GP legacy cleanup
- T165: Remove 'sum' selection mode
- T166: Memory leak investigation
- T190: Cross-ISA NN output comparison (pre-flight)
- T191: Flight log replay comparison (post-flight)

### Eval Suite Backlog
- Config stacking (`-i base -i overlay`)
- Archive data.dat/data.stc with S3 eval uploads
- `nnextractor` by S3 key/timestamp
- Eval metadata linked to S3 keys

## Approach

### Phase 1: Sim-side validation
- Apply rabbit reconstruction to `data.dat` from a known good sim run
- Verify the reconstructed path matches the actual path used in training
- If reconstruction fails on sim data, the math is wrong (not a flight issue)
- This isolates reconstruction bugs from coordinate convention bugs

### Phase 2: Direct rabbit logging
- Add `rabbit=[x,y,z]` to xiao `NN:` log line (world-relative position)
- Rebuild xiao firmware, bench test, verify renderer shows clean path overlay
- This is the ground truth for flight analysis — no reconstruction ambiguity

### Phase 3: Coordinate convention audit
- Systematic comparison of every transform in the pipeline:
  - crrcsim FDM → crrcsim inputdev_autoc → autoc NN inputs
  - INAV sensors → MSP → xiao msplink → aircraft_state → NN inputs
- Document each transform with explicit axis conventions
- Identify any sign flips, axis swaps, or frame mismatches

### Phase 4: Flight dynamics characterization
- From corrected flight data, extract transfer functions (command → response)
- Compare to crrcsim model parameters
- Define aircraft variation ranges for sim-to-real training

## Dependencies

- 015 (NN training): closed — BIG-3 weights (fitness 2,724) are the baseline
- 017 (visual target tracking): independent — can proceed in parallel
- crrcsim: may need modifications for coordinate convention fixes
- xiao firmware: needs rabbit logging addition

## Technologies

- C++17, Eigen (renderer, analysis tools)
- Python (correlation scripts, `verify_flight_log.py` as template)
- blackbox-tools (INAV log decode)
- VTK (renderer visualization)

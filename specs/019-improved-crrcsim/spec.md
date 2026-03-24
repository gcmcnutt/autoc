# 019: Improved CRRCSim Fidelity

## Problem

BIG3 training (018, 400 generations) converged to a degenerate controller: full pitch back
+ full throttle on 100% of timesteps by generation 10. The NN exploits sim physics that
reward looping/porpoising — a strategy that's survivable in CRRCSim but would instantly
crash the real aircraft.

Root cause: the sim doesn't match reality well enough. Flight data from 2026-03-22 shows:

| Axis | Flight rate | Sim rate | Ratio |
|------|------------|----------|-------|
| Roll | 0.42-0.54 °/s/unit | 0.21-0.24 | **2.0-2.3×** |
| Pitch | 0.24-0.26 °/s/unit | 0.03-0.05 | **5-7×** |

The NN trains on a sim where full pitch barely moves the aircraft, so it learns to command
full pitch always. On the real craft, that's an instant departure.

Additionally, the MSP pipeline latency (49ms measured) vs sim latency (40ms) should be
aligned, and reducing real pipeline latency via a custom MSP command would improve control
bandwidth.

## Critical Outcomes

### CO1: Base model that doesn't reward degenerate strategies
Full pitch + full throttle in CRRCSim must produce a stall/departure, not survivable
looping. Step function inputs should produce responses comparable to the real aircraft
within 2× on each axis.

### CO2: Pitch rate gain within 2× of reality
Current 5-7× mismatch is the primary gap. After tuning hb1_streamer.xml, sim pitch rate
per unit command must be within 0.5-2.0× of flight-measured rates across the speed envelope.

### CO3: Roll rate gain confirmed or adjusted
Currently ~2× off. Verify with proper elevon de-mixing (requires T273i servo logging fix).
Adjust Cl_da if needed.

### CO4: Pipeline latency aligned
COMPUTE_LATENCY in CRRCSim updated to match measured real pipeline latency. Currently
40ms sim vs 49ms real. Either update sim or reduce real latency.

### CO5: Retrain produces non-degenerate controller
BIG training with calibrated model produces a controller that modulates pitch/roll/throttle
across scenarios. Pitch output should NOT be saturated at ±1.0 for >50% of timesteps.

### CO6: Custom MSP command reduces pipeline latency
Single MSP2_AUTOC_STATE command replaces 3 sequential requests. Target: fetch time
35ms → 12ms, total pipeline 49ms → 27ms.

### CO7: Flight mode override from xiao
T273c: xiao sends flight mode channel via MSP override to force MANUAL mode, removing
dependency on pilot transmitter switch.

## Approach

### Phase 1: Measure & characterize (before touching hb1.xml)
- Step function testing in CRRCSim: command full pitch, full roll, full throttle steps
  from level flight, observe response. Does the aircraft depart? How quickly?
- Same step functions on real aircraft (bench with blackbox, or deduce from flight data)
- De-mix elevons from blackbox: servo[0,1] → pitch/roll deflection
- Compute per-axis rate gain at multiple airspeeds
- Side-by-side comparison: sim vs flight rate gain ratios

### Phase 2: Tune hb1_streamer.xml
- Adjust aero coefficients to match measured rates:
  - Cm_de (pitch effectiveness) — primary gap
  - Cm_q (pitch damping, models streamer drag)
  - Cl_da (roll effectiveness) — verify, may be close
  - Thrust curve, drag (CD0) — check speed envelope
- Validate step functions produce realistic response
- Replay actual flight NN commands through sim, compare trajectory segments

### Phase 3: Pipeline improvements
- T273h: Custom MSP2_AUTOC_STATE in INAV (autoc branch) + xiao parser
- T273c: Flight mode channel override
- T273i: Cherry-pick INAV servo logging fix
- T273b: Update COMPUTE_LATENCY to match real pipeline

### Phase 4: Retrain & validate
- BIG training with calibrated model
- Verify non-degenerate behavior (pitch/throttle not saturated)
- Eval suite: tier1 pass rate, average distance, control modulation
- Compare eval trajectories to flight data qualitatively

## Validation approaches (if standard tuning is insufficient)

- **Custom step controller**: Build a test controller that injects known step inputs
  (e.g., full pitch for 500ms then neutral) into CRRCSim, logs the response. Compare
  to same inputs on real aircraft via blackbox.
- **Flight replay**: Take actual NN commands from flight blackbox, play them through
  CRRCSim from the same initial condition. Compare attitude/position over 1-2s segments.
  Divergence quantifies the sim-to-real gap directly.
- **Frequency sweep**: Sinusoidal inputs at various frequencies to characterize bandwidth
  and phase lag differences between sim and real.

## Known constraints
- No ground truth video available yet (camera is a future addition)
- AHRS is plausible but not precisely validated (manual recovery analysis shows ~20° accuracy)
- Streamer adds significant variable drag — model as Cm_q variation, not fixed value
- INAV servo logging bug means only one elevon is logged — T273i needed for proper de-mixing

## Prior art from 018
- [018 tasks.md](../018-flight-analysis/tasks.md) — full task list with disposition
- [018 sensor-pipeline.md](../../docs/sensor-pipeline.md) — complete pipeline documentation
- [018 notes](../018-flight-analysis/tasks.md#notes) — dynamics mismatch data, CRRCSim chain audit
- BIG3 training data: `20260323-data.dat`, `20260323-data.stc`, `logs/autoc-018-BIG3.log`

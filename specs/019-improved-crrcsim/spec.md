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

### CO1: Throttle response approximates real aircraft
From a trimmed state, throttle step inputs in CRRCSim produce speed/climb changes
comparable to flight hardware. Establishes the energy baseline — everything else
(roll authority, pitch authority) depends on being at roughly the right airspeed.
Sources: training run data (early gens before saturation), flight blackbox, or
intentional step function tests in sim.

### CO2: Roll rate gain within 2× of reality
Currently ~2× off. Verify with proper elevon de-mixing (requires T273i servo logging
fix). Adjust Cl_da if needed. Roll is partially independent of pitch — can tune from
wings-level step inputs at known airspeed. Sources: training run data where roll varies,
flight blackbox roll transients.

### CO3: Pitch rate gain within 2× of reality
Current 5-7× mismatch is the primary gap. After tuning hb1_streamer.xml, sim pitch rate
per unit command must be within 0.5-2.0× of flight-measured rates across the speed
envelope. Pitch depends on airspeed (CO1) and couples with roll at higher angles.
Sources: same as CO1/CO2 — scrape from training data or intentional steps.

### CO4: Pipeline latency aligned
COMPUTE_LATENCY in CRRCSim updated to match measured real pipeline latency. Currently
50ms sim vs 49ms real. Custom MSP2 command (CO5) may reduce real latency — update sim
to match. Do this BEFORE training runs so the NN trains against the correct delay.

### CO5: Custom MSP command reduces pipeline latency
Single MSP2_AUTOC_STATE command replaces 3 sequential requests. Target: fetch time
35ms → 12ms, total pipeline 49ms → 27ms. Requires INAV firmware change (autoc branch)
+ xiao parser update. Do before training to lock in the latency the NN trains against.

### CO6: Flight mode override from xiao
T273c: xiao sends flight mode channel via MSP override to force MANUAL mode, removing
dependency on pilot transmitter switch. Can be done in parallel with sim tuning.

### CO7: Non-degenerate training
BIG training with calibrated model produces a controller that modulates pitch/roll/throttle
across scenarios. Pitch output should NOT be saturated at ±1.0 for >50% of timesteps.
This is the validation gate — only achievable after CO1-CO4 are in place.

## Approach

### Phase 1: Pipeline latency (CO4, CO5, CO6)
Lock in the real pipeline latency before tuning aero. Training must use the correct
delay from the start.
- T273h: Custom MSP2_AUTOC_STATE in INAV (autoc branch) + xiao parser
- T273c: Flight mode channel override from xiao
- T273i: Cherry-pick INAV servo logging fix (enables elevon de-mixing for Phase 2-3)
- Bench test: measure new pipeline latency
- T273b: Update COMPUTE_LATENCY in CRRCSim to match measured value

### Phase 2: Throttle tuning (CO1)
Get the energy/speed envelope right first. Everything else depends on airspeed.
- Characterize real aircraft: throttle → speed and throttle → climb rate from flight
  blackbox (straight-ish segments, wings level)
- Same characterization in CRRCSim: step throttle from trim, measure speed/climb response
- Tune thrust curve, CD0 (parasitic drag), streamer drag contribution
- May scrape usable data from early-gen training runs before NN saturates, or build
  a step-function test controller in CRRCSim
- Validate: sim speed envelope matches flight (Vmin, Vcruise, Vmax, climb rate)

### Phase 3: Roll tuning (CO2)
Roll is relatively independent at small angles.
- Characterize: roll rate per unit command at multiple airspeeds (flight blackbox,
  de-mixed elevons from T273i)
- Same in CRRCSim: roll step from wings-level, measure rate
- Adjust Cl_da if needed (018 estimate: sim ~2× slow, Cl_da may already be close)
- Validate: roll step response within 2× of flight

### Phase 4: Pitch tuning (CO3)
Pitch is the critical axis — 5-7× mismatch caused the degenerate training.
- Characterize: pitch rate per unit command at multiple airspeeds (flight blackbox)
- Same in CRRCSim: pitch step from trim, measure rate and stall behavior
- Adjust Cm_de (effectiveness) and Cm_q (damping, streamer model)
- Key test: full pitch + full throttle from level flight. Must stall/depart, not loop.
- Validate: pitch step response within 2× of flight

### Phase 5: Integration validation
- Replay actual flight NN commands through CRRCSim from matched initial conditions.
  Compare attitude/position over 1-2s segments. Divergence quantifies remaining gap.
- If needed: intentional step function tests — build a test controller that injects
  known inputs, logs response for direct comparison.

### Phase 6: Retrain & validate (CO7)
- BIG training with calibrated model + correct pipeline latency
- Verify non-degenerate behavior (pitch/throttle not saturated)
- Eval suite: tier1 pass rate, average distance, control modulation
- Compare eval trajectories to flight data qualitatively

## Data sources for characterization

Getting reference data for tuning does NOT require dedicated step-function flights:
- **Training runs**: early generations (before NN saturates) contain varied control
  inputs across many scenarios. Scrape pitch/roll/throttle transients from data.dat
  where one axis changes while others are ~constant.
- **Flight blackbox**: pilot recovery segments after MSP disable have clear step-like
  inputs. NN-controlled segments have mixed inputs but some clean transients.
- **Intentional sim steps**: if the above are insufficient, build a simple test
  controller that commands step inputs in CRRCSim and logs the response.
- **Flight replay**: play recorded NN commands from blackbox into CRRCSim from the
  same initial state. Compare 1-2s trajectory segments directly.

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

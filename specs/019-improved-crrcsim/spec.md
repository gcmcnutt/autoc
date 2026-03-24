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

The core loop: extract response curves from real flight data (INAV blackbox), extract
the same from CRRCSim (data.dat from training runs or intentional step tests), compare
side-by-side, adjust hb1_streamer.xml, repeat until they look similar. Available data
first — only build custom step controllers if we can't scrape enough clean transients.

### Phase 1: Throttle characterization (CO1)
Get the energy/speed envelope right first. Roll and pitch authority depend on airspeed.
- Extract from flight blackbox: throttle → speed and throttle → climb rate during
  straight-ish wings-level segments. Pilot recovery segments have clear throttle
  changes. NN-controlled segments may also have usable transients.
- Extract from data.dat: same metrics from early-gen training data (gen 1-5 before
  NN saturates). Find timesteps where throttle changes while pitch/roll are ~constant.
- Compare response curves side-by-side. Tune thrust curve, CD0, streamer drag.
- If insufficient clean transients: build a sim step-function test (step throttle
  from trim, log speed/climb response).
- Validate: sim speed envelope matches flight (Vmin, Vcruise, Vmax, climb rate)

### Phase 2: Roll characterization (CO2)
Roll is relatively independent at small angles.
- Extract from flight blackbox: roll rate per unit command at multiple airspeeds.
  Needs elevon de-mixing (T273i servo fix, or estimate from single elevon + symmetry).
- Extract from data.dat: roll transients where pitch/throttle are ~steady.
- Compare response curves. Adjust Cl_da if needed (018 estimate: sim ~2× slow).
- Validate: roll step response within 2× of flight

### Phase 3: Pitch characterization (CO3)
The critical axis — 5-7× mismatch caused the degenerate training.
- Extract from flight blackbox: pitch rate per unit command at multiple airspeeds.
- Extract from data.dat: pitch transients. Harder to isolate since NN saturates
  early — gen 1 data may be the best source, or build sim step tests.
- Compare response curves. Adjust Cm_de (effectiveness) and Cm_q (damping/streamer).
- Key test: full pitch + full throttle from level flight. Must stall/depart, not loop.
- Validate: pitch step response within 2× of flight

### Phase 4: Integration & iteration
- Overlay sim vs flight response curves across all axes. Iterate tuning where
  interactions between axes cause secondary mismatches (e.g., pitch authority
  changes with speed, which changed in Phase 1).
- Replay actual flight NN commands through CRRCSim from matched initial conditions.
  Compare attitude/position over 1-2s segments. Divergence quantifies remaining gap.
- If needed: build custom step controller for targeted experiments at the limits.

### Phase 5: Pipeline latency + retrain (CO4, CO5, CO6, CO7)
Once sim aero matches reality, lock in the correct pipeline latency and train.
- T273h: Custom MSP2_AUTOC_STATE in INAV (autoc branch) + xiao parser
- T273c: Flight mode channel override from xiao
- T273i: Cherry-pick INAV servo logging fix (if not done earlier for de-mixing)
- Bench test: measure new pipeline latency
- T273b: Update COMPUTE_LATENCY in CRRCSim to match
- BIG training with calibrated model + correct latency
- Verify non-degenerate behavior (pitch/throttle not saturated >50% of steps)
- Compare data.dat response curves to flight data — should look similar now

## Data sources for characterization

Getting reference data for tuning does NOT require dedicated step-function flights:
- **data.dat (training runs)**: early generations (before NN saturates) contain varied
  control inputs across many scenarios. Scrape pitch/roll/throttle transients where
  one axis changes while others are ~constant. This is the primary sim-side source.
- **Flight blackbox**: pilot recovery segments after MSP disable have clear step-like
  inputs. NN-controlled segments have mixed inputs but some clean transients.
- **Intentional sim steps**: if the above are insufficient, build a simple test
  controller that commands step inputs in CRRCSim and logs the response. Only if
  we can't get clean enough transients from existing data.
- **Flight replay**: play recorded NN commands from blackbox into CRRCSim from the
  same initial state. Compare 1-2s trajectory segments. Best for integration validation.

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

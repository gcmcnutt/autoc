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

## Repeatable tuning workflow

The same loop applies to each axis and will repeat when we build variant aircraft:

1. **Extract real response curve** from flight blackbox (2026-03-22 data has climb,
   coast, recovery, NN control with full throttle in various attitudes)
2. **Extract sim response curve** from data.dat (BIG3 20260323 data has 400 gens,
   early gens have varied inputs before NN saturates; later gens useful for steady-state)
3. **Compare side-by-side** (ascii art / simple plots). Identify where nonlinearities
   diverge — e.g., streamer drag at high speed, stall behavior, thrust saturation.
4. **Adjust hb1_streamer.xml** parameters for the axis under test.
5. **Run a mini training** (10-20 gens, enough to get varied inputs) to regenerate
   sim response curves with the updated model.
6. **Re-compare**. Iterate until curves look similar across the operating envelope.

This workflow is designed to be repeatable: when we build a second craft within 5-10%
of the current one, we repeat the same extract→compare→tune loop against its flight
data. The tooling built here becomes the variation characterization pipeline.

## Approach

### Phase 1: Throttle characterization (CO1)
Get the energy/speed envelope right first. Roll and pitch authority depend on airspeed.

**Real (flight blackbox):**
- Throttle → speed: straight-ish wings-level segments, pilot recovery climbs (full
  throttle + level), NN segments with varied throttle at different attitudes
- Throttle → climb rate: altitude rate vs throttle command, binned by attitude
- Speed envelope: Vmin (near stall), Vcruise (50% throttle), Vmax (full throttle level)
- Streamer drag signature: speed at full throttle should cap around 17 m/s

**Sim (data.dat):**
- Same metrics from BIG3 data. Gen 1-5 have varied throttle before saturation.
  Later gens (full throttle always) useful for steady-state Vmax and climb rate.
- Nonlinearities to check: does sim speed saturate at the same Vmax? Does climb
  rate at full throttle match? Does idle-throttle descent rate match coast segments?

**Tuning targets:** thrust curve, CD0 (parasitic drag), streamer drag contribution.
Validate: sim speed envelope matches flight within ~15%.

### Phase 2: Roll characterization (CO2)
Roll is relatively independent at small angles.

**Real (flight blackbox):**
- Roll rate per unit command at multiple airspeeds. Estimate elevon deflection from
  single logged elevon + symmetry assumption (T273i fix enables proper de-mixing later).
- Roll transients during NN control and pilot recovery segments.

**Sim (data.dat):**
- Roll transients where pitch/throttle are ~steady. BIG3 roll output varies (mean
  -0.04 with 8.7% right, 16.2% left in last gen — earlier gens have more variation).

**Tuning targets:** Cl_da (roll effectiveness). 018 estimate: sim ~2× slow.
Validate: roll rate per unit command within 2× of flight across speed range.

### Phase 3: Pitch characterization (CO3)
The critical axis — 5-7× mismatch caused the degenerate training.

**Real (flight blackbox):**
- Pitch rate per unit command at multiple airspeeds. Pilot recovery pulls are
  clean step-like inputs (seen in today's analysis: sustained -400 to -480 rcPitch).
- NN segments have full pitch always — useful for steady-state but not transients.

**Sim (data.dat):**
- Gen 1 has pitch mean=0.83 (not yet saturated) — best source for transients.
  By gen 5 it's already at 0.97. May need sim step tests for clean data.
- Key nonlinearity: does full pitch + full throttle stall/depart or loop?

**Tuning targets:** Cm_de (effectiveness), Cm_q (damping, streamer model).
Validate: pitch rate per unit command within 2× of flight. Full pitch must not
produce survivable looping.

### Phase 4: Integration & iteration
- Overlay all three axis response curves (sim vs flight). Iterate where cross-axis
  coupling causes secondary mismatches (pitch authority changes with speed from
  Phase 1 tuning, roll couples to yaw at higher angles, etc).
- Replay actual flight NN commands through CRRCSim from matched initial conditions.
  Compare attitude/position over 1-2s segments.
- If needed: build custom step controller for targeted experiments at the limits.
- Mini training runs (10-20 gens) to validate tuning hasn't broken other axes.

### Phase 5: Sensor scaling audit (pre-flight gate)
Before committing to a training run, verify the full state→sensor→NN input chain
has no unit mismatches. This was broadly confirmed in 018 but worth a final check
with the updated sim model.
- CRRCSim FDM units: ft/s, radians, slugs — verify all conversions to SI at the
  autoc interface boundary (position m, velocity m/s, attitude quaternion, rates rad/s)
- NN input scaling: confirm dPhi/dTheta/dist inputs match between sim and flight
  at known states (e.g., 10m offset at 45° should produce the same NN input values
  in both CRRCSim and INAV blackbox)
- Accelerometer/gyro scaling: if used as NN inputs, confirm units match
- Cross-check: pick a few timesteps from flight blackbox and from sim data.dat at
  similar physical states. NN input vectors should be comparable.

### Phase 6: Pipeline latency + retrain (CO4, CO5, CO6, CO7)
Once sim aero matches reality and sensor scaling is verified, lock in pipeline
latency and train.
- T273h: Custom MSP2_AUTOC_STATE in INAV (autoc branch) + xiao parser
- T273c: Flight mode channel override from xiao
- T273i: Cherry-pick INAV servo logging fix (if not done earlier for de-mixing)
- Bench test: measure new pipeline latency
- T273b: Update COMPUTE_LATENCY in CRRCSim to match
- BIG training with calibrated model + correct latency
- Verify non-degenerate behavior (pitch/throttle not saturated >50% of steps)
- Compare data.dat response curves to flight data — should look similar now

## Data sources for characterization

Available data — no dedicated step-function flights needed to start:
- **Flight blackbox (2026-03-22):** Two flights with climb, coast, rapid recovery,
  NN control at various attitudes. Pilot recovery segments have clear step-like inputs.
  5 MSPRCOVERRIDE test spans. ~8MB CSV + GPS data.
- **BIG3 data.dat (2026-03-23):** 400 generations, 51k steps per gen. Gen 1-5 have
  varied control inputs (throttle mean=0.36, pitch mean=0.83). Later gens saturated
  but useful for steady-state at full throttle. 19M+ lines total.
- **Mini training runs:** 10-20 gen runs with updated hb1_streamer.xml to regenerate
  sim response curves after each tuning iteration. Quick feedback loop.
- **Intentional sim steps:** build only if scraped data is insufficient. Simple test
  controller that commands step inputs in CRRCSim, logs response.
- **Flight replay:** play recorded NN commands into CRRCSim from matched initial state.
  Best for integration validation (Phase 4).

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

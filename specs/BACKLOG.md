# AutoC Backlog

**Last Updated**: 2026-03-18

## Legend

- `[NEXT]` - High priority, ready to start
- `[DEFERRED]` - Lower priority, will revisit
- `[DONE]` - Completed in 015 or prior

---

## NN Training Improvements (015) — active

Tracked in [specs/015-nn-training-improvements/tasks.md](015-nn-training-improvements/tasks.md).

Current milestone: robust repeatable training → flight test.

BIG-aero1 complete: 266 gens, fitness 2,955, 294/294 OK, zero crashes.

Generalization eval complete (T180–T184): 86–98% completion across novel paths,
random geometries, and 120% envelope stress. Controller has significant headroom.

Remaining 015 work:
- Phase 7: Xiao-GP sensor sync (blocker for flight test) ← NEXT
- Phase 6: Aircraft parameter variation (sim-to-real)
- Phase 8: Polish (data.stc, arena layout, legacy tearout, memory leak check)

---

## Future Features (separate from 015)

### [DEFERRED] Selection Strategy Alternatives
- NSGA-II Pareto: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- Rank-based fitness shaping: CMA-ES style rank-derived weights
- sep-CMA-ES optimizer: pop 5000→50, per-weight step size adaptation
- Only pursue if/when epsilon-lexicase plateaus

### [DEFERRED] Path-Relative Smoothness
- Normalize Δu by path curvature: penalize excess control, not turns
- On hold — slew limiting already killed bang-bang, lexicase hasn't plateaued

### [DEFERRED] Total Energy Management + Altitude-Aware Distance
- Current distance metric is flat Euclidean — treats "5m above" same as "5m below"
- In reality above is always safer (altitude = energy reserve), below-and-inside is worst
- Observed in T184: NN flies consistently low and outside at slow rabbit speeds ("race horse"
  effect — trained at 16±4 m/s, throttle oscillates at 12 m/s)
- Proposals:
  - Total energy (altitude + airspeed) as NN input or lexicase objective
  - Altitude-aware distance: asymmetric penalty (below penalized more than above)
  - Wider rabbit speed range in training (include 8–12 m/s slow regime)
- Also enables future tactics layer: arena boundary awareness, altitude floor guard
- Post-flight-test refinement — current flat Euclidean tracking is adequate for first flight

### [DEFERRED] Simulator Sampling Time Variation
- Training uses exact 100ms steps; real hardware has jitter (~100ms ± 10ms)
- Add configurable random dither to sim tick interval during training
- Makes NN robust to real-world MSP bus contention and sensor read latency
- Sim-to-real hardening item — after initial flight test data validates baseline

### [NEXT] GPU-Native Evaluation — required for 017
- Accelerate fitness evaluation on GPU (5000 sims/sec vs ~200)
- BIG-3 training was ~352M sim evaluations (pop=3000 × 294 scenarios × 400 gens)
- 017 (vision NN) at ~3K weights needs at minimum the same scale, likely 2-5×
- Beacon projection adds ~46B projection calls at 400 gens — CPU-only is ~77 min/gen
- Current crrcsim is single-threaded C++ with OpenGL dependency — not GPU-parallelizable
- Options: (a) GPU physics sim (CUDA/Vulkan compute), (b) lightweight FDM on GPU with
  beacon projection fused, (c) hybrid: crrcsim for physics, GPU batch for projection
- DGX Spark (GB10) may be sufficient for Option (c); Options (a/b) may need larger GPU
- **Blocking dependency for 017-phase3 at training scale**
- See: [017 spec](017-visual-target-tracking/spec.md)

### [DONE] Renderer: Path reveal timing with variable rabbit speed
- Fixed: renderer now uses rabbit odometer from AircraftState to reveal path
  segments by distance traveled, not time fraction. Stops at crash point.
- Was T516 from 020, closed 2026-03-31.

---

## Infrastructure

### [NEXT] Type-Safe NN Sensor Interface
- Currently NN inputs/outputs are opaque `float[]` arrays indexed by magic numbers
- Topology changes (29→27) caused silent serialization corruption — now crashes, but
  still fragile for future sensor additions (gravity vector, camera, etc.)
- Need: a typed sensor struct or enum-indexed map that
  - Names each input (e.g. `GYRO_P`, `QUAT_W`, `DPHI_NOW`)
  - Carries type, units, valid range metadata
  - Auto-generates topology count from struct definition
  - Serializes with field names or tags so format is self-describing
  - Compile-time error if evaluator.cc and topology.h disagree
- Also unify the scattered constants: NN_INPUT_COUNT in topology.h, autoc.h,
  evaluator.cc, tests, data.dat format comments, sim_response.py parser
- This is load-bearing for 021+ as we iterate on sensor inputs frequently
- Files that must change when NN_INPUT_COUNT changes (021 learnings):
  - `include/autoc/nn/topology.h` — count, weight count, topology string
  - `include/autoc/autoc.h` — duplicate defines (DISTANCE_TARGET etc.)
  - `src/nn/evaluator.cc` — nn_gather_inputs(), index mapping, comments
  - `src/autoc.cc` — data.dat format string, header, field indices
  - `tests/contract_evaluator_tests.cc` — topology assertions
  - `tests/nn_evaluator_tests.cc` — input layout assertions
  - `specs/019-improved-crrcsim/sim_response.py` — data.dat parser
  - `xiao/src/msplink.cpp` — xiao-side input gathering
  - `include/autoc/eval/aircraft_state.h` — nnInputs_ array size, serialization
- Consideration: prev commands (pitchCmd/rollCmd/throttleCmd feedback) as optional
  inputs — may want to toggle these on/off during experimentation. Type-safe
  interface should support optional/conditional inputs without recompiling everything

### [NEXT] Eval Fitness Computation — Bugs
- **Bug 1: Different metric** — ✅ FIXED post-022. Both training and eval now use
  `computeScenarioScores()` + `aggregateRawFitness()` (conical surface). Verified in
  fitness_decomposition.cc and autoc.cc.
- **Bug 2: Stale fitness in S3** — Eval uploads original NN weights (with training-time
  fitness baked into NN01 format) to S3 via `evalResults.gp`. Renderer deserializes
  this and shows the ORIGINAL stored fitness, not the eval result. Even with
  radically different eval scenarios, renderer always shows the training fitness.
  - Flow: `nn_weights.dat` (carries fitness from nnextractor) → `nn_deserialize` →
    `genome.fitness=508K` → raw bytes copied to `evalResults.gp` → S3 → renderer
  - The eval-computed fitness is only printed to console/stc, never stored
  - Fix: update `genome.fitness` with eval result before serializing to evalResults,
    OR store eval fitness in a separate evalResults field the renderer can read
- **Bug 3 (NEW 2026-04-07): Eval mode missing rabbit speed config** — `runNNEvaluation()`
  in `src/autoc.cc` (around L750-787) builds `EvalData` without setting `evalData.rabbitSpeedConfig`.
  Default is `{nominal=16.0, sigma=0.0}` from `RabbitSpeedConfig::defaultConfig()` —
  i.e., **constant 16 m/s rabbit**, NOT the configured 13±2 m/s from autoc-eval.ini.
  - Training path (L939-940 and L1028-1029) correctly sets `evalData.rabbitSpeedConfig = gRabbitSpeedConfig`.
  - Symptom: eval fitness on saved gen-N weights is "slightly different" from the training
    fitness reported at gen N — same NN, same scenarios, but different rabbit trajectories
    because rabbit speed profile is wrong.
  - Fix: add `evalData.rabbitSpeedConfig = gRabbitSpeedConfig;` (and `* computeVariationScale()`
    for consistency, though both return 1.0) at line ~756 in eval path.
  - Discovered while trying to reproduce 022 betterz2 gen 400 fitness in eval mode.

### [DONE 2026-04-07] Refactor Duplicate Fitness Constants
- ✅ Resolved by 022 conical-surface refactor. The old DISTANCE_TARGET / ATTITUDE_NORM
  constants no longer exist. fitness_computer.h is the single source of truth for
  the conical scoring surface (FitDistScaleBehind, FitDistScaleAhead, FitConeAngleDeg).

### [DEFERRED] Streak Threshold Ramp (022 T024)
- Originally proposed in 022: ramp `FitStreakThreshold` from min (e.g., 0.1, ~22m
  forgiving) to max (0.5, ~7m demanding) over training via `computeVariationScale()`.
- Goal: early generations get streak credit for "getting closer," late generations
  demand tight tracking.
- **Verdict 2026-04-07**: betterz2 (V4 conical, 400 gens) converged strongly without
  this. Not needed for current curriculum. Park as a potential tool if a future
  curriculum widens or training plateaus on a harder task.
- Implementation (when needed):
  - Add `FitStreakThresholdMin` / `FitStreakThresholdMax` to autoc.ini, config.h, config.cc
  - In `computeScenarioScores()`, interpolate threshold = min + (max-min) * computeVariationScale()
  - FitnessComputer constructor takes the interpolated threshold

### [NEXT] Batch and Cache Deterministic Scenarios
- With 150+ scenarios per individual, serializing full table per eval is expensive
- Send scenario table once at generation start, cache in crrcsim
- Reduces per-eval serialization from O(scenarios × individual) to O(individual)

### [DEFERRED] Output Cleanup
- OutputDir config key, auto-created run subdirectory, clean eval prefix naming

### [NEXT] Make pathgen.h Portable for Embedded
- Single pathgen.h that works on both desktop and embedded
- Current state: embedded_pathgen_selector.h is a manual clone of desktop pathgen.cc
  with different helpers — changes don't propagate (e.g. FortyFiveDegreeAngledLoop
  still at 0.5 rad step vs desktop 0.05 rad after fix 45df719)
- Immediate fix: update embedded FortyFiveDegreeAngledLoop to 0.05 rad step
- Long-term: refactor so both desktop and embedded use the same path generation code

---

## Embedded / Hardware

### [NEXT] Export RC Commands to Xiao Log
- Log RC commands throughout entire flight for full playback visualization
- Location: xiao/src/msplink.cpp

### [ACTIVE → 021] Xiao Onboard IMU as AHRS Cross-Check
- Moved to [specs/021-xiao-ahrs-crosscheck/spec.md](021-xiao-ahrs-crosscheck/spec.md)
- P0 blocker: Mar 27 flight showed uncontrolled rotation — cannot tell if AHRS or gain mismatch
- LSM6DS3 + Madgwick on Xiao, log alongside INAV quat, compare post-flight

### [DEFERRED] GPS Dropout Handling During NN Control
- What happens when GPS drops out during NN-active control? Position freezes, NN sees stale data
- Need: xiao detects stale position (no update for N ms) and either disables autoc or flags it
- Also consider MSP communication loss detection and safe fallback

### [DEFERRED] Xiao Safety Checks Pre-Arm
- Ensure mode flip is safe: RC failsafe, RC disarm, hold/RTH should disarm co-processor

### [DEFERRED] Speed Up Logfile Download
- BLE download may be over-bucketed from prior troubleshooting

---

## Visualization

### [DEFERRED] Blackbox Rendering Improvements
- Select path + blackbox log for comparisons, FPV mode

### [DEFERRED] CRRCSim Display Dependency
- CRRCSim requires valid DISPLAY even in headless mode

### [DEFERRED] Clean CRRCSim Shutdown
- Polling loop for keepalive; clean exit when autoc exits

---

## Code Cleanup

### [DEFERRED] Memory Leak Investigation
- Small memory leak exists in autoc

### [DEFERRED] Cross-Platform Verification
- Train on aarch64, pull repo on x86
- Build and run renderer/nnextractor/eval against aarch64 S3 objects
- Validates cereal binary portability end-to-end

---

## Completed / Superseded

- ~~Sigma Floor~~ — done (015 Phase 1)
- ~~Curriculum Ramp~~ — done (015 Phase 2, wind scenario ramp)
- ~~Fitness Decomposition~~ — done (015 Phase 2, per-scenario scores)
- ~~Pareto Multi-Objective~~ — superseded by epsilon-lexicase (015 Phase 3)
- ~~Demetic Mode Elite~~ — superseded by lexicase selection
- ~~Wind Speed Variation~~ — done (WindScenarios with varied seeds)
- ~~Immelman Path Fix~~ — done (T121a, progressiveDistance split-S fixed)
- ~~Float Precision Non-Determinism~~ — done (integer timestamps in Path)
- ~~GP Eval Node Test Coverage~~ — superseded (GP removed, NN evaluator has tests)
- ~~Fitness Output Formatting~~ — superseded (aggregateRawFitness is canonical)
- ~~Training Record Consistency~~ — done (S3 upload in eval mode, consistent keys)
- ~~Consolidate PRNG~~ — done (rng.h covers all sites)
- ~~Upper-Level Intercept Director~~ — superseded by entry variation training
- ~~Future State Predictor NN~~ — superseded by temporal history + forecast inputs
- ~~Behavioral Cloning Bootstrap~~ — not needed, direct NN training working

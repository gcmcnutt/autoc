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

### [DEFERRED] GPU-Native Evaluation
- Accelerate fitness evaluation on GPU (5000 sims/sec vs ~200)
- Major effort, future research

---

## Infrastructure

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

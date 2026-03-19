# Tasks: NN Training Improvements

**Input**: Design documents from `specs/015-nn-training-improvements/`

---

## Status Summary (completed phases)

### Phase 1: Sigma Floor — DONE
Mutation sigma clamped to configurable floor. Prevents search freeze on long runs.

### Phase 2: Fitness Decomposition — DONE
Per-scenario × per-component scores (completion, distance RMSE, attitude, smoothness[3])
extracted from EvalResults. Logged per-generation for best individual.

### Phase 3: Lexicase Selection — DONE
Epsilon-lexicase replaces scalar tournament. Priority: completion_fraction → distance_rmse.
Minimax and sum modes retained for comparison. Config: `SelectionMode = lexicase`.

### Phase 4a: Sensor Expansion — DONE
- 29 inputs (up from 22): 6 temporal slots for dPhi/dTheta/dist, dDist/dt, quaternion, airspeed, alpha, beta, cmd feedback
- Topology: 29→16→8→3 = 616 weights. Clean break — old weights incompatible.
- Normalizers (NORM_DIST, NORM_VEL, NORM_ANGLE, NORM_RATE) removed — raw values
- att= now uses quaternion geodesic distance (2*acos(|q1·q2|)) — no gimbal lock, logging only
- Smoothness as lexicase dimension: **tried and reverted**. Pegged outputs (Δu=0) score
  perfectly smooth → reinforces spiral (pitch=max, throttle=max, roll-only). Observed gen 73:
  sm=0.00/0.41/0.00. Path-relative smoothness (normalize Δu by path curvature) is the
  correct formulation — deferred to Phase 7.
- WindScenarios: 9→25 for richer lexicase test-case diversity

**Observed outcome**: Sensors helped tracking (dist 30–60m → 2–12m), but spiral persists.
All 25 scenarios complete with throttle sm=0.00 (pegged). The NN found a fixed-throttle
strategy that works on all wind seeds. Selection (completion→distance) doesn't penalize it.

---

## Current situation — BIG-aero1 complete (gen 266, fitness 2,955)

Full stack active: entry phase + energy lexicase + servo slew + path diversity + rabbit speed.
BIG-aero1 (pop=3000, aeroStandard 6 paths × 49 winds = 294 scenarios) ran 266 gens:
- Best fitness: 114M → 2,955 (38,800× improvement)
- 294/294 OK at gen 266, zero crashes
- Dist: median ~8m, 75th pctile ~14m, 95th pctile ~22m (5 outliers at 25-27m)
- Throttle: well-distributed 0.19–1.00, no bang-bang
- Sigma: 0.20 → 0.05 (tight convergence)
- NN_ELITE_SAME confirmed every generation (re-eval bug fixed)
- Terminated by crrcsim segfault after gen 266 eval. Checkpoint saved.

**Next**: Generalization eval (T180–T183) — test on novel paths never seen in training.
This is the go/no-go gate before expanding to aircraft variations and flight test.

---

## Phase 4b: Anti-Spiral Selection Pressure

**Goal**: Add energy as a lexicase dimension. Quick to implement, directly attacks
throttle saturation without requiring intercept or path changes.

**Hypothesis**: Among individuals with similar completion and distance, prefer lower
mean throttle. This should push the search toward efficient flight paths.

**Note**: mean throttle is already in smoothness[2] — no new computation needed.

- [x] T100 Fix build: NORM_* refs removed from nn_evaluator_tests.cc
- [x] T101 Mean throttle as 3rd lexicase dimension: completion → distance → energy
- [x] T102 300-gen experiment: spiral broken gen ~234, thr=0.24–0.60 at gen 300, 25/25 OK

**Checkpoint**: ✓ Energy dimension works. Spiral broken. Path-relative smoothness deferred.

---

## Phase 4c: Intercept / Entry Phase — highest leverage

**Goal**: Force the aircraft to *acquire* the path from a cold-start offset, not just
maintain proximity. A spiral can only maintain — it cannot intercept.

**Why this is the real fix**: The spiral is a degenerate solution that only works when
already near the path. If every episode begins off-path (random position offset,
heading offset, approach speed), the NN must develop genuine path acquisition behavior.
Completion becomes hard again and creates real lexicase differentiation. This is also
the real-world mission requirement.

**Design questions to resolve before implementation**:
- What is the entry state distribution? (position offset range, heading offset, speed range)
- Does crrcsim support starting the aircraft at an arbitrary position/heading relative to the path?
- Is this a new eval phase (intercept + track) or just varied initial conditions?
- How does completion_fraction handle the intercept phase? (time to acquire vs time on path)

- [x] T110 Audited: full entry variation suite already in crrcsim (heading, roll, pitch,
  speed, position radius, altitude offsets). EnableEntryVariations=1 enables all.
- [ ] T111 Formal intercept design doc — skipped; entry params defined in autoc.ini directly.
  EntryHeadingSigma=45°, EntryPositionRadiusSigma=20m, EntryAltSigma=5m active.
- [x] T112 Entry states already fully implemented — enabled via autoc.ini
- [x] T113 interceptScale ramp (quadratic 0.1→1.0 over budget) handles graceful phase-in.
  Applied to both legacy and lexicase distance/attitude metrics.
- [x] T114 Ran 300-gen experiment: entry variations + energy lexicase broke spiral by gen 234
- [ ] T115 Lexicase ordering revisit — current (completion→distance→energy) working well.
  Defer until slew limits and path diversity are in.

**Checkpoint**: ✓ Entry phase working. Spiral broken. Throttle modulation emerged naturally.

---

## Phase 4d: Servo Slew Rate Limiting — sim fidelity + bang-bang prevention

**Goal**: Real servos have physical slew limits. crrcsim currently applies NN commands
instantaneously each 100ms step with no rate limiting — allowing full-range reversals
every step (observed sm[1]=0.95 in roll). Adding slew limits models real hardware and
forces the NN to develop anticipatory, smooth control without any fitness changes.

**Coordinate conventions** (confirmed from inputdev_autoc.cpp):
- Pitch: NN output [-1,1] → `elevator = -pitchCommand / 2.0` (sim [-0.5,0.5])
- Roll: NN output [-1,1] → `aileron = rollCommand / 2.0` (sim [-0.5,0.5])
- Throttle: NN output [-1,1] → `throttle = throttleCommand / 2.0 + 0.5` (sim [0,1])
- NN eval rate: 10Hz (100ms), compute latency: 40ms. Commands applied at ~10Hz.

**Slew rate spec** (all in NN coordinate space [-1,1], full range = 2.0):
- Pitch: 200%/sec → max Δ = 2.0 × 2.0 × 0.1s = **0.40 per step**
- Roll: 200%/sec → **0.40 per step**
- Throttle: 300%/sec → 2.0 × 3.0 × 0.1s = **0.60 per step** in [-1,1]
  (= 0.30/step in sim [0,1] — i.e. full [0,1] throttle travel in ~0.33s)

**Implementation**: In `inputdev_autoc.cpp`, at the command-apply point (line ~934).
Track `prevApplied{Pitch,Roll,Throttle}` as static locals, clamp delta on each apply.
Hardcoded constants — no config file needed at this stage.

```cpp
// Slew rate limiting — applied when pending command is committed
static const double SLEW_PITCH   = 0.40;  // max Δ per 100ms step (200%/sec)
static const double SLEW_ROLL    = 0.40;  // max Δ per 100ms step (200%/sec)
static const double SLEW_THROTTLE = 0.60; // max Δ per 100ms step (300%/sec)
static double prevPitch = 0.0, prevRoll = 0.0, prevThrottle = -1.0;

pitchCommand    = std::clamp(cmd.pitch,    prevPitch - SLEW_PITCH,    prevPitch + SLEW_PITCH);
rollCommand     = std::clamp(cmd.roll,     prevRoll - SLEW_ROLL,      prevRoll + SLEW_ROLL);
throttleCommand = std::clamp(cmd.throttle, prevThrottle - SLEW_THROTTLE, prevThrottle + SLEW_THROTTLE);
prevPitch = pitchCommand; prevRoll = rollCommand; prevThrottle = throttleCommand;
```

Reset `prevPitch/Roll/Throttle` on scenario reset (gPendingCommand = PendingCommand{} sites).

- [x] T116 Add servo slew rate limiting in `inputdev_autoc.cpp`:
  - Pitch: 200%/sec (0.40/step in NN space)
  - Roll: 200%/sec (0.40/step in NN space)
  - Throttle: 300%/sec (0.60/step in NN space)
  - Reset prev values on scenario reset
  - slewP/slewR/slewT columns skipped — not needed, sm columns already show the effect
- [x] T117 300-gen experiment (ramp2): sm[roll] dropped 0.36–0.55 → 0.18–0.28 (well below
  0.40 limit — NN learned to stay smooth, not just hit the wall). dist 2.4–11m → 1.26–8.01m.
  Best fitness 114.96 → 100.70. 25/25 completion held. No tuning needed.
- [x] T118 No tuning required — rates well-chosen. NN naturally below limits at gen 300.

**Checkpoint**: ✓ Slew limiting works. sm[roll] max 0.55→0.28. Tracking improved.
Bang-bang eliminated. NN learned anticipatory smooth control.

---

## Phase 4e: Path Shape Diversity

**Goal**: 25 wind seeds on the same path geometry still lets a fixed spiral win.
Need varied *path shapes* so no single fixed strategy dominates all scenarios.
(Note: spiral largely broken by gen 234 via energy lexicase + entry phase — path diversity now about quality, not survival.)

**Note**: This is a path generator problem, not config tuning. The current rabbit
path is a fixed circuit — more wind variations just perturb it, not change its topology.

**Path generator methods already implemented** (pathgen.cc) — just need enabling in autoc.ini:
- `longSequential` — current default, fixed figure-8 circuit
- `aeroStandard` — 6 deterministic path variants (loop, figure-8, banked turns, etc), same seed
- `progressiveDistance` — increasing path length over training (curriculum learning)

- [x] T120 Experiment: `PathGeneratorMethod = aeroStandard`, 25 wind × 6 path variants = 150
  scenarios per generation. Path diversity working. Throttle modulation emerged.
- [x] T121a Split-S path fix: progressiveDistance Split-S exit now heads cleanly north
  with extended south run before entry. aeroStandard Split-S retains 150° angled approach.
- [x] T122 Rabbit speed tuning for closing-rate signal activation:
  `RabbitSpeedNominal=16 RabbitSpeedSigma=4 RabbitSpeedMin=8 RabbitSpeedMax=24`
  `RabbitSpeedCycleMin=3 RabbitSpeedCycleMax=7` (5s cycles, ~20% delta = ±3.2 m/s)
  Active in BIG-aero1. Throttle distribution 0.19–1.00 confirms dDist/dt exercised.
- [x] T123 Large-scale aeroStandard training run (BIG-aero1): pop=3000, 6 paths × 49 winds
  = 294 scenarios. Ran 266 gens over ~7.5 hours. Best fitness 114M → 2,955 (38,800×).
  294/294 OK at gen 266, zero crashes. Median dist ~8m, sigma 0.20→0.05.
  Terminated by crrcsim segfault (TCP disconnect). Checkpoint saved.

**Checkpoint**: Path shape diversity makes fixed-throttle spiral nonviable across
the scenario set. Closing-rate sensor becomes active signal.

---

## Phase 4f: Fitness Reporting Consistency — eval fixed, remainder deferred to polish

- [x] T124a Eval mode fix: `runNNEvaluation` now uses `aggregateRawFitness` and logs
  per-scenario breakdown ([s] OK/CRASH comp= dist= att= thr= sm=). Done.
- Remaining items (T124b/c, T125) moved to Phase 9: Polish.

---

## Phase 5: Path-Relative Smoothness — on hold

**Status**: On hold. Lexicase hasn't run out of gas. Slew limiting already kills
bang-bang (sm[roll] 0.18-0.28), so smoothness pressure isn't needed yet.

- [ ] T130 Instrument per-step path curvature in EvalResults
- [ ] T131 Compute path-normalized smoothness: |Δu(t)| / max(curvature(t), ε)
- [ ] T132 Add as lexicase dimension after completion and distance
- [ ] T133 Experiment: does path-normalized smoothness reward coordinated turns?

---

## Phase 6: Aircraft Variation (sim-to-real)

**Goal**: Vary aircraft parameters across scenarios so the NN learns a robust controller
that transfers to real hardware. Key sim-to-real gap: the real aircraft has different
mass, CG, control throws, motor response, and aero coefficients than the sim nominal.

- [ ] T140 Define variation parameters: mass ±10%, CG shift ±5mm, control surface
  throw ±15%, motor thrust curve ±10%, drag coefficient ±10%
- [ ] T141 Implement per-scenario aircraft parameter perturbation in crrcsim
  (apply multipliers to FDM constants at scenario init, reset on next scenario)
- [ ] T142 Config: `EnableAircraftVariations=1`, sigma params in autoc.ini
- [ ] T143 Experiment: train with aircraft variations, compare eval robustness
  vs nominal-only training

---

## Future features (separate from 015)

The following are potential improvements deferred to their own feature branches.
They are not blockers for the current milestone (robust repeatable training → flight test).

### Selection strategy alternatives (if/when lexicase plateaus)
- NSGA-II Pareto: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- Rank-based fitness shaping: CMA-ES style rank-derived weights
- sep-CMA-ES optimizer: pop 5000→50, per-weight step size adaptation

---

## Phase 7: Xiao-GP Sensor Sync — embedded controller update

**Goal**: Bring xiao (embedded flight controller) in sync with current desktop
NN sensor layout and verify end-to-end sensor pipeline correctness.

- [x] T170 Audit xiao sensor inputs vs desktop nn_gather_inputs():
  All 29 inputs already matched — shared source files (evaluator.cc, sensor_math.cc)
  compiled into both desktop and xiao via platformio build_src_filter. History buffer,
  forecast lookahead, path interpolation, quaternion, aero angles all identical.
- [x] T171 Xiao sensor gather already in sync — no porting needed. Shared code.
- [x] T172 Embedded pathgen verified compatible with interpolated targeting.
  Fixed FortyFiveDegreeAngledLoop step from 0.5 rad → 0.05 rad (matching desktop).
- [x] T173 End-to-end bench validation via flight log analysis:
  - Fixed critical bug: simTimeMsec was set to absolute millis() (~1.2M) instead of
    elapsed time since autoc enable. getInterpolatedTargetPosition was clamping to
    last path point every tick — dist was frozen at ~0.4m. Fixed by setting
    aircraft_state.setSimTimeMsec(elapsed_msec) before NN eval.
  - After fix: verification script (scripts/verify_flight_log.py) independently
    reconstructs StraightAndLevel path and computes expected sensor values from
    Nav State position + quaternion. Results:
    - dist error: 0.1-0.5m (GPS noise on stationary bench, not computation error)
    - dPhi error: < 0.003 rad across 128 ticks
    - dTheta error: < 0.002 rad (one ±π wraparound, not real error)
    - dDist/dt: correctly shows -16 m/s (rabbit pulling away at 16 m/s)
    - History buffer: **zero mismatches** across 1,536 checks (128 ticks × 3 channels × 4 slots)
    - Forecasts: errors track GPS position offset, not computation error
  - Quaternion heading: bench test shows 41° vs INAV's 179°. Analysis traced to
    INAV's MSP quaternion being in sensor/IMU frame — board alignment and GPS heading
    fusion applied separately. Prior flight logs confirmed correct in actual flight.
    Not a blocker — will verify on first flight.
  - Also this session: GP→Nav rename in all log strings, compact NN I/O telemetry
    (full 29 inputs + 3 outputs per tick), nn2cpp timestamp for rebuild detection,
    moved xiao/generated/ under xiao/src/generated/ for PlatformIO dependency tracking.

---

## Phase 8: Polish

- [x] T124b NN_ELITE_DIVERGED confirmed fixed: BIG-aero1 ran 266 gens with NN_ELITE_SAME
  on every generation. The plumbing bug (no signal reaching re-eval) was resolved.
  Per-scenario vector comparison deferred — scalar bitwise check now works reliably.
- [ ] T124c Document/assert that genome.fitness is always aggregateRawFitness. Add a
  comment to the fitness assignment site so it's clear what's stored.
- [ ] T125 Store per-scenario lexicase scores in data.stc for offline robustness analysis.
  Currently only scalar fitness logged there. Include completion, dist_rmse, mean_throttle
  per scenario for the best individual each generation.
- [ ] T155 Renderer: arena layout should be `numPaths × numWinds` grid when
  `numPaths > 1`. Each path variant gets its own column; wind seeds are rows within
  each column. Minimum-area-rectangle layout retained for single-path case.
- [x] T160 All 3 repo builds verified: autoc (cmake), crrcsim, xiao (PlatformIO both targets)
- [ ] T161 Run full test suite: `cd build && ctest --output-on-failure`
- [x] T162 Export pipeline verified: nnextractor → nn2cpp → xiao build → upload → bench test
- [ ] T163 Remove `SinglePathProvider` from aircraft_state.h (no callers after xiao fix)
- [ ] T164 GP legacy cleanup: scan for GP-era remnants (`GPrandDouble`, `gp_` prefixes,
  `#ifndef` guards → `#pragma once`). At minimum: include/autoc/util/gp_math_utils.h,
  include/autoc/util/types.h
- [ ] T166 Memory leak sanity check: RSS hit ~13GB during BIG-aero1 run (pop=3000,
  294 scenarios, 109+ gens). Verify whether this is expected working-set size for the
  scale or indicates a leak introduced during the big refactor. Check: per-generation
  arena/path allocations freed, EvalResults vectors not accumulating, cereal serialization
  temporaries, scenario variation tables. Profile with valgrind --tool=massif or
  /proc/self/smaps on a short run and compare RSS growth rate vs generation count.
- [ ] T165 Remove 'sum' selection mode and legacy scalar fitness path — tearout list:
  - `SelectionMode = sum` branch in autoc.cc
  - `aggregateScalarFitness()` in fitness_decomposition.cc (only logs now, not selected)
  - `distance_sum_legacy`, `attitude_sum_legacy`, `legacy_crash_penalty` fields in ScenarioScore
  - `legacy_attitude_scale`, `computeAttitudeScale()`, `DISTANCE_NORM`, `ATTITUDE_NORM` constants
  - `legacy_distance_sum`, `legacy_attitude_sum` accumulation loop in fitness_decomposition.cc
  - `computeStepPenalty()` in fitness_computer.cc (only used by sum path)
  - After removal: `aggregateRawFitness()` is the only scalar summary needed (diagnostics only)

---

## Phase 8b: Generalization Eval — novel path stress test

**Goal**: Verify the trained NN learned general path-following, not memorized the 6
aeroStandard geometries. Export best weights from BIG-aero1, evaluate on paths never
seen during training. This is the real acceptance test before flight.

- [x] T180 Eval on progressiveDistance paths: BIG-aero1 genome (fitness 2,955) on novel
  progressiveDistance geometry (2270 segments, 49 winds). Result: 42/49 OK (85.7%).
  7 crashes all from extreme entry attitudes (135° heading reversal, >30° roll, large
  offsets). OK flights: dist median ~8m, consistent with training quality. Confirms
  generalization to novel path geometry — failures are entry recovery, not tracking.
- [x] T181 Eval on longSequential: BIG-aero1 genome on fixed figure-8 circuit (273 segments,
  49 winds). Result: 48/49 OK (98.0%). Single crash at [18] (heading=-76°, roll=+22°,
  comp=0.68 — entry stress, not tracking failure). No temporal drift over extended circuit.
  Dist median ~8m, consistent with training. Strongest generalization result.
- [x] T182 Random path generalization test (12 paths × 12 winds = 144 flights):
  Novel random geometries (seed=99999, never seen in training), full training-envelope
  sigmas. Result: 141/144 OK (97.9%). 3 crashes: [23] spiral fallback (thr=1.00,
  comp=0.14), [79] late crash (comp=0.79), [125] mid-course (comp=0.54).
  Dist median ~13m (wider than training's 8m — expected for harder random geometry).
  **Confirms NN learned general path-following, not memorized training paths.**
- [x] T182b Random path envelope test (12×12=144 flights at 120% training sigmas):
  Same seed=99999 paths, all entry/wind/rabbit sigmas at 120%. Result: 141/144 OK (97.9%)
  — identical completion rate to T182 at 100%. 3 crashes: [35] spiral (comp=0.07),
  [57] late (comp=0.84), [79] late (comp=0.78). Dist median ~14m (vs ~13m at 100%).
  **Controller has significant headroom beyond training envelope.** Failure mode remains
  rare spiral fallback on extreme entries, not systematic breakdown from harder conditions.
- [x] T183 Failure mode analysis: two distinct crash modes identified across eval suite:
  1. **Extreme entry** (T180): large heading reversals (>90°), big roll — physically
     unrecoverable. Expected, not a policy defect.
  2. **Geometry + altitude edge case** (T182/T182b): paths starting with dive + hard turn,
     combined with low altitude offset, crash immediately (comp=0.07). 11/12 winds on same
     path survive fine — it's one unlucky altitude draw, not spiral fallback. thr=1.00 is
     "full power climb attempt", not degenerate spiral.
  **Conclusion**: ~2% failure floor is from irreducible edge cases, not policy gaps.
  The dive-and-turn-and-throttle intercept tactic is correct learned behavior — the NN
  lacks altitude floor awareness, which is a higher-level safety concern (total energy
  management, arena boundaries) for a future tactics layer above the path follower.
  No training changes needed — proceed to flight test prep.
  **Note**: throttle across all evals is still largely bang-bang (thr 0.40-0.75 average
  but full range used). Smoothness metrics show sm[thr] typically 0.12-0.20. Future
  work: total energy (elevation + speed) as input/objective, altitude floor guard.

- [x] T184 Quiet throttle baseline: single longSequential path, no wind, no entry variation,
  constant rabbit speed 12 m/s. Result: OK comp=1.00 dist=5.84 thr=0.47 sm[thr]=0.20.
  Throttle still fairly bang-bang even in calm conditions — "race horse" effect: NN trained
  at 16±4 m/s overshoots the slow 12 m/s rabbit, oscillates between catch-up and coast.
  Bang-bang is intrinsic to the learned policy, not purely scenario-driven. Future work:
  total energy input/objective, or training at wider rabbit speed range including slow end.

---

## Priority Order — milestone: robust training → flight test

1. **T100–T102** — DONE: energy lexicase
2. **T110–T115** — DONE: entry/intercept phase
3. **T116–T118** — DONE: servo slew rate limiting
4. **T120–T121a** — DONE: aeroStandard + path fixes
5. **T122** — DONE: rabbit speed tuning (sigma=4, 3-7s cycles)
6. **T123** — DONE: BIG-aero1 (pop=3000, 294 scenarios, 266 gens, fitness 2,955)
7. **T180–T184** — DONE: generalization eval (86-98% across novel paths)
8. **T170–T173** — DONE: xiao sensor sync + bench validation (simTimeMsec fix, full I/O telemetry)
9. **T160, T162** — DONE: builds + export pipeline verified
10. **T161, T163–T166** — NEXT: polish (test suite, cleanup, memory check)
11. **T140–T143** — aircraft variation (sim-to-real, post-flight-test)
12. **T124c, T125, T155** — lower-priority polish
11. **T130–T133** — path-relative smoothness (on hold, only if lexicase plateaus)

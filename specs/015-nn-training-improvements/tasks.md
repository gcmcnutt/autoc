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

## Current situation — BIG-3 complete (gen 400, fitness 2,724) — BEST RUN

Three BIG training runs completed. BIG-3 is the best result:

| Run | Config delta | Gens | Best fitness | Crashes | Dist avg | Throttle | Sigma |
|-----|-------------|------|-------------|---------|----------|----------|-------|
| BIG-aero1 | baseline (Z sigma=5) | 266 | 2,955 | 0/294 | 15-25m | 0.19-1.00 (bang-bang) | 0.05 |
| BIG-2 | same as aero1 | 313 | 5.7M | 8/294 | 13-20m | 0.66-0.97 | 0.06 |
| **BIG-3** | **Z sigma=3** | **400** | **2,724** | **0/294** | **9.1m** | **0.48 avg (proportional)** | **0.05** |

**BIG-3 detailed results (gen 400, pop=3000, aeroStandard 6 paths × 49 winds = 294 scenarios)**:
- Best fitness: 124M → 2,724 (45,600× improvement)
- **294/294 OK, zero crashes** across all wind/path/entry combinations
- Distance: avg 9.1m, 17% under 5m, 64% under 10m
- Attitude: avg 0.21 (clean, well-controlled flight)
- **Throttle: avg 0.48 — real proportional control** (60% below 0.5, 36% mid-range, only 5% above 0.8)
- **Smoothness: pitch=0.19, roll=0.15, thr=0.22** — smooth control throughout
- Sigma: 0.20 → 0.05 (tight convergence)
- Key transitions: 124M→20.8M (gen 35), 2.96M (gen 96), 2,850 (gen 157), 2,724 (gen 383)
- Avg population fitness: 250M → 105M (whole population learning, not just elite)
- NN_ELITE_SAME confirmed every generation
- **Ran to completion (400 gens)** — first BIG run to finish without crrcsim crash

**Key insight**: Z sigma 5→3 was the differentiator. BIG-aero1 had bang-bang throttle
(thr 0.91-0.97, sm[thr] 0.01-0.05). BIG-3 has proportional throttle (avg 0.48,
sm[thr] 0.22) and tighter tracking (9.1m vs 15-25m). Narrower altitude entry variation
lets the NN learn energy management rather than emergency recovery. BIG-2 shows training
is sensitive to initial population seeding (same config as aero1, never converged well).

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

**Scaling concerns** (observed BIG-aero2 vs BIG-aero1): identical config except
EntryPositionAltSigma 5→3 (should be easier), yet BIG-aero2 at gen 305 is 5.7M
vs BIG-aero1's 2,955 at gen 266. Training is sensitive to initial population seeding.
When adding aircraft variations, the scenario space expands significantly. Consider:
- **Warm start**: seed population from prior best genome (e.g. BIG-aero1 weights)
  and mutate from there, rather than random init. Most efficient for incremental dims.
- **Variation ramp**: start narrow envelope, widen over generations (curriculum learning).
  BIG-aero1 succeeded without it, but BIG-aero2's struggle suggests it's not reliably
  optional. Especially important when adding aircraft param variations on top of
  entry/wind/rabbit variations.
- **More scenarios**: 49 may not provide enough lexicase differentiation with aircraft
  variations added. Consider 64, 81, or 100 (trade-off: wall-clock per gen).
- **Larger population**: 3000→5000 gives more genetic diversity but linear cost.
  May be needed if warm start isn't sufficient.

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
10. **BIG-3** — DONE: best run (gen 400, fitness 2,724, 0 crashes, proportional throttle)
11. **Eval suite** — DONE: `scripts/eval_suite.sh` automated regression (T0-T3), 6/7 PASS
12. **Flight test** — NEXT: load BIG-3 weights onto flight hardware, first NN flight
13. **Flight data analysis** — post-flight: compare flight logs to desktop sensor pipeline
14. **Feature 015 closeout** — wrap up after flight data analysis
15. **T161, T163–T166** — polish (test suite, cleanup, memory check) — post-flight
16. **T140–T143** — aircraft variation (sim-to-real, next feature after 015 closeout)
17. **T124c, T125, T155** — lower-priority polish
18. **T130–T133** — path-relative smoothness (on hold, only if lexicase plateaus)

---

## Automated Evaluation Suite — `scripts/eval_suite.sh`

**Usage**: `./scripts/eval_suite.sh nn_weights.dat [tier]`

Automated regression suite. `autoc-eval.ini` is the single source of truth for all
sigmas — must match training config exactly. Tier overrides only change path type,
dimensions, and seed — never sigmas. Results archived in `eval-results/<timestamp>/`.

### BIG-3 eval suite results (2026-03-20, fitness 2,724):

| Tier | Test | Result | OK/Total | Dist | Thr | SmT |
|------|------|--------|----------|------|-----|-----|
| T0 | repro (exact seed) | **PASS** | 294/294 (100%) | 9.1 | 0.48 | 0.22 |
| T1 | aeroStandard (novel seed) | **PASS** | 294/294 (100%) | 8.5 | 0.46 | 0.23 |
| T2 | progressiveDistance | **PASS** | 48/49 (97.9%) | 6.2 | 0.30 | 0.27 |
| T2 | longSequential | **PASS** | 49/49 (100%) | 6.1 | 0.35 | 0.27 |
| T2 | random 12×12 | FAIL | 133/144 (92.3%) | 12.0 | 0.48 | 0.22 |
| T3 | stress 120% sigmas | **PASS** | 142/144 (98.6%) | 13.0 | 0.49 | 0.22 |
| T3 | quiet (12 m/s, no wind) | **PASS** | 1/1 (100%) | 4.9 | 0.39 | 0.26 |

**Key findings**: T0 bitwise fitness match confirms toolchain integrity. T1-T2-T3 show
strong generalization — only T2-random below 95% (novel path geometry is the frontier).
Stress at 120% sigmas actually scores *better* than random at training sigmas — path shape
matters more than envelope edge. Throttle proportional across all tiers (avg 0.30-0.49).

### Tier definitions:

#### Tier 0: Reproducibility (1 run)
- Exact seed repro using `autoc-eval.ini` as-is (must match training seed + config)
- **Pass**: bitwise fitness match with stored value
- **Purpose**: verify toolchain, weights extraction, config parity

#### Tier 1: Quick validation (1 run, ~3 min)
- aeroStandard 6×49 = 294 flights, novel seed (`Seed = -1`)
- **Pass**: ≥95% completion, avg dist < 15m

#### Tier 2: Generalization (3 runs, ~10 min)
- progressiveDistance 1×49 = 49 flights (novel sequential geometry)
- longSequential 1×49 = 49 flights (stability/drift test)
- random 12×12 = 144 flights (novel paths, seed=99999)
- **Pass**: ≥95% completion per test

#### Tier 3: Stress / envelope (2 runs, ~5 min)
- random 12×12 at 120% training sigmas = 144 flights
- quiet: longSequential 1×1, no wind, rabbit 12 m/s (throttle character)
- **Pass**: ≥95% completion (stress), 0 crashes (quiet)

#### Tier 4: Future (post aircraft variations)
- random 25×25 with craft variation sigmas = 625 flights
- random paths at 20% inside training envelope = regression guard
- Pass criteria: TBD based on Phase 6 results

### Eval suite backlog:
- [ ] Config stacking (`-i base -i overlay`) to eliminate sigma drift between training/eval INIs
- [ ] Archive data.dat and data.stc alongside S3 eval uploads
- [ ] `nnextractor` option to pull weights by S3 key/timestamp (not just latest)
- [ ] Eval metadata (tier, weights hash, pass/fail) linked to S3 keys for renderer lookup

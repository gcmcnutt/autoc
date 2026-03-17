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

## Current situation: the spiral problem

The spiral exploit survives because:
1. The path is fixed geometry — a fixed spiral can stay near the rabbit
2. Lexicase only asks: *did you survive?* and *did you stay close?* — spiral answers both
3. No test case asks: *can you acquire the path from a cold start?*
4. No penalty for high throttle / high energy

The three levers to break it (in priority order):
- **Intercept/entry phase**: force path acquisition from varied position/heading/speed — a spiral can't intercept, only maintain
- **Energy pressure**: penalize high throttle — directly attacks the exploit
- **Path shape diversity**: varied geometry so no fixed strategy dominates all scenarios

---

## Phase 4b: Anti-Spiral Selection Pressure

**Goal**: Add energy as a lexicase dimension. Quick to implement, directly attacks
throttle saturation without requiring intercept or path changes.

**Hypothesis**: Among individuals with similar completion and distance, prefer lower
mean throttle. This should push the search toward efficient flight paths.

**Note**: mean throttle is already in smoothness[2] — no new computation needed.

- [ ] T100 Fix build: NORM_ANGLE/NORM_DIST/NORM_VEL/NORM_RATE removed from topology.h
  but still referenced in tests/nn_evaluator_tests.cc — remove stale test lines
- [ ] T101 Add mean throttle (energy) as 3rd lexicase dimension: completion → distance → energy
  in src/eval/lexicase_selection.cc. Use smoothness[2] (already computed). Epsilon same as distance.
- [ ] T102 Run 50-gen experiment, log throttle distribution — does mean throttle drop?
  Does spiral break or just become a more efficient spiral?

**Checkpoint**: Energy dimension tested. If throttle drops significantly → keep. If spiral
just adapts → confirms intercept phase is the required fix.

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

- [ ] T110 Audit current entry variation: what does `EntryRollSigma=22.5` actually do?
  Read crrcsim inputdev_autoc.cpp entry logic. Is position offset already supported?
- [ ] T111 Design intercept phase: define entry state distribution (position: ±Xm from path
  start, heading: ±Y°, speed: Z m/s range). Document in specs/015.../intercept-design.md
- [ ] T112 Implement varied entry states in crrcsim — randomize initial position/heading
  relative to path start using seed from scenario index
- [ ] T113 Update completion_fraction to include intercept: time-to-acquire counts against
  completion, or add intercept_success as a new lexicase dimension
- [ ] T114 Run 50-gen experiment with intercept — does spiral fail to complete? Does
  lexicase find genuine path-acquisition behavior?
- [ ] T115 Revisit lexicase ordering once intercept is working: completion → energy → distance
  may be better ordering at this point (survive, then be efficient, then be precise)

**Checkpoint**: Spiral fails to complete intercept phase. Lexicase differentiates on
acquisition ability. Throttle modulation emerges naturally.

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

- [ ] T116 Add servo slew rate limiting in `inputdev_autoc.cpp`:
  - Pitch: 200%/sec (0.40/step in NN space)
  - Roll: 200%/sec (0.40/step in NN space)
  - Throttle: 300%/sec (0.60/step in NN space)
  - Reset prev values on scenario reset
  - Add `slewP`, `slewR`, `slewT` columns to data.dat (actual vs requested delta) for analysis
- [ ] T117 Run 50-gen experiment after slew limiting — observe sm[1] drop, check completion
  holds. Does roll sm[1] drop from ~0.7 to ≤0.40? Does tracking degrade?
- [ ] T118 Tune slew rates if needed — if tracking degrades badly, loosen limits.
  If sm still high, tighten. Goal: sm[1] ≤ 0.3 without completion loss.

**Checkpoint**: sm values physically bounded. Bang-bang eliminated as a strategy.
Controller must anticipate turns and modulate smoothly.

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

- [ ] T120 Experiment: `PathGeneratorMethod = aeroStandard`, 25 wind × 6 path variants = 150
  scenarios per generation. Observe if path diversity breaks remaining degenerate strategies.
- [ ] T121a **Prereq — Immelmann renderer fix**: progressiveDistance paths include
  inverted/loop segments; renderer currently assumes upright flight for path plotting,
  producing garbled visualization during Immelmann/loop phases. Fix renderer path
  drawing to handle inverted attitude before running T121 experiment.
- [ ] T121b Experiment: `PathGeneratorMethod = progressiveDistance` — curriculum learning,
  shorter paths early, full paths later. Does it improve early convergence?
- [ ] T122 Rabbit speed tuning for closing-rate signal activation:
  `RabbitSpeedNominal=16 RabbitSpeedSigma=4 RabbitSpeedMin=8 RabbitSpeedMax=24`
  `RabbitSpeedCycleMin=3 RabbitSpeedCycleMax=7` (5s cycles, ~20% delta = ±3.2 m/s)
  Goal: force dDist/dt sensor to become active. Does thr= track rabbit speed?
- [ ] T123 Overnight scale-up config: pop=5000, gens=500, WindScenarios=25, aeroStandard paths.
  Run on full hardware allocation. Baseline for post-slew-limit comparison.

**Checkpoint**: Path shape diversity makes fixed-throttle spiral nonviable across
the scenario set. Closing-rate sensor becomes active signal.

---

## Phase 4f: Elite Fitness Fix — NN_ELITE_DIVERGED and multivariate storage

**Problem**: `NN_ELITE_DIVERGED` warnings appear every generation because the elite
re-evaluation uses `aggregateScalarFitness()` (legacy scalar sum) while selection uses
lexicase. These are different metrics — they will never match. The warning is a false
positive that obscures genuine divergence (e.g. stochastic wind causing different outcomes).

**Root cause** (autoc.cc ~line 1197-1203):
```cpp
double reevalFitness = aggregateScalarFitness(reevalScores);  // scalar sum
if (!bitwiseEqual(reevalFitness, storedFitness)) {            // vs lexicase-selected
    logger.warn() << "NN_ELITE_DIVERGED" ...
```

**Fix**: Store the elite's per-scenario scores (vector of ScenarioScore) rather than a
scalar. Re-evaluate and compare the scenario-level completion fractions + distance RMSE
vectors. A genuine divergence is when the completion profile changes materially (e.g.
scenario that used to complete now crashes). Tolerate small numeric differences (float
rounding, wind stochasticity).

- [ ] T124 Fix NN_ELITE_DIVERGED: replace scalar comparison with per-scenario vector comparison.
  Store `bestScores: vector<ScenarioScore>` alongside best genome. On re-eval, compare
  completion_fraction per scenario with tolerance (e.g. ≥0.95 of stored). Warn only on
  material degradation (completion drops or dist RMSE grows >20%).
- [ ] T125 Consider storing lexicase rank/scores in data.stc for offline analysis —
  currently only scalar fitness logged there.

**Checkpoint**: Elite divergence warnings only fire on genuine instability, not metric mismatch.

---

## Phase 5: Path-Relative Smoothness — deferred

**Goal**: Correct formulation of smoothness: normalize Δu by the path curvature at
each step. Penalize control change *above what the path demands*.

**Why deferred**: Requires per-step curvature from path geometry (radiansFromStart or
per-segment curvature derivative). The path has this data but instrumenting it in the
fitness pipeline is non-trivial. Correct in principle — do after intercept works.

- [ ] T130 Instrument per-step path curvature in EvalResults (or derive from existing path data)
- [ ] T131 Compute path-normalized smoothness: |Δu(t)| / max(curvature(t), ε)
- [ ] T132 Add as lexicase dimension after completion and distance
- [ ] T133 Run experiment — does path-normalized smoothness reward coordinated turns
  while penalizing oscillation on straight segments?

---

## Phase 6: Research Spikes — after intercept validated

**Goal**: Explore alternative selection strategies once the fitness signal is trustworthy
(post-intercept, post-energy). Each on its own branch.

### NSGA-II Pareto Spike
- [ ] T140 Branch `015-spike-nsga2`: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- [ ] T141 Run 500-gen experiment, compare Pareto front evolution against lexicase

### Rank-Based Fitness Shaping Spike
- [ ] T142 Branch `015-spike-rank-shaping`: CMA-ES style rank-derived weights
- [ ] T143 Run 500-gen experiment, compare against lexicase baseline

### Analysis
- [ ] T144 Compare all spike results, document in research.md — which strategy broke the plateau?
- [ ] T145 Merge winner to 015-nn-training-improvements

---

## Phase 7: CMA-ES Optimizer — after fitness signal is trustworthy

**Goal**: Replace GA with sep-CMA-ES. Pop 5000→50. Only worth doing once fitness signal
gives real gradient — with a spiral exploit in place, CMA-ES will just optimize the spiral faster.

- [ ] T150 Branch `015-spike-cmaes`
- [ ] T151 Implement SepCMAES (ask/tell) in include/autoc/nn/sep_cmaes.h + src/nn/sep_cmaes.cc
- [ ] T152 Wire into evolution loop, run 500-gen experiment with pop=50
- [ ] T153 If validated, merge and remove GA code

---

## Phase 8: Polish

- [ ] T160 Verify all 3 repo builds: autoc, crrcsim, xiao
- [ ] T161 Run full test suite: `cd build && ctest --output-on-failure`
- [ ] T162 Verify export pipeline: nnextractor → nn2cpp → xiao build
- [ ] T163 GP legacy cleanup: scan for GP-era remnants (`GPrandDouble`, `gp_` prefixes,
  `#ifndef` guards → `#pragma once`). At minimum: include/autoc/util/gp_math_utils.h,
  include/autoc/util/types.h
- [ ] T164 Remove 'sum' selection mode and legacy scalar fitness path — tearout list:
  - `SelectionMode = sum` branch in autoc.cc
  - `aggregateScalarFitness()` in fitness_decomposition.cc (only logs now, not selected)
  - `distance_sum_legacy`, `attitude_sum_legacy`, `legacy_crash_penalty` fields in ScenarioScore
  - `legacy_attitude_scale`, `computeAttitudeScale()`, `DISTANCE_NORM`, `ATTITUDE_NORM` constants
  - `legacy_distance_sum`, `legacy_attitude_sum` accumulation loop in fitness_decomposition.cc
  - `computeStepPenalty()` in fitness_computer.cc (only used by sum path)
  - After removal: `aggregateRawFitness()` is the only scalar summary needed (diagnostics only)

---

## Priority Order

1. **T100–T102** — DONE: energy lexicase. Spiral broken by gen 234 (thr=0.30–0.65, 25/25 OK)
2. **T110–T115** — entry/intercept phase (already enabled via autoc.ini, intSc ramp working)
3. **T116–T118** — servo slew rate limiting (next: kill bang-bang, model real hardware)
4. **T120–T123** — path shape diversity (after slew works)
5. **T130–T133** — path-relative smoothness (correct formulation, after above works)
6. **T140–T145** — research spikes (after fitness signal is trustworthy)
7. **T150–T153** — CMA-ES (last, needs good gradient)
8. **T160–T164** — polish

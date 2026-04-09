# Feature 022 Development Report

**Branch**: `022-tracking-cone-fitness`
**Date range**: 2026-04-01 to 2026-04-08
**Status**: Shipped. betterz2 (V4 conical, 400 gens) flown 2026-04-07 (see
[flight-report.md](../../flight-results/flight-20260407/flight-report.md)).
Follow-on work captured in [023-ood-and-engage-fixes](../023-ood-and-engage-fixes/spec.md).

## Summary

Replaced the distance-penalty + crash-cliff fitness function with point-accumulation
scoring (ellipsoidal surface + streak multiplier), then discovered and fixed a
~25m Z coordinate bug that had been corrupting fitness signals across the entire
training pipeline. Both fixes shipped together; renderer required a follow-up
to display the new virtual coordinates correctly.

## Phase Timeline

### Phase 1-3: Point-Accumulation Fitness (2026-04-01 to 2026-04-05)

Implemented and shipped before this session:
- 6 `Fit*` config parameters in `autoc.ini`
- New `FitnessComputer` class with Lorentzian scoring surface and streak multiplier
- Simplified `ScenarioScore` (single negated score field)
- Lexicase selection collapsed to one dimension per scenario
- Removed: `DISTANCE_TARGET`, `DISTANCE_NORM`, `DISTANCE_POWER`, `ATTITUDE_*`,
  `CRASH_COMPLETION_WEIGHT`, intercept budget/scale, attitude delta penalty
- 25+ unit tests for scoring surface, streak logic, and end-to-end fitness

### Phase 3b: Coordinate Convention Cleanup (2026-04-06)

#### Discovery

Initial test runs (test1, test2) showed the NN learning to "hug the hard deck"
instead of tracking the path. test2 attempted a fix by switching
`fitness_decomposition.cc` from `getPosition()` to `getVirtualPosition()`.

Deep research revealed the attempted fix was a **no-op**:

- `AircraftState::originOffset_` was NOT serialized in cereal
- CRRCSim stored raw FDM position via `setPosition(p)` and set `originOffset_` at runtime
- After RPC deserialization on the autoc side, `originOffset_` defaulted to zero
- `getVirtualPosition()` returned `position - 0 = position` (raw)
- Fitness scoring was operating on raw coordinates with a ~25m Z offset
- The "hard deck hugging" was the NN finding the closest it could get to virtual Z=0
  (which was actually raw Z=-25) while bounded above by the OOB ceiling (raw Z=-7)

See [coordinate-cleanup-research.md](coordinate-cleanup-research.md) for the full audit.

Additional discoveries:
- CRRCSim and xiao used different conventions for what `setPosition()` stored
  (CRRCSim: raw, xiao: virtual)
- `data.dat` X,Y,Z columns logged raw aircraft positions while `pathX/Y/Z` and
  metric columns were virtual — mixed coordinate frames in one file
- Tests in `fitness_decomposition_tests.cc` were hardcoded with all positions at
  Z=-25, which made tests pass against the broken code (the Z error cancelled
  on both sides)

#### Solution: Virtual-at-Boundary

Convert raw→virtual at the producer boundary (CRRCSim, minisim, xiao), once.
All downstream code sees virtual position via `getPosition()`. Mirrors the
INAV sign convention pattern: convert once at the boundary, standard everywhere
downstream.

Constitution III enforced: no deprecated alias shim. Removed
`getVirtualPosition()` entirely; updated all callers in one cut.

#### Implementation (Phase 3b-i + 3b-ii)

| Change | File |
|--------|------|
| Removed `originOffset_`, `setOriginOffset()`, `getOriginOffset()`, `getVirtualPosition()` | `aircraft_state.h` |
| Added `gp_vec3 originOffset` to `ScenarioMetadata` (cereal-serialized) | `protocol.h` |
| Updated all callers: `getVirtualPosition()` → `getPosition()` | `fitness_decomposition.cc`, `sensor_math.cc`, `evaluator.cc`, `inputdev_autoc.cpp`, `minisim.cc` |
| Added `pathOriginOffset` module-level var; store virtual position; raw for OOB | `inputdev_autoc.cpp` |
| Start at virtual (0,0,0); reconstruct raw for OOB | `minisim.cc` |
| Removed manual offset hack; uses `getPosition()` directly | `autoc.cc` (`logEvalResults()`) |
| Rewrote tests in virtual coordinate space (paths and aircraft at Z=0) | `fitness_decomposition_tests.cc` |
| Added "Virtual Frame" section | `docs/COORDINATE_CONVENTIONS.md` |
| Fixed pre-existing AWS linker issue for test binaries | `CMakeLists.txt` |

Test-first approach validated: rewrote test fixtures in virtual space first,
new `RawPositionGivesWrongScore` test confirms the Z error is dramatic when
present (raw score < 20% of correct score).

All 12 fitness_decomposition tests pass after the fix, including the new
coordinate contract tests (`VirtualOriginPerfectTracking`, `RawPositionGivesWrongScore`,
`ZOffsetTreatedAsLateral`).

#### Phase 3b-iii: Renderer Follow-Up (post-test3 visual)

After Phase 3b-i+ii, test3 launched. Visual inspection of the renderer showed
aircraft "diving to ground level" — apparent regression. Investigation revealed
this was a **renderer artifact**, not a training problem:

- Renderer's `updateGenerationDisplay()` shifted PATHS by `SIM_INITIAL_ALTITUDE`
  for display (path Z=0 → display Z=-25)
- Did NOT shift aircraft positions (which used to be raw, already at Z≈-25)
- After our fix, aircraft positions were now virtual (Z≈0) but renderer still
  treated them as raw
- Result: paths floated 25m above the ground plane while aircraft sat at
  ground level — the visual "dive" was a 25m display offset

Fix applied to `renderer.cc`: shift aircraft positions by `SIM_INITIAL_ALTITUDE`
in `updateGenerationDisplay()` (same as paths and rabbit positions). Verified
in actual training data — aircraft Z distribution centers between virtual 0
(path) and +18 (hard deck), with only 1.3% of steps near the hard deck.

**Lesson**: Phase 3b-iii (renderer) was a hard prerequisite for visual validation,
not optional polish. Should have been batched with 3b-i/3b-ii. The data was
always right; the renderer was lying.

## Test3 Findings (2026-04-06)

### Conversions verified correct

- Aircraft Z values in `data.dat` are virtual coordinates (range ~-39 to +18)
- Z distribution: 41% above path, 57% below path (between path and hard deck),
  1.3% near hard deck
- Streak multiplier formula matches data: `mult = 1 + 4 * count / 25`
- `dist`, `along`, `stpPt` columns all consistent with virtual coordinate computation
- Fitness scores match Python recreation from data.dat columns

### Minisim: solid path tracking

Multi-generation training in minisim showed clean path following with
appropriate Z behavior. Fitness improving over generations. The simpler kinematic
model exposed clean signal.

### CRRCSim: curriculum exploit discovered

CRRCSim test3 metrics showed compression and slow improvement:

```
gen 1:   best=-88,  avgMaxStreak=25.0, pctInStreak=69.8
gen 207: best=-130, avgMaxStreak=25.0, pctInStreak=63.1
```

`avgMaxStreak=25.0` is the cap. Random NNs from gen 1 already saturated streak
diagnostics. Investigation of trajectory data revealed the cause:

- Aircraft starts AT virtual origin (0,0,0) — exactly where the path begins
- CRRCSim's default initial orientation matches path tangent (heading south)
- Initial velocity matches rabbit direction
- For ~50 steps, the aircraft just *exists* in the right place — no learning required
- Streak hits max multiplier at step 47 purely from initial alignment
- First path turn breaks the streak; aircraft can't recover; wanders to hard deck

**Random baseline scores ~88 points just by flying straight south.** Best evolved
NN at gen 207 only reaches ~130 points. 1.5x spread between random and "best" gives
selection no signal to differentiate.

Compounding factors in current `autoc.ini`:
- `EnableEntryVariations = 0` (variations disabled for baseline)
- `EnableWindVariations = 0`
- `RabbitSpeedSigma = 0.0`
- `SimNumPathsPerGeneration = 1` (single path per generation)

This is the test1 entry-exploit issue from spec.md L376-381, but worse: the
aircraft doesn't even need to dive — flying straight is enough.

### Canonical Origin Offset Fix (2026-04-06, post-test3)

Important refinement to the virtual-at-boundary principle: `pathOriginOffset`
must be the **canonical** start position `(0, 0, SIM_INITIAL_ALTITUDE)`, NOT
the actual FDM start position.

**Why**: Entry variations (`EnableEntryVariations=1` with sigmas) intentionally
start the craft *off-target* — north/east/altitude offsets from the canonical
launch point. If `pathOriginOffset` captured the FDM position *including* those
variations, the aircraft would always start at virtual (0,0,0) regardless of
how it was offset. The fitness function would never see the entry deviation.

The variations are supposed to be visible to the fitness as "the aircraft
starts here, the path is over there" — that's what makes them training challenges.

**Fix** in `inputdev_autoc.cpp`:
```cpp
// Canonical (NOT the actual FDM start)
pathOriginOffset = gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);

// Without variations: virtualInitialPos = (0,0,0)
// With variations:    virtualInitialPos = (entryNorth, entryEast, entryAlt)
gp_vec3 virtualInitialPos = initialPos - pathOriginOffset;
```

This is consistent with minisim, which already uses canonical (0,0,0) start
and stores `(0, 0, SIM_INITIAL_ALTITUDE)` as the metadata offset.

### Tuning decision (2026-04-06)

User decision: **`FitStreakRampSec = 2.5 → 5.0`** (and other reverts to be made
by user). Doubles the steps required to reach max multiplier (25 → 50), making
the freebie streak less generous. Alternative levers (entry variations, threshold
ramp) are more powerful but the user wants to test simpler tuning first before
re-enabling the curriculum knobs.

## betterz1 Run Configuration

**Important context for the results below**: betterz1 ran with the **FULL
curriculum enabled** — every variation knob at full strength, not just the
coordinate fix in isolation. This is what the system handled successfully:

| Knob | Value | Effect |
|------|-------|--------|
| `EnableEntryVariations` | 1 | Entry attitude/position variations ON |
| `EnableWindVariations` | 1 | Wind direction variations ON |
| `WindScenarios` | 49 | 49 different wind seeds per path |
| `EntryConeSigma` | 30° | Heading cone half-angle |
| `EntryRollSigma` | 30° | Initial roll variance |
| `EntrySpeedSigma` | 0.1 | 10% speed variance |
| `EntryPositionRadiusSigma` | 15.0m | XY entry position dither |
| `EntryPositionAltSigma` | 3.0m | Z entry position dither |
| `WindDirectionSigma` | 45° | Wind direction variance |
| `RabbitSpeedSigma` | 2.0 m/s | Variable rabbit speed (13±2) |
| `RabbitSpeedCycleMin/Max` | 3-7s | Speed regime switching |
| `VariationRampStep` | 40 | Curriculum ramp on |
| `PathGeneratorMethod` | aeroStandard | 5 specific maneuver paths |
| `SimNumPathsPerGeneration` | 5 | Multi-path per generation |
| `PopulationSize` | 3000 | Large population |
| `FitStreakRampSec` | 5.0 | 50-step ramp to max multiplier (was 2.5) |

Total: **245 scenarios per individual** (5 paths × 49 winds), with entry
attitude/position/speed dither, variable rabbit speed, wind direction variance,
and full curriculum ramp. This is the *hardest* training configuration we have.

**Comparison**:
- **test1** (broken coords, partial curriculum): plateau at best=-3600
- **test2** (broken coords, getVirtualPosition no-op fix): same plateau ~-3600
- **betterz1** (fixed coords + canonical offset + 50-step ramp + FULL curriculum):
  best=-9k at gen 1, -55k at gen 132, still climbing

**~15x improvement under HARDER training conditions.** The previous plateau
was clearly the coordinate bug masking everything; the fitness function was
operating on noise.

## Per-Path Brittleness: 45° Angled Loop is the Outlier

Per-arena analysis of betterz1 (245 scenarios = 5 paths × 49 winds) reveals
that brittleness is concentrated on **one specific path**:

| Path | Index | Arenas | Mean Score | Mean MaxStrk | Crash% |
|------|------:|--------|-----------:|-------------:|-------:|
| StraightAndLevel | 0 | 0-48 | 116.7 | 31.8 | 55.1% |
| SpiralClimb | 1 | 49-97 | 127.0 | 29.8 | 52.6% |
| HorizontalFigureEight | 2 | 98-146 | 93.5 | 25.0 | 58.9% |
| **FortyFiveDegreeAngledLoop** | **3** | **147-195** | **15.6** | **8.0** | **45.0%** |
| HighPerchSplitS | 4 | 196-244 | 93.8 | 27.4 | 58.1% |

**Path 3 averages 7-8x lower score and 3-4x lower streak than the others**,
yet has the *lowest* crash rate (45% vs 55-59%). The aircraft survives but
never builds streak. This is the "rapid loop in place" pattern: the NN finds
a degenerate stable orbit that maintains altitude and bounds but never tracks
the rabbit on this specific path.

**Why FortyFiveDegreeAngledLoop is hard** (`pathgen.h:375-397`):
- 15m radius closed loop in a tilted plane (45° from horizontal)
- 126 waypoints — long enough that getting behind = stay behind
- 3D simultaneously (roll + pitch + yaw all matter)
- Inversion at top of loop = control phase change
- Closed loop = "approach from behind" heuristic fails — rabbit comes back

The other 4 paths are progressively easier:
- **StraightAndLevel** — trivial
- **SpiralClimb** — gentle monotonic 3D
- **HorizontalFigureEight** — planar, horizontal
- **HighPerchSplitS** — single inversion, but 1D in altitude

**Lexicase selection lets brittleness persist**: an individual that wins on
4 out of 5 paths beats one that's mediocre on all 5. With per-scenario
lexicase, the bad path becomes a "skip" rather than a forcing function.

**Most likely root cause: physical infeasibility**. The 45° angled loop is
probably *beyond the performance envelope* of the streamer-loaded model:
- Centripetal: `v²/r ≈ 13²/15 ≈ 1.15g` sustained turn
- Plus bank angle to stay in the tilted plane (additional load factor)
- Plus pitch reversal through inversion (no thrust to climb out of bottom)
- Combined load likely exceeds Cl_max + thrust-to-weight budget

The "rapid loop in place" isn't laziness — it's the NN finding the local
minimum where a tighter, slower bank meets the streak threshold for a few
steps before the rabbit escapes. Following the actual path crashes the craft.

**Implication**: This isn't a fitness function or curriculum bug; it's a path
selection issue. The training set includes a path the craft cannot fly.
Options for future iterations:
- Drop FortyFiveDegreeAngledLoop from `aeroStandard` for this airframe
- Reduce loop radius (smaller loops are *harder*, won't help)
- Reduce loop tilt angle (make it closer to horizontal/vertical, both easier)
- Tag paths by required performance envelope; auto-select feasible ones
- Per-craft path catalogs once the airframe parameters stabilize

## Brittleness Pattern (betterz1 run, gen 132 observation)

During the betterz1 training run with the canonical offset + tuning, evolution
converged strongly past gen 100 (best fitness improving from -9k → -55k). At
gen 132, visual inspection of the renderer showed scenarios 31, 143, and 198
exhibiting "rapid loop in place" behavior — the aircraft entering a tight
control oscillation when it lost track of the rabbit.

Log analysis confirmed this is **per-scenario brittleness**:

- Scenario 31's elite scores across 132 generations cluster around discrete
  values (75.45 ×24, 38.88 ×11, 66.71 ×10, 287.93 ×5, ...) — different elites
  fall into the same local minima on this scenario, deterministically.
- At gen 132 specifically, the new elite went from 287.93 → 1.50 on scenario 31
  (and similar regressions on 143, 198) — overall fitness improved but a few
  scenarios got dramatically worse.
- This is the classic **lexicase brittleness pattern**: a policy that wins on
  most scenarios but catastrophically loses on a few specific path/wind combos.
- The "rapid loop in place" is the NN entering a learned escape oscillation
  that happens to work on training scenarios but produces a degenerate limit
  cycle on these.

**Pattern to watch as we bake**:
- Per-scenario score variance grows with generation — population is over-fitting
  to the easy scenarios
- Specific scenario indices get repeatedly stuck at low scores
- Visual: tight oscillation/loop near where the aircraft last had a streak

**Mitigations to consider** (don't act yet — let it bake):
- Variation ramp may already address this if curriculum widens late
- Lexicase epsilon tuning — too small epsilon lets brittle elites carry forward
- Longer-tail diagnostics: per-scenario score history, not just per-gen avg
- Possibly: anti-oscillation penalty (high-frequency control change rate)

Note: best fitness continued to improve past gen 132. The brittleness is a
sub-population symptom, not a global failure. Worth tracking but not blocking.

## Per-Path Smoothness Analysis (gen 144)

After the strong gen 132→144 improvement (best -54k → -59k, all 4 feasible paths
hitting 0% crash rate and ~49/50 streak), per-path control output analysis
revealed the NN's solution method:

| Path | Steps | RMS dPitch | RMS dRoll | RMS dThrot | Mean\|Pt\| | Mean\|Rl\| | Mean\|Th\| |
|------|------:|-----------:|----------:|-----------:|-----------:|-----------:|-----------:|
| 0 Straight | 6507 | 0.50 | **0.99** | 0.53 | 0.44 | 0.60 | 0.78 |
| 1 Spiral | 8121 | 0.57 | **0.96** | 0.52 | 0.42 | 0.57 | **0.83** |
| 2 Fig8 | 8431 | 0.50 | **0.97** | 0.49 | 0.45 | 0.58 | 0.81 |
| 3 45° Loop | 2884 | 0.47 | 0.95 | 0.35 | 0.43 | 0.54 | **0.88** |
| 4 SplitS | 9828 | 0.49 | **0.98** | 0.46 | 0.44 | 0.58 | 0.84 |

**Bang-bang roll control across all paths**. Roll RMS delta of ~1.0 in a [-1,1]
output range means the NN is swinging roll between extremes nearly every tick.
Pitch and throttle are also high (0.46-0.57 RMS deltas). Mean throttle 0.78-0.88
means the NN is essentially flying at near-max thrust all the time.

This is the local minimum the fitness function naturally rewarded:
- **Position scoring rewards aggressive maneuvering** (more authority = tighter tracking)
- **No penalty for control jitter** (we removed the servo low-pass filter intentionally)
- **No penalty for energy use** (max throttle is "free" in the current scoring)
- **0% crash rate** = the FDM tolerates bang-bang at this scale

### Why smoothness penalty is the wrong tool

Smoothness penalty fights tracking directly. We *want* fast control response —
that's why we dropped the servo LPF. Penalizing rate-of-change would cap the
maximum possible tracking quality and reintroduce the latency we worked to
remove.

### Energy as an indirect smoothness proxy (FUTURE — Phase 5+)

The right insight is that **bang-bang control wastes energy via induced drag**,
even when each individual deflection is "free" servo-wise:
- Bang left then bang right = double the induced drag (each deflection adds
  vortex shedding regardless of direction)
- Bang throttle high then low = wasted spool work + battery cycling
- A *useful* maneuver costs less drag/energy than a *jittery* one that nets
  to the same maneuver

**Proposed second lexicase dimension** (one of two related options):

1. **Throttle integral** (`Σ throttle²`) — captures battery drain. Simple,
   physically meaningful. Most of the joules go to the prop, so this is the
   biggest waste vector. Mean throttle 0.78-0.88 in current run shows lots of
   headroom for improvement.

2. **Integrated control deflection** (`Σ (pitch² + roll²)` over flight) —
   captures induced drag from control surfaces. Bang-bang aileron has the
   same integrated deflection regardless of direction, so this naturally
   penalizes jitter without explicitly measuring rate-of-change. A smooth
   sustained turn costs less than a jittery one with the same average bank.

3. **Combined**: `Σ throttle² + α × Σ (pitch² + roll²)` — both effects.
   But two terms = tuning knob = avoid.

**Recommended approach**: Add as a **second lexicase dimension** (not weighted
sum). Lexicase selection picks individuals on either dimension randomly per
tournament. Population evolves toward the **Pareto frontier** of "good tracking
AND low energy" rather than collapsing to a single weighted sum.

This naturally completes the "second lexicase dimension" item the spec mentions
in clarifications (Session 2026-04-05): *"Mean throttle deferred as a future
second dimension."* — but using integrated deflection or throttle² instead of
mean throttle, since those better capture the bang-bang waste.

### The path-difficulty caveat

The naive integral `Σ deflection²` is **unfair to hard paths**. Different paths
require different amounts of control by their nature:
- A straight line: near-zero deflection
- A figure-eight: continuous bank reversals (legitimate, sustained)
- A 45° angled loop: sustained high deflections in all axes
- A long path: more steps = more accumulated deflection regardless of quality

Without correction, energy penalty would punish individuals for flying hard
paths *correctly*. Same dilemma as the original fitness invariance: don't
penalize hard work, only wasted work.

### Three framings of "wasted" deflection

| Framing | Energy = | Penalizes | Doesn't penalize | Verdict |
|---------|----------|-----------|------------------|---------|
| **A. Subtract path's intrinsic cost** | actual − minimum theoretical | jitter, overshoot | required maneuvers | Theoretically fair, but requires solving optimal control per path. Impossible paths break the "minimum" definition. |
| **B. Energy per fitness point** | `Σ deflection² / score` | high cost per point | high cost in service of tracking | Self-normalizing but score appears in two dimensions; crashes give infinite ratio. |
| **C. Frequency content (high-frequency only)** | `Σ (d²output)²` | reversals, bang-bang | sustained slopes, constant deflection | **Best fit.** |

### Recommended: Second derivative of control output

```
Σ (output[t] − 2*output[t-1] + output[t-2])²
```

This is the **"jerk" of control commands** — the rate-of-change of rate-of-change.

**Worked example** (10 ticks of bank input):

| Pattern | Bank values | Σ\|bank\| | Σ(bank)² | Σ(d²bank)² |
|---------|-------------|----------:|---------:|-----------:|
| Straight | 0×10 | 0 | 0 | **0** |
| Smooth slow turn | 0,.1,.2,.3,.4,.5,.5,.5,.5,.5 | 3.5 | 1.55 | **0.04** |
| Smooth full turn | 0,.2,.4,.6,.8,1,1,1,1,1 | 7.0 | 5.20 | **0.08** |
| Mid bang-bang | .5,-.5,.5,-.5,... | 5 | 2.5 | **20** |
| Full bang-bang | 1,-1,1,-1,... | 10 | 10 | **80** |

The second derivative penalizes a smooth full turn only **2x** more than the
slow turn — but **1000x less** than the chatter. Sustained maneuvers (figure-eight,
loop, hard turns) barely register; chatter dominates. **Path-length and
path-difficulty fair**.

### Combined energy proxy

```
energy = Σ throttle²  +  α × Σ (d²pitch² + d²roll²)
```

- **First term** (`Σ throttle²`): real prop-driven battery drain. Path-fair by
  construction — long flights legitimately use more energy, hard maneuvers
  legitimately need more thrust.
- **Second term** (`Σ d²output²`): high-frequency chatter on control surfaces.
  Path-fair via the second-derivative property — sustained slopes (real
  maneuvers) contribute almost nothing, only reversals do.

The two terms capture different physics (battery drain vs induced drag from
chatter) and don't overlap. Tuning `α` is still a knob to avoid; the cleaner
approach is **two separate lexicase dimensions**:

| Lexicase Dimension | Metric | Lower = better |
|---|---|---|
| 1 | `-points` (current) | More points = better |
| 2 | `Σ throttle²` | Less prop energy = better |
| 3 | `Σ (d²pitch² + d²roll²)` | Smoother control surfaces = better |

Three-dimensional lexicase tournaments would evolve toward the Pareto frontier
of all three.

### When to revisit

**Phase 5+, contingent on**:
1. Current training strategy stabilizes and produces reliable convergence
2. Real flight results validate the bang-bang behavior is or isn't a problem
3. Craft variations come into play (different airframes, CG positions, etc.)

**Why craft variations matter for this**: A craft with CG forward needs more
elevator trim to fly level — a different "baseline" deflection. A craft with
CG aft is more pitch-sensitive — needs *less* deflection per unit response.
Drag-causing variations (loose hatches, streamers, landing gear) shift the
energy budget. Generic energy penalties trained on one airframe may be wrong
for another.

This argues for the **second derivative approach over the integrated approach**:
the second derivative is invariant to baseline trim. A craft that needs +0.3
elevator to fly level has the same `d²pitch` profile as one that needs −0.1.
Only the *changes* matter, not the absolute values. Cross-airframe transfer
is much more likely with a chatter penalty than a magnitude penalty.

**Why this is Phase 5+, not Phase 4**: The current run is already producing
unprecedented tracking quality. Energy optimization is a polish step that
should come *after* the tracking quality stabilizes and we've seen real flight
results. Premature energy optimization could reintroduce tracking weaknesses
we just fixed. May also need to revisit when craft variations enter the
training loop — different airframes have different baseline energy budgets.

## Conical Surface Implementation (V1 → V3 → V4)

The original "tracking cone fitness" feature shipped with an **ellipsoidal**
scoring surface, not a conical one. The name turned out to be aspirational —
we built the simpler shape and the conical version was deferred. After test3
demonstrated the unprecedented evolution quality of the corrected coordinate
pipeline, gen 144 analysis revealed the NN was happy hovering 5m off-axis at
along=0 because the ellipsoidal score there was exactly 0.5 (right at the
streak threshold). The "happy hovering" gradient-free zone was preventing
the NN from learning aggressive tracking corrections.

### The gravity problem

In the ellipsoidal form (`score = 1 / (1 + (along/scale)² + (lat/cross)²)`),
5m left, 5m right, 5m above, and 5m below the rabbit's longitudinal position
all scored identically — a sphere of mediocrity. The NN found a stable orbit
where staying ~5m off-axis met the threshold without needing to actually
track the rabbit. Below-the-rabbit was rewarded the same as above-the-rabbit
which is wrong physically (gravity makes below dangerous: terrain, energy
deficit, harder recovery).

### Iteration journey

**V1 — Pure polar (angular cone)**:
```
distance = sqrt(along² + lateral²)
angle    = acos(-along / distance)        # 0 = behind, π = ahead
score    = 1 / (1 + (dist/distScale)² + (angle/coneAngle)²)
```
- ✓ Fixes the gravity problem: 5m to side at along=0 → angle=π/2, eff_angle² = 4
  with cone=45°, score drops from 0.5 to ~0.18
- ✓ Naturally penalizes ahead via angle
- ✗ **No gradient ahead**: when along > 0, angle ≈ π regardless of distance,
  so eff_angle² saturates at ~16. The distance term (small numbers) gets
  drowned out. +0.5m ahead and +5m ahead score nearly identically (~0.06).
- ✗ NN has no signal to back off if it overshoots — once past the rabbit,
  every position is "uniformly bad" with no recovery gradient.

**V3 — Polar + directional distance**:
```
dist_scale = distScaleBehind  if along <= 0 else distScaleAhead
score = 1 / (1 + (dist/dist_scale)² + (angle/coneAngle)²)
```
- ✓ Adds directional distance asymmetry
- ✗ Same fundamental problem: angle term still saturates ahead, distance term
  still drowned out
- The "directional distance" idea was right, but couldn't take effect because
  the angle term was too dominant

**V4 — Polar + directional distance + clamped angle** (production):
```
distance     = sqrt(along² + lateral²)
angle        = acos(-along / distance)
angle_clamp  = min(angle, π/2)              # ahead saturates at "sideways"
dist_scale   = distScaleBehind  if along <= 0 else distScaleAhead
score        = 1 / (1 + (dist/dist_scale)² + (angle_clamp/coneAngle)²)
```
- ✓ Gravity problem fixed (lateral pinched as in V1)
- ✓ Real ahead gradient (small distScaleAhead carries the slope)
- ✓ Tail-chase on-axis behavior preserved (angle = 0 → distance only)
- ✓ The angle clamp at π/2 prevents ahead saturation: once past the rabbit,
  the angle term stops growing and the directional distance term dominates

The clamp is the key insight. Instead of letting "ahead" mean "angle = π
which crushes everything," we say "ahead is as bad as sideways from an
angle perspective; the distance penalty is what tells you HOW bad."

### Production parameters

```
FitDistScaleBehind  = 7.0   m   # -7m back = 0.5 threshold (matches user expectation)
FitDistScaleAhead   = 2.0   m   # sharp ahead penalty
FitConeAngleDeg     = 45.0  deg # 45° half-angle cone
```

### Test point comparison

| Position | Old ellipsoid | V1 polar | V4 (production) |
|----------|--------------:|---------:|----------------:|
| At rabbit | 1.000 | 1.000 | 1.000 |
| -3m back on-axis | 0.917 | 0.917 | 0.845 |
| -5m back on-axis | 0.800 | 0.800 | 0.662 |
| **-7m back on-axis** | **0.500** | 0.331 | **0.500** ★ threshold |
| -10m back on-axis | 0.293 | 0.169 | 0.329 |
| -14m back on-axis | 0.165 | n/a | 0.200 |
| **5m to side at along=0** | **0.500** | 0.181 | **0.182** (gravity fix) |
| 10m to side at along=0 | 0.200 | 0.142 | 0.142 |
| **+0.5m ahead** | 0.5 | **0.058** | **0.198** (gradient!) |
| +1m ahead | 0.200 | 0.058 | 0.190 |
| **+5m ahead** | 0.001 | 0.043 | **0.089** (much worse) |
| **+10m ahead** | n/a | 0.024 | **0.033** (catastrophic) |
| -5m back, 5m side (45° cone-edge) | 0.300 | 0.331 | 0.331 |
| -7m back, 7m side (45°) | n/a | n/a | 0.250 |

Key wins:
- **-7m back** still hits 0.5 exactly (matches the original target)
- **5m off-axis** drops from 0.5 (exactly at threshold) to 0.18 (firmly out)
- **+0.5m ahead** has real signal (0.198) instead of being lost in saturation (0.058)
- **+5m ahead vs +0.5m ahead** is now 2.2x penalty difference (was no difference)

### Visualization

See [scoring_surface.png](scoring_surface.png) for the production V4 surface
and [scoring_surface_cone_angles.png](scoring_surface_cone_angles.png) for
the cone-angle tuning sweep (20°, 30°, 45°, 60°, 75°, 90°).

The V4 surface shows:
- Bowtie/teardrop streak threshold contour swept back from the rabbit
- Concentric circles centered near the rabbit on the ahead side (the angle
  clamp at work — distance dominates ahead)
- Wide forgiving zone behind extending to ~14m at threshold
- Sharp boundary at along=0 — moving from -ε to +ε is a step change in dist_scale

### Implementation details

The clean cut from ellipsoidal V0 → polar V4 went through both formulas
without an intermediate stop. The V1 polar form was visualized in Python and
briefly committed before V3/V4 superseded it. The constructor signature
changed twice:
- V0: `FitnessComputer(behindScale, aheadScale, crossScale, ...)`
- V1: `FitnessComputer(distScale, coneAngleDeg, ...)`
- V4: `FitnessComputer(distScaleBehind, distScaleAhead, coneAngleDeg, ...)`

Each was a clean cut (Constitution III): no shims, all callers updated in
one commit. The Python sandbox `scoring_surface_explore.py` is preserved
as documentation of the design exploration.

## Acceptance Criteria Status

| Criterion | Status |
|-----------|--------|
| **A**: Aircraft Z distribution centers on path altitude (~Z=0 virtual), NOT hard deck | **PASS** — only 1.3% of steps near hard deck; 41% above path, 57% in path-to-deck region. The "hard deck hugging" symptom is gone. |
| **B**: Fitness scores increase significantly vs test2 | **Partial** — best score went from "stuck at -3600" (test2) to actively evolving (-88 → -130 in 200 gens). However, evolution is slow due to the curriculum exploit, not the coordinate fix. |
| Throughput regression check | Not measured yet — defer to Phase 4 polish |
| Build stability | **PASS** — clean rebuild, all 9 test suites passing (96 tests total) |
| Constitution III (no shims) | **PASS** — getVirtualPosition removed, no deprecated alias |

## Outstanding Work

### Phase 3b-iii (closed)
- [X] T035: Renderer aircraft position shift (shipped 7882550)
- [X] T036: Verify xiao modes unchanged (validated by 2026-04-07 flight rendering)

### Phase 4: Polish & Validation (closed)
- [X] T020-T021: test3/betterz* monitoring, convergence verified through gen 400
- [X] T022: data.dat per-step diagnostic columns (along, stpPt, mult) in place
- [X] T023: Dead code removed (no DISTANCE_TARGET/computeStepPenalty refs remain)
- [📁] T024: Streak threshold ramp — DEFERRED to BACKLOG.md (betterz2 converged without it)
- [X] T024b: `EnableRabbitSpeedVariations` flag added and verified (flag=0 forces sigma=0)
- [X] T025: Final wrap-up commit

### Curriculum Re-Enable Plan

User-defined progression order for re-introducing variations:

| Step | Config | Goal |
|------|--------|------|
| 1 (current) | 1 path, no variations, FitStreakRampSec=5 | Verify simple tuning breaks the freebie |
| 2 | 5-6 paths, no variations | Multi-path origin offset reset works |
| 3 | + EntryConeSigma | Direction variation |
| 4 | + EntryPositionRadiusSigma + AltSigma | Position variation tests virtual origin shift |
| 5 | + RabbitSpeed variation (with new EnableRabbitSpeedVariations bool) | Speed variation |
| 6 | + Wind | Full curriculum |

## Key Lessons

1. **Test the contract, not the bug.** Tests that hardcoded Z=-25 made the
   broken code "pass" because the bug cancelled on both sides. Test fixtures
   should encode the *correct* coordinate convention, not match whatever the
   code happens to do.

2. **Coordinate frames are a system-wide concern.** The bug touched: producer
   (CRRCSim/xiao), serialization (cereal), consumers (fitness, sensors,
   evaluator), logging (data.dat), and renderer. Phase 3b-iii (renderer)
   should not have been split off — visual validation requires it.

3. **Constitution III matters.** Initial plan included a deprecated
   `getVirtualPosition()` alias for "phased rollout." Removed in favor of a
   clean cut. The compiler became a tool for finding all the callers in one
   pass, with zero risk of stragglers.

4. **Symmetry breaks selection signal.** When the aircraft starts at the rabbit
   with aligned heading, "doing nothing" scores well. Random NNs hit 70% streak.
   Selection has no gradient. Curriculum variations aren't optional features —
   they're prerequisites for meaningful evolution. The test1 finding (entry
   exploit) proved this; we re-discovered it because variations were off for
   the baseline.

5. **Renderer is part of the system.** "Looks broken in the renderer" is a
   first-class signal. The user's "100% diving to minimum elevation" report was
   accurate about the visual but wrong about the code — we needed to investigate
   the render pipeline, not just the training pipeline.

6. **Hand-tuned knobs are a smell.** User noted aversion to manual scale tuning.
   Long-term direction is fewer parameters with adaptive behavior (e.g., streak
   threshold ramp tied to variation scale, scoring surface adapting to path
   curvature).

## Files Touched (Phase 3b)

```
include/autoc/eval/aircraft_state.h      — removed originOffset machinery
include/autoc/rpc/protocol.h              — added originOffset to ScenarioMetadata
src/eval/fitness_decomposition.cc         — getVirtualPosition→getPosition
src/eval/sensor_math.cc                   — getVirtualPosition→getPosition + comments
src/nn/evaluator.cc                       — getVirtualPosition→getPosition
src/autoc.cc                              — removed manual offset hack
crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp
                                          — pathOriginOffset, virtual store, raw OOB, metadata
tools/minisim.cc                          — virtual start, raw OOB reconstruction, metadata
tools/renderer.cc                         — shift aircraft positions for display (T035)
tests/fitness_decomposition_tests.cc      — rewrote in virtual coordinate space
CMakeLists.txt                            — fixed pre-existing AWS linker issue for tests
docs/COORDINATE_CONVENTIONS.md            — added Virtual Frame section
specs/022-tracking-cone-fitness/coordinate-cleanup-research.md  — full audit
specs/022-tracking-cone-fitness/spec.md   — observations, clarifications
specs/022-tracking-cone-fitness/plan.md   — Phase 3b sub-phases
specs/022-tracking-cone-fitness/tasks.md  — T029-T038b
```

## References

- [spec.md](spec.md) — feature specification with clarifications
- [plan.md](plan.md) — implementation plan and phases
- [tasks.md](tasks.md) — task breakdown and execution order
- [coordinate-cleanup-research.md](coordinate-cleanup-research.md) — full coordinate system audit
- [docs/COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md) — virtual frame documentation

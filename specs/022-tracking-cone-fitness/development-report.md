# Feature 022 Development Report

**Branch**: `022-tracking-cone-fitness`
**Date range**: 2026-04-01 to 2026-04-06
**Status**: Phase 3b complete (coordinate cleanup landed). Phase 4 (tuning + validation) in progress.

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

## Acceptance Criteria Status

| Criterion | Status |
|-----------|--------|
| **A**: Aircraft Z distribution centers on path altitude (~Z=0 virtual), NOT hard deck | **PASS** — only 1.3% of steps near hard deck; 41% above path, 57% in path-to-deck region. The "hard deck hugging" symptom is gone. |
| **B**: Fitness scores increase significantly vs test2 | **Partial** — best score went from "stuck at -3600" (test2) to actively evolving (-88 → -130 in 200 gens). However, evolution is slow due to the curriculum exploit, not the coordinate fix. |
| Throughput regression check | Not measured yet — defer to Phase 4 polish |
| Build stability | **PASS** — clean rebuild, all 9 test suites passing (96 tests total) |
| Constitution III (no shims) | **PASS** — getVirtualPosition removed, no deprecated alias |

## Outstanding Work

### Phase 3b-iii (in progress)
- [X] T035: Renderer aircraft position shift (DONE this session)
- [ ] T036: Verify xiao modes unchanged (xiao has zero references to originOffset; just needs visual spot-check)

### Phase 4: Polish & Validation
- [ ] T020-T021: Continue test3 monitoring
- [ ] T022: Update data.dat per-step diagnostic columns (along, stpPt, mult — already partially done in current logging)
- [ ] T023: Remove dead code (any remaining DISTANCE_TARGET refs)
- [ ] T024: Streak threshold ramp (from spec proposal — defer until simpler tuning evaluated)
- [ ] T024b: Add `EnableRabbitSpeedVariations` bool (queued — matches existing `EnableEntryVariations`/`EnableWindVariations` pattern)
- [ ] T025: Final commit

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

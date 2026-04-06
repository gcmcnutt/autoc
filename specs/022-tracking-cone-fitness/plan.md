# Implementation Plan: 022 — Point-Accumulation Fitness

**Branch**: `022-tracking-cone-fitness` | **Date**: 2026-04-06 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/022-tracking-cone-fitness/spec.md`

## Summary

Replace the current distance-penalty + crash-cliff fitness function with a point-accumulation
model using an ellipsoidal scoring surface and streak multiplier. Also fix the raw/virtual
coordinate bug that caused fitness to operate on raw positions (~25m Z error), corrupting
the training signal (hard-deck-hugging behavior). No NN topology or xiao changes required.
CRRCSim and minisim changes are limited to the coordinate boundary (store virtual position).

## Technical Context

**Language/Version**: C++17
**Primary Dependencies**: Eigen (vec3/dot), inih (config parsing), cereal (serialization), GoogleTest
**Storage**: File-based (autoc.ini, data.dat, data.stc)
**Testing**: GoogleTest — `build/fitness_computer_tests`
**Target Platform**: Linux (training), no embedded changes
**Project Type**: Neuroevolution training engine
**Performance Goals**: No regression in eval throughput (~16K sims/sec)
**Constraints**: Fitness computation is per-step in inner loop — must be lightweight (no sqrt, no trig)
**Scale/Scope**: ~12 files changed, ~300 lines of code (fitness + coordinate cleanup)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Testing-First | PASS | New tests for computeStepScore, streak logic, negation. Old penalty tests removed. |
| Build Stability | PASS | `scripts/rebuild.sh` must pass. No xiao changes. CRRCSim/minisim: coordinate boundary only. |
| No Compatibility Shims | PASS | Clean replacement — old fitness constants removed, not shimmed. No backwards compat for old serialized data — fix forward. |
| Unified Build | PASS | No new dependencies. Same CMakeLists.txt. |

## Project Structure

### Documentation (this feature)

```text
specs/022-tracking-cone-fitness/
├── spec.md
├── plan.md                          # This file
├── research.md                      # Phase 0 output
├── coordinate-cleanup-research.md   # Phase 3b research — raw/virtual audit
├── data-model.md                    # Phase 1 output
├── scoring_surface.png              # Visualization
└── tasks.md                         # Phase 2 output (from /speckit.tasks)
```

### Source Code (files touched)

```text
include/autoc/
├── autoc.h                      # Remove old fitness constants only
├── eval/
│   ├── fitness_computer.h       # New computeStepScore(), remove old methods
│   ├── fitness_decomposition.h  # Simplified ScenarioScore (single score field)
│   └── aircraft_state.h         # Remove originOffset_, getVirtualPosition() (Phase 3b)
├── rpc/
│   └── protocol.h               # Add originOffset to ScenarioMetadata (Phase 3b)
├── util/
│   └── config.h                 # Add Fit* fields to AutocConfig (with defaults)

src/
├── autoc.cc                     # Wire computeScenarioScores(), update logging, remove offset hack
├── eval/
│   ├── fitness_computer.cc      # New scoring surface + streak implementation
│   ├── fitness_decomposition.cc # Rewrite: along/cross decomposition, point accumulation
│   ├── sensor_math.cc           # getVirtualPosition→getPosition (Phase 3b)
│   └── selection.cc             # Simplify lexicase to single dimension
├── nn/
│   └── evaluator.cc             # getVirtualPosition→getPosition (Phase 3b)
├── util/
│   └── config.cc                # Parse Fit* from autoc.ini

crrcsim/src/mod_inputdev/inputdev_autoc/
└── inputdev_autoc.cpp           # Store virtual position at boundary, raw for OOB (Phase 3b)

tools/
├── minisim.cc                   # Virtual start, raw OOB reconstruction (Phase 3b)
└── renderer.cc                  # Handle virtual aircraft positions from eval (Phase 3b)

tests/
├── fitness_computer_tests.cc    # New tests for scoring surface + streak
├── fitness_decomposition_tests.cc # Adapted for new ScenarioScore
└── selection_tests.cc           # Adapted for single-dimension lexicase

docs/
└── COORDINATE_CONVENTIONS.md    # Add Virtual Frame section (Phase 3b)

autoc.ini                        # Add Fit* parameters
```

**Structure Decision**: Single project, existing directory layout. No new directories needed.

## Complexity Tracking

No constitution violations. No complexity justification needed.

## Phase 0: Research

Original fitness design decisions resolved in spec clarifications. Coordinate convention
research added post-initial-plan — see Phase 3b and [coordinate-cleanup-research.md](coordinate-cleanup-research.md).
Resolved decisions:
- Aggregation: lexicase (unchanged)
- Attitude penalty: removed
- Intercept budget: removed
- Path end tangent: reuse previous
- Variation ramp: kept as-is
- Fitness direction: negate score

### research.md

See [research.md](research.md) (generated below).

## Phase 1: Design

### Data Model

**New config fields** (added to `AutocConfig`):

| Field | Type | ini key | Default |
|-------|------|---------|---------|
| fitBehindScale | double | FitBehindScale | 7.0 |
| fitAheadScale | double | FitAheadScale | 0.5 |
| fitCrossScale | double | FitCrossScale | 5.0 |
| fitStreakThreshold | double | FitStreakThreshold | 0.5 |
| fitStreakRampSec | double | FitStreakRampSec | 2.5 |
| fitStreakMultiplierMax | double | FitStreakMultiplierMax | 5.0 |

**Derived at startup** (not config):
- `streakStepsToMax = fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0)`

**Removed constants** from `autoc.h`:
- DISTANCE_TARGET, DISTANCE_NORM, DISTANCE_POWER
- ATTITUDE_NORM, ATTITUDE_POWER
- CRASH_COMPLETION_WEIGHT
- INTERCEPT_SCALE_FLOOR, INTERCEPT_SCALE_CEILING, INTERCEPT_BUDGET_MAX, INTERCEPT_TURN_RATE

**New FitnessComputer interface**:

```cpp
class FitnessComputer {
public:
    FitnessComputer(double behindScale, double aheadScale, double crossScale,
                    double streakThreshold, int streakStepsToMax, double streakMultMax);

    // Compute step score from along-track and cross-track distances
    double computeStepScore(double along, double lateralDist) const;

    // Update streak state and return multiplied score
    double applyStreak(double stepPoints);

    // Reset streak (call at start of each scenario)
    void resetStreak();

    // Diagnostics
    int getMaxStreak() const;
    int getStreakSteps() const;  // total steps in streak
    double getMaxMultiplier() const;

private:
    double behindScale_, aheadScale_, crossScale_;
    double streakThreshold_;
    int streakStepsToMax_;
    double streakMultMax_;

    // Per-scenario state
    int streakCount_ = 0;
    int maxStreak_ = 0;
    int totalStreakSteps_ = 0;
    double maxMultiplier_ = 1.0;
};
```

### Contracts

No external interfaces. Internal API change only (FitnessComputer class).

### Implementation Order

1. **autoc.ini + config.h/.cc**: Add 6 Fit* parameters
2. **fitness_computer.h/.cc**: New class with computeStepScore(), streak logic
3. **fitness_computer_tests.cc**: Tests for scoring surface, streak, edge cases
4. **autoc.h**: Remove old constants
5. **fitness_decomposition.h/.cc**: Replace ScenarioScore — single `score` field (negated accumulated points). Remove completion_fraction, distance_rmse, attitude_error, legacy_* fields.
6. **selection.cc**: Simplify lexicase to single dimension per scenario (`-score`, lower is better). Remove 3-phase cascade (completion → distance → throttle).
7. **autoc.cc computeNNFitness()**: Replace inner loop — decompose offset into along/cross, call new fitness, negate total, remove crash penalty and intercept budget
8. **autoc.cc logging**: Add streak diagnostics to per-scenario summary and data.stc
9. **Build + verify**: `scripts/rebuild.sh`, run tests, start training run

## Phase 3b: Coordinate Convention Cleanup

**Rationale**: Deep research (see [coordinate-cleanup-research.md](coordinate-cleanup-research.md))
revealed that `originOffset_` is not serialized, causing `getVirtualPosition()` to
return raw position on the autoc side. CRRCSim and xiao use different conventions
for what `setPosition()` stores. This must be fixed for correct fitness scoring and
consistent logging.

**Principle**: Convert raw→virtual at the producer boundary, once. All downstream
code sees virtual position via `getPosition()`.

**Test-first**: Existing tests encode the bug (all positions at Z=-25 raw). Rewrite
tests to express the virtual-space contract BEFORE fixing code. Tests fail against
current code (proving the bug), pass after the fix.

**Ordering**: Internal + minisim first (prove the fix), then CRRCSim (real sim),
then renderer. Xiao already stores virtual — no changes needed.

### Phase 3b-i: Tests + Internal Framework + Minisim

1. **T029: Rewrite tests** — paths at Z=0, aircraft at Z=0, no setOriginOffset().
   Tests FAIL against current code → proves the coordinate bug exists.

2. **T030: aircraft_state.h** — Clean-cut remove `originOffset_`, `setOriginOffset()`,
   `getOriginOffset()`, `getVirtualPosition()`. No deprecated alias (Constitution III).
   Compile breaks at all callers → forces updating everything in one cut.

3. **T031: protocol.h** — Add `gp_vec3 originOffset` to `ScenarioMetadata`. Serialize via cereal.

4. **T033: ALL callers** — `getVirtualPosition()` → `getPosition()` in one cut:
   fitness_decomposition.cc, sensor_math.cc, evaluator.cc, inputdev_autoc.cpp, minisim.cc.

5. **T034: autoc.cc** — Remove manual offset hack. Use `getPosition()` directly.

6. **T033b: minisim.cc** — Start at virtual (0,0,0). Reconstruct raw for OOB.
   **Critical**: without this, immediate OOB crash on first tick.

7. **T037: COORDINATE_CONVENTIONS.md** — Add "Virtual Frame" section (parallel).

8. **T038a: Validate** — Minisim training shows aircraft Z≈0 (not hard deck).

### Phase 3b-ii: CRRCSim

9. **T032: inputdev_autoc.cpp** — Store virtual position at boundary. Raw for OOB.
   Module-level `pathOriginOffset`. Store offset in ScenarioMetadata.
   (CRRCSim callers already updated in T033.)

10. **T038b: Validate** — Fresh test3 run. Acceptance: Z distribution + fitness uplift.

### Phase 3b-iii: Renderer

12. **T035: renderer.cc** — Add `SIM_INITIAL_ALTITUDE` to aircraft positions from eval
    (same as paths). Update all-flight mode to use originOffset from ScenarioMetadata.

13. **T036: Verify xiao modes** — Already virtual, zero code changes. Spot-check display.

## Implementation Strategy

### MVP (Phase 1-3b)

1. Add config params (Phase 1)
2. Build + test new FitnessComputer in isolation (Phase 2)
3. Wire into evolution pipeline (Phase 3)
4. Fix coordinate convention — virtual-at-boundary (Phase 3b)
5. **VALIDATE**: Fresh test3 training run with new random population. Test1/test2 populations
   are poisoned by the Z-error policy (hard-deck-hugging). Acceptance: aircraft Z centers
   on path altitude, not hard deck; fitness scores increase significantly.

### Key Risks

1. Local minimum where streak on partial course dominates. Monitor `pctInStreak` in
   diagnostics — should increase over generations, not plateau early.
2. Coordinate fix changes OOB behavior in minisim — must reconstruct raw for OOB check
   or immediate crash on first tick (virtual Z=0 > SIM_MIN_ELEVATION of -7).
3. Renderer needs matching coordinate update — aircraft positions shift from raw to virtual.
   Training playback will show aircraft at Z=0 instead of Z=-25 if renderer isn't updated.

### Backwards Compatibility

None. Fix forward only. Old serialized EvalResults, data.dat files, and trained populations
from test1/test2 are invalidated by this change. No cereal version shims needed.

## Notes

- Constitution requires tests — T004/T005 before T006/T007
- CRRCSim changes limited to coordinate boundary cleanup (Phase 3b) — no NN topology changes
- Existing variation ramp (VariationRampStep) unchanged
- Path tangent at last waypoint: reuse previous segment tangent
- Fitness direction: negate score so lower = better (existing convention)
- Phase 3b is the critical correctness fix — fitness scoring has been using raw positions (~25m Z error)
- NED/body frame conventions verified correct across CRRCSim, minisim, and xiao — no changes needed

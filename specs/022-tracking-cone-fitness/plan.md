# Implementation Plan: 022 — Point-Accumulation Fitness

**Branch**: `022-tracking-cone-fitness` | **Date**: 2026-04-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/022-tracking-cone-fitness/spec.md`

## Summary

Replace the current distance-penalty + crash-cliff fitness function with a point-accumulation
model using an ellipsoidal scoring surface and streak multiplier. The change is confined to the
fitness computation — no NN topology, CRRCSim, or xiao changes required.

## Technical Context

**Language/Version**: C++17
**Primary Dependencies**: Eigen (vec3/dot), inih (config parsing), cereal (serialization), GoogleTest
**Storage**: File-based (autoc.ini, data.dat, data.stc)
**Testing**: GoogleTest — `build/fitness_computer_tests`
**Target Platform**: Linux (training), no embedded changes
**Project Type**: Neuroevolution training engine
**Performance Goals**: No regression in eval throughput (~16K sims/sec)
**Constraints**: Fitness computation is per-step in inner loop — must be lightweight (no sqrt, no trig)
**Scale/Scope**: ~6 files changed, ~200 lines of code

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Testing-First | PASS | New tests for computeStepScore, streak logic, negation. Old penalty tests removed. |
| Build Stability | PASS | `scripts/rebuild.sh` must pass. No xiao changes needed. |
| No Compatibility Shims | PASS | Clean replacement — old fitness constants removed, not shimmed. |
| Unified Build | PASS | No new dependencies. Same CMakeLists.txt. |

## Project Structure

### Documentation (this feature)

```text
specs/022-tracking-cone-fitness/
├── spec.md
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── scoring_surface.png  # Visualization
└── tasks.md             # Phase 2 output (from /speckit.tasks)
```

### Source Code (files touched)

```text
include/autoc/
├── autoc.h                      # Remove old fitness constants only
├── eval/
│   ├── fitness_computer.h       # New computeStepScore(), remove old methods
│   ├── fitness_decomposition.h  # Simplified ScenarioScore (single score field)
│   └── aircraft_state.h         # SIM_TIME_STEP_MSEC (read-only, unchanged)
├── util/
│   └── config.h                 # Add Fit* fields to AutocConfig (with defaults)

src/
├── autoc.cc                     # Wire computeScenarioScores(), update logging
├── eval/
│   ├── fitness_computer.cc      # New scoring surface + streak implementation
│   ├── fitness_decomposition.cc # Rewrite: along/cross decomposition, point accumulation
│   └── selection.cc             # Simplify lexicase to single dimension
├── util/
│   └── config.cc                # Parse Fit* from autoc.ini

tests/
├── fitness_computer_tests.cc    # New tests for scoring surface + streak
├── fitness_decomposition_tests.cc # Adapted for new ScenarioScore
└── selection_tests.cc           # Adapted for single-dimension lexicase

autoc.ini                        # Add Fit* parameters
```

**Structure Decision**: Single project, existing directory layout. No new directories needed.

## Complexity Tracking

No constitution violations. No complexity justification needed.

## Phase 0: Research

No unknowns requiring research. All design decisions resolved in spec clarifications:
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
| fitBehindScale | double | FitBehindScale | 10.0 |
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

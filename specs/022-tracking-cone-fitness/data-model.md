# Data Model: 022 — Point-Accumulation Fitness

## New Config Fields (AutocConfig)

| Field | Type | ini key | Default | Description |
|-------|------|---------|---------|-------------|
| fitBehindScale | double | FitBehindScale | 10.0 | Along-track half-decay distance when behind rabbit (m) |
| fitAheadScale | double | FitAheadScale | 0.5 | Along-track half-decay distance when ahead of rabbit (m) |
| fitCrossScale | double | FitCrossScale | 5.0 | Cross-track half-decay distance (m) |
| fitStreakThreshold | double | FitStreakThreshold | 0.5 | Min step_points to maintain streak |
| fitStreakRampSec | double | FitStreakRampSec | 2.5 | Seconds of sustained tracking to reach max multiplier |
| fitStreakMultiplierMax | double | FitStreakMultiplierMax | 5.0 | Maximum streak multiplier |

## Derived Values (computed at startup)

| Value | Formula | Example |
|-------|---------|---------|
| streakStepsToMax | fitStreakRampSec / (SIM_TIME_STEP_MSEC / 1000.0) | 2.5 / 0.1 = 25 |

## Removed Constants (from autoc.h)

| Constant | Old Value | Reason |
|----------|-----------|--------|
| DISTANCE_TARGET | 1.0 | Optimal is now 0m (at rabbit) |
| DISTANCE_NORM | 5.0 | Replaced by directional scales |
| DISTANCE_POWER | 1.5 | Replaced by Lorentzian decay |
| ATTITUDE_NORM | 0.349 | Attitude penalty removed |
| ATTITUDE_POWER | 1.5 | Attitude penalty removed |
| CRASH_COMPLETION_WEIGHT | 1e6 | No crash penalty |
| INTERCEPT_SCALE_FLOOR | 0.1 | Intercept budget removed |
| INTERCEPT_SCALE_CEILING | 1.0 | Intercept budget removed |
| INTERCEPT_BUDGET_MAX | 15.0 | Intercept budget removed |
| INTERCEPT_TURN_RATE | π/4 | Intercept budget removed |

## Removed from fitness_computer.h

| Duplicated constant | Notes |
|---------------------|-------|
| DISTANCE_TARGET (ifndef guard) | Was duplicated from autoc.h |
| DISTANCE_NORM, DISTANCE_POWER | Same |
| ATTITUDE_NORM, ATTITUDE_POWER | Same |
| CRASH_COMPLETION_WEIGHT | Same |
| INTERCEPT_SCALE_* | Same |

## Per-Scenario State (FitnessComputer instance)

| Field | Type | Reset | Description |
|-------|------|-------|-------------|
| streakCount | int | Per scenario | Current consecutive on-track steps |
| maxStreak | int | Per scenario | Longest streak in this scenario |
| totalStreakSteps | int | Per scenario | Total steps with score >= threshold |
| maxMultiplier | double | Per scenario | Highest multiplier reached |

## Logging Additions

### Per-scenario summary line (existing format, new fields appended)

```
[scn] OK/CRASH comp=... score=... maxStrk=... strkPct=... maxMult=...
```

### data.stc per-generation line (new fields appended)

```
#NNGen gen=N best=F ... bestScore=S avgMaxStreak=K pctInStreak=P%
```

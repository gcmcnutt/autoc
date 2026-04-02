# 022 — Tracking Cone Fitness Function

**Status**: Draft
**Priority**: P1 — next training cycle improvement
**Created**: 2026-04-01

## Problem

The current fitness function uses flat Euclidean distance to a target offset
(DISTANCE_TARGET) with a power-law penalty. This has several weaknesses:

- **Symmetric**: 10m above and 10m below score the same. Below is dangerous.
- **No bearing awareness**: 10m dead ahead on the path tangent scores the same
  as 10m perpendicular. Ahead-on-path is much better for tracking.
- **No sustained tracking reward**: a lucky 2m flyby scores as well as 10
  seconds of steady 5m tracking. The NN has no incentive for lock-on.
- **Crash is catastrophic**: a 10s perfect track followed by a crash scores
  worse than 20s of lazy 15m orbiting. But the 10s track was actually better.
- **Bang-bang friendly**: no penalty for jerky control that achieves low
  distance error.
- **Distance target offset**: the 7.5m (now 1.0m) target creates an
  artificial orbit distance rather than rewarding being on the path.

## Solution: Tracking Cone with Combo Multiplier

Replace the distance-based fitness with a pursuit/tracking model inspired by
missile guidance and video game scoring:

### Core Concept

The rabbit moves along the path. The aircraft should be in a **trailing cone**
behind the rabbit, aligned with the path tangent. The score rewards:

1. **Alignment** — body heading aligned with path-to-rabbit vector
2. **Proximity** — closer is better, non-linear decay
3. **Asymmetric bias** — above > below, behind > ahead, inside turn > outside
4. **Combo multiplier** — sustained in-cone tracking builds a multiplier
5. **Break penalty** — losing track costs proportional to combo length
6. **Lead cone** — score against future rabbit positions, not just current

### Flight Phases

```
Intercept:  Entry → approaching rabbit
            Score: alignment improving, distance decreasing
            Low penalty — working toward lock-on

Lock-on:    Entering the tracking cone, combo starts
            Score: combo multiplier begins building
            High reward rate — golden zone

Track:      Sustained in-cone
            Score: multiplier climbing, best reward per step
            Longer = more valuable per step

Break:      Left the cone
            Penalty: combo resets + break penalty ∝ lost combo length
            Worse than never having been on track

Re-intercept: Back toward cone after break
              Same as intercept but time burned

Crash:      Permanent break. Penalty = remaining steps × expected combo value
```

### Step Score Computation

```
// 1. Bearing alignment: aircraft-to-rabbit vs path tangent
alignment = dot(normalize(rabbit - aircraft), path_tangent)  // [-1, 1]
alignment_score = max(0, alignment)  // 0 if perpendicular or ahead

// 2. Proximity: smooth distance decay
proximity = 1.0 / (1.0 + (dist / proximity_scale)^2)

// 3. Asymmetric bias in path-relative frame
error_along = dot(aircraft - rabbit, path_tangent)  // + = ahead (bad)
error_up = -dot(aircraft - rabbit, gravity_dir)      // + = above (good)
asym = 1.0 + above_bonus * error_up/dist
           - ahead_penalty * max(0, error_along)/dist

// 4. Multi-cone leading: weight current + future rabbit positions
cone_now  = alignment_score * proximity * clamp(asym, 0.5, 1.5)
cone_01s  = cone(rabbit_+0.1s, tangent_+0.1s)  // near lead
cone_05s  = cone(rabbit_+0.5s, tangent_+0.5s)  // far lead (turn anticipation)
step_base = w0 * cone_now + w1 * cone_01s + w2 * cone_05s

// 5. Combo multiplier
if dist < combo_threshold:
    combo_count++
    multiplier = 1.0 + combo_count * combo_growth
else:
    if combo_count > 0:
        break_penalty += combo_count * break_cost  // losing track hurts
    combo_count = 0
    multiplier = 1.0

step_score = step_base * multiplier
```

### Scenario Fitness

```
scenario_score = Σ step_score - break_penalty

// Crash: permanent break, penalize remaining potential
if crashed:
    remaining_steps = total_steps - crashed_at
    scenario_score -= remaining_steps * expected_step_value

// Higher is better (invert for minimization if needed)
total_fitness = -Σ scenario_score  // minimize negative score
```

### Tunable Parameters

| Parameter | Description | Starting value |
|-----------|-------------|---------------|
| proximity_scale | Distance for 50% proximity score | 5.0m |
| above_bonus | Reward for being above path | 0.1 |
| ahead_penalty | Penalty for being ahead of rabbit | 0.2 |
| w0, w1, w2 | Cone weights (now, +0.1s, +0.5s) | 0.5, 0.3, 0.2 |
| combo_threshold | Distance for combo to build | 5.0m |
| combo_growth | Multiplier increment per step | 0.02 (2% per 100ms) |
| break_cost | Penalty per lost combo step | 0.5 |
| expected_step_value | Crash remaining-step penalty | 1.0 |

### Selection Strategy

With a well-shaped scalar fitness per scenario, lexicase may no longer be
needed. Options:

- **Sum**: total fitness across all scenarios. Simple, strong gradient.
- **Tournament**: rank-based selection on sum fitness. Standard EA approach.
- **Lexicase on scenario scores**: keep per-scenario pressure but with
  single-dimension cone score instead of multi-phase filtering.

Start with sum across scenarios, compare to lexicase.

## Available Inputs

The NN already receives everything needed to compute cone scores:

- **dPhi/dTheta**: bearing to rabbit in body frame (current + forecast)
- **dist**: distance to rabbit (current + forecast)
- **quaternion**: attitude relative to earth (gravity direction)
- **airspeed**: for energy management
- **gyro rates**: for control smoothness assessment

The fitness function has access to:
- Aircraft position (virtual frame)
- Path points with distanceFromStart
- Path tangent (from consecutive path points)
- Rabbit position (from AircraftState.rabbitPosition)

## Relationship to Other Features

- **021 (ACRO mode + rate gyros)**: This fitness works with either MANUAL or
  ACRO mode. The rate PID is orthogonal to the fitness function.
- **021 (DISTANCE_TARGET)**: Replaced entirely — no target offset, the cone
  geometry handles positioning.
- **Backlog (smoothness penalty)**: Implicit in combo multiplier — jerky
  control breaks tracking streaks. No explicit Σ|Δu| penalty needed.
- **Backlog (altitude-aware distance)**: Captured by asymmetric bias
  (above > below in cone scoring).

## Implementation Notes

- Replaces `computeStepPenalty()` in `fitness_computer.h/.cc`
- Replaces distance/attitude accumulation in `autoc.cc` and
  `fitness_decomposition.cc`
- Path tangent: `normalize(path[i+1].start - path[i].start)`
- Gravity direction: `[0, 0, 1]` in NED (down)
- Forecast rabbit position: already available via `getInterpolatedTargetPosition()`
- Tests: update `fitness_computer_tests.cc` and `fitness_decomposition_tests.cc`

## Success Criteria

1. NN converges to lower mean distance than current RMSE-based fitness
2. Sustained tracking streaks visible in renderer playback (combo > 20 steps)
3. Fewer crashes on FigureEight (turn anticipation from lead cone)
4. No bang-bang throttle (combo breaks on jerky control)
5. Fitness number is meaningful: higher combo = clearly better flight

## Out of Scope

- Optical tracking / beacon detection (future — this is path-only)
- Real-time combo display in renderer (nice to have, not blocking)
- Adaptive combo threshold based on path curvature

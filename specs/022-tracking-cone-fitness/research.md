# Research: 022 — Point-Accumulation Fitness

No unknowns required external research. All decisions resolved in spec clarifications.

## Decisions

### Aggregation Strategy
- **Decision**: Lexicase selection (unchanged)
- **Rationale**: Avoid changing too many things at once. Smoothness objectives may be added later as separate lexicase dimensions.
- **Alternatives**: Sum aggregation (simpler but loses per-scenario pressure)

### Fitness Direction
- **Decision**: Negate accumulated score (`fitness = -score`)
- **Rationale**: Existing selection operators minimize. Sign flip is simpler and safer than an arbitrary offset constant like `1M - score`.
- **Alternatives**: Large offset constant (rejected — breaks if score exceeds constant)

### Scoring Surface Shape
- **Decision**: Ellipsoidal with Lorentzian decay: `1 / (1 + eff_dist_sq)`
- **Rationale**: Smooth, always positive, no sqrt needed (just dot products and division). Computationally cheap for inner loop.
- **Alternatives**: Gaussian (`exp(-dist²)`) — more expensive, decays to zero faster (less intercept gradient)

### Streak Implementation
- **Decision**: Integer counter with hard reset, linear ramp to max multiplier
- **Rationale**: Simple, deterministic, easy to log and debug. Rate-independent via FitStreakRampSec config.
- **Alternatives**: Exponential ramp (rewards long streaks disproportionately — harder to tune for V1)

### Removed Components
- **Attitude delta penalty**: Dropped. Smoothness signals to be added as separate objectives later.
- **Intercept budget/scale**: Dropped. Scoring surface natural gradient is sufficient.
- **Crash penalty**: Dropped. Crash = stop earning points (no penalty needed).
- **DISTANCE_TARGET**: Dropped. Optimal is 0m (at the rabbit).

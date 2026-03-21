# Research: Flight Analysis & Sim-to-Real Verification

## R1: INAV Coordinate Conventions

**Decision**: INAV uses NEU (North-East-Up) internally for position, with altitude positive up.
Quaternion is body-to-earth in (w, x, y, z) format, scaled by 10000 in MSP wire format.

**Rationale**: Confirmed from:
- Flight data correlation (T200-T204): `xiao_z = -inav_navPos[2]/100` consistently
- INAV source code (`navigation.c`): `navPos` is lat/lon/alt with alt positive up
- MSP2_INAV_LOCAL_STATE: position in cm (NEU), velocity in cm/s, quaternion × 10000

**Alternatives considered**:
- NED throughout: rejected — INAV's internal convention is NEU, converting at the MSP boundary is correct
- ENU: not used — INAV uses North-East-Up, not East-North-Up

**Key conversions (INAV → xiao → NN)**:
- Position: `[N/100, E/100, -U/100]` → NED meters
- Velocity: `[VN/100, VE/100, -VU/100]` → NED m/s
- Quaternion: body→earth `(w,x,y,z)/10000` → conjugate to earth→body `(w,-x,-y,-z)`
- Heading: INAV applies `align_board_yaw` for display but MSP quaternion is in **sensor frame** (bench: 138° offset; flight: 0° offset confirmed)
- Attitude: `attitude[0,1,2]` in decidegrees (roll, pitch, heading) — for reference only, NN uses quaternion

## R2: Rabbit Position Projection from NN Inputs

**Decision**: Direct logging of rabbit world position in xiao is the primary approach.
Body-frame inverse projection is the secondary/diagnostic approach.

**Rationale**: The inverse projection from two independent atan2 outputs back to 3D is
ill-conditioned. The forward model (aircraft_state → dPhi, dTheta, dist) is well-defined,
but the inverse has ambiguity:
- `dPhi = atan2(target_local_y, -target_local_z)` — azimuth in body YZ plane
- `dTheta = atan2(-target_local_z, target_local_x)` — elevation in body XZ plane

These share the `-z` term but are independent projections, not spherical coordinates.
Inverting requires solving a system with one degree of freedom resolved by `dist`.

**Reconstruction approach** (for diagnostic overlay):
Given `dPhi`, `dTheta`, `dist`, and quaternion `q`:
1. Compute body-frame unit direction: solve for (x, y, z) from the two atan2 equations + unit norm
2. Scale by `dist` to get body-frame target offset
3. Rotate to world frame using quaternion inverse
4. Add aircraft world position

The system of equations:
- `y / (-z) = tan(dPhi)`
- `(-z) / x = tan(dTheta)`
- `x² + y² + z² = dist²`

From equation 2: `z = -x * tan(dTheta)`
From equation 1: `y = -z * tan(dPhi) = x * tan(dTheta) * tan(dPhi)`
Substituting into equation 3:
- `x² + x²·tan²(dTheta)·tan²(dPhi) + x²·tan²(dTheta) = dist²`
- `x² · (1 + tan²(dTheta)·(1 + tan²(dPhi))) = dist²`
- `x = ±dist / sqrt(1 + tan²(dTheta)·(1 + tan²(dPhi)))`

The ±x ambiguity is the front/back issue: target in front (x > 0) or behind (x < 0).
Resolve by using the sign from `cos(dTheta)`: if `cos(dTheta) > 0`, target is in front.

**Alternatives considered**:
- Spherical coordinates: rejected — dPhi/dTheta are not standard spherical (φ,θ)
- Iterative refinement: overkill — closed-form solution exists with sign disambiguation
- Unit vector + distance: would require changing NN inputs — deferred

**Key risk**: `tan()` singularity when dTheta = ±π/2 (target directly above/below) or
dPhi = ±π/2 (target directly left/right in body frame). Clamp to avoid NaN.
In practice during normal flight, these extremes are rare.

## R3: MSP RC Override Path in INAV

**Decision**: Needs investigation — determine if MSP_SET_RAW_RC goes direct to mixer/servos
or through INAV's PID controller.

**Rationale**: If INAV's attitude PID is still active during RC override, it will fight
the NN's direct control commands. The NN was trained with direct servo control in crrcsim
(no PID layer). If INAV adds PID, the effective control gain is different from training.

**Research needed**: Examine INAV source for `MSP_SET_RAW_RC` handling path. Check if
override mode bypasses PID (direct servo passthrough) or applies PID on top.

**Status**: Open — will be resolved during T231 implementation.

## R4: Cross-ISA FP Divergence Magnitude

**Decision**: ARM aarch64 (DGX Spark) → x86 (WSL2) produces 75× fitness degradation.
ARM aarch64 → ARM Cortex-M4F (xiao) is expected to be much smaller but unquantified.

**Rationale**: Observed fitness 2,724 (aarch64) vs 205,522 (x86) on identical
code/weights/config. The NN weights are optimized for ARM's specific FP rounding behavior.
Cortex-M4F is ARM but single-precision only (no fp64). If any desktop computation uses
double precision, truncation to float32 on Cortex-M4F is a divergence source.

**Mitigation**: T209 (pre-flight NN output comparison via `pio test`) and T210 (post-flight
replay) will quantify the actual ARM-to-ARM divergence.

## R5: Temporal Pipeline Model

**Decision**: The MSP sensor-to-actuator pipeline has ~200ms total latency.

**Measured**:
- MSP transport (INAV → xiao): ~200ms (stable ±25ms across 5 test spans)
- NN eval: 3-8ms (bench test), expected similar in flight
- MSP RC override send: ~5ms (estimate)
- Total: ~210ms sensor-to-actuator

**Implications**: At 16 m/s, 210ms = 3.4m uncompensated motion. The NN was trained with
zero pipeline delay in sim (instantaneous state → command). The temporal forecast inputs
(+0.1s, +0.5s path lookahead) partially compensate, but they predict the rabbit's future
position, not the aircraft's future position given the delay.

**Alternatives considered**:
- Delay compensation in NN inputs: add the pipeline delay to path lookahead
- Higher sample rate: 20Hz reduces per-step delay impact but doesn't eliminate transport
- State prediction: predict aircraft state forward by delay amount — complex, deferred

# AutoC / crrcsim Coordinate & Control Conventions

This note captures the shared assumptions between the AutoC GP framework and the
crrcsim‐based evaluation environment. It is intended as a quick reference when
analysing logs or integrating new flight data.

## Frames & Axes
- **World frame:** North–East–Down (NED).  
  - `+X` points toward geographic north.  
  - `+Y` points toward geographic east.  
  - `+Z` points downward toward the centre of the earth.
- **Body frame:** Right-handed aircraft axes.  
  - `+x_b` forward through the nose.  
  - `+y_b` out the right wing.  
  - `+z_b` downward through the belly.

## Position & Velocity Units
- crrcsim publishes FDM state in feet / feet-per-second.  
- The AUTOC bridge converts to metres / metres-per-second before storing in
  `AircraftState`.  
- Path generator targets and GP sensors operate strictly in metres and radians.

## ⚠️ CRITICAL: INAV airframe quat ↔ aerospace q_EB (msplink boundary)

INAV's AHRS publishes an airframe orientation quaternion that follows its
own internal convention: pitch is nose-down-positive (INAV Configurator
shows -30° for nose up), and yaw is nose-left-positive (N→E rotation is
negative). Roll matches aerospace (right-wing-down-positive). Board
alignment (`align_board_roll/pitch/yaw`) is applied INSIDE INAV before
any MSP message leaves the FC, so what arrives at xiao is already in
airframe body frame — just in INAV's sign conventions for pitch and yaw.

**Our boundary fix at xiao `msplink.cpp neuQuaternionToNed`** (T042,
bench-verified 2026-04-19):

```cpp
return gp_quat(q[0], q[1], -q[2], -q[3]);   // flip qy (pitch) and qz (yaw)
```

This flips the two components whose rotation axes have opposite sign in
INAV vs aerospace. `qw` and `qx` pass through unchanged. The result is a
quaternion whose static attitude extraction (Euler, direction cosines,
body-axis-in-world rotations) matches CRRCSim's aerospace q_EB. Training
↔ deployment parity verified: all 9 bench poses plus per-axis rate senses
conform.

**The gyro needs the same sign flips for consistency** — applied in
`convertMSPStateToAircraftState` ([msplink.cpp:710-719](../xiao/src/msplink.cpp#L710-L719)):

```cpp
gp_scalar p =  gyro_inav[0] * deciDegToRadS;   // roll: no change
gp_scalar q = -gyro_inav[1] * deciDegToRadS;   // pitch: flip
gp_scalar r = -gyro_inav[2] * deciDegToRadS;   // yaw: flip
```

**NOT KINEMATICALLY VALID.** Flipping two quaternion components is a
reflection as viewed by the rotation group — not a proper rotation. The
output quat is suitable for **static lookup** (attitude extraction,
direction cosines, body-axis-in-world projection) but **`dq/dt = ½·q·ω`
does not hold** between the transformed quat and the transformed gyro.

**Forbidden**:
- Feeding the (quat, gyro) pair into a filter that integrates body rates
  to update attitude (Kalman, Mahony, Madgwick) — the math will diverge.
- Computing `d(fixed_quat)/dt = ½·fixed_quat·ω` — kinematic relation broken.
- Composing the fixed quat with other rotation quaternions via Hamilton
  product under rotation-composition assumptions.

**Allowed**:
- Static attitude extraction (Euler, body-axis-in-world rotations).
- Direction-cosine computation for NN input.
- Sign-of-pitch / sign-of-roll / sign-of-yaw comparisons.

If a future Kalman/attitude filter wants a kinematically valid q_EB, it
MUST operate in INAV's native convention — use the raw INAV quat +
unflipped INAV gyro, integrate in INAV frame, and apply the qy/qz flip
only at the CONSUMER side to extract aerospace attitude.

Empirical derivation and bench evidence: see
`specs/024-sim-real-fidelity/spec.md` Findings Log and
`include/autoc/imu/inav_quat_convention.h` header. Covered by 8 contract
tests in `tests/msplink_quat_convention_tests.cc`.

## Orientation Representation
- **Canonical state:** Unit quaternion `(w, x, y, z)` — scalar-first, Hamilton
  convention — carried through the autoc stack and exchanged with the simulator.
- **Rotation direction:** **earth→body** (`q_EB`). A world-frame vector `v_w`
  is transformed to body frame via `v_b = q_EB * v_w * q_EB⁻¹`
  (or in Eigen notation, `v_b = q.inverse() * v_w`).
- **Why earth→body (industry split note):** Aerospace is divided on this.
  - **Earth→body** (our choice): Stevens & Lewis *Aircraft Control and Simulation*,
    Zipfel *Modeling and Simulation of Aerospace Vehicle Dynamics*, Etkin, most
    flight-dynamics textbooks. CRRCSim EOM01 FDM uses this natively (verified
    below). Convenient for rotating world gravity / wind / target vectors into
    body frame for sensor readout (the dominant operation in this codebase).
  - **Body→earth:** Shuster's survey, MATLAB Aerospace Toolbox, many spacecraft
    attitude references.
  - **INAV's MSP quat**: airframe orientation in INAV's internal sign
    conventions (pitch nose-down-pos, yaw N→W-pos). Bringing it to
    aerospace q_EB requires flipping qy and qz — the T042 `(w, x, -y, -z)`
    transform in `inavQuatToAerospaceEB`. Applied once at `msplink.cpp`
    and in the renderer's INAV-blackbox loader. Nothing downstream
    (NN evaluator, CRRCSim bridge) needs to worry about
    direction — it receives aerospace `q_EB` by contract. Caveat: the
    transformed quat is not kinematically valid — see the "CRITICAL"
    section above for forbidden vs allowed uses.
- **Euler read-out:** When needed for logging, the quaternion is converted to
  yaw–pitch–roll via Eigen's `eulerAngles(2, 1, 0)` (Z–Y–X intrinsic sequence)
  and wrapped to ±π radians. Return order: `[yaw, pitch, roll] = [ψ, θ, φ]`.
- **Body attitude sense (standard aerospace RHR):**
  - Positive roll φ (about body +X) = right wing down.
  - Positive pitch θ (about body +Y) = nose up.
  - Positive yaw ψ (about body +Z) = nose right (clockwise viewed from above).

## Sensor & Derived Quantities
- `GETROLL_RAD`, `GETPITCH_RAD` expose the wrapped Euler angles in radians.
- Distance-related primitives (`GETDHOME`, `GETDTARGET`, etc.) operate on metre
  vectors in the NED frame.
- **Deprecated (removed from NN input path, 023):** `GETALPHA` and `GETBETA`
  (angle of attack / sideslip) are no longer fed to the NN. If ever reintroduced,
  use the standard forms with no sign negation:
  - `α = atan2(v_body_z, v_body_x)` — positive when velocity has a downward
    body-Z component (relative wind comes from below the nose).
  - `β = atan2(v_body_y, v_body_x)` — positive when wind comes from the right.
- **Current NN perception (023):** Target geometry is presented to the NN as
  **direction cosines** — the unit vector `target_body / ||target_body||`
  where `target_body = q_EB * (rabbit_world - aircraft_world)`. Direction
  cosines are unitless, bounded ±1, and eliminate atan2 discontinuities.
  Distance is a separate scalar input in metres. See
  `include/autoc/nn/nn_input_computation.h` for the canonical implementation.

## 030/032 Tracker-Mode NN Inputs (54 floats)

Tracker-mode NN sees a flat `float[54]` ([include/autoc/nn/nn_inputs.h](../include/autoc/nn/nn_inputs.h),
populated by `gather_tracker_inputs` in
[src/nn/evaluator.cc](../src/nn/evaluator.cc)). Same struct + transforms feed
autoc-side training and the crrcsim worker — single source of truth
gated by `#ifndef ARDUINO`. Xiao firmware does NOT yet implement tracker-mode
(see BACKLOG).

**Schema growth history**: 45 floats in 030 baseline → **54 floats in 032 phase 1** (9 derived perceptual features added). 030 dmps no longer load against the 032 binary (M2 greenfield policy, no backward compat — cereal length-mismatch fails loudly).

| Slot      | Field                       | Meaning                                                   | Units / range                  | Source                                  |
|-----------|-----------------------------|-----------------------------------------------------------|--------------------------------|-----------------------------------------|
| 0–5       | `beacon_l_x[6]`             | Port (red) wingtip beacon screen x, history -0.5s…now     | NDC [-1, +1], +x = pilot right | `projectBeacon`, image-coord            |
| 6–11      | `beacon_l_y[6]`             | Port wingtip beacon screen y, history                     | NDC [-1, +1], +y = pilot down  | `projectBeacon`, image-coord            |
| 12–17     | `beacon_l_cep[6]`           | Port beacon CEP (uncertainty), history                    | [0, 1]; sentinel ⇒ `INT8_MIN`  | `projectBeacon` (max-edge distance proxy)|
| 18–23     | `beacon_r_x[6]`             | Starboard (green) wingtip beacon screen x, history        | same as `beacon_l_x`            | `projectBeacon`                         |
| 24–29     | `beacon_r_y[6]`             | Starboard wingtip beacon screen y, history                | same as `beacon_l_y`            | `projectBeacon`                         |
| 30–35     | `beacon_r_cep[6]`           | Starboard beacon CEP, history                             | same as `beacon_l_cep`          | `projectBeacon`                         |
| 36–39     | `quat_w/x/y/z`              | Chase body→world quaternion (Hamilton, w-first)           | unit-norm, components ±1       | `chase.getOrientation()`                |
| 40        | `airspeed`                  | Chase airspeed, **cruise-normalized** (M11.preA.2)        | dimensionless, ≈ [0, 2]; cruise=1 | `chase.getRelVel() / kCruiseSpeed_mps` |
| 41–43     | `gyro_p/q/r`                | Body-frame angular rates (aerospace RHR)                  | rad/s; standard ranges ±10     | `chase.getGyroRates()` (x=p roll, y=q pitch, z=r yaw) |
| 44        | `dist_to_boundary_along_vel`| Soft-saturated forward-flight margin to arena (M11.preA.2)| dimensionless [0, 1)            | `tanh(distanceToBoundary(...) / kDistToBoundaryScale_m)` |
| **45–50** | `beacon_pair_span[6]`       | **032 phase 1** — raw NDC Euclidean distance between port + starboard beacon centroids, 6-tick history. Inverse-distance proxy (bigger span = closer target). CEP-gated at "now" tick: substitute 0 if EITHER beacon's CEP ≥ `CepGateThreshold` | NDC, ~[0, 2.83] (geometric max); units same as `beacon_*_x/y` | `compute_pair_span(left.xy, right.xy)` in [include/autoc/eval/derived_features.h](../include/autoc/eval/derived_features.h); CEP-gating in [src/eval/tracker_stepper.cc](../src/eval/tracker_stepper.cc) `projectAndShiftHistory` (mirrored in [crrcsim_tracker_helper.cpp](../crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp)) |
| **51**    | `span_rate`                 | **032 phase 1** — `span[5] - span[4]` one-tick raw diff (no scaling, no smoothing). Closure-rate proxy (+ve = approaching) | NDC/tick, signed | [src/nn/evaluator.cc](../src/nn/evaluator.cc) `gather_tracker_inputs` |
| **52**    | `target_tilt_sin`           | **032 phase 1** — sin(θ) where θ = atan2(dy, dx) over port→starboard NDC line. θ = 0 ⇔ chase + target wings level relative. CEP-gate + geometric-degenerate guard substitute (sin, cos) = (0, 1) | dimensionless [-1, +1] | `compute_tilt(left.xy, right.xy)` in [derived_features.h](../include/autoc/eval/derived_features.h) |
| **53**    | `target_tilt_cos`           | **032 phase 1** — cos(θ); paired with slot 52 to avoid the ±π wraparound discontinuity of raw angle. NN can recover θ = atan2(sin, cos) internally if needed | dimensionless [-1, +1] | same as slot 52 |

### Beacon identity-stable ordering invariant (Feature A)

**Constitutional**: `beacon_l_*` slots ALWAYS carry the port-beacon (target body −y mount, red wingtip) projection; `beacon_r_*` slots ALWAYS carry the starboard-beacon (target body +y mount, green wingtip) projection. This is true regardless of which beacon lands on which side of the image plane at any given tick (e.g., target heading away from chase swaps image sides; the slot mapping does NOT swap).

In sim this is already enforced by the mount-keyed projection pipeline ([tracker_stepper.cc](../src/eval/tracker_stepper.cc) and [crrcsim_tracker_helper.cpp](../crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp) both project `beacon_left_` → `left` → `history.left_*` by construction; no NDC-x re-sort happens). For future **xiao tracker-mode port** (deferred), the FPGA emits per-detection `(x, y, CEP, code_id)` tuples; xiao firmware MUST map `code_id → {port, starboard}` via Gold-code identity (configured at xiao build time) and NEVER sort by image-plane position. Contract test in [tests/gather_tracker_inputs_tests.cc](../tests/gather_tracker_inputs_tests.cc) guards against accidental sim-side regression.

Failure mode if this contract breaks: the NN would see apparent +180° tilt flips between consecutive ticks even when target is flying steadily → contradictory gradient → no tilt-based learning possible. Identity-stable ordering preserves the geometric continuity the recurrent state can build on.

### Tilt convention (032 phase 1)

θ = `atan2(right.y − left.y, right.x − left.x)` over the NDC port→starboard line, measured in image-plane coordinates.

- θ = 0 ⇔ wings level relative to chase (port→starboard line projects horizontally with port on the image-plane-left side, dy = 0, dx > 0)
- θ = +π/2 ⇔ target rolled 90° CW relative to chase (port below image center, starboard above)
- θ = −π/2 ⇔ target rolled 90° CCW relative to chase
- θ wraps continuously through ±π (sin/cos encoding handles the wraparound — gradients stay meaningful)

This complements the chase's existing absolute-world attitude inputs (`quat_w/x/y/z`, slots 36-39). The chase NN gets both "I'm rolled in the world" (quat) and "I'm rolled relative to target" (tilt) and can separate them.

**Time samples on the 6-slot history**: `[-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]` on a 100ms NN cadence. Slot index 0 = oldest, slot 5 = "now". Pre-fill replicates source[0] × 6 at `initScenario` so first NN tick sees a coherent stationary-source history.

**Normalization constants** (`include/autoc/nn/nn_inputs.h`):
- `kCruiseSpeed_mps = 13.0f` — hb1 cruise estimate; chase airspeed = cruise → input = 1.0.
- `kDistToBoundaryScale_m = 20.0f` — `tanh(d/20)`: 10m→0.46, 15m→0.64, 20m→0.76, 30m→0.91, 40m→0.96. Sized to put the meaningful-gradient band around 1-2× the craft's ~10-15m emergency-180° turn budget so the NN gets *anticipatory* warning, not a reactive cliff inside the commit zone. Polarity: `dBnd → 0 = at the wall`, `dBnd → 1 = far from wall`. (`distanceToBoundary` itself is non-negative as long as the chase remains inside the cylinder, which it always does in sim — the cylinder has both top and ceiling. This input is a **soft saturation in NN space**, not a real-world safety layer; the safety layer remains a future addition that will operate in world coordinates.)

**Greenfield value-only changes**: cruise-norm + tanh on dist_to_boundary change the values seen by the NN but NOT the byte layout (`TrackerInputs` struct is still 45×float). data.dat schema is binary-equivalent; existing genomes from prior runs are not portable across the input-transform change (expected — no backward compat per project policy).

### Camera-POV HUD projection (renderer mini-screen)

[tools/renderer.cc:2915-3090](../tools/renderer.cc#L2915-L3090) shows a 2D "HUD"
rectangle in the upper-left of the renderer when the operator views a
tracker-mode dump. The two splat dots are the projected wingtip beacons —
**red = port (left wing), green = starboard (right wing)** — drawn at radii
proportional to CEP.

The mapping is direct from the NN-input convention:

| NN screen coord | Body / NED meaning            | HUD pixel mapping                       |
|-----------------|-------------------------------|-----------------------------------------|
| `screen_x = -1` | Beacon at left FOV edge       | Left edge of HUD rect (`xLeft`)         |
| `screen_x = +1` | Beacon at right FOV edge      | Right edge of HUD rect (`xRight`)       |
| `screen_y = -1` | Beacon at top FOV edge        | **Top** of HUD rect (`yTop`)            |
| `screen_y = +1` | Beacon at bottom FOV edge     | **Bottom** of HUD rect (`yBottom`)      |

Source convention from `projectBeacon`
([src/eval/camera_projection.cc:166-175](../src/eval/camera_projection.cc#L166-L175)):
camera frame is +x forward, +y right, +z down (NED-like body frame). The
projection writes `screen_x = camera.y / camera.x / tan(fov_h/2)` (right
positive) and `screen_y = camera.z / camera.x / tan(fov_v/2)` (down positive,
standard image-coord). Therefore a target craft **below** the chase optical
axis (camera.z > 0) projects with positive `screen_y`, which the renderer
maps to the **bottom** of the HUD rect — the operator sees the dot below
the centerline crosshair, matching real camera/cockpit-window geometry. No
inversion in any axis. Verified against the tick-0 frames of the M11.preA
crrcsim runs (chase faces world –x, target ahead-and-below at tick 0 →
dots below center, port red on chase's right when target is heading away,
matches operator inspection 2026-05-09).

## Control Command Polarity & Scaling
- GP program outputs are clamped in the range **−1 … 1** for pitch, roll,
  throttle.  
- Before passing to crrcsim (`TSimInputs`), the bridge rescales to simulator
  stick conventions:
  - Elevator = **−pitch / 2** ⇒ matches crrcsim’s `−0.5 … 0.5`, positive value
    = trailing edge down (nose up).  
  - Aileron  = **roll / 2** ⇒ `−0.5 … 0.5`, positive value = right wing down.  
- Throttle = **(throttle / 2) + 0.5** ⇒ `0 … 1`, positive GP command =
    increasing power.  
- Xiao/MSP mapping: GP −1…1 is sent as RC 1000…2000 µs. Roll is direct
  (`1500 + roll*500`), pitch is inverted (`1500 − pitch*500`), throttle is
  direct (`1500 + throttle*500`).
- Rudder, flaps, spoilers, etc. remain neutral in the current integration.

## Propeller Rotation & Reaction Torque (crrcsim FDM)

The crrcsim power model carries a per-propeller rotation-direction sign that
determines the airframe's reaction-torque polarity. This is a load-bearing
convention because it produces an asymmetric roll tendency that the controller
must learn to compensate (i.e., what real fixed-wing pilots call "P-factor +
torque effect requiring right rudder/aileron").

### Convention

Per [`crrcsim/src/mod_fdm/power/propeller.h:168`](../crrcsim/src/mod_fdm/power/propeller.h):

> *"Direction of rotation (view from behind): +1 = cw, -1 = ccw."*

- `<propeller rotation="+1">` (or no attribute — `+1` is the default)
  = **CW viewed from behind the craft** = **CCW viewed from in front of the craft**.
- `<propeller rotation="-1">` = CCW from behind = CW from front.

The XML attribute is parsed at [`propeller.cpp:130`](../crrcsim/src/mod_fdm/power/propeller.cpp):
`rot = prop->getDouble("rotation", 1);`

### Roll-torque sign (matches standard US engine convention)

For `rotation = +1` (CW-from-behind, CCW-from-front):

- Prop angular-momentum vector points **forward along craft +X** (right-hand rule).
- Newton's third-law reaction torque on the airframe is in **−X** direction.
- Body-frame −X torque rolls the craft **left** (port wing down).
- Pilot/controller compensates with right aileron/rudder.

This matches standard US-engine rotation convention (Cessna, Piper, etc.) — and
matches what evolved controllers in autoc training will learn to compensate for.

### hb1_streamer.xml — uses the default

[`crrcsim/models/hb1_streamer.xml`](../crrcsim/models/hb1_streamer.xml) declares:

```xml
<propeller D="0.127" H="0.114" J="0" n_fold="5" />
```

No `rotation` attribute → defaults to `+1` → **CCW viewed from in front of craft**
→ produces left-roll reaction torque, consistent with the trainer's intended
real-world behavior.

### Visual model vs aero model — independent

The `<graphics version="1" model="zagi-xs.ac">` line uses a delta-wing pusher
visual; the actual aero / torque model is independent of the rendered geometry.
The propeller's reaction torque axis is **along craft +X regardless of whether
the visual prop sits at the front (tractor) or back (pusher)** — the same
`rotation` sign produces the same airframe roll torque either way. So a
visual/physical mismatch (pusher rendered, tractor configured) is cosmetic only;
torque physics matches the aerodynamic model, not the rendered geometry.

### Implementation reference

Torque application in body frame at [`propeller.cpp:219`](../crrcsim/src/mod_fdm/power/propeller.cpp):

```cpp
*values->moment += mulMoment * F_X - dirThrust * (rot * M) + mulPfac * (rot * M_Pfac);
```

Where `dirThrust` is the prop axis direction (≈ craft +X), `M` is shaft torque
magnitude, and `rot` is the ±1 sign. With `rot = +1` and `dirThrust = +X`, the
contribution to the body-frame moment is `−X · M`, i.e., roll-left.

### When this would change

`rotation = -1` should only be set when modeling a craft where the actual
engine/prop rotates opposite the standard convention (e.g., counter-rotating
prop on a twin, or specific European/Russian engines). For paired-prop multicopters,
each prop's `rotation` is set independently to balance net yaw torque — see
[`propeller.cpp:124`](../crrcsim/src/mod_fdm/power/propeller.cpp).

## Summary Table

| Aspect                    | AutoC GP side                        | crrcsim / FDM side                 | Notes                                     |
|---------------------------|--------------------------------------|------------------------------------|-------------------------------------------|
| World axes                | NED (m, m/s)                         | NED (ft, ft/s)                     | bridge converts ft → m                    |
| Body axes                 | Right-handed aircraft                | Right-handed aircraft              | x forward, y right, z down                |
| Orientation               | Quaternion `(w,x,y,z)`               | Quaternion from EOM01              | logging uses wrapped yaw/pitch/roll [rad] |
| Pitch command             | `−1 … 1`, positive = pull up         | Elevator `−0.5 … 0.5`               | mapped via `−pitch/2` (sim) / `1500−pitch*500` (MSP) |
| Roll command              | `−1 … 1`, positive = right wing down | Aileron `−0.5 … 0.5`                | mapped via `roll/2` (sim) / `1500+roll*500` (MSP)    |
| Throttle command          | `−1 … 1`, positive = more power      | `0 … 1` (sim) / `1000…2000 µs` (MSP) | mapped via `(thr/2)+0.5` (sim) / `1500+thr*500` (MSP) |
| Angle units               | Radians                              | Radians                            | derived sensors wrap to ±π                |
| Distances / positions     | Metres                               | Feet                               | NED frame                                 |
| Velocity                  | m/s (body/world)                     | ft/s                               | conversion done before GP access          |

Keep this file updated if additional actuators, sensors, or frames are added.

## Virtual Frame (Path-Relative Coordinates)

Position data in the autoc pipeline uses a **virtual frame** where the origin
is at the aircraft's position when a test begins (sim path reset or xiao arm).
Paths are generated in this frame starting at (0,0,0).

### Principle: Convert Once at the Boundary

Raw (world) coordinates are converted to virtual coordinates **at the producer
boundary**, once. All downstream code — NN sensors, fitness evaluation, logging,
serialization — receives virtual position via `getPosition()`.

| Producer | Boundary | Raw→Virtual |
|----------|----------|-------------|
| CRRCSim | `inputdev_autoc.cpp` | `setPosition(fdmPos - pathOriginOffset)` |
| Xiao | `msplink.cpp` | `setPosition(inavPos - test_origin_offset)` |

### Invariant

**`AircraftState::getPosition()` always returns virtual coordinates.**

There is no `getVirtualPosition()` / `getRawPosition()` distinction. Downstream
code does not need to know whether the data came from CRRCSim or xiao.

### Raw Coordinates

Raw (world) coordinates are used for:
- **Out-of-bounds detection** in CRRCSim — checked with local raw variable
  before storing virtual position
- **Renderer "all flights" display** — reconstructs raw from virtual +
  origin offset metadata

### Origin Offset Metadata

The raw→virtual offset is stored per-scenario in `ScenarioMetadata::originOffset`
(serialized via cereal). This allows the renderer and analysis tools to reconstruct
raw positions when needed. It is NOT stored per-AircraftState.

**The offset is canonical, not actual.** It is the *expected* launch position
`(0, 0, SIM_INITIAL_ALTITUDE)`, NOT the actual FDM start position. Entry
variations (`EnableEntryVariations`) intentionally start the craft off-target —
north/east/altitude offsets from the canonical launch point. With a canonical
offset, those variations show up as virtual position deviation from (0,0,0)
that the fitness function correctly sees as "the aircraft started off-path."

If `pathOriginOffset` captured the actual FDM position (including variations),
the aircraft would always start at virtual (0,0,0) regardless of variations,
hiding the training challenge.

### Renderer Display Convention

The renderer shows paths and aircraft at a display altitude offset from virtual:
- Paths (virtual Z=0) → shifted by `SIM_INITIAL_ALTITUDE` (−55 m as of 041) for display, so
  **display z = −AGL** exactly and the arena cylinder lands on the checkerboard
- Aircraft positions (virtual Z≈0) → shifted by `SIM_INITIAL_ALTITUDE` for display
- Both use the same transform, so they overlap correctly

### Logging

- **data.dat**: All position columns (X, Y, Z, pathX, pathY, pathZ) are in
  virtual coordinates. Origin offset is available in ScenarioMetadata.
- **xiao flash log**: Logs both `pos_raw` (raw INAV) and `pos` (virtual)
  for correlation.

## INAV ↔ AutoC Interface Transform Summary

INAV uses an internal sign convention where pitch and yaw are systematically
inverted from standard aerospace right-hand rule. Every signal crossing the
INAV ↔ autoc boundary requires correction. The autoc pipeline (CRRCSim, NN,
xiao state) uses standard aerospace NED/FRD with RHR throughout.

| Signal | INAV convention | AutoC convention (standard) | Transform | Where applied |
|--------|----------------|---------------------------|-----------|---------------|
| Position XY | North+, East+ (NEU cm) | North+, East+ (NED m) | cm→m | `neuVectorToNedMeters` in msplink.cpp |
| Position Z | **Up+ (NEU)** | **Down+ (NED)** | **Negate + cm→m** | `neuVectorToNedMeters` line 161 |
| Quaternion | Airframe quat, pitch nose-down-pos, yaw N→W-pos | Aerospace q_EB, standard RHR | **Flip qy+qz**: `(w, x, -y, -z)` | `inavQuatToAerospaceEB` (T042) |
| Gyro roll | Right wing down = + | Right wing down = + | None | T041 (021) |
| Gyro pitch | **Nose down = +** | **Nose up = +** | **Negate** | T041 (021) |
| Gyro yaw | **Nose left = +** | **Nose right = +** | **Negate** | T041 (021) |
| Cmd roll (out) | MSP 2000 = right | NN +1 = right | Direct: `1500 + cmd*500` | msplink.cpp line 688 |
| Cmd pitch (out) | MSP 1000 = nose up | NN +1 = nose up | **Invert**: `1500 - cmd*500` | msplink.cpp line 695 |
| Cmd throttle (out) | MSP 2000 = full | NN +1 = full | Direct: `1500 + cmd*500` | msplink.cpp line 702 |

All transforms are applied at the INAV↔xiao boundary (msplink.cpp). Downstream
code (NN evaluator, CRRCSim, renderer) uses standard aerospace convention only.

## Accelerometer as an INTERFACE quantity (041, 2026-08-10)

⚠️ **New status.** Until 041 the accelerometer was an **internal** value — it has never been an interface
quantity, going back to minisim. The 021 table below is therefore *observational* (what blackbox
`accSmooth` logs), **not** a ratified NN-input standard. 041 promotes accel to an interface quantity
because `ACCEL_X/Y/Z` become NN inputs, so the standard is stated here.

**The standard: aerospace body FRD** — x forward, y right, z down — the same frame the quat and gyro use
downstream of the msplink boundary. **Explicitly NOT FRU/FLU** (operator 2026-08-10). Units: **g**
(dimensionless), scaled by `kAccelScale_g` only at the NN slot write.

**Two converters, deliberately different.** Native CRRCSim is its own beast and INAV is another; neither is
the standard, and each gets its own boundary conversion:

| source | native form | converter | lands as |
|---|---|---|---|
| CRRCSim FDM | world-frame kinematic accel (LaRCSim, ft-based) + gravity + q_EB | `autoc/eval/specific_force.h`, worker-side | body FRD, g |
| INAV | `acc.accADCf` — proper acceleration in INAV's native **FLU** frame; post `applySensorAlignment` + `applyBoardAlignment`, ÷ `acc_1G` | `msplink.cpp`, the **same y/z flip** as the quat's `(w, x, -y, -z)` and the gyro's pitch/yaw negation | body FRD, g |

⚠️ **Board alignment differs between bench and flight** (`xiao/inav-bench.cfg` vs `xiao/inav-hb1.cfg`) —
the devices are mounted differently. Alignment is applied INSIDE INAV before MSP, so the converter is the
same for both, but **T073 must verify on both targets**; a bench-only check is not a flight check.

### ✅ RESOLVED 2026-08-11 — the bench table is correct as recorded; the *assumed frame* was wrong

The nose-up row was flagged as not fitting the convention. It fits. **None of the three candidate
explanations previously listed here is right** (no transcription error, no per-axis sign, no mislabelled
maneuver) — the fourth possibility is the answer:

> **INAV's body frame is FLU** — x forward, y **LEFT**, z **UP** — and `acc.accADCf` is plain
> **proper acceleration** (specific force), which at rest points **UP**, away from the earth.

Under that reading all three bench rows are consistent, with no exceptions:

| attitude | which body axis points UP | INAV reads (FLU) | → aerospace FRD |
|---|---|---|---|
| level | `+z` (up) | `[0, 0, +1g]` | `[0, 0, −1g]` |
| nose up 90° | `+x` (forward) | `[+1g, 0, 0]` | `[+1g, 0, 0]` |
| right wing down 90° | `+y` (left) | `[0, +1g, 0]` | `[0, −1g, 0]` |

**Why nose-up was the row that disagreed** — and why that is the *expected* place for a frame error to
show. FLU→FRD negates **y and z only**; **x is shared**. So on the level and RWD rows the frame flip and
the specific-force-vs-gravity-direction flip *cancel*, and both hypotheses predict the same number. X is
the one axis where they cannot cancel. The nose-up row is therefore not an anomaly — it is the single
discriminating measurement in the table, and it discriminates in favour of FLU + specific force.

**Evidence, from `~/inav` @ `63cffaf4` (not from first principles):**

1. `sensors/acceleration.c:567-568` — `acc.accADCf[axis] = accADC[axis] / acc.dev.acc_1G`. No negation
   anywhere in `accUpdate()`; the value is the raw chip reading after alignment and scaling only. Whatever
   sign the sensor produces is the sign INAV publishes.
2. `sensors/acceleration.c:335-337` with `:360-366` — `getPrimaryAxisIndex` returns index **0** when
   `sample[Z] > 0`, and index 0 is the **TOP-UP** calibration position (`accStartCalibration` rejects a
   recalibration that does not start there). A MEMS accelerometer at rest reads `+1 g` on whichever of its
   axes points up ⇒ a level, top-up board reads **positive Z** ⇒ **INAV's board `+z` is UP**.
3. `flight/imu.c:483-491` — `vGravity = {0, 0, 1}` in the earth frame is rotated EF→BF and crossed with the
   *normalised measured accel*, with the error driven to zero. The measured vector is therefore parallel to
   earth `+z`, consistent with a Z-up earth frame and a reaction-up reading.
4. `y` is LEFT follows from the RWD row given (2): the reading is `+1 g` on Y, so Y is the axis pointing up
   in that attitude, which is the **left** wing.
5. This is the **same** frame difference the project already handles twice: the quat's `(w, x, −y, −z)`
   (`inavQuatToAerospaceEB`) and the gyro's pitch/yaw negation (`msplink.cpp:964-966`) are each exactly a
   y/z flip. They were documented as "INAV inverts pitch and yaw"; structurally they are one frame change,
   FLU→FRD, and accel is the third quantity through the same door.
6. Board alignment is *not* involved: bench `align_board_roll = −16` (−1.6°, a mount trim) and flight
   `roll = 1700, yaw = 900` are applied **inside** INAV before MSP (`acceleration.c:563-564`), so the
   boundary converter is identical for both targets. T073 still verifies on both — the alignment values
   differ, so a bench-only check is not a flight check.

**Consequences — read these before writing a sign anywhere:**

- **The msplink converter is `accel_FRD = (accADCf[0], −accADCf[1], −accADCf[2])`** — the same y/z flip as
  the quat and the gyro, applied once at the same boundary (T074).
- **In aerospace FRD, steady level flight reads `[0, 0, −1 g]`.** Body `+z` is DOWN and the measured
  reaction points UP. This matches `spec.md` § Clarifications (which governs) and matches
  `include/autoc/eval/specific_force.h` **as already written** — `f = R(q)ᵀ·(a_world − g_world)/g`.
  ⚠️ **No sign flip is owed.** An earlier 041 handoff note asserted the adopted convention was the *sensed
  gravity direction* (level ⇒ `[0, 0, +1g]`) and that the header needed flipping. That note was written
  against this unresolved datum and is **withdrawn**; flipping the header would have put the NN's load axis
  backwards relative to flight.
- The load factor quoted in flight reports is `nz = −ACCEL_Z`, which is what `load_factor_nz` in
  `specific_force.h` carries. That negation is for **human-facing reporting only** — the NN input is the
  un-negated FRD component.
- **All three attitudes are now safe to assert** in T073, in FRD: level `[0, 0, −1]`, nose up
  `[+1, 0, 0]`, right wing down `[0, −1, 0]`. Compare against the counts table below by dividing by
  `acc_1G ≈ 2048` **and** applying the y/z flip — the table is in INAV's FLU counts, not FRD g.

### ✅ MEASURED ON THE WIRE 2026-08-22 (041 P5-2) — prediction confirmed, bench target

The three attitudes above are no longer a prediction. `MSP2_AUTOC_STATE` was extended to carry
`acc.accADCf` as milli-g `int16` (041 P5-1), and the payload was read **directly off the FC's USB VCP** by
[`specs/041-m2-depth/msp_state_probe.py`](../specs/041-m2-depth/msp_state_probe.py) — host-side, no xiao in
the loop, so a convention error could not hide behind a firmware bug. Bench FC `MAMBAF722_2022A`
(`align_board_roll = −16°`), INAV 8.0.0, payload 58 → **64 bytes**.

| attitude | FLU as INAV sends it (g) | FRD after the msplink flip (g) | expected | |
|---|---|---|---|---|
| level | `[−0.011, +0.032, +0.997]` | `[−0.011, −0.032, −0.997]` | `[0, 0, −1]` | ✅ |
| nose up | `[+0.998, +0.023, +0.091]` | `[+0.998, −0.023, −0.091]` | `[+1, 0, 0]` | ✅ |
| right wing down | `[+0.001, +1.000, −0.003]` | `[+0.001, −1.000, +0.003]` | `[0, −1, 0]` | ✅ |

⭐ **Confirmed a second time, independently, through the xiao (2026-08-22).** With the 45-input firmware
flashed, the 1 Hz console heartbeat prints `accel=[…]` straight off the NN carrier — i.e. AFTER msplink's
FLU→FRD flip, which is the value the policy actually consumes. Operator read all three attitudes: level
`z = −1`, nose up `x = +1`, right wing down `y = −1`. That matches the host-side probe exactly, which
matters because the two paths share no code: the probe parses the wire in Python, the xiao parses it in C++
and applies its own flip. **A disagreement would have localised the bug to msplink; agreement clears the
whole chain** — INAV `acc.accADCf` → MSP → `msp_autoc_state_t` → the flip → `setSpecificForceG`.

Hand-held attitudes, n=20 averaged, `|a|` within 0.3% of 1 g throughout. The off-axis terms are holding
angle (nose-up sits ~5° off vertical), not convention error — the sign and the carrying axis are
unambiguous in all three.

⚠️ **This certifies the BENCH board only.** Board alignment is applied inside INAV before MSP, so the
flight FC (`MATEKF722MINI`, `align_board_roll` 170° vs 180° — see auto-memory `project_board_alignment`)
must be re-measured after it is flashed. A 10° residual misalignment puts ~0.17 g of gravity on the wrong
axis **and rotates with attitude**, so unlike a fixed offset it does not average out in flight.

⭐ **Bias/scale residuals, for the sim-fidelity question**: the bench board's worst offset is +0.032 g (y at
level) with a −0.3% scale error. Against `kAccelScale_g = 8.0` that reaches the NN as **0.004 input units**,
versus a tanh linear region running to ~1.4 (the 11.2 g flight record) — ~0.3% of useful range. Sim models
**no** accel bias or noise (`config.h` has `enableAccelInputs`/`accelScaleG` and no sigma); on these numbers
that is defensible, but it is an untested assumption for the flight board until its residuals are measured.

### ⚠️ OPEN (2026-08-22): the accel channel's DYNAMICS differ between sim and INAV, though its units do not

Normalization is settled and identical — both sides store body-FRD g in `AircraftState` and divide by
`kAccelScale_g` in the ONE shared gather (`src/nn/evaluator.cc`), so level flight reaches the NN as `−0.125`
on either path. What is **not** matched is the filtering:

| | source | filtering |
|---|---|---|
| sim | `bodySpecificForce((a_world − g_world), q, g)` off the FDM | **none** |
| INAV | `acc.accADCf` | `accSoftLpfFilter` at `acc_lpf_hz`, **default 15 Hz** (PT1 or BIQUAD per `acc_lpf_type`), plus `acc_notch_hz` if set |

At a 20 Hz control cadence a 15 Hz LPF is a real phase difference, so the policy sees a slightly laggier,
smoother accel in the air than it trained against. ⚠️ **Unverified operator recollection (2026-08-22)**:
that this filter is *disabled* when autoc/xiao is driving INAV. Reading `acceleration.c` `accUpdate()`, the
LPF is applied **unconditionally** whenever `acc_lpf_hz != 0` — it is not gated on flight mode or MSP
override, so the recollection may concern the gyro/PID path instead. **Resolve it by reading the bench FC's
actual setting (`get acc_lpf_hz`) rather than from source or memory**, and record the answer here.

## Gyro & Accelerometer Conventions (021, 2026-03-28)

### INAV Sensor Processing Chain
Both gyro and accelerometer follow the same transformation pipeline:

```
Raw ADC → Zero calibration → Sensor alignment → Board alignment → Filter → Output
```

Board alignment is applied at the RAW SENSOR level (`gyro.c:439`, `acceleration.c:564`)
BEFORE filtering and BEFORE IMU quaternion fusion. The logged values (gyroADC, accSmooth)
are already in aircraft body frame, not sensor frame.

### Blackbox Gyro: gyroADC[0-2] (VERIFIED bench 2026-03-30)

**WARNING**: INAV gyro pitch and yaw signs are INVERTED from standard aerospace RHR.

| Index | Axis | INAV positive direction | Standard aerospace positive | Units |
|-------|------|------------------------|----------------------------|-------|
| [0] | Roll (X body) | Right wing down | Right wing down | deg/s (int16) |
| [1] | Pitch (Y body) | **Nose DOWN** | Nose UP | deg/s (int16) |
| [2] | Yaw (Z body) | **Nose LEFT / CCW** | Nose RIGHT / CW | deg/s (int16) |

- **Frame**: Body-frame, board-alignment-corrected
- **Filtering**: Post-LPF (anti-alias 250Hz + main LPF 25Hz + dynamic notch)
- **gyroRaw[0-2]**: Same axes but pre-main-filter (only anti-alias LPF applied)
- **INAV convention**: roll matches standard RHR, but pitch and yaw are negated.
  INAV configurator confirms: it displays negative pitch for nose up.
- **This is consistent with the quaternion**: INAV's airframe quaternion also
  has pitch and yaw in the same inverted convention. The T042 boundary
  transform `(w, x, -y, -z)` flips qy and qz to bring the quat into
  aerospace convention (see the "CRITICAL" section at the top).

**Required transform for NN / CRRCSim compatibility (021):**
```
roll_rate  =  gyroADC[0]    // matches standard, no change
pitch_rate = -gyroADC[1]    // negate to match aerospace RHR (nose up = positive)
yaw_rate   = -gyroADC[2]    // negate to match aerospace RHR (nose right = positive)
```

### Blackbox Accelerometer: accSmooth[0-2] (VERIFIED bench 2026-03-30)

⚠️ **Axis labels corrected 2026-08-11** (041). The rows are as measured; the *interpretation* below them
was wrong — INAV's frame is **FLU**, and the reading is proper acceleration (points UP at rest), not the
gravity direction. See "Accelerometer as an INTERFACE quantity (041)" above for the derivation and for the
FRD conversion. The measured numbers never changed.

| Index | Axis (INAV FLU) | At rest (level) | Nose up 90° | Right wing down 90° | Units |
|-------|------|-----------------|-------------|---------------------|-------|
| [0] | X body (forward) | ~0 | **+1G** | ~0 | acc_1G scale |
| [1] | Y body (**left**) | ~0 | ~0 | **+1G** | acc_1G scale |
| [2] | Z body (**up**) | **+1G** | ~0 | ~0 | acc_1G scale |

- **Frame**: Body-frame **FLU**, board-alignment-corrected (same pipeline as gyro)
- **The reading is the reaction, pointing UP** — a MEMS accelerometer at rest reads +1 G on whichever axis
  points at the sky. Every row below is that one rule.
- **At level rest**: accel ≈ [0, 0, +1G] — body +Z (up) is the skyward axis ✓ verified
- **Nose up**: accel ≈ [+1G, 0, 0] — body +X (forward) is the skyward axis ✓ verified
- **Right wing down**: accel ≈ [0, +1G, 0] — body +Y (**left**) is the skyward axis ✓ verified
- **In aerospace FRD** (after the msplink y/z flip): `[0,0,−1]`, `[+1,0,0]`, `[0,−1,0]` respectively
- **acc_1G scale**: ~2050 counts = 1G on this hardware (bench measurement)
- **In turns**: centripetal acceleration adds to gravity vector (useful, not noise)

### CRRCSim FDM Body Rates

LaRCSim FDM computes body angular rates (p, q, r) internally:
- `v_R_omega_total.r[0]` = p (roll rate, rad/s)
- `v_R_omega_total.r[1]` = q (pitch rate, rad/s)
- `v_R_omega_total.r[2]` = r (yaw rate, rad/s)

Same right-hand convention as INAV gyroADC. Note CRRCSim uses rad/s while
INAV uses deg/s — conversion needed at the interface.

### Ground Verification Protocol
Before flight, verify polarity on the ground:
1. Roll aircraft right → gyroADC[0] should be positive
2. Pitch aircraft nose up → gyroADC[1] should be positive
3. Yaw aircraft nose right → gyroADC[2] should be positive
4. Level aircraft → accSmooth[2] should be positive (~1G)

## Quaternion & Euler sign conventions (VERIFIED via bench testing 2026-04-19)

### Standard Aerospace Convention (NED/FRD)
- **World frame:** NED (North-East-Down), right-handed
- **Body frame:** FRD (Forward-Right-Down), right-handed
- **Quaternion format:** `(w, x, y, z)` representing **earth→body** rotation
- **Right-hand rule rotations:**
  - **Roll** (φ, about +X body): Positive = right wing down
  - **Pitch** (θ, about +Y body): Positive = nose up
  - **Yaw** (ψ, about +Z body): Positive = nose right (clockwise from above)

### Actual Implementation (Bench-Tested 2026-04-19, T042)

#### INAV (xiao-gp/msplink.cpp)
- **MSP2_AUTOC_STATE** sends quaternion as `(q0, q1, q2, q3)` = `(w, x, y, z)`.
- **Convention as sent**: airframe body→earth, but in INAV's internal
  sign conventions — pitch nose-down-positive and yaw N→W-positive (same
  convention as INAV's `attitude[]` Euler and `gyroADCf[]` rates).
- **Transformation applied** (T042 bench-verified):
  `inavQuatToAerospaceEB` returns `gp_quat(q[0], q[1], -q[2], -q[3])` —
  flip qy (pitch axis) and qz (yaw axis); qw and qx pass through.
- **Not a proper rotation** — flipping two components is a reflection
  relative to the rotation group. Result is valid for STATIC lookup only,
  NOT kinematic integration. See the "CRITICAL" section at the top.
- **Board alignment**: INAV applies sensor→board and board→airframe
  rotations at the RAW SENSOR level (`gyro.c:439`, `acceleration.c:564`)
  BEFORE IMU quaternion fusion. What arrives at xiao is already in
  airframe body frame — no board-alignment residual leaks into the MSP
  stream.
  - **Flight hardware (hb1, per `xiao/inav-hb1.cfg`)**: `align_board_roll=1700,
    align_board_pitch=0, align_board_yaw=900` — board is mounted with ~170°
    roll and 90° yaw relative to aircraft body. INAV corrects internally.
  - **Pre-flight sanity check**: before each flight test, verify via INAV
    configurator that the FC shows near-zero roll/pitch/yaw when the
    aircraft is held nose-north-level (remember: configurator shows
    negative pitch for nose-up — that's INAV's convention, not a bug).
    This catches misaligned board config (e.g., physical re-mount
    without config update).
- **INAV Configurator UI**: shows **negative pitch** for nose up — this
  is the INAV convention showing through, not a display artifact. The
  T042 boundary flip aligns the stored quat to aerospace nose-up-pos.

#### GP/CRRCSim Contract (Standard NED/FRD)
- **Required quaternion**: Earth→body in NED/FRD with standard right-hand rule.
- **Validation**: sim gen-400 paths 1 and 2 both pass all applicable
  `sensor_self_check` cross-checks (pos↔vel integration, gyro↔quat-delta,
  heading↔track, nose-vs-velocity direction). EOM01 formulas in
  `crrcsim/src/mod_fdm/eom01/eom01.cpp:111-118` expand to aerospace q_EB
  ZYX (see math block below).
- **Frame consistency**: both the xiao pipeline (after T042 boundary) and
  CRRCSim produce aerospace q_EB. Training ↔ deployment parity.

### Bench Test Results (Verification Table, T042, 2026-04-19)

Physical craft orientations measured with INAV board alignment
`align_board=1700/0/900` and the T042 `(w, x, -y, -z)` boundary transform.
The "INAV sends" column is the raw airframe quat before our flip; the
"After T042" column is what xiao logs and what flows into `AircraftState`.

| Physical orientation | INAV sends (raw airframe) | After T042 (aerospace q_EB) | Validates |
|---|---|---|---|
| Level, nose north | `[1, 0, 0, 0]` | `[1, 0, 0, 0]` | ✅ identity baseline |
| Nose up 30° | `[.97, 0, -.26, 0]` | `[.97, 0, +.26, 0]` | ✅ +qy = nose up |
| Nose down 30° | `[.97, 0, +.25, 0]` | `[.97, 0, -.25, 0]` | ✅ -qy = nose down |
| Right wing down 30° | `[.97, +.26, 0, 0]` | `[.97, +.26, 0, 0]` | ✅ +qx = RWD (qx pass-through) |
| Right wing up 30° | `[.97, -.26, 0, 0]` | `[.97, -.26, 0, 0]` | ✅ -qx = RWU |
| Nose east 90° | `[.71, 0, 0, -.71]` | `[.71, 0, 0, +.71]` | ✅ +qz = N→E |
| Nose west 90° | `[.71, 0, 0, +.71]` | `[.71, 0, 0, -.71]` | ✅ -qz = N→W |
| Compound: nose up 30° + RWD 30° | `[.92, +.26, -.3, 0]` | `[.93, +.25, +.25, -.07]` | ✅ combined axes |
| Inverted (180° roll) | `[0, -1, 0, 0]` | `[0, -1, 0, 0]` | ✅ 180° roll (double-cover ambiguity OK) |

**Per-axis rate sense** (slow rotation into each pose, `gyro=` field on
Nav State log line, aerospace rad/s post-msplink):

| Motion | Axis expected + | Bench observation |
|---|---|---|
| Slowly roll right (RWD) | `p` (gyro.x) | ✅ positive during motion |
| Slowly pitch up (nose up) | `q` (gyro.y) | ✅ positive during motion |
| Slowly yaw right (N→E) | `r` (gyro.z) | ✅ positive during motion |

**Conclusion**: with T042's `(w, x, -y, -z)` quat flip and the existing
gyro pitch/yaw negations in `convertMSPStateToAircraftState`, the xiao
pipeline delivers aerospace q_EB and aerospace body rates. Both sides
(sim and xiao) feed the NN the same convention. Contract-tested in
`tests/msplink_quat_convention_tests.cc` (8 tests, all pass).

### Euler Angle Extraction
- Use Eigen's `eulerAngles(2, 1, 0)` for ZYX intrinsic sequence (yaw-pitch-roll)
- Wrap to ±π radians for logging
- Order: `[yaw, pitch, roll]` = `[ψ, θ, φ]`

## CRRCSim EOM01 Quaternion Convention (VERIFIED 2025-12-20)

### Mathematical Analysis

The EOM01 flight dynamics model in CRRCSim uses standard earth→body quaternion convention, confirmed by analyzing the implementation:

#### Initialization from Euler Angles (eom01.cpp:111-118)
```cpp
e_0 = cos(Psi*0.5)*cos(Theta*0.5)*cos(Phi*0.5) + sin(Psi*0.5)*sin(Theta*0.5)*sin(Phi*0.5);
e_1 = cos(Psi*0.5)*cos(Theta*0.5)*sin(Phi*0.5) - sin(Psi*0.5)*sin(Theta*0.5)*cos(Phi*0.5);
e_2 = cos(Psi*0.5)*sin(Theta*0.5)*cos(Phi*0.5) + sin(Psi*0.5)*cos(Theta*0.5)*sin(Phi*0.5);
e_3 = -cos(Psi*0.5)*sin(Theta*0.5)*sin(Phi*0.5) + sin(Psi*0.5)*cos(Theta*0.5)*cos(Phi*0.5);
```

Where: `Psi` = yaw (ψ), `Theta` = pitch (θ), `Phi` = roll (φ)

This follows the **standard ZYX (3-2-1) Euler sequence** for earth→body rotation in aerospace:
- First rotate about earth Z-axis (yaw)
- Then rotate about intermediate Y-axis (pitch)
- Finally rotate about body X-axis (roll)

The formula matches the standard aerospace convention for quaternion from Euler angles representing **earth→body** transformation.

#### Quaternion Integration (eom01.cpp:240-243)
```cpp
e_dot_0 = 0.5*( -v_R_omega_total.r[0]*e_1 - v_R_omega_total.r[1]*e_2 - v_R_omega_total.r[2]*e_3 );
e_dot_1 = 0.5*(  v_R_omega_total.r[0]*e_0 - v_R_omega_total.r[1]*e_3 + v_R_omega_total.r[2]*e_2 );
e_dot_2 = 0.5*(  v_R_omega_total.r[0]*e_3 + v_R_omega_total.r[1]*e_0 - v_R_omega_total.r[2]*e_1 );
e_dot_3 = 0.5*( -v_R_omega_total.r[0]*e_2 + v_R_omega_total.r[1]*e_1 + v_R_omega_total.r[2]*e_0 );
```

Where `v_R_omega_total` is the angular velocity in body frame (p, q, r).

These equations match the **standard quaternion kinematics for earth→body quaternion** with body-frame angular rates:
```
q̇ = 0.5 * q ⊗ ω_body
```

This is the correct formulation for integrating an earth→body quaternion using body-frame angular velocities.

#### Direct Access (eom01.h:86-89)
```cpp
virtual double getQuatW() { return e_0; }
virtual double getQuatX() { return e_1; }
virtual double getQuatY() { return e_2; }
virtual double getQuatZ() { return e_3; }
```

The quaternion is passed **directly** to the GP system with no transformations:
- EOM01 stores `(e_0, e_1, e_2, e_3)` as earth→body quaternion
- CRRCSim bridge reads via `getQuat*()` accessors (inputdev_autoc.cpp:455)
- GP receives identical quaternion in `AircraftState`

### GP Sensor Verification (gp_evaluator_portable.cc:226-240)

#### GETALPHA (Angle of Attack)
```cpp
case GETALPHA: {
    gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
    result = fastAtan2(-velocity_body.z(), velocity_body.x());
    break;
}
```

**Analysis:**
- `getOrientation()` returns the earth→body quaternion
- `.inverse()` creates body→earth quaternion
- Multiplying world-frame velocity by body→earth quaternion transforms it to body frame
- This is **correct usage** of an earth→body quaternion

#### GETBETA (Sideslip Angle)
```cpp
case GETBETA: {
    gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
    result = fastAtan2(velocity_body.y(), velocity_body.x());
    break;
}
```

**Analysis:**
- Uses same earth→body quaternion inversion to transform velocity to body frame
- Computes sideslip angle from body-frame velocity components
- This is **correct usage** of an earth→body quaternion

### Contract Compliance Summary

✅ **EOM01 (CRRCSim FDM):** Produces earth→body quaternions via standard ZYX Euler initialization and standard quaternion kinematics with body-frame rates

✅ **CRRCSim Bridge:** Passes quaternions directly from EOM01 to GP with no transformations

✅ **GP Sensors:** Use `.inverse()` correctly to transform world-frame vectors to body frame, confirming they expect earth→body quaternions

✅ **INAV (after T042 boundary flip):** The `(w, x, -y, -z)` transform in `inavQuatToAerospaceEB` brings INAV's airframe-convention quat into aerospace q_EB for static lookup. Not kinematically valid; see "CRITICAL" section.

✅ **Renderer:** AircraftState stores aerospace q_EB post-T042, so renderer gets the right convention directly.

**Conclusion:** After the T042 boundary flip, both the xiao pipeline and CRRCSim produce aerospace q_EB in NED/FRD. The transformed quat is safe for static attitude lookup (Euler extraction, direction cosines, renderer projections); do NOT feed it into a Kalman/Mahony filter alongside the gyro — see the "CRITICAL" section at the top for the full forbidden/allowed list.

## Renderer Quaternion Handling (FIXED 2025-12-20)

The renderer (renderer.cc) expects earth→body quaternions in `AircraftState`, matching the GP contract:

### Visualization Usage
- **Line 1054:** `s.getOrientation() * -vec3::UnitZ()` - Rotates body-up vector to earth frame for ribbon orientation
- **Lines 2315-2319:** Attitude indicator rotates body frame axes to earth frame to extract pitch/roll

Both operations correctly use earth→body quaternions by multiplying body-frame vectors.

### INAV Blackbox Loading (renderer)

The renderer's INAV-blackbox path applies the same T042 boundary transform
as msplink so historical flight data is interpreted in aerospace q_EB:

```cpp
// INAV blackbox quat is in INAV's airframe convention (pitch-nose-down-pos,
// yaw-N→W-pos). Flip qy and qz to get aerospace q_EB for rendering.
quat inavQuat(qw, qx, qy, qz);
inavQuat.normalize();
quat aero_q_EB(inavQuat.w(), inavQuat.x(), -inavQuat.y(), -inavQuat.z());
aero_q_EB.normalize();
```

(The sensor_self_check analysis library uses the same T042 boundary in
its `blackbox_to_canonical()`; see `sensor_self_check_lib.py` — it
exposes both `quat` (aerospace, post-flip) and `quat_raw` (INAV, for
kinematic rate checks that would break under the reflection).)

### CRRCSim/GP Evaluation Data Loading
CRRCSim `EvalResults` already contain earth→body quaternions (verified above), so no transformation is needed when loading evaluation data - the renderer uses them directly.

### Ground Verification Results (bench 2026-03-30)

Bench FC (MAMBAF722_2022A), board alignment: roll=-16 pitch=0 yaw=0 (near identity).
Craft held by hand, tilted through each axis, blackbox recorded at 1/32.

| Physical maneuver | gyroADC[0] | gyroADC[1] | gyroADC[2] | accSmooth at hold | Euler (from quat) |
|-------------------|-----------|-----------|-----------|-------------------|-------------------|
| Level (start) | ~0 | ~0 | ~0 | [~0, ~0, +2050] | roll=0° pitch=0° |
| Right wing down 90° | **+156** | ~0 | ~0 | [~0, **+2060**, ~0] | roll=**+87°** |
| Right wing back up | **-166** | ~0 | ~0 | — | roll→0° |
| Nose up 90° | ~0 | **-136** | ~0 | [**+2050**, ~0, ~0] | pitch=**-84°** |
| Nose back level | ~0 | **+149** | ~0 | — | pitch→0° |
| Yaw right (N→E) | ~0 | ~0 | **-113** | [~0, ~0, +2050] | yaw: -9°→**-97°** |
| Yaw back (E→N) | ~0 | ~0 | **+100** | — | yaw→-9° |

**Key finding**: INAV gyro pitch and yaw sign convention is inverted from
standard aerospace right-hand rule. Roll matches. This is consistent with
INAV's quaternion convention (also requires qy and qz flips — the T042
boundary transform — to match aerospace).

**Required gyro transform for autoc pipeline (021):**
```
roll_rate  =  gyroADC[0]    // no change
pitch_rate = -gyroADC[1]    // negate: INAV positive = nose down, we need nose up = positive
yaw_rate   = -gyroADC[2]    // negate: INAV positive = nose left, we need nose right = positive
```

**NN input convention (data.dat columns gyrP/Q/R):**
Gyro rates are in **rad/s**, unscaled. The NN learns the natural scale from training.
- `gyrP` = p (roll rate, rad/s)
- `gyrQ` = q (pitch rate, rad/s)
- `gyrR` = r (yaw rate, rad/s)
Typical range: ±10 rad/s (≈ ±560 deg/s at max roll rate).
No normalization by max rate — avoids baking INAV rate config into the NN.

## Altitude DATUM as an INTERFACE quantity (041, 2026-08-17)

⚠️ **New status, and filed here for the same reason the accelerometer was** (see the 041 section above):
altitude is about to become an NN input (`SPECIFIC_ENERGY`), and the moment a quantity crosses the
sim↔flight boundary its **datum** is as load-bearing as its units and its frame. This project has already
paid once this month for an unstated frame convention; a datum is the same hazard wearing different
clothes.

### The three candidate datums, and why only one works

| datum | sim | flight | verdict |
|---|---|---|---|
| **virtual-frame z** | engage-relative, z ≈ 0 at start | same | ❌ arbitrary offset; "energy" would be measured from wherever the run happened to begin |
| **AGL** | `-(pos.z + SIM_INITIAL_ALTITUDE)` — ground-referenced, the sim knows the ground | ❌ **not available** — `resolveEngageArena` centres the band on the engage point precisely because the aircraft has no ground reference | ❌ **sim-only**. Would train against a quantity flight cannot reproduce |
| **height above the arena floor ("hard deck")** | `floor_agl_m` = **25 m AGL** | `floor_z_ned = z_engage + down_m` | ✅ **defined identically in both**, because both carry an explicit floor |

### Why the hard deck is the RIGHT answer, not merely the available one

**Operator 2026-08-17**: *"elevation above a datum is more energy — would be weird for elevation to go
negative with velocity positive… specific energy is arguably above the arena floor."*

That objection is the strongest argument FOR this datum. With the hard deck:

```
Es = h_hd + v²/2g          h_hd = height above the arena floor
```

**`Es` is non-negative in every VALID state, and `Es < 0` is definitionally out of bounds** — below the
floor is an egress, not a flight condition. So the variable never has to represent a nonsensical
combination, and no clamping is needed to keep it sane. (Measured on the 041 t1 run: **0 of 129 519 ticks
below the deck**.) A datum that put the zero anywhere else — engage point, ground, sea level — would allow
"negative height, positive speed" states that are physically fine but semantically confusing, exactly the
weirdness the operator named.

It also makes energy and containment **one concern instead of two**: running out of energy and hitting the
floor become the same failure.

### ✅ RESOLVED 2026-08-18 — placement is now identical, by construction

The placement gap below was real and is closed. It used to read:

> sim placed the floor **20 m below** engage (5–100 m AGL band, engage at 25 m AGL) while flight placed it
> **47.5 m below** (`resolveEngageArena`, K = 47.5). Same 95 m band, different placement — a live
> sim-to-real gap affecting `DIST_TO_BOUNDARY`, which TA01 measured as the **third most important input**.

**Now: ONE arena, radius 70 m, vertically ASYMMETRIC — `+50 m up / −30 m down` about the arm point.**
In sim that lands at floor **25** / arm **55** / ceiling **105** m AGL.

⭐ `resolveEngageArena(...).virtual_arena` is an **exact identity** on the training `FlightArena`, at any
engage altitude and any asymmetry, because it derives the up and down extents separately from where the arm
point sits inside the band. Sim and flight stopped agreeing by convention and now agree by construction.
`tests/arena_tests.cc` and `tests/arena_recenter_tests.cc` assert it.

⚠️ **The band is asymmetric on purpose, and sized to the CHASE, not the rabbit.** The M1 rabbit climbs
34.98 m above the arm point and descends 2.74 m below it; the chase manoeuvres, bleeds energy and settles.
A first cut at +60/−10 (sized to the rabbit) killed 16 of 16 scenarios on the deck within 4.9 s.

⛔ **The arena is RELATIVE, not absolute.** Radius about the arm point, floor and ceiling at ∓extent. The AGL
figures are only where the band lands *in sim*, where the ground is known. Keeping it above terrain and
inside the site's 400 ft working envelope are **arm-time operator responsibilities** — nothing in the code
enforces an absolute altitude, and nothing should.

📐 **Every hop in the eleven-stage chain, measured**:
[`specs/041-m2-depth/toolchain-datum-validation.md`](../specs/041-m2-depth/toolchain-datum-validation.md).
⚠️ That file is feature-scoped and will age out with 041 — **this section is the durable home**; fold
anything still load-bearing back here before the feature closes.

**Standing guidance until it is resolved**: normalise altitude-derived inputs by the **band**
(`ceiling − floor`, 95 m in both), so a slot means *"fraction of my usable vertical band"* rather than
metres from a differently-placed floor. That makes the *inputs* agree while the *geometry* still differs —
a mitigation, and it should be labelled as one wherever it is used.

## Units at the CRRCSim boundary (041, 2026-08-18)

⚠️ **CRRCSim is FOOT-native; autoc is METRE-native. The bridge converts, and the conversion is easy to
forget because both units appear in the same files.** Operator 2026-08-18: *"Check CRRCSim unit of measure.
It is both feet and meters… This is silly but hey we aren't crashing probes into mars quite yet."*

| side | units | frame |
|---|---|---|
| CRRCSim FDM (`eom01`) | **feet, ft/s** — `v_P_CG_Rwy`, `v_V_local_rel_ground`, `getAccel`, `getGravity` | north/east/down |
| autoc `AircraftState` | **metres, m/s** | NED, but **virtual** — engage-relative; the origin sits at **55 m AGL** in sim (`SIM_INITIAL_ALTITUDE = −55`) |
| conversion | `FEET_TO_METERS = 0.3048`, `inputdev_autoc.h:53` | applied at the bridge, both directions |

**Both directions appear**: FDM→autoc multiplies (`vGround(0) * FEET_TO_METERS`), autoc→FDM divides
(`entryAltOffset / FEET_TO_METERS`). A missing conversion is therefore a 3.28× error in one direction and
0.305× in the other — large enough to be obvious in a trajectory, **but not always obvious in a scalar**.

### ⭐ Worked example — the `82` that looked like a discrepancy

`crrcsim/autoc_config.xml` carried `<launch altitude="82">`. 041 briefly recorded this as an **open
reconciliation** against `SIM_INITIAL_ALTITUDE = −25`, on the assumption both were metres. They are not —
and the full arithmetic is worth having, because it is what the launch value is *derived from*:

```
crrc_main.cpp:  Altitude = launch.altitude + zLow + groundHeight       (all FEET)
                zLow        = 0.125 ft   hb1_streamer.xml <wheels units="0">, max wheel z
                groundHeight= −0.1  ft   BuiltinSceneryDavis::getHeight(), a flat plane
    82 + 0.125 − 0.1 = 82.025 ft × 0.3048 = 25.0012 m   vs  25 m  → agree to 1.2 mm
```

**The launch altitude is in CRRCSim's native FEET and it matched the autoc virtual origin exactly.** There
was never a discrepancy — only a unit assumption, plus two unstated terms. Kept here rather than deleted,
because the *shape* of the mistake (comparing two numbers without checking they share a unit) is what this
section exists to prevent.

⚠️ **CURRENT VALUE: `<launch altitude="180.421">`** — inverting the same formula for a 55 m origin. And
there are **TWO** files carrying it: `crrcsim/autoc_config.xml` (headless training worker) and
`crrcsim/autoc_config-eval.xml` (**visual** worker, `scripts/crrcsim-visual.sh`). When the frame moved,
only the first was updated; the visual craft then spawned at 25.0012 m AGL against a 25 m hard deck —
**1.2 mm of clearance** — and every scenario floor-egressed within a second, which read as a broken policy.
`ArenaDatum.EveryCrrcsimLaunchAltitudeMatchesTheVirtualOrigin` now parses **both** files and checks each
against `SIM_INITIAL_ALTITUDE`.

### One consequence worth keeping

`autoc/eval/specific_force.h` deliberately takes gravity as a **parameter in the caller's units** rather
than assuming 9.81, precisely so the ft-native FDM path stays unit-free: `(a_world − g_world)/g` is
dimensionless whichever unit both sides are in. Prefer that pattern for any new FDM-derived quantity —
including `Es`/`Ps`, where the altitude and speed terms must share a unit before they are added.

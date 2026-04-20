# Data Model — 024 Sensor Signals

**Feature**: [024-sim-real-fidelity](./spec.md)
**Source of truth**: [docs/COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md),
[docs/INAV_BLACKBOX.md](../../docs/INAV_BLACKBOX.md),
[research.md](./research.md).

This feature is primarily about **verifying existing signals**, not creating
new entities. The "data model" here is the set of sensor signals the NN sees
or touches, tabulated as a single cross-source dictionary: which columns in
which source-format (blackbox CSV, xiao log, sim data.dat) correspond to
which canonical signal, and what their units and frames are.

## Canonical Signals (the NN's world)

These are the nine families of signals that appear in analysis. Each canonical
signal has a single stack-wide convention (the "target"); each source-format
emits it in a source-specific convention and is transformed at the read
boundary.

### POS — Aircraft position in world frame

- **Canonical**: `(N, E, D)` meters, NED frame, world origin at arm/launch point.
- **Target NN consumer**: `AircraftState::getPosition()` — virtual NED m (raw
  minus `pathOriginOffset`).
- **State**: per-sample vector of 3 floats.

### VEL — Aircraft velocity in world frame

- **Canonical**: `(VN, VE, VD)` m/s, NED frame. Downward +.
- **Target NN consumer**: `AircraftState::getVelocity()`.
- **State**: per-sample vector of 3 floats.

### QUAT — Aircraft orientation

- **Canonical**: `q_EB = (w, x, y, z)`, Hamilton convention, scalar-first,
  earth→body rotation, aerospace RHR (pitch = nose-up-positive; roll = right-
  wing-down-positive; yaw = nose-right-positive).
- **Target NN consumer**: `AircraftState::getOrientation()`.
- **State**: per-sample unit quaternion.

### GYRO — Body-frame angular rates

- **Canonical**: `(p, q, r)` rad/s, aerospace RHR. p = roll rate (body +X);
  q = pitch rate (body +Y, nose-up +); r = yaw rate (body +Z, nose-right +).
- **Target NN consumer**: NN inputs `gyro_p/q/r` at `NNInputs[30..32]`.
- **State**: per-sample vector of 3 floats.

### ACCEL — Body-frame specific force

- **Canonical**: `(ax, ay, az)` m/s², body FRD. At level rest, accel = `(0, 0, +g)`.
- **Target NN consumer**: not directly input to NN today; used as cross-check
  signal in WI1 (accel vs gravity + centripetal in turn).
- **State**: per-sample vector of 3 floats.

### MAG — Body-frame magnetic field direction

- **Canonical**: unit direction vector in body frame. Magnitude ignored.
- **Target NN consumer**: not directly input; used in WI1 heading cross-check.
- **State**: per-sample direction (normalized).

### EULER — Aircraft attitude as Euler angles

- **Canonical**: `(φ, θ, ψ)` radians (or degrees at display time), ZYX intrinsic,
  aerospace convention. roll = right-wing-down +, pitch = nose-up +, yaw =
  nose-right +.
- **Target NN consumer**: not directly consumed; used in WI1 cross-check
  against quat extraction.
- **State**: derived from QUAT; per-sample 3-tuple.

### TARGET — Direction to rabbit (NN perception)

- **Canonical**: unit vector `(tx, ty, tz)` in body frame. `dist` m as scalar.
- **Target NN consumer**: `NNInputs[0..23]` as 6-element history × 3
  components + 6-element distance history.
- **State**: per-tick, with 6-sample temporal history.

### CMD — NN output commands

- **Canonical**: `(pitch_cmd, roll_cmd, thr_cmd)`, each ∈ [−1, +1]. pitch +1
  = pull up, roll +1 = right wing down, thr +1 = full power.
- **Target**: NN output array `out[0..2]`.
- **State**: per-tick scalar triplet.

## Source-Format Mapping

Each canonical signal above maps to specific columns / fields in each source.
The transforms are what the analysis tool applies at the read boundary.

### Source: INAV blackbox CSV (`flight-results/*/blackbox_log_*.csv`)

| Canonical | CSV column(s) | Raw units/frame | Transform to canonical |
|---|---|---|---|
| POS | `navPos[0..2]` | cm, NEU | `/100`, negate Z |
| VEL | `navRealVel[0..2]` | cm/s, NEU | `/100`, negate Z |
| QUAT | `quaternion[0..3]` | ×10000 int, body→earth NEU, INAV pitch | `/10000`; NEU→NED + pitch sign (WI5-characterized) |
| GYRO | `gyroADC[0..2]` | deg/s, INAV body | `× π/180`; negate index 1, 2 |
| ACCEL | `accADC[0..2]` | acc_1G/256 LSB, body FRD | `× (1/256) × 9.81` |
| MAG | `magADC[0..2]` | raw ADC, body | normalize |
| EULER | `attitude[0..2]` | decideg, `[roll, pitch_INAV, yaw]` | `/10` then `× π/180`; negate pitch |
| TARGET | (not in blackbox) | — | — (NN-perception not reconstructible from pilot flight) |
| CMD | (not in blackbox) | — | — (no NN command during pilot flight) |

### Source: Xiao flight log (`flight-results/*/flight_log_*.txt`)

Lines like:
```
#<seq> <xiao_ms> <inav_ms> i NN: ... tX=[..] tY=[..] tZ=[..] d=[..] cr=<cr> q=[w,x,y,z] as=<as> g=[p,q,r] out=[pt,rl,th] rc=[..]
#<seq> <xiao_ms> <inav_ms> i Nav State: pos_raw=[..] pos=[..] vel=[..] quat=[w,x,y,z] gyro_raw=[..] armed=Y ... autoc=Y rabbit=Y path=0
```

| Canonical | Log field | Raw units/frame | Transform |
|---|---|---|---|
| POS (virtual) | `Nav State pos=[..]` | m, NED virtual (post-offset) | (none — already canonical) |
| POS (raw) | `Nav State pos_raw=[..]` | m, NED raw | (for debugging; not canonical) |
| VEL | `Nav State vel=[..]` | m/s, NED | (none — already canonical) |
| QUAT | `Nav State quat=[..]` or `NN q=[..]` | unit quat, q_EB aerospace | (claimed; WI5 verifies) |
| GYRO | `Nav State gyro_raw=[..]` | raw INAV | (pre-transform, for debugging) |
| GYRO | `NN g=[p,q,r]` | rad/s, aerospace | (none — already canonical) |
| TARGET | `NN tX=[..] tY=[..] tZ=[..] d=[..]` | body FRD unit vec + m | (none — already canonical) |
| CMD | `NN out=[pt, rl, th]` | [−1, +1] | (none — already canonical) |

### Source: Sim data.dat (`eval-results/*/tier*/data.dat`, `test4-data.dat`)

Whitespace-separated with header row. Columns include:
`Scn Bake Pth/Wnd:Step: Time Idx tgX-9..tgX+5 tgY-9..tgY+5 tgZ-9..tgZ+5 ds-9..ds+5 dd/dt qw qx qy qz vel gyrP gyrQ gyrR outPt outRl outTh pathX pathY pathZ X Y Z vxBdy vyBdy vzBdy dhome dist along rabVl stpPt mult rampSc`

| Canonical | data.dat column(s) | Raw units/frame | Transform |
|---|---|---|---|
| POS | `X Y Z` | m, NED virtual | (none) |
| VEL | `vxBdy vyBdy vzBdy` | m/s, **body frame** | rotate to world via `q_EB.inverse()` for world-NED |
| QUAT | `qw qx qy qz` | q_EB aerospace | (none) |
| GYRO | `gyrP gyrQ gyrR` | rad/s, aerospace | (none) |
| TARGET | `tgX0..5 tgY0..5 tgZ0..5` | body FRD unit vec | (none); `dist = ds0` (m) |
| CMD | `outPt outRl outTh` | [−1, +1] | (none) |

## Validation Rules (contract)

Per the clarification: directional pass/fail only (sign-inversion gates fail);
correlation reviewed not gated.

For each cross-check comparing two signals `A` vs derived-B:

- **Sign**: regression slope must be positive where physical expectation is
  positive (or negative where expectation is negative). Sign-flip = **fail**.
- **Correlation**: Pearson `r` reported and visually compared to sim baseline.
  Anomalously low `r` (say < 0.3 where sim shows > 0.6) flagged as warning,
  not a fail.

## State Transitions

The canonical signals are per-sample, not stateful. The only state model in
this feature is **history buffers**:

- `targetDirHistory_`, `distHistory_`, `timeHistory_` — circular buffers in
  `AircraftState`.
- Clarified semantics: entry at index `[t]` stores the value **as observed
  at sim tick t**, not at the time of recording (which is t anyway in normal
  flow, but matters at engage-reset).
- `resetHistory(targetPos, pathTangent)` pre-fills all slots with the
  current-tick direction and distance.

## Entity Summary

| Entity | Lifetime | Identity | Relationships |
|---|---|---|---|
| Canonical sensor sample | One sim tick (100 ms) | `(time, source)` | 1-to-1 with source-format row |
| Source-format row | One line/record | `(source, timestamp)` | produces one canonical sample |
| Cross-check pair | One analysis run | `(signal_A, signal_B)` | derives derived-B from QUAT, compares to logged A |
| History buffer entry | `HISTORY_SIZE` ticks | `(scenario, tick_index)` | ordered; oldest evicted |

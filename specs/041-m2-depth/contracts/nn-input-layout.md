# Contract — NN input layout (both modes)

**Surface**: `include/autoc/nn/nn_inputs.h`, `include/autoc/nn/topology.h`, `src/nn/evaluator.cc`,
`tools/nn2cpp.cc`, `xiao/src/generated/`, `specs/019-improved-crrcsim/sim_response.py`.

**Why this is a contract**: field declaration order **is** on-disk byte order for cereal, `data.dat`,
`nn2cpp` codegen and the xiao forward pass. A layout change that compiles but disagrees across those
consumers produces plausible-but-wrong flight behaviour, which is the failure mode Constitution V exists to
prevent.

## Craft state today, and what completes it

Craft state is **identical in both modes — 8 slots**:

```text
QUAT_W QUAT_X QUAT_Y QUAT_Z     attitude          (4)
AIRSPEED                        cruise-normalized (1)
GYRO_P GYRO_Q GYRO_R            angular rate      (3)
```

The inertial block is therefore **3 rate, 0 accel — half an IMU.** Craft state is 6-DOF (operator
2026-08-07): adding `ACCEL_X/Y/Z` *completes* the sensor rather than adding a feature, and it maps onto
hardware exactly — the xiao carries an **LSM6DS3, a 6-DOF IMU (3 accel + 3 gyro)**. After the change craft
state is 11 slots, of which 6 are the inertial block.

## Slots added

Appended at the documented end-of-struct point in **both** `NNInputs` and `TrackerInputs`, in this order.
`ACCEL_*` sits **adjacent to `GYRO_*`** so the 6-DOF block is visible in the enum, the metadata table, and
every `data.dat` header:

```text
IN_ENVELOPE     float, {0,1}
ENVELOPE_SECS   float, [0,1]    min(seconds_in_envelope / FitStreakRampSec, 1)
ACCEL_X         float, ≈[-2,2]  body specific force / kAccelScale_g   (longitudinal)
ACCEL_Y         float, ≈[-2,2]                                        (lateral)
ACCEL_Z         float, ≈[-2,2]                                        (normal — the load axis)
```

## Counts

| symbol | before | after |
|---|---:|---:|
| `NN_INPUT_COUNT` / `PathgenInput::COUNT` | 37 | **42** |
| `TRACKER_NN_INPUT_COUNT` / `TrackerInput::COUNT` | 58 | **63** |
| `TRACKER_NN_TOPOLOGY_STRING` | `"58,32,16r,7"` | `"63,32,16r,<out>"` |

`<out>` is 7 if a predictor head survives US5, 3 if retired. Weight-count `static_assert`s must be
**recomputed**, never relaxed.

## Semantics — binding definitions

### `IN_ENVELOPE`

The **observable** scoring-envelope condition, never the fitness machinery's internal streak counter
(FR-015).

- **M1**: **exact, in sim and in flight.** `stepPoints ≥ FitStreakThreshold` from the same along/lateral
  decomposition the objective uses. Flight M1 carries the same signals — a virtual target location and the
  chase's own location — so this input has **no sim-to-real gap**, and the xiao firmware carries the cone
  constants (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`, `FitStreakThreshold`).
- **Computed once, in the tick loop** (FR-018a) — the same result feeds the NN input **and** the fitness
  accumulation, replacing today's post-hoc computation over recorded states. Two independent computations
  of this quantity are forbidden: the disagreement they permit is the exact failure this feature exists to
  remove.
- **M2**: estimated from perception — both beacons CEP-visible **and** separation within
  `[EnvelopeSpanLo, EnvelopeSpanHi]` **and** pair centroid within `EnvelopeCentroidRadius`. Slots are
  reserved and populated in the bundle; estimator fidelity is characterised in Phase C/D, not assumed.

### `ENVELOPE_SECS`

`min(consecutive_seconds_in_envelope / FitStreakRampSec, 1)`.

- Resets **on envelope exit only** — matching the reward's own reset condition
  (`fitness_computer.cc:60-64`), so the observation is a faithful proxy of the multiplier. **Not** on
  regime change.
- Millisecond-based, so a cadence change re-derives rather than silently rescales (FR-016) — the same
  discipline as `kNNHistoryLagsMsec`.
- 1.0 means "multiplier saturated", which is the decision-relevant boundary.

### `ACCEL_*`

**Specific force in body frame — gravity included — normalized by `kAccelScale_g`.**

```text
spec_force_body = R(quat)ᵀ · (a_world − g_world)      then / kAccelScale_g
```

⚠️ **This is the single most error-prone line in the feature.** A real accelerometer measures specific
force; FDM kinematic acceleration is a different quantity. Feeding the latter puts a **constant ~1 g offset
into the normal axis** — invisible in sim, wrong in flight, and exactly the Principle VII shape (a silently
wrong value that looks correct). A test pins this: **in steady level flight the normal channel reads ≈1 g,
not ≈0.**

### On-target source: the TRANSFORMED field, never a raw sensor read

⚠️ **This is a second, independent way to get the accel wrong**, and it is invisible in sim: taking a
pre-alignment sensor value would bake each board's own misalignment into the input — differently on bench
(roll = −16) than on flight.

| use | do not use |
|---|---|
| **`acc.accADCf[]`** — post `applySensorAlignment` + `applyBoardAlignment`, divided by `acc.dev.acc_1G`, then filtered (`acceleration.c:563-568`, `:593-598`). **Units: g.** | `accADC[]` (file-static, `acceleration.c:73`) or `acc.dev.ADCRaw[]` — pre-alignment |

This mirrors the gyro exactly — `gyro.gyroADCf` goes through the same two alignment calls
(`gyro.c:438-442`) — which is why the existing `MSP2_AUTOC_STATE` gyro extension is correct and is the right
template to copy. The AHRS `orientation` quaternion is likewise an aligned-frame product.

### Convention confirmed against hardware, not assumed

`docs/COORDINATE_CONVENTIONS.md` §"Ground Verification Results (bench 2026-03-30)" already contains a
measured table on the bench FC (`MAMBAF722_2022A`, board alignment roll = −16):

| attitude | measured |
|---|---|
| level | `[~0, ~0, +2050]` |
| right wing down 90° | `[~0, +2060, ~0]` |
| nose up 90° | `[+2050, ~0, ~0]` |

⚠️ **Units caveat**: those are **blackbox `accSmooth` counts** (`acc_1G ≈ 2048`), *not* runtime
`acc.accADCf`, which is the same vector already divided by `acc_1G`. So `+2050 counts` ⇔ `+1.0 g`. The
**axes and signs transfer directly**; only the scale differs.

The load-bearing confirmation: **level reads +1 g on the normal axis, not 0** — INAV already reports
specific force including gravity, exactly the semantics this contract requires. That makes the sim-side
`acc − g` decision a match to hardware rather than merely a defensible choice.

**No history window on these channels** — decided, with a recorded fallback ladder (research.md R1):
instantaneous (+3) → **differentiation**, i.e. jerk channels (+3) → full 6-lag history (+18). Craft state
carries no history today (target gets 6 lags, self gets none) and the recurrent layer is the intended
memory. If temporal depth proves necessary, the jerk channel is the first step and has direct in-tree
precedent — `SPAN_RATE` and `CLOSING_RATE` are both "derivative instead of history" already, with a
cadence-invariant formulation to copy.

## Invariants (each enforced by an existing or new assertion)

1. `sizeof(struct) == COUNT * sizeof(float)`; no padding; float-aligned.
2. Enum order == struct field order == metadata table order.
3. Metadata table length == `COUNT` (existing `static_assert`).
4. Every new slot has a metadata row — name, short name, print width — since the ablation tool selects
   columns by name and `dmp-dump`/analytics label by short name.
5. New members annotated `// raw-ok: NN-byte-format buffer` (Constitution VI whitelist).
6. `kNNHistoryLayoutVersion` unchanged — the history window is untouched.

## Tests required

| test | asserts |
|---|---|
| layout/count | struct size, enum count, meta-table length agree for both modes |
| slot semantics | `IN_ENVELOPE` is 0/1; `ENVELOPE_SECS` monotone within a streak and 0 immediately after exit; saturates at 1 |
| cadence invariance | `ENVELOPE_SECS` for a given wall-clock duration is identical at two cadences |
| **specific force** | steady level flight → normal channel ≈1 g (catches the kinematic-vs-specific error) |
| **sign convention** | a positive pull-up produces the documented sign on the normal channel |
| M1 exactness | `IN_ENVELOPE` agrees tick-for-tick with the objective's own threshold decision |
| **single source of truth** | the step score feeding `IN_ENVELOPE` and the step score feeding fitness are the *same value*, not two computations that agree |
| **refactor equivalence** | fitness on a fixed genome is **bit-identical** before and after moving the computation from recorded states into the tick loop — otherwise the objective changed silently |
| xiao parity | `nn2cpp`-generated forward pass matches the desktop forward pass on a fixed input vector |

## Blast radius checklist (from prior input-count changes)

`topology.h` · `nn_inputs.h` · `src/nn/evaluator.cc` (both gather functions) · `src/autoc.cc` (data.dat
header/format) · `aircraft_state.h` (NN block size + serialization) · `tools/nn2cpp.cc` ·
`xiao/src/generated/` · `tests/contract_evaluator_tests.cc` · `tests/nn_evaluator_tests.cc` ·
`specs/019-improved-crrcsim/sim_response.py`.

**Gate**: `bash scripts/rebuild.sh` green **and** `pio run -e xiaoblesense_arduinocore_mbed` green. A
`CMakeLists.txt` touch requires a clean `scripts/rebuild-perf.sh`, operator-driven.

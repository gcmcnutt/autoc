# 032 — Data Model (Phase 1)

Entities, schemas, validation rules, and per-tick flow for the derived perceptual features. Anchored to [spec.md](./spec.md) clarifications Q1–Q8 and [research.md](./research.md) R1–R9.

> **Heritage**: this extends the 030 tracker-mode data model ([specs/030-tracker-mode/data-model.md](../030-tracker-mode/data-model.md)) by growing the NN input vector from 45 to 54. The 030 pre-feature surface — beacon NDC history, aircraft state, arena-awareness — carries forward unchanged.

## 1. `TrackerInputs` struct (extended 45 → 54)

**Per spec §1.5 Feature B**: 9 new fp32 slots appended after the existing 45. Layout order matches the extended `TrackerInput` enum (declaration order = cereal byte order = data.dat column order).

### 1.1 New slots (in order)

| Slot | Type | Range | Units | Description |
|---|---|---|---|---|
| `beacon_pair_span_tm5` | `float` | [0, ~2.83] | screen-fraction | NDC distance between port and starboard beacon centroids at t−500ms |
| `beacon_pair_span_tm4` | `float` | [0, ~2.83] | screen-fraction | t−400ms |
| `beacon_pair_span_tm3` | `float` | [0, ~2.83] | screen-fraction | t−300ms |
| `beacon_pair_span_tm2` | `float` | [0, ~2.83] | screen-fraction | t−200ms |
| `beacon_pair_span_tm1` | `float` | [0, ~2.83] | screen-fraction | t−100ms |
| `beacon_pair_span_now` | `float` | [0, ~2.83] | screen-fraction | t (now) |
| `span_rate` | `float` | small, signed | span-Δ per tick | `beacon_pair_span_now − beacon_pair_span_tm1`; one-tick raw diff |
| `target_tilt_sin` | `float` | [−1, +1] | dimensionless | `sin(atan2(y_r − y_l, x_r − x_l))` over NDC |
| `target_tilt_cos` | `float` | [−1, +1] | dimensionless | `cos(atan2(y_r − y_l, x_r − x_l))` over NDC |

> **Range note**: NDC coordinates span [-1, +1] in each of x and y. The largest geometrically possible 2D distance between two points in that box is `2√2 ≈ 2.828`. Typical operational range is `0.0` (beacons coincident at extreme range) through `~1.0` (target filling roughly half the camera FOV).

### 1.2 Full enum + struct shape

```cpp
// include/autoc/nn/nn_inputs.h (extended)

enum class TrackerInput : uint16_t {
  // ============================================================
  // EXISTING 030 SLOTS (0..44) — unchanged ordering and semantics
  // ============================================================
  BEACON_L_X_TM5 = 0, ..., BEACON_R_CEP_NOW,   // 36 slots (6 history × 2 beacons × {x, y, cep})
  QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,              // 4 slots
  AIRSPEED,                                    // 1 slot (cruise-normalized)
  GYRO_P, GYRO_Q, GYRO_R,                      // 3 slots (rad/s)
  DIST_TO_BOUNDARY_ALONG_VEL,                  // 1 slot (tanh)

  // ============================================================
  // 032 PHASE 1 NEW SLOTS (45..53)
  // ============================================================
  BEACON_PAIR_SPAN_TM5 = 45,
  BEACON_PAIR_SPAN_TM4,
  BEACON_PAIR_SPAN_TM3,
  BEACON_PAIR_SPAN_TM2,
  BEACON_PAIR_SPAN_TM1,
  BEACON_PAIR_SPAN_NOW,
  SPAN_RATE,
  TARGET_TILT_SIN,
  TARGET_TILT_COS,
  COUNT  // = 54
};

struct TrackerInputs {  // raw-ok: NN-byte-format struct
  // ----- existing 45 fields unchanged -----
  float beacon_l_x[6];     // raw-ok: NN-byte-format buffer
  float beacon_l_y[6];     // raw-ok: NN-byte-format buffer
  float beacon_l_cep[6];   // raw-ok: NN-byte-format buffer
  float beacon_r_x[6];     // raw-ok: NN-byte-format buffer
  float beacon_r_y[6];     // raw-ok: NN-byte-format buffer
  float beacon_r_cep[6];   // raw-ok: NN-byte-format buffer
  float quat_w, quat_x, quat_y, quat_z;
  float airspeed;
  float gyro_p, gyro_q, gyro_r;
  float dist_to_boundary_along_vel;

  // ----- 032 phase 1 NEW fields -----
  float beacon_pair_span[6];   // raw-ok: NN-byte-format buffer — screen-fraction, 100ms history grid
  float span_rate;             // raw-ok: NN-byte-format buffer — one-tick diff, signed
  float target_tilt_sin;       // raw-ok: NN-byte-format buffer — sin(atan2(dy, dx)) over NDC port→starboard line
  float target_tilt_cos;       // raw-ok: NN-byte-format buffer — cos(atan2(dy, dx))
};

static_assert(sizeof(TrackerInputs) == 54 * sizeof(float),
              "TrackerInputs layout must be contiguous float[54] with no padding");
static_assert(static_cast<int>(TrackerInput::COUNT) == 54,
              "TrackerInput::COUNT must equal 54 per 032 phase 1");
```

### 1.3 `kTrackerInputMeta` extension

Nine new entries appended (matching enum order):

```cpp
{"BEACON_PAIR_SPAN_TM5", "spn-5", 7},   {"BEACON_PAIR_SPAN_TM4", "spn-4", 7},
{"BEACON_PAIR_SPAN_TM3", "spn-3", 7},   {"BEACON_PAIR_SPAN_TM2", "spn-2", 7},
{"BEACON_PAIR_SPAN_TM1", "spn-1", 7},   {"BEACON_PAIR_SPAN_NOW", "spn0",  7},
{"SPAN_RATE",            "dspn",  7},
{"TARGET_TILT_SIN",      "tltS",  7},
{"TARGET_TILT_COS",      "tltC",  7},
```

Static asserts in `nn_inputs.h` carry forward (`kTrackerInputMeta` length must equal `TrackerInput::COUNT`).

### 1.4 Validation rules (asserted in `gather_tracker_inputs_tests.cc`)

- `beacon_pair_span[i] ∈ [0, 2√2 + ε]` for all i (geometric bound on NDC point distance)
- `target_tilt_sin² + target_tilt_cos² ≈ 1.0` (within fp32 epsilon) unless CEP-gated (in which case `(sin, cos) = (0, 1)` and the identity holds trivially)
- When `beacon_l_cep[5] ≥ CepGateThreshold || beacon_r_cep[5] ≥ CepGateThreshold`:
  - `beacon_pair_span[5] == 0.0f`
  - `span_rate == 0.0f`
  - `target_tilt_sin == 0.0f`
  - `target_tilt_cos == 1.0f`

## 2. `TrackerHistoryWindow` (extended)

`include/autoc/eval/tracker_stepper.h`'s `TrackerHistoryWindow` (carrying 6-tick NDC history) gains a parallel `span[6]` slot to track span history before it's copied into `TrackerInputs`.

```cpp
struct TrackerHistoryWindow {
  // ----- existing 030 fields -----
  float left_x[6];    // raw-ok: NN-byte-format primitive
  float left_y[6];    // raw-ok: NN-byte-format primitive
  float left_cep[6];  // raw-ok: NN-byte-format primitive
  float right_x[6];   // raw-ok: NN-byte-format primitive
  float right_y[6];   // raw-ok: NN-byte-format primitive
  float right_cep[6]; // raw-ok: NN-byte-format primitive

  // ----- 032 phase 1 NEW field -----
  float span[6];      // raw-ok: NN-byte-format primitive — cached beacon-pair span across history
};
```

**Why cache span in the history window instead of recomputing per gather call**: per-tick span is computed in `projectAndShiftHistory` once when the NDC pair is freshly projected; the history shift then carries that scalar through the same memcpy pattern as the NDC slots. Recomputing from NDC slots in `gather_tracker_inputs` would (a) duplicate the math and (b) recompute over the CEP-gated substituted NDC values during the blind-tick fallback path, producing inconsistent gating semantics. Caching is cheaper AND semantically cleaner.

## 3. Per-tick data flow (extended)

```text
┌─────────────────────────────────────────────────────────────────┐
│ tracker_stepper.cc::projectAndShiftHistory (or crrcsim helper)  │
│                                                                  │
│ 1. Shift slots [1..5] → [0..4] for ALL history channels         │
│    (NDC channels — unchanged from 030)                           │
│    + NEW: shift `span[1..5] → span[0..4]`                       │
│                                                                  │
│ 2. Project port + starboard beacons (unchanged from 030)        │
│    → BeaconObservation left, right (each w/ screen_x, _y, cep)  │
│                                                                  │
│ 3. Write NEW NDC into slot [5] (unchanged from 030)             │
│                                                                  │
│ 4. NEW: compute span[5] from (left, right) NDC                  │
│    if (left.cep >= CepGateThreshold || right.cep >= CepGate-)   │
│      span[5] = 0.0f                                              │
│    else                                                          │
│      span[5] = norm(right.xy - left.xy)                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ src/nn/evaluator.cc::gather_tracker_inputs                       │
│                                                                  │
│ 1. Copy history.{left,right}_{x,y,cep}[i] → out.beacon_*[i]     │
│    (unchanged from 030)                                          │
│                                                                  │
│ 2. Copy aircraft state slots (unchanged from 030)               │
│                                                                  │
│ 3. NEW: copy history.span[i] → out.beacon_pair_span[i]          │
│                                                                  │
│ 4. NEW: compute span_rate = span[5] - span[4]                   │
│    (no extra CEP-gate — span already gated upstream;             │
│     a gated tick's span = 0 makes rate reflect "step back to 0")│
│                                                                  │
│ 5. NEW: compute target_tilt_sin, target_tilt_cos                │
│    if (history.left_cep[5] >= CepGateThreshold ||                │
│        history.right_cep[5] >= CepGateThreshold)                 │
│      out.target_tilt_sin = 0.0f                                  │
│      out.target_tilt_cos = 1.0f                                  │
│    else                                                          │
│      dx = history.right_x[5] - history.left_x[5]                 │
│      dy = history.right_y[5] - history.left_y[5]                 │
│      if (sqrt(dx*dx + dy*dy) < kTiltDegenerateEpsilon)           │
│        out.target_tilt_sin = 0.0f; out.target_tilt_cos = 1.0f    │
│      else                                                        │
│        θ = atan2(dy, dx)                                         │
│        out.target_tilt_sin = sin(θ)                              │
│        out.target_tilt_cos = cos(θ)                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
              NN forward pass (54 → ... → 3)
```

### 3.1 Open-question note on `span_rate` under CEP-gate transitions

By design, `span_rate` does NOT apply its own CEP-gate; it derives mechanically from `span[5] - span[4]`. Cases:
- Both ticks visible → real rate, signed
- Both ticks CEP-gated → 0 − 0 = 0 (correct: no signal)
- Visible → gated (acquired-then-lost) → 0 − span_real = negative (looks like recession)
- Gated → visible (lost-then-acquired) → span_real − 0 = positive (looks like approach)

These transition artifacts ARE signal — the NN gets a "something just happened" pulse at the visibility boundary. Documented as intentional. If the bake shows this creates evolutionary instability, phase-1b option is to gate `span_rate` directly (substitute 0 when either current OR previous tick was gated).

## 4. ini schema addition

```ini
[DerivedFeatures]
CepGateThreshold                = 1.25   # Derived features substitute neutral values when EITHER beacon's CEP ≥ this. Default matches kCepSentinelThreshold.

# NOT included in v1 (intentional; see contracts/ini_schema.md):
# - EnableDerivedFeatures or any per-feature toggle (greenfield M2 — B-off attribution
#   comes from git revert to a pre-032 commit, not a runtime flag; per research.md R7)
# - tilt epsilon / span smoothing (operational design has no math-level knobs)
```

## 5. `data.dat` columns

Per the unchanged header-emission pattern in `src/autoc.cc` (walks `kTrackerInputMeta`), the data.dat header for tracker mode automatically gains 9 new columns (`spn-5 spn-4 spn-3 spn-2 spn-1 spn0 dspn tltS tltC`) appended after `dBnd`. The Python parser (`specs/019-improved-crrcsim/sim_response.py`) indexes by header-name match — no parser change required.

## 6. dmp schema impact

Per [research.md R5](./research.md#r5--dmp-schema-growth-without-version-bump-m2-policy-reinforcement):
- The cereal `serialize` template on the `EvalData` (or whatever struct embeds `TrackerInputs` for dmp output) automatically picks up the new fields by virtue of `serialize` walking the struct members. No new `CEREAL_CLASS_VERSION`. No fallback reader.
- Per [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md): /speckit.implement closeout MUST audit the dmp emission path and confirm all 9 new input slots are persisted (and the 3 NN outputs continue to be persisted). Any gap gets reconciled — either by fixing the dmp emission OR by writing a rationale memory describing why a particular slot is intentionally excluded.

## 7. M1 pathgen mode — explicit no-op confirmation

Phase 1 does NOT touch:
- `NNInputs` struct (`nn_inputs.h:35-51`)
- `PathgenInput` enum
- `kPathgenInputMeta`
- `evaluatePathgen` / pathgen-side `gather_*` functions
- M1 dmps' cereal layout

The rebuild-perf.sh regression gate validates this: M1 fitness numbers must remain bitwise-identical to the pre-032 baseline, run-for-run.

## 8. Cross-platform mirroring

| Consumer | Path | 032 impact |
|---|---|---|
| autoc minisim (training) | `src/eval/tracker_stepper.cc`, `src/nn/evaluator.cc` | MODIFIED (phase 1 scope) |
| crrcsim FDM (training) | `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` | MODIFIED (phase 1 scope — wire-equivalent to autoc minisim) |
| xiao firmware (deploy) | `xiao/` | UNCHANGED (xiao tracker-mode port deferred; phase 1 only updates `docs/COORDINATE_CONVENTIONS.md` and `docs/sensor-pipeline.md` as migration prep) |
| renderer / inspect tools | `tools/`, `specs/019-.../sim_response.py` | UNCHANGED CODE; automatically picks up new columns via header-name indexing |

The autoc minisim and crrcsim helper paths MUST stay wire-equivalent — both populate the same `TrackerInputs` struct via the same `gather_tracker_inputs` helper. Phase 1 task breakdown should consolidate the two `projectAndShiftHistory` implementations or at minimum extract a shared helper for the span computation so the cep-gate semantics can't diverge.

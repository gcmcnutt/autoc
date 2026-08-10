# 041 Phase 1 — Data Model

Entities, layout deltas and schema changes. Decisions trace to [research.md](research.md); interface
detail lives in [contracts/](contracts/).

---

## 1. NN input layout — both modes

Current state verified 2026-08-07: `NNInputs` is `float[37]` (`nn_inputs.h:129`), `TrackerInputs` is
`float[58]` (`:352`). Each has a parallel enum (`PathgenInput`, `TrackerInput`) and a per-slot metadata
table (`kPathgenInputMeta`, `kTrackerInputMeta`), all cross-`static_assert`-ed.

### 1.0 The current full input set (verified 2026-08-07)

**M1 — `NNInputs`, 37 slots**

| group | slots | count |
|---|---|---:|
| Target perception, 6 history lags each | `TARGET_X/Y/Z_TM5…NOW` (body-frame unit vector to rabbit), `DIST_TM5…NOW` | 24 |
| Target derived | `CLOSING_RATE` | 1 |
| **Craft state** | `QUAT_W/X/Y/Z`, `AIRSPEED`, `GYRO_P/Q/R` | **8** |
| Arena | `DIST_TO_BOUNDARY`, `INWARD_BODY_X/Y/Z` | 4 |

**M2 — `TrackerInputs`, 58 slots**

| group | slots | count |
|---|---|---:|
| Beacon perception, 6 lags × {x, y, CEP} × 2 beacons | `BEACON_L/R_X/Y/CEP_TM5…NOW` | 36 |
| Derived perception | `BEACON_PAIR_SPAN_TM5…NOW`, `SPAN_RATE`, `TARGET_TILT_SIN/COS` | 9 |
| Perception state | `TIME_SINCE_SEEN` | 1 |
| **Craft state** | `QUAT_W/X/Y/Z`, `AIRSPEED`, `GYRO_P/Q/R` | **8** |
| Arena | `DIST_TO_BOUNDARY_ALONG_VEL`, `INWARD_BODY_X/Y/Z` | 4 |

**Two observations that shape 041:**

1. **Craft state is identical across modes and its inertial block is half-complete** — 3 rate, 0 accel.
   Craft state is 6-DOF; `ACCEL_X/Y/Z` completes it (research.md R1).
2. **History is target-only.** Every perception channel carries 6 lags; every craft-state channel is
   instantaneous. That asymmetry is deliberate and standing — it is why the new accel channels are
   recommended instantaneous too, and why that recommendation is flagged rather than assumed.

### 1.1 New slots (identical semantics in both modes)

| slot | type | range | meaning |
|---|---|---|---|
| `IN_ENVELOPE` | float 0/1 | {0, 1} | 1 when the observable scoring-envelope condition holds this tick |
| `ENVELOPE_SECS` | float | [0, 1] | `min(consecutive-in-envelope seconds / FitStreakRampSec, 1)`; 1 = streak multiplier saturated |
| `ACCEL_X` | float | ≈[−2, 2] | body-frame specific force / `kAccelScale_g`, longitudinal |
| `ACCEL_Y` | float | ≈[−2, 2] | lateral |
| `ACCEL_Z` | float | ≈[−2, 2] | normal — the load-relevant axis |

### 1.2 Count deltas

| | before | after |
|---|---:|---:|
| `NNInputs` / `PathgenInput::COUNT` / `NN_INPUT_COUNT` | 37 | **42** |
| `TrackerInputs` / `TrackerInput::COUNT` / `TRACKER_NN_INPUT_COUNT` | 58 | **63** |

Weight counts and topology strings both change and their `static_assert`s must be recomputed, not
loosened. `TRACKER_NN_TOPOLOGY_STRING` becomes `"63,32,16r,<out>"`, where `<out>` depends on the US5
predictor decision (7 if a head is retained, 3 if retired).

### 1.3 Invariants that must hold after the change

- Field declaration order **is** on-disk byte order for cereal, `data.dat`, `nn2cpp`, and
  `sim_response.py` — appending at the documented end-of-struct point is required, reordering is a
  format break beyond the intended one.
- `sizeof(struct) == COUNT * sizeof(float)`, no padding, float-aligned.
- Every new slot has a metadata row; the existing `static_assert` on table length vs `COUNT` enforces it.
- New members are NN byte-format buffers → raw `float` with `// raw-ok:` annotation (Constitution VI
  whitelist).
- `kNNHistoryLayoutVersion` is **not** affected — the history window is unchanged. The *input count*
  changing is what makes old genomes non-portable, and that is the intended greenfield break.

### 1.4 Derivation, per mode

| | M1 (pathgen) | M2 (tracker) |
|---|---|---|
| `IN_ENVELOPE` | **exact**, in sim **and flight** — `stepPoints ≥ FitStreakThreshold` from the same along/lateral decomposition the objective uses. Flight M1 has the same signals (virtual target location + chase location), so no sim-to-real gap | **estimated** — both beacons CEP-visible, separation within `[lo, hi]`, pair centroid within a centred radius. Design + fidelity characterisation deferred to Phase C/D (research.md R3); slots reserved and populated in A1 |
| `ENVELOPE_SECS` | accumulator over exact flag | accumulator over estimated flag |
| `ACCEL_*` | FDM-derived specific force, body frame | identical |

The fitness machinery's internal streak counter is **never** fed to the NN (FR-015) — both modes derive
from observables.

### 1.5 Single source of truth — the step score moves into the tick loop (FR-018a)

**Today**: the step score / streak is computed **post-hoc**, in `computeScenarioScores`, iterating the
*recorded* `AircraftState` list after the run.

**After**: computed **once per tick in the eval path**, consumed by **both** the NN input gather and the
fitness accumulation, and recorded so the source dmp carries it.

```text
  before                                   after
  ──────                                   ─────
  tick loop ──► record states              tick loop ──► stepScore(t) ──┬──► IN_ENVELOPE / ENVELOPE_SECS
                     │                                                  ├──► fitness accumulation
                     └─ post-hoc ─► stepScore ─► fitness                └──► recorded in the tick record
```

Why it must be one computation, not two: the entire feature exists because the reward is invisible to the
policy. Computing the same quantity in two places would reintroduce exactly the disagreement being fixed —
and it is the same shape as the index-coupling class US1 retires.

⚠️ **Two cautions**:

1. **Fitness-affecting** → A1 bundle only, never mid-bake.
2. **Numerical equivalence must be proven, not assumed.** Post-hoc reads serialized state; inline reads
   live state. If those differ by even a rounding step, the objective has silently changed. The acceptance
   test is a bit-identical fitness reproduction on a fixed genome across the refactor.

**Related backlog**: this is a partial trigger of *"Move `computeScenarioScores` from parent to worker
side"* ([[project_fitness_to_worker_backlog]]) — 041 moves the *step-score* computation worker-side; the
full parent-side aggregation split is out of scope here.

---

## 2. Per-tick record — the structural change

### 2.1 Today

Three collections indexed in parallel per scenario, with the invariant recorded only in comments:

```text
aircraftStateList[i][k]    length 1 + N     ← initial state pushed BEFORE the loop
cameraViewList[i][k]       length N
targetTrajectoryList[i][k] length N
```

Consumers must know `targets[j] ↔ states[j+1]`. Three of four known consumers had it wrong, one of them
the M2 objective, since 030 — and it was invisible in recorded data because neither
`CopiedTargetSample` nor `CameraViewSample` carries a tick index.

### 2.2 After

```text
initialState                       ← named field, stored ONCE beside the list (research.md R5)
tickList[i][k] = { state, cameraView, targetSample }    length N, one index
```

- The pre-loop initial state becomes an **explicit named field**, not `tickList[0]` with sentinel
  members — so the asymmetry is typed and visible instead of accidental, and consumers iterating ticks
  need no offset arithmetic. The objective's `stepIndex - 1` clamp *disappears* rather than moving.
- Tracker-only members remain optional-by-mode: a pathgen record has no camera view or target sample.
  Representation choice (empty vectors vs a mode-tagged variant) is an implementation detail; the contract
  is that a pathgen consumer never reads a tracker member.

### 2.3 Consumers to update

`fitness_decomposition.cc` (objective, `vis_frac`, prediction score), `dmp_dump.cc`, `tools/renderer.cc`,
`source_dmp_loader.cc`, `tracker_stepper.cc`, `crrcsim_tracker_helper.cpp`, `inputdev_autoc.cpp` (the push
sites), plus any analytics reading per-tick arrays.

---

## 3. Dmp schema (`EvalResults`)

| change | driver |
|---|---|
| Parallel per-tick lists → grouped `tickList` + `initialState` | FR-002 |
| Config block recorded per gen (fitness + cadence parameters) | FR-006 |
| `wind_velocity` populated at record time (getter/setter and `dmp_dump` columns already exist; the field has been serialized-but-never-set, zero in every dmp) | FR-008 |
| `simTimeMsec` stamped exactly (no ±1 ms truncation) — **crrcsim submodule** | FR-007 |
| **Version field bumped**; readers fail loud naming both versions; no migration path | FR-009, research.md R6 |

Physics trace is **unchanged** — `PhysicsTraceEntry` already carries `acc[3]`, `omegaDotBody[3]`, `alpha`,
`vRelWind` per tick for every elite reeval and is already serialized. 041 adds a **reader**, not a
recording (research.md R9).

---

## 4. Derived entities (not stored — computed by analysis or fitness)

### 4.1 Regime

| regime | condition |
|---|---|
| `tracking` | in-cone location metric ≥ threshold (`stpPt ≥ 0.5`) |
| `intercept` | below threshold **and** closing (smoothed `d(dist)/dt < 0`) |
| `patrol` | below threshold, not closing |

Established in `dynamics_progress.py:74-80` since 2026-06-10. The threshold is the **same value** as
`FitStreakThreshold`, so *in-tracking* and *streak-maintaining* are the same condition by construction.
**No dedicated NN input** — `IN_ENVELOPE` gives tracking, and `CLOSING_RATE` (already an input) splits the
other two.

### 4.2 Load

Body-frame normal acceleration from recorded `acc[]` + `quat[]`. Reported per axis, per regime, as
distribution + peak. This is the quantity a load objective would constrain and the quantity
`ACCEL_Z` observes — objective and observation of the same physical thing.

### 4.3 Target-bearing estimate (the predictor target)

**Per tick**, not per gap: the head's belief about the target's *current* bearing, plus the truth wherever
truth exists (visible ticks and the reacquisition tick). Horizon-free — a state estimate, not a forecast
(clarified 2026-08-07, research.md R10).

Derived quantities the study needs:

| quantity | use |
|---|---|
| `blind_gap_age` (seconds since last truth) | the **primary conditioning variable** — error must be reported against it, because pooled error is dominated by easy visible ticks |
| hold-last-seen bearing | the no-information baseline (dead-reckon at zero rate) |
| truth bearing at reacquisition | closes each gap and is the highest-value scoring sample |

**Blind-gap distribution** — frequency, duration histogram, exit→re-entry bearing offset — remains a
required by-product (unmeasured to date, and the input to any future field-of-view decision).

### 4.4 Prediction error (innovation) — a fed-back INPUT, tracker-only

The head is **actuated**: when truth arrives, `error = truth − estimate` is written into the *next* tick's
input vector (FR-025c–f). Signed per axis, mirroring the estimate's dimensionality.

```text
tick t:    head emits estimate(t)
tick t:    truth available? ──yes──► error = truth − estimate   ──┐
                            └──no───► hold previous error   ─────┤
tick t+1:  input vector receives error  ◄─────────────────────────┘
```

| property | value |
|---|---|
| direction | **input**, not output — the estimate is *not* fed back (redundant with hidden state; the error is not) |
| sign | signed per axis — direction of bias is the information |
| during blindness | **holds last value** — zeroing would falsely assert "my model is correct" |
| staleness companion | the **existing** `TIME_SINCE_SEEN` input; no new slot needed |
| wiring | computed in the stepper; **no output-layer recurrence** required |
| mode | tracker-only. `TrackerInput` grows; `PathgenInput` unaffected |
| count impact | `TrackerInput` 63 → 63 + (estimate dimensionality) |

⚠️ **Not a learning signal.** This is a GA — the innovation is an *observation the policy conditions on*,
analogous to a TD error in shape but not in function. Evolution does all weight updating; nothing here
backpropagates.

---

## 5. Config additions

Added through the `AUTOC_CONFIG_FIELDS(X)` X-macro so declaration, parse and startup print stay in one
place. Expected keys:

| key | purpose |
|---|---|
| `EnableEnvelopeInputs` | ablation gate for the US4 inputs |
| `EnableAccelInputs` | ablation gate for the accelerometer inputs |
| `AccelScaleG` | `kAccelScale_g` normalizer |
| `EnvelopeSpanLo` / `EnvelopeSpanHi` / `EnvelopeCentroidRadius` | M2 estimator envelope (reserved; Phase C/D) |
| ~~`EnableLoadAxis`~~ | **not in 041** — no load objective (clarified 2026-08-07, research.md R4) |

⚠️ Two config-surface hazards, both already-paid lessons: `INI_MAX_LINE` is 200 chars and a knob plus its
explanatory comment has already exceeded it once (aborting startup with no line number, costing a bisect);
and `contract_tracker_config_tests.cc` pins `FitStreakThreshold == 0.5` against the *mutable production
ini*, so it will fail for reasons unrelated to correctness once 041 touches streak config. Both are
prerequisites in research.md R12.

Per Constitution VII, config-supplied members carry no in-class default initializers — the constructor
initializer list is the single assignment site.

---

## 6. Downstream artifacts affected

| artifact | effect |
|---|---|
| `nn_weights*.dat` (NN01) | new weight count; old genomes non-portable (intended). **Archive alongside every pinned dmp** (FR-010) — this is what the 038-baseline-unloadable lesson buys |
| `xiao/src/generated/nn_program_generated.cpp` | regenerate via `nn2cpp`; xiao build is part of the gate |
| `data.dat` | column set grows. Verified gone from repo root — preserve the final M1/M2 traces (FR-022) |
| `data.stc` | per-gen aggregate; unchanged in shape |
| S3 dmps | new version; prior runs unreadable by the new binary **by design**, which is why both comparators were pinned before the break |

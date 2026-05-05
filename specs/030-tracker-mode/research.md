# 030 tracker-mode — plan-phase research

Fresh rewrite 2026-05-04 against the post-housekeeping spec (28 FRs, 16 design notes). Resolves R1–R10 from `plan.md`'s M0 phase. Each section ends with a Decision / Rationale / Alternatives / Citations block that subsequent plan phases write tasks against.

> **Heritage**: a previous research pass dating from the 029-era pre-pivot is superseded by this rewrite. Where prior research (R1 .crrclog format, R2 dmp loader carve-out, R3 projection math) carries forward unchanged, citations note the carry-forward; otherwise the prior decisions are archived as "alternatives considered" within the relevant R-section here.

---

## R1 — Source dmp → projection-module integration path (v1)

**Decision (revised 2026-05-04)**: **Target trajectory lives in autoc memory only for v1**. crrcsim runs the chase craft alone; the autoc-side beacon projection module (FR-005, M5) reads the per-tick target pose directly from the in-memory `SourceScenarioTrajectory` and computes `(x, y, CEP)` from `(chase pose from crrcsim, target pose from memory)`. **No crrcsim-side multi-aircraft work for v1** — `RobotProgrammable` in `crrcsim/src/mod_robots/` is **deferred to post-v1**, when the renderer's video-enabled mode requires crrcsim to actually host the target as a second visible aircraft.

**Rationale**: The smoke test (D13) needs to compute virtual beacons on a screen of a certain warp — strictly the math, no visualization in crrcsim's 3D viewer during training. crrcsim's role at training time is unchanged: integrate the chase craft physics, send its pose to autoc-side, accept control commands back. The target craft's pose at the current sim time is held in autoc memory (loaded once at startup per FR-001) and is available to the projection module without crrcsim needing to know about it.

**v1 data flow** (training time):
```
autoc startup:
  loadSourceDmp()  → vector<SourceScenarioTrajectory> in autoc memory

per-tick during training (FR-018 timing model):
  source-tick t_i:
    target_pose_at_t_i = source_traj.sample(t_i)        // in-memory lookup
    chase_pose = crrcsim.getChasePose()                  // existing pathgen-mode flow
    obs = projectBeacon(chase_pose, target_pose_at_t_i, camera_config, beacon_config)
    nn_inputs[BEACON_*] = obs                           // typed sensor (R4)
    nn_outputs = NN.forward(nn_inputs)
    crrcsim.applyControls(nn_outputs)
    crrcsim.advancePhysicsUntil(t_{i+1})
```

The renderer (M9) shows the target aircraft from the M2 dmp's `targetTrajectoryList` (FR-015 self-containedness) using its own VTK actor — also no crrcsim mod_robots dependency.

**What gets deferred to post-v1** (when video-enabled mode opens):
- New subclass `RobotProgrammable : public RobotBase` in `crrcsim/src/mod_robots/` consuming an in-memory pose stream pushed from autoc.
- `Robots::AddRobot()` integration so crrcsim's 3D viewer renders both aircraft live during training.
- Use case for landing this: when an operator wants to *watch training in progress* in crrcsim's 3D viewer (rather than playback after-the-fact in the renderer).

This deferral is captured as a v2/post-v1 candidate; not yet promoted to BACKLOG.md because it's a 030 v2 concern, not a 031 candidate.

**Wingspan / airframe dimensions** (still needed for FR-005 self-occlusion check, even without mod_robots): v1 hardcodes hb1 wingspan in `beacon_config.h` defaults; v2+ may route this through `ScenarioMetadata` if 025 craft variations land in a tracker-relevant role. Per R8 finding: source dmps don't carry airframe metadata.

**Alternatives considered**:
- *Land `RobotProgrammable` in v1 (the prior R1 decision)*. Rejected — adds crrcsim mod_robots scope (~150 LOC + integration glue + Robots accessor changes) for a feature (live in-crrcsim two-aircraft display during training) that the smoke test doesn't need. Plan-research scoping pass 2026-05-04 explicitly defers M4-style work.
- *Write per-scenario `.crrclog` files at autoc startup, replay via existing `CRRC_AirplaneSim_Playback`* (the original pre-2026-05-04 R1). Rejected by the FR-001 revision (load directly, no intermediate files) AND by the M4-defer (no crrcsim multi-aircraft for v1).

**File:line citations**:
- `crrcsim/src/mod_robots/robot.h:38-101` (RobotBase interface — informational; not implemented in v1)
- `include/autoc/rpc/protocol.h:234-336` (source EvalResults schema — see R8)
- 030 spec FR-002 (v1 deferral note)
- [reference_crrcsim_mod_robots.md](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md) (post-v1 path)

---

## R2 — Arena enforcement primitive: extend `ENTRY_SAFE_*` or add `FLIGHT_ARENA_*`?

**Decision**: **Add parallel `FLIGHT_ARENA_RADIUS` / `FLIGHT_ARENA_FLOOR_AGL` / `FLIGHT_ARENA_CEILING_AGL` constants** alongside the existing entry-time ones. Do NOT silently overload `ENTRY_SAFE_RADIUS` etc. into the in-flight enforcement role.

**Rationale** (per spec D11): `ENTRY_SAFE_RADIUS = 55m`, `ENTRY_SAFE_ALT_MIN = 3m`, `ENTRY_SAFE_ALT_MAX = -80m` (`include/autoc/autoc.h:19-22`) are entry-only per their existing call sites (`src/autoc.cc:266-282`). They clamp where a scenario *spawns*, not where the craft can fly mid-scenario. Extending them into the fitness-evaluator path is silent semantic overload — anyone reading `ENTRY_SAFE_RADIUS` today reasonably assumes "spawn-time only," and grafting a fitness-time meaning onto the same identifier surprises future readers / auditors.

A separate `FLIGHT_ARENA_*` set:
- Documents the in-flight-bound semantics in the name.
- Allows the two values to *differ* (entry-time tighter than flight-time, or vice versa) without one having to compromise.
- Lets the fitness evaluator import a clean primitive rather than reaching into autoc.h's entry-spawn config.

**v1 default values** (committed):
- `FLIGHT_ARENA_RADIUS = 80.0` (m, horizontal cylinder radius — slightly larger than the entry-time 55m so trained controllers have working room without immediately tripping arena egress on entry).
- `FLIGHT_ARENA_FLOOR_AGL = 5.0` (m above ground, hard floor — slightly conservative vs the M1.3 flight-test field which has been operated as low as 3m).
- `FLIGHT_ARENA_CEILING_AGL = 100.0` (m above ground, optional ceiling — generous; matches `|ENTRY_SAFE_ALT_MAX| = 80m` plus headroom).
- All three live in `autoc-tracker.ini` for per-scenario override (e.g., a flight-field-specific tuning); the `#define`s in autoc.h serve as the documented defaults.

**Enforcement shape** (FR-016 lock-in 2026-05-04): per-tick check inside the tracker-mode fitness path. Egress = scenario-terminating event with a fitness penalty. Plan-phase decision still open: **graceful** termination (record the egress tick, terminate cleanly, scenario-complete is "egressed at tick N") vs. **hard** termination (immediate scenario abort, no further fitness contribution). Lean toward graceful — operator can post-hoc analyze egress geometry — but locking in the choice is plan-task work.

**Alternatives considered**:
- *Extend `ENTRY_SAFE_*` to fitness-time use (rename them too)*. Rejected per D11: the existing entry-time call site lives in pathgen-mode code; tracker mode is the right place to introduce in-flight semantics, and forcing pathgen to also adopt the in-flight enforcement (or carrying the rename across both modes) is scope creep on a P0 v1 change.
- *Single combined `ARENA_*` constants used for both entry clamp and in-flight bound*. Rejected — there are real reasons to want them to differ (e.g., entry-time clamp prevents spawn outside the field; in-flight bound permits the controller to *approach* but not cross the field boundary — different mathematical relationships).

**File:line citations**:
- `include/autoc/autoc.h:19-22` (existing `ENTRY_SAFE_*` constants)
- `src/autoc.cc:266-282` (existing entry-time use)
- 030 spec FR-016 (in-flight bound requirement)
- 030 spec D11 (rationale — don't silently overload)

---

## R3 — `p_crash` v1 default

**Decision**: **Curriculum-anneal**: `p_crash` ramps from **0.0 at gen 0 → 0.15 at gen 100 → 0.30 plateau from gen 200 onward**. Configurable in `autoc-tracker.ini` as `pCrashGen0` / `pCrashGenRamp` / `pCrashGenPlateau`. Per-tick (10 Hz) probabilistic firing.

**Rationale**: Three considerations:

1. **Early-gen exploration**: untrained / barely-trained genomes have to be free to fly *into* the target's nominal envelope to discover that the crash hull is a thing. With deterministic crash from gen 0, the fitness landscape has a sharp cliff at the hull boundary that early-gen exploration can't navigate around — controllers get clobbered by crash penalty before they've evolved any structure. `p_crash = 0` for the first ~100 gens lets exploration happen.

2. **Late-gen sharpening**: by gen 200+ the controller should have evolved enough trail-following structure that the hull becomes a useful negative reinforcement. Plateau at 0.30 (i.e., 30% chance per 100ms tick of crash given hull intersection) is non-fatal-on-one-tick but fatal-over-a-pass — a controller that *grazes* the hull for one tick gets a small penalty; one that *holds* inside the hull gets crashed within ~3 ticks (E[ticks until crash] ≈ 1/p ≈ 3.3 ticks ≈ 333ms).

3. **Aggregate-fitness math**: a gen-200+ controller doing 30s engagements with ~300 ticks should expect ~0 hull-intersection ticks if it's tracking well; controllers that aren't get terminated quickly enough that the wasted compute per gen is bounded.

The 0.30 plateau is a starting estimate; if smoke-test signal shows the hull never fires (controller too cautious) or fires too often (fitness-landscape still cliff-y), tune in v1 follow-up.

**Per-tick determinism**: the `p_crash` random draw uses the same per-scenario PRNG that wind / variation use, with a dedicated subsequence so it doesn't perturb the existing joint-PRNG sample. Scenario re-runs are bit-identical.

**Alternatives considered**:
- *Deterministic (`p_crash = 1.0` from gen 0)*. Rejected per the early-gen exploration argument above.
- *Zero (`p_crash = 0.0`, hull defined but inert)*. Rejected — the hull's load-bearing job is to keep training away from collision-style policies; an inert hull lets evolution favor chase-aggressive policies that look great in fitness numbers but would crash on a real flight test.
- *Linear ramp the whole way (`p_crash` linearly from 0 to 1.0 at end of run)*. Rejected — the plateau-after-200 captures the "useful negative reinforcement" regime more clearly than a forever-ramping signal that might sit too low at the moment a converged policy needs the constraint.
- *Single-config `p_crash` with no ramp*. Rejected — early-gen and late-gen needs are different; a curriculum lets one parameter set serve both.

**File:line citations**:
- 030 spec FR-008b (probabilistic firing committed for v1)
- 030 spec D9 (curriculum-anneal rationale)

---

## R4 — Type-safe NN sensor interface concrete shape

**Decision**: **Strongly-typed `enum class NNInput : uint16_t`** with a single source-of-truth header (`include/autoc/nn/nn_inputs.h`) defining the enum, a parallel `constexpr` array of metadata (`name`, `dtype`, `range_min`, `range_max`, `units`), and an `inputs[NNInput::COUNT]` accessor pattern. **Mirrored to xiao firmware** via the same header (compile-time-shared between desktop and embedded — autoc.h pattern).

**Rationale**:
- **Enum-keyed indexing** gets compile-time errors for typos; the magic-number `float[]` pattern that hit silent-corruption in the 027 topology change is structurally prevented.
- **Single-header source-of-truth** (`nn_inputs.h`) eliminates the scattered `NN_INPUT_COUNT` constants that live today across `topology.h / autoc.h / evaluator.cc / data.dat header / sim_response.py / aircraft_state.h` (per the BACKLOG entry's files-to-touch list).
- **Compile-time count derivation** (`NNInput::COUNT` is the last enum entry +1, with `static_assert` matching the topology weight count) ensures any addition / removal of an input forces all downstream code to update consistently.
- **Metadata array** (parallel `constexpr` in the same header) carries dtype + range so analysis scripts can read the names canonically: `BEACON_L_X[t=-0.5s]`, `BEACON_L_CEP[t=now]`, `GYRO_P`, etc.

**Schema sketch**:
```cpp
// include/autoc/nn/nn_inputs.h (illustrative)
enum class NNInput : uint16_t {
  // Beacon left observations
  BEACON_L_X_TM5,    // x, t = -0.5s
  BEACON_L_X_TM4,    // x, t = -0.4s
  // ...
  BEACON_L_X_NOW,    // x, t = now
  BEACON_L_Y_TM5,
  // ... (matching y, CEP slots)
  BEACON_L_CEP_NOW,
  // Beacon right observations (mirror)
  BEACON_R_X_TM5,
  // ... 
  BEACON_R_CEP_NOW,
  // Aircraft state
  QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
  AIRSPEED,
  GYRO_P, GYRO_Q, GYRO_R,
  COUNT  // sentinel = total input count
};

struct NNInputMeta {
  const char* name;
  // dtype is always fp32 at this interface (FR-006); dynamic range varies
  float range_min;
  float range_max;
  const char* units;
};

inline constexpr NNInputMeta kNNInputMeta[static_cast<size_t>(NNInput::COUNT)] = {
  { "BEACON_L_X_TM5", -1.0f, +1.0f, "screen_norm" },
  // ...
};

static_assert(static_cast<size_t>(NNInput::COUNT) == TRACKER_INPUT_COUNT,
              "NN input count mismatch — check enum vs topology");
```

**Why enum + array (not struct-of-fields)**: the NN evaluator computes a `float[]` weight × input vector inner product; needing `inputs[i]` random access is mandatory. A struct-of-fields would force per-field code paths or a manual index mapping, defeating the compile-time-safety property. Enum + array gets compile-time naming for free at the cost of a `static_cast<size_t>` at the index site — small price.

**xiao mirror**: the same header lands on xiao via the existing autoc.h pattern. xiao computes its perception inputs directly into the `inputs[]` array using `inputs[static_cast<size_t>(NNInput::BEACON_L_X_NOW)] = ...`. No magic numbers on either side.

**Alternatives considered**:
- *Struct-of-arrays (named per-input fields, with a separate flat-buffer for NN weight × input math)*. Rejected — the weight × input math forces a flat buffer anyway; a parallel struct doubles the storage and adds a sync-between-them step.
- *String-keyed map (`std::map<std::string, float>`)*. Rejected — runtime cost (hash / string compare per input access) is unacceptable in the 10 Hz × population × scenarios training loop.
- *X-macro list expansion*. Considered. Slightly more error-prone than enum + parallel constexpr metadata array; the enum + constexpr pattern is C++17-idiomatic and reads cleanly. X-macros become attractive if the input list grows past ~50 entries; for 030 v1 (44 entries) the enum+constexpr pattern is fine.

**File:line citations**:
- `include/autoc/nn/topology.h` (current `NN_INPUT_COUNT` source)
- `include/autoc/autoc.h` (current duplicate `DISTANCE_TARGET` etc. defines)
- `src/nn/evaluator.cc:nn_gather_inputs` (current magic-number population)
- BACKLOG.md "[NEXT] Type-Safe NN Sensor Interface" (full files-to-touch list)
- 030 spec FR-006 (concrete naming requirements)

---

## R5 — FR-018 main-loop refactor: pathgen ↔ tracker pipeline split

**Decision**: **Strategy-pattern split inside the worker**. Introduce a `ScenarioStepper` interface with two implementations: `PathgenStepper` (current pathgen-mode per-tick logic) and `TrackerStepper` (FR-018's source-timestamp-driven logic). The worker / population / fitness-aggregation / RPC pipelines stay shared and unchanged; the divergence is *only* in the per-tick stepping semantics.

**Rationale**:
- Pathgen-mode runs a 10 Hz NN tick on an independent virtual clock; tracker-mode keys off M1 source timestamps (FR-018). The *only* code path that knows about the timing model is the per-tick stepping logic; everything above (worker accepting scenarios, distributing to crrcsim, collecting per-scenario fitness, aggregating per-gen statistics) is identical.
- Strategy pattern keeps the divergence at the right layer. Worker plumbing (~80% of the existing main loop) doesn't fork.
- Constitution III (no compatibility shims) is honored: `PathgenStepper` is *the* path for pathgen mode (no shim, no toggle), `TrackerStepper` is *the* path for tracker mode. The `ScenarioStepper` interface is the boundary.

**Sketch**:
```cpp
// include/autoc/eval/scenario_stepper.h (illustrative)
class ScenarioStepper {
public:
  virtual ~ScenarioStepper() = default;
  virtual void initScenario(const ScenarioMetadata& meta) = 0;
  virtual bool stepUntilNextNNTick() = 0;  // returns false at scenario end
  virtual AircraftState currentChaseState() const = 0;
  virtual NNInputs currentNNInputs() const = 0;
  // tracker-only:
  virtual std::optional<AircraftState> currentTargetState() const { return std::nullopt; }
};

class PathgenStepper : public ScenarioStepper { /* current logic */ };
class TrackerStepper : public ScenarioStepper {
  // owns the SourceScenarioTrajectory, RobotProgrammable instance, BeaconProjector,
  // TrailRabbit, CrashHull. Per-tick: advance source-timestamp index; project beacons;
  // compute trail rabbit; check crash hull; check arena; drive crrcsim physics until
  // next source-timestamp.
};
```

**Implementation order**: lands in M6 of the plan. Refactor pathgen's existing per-tick logic *into* `PathgenStepper` first (no behavior change; existing tests stay green), then add `TrackerStepper` as a sibling. The first step is the regression-tight refactor that gets pathgen-mode running through the new interface; the second adds tracker-mode without touching pathgen-mode.

**Alternatives considered**:
- *Mode flag (`if (mode == TRACKER) { ... } else { ... }`) sprinkled through the per-tick code*. Rejected — accumulates branching everywhere, defeats the "pathgen unchanged" property by making every tick of pathgen execution observe a useless conditional.
- *Two entirely separate worker binaries*. Rejected — duplicates ~80% of the existing main loop, every cross-cutting infrastructure change (e.g., dmp versioning) lands in two places.
- *Template-parameterize the worker on a stepper type*. Considered. Equivalent to the strategy pattern at run time; slightly more rigid (no run-time mode selection from config). Strategy pattern + virtual dispatch is fine for the per-tick rate; the cost vs the maintenance benefit is negligible at 10 Hz.

**File:line citations**:
- `src/autoc.cc` main loop (current pathgen-mode per-tick logic to be refactored into `PathgenStepper`)
- 030 spec FR-018 (source-timestamp-driven semantics)
- 030 spec D14 (timing-model rationale)

---

## R6 — CEP encoding semantics + invisibility sentinel

**Decision**: **Linear CEP scaling** with `CEP ∈ [0.0, 1.0]` for visible beacons, `CEP = 1.5` (encoded as `INT8_MIN = -128` after int8 quantization) as the invisibility sentinel. v1 uses linear; if smoke-test signal indicates the linear gradient is poorly suited to NN learning, sweep log-spread / piecewise alternatives in v1 follow-up.

**Encoding semantics**:
- `CEP = 0.0`: ideal centroid, sharply localized in the middle of the camera frame.
- `CEP = 0.3` (suggested edge-of-frame baseline): centroid near image edge, where aberrations + sample-quantization noise grow.
- `CEP = 0.7` (suggested motion-blur / aberration-zone baseline): clearly degraded centroid — the (x, y) values are usable but heavily uncertain.
- `CEP = 1.0`: maximum-noise visible centroid (the floor below the sentinel).
- `CEP = 1.5` (encoded `INT8_MIN`): not visible / sentinel.

**Int8 mapping**:
- For the visible range `[0.0, 1.0]`: linear → `[0, +127]` (positive half of int8). `quantize(c) = round(c * 127)`.
- Sentinel: reserved `INT8_MIN = -128`. The receiving code checks `if (raw_int8 == INT8_MIN) → CEP = sentinel; (x, y) ignored; else CEP = raw_int8 / 127.0f`.
- Negative int8 values `[-127, -1]` are unused (reserved for future signed-CEP encoding if needed).

**Rationale for linear baseline**:
- Linear is the "no curve assumption" choice. The NN is a black box; it can learn a non-linear mapping internally if linear-input is wrong.
- Matches the eventual hardware extractor's natural output (cluster spread is a moment computation, linear in pixel count above threshold).
- Keeps the int8 mapping trivial and inspectable.

**Sentinel choice**: `INT8_MIN = -128` over NaN-marker because:
- Trivially distinguishable from any in-range value (no float-special-value plumbing required).
- Fits naturally in the int8 stream (the int8 range is `[-128, +127]`; we use `[+0, +127]` for valid CEP and `-128` as "no centroid"). Sentinel is *separated* from the valid range without requiring float infrastructure.
- Easy to gate on in the perception extractor without conditional branches: `if (raw == -128) sentinel; else valid`.

**Why NOT NaN-marker**: NaN propagation through downstream NN math is a foot-gun (NaN × 0 = NaN; the NN's hidden layer sums would propagate NaN to outputs). Forcing `if (input == sentinel) input = 0` at the dequantization step is cheap and avoids NaN entirely.

**Alternatives considered**:
- *Log-spread CEP encoding* (more resolution near 0 = "good localization", less near sentinel). Rejected for v1 — adds non-monotonicity in the gradient that the NN might or might not learn. Revisit if smoke-test shows poor discount-on-noisy behavior.
- *Piecewise CEP* (dense in [0, 0.3], coarser [0.3, 1.0]). Rejected for v1 — same reason; nail the linear baseline first, derive nonlinear shape from data.
- *NaN-marker for sentinel*. Rejected per the foot-gun argument.

**File:line citations**:
- 030 spec FR-005 (CEP semantics)
- 030 spec FR-007 (sentinel handling)
- 030 spec FR-017 (int8 quantization)
- 030 spec D2 (CEP rationale)

---

## R7 — int8 quantization round-trip math

**Decision**: locked-in math from R6 sentinel choice:

```
// Quantize (sim-side perception):
int8_t quantize_xy(float v_in_minus_one_plus_one) {
  // Clamp to [-1, +1] first to avoid overflow on slightly-out-of-range inputs.
  float clamped = std::clamp(v_in_minus_one_plus_one, -1.0f, +1.0f);
  return static_cast<int8_t>(std::round(clamped * 127.0f));
}

int8_t quantize_cep(float cep_in_zero_one_or_sentinel) {
  if (cep_in_zero_one_or_sentinel >= kCepSentinelThreshold) return INT8_MIN;
  float clamped = std::clamp(cep_in_zero_one_or_sentinel, 0.0f, 1.0f);
  return static_cast<int8_t>(std::round(clamped * 127.0f));
}

// Dequantize (NN-input boundary):
float dequantize_xy(int8_t q) { return static_cast<float>(q) / 127.0f; }
float dequantize_cep(int8_t q) {
  return (q == INT8_MIN) ? kCepSentinelFloat : static_cast<float>(q) / 127.0f;
}

constexpr float kCepSentinelThreshold = 1.25f;  // anything ≥ this is "invisible"
constexpr float kCepSentinelFloat = 1.5f;       // dequantized sentinel marker
```

**Round-trip property** (contract test): for any visible `(x, y, cep)`:
`dequantize(quantize(x)) ≈ x` within `1/127.0 ≈ 0.0079` (one int8 step). For invisibility: `dequantize_cep(quantize_cep(any_value ≥ 1.25)) == kCepSentinelFloat` exactly.

**The NN sees** `dequantize_*` outputs — fp32 at the input boundary, but with int8-quantized resolution. The sentinel arrives as exactly `1.5f`, distinguishable from any in-range CEP value (which sits in `[0, 1.0]`).

**Plan-task implication**: the `beacon_projection_tests.cc` contract test (M5) asserts the round-trip property and the sentinel mapping. The `nn_evaluator_tests.cc` regression suite checks dequantize behavior at the input-gathering boundary.

**Alternatives considered**:
- *Asymmetric int8 mapping* (more resolution near 0, less near edges of `[-1, +1]`). Rejected for v1 — symmetric linear is simplest; if smoke-test signal shows the NN learns better with non-linear x/y resolution, revisit.
- *Use full int8 range `[-128, +127]` for x/y `[-1, +1]` mapping*. Considered; rejected because reserving `INT8_MIN` as sentinel for CEP only is fine (xy doesn't need a sentinel — invisible beacons emit `(x=0, y=0, cep=INT8_MIN)` per FR-007; the xy zero is conventional, not load-bearing).

**File:line citations**:
- R6 above (encoding decisions)
- 030 spec FR-017 (int8 quantization requirement)

---

## R8 — Source dmp schema gap analysis

**Decision**: **`EvalResults` schema as it stands today already provides everything tracker-mode needs as source data**. No source-side schema change required. (The tracker-mode dmp *output* is a different story — that's M8 / FR-015a's schema bump.)

**What the current `EvalResults` carries** (per `include/autoc/rpc/protocol.h:234-336`):

| Field | Type | Tracker-mode use |
|---|---|---|
| `aircraftStateList[scenario][step]` | `vector<vector<AircraftState>>` | Per-tick target-craft pose stream (position via `getPosition()`, attitude via `getOrientation()` quat, velocity via getter, simTimeMsec) — direct input to `RobotProgrammable` (R1) |
| `scenarioList[scenario]` | `vector<ScenarioMetadata>` | Per-scenario joint-PRNG variation params (`pathVariantIndex`, `windVariantIndex`, `windSeed`, `scenarioSequence`) — for FR-010 deterministic re-derivation of wind / craft / entry per source scenario |
| `pathList[scenario]` | `vector<Path>` | Source path geometry — *not used* by tracker mode (tracker uses target-craft pose, not the underlying path) but harmless to ignore |
| `gp` (NN weights) | bytes | Source controller weights — *not used* by tracker mode (D13 / Architectural clarification — source's fitness/weights are irrelevant to 030) |
| `crashReasonList` | vector | Source-scenario crash flag — relevant for "skip crashed source scenarios" in tracker config (potential v1 follow-on; v1 may either use them or include crashed scenarios with truncated trajectories — plan-task call) |

**What the source dmp *does not* carry** (and tracker-mode needs to source elsewhere):

- **Target airframe wingspan / dimensions**: `AircraftState` doesn't store airframe metadata; it stores a per-tick state snapshot. For `RobotProgrammable::getWingspan()` we need a separate source — v1 hardcodes hb1 wingspan (matching M1.3 flight test airframe); v2+ may route this through `ScenarioMetadata` if 025 craft variations land in a tracker-relevant role.
- **Target beacon geometry**: not in source dmp (beacons are the chase-side perception input, computed against the target's pose). v1 lives in `beacon_config.h`.
- **Source-craft camera config (if recorded with a camera)**: not relevant — source dmp is from pathgen-mode runs that have no camera; tracker-mode's camera config is on the chase craft, configured fresh.

**Schema bump deferred to OUTPUT**: FR-015a bumps the cereal class version on tracker-mode dmp WRITE (M8); source dmp READ is unchanged for v1.

**Alternatives considered**:
- *Add a "TargetCraftMetadata" field to source dmp*. Rejected — would require re-running prior training to embed it, defeating the "use existing dmps" property.
- *Read airframe dimensions from a side-channel file (e.g., `hb1_streamer.xml`)*. Considered — tracker-mode startup could read the same XML model file crrcsim does. v1 hardcoding is simpler; revisit if a multi-airframe library lands.

**File:line citations**:
- `include/autoc/rpc/protocol.h:80-114` (ScenarioMetadata)
- `include/autoc/rpc/protocol.h:234-336` (EvalResults)
- `include/autoc/eval/aircraft_state.h:295-490` (AircraftState getters + cereal serialization)
- 030 spec FR-001 (load source dmp directly)
- 030 spec FR-010 (joint-PRNG variation re-derivation)

---

## R9 — Renderer: confirm independence from FR-005 analytic projection

**Decision**: **Renderer's `vtkCamera` viewport math is structurally separate from the FR-005 analytic projection module and stays that way**. FR-012 renderer tracker-mode views consume the *output* of FR-005 (per-tick beacon `(x, y, CEP)` from the M2 dmp), they do not duplicate or share projection code with the training pipeline.

**Boundaries**:
- FR-005's `camera_projection.{h,cc}` (Eigen, headerless, unit-tested per M5) computes per-tick beacon screen coordinates that go into the M2 dmp + the NN inputs.
- Renderer's `vtkCamera` (existing in `tools/renderer.cc`) drives 3D viewport rendering for the operator's display — VTK / OpenGL pipeline, GPU-bound. Not callable from the autoc training binary (no OpenGL context available).
- Renderer's camera-POV mini-panel (FR-012, M9) reads the *recorded* `(x, y, CEP)` triples from the M2 dmp and renders them as 2D dots on a virtual screen. It does NOT recompute projection — that's the load-bearing self-containedness property of FR-015 (the M2 dmp carries the full perception state).
- Renderer's 1st-person camera-POV mode (FR-012, M9) likewise reads recorded camera pose + projection state from the M2 dmp; if it needs to render scene geometry through that camera (vs reading recorded beacon positions only), it uses VTK's `vtkCamera` with parameters set to match the recorded projection state — *but* the FR-005 projection math itself is not shared.

**Why this matters**: keeps the FR-005 module headless (testable on a CI runner without an OpenGL context), keeps the renderer free to use VTK abstractions without dragging the training binary into a VTK dependency, and makes the M2 dmp the single source of truth for "what did the controller see" at playback time.

**Pathgen-mode dmp back-compat** (FR-015a): the renderer must continue to load pre-versioning pathgen-mode dmps. Implementation: read the version field; if version == 0 (pre-versioning) or version matches pathgen-mode schema, dispatch to the existing pathgen renderer path; if version matches tracker-mode schema, dispatch to the new tracker renderer path. Loud-fail on unknown versions per Constitution V.

**Alternatives considered**:
- *Reuse the renderer's `vtkCamera` projection math in autoc training*. Rejected — autoc training binary doesn't link VTK and shouldn't (would force OpenGL context dependency on a training-pipeline binary that runs headless, including in CI / on remote workers).
- *Re-run FR-005 projection at render time from raw target trajectory*. Rejected — would require the renderer to import autoc-side projection code (couples renderer to evaluator), and would risk drift between training-time and render-time projections (whichever path makes a config tweak first wins). M2 dmp self-containedness is the better contract.

**File:line citations**:
- `tools/renderer.cc:920-928, 3245-3260` (vtkCamera — confirmed unrelated)
- 030 spec FR-012 (renderer two views + mini-panel)
- 030 spec FR-015 (M2 dmp self-containedness)
- 030 spec D5 (mini-panel HUD overlay logic)

---

## R10 — Perception representation: raw `(x, y, CEP)` vs polar / derived geometric features

**The load-bearing v1 research question.** Captured here as a research artifact; **the v1 smoke test (M10) IS the experiment that answers it.** No M0-resolvable decision possible.

**The question**: with two wingtip beacons projected to screen `(x, y)` per FR-005 — across a 6-slot history per beacon — can the NN extract **range to target** from the joint distribution of these eight time-series signals?

**Why this is hard** (vs the analogous-feeling problem of "estimate distance from a known-baseline feature pair on a camera"):

A car ADAS system uses, e.g., the rear-window pixel separation of a leading vehicle to estimate range. That problem's geometry is benign: the leading vehicle presents an approximately *fixed-aspect baseline* (the rear pixel pair is approximately rigid in the camera frame; lateral / vertical motion changes mostly screen position, not separation magnitude).

A banking aircraft *does not present a fixed-aspect baseline*. Inter-wingtip separation in screen space is foreshortened by:
- **Range** (the signal we want).
- **Target yaw** relative to camera direction (full-broadside maximizes apparent separation; nose-on / tail-on collapses it to a point).
- **Target pitch** relative to camera (nose-up / nose-down asymmetrically foreshortens).
- **Target roll** relative to camera (rotates the inter-beacon line on screen, doesn't foreshorten — but couples L/R discrimination with screen geometry).
- Camera mount perturbations (FR-003a aberrations, mount alignment; partly controlled by FR-006 typed interface, partly by per-scenario PRNG variations once those land).

Range estimation from the raw two-beacon signal is an *inverse* problem: separation is a non-injective function of `(range, yaw, pitch, roll)`. The NN's job under the v1 spec is to learn this inverse from temporal context (6 history slots of beacon trajectories) — the dynamics implicitly disambiguate static ambiguities (a target rolling makes the inter-beacon line rotate; a target yawing changes separation; a target translating changes screen position).

**Three plausible representations** the NN could be fed:

1. **Raw Cartesian `(x, y, CEP)` per beacon** (current FR-005 / FR-006 spec):
   - 6 slots × 2 beacons × 3 channels = 36 inputs.
   - NN extracts range / attitude implicitly from temporal context.
   - Pros: minimal preprocessing; mirrors the eventual FPGA centroid output exactly; preserves all information.
   - Cons: harder learning problem — the NN has to learn the inverse projection from raw points.

2. **Polar `(θ, r, CEP)` per beacon, around screen center**:
   - Same dimensionality (36 inputs).
   - `r = sqrt(x² + y²)` is distance from optical axis; `θ = atan2(y, x)` is angle around optical axis.
   - Pros: matches the radial structure of cone-surface fitness (which is itself a radial-distance-from-target metric); some natural symmetries align (rolling target → θ rotates, range → r and inter-beacon-r-difference change).
   - Cons: foreshortening still entangles range with attitude; doesn't fundamentally simplify the inverse problem.

3. **Derived geometric features** alongside or replacing raw:
   - Inter-beacon midpoint `(x_m, y_m)` — robust target direction signal.
   - Inter-beacon separation distance `d` — first-order range proxy (modulated by attitude).
   - Inter-beacon angle on screen — first-order target-roll proxy.
   - Per-beacon CEP averaged or per-beacon kept.
   - Pros: hands the NN physically grounded features; matches the "pose-derived geometric features" intent in spec line 19 (perception-boundary clarification).
   - Cons: bigger input vector if appended (44 raw + ~24 derived = 68); loses information if substituted (per-beacon visibility / CEP averaged out).

**v1 stance** (decision deferred to smoke-test signal): **ship the current spec's raw `(x, y, CEP)` representation** for the smoke test. The smoke test answers signal-or-not on the raw representation directly. Outcomes:
- **Smoke green** (loop closes, fitness descends, controller exhibits some tracking behavior): raw representation has enough signal; defer the polar / derived-features experiment to v1+ as a fitness-improvement-style follow-on.
- **Smoke red specifically because of representation** (loop closes mechanically but fitness flat-lines on a profile that suggests "controller can't extract range"): iterate to representation 2 or 3. Diagnostic: per-axis aggressiveness on outputs vs. mean target-screen-distance over time — if controller commands are saturating but mean target-distance isn't shrinking, range extraction is failing.
- **Smoke red mechanically** (loop doesn't close, NaN, build issues, etc.): unrelated to representation; fix the mechanical issue first.

**Why we can't decide R10 at M0**: there's no closed-form analysis that says "the NN topology in 028/029 can reliably extract range from 6 history slots of two-beacon (x,y) signals." The smoke test is the cheapest experiment to answer this. M10 IS R10's experiment.

### R10 appendix — what the NN actually has to learn (the inverse problem)

Color filtering (FR-004) gives the NN a clean L vs R *nameplate* for each beacon — but it does not resolve the geometric inverse problem of "given two color-labeled screen positions, what attitude is the target in?" The single-frame ambiguity is **four-way**, not just front/back:

| Aspect | Target attitude | L (850nm) appears on | R (940nm) appears on |
|---|---|---|---|
| Head-on (target heading toward chase) | upright | chase **right** | chase **left** |
| Head-on | inverted (180° rolled) | chase **left** | chase **right** |
| Tail-on (target receding from chase) | upright | chase **left** | chase **right** |
| Tail-on | inverted | chase **right** | chase **left** |

Pairs `(head-on, upright) ↔ (tail-on, inverted)` and `(head-on, inverted) ↔ (tail-on, upright)` produce identical beacon screen configurations. Color labels remain correct — they just don't disambiguate which of the four geometric realities is producing them.

(Quick derivation: target body→world quat `Q`; chase body→world quat `Q_C`; chase-frame beacon position is `Q_C⁻¹ · Q · b_L` for `b_L = (0, -w/2, 0)`. Head-on upright: `Q = Q_C · R_yaw_180`, so `Q_C⁻¹·Q·b_L = R_yaw_180·(0,-w/2,0) = (0,+w/2,0)` → chase right. Add target-frame roll-180: `R_roll_180·b_L = (0,+w/2,0)`, then `R_yaw_180·(0,+w/2,0) = (0,-w/2,0)` → chase left. Tail-on aligns target +x with chase +x, so `Q_C⁻¹·Q = identity` then add roll. The four cases collapse to two screen configurations.)

**Disambiguation cues — the NN must integrate all three**:

1. **Temporal context — separation derivative**. Approaching → `Δ|d|/Δt > 0`; receding → `Δ|d|/Δt < 0`. Distinguishes head-on vs tail-on cleanly *if closure rate isn't ~0*. Constant-range orbital scenarios degenerate this signal and the NN must fall back to cue 2.
2. **Chase-side proprioception (QUAT_*, AIRSPEED, GYRO_*)**. The NN already has these inputs per FR-006. They're load-bearing in a way the spec didn't fully spell out: chase body-frame motion + attitude is a major factor in resolving the four-way ambiguity. If chase is upright + airspeed positive + beacons growing → target in front (head-on). If chase upright + airspeed positive + beacons shrinking → target overtaking from behind. This argues against any FR-006 simplification that drops chase-side state inputs in v2+.
3. **Physics consistency across history**. Target attitude can't teleport; if beacon-line angle on screen rotates smoothly, target is in a continuously evolving attitude. Aliasing breaks this — see Trouble 8 below.

This is why representation choice (raw vs polar vs derived) doesn't eliminate the ambiguity: all three representations carry the same information; the inverse is fundamentally under-determined per single frame regardless. The capacity question (R11) is what bites, not the encoding.

### Trouble list (consolidating and expanding)

For each, the v1 disposition: ship-as-spec'd, instrument heavily, defer remediation to a later iteration unless smoke-test signal forces it.

**Trouble 1 — quantization clobbers range at long distance.** Hard limit ~60-65m for hb1 at 120° FOV / int8. **Mitigated 2026-05-04 reasoning**: at 15 m/s closure-class scenarios, beyond ~50m the controller is in *intercept mode* — task simplifies to "head toward midpoint screen position with throttle pinned." Range becomes load-bearing only inside trail-rabbit envelope (~20-40m, near the 10 ft trail offset). v1 disposition: **OK as-is**. Real-flight escalation: telephoto secondary camera (FR-003b) extends the range envelope when needed.

**Trouble 2 — capacity-limited topology.** 1923 weights / 16-wide hidden may not have headroom to learn (a) inter-beacon foreshortening inverse, (b) chase-attitude compensation, (c) CEP gating, (d) temporal disambiguation simultaneously. v1 disposition: ship-as-spec'd; if fitness plateaus, see R11 (Path A — bigger network, supported by compute budget).

**Trouble 3 — CEP-sentinel discontinuities.** RNN hidden states aren't structurally aware of "this slot is missing." NN must learn to gate on CEP sentinel — learnable but capacity-hungry. v1 disposition: ship-as-spec'd; expected outcome is the NN learns crude gating in early gens (output collapses on sentinel, smooth dead-reckoning emerges later if at all). Diagnostic: per-tick output variance vs CEP-sentinel-rate correlation.

**Trouble 4 — chase-roll extrapolation.** Pastonly3 sources exhibit high-roll spiral patrol; NN must internalize "rotate screen-x/y by chase quat" as a learned approximation; extrapolates badly outside training distribution. v1 disposition: ship-as-spec'd; expected outcome is graceful degradation (controller works in mild attitudes, degrades sharply at extremes). Diagnostic: chase-quat-extreme-event flag in M2 dmp; correlate with per-tick output saturation.

**Trouble 5 — non-injective single-frame ambiguity for constant-range orbits.** Recovered substantially — chase-side proprioception (QUAT_*, AIRSPEED, GYRO_*) plus history-of-history derivatives explain the no-apparent-motion case. v1 disposition: **OK as-is**. Diagnostic: variance of beacon screen position vs chase angular rate — if chase is turning and beacons aren't moving on screen, NN should infer "we're in coordinated orbit" via the proprioception inputs.

**Trouble 6 — L/R beacon swap.** v1 sim simulates perfect optical filter; real-flight perception-front-end may have wavelength bleed. v1 disposition: deferred (031 candidate territory).

**Trouble 7 — variable-rate source.** Forward-looking; v1 sources are uniform-rate pathgen-derived. Already a 031 candidate.

**Trouble 8 — roll-rate aliasing**. (Newly identified.) Target rolls > ~1800°/sec → beacon-line angle on screen advances > 180° per 100 ms tick → NN can't tell rotation direction. Pastonly3 controllers exhibit "tight-spiral patrol"; **need to measure peak roll rate in pastonly3 source dmps**. If anywhere near 1800°/sec at 10 Hz tick, we're in or near the aliasing regime. **Strongest single argument for the 20 Hz / 50 ms tick option** in R11 — doubles aliasing limit to ~3600°/sec, comfortably above any realistic airframe roll. Diagnostic: histogram of frame-to-frame inter-beacon-angle change in source dmp + M2 dmp; flag scenarios where >90°/tick fires.

**Trouble 9 — chase-and-target both inverted degenerate case.** If chase is rolled 180° AND target is rolled 180°, the chase-relative geometry is identical to "both upright"; NN must disambiguate from QUAT_* alone. Smoke-test single-path scenarios likely never exercise this corner; surfaces only when 030+ controllers themselves end up flying inverted in tracker-mode evaluation. v1 disposition: **OK as-is**, accept as known failure mode in extreme attitude regime. Diagnostic: scenario-level chase-quat-extreme-event flag.

**Trouble 10 — initial-condition ambiguity at engagement start.** First tick has no temporal context; NN must default to *some* consistent assumption. The cost of getting this wrong is a few ticks of mistaken control before history fills. v1 disposition: ship-as-spec'd; expected outcome is the NN evolves a default "head toward midpoint, throttle pinned" behavior that's roughly correct in expectation. Diagnostic: compare first-1-second control output trajectory across scenarios with same target trajectory but different initial chase poses — should converge after ~3-5 ticks.

**Citations**:
- 030 spec FR-005 (current raw `(x, y, CEP)` interface)
- 030 spec FR-006 (typed sensor names — chase-side proprioception is load-bearing here)
- 030 spec line 19 (pose-derived geometric features intent — currently unrealized in FRs)
- 030 spec line 27 (front/back-aliasing acknowledged — now extended to four-way)
- 030 spec D2 (CEP rationale)
- R11 below (NN capacity headroom for the trouble list)
- M10 smoke test (the experimental answer)

---

## R11 — NN compute budget + topology growth options + RL / attention / co-evolution forward-looking

**Companion to R10.** Where R10 captures the *representational* question, R11 captures the *capacity / architectural* question. Like R10, the v1 stance is "ship-as-spec'd, instrument heavily, defer to smoke-test signal." Unlike R10, this section also catalogs more-speculative architectural responses (RL, attention, co-evolution) that the operator wants on file as forward-looking research candidates — explicitly *not* v1 commitments, but recorded so plan-research has a vocabulary for the responses if smoke-test outcomes call for them.

### Compute budget — back of the envelope

Current xiao deploy (28/29 baseline):
```
64 MHz × ~3 ms eval = ~192,000 cycles per NN tick
192,000 / 1923 weights ≈ 100 cycles / weight
```

Likely cycle-cost breakdown (educated estimate, no profiling — measure to confirm):

| Component | Approx cycles | % of budget |
|---|---|---|
| Per-weight MAC + load (1923 weights × ~5 cycles) | ~9,600 | ~5 % |
| Per-neuron tanh (~30-60 calls × ~150-400 cycles each) | ~5,000 – 25,000 | ~3-13 % |
| W_hh recurrent block (256 weights) | ~1,300 | <1 % |
| Memory thrashing (cold flash lines, unaligned loads) | unknown | ? |
| Loop / indexing / function call overhead | ~20-30 % of remaining | substantial |
| **Accounted** | ~50-80% | |
| **Unaccounted** | ~20-50% | profiling target |

Cleanup leverage (without changing topology): tanh table → RAM, profile-driven optimization, packed/quantized weight storage, vectorized MAC if hardware supports. **Plausibly 2–4× speedup to ~1 ms eval.**

At **20 Hz / 50 ms tick** budget:
```
50 ms × 64 MHz = 3,200,000 cycles per eval
vs current ~192,000 = ~16× headroom
plus 2-4× cleanup = 30-50× effective compute available
```

That's a comfortable headroom. The 1923-weight topology was inherited from 028/029 sized for pathgen rabbit-tracking; tracker mode has more inverse-projection nonlinear structure to learn (R10 troubles 2-4 + 8-10). **NN topology shouldn't be a hard constraint on 030 v1.**

### Memory budget on xiao

Current footprint:
- 1923 fp32 weights = **7.7 KB** (read-only, flash-resident)
- Hidden state: ~16 fp32 = 64 bytes
- Recurrent W_hh: 256 fp32 = 1 KB

Xiao BLE Sense has ~256 KB RAM + 1 MB flash — plenty of headroom. Scaling to 10× weight count (≈ 19,000 weights, 76 KB) is trivially within budget. Int8 weight quantization (post-training, deploy-side) divides this by 4 again — even more headroom but unnecessary for 030 v1.

### Topology growth options — Path A / B / C

**Path A — bigger network for beacon perception** (responds to R10 Trouble 2):

Hidden width 16 → 32 or 48: roughly quadratic weight growth in the bottleneck layers (~6,000-10,000 total weights). Fits comfortably in compute and memory budget.

**Sub-option A1 — pure hidden-width scale**: same topology shape, wider layers. Crude but cheap.

**Sub-option A2 — separate perception sub-network**: 2-3 small fully-connected layers that pre-process the 36 beacon inputs into 8-12 derived features (learned), feeding into the existing controller layers. Effectively *learned preprocessing* — the NN gets to discover its own (θ, r) or (midpoint, separation) representation without us prescribing it. Most compelling architectural variant: directly probes whether R10's "raw vs derived" question matters at all (if A2 evolves features that look like polar / geometric, that's the answer; if it evolves something stranger, even better).

**Triggers for Path A** (smoke-test diagnostics):
- Fitness plateaus and per-axis aggressiveness is balanced (not bang-bang collapsed) → controller has the wiring to be smooth but not the capacity to extract the signal.
- Mean-target-screen-distance is flat across gens → not learning to converge target into sights.
- CEP-sentinel-rate is high but controller behavior doesn't differentiate between visible / sentinel cases → NN can't gate, capacity-bound.
- Per-output variance high across all gens → NN noisy, hasn't found structure.

Expected outcome if Path A is the right response: smoke-test fitness curve plateaus at intermediate value; A2 ablation (vs A1 at matched weight count) shows A2 wins → confirms learned preprocessing matters → suggests R10's polar/derived representation question may also yield gains; if A1 ≈ A2, the learned features end up similar to raw-projection gradients and representation-choice is a wash.

**Path B — keep 1923 weights, go to 20 Hz tick** (responds to R10 Trouble 8):

Halves between-tick target motion. Doubles roll-rate aliasing limit to ~3600°/s. **Strongest specific argument**: if measurement of pastonly3 source dmps shows peak target roll rates near 1800°/s (Trouble 8 measurement), Path B is essentially mandatory.

Tradeoffs:
- 30 Hz camera × 20 Hz tick = 1.5 frames/tick (awkward). Either bump camera to 60 Hz (compute headroom supports) or accept reduced perception-history density.
- 6-slot history × 50 ms = **250 ms time-context** (vs current 500 ms). Halves the recurrence-horizon explicit-history covers; recurrent state has to carry more long-horizon load. Path B + Path A combo (both bigger network *and* faster tick) could compensate by adding W_hh capacity.

**Triggers for Path B**:
- Target-roll-rate distribution in pastonly3 source dmps shows >90°/tick incidence (i.e., rolling fast enough that 100 ms tick samples are aliasing).
- M2 dmp playback shows discontinuous beacon-line-angle jumps in renderer (visual aliasing signature).
- Per-axis aggressiveness on chase output's roll axis is high specifically when target roll rate is high.

Expected outcome: smoke-test fitness probably similar to Path C if pastonly3 sources don't actually push the aliasing limit; meaningfully better if they do.

**Path C — keep 10 Hz tick, spend slack on inference quality**:

Use freed compute for double-precision activation, multi-step Newton solver inside tanh, or evaluate-twice-and-average for fitness-landscape smoothing. Lowest risk; smallest payoff. Reasonable insurance if smoke green and we have nothing else to spend cycles on.

**Triggers for Path C**: smoke-test green and there's no specific fitness-quality issue to chase.

### Forward-looking architectural bets (the speculative end)

These are NOT v1 commitments. They are flagged as forward-looking research candidates so plan-research has a vocabulary for the response if smoke-test outcomes call for it. Each would constitute its own feature spec (likely 032+ territory).

**RL fine-tuning on top of GA-evolved base** (1-2 features out, speculative):

GA evolution finds a local optimum of the fitness landscape. RL methods (REINFORCE, PPO, etc.) use gradient information from rollouts to refine policy locally. *Could* squeeze additional fitness from a GA-evolved elite by smoothing its rough edges, especially in the per-tick nonlinear gating regimes (Trouble 3 sentinel handling, Trouble 4 chase-attitude extrapolation).

**Triggers**: GA fitness clearly plateaus AND eval against held-out scenarios shows the elite has *systematic* weaknesses (e.g., loses lock at every transition from open-FOV to obstructed-FOV) AND local exploration around the elite (genome-perturbation eval) shows nearby genomes with better fitness on those weakness modes.

**Concerns / expected costs**:
- Differentiable-policy infrastructure: requires the NN forward pass to be differentiable at training time. Current evaluator uses tanh — differentiable but the codebase doesn't carry a backprop framework. PyTorch / JAX side-channel during RL phases is plausible but adds dependency surface.
- Reward shaping: cone-surface fitness is computed per-tick and additive — already RL-friendly. Translating fitness into per-tick rewards is mechanical; harder is whether the elite-only RL training set is rich enough.
- Hybrid: GA finds the elite, RL refines it; deploy the RL-refined version. Decoupled cleanly.

**Expected outcome**: if RL is the right response, fitness improves another 5-15% over the GA elite (typical RL-on-top-of-imitation-learning gain). Any larger gain suggests the GA was severely undertrained; smaller gain suggests the topology is the bottleneck (back to Path A).

**Attention layers (over history slots, over beacon channels, over inputs broadly)**:

The NN currently treats all 44 inputs symmetrically — no preferential gating across history slots or input channels. Attention mechanisms (scaled dot-product, simple gated multiplicative units, etc.) let the network *learn to attend* to specific inputs based on context: e.g., when CEP is sentinel, attend to chase-side proprioception more; when both beacons clear, attend to inter-beacon geometry more.

**Triggers**: smoke green but per-axis aggressiveness telemetry shows the controller is using *all* inputs uniformly even in regimes where one input class is degraded (e.g., still using beacon (x, y) when CEP is sentinel — should be gating it off). Suggests the NN can't structurally gate, attention would let it.

**Concerns**: attention mechanisms are typically gradient-trained, not GA-evolved. Adding attention to the GA-evolved base is non-trivial — the GA would need to evolve gating weights as well as feature weights. Hybrid (GA evolves attention-equipped topology; RL refines gates) is a 032-class feature. Memory/compute cost is moderate (attention is O(n²) over n inputs but n=44 is small).

**Expected outcome**: if attention is the right response, fitness improves substantially in regimes with degraded perception (sentinel-heavy, chase-attitude-extreme). Diagnostic: fitness gap between "easy" scenarios (clean beacon visibility throughout) and "hard" scenarios (high sentinel rate, high chase-roll variance) shrinks markedly.

### Synthetic scenarios + co-evolution — the operator's open worry

**Operator concern flagged 2026-05-04**: M1 source dmps from pastonly3 / more-rnn3 are recordings of *path-tracking* controllers — the recorded craft was optimizing to follow a path point, not to be chased. As tracker-mode targets, these recordings may be *too easy* (predictable path-following) or *too hard in the wrong way* (spiral patrol behaviors that don't reflect realistic adversary motion). The chase craft converges to whatever the source distribution rewards, which may not generalize to real targets.

Two responses, both speculative:

**Synthetic scenarios — programmatically generated target trajectories**:

Tailored to test specific chase capabilities. Examples:
- "Target makes a 180° turn at constant range to chase" — exercises Trouble 5 (constant-range orbital + chase-side disambiguation).
- "Target accelerates from 5 m/s to 18 m/s during the engagement" — exercises closure-rate dynamics, the historical-R10 territory.
- "Target performs progressively-aggressive maneuvers" — curriculum on target-difficulty.
- "Target is in chaotic / random-bounded motion" — robustness to non-cooperative motion.

**Pros**: cheap (Python script generates trajectories per spec), targeted (test specific weaknesses), expandable. Map directly to the source-dmp loading interface (FR-001) — synthetic scenarios serialize as `EvalResults` with `aircraftStateList[scenario]` populated programmatically.

**Cons**: synthetic doesn't match real-target distribution. Eventually you want real flying targets; synthetic can't substitute for them.

**Co-evolution — simultaneously evolve chase + target**:

Two GA populations: chase population evolves to track; target population evolves to evade. Each chase × each target produces a fitness pair (chase fitness for tracking quality, target fitness for evasion success). This is **n² scaling** in evaluations per generation — for population N=100 each, that's 10,000 evaluations vs 200 in independent training. Hard cost.

**Pros**:
- Drives target behavior toward genuinely-evasive (not path-following or spiral-patrol) — matches the eventual real-flight adversary distribution.
- Closes a coevolutionary loop: chase improves → target improves to evade chase's specific weaknesses → chase improves to defeat new target tricks → continually rising baseline.
- Naturally surfaces the *type* of target behavior that's hardest for the chase, which informs camera-spec / NN-topology decisions downstream.

**Cons**:
- **n² compute cost** is real. With GPU-Native Evaluation (BACKLOG entry, currently re-flagged at v1→v2 boundary), n² becomes feasible — without it, prohibitive at full scale.
- Co-evolution dynamics are notoriously fragile (Red Queen dynamics, intransitive cycles, mode collapse). Mature techniques exist (hall-of-fame, Pareto-archived elites, etc.) but each adds spec complexity.
- Determinism harder to preserve: both populations sampling from joint-PRNG simultaneously.

**Triggers for synthetic / co-evolution**:
- Smoke green AND post-smoke analytics show the chase is "overfitting to pastonly3-style targets" (e.g., does great when target is doing spiral-patrol, falls apart when target is anything else).
- Eval suite against held-out source dmps from a *different* training run shows large fitness gap (controller doesn't generalize across source-controller styles).

**Expected outcome if synthetic scenarios are the right response**: synthetic curriculum (start with easy synthetic, progressively introduce harder, eventually mix in pastonly3-style) produces a controller that holds its own across all categories. Modest fitness improvement on familiar styles; large improvement on novel styles. Lower-cost intermediate option vs full co-evolution.

**Expected outcome if co-evolution is the right response**: emergent target behaviors qualitatively different from anything in the M1 source library; chase fitness against the evolved-target population at gen N looks similar to fitness against gen 0 (Red Queen at work — neither side gets "easier"). Real benefit shows up at deploy time when real targets exhibit novel behaviors the static-source training never saw.

### v1 stance on R11

Same as R10: **ship the current 1923-weight RNN at 10 Hz tick with the FR-006 input layout; instrument heavily; let smoke-test signal pick the response.** The Path A/B/C / RL / attention / synthetic / co-evolution catalog above is the menu, not the meal.

**Critical instrumentation to add at smoke-test time** (so the right response is choosable from data, not guesswork):

1. Per-tick output saturation + per-axis aggressiveness time series.
2. CEP-sentinel-rate per scenario + correlation with controller output magnitude.
3. Chase-quat-extreme-event flag per tick.
4. Inter-beacon angle change rate histogram per scenario (for Trouble 8 measurement).
5. Mean-target-screen-distance trajectory over scenario duration.
6. Fitness-vs-gen plateau detection (auto-flag when 50-gen rolling fitness improvement < 1%).

These are M11+ analytics-experimentation deliverables (per plan.md M11c "Tracker-specific analytics").

### Citations

- 030 spec FR-005 / FR-006 (current input layout, fixed v1 topology)
- R10 (perception representation question — couples with capacity question)
- 028 spec (1923-weight RNN topology origin)
- BACKLOG.md "[NEXT] GPU-Native Evaluation" (necessary for co-evolution at scale)
- BACKLOG.md "[NEXT — post more-rnn3] Genome ablation tool" (would be one tool for evaluating Path A1 vs A2)

---

## R12 — Proprioceptive history vs hidden-state integration vs pre-rotated perception

**Companion to R10 (representation) and R11 (capacity).** Where R10 asks "what shape is the perception input?" and R11 asks "how much capacity does the NN need?", R12 asks "where does the *temporal context* of the chase craft's own attitude live — in explicit input history, in the recurrent hidden state, or pre-baked into the perception input?" Decision deferred to smoke-test signal per the same pattern.

### The chase-frame coupling problem (already present in pathgen, amplified in tracker)

Pathgen `target_x/y/z[i]` is the unit vector to target's world-position-at-`t_i`, expressed in chase **body frame at time `t_i`**. If chase rolls between `t_{i-1}` and `t_i`, the unit vector appears to rotate in body-frame coordinates — *indistinguishable from target motion* without proprioceptive context. The recurrent hidden state has to integrate gyro (now-only input) into a "chase-rotated this much over the recent history" representation, and use that to disentangle apparent vector motion into "chase rolled" vs "target moved."

Tracker `BEACON_L_X[i]` is the screen position at `t_i`, in chase **camera frame at time `t_i`**. Identical structure. Same coupling. Same disentanglement burden falls on the same machinery (gyro now-input + recurrent hidden state).

So at the *architectural* level, the question "do I need proprioceptive history?" applies equally to pathgen and tracker. **The architectural problem is not tracker-specific.** Pathgen ships without explicit proprioceptive history and reportedly works; the recurrent hidden state with W_hh (256 weights, 16-wide) integrates gyro adequately for pathgen-class tasks.

### Where tracker mode genuinely raises the stakes

Two things are new and tracker-specific:

**1. Information density per slot is lower.** Pathgen gets 4 highly-informative values per slot (target unit vec + distance — fully resolves target position relative to chase). Tracker gets 6 weakly-informative values per slot (`(x_L, y_L, CEP_L, x_R, y_R, CEP_R)` — projection only, range/attitude implicit per R10's inverse-projection analysis). The disentanglement work per slot is harder; the temporal-context machinery has to do more inference with the same hidden-state capacity.

**2. Sentinel-CEP perception loss is a new regime.** Pathgen never has perception loss — the path is always known. Tracker mode does (target out of FOV, behind chase, occluded by airframe self-shadow). When CEP fires sentinel for `k` consecutive slots, the NN must **dead-reckon** for that interval — predict where the target is now from where it was last seen + chase motion since then. **Dead-reckoning across a perception gap requires knowing chase motion across the gap.** The recurrent hidden state can in principle carry this — it's been integrating gyro since boot — but the capacity question (R11) bites here. With ~16 hidden units, the hidden state has to simultaneously carry chase-pose context, target-state context, target-attitude context, and control-history context. Adding "chase pose at the time beacons were last visible" to that list may exceed the capacity.

### Four structural responses

**Option A — Hidden state suffices (status quo, what FR-006 currently specifies):**

- Existing `quat`, `airspeed`, `gyro` as 8 scalar now-only inputs. Total NN input vector = 36 + 8 = 44.
- Recurrent hidden state integrates them into temporal chase-pose context.
- Per pathgen-mode evidence: this works *for pathgen-class tasks*. For tracker-mode-class tasks with sentinel events, unproven.

**Option B — Explicit proprioceptive history:**

- Promote `quat`, `airspeed`, `gyro` to 6-slot history arrays matching beacon history.
- Input growth: from 8 scalar to 8 × 6 = **48 proprioceptive inputs**, total 36 + 48 = **84 inputs** (vs 44 today).
- Hidden state freed from gyro integration — spends capacity on inverse-projection, sentinel gating, four-way ambiguity disambiguation (R10's troubles 8/9/10).
- Cost: ~doubles input vector → ~doubles input-layer weight count → modest topology growth.
- **Quaternion-history awkwardness**: quaternions don't subtract well. Raw quat history slots present interpretable input only after the NN learns to compute `q_now * q_{t-i}.inverse()` internally — a multi-cycle nonlinear operation. Practical version uses **quaternion-difference features** (5 values per slot: rotation axis × 3 + sin/cos of angle, or 4-component delta-quat) computed on autoc side and presented to the NN. Reduces awkwardness; preserves the pre-computed-context advantage.

**Option C — Pre-rotate beacons on autoc side (the pathgen analogue):**

- At each tick, transform historical beacon positions from "chase camera frame at `t_i`" to "chase camera frame at `t_now`" using chase's own historical pose (which autoc has access to from crrcsim).
- NN sees beacons all in one consistent current-camera-frame reference. Chase-rotation history is baked into the perception input — the NN never sees the historical-frame-coupled version.
- Eliminates one inverse-problem layer entirely — same pattern pathgen uses for `target_x/y/z[i]`.
- **Sim/deploy parity requirement**: the deployed perception pipeline must do the same pre-rotation. On xiao that means buffering 6-slot quat history alongside 6-slot beacon history, then per-tick rotating older beacon slots by `q_now * q_{t-i}.inverse()` before feeding the NN. Compute is cheap (~30 cycles per slot per beacon = ~360 cycles/tick — negligible against R11's compute budget), code is small, but it's **new perception-pipeline state on the deploy side**. The 031-candidate parallel perception-front-end has to know about it.
- Diverges from "the NN sees what the FPGA produces" simplicity that the `(x, y, CEP)` interface was sold on (FR-005 / D2). The FPGA itself doesn't pre-rotate; the rotation happens in xiao firmware between FPGA output and NN input.

**Option D — Hybrid: keep raw beacon history, add quat-difference history only:**

- 36 beacon inputs (raw, current spec) + 6-slot quat-difference history (5 values × 6 slots = 30 inputs) + scalar airspeed/gyro at now (1 + 3 = 4 inputs).
- Total ~70 inputs.
- Captures the dead-reckoning-relevant proprioception explicitly without doubling everything. Airspeed and gyro stay scalar (their *now* values are sufficient as long as quat-history covers the rotational context).
- Doesn't require sim/deploy preprocessing parity *for beacons* (each side computes its own quat-difference history from its own quat stream — same pattern, but on a self-contained input dimension rather than coupled into the perception path).
- Probably the smallest meaningful architectural step that addresses the sentinel-dead-reckoning issue without over-committing to full Options B or C.

### v1 stance — ship Option A, smoke-test signal picks the response

Same pattern as R10/R11. v1 ships with FR-006's current 44-input layout (Option A); smoke-test diagnostics distinguish which option is the right next step.

**Diagnostics that distinguish which option is right** (M11+ analytics deliverables):

1. **Sentinel-recovery quality** (the load-bearing one): for each scenario, slice into `(visible-stretch, sentinel-burst, post-sentinel-recovery-stretch)` segments. Compare the controller's tracking-error trajectory in the post-recovery stretch vs the visible stretch immediately preceding the sentinel burst. If post-recovery error is systematically higher (tail of error decays slowly, or the controller has to "re-acquire" the target via fresh search-orbit-style behavior), dead-reckoning is failing → Options B/C/D become more attractive.
2. **Chase-rotation-vs-beacon-motion correlation**: when chase is rotating quickly (high `|gyro|`) and beacons appear to rotate on screen, does the NN's output reflect "I'm rotating, target is steady" (correct attribution) or "target is moving fast" (wrong attribution)? Detectable via correlating `|chase gyro|` with `|chase output|` — if they're tightly coupled regardless of actual target motion (measured via `|target_velocity|` from M2 dmp's `targetTrajectoryList`), the NN is mis-attributing chase rotation to target motion → hidden state isn't disentangling adequately.
3. **Hidden-state activation patterns**: if smoke-test fitness plateaus and the hidden-state activations are saturating broadly (most units pinned at ±1), capacity exhaustion → R11 Path A *or* R12 Option B/D — explicit history may be cheaper than just growing hidden width.

### Expected outcomes per option

| Option | Smoke-test outcome (subjective probability) | Diagnostic signature |
|---|---|---|
| **A** wins (status quo) | 30-40% | Smoke fitness descends; sentinel-recovery time-constant similar to in-stretch tracking quality; chase-rotation-vs-output correlation low |
| **D** wins (quat-diff history) | 30-40% | A plateaus at intermediate fitness; adding ~30 quat-diff inputs (cheap topology change) recovers fitness; sentinel-recovery improves measurably |
| **B** wins (full proprio history) | 15-20% | D insufficient; specifically airspeed-history or gyro-history *also* shows up as a missing-input failure mode (e.g., chase mis-attributes its own rate-of-rotation to perception inputs) |
| **C** wins (pre-rotated perception) | 10-15% | Even B's input expansion isn't enough; the inverse-projection burden in the NN is the actual bottleneck and pre-rotation is the only relief. Lowest probability because it requires sim/deploy parity changes |

The probabilities are subjective and reflect: D feels right intuitively (smallest meaningful step that addresses sentinel-dead-reckoning explicitly); A is the pathgen-precedent default; C is the architecturally cleanest but most expensive in deploy-side work; B is the "if D isn't enough, throw input vector at it" fallback.

### v1 expected outcome (best-guess if smoke is green)

Best-case smoke result: A is *adequate* (loop closes, fitness descends, sentinel-recovery is mediocre but not catastrophic). Then the obvious M11+ followup is "add quat-diff history (Option D-light) and re-run, compare." If that yields measurable improvement, the case for D solidifies. If not, A holds and the architectural question is settled until 030+ stress regimes (hard sentinel events, fast chase rotation) push A past breakage.

If smoke is *red* and the diagnostics from above point at sentinel-recovery or chase-rotation-mis-attribution, that argues directly for D (cheapest fix). If diagnostics point at hidden-state saturation broadly, that's R11's Path A territory (bigger hidden state) — possibly combined with R12's Option D (explicit history reduces what the hidden state has to carry, making bigger-hidden-state more effective).

### Citations

- 030 spec FR-005 / FR-006 (current input layout)
- R10 (perception representation — the inverse-projection problem this section interacts with)
- R11 (capacity question — Path A interacts with R12 Option D in particular: explicit history reduces hidden-state load)
- pathgen `nn_input_computation.cc` / `nn_inputs.h` (existing pattern that v1 inherits — where the chase-frame coupling and recurrent hidden-state-integrates-gyro pattern originates)
- M11+ analytics (where these diagnostics would actually be implemented and run)

---

## R10-historical — Trail-rabbit degenerate-velocity fallback (moot for v1)

> **Note (2026-05-04 scope correction)**: the original R10 was about the trail-rabbit fallback at near-zero target speed (per FR-008a). This is **moot for v1**: source M1 dmps come from pastonly3 / more-rnn3-class flying controllers cruising 13–18 m/s; the slowest event in the entire 2026-05-03 flight log was ~4 m/s during a brief stall-like episode — well above any conceivable nose-fallback threshold. The degenerate branch is dead code on v1 source data.
>
> The fallback IS forward-looking-relevant for the [031 CANDIDATE] Variable-rate / real-flight source robustness work, when real-flight recordings might include taxi / hover / takeoff / loiter segments. Folded into that BACKLOG entry; not a v1 plan-research item.
>
> v1 trail rabbit implementation: **unconditional velocity-trail** (`rabbit = target_position − target_velocity_unit × trail_distance`) with an optional sanity assertion that `|target_velocity| > 0.5 m/s` per source-tick (rejects the source dmp loudly on violation, per Constitution V's loud-fail rule).

---

## Summary table — R1–R10 outcomes

| ID | Topic | Decision (one-line) |
|---|---|---|
| R1 | Source dmp → projection integration (v1) | Target trajectory lives in autoc memory only; crrcsim runs chase alone; mod_robots / `RobotProgrammable` deferred to post-v1 (video-enabled mode) |
| R2 | Arena enforcement primitive | New `FLIGHT_ARENA_*` constants (parallel to existing `ENTRY_SAFE_*`); v1 defaults `RADIUS=80m`, `FLOOR_AGL=5m`, `CEILING_AGL=100m` |
| R3 | `p_crash` v1 default | Curriculum-anneal `0.0 → 0.30` plateau by gen 200; `pCrashGen0 / pCrashGenRamp / pCrashGenPlateau` in `autoc-tracker.ini` |
| R4 | Type-safe sensor interface shape | `enum class NNInput : uint16_t` + parallel `constexpr` metadata array; xiao mirror via shared header |
| R5 | FR-018 main-loop refactor | Strategy pattern: `ScenarioStepper` interface; `PathgenStepper` (refactor) + `TrackerStepper` (new) |
| R6 | CEP encoding | Linear `[0.0, 1.0]` for visible; `1.5` (encoded `INT8_MIN`) as invisibility sentinel |
| R7 | int8 quantization math | Symmetric `[-127, +127] ↔ [-1, +1]` for x/y; `[0, +127] ↔ [0, 1]` for CEP; `INT8_MIN` reserved for invisibility |
| R8 | Source dmp schema gap | None — existing `EvalResults` carries everything; v1 hardcodes target wingspan (hb1) |
| R9 | Renderer / FR-005 independence | Structurally separate; renderer reads recorded `(x, y, CEP)` from M2 dmp, does not share projection code |
| R10 | **Perception representation** (load-bearing v1 question) | Ship raw `(x, y, CEP)` per FR-005; smoke test (M10) IS the experiment; trouble-list 1-10 catalogued with per-trouble v1 disposition; polar / derived-features as next-iteration response if smoke signals representation-bound failure |
| R11 | **NN compute / memory / topology growth** (companion to R10) | Compute headroom 16-50× available at 20 Hz tick; memory headroom 30× on xiao; ship 1923-weight v1; Path A/B/C topology-growth options + RL / attention / synthetic-scenarios / co-evolution forward-looking with explicit triggers tied to smoke-test diagnostics |
| R12 | **Proprioceptive history vs hidden-state integration vs pre-rotated perception** (companion to R10/R11) | Chase-frame coupling problem (already in pathgen, amplified in tracker by lower per-slot info density + sentinel-CEP dead-reckoning need); ship Option A (status quo, hidden state only) for v1; Options B (explicit proprio history, +48 inputs), C (pre-rotated beacons, sim/deploy parity required), D (hybrid: quat-diff history only, +30 inputs) on the menu with explicit smoke-test triggers |
| R10-historical | Trail-rabbit degenerate fallback | Moot for v1 (source dmps never approach low-speed regime); v1 = unconditional velocity-trail; folded into 031 variable-rate source robustness |

All R-questions resolved. Plan phase 0 complete; proceed to plan phase 1 (data-model, contracts, quickstart).

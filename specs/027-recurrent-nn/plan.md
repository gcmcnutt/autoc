# 027 — Implementation Plan (Recurrent NN + lexicase smoothness)

Companion to [`spec.md`](./spec.md) (scope + 5 locked clarifications)
and [`research.md`](./research.md) (literature + bias ladder). This
doc orders the work to get the sim binary compiling, running, and
producing a measurable cadence9 training result — the primary bet
(**D-simple + C2-via-lexicase**) from the clarify phase.

## Ordering principle

**Compile before behavior before training before flight.** Phases 1–5
are code changes needed to get autoc + crrcsim + minisim + renderer +
nn2cpp all compiling together with the new topology and selection
plumbing. Phase 6 rebuilds + runs the cadence7-parity sanity (#0).
Phase 7 is the primary bet training run (#3). Phase 8 is analysis and
go/no-go. Xiao porting (Q5) is deferred behind the sim gate and not
part of this plan — a separate plan will cover the xiao side when
#3 proves the architecture works.

## Go/no-go gate (Phase 7 → 8, trigger escalation or close)

Same shape as 026's gate, retargeted for the D-simple + C2 bet.

| Metric | cadence7 baseline | pid1 (NO-GO ref) | **027 #3 target** | Interpretation |
|---|---:|---:|---:|---|
| `<|Δout|>` / tick (dCtrl, late plateau, mean-of-paths) | ~1.00 | 1.60 | **≤ 0.80** | ≥ 20 % reduction in stick speed vs cadence7 |
| `<|out|>` / tick (amplitude, late plateau, mean-of-paths) | ~2.20 | 2.54 | **≤ 2.00** | ≥ 10 % reduction in saturation |
| Best fitness @ gen 400 | −35951 | −27045 | **≥ −30000** | Within ~15 % of cadence7; no worse than pid1 |
| Output histogram (final gen) | bimodal ±1 | more bimodal | spread toward 0 | visual proof of regime change |

**All three quantitative bars must hit.** Histogram is qualitative.
Missing any quantitative bar → escalation (decide #4 D-alone diagnostic
vs. 32-wide vs. D-ESN after reviewing the shape of the failure).

Note: `< -30000` is explicitly *above* pid1's −27045 and roughly 83 %
of cadence7. The cost of memory must be paid back by the time this
many gens are done.

## Phase 1 — NN topology + evaluator forward pass (foundation)

### 1.1 `include/autoc/nn/topology.h`

Extend the topology descriptor with per-layer recurrent flag. Two
options:
- **Option A (chosen)**: keep `NN_TOPOLOGY` as sizes; add a parallel
  `NN_RECURRENT[NN_NUM_LAYERS]` bool constexpr (layer 2 = true,
  others = false). Keeps existing code reading `NN_TOPOLOGY[i]`
  unchanged.
- Option B: change `NN_TOPOLOGY` to `struct LayerSpec { int size;
  bool recurrent; }` — cleaner but touches many call sites.

Update `NN_WEIGHT_COUNT` to include W_hh for each recurrent layer:
```
for each recurrent layer i: weights += NN_TOPOLOGY[i]² (W_hh matrix, no bias)
```
Target: 1667 + 256 = **1923**.

### 1.2 `include/autoc/nn/evaluator.h` + `src/nn/evaluator.cc`

Two signature options for carrying state across ticks:
- **Option A (chosen)**: `NNControllerBackend` grows a persistent
  state member (`std::vector<float> hidden_state_`), a `reset()`
  method that zeros it, and `nn_forward_recurrent()` internally
  maintains it across calls. Callers construct the backend **once
  per span** (not per tick) and call `evaluate()` per tick. Reset
  on span start.
- Option B: state lives on AircraftState (tick-external); pure
  `nn_forward()` function takes state as in/out parameter. Rejected
  because it couples NN memory to aircraft_state.h which lots of
  non-NN code includes.

Inside `nn_forward_recurrent`, the recurrent layer becomes:
```
h_t = tanh(W_xh · x_t + W_hh · h_{t-1} + b_h)
```
Non-recurrent layers unchanged.

**Per clarify Q4**: `evaluate()` advances `hidden_state_` on every
call. Callers in crrcsim / minisim must call `evaluate()` only on
NN eval ticks (10 Hz), not intermediate outer-frame ticks — and
they already do, so this is a contract, not a code change.

### 1.3 `src/nn/population.cc` — xavier init handles W_hh

`nn_xavier_init` currently walks `NN_TOPOLOGY` layer-by-layer and
emits `W · input_count` + `bias` float ranges. Extend to emit the
extra W_hh block for each recurrent layer (use Xavier fan-in of
layer size + input size together, or a simpler orthogonal init —
decide in tasks). Crossover/mutation operate on flat weight vector
and need no change.

## Phase 2 — Serialization + downstream consumers

### 2.1 `src/nn/serialization.cc`

Serialized binary is a flat float array + topology header. Topology
serialization needs to include the recurrent-flag array so a loaded
NNGenome knows which layers carry state. Backward compat: none (per
project policy; old `.dmp` files become unloadable).

### 2.2 `tools/nn2cpp.cc`

Generated C code for xiao must compile even though we're not
flashing. Add to the generator:
- Emit `static const bool nn_recurrent[] = {...}` array alongside
  existing `nn_topology[]`.
- Emit `static float nn_hidden_state[N] = {0}` for the recurrent
  layer.
- Emit `nn_reset()` function that zeros `nn_hidden_state`.
- Update the forward-pass emission to add the W_hh mat-vec for the
  recurrent layer.

This is compile-only deliverable for now. The xiao won't link
against it until Q5's sim gate passes (deferred).

### 2.3 `tools/nnextractor.cc` and `tools/minisim.cc`

- `nnextractor`: reads NNGenome from S3 .dmp, writes `nn_weights.dat`
  — weight count changed. Should "just work" if it reads size from
  the file header; verify.
- `minisim`: uses `NNControllerBackend` per-tick. Switch to the
  construct-once-per-span + `reset()` pattern (same as crrcsim).

### 2.4 `tools/renderer.cc`

Uses EvalResults (via cereal) for rendering. No NN forward pass on
the renderer side. Just needs to compile against the new topology
constants. If it does NN math anywhere (e.g., per-tick re-evaluation
for scrubbing) — need a reset hook on span-start playback. TBD
during implementation.

## Phase 3 — C2 smoothness test case for lexicase

### 3.1 `include/autoc/eval/fitness_decomposition.h`

Extend `ScenarioScore`:
```cpp
struct ScenarioScore {
    double score;              // existing tracking score
    double smoothness_score;   // NEW: sum of |Δout| across scenario ticks
    bool crashed;
    int steps_completed;
    // ... existing fields
};
```

Lower smoothness_score = better (less bang-bang).

### 3.2 `src/nn/evaluator.cc` / `src/autoc.cc`

Compute smoothness per scenario during the aircraftStates walk that
already runs for data.dat writing. Formula:
```
smoothness = Σ over consecutive ticks (|ΔoutPt| + |ΔoutRl| + |ΔoutTh|)
```
Normalize by scenario step count so short/long scenarios aren't
scored differently on duration alone.

### 3.3 `src/eval/selection.cc` — lexicase pool extension

Current lexicase shuffles per-scenario tracking scores. Extend to
include per-scenario smoothness scores as additional test cases in
the pool. Effect: each lexicase selection event can filter on
either a tracking dimension or a smoothness dimension (in random
order), so the survivors must be both well-tracking AND
non-jittery.

Implementation: `lexicase_select()` takes a flat vector of test
cases per individual; tracking and smoothness both become
`(scenario_idx, kind, score)` triples. Pool size doubles. Epsilon
tuning may need a per-kind scale (tracking and smoothness have
different numeric ranges).

### 3.4 `tests/selection_tests.cc` — smoothness test case coverage

Add 2-3 cases:
- Two individuals, equal tracking, different smoothness → smoother
  survives.
- Tradeoff case: individual A tracks better but is jittery;
  individual B tracks worse but smooth — lexicase should let both
  survive depending on test-case order.

## Phase 4 — Sim integration (crrcsim reset hook)

### 4.1 `crrcsim/.../inputdev_autoc.cpp`

Where the span-start reset block already clears aircraftStates and
scenario state (~line 520), add:
```cpp
nnController_.reset();   // zeros recurrent hidden state
```

Requires `T_TX_InterfaceAUTOC` to own an `NNControllerBackend
nnController_` member (currently constructed per-tick; see Phase
1.2 refactor).

### 4.2 Cadence check

Confirm `evaluate()` fires only on NN eval ticks (10 Hz), not on
every outer-frame (20 Hz) tick. This is the existing cadence — the
`framesPerEval` gate — so no new code, just a comment noting the
hidden-state semantics depend on this behavior.

## Phase 5 — Tests + build verification

- `tests/contract_evaluator_tests.cc`: `NN_WEIGHT_COUNT == 1923`,
  topology shape, recurrent flag array shape.
- `tests/nn_evaluator_tests.cc`: forward pass round-trip with known
  inputs + reset semantics (h_t zeros after reset, advances on
  evaluate, identical output for identical input when reset between).
- `tests/nn_serialization_tests.cc`: serialize NNGenome with recurrent
  layer, deserialize, weight count matches.
- `tests/selection_tests.cc`: Phase 3.4 additions.
- CMakeLists: no new test files needed beyond Phase 5 updates.

**All tests must pass after a full rebuild before Phase 6 starts.**

## Phase 6 — Rebuild + baseline sanity (#0)

### 6.1 Rebuild
```
cd build && make -j8
```
Targets: autoc, crrcsim, renderer, minisim, nn2cpp, all _tests.

### 6.2 Baseline smoke (#0)

Short training run, ~30-50 gens, on the 027 binary with the new
topology + C2 wired in but D-simple disabled (i.e., recurrent flag
off, behaves like cadence7-parity feedforward). Optional — the PID
nullification alone was enough to recover pre-026 behavior in the
prior training. If we want an explicit #0, add a compile-time
`#define DSIMPLE_DISABLE` or a per-run no-op of the recurrent
layer.

**Actually, leaving #0 optional.** The infrastructure change alone
(1923 weights, hidden state, reset hook) is the first novelty;
running #3 directly IS the first meaningful data point. If #3
collapses immediately we'll learn from the collapse curve.

## Phase 7 — Primary training run (#3)

```
nohup ./build/autoc -c autoc.ini > logs/autoc-027-cadence9.log 2>&1 &
```

Budget: 400 generations, matched-compute with cadence7/pid1.
Monitor during run:
- Every 50 gens: regenerate fitness ramp PNG (plot_fitness_ramp.py)
  with cadence7 + pid1 as priors.
- Every 50 gens: regenerate control_aggressiveness PNG.
- If dCtrl trending *above* pid1's 1.60 by gen 100 — early-stop and
  diagnose.

## Phase 8 — Analyze + decide

Produce:
- `specs/027-recurrent-nn/cadence9_measurement.md` with fitness
  ramp, aggressiveness PNG, decision.
- Go/no-go against Phase 0 gate.

**If GO**: move to xiao porting plan (separate doc, triggered by
this phase's outcome). Escalation ladder remains in reserve.

**If NO-GO**: branch decision tree —
- Aggressiveness improved but fitness regressed → D-simple too
  expensive; try 32-wide (#4) OR diagnose with D-alone no-C2 (#5).
- Neither improved → D-simple not the right architecture. Revisit
  D-ESN or rethink.
- Fitness fine, aggressiveness unchanged → C2 signal not biting.
  Revisit C1 α-penalty as additional pressure or tune ε-lexicase.

## What this plan intentionally does NOT cover

- **Xiao port** — deferred behind sim gate per clarify Q5.
- **Flight test** — same.
- **#1 A2-alone, #2 A2+C2** — reserved per clarify Q2, not in
  MVP.
- **#4 D-alone diagnostic** — decided post-#3 based on outcome.
- **32-wide recurrent, D-ESN, GRU-lite** — escalation, out of
  MVP.
- **025 craft variations** — blocked pending 027 outcome.

## Open implementation decisions (resolve in tasks)

1. **Xavier init for W_hh** — standard Xavier fan-in-based sigma,
   or orthogonal init (standard for RNN in gradient descent, may
   be overkill for evolution)?
2. **Smoothness normalization** — per-tick mean, per-scenario
   total, or per-second rate? Affects epsilon tuning in lexicase.
3. **Lexicase pool ordering** — shuffle tracking and smoothness
   test cases together (equal weight) or interleave (alternating
   kinds)?
4. **Per-kind epsilon for lexicase** — one epsilon for everything,
   or tracking-epsilon and smoothness-epsilon separately calibrated
   to their scales?
5. **Renderer scrubbing and hidden state** — when user scrubs the
   renderer timeline backwards, does the hidden state get
   recomputed from span start or persisted from forward-only? Low
   priority; can land incomplete if it's nontrivial.

These should be resolved in `/speckit.tasks` or during
implementation — not worth a new clarify pass.

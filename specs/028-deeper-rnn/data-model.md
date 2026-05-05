# 028 — Data Model

Telemetry-signal data model + per-generation log schema additions for 028.
028 introduces no new persistent file formats and no NNGenome schema
changes (per [spec §Carry-forward](./spec.md#carry-forward-inventory) —
recurrent flag is already serialized from 027). All new state is
**evolution-log only** (one new column-set in the per-generation log
line) plus **derived robustness metrics** computed at close-of-attempt.

## 1. Per-generation log line — `#NNGen`

### 1.1 Existing schema (027 baseline, before 028 changes)

Source: [`src/autoc.cc:1194-1203`](../../src/autoc.cc).

```text
#NNGen gen=<int> best=<float> avg=<float> worst=<float> bestSigma=<float>
       avgMaxStreak=<float> pctInStreak=<float> stability=<float> energy=<float>
```

| Field | Type | Meaning | Sign convention |
|---|---|---|---|
| `gen` | int | Generation index (0-based) | — |
| `best` | float | Min fitness (lower = better) | negative |
| `avg` | float | Population mean fitness | negative |
| `worst` | float | Max fitness (worst individual) | negative |
| `bestSigma` | float | Best individual's mutation sigma | positive |
| `avgMaxStreak` | float | Mean of `maxStreak` across best's scenarios | positive |
| `pctInStreak` | float | % of total steps that were "in streak" | 0–100 |
| `stability` | float | Σ stability_score across best's scenarios | negative |
| `energy` | float | Σ energy_score across best's scenarios | negative |

### 1.2 028 additions

Two new column-sets, appended after the existing fields:

```text
#NNGen gen=... energy=...
       whh_xh_ratio=<float>
       w_xh0_cv=<float> w_xh1_cv=<float> w_hh_cv=<float>
```

| Field | Type | Meaning | Sentinel (no recurrent layer) |
|---|---|---|---|
| `whh_xh_ratio` | float | **Signal 1**: best individual's W_hh activation magnitude / W_xh activation magnitude (recurrent layer only) | `0.0` |
| `w_xh0_cv` | float | **Signal 2a**: population-level CV of layer-0 input weight block | computed normally |
| `w_xh1_cv` | float | **Signal 2b**: population-level CV of layer-1 input weight block | computed normally |
| `w_hh_cv` | float | **Signal 2c**: population-level CV of recurrent layer's W_hh block | `NaN` (decision deferred to tasks; alternative: `0.0`) |

**Decision (deferred to `/speckit.tasks`):** sentinel for `w_hh_cv` when no recurrent layer.
NaN signals "not applicable" cleanly to plotting code; 0.0 plots as a flat line. NaN preferred
unless plot library handling is awkward.

### 1.3 Sign / scale conventions

- `whh_xh_ratio`: dimensionless, ≥ 0. Expected range from §Phase 1.5 calibration: 0 (block dead)
  to O(1) (block engaged). Final threshold for "engaged" picked in tasks.
- `w_*_cv`: dimensionless, ≥ 0. Standard deviation / mean(|w|). Smaller = population converging.
  Useful comparison is **between blocks** in the same generation, not between generations.

## 2. Signal 1 — W_hh / W_xh activation ratio (computation)

### 2.1 Definition

For the best individual at end-of-generation, evaluate over a representative scenario set (or
piggyback on the existing best-of-gen eval pass). At each tick `t` of recurrent layer evaluation:

```
xh_mag_t  = |W_xh · x_t + b_h|             (per-neuron, vector of size hidden_dim)
hh_mag_t  = |W_hh · h_{t-1}|                (per-neuron, vector of size hidden_dim)
```

Aggregate across ticks and neurons:

```
xh_mean = mean over (neurons, ticks) of xh_mag_t
hh_mean = mean over (neurons, ticks) of hh_mag_t
ratio   = hh_mean / max(xh_mean, ε)        (ε = 1e-9, guards against zero division)
```

**Aggregation choice (deferred to tasks)**: mean vs max across neurons. Mean is more robust;
max highlights any single neuron with strong recurrent feedback. Defaulting to **mean**.

### 2.2 Capture cadence

- **Best-of-gen only** — not per-individual during selection (would dominate eval cost).
- Set a `telemetry_capture_flag` in the eval call for the best individual after selection
  ranks the population. Recurrent forward pass checks the flag and accumulates magnitudes.
- Default: every generation. Cheaper alternative (every Nth gen) deferred to tasks.

### 2.3 Test surface

`tests/nn_telemetry_tests.cc` (new):
- **Lower bound**: hand-set `W_hh` to all zeros, run forward pass on a known input sequence.
  Assert `ratio == 0.0`.
- **Upper bound**: hand-set `W_hh` to identity (16×16 I), `W_xh` to 0.1·I (or similar),
  feed unit-norm inputs. Assert ratio is in expected order-of-magnitude.
- **Sentinel**: build with `NN_RECURRENT[2] = false`, run, assert `ratio == 0.0`.

## 3. Signal 2 — W_hh population coefficient of variation (computation)

### 3.1 Definition

For each weight block separately, across the entire population:

```
For block B in {W_xh[0], W_xh[1], W_hh[2]}:
  pop_mean[B]   = mean over individuals of mean(|w| for w in B)
  pop_stddev[B] = stddev over individuals of mean(|w| for w in B)
  cv[B]         = pop_stddev[B] / max(pop_mean[B], ε)
```

This is a "per-block CV": how much variation is there in *aggregate magnitude* of each block
across the population, normalized by mean magnitude. Falling CV = population converging on
similar magnitudes for that block. Persistent high CV in `W_hh` while `W_xh` CV falls = GA
explores W_hh inefficiently → hypothesis 1.

**Definition rationale (load-bearing for hypothesis-1 discrimination)**: this metric is
*aggregate per-individual block magnitude*, not per-weight position-wise stddev across the
population. Reason: hypothesis 1 predicts the GA fails to *explore* W_hh as a region — i.e.,
the population settles on similar block-level scales — which this metric captures directly.
A per-weight CV would conflate "GA isn't searching" with "GA found a symmetric weight
distribution" and could mask the signal. If the metric ever needs a sanity-check companion,
add per-weight Frobenius-norm spread as a second column rather than replace this one.

### 3.2 Capture cadence

- Every generation (cost is `O(N_pop × N_weights_per_block)` — already incurred during the
  per-gen population walk for fitness aggregation; piggyback there).
- Hook: extend the existing population stats loop in `src/autoc.cc` around line 1180–1200.

### 3.3 Test surface

`tests/nn_telemetry_tests.cc`:
- **Identical population**: all individuals have identical weights → cv == 0.0 for all blocks.
- **Bimodal population**: half at +1.0, half at −1.0 → cv computed against |w| means, should
  match analytic prediction.
- **Sentinel**: layer with `NN_RECURRENT[i] = false` → block doesn't exist → field emits
  `NaN` (or 0.0 — pick in tasks).

## 4. Robustness metric — `pop_spread` (close-of-attempt only)

### 4.1 Definition

After a candidate-winner attempt finishes (hits sim gate), compute:

```
pop_spread = IQR(fitness over last 50 gens × all individuals) / |median(fitness)|
```

Equivalent forms acceptable: top-5%-vs-95% range / median, late-gen population
fitness stddev / |mean|. Final form picked in tasks based on what reads cleanly off existing
population stats.

### 4.2 Threshold

Spec Q5 mitigation: `pop_spread < 0.05` flags the run as seed-suspect. This number is a
placeholder — calibrated empirically against the cadence7-redux baseline run, where the
population is known-non-degenerate.

### 4.3 Storage

Computed post-hoc by an analysis script reading the evolution log and per-individual fitness
series. **Not** added to the per-gen log line. Output: a one-line summary in
`logs/autoc-028-<attempt>.summary` or echoed by the analysis script. No new on-disk schema.

## 5. ScenarioScore — no changes for 028

[`include/autoc/eval/fitness_decomposition.h`](../../include/autoc/eval/fitness_decomposition.h)
already carries `score`, `stability_score`, `energy_score` from 027 (per
[spec §Carry-forward](./spec.md#carry-forward-inventory)). 028 adds no new
ScenarioScore fields. Telemetry signals are computed alongside, not stored on
ScenarioScore.

## 6. NNGenome serialization — no changes for 028

`src/nn/serialization.cc` already serializes the recurrent flag array (027 deliverable T023).
Per [feedback memory: no cereal versioning](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md),
no schema bump. 028 reads 027-format `.dmp` files unchanged.

## 7. Field validation rules

- `whh_xh_ratio`: must be ≥ 0 OR `0.0` (sentinel). Negative or NaN value at runtime indicates
  a bug.
- `w_*_cv`: must be ≥ 0 OR sentinel. Same.
- `pop_spread`: must be > 0 for any non-degenerate population. ≤ 0 indicates a bug or
  degenerate population (and should trigger investigation, not just a silent flag).

## 8. State transitions — none

028 introduces no new state-machine entities. All telemetry is per-generation snapshot
data, computed and emitted in line with the existing evolution loop.

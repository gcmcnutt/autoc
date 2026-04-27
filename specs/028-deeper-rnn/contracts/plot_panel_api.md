# Contract: 6-panel evolution plot

**Producer**: [`specs/028-deeper-rnn/plot_evolution_progress.py`](../plot_evolution_progress.py)
(new — extends or imports 027's
[`plot_evolution_progress.py`](../../027-recurrent-nn/plot_evolution_progress.py)).

**Consumer**: Operator looking at training-run progress. Output is a PNG; no programmatic
downstream consumer beyond visual inspection.

**Invocation**: `python3 specs/028-deeper-rnn/plot_evolution_progress.py <log_file> [--out <out.png>]`

## Panel layout

```text
┌─────────────────────────────────────────────────────────────────┐
│ Panel 1: fitness   │ best, avg, worst over generations          │
├─────────────────────────────────────────────────────────────────┤
│ Panel 2: streak    │ avgMaxStreak + pctInStreak                  │
├─────────────────────────────────────────────────────────────────┤
│ Panel 3: stability │ totalStability over generations             │
├─────────────────────────────────────────────────────────────────┤
│ Panel 4: energy    │ totalEnergy over generations                │
├─────────────────────────────────────────────────────────────────┤
│ Panel 5: sigma     │ bestSigma over generations                  │
├─────────────────────────────────────────────────────────────────┤
│ Panel 6: telemetry │ ─── NEW IN 028 ───                          │
│  - top half:    whh_xh_ratio + horizontal threshold line        │
│  - bottom half: w_xh0_cv (orange), w_xh1_cv (blue),             │
│                 w_hh_cv (red) — three traces overlaid           │
└─────────────────────────────────────────────────────────────────┘
```

Panels 1–5 carry through unchanged from 027.

## Panel 6 contract

### Inputs (from `#NNGen` log lines per [`evolution_log_columns.md`](./evolution_log_columns.md))

- `whh_xh_ratio` (one float per generation)
- `w_xh0_cv`, `w_xh1_cv`, `w_hh_cv` (three floats per generation)

### Top half — activation ratio

- X axis: generation index
- Y axis: ratio value (linear, ≥ 0)
- Trace: `whh_xh_ratio` as a line
- Horizontal reference line: threshold from [`data-model.md` §2.3](../data-model.md#23-test-surface)
  upper-bound calibration. Default: 10 % of the unit-test-derived "engaged" value, or midpoint
  (decision deferred to tasks). Drawn as a dashed gray line.
- Annotation: a label "block engaged" above the threshold line, "block dead" below.

### Bottom half — population CV

- X axis: generation index (shared with top half)
- Y axis: CV (linear, ≥ 0)
- Three traces:
  - `w_xh0_cv` — color: tab:orange (matplotlib default cycle [1])
  - `w_xh1_cv` — color: tab:blue (matplotlib default cycle [0])
  - `w_hh_cv` — color: tab:red (matplotlib default cycle [3])
- Legend: layer/block names, top-right corner.
- No threshold line — interpretation is *relative*: w_hh_cv consistently above w_xh*_cv =
  GA not searching W_hh effectively (hypothesis 1 signal).

### Sentinel handling

- When `whh_xh_ratio == 0.0` for all generations (no recurrent layer), top half renders
  an empty plot with a "no recurrent layer" annotation rather than a flat zero line.
- When `w_hh_cv == nan` for all generations, the red trace is omitted from the bottom-half
  legend.

## Failure modes (parser robustness)

- **Missing 028 fields** in older log files: panels 1–5 render, panel 6 renders empty
  with a "no telemetry data" annotation. **Do not crash.**
- **Out-of-order keys**: parser is key-value (not positional); robust to ordering changes.
- **NaN values**: matplotlib renders gaps; do not error.
- **Duplicate generation numbers**: keep the *last* line for each gen index (covers reruns
  that overwrite without log truncation).

## Test surface

A pytest test under `specs/028-deeper-rnn/test_plot_evolution_progress.py` (location TBD —
match where 027's plot tests live, if any):

| Test | Fixture | Assertion |
|---|---|---|
| 027-only log → 5 panels rendered, 6th empty | `fixtures/027_log.txt` | image dimensions match 6-panel layout, panel 6 has empty annotation |
| 028 full log → all 6 panels rendered with data | `fixtures/028_log.txt` | all four traces present, threshold line drawn |
| Sentinel `whh_xh_ratio == 0.0` → top half labeled "no recurrent layer" | `fixtures/028_no_recurrent.txt` | annotation present, no flat-zero line drawn |
| Out-of-order keys parse correctly | `fixtures/028_shuffled_keys.txt` | output identical to in-order fixture |

If pytest infrastructure for plot scripts doesn't exist in 027's tree, the contract test
becomes a one-shot validation in `quickstart.md` instead. Decision in tasks.

## Open contract decisions (resolve in tasks)

1. Whether to fork 027's plot script or extend via Python import. Fork is simpler; extend
   keeps 5-panel rendering DRY.
2. Threshold line position on panel 6 top half (10 % of engaged vs midpoint).
3. Whether "block engaged" / "block dead" labels are useful or visual clutter.

# Contract: Evolution log columns (`#NNGen` line)

**Producer**: [`src/autoc.cc:1194-1203`](../../../src/autoc.cc) (existing) +
028 telemetry hooks ([plan §1.1, §1.2](../plan.md#11-signal-1--w_hh--w_xh-activation-ratio-best-individual-per-generation)).

**Consumer**: [`specs/028-deeper-rnn/plot_evolution_progress.py`](../plot_evolution_progress.py)
(extending 027's plot script), any downstream analysis scripts that grep `#NNGen` lines.

**Format**: Single-line, space-delimited `key=value` pairs, prefixed by `#NNGen`. One line per
generation. Newline-terminated. UTF-8.

## Field order (left to right)

```text
#NNGen gen=<int>
       best=<float> avg=<float> worst=<float>
       bestSigma=<float>
       avgMaxStreak=<float> pctInStreak=<float>
       stability=<float> energy=<float>
       whh_xh_ratio=<float>
       w_xh0_cv=<float> w_xh1_cv=<float> w_hh_cv=<float>
```

(Line breaks above are for readability — the actual log is one physical line.)

## Field contracts

### Existing (027 baseline — unchanged in 028)

See [`data-model.md` §1.1](../data-model.md#11-existing-schema-027-baseline-before-028-changes).
Producer must keep these fields stable in name and order; downstream tooling depends on
`grep '^#NNGen'` + key-value parse.

### New (028 additions, appended after `energy`)

| Field | Format | Range | Sentinel | Producer hook |
|---|---|---|---|---|
| `whh_xh_ratio` | `%.4f` | `[0.0, ~10.0]` typical; ≥ 0 always | `0.0` when `NN_RECURRENT[i]==false` for all i | recurrent forward pass, telemetry capture flag set for best-of-gen eval |
| `w_xh0_cv` | `%.4f` | `[0.0, ~5.0]` typical; ≥ 0 always | computed normally (block always present) | population stats loop |
| `w_xh1_cv` | `%.4f` | `[0.0, ~5.0]` typical; ≥ 0 always | computed normally (block always present) | population stats loop |
| `w_hh_cv` | `%.4f` | `[0.0, ~5.0]` typical; ≥ 0 always | `nan` when `NN_RECURRENT[2]==false` (decision deferred to tasks; alt: `0.0`) | population stats loop |

## Backward-compatibility contract

- 028 is **additive**: existing fields unchanged in name, order, or format.
- Older parsers that don't know about the four new fields ignore unknown keys (key-value
  format makes this safe).
- Per [feedback memory: no cereal versioning](/home/gmcnutt/.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md),
  no version field added to the log line — schema bumps are tracked via spec/feature numbers.

## Producer test surface

`tests/nn_telemetry_tests.cc` (new):

| Test | Assertion |
|---|---|
| `whh_xh_ratio` zero-W_hh case | Hand-zero W_hh, run forward, assert ratio == 0.0 |
| `whh_xh_ratio` identity-W_hh case | Hand-set W_hh = I, assert ratio in expected order-of-magnitude |
| `whh_xh_ratio` no-recurrent sentinel | Build with `NN_RECURRENT[2]=false`, assert ratio == 0.0 |
| `w_*_cv` identical-population | All individuals identical, assert cv == 0.0 for all blocks |
| `w_*_cv` bimodal | Half +1.0, half −1.0, assert cv matches analytic prediction |
| `w_hh_cv` no-recurrent sentinel | `NN_RECURRENT[2]=false`, assert nan emitted (or 0.0; pick in tasks) |

Tests live under the existing GoogleTest infrastructure; CMake target gets one new entry
in `tests/CMakeLists.txt`.

## Consumer test surface

The plot script's parser is exercised by:
- A canned `#NNGen`-line fixture for the 027 (no-028-fields) case → script must render
  the 5 existing panels and skip the 6th (or render it empty).
- A canned 028-line fixture → script renders all 6 panels, including the threshold line on
  panel 6.

Plot tests are pytest-style under `specs/028-deeper-rnn/` (or wherever 027's plot tests
live; check in tasks).

## Open contract decisions (resolve in tasks)

1. `w_hh_cv` sentinel value: `nan` vs `0.0`. Plot script behavior differs.
2. `whh_xh_ratio` aggregation across recurrent neurons: mean (default) or max.
3. Capture cadence: every gen (default) vs every Nth gen.

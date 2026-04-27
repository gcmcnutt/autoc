# 028 — Deeper RNN (carry-forward of 027)

**Status**: SEED — 2026-04-26. Inherits 027's open work. Needs
clarify pass + plan before implementation.

**Origin**: [027 — Recurrent NN](../027-recurrent-nn/spec.md) closed
2026-04-26 with infrastructure landed but primary bet not validated.

**Required reading before working 028**:

- [`specs/027-recurrent-nn/findings.md`](../027-recurrent-nn/findings.md)
  — what was built, what happened in rnn1/2/3, the four
  not-yet-disambiguated failure modes, and why the cadence7-redux
  diagnostic rules out an infrastructure bug.
- [`specs/028-deeper-rnn/research.md`](./research.md) — literature
  priors (copied verbatim from 027; unchanged).
- [`flight-results/flight-20260426/FLIGHT_REPORT.md`](../../flight-results/flight-20260426/FLIGHT_REPORT.md)
  — flight outcome on the cadence7-redux build.

028 inherits 027's findings as **starting facts**, not open
questions. The next experiments are scoped to disambiguate the
four candidate failure modes from findings.md, not to re-discover
that they exist.

## Carry-forward inventory

### Code already in tree (027 commits — disabled but ready)

Five `CADENCE7-REDUX` markers point to the restore points — flip
them to re-enable D-simple and the multi-axis lexicase pool:

| File | Current (diagnostic) | Restore for 028 |
|---|---|---|
| [`include/autoc/nn/topology.h:52-57`](../../include/autoc/nn/topology.h) | `NN_RECURRENT[2] = false` | `true` |
| [`include/autoc/nn/topology.h:69`](../../include/autoc/nn/topology.h) | `static_assert == 1667` | `== 1923` |
| [`src/eval/selection.cc:66-69`](../../src/eval/selection.cc) | stability + energy `pool.push_back` commented | uncomment |
| [`tests/contract_evaluator_tests.cc:14`](../../tests/contract_evaluator_tests.cc) | expects 1667 / FF | expects 1923 / `[2]=true` |
| [`tests/selection_tests.cc:152+`](../../tests/selection_tests.cc) | 4 `Selection027*` `DISABLED_` | rename back |

Already implemented and unit-tested:
- `NNControllerBackend` recurrent forward pass + `reset()` (T020–T021)
- `nn_xavier_init` for W_hh recurrent block (T022)
- Serialization of recurrent-flag array (T023)
- Three-axis `ScenarioScore` (`score`, `stability_score`,
  `energy_score`) with `gp_fitness` typing
- 3-axis lexicase pool with per-axis epsilon floor 0.5
- `nn2cpp` emits recurrent C code; `minisim` construct-once-per-span;
  crrcsim `nnController_` member with span-start `reset()`

### Required prework (before any retraining)

The four candidate failure modes are documented in
[`027/findings.md`](../027-recurrent-nn/findings.md) §"Plausible
failure modes":

1. D-simple architecture too expensive for evolutionary search at
   this budget.
2. C2 axes equal-weighted in lexicase suppresses tracking
   convergence.
3. C2 axis numeric ranges mis-scaled vs the ε-lexicase floor.
4. Hidden-state init/reset interacts badly with random Xavier
   outputs.

These aren't to be rediscovered — 027's stcs, logs, and
[`evolution PNGs`](../027-recurrent-nn) are the evidence base. 028
disambiguates with one cheap experiment first:

**D-alone diagnostic** — recurrent ON (`NN_RECURRENT[2]=true`),
lexicase pool tracking-only (stability + energy `pool.push_back`
calls stay commented). One 400-gen run, same pop/scenarios as
cadence7-redux, fresh seed.

| D-alone outcome | Implication | Drives 028 toward |
|---|---|---|
| Descends to ≤ −30000 | Hypothesis 1 wrong; hypotheses 2/3 likely right | Re-design C2 (formulation, weighting, ε-scaling) |
| Stalls similar to rnn1/2/3 | Hypothesis 1 likely right (architecture or budget) | Bigger pop / longer training, OR 32-wide variant, OR rethink architecture |
| Descends partway then stalls | Hypothesis 4 plausible (state-init effects late in run) | Investigate hidden-state init schemes (orthogonal init, zero-init at gen 0 only, layer-norm) |

This is **the first sim experiment in 028**, regardless of clarify
outcome — it's information-cheap and bounds what the rest of the
plan even needs to cover.

### Open questions for clarify

1. **Topology size** — 16-wide layer 2 (1923 weights) confirmed
   from 027 clarify Q3, but rnn1/2/3 stalled. Is the answer
   "bigger" (32-wide, +1024 = 2691) or "different recurrent
   placement" (layer 1 instead of layer 2)? Plan 027 reserved
   32-wide as escalation.
2. **C2 formulation** — three formulations tried (smoothness =
   Σ\|Δout\|, energy = Σ(out_th−1)/2, stability =
   Σ(\|out_pt\|−1)+(\|out_rl\|−1)); none paired well with D-simple.
   Is the formulation wrong, the *weighting* wrong (lexicase
   pushes equal-pressure on each axis), or both?
3. **Selection regime** — stay on lexicase, or revisit Pareto /
   NSGA-II (027 spec's "out of scope") given that lexicase's
   equal-pressure assumption might be the failure mode?
4. **Population / training budget** — pop 3500 × 400 gens at 1923
   weights might just be undersized. cadence7's 1667-weight
   trajectory might not extrapolate.
5. **Xiao port timing** — 027 deferred behind sim gate; same
   deferral applies to 028. When 028 finds a winning combo, the
   xiao-side recurrent forward pass needs to be wired into
   `xiao/src/generated/nn_program_generated.cpp`.

### Out of scope for 028

- **Flight hardware changes.** xiao stays on INAV MANUAL until 028's
  sim gate clears.
- **Craft variations** ([025](../025-craft-variations/spec.md)). Stays
  blocked on 028 outcome — same chain as 027.
- **D-ESN / GRU-lite reservoir architectures.** 027's research bias
  still applies; D-simple gets one more honest shot before reaching
  for those.

## Validation gate (carries from 027 plan)

Same triple-bar quantitative gate, same rationale:

| Metric | cadence7 baseline | pid1 (NO-GO ref) | 028 target | Interpretation |
|---|---:|---:|---:|---|
| `<\|Δout\|>` / tick (dCtrl, late plateau) | ~1.00 | 1.60 | **≤ 0.80** | ≥ 20 % reduction in stick speed vs cadence7 |
| `<\|out\|>` / tick (amplitude, late plateau) | ~2.20 | 2.54 | **≤ 2.00** | ≥ 10 % reduction in saturation |
| Best fitness @ matched gen | −35951 (gen 400) | −27045 | **≥ −30000** | Within ~15 % of cadence7 |

All three must hit. Histogram (output distribution spread toward
0) is qualitative confirmation, not a gate.

## Relationship to 027 / 025

- **027 closed as iteration.** Infrastructure carries forward; the
  design questions carry forward; the rnn1/2/3 logs carry forward
  as the prework dataset for the failure-mode diagnostic.
- **025 (craft variations) stays blocked on 028.** If 028 finds a
  non-bang-bang controller, 025 is the natural follow-on for
  robustness against airframe variation.
- **Renderer scrub-with-state** (027 plan open decision #5) stays
  parked.

## Next steps

1. `/speckit.clarify` — work the 5 questions above.
2. Run the **#4 D-alone diagnostic** as the first sim experiment
   regardless of clarify outcome — it disambiguates architecture
   vs incentive cleanly and cheaply (one 400-gen run).
3. `/speckit.plan` after clarify lands.

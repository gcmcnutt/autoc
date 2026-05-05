# more-rnn1 — D-alone diagnostic outcome (gen 400, 2026-04-26)

**Run**: more-rnn1, single seed, recurrent ON (`NN_RECURRENT[2]=true`),
lexicase OFF (tracking-only), 1923 weights, pop 3500 × 400 gens.
Log: `logs/autoc-028-more-rnn1.log`. Data: `data.stc`, `data.dat`.

## TL;DR

| Verdict | Status |
|---|---|
| **Existence test** (does RNN work at this setup?) | **PASS** — fitness slope healthy throughout, telemetry showed engaged W_hh + searched W_hh, vastly outperformed all three 027 RNN runs |
| **Fitness gate** (≤ −30000) | **PASS** (just) — final best −30626 |
| **Smoothness gate dCtrl ≤ 0.80** | **FAIL** — final 1.94 (2.4× over) |
| **Smoothness gate `<\|out\|>` ≤ 2.00** | **FAIL** — final 2.34 (17 % over) |
| **Branch routing** | spec §Phase 5a (stability-only single-axis lexicase) — but see "next attempt" below |

## Final fitness vs comparators

| Run | Topology | Final best fitness | Notes |
|---|---|---:|---|
| more-rnn1 (this run) | D-simple recurrent 16-wide | **−30626 @ gen 400** | gate cleared, slope still active |
| cadence7-redux | FF, 1667 weights | −33037 @ gen 400 | spec gate 028 baseline |
| pid1 (NO-GO ref) | FF + slew limiter | −27045 | spec NO-GO ref |
| 027 rnn1 | D-simple recurrent 16-wide | −8437 @ gen 312 (aborted) | 027 stalled |
| 027 rnn2 | D-simple recurrent + v3 energy | −7152 @ gen 400 | 027 stalled |
| 027 rnn3 | D-simple recurrent + v4 stability + v3 energy | −12626 @ gen 237 (aborted) | 027 stalled |

more-rnn1 is **2.4× more negative** than the best 027 RNN run (rnn3) and
93 % of cadence7-redux's level. Slope was still descending at gen 400
(gen 384: −29544 → gen 398: −30626) — more compute would push further.

## Telemetry — architecture-level failure modes ruled out

| Signal | gen 1 | gen 100 | gen 200 | gen 300 | gen 400 | Interpretation |
|---|---:|---:|---:|---:|---:|---|
| `whh_xh_ratio` | 0.48 | ~0.6 | ~0.7 | ~0.6 | 0.73 | recurrent block engaged throughout — **H1 ruled out**, **H4 ruled out** |
| `w_hh_cv` | 0.05 | 0.17 | 0.19 | 0.18 | 0.16 | population searched W_hh in lockstep with W_xh blocks — **H1 ruled out** |
| `w_xh0_cv` / `w_xh1_cv` | 0.02 / 0.03 | 0.15 / 0.15 | 0.18 / 0.17 | 0.18 / 0.17 | 0.16 / 0.16 | feedforward blocks searched at parity |

The two telemetry signals delivered as 028 prework did exactly what they
were designed to do: **architecture/search failure modes (H1, H4) are
ruled out by direct measurement**. If 028 had stalled, we'd have known
*why*. It didn't stall — slope was active, telemetry healthy.

## Smoothness — per-axis breakdown (last 50 gens, paths 0-4 wind 00)

| Axis | `<\|Δout\|>` | `<\|out\|>` | per-axis budget | Read |
|---|---:|---:|---:|---|
| pitch | 0.58 | 0.66 | ~0.27 / ~0.67 | mostly within budget — **pitch is smooth** |
| **roll** | **1.08** | 0.80 | ~0.27 / ~0.67 | 4× over per-axis dCtrl budget — **roll is bang-bang** |
| throttle | 0.36 | 0.88 | ~0.27 / ~0.67 | within budget on dCtrl, slight over on `<\|out\|>` |

**Surprise finding**: bang-bang axis flipped vs cadence7-redux. The
04-26 flight on cadence7-redux's `gen9600.dmp` had pitch porpoise + roll
smooth. more-rnn1 has pitch smooth + roll bang-bang. Same airframe, same
scenarios, **different controller, different dominant axis**.

Implication for pattern 2: the stability axis formulation should remain
**027 v4 pt+rl together** rather than the pitch-only refinement that
flight observation initially suggested. Single-axis pitch-only stability
would miss more-rnn1's failure mode entirely.

Implication for the broader research: if the bang-bang axis is
controller-specific, future runs may shift it again. Worth watching
across the next few attempts whether the dominant axis migrates — that's
direct evidence for whether single-axis lexicase suffices, or whether a
multi-axis (or even Pareto / NSGA-II) approach is needed.

## Phase 4 routing — original spec mapping vs revised plan

**Original spec mapping** ([spec §Required prework](./spec.md#required-prework)):
"descends to ≤ −30000 but bang-bang persists" → spec §Phase 5a, pattern 2
stability-only single-axis lexicase.

**Revised plan** (per [tuning_notes.md](./tuning_notes.md) and operator
direction 2026-04-26):

> Slope was still descending at gen 400, fitness barely cleared the
> gate, smoothness clearly failed. Before adding C2 incentive complexity,
> try a **tuned D-alone retry (more-rnn2)** — same architecture, more
> compute, better GA dynamics — to see if smoothness emerges organically
> with more time and softer crossover. Only if more-rnn2 still misses
> smoothness do we route to pattern 2.

This is structurally a "Phase 4.5" insertion before Phase 5a — same
envelope (one of the ≤ 2 follow-on patterns from spec §Q3), different
lever (compute/GA vs incentive). See `tasks.md` for the more-rnn2 task
list.

## Artifacts

- [`more-rnn1_evolution.png`](./more-rnn1_evolution.png) — 6-panel evolution plot (2-column layout)
- [`more-rnn1_aggressiveness.png`](./more-rnn1_aggressiveness.png) — control aggressiveness across gens
- `data.stc` — per-gen `#NNGen` log (snapshot of run-end state will land in archive)
- `data.dat` — per-tick eval data for all 400 gens
- `logs/autoc-028-more-rnn1.log` — training log

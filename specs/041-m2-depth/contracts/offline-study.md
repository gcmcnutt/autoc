# Contract — offline studies (regime/load, and predictor feasibility)

Two studies, both running on **already-recorded data**, both producing a verdict **before** compute is
committed. Neither requires a training run, a recording change, or a bake.

**New**: `src/analytics/regime_load_study.py` (maintained analytics home — the dmp-fed carve-out from the
historical-scripts-immutable rule) plus physics columns in `tools/dmp_dump.cc`. Feature-local one-offs stay
under `specs/041-m2-depth/` ([[feedback_scripts_dir_scope]]).

---

## Study A — regime and load (gates the aggressiveness design)

### Inputs

Per-tick data from **pinned S3 dmps** — the pinned M1 source and the 040-t4 M2 comparator, both verified
retained 2026-08-07.

⚠️ **Not `data.dat`** — verified gone from the repo root, and overwritten by each new run anyway. The
analysis path must not depend on it (research.md R9).

`dmp-dump` already emits `out_pt,out_rl,out_th,stpPt,dist,mult` per tick for both modes. This feature adds
**physics columns** from the existing, already-serialized `PhysicsTraceEntry`: `acc[3]`, `omegaDotBody[3]`,
`alpha`, `vRelWind`. That trace is populated for every elite reeval and currently has **no consumer at all**
— load has been recorded all along and never read.

### Required outputs

1. **Regime classification per tick** — `{tracking, intercept, patrol}` using the established rule:
   `stpPt ≥ 0.5` is tracking; below that, smoothed `d(dist)/dt < 0` is intercept, else patrol. Use the
   *existing* definition (`dynamics_progress.py:74-80`) rather than a new one, so the numbers are
   comparable with every prior report.
2. **Load per regime, per axis** — body-frame normal acceleration derived as
   `R(quat)ᵀ·(a_world − g_world)`; report distribution and **peak**. Peak is the damage-relevant statistic;
   a mean hides the ±11 g excursions entirely.
3. **Control statistics per regime** — `dCtrl` and `⟨|u|⟩` per axis, reported **per regime, never pooled**.
   Pooling is what made "aggressiveness" look like one phenomenon.
4. **H2 test** — within each regime, does pitch/roll `dCtrl` predict throttle level and load? Report
   correlation with a stated confidence, not a scatter plot alone.
5. **Per-regime intent profile, for BOTH the prior and the new M1** — the comparison that answers whether
   the new controller behaves *differently by regime* where the old one did not (FR-011c). Same statistics,
   same code path, one row per (run, regime).

   ⚠️ **This is the only way to get the prior M1's side.** Its genome has 37 inputs and cannot be loaded by
   a 041 binary, so it can never be re-evaluated or ablated — only profiled from its recorded dmp. Plan for
   this rather than discovering it: it is the "baseline weights expire" trap, and profiling-from-recording
   is the mitigation.

   Because this is a cross-run profile comparison and **not** a controlled delta, report it as a ballpark
   read — the same standing this project gave 040's "no significant regression" conclusion. Do not present
   it as an attributable effect size.

### Decisions this study gates

| finding | consequence |
|---|---|
| Load is concentrated on pitch, in intercept | build the load axis, pitch-first, limit-based (research.md R4) |
| Load is uniform across regimes | a regime-conditional objective is unjustified; reconsider the axis entirely |
| Pitch `dCtrl` strongly predicts throttle and load (H2 holds) | penalize the cause; **no throttle axis** |
| No such relationship (H2 refuted) | throttle is its own phenomenon — record the refutation and reconsider |
| Peak load already inside a sane structural limit | **no load axis at all** — a limit nobody exceeds is not an objective |

The last row matters: this study can remove work from the feature, which is why it precedes the bundle
closing.

---

## Study B — predictor feasibility (gates the M2 design)

### Inputs

Recorded tracker runs (040-t4). Extract **visible → blind → reacquire** triples: the perceptual history
before a visibility gap, the gap duration, and the geometry at re-entry.

### Targets tested, each against an explicit no-information baseline

| target | baseline | horizon |
|---|---|---|
| **(a) current target bearing, continuously** *(the one that matters)* | **hold-last-seen** (dead-reckon at zero rate) | **none — horizon-free**, it is a state estimate |
| (b) Δspan at 50/100/150 ms | persistence | actuation-scale — **control only**, to confirm *why* the old head failed |
| (c) discounted future step points | constant mean | value-head fallback |

### ⚠️ Target (a) MUST be reported conditioned on blind-gap age

A single pooled r² on (a) is **misleading by construction**: on visible ticks the truth is an input, so both
the head and the baseline are near-perfect and the metric is dominated by easy samples. The informative
regime is blindness.

Report a curve, not a number: error and r² **binned by seconds since last truth** (e.g. 0, 0–0.25, 0.25–0.5,
0.5–1, 1–2, 2–4, 4+ s), with sample counts per bin. The claim "the head beats hold-last-seen" is only
meaningful per bin.

**Which bins qualify is decided by the data, not by this document** (FR-024a). Compute the blind-gap
distribution **first**, identify the bins holding the bulk of real excursion duration, and require the win
*there*. Do not assume the long bins matter most: narrowing the field of view — the payoff this whole thread
is aimed at — makes excursions **more frequent and likely shorter**, so the decisive bins may be sub-second
even though worst-case windows reach ~8 s. An arbitrary ≥1 s threshold would set the bar in the wrong place.

### Binding scoring rule

**Report variance explained (r²) against the stated baseline. Never mean absolute error against the
quantity's level.**

This is the lesson that produced the whole re-target: the old metric conflated offset/scale error with
information content. Span moves ~0.0075 rad per 150 ms against a ~0.049 rad level, so **persistence was
already right to within 15%** — a perfect head added nothing, and measured `r(Δspan)` was ≈0 at every
horizon with best error at *generation 1*.

⚠️ **Target (b) is confounded until the prediction-pairing fix lands** (FR-004). Run it after.

### Free by-product — required output

The **blind-gap distribution**: frequency, duration histogram, and exit→re-entry bearing offset. Nobody has
measured this, and it is the actual input to any future field-of-view decision. Record it whatever the
predictor verdict is.

### Decisions this study gates

| finding | consequence |
|---|---|
| (a) beats persistence | build the re-targeted head: gap-scale horizons, scored **across** the gap rather than visibility-gated, output scaled into the target domain |
| (a) fails, (c) beats constant-mean | fall back to the value head |
| all fail | **retire the head** — topology output 7→3, reclaiming 119 output weights and a third of the lexicase pool. An accepted outcome (FR-027) |

---

## Conventions for both studies

- CLI flags, not env vars; config file flag is `-i`.
- Deterministic: same dmp + same flags ⇒ same numbers.
- Emit machine-readable output (CSV) alongside any plot, so a later feature can re-derive without re-running.
- State sample sizes and any excluded ticks. A study that silently drops data reads as coverage it does not
  have.
- Absolute fitness sums are **not** comparable across runs with different scenario counts — use
  per-scenario or per-step rates.

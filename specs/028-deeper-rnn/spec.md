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

## Clarifications

### Session 2026-04-26

- Q: If D-alone descends to fitness ≤ −30000, which C2 redesign
  lever (weighting vs ε-floor vs reformulation vs alternative
  algorithm) does 028 tune first? → A: None unconditionally. The
  D-alone run is judged against the full triple-bar gate, not just
  the fitness threshold. If recurrence alone also delivers
  smoothness (dCtrl ≤ 0.80, ⟨\|out\|⟩ ≤ 2.00), 028 checkpoints and
  proceeds to flight test — no C2 redesign needed. C2 lexicase
  tuning is **contingent** on D-alone descending in fitness *but*
  failing the smoothness gates (bang-bang persisting). Lever
  ordering stays deferred until D-alone outcome is in hand.
- Q: If D-alone stalls (failure mode 1), what's the first
  architectural escalation? → A *(revised post-clarify by plan-phase
  research, [`research_rlayer_placement.md`](./research_rlayer_placement.md)
  §5)*: **Final escalation order** —
  (1) **orthogonal W_hh init** at 16-wide layer 2 (addresses
  failure mode 4 directly; cheap, ~1 file change; strong tanh-RNN
  literature support);
  (2) **larger search budget** at 16-wide layer 2 (pop ↑ 3500→~5000
  or gens ↑ 400→~600; no code change);
  (3) **topology change** (24-wide intermediate, then 32-wide layer
  2, then layer-1 placement) — only if budget remains within the
  3-attempt envelope.
  Original clarify answer put larger budget first; plan-phase
  research found stronger evidence for orthogonal init as a
  pre-topology lever. To direct subsequent attempts, 028 instruments
  three new signals before the next escalation:

  1. **W_hh / W_xh activation ratio** (per-generation, best
     individual) — measures whether the recurrent block is being
     used. Hook: recurrent forward pass in
     [`src/nn/evaluator.cc:198-211`](../../src/nn/evaluator.cc).
     Cost low. Renders as a 6th panel in
     [`plot_evolution_progress.py`](../027-recurrent-nn/plot_evolution_progress.py).
     Discriminates H1/H4 (block dead) from H2/H3 (block active but
     incentive wrong).
  2. **W_hh population-level diversity (CV)** vs W_xh CV
     (per-generation). Hook: `logGenerationStats(gen)` in
     [`src/autoc.cc:1206`](../../src/autoc.cc). Cost low. Side log
     column or 6th-panel overlay. Discriminates H1 (GA isn't
     searching W_hh) from H4 (searches it but state structure
     locks down too early).
  3. **Within-span h_t saturation / gradient** — post-run replay
     tool over best individual from final gen. Cost medium; deferred
     until signals 1–2 narrow the mode. Discriminates H4 directly.

  Signals 1 and 2 are 028 deliverables before the next post-D-alone
  experiment runs.
- Q: When does 028 close? → A: Bounded effort, not open-ended.
  028's experimental envelope is **D-alone + at most 2 follow-on
  patterns**, each within a reasonable compute envelope (e.g.,
  pop ≤ ~5000, gens ≤ ~600 — *not* 1M-pop or 4000-gen runs; if
  reaching the gate would require that scale, 028 is the wrong
  feature). Two terminal conditions:

  1. **Win**: any pattern hits the triple-bar sim gate and a
     subsequent flight on that architecture is successful → 028
     closes, ships winner.
  2. **Bounded no-go**: all three attempts (D-alone + 2 follow-ons)
     exhausted within budget without clearing the gate → 028 closes
     with a findings.md handoff to 029, telemetry signals from this
     run included as carry-forward evidence.

  Working assumption: D-alone reduces bang-bang on its own; the
  most likely follow-on pattern, if a second one is needed, is
  re-introducing a single C2 axis (see Q4) rather than escalating
  topology.
- Q: If D-alone descends but smoothness misses, which single C2
  axis comes back as pattern 2? → A: Stability-only (027 v4 =
  Σ(\|out_pt\|−1) + (\|out_rl\|−1)). Rationale: the in-flight
  bang-bang signature is pitch/roll, not throttle. The 04-26
  flight's full-throttle behavior is a **scenario-physics
  artifact**, not a controller-smoothness problem — the craft falls
  behind the rabbit on short courses, saturates throttle to catch
  up, overshoots, then perennially over-corrects. Even a perfectly
  smooth controller would saturate throttle in that regime, so
  penalizing throttle smoothness (energy axis) would push against
  scenario coupling rather than against controller jitter. Energy
  axis stays available as a fallback only if pitch/roll smoothness
  is in spec but throttle bang-bang persists *and* scenario design
  has been ruled out as the cause.

  **Adjacent observation (out of 028 scope, captured for 029
  carry-forward):** the throttle-saturation pattern suggests
  scenario-design changes are likely needed before any throttle
  smoothness objective can be evaluated fairly. Candidates: more
  faithful momentum/inertia in the FDM so catch-up dynamics are
  realistic; rabbit-speed profile or tracking-cone reshaping so
  short-course catch-up isn't pathological; a "catch up smoothly"
  objective that rewards graceful overshoot recovery rather than
  raw throttle delta.
- Q: Seed replication policy per attempt? → A: 1 seed per attempt
  (status quo from 027). Within-run population statistics at the
  time of a sim-gate pass (best individual + neighborhood of fit
  individuals) are taken as the robustness signal — a passing
  attempt already covers thousands of evaluations of the winning
  genotype family, so a separate seed-replication run isn't
  required before flight. Trade-off: a seed-lucky pass could ship.
  Mitigation: if a candidate winner's population late-gen fitness
  *spread* (not just best) looks degenerate, treat as suspect and
  pull a second seed before flight.

## Carry-forward inventory

### Code already in tree (027 commits — disabled but ready)

Five `CADENCE7-REDUX` markers point to the restore points — flip
them to re-enable D-simple and the multi-axis lexicase pool:

| File | Current (diagnostic) | Restore for 028 |
|---|---|---|
| [`include/autoc/nn/topology.h:52-57`](../../include/autoc/nn/topology.h) | `NN_RECURRENT[2] = false` | `true` |
| [`include/autoc/nn/topology.h:69`](../../include/autoc/nn/topology.h) | `static_assert == 1667` | `== 1923` |
| [`src/eval/selection.cc:66-69`](../../src/eval/selection.cc) | stability + energy `pool.push_back` commented | **stability**: uncomment for pattern 2 5a only; **energy**: stays commented (out of 028 scope per Q4) |
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
| Descends to ≤ −30000 **and** hits smoothness gates (dCtrl ≤ 0.80, ⟨\|out\|⟩ ≤ 2.00) | Recurrence alone delivered smooth flight; C2 not needed | **Checkpoint + flight test** — no C2 redesign |
| Descends to ≤ −30000 but bang-bang persists (smoothness gates miss) | Hypothesis 1 wrong; hypotheses 2/3 likely right | Re-design C2 (formulation, weighting, ε-scaling) — lever order TBD |
| Stalls similar to rnn1/2/3 | Hypothesis 1 likely right (architecture or budget) | Bigger pop / longer training, OR 32-wide variant, OR rethink architecture |
| Descends partway then stalls | Hypothesis 4 plausible (state-init effects late in run) | Investigate hidden-state init schemes (orthogonal init, zero-init at gen 0 only, layer-norm) |

This is **the first sim experiment in 028**, regardless of clarify
outcome — it's information-cheap and bounds what the rest of the
plan even needs to cover.

### Resolved pre-clarify questions

The five open questions originally seeded here (topology size, C2
formulation, selection regime, training budget, xiao port timing)
were resolved in the [`## Clarifications`](#clarifications) session
above:

- Topology / budget: bounded effort (≤ ~5000 pop × ~600 gens,
  D-alone + ≤2 follow-on patterns); larger budget at 16-wide first,
  32-wide / layer-1 / state-init alternatives are telemetry-gated.
- C2 formulation: deferred unless D-alone descends but smoothness
  misses; pattern 2 then = stability-only single-axis lexicase.
- Selection regime: stays on lexicase (single-axis pattern 2
  doesn't engage the equal-pressure failure mode).
- Xiao port timing: deferred behind 028 sim gate, then flight gate.
  When 028 ships a winner, the xiao-side recurrent forward pass
  must be wired into
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

**Gen at which the gate is checked**: matched-compute by default —
gen 400, equal to cadence7 baseline. For larger-budget attempts
(plan §Phase 5c, pop ↑ to ~5000 or gens ↑ to ~600), the gate is
checked at **run-end**, not gen 400. Total compute is the
comparison axis: if a 600-gen run hits ≥ −30000 by gen 600, that
counts; if it doesn't beat cadence7's 400-gen run on equal
wall-clock, the cost of memory wasn't paid back.

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

1. ~~`/speckit.clarify`~~ — done 2026-04-26.
2. `/speckit.plan` — produce the implementation plan from the
   resolved clarifications. Plan must cover: telemetry signals 1–2
   (W_hh/W_xh ratio, W_hh population CV) as 028 deliverables before
   any post-D-alone retraining; the D-alone diagnostic run; the
   3-attempt experimental envelope and its terminal conditions.
3. Run the **D-alone diagnostic** as the first sim experiment —
   recurrent ON, lexicase OFF, single-seed — to disambiguate
   architecture vs incentive cheaply.

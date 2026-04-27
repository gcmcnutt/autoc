# 028 — Tuning notes (D-alone reframed as existence test)

**Status**: scratch notes during the more-rnn1 run. Created 2026-04-26 mid-run
(gen ~121 of 400). Captures the question reframe and a tuning-lever inventory
for follow-on attempts.

## Reframed question

The D-alone diagnostic was originally scoped to answer the full triple-bar gate
(fitness ≥ −30000 + dCtrl ≤ 0.80 + ⟨|out|⟩ ≤ 2.00). At gen 121 the visual
read is **throttle smooth, pitch/roll bang-bang** — which means the
smoothness question on the pt/rl axis is *already answered* (D-alone alone does
not produce smooth pt/rl) and pattern 2 5a (stability-only lexicase) is the
mapped next step **if and only if** fitness clears ≥ −30000.

Reframe to a more honest first-question: **does the recurrent setup (D-simple
at 16-wide layer 2) work at all here?** Specifically:

1. Does fitness descend to a meaningful level (≥ −30000 nominal target, but
   "comparable to or somewhat better than cadence7-redux's −33037" is the
   real comparator).
2. Is the recurrent block doing useful work (the existing telemetry signals
   are saying yes — `whh_xh_ratio` has stayed 0.5–0.95 throughout).
3. Is RNN somewhat better than FF on the *fitness axis* alone?

Smoothness is now a **secondary** (route-to-pattern-2) signal, not a co-equal
gate.

## Budget implication — 400 gens may not be enough

- More-rnn1 at gen 121: best = −13257.
- Cadence7-redux at gen ~121 was around −22000 to −25000 (extrapolating).
- More-rnn1 slope from gen 88→121 is ≈ −65/gen. If it holds, gen 400 lands
  near −31000. If late-run taper kicks in (typical), gen 400 lands near
  −20000 to −25000.
- To get a clean "matches cadence7" or "exceeds cadence7" call, **800 gens**
  may be needed for a confident answer.

This stretches the spec envelope (currently capped ≤ 600 gens per attempt
in [spec §Q3](./spec.md#clarifications)). If we commit to 800, update the
spec accordingly — bounded effort still applies, the 800-gen run is the
attempt itself, just longer.

## Tuning levers — current values + candidate adjustments

Read the autoc.ini values straight from the running config:

| Knob | Current | Candidate | Rationale |
|---|---:|---:|---|
| `PopulationSize` | 3500 | 5000 (envelope cap) or 7000 (extend) | High-D recurrent benefits from more diversity. Heuristic floor for 1923 weights ~440; we're already well above, but doubling helps W_hh exploration when block engagement is structurally novel |
| `NumberOfGenerations` | 400 | **800** | Reframe target. Diminishing returns last 200 gens, but recurrent slope ramps later than FF in most NEAT-RNN literature |
| `TournamentSize` | 5 | 3 | Lower selection pressure → more diverse parents → better W_hh exploration. Empirically helps high-D NN evolution. Cost: slower convergence |
| `NNMutationSigma` (init) | 0.2 | 0.3 | Bigger initial step → more aggressive exploration of W_hh basin (currently flat in early gens). Self-adaptation will narrow it anyway |
| `NNSigmaFloor` | 0.05 | 0.03 | Lower floor → finer late-stage refinement once basin is found. Cadence7 hit 0.05 floor early; recurrent may need more refinement room |
| `NNCrossoverAlpha` | −1 (uniform) | 0.3 (BLX-α) | BLX-α with positive alpha generates offspring within parent envelope + alpha extension — more local than uniform-per-weight. Recurrent search benefits from local moves; uniform-per-weight crossover is too disruptive across W_hh |
| `CrossoverProbability` | 95 | 80 | More mutation-only offspring; mutation respects W_hh structure better than uniform crossover does |
| `SwapMutationProbability` | 15 | 25 | Co-balance with the crossover drop above |
| `AddBestToNewPopulation` | 1 | 3 | Slightly stronger elitism to protect the recurrent policy once it emerges. Tradeoff: more drift resistance vs less turnover. Currently elite IS being re-evaluated each gen against fresh variations (verified gen 121 fitness regression vs gen 119) — so `=3` doesn't lock in stale individuals |
| `VariationRampStep` | 40 | 60 | Slower variation ramp → more stable selection pressure during the recurrent block's emergence. Sharp variation jumps in early gens may mask which policy structures are learning |
| `Seed` | −1 (time) | fixed (e.g., 42) | For reproducibility on follow-on attempts. Keep −1 if you want seed-luck distribution awareness; pin if comparing two configs back-to-back |

## Recurrent-specific knobs (not in autoc.ini, would require code changes)

Out-of-envelope until first 800-gen run resolves, but worth listing:

- **Orthogonal init for W_hh** (per [research_rlayer_placement.md §4](./research_rlayer_placement.md)). Already in pattern 2 5b. If 800-gen stalls, this is the next escalation.
- **Block-aware crossover**: respect layer/block boundaries when crossing parents. Specifically, never bisect a W_hh row mid-row. `nn_crossover` in `src/nn/population.cc` is currently flat-vector. ~30 LOC change.
- **Block-aware mutation rate**: separate sigma for W_hh vs W_xh blocks. Reasonable prior: scale W_hh mutation 0.5×–0.7× of W_xh once `whh_xh_ratio > 0.5` (block engaged). Encourages refinement of the recurrent dynamics rather than disrupting them.
- **Spectral radius preservation** in mutation: clip W_hh mutations that push spectral radius outside [0.7, 1.05]. Prevents recurrent layer from collapsing to dead or exploding to chaotic during search.

## Recommended priority order if first 800-gen run isn't enough

If gen 800 best fitness > −25000 (clearly stalled below cadence7 class):
1. **TournamentSize 5 → 3** (cheap config flip, biggest expected GA-search-efficiency gain at high-D)
2. **NNCrossoverAlpha −1 → 0.3** (BLX-α; one-line config, reduces crossover destruction)
3. **PopulationSize 3500 → 5000** (compute uplift, but most direct lever for high-D coverage)
4. Then escalate to pattern 2 5b orthogonal init (code change)
5. Block-aware crossover/mutation (medium code change, only if 1–4 don't move the needle)

Knobs 1+2 together is a **single-config-file-edit experiment** — cheapest possible follow-on after extending to 800 gens. Worth running before any architecture change.

## What stays unchanged

- `NN_RECURRENT[2] = true` — D-alone topology stays as-is for the existence test
- Lexicase OFF — still tracking-only; smoothness is route-to-pattern-2 signal, not a co-equal gate
- Eval scenarios + variations unchanged — same problem definition

## Pattern 2 refinement: pitch-only stability (flight evidence)

The 04-26 flight on cadence7-redux's `gen9600.dmp` produced an axis split:
**roll smooth, pitch porpoise**. This refines the [spec §Q4](./spec.md#clarifications)
pattern 2 design. Locked answer was "stability-only single-axis lexicase"
using 027's v4 formulation:

```
stability_score = Σ (|out_pt| − 1) + (|out_rl| − 1)    # 027 v4 — penalizes both axes
```

With the flight evidence, the right formulation is **pitch-only**:

```
stability_score = Σ (|out_pt| − 1)                     # 028 pattern 2 — pitch only
```

Why: roll on this airframe is already acceptable in flight without
smoothness pressure. Penalizing roll smoothness pushes the policy against a
non-problem and may compete for fitness budget that should be spent on the
real issue (pitch). Pitch porpoise IS the controller-driven oscillation that
the smoothness objective is for.

If/when pattern 2 5a actually runs:
- Flip CADENCE7-REDUX marker 3 in `src/eval/selection.cc:66-69` (uncomment
  ONLY the stability push)
- Modify `stability_score` formula in `src/eval/fitness_decomposition.cc`
  (or wherever stability_score is computed) to drop the roll term
- ~3-line code change beyond the marker flip

Don't update spec §Q4 text yet — only commit when pattern 2 actually runs.

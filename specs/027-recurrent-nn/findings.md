# 027 findings — infrastructure landed, primary bet not validated

**Status**: CLOSED 2026-04-26. 027's hypothesis (D-simple recurrent +
C2-via-lexicase) is **not disproven** — it's **untested**: three
training experiments with the architecture all stalled on the
*tracking* axis before the architectural value of memory could be
isolated. The diagnostic build (FF + tracking-only, "cadence7-redux")
recovered cadence7-class fitness, ruling out infrastructure bugs.
The 04-26 flight ran the diagnostic, not the architectural change.

See [spec.md](./spec.md) for original scope, [tasks.md](./tasks.md)
for retrospective task status, and
[FLIGHT_REPORT.md](../../flight-results/flight-20260426/FLIGHT_REPORT.md)
for the flight outcome. Carry-forward lives in
[028 — deeper-rnn](../028-deeper-rnn/spec.md).

## Hypothesis (from spec.md)

> Bang-bang is a consequence of evolutionary pressure on a
> **stateless feedforward** NN. To exit the bang-bang plateau, the
> NN itself must gain the ability to represent and reason about its
> own history (D-simple recurrent layer), and the selection process
> must apply pressure against jitter (C2 via lexicase smoothness
> test case).

## What was built

All landed in commits `8b69dd2` and `3972afc`:

- **D-simple recurrent topology** — `NN_RECURRENT[]` array in
  [`include/autoc/nn/topology.h`](../../include/autoc/nn/topology.h),
  weight-count math 1667 → 1923 when layer 2 is recurrent, hidden-state
  array sizing.
- **Recurrent forward pass** — `NNControllerBackend` with
  `hidden_state_` member + `reset()` method;
  `h_t = tanh(W_xh·x_t + W_hh·h_{t-1} + b_h)` in
  [`src/nn/evaluator.cc`](../../src/nn/evaluator.cc).
- **Xavier init for W_hh** in
  [`src/nn/population.cc`](../../src/nn/population.cc).
- **Serialization of recurrent-flag array** in
  [`src/nn/serialization.cc`](../../src/nn/serialization.cc).
- **3-axis ScenarioScore** with `gp_fitness` typing — `score`
  (tracking), `stability_score` (027 v4 = Σ(|out_pt|−1)+(|out_rl|−1)),
  `energy_score` (027 v3 = Σ(out_th−1)/2). All "lower = better".
- **3-axis lexicase pool** in
  [`src/eval/selection.cc`](../../src/eval/selection.cc), per-axis
  epsilon floor 0.5.
- **Span-start reset hook** in crrcsim
  [`inputdev_autoc.cpp`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp);
  `nnController_` member, construct-once-per-span pattern.
- **`nn2cpp`** emits recurrent forward C code (W_hh mat-vec,
  hidden-state array, `nn_reset()`).
- **Unit tests** — recurrent forward determinism, reset semantics,
  serialization round-trip, multi-objective selection
  (`Selection027*`, `Selection027v4*` — currently `DISABLED_` for
  cadence7-redux).
- **Plotting** — [`plot_evolution_progress.py`](./plot_evolution_progress.py)
  5-panel chart (fitness | streak | stability | energy | sigma).

## What happened — three rnn experiments

All ran with the recurrent layer enabled (`NN_RECURRENT[2]=true`,
1923 weights, 16-wide). Each used a different C2 lexicase axis
formulation:

| Run | C2 formulation | Final/best fitness | Status |
|---|---|---:|---|
| rnn1 | smoothness = Σ\|Δout\| | −8437 @ gen 312 | aborted, stalled high above cadence7's −35951 |
| rnn2 | v3 energy = Σ(out_th−1)/2 | −7152 @ gen 400 | full run, stalled |
| rnn3 | v3 energy + v4 stability (3-axis) | −12626 @ gen 237 | aborted, stalled |
| cadence7-redux (FF, tracking-only) | none | **−33037 @ gen 400** | full run, recovered cadence7 |

PNGs: [`rnn1_evolution.png`](./rnn1_evolution.png) ·
[`rnn2_evolution.png`](./rnn2_evolution.png) ·
[`rnn3_evolution.png`](./rnn3_evolution.png) ·
[`cadence7redux_evolution.png`](./cadence7redux_evolution.png).

### Common failure signature across rnn1/2/3

- **Tracking fitness barely descended** past initial Xavier-init
  levels. Best individuals across 200+ gens of search couldn't find
  the tracking-relevant policy modes that cadence7 reaches in
  ~80–100 gens.
- **`bestSigma` pinned high** at ~0.15, never reached the cadence7
  0.05 floor. Population diversity didn't collapse — search was
  exploring without finding.
- **`pctInStreak` flat or oscillating low**, never climbed. The
  rabbit-tracking quality never developed.
- Energy and stability axes (when enabled) **did move** — those
  axes' scores improved monotonically. Selection was responding to
  them, just not in a way that helped tracking.

### Why it isn't an infrastructure bug

cadence7-redux runs the same binary, same scenarios, same
pop/gen budget — only difference is `NN_RECURRENT[]` all-false and
the C2 axis pushes commented out. It descends cleanly to −33037
(within seed variance of cadence7's −35951). The plumbing works;
the design choices in rnn1/2/3 don't.

## Plausible failure modes (not yet disambiguated)

The rnn1/2/3 outcomes are consistent with any combination of:

1. **D-simple alone is too expensive** at this evolutionary search
   budget (pop 3500, 400 gens). 1923 weights vs cadence7's 1667 is
   only +15 % parameters, but the *structure* changes — the W_hh
   block adds a feedback loop that the GA's flat-vector
   crossover/mutation may explore inefficiently. Hypothesis: 028
   should run **D-alone** (recurrent ON, C2 OFF) as the cleanest
   first experiment to test this.
2. **C2 axes equal-weighted with tracking suppress tracking
   convergence.** Lexicase with 3-axis pool gives each axis equal
   probability of being the next filter, so on average 67 % of
   filtering events push toward stability/energy improvement
   *before* the policy has found tracking-relevant outputs at all.
   Selection is steering away from "use the controls aggressively
   to track" before tracking is even discovered.
3. **C2 axis numeric ranges mis-scaled vs tracking ε-lexicase
   floor.** ε=0.5 absolute floor was chosen heuristically and
   applied uniformly. Stability and energy scores have different
   distributions than tracking; the same floor may be over- or
   under-pressuring those axes.
4. **Hidden-state init/reset interacts badly with random Xavier
   outputs.** A fresh genome's first ticks emit ~random outputs;
   those feed into the recurrent layer immediately and persist
   into the per-tick state. The recurrent feedback may amplify
   random initial noise and stall search before useful
   gradients/structure can emerge.

The 027 close was triggered by realizing all four are open and
none was diagnosed during the runs.

## What carries forward to 028

Spec at [`specs/028-deeper-rnn/spec.md`](../028-deeper-rnn/spec.md);
this section is a pointer summary.

**Code (in tree, gated off)**: 5 `CADENCE7-REDUX` markers, each one
flip away from re-enable.

**Required prework**: failure-mode disambiguation. Cleanest first
experiment is the **D-alone diagnostic** (T310 from 027 escalation,
never executed) — recurrent ON, C2 OFF. If it descends cleanly,
hypothesis 1 is wrong and the issue is C2 incentive design. If it
also stalls, hypothesis 1 is the problem and 028 needs to either
budget more (bigger pop / longer training) or escalate architecture
(32-wide, layer 1 recurrent) before retrying C2.

**Open clarify questions**: 5, listed in 028 spec.md (topology size,
C2 formulation, selection regime, training budget, xiao-port
timing). These shouldn't be answered until the D-alone diagnostic
narrows the failure mode.

## Why this is "iteration", not "feature"

027 spec.md framed the work as a *feature* with a GO/NO-GO gate. In
hindsight that framing was premature: 027's contribution was
**infrastructure plus an under-tested hypothesis**, not a
design-validated controller. The honest framing is that 027 prepared
the ground; 028 runs the experiments that actually settle the
hypothesis. Future evolutionary-search research features in this
project should probably plan for an explicit "diagnostic before
gate" phase — exactly what's missing from 027's plan.md.

## Flight outcome

The 04-26 flight on the cadence7-redux build was the **first
successful path-tracking flight on a learned policy** in this
project's history. All three engage spans ran to path-end, not
pilot abort. Roll bang-bang dropped 33 % vs the 04-22 cadence7
flight — likely attributable to lighter wind, since the binary was
the FF cadence7 architecture (different seed, same topology).

Full report:
[`flight-results/flight-20260426/FLIGHT_REPORT.md`](../../flight-results/flight-20260426/FLIGHT_REPORT.md).

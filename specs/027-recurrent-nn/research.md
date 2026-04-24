# 027 research — recurrent architectures + the incentive question

**Purpose**: Ground the 027 design in what's actually known about
recurrent policies for physics control. Frame the "does memory alone
fix bang-bang?" question concretely. Lay out options with costs so
the spec can refine into a ranked experimental plan.

**Status**: Basic research (draft). Expected to iterate as we refine
spec + clarifications.

---

## 1. Restating the problem after 026

026 findings distilled the bang-bang cause:

- The NN is **stateless, feedforward**. Each tick is decided in
  isolation.
- Fitness is **point-accumulation tracking**. Rewards instantaneous
  position error; silent on output dynamics.
- **No derivative or history inputs.** The NN sees a snapshot, can't
  anticipate or punish-itself-for-jitter.

026 modified only what happens **downstream** of the NN output (add
a PID). Result: same pathology, different variable name. The NN
just evolved to bang-bang its rate command instead of its surface
command.

The user's sharp framing after 026:

> the control feedback still needs incentive, but will both need
> incentive?

Rephrased: there are **two orthogonal axes** of intervention:

- **Axis A — Architecture**: can the NN *represent* state?
  (feedforward → recurrent, or prev-outputs as inputs)
- **Axis I — Incentive**: does the fitness *reward* using that
  state to smooth? (tracking-only → tracking + action-rate penalty)

026 hit neither axis; that's why it didn't move the needle. 027's
central question is which cells of the 2×2 (architecture × incentive)
produce a non-bang-bang controller at acceptable cost.

|                     | Fitness = tracking only     | Fitness = tracking + Δ-penalty     |
|---------------------|-----------------------------|------------------------------------|
| **FF NN** (today)   | cadence7: **bang-bang**     | open-loop smoothing → "mushes" (B) |
| **Recurrent NN**    | **?** — the key question    | most-likely-to-work cell           |

The top-left is our known bad state. The bottom-right is
literature-favored. The open question is whether the bottom-left
cell (memory *without* explicit incentive) can on its own pull the
NN off the rails — i.e., is the extra data in the architecture
enough to create an implicit gradient against bang-bang?

## 2. Does memory alone suffice? (the incentive question)

**Argument "yes":** With prior outputs visible, the NN can in
principle observe that saturation → cost (the aircraft's aero
response to rapid surface reversals is messy, tracking suffers).
Evolutionary pressure on tracking fitness then implicitly punishes
bang-bang through its aero downstream effects. No explicit
smoothness reward needed.

**Argument "no":** That implicit gradient is weak. Evolved NNs
find brittle edge-case exploits; the genetic algorithm is happy to
bang-bang if the tracking advantage exceeds the aero penalty in
*some* scenarios. More importantly, the literature has explicit
data points:

- DRL-trained **LSTM policies for trajectory tracking** are
  documented to *amplify* control-action variability versus FF
  policies unless action-rate regularization is added
  ([MDPI LSTM-enhanced RL, 2025](https://www.mdpi.com/2218-6581/14/6/74)).
  Memory alone made it *worse*.
- **Action-rate / control-rate penalties in reward** are standard
  practice in modern RL fixed-wing/quad work — not an afterthought
  ([arxiv 2409.17896 fixed-wing RL](https://arxiv.org/html/2409.17896v1)).
- Review papers on RL for fixed-wing aircraft treat smoothness as a
  distinct sub-problem needing its own mechanism, not an emergent
  property of memory
  ([Neural Network Flight Control review](https://arxiv.org/abs/2206.05596)).

**Working hypothesis:** probably need **both** architecture and
incentive. A2 alone is worth trying as a cheap falsification — if
it *does* work, great, we got lucky. But we should plan for needing
a smoothness term in fitness too.

This matters for test order: *don't* spend 400 gens on D-alone
before trying D-with-incentive if A2-alone fails.

## 3. Architecture options, ranked by cost

Current topology (cadence7, pid1): `33 → 32 → 16 → 3`,
**1,667 weights**. This is the reference point; every option's cost
is an increment on top.

### A2 — output history as inputs

Feed the last N ticks of NN output back as inputs next tick.
Architecturally still feedforward — cheapest possible "NN has
memory" experiment.

| N | Extra inputs | New input count | First-layer weight cost | Total weights | Δ vs 1667 |
|---|---|---|---|---|---|
| 1 | 3  | 36 | (36+1)·32 = 1184 | **1763** | +6 % |
| 3 | 9  | 42 | (42+1)·32 = 1376 | **1955** | +17 % |
| 5 | 15 | 48 | (48+1)·32 = 1568 | **2147** | +29 % |

Pros: tiny change; just `include/autoc/nn/nn_inputs.h` +
`src/nn/evaluator.cc` populate + the xiao mirror. Cereal flag exists
already (`cached_*_cmd` in xiao/src/msplink.cpp).

Cons: feedforward-with-history is a lossy proxy for real state —
NN can't *learn* what to remember, only see a fixed window.

### A3 — derivative/history of **inputs** (not outputs)

Feed rate-of-change of target direction, distance, etc. as extra
inputs. Not really "memory of self" — gives trend visibility but
NN still has no view of what *it* was just doing.

Costs similar to A2 per extra input.

**The user's read** (from conversation):
> a3 is about target not as much about our control history

Correct. A3 helps anticipation; doesn't attack self-jitter. De-prioritized
for this spec's focus on smoothness.

### D — simple recurrent layer

Make the 32-wide hidden layer recurrent. Forward pass becomes:

    h_t = tanh(W_xh · x_t + W_hh · h_{t-1} + b_h)
    out_t = W_out · layer2(h_t)

| Hidden size (recurrent) | W_hh cost | Total weights | Δ vs 1667 |
|---|---|---|---|
| 16-wide (make layer2 recurrent) | 16·16 = 256   | **1923** | +15 % |
| 32-wide (make layer1 recurrent) | 32·32 = 1024  | **2691** | +61 % |

Pros: NN can learn *what* to remember, not fixed window. Standard
in modern RL ([arxiv 2206.05596](https://arxiv.org/abs/2206.05596)
review). Active research area for evolved controllers
([Recurrent NEAT, NEORL](https://neorl.readthedocs.io/en/latest/modules/neuroevolu/rneat.html);
[CTRNN in NEAT-Python](https://neat-python.readthedocs.io/en/latest/ctrnn.html)).

Cons: evaluator rewrite — single-pass → per-tick carried state.
Serialization schema change. xiao forward pass updated to carry
state across MSP frames. Weight-count growth slows genetic search
proportionally.

### D-gated — GRU-lite

Same structure as D but with update/reset gates. ~3× the weight
count of simple RNN for the same hidden size. Standard choice in
sequence-learning literature when gradient-based training; less
obvious for genetic evolution since the gates add evolutionary
search dimensions that may not pay off without gradient signal.

Precedent: [GRU-NEAT for navigation (arxiv 1904.06239)](https://arxiv.org/pdf/1904.06239)
shows it works but with carefully-designed tasks.

| Recurrent layer | Weight cost | Total |
|---|---|---|
| 16-wide GRU-lite (update + reset) | ~768 | ~2435 (+46 %) |
| 32-wide GRU-lite | ~3072 | ~4739 (+184 %) |

Probably **overkill** for our 33→3 mapping at current horizons.
Holding in reserve.

### D-esn — echo state network

Random-fixed recurrent reservoir; only output weights are evolved.
The reservoir is the "memory"; evolution learns to read it.

Pros: drastically reduces what evolution has to search over (~only
output weights). Documented for fixed-wing UAV control
([IEEE 7525104 ESN UAV](https://ieeexplore.ieee.org/document/7525104/)).
Potentially the **sweet spot** for genetic search + recurrent state.

Cons: reservoir design is its own art; fixed-weight init adds a
hyperparameter (sparsity, spectral radius) we'd have to tune.
Xiao-side weight export gets a little messier.

Worth keeping in the candidate set, especially if D's weight-count
cost bites training time harder than expected.

## 4. Incentive options, ranked by cost

### C1 — action-rate penalty in fitness

Subtract `α · (|ΔoutPt|+|ΔoutRl|+|ΔoutTh|)` per tick from the
step score in `FitnessComputer::computeStepScore`. One line. α is
a hyperparameter; needs light sweep (start 0.05, range 0.01–0.2 of
peak step score).

Cons: open-loop (pre-026 experience) "tends to B" — mushes. But
**pre-026 this was tried without memory**. With A2 or D providing
memory, the NN can potentially learn to use smoothness selectively
rather than uniformly blur. This is the key hypothesis for the
bottom-right cell.

### C2 — multi-objective / Pareto selection

Lexicase or Pareto with `(tracking, smoothness)` objectives. More
principled than C1 (no α tuning), at the cost of a selection-code
change.

Current code uses `lexicase` (see `src/nn/selection.cc`). Adding
Pareto is moderate effort; multiple precedents in evolutionary
literature.

### C3 — hard slew-rate limiter in consumer (== B)

Known bad from prior art (the 026-findings table). Structurally
prevents bang-bang but the NN still evolves around it. Not listed
as a primary 027 option.

## 5. Carry-forward from 026 (what we don't have to rebuild)

- **PidInternals struct on AircraftState** — kept, repurposed as
  observational diagnostic (rateCmd = "if we PID'd this", rateAch =
  actual body rate, FF = unity passthrough).
- **13 rateCmd/Ach/pid* columns in data.dat** — same.
- **Variation landscape, fitness surface, cadence fix, coord
  conventions** — all validated.
- **Cereal/RPC payload shape** — no reshuffle needed for A2.
- **cadence7 baseline (−35951 @ 400 gen)** — reference for fitness
  comparison. pid1's −27045 is the "worse than baseline" mark.
- **Control aggressiveness plot + script** — measurement tool is
  ready; bakes into any 027 gen-100+ check.

## 6. Proposed experimental test matrix

Cheapest-first, escalating along whichever axis moves the needle:

| # | Config | Axis | Total weights | Est. gen budget | Decision rule |
|---|---|---|---|---|---|
| 0 | 027 baseline (nullified PID, cadence7-parity) | none | 1667 | ~50 (smoke) | sanity: same fitness & bang-bang as cadence7 |
| 1 | A2 (prev-output, N=3) | A only | 1955 | 200 | does |out| or dCtrl drop at matched gen? |
| 2 | A2 + C1 (light α) | A + I | 1955 | 200 | bottom-right cell, light-touch |
| 3 | D 16-wide simple RNN | A only (stronger) | 1923 | 400 | does memory alone do it? |
| 4 | D 16-wide simple RNN + C1 | A + I | 1923 | 400 | primary hypothesis |
| 5 | D-esn reservoir | A only (cheap) | ~1750 | 300 | faster convergence? |

"Est. gen budget" assumes training time scales linearly with weight
count versus cadence7's 400-gen baseline. Will refine when we have
observed per-gen times on the 027 binary.

**Decision logic**:

- Step 0 establishes that nullifying the PID recovered cadence7
  parity (sanity only).
- Step 1 (A2 alone) is the cheap falsification of "memory alone
  suffices." If it works: skip to step 5 for efficiency or stop.
- Step 2 gives us the first bottom-right data point with minimal
  infrastructure cost.
- Steps 3-4 commit to the real architecture change. Step 4 is the
  primary bet.
- Step 5 explores if D-simple is weight-budget-painful.

## 7. Open questions for spec refinement

These should drive the next `/speckit.clarify` pass before 027
moves to plan/task:

1. **Scope of 027 — how many cells?** Commit to #0-2 as MVP and
   treat #3-5 as escalation? Or go straight to #3-4 on the premise
   that A2 is a weaker test?
2. **N for A2** — pick 3 or 5? Weight-cost difference is
   significant for evolution budget.
3. **α tuning for C1** — how much retraining per α? Single α or
   sweep?
4. **D architecture — simple RNN vs ESN first?** The ESN angle
   might be the right bet for genetic evolution specifically; not
   well-known outside niche literature.
5. **Xiao implications** — recurrent forward pass isn't free on
   the flight FC. Do we commit to generating a per-tick state
   variable in `nn_program_generated.cpp`? When?
6. **Flight-gate criteria** — at what 027 result do we actually
   flash cadence11 (or whatever) and fly? What does "passes the
   bench" look like now that rate-tracking isn't a valid proxy
   (PID nullified)?
7. **Reset semantics** — hidden state reset on span start is
   obvious; but what about reset on NN-cadence tick-boundaries
   (since NN runs at 10Hz, outer frame at 20Hz)? Does the NN's
   hidden state carry across outer-frame ticks or only across NN
   evaluations?

## 8. What this research is NOT

- A full lit review. This is grounding, not a thesis.
- A commitment to a specific cell of the 2×2. That comes out of
  clarify + spec refinement.
- A quantitative cost estimate on wall-clock training time. Need
  measured per-gen times on the 027 binary for that.
- An opinion on whether 025 (craft variations) should move forward.
  Held as blocked until 027 has a result.

## Sources

- [arxiv 2206.05596 — Neural network-based flight control systems: present and future (review)](https://arxiv.org/abs/2206.05596)
- [arxiv 2409.17896 — Model-free vs model-based RL for fixed-wing UAV attitude control](https://arxiv.org/html/2409.17896v1)
- [arxiv 1904.06239 — Evolving indoor navigational strategies using GRUs in NEAT](https://arxiv.org/pdf/1904.06239)
- [MDPI 14/6/74 — LSTM-Enhanced Deep RL for robust trajectory tracking (action-variability warning)](https://www.mdpi.com/2218-6581/14/6/74)
- [IEEE 7525104 — Recurrent NN (ESN) for nonlinear control of fixed-wing UAV](https://ieeexplore.ieee.org/document/7525104/)
- [Springer s11071-023-08725-y — DRL control approach for high-performance aircraft](https://link.springer.com/article/10.1007/s11071-023-08725-y)
- [NEORL rNEAT documentation — recurrent neuroevolution of augmenting topologies](https://neorl.readthedocs.io/en/latest/modules/neuroevolu/rneat.html)
- [NEAT-Python CTRNN implementation (continuous-time recurrent NN in NEAT)](https://neat-python.readthedocs.io/en/latest/ctrnn.html)
- [Stanley/Miikkulainen 2002 — Efficient RL through evolving NNs (NEAT foundational)](https://nn.cs.utexas.edu/downloads/papers/stanley.gecco02_1.pdf)

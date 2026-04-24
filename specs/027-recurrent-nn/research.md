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

**Already implemented.** Current NNInputs
([include/autoc/nn/nn_inputs.h](../../include/autoc/nn/nn_inputs.h))
carries:
- 6-sample target-direction history: `[t−0.9, t−0.3, t−0.1, t+0, t+0.1, t+0.5]`
  for body-frame target_x/y/z and distance
- `closing_rate` — explicit derivative of distance
- `gyro_p/q/r` — explicit derivatives of attitude

The NN *already* sees trend information about the world. Bang-bang
emerges despite it. That localizes the missing piece: the NN sees
what the **world** is doing, not what **it** is doing. Self-memory
— not more world-memory — is the gap. This removes A3 from the
027 option set entirely; it's solved-state-of-the-art for our
constraints already.

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

### D-gated (GRU-lite) and D-esn (echo state)

Held in reserve, not primary options.

- **GRU-lite**: ~3× weight count of simple RNN; gate dimensions
  are hard for genetic evolution to navigate without gradient
  signal. Literature precedent (GRU-NEAT,
  [arxiv 1904.06239](https://arxiv.org/pdf/1904.06239)) works but
  on carefully designed tasks.
- **Echo state network** (fixed-random reservoir, evolve only
  readout): theoretically compatible with genetic search, but the
  reservoir itself is a hyperparameter tuning surface (sparsity,
  spectral radius, size). The user's read: "smacks of fine tuning."
  Project bias is against fine-tuning-heavy approaches when a
  direct-to-evolve simple form exists.

D-simple is the primary architectural bet.

## 4. Incentive options, ranked by cost

### C2 — extend lexicase with a smoothness test case (primary)

Lexicase selection ([src/eval/selection.cc:30](../../src/eval/selection.cc))
is already the selection method. Epsilon-lexicase works by running
candidates through a sequence of test cases in random order; at
each stage individuals within ε of the best survive. Currently the
test cases are per-scenario tracking scores.

Extending lexicase with a smoothness test case (e.g.,
`sum_over_ticks(|ΔoutPt|+|ΔoutRl|+|ΔoutTh|)` computed per
individual per scenario) gives the selection mechanism direct
access to the smoothness axis without α tuning — individuals that
track well in most scenarios but bang-bang get filtered in the
smoothness round, and vice versa.

Pros:
- No α hyperparameter sweep.
- Same selection primitive, just more test cases in the pool.
- Prior art in the codebase: we've made "a few stabs" at
  smoothness-flavored selection variants
  (see commit d6a970c era work).

Cons: harder to reason about the exact pressure than C1. Needs a
defined "smoothness score per scenario" that's computed alongside
tracking score — small but non-trivial plumbing in
`FitnessComputer` and `ScenarioScore`.

**Primary 027 incentive option.**

### C1 — action-rate penalty in fitness (held in reserve)

Subtract `α · (|ΔoutPt|+|ΔoutRl|+|ΔoutTh|)` per tick from the
step score in `FitnessComputer::computeStepScore`. One line. α is
a hyperparameter; needs light sweep.

Prior experience: "c1, c2 yes but open loop this tends to look
like B [rate-limit mush]." That's without memory. With memory
(D-simple) the NN can in principle use smoothness selectively
rather than uniformly blur — but C2 is the cleaner mechanism to
test that hypothesis given our existing lexicase plumbing.

Held as a fallback if C2-via-lexicase plumbing proves costly or
ambiguous in signal.

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

Reordered by project bias (per conversation 2026-04-24): D-simple +
C2-lexicase is the primary bet; A2-alone and A2+C2 are cheap
falsification upstream of it; D-ESN and GRU held in reserve.

| # | Config | Axis | Total weights | Est. gen budget | Decision rule |
|---|---|---|---|---|---|
| 0 | 027 baseline (nullified PID) | none | 1667 | ~50 (smoke) | sanity: cadence7-parity fitness & bang-bang |
| 1 | A2 (prev-output, N=3) | A only | 1955 | ~200 | cheap falsification: does memory alone help at all? |
| 2 | A2 + C2-lexicase-smoothness | A + I | 1955 | ~200 | does lightweight memory + selection pressure suffice? |
| 3 | **D 16-wide simple RNN + C2** | **A + I** | **1923** | ~400 | **primary bet** — real memory, principled incentive |
| 4 | D 16-wide simple RNN alone | A only | 1923 | ~400 | diagnostic: is the memory doing the work or the incentive? |

Est. gen budget scales roughly with weight count versus cadence7's
1667 / 400-gen baseline. Will refine with observed per-gen times on
the 027 binary.

**Decision logic**:

- **Step 0** is the one definite starting point — confirms PID
  nullification recovered cadence7 parity. No surprises expected;
  if fitness or aggressiveness differs from cadence7 meaningfully,
  something's wrong with the passthrough.
- **Step 1** (A2 alone, cheapest axis-A test) tells us whether
  giving the NN its own outputs as inputs produces *any* movement
  off rails. Low budget because we expect it to fail (literature
  bias: LSTM-alone amplified variability; less powerful A2 likely
  worse). Result informs order after step 2.
- **Step 2** is the first bottom-right cell. If this works, we
  save the recurrent-rewrite cost entirely.
- **Step 3** is the primary experiment. Architecture change AND
  incentive change together — the configuration literature most
  strongly endorses.
- **Step 4** is diagnostic if 3 works — if memory alone (without
  C2) *also* gets off rails, it tells us the incentive wasn't
  load-bearing; if it plateaus at bang-bang, we confirm both axes
  were necessary. Useful for deciding where future compute goes.

**Reserved** (not in initial matrix): GRU-lite, D-ESN, C1
rate-penalty fallback. Revisit if 1-4 don't land.

## 7. Open questions for spec refinement

Project biases have narrowed several of the original questions.
Remaining ones for the next `/speckit.clarify` pass:

1. **N for A2** — 3 or 5 past outputs? Cheap falsification
   lands at N=3 (lowest cost that still gives sequence
   information); N=5 if we want a stronger test of the weaker
   architecture. Bias: start at 3.
2. **C2 smoothness score definition** — sum of |Δout|? RMS? Per-
   axis or combined? How aggregated across ticks to a single
   per-scenario score for lexicase? Light design work.
3. **D-simple hidden-size decision** — 16-wide recurrent
   (make layer2 recurrent, +256 weights, +15%) versus 32-wide
   (layer1 recurrent, +1024, +61%)? Bias: start at 16 for budget.
4. **Hidden-state reset semantics** — clear reset on span start
   is obvious. What about NN tick vs outer-frame tick (10Hz vs
   20Hz cadence)? Does the hidden state advance each outer frame
   even when NN isn't evaluated, or only on NN ticks? Inclined
   toward "only on NN ticks" — keeps state semantics close to
   conventional recurrent policy.
5. **Xiao implications** — recurrent forward pass needs per-tick
   state in `nn_program_generated.cpp`. Small state array + an
   extra mat-vec per tick; not free but tractable. When do we
   do the xiao-side work — after step 3 succeeds, or in parallel?
6. **Flight-gate criteria** — rate-tracking is no longer a valid
   proxy (PID nullified). What replaces it? Likely: aggressiveness
   signature off cadence7 plateau + fitness at or above cadence7
   at same gen count + tier-0/1 eval pass.

**Decided via research + history biases** (no clarify needed):
- A3 skipped — already implemented via NNInputs trend samples.
- GRU-lite skipped — weight cost disproportionate for our scale.
- D-ESN skipped — project bias against hyperparameter tuning
  burden.
- C1 skipped as primary — C2-via-lexicase is cleaner given the
  existing selection plumbing.

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

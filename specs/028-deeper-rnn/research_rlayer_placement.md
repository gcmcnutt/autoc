# 028 — Recurrent layer placement, width, and init: first-principles research

**Purpose**: anchor the topology decision for 028's experiment ladder in
literature priors and physical reasoning, not vibes. Inputs: current
topology `{33, 32, 16, 3}` with layer-2 (16-wide) recurrent (W_hh = 256,
total 1923), 027 rnn1/2/3 stalled, 028 D-alone diagnostic budget bounded
(≤ 5000 pop × ≤ 600 gens × ≤ 3 attempts). Sim NN tick = 100 ms (10 Hz)
confirmed via `SIM_TIME_STEP_MSEC` in
[`include/autoc/eval/aircraft_state.h:41`](../../include/autoc/eval/aircraft_state.h).

## 1. Where should recurrence live?

**Recommendation: keep recurrence on layer 2 (16-wide) for D-alone.
Escalation order if it stalls is W_hh-only on layer 1 (32-wide), then
*both* layers recurrent, before considering output-adjacent recurrence.**

The literature on stacked recurrent networks consistently places memory
*above* a feedforward feature extractor: lower feedforward layers
condense the raw sensor flood into stable mid-level features, and the
recurrent block then integrates *those features* over time
([MachineLearningMastery, stacked LSTMs](https://machinelearningmastery.com/stacked-long-short-term-memory-networks/);
[Güçlü & van Gerven, 2017, hierarchical process memory](https://pmc.ncbi.nlm.nih.gov/articles/PMC5895512/)).
Lower layers handle "what the world looks like right now"; higher
layers handle "how the world has been moving." That maps cleanly onto
this controller: layer 1 (33→32) compresses target-direction history,
quaternion, gyro, and airspeed into a 32-dim flight-state embedding;
layer 2 (32→16) is the natural place to integrate that embedding with
its own recent history before projecting to surface commands. Putting
memory on layer 2 also keeps W_hh small (256 vs 1024 weights), which
matters because the GA is doing flat-vector real-coded search with no
gradient signal to amortize wasted parameters.

Putting recurrence on layer 1 (32-wide) couples the recurrent state
directly to raw, noisy, wildly-varying sensor channels (gyro rates,
quaternion). The recurrent block then has to *both* learn what to
remember *and* learn to filter raw inputs simultaneously — a strictly
harder evolutionary search problem at 4× the W_hh weight count. EXAMM
findings ([Ororbia et al. 2019](https://arxiv.org/abs/1902.02390))
show that under neuroevolution, deep simple-recurrent connections
often outperform complex memory cells *when placed at intermediate
depth*, not at the input layer.

Output-adjacent recurrence (a recurrent connection on the 3-output
layer, or just before it) collapses to "remember last command" — that
is functionally A2 (prev-output as input) reframed, and 027's research
already deferred A2 because the codebase already provides target/dist
trend inputs. No new information.

The argument against layer 2 is that 16 hidden units may be too few to
encode the relevant dynamics richly. That's a width concern, not a
placement concern, and is addressed in §2.

## 2. How wide should the recurrent layer be?

**Recommendation: keep 16-wide for D-alone. If it stalls and telemetry
signals 1–2 (W_hh CV / activation ratio per the spec) show the GA is
*searching* W_hh but not converging, escalate to 24-wide (W_hh = 576,
total ≈ 2243) before jumping to 32-wide (W_hh = 1024, total 2691).**

Three independent threads of reasoning all favor "small first."

**(a) Time-constant argument.** Useful memory horizon for this
controller (see §3) is 5–10 ticks. A simple-RNN at tanh saturation has
an effective memory time-constant set by the W_hh spectral radius:
ρ ≈ 0.9 stores ~10 ticks before decay; ρ ≈ 0.95 stores ~20. Encoding
a 10-tick window of *one* scalar requires roughly one effective hidden
dimension. The relevant scalar count for this problem is small — body-
frame target azimuth, elevation, closing rate, pitch-rate trend, roll-
rate trend, throttle history — call it 6–8 latent dimensions. 16 is
already 2× that headroom; 32 is 4×. A reservoir-computing
([Jaeger ESN](https://www.researchgate.net/figure/The-spectral-radius-for-the-tanh-reservoir-of-eq-1-as-the-fraction-of-nonzero-entries_fig4_358638807))
informational-capacity argument says memory capacity scales with N for
random reservoirs; for *evolved* low-rank dynamics it scales worse,
but the floor is still well within 16.

**(b) GA search-efficiency argument.** Real-coded GA convergence on
flat parameter vectors empirically scales as roughly Θ(N · log N) in
the number of weights N for smooth fitness landscapes (Stanley &
Miikkulainen NEAT lineage; EXAMM throughput reports
[arxiv 1902.02390](https://arxiv.org/abs/1902.02390)). W_hh is the
*only* sub-block whose fitness gradient is purely indirect — a W_hh
weight only matters through the temporal coupling, so its per-weight
fitness signal is much weaker than a W_xh or W_out weight. Quadrupling
W_hh from 256 → 1024 doesn't quadruple search difficulty linearly,
but it does *meaningfully* slow GA descent on the hidden-block
sub-problem at exactly the budget regime (3500–5000 pop × 400–600 gens)
028 lives in.

**(c) Quadrotor RL prior.** Modern deployed RL flight controllers run
*small*: Hwangbo et al. 2017 quadrotor (~64 hidden, FF), Neuroflight /
GymFC (Koch et al. 2019, 2× ~64-unit FF layers, no recurrence
[arxiv 1901.06553](https://ar5iv.labs.arxiv.org/html/1901.06553)),
RAPTOR foundation policy (~2084 total parameters across three layers
[arxiv 2509.11481](https://arxiv.org/html/2509.11481v1)). The bias of
the field is: small policy nets work; capacity is rarely the
bottleneck for low-DOF flight control. This problem (3 outputs, 33
inputs, 10 Hz) is in the small-net regime. There's no Bitter-Lesson
case for going larger by default.

**(d) Counter-argument to consider but reject.** A bigger W_hh gives
more redundant pathways and (in gradient learning) faster training.
Under GA, that intuition inverts: redundant pathways multiply the
search volume, and the GA cannot exploit them via the gradient-based
ensembling that makes wide nets train fast. 8-wide would be even
faster to search but falls below the 6-8 latent-dim floor; 16 is the
right balance.

## 3. Memory horizon for this problem

**Recommendation: design for 5–10 ticks (0.5–1.0 s) of effective
memory horizon. Beyond ~15 ticks, returns are minimal for this
craft.**

Sim NN tick = **100 ms** (10 Hz, `SIM_TIME_STEP_MSEC` in
[`aircraft_state.h:41`](../../include/autoc/eval/aircraft_state.h)).
Stacking the relevant time constants:

- Servo response: 20–30 ms = 0.2–0.3 ticks. Sub-tick; not
  observable as state.
- ESC + prop spool: 30–50 ms = 0.3–0.5 ticks. Sub-tick.
- AHRS pipeline (INAV): 10–20 ms = 0.1–0.2 ticks. Sub-tick.
- Roll axis time constant: 100–300 ms = 1–3 ticks. **Edge of
  observability.** This is the dominant short-horizon dynamic.
- Pitch axis time constant: 200–500 ms = 2–5 ticks. **In-band.**
- Phugoid mode: 5–10 s = 50–100 ticks. **Out of band.** The NN
  shouldn't try to learn this; the rabbit-track problem is a
  short-horizon control problem, not a long-horizon planning one.
- Tracking-cone time-to-rabbit at typical speed: 0.5–1.5 s = 5–15
  ticks. The dominant *task-level* horizon.

Useful memory horizon clusters at **3–10 ticks**: long enough to span
the airframe rotational time constants (so the NN can learn "I just
commanded right roll, the body is still rolling, *don't add more*"),
and long enough to span one tracking-cone settling event. Beyond 15
ticks the relevant state is already encoded in the existing 6-sample
target-history input. A simple-RNN with tanh and W_hh spectral radius
near 0.85–0.95 covers exactly this band; this is a good fit for evolved
weights without explicit gating.

## 4. Hidden-state initialization

**Recommendation: keep current Xavier as the D-alone baseline. If
D-alone descends partway then stalls (failure mode 4), try (a)
orthogonal init of W_hh (cheap, well-supported), then (b) zero-init
of h_0 at span start independent of W_hh init (the hidden *state*,
not the *weights*). Skip identity init and layer-norm.**

For tanh recurrent networks under *evolutionary* (not gradient) search,
the relevant question isn't "which init gives best gradients" — it's
"which init gives the GA a sane starting fitness landscape."

- **Xavier (current)**: random, symmetric, scaled to keep tanh
  preactivations near linear regime. Reasonable baseline. Risk: random
  W_hh has spectral radius scattered around √(2/n) ≈ 0.35 for n=16;
  initial memory horizon is short (~1–2 ticks). The GA must evolve
  longer horizons from scratch. That may be why rnn1/2/3 stall — the
  recurrent block is essentially inert at gen 0 and the GA finds
  feedforward-equivalent solutions before recurrence ever activates.
- **Orthogonal**: spectral radius exactly 1, all eigenvalues unit
  modulus. Initial memory is *long* and *stable*, no decay/explosion.
  Strongly endorsed for tanh RNNs
  ([Saxe et al. 2014, Smerity 2016](https://smerity.com/articles/2016/orthogonal_init.html);
  [Vorontsov et al. 2017](http://proceedings.mlr.press/v70/vorontsov17a/vorontsov17a.pdf)).
  For *gradient* training, orthogonal init dramatically reduces
  vanishing-gradient pathology. For *evolutionary* search, the analog
  is that the recurrent block is *active from gen 0*, so the GA gets
  immediate fitness signal on W_hh perturbations rather than having to
  first rediscover that recurrence helps. **This is the #1 follow-on
  bet if D-alone stalls and the W_hh activation-ratio telemetry shows
  a dead block.**
- **Identity**: ρ = 1 exactly, but degenerately structured (no
  cross-talk between hidden units initially). Useful for very-deep
  networks under gradient training. Under GA at this scale, identity
  is just orthogonal-restricted; orthogonal dominates.
- **Small-random / scaled Xavier with gain > 1**: cheap variant — bump
  Xavier scale by ~1.5× on W_hh only. Gives ρ closer to 0.8 at
  init. Worth as a quick-and-dirty alternative if orthogonal plumbing
  is more invasive than expected.
- **Layer-norm** on h_t: changes the dynamical system meaningfully,
  introduces extra trainable scale/shift parameters that the GA must
  also evolve. Skip — too much new search surface for a diagnostic.

The orthogonal-init bet for D-alone hypothesis-4 escalation is well-
matched to the W_hh CV telemetry signal: if orthogonal init makes the
recurrent block fitness-active at gen 0, you'll see W_hh CV climb in
parallel with W_xh CV from the start instead of lagging.

## 5. Final ranked recommendation

**For 028 D-alone diagnostic: stay at 16-wide layer-2 simple recurrent
with Xavier init.** This matches the inherited config; no change. The
inherited choice is defensible on placement (mid-stack memory above a
sensory feature extractor), width (within the 6–8 latent floor with
2× headroom, fits GA budget at this pop/gen scale), and init (the
diagnostic *purpose* is to see if D-alone descends — switching init
simultaneously confounds the failure-mode disambiguation).

**Escalation ladder if D-alone stalls**, ordered by information yield
per compute spend:

1. **Larger budget at current config** (5000 pop, 600 gens) — already
   the spec's first escalation. Confirms it's not a search-budget
   problem before changing the architecture.
2. **Orthogonal W_hh init at 16-wide layer-2** — cheapest architectural
   change, high theoretical payoff, directly addresses failure-mode 4.
3. **24-wide layer-2 recurrent** (W_hh = 576) — modest capacity bump
   without the 4× W_hh blowup of 32-wide. Diagnostic for "is 16 a
   capacity wall?"
4. **32-wide layer-2 recurrent** — only if (3) shows clear improvement
   trajectory.
5. **Move recurrence to layer 1 (32-wide)** — placement change, much
   bigger search space; only if the layer-2 ladder above plateaus.
6. **Both layer 1 and layer 2 recurrent** — last resort within 028's
   budget; closer to 029 territory if it doesn't work.

This ranking is *the same answer* as the inherited 027 architectural
bet, with one substantive addition: orthogonal init promoted to step 2,
ahead of any width or placement change. It's the single highest-EV
intervention if the current run reproduces 027's stall, because it
costs ~50 LOC, the telemetry signals already specified will cleanly
show whether it worked, and the literature support is unusually strong
for tanh RNNs.

## Implications for 028 plan

The 028 plan as drafted is sound on placement and width — keep
16-wide layer-2 for D-alone, escalate budget first, then topology.
The one addition this research surfaces is to **promote orthogonal
W_hh initialization ahead of width/placement changes** in the
post-D-alone escalation ladder, because (a) it's cheaper to implement
than a topology change, (b) the literature support for tanh RNNs is
unambiguous, and (c) it directly addresses failure-mode 4 (state-init
× Xavier random outputs interacting badly), which the spec already
flags as a candidate. The 5–10-tick memory horizon derived in §3
should be referenced in the plan's success criteria for the W_hh
diversity telemetry — if W_hh evolves in 028 and the resulting
spectral radius is well below 0.7 or above 1.05, the network is not
operating in the useful memory band regardless of fitness, and that's
itself a diagnostic signal.

## Sources

- [Ororbia et al. 2019, Investigating RNN Memory Structures using Neuro-Evolution (EXAMM, arxiv 1902.02390)](https://arxiv.org/abs/1902.02390)
- [Desell et al. 2019, Empirical Exploration of Deep Recurrent Connections via Neuro-Evolution (arxiv 1909.09502)](https://arxiv.org/abs/1909.09502)
- [Hwangbo et al. 2017, Control of a Quadrotor with Reinforcement Learning (arxiv 1707.05110)](https://arxiv.org/abs/1707.05110)
- [Koch et al. 2019, Neuroflight: Next Generation Flight Control Firmware (arxiv 1901.06553)](https://ar5iv.labs.arxiv.org/html/1901.06553)
- [Eschmann et al. 2025, RAPTOR: Foundation Policy for Quadrotor Control (arxiv 2509.11481)](https://arxiv.org/html/2509.11481v1)
- [Saxe et al. 2014 / Smerity 2016, orthogonal initialization for RNNs](https://smerity.com/articles/2016/orthogonal_init.html)
- [Vorontsov et al. 2017, On orthogonality and learning recurrent networks with long term dependencies (PMLR v70)](http://proceedings.mlr.press/v70/vorontsov17a/vorontsov17a.pdf)
- [Henaff et al. 2018, Initialization Matters: Orthogonal Predictive State RNNs (ICLR)](https://homes.cs.washington.edu/~bboots/files/OrthogonalPSRNNs.pdf)
- [Stacked LSTM hierarchical abstraction (MachineLearningMastery)](https://machinelearningmastery.com/stacked-long-short-term-memory-networks/)
- [Güçlü & van Gerven, hierarchy of process memory in deep RNNs (PMC5895512)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5895512/)
- [Influence-aware memory architectures for deep RL in POMDPs (Springer 2022)](https://link.springer.com/article/10.1007/s00521-022-07691-7)
- [arxiv 2206.05596 — Neural network-based flight control review (carried from 027)](https://arxiv.org/abs/2206.05596)
- [NEORL rNEAT documentation](https://neorl.readthedocs.io/en/latest/modules/neuroevolu/rneat.html)

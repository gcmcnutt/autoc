# Approach proposal — per-tick credit, a computable energy reference, and a trimmed input vector

**Written 2026-08-17**, after t1 was stopped at gen 608. Not a spec; a proposal to argue with.

## The diagnosis

Three findings from t1 point the same way.

1. **Determinism is clean and capacity is spare.** 487+ `NN_ELITE_SAME` with zero divergence; `W_hh`
   effective rank 11.1–11.8 of 16, flat for 608 generations. So slow progress is **not** noise and **not**
   the network being too small. That leaves the *objective* and the *search*.
2. **The policy spends everything, everywhere.** nz median 3.17 g (≈71° sustained bank), peak 11.13 g,
   throttle amplitude pinned at 0.99 with near-zero change rate. It has converged on a tight spiral, which
   [project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md)
   already records as **objective-optimal under no-future inputs**.
3. **Every energy/smoothness objective tried so far muted the whole regiment.** Operator, and it is the
   crux: full power is genuinely right when far behind, genuinely right to hold a spiral, and genuinely
   right against pitch-induced drag. A penalty on an **absolute** quantity cannot separate "spending
   because the situation demands it" from "spending because nothing said not to" — so it can only quiet
   everything. [project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)
   records the 033 version of this failure (Pareto-corner collapse, killed at gen 457).

**What is missing is a reference.** Not a smaller penalty — a *state-conditioned* one.

## The three pieces

### A. A computable energy reference (do this first — it is not speculative)

The "uncomputable optimal energy" has an aerospace-native, **fully computable** form: Boyd's
energy-maneuverability quantities.

```
Es = h + v²/2g              specific energy      (metres of energy height)
Ps = dEs/dt = (T − D)·V / W specific excess power (metres/second)
```

Every term is already available — altitude and velocity are per-tick in the dmp; thrust, drag and weight
are in the FDM. **Nothing needs to be invented, only computed.**

Why this is the right reference and a throttle penalty is not:

- `Ps` is *how much energy the airframe could gain at this state*. `Ps_actual / Ps_max(state)` says whether
  a manoeuvre was expensive **for what it achieved**, which is exactly the distinction a flat penalty
  cannot make.
- Far behind at full power reads as **efficient** (near-max `Ps`, all of it going into closing). A spiral
  bleeding energy into induced drag reads as **expensive**. Same throttle, different verdict — the thing
  every previous attempt failed to do.
- Boyd developed E-M **for air combat**, so it falls directly out of the M3 offense/defense branch:
  energy state relative to an opponent *is* the strategic variable there.

⚠️ `Ps` is a **rate**, so any objective built on it must be millisecond-denominated like `kNNHistoryLagsMsec`
and `ENVELOPE_SECS`, or it silently rescales when the control interval changes.

**Cheapest first step, no objective change**: compute `Es`/`Ps` from the *existing* t1 dmps and plot energy
state along a trajectory. That alone tests the operator's claim that the spiral is bleeding energy to
induced drag. It is a read, not a bake.

### B. Per-tick credit — and the one theoretical guard rail

Moving from one scalar per ~1000 ticks to per-tick credit is the ~1000× density increase. But "score each
step" splits two ways, and conflating them wastes the effort:

- **per-step REWARD** → still RL; owes temporal credit assignment.
- **per-step TARGET** → supervised; owes a teacher the sim does not provide.

The safe, principled middle is **potential-based reward shaping** (Ng, Harada & Russell): with

```
F(s, s') = γ·Φ(s') − Φ(s)
```

the shaped reward provably **leaves the optimal policy unchanged** while making the signal dense. Take
Φ = the track score (or −distance-to-target). This matters because it is the one form of per-step shaping
that *cannot* introduce a new attractor — and the tight spiral is precisely a policy that found an
unintended attractor. Every ad-hoc per-step penalty tried so far lacked this guarantee.

Then the state-conditioned part — **the advantage** — is what makes density useful:

```
A(s,a) = Q(s,a) − V(s)
```

⚠️ **You never need to know the optimum.** `V(s)` is *learned*, and only has to be good enough to **rank**
actions at that state. That is the entire reason actor-critic exists, and it is the direct answer to "if we
knew an optimal path…" — we do not need to know it. `Ps/Ps_max` is a **hand-built** baseline; `V(s)` is a
**learned** one. Get the hand-built one working first: it tells us how much the learned one must beat.

### C. The input vector — what earns its place

Ranked by **contribution** (relative first-layer weight × that input's recorded std — what the network
actually feels), gen 608:

| slot | contribution | verdict |
|---|---|---|
| `INWARD_BODY_X` | 0.590 | keep |
| **`IN_ENVELOPE`** | **0.324** | keep — but see the reshape below |
| `ACCEL_Z` (load) | 0.252 | keep — the load axis, and the only one a load objective could use |
| `ACCEL_X` | 0.089 | keep (longitudinal accel ≈ thrust/drag balance — feeds directly into `Ps`) |
| `ENVELOPE_SECS` | 0.082 | **drop or reshape** |
| `DIST_TO_BOUNDARY` | 0.050 | **review** — an established input, nearly inert |
| `ACCEL_Y` | 0.014 | **drop for M1** — coordinated turns pin it at zero (std 0.013). Keep for M2, where sideslip may carry real information |

⚠️ Contribution is a **screen**, not a verdict. The T068 ablation on the gen-608 elite is the verdict, it is
still available, and it should run **before** anything is dropped.

**The reshape worth trying** (operator's idea, and it is the best one here): replace the binary in-track
flag with the **track-score gradient × streak multiplier**.

- A binary flag says *"you are/are not scoring"* — a state label. A controller can only switch on it.
- A **gradient** says *"move this way to score more"* — a local improvement direction, in body frame,
  which is directly actionable and is what a controller actually needs.
- It is analytically available: the step score is a Lorentzian in (along, lateral) from
  `FitnessComputer::decomposeStepScore`, so ∂score/∂position has a closed form; project it into body axes.
- Multiplying by the streak multiplier weights it by **how much reward is currently at stake** — the
  policy learns not just which way to improve but when improving is worth most.

Operator's own framing is the key insight: *"streak was a crude proxy for rewarding in-track range."* The
gradient is the un-crude version. And note it is the same mathematical object as the advantage in (B) — a
**local** improvement signal rather than a **global** state label. A and C converge.

## Instrumentation owed either way (do it while no bake is running)

We are re-baking M1 regardless, so the format break is free right now:

1. **Emit the FULL input vector in `dmp-dump`** — today only 9 of 42 slots have CSV columns, which is why
   the contribution table above covers 9. Full coverage unlocks: contribution ranking for every input, the
   behavior-clone corpus (042), and any per-step study.
2. **Record `Es` and `Ps` per tick** — cheap, derived from state already recorded.
3. Both need a rebuild, which is why they were deferred while t1 ran. That constraint is gone.

## Suggested order

| # | step | cost | what it answers |
|---|---|---|---|
| 1 | T068 ablation on the gen-608 elite | hours | **H1a** — does the policy use the envelope inputs? Settles C's drop list with evidence |
| 2 | `Es`/`Ps` off existing dmps | hours | is the spiral bleeding energy? Is `Ps/Ps_max` discriminating? |
| 3 | dmp-dump full input vector + `Es`/`Ps` | ~a day | unlocks everything downstream |
| 4 | score-gradient input + potential-based shaping | days | the reshape — new M1 bake |
| 5 | critic head → per-tick advantage | weeks | the real per-tick RL move; reuses 041's extra-head machinery |

⚠️ Steps 1–2 spend **no bake time** and could change steps 3–5. They should go first.

## What could make this wrong

- **The gradient may be too informative.** Handing the policy ∂score/∂position is close to handing it the
  answer; it may learn to follow the gradient greedily and lose the anticipation that the recurrent layer
  was building. Watch for the accelerating-curve shape disappearing.
- **`Ps_max(state)` may be awkward to obtain** from the FDM without a lookup or a fit. If so, `Ps` relative
  to a *learned* baseline (B) is the fallback, at the cost of needing the critic first.
- **Potential-based shaping preserves the optimum, which includes the spiral.** If the spiral is genuinely
  objective-optimal, shaping alone will not remove it — only changing the objective (or the airframe) will.
  Shaping makes learning *faster*, not the target *different*. Do not expect it to fix strategy.

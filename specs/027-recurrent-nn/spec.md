# 027 — Recurrent / history-aware NN architectures

**Status**: DRAFT — baseline skeleton. Full spec to be developed on
the `027-recurrent-nn` branch.

**Origin**: 026 (ACRO PID delegation) closed NO-GO — downstream
smoothing can't fix bang-bang because the NN itself has no memory.
See [`specs/026-nn-temporal-state/findings.md`](../026-nn-temporal-state/findings.md)
for the evidence and the reasoning that leads here.

## Core hypothesis

> Bang-bang is a consequence of evolutionary pressure on a **stateless
> feedforward** NN: point-in-time inputs → point-in-time outputs, no
> concept of "continuity" or "last tick." To exit the bang-bang
> plateau, the NN itself must gain the ability to represent and
> reason about its own history — either explicitly (past outputs as
> inputs) or implicitly (persistent hidden state).

## Approach — two tracks, cheapest-first

### Track A2 — output history window as inputs

Smallest architectural change. Feed the last N ticks of NN output
back in as inputs next tick (plus the current sensor inputs). This
gives the NN direct visibility of its own trajectory through its
output space — it can learn to reason about "was I just saturating?"
or "am I flipping sign?"

**Why first**: cheapest falsification of the core hypothesis. If
this alone moves the needle materially (dCtrl off 1.0, |out| off
2.2), we have a clean answer without the full recurrent rewrite.
If it doesn't, we've ruled out "just give it history" and earned the
right to do the real architectural work.

**Open design questions** (to be decided on-branch):
- N = how many past ticks (1, 3, 5, 10)? Each costs NN_INPUT_COUNT+3.
- Include previous inputs (e.g. target direction) too, or just
  previous outputs?
- Weighting in fitness: anything explicit, or purely evolutionary
  pressure?

### Track D — true recurrent architecture

Persistent hidden state *inside* the NN: `h_t = f(W_hh h_{t-1} + W_xh x_t)`,
output `y_t = g(W_hy h_t)`. The NN learns what to remember and how
to use it. State resets on span start.

**Why second**: bigger engineering cost — changes the forward pass,
the weight serialization format, the weight-count accounting. But
it's the architecturally honest answer to "the NN has no memory":
*give it memory*.

**Open design questions**:
- Simple RNN vs GRU-lite vs LSTM? Simple RNN is easiest, GRU gates
  help stability, LSTM is overkill for our input/output dims.
- Hidden state size: current topology 33→32→16→3. Natural choice
  is to make the first hidden layer (32) recurrent.
- Weight count impact: adding W_hh (32×32) = 1024 extra weights →
  NN_WEIGHT_COUNT 1667 → 2691 (~60 % larger search space, ~60 %
  slower evolution per gen at constant compute).
- Evaluator rewrite scope: single-pass → per-tick with carried state.
- Xiao implications: C-generated NN on the xiao needs the same
  recurrent forward pass; not free but tractable.

## Research agenda (pre-implementation)

Before committing to specific architecture details, survey the
current state-of-the-art in recurrent policies for physics control
(the user's phrase: "is this phys AI state of the art?"). Topics:

- Evolved RNNs (NEAT variants, CMA-ES with recurrent topology)
- Deep-RL with recurrent policies (PPO-LSTM, SAC-RNN) — what
  architectures are they picking and why
- GRU vs LSTM vs simple RNN in short-horizon tight-loop control
- Recent arXiv work on memory-augmented policies for
  quad/fixed-wing flight

Output: `research.md` on the 027 branch summarizing what maps to our
evolved-NN + genetic-evolution constraint set and what doesn't.

## What carries forward from 026

**Retained** (already in the tree, just retargeted as observational):
- `PidInternals` struct on `AircraftState`
- 13 `rateCmd*/rateAch*/pid*` columns in `data.dat`
- `ACRO_MAX_RATE_*` constants
- Cereal/RPC payload for PID internals

**Removed** in 027's first commit (math-null the PID, drop unused
code):
- PID math in `crrcsim/.../inputdev_autoc.cpp` → passthrough
- LPF/integrator globals + span-start resets
- `ACRO_FF_*`, `ACRO_P_*`, `ACRO_I_*`, `ACRO_PID_SCALE`,
  `ACRO_*_LPF_HZ` constants
- `include/autoc/eval/acro_pid.h`
- `tests/acro_pid_tests.cc`

Why math-null instead of full-revert: the serialization and column
schema change already landed. Oscillating the cereal payload twice
in one week for the same data is worse than keeping the columns
alive as diagnostics.

## Out of scope

- **Fitness-surface changes** (Pareto, smoothness penalty). 026
  findings show open-loop smoothing "tends to look like rate-
  limit" — not the intervention we need first. Can revisit if A2/D
  alone don't land.
- **Flight hardware changes.** xiao stays on INAV MANUAL mode as it
  has been. No handoff-mode changes until sim signal is strong.
- **New craft variations.** 025 remains blocked; we don't complicate
  the training signal until we know the architecture can produce
  a non-bang-bang controller.

## Validation (to be fleshed out)

Same gate as 026 originally had, with the organic-call posture:

- **Primary signal** — aggressiveness off plateau: dCtrl measurably
  below cadence7's ~1.0 and |out| below ~2.2 at matched gen count.
- **Secondary signal** — fitness no worse than cadence7 at matched
  gen count. If it's worse, the memory is costing more than it's
  earning.
- **Sanity** — tier-0/1 evals must PASS (no regression vs baseline).

## Relationship to 025

025 (craft variations) is now blocked on 027. If 027 A2 or D
produces a non-bang-bang controller, 025 becomes the natural
follow-on for robustness. If 027 also falls short, 025 gets
revisited with a different framing — at that point the problem is
probably upstream of both architecture and variations (fitness
surface, training objective, or the core "evolve feedforward weights
against a tracking score" premise).

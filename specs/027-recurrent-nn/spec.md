# 027 — Recurrent / history-aware NN architectures

**Status**: CLARIFIED — 5/5 open questions resolved 2026-04-24;
ready for `/speckit.plan`.

**Origin**: 026 (ACRO PID delegation) closed NO-GO — downstream
smoothing can't fix bang-bang because the NN itself has no memory.
See [`specs/026-nn-temporal-state/findings.md`](../026-nn-temporal-state/findings.md)
for the evidence and the reasoning that leads here.

## Clarifications

### Session 2026-04-24

- Q: Incentive axis — is extending lexicase with a smoothness test case in scope for 027? → A: In scope as primary incentive; lexicase extension (NOT a separate Pareto selection mode).
- Q: Test matrix scope — full 5-step escalation or MVP? → A: MVP — step #0 baseline-sanity + step #3 primary bet (D-simple + C2-via-lexicase-smoothness). A2-alone, A2+C2, and D-alone reserved; decide #4 diagnostic after #3 completes.
- Q: D-simple hidden-layer size — 16-wide (layer2 recurrent) or 32-wide (layer1 recurrent)? → A: 16-wide layer2 (W_hh 16×16 = 256 extra weights, 1667 → 1923, +15 % budget). 32-wide reserved as escalation.
- Q: Hidden-state reset semantics — when does h_t clear/advance? → A: Zero on span start; advance only on NN eval ticks (~10 Hz). h_t does NOT advance on intermediate outer-frame ticks (~20 Hz) when NN doesn't fire.
- Q: Xiao deployment timing — when port the recurrent forward pass to `nn_program_generated.cpp`? → A: After step #3 passes the sim gate (aggressiveness off cadence7 plateau + fitness ≥ cadence7 at matched gen). Sim-first discipline; don't port until sim proves the architecture works.

## Core hypothesis

> Bang-bang is a consequence of evolutionary pressure on a **stateless
> feedforward** NN: point-in-time inputs → point-in-time outputs, no
> concept of "continuity" or "last tick." To exit the bang-bang
> plateau, the NN itself must gain the ability to represent and
> reason about its own history — either explicitly (past outputs as
> inputs) or implicitly (persistent hidden state).

## Approach — primary bet: D-simple + C2-via-lexicase-smoothness

The MVP for 027 is **a single combined experiment** on both axes:
- **D-simple** (architecture): one recurrent layer inside the NN
  with persistent hidden state `h_t = f(W_hh h_{t-1} + W_xh x_t)`,
  output `y_t = g(W_hy h_t)`. State resets on span start (see
  reset semantics clarification below). Simple RNN (not GRU / LSTM
  — per research bias, gates add evolutionary search dimensions
  without gradient signal to justify them).
- **C2 via lexicase smoothness test case** (incentive): extend
  existing lexicase selection with an additional per-scenario
  smoothness test case (score computed from `Δout` sequence during
  evaluation). No new selection mode — it's a test case in the
  lexicase pool.

The combined "#3" in the research matrix is our primary bet —
literature (research.md §2) strongly suggests memory alone without
incentive regularization amplifies rather than reduces action
variability, and our prior open-loop smoothing attempts (B, C1, E)
confirm the symmetric failure mode from the incentive-only side.

Reserved experiments (not in MVP scope, decide after #3 result):
- **#1 A2 alone** (output history window as inputs, feedforward) —
  skipped; memory-alone-style attempts in project history (B, E,
  C1) underwhelm and the literature bias is the same direction.
- **#2 A2 + C2** — skipped for same reason.
- **#4 D-simple alone** (no C2) — diagnostic-only; decide whether
  to run AFTER #3 completes, based on what question remains.
- **D-gated GRU-lite, D-ESN, C1 α-penalty** — reserved, revisit
  only if #3 plateaus.

**Topology (decided)**: `33 → 32 → 16-recurrent → 3`. The 16-wide
second hidden layer gains a self-connection matrix `W_hh` (16×16 =
256 extra weights). Total NN_WEIGHT_COUNT: 1667 → **1923** (+15 %).
32-wide recurrent variant reserved as escalation.

**State semantics (decided)**: hidden state `h_t` is zero on span
start and advances **only** on NN evaluation ticks (~10 Hz). It
does NOT advance on intermediate outer-frame ticks when the NN
doesn't fire. This keeps "memory" aligned with what the NN
actually decides/observes, matches xiao MSP cadence, and is the
conventional recurrent-policy semantics.

**Xiao deployment (decided)**: port only after step #3 passes the
sim gate. Sim-first discipline — no recurrent forward pass in
`xiao/src/generated/nn_program_generated.cpp` until sim data
confirms D-simple + C2 produces a non-bang-bang controller.
Matches the project pattern used by 024 and 026.

## Research grounding

Done — see [`research.md`](./research.md). Key takeaways that
shaped the decisions above:

- **Literature bias**: memory alone (LSTM without action-rate
  regularization) documented to *amplify* action variability, not
  reduce it — supports the combined D + C2 bet.
- **A3 (input derivatives/history) is already implemented** via
  `NNInputs`' 6-sample target history + `closing_rate` + gyro
  rates; bang-bang emerges despite these. Missing piece is
  **self-memory**, not more world-memory.
- **GRU-lite, ESN held in reserve**: gates add search dimensions
  without gradient signal; ESN adds reservoir hyperparameter
  surface. Direct-to-evolve simple RNN is the right first bet.

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

- **Pareto selection as a new selection mode.** Lexicase stays the
  selection primitive; C2's contribution is an *additional test
  case* (smoothness score per scenario) added to the lexicase
  pool — not a switch to Pareto-dominance selection.
- **C1 α-penalty in fitness (open-loop rate penalty).** Prior
  experience shows this "tends to look like rate-limit mush"
  without memory. Reserved as fallback if C2-via-lexicase proves
  infeasible; not a primary 027 experiment.
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

025 (craft variations) is now blocked on 027. If D-simple + C2
produces a non-bang-bang controller, 025 becomes the natural
follow-on for robustness. If 027 also falls short, 025 gets
revisited with a different framing — at that point the problem is
probably upstream of both architecture and variations (fitness
surface, training objective, or the core "evolve weights against a
tracking score" premise).

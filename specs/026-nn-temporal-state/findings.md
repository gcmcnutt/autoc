# 026 findings — NO-GO, and what we learned

**Status**: CLOSED — NO-GO. 026's hypothesis disproven; infrastructure
retained. See [spec.md](./spec.md) for original scope, this doc for
outcome and carry-forward.

**Date**: 2026-04-24

## Hypothesis (from spec.md)

> An ACRO rate PID, applied between the NN output and the FDM surface
> input, would absorb the fast inner-loop dynamics. The NN — now
> commanding body rates instead of surface deflections — would shed
> the evolutionary pressure to bang-bang its outputs, and settle on a
> smoother "stretchy" control policy that tracks the rabbit tightly
> without instantaneous full-throw surface slams.

## What was built

- **CRRCSim ACRO rate PID** (pitch/roll, yaw passive) — FF + P + I
  via [`include/autoc/eval/acro_pid.h`](../../include/autoc/eval/acro_pid.h)
  shared helper. Empirical 021-era gains (FF=50, P=40, I=15, scale=350),
  sim-cadence LPFs (40 Hz gyro, 20 Hz dterm dormant), anti-windup ±10
  rad. ([crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp))
- **Diagnostics end-to-end** — `PidInternals` struct on
  [`AircraftState`](../../include/autoc/eval/aircraft_state.h), 13 new
  columns in `data.dat`, cereal-serialized through the RPC/worker path.
- **Unit test for the PID math** —
  [`tests/acro_pid_tests.cc`](../../tests/acro_pid_tests.cc), 7 cases.
- **Smoke validator** — [`026_acro_smoke.sh`](./026_acro_smoke.sh)
  post-run data.dat checker (schema + tracking + saturation).

## What happened — pid1 run

Full 400-gen training, pop 3500, longSequential, variations-enabled
(parity with cadence7). Log:
`logs/autoc-026-pid1.log`. Output:
- [`autoc-026-pid1-fitness.png`](./autoc-026-pid1-fitness.png)
- [`control_aggressiveness_pid1.png`](./control_aggressiveness_pid1.png)

### Fitness (lower = better)

| Run | Final fitness (gen 400) | Notes |
|---|---:|---|
| cadence7 (MANUAL, pre-026) | **−35,951** | proven baseline |
| hb1-adjust4 (older sim) | −29,358 | |
| **pid1 (ACRO PID)** | **−27,045** | **worst of the three** |

pid1 finished **~25 % worse than cadence7**. Not a small gap; not a
noise artifact.

### Control aggressiveness (bang-bang signature)

Mean across 5 starter paths at plateau:

| Run | mean dCtrl (stick speed) | mean \|out\| (magnitude) | final sigma |
|---|---:|---:|---:|
| cadence7 plateau | ~1.00 | ~2.20 | 0.08 |
| **pid1 final** | **~1.60** | **~2.54** | 0.08 |

pid1 is **more bang-bang than cadence7**, not less. Through training,
pid1's aggressiveness *increased* (gen 126: 1.22/2.36 → gen 333:
1.58/2.54 → gen 400: 1.60/2.54). Sigma collapsed to 0.08 normally.
Evolution converged — just onto a worse place.

### PID itself

The PID math is correct (unit tests pass). During pid1, median
rate-tracking error was ~16 % of max on roll and ~45 % on pitch, with
pitch saturating 66 % of ticks — the NN was asking for pitch rates the
aircraft couldn't deliver, so the PID stayed pinned and authority was
capped.

## What we learned (the hard part)

### The root cause is upstream of the PID

Bang-bang is the product of three things *none of which 026 touched*:

1. **Stateless feedforward NN** — no memory → no "smooth continuation"
   concept. Each tick is decided in isolation.
2. **Fitness surface is point-accumulation** — rewards instantaneous
   tracking with no cost for switching. At saturation, the NN locally
   maximizes by flipping the sign instantly when target moves.
3. **No derivative-or-history inputs** — the NN sees a snapshot, can't
   anticipate trends or costs of change.

Putting a PID *downstream* of the NN output doesn't change any of
those. The NN will still evolve outputs that, in aggregate, look like
bang-bang — because nothing in the fitness or inputs makes it
evolve otherwise. With PID in the path, the NN just learned to bang
its *rate command* instead of its *surface command*. Same pathology,
one level of indirection.

### PID adds a small lag that the NN can't compensate for

With PID in the loop, there's a finite-delay response between NN
output and surface effect. A stateless NN has no way to account for
this — the tracking-vs-time gradient gets slightly distorted. Not a
killer on its own, but plausibly part of why pid1 lands below
cadence7 in fitness even at the same gen count.

### Prior art: we've explored most of the "downstream of NN" fixes

From git history and conversation context:

| Approach | Outcome |
|---|---|
| A1 — previous NN output as input (older) | Tried; "unclear how much it helped" |
| B — rate-limited output at consumer | Tried; "just mushes around" |
| C1 — smoothness penalty in fitness, open loop | Tends to look like B |
| C2 — Pareto selection (tracking, smoothness) | Not tried, but open-loop same issue |
| E — delta output (NN outputs Δcmd) | Tried early; "result is more like B" |
| **This (026) — external PID** | **NO-GO: worse fitness, more bang-bang** |

The common failure mode: **smoothing the NN's output without giving
the NN any ability to see or reason about its own history**. The
evolution keeps finding the saturation strategy because — from
inside the NN's one-tick-at-a-time view — that *is* the locally
optimal move.

## What's next — feature 027

The conclusion: changing what happens **to the NN's output** doesn't
work. We have to change **what the NN is** — give it persistent
state, so that "smoothness" becomes a property it can learn to reason
about.

Two tracks for 027, in escalation order:

1. **A2 — output history window as inputs.** Previous N ticks of NN
   output fed back as inputs next tick. Smallest change that gives
   the NN visible self-history. Cheap falsification.
2. **D — true recurrent architecture.** Hidden state that persists
   across ticks; the NN learns *what* to remember and *how* to use
   it. Bigger engineering cost, addresses root cause.

See [specs/027-recurrent-nn/spec.md](../027-recurrent-nn/spec.md)
(draft) for the full plan.

## Carry-forward from 026

Kept, retargeted as observational diagnostics:
- `PidInternals` struct on AircraftState
- 13 `rateCmd*/rateAch*/pid*` columns in `data.dat`
- `ACRO_MAX_RATE_*` constants (used to express NN output in rate units)
- Cereal/RPC payload carrying PID internals

Removed in 027's first commit:
- PID math block in `inputdev_autoc.cpp` (replaced with passthrough
  + diagnostic capture)
- LPF/integrator state globals + span-start resets
- `ACRO_FF_*`, `ACRO_P_*`, `ACRO_I_*`, `ACRO_PID_SCALE`,
  `ACRO_*_LPF_HZ` constants
- `include/autoc/eval/acro_pid.h` and
  `tests/acro_pid_tests.cc`

Why null-out instead of full-remove: the serialization schema change
(PidInternals) already landed; thrashing the cereal payload twice in
one week for the same data is worse than keeping one set of columns
that now simply observe "what the NN commanded" and "what the
aircraft did" in rate units.

## Relationship to 025

025 (craft variations) remains BLOCKED — now by 027 rather than 026.
If 027 gets the NN off the bang-bang plateau, 025 is the natural
follow-on to explore robustness. If 027 also falls short, 025
probably gets revisited with a different framing.

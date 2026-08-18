# 043 — M2 tracking (seed)

**Created 2026-08-17** when 041 was scoped down to *"a fresh full M1 toolchain, flown"*. This holds the M2
work moved out of it. **Not started.**

## Why M2 moved

⚠️ **M2 was not blocked by M2 work — it was blocked by M1.** Three findings inside 041:

1. **The M1 source was not good enough to chase with.** t1 reached `pctInStreak` 16.1% against the prior
   M1's 30.9% tracking occupancy, and converged on a tight spiral that is objective-optimal under
   no-future inputs but is not the behaviour we want a tracker built on.
2. **The objective was missing its main term.** The policy cannot observe its own energy — the input vector
   carries `AIRSPEED` but no altitude — so every prior energy objective muted the whole regiment. M2
   inherits that verbatim.
3. **The datum chain is unverified** — eleven hops, four references, at least one open reconciliation.

Building a tracking problem on that foundation would have measured the foundation, not the tracking.

## What moved here from 041

| item | 041 refs | state |
|---|---|---|
| Predictor head — offline verdict, re-target or retire | T081–T088, FR-024…FR-027 | ⚠️ **T082's blind-gap distribution is a LOWER BOUND** — measured at 120°×90°, the real lens is 97.3°×60.8° |
| M2 bake, innovation channels (FR-025c–f), output topology 7↔3 | T089–T094, FR-005a | not started |
| Repoint tracker at the new M1 source; novel-geometry read | T095–T098 | **blocked on 041 producing a pinned M1** |
| M2 smoke | T060 | needs a v-current M1 source dmp |

## What 041 hands over

- A pinned M1 source in the current schema — **the actual dependency**.
- The energy observation + `Ps` axis, which M2 inherits (`CraftCommonInputs` is shared).
- A validated datum chain, proven by an M1 flight.
- The measured camera model (97.3° × 60.8°, equidistant, 0.304 °/px).

## Sequencing

**041 (better M1, flown) → 042 (physics/camera, child of 031) → 043.**

042 sits between because the camera constants it produces feed M2's sensor model, and measuring them
depends on neither. ⚠️ Do not start 043 before 041 pins an M1 — the source dependency is hard, and the
`EvalResults` version bump means an older source dmp cannot be read at all.

## Known traps carried forward

- **`FR-005a`** permits the tracker innovation channels as the one post-A1 layout change — legal because
  `TrackerInputs` has a separate genome and 040's T023 serialize split means no M1 source re-bake.
- **M2 has no direct distance or bearing** — only two beacon bearings and their separation, range inferred
  from span. That asymmetry is the whole M1↔M2 difference and should shape the input work.
- **Contribution/weight screens mis-rank inputs in both directions** (041 TA01 got it wrong three times).
  Ablate the set you intend to remove; ablation is non-monotonic.

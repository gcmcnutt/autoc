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

## ⚠️ The sensor model is NOT settled — a 1.56 mm lens is on order

Operator 2026-08-18: *"042 has ordered a 1.56mm lens which is close to 120deg fov. So that part of m2 is
dependent on range and response findings anyway."*

So M2's perception scope cannot be fixed yet. The tree currently carries the **1.8 mm** measurement
(97.3° × 60.8° derived from 320×200 @ 0.304 °/px), and 031 retired the single-fisheye-at-120° assumption
**for that lens**. A 1.56 mm near 120° would partially reverse that, which changes:

- the blind-gap distribution — 041's T082 figure is a **lower bound** measured at 120°×90°, and a genuinely
  120° lens moves reality back toward it;
- the **predictor's value**, since its whole justification is bridging blind gaps (fewer/shorter gaps ⇒ less
  to predict);
- whether the **birded pair** is needed at all — its main justification was reaching 120°.

⛔ **Do not spec M2's perception against either lens until 042 measures the 1.56 mm and reports range +
response.** Waiting is cheap; specifying twice is not.

## Sequencing

**041 (better M1, flown) → 042 (physics/camera, child of 031) → 043.**

042 sits between because the camera constants it produces feed M2's sensor model, and measuring them
depends on neither. ⚠️ Do not start 043 before 041 pins an M1 — the source dependency is hard, and the
`EvalResults` version bump means an older source dmp cannot be read at all.

## ⭐ Phase 1 inherits M1's fidelity — the hard perception work is phase 2

Operator 2026-08-18: *"Score grad in Xiao in m1 is virtual so we can do that. In m2 it'll proxy from camera
range estimation. Should be an interesting 043 study… Wait. First m2 flights will also be virtual so roughly
the same fidelity as today's m1 virtual paths."*

Because **M2 phase 1 is a virtual target with a synthetic camera**, the `SCORE_GRAD_*` input stays **exact**
there — the xiao computes it from a target it knows, exactly as M1 flight does. Nothing new is owed to fly
phase 1.

**The camera-proxy gradient — estimating ∂score/∂position from span-based range — is a phase-2 study.** That
is a genuine piece of research (range from span is the same inference the whole M2 sensor model rests on),
and it is now **deferred behind a flyable phase 1** instead of gating it.

⚠️ Read this together with the lens block above: phase 2's perception scope depends on 042's range and
response findings *and* on which lens is chosen, so specifying the proxy before 042 reports would be
specifying against an unknown sensor.

## Known traps carried forward

- **`FR-005a`** permits the tracker innovation channels as the one post-A1 layout change — legal because
  `TrackerInputs` has a separate genome and 040's T023 serialize split means no M1 source re-bake.
- **M2 has no direct distance or bearing** — only two beacon bearings and their separation, range inferred
  from span. That asymmetry is the whole M1↔M2 difference and should shape the input work.
- **Contribution/weight screens mis-rank inputs in both directions** (041 TA01 got it wrong three times).
  Ablate the set you intend to remove; ablation is non-monotonic.

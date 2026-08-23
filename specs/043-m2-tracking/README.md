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

---

## ⛔ TWO GATES BEFORE ANY 043 RUN (added 2026-08-22)

### Gate 1 — the 042 camera parameters must have LANDED

Operator 2026-08-22: *"we will have some new camera params from the 042 modelling that occurs in parallel,
so be sure before we start any 043 runs we've received the camera updates."*

⛔ **Do not launch a 043 bake against the current camera model.** 042 is running in parallel and its output
is a changed perception front-end. A bake started before those land is spent: its results are attributable
to a camera model that no longer exists, and it cannot be compared to anything after.

Check before launching: the 042 camera constants are merged, and `beacon_config.h` / the camera-projection
parameters are the 042 values, not 040's.

### Gate 2 — ⚠️ AUDIT THE 46 TRACKER-SPECIFIC INPUT SCALES

⭐ **M2 inherits the shared block's normalization automatically and completely.** `TrackerInputs` embeds
`CraftCommonInputs` (the same struct M1 uses), and `gather_tracker_inputs` calls the same
`writeCraftCommonInputs`. So all 20 shared slots arrive with the 041 P2-8 scaling for free — quat,
airspeed, gyro, accel, Es, boundary-closure, dist-to-boundary, inward-body, score-grad.

⛔ **But P2-8 never touched the 46 tracker-specific slots, and at least two are explicitly raw.**
`nn_inputs.h` says of `beacon_pair_span`: *"in RADIANS … with no scaling, no normalization, no clipping."*
`span_rate` is raw rad/s.

⚠️ **The magnitudes are in the danger band.** With the measured 0.772 m beacon separation, the pair
subtends 0.154 rad at 5 m and 0.011 rad at 70 m — an implied spread of roughly **0.02–0.03**. That sits
alongside `DIST_TO_BOUNDARY` (0.036), `SCORE_GRAD` (0.062) and `SPECIFIC_ENERGY` (0.090) — precisely the
quiet band that 041 proved was **never selected on at all** until P2-8 rescaled it. On 041-t5, every
input's weight investment sat flat at ~1.0 for 475 generations and the network never differentiated.

⛔ **This is not a prediction; it is the same defect, one milestone over.** 041 spent three runs
(t4/t5/t6) blaming the objective and the ramp before measuring the inputs. Do not repeat that. **Measure
the tracker-input spreads on a real M2 tick set BEFORE the first 043 bake** and rescale by measured p95,
the derivation already used for `kEnergyScale_m`, `kScoreGradScale` and the four P2-8 constants.

⚠️ **Unmeasured as of this note.** The 040-t4 pre-break archive does not carry the beacon input columns
(its `spP1/2/3`/`spdR` are the aux span-PREDICTOR OUTPUTS, not inputs), so the figures above are a physical
estimate from the beacon geometry, not a measurement. Getting the real numbers needs a tracker `--csv-only`
dump — cheap, and it is the first thing to do here.

⭐ The durable fix is the backlog entry *"Formal input normalization — measured statistics, not
hand-derived constants"* (specs/BACKLOG.md). If that lands first, both gates collapse into it and the
tracker block can never drift raw again.

---

## M2 input-normalization strategy (2026-08-22) — reasoned, not yet measured

⭐ **The shared 20 are already correct and PROVEN identical.** `TrackerInputs` embeds `CraftCommonInputs`;
`writeCraftCommonInputs` is a single unconditional path with no mode branch, called identically from
`gather_pathgen_inputs` (`evaluator.cc:464`) and `gather_tracker_inputs` (`:652`), and nothing overwrites a
`common.*` slot afterwards. M1 and M2 cannot diverge on those without editing one function. A guard test
pins it.

**So the work is the 46 tracker-specific slots.** Current state and proposal:

| slot(s) | n | today | conditioning | proposal |
|---|---:|---|---|---|
| `beacon_l/r_cep` | 12 | `[0,1]`, 1.5 = invisible sentinel | ✅ already unit | **leave alone** |
| `target_tilt_sin/cos` | 2 | ±1 by construction | ✅ | **leave alone** |
| `time_since_seen` | 1 | `tanh(ticks·scale)`, `[0,1)` | ✅ | **leave alone** |
| `beacon_l/r_x` | 12 | raw rad, ±0.849 (half-FOV H) | ⚠️ ~0.4 spread | scale — **see the isotropy trap** |
| `beacon_l/r_y` | 12 | raw rad, ±0.531 (half-FOV V) | ⚠️ ~0.3 spread | scale — **same constant as x** |
| `beacon_pair_span` | 6 | raw rad, 0.011–0.154 | ⛔ **~0.02–0.03** | **the real problem** |
| `span_rate` | 1 | raw rad/s | ⛔ unmeasured | follows span |

### ⛔ TRAP 1 — do NOT normalize x and y by their own half-FOVs

The obvious move is `x /= 0.849`, `y /= 0.531`. **That breaks a stated invariant.**
`camera_projection.h:90-93`: *"Both axes carry the SAME angular scale, so a given angular separation reads
identically at any orientation (FR-002)."* Dividing by different constants makes a fixed angular separation
read differently depending on whether the pair is horizontal or vertical — which is exactly the property the
beacon-pair geometry depends on.

⭐ **Scale both axes by ONE constant.** The larger half-FOV (0.849) maps H to ±1 and leaves V at ±0.63,
preserving isotropy. A measured p95 of `|bearing|` over both axes jointly is the alternative, and matches
the `kEnergyScale_m` / `kScoreGradScale` / P2-8 derivation.

### ⛔ TRAP 2 — `beacon_pair_span` is a 1/distance cue, so linear rescaling barely helps

With the measured 0.772 m separation, span is `kBeaconSeparationM / range`:

| range | 5 m | 10 m | 20 m | 40 m | 70 m |
|---|---:|---:|---:|---:|---:|
| span (rad) | 0.154 | 0.077 | 0.039 | 0.019 | 0.011 |

**Half the dynamic range is spent inside 10 m.** Divide by a p95 of ~0.15 and the 20–70 m band still lives
in 0.07–0.13 of unit scale — better than today, but the far field, which is where reacquisition happens,
stays compressed. And the network has to learn `1/x` internally to recover range at all.

⭐ **PROPOSAL A (safe, P2-8-shaped)**: `span /= kSpanScale_rad`, p95-derived. Keeps semantics, fixes the
worst of the conditioning, low risk. Do this if 043 wants one variable changed.

⭐ **PROPOSAL B (better conditioned, changes semantics)**: reparametrize to **range**:
`r̂ = kBeaconSeparationM / span`, then scale by `kTargetDistScale_m` (26.0) — **the same constant M1's
`DIST` uses**. Likewise `span_rate` → **range-rate**, scaled by `kClosingRateScale_mps` (16.0), the same
constant M1's `CLOSING_RATE` uses.

⭐ **Why B is attractive beyond conditioning**: it makes M2's depth channel *dimensionally identical* to
M1's `DIST` / `CLOSING_RATE` pair. The tracker stops having to learn the reciprocal, and the two milestones
express range the same way — which matters if weights or intuition are ever carried across.

⚠️ **B's cost, stated honestly**: a singularity at `span → 0`, which is exactly the CEP-gated
invisible case (span is substituted to 0.0 when either CEP ≥ `CepGateThreshold`). B needs an explicit
sentinel — "no range estimate" must be representable and distinguishable from "very far". `time_since_seen`
already carries visibility, so the pairing is natural, but it must be designed, not left to a divide.

⛔ **B is a REPRESENTATION change, not a normalization change. Do not bundle it with A.** 041's whole lesson
was that two changes at once make a run unattributable — t4 moved the objective and the inputs together and
cost three runs to untangle. If both are wanted, A first, then B as its own single-variable read.

### ⚠️ ALL OF THE ABOVE IS UNMEASURED

The spreads quoted for the bearings are inferred from FOV limits and the span figures from beacon geometry —
**not** from a tick set. The 040-t4 pre-break archive does **not** carry beacon input columns (its
`spP1/2/3` + `spdR` are the aux span-*predictor outputs*). ⛔ **First action here is a tracker
`--csv-only` dump and a spread table**, exactly as P2-8 did for M1 — 041 spent three runs blaming the
objective and the ramp before anyone measured the inputs.

⭐ If the *"Formal input normalization — measured statistics, not hand-derived constants"* backlog entry
lands first, all of this collapses into it: the stats would be measured and serialized with the weights, and
the tracker block could never drift raw again.

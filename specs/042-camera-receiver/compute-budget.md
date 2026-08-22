# Compute budget for acquisition-under-motion (2026-08-22)

**Question (operator)**: *"do we have enough compute if this winds up being a larger n² search? Imagine in
the limit each pixel is a candidate and each track is possible and we are acquiring across a moving field
with high gain."*

**Answer**: brute force is **out by 14× at bench rates and ~15 000× at flight rates**. But brute force was
never the plan, and the structure that avoids it also happens to be the structure spec §1 already
prescribes. The binding constraint turns out not to be MACs — it is **how many candidate detections per
frame the front end hands to the track former**. That number must stay under ~1000–2000, and keeping it
there is what ego-motion registration buys.

> **Every number here is rated at 288 fps / 120 Hz — see spec §2.5.1 for the scaling table.** Rate limits
> scale ×1.667 and hypothesis counts ×0.36 if the platform ever reaches 480 fps, so these are floors.

## Measured, not assumed

`beacon_bench --kernels` on the Pi 5, single-threaded:

| kernel | throughput |
|---|---|
| `mac_i16` (scalar) | **5.07 G MAC/s** ← the correlator's unit of work |
| `reduce2` scalar → NEON | 1.24 → **11.7 G op/s** (×9.4) |
| `reduce4` NEON | 18.2 G op/s |
| `hipass M2` NEON | 0.672 G op/s (memory-bound) |

Two things follow. The correlator datum is **scalar**, and the reduce kernels show NEON is worth ~9× on
this machine — so a NEON correlator plausibly reaches 20–30 G MAC/s. That is the **single largest
un-cashed lever** in the whole budget and it costs no architecture.

## The brute-force cost

Per **velocity hypothesis**, full field, full word (74 frames, 64 000 M2 px):

| stage | cost |
|---|---|
| shift-and-accumulate frames into 31 chip bins | 4.8 M ops |
| correlate 31 phases × 31 chips × 2 codes, per pixel | **123 M MAC** |
| **total** | **128 M** → 25.2 ms on 1 core, 8.4 ms on 3, **2.1 ms on 3 with NEON** |

*(123 M MAC independently reproduces spec §1's "≈123 MMAC per pass" — a useful consistency check on both.)*

Hypothesis count is set by resolution — a velocity error must not smear more than ~1 M2 px across the
integration, so δv = 1 px / word = 3.88 px/s — and it grows as **V²**:

| rate to cover | bins/axis | hypotheses | 3 cores scalar | 3 cores NEON |
|---|---|---|---|---|
| ±15 °/s (measured hand motion) | 25 | 650 | **5.5 s** | 1.4 s |
| ±30 °/s | 51 | 2 600 | 21.8 s | 5.5 s |
| ±100 °/s | 170 | 28 885 | 243 s | 61 s |
| **±500 °/s** (039 roll transients) | 850 | 722 127 | **6 066 s = 1.7 h** | 1 517 s |

Against the §3 relock bar of **400 ms** — 6.1 G MAC on 3 cores — the ±15 °/s search needs 86.4 G MAC:
**14× over.** The flight case is four orders out. So yes: taken literally, the operator's limiting case is
not merely expensive, it is unreachable by any constant factor. NEON does not save it; a Jetson does not
save it. **The V² term has to be removed, not out-run.**

## What removes it

**Ego-motion registration.** Estimate the global frame-to-frame shift first; then the residual velocity to
search is the target's motion *relative to the scene*, not the camera's. On the bench with a static emitter
that residual is ≈0 — the entire 15–18 °/s measured in `pan2` is ego-motion. In flight it is the *relative*
motion of two aircraft, which is far smaller than body rates (039: pitch RMS 128–141 °/s, roll to 500 °/s).

| residual search | hypotheses | 3 cores scalar | 3 cores NEON |
|---|---|---|---|
| ±1 px/word (3×3) | 9 | **75.6 ms** | 18.9 ms |
| ±2 px/word (5×5) | 25 | 210 ms | 52.5 ms |

**Both fit inside the 400 ms relock bar.** Registration is therefore not an optimisation — it is the
enabling architecture, worth ~75× where NEON is worth ~4×.

> ⚠️ **SUPERSEDED 2026-08-22 by measurement — see [results/stage1-K-census.md](results/stage1-K-census.md).**
> This section argued K was the binding constraint and that registration's job was to keep it small. K has
> now been measured at **276–1292**, comfortably inside the budget in every condition including motion, and
> it *falls* as a fraction of the plane under motion because the 4×-mean gate is self-normalising. The real
> failure is a **ranking/sensitivity** one: the beacon ranks ~126th (still) / ~230th (moving) among
> detections and only the **top 3** are seeded, so it enters the bank on 15.9 % / **6.2 %** of passes. The
> arithmetic below is still correct as arithmetic; its conclusion about what binds is not.

## The constraint that actually binds: candidate count

The plan is not a velocity sweep. It is spec §1's **detect → proto-track → decode-along-track**, where the
velocity comes from the detections themselves rather than from a grid. Cost is then set by K, the number of
candidate detections per frame, because pairing detections across a time baseline to form proto-tracks is
**O(K²)** and each surviving track costs ~2 000 MAC to decode:

| K (detections/frame) | candidate tracks | MAC | verdict |
|---|---|---|---|
| 100 | 10 000 | 20 M | trivial |
| 1 000 | 1 M | 2 G | fits the 6.1 G budget |
| 2 000 | 4 M | 8 G | marginal |
| 5 000 | 25 M | 50 G | **over** |
| 64 000 (every pixel — the operator's limit) | 4×10⁹ | 8×10¹² | hopeless |

**So the design constraint is K ≲ 1000–2000 per frame**, and the operator's limiting case is 30–60× beyond
it. Three mechanisms hold K down, and they compose:

1. **Registration** — a static background stops blinking once the frame is stabilised, which is precisely
   what fails today (measured: the blink detector is swamped under motion because *everything* differs
   frame-to-frame).
2. **Spatial high-pass** — already built; passes point sources, rejects extended structure.
3. **The blink threshold** — already built; 4× the field's mean |diff|.

**Owed measurement**: K has never been measured. `acquire_pass` currently returns at most 3 seeds by
policy, so the *underlying* detection count is unknown and could be anywhere. Measuring K — before and
after registration, on `pan2.bcnr` — is the cheapest high-value experiment available and it decides whether
the RANSAC stage is viable as designed.

## The sky case, and what it does to AHRS

Two regimes, and they want different things:

- **Cluttered (bench, ground background)** — many candidates; registration is what keeps K small.
- **Clean (sky)** — few candidates, possibly only the beacon; K is small for free, and RANSAC recovers
  velocity directly with no ego-motion estimate needed.

But **visual registration needs texture, and sky has none.** That is the one case where the mechanism this
budget depends on is unavailable — and it coincides exactly with the 039 body rates where brute force is
four orders out. §2.3 currently frames AHRS as optional feed-forward, *"built last, measured not assumed"*.
This budget says something sharper for the flight article specifically: against a featureless sky at flight
body rates, **AHRS is not an accelerator, it is the only available source of the term that makes the search
tractable** — unless the clean-sky RANSAC path proves sufficient on its own, which is exactly what a
sky-background clip would settle.

Nothing here changes the 042 build order (AHRS is still 043). It changes what 043 is *for*.


---

## Addendum 2026-08-22 — RANSAC is out, track-before-detect is in

The operator pushed back on RANSAC and was right. RANSAC proto-tracks fit a **straight line over a word**,
but a word is 258 ms and at a 500 °/s roll that sweeps 129°. Cross-track plus roll with both platforms
moving traces an arc, so a line-fitter rejects the target as an outlier from its own trajectory. The
replacement is **track-before-detect**: propagate hypotheses frame-by-frame through an acceleration-bounded
reachable cone, accumulating code correlation along each path, with no global motion model — the operator's
*"little triangle guesses where last known point is a vertex."*

**The reason it is cheap, which is the non-obvious part:**

| | |
|---|---|
| cone with **no** velocity state, &#124;v&#124; ≤ 500 °/s | ±5.7 M2 px → 121 cells/frame |
| cone **with** a velocity state, at 10 000 °/s² | **0.198 M2 px — sub-pixel** |

Once a hypothesis carries velocity, the per-frame reachable set is bounded by ACCELERATION, and acceleration
is small over 3.47 ms. **Propagation is nearly free; the entire cost is in not-yet-knowing velocity.**

| formulation | cost | verdict |
|---|---|---|
| brute-force velocity sweep (±15 °/s) | 5.5 s per attempt | out |
| full position×velocity lattice, unpruned | 104–431 G ops/s | out |
| **beam TBD, 100 k particles** | **0.26 G ops/s** | fits (15.2 available) |
| beam TBD, 1 M particles | 2.59 G ops/s | fits |

Even allowing 20× for real per-particle work (sampling, bin accumulation, incremental scoring), 100 k
particles is ~0.6 G ops/s. **The affordable population is large.** So the design problem is not throughput —
it is **initialisation**: seeding density, pruning rule, score gate. That is where the effort goes (T050).

### Ego-motion is a *range-dependent* hint, not a uniformly weak one

The operator called it "a weak hint" because the target moves too. True at close range, and the pattern is
favourable:

| geometry | target relative rate | body rate (039) | ego-motion is |
|---|---|---|---|
| 13 m/s crossing at 3 m | 248 °/s | 128–500 °/s | comparable — weak |
| same at 30 m | 25 °/s | 128–500 °/s | dominant |
| same at 100 m | 7 °/s | 128–500 °/s | overwhelming |

**It is strongest exactly where it is needed most** — long range, low SNR, longest integration, least
tolerance for smear. Close in, where it is weak, SNR is high and integration can be short instead. Weight it
by range/q rather than discounting it.

### On evolving the estimator

Raised by the operator; the objection they raised themselves is the correct one. This is an *estimator*, and
its objective (a detection / false-alarm operating curve) is a clean scalar, so search is applicable in
principle. The blocker is generating enough honest variety — and **`inject.c` (T045) is the answer**:
injecting a synthetic coded point source into REAL recorded backgrounds gives real clutter, real ego-motion
and real sensor noise with exact truth, SNR/rate/range dialable, and a space as large as the clip library.

Verdict: searching over **parameters** (cone width, pruning threshold, seeding density, score gate) becomes
defensible once inject.c exists. Evolving **structure** should be avoided — it fits the injector's model of a
target rather than reality — and either way held-out real clips are the only honest test. **After** a working
TBD, not instead of one.

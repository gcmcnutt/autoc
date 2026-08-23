# Noise ladder — rung 0: the dark baseline (2026-08-22)

Operator's design: establish a solid first-acquire with the scene as clean as it gets, then **add background
noise progressively** and watch what breaks. This is rung 0. Geometry is fixed from here on — **only
illumination changes** — which is what makes the rungs comparable, and what the earlier clips lacked.

## Rung 0 — 200 W off, bench lamp off

**Live acquisition, 5 × 15 s runs:**

| run | first lock | present | q p50 | drops |
|---|---|---|---|---|
| 1 | 0.75 s | 93 % | 1.00 | 1 |
| 2 | 0.35 s | 96 % | 0.99 | 2 |
| 3 | 0.40 s | 95 % | 0.99 | 2 |
| 4 | 0.30 s | 95 % | 1.00 | 2 |
| 5 | 0.35 s | 97 % | 1.00 | 1 |

**First lock: median 0.35 s, range 0.30–0.75 s, 5/5 locked.** Compare the same measurement this morning:
median **2.95 s**, range 2.10–6.95 s.

**60 s reference clip** (`/data/dark60.bcnr`, 17275/17275 frames, 0 dropped, 0/1200 deadline misses):

| | |
|---|---|
| first lock | **0.30 s** |
| present | **99 %** |
| drops | **1** in 60 s |
| q p10 / p50 | **0.91 / 1.00** |
| cep p50 | 0.32 M2 px |

**This is what "reliably lock on the beacon" looks like**, and it is now demonstrated rather than asserted.

## The photometry, which is the ladder's x-axis

| | frame mean | p99 | K per pass |
|---|---|---|---|
| **rung 0 — dark** | **0.14** | **0.0** | **36** |
| lit60 (200 W on) | 2.1 | 49 | 1134 |
| static_bench (room light) | 5.6 | 165 | 1277 |

The scene is essentially **black except the beacon** (max 255 — it saturates the sensor), and clutter
collapses **35×** versus a lit scene. That is why acquisition is instant: the beacon is not competing for a
top-3 seed slot, it *is* the field.

## Reading the three conditions together

| condition | frame mean | K | q p50 | present | first lock |
|---|---|---|---|---|---|
| dark | 0.14 | 36 | **1.00** | **99 %** | **0.30 s** |
| room light | 5.6 | 1277 | 0.79 | 79 % | 0.40 s |
| 200 W | 2.1 | 1134 | 0.45 | 42 % | 1.00 s |

⚠️ **The last two are not on the same rung** — the bench was reconfigured between them, so geometry moved as
well as light. That is exactly the confound this ladder exists to remove: from rung 0 onward, **do not move
anything but the lighting.**

Note also that frame mean alone does not predict the outcome (room light is *brighter* than the 200 W clip
yet scores better), which is further evidence that the earlier pair differ in beacon geometry, not just
illumination.

## Protocol for the remaining rungs

For each rung, record 60 s continuous at the bench operating point (53 µs) plus 5 × 15 s live runs, and
publish the same six numbers: **frame mean, K per pass, q p10/p50, present %, first-lock median, drops**.
Change *only* the illumination between rungs.

Suggested rungs: bench lamp only → room lights → 200 W → 200 W + daylight. The interesting question is
which of q, K or first-lock degrades first — the model says K rises with ambient and pushes the beacon down
the seed ranking, while q falls with shot noise, and those two failure modes want different fixes.

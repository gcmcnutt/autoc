# T077 — the pendulum fixture, and the first honest rate curve (2026-08-25)

Operator built the T077 rig: the beacon on ~0.80 m strings at 3 m range, **with a pole placed to occlude
it as it swings** — so one fixture carries both the rate axis and a reacquisition dataset. Three clips
were recorded, all 288 fps continuous, all **0 sensor seq gaps**, exposure pinned at 53 µs throughout.

| clip | length | what it is |
|---|---|---|
| `beaconpi5:/data/pend_static.bcnr` | 5 s | at rest — the reference, and the truth-tool validation |
| `beaconpi5:/data/pend60.bcnr` | 60 s, 4.42 GB | ~30 cm release, p90 **22.9 °/s**, **63 pole occlusions** — the reacquisition fixture |
| `beaconpi5:/data/pend15cm.bcnr` | 240 s (210 usable), 17.7 GB | ~15 cm release left to decay — **the rate ladder** |

Tools added with this work: [`tools/pendulum_truth.py`](../../../firmware/beacon-receiver/tools/pendulum_truth.py)
and [`tools/pendulum_analyze.py`](../../../firmware/beacon-receiver/tools/pendulum_analyze.py).

## Why a new truth tool, and why it is allowed to do what `oracle.py` forbids

`oracle.py` is the right truth for a static target and the wrong one here: it needs the beacon to sit in
one pixel for a 258 ms code word, and at 20 °/s the beacon crosses 17.7 M2 px in a word. **It cannot
measure the regime this fixture exists to measure.** `pendulum_truth.py` swaps the code discriminator for
a geometric one — brightest blob, half-max centroid — which `oracle.py` explicitly warns against.

That warning is real and **it fired on this rig**: the unrestricted per-frame maximum lands on the
**ceiling light**, not the beacon. The ROI (the measured swept arc) is what makes the method valid, and it
is mandatory, not optional — a too-generous ROI silently re-admits the lamp.

**Validation**: on `pend_static.bcnr` the tool gives M2 **(−7.74, +36.15)** where `oracle.py`'s full-field
31-phase matched filter gives **(−8.00, +36.00)** — agreement to **0.30 M2 px = 0.09°**. Re-run that check
whenever the rig or ROI changes.

## The rig, as measured rather than as designed

T077 sized a 1.0 m pendulum at 2.0 s period. The actual rig ran a **1.8 s period**, so the strings are
**~0.80 m**. That gives the working constant for future clips:

> **apparent peak rate ≈ 66.6 °/s per metre of release amplitude**, at 3 m range.

The 30 cm release measured p90 22.9 °/s against 20.6 predicted — the design point, hit within 10%. (The
`max 64.9 °/s` in the raw output is **not** real: those are centroid jumps where the pole clips the blob
edge. p90 is the trustworthy statistic.)

## The headline: present ≠ tracking

On the 30 cm clip the tracker was **present on 49% of ticks and carried a MEASURED fix on 11%.** The rest
is coasting on a stale velocity with no fresh decode at all. Reading `n>=1` as "tracking" would have
overstated performance by **4.5×**, and that distinction is now built into the analyzer.

## The rate curve — 15 cm clip, decay as the sweep

Decay makes time a proxy for *sustained* rate, which is the axis that matters. A pendulum passes through
low rate instantly, so an instantaneous-rate bin mixes "sustained slow" with "just arrived from fast":

Fixes are split **on-beacon vs false** (>20 M2 px from truth) — see the false-lock section below; on
this clip the false share is small (3.8%) so the curve is unaffected, but the split has to be shown.

| sustained rate | ticks | on-beacon fix | false | present | bearing err p50 |
|---|---|---|---|---|---|
| 5.9 °/s | 336 | **12%** | 5 | 62% | 0.78° |
| 4.4 | 400 | 16% | 8 | 79% | 0.44° |
| 3.7 | 400 | 22% | 18 | 76% | 0.61° |
| 2.8 | 400 | 26% | 0 | 72% | 0.55° |
| 2.4 | 400 | 30% | 2 | 70% | 0.50° |
| 1.9 | 400 | 40% | 2 | 77% | 0.48° |
| 1.7 | 400 | 35% | 13 | 86% | 0.38° |
| 1.5 | 400 | 37% | 7 | 83% | 0.45° |
| 1.3 | 400 | 50% | 1 | 87% | 0.41° |
| 0.9 | 400 | 56% | 0 | 95% | 0.32° |
| 0.9 | 201 | **63%** | 0 | 90% | 0.34° |

**Three things this settles.**

1. **The knee is a smooth roll-off, not a cliff.** Decode rate roughly halves per doubling of angular
   rate: 63% at 0.9 °/s down to 12% at 5.9 °/s. The "1–2 °/s knee" from `pan2` is confirmed and given a
   shape — 1.9 °/s is where roughly 40% of ticks decode.
2. **Bearing accuracy rides the same curve** — 0.32° at 0.9 °/s, which is exactly the §3 bar of 0.3°,
   degrading to 0.86° at 5.9 °/s. **The spec bar is met only at the very bottom of the rate range.**
3. **The gap to the mission is now a number.** The engagement is ~19 °/s (10 m/s at 30 m) and hand motion
   is 15–18 °/s. At 5.9 °/s we decode 12% of ticks. That is the gap T081/T082/T050 have to close, and
   these clips are the fixture that will say whether they did.

## A MEASURED fix is not necessarily a fix ON THE BEACON

Found by watching [`track_overlay.py`](../../../firmware/beacon-receiver/tools/track_overlay.py) — the
x strip chart showed the estimate excursing to **+127 M2 px**, which is not on the pendulum at all, it is
the **window**. Scored against truth:

| clip | on-beacon fixes | false (>20 M2 px) | false share |
|---|---|---|---|
| `pend60` (30 cm, fast) | 86 | 25 | **22.5%** |
| `pend15cm` (slow) | 1413 | 56 | 3.8% |

**On the fast clip, nearly a quarter of the tracker's "decodes" are not on the beacon**, and almost all of
them land at x>60 M2 px — the window region. `BCN_F_MEASURED_FIX` means *a correlation passed*, not *a
correlation passed on the beacon*, and only truth can tell the two apart.

The mechanism is almost certainly **T085's**: the window is a saturated, flat, mains-flicker region at
**120 Hz — the same rate as our chip clock** — and `quality_q8()` is a pure ratio `|corr|*256/energy` whose
denominator collapses on a saturated flat, so noise-level correlation scores arbitrarily high. This is the
rung-1 lamp defect, now caught on a moving-target fixture with independent truth and a quantified cost.

It also **raises the stakes on the pattern that killed both T085 attempts** — a gate validated against a
distribution collected with the gate disabled. This fixture gives the end-to-end validation those attempts
lacked: false share and on-beacon decode rate are now both measurable on the same clip, so a candidate
gate can be scored on whether it cuts the former without cutting the latter.

Note the rate dependence: 22.5% false on the fast clip vs 3.8% on the slow one. When the beacon is decoding
well the tracker stays on it; false locks fill the vacuum left when coherence fails. **So this is a
consequence of the rate limit as well as a defect in its own right**, and fixing either one helps.

## Operator observation, 2026-08-26 (live) — temporal match, spatial attribution wrong

Watching the new live crosshairs, the operator saw them **jump to the overhead lights**:

> *"even though most of the code comes from the beacon pretty far away, the tracker sees a light edge
> and it matches partial code … that is temporal match but spatial is a bit far out."*

**This is not the same statement as the measured lamp defect above, and it may not be the same
mechanism.** Recorded now, unmeasured, so the next session starts from the observation rather than from
a reconstruction of it.

What it adds: the earlier analysis treated the false lock as *"a flat source fakes a code match"*. This
says the code match may be **genuine** — carried by the beacon — while only the **position** is
misattributed. Temporal evidence and spatial evidence would then be coming from different sources, which
no gate on the aperture-sum statistics can catch, because on those statistics the window really is
carrying the code.

Three candidate mechanisms, and they are distinguishable:

1. **Split attribution.** `track.c` adopts phase from the aperture-SUM correlation, then takes the peak
   of a per-pixel `|corr|` surface *at that phase*. A bright pixel carries more variance, so if anything
   bright sits in the aperture it can win the surface even when the code energy came from elsewhere.
   Test: for a false fix, compare the aperture-sum corr against the per-pixel corr at the winning pixel.
2. **120 Hz mains against a 120 Hz chip clock.** Our chip rate and mains flicker are the same frequency —
   the pod runs 120.0077 Hz, a 64 ppm offset, so a lamp's flicker holds nearly constant phase across a
   258 ms word instead of averaging out. That is exactly the condition under which a non-code source
   produces a stable, code-like bin pattern. Test: measure a lamp region's per-chip bin pattern directly.
3. **A different source than the one I characterised.** The lamp I measured on `pend1m` at native
   (422,108) is **constant** — p2p 25, values 186±7 — and a constant source cannot match a code
   temporally. The overhead lights in the live scene may be a genuinely different, genuinely flickering
   source. The live scene is not the `pend1m` scene.

⚠️ **Do not assume (3) away.** The measured `mod_depth` separation (lamp p95 0.88 vs beacon median
1.9–22.6) was derived from the constant lamp on `pend1m`. If the live overhead lights flicker, they are a
different population and that separation has not been tested against them. This is the same trap that
killed both T085 attempts — a threshold read off a population that was not the one that matters.

**First measurement next session**: record a short live clip of the current scene with the lights in
frame, and run the `--diag` split on it. That says which of the three it is before any more gate design.

## Reacquisition — what the pole bought

On the 30 cm clip, **63 occlusions in 60 s** (one per crossing), duty 11.5%, duration p50 76 ms /
p90 170 ms / max 424 ms:

| occlusion end → next MEASURED fix | |
|---|---|
| p50 | **363 ms** |
| p90 | **1295 ms** |
| max | 2492 ms |
| within the 400 ms bar (§3) | **34 of 63 = 54%** |
| never recovered | **0** |

⚠️ These count **any** measured fix, including the false ones above — and this is the clip with a 22.5%
false share, so read the recovery figures as an **upper bound** on true reacquisition.

So the median just squeaks inside the bar and **the tail is 3× over it** — and that is with the beacon
re-emerging at a completely predictable place, phase unchanged, after ≤424 ms. This is the strongest
argument yet for T081: a phase-known broad search makes each of these a 0.39 ms re-detection instead of a
fresh acquisition lottery.

## Two analysis errors made and corrected, both worth keeping

1. **`rpicam-raw` gave R16, and it was parsed as R8.** The first two "quick looks" at the rig produced a
   phantom beacon position and a false reflection scare. The tell was a halftone texture in the preview
   images — that is byte interleaving, not dither. The tracker's own path is **Y8**; the recording path
   (`beacon_record` → `.bcnr` → project tools) is the one to trust, and ad-hoc `rpicam-raw` parsing is not.
2. **The pole was assumed to dominate the 15 cm clip. It does not** — only **12 occlusions in 210 s
   (1.3% duty)**, because a small swing barely reaches it, and filtering ticks near occlusions changes the
   result by under 1 point. The ~40% ceiling at low rate is a genuine coherence limit, not an occlusion
   artifact. Worth stating because the opposite is true on the 30 cm clip (11.5% duty), so the two clips
   must not be pooled.

## The battery, and how the clip ended

The pod was on battery (untethered for the pendulum), and the 240 s recording ends at **210.2 s**. Beacon
peak amplitude held at **255 (p10 232) right up to the final sample and then stopped dead** — a clean
cutoff at full amplitude, which is the pod's **3.48 V UVLO** behaving exactly as built, not a brownout
fade. `pendulum_analyze.py --decay` prints peak amplitude per block precisely so this is legible; a fade
would mean something else and would invalidate the tail of a clip.

**Use `--limit 210` on `pend15cm.bcnr`.** The 200–220 s block's apparent amplitude jump (0.97° → 2.40°) is
34 straggler samples after cutoff, not a real swing.

## What to record next

The rate axis above stops at 5.9 °/s and the mission needs ~19 °/s **with sustained dwell** — which this
fixture can deliver, since a bigger release simply starts the same decay higher up:

| release | peak rate | purpose |
|---|---|---|
| 30 cm | ~20 °/s | already have it (`pend60`), but decay is slow — needs ≥240 s to reach the knee |
| 45 cm | ~30 °/s | brackets the engagement rate from above |

One long clip per release beats several short ones: decay sweeps every rate below the peak for free, and
a fresh battery is the binding constraint (210 s per charge, measured).

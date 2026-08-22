# pan2.bcnr — the first envelope measured against real ground truth (2026-08-21)

**Clip**: `beaconpi5:/data/pan2.bcnr`, 640×400, **continuous**, 17275 frames, 60.003 s, 0 seq gaps,
exposure pinned 1499 µs (`beacon-fiducial.ini`). Operator: a pan, then a tilt, then circular motion, *"all
reasonably slow"*.
**Truth**: [stage1-pan2-truth.csv](stage1-pan2-truth.csv), from `tools/fiducial_truth.py` over four printed
ArUco markers. **Analysis ran on the Pi** (37 s) rather than moving 4.2 GB over its WiFi link.

## The ground-truth chain is validated, not assumed

Against the tracker's own fixes on 454 locked ticks, with the reference anchored properly:

| | value |
|---|---|
| bias | dx **−0.30**, dy **+0.42** M2 px |
| median &#124;err&#124; | **0.59 M2 px = 0.18°** |
| p90 &#124;err&#124; | 2.51 M2 px = 0.76° |

**It failed the first time and the failure is worth recording.** `fiducial_truth.py` defaults the emitter
reference to "brightest spatial-high-pass pixel", which on this bench landed on one of the LED light bars,
not the beacon: dx bias **−42 px** while dy bias was +0.4. A single-axis bias that large with a clean
orthogonal axis is the signature of a mis-picked reference, not a broken transform — the spread was already
only 4.4 px. Re-anchoring with `--ref-xy` from the tracker's own locked position fixed it. **Always
validate truth against a known-good stretch before trusting it.**

## The envelope — lock vs measured apparent rate, emitter on screen only

| rate (°/s) | n | **tracker locked** | truth coverage |
|---|---|---|---|
| 0.0–0.5 | 4000 | 65.6 % | 99.5 % |
| 0.5–1.0 | 387 | **78.0 %** | 93.8 % |
| 1.0–2.0 | 66 | **45.5 %** | 63.6 % |
| 2.0–3.0 | 52 | 50.0 % | 61.5 % |
| 3.0–5.0 | 89 | 34.8 % | 73.0 % |
| 5–10 | 230 | 18.3 % | 60.0 % |
| 10–20 | 1452 | **3.0 %** | 68.5 % |
| 20–50 | 1198 | 6.0 % | 62.9 % |

**The knee is between 1 and 2 °/s** — the third independent measurement to land there:

1. theory — 1 M2 px of smear per 258 ms word ⇒ **1.18 °/s**
2. `pan1` coherence ratio (a wholly different method: q at 33 chips vs 5 chips) ⇒ **1–3 °/s**
3. this clip, lock rate against fiducial truth ⇒ **1–2 °/s**

The 0.0–0.5 bin reading only 65.6 % is *not* a rate effect — it is acquisition latency contaminating the
stationary samples (see below).

## What "reasonably slow" actually was

| segment | rate p50 / p90 | in frame | locked |
|---|---|---|---|
| settle (still) | 0.1 / 0.2 °/s | 100 % | 20.5 % |
| **PAN** 10–20 s | **15.2 / 25.9 °/s** | 100 % | 30.5 % |
| **TILT** 20–26 s | **16.6 / 30.9 °/s** | 74.2 % | **0.0 %** |
| **CIRCULAR** 26–41 s | **17.7 / 30.5 °/s** | 94.7 % | **0.0 %** |
| still 42–60 s | 0.2 / 0.5 °/s | 100 % | 97.8 % |

**Deliberately gentle hand motion is 15–18 °/s median, 26–31 °/s at p90 — an order of magnitude above the
~1.2 °/s coherence limit.** That is the gap stated plainly, and it is why this cannot be tuned away.

The emitter was **in frame 96.8 %** of the clip, so this is a genuine tracking failure and not the target
leaving the field. Only the tilt drove it off-screen appreciably (74 % in frame), and correcting for it
changed nothing: 0.0 % locked either way.

## Reacquire — the headline metric under §3's reframe

| episode | gap |
|---|---|
| at 11.0 s | 2.65 s |
| at 15.1 s | 3.85 s |
| at 19.9 s | **22.25 s** |
| at 52.2 s | 0.30 s |

**Median 3.85 s against the §3 band-3 bar of 0.40 s — an order of magnitude over.** And the shape matters
more than the median: the 22.25 s episode spans the entire tilt *and* circular segments and ends only when
the motion stops at ~42 s. **Through 22 seconds of continuous slow motion the receiver never reacquired
once**, with the target on screen for 94 % of it. Only the 0.30 s episode — a brief dropout while already
essentially stationary — meets the bar.

This is the direct measurement of §3.2 band A: cold acquisition is a static full-field search and it
simply does not function while the camera moves.

## Two more things this clip shows

- **Cold acquisition took ~8 s with everything stationary** (0–8 s, 0.1 °/s, target dead centre). That is
  not a motion effect at all. Caveat: this clip runs at the fiducial exposure (1499 µs), not the bench
  operating point, so the blink detector sees a much brighter scene — the figure should be re-measured at
  53 µs before being treated as the acquisition-latency number.
- **Fiducial coverage degrades exactly where it is needed**, as predicted when the markers were placed:
  99.5 % of frames have ≥3 markers while slow, ~60–68 % while moving. The envelope table above therefore
  publishes coverage per bin. Bigger or closer markers would buy margin.

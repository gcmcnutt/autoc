# Outdoor: sky, sun and ego-motion — the first measurements outside (2026-08-29)

The five-minute experiment [event-camera-emulation.md §9](../event-camera-emulation.md) asked for, run at
high noon on the flight optical configuration (1.26 mm + 850 nm bandpass, 53 µs, gain 1.0 pinned). It
answered the question it was asked and then overturned the premise behind it.

## 1. Clear sky at 850 nm is nearly black

First frame, camera ~30° up from horizontal, pointed away from the sun. The frame turned out to contain
house, foliage and ground with sky only in the top third — which made it accidentally the more useful
measurement, because the regions separate cleanly:

| region | mean | ev/frame @thr20 | ev/frame @thr40 | beacon delta needed @5σ |
|---|---|---|---|---|
| **sky** (top) | **1.2 ADU** | **17.5** | **0.0** | **7.8 ADU** |
| mid (rooflines) | 15.0 | 106 | 40 | 27.4 |
| **terrain / foliage** | **49.2** | 3050 | 1871 | **49.6** |

*(mid and terrain scaled to a full frame of that material)*

**Sky is a solved case.** 1.2 ADU is barely above the indoor dark room's 0.02, and 17.5 events/frame at
thr 20 is indistinguishable from the pendulum room's 18.8. A beacon delta of 7.8 ADU clears it, against
measured deltas of 46–255 across the whole indoor range ladder. Enormous margin.

## 2. Sunlit vegetation is the background adversary — and the filter makes it worse

**Terrain needs a ~50 ADU beacon delta and still gives 1871 events/frame at thr 40.** That is not a
threshold that can be tuned around; a far-field target at delta 10–20 is simply lost against sunlit
foliage.

The counterintuitive part: **the 850 nm bandpass amplifies this problem.** Vegetation is exceptionally
bright in NIR — the "red edge" that NDVI exploits — so the filter that annihilated indoor lamps
(212× background reduction) *preferentially passes* the brightest natural terrain there is.

So the limit maps onto **geometry, not range**:

- **target against sky** — same margin as indoors; the good case, and genuinely good
- **target against sunlit terrain** — needs ~50 ADU delta, i.e. roughly the indoor ≤8–17 m equivalent

## 3. The sun barely matters. Ego-motion dominates by 100×

A 30 s pan sweeping the sun across the frame, at flight settings:

| t (s) | sat px | ev/frame @thr20 | cells with events (of 64) | |
|---|---|---|---|---|
| 2.1 | 0 | 15 | 5 | localised |
| 10.4 | **511** | **37** | 14 | mixed |
| 20.8 | **0** | **3635** | **45** | **field-wide** |
| 27.1 | 0 | 3284 | 41 | field-wide |

**With the sun in frame — 511 saturated pixels — the event rate is 37/frame over 14 of 64 cells.** Barely
worse than a static scene, and *localised*, so a cluster test rejects it trivially. The sun is compact and
mostly **saturates**, and a saturated pixel produces no delta.

**With no sun at all, panning gives 3635 events/frame over 45 of 64 cells.** Two orders of magnitude
worse, across the whole field.

The operator's instinct — *"if the sun comes into view, that is ok, we don't want to adjust down"* — is
right, and for a better reason than either of us had: **the sun genuinely does not matter much**, so
ignoring it costs little.

**The real adversary is ego-motion, and in all-attitude flight it is constant rather than occasional.**
Every edge generates events whenever the airframe moves. Against sunlit terrain — already 49 ADU with
structure everywhere — that is a permanent flood.

This lands on **T071 (ego-motion registration)**, already called "THE enabling piece" for the correlator.
It turns out to be equally enabling for the event detector: **the architecture change does not route
around it.** Registering frames before differencing would cancel the field-wide events and leave only
genuinely moving or modulating sources.

## 4. Sensor safety: one pan is safe, dwell untested

Before/after hot-pixel census, using residual-against-local-median so no covered lens is needed:

| threshold | before | after | worse | better | ratio |
|---|---|---|---|---|---|
| 30 ADU | 2850 | 2998 | 1499 | 1433 | **1.05** |
| 50 | 216 | 223 | 148 | 128 | 1.16 |
| 80 | **0** | **0** | 2 | 2 | **1.00** |

**No detectable damage.** Damage is one-sided — a burned pixel gets brighter and stays brighter — so it
would show as worse ≫ better. The counts are balanced, which is scene change from the pan. Zero pixels
above residual 80 either side, and the worst-offender coordinate lists share no entries between runs.

⚠️ **This tests a pan, not a dwell, and one pan only.** It says nothing about parking the sun in frame,
and repeated transits could accumulate in a way a single one will not.

## What to measure next

**Beacon against foliage at range** (operator's call, and the right one). Everything above is background
characterisation; none of it has put a beacon in front of the hard background. The specific question:
at what range does a beacon still clear the ~50 ADU delta that sunlit foliage demands? The indoor ladder
says delta 46 at 8.15 m and 96 at 16.7 m, so the prediction is that **foliage-backed detection fails
somewhere around 10–20 m** — much sooner than sky-backed.

Worth doing with the camera static, so ego-motion is not confounded into the result. Then repeat with a
deliberate pan to measure what motion costs on top.


---

# Beacon against sunlit foliage, 10 m (2026-08-29)

The measurement §4 asked for. Camera static, beacon at ~10 m, moved between backgrounds rather than
between ranges — so background is the **only** variable.

| beacon aim / background | local bg | **MEASURED** | q p50 | first lock |
|---|---|---|---|---|
| at camera, transition zone | 11.6 ADU | **97 %** | 1.00 | 0.40 s |
| at camera, **in foliage** | **68 ADU** (p90 124) | **33 %** | 0.82 | 1.70 s |
| **aimed away**, in foliage | 68 | **3 %** | 0.59 | 3.10 s |

**Sunlit foliage costs two thirds of the decode rate** at a range where sky-backed detection is
effortless. Same beacon, same range, same camera.

The mechanism is in the local event floor around the beacon: **1186 events/frame (full-frame equivalent)
even at threshold 100**, barely falling from 2428 at threshold 20 — the wind-driven foliage plateau,
now surrounding the target rather than sitting elsewhere in frame.

**But the code is doing real work.** 33 % decode while surrounded by ~1200 competing events per frame,
because foliage events are at random phases and the beacon's are coherent. That was genuinely uncertain
beforehand: stage 1 contributes almost nothing here (indoors it gave a 13600× reduction with a *pure*
list; here the beacon is *inside* the event population by amplitude), so detection rests entirely on
stage 2.

## Two errors of mine, both worth recording

**A prediction built on a datum I had already flagged as bad.** I predicted foliage-backed failure at
10–20 m from the indoor ladder's "delta 46 at 8.15 m". That came from the one station whose photometry I
had already noted was erratic — the sequence ran 46 → 130 → 96 as range *increased*. Using it as a
prediction anyway was careless. The beacon is far brighter: saturated at 10 m outdoors.

**The photometry aperture was in the wrong place.** `range_station` took the brightest blob in frame,
which is the beacon indoors on a black background and is **sunlit ground** outdoors. It reported peak 255
throughout — the driveway, not the beacon — so the intensity axis stayed unmeasured even though the
operator's angling trick worked exactly as intended. Same class of error as `oracle.py`'s brightest-pixel
warning, which finds the ceiling light.

**Fixed**: photometry now measures a fixed aperture **at the tracker's reported position**, with no
threshold and no blob search, because a dim beacon does not win a brightest-blob contest against sunlit
ground. Tracking runs first so the position is available.

## What is still unmeasured

The beacon was saturated in every capture, so **its delta is unknown and the failure point cannot be
extrapolated**. 10 m is also short. Both point at the same conclusion: **this needs the field**, at
30–100 m, where the beacon comes off the rail and the margin becomes measurable rather than merely
present.

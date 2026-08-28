# Emulating an event camera — count every edge, keep the ones that fit the code (2026-08-27)

**Operator**: *"figure out how close we are to emulating an event camera — e.g. instead of dynamic
aperture, we count all the edges across every frame, and the edges that line up with our locked code."*

Short answer: **the arithmetic is already affordable; the memory read is not.** One measured hardware
number decides the whole question, and there is a clean way around it.

## Why the current architecture is the wrong shape for two fast targets

Operator's framing, and it is the right one: *"we are a ways off from decoding two at high rate across
the screen — recall our attitude and target can be in different directions where the span is large."*

The receiver today is **ROI-proportional by design** (`engine.c`): the per-frame path touches only the
tracked apertures, 16 slots × ~576 native px ≈ 9 KB. That is what makes it fit the 3.47 ms frame budget,
and it has two consequences that bite exactly in the two-target case:

1. **Acquisition is a seeded lottery, not a search.** `acquire_pass` returns the top 3 seeds from a
   coarse plane, throttled to roughly 1 pass in 25 frames. A second target far from the first has to win
   a top-3 slot on its own merits, in a field that may be 145° wide.
2. **Holding lock is rate-limited per track.** Measured on the flight optics: **13 % decode at 16.5 °/s**
   ([pendulum re-run](results/stage1-pendulum.md)). Two targets both moving fast means two tracks both
   in the regime where the aperture is being outrun.

An event-style detector inverts this: **every pixel is a candidate on every frame**, so there is no
seeding lottery, no top-3, and no dependence on a target being among the strongest blinkers. That is the
same conclusion [sync-first](sync-first-acquisition.md) reached from the cost side; the event framing is
the same idea reached from the *detector* side, and it is a better fit for "two targets, wide apart."

## What the emulation actually is

Not a real event sensor — a **per-frame temporal difference, sign-checked against the known code**:

```
  event[p]  = frame[p] - prev[p]                    (one subtract per pixel per frame)
  expected  = code[chip_at(t)] - code[chip_at(t-1)] (known, once phase is locked)
  keep      = sign(event[p]) == sign(expected) and |event[p]| > floor
```

Three properties make this the right operation rather than a gimmick:

- **It is AC coupling in the digital domain.** Static background cancels exactly; a constant lamp cancels
  exactly. That is [ac-coupling-and-exposure.md](ac-coupling-and-exposure.md)'s whole argument, and this
  is its full-field form.
- **The polarity is known**, so it is a *matched* differential rather than a blind one. No threshold on
  q, energy or modulation depth is needed — the three statistics that failed in the T085 work.
- **Only the transition frames carry information**, and those are deterministic: at 2.3992 frames/chip
  the 120 Hz retune made the pattern **12 frames per 5 chips**, of which **5 straddle a chip boundary**
  ([T082](tasks.md)). So the check runs on ~42 % of frames, not all of them.

## The compute is affordable — MEASURED, not assumed

`beacon_bench --kernels` on this Pi 5, 256000 px/frame (the current 640×400 mode):

| kernel | time | vs the 3.47 ms frame period |
|---|---|---|
| `reduce4` (NEON) | **0.45 ms** | 13 % |
| `reduce2` (NEON) | **0.65 ms** | 19 % |
| `mac_i16` (scalar) | 1.37 ms | 39 % |
| `hipass M2` (NEON) | 2.65 ms | 76 % |
| `reduce2x2` (scalar) | 4.46 ms | **129 % — over budget** |

A per-pixel subtract-and-compare is `reduce`-class work, so **the differencing itself costs ~0.5 ms of a
3.47 ms frame. There is no arithmetic problem.** Note also how much NEON matters here: reduce2 goes
5.67 → 0.65 ms, a factor of 8.7, so this is only affordable in its vectorised form.

## The blocker is the dmabuf read, and it is a hardware property

From [research.md](research.md) and re-stated in `engine.c`:

> **dmabuf mmap is write-combine — one uncached 256 KB frame read costs ~4 ms**, slightly more than the
> frame period.

That is the entire problem in one line. **4 ms > 3.47 ms.** Reading every pixel of every frame does not
fit, no matter how cheap the arithmetic that follows. This is precisely why the architecture went
ROI-proportional in the first place, and it is why acquisition is budgeted through `sched` at ~1 pass in
25 frames rather than run continuously.

So the honest position: **we are not close on the current capture configuration, and no amount of kernel
optimisation gets us there** — the cost is in touching the memory, before any arithmetic happens.

## The way through: a smaller sensor mode

The OV9281 offers **320×200 at up to 588 fps** natively (`rpicam-hello --list-cameras`). That is
**64000 px = 1/4 the bytes**, so the WC read drops from ~4 ms to **~1 ms**.

And it lands on a happy coincidence: **320×200 IS the M2 record grid** — the coordinate system the
tracker already reports in (`track.h`: "the M2 grid — 320×200, centre (0,0)"). A 320×200 sensor mode maps
**1:1** onto the output contract with no reduction step at all.

Budget at 320×200, 288 fps (3.47 ms period):

| | cost |
|---|---|
| WC frame read | ~1.0 ms |
| per-pixel difference + sign check | ~0.15 ms |
| **total** | **~1.2 ms = 33 % of one core** |

Even at the sensor's full 588 fps (1.70 ms period) it is ~68 % of a core — tight but not absurd.

**That makes full-field, every-frame, code-matched event detection affordable.** It is the first
architecture that removes the acquisition lottery entirely.

## What it costs, stated plainly

**The fine scale disappears.** The tracker currently runs `TRK_SCALE_FINE` = native resolution for the
precision fix, and on the pendulum re-run it sat pinned at scale 2 for essentially every tick. A 320×200
sensor mode has no native plane to refine into, so sub-pixel bearing precision degrades.

How much that matters has *changed*, and in our favour: the 1.26 mm lens already fails the §3 0.3° bar at
every rate measured (best block 0.44° at 2.1 °/s). **The fine scale is buying precision we no longer
meet the bar with anyway.** That weakens the main objection considerably — though it does not remove it,
because 0.44° is still better than what a coarser grid would give.

Two other honest caveats:

- **Phase must be receiver-global first.** "Edges that line up with our locked code" presumes a locked
  code, and today `chip_phase` is per-track. That is exactly [T081](sync-first-acquisition.md), which
  therefore becomes a **prerequisite**, not an alternative.
- **Straddling frames need weighting, not discarding.** 42 % of frames span a chip boundary; discarding
  them costs 2.34 dB of the word's 14.9 dB gain ([T082](tasks.md)). The event detector must weight each
  frame by its overlap with the chips it spans — and usefully, the straddlers are also the sub-chip phase
  discriminator.

## Where this sits against the existing plan

It is not a new direction so much as the **full-field form of work already queued**, and it reorders the
queue:

| task | relation |
|---|---|
| **T081 sync-first** | **prerequisite** — the "locked code" this depends on |
| **T082 sub-frame edges** | **prerequisite** — the straddler weighting |
| T080 (accumulated \|Δ²\|) | superseded — that was blind differencing to fix *ranking*; this is matched differencing with no ranking step |
| T050 decode-along-track | still needed for smear at high rate; events find, TBD follows |
| T085 min_mod_depth | already retired by the bandpass; matched differencing would retire it again |

**The one new thing this analysis adds** is that the blocker is not compute and never was — it is the
write-combine read — and that a **320×200 sensor mode dissolves it while landing exactly on the M2 output
grid.** That is a capture-configuration change, not an algorithm change, and it is cheap to test:
re-record a pendulum clip in 320×200 and measure whether decode holds.

## Suggested first experiment

Before building anything: **record one pendulum clip at 320×200** and check two things.

1. Does the tracker still decode at all with no fine scale? That is a direct measurement of what the
   precision loss costs, on a fixture we now have a baseline for.
2. Measure the actual WC read time at 320×200 rather than trusting the 1/4 scaling. The 4 ms figure is
   measured for 256 KB; 64 KB may not be exactly 1 ms.

If both come back acceptable, the event detector has a home. If the decode collapses without the fine
scale, the idea still stands but needs a dual-stream capture, which is a much larger change.


---

# MEASURED, 2026-08-27 — the idea holds, and the numbers are better than the estimate

## 1. Threads do not buy the write-combine read (my hypothesis, refuted)

I argued above that a WC read is latency-bound rather than bandwidth-bound, so several threads could hide
what one cannot. **Measured with `beacon_record --read-bench`, that is wrong:**

| threads | 640×400 read | throughput |
|---|---|---|
| 1 | 3.211 ms | 80 MB/s |
| 2 | 3.207 ms | 80 MB/s |
| 4 | 3.184 ms | 80 MB/s |

Flat. It is a hard **80 MB/s bandwidth wall** and **4 cores buy nothing**. Operator called this
("me is an issue yes"); the latency-hiding theory was mine and it did not survive contact.

At 640×400 the read is **96 % of the 3.33 ms frame period**, which leaves nothing for anything else.

## 2. 320×200 is broken in this pipeline; 640×200 is not

The 320×200 mode the analysis above proposed **does not stream**. `rpicam-vid` fails identically to our
app — the sensor format is selected, a few frames trickle out, then "Camera frontend has timed out". So
the mode is advertised by the driver and not usable through this PiSP/CFE path. **The proposed route was
blocked by a driver issue, not by anything in the design.**

**640×200 works and is the practical answer:**

| mode | bytes | WC read | % of frame |
|---|---|---|---|
| 640×400 | 256 000 | 3.21 ms | **96 %** |
| **640×200** | 128 000 | **1.29 ms** | **39 %** |

Half the bytes and slightly better throughput (99 vs 80 MB/s), streaming cleanly at 0 seq gaps. It is a
**crop, not a bin**, so horizontal scale is unchanged and **the fine scale survives** — the objection
raised against 320×200 does not apply. The cost is half the vertical field (~147° × 45° instead of
147° × 90°), which is the wrong axis to give up for "attitude and target in different directions" and is
a judgement about the engagement rather than something the bench can settle.

## 3. The event list is PURE SIGNAL — measured on `pend_ir.bcnr`

`event_probe.py` over the flight-optics pendulum clip, threshold in ADU of frame-to-frame delta:

| thr | fast (~16 °/s) | slow (~2 °/s) | on-beacon |
|---|---|---|---|
| **3** | **18.8 ev/frame** — 1 in 13 600 | **12.9** — 1 in 19 800 | **100.0 %** |
| 5 | 17.3 | 10.9 | 100.0 % |
| 10 | 14.6 | 8.6 | 100.0 % |
| 40 | 7.4 | 6.1 | 100.0 % |

**A ~13 600× search-space reduction with ZERO false events, at a 3 ADU threshold sitting essentially on
the noise floor.** It holds at both ends of the swing, which matters: a moving target fires events on
both edges of its motion while a stationary one only blinks the code, and both give a sparse pure list.

## 4. The physical filter is what buys the aggressive threshold

The same probe on the **unfiltered** clip (`pend1m.bcnr`, 2.31 mm lens, cluttered scene):

| thr | filtered | unfiltered |
|---|---|---|
| 3 | 18.8 | **66.1** |
| 5 | 17.3 | 56.7 |
| 10 | 14.6 | 36.5 |

**3.5× more events at the low threshold, and the extras are clutter.** To recover comparable sparsity
without the filter you would have to raise the threshold to 20–40 — exactly the wrong direction, because
a high threshold is what loses a far-field target. So: **the filter buys the low threshold, and the low
threshold buys range.** That is the quantified form of the operator's "aggressive across wide ranges".

## The resulting cost model

```
  stage 1   O(pixels)   1.29 ms @ 640x200   (WC read dominates; the compare is ~0.33 ms)
  stage 2   O(events)   ~19 coordinates      negligible
                        ---------
                        ~1.6 ms of a 3.33 ms frame  =  under half a core
```

No seeding lottery, no top-3 cap, no ROI, no preference for the strongest blinker — which is precisely
what the two-target wide-span case needs.

**Still prerequisites**, unchanged: [T081](sync-first-acquisition.md) makes phase receiver-global so
"events that line up with our locked code" means something, and [T082](tasks.md) weights the 42 % of
frames that straddle a chip boundary rather than discarding them (worth 2.34 dB).


## 5. The full three-stage detector, prototyped offline and working

`event_probe.py --decode` on `pend_ir.bcnr`, 300 frames of the slow part, threshold 5 ADU:

**Stage 2 — per-coordinate code correlation.** 76 coordinates touched out of 256000. The matched quantity
is the TRANSITION, not the level: at a chip boundary the expected change is `code[k]-code[k-1]` ∈ {-1,0,+1},
and frames inside a chip carry no information and are skipped. Top scorers:

```
score  phase  native(x,y)
   21     27   (354, 197)
   21     27   (354, 196)
   21     27   (351, 197)
   21     27   (351, 196)
   ...        p50 score 9, max 21
```

**Every top pixel independently votes phase 27.** That unanimity is a far stronger discriminator than
score alone — a noise pixel scores 9 at a random phase; the beacon's pixels agree.

**Stage 3 — cluster the point cloud** (operator: *"we do need to figure out how to find near neighbors
when close in as a point cloud"*). Flood-fill on **adjacency AND phase agreement**:

```
weight  phase   centroid native(x,y)   px   events
   773     27    ( 351.33, 195.68)     59    2766     <- the beacon
    13     16    ( 350.00, 194.08)      2     118
    11     13    ( 349.00, 193.00)      1      77
```

**59× margin**, 59 pixels collapsing to one sub-pixel centroid, stragglers are single pixels at scattered
phases. **Phase is the right clustering axis**: spatial adjacency alone would merge two nearby beacons,
while phase agreement separates them — so the same mechanism handles the close-in point cloud *and* the
two-target case.

## 6. Exposure control may be droppable on this path

Operator: *"this probably doesn't need much exposure control — close in bloom should be ok."*

The reason it holds: **an event is a delta, and a saturated core still swings 255 → 0 when the code goes
dark.** Saturation costs the event detector nothing — in sharp contrast to the centroid photometry it
wrecked during the [range ladder](results/stage1-indoor-range-ladder.md), where a clipped blob's centroid
drifted 10 px and the flux sum became a lower bound.

If that survives a close-range test it retires the reactive AGC, which has been a known defect since
`lit60` (it sat pinned at 53 µs while the beacon struggled) and which
[ac-coupling-and-exposure.md](ac-coupling-and-exposure.md) already argued should be replaced by
range-fed-forward control. The event path may not need either.

**Not yet tested**: a genuinely close, heavily bloomed target. The clip used here is at pendulum range.
That is the next cheap experiment — and the existing near-field range-ladder clips may already answer it.


## 7. Reflections share the phase — the one case clustering cannot solve

Operator: *"stragglers could be specular reflection or lens flare — so one more pass to find the densest
point cloud may be needed."*

Correct, and it exposes a limit in §5's clustering that is worth stating precisely. **A specular
reflection travels a path longer by centimetres — nanoseconds — so it carries the SAME code at the SAME
phase.** Phase agreement, which cleanly separates two different emitters, is *blind* to a reflection of
one emitter. Two clusters sharing a phase are therefore either two reflections or a beacon and its
mirror, and no temporal test distinguishes them.

That leaves **amplitude and density**, which is exactly what spec §9's mirror-pair rule already
arbitrates — and it flags the lower member `MULTIPATH_SUSPECT` and **keeps** it rather than deleting it.

**So the detector's job stops at surfacing the group.** `event_probe.py` now reports same-phase groups
(stage 4) with weight, pixel count and compactness, and leaves the decision to the mirror rule.
Duplicating that arbitration inside the detector would be a second place to get it wrong.

**⚠️ Stage 4 is UNTESTED against a real reflection.** On `rng_8m.bcnr` it finds a single dominant cluster
(weight 149 vs 2, a 75× margin, 13 coordinates touched) and **no mirror pairs — because that clip has
none.** The five modulating sources found live during the range ladder were at the 38 ft station, which
was never recorded, and 8.15 m was one of the healthy stations.

**The experiment that would settle it**, and it is cheap: beacon plus a deliberate reflective surface
(mirror, glass, or the hallway geometry that produced the 4.83 m null), recorded to a clip. That would
validate stage 4 **and** the §9 mirror rule against a known-truth mirror pair — which has never been
done. T055 was unit-tested against *injected* pairs, never against a real reflection.


## 8. A 200 s capture with pinned exposure — the operating point, measured

`/data/event200.bcnr`, 57579 frames, 0 seq gaps, recorded with `beacon-event.ini`: **exposure pinned at
53 µs and gain pinned at 1.0**, matching what `event_view` ran at. The bench config's AGC would have used
gain 2.0 and a drifting exposure, doubling the noise and making the event rate move for reasons unrelated
to the scene.

### The noise floor is a CLIFF, not a tail

| thr | events/frame |
|---|---|
| 5 | 1055 |
| **8** | **30.7** |
| 10 | 19.4 |
| 40 | 7.5 |

**A 34× drop between 5 and 8.** The noise is a sharply-bounded population at |Δ| ≤ 7 with almost nothing
between it and the signal. That is why an earlier reading of "12500 events/frame" looked alarming: thr=5
sits *inside* the noise population. **Set the threshold just above the cliff** — here 8–10 — and the same
scene that looked hopeless gives ~20 events/frame.

The adaptive servo independently settled on 8, which validates rate-targeting as the control.

**And it is stationary**: 1670.5 events/frame at t=10 s against 1670.4 at t=180 s. No drift across 200 s,
so a fixed threshold above the cliff is viable and the servo is insurance rather than a necessity.

### The integration window trades margin against trail length

| window | leader weight | margin over 2nd | cluster span |
|---|---|---|---|
| 74 fr (0.26 s) | 65 | 1.6× | 16 px |
| 150 fr (0.52 s) | 467 | **8×** | 17 px |
| 300 fr (1.04 s) | 1385 | **12×** | 31 px |
| 900 fr (3.1 s) | — | phases scatter, cluster fragments | — |

**A moving beacon does not make a blob, it makes a TRAIL**, and the flood-fill chains along it. A longer
window buys events and margin right up until the trail is long enough that its phase votes become
inconsistent and the cluster breaks apart. At 300 frames the span is 31 native px — the beacon moved
15.5 M2 px in 1.04 s, about **8 °/s**.

**So the event detector inherits the same integration-window-versus-motion trade as the correlator.** It
removes the acquisition lottery; it does **not** escape the coherence limit. And the centroid of a trail
lags the true position, which matters when handing a fix to the tracker.

### The clutter is persistent, not transient

40–56 clusters survive in this room at thr=10, and they are still there at t=180 s. The leader beats them
**8–12×**, so they lose decisively on weight — but they are a permanent feature of a lit indoor scene,
not noise that averages away. That is the question the live view could not answer.


## 9. Outdoor adaptation — and a flaw in the rate-targeting servo (operator, 2026-08-27)

**Operator**: *"outdoor, AGC is tricky — prob during scene init we DO want to adjust the threshold, but
during flight if the sun comes into view, that is ok, we don't want to adjust down. Again, the beacon is
constant illumination, the noise background floor rises and falls, so not sure we need much exposure
control vs the delta threshold and only a little."*

That framing exposes something wrong with the `--target-events` servo built above, and it is worth
recording before anyone relies on it in the field.

### Rate-targeting is confounded by the thing it is looking for

The servo raises the threshold when the event rate rises. But **a real target arriving raises the event
rate**, so the servo responds by raising the threshold and suppressing the target it is supposed to find.
It cannot distinguish "the sun came into view" from "the beacon appeared", because both look like more
events. That is the operator's *"we don't want to adjust down"* in its precise form: the control variable
is wrong.

**The threshold should track the NOISE FLOOR, not the event rate.** The floor is measurable independently
of any target — e.g. a high percentile of |Δ| over the field excluding confirmed tracks — and it is
immune to a target appearing. The measured cliff makes this easy: the noise population is sharply bounded
at |Δ| ≤ 7 with almost nothing between it and the signal, so "just above the floor" is a well-defined
place to sit rather than a compromise.

### Asymmetric adaptation: free at init, slew-limited in flight

The operator's split is the right shape and has precedent in this codebase (the DPLL rate loop is parked
for exactly this class of reason — a loop that adapts freely on a noisy statistic destabilises).

- **Scene init**: adapt freely. The floor is unknown and could be anything from 0.02 ADU (dark, filtered)
  to whatever the sky gives.
- **In flight**: slew-limit hard, and asymmetrically. Letting the threshold rise slowly is safe; letting
  it *fall* fast on a transient is what loses a target. A sun transit should cost margin, not lock.

### Exposure control probably is not needed, and there is a reason

**The beacon is a constant-brightness source**, so at fixed exposure and gain its frame-to-frame delta is
a **constant** — a known quantity that only range changes. The background is what varies. Exposure control
would move both, coupling a disturbance into the one quantity that was stable. The event detector wants a
*fixed* exposure and a *tracking* threshold, which is the opposite of an AGC.

This also matches §6: an event is a delta, so a saturated core still swings 255 → 0 and bloom costs
nothing. Exposure control was there to prevent saturation, and saturation is not a problem on this path.

### The hard limit is physics, not tuning — and sky was never measured

An event needs the beacon's delta to clear the background's shot noise (√2 worse for a difference of two
frames). At 5 σ:

| beacon delta | max tolerable background |
|---|---|
| 255 ADU | 1300 ADU |
| 96 | 184 |
| 46 | **42** |
| 20 | 8 |
| 10 | **2** |

Against the indoor range ladder's measured deltas (53 µs, gain 1): at 2.54 m the beacon tolerates a
background of 1300 ADU; at 8.15 m only **42**; and a far-field target at delta 10 tolerates **2 ADU**.

**Indoor backgrounds ran 0.02 (dark, filtered) to 4.3 (lit room)** — enormous margin everywhere. **Sky was
never measured**, and it is the case where the background is large *and* structureless. So the honest
position is that nothing here predicts outdoor behaviour: the detector could have ample margin or none,
and which one depends on a single number we have not got.

**The one measurement that would settle it**: point this camera at the sky through the bandpass at 53 µs
and read the background level. That is a five-minute experiment requiring no beacon, no pendulum and no
flight, and it converts the entire outdoor risk from speculation into arithmetic against the table above.
It should happen before the field campaign, not during it.

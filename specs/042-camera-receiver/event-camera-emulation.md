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

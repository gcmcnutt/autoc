# pend5 — three swings, three regimes: the envelope, a real reflection, and a gate fix (2026-08-30)

Operator ran the first live session with the trail search and deliberately made it hard: *"the first
swing is at perhaps 3 m, the second is perhaps 1.5' and the third is sideways (so lower intensity) AND
has the pole obscuration — it also goes a bit beyond the angle limit."* 154 s, 44438 frames, 0 seq gaps.
This clip earned its deep dive: every segment found something different.

## Swing 1 (14–74 s, ~3 m) — the trail search doing its job on live-recorded data

LOCK 40/40 in every 2 s bucket, measured fixes 22–40/40, tracked velocity to **73 M2 px/s = 40 °/s**.
The same regime pend4 scored 100 % on replay, now reproduced from a live session. No surprises — which is
the point.

## Swing 2 (76–110 s, ~45 cm) — beyond the envelope, and the failure was UGLY rather than honest

Measured from the clip (region-restricted blob tracking): **p50 62 °/s, p90 91, max 115 °/s** — close
range multiplied physical speed ~6.7× into angular rate. The trail grid caps at vmax = 64 M2 px/s =
**35 °/s**, so **79 % of the swing had no correct hypothesis in the grid at all**, and a 115 °/s trail
(54 M2 px per word) does not even fit inside the ±24 M2 px crop.

The defect was what happened *next*: the best wrong in-grid hypothesis scored q 0.55–0.68, cleared the
`q_lock` acceptance bar, and — because a trail fix REPLACES the state — teleported the track across the
field, tick after tick (x excursions −63…+89 M2 px), without even setting `measured_fix`.

**Fixed**: trail-fix acceptance is now `q_fix` (0.75), the measured real/junk separation bar, not
`q_lock`. Swing-2 teleports (state jumps > 10 M2 px/tick) fell 61 → 36 with the remainder being mostly
*genuine* motion at 115 °/s (= 10.5 M2 px/tick — right at the metric's threshold, so the metric
saturates there). Below the bar, HOLD's honest coast beats a confident teleport. **pend4 regression:
unchanged — 4192 fixes, 0 false, 100 % at ≥8 °/s** (the search's real fixes score ~0.99; the gate is
free where it works).

**The envelope, stated as a design fact rather than a bug**: goal-line transits at close range reach
100+ °/s, and that regime belongs to the full-field event path (a trail crosses the whole 147° field in
~1.3 s — no per-track crop can follow it). The trail bank owns ~0–35 °/s (extendable modestly by
widening grid+crop at quadratic cost); the event detector owns the transit case. They meet in the
middle, which is what the layered-estimator plan already assumed.

## The "second source" — RETRACTED: it was the beacon at the right swing extreme

The first pass concluded a second code-carrying source at native (400–450, 144–176) — a reflection or
illumination patch. **Wrong, and the operator's question ("was it lens flare?") prompted the test that
settled it.** Three measurements, in order of decisiveness:

1. Only **36 of 5182** frames show the "beacon region" and the "source2 region" bright simultaneously —
   and those 36 sit exactly at the analysis-box boundary (x ≈ 395). The two appearances are mutually
   exclusive in time.
2. Brightness in the two regions is *anti*-correlated (−0.18) — when one is lit the other is dark.
3. The decisive one: the per-frame argmax trajectory through 92–97 s is **one continuous sweep**,
   x 237 → 424 and back every ~1.9 s, with y RISING at the extremes — a pendulum arc, exactly.

So "source2" was the beacon itself at the right extreme of the close swing; the smooth (448,144) →
(400,176) "drift" was the extreme retreating as the swing decayed. Neither a reflection nor lens flare —
an artifact of splitting one trajectory across two analysis boxes. (A ghost flare would also have failed
the geometry test: a point-reflection through the optical centre lands in the LOWER half here.)

The `MULTIPATH_SUSPECT` flags at 112–118 s are then two tracks on ONE emitter — a stale coasting
CONFIRMED track paired with the freshly re-acquired beacon — which is the mirror rule doing double duty
as a stale-track detector. The real-reflection test of T055/stage 4 therefore REMAINS UNDONE (the
deliberate-mirror experiment from event-camera-emulation.md §7 is still the way to do it), and the
"upper keeps CONFIRMED" concern raised here is withdrawn along with the claim that prompted it.

**Analysis trap recorded**: two regions that are never bright in the same frame are not two sources —
check trajectory continuity before declaring a second object.

## Swing 3 (112–152 s, sideways + pole) — the emitter's angle limit, visible as the ladder's choice

Lower intensity throughout (LEDs aimed off-axis, "beyond the angle limit" at the extremes): saturated
pixel count 1–2 vs swing 1's 3–8, and the ladder sat at COARSE (scale 0) for the whole segment — the
widest aperture compensating for the weakest signal, exactly as designed. Still LOCK 40/40 with measured
15–39/40 through pole occlusions. This is the closest existing fixture to the far-field SNR regime.

## Two analysis traps met along the way (kept so they are not re-met)

- **Median of a bimodal argmax fabricates positions.** Bucketed medians of per-frame argmax mixed
  all-dark frames (argmax = (0,0)) with beacon hits (~x 280) and reported a phantom source at x ≈ 140.
  The direct scan (list every non-beacon argmax, cluster) is the honest method.
- **An all-zero frame's argmax is (0,0)**, and a bucket's max-peak column can still show 255 from other
  frames in the bucket — the pair looks like a hot pixel at the origin. It is a dark chip.

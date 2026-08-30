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

## The second source — a REAL code-carrying reflection, and the mirror rule's first live engagement

From ~84 s a second saturating source appears at native (400–450, 144–176), drifting smoothly
(448,144) → (400,176) over 26 s. Measured: **it carries code B as strongly as the beacon does** (corr
ratio 0.27 vs 0.29). At 45 cm the beacon floods nearby surfaces — this is an illumination patch /
diffuse reflection, i.e. *"reflections share the phase"* (event-camera-emulation.md §7) made real. The
tracker locked it at **q = 0.97** at t = 80.

The mirror-pair rule (T055) engaged for the first time against a real reflection: `MULTIPATH_SUSPECT`
flags fired on 8–16 ticks/bucket at 112–118 s. **But the geometric half of the rule is inverted here**:
"the upper track keeps CONFIRMED" assumed ground reflections below the beacon, and this patch sits
UPPER-RIGHT of the true beacon — so the rule would keep the reflection and flag the truth. Recorded as a
known limitation: at close range (and near structures) the arbiter needs amplitude/track-history, not
geometry. Stage 4's "surface the group, let the rule decide" design stands; the rule itself needs work
before the field.

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

# Flight 2026-05-03 — pastonly3 (gen 391, no-future arch, intermediate)

**Date**: 2026-05-03 ~12:14 local (two flights, ~17:34 / ~18:13 INAV-side)
**NN weights**: **pastonly3** / 029-no-future-arch, **gen ~391
mid-training pull** (`gen9609.dmp`) under run id
`autoc-9223370259105171692-2026-05-02T19:20:04.115Z` — flown S3 artifact.
**pastonly3 config (vs pastonly2)**: `SimNumPathsPerGeneration = 6`
(path 5 re-included, having been excluded in pastonly2 — see
[`pastonly2_outcome.md`](../../specs/029-no-future-arch/pastonly2_outcome.md)),
49 wind variations per path → **294 scenarios per gen** (vs 245 in
pastonly2). pastonly3 is intentionally pastonly2's path-distribution
fix-up; this flight pulled it mid-run before convergence.
**Xiao binary**: 029 tree, past-only sensor input (5 lookback ticks + now,
6 slots indexed 0..5). Same airframe as 04-26.
**INAV**: same fork as 04-26 (world→body blackbox quat, MSP override
engage path).
**Wind**: pilot reported nominal/light (similar to 04-26).

**Overall**: **first real-flight test of the no-future architecture.**
Six engage spans across two flights covering **all six configured paths
(0–5), every one terminated by natural path-end** (one pilot disable on
the longest, span 3 of flight 1, after 31.5 s on path 5). Pitch axis
quieted dramatically vs 04-26 cadence7-redux (porpoise gone); the
bang-bang dominant axis **migrated to roll**, exactly matching the
late-pastonly2 sim signal of "sustained moderate roll commands /
spiral-tracker patrol." The airframe sustained the evolved spiral
strategy through every path; real tracking quality is positive forward
lock on 5/6 spans (the 6th, path 3, is a 13-segment configured short
path, not an abort).

## Engage spans

Six engage spans, six paths, six natural terminations:

| Span | Flight | Path | Path name           | Termination          | Engage dur. | NN evals |
|------|--------|------|---------------------|----------------------|-------------|----------|
| 1    | 1      | 0    | StraightAndLevel    | path complete (17.0 s rabbit) | 19.20 s     | 170      |
| 2    | 1      | 2    | HorizontalFigureEight | path complete (21.9 s)        | 22.17 s     | 219      |
| 3    | 1      | 5    | SeededRandomB       | pilot disable (after 31.5 s)  | 31.48 s     | 315      |
| 4    | 2      | 1    | SpiralClimb         | path complete (21.1 s)        | 23.78 s     | 211      |
| 5    | 2      | 4    | HighPerchSplitS     | path complete (25.6 s)        | 27.49 s     | 256      |
| 6    | 2      | 3    | FortyFiveDegreeAngledLoop | path complete (6.8 s; 13/1000 segments configured) | 9.29 s | 68 |

vs 04-26 cadence7-redux: 3 path-end spans (19.5 / 24.1 / 23.2 s), all
on different paths. Today's session is **2× the path coverage and
matches duration on the comparable paths**. Path 5 (SeededRandomB —
the held-out OOD geometry + random-intercept proof per
[project_path5_random_intercept](../../.claude/projects/-home-gmcnutt-autoc/memory/project_path5_random_intercept.md))
was flown for the first time and ran 31.5 s before pilot called it.

## Convention / sensor audit

Per-span anchors via the new
[`specs/029-no-future-arch/join_flight_analysis.py`](../../specs/029-no-future-arch/join_flight_analysis.py)
(029 fork: "now" slot = index 5, not 3; per-span aggressiveness block
added). All 6 spans:

| Anchor                                | Status                                          |
|---------------------------------------|-------------------------------------------------|
| A. Gravity (FRD body Z)               | ✓ pass, all spans (mean accZ +3.3 to +4.9 k)    |
| B. xiao quat ↔ INAV blackbox          | ~ ok, RMS 0.32–0.39, sample-time slop (xiao 100ms vs INAV 2ms) — INAV stored as `world→body`, xiao as `body→world` (script picks the right convention) |
| C. cos(nose, vel) using xiao quat     | +0.87–0.94 ✓ forward flight all spans           |
| D. Control chain (end-to-end rcData→gyro) | +0.67 to +0.79 @ 10–14 ms, every stage matches the build's expected polarity |

Same chain shape as 04-26: msplink intentionally inverts pitch, INAV
elevon mix inverts both servos, end-to-end rcData→gyro is positive on
every span. No regression.

## Control behaviour — vs 04-26 cadence7-redux

Pooled across **all 6 spans, 1239 NN ticks**:

| Metric                              | 04-26 cad7-redux | 0503 pastonly3   | Δ                      |
|-------------------------------------|------------------|------------------|------------------------|
| `<\|out\|>`  total / tick           | 2.11             | 2.31             | +9 %                   |
| `<\|Δout\|>` total / tick           | 0.88             | **1.26**         | **+43 % (more chatter)** |
| `<\|pt\|>`                          | 0.76             | **0.66**         | **−13 % (pitch quieter)** |
| `<\|rl\|>`                          | 0.40             | **0.69**         | **+73 % (roll louder)** |
| `<\|th\|>`                          | 0.95             | 0.96             | flat (Vmax-pinned)     |
| roll near-zero band ([−0.3, +0.3])  | 52 %             | **12 %**         | **roll no longer quiet** |
| pitch near-zero band                | (n/a)            | 23 %             | (new datum)            |
| pitch saturation ( \|·\| > 0.95 )   | 8 %              | 9 %              | flat                   |
| roll  saturation                    | 9 %              | 12 %             | mild rise              |
| throttle saturation                 | 88 %             | 91 %             | unchanged (Vmax)       |

**Per-axis Δout (the no-future-specific axis-migration signal):**

| Axis     | `<\|Δ·\|>` per tick | Sign-flip rate (tick-to-tick) |
|----------|---------------------|-------------------------------|
| Pitch    | 0.32                | 24 %                          |
| **Roll** | **0.81**            | **49 %** (≈ flips every other tick) |
| Throttle | 0.13                | low (pinned)                  |

This is **textbook bang-bang on roll**, mild oscillation on pitch.
The pitch axis on 04-26 was the noisier one (`<|pt|>` = 0.76). Today
pitch is the calmer axis (`<|pt|>` = 0.66, `<|Δpt|>` = 0.32). The
controller did not get smoother in aggregate — it got **smoother on
pitch** while picking up roll-axis activity. That matches the
[project_bangbang_axis_migration](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md)
pattern (dominant bang-bang axis migrates across controllers) and the
[project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md)
story (no-future controllers exchange smooth pitch for high roll-rate
spiral coverage).

## Target acquisition (the C0 anchor)

Per-span mean target body-frame X (forward direction-cosine; +1 = nose
on target) and tX rate of change:

| Span | Path | mean tX | dtX/dt    | mean dist | climb        | Verdict                |
|------|------|---------|-----------|-----------|--------------|------------------------|
| 1    | 0    | +0.604  | −0.003 /s | 14.1 m    | −0.89 m/s    | steady (rabbit-chase eq.) |
| 2    | 2    | +0.691  | +0.001 /s | 12.3 m    | +0.20 m/s    | steady                 |
| 3    | 5    | +0.676  | **+0.047 /s** | 31.1 m    | +2.32 m/s    | **target coming toward nose** ✓ |
| 4    | 1    | +0.555  | −0.044 /s | 12.5 m    | +0.77 m/s    | drifting (steady ≈ noise) |
| 5    | 4    | +0.583  | −0.012 /s | 12.7 m    | +0.45 m/s    | steady                 |
| 6    | 3    | +0.113  | −0.004 /s | 19.3 m    | −1.66 m/s    | weak lock (13-segment configured short path) |

NN pitch → dtZ/dt correlation **+0.18 to +0.53** (right sign on every
span) and NN roll → dtY/dt correlation **−0.55 to −0.61** (consistent
sign across all spans — control conventions intact, no flipped axis).

## End-to-end physics

Cmd → gyro lag and correlation per span:

| Span | rcData[0] → gyro p (roll) | rcData[1] → gyro q (pitch) |
|------|---------------------------|----------------------------|
| 1    | +0.77 @ 12 ms             | +0.75 @ 12 ms              |
| 2    | +0.76 @ 12 ms             | +0.76 @ 12 ms              |
| 3    | +0.77 @ 12 ms             | +0.75 @ 10 ms              |
| 4    | +0.77 @ 12 ms             | +0.78 @ 12 ms              |
| 5    | +0.74 @ 12 ms             | +0.79 @ 12 ms              |
| 6    | +0.67 @ 10 ms             | +0.72 @ 12 ms              |

Identical lag/correlation envelope to 04-26 (~+0.78 @ 8–14 ms). Aero
and servo response unchanged.

## Departure / short-stall events

Scanned each engaged span for low-airspeed dropouts (`as < 8 m/s` for
≥ 3 consecutive NN ticks ≈ ≥ 0.3 s) and high-pitch + low-airspeed
co-occurrences (`pitch > +25°` AND `as < 10`):

| Span | Path                | as min/mean/max  | pitch° min/mean/max | low-as runs (≥ 3 ticks)            | high-α stall-like ticks |
|------|---------------------|------------------|---------------------|------------------------------------|-------------------------|
| 1    | StraightAndLevel    | 8.7 / 15.0 / 22.7 | −78 / +5 / +81      | none                                | 5 at t = 1.0 s (entry pull-up) |
| 2    | HorizontalFigureEight | 4.2 / 16.0 / 21.7 | −46 / +7 / +57    | one 0.4 s @ t = 10.4 s (min 4.2)    | 3 at t = 10.2 s (recovered) |
| 3    | SeededRandomB (path 5) | 4.5 / 13.2 / 25.1 | −76 / **+24** / +87 | **four 0.2–0.5 s** @ 10.8 / 18.6 / 24.7 / 28.9 s | **50 ticks** ≈ 5 s, starting t = 4.7 s |
| 4    | SpiralClimb         | 8.2 / 15.4 / 21.6 | −77 / +7 / +63      | none                                | 0                       |
| 5    | HighPerchSplitS     | 7.7 / 15.2 / 22.0 | −87 / +7 / +77      | none                                | 2 at t = 2.2 s          |
| 6    | FortyFiveDegreeAngledLoop | 8.6 / 18.6 / 25.3 | −30 / +7 / +89  | none                                | 2 at t = 2.3 s          |

**No span departed.** All low-airspeed dropouts are brief (≤ 0.5 s)
and the controller flew through them without uncommanded pitch breaks
or sustained dives. The longest stall-like residence is on **span 3
(path 5)**: ~ 16 % of the span has `pitch > +25° AND as < 10` —
consistent with the pastonly-policy "spiral patrol" behaviour
(sustained nose-up while ranging the rabbit) rather than a stall
event. Path 5's mean pitch is **+23.5°** (vs +5–7° on the other
spans), confirming this span ran nose-high for much of its 31.5 s.

**Caveat on the airspeed sensor**: the pitot reading at very low
values (~ 4 m/s) likely includes wind component / dynamic-pressure
noise rather than true free-stream — order-of-magnitude indicator
only. The gross signal (no extended low-as runs, no nose-down
recovery dives) is what's load-bearing here.

**Implication for the spiral-strategy hypothesis**: if the controller
were on the edge of envelope, the brief low-as runs on path 5 would
have triggered a departure. They didn't. The current fixed-wing
appears to have margin against the evolved spiral strategy at gen
391; whether that margin holds at gen 575+ (more aggressive
late-evolution roll commands) is the next test.

## What flew vs what didn't

- **Architecture**: 029 past-only — 5 lookback ticks + now, no future
  lookahead. Confirmed in xiao log via the 6-slot `tX/tY/tZ` array
  with shift-register evolution (slot[5] = now, slots[0..4] = past).
- **Weights**: gen ~391 (mid-pastonly3 run, intentional intermediate
  pull and field-flash before pastonly3 converges). Spec-comparable
  end-of-run elite is still training; this flight does **not**
  validate converged pastonly3. It validates the **architecture +
  the pastonly3 path-distribution fix** at a working fitness level
  ~halfway through training.
- **Run lineage**: pastonly3 — the pastonly2 follow-up that
  re-includes path 5 (random-aerostandard) in the training joint-PRNG.
  pastonly2 carries the OOD-brittleness finding from 2026-05-02 (see
  [project_evolved_strategy_vs_airframe](../../.claude/projects/-home-gmcnutt-autoc/memory/project_evolved_strategy_vs_airframe.md)
  and [`pastonly2_outcome.md`](../../specs/029-no-future-arch/pastonly2_outcome.md)
  §"path-5 exclusion") that motivated pastonly3. Today's gen-391
  controller has *seen* path-5 geometries during training (unlike
  pastonly2), so its real-flight path-5 result is not subject to the
  pastonly2 caveat.
- **Tracker mode (030)**: NOT flown. Today's controller still uses
  rabbit-oracle inputs (tX/tY/tZ/d direction cosines), not beacon
  features. 030 is parked behind 025 craft variations.
- **025 craft variations**: NOT flown.
- **Prediction-aiding (T061-T065)**: NOT enabled (parked on backlog).
- **X-wing airframe**: NOT used (current fixed-wing trainer; per
  2026-05-01 decision, accept spiral on current airframe for M1/M2).

## Implications

- **The no-future architecture flies on real hardware.** First time
  flown; six paths flown without crash; all naturally terminated; all
  control-chain anchors green; forward target lock on the held-out
  path 5 (random-intercept geometry). This is **the real-flight signal
  the 2026-05-01 architecture-vs-airframe decision was waiting for** —
  the airframe sustains the evolved spiral strategy at gen 391.
  Whether **converged pastonly3 (gen ~575+)** sustains it equally
  well is the next flight test, not this one.
- **Bang-bang axis migrated pitch → roll, as the sim training
  predicted.** 04-26 had pitch as the noisy axis (0426 cadence7-redux:
  `<|pt|>` = 0.76, pitch saturation flat). 0503 has roll as the noisy
  axis (`<|rl|>` = 0.69, sign-flip 49 %). Total `<|Δout|>` is **+43 %
  vs 04-26** — the controller is *not* smoother overall, just
  redistributed. This matches the late-pastonly2 sim observation
  (sustained moderate roll commands, intensifying spiral coverage as
  fitness pressure tightens).
- **Pitch porpoising gone, roll wiggle present** — pilot's
  qualitative report is borne out by the data on every metric.
- **Real tracking quality**: forward target lock (mean tX +0.55 to
  +0.69) on 5/6 spans, with the 6th being a configured-short path
  rather than a tracking failure. Path 5 (OOD random-intercept) ran
  31.5 s with `dtX/dt = +0.047 /s` — the controller is actively
  pulling the target toward the nose on the OOD geometry.
- **Open question this flight does NOT settle**: whether `<|rl|>` and
  the 49 % roll sign-flip rate are *acceptable* for downstream
  milestones (real beacons, virtual-beacon flight test, real-target
  tracking). The airframe held; the question is whether the spiral
  strategy preserves enough margin for added perception noise and
  real-target intercept geometry. That will fall out of M1.3-style
  flight tests on the **converged pastonly3** controller and on 025
  craft-varied controllers — not this one.

## Hypotheses worth a discussion thread

1. **M1.3 is a wrap on the architecture but not on the converged
   weights.** The architecture flies; the pastonly3 gen-391 mid-run
   weights fly; **converged pastonly3 weights (gen 575+)** are the
   right thing to fly next, on a calmer day, to compare
   apples-to-apples vs gen 391 on the same airframe and weather.
   Question: do we re-fly with the converged pastonly3 elite before
   declaring M1.3 done, or treat gen 391 as sufficient evidence and
   route to 025?
2. **The roll-axis bang-bang is the predicted spiral strategy
   surfacing in flight.** If true, we should see roll airframe rate
   (gyro p RMS) ≈ several × the path-required roll envelope, similar
   to the sim per-path roll-degree totals at gens 400–575. Quick
   test: integrate `gyro p` over engaged ticks per span; compare to
   path-geometry roll requirement. Easy follow-on.
3. **Sim-to-real on roll wiggle is consistent with the 028 / 029 sim
   evolution signal.** Total `<|Δout|>` 1.26 / tick on flight is
   higher than the spec-gate per-axis budget (~0.27 × 3 = 0.81), but
   that gate was set against with-future controllers. Question: does
   the past-only spec gate need to shift, or is roll-axis budget
   simply the wrong shape for spiral-tracker policies?
4. **Path 3 (45° loop) was the only weak-lock span.** Mean tX +0.11.
   Two non-exclusive explanations: (a) the path is configured short
   (13/1000 segments) so the rabbit moves through too quickly for
   stable tracking; (b) 45-degree-angled-loop geometry is genuinely
   harder for the past-only controller. Which one matters depends on
   how often path 3 will get re-flown.
5. **Throttle is still Vmax-pinned (91 % saturation).** Identical to
   04-26. Same hardware envelope problem. Doesn't gate anything but
   worth noting that path-energy / Vmax mismatch hasn't been
   addressed and will eventually need a path-rabbit speed retune.

## Artifacts

All in this directory:

- `bangbang_flight_*.png` (6) — per-span output time series + |Δout|
  + |out| + per-axis distribution from
  [`specs/024-sim-real-fidelity/plot_bangbang_flight.py`](../../specs/024-sim-real-fidelity/plot_bangbang_flight.py).
- `join_analysis_*.png` / `.csv` (6) — per-span 10-panel diagnostic
  from the new
  [`specs/029-no-future-arch/join_flight_analysis.py`](../../specs/029-no-future-arch/join_flight_analysis.py)
  (slot[5] = now; per-span aggressiveness block).
- `blackbox_log_2026-05-03_121443.{01,02}.csv` — INAV blackbox CSVs.
  `.01` is flight 1 (xiao log `T17-34-11`); `.02` is flight 2 (xiao
  log `T18-13-21`).
- `flight_log_2026-05-03T17-34-11.txt` — xiao log paired with `.01`.
- `flight_log_2026-05-03T18-13-21.txt` — xiao log paired with `.02`.

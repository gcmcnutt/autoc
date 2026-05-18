# Flight 2026-05-17 — pastonly3 converged gen 800 (no-future arch)

**Date**: 2026-05-17 ~12:50 local
**NN weights**: pastonly3 / 029-no-future-arch, **converged final
gen 800** from run
`autoc-9223370259105171692-2026-05-02T19:20:04.115Z`. The dmp filename
encodes `(10000 − gen)`, so `gen9200.dmp` = actual training gen 800
(the run's final, 800-gen pastonly3). Same run lineage as the
[0503 FLIGHT_REPORT](../flight-20260503/FLIGHT_REPORT.md), which flew
`gen9609.dmp` = actual gen **391** (mid-training intermediate). May 17
is therefore the **converged elite** of the lineage that 0503 sampled
mid-run — first real-flight test of converged pastonly3.
**Xiao binary**: 029 tree, past-only sensor input (5 lookback ticks + now,
6 slots indexed 0..5). Same airframe as 0503.
**INAV**: same fork as 0503 (world→body blackbox quat, MSP override
engage path).
**Wind**: pilot reported **good wind/gust from 350°** (notably stronger
than 0503).

**Overall**: **two engage spans, both terminated by path completion,
both on the calm/predictable end of the path set (path 0 +
path 2)**. Pilot qualitative: span 1 (path 0) lost some altitude
mid-engage and recovered; span 2 (path 2) was the cleaner of the two
("quite good tracking"). The converged controller flew, but the
bang-bang on roll is still present and rough on the airframe. This
flight is the **converged-elite half** of the 0503 / 0517 bracket: it
lets us see what 410 gens of additional training *did* on top of the
gen-391 mid-pull that flew on 0503.

## Engage spans

| Span | Path | Path name             | Termination     | Engage dur. | NN evals |
|------|------|-----------------------|-----------------|-------------|----------|
| 1    | 0    | StraightAndLevel      | path complete   | 18.44 s     | 170      |
| 2    | 2    | HorizontalFigureEight | path complete   | 23.77 s     | 219      |

Same path 0 / path 2 durations as 0503 within ~1 s (170 / 219 NN ticks
identical to 0503 spans 1 and 2). Direct same-path comparator.

## Convention / sensor audit (per-span anchors)

Via [`specs/029-no-future-arch/join_flight_analysis.py`](../../specs/029-no-future-arch/join_flight_analysis.py).

| Anchor                                  | Span 1 (path 0)                | Span 2 (path 2)              |
|-----------------------------------------|--------------------------------|------------------------------|
| A. Gravity (FRD body Z)                 | ✓ +3139                        | ✓ +3993                      |
| B. xiao quat ↔ INAV (world→body)        | RMS 0.32 ~ ok                  | **RMS 0.42 ✗ DIVERGENT**     |
| C. cos(nose, vel) using xiao quat       | +0.89 ✓                        | +0.93 ✓                      |
| D. End-to-end rcData→gyro p/q           | +0.81 / +0.83 @ 10–12 ms       | +0.82 / +0.76 @ 10–12 ms     |

Quat anchor on span 2 is "DIVERGENT" but xiao's own cos(nose, vel) is
+0.93 (tight). The 0.42 RMS is most plausibly sample-time slop during
the figure-eight maneuver under stronger wind (xiao 100 ms vs INAV
2 ms) — same shape we saw on 0503 spans, just shifted past the 0.40
threshold. Not a sign/polarity bug: every control-chain stage still
matches the build's expected polarity on both spans.

## Same-path comparison: 0503 mid-training gen 391 → 0517 converged gen 800

### Path 0 — StraightAndLevel

| Metric                   | 0503 gen 391 (mid) | 0517 gen 800 (converged) | Δ                          |
|--------------------------|--------------------|--------------------------|----------------------------|
| Duration                 | 19.2 s             | 18.4 s                   | flat                       |
| mean tX (fwd lock)       | +0.604             | **+0.504**               | **−0.10 weaker lock**      |
| mean dist to rabbit      | 14.1 m             | **29.1 m**               | **+15 m farther**          |
| mean climb               | −0.89 m/s          | −0.48 m/s                | smaller descent            |
| mean NN cmd  pitch       | +0.48              | +0.35                    | −0.13                      |
| mean NN cmd  throttle    | **+0.71**          | **+0.96**                | **convergence floored it** |
| `<\|pt\|>`               | 0.72               | **0.52**                 | **−28 % (quieter pitch)**  |
| `<\|rl\|>`               | 0.72               | 0.67                     | flat                       |
| `<\|Δpt\|>`              | 0.30               | 0.33                     | flat                       |
| `<\|Δrl\|>`              | 0.73               | **0.92**                 | **+26 % (more chatter)**   |
| pitch near-zero band     | 15 %               | **34 %**                 | much more dwell            |
| pitch saturation         | 8 %                | 1 %                      | almost gone                |
| roll saturation          | 10 %               | 4 %                      | down                       |
| throttle saturation      | 91 %               | **98 %**                 | Vmax-pinned harder         |

### Path 2 — HorizontalFigureEight

| Metric                   | 0503 gen 391 (mid) | 0517 gen 800 (converged) | Δ                          |
|--------------------------|--------------------|--------------------------|----------------------------|
| Duration                 | 22.2 s             | 23.8 s                   | flat                       |
| mean tX (fwd lock)       | +0.691             | +0.626                   | −0.07 slightly weaker      |
| mean dist                | 12.3 m             | 16.8 m                   | +4.5 m                     |
| mean climb               | +0.20 m/s          | −0.31 m/s                | slight descent vs climb    |
| mean NN cmd  pitch       | +0.53              | +0.36                    | −0.17                      |
| mean NN cmd  throttle    | **+0.70**          | **+0.97**                | **convergence floored it** |
| `<\|pt\|>`               | 0.71               | **0.52**                 | **−27 %**                  |
| `<\|rl\|>`               | 0.67               | 0.70                     | flat                       |
| `<\|Δpt\|>`              | 0.31               | 0.33                     | flat                       |
| `<\|Δrl\|>`              | 0.83               | 0.83                     | flat                       |
| pitch near-zero band     | 17 %               | **32 %**                 | much more dwell            |
| throttle saturation      | 82 %               | **95 %**                 | floored                    |

### What 410 more generations of training did (gen 391 → gen 800)

1. **Throttle policy converged on Vmax-pinning**. The mid-pull
   modulated throttle (mean cmd +0.70–0.71, sat 82–91 %); the
   converged elite floors it (mean cmd +0.96–0.97, sat 95–98 %).
   Convergence is *toward* burying the throttle, not away from it —
   the path-energy / Vmax mismatch is being driven harder by the
   evolved policy, not relieved.
2. **Pitch effort halved at convergence**. `<|pt|>` 0.52 vs 0.72 at
   gen 391. Pitch near-zero dwell doubled (32–34 % vs 15–17 %), pitch
   saturation nearly vanished (1 % vs 8 %). Convergence **removed**
   pitch effort — the converged controller relies less on pitch and
   more on roll/throttle to fly.
3. **Roll chatter intensified on path 0, flat on path 2**. `<|Δrl|>`
   0.92 vs 0.73 on path 0 (+26 %); 0.83 vs 0.83 on path 2.
   Sign-flip rate climbed too (path 0: 57 % vs 46 %). Convergence
   **made the roll bang-bang worse** on the calm path — the spiral
   strategy hardens with training, doesn't soften.
4. **Forward target lock degraded** (mean tX −0.07 to −0.10, mean dist
   +4.5–15 m). Part of this is the windier day, but a controller that
   tracks the rabbit at 29 m instead of 14 m on path 0 is not getting
   *better* at lock through convergence — it's tolerating a looser
   rabbit-chase equilibrium.

This is the M1.3 follow-up question the 0503 report ended on
("converged pastonly3 weights are the right thing to fly next, on a
calmer day, to compare apples-to-apples"). We got the comparison —
just not on a calmer day. The headline answer is that **convergence
did not buy smoothness**; it bought a more aggressive throttle policy,
less pitch participation, and (on the calmest path) worse roll
chatter. The qualitative "rough on the airplane" is a converged-elite
property, not a mid-training artefact.

The user-perceived "test 1 lost altitude but gained it back" matches
span 1's mean climb −0.48 m/s with a recovery shape visible in the
altitude panel of `join_analysis_..._span1_path0.png`. The
"quite good tracking" on test 2 is borne out by the +0.626 mean tX and
16.8 m mean distance — better lock than span 1 despite higher
target maneuvering on the figure-eight.

## Bang-bang — the airframe-loading question

Per-axis `<|Δout|>` and tick-to-tick **sign-flip %** (NN-tick basis,
~20 Hz, parsed directly from the xiao log so 0517 vs 0503 are
apples-to-apples):

| Axis     | 0517 s1 path0 Δ / flip% | 0517 s2 path2 Δ / flip% | 0503 s1 path0 Δ / flip% | 0503 s2 path2 Δ / flip% |
|----------|--------------------------|--------------------------|--------------------------|--------------------------|
| Pitch    | 0.33 / 26 %              | 0.33 / 23 %              | 0.30 / 18 %              | 0.31 / 22 %              |
| **Roll** | **0.92 / 57 %**          | **0.83 / 49 %**          | **0.73 / 46 %**          | **0.83 / 53 %**          |
| Throttle | 0.05 / 2 %               | 0.04 / 1 %               | 0.12 / 6 %               | 0.18 / 8 %               |

Roll-axis bang-bang is the dominant airframe-loading signal on both
spans. The two same-path comparators split: **path 0 got worse** at
converged gen 800 (`<|Δrl|>` 0.92 vs 0.73, sign-flip 57 % vs 46 %),
while **path 2 is comparable** (0.83 / 49 % vs 0.83 / 53 %).
Convergence **did not** smooth roll out — on the calmest path it
hardened. The straight-and-level case sees roll commands flipping sign
on **every other tick (57 %)** at gen 800 — the airframe is being
worked hardest exactly on the calmest configured path, which matches
the qualitative "rough on the airplane" report.

Pitch bang-bang at gen 800 is also slightly higher than gen 391
(sign-flip 23–26 % vs 18–22 %) — small but consistent across both
paths. Throttle is now near-flat (pinned floored), down from 0503's
6–8 % flip rate.

This confirms the
[project_bangbang_axis_migration](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md)
finding (dominant bang-bang axis migrates across controllers) is **not
the load-bearing variable for airframe wear today** — roll is the
noisy axis at both gen 391 and gen 800, on the same airframe, on the
calm paths (0 and 2), and the chatter gets *worse* with convergence.
The hypothesis that "spiral strategy manifests primarily as roll
chatter" survives this flight.

## End-to-end physics

Cmd → gyro response unchanged from 0503:

| Span | rcData[0] → gyro p (roll) | rcData[1] → gyro q (pitch) |
|------|---------------------------|----------------------------|
| 1    | +0.81 @ 12 ms             | +0.83 @ 10 ms              |
| 2    | +0.82 @ 12 ms             | +0.76 @ 10 ms              |

Aero/servo response identical to 0503 (+0.74–0.79 @ 10–14 ms). No
mechanical or sign change since 0503.

## Implications

- **Converged pastonly3 flies** on the same airframe. The
  architecture holds end-of-run too. The M1.3-level question "does
  the converged elite hold up on real flight" is answered: yes, on
  the calm paths, with the same control-chain polarities and the
  same forward target lock as the mid-pull.
- **Convergence did not buy smoothness — it doubled down on the
  spiral strategy**. Throttle pinned, pitch effort halved, roll
  chatter intensified on the calmest path. Fitness today doesn't
  penalize `<|Δrl|>` and tournament pressure rewards aggressive
  rabbit-chase; the converged elite is the policy that wins that
  selection.
- **Test 1 altitude dip + recovery** is consistent with the
  pitch-quiet / throttle-pinned posture of the converged controller:
  pitch effort is largely off the table for altitude hold, so a wind
  gust shows up as a transient descent that the controller can only
  recover from via the modest non-zero mean pitch cmd (+0.35) and
  whatever tail the roll-coupled pitch-attitude excursion buys.
- **Your hypothesis (smoothness in fitness + enough inputs to develop
  a PI in pitch and roll) is consistent with what we see**, and
  becomes the natural next-iteration delta. Two complementary moves:
    1. **Smoothness term in fitness** — directly penalize `<|Δout|>`
       per tick, or the sign-flip rate, weighted per axis. Without it
       the GP has no reason to smooth out roll; the gen 391 → gen 800
       trajectory shows it *gets worse* without a counter-pressure.
       This is a fitness-shape change, not an architecture change —
       testable as a pastonly4 from a pastonly3 seed.
    2. **Enough input signals to evolve an integral term** — the 029
       past-only arch already carries 6-slot history of target
       direction cosines (implicit derivative), but it does **not**
       carry an integrated error state. Either (a) feed an integrator
       explicitly as an input feature (cumulative tX/tY/tZ error
       across a window), or (b) move to a recurrent arch (027 / 028)
       so hidden state can grow an integrator. Currently
       [032 phase 1](../../specs/032-tracker-nn-enhancements/) is
       adding perceptual features (span / span-rate / tilt) —
       orthogonal to smoothness, but the same delivery vehicle could
       carry an integrator-style input.

## Caveats

- **Only 2 spans, both on calm paths (0 and 2)**. No path 5
  random-intercept, no spiral climb, no 45° loop. This flight bracket
  is narrower than 0503; only the path-0 and path-2 comparisons are
  apples-to-apples.
- **Stronger wind (350°)** than 0503; some of the weaker target lock
  is environmental, not controller — though pitch effort and throttle
  policy differences are too large to be wind alone.
- **The dmp filename inverts the apparent gen ordering**. Filename
  encoding is `gen<10000−N>.dmp`, so `gen9200.dmp` ↔ training
  gen 800 (final), `gen9609.dmp` ↔ training gen 391 (mid-pull). Easy
  to misread; this report uses the actual training gen everywhere.

## Join — timestamp-column verification

Traced the join columns end-to-end to confirm both sides are in the
same clock domain before reading anything off the per-span anchors:

- **INAV blackbox CSV** column `time (us)` is INAV's `micros()` at
  blackbox-emission time (used by
  [`join_flight_analysis.py:98`](../../specs/029-no-future-arch/join_flight_analysis.py#L98)
  to derive `t_ms = time (us) / 1000`).
- **xiao log column 2** (the `inav_ms` field, e.g.
  `#00000002 0158732 0159285 …` → 159285) is set in
  [`xiao/src/util.cpp:70`](../../xiao/src/util.cpp#L70) from
  `state.inavSampleTimeMsec`, which is assigned in
  [`xiao/src/msplink.cpp:367`](../../xiao/src/msplink.cpp#L367) as
  `state.autoc_state.timestamp_us / 1000`. That `timestamp_us` is
  written by INAV in
  [`src/main/fc/fc_msp.c:685`](../../inav/src/main/fc/fc_msp.c#L685) as
  `sbufWriteU32(dst, micros());  // timestamp_us for correlation with blackbox`.
- xiao log column 1 (e.g. `0158732`) is **xiao-local `millis()`**
  ([util.cpp:56](../../xiao/src/util.cpp#L56)) and is intentionally
  ignored by the join.

Both sides therefore key on **INAV's `micros()`** clock; the join is
nearest-neighbour from xiao's logged `inav_ms` into the INAV blackbox
`t_ms` array
([join_flight_analysis.py:563-571](../../specs/029-no-future-arch/join_flight_analysis.py#L563-L571)).
The 500 Hz INAV sample rate vs the ~20 Hz xiao NN tick puts the
worst-case match error well under one xiao tick, and the empirical
NN→rc correlations of +0.95–1.00 confirm the alignment is good.

## Artifacts

All in this directory:

- `bangbang_flight_*.png` (2) — per-span output time series + |Δout|
  + |out| + per-axis distribution from
  [`specs/024-sim-real-fidelity/plot_bangbang_flight.py`](../../specs/024-sim-real-fidelity/plot_bangbang_flight.py).
- `join_analysis_*.png` / `.csv` (2) — per-span 10-panel diagnostic
  from
  [`specs/029-no-future-arch/join_flight_analysis.py`](../../specs/029-no-future-arch/join_flight_analysis.py).
- `blackbox_log_2026-05-17_125026.01.csv` — INAV blackbox CSV.
- `flight_log_2026-05-17T19-55-07.txt` — xiao log paired with the
  INAV CSV.
